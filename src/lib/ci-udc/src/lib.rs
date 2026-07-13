// SPDX-License-Identifier: GPL-2.0-only
//! Polled USB 2.0 device on a ChipIdea (EHCI-style dQH/dTD) device
//! controller, offering the one-interface bulk function of `usb-device`.
//!
//! Moved from the SpacemiT K1 bt0 `usb.rs` (ore-edk-boot-opi 78479e21ba).
//! The controller registers are relative to [`Config::base`]; the SoC's
//! clock, reset and UTMI PHY bring-up and the data cache maintenance are the
//! board's [`UdcPlatform`]. Queue heads, transfer descriptors and the control
//! buffers live in a [`Dma`] area the board places (K1: SRAM); bulk data goes
//! straight to and from the caller's memory. Caches are assumed not coherent
//! with the controller: every buffer is cleaned or invalidated around a
//! transfer.
//!
//! Every controller operation and active transfer has a poll budget; only
//! the idle wait for the next host command is host driven.
#![no_std]

use core::ptr::addr_of;

use regio::RegIo;
use usb_device::{decode, descriptor, Request, Setup, EP1};
pub use usb_device::{Descriptors, EP0_MAX};

const USBCMD: usize = 0x140;
const USBSTS: usize = 0x144;
const DEVICEADDR: usize = 0x154;
const EPLISTADDR: usize = 0x158;
const PORTSC: usize = 0x184;
const USBMODE: usize = 0x1a8;
const EPSETUPSTAT: usize = 0x1ac;
const EPPRIME: usize = 0x1b0;
const EPFLUSH: usize = 0x1b4;
const EPSTAT: usize = 0x1b8;
const EPCOMPLETE: usize = 0x1bc;
const EPCTRL0: usize = 0x1c0;

const CMD_RUN: u32 = 1 << 0;
const CMD_RST: u32 = 1 << 1;
const CMD_ITC8: u32 = 8 << 16;
const STS_PCI: u32 = 1 << 2;
const STS_URI: u32 = 1 << 6;
const MODE_DEVICE: u32 = 2;

const CTRL_RXS: u32 = 1 << 0; // OUT stall
const CTRL_RXT_BULK: u32 = 2 << 2;
const CTRL_RXR: u32 = 1 << 6; // OUT data toggle reset
const CTRL_RXE: u32 = 1 << 7;
const CTRL_TXS: u32 = 1 << 16; // IN stall
const CTRL_TXT_BULK: u32 = 2 << 18;
const CTRL_TXR: u32 = 1 << 22;
const CTRL_TXE: u32 = 1 << 23;

// Queue head and transfer descriptor fields.
const QH_IOS: u32 = 1 << 15;
const QH_ZLT_OFF: u32 = 1 << 29;
const TD_TERMINATE: u32 = 1;
const TD_ACTIVE: u32 = 1 << 7;
const TD_ERRORS: u32 = (1 << 6) | (1 << 5) | (1 << 3); // halted, buffer, transaction
const TD_IOC: u32 = 1 << 15;
const PAGE: usize = 4096;
/// Bytes one descriptor moves when its buffer starts on a page boundary.
pub const TD_BYTES: usize = 4 * PAGE;
/// Descriptors chained for one bulk OUT batch (TD_BYTES each).
pub const TD_CHAIN: usize = 64;

/// Poll budgets (read counts, not calibrated wall-clock deadlines) and the
/// controller's base address.
#[derive(Clone, Copy, Debug)]
pub struct Config {
    pub base: usize,
    pub reset_polls: usize,
    pub flush_polls: usize,
    pub transfer_polls: usize,
    /// Loop iterations the pull-up stays off so the host sees a disconnect.
    pub disconnect_spin: usize,
}

impl Config {
    /// The budgets the K1 bt0 used, at controller `base`.
    pub const fn new(base: usize) -> Self {
        Self {
            base,
            reset_polls: 10_000_000,
            flush_polls: 1_000_000,
            transfer_polls: 50_000_000,
            disconnect_spin: 2_000_000,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    /// The host reset or left the bus; the caller should start over.
    Reset,
    Transfer(u32),
    Timeout,
    PhyTimeout,
    ResetTimeout,
    FlushTimeout,
}

/// The PHY did not become ready during [`UdcPlatform::bring_up`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PhyTimeout;

/// The SoC's side: clocks, resets and PHY, cache maintenance, delays.
pub trait UdcPlatform {
    /// Clock the controller, release its reset and bring the PHY up.
    /// `Err(PhyTimeout)` if the PHY did not become ready (the driver then
    /// stops the controller and fails with [`Error::PhyTimeout`]).
    fn bring_up<IO: RegIo>(&mut self, io: &mut IO) -> Result<(), PhyTimeout>;
    /// Clean `len` bytes at `addr` to memory before the controller reads them.
    fn flush_dcache(&mut self, addr: usize, len: usize);
    /// Discard cached `len` bytes at `addr` before reading what the
    /// controller wrote.
    fn invalidate_dcache(&mut self, addr: usize, len: usize);
    /// Busy-wait about `n` loop iterations.
    fn spin(&mut self, n: usize) {
        for _ in 0..n {
            core::hint::spin_loop();
        }
    }
}

#[repr(C, align(64))]
#[derive(Clone, Copy)]
pub struct Qh {
    config: u32,
    current: u32,
    next: u32,
    token: u32,
    page: [u32; 5],
    _r0: u32,
    setup: [u8; 8],
    _r1: [u32; 4],
}

#[repr(C, align(32))]
#[derive(Clone, Copy)]
pub struct Td {
    next: u32,
    token: u32,
    page: [u32; 5],
    /// Unused by the controller: the length the descriptor was programmed with.
    len: u32,
}

const QH0: Qh = Qh {
    config: 0,
    current: 0,
    next: TD_TERMINATE,
    token: 0,
    page: [0; 5],
    _r0: 0,
    setup: [0; 8],
    _r1: [0; 4],
};
const TD0: Td = Td {
    next: TD_TERMINATE,
    token: 0,
    page: [0; 5],
    len: 0,
};

/// Everything the controller reads or writes besides bulk data. Place it in
/// memory the controller can reach (a `static`).
#[repr(C, align(4096))]
pub struct Dma {
    // ep0 OUT, ep0 IN, ep1 OUT, ep1 IN
    qh: [Qh; 4],
    ep0: [Td; 2],
    ep1_in: Td,
    ep1_out: [Td; TD_CHAIN],
    ep0_buf: [u8; 256],
    small: [u8; 512],
}

impl Dma {
    pub const fn new() -> Self {
        Self {
            qh: [QH0; 4],
            ep0: [TD0; 2],
            ep1_in: TD0,
            ep1_out: [TD0; TD_CHAIN],
            ep0_buf: [0; 256],
            small: [0; 512],
        }
    }
}

impl Default for Dma {
    fn default() -> Self {
        Self::new()
    }
}

fn ep_bit(num: u32, dir_in: bool) -> u32 {
    if dir_in {
        1 << (16 + num)
    } else {
        1 << num
    }
}

fn fill_td(td: &mut Td, buf: usize, len: usize, ioc: bool) {
    td.next = TD_TERMINATE;
    td.token = ((len as u32) << 16) | TD_ACTIVE | if ioc { TD_IOC } else { 0 };
    td.len = len as u32;
    td.page[0] = buf as u32;
    for i in 1..5 {
        td.page[i] = ((buf & !(PAGE - 1)) + i * PAGE) as u32;
    }
}

pub struct Udc<'a, IO: RegIo, P: UdcPlatform> {
    io: IO,
    plat: P,
    cfg: Config,
    // The controller owns parts of this area while transfers run, so it is
    // reached through a pointer, as the K1 driver reached its static.
    dma: *mut Dma,
    desc: Descriptors<'a>,
    high_speed: bool,
    configured: bool,
    fault: Option<Error>,
}

impl<'a, IO: RegIo, P: UdcPlatform> Udc<'a, IO, P> {
    fn d(&self) -> &'static mut Dma {
        // SAFETY: `dma` comes from a `&'static mut Dma` handed to `init`;
        // this driver is its only CPU user.
        unsafe { &mut *self.dma }
    }
    fn r(&mut self, o: usize) -> u32 {
        self.io.read32(self.cfg.base + o)
    }
    fn w(&mut self, o: usize, v: u32) {
        self.io.write32(self.cfg.base + o, v)
    }
    fn stop(&mut self) {
        let v = self.r(USBCMD) & !CMD_RUN;
        self.w(USBCMD, v);
    }
    fn flush<T>(&mut self, r: &T) {
        self.plat
            .flush_dcache(r as *const T as usize, core::mem::size_of::<T>());
    }
    fn invalidate<T>(&mut self, r: &T) {
        self.plat
            .invalidate_dcache(r as *const T as usize, core::mem::size_of::<T>());
    }

    /// Flush completion alone can race a primed endpoint. Reissue while
    /// EPSTAT still reports it active, sharing one finite budget.
    fn flush_endpoints(&mut self, bits: u32) -> Result<(), Error> {
        self.w(EPFLUSH, bits);
        for _ in 0..self.cfg.flush_polls {
            if self.r(EPFLUSH) & bits == 0 {
                if self.r(EPSTAT) & bits == 0 {
                    return Ok(());
                }
                self.w(EPFLUSH, bits);
            }
        }
        // A failed quiescence must never leave a running controller behind.
        self.stop();
        Err(Error::FlushTimeout)
    }

    /// Bring the controller up and connect to the host as a new device
    /// (dropping any earlier session, e.g. the BootROM's download mode).
    pub fn init(
        io: IO,
        plat: P,
        cfg: Config,
        dma: &'static mut Dma,
        desc: Descriptors<'a>,
    ) -> Result<Self, Error> {
        let mut u = Self {
            io,
            plat,
            cfg,
            dma,
            desc,
            high_speed: false,
            configured: false,
            fault: None,
        };
        if u.plat.bring_up(&mut u.io).is_err() {
            u.stop();
            return Err(Error::PhyTimeout);
        }
        // Drop the pull-up long enough for the host to see a disconnect,
        // then reset the controller.
        u.stop();
        u.plat.spin(u.cfg.disconnect_spin);
        u.w(USBCMD, CMD_ITC8 | CMD_RST);
        let mut reset = false;
        for _ in 0..u.cfg.reset_polls {
            if u.r(USBCMD) & CMD_RST == 0 {
                reset = true;
                break;
            }
        }
        if !reset {
            return Err(Error::ResetTimeout);
        }
        let v = (u.r(USBMODE) & !3) | MODE_DEVICE;
        u.w(USBMODE, v);
        u.flush_endpoints(0xffff_ffff)?;
        let d = u.d();
        d.qh = [QH0; 4];
        d.qh[0].config = ((EP0_MAX as u32) << 16) | QH_IOS | QH_ZLT_OFF;
        d.qh[1].config = ((EP0_MAX as u32) << 16) | QH_ZLT_OFF;
        u.flush(&d.qh);
        u.w(EPLISTADDR, addr_of!(d.qh) as u32);
        let v = u.r(EPSETUPSTAT);
        u.w(EPSETUPSTAT, v);
        let v = u.r(EPCOMPLETE);
        u.w(EPCOMPLETE, v);
        let v = u.r(USBSTS);
        u.w(USBSTS, v);
        let v = u.r(USBCMD) | CMD_ITC8 | CMD_RUN;
        u.w(USBCMD, v);
        u.io.log(format_args!(
            "[usb] device controller running, waiting for the host\n"
        ));
        Ok(u)
    }

    fn check(&self) -> Result<(), Error> {
        self.fault.map_or(Ok(()), Err)
    }

    /// A transport fault is terminal for this session. In particular, a
    /// failed flush must not be followed by rewriting controller-owned DMA.
    fn remember<T>(&mut self, result: Result<T, Error>) -> Result<T, Error> {
        if let Err(e) = result {
            if !matches!(e, Error::Reset) {
                self.fault = Some(e);
                self.stop();
            }
        }
        result
    }

    pub fn configured(&self) -> bool {
        self.configured
    }

    /// Leave the bus: stop the controller, which drops the pull-up.
    pub fn disconnect(&mut self) -> Result<(), Error> {
        let result = self.flush_endpoints(0xffff_ffff);
        self.stop();
        self.configured = false;
        self.remember(result)
    }

    fn max_packet(&self) -> usize {
        if self.high_speed {
            512
        } else {
            64
        }
    }

    fn bus_reset(&mut self) -> Result<(), Error> {
        self.configured = false;
        let v = self.r(EPSETUPSTAT);
        self.w(EPSETUPSTAT, v);
        let v = self.r(EPCOMPLETE);
        self.w(EPCOMPLETE, v);
        self.flush_endpoints(0xffff_ffff)?;
        self.w(EPCTRL0 + 4 * EP1 as usize, 0);
        self.w(DEVICEADDR, 0);
        Ok(())
    }

    /// Handle bus events and control requests. `Err(Reset)` when the host
    /// reset the bus.
    pub fn poll(&mut self) -> Result<(), Error> {
        self.check()?;
        let result = self.poll_inner();
        self.remember(result)
    }

    fn poll_inner(&mut self) -> Result<(), Error> {
        let sts = self.r(USBSTS);
        self.w(USBSTS, sts);
        let mut reset = false;
        if sts & STS_URI != 0 {
            self.bus_reset()?;
            reset = true;
        }
        if sts & STS_PCI != 0 {
            let hs = (self.r(PORTSC) >> 26) & 3 == 2;
            if hs != self.high_speed {
                self.io.log(format_args!(
                    "[usb] {} speed\n",
                    if hs { "high" } else { "full" }
                ));
            }
            self.high_speed = hs;
        }
        if self.r(EPSETUPSTAT) & 1 != 0 {
            self.setup()?;
        }
        if reset {
            Err(Error::Reset)
        } else {
            Ok(())
        }
    }

    fn stall_ep0(&mut self) {
        let v = self.r(EPCTRL0) | CTRL_TXS | CTRL_RXS;
        self.w(EPCTRL0, v);
    }

    /// Run one ep0 transfer (data or status stage) and wait for it.
    fn ep0_xfer(&mut self, dir_in: bool, len: usize) -> Result<(), Error> {
        let d = self.d();
        let qh = if dir_in { 1 } else { 0 };
        if dir_in {
            self.flush(&d.ep0_buf);
        }
        let td = &mut d.ep0[qh];
        fill_td(td, addr_of!(d.ep0_buf) as usize, len, true);
        self.flush(td);
        d.qh[qh].next = td as *const Td as u32;
        d.qh[qh].token = 0;
        self.flush(&d.qh[qh]);
        let bit = ep_bit(0, dir_in);
        self.w(EPPRIME, bit);
        for _ in 0..self.cfg.transfer_polls {
            if self.r(EPCOMPLETE) & bit != 0 {
                self.w(EPCOMPLETE, bit);
                self.invalidate(td);
                if td.token & TD_ERRORS != 0 {
                    return Err(Error::Transfer(td.token));
                }
                return Ok(());
            }
            // A new SETUP aborts this transfer.
            if self.r(EPSETUPSTAT) & 1 != 0 || self.r(USBSTS) & STS_URI != 0 {
                self.flush_endpoints(bit)?;
                return Err(Error::Reset);
            }
        }
        self.flush_endpoints(bit)?;
        Err(Error::Timeout)
    }

    /// IN data stage of `data`, then the OUT status stage.
    fn ep0_send(&mut self, data: &[u8], w_length: usize) -> Result<(), Error> {
        let d = self.d();
        let n = data.len().min(w_length).min(d.ep0_buf.len());
        d.ep0_buf[..n].copy_from_slice(&data[..n]);
        self.ep0_xfer(true, n)?;
        self.ep0_xfer(false, 0)
    }

    /// No data stage: the IN status stage.
    fn ep0_ack(&mut self) -> Result<(), Error> {
        self.ep0_xfer(true, 0)
    }

    fn setup(&mut self) -> Result<(), Error> {
        let d = self.d();
        self.invalidate(&d.qh[0]);
        let s = Setup::parse(d.qh[0].setup);
        self.w(EPSETUPSTAT, 1);
        // Any ep0 transfer still primed belongs to the aborted request.
        self.flush_endpoints(ep_bit(0, true) | ep_bit(0, false))?;
        let length = s.length as usize;
        match decode(&s) {
            Request::GetDescriptor { value } => {
                let mut buf = [0u8; 64];
                match descriptor(&self.desc, value, self.max_packet() as u16, &mut buf) {
                    Some(n) => self.ep0_send(&buf[..n], length)?,
                    None => self.stall_ep0(),
                }
            }
            Request::SetAddress(value) => {
                // Applied after the status stage.
                self.w(DEVICEADDR, ((value as u32) << 25) | (1 << 24));
                self.ep0_ack()?;
            }
            Request::SetConfiguration(value) => {
                if value == 1 {
                    self.enable_ep1();
                    self.configured = true;
                } else {
                    self.w(EPCTRL0 + 4 * EP1 as usize, 0);
                    self.configured = false;
                }
                self.ep0_ack()?;
            }
            Request::GetConfiguration => self.ep0_send(&[self.configured as u8], length)?,
            Request::GetStatusDevice => self.ep0_send(&[1, 0], length)?, // self powered
            Request::GetStatusOther => self.ep0_send(&[0, 0], length)?,
            Request::ClearEndpointHalt { index } => {
                // Unstall and reset the data toggle.
                if index & 0xf == EP1 as u16 {
                    let r = EPCTRL0 + 4 * EP1 as usize;
                    let v = self.r(r);
                    if index & 0x80 != 0 {
                        self.w(r, (v & !CTRL_TXS) | CTRL_TXR);
                    } else {
                        self.w(r, (v & !CTRL_RXS) | CTRL_RXR);
                    }
                }
                self.ep0_ack()?;
            }
            Request::SetInterface => self.ep0_ack()?,
            Request::Unsupported => {
                self.io.log(format_args!(
                    "[usb] unsupported request {:02x} {:02x}\n",
                    s.request_type, s.request
                ));
                self.stall_ep0();
            }
        }
        Ok(())
    }

    fn enable_ep1(&mut self) {
        let d = self.d();
        let mps = (self.max_packet() as u32) << 16;
        d.qh[2] = QH0;
        d.qh[3] = QH0;
        d.qh[2].config = mps | QH_ZLT_OFF;
        d.qh[3].config = mps | QH_ZLT_OFF;
        self.flush(&d.qh);
        self.w(
            EPCTRL0 + 4 * EP1 as usize,
            CTRL_TXE | CTRL_TXR | CTRL_TXT_BULK | CTRL_RXE | CTRL_RXR | CTRL_RXT_BULK,
        );
    }

    /// Wait until the host has configured the device.
    pub fn wait_configured(&mut self) -> Result<(), Error> {
        self.check()?;
        while !self.configured {
            match self.poll() {
                Ok(()) | Err(Error::Reset) => (),
                Err(e) => return Err(e),
            }
        }
        Ok(())
    }

    /// Send `data` (at most 512 bytes) on EP1 IN.
    pub fn send(&mut self, data: &[u8]) -> Result<(), Error> {
        self.check()?;
        let d = self.d();
        let n = data.len().min(d.small.len());
        d.small[..n].copy_from_slice(&data[..n]);
        self.flush(&d.small);
        let result = self
            .bulk(true, addr_of!(d.small) as usize, n, true)
            .map(|_| ());
        self.remember(result)
    }

    /// Receive one command (a single transfer of at most 512 bytes) on
    /// EP1 OUT into `out`; returns its length.
    pub fn recv(&mut self, out: &mut [u8]) -> Result<usize, Error> {
        self.check()?;
        let d = self.d();
        self.flush(&d.small);
        let result = self.bulk(false, addr_of!(d.small) as usize, d.small.len(), false);
        let n = self.remember(result)?;
        self.invalidate(&d.small);
        let n = n.min(out.len());
        out[..n].copy_from_slice(&d.small[..n]);
        Ok(n)
    }

    /// Receive exactly `len` bytes into memory at `buf` (page aligned).
    pub fn recv_into(&mut self, buf: usize, len: usize) -> Result<(), Error> {
        self.check()?;
        let mut done = 0;
        let mut next_report = 64 << 20;
        while done < len {
            let chunk = (len - done).min(TD_BYTES * TD_CHAIN);
            self.plat.flush_dcache(buf + done, chunk);
            let result = self.bulk(false, buf + done, chunk, true);
            let got = self.remember(result)?;
            self.plat.invalidate_dcache(buf + done, chunk);
            if got != chunk {
                return self.remember(Err(Error::Transfer(got as u32)));
            }
            done += chunk;
            if done >= next_report {
                self.io.log(format_args!("[usb] {} MiB\n", done >> 20));
                next_report += 64 << 20;
            }
        }
        Ok(())
    }

    /// One bulk transfer on EP1, split over a descriptor chain. Returns the
    /// bytes moved (an OUT transfer ends early on a short packet).
    fn bulk(&mut self, dir_in: bool, buf: usize, len: usize, active: bool) -> Result<usize, Error> {
        let d = self.d();
        let tds: &mut [Td] = if dir_in {
            core::slice::from_mut(&mut d.ep1_in)
        } else {
            &mut d.ep1_out[..]
        };
        // Descriptors of TD_BYTES each; the first may start mid-page.
        let mut count = 0;
        let mut off = 0;
        while off < len || count == 0 {
            if count == tds.len() {
                return Err(Error::Transfer(0));
            }
            let start = buf + off;
            let room = 5 * PAGE - (start & (PAGE - 1));
            let n = (len - off).min(room).min(TD_BYTES);
            fill_td(&mut tds[count], start, n, false);
            if count > 0 {
                tds[count - 1].next = addr_of!(tds[count]) as u32;
            }
            count += 1;
            off += n;
        }
        tds[count - 1].token |= TD_IOC;
        for td in tds[..count].iter() {
            self.flush(td);
        }
        let qh = 2 + dir_in as usize;
        d.qh[qh].next = addr_of!(tds[0]) as u32;
        d.qh[qh].token = 0;
        self.flush(&d.qh[qh]);
        let bit = ep_bit(EP1 as u32, dir_in);
        self.w(EPPRIME, bit);
        let mut polls = 0;
        loop {
            if self.r(EPCOMPLETE) & bit != 0 {
                self.w(EPCOMPLETE, bit);
            }
            // Done when no descriptor in the chain is active any more.
            self.invalidate(&tds[count - 1]);
            if tds[count - 1].token & TD_ACTIVE == 0 {
                break;
            }
            // An OUT short packet retires the descriptor it ended in; the
            // ones after it stay active, so look for a short descriptor.
            if !dir_in {
                let mut short = false;
                for td in tds[..count].iter() {
                    self.invalidate(td);
                    if td.token & TD_ERRORS != 0 {
                        let token = td.token;
                        self.flush_endpoints(bit)?;
                        return Err(Error::Transfer(token));
                    }
                    if td.token & TD_ACTIVE != 0 {
                        break;
                    }
                    if td.token >> 16 & 0x7fff != 0 {
                        short = true;
                        break;
                    }
                }
                if short {
                    self.flush_endpoints(bit)?;
                    break;
                }
            }
            if let Err(e) = self.poll() {
                self.flush_endpoints(bit)?;
                return Err(e);
            }
            if active {
                polls += 1;
                if polls == self.cfg.transfer_polls {
                    self.flush_endpoints(bit)?;
                    return Err(Error::Timeout);
                }
            }
        }
        let mut moved = 0;
        for td in tds[..count].iter() {
            self.invalidate(td);
            if td.token & TD_ERRORS != 0 {
                return Err(Error::Transfer(td.token));
            }
            if td.token & TD_ACTIVE != 0 {
                break;
            }
            let total = td.len as usize;
            let left = (td.token >> 16 & 0x7fff) as usize;
            moved += total - left;
            if left != 0 {
                break;
            }
        }
        Ok(moved)
    }

    /// The register access, for the board.
    pub fn io(&mut self) -> &mut IO {
        &mut self.io
    }
}

/// `fastboot` over this controller (feature `fastboot`).
#[cfg(feature = "fastboot")]
impl<IO: RegIo, P: UdcPlatform> fastboot::Transport for Udc<'_, IO, P> {
    type Error = Error;
    fn wait_configured(&mut self) -> Result<(), Error> {
        Udc::wait_configured(self)
    }
    fn recv(&mut self, out: &mut [u8]) -> Result<usize, Error> {
        Udc::recv(self, out)
    }
    fn recv_exact(&mut self, out: &mut [u8]) -> Result<(), Error> {
        self.recv_into(out.as_mut_ptr() as usize, out.len())
    }
    fn send(&mut self, data: &[u8]) -> Result<(), Error> {
        Udc::send(self, data)
    }
    fn disconnect(&mut self) -> Result<(), Error> {
        Udc::disconnect(self)
    }
    fn is_reset(e: &Error) -> bool {
        matches!(e, Error::Reset)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn descriptor_pages_follow_the_buffer() {
        let mut td = TD0;
        fill_td(&mut td, 0x1000_0800, 0x3000, true);
        assert_eq!(td.token, (0x3000 << 16) | TD_ACTIVE | TD_IOC);
        assert_eq!(
            td.page,
            [
                0x1000_0800,
                0x1000_1000,
                0x1000_2000,
                0x1000_3000,
                0x1000_4000
            ]
        );
        assert_eq!(td.len, 0x3000);
    }
    #[test]
    fn endpoint_bits() {
        assert_eq!(ep_bit(0, false), 1);
        assert_eq!(ep_bit(0, true), 1 << 16);
        assert_eq!(ep_bit(1, true), 1 << 17);
    }
    #[test]
    fn dma_layout_matches_the_controller() {
        assert_eq!(core::mem::size_of::<Qh>(), 64);
        assert_eq!(core::mem::size_of::<Td>(), 32);
        assert_eq!(core::mem::align_of::<Dma>(), 4096);
    }
    #[cfg(feature = "fastboot")]
    #[test]
    fn udc_is_a_fastboot_transport() {
        fn transport<T: fastboot::Transport>() {}
        struct P;
        impl UdcPlatform for P {
            fn bring_up<IO: RegIo>(&mut self, _: &mut IO) -> Result<(), PhyTimeout> {
                Ok(())
            }
            fn flush_dcache(&mut self, _: usize, _: usize) {}
            fn invalidate_dcache(&mut self, _: usize, _: usize) {}
        }
        transport::<Udc<'static, regio::mock::Mock<4>, P>>();
        assert!(
            <Udc<'static, regio::mock::Mock<4>, P> as fastboot::Transport>::is_reset(&Error::Reset)
        );
    }
}
