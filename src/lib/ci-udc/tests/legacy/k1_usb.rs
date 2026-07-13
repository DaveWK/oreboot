// Verbatim copy for equivalence tests: ore-edk-boot-opi 78479e21ba
// oreboot/src/mainboard/spacemit/k1x/bt0/src/usb.rs (SpacemiT K1 bt0).
//! Polled USB 2.0 device for the SpacemiT K1 OTG port: the port the BootROM
//! uses for download mode (the R2S's bottom USB-A port, the RV2's USB-C).
//!
//! The controller is a ChipIdea device controller at 0xc090_0000 with its
//! UTMI PHY at 0xc094_0000; the clock, PHY and controller set-up follow
//! SpacemiT's U-Boot (drivers/usb/gadget/k1x_usb2_ci.c). One configuration
//! with one vendor interface and a bulk endpoint pair (EP1 IN/OUT) is
//! offered; the class codes are the caller's (fastboot: ff/42/03).
//!
//! Queue heads, transfer descriptors and the control buffers live in SRAM;
//! bulk data goes straight to and from DRAM. The caches are not coherent with
//! the controller, so every buffer is cleaned or invalidated around a
//! transfer.

use core::cell::Cell;
use core::ptr::{addr_of, addr_of_mut, read_volatile, write_volatile};
use util::mmio::{read32, write32};

use crate::cache;

const USB: usize = 0xc090_0000;
const USBCMD: usize = USB + 0x140;
const USBSTS: usize = USB + 0x144;
const DEVICEADDR: usize = USB + 0x154;
const EPLISTADDR: usize = USB + 0x158;
const PORTSC: usize = USB + 0x184;
const USBMODE: usize = USB + 0x1a8;
const EPSETUPSTAT: usize = USB + 0x1ac;
const EPPRIME: usize = USB + 0x1b0;
const EPFLUSH: usize = USB + 0x1b4;
const EPSTAT: usize = USB + 0x1b8;
const EPCOMPLETE: usize = USB + 0x1bc;
const EPCTRL0: usize = USB + 0x1c0;

const CMD_RUN: u32 = 1 << 0;
const CMD_RST: u32 = 1 << 1;
const CMD_ITC8: u32 = 8 << 16;
const STS_UI: u32 = 1 << 0;
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

// APMU USB clock/reset and the PHY (k1x_usb2_ci.h).
const APMU_USB_CLK_RES_CTRL: usize = 0xd428_2800 + 0x5c;
const USB_AXICLK_EN: u32 = 1 << 1;
const USB_AXI_RST: u32 = 1 << 0; // reset released when set
const PHY_REG01: usize = 0xc094_0000 + 0x04;
const PHY_REG0D: usize = 0xc094_0000 + 0x34;
const PHY_PLL_READY: u32 = 1 << 0;

// Queue head and transfer descriptor fields.
const QH_IOS: u32 = 1 << 15;
const QH_ZLT_OFF: u32 = 1 << 29;
const TD_TERMINATE: u32 = 1;
const TD_ACTIVE: u32 = 1 << 7;
const TD_ERRORS: u32 = (1 << 6) | (1 << 5) | (1 << 3); // halted, buffer, transaction
const TD_IOC: u32 = 1 << 15;
const PAGE: usize = 4096;
/// Bytes one descriptor moves when its buffer starts on a page boundary.
const TD_BYTES: usize = 4 * PAGE;
/// Descriptors chained for one bulk OUT batch (TD_BYTES each).
const TD_CHAIN: usize = 64;

pub const EP0_MAX: usize = 64;
const EP1: u32 = 1;
// Read budgets, not calibrated wall-clock deadlines. Idle host command waits
// remain unbounded; controller operations and active transfers do not.
const PHY_POLLS: usize = 1_000_000;
const RESET_POLLS: usize = 10_000_000;
const FLUSH_POLLS: usize = 1_000_000;
const TRANSFER_POLLS: usize = 50_000_000;

#[repr(C, align(64))]
#[derive(Clone, Copy)]
struct Qh {
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
struct Td {
    next: u32,
    token: u32,
    page: [u32; 5],
    _r: u32,
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
    _r: 0,
};

/// Everything the controller reads or writes in SRAM.
#[repr(C, align(4096))]
struct Dma {
    // ep0 OUT, ep0 IN, ep1 OUT, ep1 IN
    qh: [Qh; 4],
    ep0: [Td; 2],
    ep1_in: Td,
    ep1_out: [Td; TD_CHAIN],
    ep0_buf: [u8; 256],
    small: [u8; 512],
}

static mut DMA: Dma = Dma {
    qh: [QH0; 4],
    ep0: [TD0; 2],
    ep1_in: TD0,
    ep1_out: [TD0; TD_CHAIN],
    ep0_buf: [0; 256],
    small: [0; 512],
};

fn dma() -> &'static mut Dma {
    unsafe { &mut *addr_of_mut!(DMA) }
}

fn flush<T>(r: &T) {
    cache::flush(r as *const T as usize, core::mem::size_of::<T>());
}

fn invalidate<T>(r: &T) {
    cache::invalidate(r as *const T as usize, core::mem::size_of::<T>());
}

fn spin(n: usize) {
    for _ in 0..n {
        core::hint::spin_loop();
    }
}

fn ep_bit(num: u32, dir_in: bool) -> u32 {
    if dir_in {
        1 << (16 + num)
    } else {
        1 << num
    }
}

#[derive(Debug, Clone, Copy)]
pub enum Error {
    /// The host reset or left the bus; the caller should start over.
    Reset,
    Transfer(u32),
    Timeout,
    PhyTimeout,
    ResetTimeout,
    FlushTimeout,
}

/// Flush completion alone can race a primed endpoint. Reissue while EPSTAT
/// still reports it active, sharing one finite budget across both waits.
fn flush_endpoints(bits: u32) -> Result<(), Error> {
    write32(EPFLUSH, bits);
    for _ in 0..FLUSH_POLLS {
        if read32(EPFLUSH) & bits == 0 {
            if read32(EPSTAT) & bits == 0 {
                return Ok(());
            }
            write32(EPFLUSH, bits);
        }
    }
    // A failed quiescence must never leave a running controller behind.
    write32(USBCMD, read32(USBCMD) & !CMD_RUN);
    Err(Error::FlushTimeout)
}

/// What a control request needs from the function layer.
pub struct Descriptors<'a> {
    pub vendor: u16,
    pub product: u16,
    pub class: [u8; 3],
    pub strings: [&'a str; 4], // manufacturer, product, serial, interface
}

pub struct Usb<'a> {
    desc: Descriptors<'a>,
    high_speed: bool,
    configured: bool,
    fault: Cell<Option<Error>>,
}

fn fill_td(td: &mut Td, buf: usize, len: usize, ioc: bool) {
    td.next = TD_TERMINATE;
    td.token = ((len as u32) << 16) | TD_ACTIVE | if ioc { TD_IOC } else { 0 };
    // The controller does not use this word; keep the length for later.
    td._r = len as u32;
    td.page[0] = buf as u32;
    for i in 1..5 {
        td.page[i] = ((buf & !(PAGE - 1)) + i * PAGE) as u32;
    }
}

impl<'a> Usb<'a> {
    /// Reset the controller and connect to the host as a new device. In
    /// download mode this drops the BootROM's own session.
    pub fn init(desc: Descriptors<'a>) -> Result<Self, Error> {
        let v = read32(APMU_USB_CLK_RES_CTRL);
        write32(APMU_USB_CLK_RES_CTRL, v | USB_AXICLK_EN);
        write32(APMU_USB_CLK_RES_CTRL, (v | USB_AXICLK_EN) & !USB_AXI_RST);
        write32(APMU_USB_CLK_RES_CTRL, v | USB_AXICLK_EN | USB_AXI_RST);
        spin(200_000);
        let mut ready = false;
        for _ in 0..PHY_POLLS {
            if read32(PHY_REG01) & PHY_PLL_READY != 0 {
                ready = true;
                break;
            }
        }
        if !ready {
            write32(USBCMD, read32(USBCMD) & !CMD_RUN);
            return Err(Error::PhyTimeout);
        }
        write32(PHY_REG01, 0x60ef);
        write32(PHY_REG0D, 0x1c);

        // Drop the pull-up long enough for the host to see a disconnect
        // (about a second at the clock bt0 runs at),
        // then reset the controller.
        write32(USBCMD, read32(USBCMD) & !CMD_RUN);
        spin(2_000_000);
        write32(USBCMD, CMD_ITC8 | CMD_RST);
        let mut reset = false;
        for _ in 0..RESET_POLLS {
            if read32(USBCMD) & CMD_RST == 0 {
                reset = true;
                break;
            }
        }
        if !reset {
            return Err(Error::ResetTimeout);
        }
        write32(USBMODE, (read32(USBMODE) & !3) | MODE_DEVICE);
        flush_endpoints(0xffff_ffff)?;
        let d = dma();
        d.qh = [QH0; 4];
        d.qh[0].config = ((EP0_MAX as u32) << 16) | QH_IOS | QH_ZLT_OFF;
        d.qh[1].config = ((EP0_MAX as u32) << 16) | QH_ZLT_OFF;
        flush(&d.qh);
        write32(EPLISTADDR, addr_of!(d.qh) as u32);
        write32(EPSETUPSTAT, read32(EPSETUPSTAT));
        write32(EPCOMPLETE, read32(EPCOMPLETE));
        write32(USBSTS, read32(USBSTS));
        write32(USBCMD, read32(USBCMD) | CMD_ITC8 | CMD_RUN);
        println!("[usb] device controller running, waiting for the host");
        Ok(Self {
            desc,
            high_speed: false,
            configured: false,
            fault: Cell::new(None),
        })
    }

    fn check(&self) -> Result<(), Error> {
        self.fault.get().map_or(Ok(()), Err)
    }

    /// A transport fault is terminal for this session. In particular, a
    /// failed flush must not be followed by rewriting controller-owned DMA.
    fn remember<T>(&self, result: Result<T, Error>) -> Result<T, Error> {
        if let Err(e) = result {
            if !matches!(e, Error::Reset) {
                self.fault.set(Some(e));
                write32(USBCMD, read32(USBCMD) & !CMD_RUN);
            }
        }
        result
    }

    pub fn configured(&self) -> bool {
        self.configured
    }

    /// Leave the bus: stop the controller, which drops the pull-up.
    pub fn disconnect(&mut self) -> Result<(), Error> {
        let result = flush_endpoints(0xffff_ffff);
        write32(USBCMD, read32(USBCMD) & !CMD_RUN);
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
        write32(EPSETUPSTAT, read32(EPSETUPSTAT));
        write32(EPCOMPLETE, read32(EPCOMPLETE));
        flush_endpoints(0xffff_ffff)?;
        write32(EPCTRL0 + 4 * EP1 as usize, 0);
        write32(DEVICEADDR, 0);
        Ok(())
    }

    /// Handle bus events and control requests. Returns Err(Reset) when the
    /// host reset the bus.
    pub fn poll(&mut self) -> Result<(), Error> {
        self.check()?;
        let result = self.poll_inner();
        self.remember(result)
    }

    fn poll_inner(&mut self) -> Result<(), Error> {
        let sts = read32(USBSTS);
        write32(USBSTS, sts);
        let mut reset = false;
        if sts & STS_URI != 0 {
            self.bus_reset()?;
            reset = true;
        }
        if sts & STS_PCI != 0 {
            let hs = (read32(PORTSC) >> 26) & 3 == 2;
            if hs != self.high_speed {
                println!("[usb] {} speed", if hs { "high" } else { "full" });
            }
            self.high_speed = hs;
        }
        if read32(EPSETUPSTAT) & 1 != 0 {
            self.setup()?;
        }
        if reset {
            Err(Error::Reset)
        } else {
            Ok(())
        }
    }

    fn stall_ep0(&self) {
        write32(EPCTRL0, read32(EPCTRL0) | CTRL_TXS | CTRL_RXS);
    }

    /// Run one ep0 transfer (data or status stage) and wait for it.
    fn ep0_xfer(&self, dir_in: bool, len: usize) -> Result<(), Error> {
        let d = dma();
        let qh = if dir_in { 1 } else { 0 };
        let td = &mut d.ep0[qh];
        if dir_in {
            flush(&d.ep0_buf);
        }
        fill_td(td, addr_of!(d.ep0_buf) as usize, len, true);
        flush(td);
        d.qh[qh].next = td as *const Td as u32;
        d.qh[qh].token = 0;
        flush(&d.qh[qh]);
        let bit = ep_bit(0, dir_in);
        write32(EPPRIME, bit);
        for _ in 0..TRANSFER_POLLS {
            if read32(EPCOMPLETE) & bit != 0 {
                write32(EPCOMPLETE, bit);
                invalidate(td);
                if td.token & TD_ERRORS != 0 {
                    return Err(Error::Transfer(td.token));
                }
                return Ok(());
            }
            // A new SETUP aborts this transfer.
            if read32(EPSETUPSTAT) & 1 != 0 || read32(USBSTS) & STS_URI != 0 {
                flush_endpoints(bit)?;
                return Err(Error::Reset);
            }
        }
        flush_endpoints(bit)?;
        Err(Error::Timeout)
    }

    /// IN data stage of `data`, then the OUT status stage.
    fn ep0_send(&self, data: &[u8], w_length: usize) -> Result<(), Error> {
        let n = data.len().min(w_length).min(dma().ep0_buf.len());
        dma().ep0_buf[..n].copy_from_slice(&data[..n]);
        self.ep0_xfer(true, n)?;
        self.ep0_xfer(false, 0)
    }

    /// No data stage: the IN status stage.
    fn ep0_ack(&self) -> Result<(), Error> {
        self.ep0_xfer(true, 0)
    }

    fn setup(&mut self) -> Result<(), Error> {
        let d = dma();
        invalidate(&d.qh[0]);
        let s = d.qh[0].setup;
        write32(EPSETUPSTAT, 1);
        // Any ep0 transfer still primed belongs to the aborted request.
        flush_endpoints(ep_bit(0, true) | ep_bit(0, false))?;
        let request_type = s[0];
        let request = s[1];
        let value = u16::from_le_bytes([s[2], s[3]]);
        let index = u16::from_le_bytes([s[4], s[5]]);
        let length = u16::from_le_bytes([s[6], s[7]]) as usize;
        match (request_type, request) {
            // GET_DESCRIPTOR
            (0x80, 6) => self.get_descriptor(value, length)?,
            // SET_ADDRESS, applied after the status stage
            (0x00, 5) => {
                write32(DEVICEADDR, ((value as u32) << 25) | (1 << 24));
                self.ep0_ack()?;
            }
            // SET_CONFIGURATION
            (0x00, 9) => {
                if value == 1 {
                    self.enable_ep1();
                    self.configured = true;
                } else {
                    write32(EPCTRL0 + 4 * EP1 as usize, 0);
                    self.configured = false;
                }
                self.ep0_ack()?;
            }
            // GET_CONFIGURATION
            (0x80, 8) => self.ep0_send(&[self.configured as u8], length)?,
            // GET_STATUS (device, interface, endpoint)
            (0x80, 0) => self.ep0_send(&[1, 0], length)?, // self powered
            (0x81, 0) | (0x82, 0) => self.ep0_send(&[0, 0], length)?,
            // CLEAR_FEATURE(ENDPOINT_HALT): unstall and reset the toggle
            (0x02, 1) => {
                if index & 0xf == EP1 as u16 {
                    let r = EPCTRL0 + 4 * EP1 as usize;
                    if index & 0x80 != 0 {
                        write32(r, (read32(r) & !CTRL_TXS) | CTRL_TXR);
                    } else {
                        write32(r, (read32(r) & !CTRL_RXS) | CTRL_RXR);
                    }
                }
                self.ep0_ack()?;
            }
            // SET_INTERFACE (only alternate setting 0 exists)
            (0x01, 11) => self.ep0_ack()?,
            _ => {
                println!("[usb] unsupported request {request_type:02x} {request:02x}");
                self.stall_ep0();
            }
        }
        Ok(())
    }

    fn enable_ep1(&self) {
        let d = dma();
        let mps = (self.max_packet() as u32) << 16;
        d.qh[2] = QH0;
        d.qh[3] = QH0;
        d.qh[2].config = mps | QH_ZLT_OFF;
        d.qh[3].config = mps | QH_ZLT_OFF;
        flush(&d.qh);
        write32(
            EPCTRL0 + 4 * EP1 as usize,
            CTRL_TXE | CTRL_TXR | CTRL_TXT_BULK | CTRL_RXE | CTRL_RXR | CTRL_RXT_BULK,
        );
    }

    fn get_descriptor(&self, value: u16, length: usize) -> Result<(), Error> {
        let kind = (value >> 8) as u8;
        let index = (value & 0xff) as usize;
        let mut buf = [0u8; 64];
        let n = match kind {
            // Device
            1 => {
                let [vl, vh] = self.desc.vendor.to_le_bytes();
                let [pl, ph] = self.desc.product.to_le_bytes();
                buf[..18].copy_from_slice(&[
                    18,
                    1,
                    0x00,
                    0x02,
                    0,
                    0,
                    0,
                    EP0_MAX as u8,
                    vl,
                    vh,
                    pl,
                    ph,
                    0x00,
                    0x01,
                    1,
                    2,
                    3,
                    1,
                ]);
                18
            }
            // Configuration: interface and the two bulk endpoints
            2 => {
                let [ml, mh] = (self.max_packet() as u16).to_le_bytes();
                let c = self.desc.class;
                buf[..32].copy_from_slice(&[
                    9, 2, 32, 0, 1, 1, 0, 0xc0, 50, // configuration
                    9, 4, 0, 0, 2, c[0], c[1], c[2], 4, // interface
                    7, 5, 0x81, 2, ml, mh, 0, // EP1 IN
                    7, 5, 0x01, 2, ml, mh, 1, // EP1 OUT
                ]);
                32
            }
            // String: language list, then UTF-16LE strings
            3 => {
                if index == 0 {
                    buf[..4].copy_from_slice(&[4, 3, 0x09, 0x04]);
                    4
                } else if index <= 4 {
                    let s = self.desc.strings[index - 1].as_bytes();
                    let n = s.len().min(30);
                    buf[0] = (2 + 2 * n) as u8;
                    buf[1] = 3;
                    for (i, c) in s[..n].iter().enumerate() {
                        buf[2 + 2 * i] = *c;
                    }
                    2 + 2 * n
                } else {
                    self.stall_ep0();
                    return Ok(());
                }
            }
            // Device qualifier: the other speed has the same layout
            6 => {
                buf[..10].copy_from_slice(&[10, 6, 0x00, 0x02, 0, 0, 0, EP0_MAX as u8, 1, 0]);
                10
            }
            _ => {
                self.stall_ep0();
                return Ok(());
            }
        };
        self.ep0_send(&buf[..n], length)
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
        let d = dma();
        let n = data.len().min(d.small.len());
        d.small[..n].copy_from_slice(&data[..n]);
        flush(&d.small);
        let result = self
            .bulk(true, addr_of!(d.small) as usize, n, true)
            .map(|_| ());
        self.remember(result)
    }

    /// Receive one command (a single transfer of at most 512 bytes) on
    /// EP1 OUT into `out`; returns its length.
    pub fn recv(&mut self, out: &mut [u8]) -> Result<usize, Error> {
        self.check()?;
        let d = dma();
        flush(&d.small);
        let result = self.bulk(false, addr_of!(d.small) as usize, d.small.len(), false);
        let n = self.remember(result)?;
        invalidate(&d.small);
        let n = n.min(out.len());
        out[..n].copy_from_slice(&d.small[..n]);
        Ok(n)
    }

    /// Receive exactly `len` bytes into DRAM at `buf` (page aligned).
    pub fn recv_into(&mut self, buf: usize, len: usize) -> Result<(), Error> {
        self.check()?;
        let mut done = 0;
        let mut next_report = 64 << 20;
        while done < len {
            let chunk = (len - done).min(TD_BYTES * TD_CHAIN);
            cache::flush(buf + done, chunk);
            let result = self.bulk(false, buf + done, chunk, true);
            let got = self.remember(result)?;
            cache::invalidate(buf + done, chunk);
            if got != chunk {
                return self.remember(Err(Error::Transfer(got as u32)));
            }
            done += chunk;
            if done >= next_report {
                println!("[usb] {} MiB", done >> 20);
                next_report += 64 << 20;
            }
        }
        Ok(())
    }

    /// One bulk transfer on EP1, split over a descriptor chain. Returns the
    /// bytes moved (an OUT transfer ends early on a short packet).
    fn bulk(&mut self, dir_in: bool, buf: usize, len: usize, active: bool) -> Result<usize, Error> {
        let d = dma();
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
            flush(td);
        }
        let qh = 2 + dir_in as usize;
        d.qh[qh].next = addr_of!(tds[0]) as u32;
        d.qh[qh].token = 0;
        flush(&d.qh[qh]);
        let bit = ep_bit(EP1, dir_in);
        write32(EPPRIME, bit);
        let mut polls = 0;
        loop {
            if read32(EPCOMPLETE) & bit != 0 {
                write32(EPCOMPLETE, bit);
            }
            // Done when no descriptor in the chain is active any more.
            invalidate(&tds[count - 1]);
            if tds[count - 1].token & TD_ACTIVE == 0 {
                break;
            }
            // An OUT short packet retires the descriptor it ended in; the
            // ones after it stay active, so look for a short descriptor.
            if !dir_in {
                let mut short = false;
                for td in tds[..count].iter() {
                    invalidate(td);
                    if td.token & TD_ERRORS != 0 {
                        let token = td.token;
                        flush_endpoints(bit)?;
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
                    flush_endpoints(bit)?;
                    break;
                }
            }
            if let Err(e) = self.poll() {
                flush_endpoints(bit)?;
                return Err(e);
            }
            if active {
                polls += 1;
                if polls == TRANSFER_POLLS {
                    flush_endpoints(bit)?;
                    return Err(Error::Timeout);
                }
            }
        }
        let mut moved = 0;
        for td in tds[..count].iter() {
            invalidate(td);
            if td.token & TD_ERRORS != 0 {
                return Err(Error::Transfer(td.token));
            }
            if td.token & TD_ACTIVE != 0 {
                break;
            }
            let total = td_len(td);
            let left = (td.token >> 16 & 0x7fff) as usize;
            moved += total - left;
            if left != 0 {
                break;
            }
        }
        Ok(moved)
    }
}

/// The length a descriptor was programmed with (kept by fill_td).
fn td_len(td: &Td) -> usize {
    td._r as usize
}
