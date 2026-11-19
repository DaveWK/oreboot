// SPDX-License-Identifier: GPL-2.0-only

//! EHCI1 host for `fetch:usb0` (feature `fastboot`): the blue USB-A port's
//! high-speed pair, one mass storage disk, polled, read-only.
//!
//! The blue port's USB 2.0 pair is wired to USB1/EHCI1 (RS501 block
//! diagram); with combo0 off, a USB 3 disk connects there at high speed.
//! That is how the FEL boot runs Fedora from a thumb drive (main feature
//! `usb-hs-only`, EDK2's EHCI driver), so this path needs neither the
//! dwc3/xHCI, combo0 nor the PCK-600 USB2 domain.
//!
//! Bring-up: main's `usb.rs` order for USB1 (`ccu_init`, `phy_init`:
//! master gates, MSI_LITE2, RES_DCAP, USB1 clock/gates/resets, the 24 MHz
//! reference, `res_cal(1)`, `hci_phy_enable(EHCI1)`, usb1-vbus PM5),
//! except that shared clock registers only get bits set, never rewritten:
//! USB0 (the fastboot device) shares them. Then an EHCI reset, one port
//! reset, enumeration at address 1 and the Bulk-Only interface.
//!
//! Every control request passes `usbhost::control_allowed`, every bulk
//! command is built by `msc::cbw` (read-only SCSI allow-list); this module
//! adds no request of its own. Descriptors, qTDs and small buffers live in
//! DRAM at [`DMA`] (bt0 runs with caches off, so no cache maintenance);
//! bulk data goes straight to the caller's DRAM buffer, or through a
//! bounce buffer when the caller's is not in DRAM.
use core::arch::asm;

use cubie_a7a_fastboot::msc::{Pipe, XferError};
use cubie_a7a_fastboot::usb::Setup;
use cubie_a7a_fastboot::usbhost::{self, MscInterface};
use util::mmio::{read32, write32};

use crate::{recovery, soc};

const EHCI1: usize = 0x0420_0000;
const CCU: usize = soc::CCU_BASE;
// main usb.rs (BSP ccu-sun60iw2.c, live Linux values).
const AHB_GATE: usize = CCU + 0x05c0;
const AHB_GATE_USB: u32 = (1 << 9) | (1 << 8);
const MBUS_GATE: usize = CCU + 0x05e0;
const MBUS_GATE_USB: u32 = (1 << 31) | (1 << 28);
const MSI_LITE2: usize = CCU + 0x05a4;
const MSI_LITE2_ON: u32 = (1 << 0) | (1 << 16) | (1 << 17);
const RES_DCAP: usize = CCU + 0x1a00;
const RES_DCAP_USB_PHY: u32 = 1 << 3;
const USB1_CLK: usize = CCU + 0x1308;
const USB1_GATE_RST: usize = CCU + 0x130c;
const USB_REF: usize = CCU + 0x1340;
const USB_CLK_LIVE: u32 = 0xc000_0000;
const USB_PHY_RSTN: u32 = 1 << 30;
const USB_GATE_RST_LIVE: u32 = 0x0011_0011;
const USB_REF_GATE: u32 = 1 << 31;
const SYSCFG_RESCAL_CTRL: usize = 0x0300_0000 + 0x160;
const SYSCFG_RES0_CTRL: usize = 0x0300_0000 + 0x164;
const RESCAL_CAL_EN: u32 = 1 << 0;
const HCI_USB_CTRL: usize = 0x800;
const HCI_CTRL_3: usize = 0x808;
const HCI_PHY_CTRL: usize = 0x810;
const HCI_PHY_TUNE: usize = 0x818;
const HCI_81C: usize = 0x81c;
const HCI_SIDDQ: u32 = 1 << 3;
const HCI_FORCE_SUSPEND: u32 = 1 << 9;
const HCI_PHY_TUNE_VALUE: u32 = 0x0633_38c6;
const HCI_PASSBY: u32 = (1 << 15) | (1 << 11) | (1 << 9) | (1 << 8) | (1 << 0);
const R_PIO: usize = 0x0702_5000;
const BANK_M: usize = 0x30;
const PIN_USB1_VBUS: u32 = 5; // PM5

// EHCI operational registers (from base + CAPLENGTH).
const USBCMD: usize = 0x00;
const USBSTS: usize = 0x04;
const USBINTR: usize = 0x08;
const CTRLDSSEGMENT: usize = 0x10;
const PERIODICLISTBASE: usize = 0x14;
const ASYNCLISTADDR: usize = 0x18;
const CONFIGFLAG: usize = 0x40;
const PORTSC: usize = 0x44;
const CMD_RS: u32 = 1 << 0;
const CMD_HCRESET: u32 = 1 << 1;
const CMD_ASE: u32 = 1 << 5;
const CMD_ITC_8: u32 = 8 << 16;
const STS_HCH: u32 = 1 << 12;
const STS_ASS: u32 = 1 << 15;
const PORT_CCS: u32 = 1 << 0;
const PORT_CSC: u32 = 1 << 1;
const PORT_PE: u32 = 1 << 2;
const PORT_PEC: u32 = 1 << 3;
const PORT_OCC: u32 = 1 << 5;
const PORT_PR: u32 = 1 << 8;
const PORT_PP: u32 = 1 << 12;
const PORT_W1C: u32 = PORT_CSC | PORT_PEC | PORT_OCC;

/// DRAM for the schedule: QHs, qTDs, setup and bounce buffers; above the
/// fastboot download buffer (0x48000000 + 128 MiB).
pub const DMA: usize = 0x5800_0000;
const QH_CTRL: usize = DMA;
const QH_IN: usize = DMA + 0x40;
const QH_OUT: usize = DMA + 0x80;
const QTD0: usize = DMA + 0x100;
const QTDS: usize = 24;
const QTD_SIZE: usize = 32;
const SETUP_BUF: usize = DMA + 0x1000;
const BOUNCE: usize = DMA + 0x2000;
const BOUNCE_LEN: usize = 0x1000;

const T: u32 = 1;
const TOK_ACTIVE: u32 = 1 << 7;
const TOK_HALTED: u32 = 1 << 6;
const TOK_ERRORS: u32 = (1 << 5) | (1 << 4) | (1 << 3);
const TOK_CERR3: u32 = 3 << 10;
const PID_OUT: u32 = 0;
const PID_IN: u32 = 1 << 8;
const PID_SETUP: u32 = 2 << 8;
const TOK_DT: u32 = 1 << 31;

const CONTROL_TIMEOUT_US: u64 = 500_000;
const BULK_TIMEOUT_US: u64 = 5_000_000;
const DEV_ADDR: u8 = 1;

fn barrier() {
    unsafe { asm!("dsb sy") };
}

fn w(addr: usize, v: u32) {
    write32(addr, v)
}

fn r(addr: usize) -> u32 {
    read32(addr)
}

fn set_bits(addr: usize, bits: u32) {
    let v = r(addr);
    if v & bits != bits {
        w(addr, v | bits);
    }
}

fn in_dram(p: usize, len: usize) -> bool {
    p >= 0x4000_0000 && p.checked_add(len).is_some_and(|e| e <= 0xc000_0000)
}

/// What [`Ehci::open`] found, for the console and `oem usb0`.
#[derive(Clone, Copy, Debug)]
pub struct Found {
    pub vid: u16,
    pub pid: u16,
    pub msc: MscInterface,
    pub rescal_before: u32,
}

pub struct Ehci {
    op: usize,
    msc: MscInterface,
    ep0_maxp: u16,
}

#[derive(Clone, Copy, Debug)]
pub enum Error {
    NotAlive(u32),
    Timeout(&'static str),
    NoDevice,
    NotHighSpeed(u32),
    Control(&'static str, XferError),
    NoMassStorage,
}

impl Error {
    /// The register value behind the error, if any.
    pub fn detail(&self) -> u32 {
        match self {
            Error::NotAlive(v) | Error::NotHighSpeed(v) => *v,
            _ => 0,
        }
    }

    pub fn message(&self) -> &'static str {
        match self {
            Error::NotAlive(_) => "EHCI1 does not answer (clocks?)",
            Error::Timeout(w) => w,
            Error::NoDevice => "no device on the blue USB port",
            Error::NotHighSpeed(_) => "device is not high speed (EHCI only)",
            Error::Control(w, _) => w,
            Error::NoMassStorage => "no Bulk-Only mass storage interface",
        }
    }
}

fn wait(addr: usize, mask: u32, want: u32, us: u64) -> bool {
    let t0 = soc::time_us();
    loop {
        if r(addr) & mask == want {
            return true;
        }
        if soc::time_us() - t0 > us {
            return false;
        }
    }
}

/// main usb.rs ccu_init/phy_init for USB1 only; shared registers are only
/// ever OR-ed (USB0 runs the fastboot device meanwhile).
fn bring_up() -> u32 {
    set_bits(AHB_GATE, AHB_GATE_USB);
    set_bits(MBUS_GATE, MBUS_GATE_USB);
    set_bits(MSI_LITE2, MSI_LITE2_ON);
    set_bits(RES_DCAP, RES_DCAP_USB_PHY);
    soc::udelay(10);
    w(USB1_CLK, USB_CLK_LIVE);
    w(USB1_GATE_RST, USB_GATE_RST_LIVE);
    set_bits(USB_REF, USB_REF_GATE);
    soc::udelay(100);
    // res_cal(1): this port's RES200_SEL and trim only (and CAL_EN off, as
    // main does for every port).
    let rescal = r(SYSCFG_RESCAL_CTRL);
    w(SYSCFG_RESCAL_CTRL, (rescal & !RESCAL_CAL_EN) | (1 << 5));
    w(SYSCFG_RES0_CTRL, r(SYSCFG_RES0_CTRL) & !(0xff << 8));
    soc::udelay(10);
    // hci_phy_enable(EHCI1): PHY reset, SIDDQ off, passby, tune, release.
    w(USB1_CLK, USB_CLK_LIVE & !USB_PHY_RSTN);
    soc::udelay(200);
    w(
        EHCI1 + HCI_CTRL_3,
        r(EHCI1 + HCI_CTRL_3) & !HCI_FORCE_SUSPEND,
    );
    w(EHCI1 + HCI_PHY_CTRL, r(EHCI1 + HCI_PHY_CTRL) & !HCI_SIDDQ);
    set_bits(EHCI1 + HCI_USB_CTRL, HCI_PASSBY);
    w(EHCI1 + HCI_PHY_TUNE, HCI_PHY_TUNE_VALUE);
    w(EHCI1 + HCI_81C, 0x53);
    w(USB1_CLK, USB_CLK_LIVE);
    soc::udelay(1000);
    // usb1-vbus (PM5), active high: data bit first, then output.
    let cfg = R_PIO + BANK_M + (PIN_USB1_VBUS as usize / 8) * 4;
    let shift = (PIN_USB1_VBUS % 8) * 4;
    set_bits(R_PIO + BANK_M + 0x10, 1 << PIN_USB1_VBUS);
    w(cfg, (r(cfg) & !(0xf << shift)) | (1 << shift));
    rescal
}

/// One qTD: next, alternate next, token, five buffer pointers.
fn qtd(i: usize) -> usize {
    QTD0 + i * QTD_SIZE
}

fn write_qtd(at: usize, next: u32, alt: u32, token: u32, buf: usize) {
    w(at, next);
    w(at + 4, alt);
    w(at + 8, token);
    for k in 0..5 {
        let p = if k == 0 {
            buf
        } else {
            (buf & !0xfff) + k * 0x1000
        };
        w(at + 12 + 4 * k, p as u32);
    }
}

impl Ehci {
    /// Bring EHCI1 up, reset the port, enumerate the device and find its
    /// Bulk-Only interface. Takes up to ~4 s with a slow disk.
    pub fn open() -> Result<(Ehci, Found), Error> {
        let rescal_before = bring_up();
        let cap = r(EHCI1);
        let caplength = (cap & 0xff) as usize;
        if !(0x10..0x40).contains(&caplength) || cap >> 16 == 0 || cap == u32::MAX {
            return Err(Error::NotAlive(cap));
        }
        let op = EHCI1 + caplength;
        let hcs = r(EHCI1 + 4);
        w(op + USBCMD, r(op + USBCMD) & !CMD_RS);
        wait(op + USBSTS, STS_HCH, STS_HCH, 20_000);
        w(op + USBCMD, CMD_HCRESET);
        if !wait(op + USBCMD, CMD_HCRESET, 0, 250_000) {
            return Err(Error::Timeout("EHCI reset"));
        }
        w(op + USBINTR, 0);
        w(op + CTRLDSSEGMENT, 0);
        w(op + PERIODICLISTBASE, 0);
        w(op + ASYNCLISTADDR, QH_CTRL as u32);
        w(op + USBCMD, CMD_ITC_8 | CMD_RS);
        w(op + CONFIGFLAG, 1);
        if !wait(op + USBSTS, STS_HCH, 0, 20_000) {
            return Err(Error::Timeout("EHCI run"));
        }
        if hcs & (1 << 4) != 0 {
            set_bits(op + PORTSC, PORT_PP);
        }
        // Connect (VBUS has just come up; a USB 3 disk may take a while).
        let t0 = soc::time_us();
        while r(op + PORTSC) & PORT_CCS == 0 {
            recovery::kick();
            if soc::time_us() - t0 > 3_000_000 {
                return Err(Error::NoDevice);
            }
        }
        soc::mdelay(100); // debounce
        let p = r(op + PORTSC) & !(PORT_W1C | PORT_PE);
        w(op + PORTSC, p | PORT_PR);
        soc::mdelay(50);
        w(op + PORTSC, r(op + PORTSC) & !(PORT_W1C | PORT_PR));
        if !wait(op + PORTSC, PORT_PR, 0, 10_000) {
            return Err(Error::Timeout("port reset"));
        }
        soc::mdelay(10);
        let ps = r(op + PORTSC);
        if ps & PORT_PE == 0 {
            return Err(Error::NotHighSpeed(ps));
        }
        let mut e = Ehci {
            op,
            msc: MscInterface {
                configuration: 0,
                interface: 0,
                ep_in: 0,
                ep_out: 0,
                maxp_in: 512,
                maxp_out: 512,
            },
            ep0_maxp: 64,
        };
        // Enumerate: device descriptor at address 0, address 1, config.
        let mut d = [0u8; 18];
        e.control(0, usbhost::setup(0x80, 6, 0x0100, 0, 8), &mut d[..8])
            .map_err(|x| Error::Control("GET_DESCRIPTOR(device)", x))?;
        e.ep0_maxp = (d[7] as u16).max(8);
        e.control(0, usbhost::setup(0x00, 5, DEV_ADDR as u16, 0, 0), &mut [])
            .map_err(|x| Error::Control("SET_ADDRESS", x))?;
        soc::mdelay(5);
        e.control(DEV_ADDR, usbhost::setup(0x80, 6, 0x0100, 0, 18), &mut d)
            .map_err(|x| Error::Control("GET_DESCRIPTOR(device)", x))?;
        let mut cfg = [0u8; 512];
        e.control(
            DEV_ADDR,
            usbhost::setup(0x80, 6, 0x0200, 0, 9),
            &mut cfg[..9],
        )
        .map_err(|x| Error::Control("GET_DESCRIPTOR(config)", x))?;
        let total = (u16::from_le_bytes([cfg[2], cfg[3]]) as usize).min(cfg.len());
        let n = e
            .control(
                DEV_ADDR,
                usbhost::setup(0x80, 6, 0x0200, 0, total as u16),
                &mut cfg[..total],
            )
            .map_err(|x| Error::Control("GET_DESCRIPTOR(config)", x))?;
        let msc = usbhost::find_msc(&cfg[..n]).ok_or(Error::NoMassStorage)?;
        e.control(
            DEV_ADDR,
            usbhost::setup(0x00, 9, msc.configuration as u16, 0, 0),
            &mut [],
        )
        .map_err(|x| Error::Control("SET_CONFIGURATION", x))?;
        e.msc = msc;
        // Bulk QHs start at DATA0 (their overlays keep the toggles).
        w(QH_IN + 0x18, 0);
        w(QH_OUT + 0x18, 0);
        let found = Found {
            vid: u16::from_le_bytes([d[8], d[9]]),
            pid: u16::from_le_bytes([d[10], d[11]]),
            msc,
            rescal_before,
        };
        Ok((e, found))
    }

    /// Point the async schedule at `qh` (itself its only member), run the
    /// qTDs `first..=last` and wait. Returns the bytes not transferred,
    /// summed over the qTDs done. With `stop_on_short`, a short IN packet
    /// ends the transfer (bulk: the qTD's alternate next is an inactive
    /// terminator); without, the transfer runs to `last` (control: a short
    /// data stage goes on to the status stage).
    #[allow(clippy::too_many_arguments)]
    fn run(
        &self,
        qh: usize,
        chars: u32,
        first: usize,
        last: usize,
        keep_dt: bool,
        stop_on_short: bool,
        us: u64,
    ) -> Result<u32, XferError> {
        let overlay_dt = if keep_dt { r(qh + 0x18) & TOK_DT } else { 0 };
        w(qh, qh as u32 | 2);
        w(qh + 4, chars);
        w(qh + 8, 1 << 30);
        w(qh + 0x0c, 0);
        w(qh + 0x10, qtd(first) as u32);
        w(qh + 0x14, T);
        w(qh + 0x18, overlay_dt);
        for k in 0..5 {
            w(qh + 0x1c + 4 * k, 0);
        }
        barrier();
        let op = self.op;
        w(op + ASYNCLISTADDR, qh as u32);
        w(op + USBCMD, r(op + USBCMD) | CMD_ASE);
        let mut result = Err(XferError::Timeout);
        if wait(op + USBSTS, STS_ASS, STS_ASS, 10_000) {
            let t0 = soc::time_us();
            'poll: loop {
                recovery::kick();
                let mut left = 0;
                for i in first..=last {
                    let tok = r(qtd(i) + 8);
                    if tok & TOK_HALTED != 0 {
                        result = Err(if tok & TOK_ERRORS != 0 {
                            XferError::Error
                        } else {
                            XferError::Stall
                        });
                        break 'poll;
                    }
                    if tok & TOK_ACTIVE != 0 {
                        break;
                    }
                    let rem = (tok >> 16) & 0x7fff;
                    left += rem;
                    if i == last || (stop_on_short && rem != 0 && tok & (3 << 8) == PID_IN) {
                        result = Ok(left);
                        break 'poll;
                    }
                }
                if soc::time_us() - t0 > us {
                    break;
                }
            }
        }
        w(op + USBCMD, r(op + USBCMD) & !CMD_ASE);
        wait(op + USBSTS, STS_ASS, 0, 10_000);
        result
    }

    fn chars(addr: u8, ep: u8, maxp: u16, control: bool) -> u32 {
        (addr as u32)
            | ((ep as u32 & 0xf) << 8)
            | (2 << 12)
            | ((control as u32) << 14)
            | (1 << 15)
            | ((maxp as u32) << 16)
    }

    /// One control transfer (after the allow-list); bytes of data moved.
    fn control(&mut self, addr: u8, s: Setup, data: &mut [u8]) -> Result<usize, XferError> {
        if !usbhost::control_allowed(&s)
            || data.len() != s.length as usize
            || data.len() > BOUNCE_LEN
        {
            return Err(XferError::Error);
        }
        let setup = usbhost::setup_bytes(&s);
        for (i, b) in setup.chunks(4).enumerate() {
            w(
                SETUP_BUF + 4 * i,
                u32::from_le_bytes([b[0], b[1], b[2], b[3]]),
            );
        }
        let dir_in = s.request_type & 0x80 != 0;
        if !dir_in {
            for (i, &b) in data.iter().enumerate() {
                unsafe { core::ptr::write_volatile((BOUNCE + i) as *mut u8, b) };
            }
        }
        let n = data.len() as u32;
        let status = if data.is_empty() { 1 } else { 2 };
        write_qtd(
            qtd(0),
            qtd(1) as u32,
            T,
            TOK_ACTIVE | TOK_CERR3 | PID_SETUP | (8 << 16),
            SETUP_BUF,
        );
        if n > 0 {
            let pid = if dir_in { PID_IN } else { PID_OUT };
            write_qtd(
                qtd(1),
                qtd(2) as u32,
                qtd(2) as u32,
                TOK_DT | TOK_ACTIVE | TOK_CERR3 | pid | (n << 16),
                BOUNCE,
            );
        }
        let spid = if dir_in && n > 0 { PID_OUT } else { PID_IN };
        write_qtd(
            qtd(status),
            T,
            T,
            TOK_DT | TOK_ACTIVE | TOK_CERR3 | spid | (1 << 15),
            0,
        );
        let chars = Self::chars(addr, 0, self.ep0_maxp, true);
        self.run(QH_CTRL, chars, 0, status, false, false, CONTROL_TIMEOUT_US)?;
        // What a short IN data stage left is in the data qTD.
        let got = if n > 0 {
            n - ((r(qtd(1) + 8) >> 16) & 0x7fff)
        } else {
            0
        };
        if dir_in {
            for (i, b) in data.iter_mut().enumerate().take(got as usize) {
                *b = unsafe { core::ptr::read_volatile((BOUNCE + i) as *const u8) };
            }
        }
        Ok(got as usize)
    }

    /// A bulk transfer on the IN or OUT pipe of `len` bytes at DRAM `buf`.
    fn bulk(&mut self, in_ep: bool, buf: usize, len: usize) -> Result<usize, XferError> {
        let (qh, ep, maxp) = if in_ep {
            (QH_IN, self.msc.ep_in, self.msc.maxp_in)
        } else {
            (QH_OUT, self.msc.ep_out, self.msc.maxp_out)
        };
        let maxp = maxp.max(64) as usize;
        let pid = if in_ep { PID_IN } else { PID_OUT };
        // Split into qTDs of up to 5 pages, whole packets each but the last;
        // an inactive terminator catches a short packet (alternate next).
        let mut segs = 0;
        let mut at = buf;
        let mut left = len;
        let term = qtd(QTDS - 1);
        write_qtd(term, T, T, 0, 0);
        loop {
            if segs == QTDS - 1 {
                return Err(XferError::Error);
            }
            let room = (0x5000 - (at & 0xfff)) / maxp * maxp;
            let n = left.min(room);
            left -= n;
            let next = if left == 0 { T } else { qtd(segs + 1) as u32 };
            let ioc = if left == 0 { 1 << 15 } else { 0 };
            write_qtd(
                qtd(segs),
                next,
                term as u32,
                TOK_ACTIVE | TOK_CERR3 | pid | ioc | ((n as u32) << 16),
                at,
            );
            at += n;
            segs += 1;
            if left == 0 {
                break;
            }
        }
        let chars = Self::chars(DEV_ADDR, ep, maxp as u16, false);
        // The CPU's cache and the controller's DMA (cache.rs; no-ops with
        // the cache off or for the non-cacheable schedule MiB).
        if in_ep {
            crate::cache::clean_inv(buf, len);
        } else {
            crate::cache::clean(buf, len);
        }
        let r = self.run(qh, chars, 0, segs - 1, true, true, BULK_TIMEOUT_US);
        if in_ep {
            crate::cache::inv(buf, len);
        }
        let rem = r?;
        Ok(len - rem as usize)
    }

    /// Stop the controller (before the hand-over or the return to FEL).
    pub fn shutdown(&mut self) {
        let op = self.op;
        w(op + USBCMD, r(op + USBCMD) & !(CMD_ASE | CMD_RS));
        wait(op + USBSTS, STS_HCH, STS_HCH, 20_000);
        w(op + USBCMD, CMD_HCRESET);
        wait(op + USBCMD, CMD_HCRESET, 0, 250_000);
    }
}

impl Pipe for Ehci {
    fn bulk_out(&mut self, data: &[u8]) -> Result<(), XferError> {
        let p = data.as_ptr() as usize;
        let buf = if in_dram(p, data.len()) {
            p
        } else {
            if data.len() > BOUNCE_LEN {
                return Err(XferError::Error);
            }
            for (i, &b) in data.iter().enumerate() {
                unsafe { core::ptr::write_volatile((BOUNCE + i) as *mut u8, b) };
            }
            BOUNCE
        };
        barrier();
        let n = self.bulk(false, buf, data.len())?;
        if n != data.len() {
            return Err(XferError::Error);
        }
        Ok(())
    }

    fn bulk_in(&mut self, buf: &mut [u8]) -> Result<usize, XferError> {
        let p = buf.as_ptr() as usize;
        if in_dram(p, buf.len()) {
            return self.bulk(true, p, buf.len());
        }
        if buf.len() > BOUNCE_LEN {
            return Err(XferError::Error);
        }
        let n = self.bulk(true, BOUNCE, buf.len())?;
        for (i, b) in buf.iter_mut().enumerate().take(n) {
            *b = unsafe { core::ptr::read_volatile((BOUNCE + i) as *const u8) };
        }
        Ok(n)
    }

    fn clear_halt(&mut self, in_ep: bool) -> Result<(), XferError> {
        let ep = if in_ep {
            self.msc.ep_in
        } else {
            self.msc.ep_out
        };
        self.control(DEV_ADDR, usbhost::setup(0x02, 1, 0, ep as u16, 0), &mut [])?;
        // The device's toggle is back at DATA0.
        w(if in_ep { QH_IN } else { QH_OUT } + 0x18, 0);
        Ok(())
    }

    fn reset(&mut self) -> Result<(), XferError> {
        let (t, rq) = usbhost::BOT_RESET;
        self.control(
            DEV_ADDR,
            usbhost::setup(t, rq, 0, self.msc.interface as u16, 0),
            &mut [],
        )
        .map(|_| ())
    }

    fn delay_ms(&mut self, ms: u32) {
        recovery::kick();
        soc::mdelay(ms as u64);
    }
}
