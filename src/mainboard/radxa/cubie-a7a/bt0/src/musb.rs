//! USB0 device controller of the A733 (feature `fastboot`): the Mentor
//! MUSB OTG controller at 0x04100000 in Allwinner's register layout, run
//! polled (CPU interrupts stay masked), PIO only, as the fastboot
//! transport ([`cubie_a7a_fastboot::proto::Transport`]).
//!
//! Sources: the vendor device tree (`udc-controller@4100000`,
//! `allwinner,sunxi-udc`, SPI 156; clocks bus_otg, usb_ref, res_dcap,
//! usb_sys_ahb; resets otg and phy); Linux's sunxi MUSB glue
//! (`drivers/usb/musb/sunxi.c`: the register remap, write-1-to-clear
//! interrupt status, VEND0 = 0 for PIO) and `phy-sun4i-usb.c` (ISCR, the
//! PHY control and the PHY0 route at +0x420); the A733 CCU registers
//! (USB0_CLK 0x1300, USB0_BGR 0x1304); main's `usb.rs` for the shared PHY0
//! (OTG +0x420 bit 0 routes it to this controller, EHCI0 +0x810 SIDDQ).
//!
//! # Taking the controller over from the mask ROM
//!
//! Under FEL the mask ROM drives this controller itself (1f3a:efe8, high
//! speed, interrupt driven): clocks, resets, PHY and the ISCR device-mode
//! forcing are all up. [`Musb::take_over`] keeps that hardware set-up as
//! it is (it only checks it, and brings it up itself only when the ROM did
//! not, [`cold_init`]), saves the ROM's controller registers, leaves the
//! bus (soft disconnect), installs its own endpoint set-up and connects
//! again as a fastboot device. [`restore_rom`] undoes this for the return
//! to FEL: disconnect, the ROM's registers back, connect; the host then
//! enumerates the ROM's FEL device again.
//!
//! Endpoints: EP0 (64 bytes, FIFO RAM 0..64), EP1 IN and EP1 OUT bulk
//! (512 bytes at high speed, 64 at full speed; FIFO RAM 64..576 and
//! 576..1088, single buffered).
use core::ptr::addr_of_mut;

use cubie_a7a_fastboot::proto::{Transport, PACKET};
use cubie_a7a_fastboot::usb::{self, Descriptors, Request, Setup, EP0_MAX};
use cubie_a7a_fastboot::xfer;
use util::mmio::{read16, read32, read8, write16, write32, write8};

use crate::{recovery, soc};

const BASE: usize = 0x0410_0000;
const fn fifo(ep: usize) -> usize {
    BASE + 4 * ep
}
const POWER: usize = BASE + 0x40;
const INDEX: usize = BASE + 0x42;
const VEND0: usize = BASE + 0x43;
const INTRTX: usize = BASE + 0x44;
const INTRRX: usize = BASE + 0x46;
const INTRTXE: usize = BASE + 0x48;
const INTRRXE: usize = BASE + 0x4a;
const INTRUSB: usize = BASE + 0x4c;
const INTRUSBE: usize = BASE + 0x50;
// Indexed endpoint registers (INDEX selects the endpoint).
const TXMAXP: usize = BASE + 0x80;
const CSR0: usize = BASE + 0x82;
const TXCSR: usize = BASE + 0x82;
const RXMAXP: usize = BASE + 0x84;
const RXCSR: usize = BASE + 0x86;
const COUNT0: usize = BASE + 0x88;
const RXCOUNT: usize = BASE + 0x88;
const TXFIFOSZ: usize = BASE + 0x90;
const TXFIFOADD: usize = BASE + 0x92;
const RXFIFOSZ: usize = BASE + 0x94;
const RXFIFOADD: usize = BASE + 0x96;
const FADDR: usize = BASE + 0x98;
// PHY side (phy-sun4i-usb "phy_ctrl" = OTG + 0x400).
const ISCR: usize = BASE + 0x400;
const PHYCTL: usize = BASE + 0x410;
const OTGCTL: usize = BASE + 0x420;
/// EHCI0's PMU block: the shared PHY0's SIDDQ (main `usb.rs`).
const PMU0_PHY_CTL: usize = 0x0410_1810;

const POWER_SOFTCONN: u8 = 0x40;
const POWER_HSENAB: u8 = 0x20;
const POWER_HSMODE: u8 = 0x10;
const POWER_RESET: u8 = 0x08;
const POWER_RESUME: u8 = 0x04;

const INTR_SUSPEND: u8 = 0x01;
const INTR_RESUME: u8 = 0x02;
const INTR_RESET: u8 = 0x04;
const INTR_DISCONNECT: u8 = 0x20;

const CSR0_RXPKTRDY: u16 = 0x0001;
const CSR0_TXPKTRDY: u16 = 0x0002;
const CSR0_SENTSTALL: u16 = 0x0004;
const CSR0_DATAEND: u16 = 0x0008;
const CSR0_SETUPEND: u16 = 0x0010;
const CSR0_SENDSTALL: u16 = 0x0020;
const CSR0_SVDRXPKTRDY: u16 = 0x0040;
const CSR0_SVDSETUPEND: u16 = 0x0080;
const CSR0_FLUSHFIFO: u16 = 0x0100;

const TXCSR_TXPKTRDY: u16 = 0x0001;
const TXCSR_FIFONOTEMPTY: u16 = 0x0002;
const TXCSR_UNDERRUN: u16 = 0x0004;
const TXCSR_FLUSHFIFO: u16 = 0x0008;
const TXCSR_SENTSTALL: u16 = 0x0020;
const TXCSR_CLRDATATOG: u16 = 0x0040;
const TXCSR_INCOMPTX: u16 = 0x0080;
const TXCSR_MODE: u16 = 0x2000;
/// Bits a write of 0 clears and a write of 1 leaves alone.
const TXCSR_WZC: u16 = TXCSR_INCOMPTX | TXCSR_SENTSTALL | TXCSR_UNDERRUN | TXCSR_FIFONOTEMPTY;

const RXCSR_RXPKTRDY: u16 = 0x0001;
const RXCSR_OVERRUN: u16 = 0x0004;
const RXCSR_FLUSHFIFO: u16 = 0x0010;
const RXCSR_SENTSTALL: u16 = 0x0040;
const RXCSR_CLRDATATOG: u16 = 0x0080;
const RXCSR_WZC: u16 = RXCSR_SENTSTALL | RXCSR_OVERRUN | RXCSR_RXPKTRDY;

const ISCR_ID_PULLUP_EN: u32 = 1 << 17;
const ISCR_DPDM_PULLUP_EN: u32 = 1 << 16;
const ISCR_FORCE_ID_HIGH: u32 = 3 << 14;
const ISCR_FORCE_VBUS_HIGH: u32 = 3 << 12;
const ISCR_FORCE_MASK: u32 = 0xf << 12;
/// VBUS, ID and DP/DM change-detect status: write 1 to clear, so masked
/// out of every read-modify-write.
const ISCR_W1C: u32 = 0x70;
const PHYCTL_VBUSVLDEXT: u32 = 1 << 5;
const PHY_SIDDQ: u32 = 1 << 3;
const OTGCTL_ROUTE_MUSB: u32 = 1 << 0;

// CCU (Linux ccu-sun60i-a733.c: bus-otg, RST_BUS_OTG, RST_USB_PHY0, usb01-ref,
// ahb-usb-sys, res-dcap-24m).
const CCU_USB0_CLK: usize = soc::CCU_BASE + 0x1300;
const CCU_USB0_BGR: usize = soc::CCU_BASE + 0x1304;
const CCU_USB_REF: usize = soc::CCU_BASE + 0x1340;
const CCU_AHB_GATE0: usize = soc::CCU_BASE + 0x5c0;
const CCU_RES_DCAP: usize = soc::CCU_BASE + 0x1a00;
const USB0_CLKEN: u32 = 1 << 31;
const USB0_PHY_RSTN: u32 = 1 << 30;
const USB0_DEVICE_GATE: u32 = 1 << 8;
const USB0_DEVICE_RST: u32 = 1 << 24;
const USB0_EHCI_GATE: u32 = 1 << 4;
const USB0_EHCI_RST: u32 = 1 << 20;
const USB_REF_GATE: u32 = 1 << 31;
const AHB_USB_SYS: u32 = 1 << 9;
const RES_DCAP_USB: u32 = 1 << 3;

/// FIFO RAM: EP0 owns 0..64; EP1 TX then RX, 512 bytes each.
const EP1_TX_FIFO: u16 = 64;
const EP1_RX_FIFO: u16 = 64 + 2 * 512;
/// FIFOSZ code: 2^(code + 3) bytes.
const FIFOSZ_512: u8 = 6;
/// FIFOSZ double packet buffering: two 512-byte buffers per direction, so
/// the host fills (or drains) one while the CPU copies the other.
const FIFOSZ_DPB: u8 = 0x10;

/// How long the host sees nothing on the bus before we connect again.
const DISCONNECT_MS: u64 = 200;
/// The host is streaming a download or reading a reply: per packet.
const PACKET_TIMEOUT_US: u64 = 5_000_000;

/// The endpoints whose set-up the ROM's FEL may use and ours changes.
const SAVED_EPS: usize = 4;

#[derive(Clone, Copy, Default)]
struct EpCfg {
    txmaxp: u16,
    rxmaxp: u16,
    txfifosz: u8,
    txfifoadd: u16,
    rxfifosz: u8,
    rxfifoadd: u16,
}

/// The mask ROM's controller registers at take-over.
#[derive(Clone, Copy, Default)]
struct RomState {
    power: u8,
    index: u8,
    vend0: u8,
    intrtxe: u16,
    intrrxe: u16,
    intrusbe: u8,
    iscr: u32,
    ep: [EpCfg; SAVED_EPS],
}

/// Set while fastboot owns the controller (so every way back to FEL puts
/// the ROM's set-up back first).
static mut ROM: Option<RomState> = None;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    /// The host reset the bus (or left it): enumerate again.
    Reset,
    Timeout(&'static str),
    /// The host sent more than the transfer allows.
    Overrun,
}

impl Error {
    pub fn message(&self) -> &'static str {
        match self {
            Error::Reset => "bus reset",
            Error::Timeout(w) => w,
            Error::Overrun => "overrun",
        }
    }
}

/// What [`Musb::take_over`] found.
#[derive(Clone, Copy, Debug)]
pub struct TakeOver {
    /// The ROM had clocks, resets, PHY route and device-mode forcing up.
    pub rom_ready: bool,
    pub rom_power: u8,
    pub rom_faddr: u8,
    pub iscr: u32,
    pub usb0_clk: u32,
    pub usb0_bgr: u32,
}

pub struct Musb<'a> {
    desc: Descriptors<'a>,
    configured: bool,
    ended: bool,
    high_speed: bool,
    pending_address: Option<u8>,
}

fn now() -> u64 {
    soc::time_us()
}

fn read_fifo(ep: usize, dst: &mut [u8]) {
    let f = fifo(ep);
    let (head, words, tail) = unsafe { dst.align_to_mut::<u32>() };
    for b in head {
        *b = read8(f);
    }
    for w in words {
        *w = read32(f);
    }
    for b in tail {
        *b = read8(f);
    }
}

fn write_fifo(ep: usize, src: &[u8]) {
    let f = fifo(ep);
    let (head, words, tail) = unsafe { src.align_to::<u32>() };
    for b in head {
        write8(f, *b);
    }
    for w in words {
        write32(f, *w);
    }
    for b in tail {
        write8(f, *b);
    }
}

fn clear_pending() {
    let u = read8(INTRUSB);
    write8(INTRUSB, u);
    let t = read16(INTRTX);
    write16(INTRTX, t);
    let r = read16(INTRRX);
    write16(INTRRX, r);
}

fn save_rom() -> RomState {
    let mut s = RomState {
        power: read8(POWER),
        index: read8(INDEX),
        vend0: read8(VEND0),
        intrtxe: read16(INTRTXE),
        intrrxe: read16(INTRRXE),
        intrusbe: read8(INTRUSBE),
        iscr: read32(ISCR) & !ISCR_W1C,
        ep: [EpCfg::default(); SAVED_EPS],
    };
    for (i, e) in s.ep.iter_mut().enumerate() {
        write8(INDEX, i as u8);
        *e = EpCfg {
            txmaxp: read16(TXMAXP),
            rxmaxp: read16(RXMAXP),
            txfifosz: read8(TXFIFOSZ),
            txfifoadd: read16(TXFIFOADD),
            rxfifosz: read8(RXFIFOSZ),
            rxfifoadd: read16(RXFIFOADD),
        };
    }
    write8(INDEX, s.index);
    s
}

/// Bring the controller and PHY0 up the way the vendor and Linux drivers
/// do, for when the mask ROM did not (not exercised under FEL, where the
/// ROM has them up; inferred from the sources in the module comment).
fn cold_init() {
    soc::set_bits(CCU_AHB_GATE0, AHB_USB_SYS);
    soc::set_bits(CCU_RES_DCAP, RES_DCAP_USB);
    soc::set_bits(CCU_USB_REF, USB_REF_GATE);
    soc::set_bits(CCU_USB0_CLK, USB0_CLKEN | USB0_PHY_RSTN);
    // EHCI0's bus clock and reset for its PMU block (PHY0 SIDDQ).
    soc::set_bits(CCU_USB0_BGR, USB0_EHCI_GATE | USB0_DEVICE_GATE);
    soc::set_bits(CCU_USB0_BGR, USB0_EHCI_RST | USB0_DEVICE_RST);
    soc::udelay(10);
    soc::clr_set_bits(PMU0_PHY_CTL, PHY_SIDDQ, 0);
    soc::clr_set_bits(PHYCTL, PHY_SIDDQ, PHYCTL_VBUSVLDEXT);
    soc::set_bits(OTGCTL, OTGCTL_ROUTE_MUSB);
    soc::udelay(100);
}

fn rom_ready() -> bool {
    let bgr = read32(CCU_USB0_BGR);
    bgr & (USB0_DEVICE_GATE | USB0_DEVICE_RST) == USB0_DEVICE_GATE | USB0_DEVICE_RST
        && read32(CCU_USB0_CLK) & USB0_PHY_RSTN != 0
        && read32(OTGCTL) & OTGCTL_ROUTE_MUSB != 0
}

/// Device mode whatever the cable says: ID high, VBUS valid, pull-ups.
fn force_device_mode() {
    let iscr = read32(ISCR) & !(ISCR_W1C | ISCR_FORCE_MASK);
    write32(
        ISCR,
        iscr | ISCR_ID_PULLUP_EN | ISCR_DPDM_PULLUP_EN | ISCR_FORCE_ID_HIGH | ISCR_FORCE_VBUS_HIGH,
    );
}

fn disconnect_bus() {
    write8(
        POWER,
        read8(POWER) & !(POWER_SOFTCONN | POWER_RESET | POWER_RESUME),
    );
}

/// Drop whatever EP0 holds (FLUSHFIFO only while a packet is there).
fn flush_ep0() {
    write8(INDEX, 0);
    if read16(CSR0) & (CSR0_RXPKTRDY | CSR0_TXPKTRDY) != 0 {
        write16(CSR0, CSR0_FLUSHFIFO);
    }
}

/// EP1's FIFOs and maximum packet sizes; data toggles cleared, FIFOs
/// flushed.
fn configure_ep1(maxp: u16) {
    write8(INDEX, usb::EP1);
    write8(TXFIFOSZ, FIFOSZ_512 | FIFOSZ_DPB);
    write16(TXFIFOADD, EP1_TX_FIFO / 8);
    write8(RXFIFOSZ, FIFOSZ_512 | FIFOSZ_DPB);
    write16(RXFIFOADD, EP1_RX_FIFO / 8);
    write16(TXMAXP, maxp);
    write16(RXMAXP, maxp);
    // Double buffered: up to two packets to flush.
    for _ in 0..2 {
        let mut tx = TXCSR_MODE | TXCSR_CLRDATATOG;
        if read16(TXCSR) & TXCSR_FIFONOTEMPTY != 0 {
            tx |= TXCSR_FLUSHFIFO;
        }
        write16(TXCSR, tx);
    }
    write16(RXCSR, RXCSR_CLRDATATOG | RXCSR_FLUSHFIFO);
    write16(RXCSR, RXCSR_CLRDATATOG | RXCSR_FLUSHFIFO);
}

impl<'a> Musb<'a> {
    /// Leave the ROM's FEL session and connect as `desc` (the ROM's set-up
    /// saved for [`restore_rom`]). Call with CPU interrupts masked.
    pub fn take_over(desc: Descriptors<'a>) -> (Self, TakeOver) {
        let found = TakeOver {
            rom_ready: rom_ready(),
            rom_power: read8(POWER),
            rom_faddr: read8(FADDR),
            iscr: read32(ISCR),
            usb0_clk: read32(CCU_USB0_CLK),
            usb0_bgr: read32(CCU_USB0_BGR),
        };
        // SAFETY: bt0 is single-threaded; ROM is only touched here and in
        // restore_rom.
        let rom = unsafe { &mut *addr_of_mut!(ROM) };
        if rom.is_none() {
            *rom = Some(save_rom());
        }
        disconnect_bus();
        write8(INTRUSBE, 0);
        write16(INTRTXE, 0);
        write16(INTRRXE, 0);
        if !found.rom_ready {
            cold_init();
        }
        force_device_mode();
        write8(VEND0, 0); // PIO
        write8(INDEX, 0);
        write8(FADDR, 0);
        configure_ep1(512);
        flush_ep0();
        soc::mdelay(DISCONNECT_MS);
        clear_pending();
        // Status latches for what the poll loop handles; the CPU keeps IRQs
        // masked, so nothing is taken.
        write8(
            INTRUSBE,
            INTR_RESET | INTR_DISCONNECT | INTR_SUSPEND | INTR_RESUME,
        );
        write16(INTRTXE, 0b11);
        write16(INTRRXE, 0b10);
        let p = read8(POWER) & !(POWER_RESET | POWER_RESUME);
        write8(POWER, p | POWER_HSENAB | POWER_SOFTCONN);
        (
            Musb {
                desc,
                configured: false,
                ended: false,
                high_speed: true,
                pending_address: None,
            },
            found,
        )
    }

    /// The controller as fastboot left it, configured, after the stack
    /// was reset (the DRAM lab mode resumes after a run this way).
    pub fn resume(desc: Descriptors<'a>) -> Self {
        Musb {
            desc,
            configured: true,
            ended: false,
            high_speed: read8(POWER) & POWER_HSMODE != 0,
            pending_address: None,
        }
    }

    pub fn high_speed(&self) -> bool {
        self.high_speed
    }

    fn maxp(&self) -> usize {
        if self.high_speed {
            512
        } else {
            64
        }
    }

    /// Bus events and EP0. Once a configured session ends, never accept
    /// another configuration in this transport, even if INFO ignored an error.
    fn poll(&mut self) -> Result<(), Error> {
        recovery::kick();
        if self.ended {
            return Err(Error::Reset);
        }
        let u = read8(INTRUSB);
        if u != 0 {
            write8(INTRUSB, u);
        }
        let tx = read16(INTRTX);
        if tx != 0 {
            write16(INTRTX, tx);
        }
        let rx = read16(INTRRX);
        if rx != 0 {
            write16(INTRRX, rx);
        }
        if u & (INTR_RESET | INTR_DISCONNECT) != 0 {
            let was = self.configured;
            self.configured = false;
            self.pending_address = None;
            write8(INDEX, 0);
            write8(FADDR, 0);
            self.high_speed = read8(POWER) & POWER_HSMODE != 0;
            configure_ep1(self.maxp() as u16);
            if was {
                self.ended = true;
                return Err(Error::Reset);
            }
        }
        if tx & 1 != 0 {
            let was = self.configured;
            self.ep0();
            if was && !self.configured {
                self.ended = true;
                return Err(Error::Reset);
            }
        }
        Ok(())
    }

    fn ep0(&mut self) {
        write8(INDEX, 0);
        let csr = read16(CSR0);
        // Still in the status stage (DATAEND not yet cleared by the core).
        if csr & CSR0_DATAEND != 0 {
            return;
        }
        // The status stage of SET_ADDRESS is over: the new address applies.
        if let Some(a) = self.pending_address.take() {
            write8(FADDR, a);
        }
        if csr & CSR0_SENTSTALL != 0 {
            write16(CSR0, csr & !CSR0_SENTSTALL);
        }
        if csr & CSR0_SETUPEND != 0 {
            write16(CSR0, CSR0_SVDSETUPEND);
        }
        if csr & CSR0_RXPKTRDY == 0 {
            return;
        }
        let n = (read8(COUNT0) & 0x7f) as usize;
        if n != 8 {
            // Never an OUT data stage here: drop it and stall.
            let mut junk = [0u8; EP0_MAX];
            read_fifo(0, &mut junk[..n.min(EP0_MAX)]);
            write16(CSR0, CSR0_SVDRXPKTRDY | CSR0_SENDSTALL);
            return;
        }
        let mut s = [0u8; 8];
        read_fifo(0, &mut s);
        let setup = Setup::parse(s);
        let mut buf = [0u8; 64];
        let request = usb::decode(&setup);
        if matches!(
            request,
            Request::GetDescriptor { .. } | Request::SetConfiguration(_)
        ) {
            // The chirp is long over by now; the reset interrupt may have
            // come before HSMODE settled.
            self.high_speed = read8(POWER) & POWER_HSMODE != 0;
        }
        let reply: Option<usize> = match request {
            Request::GetDescriptor { value } => {
                usb::descriptor(&self.desc, value, self.maxp() as u16, &mut buf)
            }
            Request::GetConfiguration => {
                buf[0] = self.configured as u8;
                Some(1)
            }
            Request::GetStatusDevice => {
                buf[..2].copy_from_slice(&[1, 0]);
                Some(2)
            }
            Request::GetStatusOther => {
                buf[..2].copy_from_slice(&[0, 0]);
                Some(2)
            }
            Request::SetAddress(a) => {
                self.pending_address = Some(a);
                Some(0)
            }
            Request::SetConfiguration(c) => {
                configure_ep1(self.maxp() as u16);
                self.configured = c == 1;
                write8(INDEX, 0);
                Some(0)
            }
            Request::ClearEndpointHalt { index } => {
                write8(INDEX, usb::EP1);
                match index {
                    0x81 => write16(
                        TXCSR,
                        ((read16(TXCSR) | TXCSR_WZC) & !TXCSR_TXPKTRDY) | TXCSR_CLRDATATOG,
                    ),
                    0x01 => write16(
                        RXCSR,
                        ((read16(RXCSR) | RXCSR_WZC) & !RXCSR_RXPKTRDY) | RXCSR_CLRDATATOG,
                    ),
                    _ => {}
                }
                write8(INDEX, 0);
                Some(0)
            }
            Request::SetInterface => Some(0),
            Request::Unsupported => None,
        };
        match reply.map(|len| len.min(setup.length as usize)) {
            None => write16(CSR0, CSR0_SVDRXPKTRDY | CSR0_SENDSTALL),
            Some(0) => write16(CSR0, CSR0_SVDRXPKTRDY | CSR0_DATAEND),
            Some(len) => {
                // Every reply fits one EP0 packet (the descriptors are at
                // most 62 bytes), so one packet ends the data stage.
                write16(CSR0, CSR0_SVDRXPKTRDY);
                // EP0's FIFO serves both directions: let the controller
                // release the SETUP packet first (Linux musb_gadget_ep0.c).
                let t0 = now();
                while read16(CSR0) & CSR0_RXPKTRDY != 0 && now() - t0 < 10_000 {}
                write_fifo(0, &buf[..len]);
                write16(CSR0, CSR0_TXPKTRDY | CSR0_DATAEND);
            }
        }
    }

    /// One EP1 OUT packet into `out`; its length. `timeout` (us) None waits
    /// for ever (commands), Some for a download in progress.
    fn read_packet(&mut self, out: &mut [u8], timeout: Option<u64>) -> Result<usize, Error> {
        let t0 = now();
        let csr = loop {
            self.poll()?;
            if !self.configured {
                return Err(Error::Reset);
            }
            write8(INDEX, usb::EP1);
            let csr = read16(RXCSR);
            if csr & RXCSR_RXPKTRDY != 0 {
                break csr;
            }
            if timeout.is_some_and(|t| now() - t0 > t) {
                return Err(Error::Timeout("receive"));
            }
        };
        let n = read16(RXCOUNT) as usize;
        let overrun = n > out.len();
        if overrun {
            let mut junk = [0u8; 4];
            for _ in 0..n {
                read_fifo(usb::EP1 as usize, &mut junk[..1]);
            }
        } else {
            read_fifo(usb::EP1 as usize, &mut out[..n]);
        }
        write16(RXCSR, (csr | RXCSR_WZC) & !RXCSR_RXPKTRDY);
        if overrun {
            return Err(Error::Overrun);
        }
        Ok(n)
    }

    /// One EP1 IN packet (at most maxp bytes, possibly empty).
    fn write_packet(&mut self, data: &[u8]) -> Result<(), Error> {
        let t0 = now();
        loop {
            self.poll()?;
            if !self.configured {
                return Err(Error::Reset);
            }
            write8(INDEX, usb::EP1);
            // Double buffered: a buffer is free once TXPKTRDY clears (the
            // other may still hold a packet for the host).
            if read16(TXCSR) & TXCSR_TXPKTRDY == 0 {
                break;
            }
            if now() - t0 > PACKET_TIMEOUT_US {
                return Err(Error::Timeout("send"));
            }
        }
        write_fifo(usb::EP1 as usize, data);
        let csr = read16(TXCSR) & !(TXCSR_FLUSHFIFO | TXCSR_CLRDATATOG);
        write16(TXCSR, csr | TXCSR_WZC | TXCSR_TXPKTRDY);
        Ok(())
    }
}

impl Transport for Musb<'_> {
    type Error = Error;

    fn wait_configured(&mut self) -> Result<(), Error> {
        while !self.configured {
            // A reset before configuration is just enumeration.
            self.poll()?;
        }
        Ok(())
    }

    fn recv(&mut self, out: &mut [u8]) -> Result<usize, Error> {
        debug_assert!(out.len() >= PACKET);
        self.read_packet(out, None)
    }

    /// A download: cubie_a7a_fastboot::xfer over EP1 OUT (double
    /// buffered; the full poll only every xfer::SERVICE_EVERY packets).
    fn recv_exact(&mut self, out: &mut [u8]) -> Result<(), Error> {
        let maxp = self.maxp();
        write8(INDEX, usb::EP1);
        xfer::recv_exact(&mut Bulk(self, 0), out, maxp, PACKET_TIMEOUT_US).map_err(stop_error)
    }

    /// A reply line (at most 64 bytes), as one short packet. At full
    /// speed a 64-byte line would fill the packet, and whether the host
    /// then waits for more depends on its read size (64 or 256 bytes by
    /// fastboot version), so such a line loses its last character instead.
    fn send(&mut self, data: &[u8]) -> Result<(), Error> {
        let maxp = self.maxp();
        let data = &data[..data.len().min(maxp - 1)];
        self.write_packet(data)
    }

    /// Upload data: full packets and a short last one, no zero-length
    /// packet (the host reads exactly the announced size).
    fn send_data(&mut self, data: &[u8]) -> Result<(), Error> {
        let maxp = self.maxp();
        write8(INDEX, usb::EP1);
        xfer::send(&mut Bulk(self, 0), data, maxp, PACKET_TIMEOUT_US).map_err(stop_error)
    }

    fn disconnect(&mut self) -> Result<(), Error> {
        // Let the host take the last reply first.
        let t0 = now();
        let drained = loop {
            if let Err(e) = self.poll() {
                break Err(e);
            }
            write8(INDEX, usb::EP1);
            if read16(TXCSR) & (TXCSR_TXPKTRDY | TXCSR_FIFONOTEMPTY) == 0 {
                break Ok(());
            }
            if now() - t0 > PACKET_TIMEOUT_US {
                break Err(Error::Timeout("last reply"));
            }
        };
        disconnect_bus();
        self.configured = false;
        self.ended = true;
        drained
    }

    fn log(&mut self, dir: &str, line: &[u8]) {
        let s = core::str::from_utf8(line).unwrap_or("?");
        println!("[fastboot] {dir} {s}");
    }
}

/// Leave the bus for good (before the hand-over to main, or a reset).
pub fn release() {
    disconnect_bus();
    write8(INTRUSBE, 0);
    write16(INTRTXE, 0);
    write16(INTRRXE, 0);
    clear_pending();
}

/// Put the mask ROM's controller set-up back (no-op if fastboot never
/// took it over) and connect again, so the ROM's FEL enumerates anew.
pub fn restore_rom() {
    // SAFETY: as in take_over.
    let Some(rom) = (unsafe { (*addr_of_mut!(ROM)).take() }) else {
        return;
    };
    release();
    soc::mdelay(DISCONNECT_MS);
    for (i, e) in rom.ep.iter().enumerate() {
        write8(INDEX, i as u8);
        if i > 0 {
            if read16(TXCSR) & TXCSR_FIFONOTEMPTY != 0 {
                write16(TXCSR, TXCSR_FLUSHFIFO);
            }
            let flush = if read16(TXCSR) & TXCSR_FIFONOTEMPTY != 0 {
                TXCSR_FLUSHFIFO
            } else {
                0
            };
            write16(TXCSR, flush | TXCSR_CLRDATATOG);
            write16(RXCSR, RXCSR_FLUSHFIFO | RXCSR_CLRDATATOG);
        }
        write8(TXFIFOSZ, e.txfifosz);
        write16(TXFIFOADD, e.txfifoadd);
        write8(RXFIFOSZ, e.rxfifosz);
        write16(RXFIFOADD, e.rxfifoadd);
        write16(TXMAXP, e.txmaxp);
        write16(RXMAXP, e.rxmaxp);
    }
    flush_ep0(); // leaves INDEX 0
    write8(FADDR, 0);
    write8(VEND0, rom.vend0);
    write32(ISCR, rom.iscr & !ISCR_W1C);
    clear_pending();
    write8(INTRUSBE, rom.intrusbe);
    write16(INTRTXE, rom.intrtxe);
    write16(INTRRXE, rom.intrrxe);
    write8(INDEX, rom.index);
    write8(POWER, rom.power & !(POWER_RESET | POWER_RESUME));
}

/// EP1 for the data phases, INDEX kept at 1 between packets (`service`
/// puts it back after the full poll); the second field is the RXCSR value
/// `ready` read, for `take`.
struct Bulk<'m, 'a>(&'m mut Musb<'a>, u16);

fn stop_error(s: xfer::Stop<Error>) -> Error {
    match s {
        xfer::Stop::Service(e) => e,
        xfer::Stop::Timeout => Error::Timeout("bulk transfer"),
        xfer::Stop::Overrun => Error::Overrun,
        xfer::Stop::Short => Error::Timeout("short download"),
    }
}

impl xfer::OutEp for Bulk<'_, '_> {
    type Error = Error;
    fn ready(&mut self) -> Option<usize> {
        let csr = read16(RXCSR);
        if csr & RXCSR_RXPKTRDY == 0 {
            return None;
        }
        self.1 = csr;
        Some(read16(RXCOUNT) as usize)
    }
    fn take(&mut self, out: &mut [u8]) {
        read_fifo(usb::EP1 as usize, out);
        write16(RXCSR, (self.1 | RXCSR_WZC) & !RXCSR_RXPKTRDY);
    }
    fn discard(&mut self) {
        let n = read16(RXCOUNT) as usize;
        for _ in 0..n {
            let _ = util::mmio::read8(fifo(usb::EP1 as usize));
        }
        write16(RXCSR, (self.1 | RXCSR_WZC) & !RXCSR_RXPKTRDY);
    }
    fn service(&mut self) -> Result<(), Error> {
        let r = self.0.poll();
        write8(INDEX, usb::EP1);
        r?;
        if self.0.configured {
            Ok(())
        } else {
            Err(Error::Reset)
        }
    }
    fn now_us(&mut self) -> u64 {
        now()
    }
}

impl xfer::InEp for Bulk<'_, '_> {
    type Error = Error;
    fn free(&mut self) -> bool {
        read16(TXCSR) & TXCSR_TXPKTRDY == 0
    }
    fn put(&mut self, data: &[u8]) {
        write_fifo(usb::EP1 as usize, data);
        let csr = read16(TXCSR) & !(TXCSR_FLUSHFIFO | TXCSR_CLRDATATOG);
        write16(TXCSR, csr | TXCSR_WZC | TXCSR_TXPKTRDY);
    }
    fn service(&mut self) -> Result<(), Error> {
        <Self as xfer::OutEp>::service(self)
    }
    fn now_us(&mut self) -> u64 {
        now()
    }
}

/// USB0 as a SoC reset leaves it, for the mask ROM's FEL entry (the DRAM
/// lab's way out, labfb.rs): off the bus, then the controller's and PHY0's
/// reset lines asserted again. The ROM's own USB set-up releases them and
/// programs the controller from its reset values, so nothing of the
/// fastboot session (address, speed, FIFO layout) is left for it to find.
/// The registers saved at take-over are dropped: they belong to a FEL
/// session that is gone. Bus clocks and gates stay on (the ROM only ever
/// sets them).
#[cfg(feature = "dram-lab")]
pub fn reset_for_rom() {
    release();
    // SAFETY: as in take_over.
    unsafe { *addr_of_mut!(ROM) = None };
    soc::clr_set_bits(CCU_USB0_BGR, USB0_DEVICE_RST, 0);
    soc::clr_set_bits(CCU_USB0_CLK, USB0_PHY_RSTN, 0);
    soc::udelay(100);
}
