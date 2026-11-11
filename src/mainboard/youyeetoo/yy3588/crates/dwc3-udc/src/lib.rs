// SPDX-License-Identifier: GPL-2.0-only
//! Polled USB 2.0 device on a Synopsys DesignWare USB3 (DWC3) controller
//! in device mode, offering the one-interface bulk function of
//! `usb-device` (fastboot): core soft reset, GCTL/DCFG/DCTL set-up, one
//! event buffer, endpoint 0 control transfers and one bulk IN/OUT pair,
//! High-Speed (or Full-Speed) only. Written for the RK3588 USB3OTG0 behind
//! the YY3588 USB-C port; the SoC part (GRF, PHY, clocks) is the board's
//! [`Dwc3Platform`].
//!
//! Sources, cited per item:
//! - "TRM p.N": Rockchip RK3588 TRM Part 2 V1.0, Chapter 13 "USB3
//!   Controller" (register descriptions pp.2187-2228).
//! - "U-Boot <file>": mainline U-Boot `drivers/usb/dwc3/` (GPL-2.0, itself
//!   from Linux v3.19; Felipe Balbi, Sebastian Andrzej Siewior, TI) at
//!   934995f642a2: the bring-up order, the endpoint commands and the ep0
//!   three-stage state machine follow `core.c` dwc3_core_init /
//!   dwc3_phy_setup, `gadget.c` dwc3_gadget_start / __dwc3_gadget_ep_enable
//!   / dwc3_gadget_conndone_interrupt / dwc3_gadget_reset_interrupt, and
//!   `ep0.c` dwc3_ep0_start_trans / dwc3_ep0_xfernotready.
//! - "Linux <file>": mainline Linux `drivers/usb/dwc3/` (GPL-2.0) at
//!   7475ea8b5861, where it differs (ep0 back to the SETUP phase on a bus
//!   reset, `ep0.c` dwc3_ep0_reset_state).
//! - The RK3588 quirks (`phy_type = "utmi_wide"`, `snps,dis_enblslpm_quirk`,
//!   `snps,dis-u2-freeclk-exists-quirk`, `snps,dis-del-phy-power-chg-quirk`)
//!   are those of the usb_host0_xhci node in Linux
//!   arch/arm64/boot/dts/rockchip/rk3588-base.dtsi.
//!
//! Deliberate differences from U-Boot, all to keep the polled first stage
//! simple: GUSB2PHYCFG.SUSPHY stays 0 (so endpoint commands need no
//! SUSPHY/ENBLSLPM save-and-restore, U-Boot gadget.c
//! dwc3_send_gadget_ep_cmd); USB 2.0 LPM is never offered (no DCFG.LPMCap,
//! the descriptors are USB 2.00 without BOS); no hibernation or scratch
//! buffers; one TRB per transfer (bulk data in chunks of [`XFER_MAX`]).
//!
//! Core revisions ([`Revision`]): the RK3588 USB3OTG0 read GSNPSID
//! 0x5533300a, DWC_usb3 **3.00a**, on the first hardware run (2026-11-09);
//! the model had assumed 3.30a. Every revision check in Linux
//! `drivers/usb/dwc3/core.c`, `gadget.c` and `ep0.c` (v7.3-rc6, 1211231e2915,
//! `DWC3_VER_IS*`) was reviewed against what this driver writes:
//!
//! | Linux check | Field | Here |
//! |---|---|---|
//! | core.c dwc3_phy_setup / dwc3_enable_susphy, > 1.94a | GUSB3PIPECTL.SUSPHY | set only when > 1.94a (gated) |
//! | gadget.c dwc3_gadget_conndone_interrupt, > 1.94a | DCFG.LPMCap, DCTL.HIRD_Thres | LPM never offered: LPMCap cleared on every revision |
//! | gadget.c dwc3_gadget_run_stop, >= 1.94a | DCTL.KeepConnect cleared | cleared on every revision (reserved before 1.94a [I]) |
//! | gadget.c dwc3_gadget_enable_irq, < 2.50a / >= 2.30a | DEVTEN.ULStCngEn / U3L2L1SuspEn | the U-Boot set (ULStCng on, U3L2L1Susp off) on every revision; link-state and suspend events are ignored |
//! | core.c dwc3_core_init GUCTL1, >= 2.90a / >= 3.00a | DEV_L1_EXIT_BY_HW / DEV_DECOUPLE_L1L2_EVT | GUCTL1 not written (no LPM) |
//! | core.c dwc3_core_init GUCTL2, DWC_usb3 >= 3.10a | RST_ACTBITLATER (End Transfer polling) | GUCTL2 not written: End Transfer is CmdIOC + 100 us, the pre-3.10a model, on every revision |
//! | core.c GUCTL2, == 3.20a | LC_TIMER | not written |
//! | gadget.c dwc3_send_clear_stall_ep_cmd, >= 2.60a and SuperSpeed | DEPCMD.ClearPendIN | High-Speed only: never set |
//! | gadget.c TX FIFO sizing, < / >= 2.90a | GTXFIFOSIZn | not written (reset FIFO map) |
//! | core.c GFLADJ / GUCTL.REFCLKPER, >= 2.50a | GFLADJ, GUCTL | not written |
//! | ep0.c | | no revision checks |
//!
//! So 3.00a and 3.30a get the same register program (the host tests run
//! both); the revision only gates the GUSB3PIPECTL.SUSPHY write, for
//! cores at or before 1.94a.
//!
//! Cache maintenance goes through the platform (bt0 runs with the MMU and
//! caches off, where it is a barrier only). Every controller command and
//! active transfer has a bound; only the idle wait for the next host command
//! is host driven, like `ci-udc`.
#![no_std]

use core::ptr::{addr_of, addr_of_mut, read_volatile, write_volatile};

use regio::RegIo;
use usb_device::{decode, descriptor, Request, Setup, EP1};
pub use usb_device::{Descriptors, EP0_MAX};

/// Register offsets from the controller base and their fields.
pub mod regs {
    /// Global SoC bus configuration 0, TRM p.2190.
    pub const GSBUSCFG0: usize = 0xc100;
    /// Global core control, TRM pp.2193-2195; fields U-Boot core.h.
    pub const GCTL: usize = 0xc110;
    pub const GCTL_PRTCAPDIR_MASK: u32 = 3 << 12;
    pub const GCTL_PRTCAP_DEVICE: u32 = 2 << 12;
    pub const GCTL_CORESOFTRESET: u32 = 1 << 11;
    pub const GCTL_SCALEDOWN_MASK: u32 = 3 << 4;
    pub const GCTL_DISSCRAMBLE: u32 = 1 << 3;
    pub const GCTL_GBLHIBERNATIONEN: u32 = 1 << 1;
    /// Synopsys ID: 0x5533xxxx (DWC_usb3, xxxx the revision), TRM p.2200.
    pub const GSNPSID: usize = 0xc120;
    /// Hardware parameters 1 / 3, TRM pp.2204-2205.
    pub const GHWPARAMS1: usize = 0xc144;
    pub const GHWPARAMS3: usize = 0xc14c;
    /// USB2 PHY configuration (port 0), TRM p.2207; fields U-Boot core.h.
    pub const GUSB2PHYCFG: usize = 0xc200;
    pub const GUSB2PHYCFG_PHYSOFTRST: u32 = 1 << 31;
    pub const GUSB2PHYCFG_U2_FREECLK_EXISTS: u32 = 1 << 30;
    pub const GUSB2PHYCFG_USBTRDTIM_MASK: u32 = 0xf << 10;
    /// USBTrdTim for a 16-bit UTMI+ interface (U-Boot core.h
    /// USBTRDTIM_UTMI_16_BIT).
    pub const GUSB2PHYCFG_USBTRDTIM_16BIT: u32 = 5 << 10;
    pub const GUSB2PHYCFG_ENBLSLPM: u32 = 1 << 8;
    pub const GUSB2PHYCFG_SUSPHY: u32 = 1 << 6;
    /// PHYIf: 1 = 16-bit UTMI+ ("utmi_wide").
    pub const GUSB2PHYCFG_PHYIF_16BIT: u32 = 1 << 3;
    /// USB3 PIPE control (port 0), TRM p.2210; fields U-Boot core.h.
    pub const GUSB3PIPECTL: usize = 0xc2c0;
    pub const GUSB3PIPECTL_DEPOCHANGE: u32 = 1 << 18;
    pub const GUSB3PIPECTL_SUSPHY: u32 = 1 << 17;
    /// Event buffer 0, TRM pp.2214-2215.
    pub const GEVNTADRLO: usize = 0xc400;
    pub const GEVNTADRHI: usize = 0xc404;
    pub const GEVNTSIZ: usize = 0xc408;
    pub const GEVNTSIZ_INTMASK: u32 = 1 << 31;
    pub const GEVNTCOUNT: usize = 0xc40c;
    pub const GEVNTCOUNT_MASK: u32 = 0xfffc;
    /// Device configuration, TRM p.2219 (reset 0x00080000).
    pub const DCFG: usize = 0xc700;
    pub const DCFG_LPM_CAP: u32 = 1 << 22;
    pub const DCFG_DEVADDR_MASK: u32 = 0x7f << 3;
    pub const DCFG_SPEED_MASK: u32 = 7;
    pub const DCFG_HIGHSPEED: u32 = 0;
    pub const DCFG_FULLSPEED: u32 = 1;
    /// Device control, TRM pp.2219-2222 (reset 0x00F00000).
    pub const DCTL: usize = 0xc704;
    pub const DCTL_RUN_STOP: u32 = 1 << 31;
    pub const DCTL_CSFTRST: u32 = 1 << 30;
    pub const DCTL_HIRD_THRES_MASK: u32 = 0x1f << 24;
    pub const DCTL_KEEP_CONNECT: u32 = 1 << 19;
    pub const DCTL_ULSTCHNGREQ_MASK: u32 = 0xf << 5;
    pub const DCTL_TSTCTRL_MASK: u32 = 0xf << 1;
    /// Device event enable, TRM p.2223.
    pub const DEVTEN: usize = 0xc708;
    /// The events U-Boot gadget.c dwc3_gadget_enable_irq enables ("all but
    /// Start and End of Frame"): VndrDevTstRcved, EvntOverflow, CmdCmplt,
    /// ErrticErr, WkUpEvt, ULStCng, ConnectDone, USBRst, DisconnEvt.
    pub const DEVTEN_USED: u32 = (1 << 12)
        | (1 << 11)
        | (1 << 10)
        | (1 << 9)
        | (1 << 4)
        | (1 << 3)
        | (1 << 2)
        | (1 << 1)
        | 1;
    /// Device status, TRM pp.2224-2225 (reset 0x00520000).
    pub const DSTS: usize = 0xc70c;
    pub const DSTS_DEVCTRLHLT: u32 = 1 << 22;
    pub const DSTS_CONNECTSPD: u32 = 7;
    /// Physical endpoint enable bits, TRM p.2226.
    pub const DALEPENA: usize = 0xc720;
    /// Endpoint command parameters and command, n = physical endpoint
    /// (0 = ep0 OUT, 1 = ep0 IN, 2 = ep1 OUT, 3 = ep1 IN), TRM pp.2226-2228.
    pub const fn depcmdpar2(n: usize) -> usize {
        0xc800 + 0x10 * n
    }
    pub const fn depcmdpar1(n: usize) -> usize {
        0xc804 + 0x10 * n
    }
    pub const fn depcmdpar0(n: usize) -> usize {
        0xc808 + 0x10 * n
    }
    pub const fn depcmd(n: usize) -> usize {
        0xc80c + 0x10 * n
    }
    /// DEPnCMD fields, TRM p.2227: CommandParam [31:16] (XferRscIdx
    /// [22:16]), CmdStatus [15:12], HiPri/ForceRM [11], CmdAct [10],
    /// CmdIOC [8], command type [3:0] (U-Boot core.h).
    pub const DEPCMD_CMDACT: u32 = 1 << 10;
    pub const DEPCMD_CMDIOC: u32 = 1 << 8;
    pub const DEPCMD_FORCERM: u32 = 1 << 11;
    pub const fn depcmd_param(x: u32) -> u32 {
        x << 16
    }
    pub const DEPCMD_SETEPCONFIG: u32 = 1;
    pub const DEPCMD_SETTRANSFRESOURCE: u32 = 2;
    pub const DEPCMD_SETSTALL: u32 = 4;
    pub const DEPCMD_CLEARSTALL: u32 = 5;
    pub const DEPCMD_STARTTRANSFER: u32 = 6;
    pub const DEPCMD_ENDTRANSFER: u32 = 8;
    pub const DEPCMD_DEPSTARTCFG: u32 = 9;
    /// DEPCFG parameter 0 / 1 fields (U-Boot gadget.h).
    pub const fn depcfg_ep_type(t: u32) -> u32 {
        t << 1
    }
    pub const fn depcfg_max_packet(n: u32) -> u32 {
        n << 3
    }
    pub const fn depcfg_fifo_number(n: u32) -> u32 {
        n << 17
    }
    /// Config action "modify" (Linux gadget.h DWC3_DEPCFG_ACTION_MODIFY;
    /// U-Boot sets the same bit as DWC3_DEPCFG_IGN_SEQ_NUM on ConnectDone).
    pub const DEPCFG_ACTION_MODIFY: u32 = 2 << 30;
    pub const DEPCFG_XFER_COMPLETE_EN: u32 = 1 << 8;
    pub const DEPCFG_XFER_IN_PROGRESS_EN: u32 = 1 << 9;
    pub const DEPCFG_XFER_NOT_READY_EN: u32 = 1 << 10;
    pub const fn depcfg_ep_number(n: u32) -> u32 {
        n << 25
    }
    pub const EP_TYPE_CONTROL: u32 = 0;
    pub const EP_TYPE_BULK: u32 = 2;
}

/// TRB fields (U-Boot core.h "struct dwc3_trb").
pub mod trb {
    pub const SIZE_MASK: u32 = 0x00ff_ffff;
    pub const fn status(size: u32) -> u32 {
        size >> 28
    }
    pub const STS_SETUP_PENDING: u32 = 2;
    pub const HWO: u32 = 1 << 0;
    pub const LST: u32 = 1 << 1;
    pub const ISP_IMI: u32 = 1 << 10;
    pub const IOC: u32 = 1 << 11;
    pub const NORMAL: u32 = 1 << 4;
    pub const CONTROL_SETUP: u32 = 2 << 4;
    pub const CONTROL_STATUS2: u32 = 3 << 4;
    pub const CONTROL_STATUS3: u32 = 4 << 4;
    pub const CONTROL_DATA: u32 = 5 << 4;
}

/// Event fields (U-Boot core.h dwc3_event_depevt / dwc3_event_devt).
pub mod event {
    /// Endpoint event: bit 0 = 0, [5:1] physical endpoint, [9:6] type,
    /// [15:12] status. Device event: bit 0 = 1, [7:1] = 0, [11:8] type.
    pub const XFERCOMPLETE: u32 = 1;
    pub const XFERINPROGRESS: u32 = 2;
    pub const XFERNOTREADY: u32 = 3;
    pub const EPCMDCMPLT: u32 = 7;
    /// XferComplete status bits.
    pub const STATUS_BUSERR: u32 = 1 << 0;
    pub const STATUS_SHORT: u32 = 1 << 1;
    /// XferNotReady status on ep0.
    pub const STATUS_CONTROL_DATA: u32 = 1;
    pub const STATUS_CONTROL_STATUS: u32 = 2;
    /// Device event types.
    pub const DEV_DISCONNECT: u32 = 0;
    pub const DEV_RESET: u32 = 1;
    pub const DEV_CONNECT_DONE: u32 = 2;

    pub const fn ep(phys: u32, kind: u32, status: u32) -> u32 {
        (phys << 1) | (kind << 6) | (status << 12)
    }
    pub const fn dev(kind: u32) -> u32 {
        1 | (kind << 8)
    }
}

use regs::*;

/// GSNPSID, the core and its revision (Linux core.h DWC3_GSNPS_ID,
/// DWC3_REVISION_*): `0x5533` ("U3") + the DWC_usb3 revision as BCD-like
/// digits and a letter (0x300a = 3.00a); `0x3331` / `0x3332` are DWC_usb31
/// / DWC_usb32, whose revision lives in another register (Linux
/// DWC3_VER_NUMBER) and which Linux treats as newer than every DWC_usb3
/// revision in its `DWC3_VER_IS_PRIOR(DWC3, ...)` checks.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Revision(pub u32);

impl Revision {
    pub const DWC3_194A: u32 = 0x5533_194a;
    pub const DWC3_250A: u32 = 0x5533_250a;
    pub const DWC3_300A: u32 = 0x5533_300a;
    pub const DWC3_310A: u32 = 0x5533_310a;
    pub const DWC3_330A: u32 = 0x5533_330a;

    /// A DWC_usb3, DWC_usb31 or DWC_usb32 core (U-Boot core.c
    /// dwc3_core_is_valid, Linux core.c dwc3_core_is_valid).
    pub fn valid(self) -> bool {
        matches!(self.0 >> 16, 0x5533 | 0x3331 | 0x3332)
    }
    pub fn is_dwc3(self) -> bool {
        self.0 >> 16 == 0x5533
    }
    /// Linux `DWC3_VER_IS_PRIOR(DWC3, v)`: a DWC_usb3 core older than `v`.
    pub fn dwc3_prior(self, v: u32) -> bool {
        self.is_dwc3() && self.0 < v
    }
    /// Linux `DWC3_VER_IS_WITHIN(DWC3, ANY, v)`: a DWC_usb3 core at or
    /// before `v`.
    pub fn dwc3_at_most(self, v: u32) -> bool {
        self.is_dwc3() && self.0 <= v
    }
}

impl core::fmt::Display for Revision {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let r = self.0 & 0xffff;
        match self.0 >> 16 {
            0x5533 => {
                let d = |s: u32| char::from_digit((r >> s) & 0xf, 16).unwrap_or('?');
                write!(f, "DWC_usb3 {}.{}{}{}", d(12), d(8), d(4), d(0))
            }
            0x3331 => write!(f, "DWC_usb31"),
            0x3332 => write!(f, "DWC_usb32"),
            _ => write!(f, "unknown core"),
        }
    }
}

/// Bytes of the event buffer (a multiple of 4; U-Boot uses 256, Linux 4096).
pub const EVENT_BYTES: usize = 1024;
/// Bytes one bulk TRB moves at most: a multiple of every max packet size,
/// below the 16 MiB - 1 limit of the TRB size field.
pub const XFER_MAX: usize = 0x80_0000;
/// The bulk receive granule: OUT TRBs must be a multiple of the max packet
/// size (512 at High-Speed, 64 at Full-Speed).
pub const BULK_GRANULE: usize = 512;

const EP0_OUT: usize = 0;
const EP0_IN: usize = 1;
const EP1_OUT: usize = 2;
const EP1_IN: usize = 3;

/// The fastest speed the device offers.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Speed {
    High,
    Full,
}

/// Bounds (microseconds for register polls, counts for transfers) and the
/// controller's base address.
#[derive(Clone, Copy, Debug)]
pub struct Config {
    pub base: usize,
    pub speed: Speed,
    /// DCTL.CSFTRST to clear (U-Boot: 5000 x 1 us).
    pub reset_timeout_us: u64,
    /// DSTS.DevCtrlHlt to follow Run/Stop (U-Boot: 2000 x 2 ms).
    pub halt_timeout_us: u64,
    /// DEPnCMD.CmdAct to clear (U-Boot: 500 x 1 us).
    pub cmd_timeout_us: u64,
    /// Settle time after the PHY interface writes and the second soft
    /// reset (U-Boot core.c: mdelay(100) each).
    pub settle_us: u32,
    /// Event-buffer polls an active (device-started) bulk transfer may take.
    pub transfer_polls: usize,
}

impl Config {
    /// The U-Boot bounds, High-Speed, at controller `base`.
    pub const fn new(base: usize) -> Self {
        Self {
            base,
            speed: Speed::High,
            reset_timeout_us: 50_000,
            halt_timeout_us: 500_000,
            cmd_timeout_us: 1_000,
            settle_us: 100_000,
            transfer_polls: 50_000_000,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    /// The host reset the bus or disconnected; the caller should start over.
    Reset,
    /// A bulk transfer ended in error or short (event status / bytes).
    Transfer(u32),
    /// An active transfer ran out of its budget.
    Timeout,
    /// GSNPSID does not read as a DWC_usb3/31/32 core.
    NotDwc3(u32),
    PhyTimeout,
    /// DCTL.CSFTRST did not clear.
    ResetTimeout,
    /// DSTS.DevCtrlHlt did not follow Run/Stop.
    HaltTimeout,
    /// An endpoint command stayed active (the DEPnCMD value).
    CommandTimeout(u32),
    /// An endpoint command completed with a non-zero CmdStatus.
    Command(u32),
}

/// The SoC did not bring the PHY up during [`Dwc3Platform::bring_up`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PhyTimeout;

/// The SoC's side: GRF/PHY/clock set-up and cache maintenance.
pub trait Dwc3Platform {
    /// Make the controller and its USB 2.0 PHY usable (clocks, PHY power,
    /// USB3 port routing, VBUS indication) before the core soft reset.
    fn bring_up<IO: RegIo>(&mut self, io: &mut IO) -> Result<(), PhyTimeout>;
    /// Clean `len` bytes at `addr` to memory before the controller reads
    /// them (and order the writes before the doorbell that follows).
    fn flush_dcache(&mut self, addr: usize, len: usize);
    /// Discard cached `len` bytes at `addr` before reading what the
    /// controller wrote.
    fn invalidate_dcache(&mut self, addr: usize, len: usize);
}

/// A transfer request block (16 bytes, 16-byte aligned).
#[repr(C, align(16))]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Trb {
    pub bpl: u32,
    pub bph: u32,
    pub size: u32,
    pub ctrl: u32,
}

/// Everything the controller reads or writes besides bulk data. Place it
/// in memory the controller's AXI master reaches (a `static` in DRAM).
#[repr(C, align(4096))]
pub struct Dma {
    events: [u32; EVENT_BYTES / 4],
    /// One TRB per physical endpoint.
    trb: [Trb; 4],
    setup: [u8; 8],
    ep0_buf: [u8; 64],
    small: [u8; BULK_GRANULE],
}

impl Dma {
    pub const fn new() -> Self {
        Self {
            events: [0; EVENT_BYTES / 4],
            trb: [Trb {
                bpl: 0,
                bph: 0,
                size: 0,
                ctrl: 0,
            }; 4],
            setup: [0; 8],
            ep0_buf: [0; 64],
            small: [0; BULK_GRANULE],
        }
    }
}

impl Default for Dma {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Ep0 {
    /// A SETUP TRB is queued on ep0 OUT.
    Setup,
    /// An IN data stage is queued on ep0 IN.
    DataIn,
    /// Waiting for XferNotReady(Status) before queueing the status stage.
    WaitStatus { three_stage: bool },
    /// The status stage is queued.
    Status,
}

#[derive(Clone, Copy, Debug, Default)]
struct Ep {
    /// Transfer resource index of the active transfer (0: none).
    resource: u32,
    busy: bool,
    /// XferComplete seen: (event status, bytes left in the TRB).
    done: Option<(u32, u32)>,
}

fn lo(a: usize) -> u32 {
    a as u32
}
fn hi(a: usize) -> u32 {
    ((a as u64) >> 32) as u32
}

pub struct Udc<'a, IO: RegIo, P: Dwc3Platform> {
    io: IO,
    plat: P,
    cfg: Config,
    // The controller owns parts of this area while transfers run, so it is
    // reached through a pointer.
    dma: *mut Dma,
    desc: Descriptors<'a>,
    /// Next event to read, in bytes.
    lpos: usize,
    /// DSTS.ConnectSpd of the current connection.
    speed: u32,
    ep0: Ep0,
    ep: [Ep; 4],
    ep1_enabled: bool,
    configured: bool,
    /// A bus reset or disconnect was seen since the last poll returned.
    bus_reset: bool,
    fault: Option<Error>,
    revision: Revision,
}

impl<'a, IO: RegIo, P: Dwc3Platform> Udc<'a, IO, P> {
    fn r(&mut self, o: usize) -> u32 {
        self.io.read32(self.cfg.base + o)
    }
    fn w(&mut self, o: usize, v: u32) {
        self.io.write32(self.cfg.base + o, v)
    }
    /// DCTL read-modify-write that never requests a link state change
    /// (U-Boot gadget.h dwc3_gadget_dctl_write_safe).
    fn dctl_write_safe(&mut self, v: u32) {
        self.w(DCTL, v & !DCTL_ULSTCHNGREQ_MASK)
    }
    fn wait(
        &mut self,
        what: &'static str,
        timeout_us: u64,
        mut cond: impl FnMut(&mut IO, usize) -> bool,
    ) -> Result<(), regio::Timeout> {
        let base = self.cfg.base;
        regio::wait(&mut self.io, what, timeout_us, |io| cond(io, base))
    }

    /// Bring the controller up as a new device and connect (dropping any
    /// earlier session, e.g. the BootROM's MaskROM download device).
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
            lpos: 0,
            speed: 0,
            ep0: Ep0::Setup,
            ep: [Ep::default(); 4],
            ep1_enabled: false,
            configured: false,
            bus_reset: false,
            fault: None,
            revision: Revision(0),
        };
        if u.plat.bring_up(&mut u.io).is_err() {
            return Err(Error::PhyTimeout);
        }
        // "U3" followed by the revision (U-Boot core.c dwc3_core_is_valid).
        let id = u.r(GSNPSID);
        let rev = Revision(id);
        if !rev.valid() {
            return Err(Error::NotDwc3(id));
        }
        u.revision = rev;
        u.io.log(format_args!("[usb] dwc3 GSNPSID {id:#010x} ({rev})\n"));
        // Stop whatever session the BootROM left running: Run/Stop off,
        // acknowledging its events until the core halts (TRM p.2224:
        // DevCtrlHlt is not set while GEVNTCOUNT is non-zero).
        u.halt()?;
        // Device soft reset (U-Boot core.c dwc3_core_init: DCTL = CSFTRST).
        u.w(DCTL, DCTL_CSFTRST);
        u.wait_soft_reset()?;
        u.phy_setup();
        // dwc3_core_soft_reset: CSFTRST again, Run/Stop clear, then 100 ms.
        let v = u.r(DCTL);
        u.dctl_write_safe((v | DCTL_CSFTRST) & !DCTL_RUN_STOP);
        u.wait_soft_reset()?;
        u.io.udelay(u.cfg.settle_us);
        // GCTL: no scale-down, scrambling on, no hibernation (device mode
        // has none), port capability "device" (U-Boot core.c
        // dwc3_core_init + dwc3_set_mode; TRM pp.2193-2195).
        let v = u.r(GCTL);
        u.w(
            GCTL,
            (v & !(GCTL_SCALEDOWN_MASK
                | GCTL_DISSCRAMBLE
                | GCTL_GBLHIBERNATIONEN
                | GCTL_CORESOFTRESET
                | GCTL_PRTCAPDIR_MASK))
                | GCTL_PRTCAP_DEVICE,
        );
        // Event buffer 0 (U-Boot core.c dwc3_event_buffers_setup).
        // SAFETY: field address of the area handed to init (no access).
        let ev = unsafe { addr_of!((*u.dma).events) } as usize;
        u.w(GEVNTADRLO, lo(ev));
        u.w(GEVNTADRHI, hi(ev));
        u.w(GEVNTSIZ, EVENT_BYTES as u32);
        u.w(GEVNTCOUNT, 0);
        // DCFG: speed, address 0 (DCFG survives the soft reset, TRM
        // p.2194, and the BootROM's address is still in it), no LPM.
        let speed = match u.cfg.speed {
            Speed::High => DCFG_HIGHSPEED,
            Speed::Full => DCFG_FULLSPEED,
        };
        let v = u.r(DCFG);
        u.w(
            DCFG,
            (v & !(DCFG_SPEED_MASK | DCFG_DEVADDR_MASK | DCFG_LPM_CAP)) | speed,
        );
        // ep0 (U-Boot gadget.c dwc3_gadget_start: start config 0, then both
        // directions with a 512-byte max packet until ConnectDone).
        u.ep_cmd(EP0_OUT, DEPCMD_DEPSTARTCFG | depcmd_param(0), [0; 3])?;
        for phys in [EP0_OUT, EP0_IN] {
            u.ep_config(phys, EP_TYPE_CONTROL, 512, 0)?;
            u.ep_cmd(phys, DEPCMD_SETTRANSFRESOURCE, [1, 0, 0])?;
            let v = u.r(DALEPENA);
            u.w(DALEPENA, v | 1 << phys);
        }
        u.ep0_setup_start()?;
        u.w(DEVTEN, DEVTEN_USED);
        // Connect (U-Boot gadget.c dwc3_gadget_run_stop).
        let v = u.r(DCTL);
        u.dctl_write_safe((v & !DCTL_KEEP_CONNECT) | DCTL_RUN_STOP);
        u.wait("dwc3 run", u.cfg.halt_timeout_us, |io, b| {
            io.read32(b + DSTS) & DSTS_DEVCTRLHLT == 0
        })
        .map_err(|_| Error::HaltTimeout)?;
        u.io.log(format_args!(
            "[usb] device controller running, waiting for the host\n"
        ));
        Ok(u)
    }

    fn wait_soft_reset(&mut self) -> Result<(), Error> {
        self.wait("dwc3 soft reset", self.cfg.reset_timeout_us, |io, b| {
            io.read32(b + DCTL) & DCTL_CSFTRST == 0
        })
        .map_err(|_| Error::ResetTimeout)
    }

    /// PHY interface (U-Boot core.c dwc3_phy_setup + dwc3_hsphy_mode_setup
    /// with the rk3588-base.dtsi quirks; TRM pp.2207, 2210).
    fn phy_setup(&mut self) {
        // USB3 PHY: suspend it (only after 1.94a: U-Boot core.c
        // dwc3_phy_setup "if (dwc->revision > DWC3_REVISION_194A)", Linux
        // core.c !DWC3_VER_IS_WITHIN(DWC3, ANY, 194A)), no P0->P1/P2/P3
        // delay on PHY power change (dis-del-phy-power-chg). The SS port
        // itself is disabled by the platform (USB_GRF).
        let v = self.r(GUSB3PIPECTL);
        let susphy = if self.revision.dwc3_at_most(Revision::DWC3_194A) {
            0
        } else {
            GUSB3PIPECTL_SUSPHY
        };
        self.w(GUSB3PIPECTL, (v | susphy) & !GUSB3PIPECTL_DEPOCHANGE);
        // 16-bit UTMI+ ("utmi_wide").
        let v = self.r(GUSB2PHYCFG);
        self.w(
            GUSB2PHYCFG,
            (v & !(GUSB2PHYCFG_PHYIF_16BIT | GUSB2PHYCFG_USBTRDTIM_MASK))
                | GUSB2PHYCFG_PHYIF_16BIT
                | GUSB2PHYCFG_USBTRDTIM_16BIT,
        );
        self.io.udelay(self.cfg.settle_us);
        // dis_enblslpm, dis-u2-freeclk-exists; SUSPHY kept 0 (see above).
        let v = self.r(GUSB2PHYCFG);
        self.w(
            GUSB2PHYCFG,
            v & !(GUSB2PHYCFG_SUSPHY | GUSB2PHYCFG_ENBLSLPM | GUSB2PHYCFG_U2_FREECLK_EXISTS),
        );
        self.io.udelay(self.cfg.settle_us);
    }

    /// Run/Stop off and wait for the halt, acknowledging events meanwhile.
    fn halt(&mut self) -> Result<(), Error> {
        let v = self.r(DCTL);
        if v & DCTL_RUN_STOP != 0 {
            self.dctl_write_safe(v & !DCTL_RUN_STOP);
        }
        self.wait("dwc3 halt", self.cfg.halt_timeout_us, |io, b| {
            let n = io.read32(b + GEVNTCOUNT) & GEVNTCOUNT_MASK;
            if n != 0 {
                io.write32(b + GEVNTCOUNT, n);
            }
            io.read32(b + DSTS) & DSTS_DEVCTRLHLT != 0
        })
        .map_err(|_| Error::HaltTimeout)
    }

    /// Issue an endpoint command and wait for it (U-Boot gadget.c
    /// dwc3_send_gadget_ep_cmd; parameter order PAR2, PAR1, PAR0).
    fn ep_cmd(&mut self, phys: usize, cmd: u32, par: [u32; 3]) -> Result<u32, Error> {
        self.w(depcmdpar2(phys), par[2]);
        self.w(depcmdpar1(phys), par[1]);
        self.w(depcmdpar0(phys), par[0]);
        self.w(depcmd(phys), cmd | DEPCMD_CMDACT);
        let a = depcmd(phys);
        self.wait("dwc3 endpoint command", self.cfg.cmd_timeout_us, |io, b| {
            io.read32(b + a) & DEPCMD_CMDACT == 0
        })
        .map_err(|_| Error::CommandTimeout(cmd))?;
        let v = self.r(a);
        if (v >> 12) & 0xf != 0 {
            return Err(Error::Command(v));
        }
        Ok(v)
    }

    /// DEPCFG (U-Boot gadget.c dwc3_gadget_set_ep_config): type, max packet,
    /// TX FIFO = USB endpoint number for IN, XferComplete/XferNotReady
    /// (+ XferInProgress for non-control) events, physical endpoint number.
    fn ep_config(&mut self, phys: usize, ty: u32, mps: u32, action: u32) -> Result<(), Error> {
        let mut p0 = depcfg_ep_type(ty) | depcfg_max_packet(mps) | action;
        if phys & 1 == 1 {
            p0 |= depcfg_fifo_number(phys as u32 >> 1);
        }
        let mut p1 =
            DEPCFG_XFER_COMPLETE_EN | DEPCFG_XFER_NOT_READY_EN | depcfg_ep_number(phys as u32);
        if ty != EP_TYPE_CONTROL {
            p1 |= DEPCFG_XFER_IN_PROGRESS_EN;
        }
        self.ep_cmd(phys, DEPCMD_SETEPCONFIG, [p0, p1, 0])
            .map(|_| ())
    }

    /// Queue one TRB on `phys` and start the transfer (U-Boot ep0.c
    /// dwc3_ep0_start_trans; the same single-TRB form for bulk).
    fn start_trans(&mut self, phys: usize, buf: usize, len: usize, ctl: u32) -> Result<(), Error> {
        // SAFETY: `dma` comes from a `&'static mut Dma` handed to `init`;
        // the controller does not own this TRB (no transfer active on it).
        let t = unsafe { addr_of_mut!((*self.dma).trb[phys]) };
        unsafe {
            write_volatile(addr_of_mut!((*t).bpl), lo(buf));
            write_volatile(addr_of_mut!((*t).bph), hi(buf));
            write_volatile(addr_of_mut!((*t).size), len as u32 & trb::SIZE_MASK);
            write_volatile(
                addr_of_mut!((*t).ctrl),
                ctl | trb::HWO | trb::ISP_IMI | trb::IOC | trb::LST,
            );
        }
        if phys & 1 == 1 && len != 0 {
            self.plat.flush_dcache(buf, len);
        }
        self.plat
            .flush_dcache(t as usize, core::mem::size_of::<Trb>());
        let a = t as usize;
        let v = self.ep_cmd(phys, DEPCMD_STARTTRANSFER, [hi(a), lo(a), 0])?;
        self.ep[phys] = Ep {
            resource: (v >> 16) & 0x7f,
            busy: true,
            done: None,
        };
        Ok(())
    }

    /// End the active transfer on `phys` (U-Boot gadget.c
    /// dwc3_stop_active_transfer, CmdIOC + 100 us; ForceRM for bulk).
    fn end_transfer(&mut self, phys: usize) -> Result<(), Error> {
        let res = self.ep[phys].resource;
        self.ep[phys] = Ep::default();
        if res == 0 {
            return Ok(());
        }
        let force = if phys >= EP1_OUT { DEPCMD_FORCERM } else { 0 };
        self.ep_cmd(
            phys,
            DEPCMD_ENDTRANSFER | force | DEPCMD_CMDIOC | depcmd_param(res),
            [0; 3],
        )?;
        self.io.udelay(100);
        Ok(())
    }

    fn trb_size(&mut self, phys: usize) -> u32 {
        let t = unsafe { addr_of!((*self.dma).trb[phys]) };
        self.plat
            .invalidate_dcache(t as usize, core::mem::size_of::<Trb>());
        // SAFETY: the transfer on this TRB completed.
        unsafe { read_volatile(addr_of!((*t).size)) }
    }

    fn ep0_setup_start(&mut self) -> Result<(), Error> {
        let a = unsafe { addr_of!((*self.dma).setup) } as usize;
        self.ep0 = Ep0::Setup;
        self.start_trans(EP0_OUT, a, 8, trb::CONTROL_SETUP)
    }

    /// Stall ep0 and wait for the next SETUP (U-Boot ep0.c
    /// dwc3_ep0_stall_and_restart: SetStall on physical ep0).
    fn ep0_stall(&mut self) -> Result<(), Error> {
        self.ep_cmd(EP0_OUT, DEPCMD_SETSTALL, [0; 3])?;
        self.ep[EP0_OUT] = Ep::default();
        self.ep[EP0_IN] = Ep::default();
        self.ep0_setup_start()
    }

    fn ep0_in(&mut self, data: &[u8], w_length: usize) -> Result<(), Error> {
        let n = data.len().min(w_length).min(64);
        if n == 0 {
            // No data stage: the host asked for zero bytes.
            self.ep0 = Ep0::WaitStatus { three_stage: false };
            return Ok(());
        }
        // Replies are under 64 bytes (usb-device: at most 62), so the data
        // stage always ends with a short packet or at wLength: no ZLP.
        let b = unsafe { addr_of_mut!((*self.dma).ep0_buf) };
        for (i, v) in data[..n].iter().enumerate() {
            unsafe { write_volatile((b as *mut u8).add(i), *v) };
        }
        self.ep0 = Ep0::DataIn;
        self.start_trans(EP0_IN, b as usize, n, trb::CONTROL_DATA)
    }

    fn bulk_mps(&self) -> u16 {
        // DSTS.ConnectSpd: 0 High-Speed, 1/3 Full-Speed (U-Boot core.h).
        if self.speed == 0 {
            512
        } else {
            64
        }
    }

    /// The SETUP packet arrived (ep0 XferComplete in the SETUP phase).
    fn setup(&mut self) -> Result<(), Error> {
        let p = unsafe { addr_of!((*self.dma).setup) };
        self.plat.invalidate_dcache(p as usize, 8);
        let raw = unsafe { read_volatile(p) };
        let s = Setup::parse(raw);
        let length = s.length as usize;
        let no_data = Ep0::WaitStatus { three_stage: false };
        match decode(&s) {
            Request::GetDescriptor { value } => {
                let mut buf = [0u8; 64];
                let mps = self.bulk_mps();
                match descriptor(&self.desc, value, mps, &mut buf) {
                    Some(n) => self.ep0_in(&buf[..n], length)?,
                    None => self.ep0_stall()?,
                }
            }
            Request::SetAddress(a) if a <= 127 && !self.configured => {
                // U-Boot ep0.c dwc3_ep0_set_address: DCFG.DevAddr at once.
                let v = self.r(DCFG);
                self.w(DCFG, (v & !DCFG_DEVADDR_MASK) | (a as u32) << 3);
                self.ep0 = no_data;
            }
            Request::SetConfiguration(v) if v <= 1 => {
                if v == 1 {
                    self.enable_ep1()?;
                    self.configured = true;
                    self.io.log(format_args!("[usb] configured\n"));
                } else {
                    self.disable_ep1()?;
                    self.configured = false;
                }
                self.ep0 = no_data;
            }
            Request::GetConfiguration => self.ep0_in(&[self.configured as u8], length)?,
            Request::GetStatusDevice => self.ep0_in(&[1, 0], length)?, // self powered
            Request::GetStatusOther => self.ep0_in(&[0, 0], length)?,
            Request::ClearEndpointHalt { index } => {
                if index & 0xf == EP1 as u16 && self.ep1_enabled {
                    let phys = if index & 0x80 != 0 { EP1_IN } else { EP1_OUT };
                    self.ep_cmd(phys, DEPCMD_CLEARSTALL, [0; 3])?;
                }
                self.ep0 = no_data;
            }
            Request::SetInterface => self.ep0 = no_data,
            _ => {
                self.io.log(format_args!(
                    "[usb] unsupported request {:02x} {:02x}\n",
                    s.request_type, s.request
                ));
                self.ep0_stall()?;
            }
        }
        Ok(())
    }

    /// SET_CONFIGURATION(1): start config with XferRscIdx 2, then each bulk
    /// endpoint (U-Boot gadget.c dwc3_gadget_start_config +
    /// __dwc3_gadget_ep_enable).
    fn enable_ep1(&mut self) -> Result<(), Error> {
        if self.ep1_enabled {
            return Ok(());
        }
        self.ep_cmd(EP0_OUT, DEPCMD_DEPSTARTCFG | depcmd_param(2), [0; 3])?;
        let mps = self.bulk_mps() as u32;
        for phys in [EP1_OUT, EP1_IN] {
            self.ep_config(phys, EP_TYPE_BULK, mps, 0)?;
            self.ep_cmd(phys, DEPCMD_SETTRANSFRESOURCE, [1, 0, 0])?;
            let v = self.r(DALEPENA);
            self.w(DALEPENA, v | 1 << phys);
        }
        self.ep1_enabled = true;
        Ok(())
    }

    fn disable_ep1(&mut self) -> Result<(), Error> {
        self.end_transfer(EP1_OUT)?;
        self.end_transfer(EP1_IN)?;
        if self.ep1_enabled {
            let v = self.r(DALEPENA);
            self.w(DALEPENA, v & !(1 << EP1_OUT | 1 << EP1_IN));
            self.ep1_enabled = false;
        }
        Ok(())
    }

    fn ep0_event(&mut self, phys: usize, kind: u32, status: u32) -> Result<(), Error> {
        match kind {
            event::XFERCOMPLETE => {
                self.ep[phys].busy = false;
                self.ep[phys].resource = 0;
                match self.ep0 {
                    Ep0::Setup if phys == EP0_OUT => self.setup(),
                    Ep0::DataIn if phys == EP0_IN => {
                        let _ = trb::status(self.trb_size(EP0_IN));
                        self.ep0 = Ep0::WaitStatus { three_stage: true };
                        Ok(())
                    }
                    Ep0::Status => self.ep0_setup_start(),
                    _ => Ok(()),
                }
            }
            event::XFERNOTREADY => match (status, self.ep0) {
                (event::STATUS_CONTROL_STATUS, Ep0::WaitStatus { three_stage }) => {
                    // The status stage goes on the endpoint that asked
                    // (U-Boot ep0.c dwc3_ep0_xfernotready).
                    let a = unsafe { addr_of!((*self.dma).setup) } as usize;
                    let ty = if three_stage {
                        trb::CONTROL_STATUS3
                    } else {
                        trb::CONTROL_STATUS2
                    };
                    self.ep0 = Ep0::Status;
                    self.start_trans(phys, a, 0, ty)
                }
                (event::STATUS_CONTROL_DATA, Ep0::DataIn) if phys != EP0_IN => {
                    // Data phase in the wrong direction: end ours, stall.
                    self.end_transfer(EP0_IN)?;
                    self.ep0_stall()
                }
                _ => Ok(()),
            },
            _ => Ok(()),
        }
    }

    fn event(&mut self, ev: u32) -> Result<(), Error> {
        if ev & 1 == 0 {
            let phys = ((ev >> 1) & 0x1f) as usize;
            let kind = (ev >> 6) & 0xf;
            let status = (ev >> 12) & 0xf;
            match phys {
                EP0_OUT | EP0_IN => return self.ep0_event(phys, kind, status),
                EP1_OUT | EP1_IN if kind == event::XFERCOMPLETE => {
                    let left = self.trb_size(phys) & trb::SIZE_MASK;
                    self.ep[phys] = Ep {
                        resource: 0,
                        busy: false,
                        done: Some((status, left)),
                    };
                }
                _ => {}
            }
            return Ok(());
        }
        if (ev >> 1) & 0x7f != 0 {
            return Ok(()); // not a device event
        }
        match (ev >> 8) & 0xf {
            event::DEV_DISCONNECT => {
                self.configured = false;
                self.disable_ep1()?;
                self.bus_reset = true;
            }
            event::DEV_RESET => self.usb_reset()?,
            event::DEV_CONNECT_DONE => self.connect_done()?,
            _ => {}
        }
        Ok(())
    }

    /// U-Boot gadget.c dwc3_gadget_reset_interrupt, plus ep0 back to the
    /// SETUP phase (Linux ep0.c dwc3_ep0_reset_state).
    fn usb_reset(&mut self) -> Result<(), Error> {
        let v = self.r(DCTL);
        self.dctl_write_safe(v & !DCTL_TSTCTRL_MASK);
        self.configured = false;
        self.disable_ep1()?;
        let v = self.r(DCFG);
        self.w(DCFG, v & !DCFG_DEVADDR_MASK);
        if self.ep0 != Ep0::Setup {
            self.end_transfer(EP0_IN)?;
            self.end_transfer(EP0_OUT)?;
            self.ep0_stall()?;
        }
        self.bus_reset = true;
        Ok(())
    }

    /// U-Boot gadget.c dwc3_gadget_conndone_interrupt (without LPM).
    fn connect_done(&mut self) -> Result<(), Error> {
        self.speed = self.r(DSTS) & DSTS_CONNECTSPD;
        let mps0 = match self.speed {
            4 => 512,
            2 => 8,
            _ => EP0_MAX as u32,
        };
        let v = self.r(DCTL);
        self.dctl_write_safe(v & !DCTL_HIRD_THRES_MASK);
        self.ep_config(EP0_OUT, EP_TYPE_CONTROL, mps0, DEPCFG_ACTION_MODIFY)?;
        self.ep_config(EP0_IN, EP_TYPE_CONTROL, mps0, DEPCFG_ACTION_MODIFY)?;
        let name = match self.speed {
            0 => "high",
            4 => "super",
            2 => "low",
            _ => "full",
        };
        self.io.log(format_args!("[usb] {name} speed\n"));
        Ok(())
    }

    fn check(&self) -> Result<(), Error> {
        self.fault.map_or(Ok(()), Err)
    }

    /// A transport fault is terminal for this session: the controller is
    /// stopped and keeps no transfer.
    fn remember<T>(&mut self, result: Result<T, Error>) -> Result<T, Error> {
        if let Err(e) = result {
            if !matches!(e, Error::Reset) {
                self.fault = Some(e);
                let v = self.r(DCTL);
                self.dctl_write_safe(v & !DCTL_RUN_STOP);
            }
        }
        result
    }

    /// Handle pending events. `Err(Reset)` when the host reset the bus or
    /// disconnected since the last call.
    pub fn poll(&mut self) -> Result<(), Error> {
        self.check()?;
        let r = self.poll_inner();
        self.remember(r)
    }

    fn poll_inner(&mut self) -> Result<(), Error> {
        let n = (self.r(GEVNTCOUNT) & GEVNTCOUNT_MASK) as usize;
        let mut done = 0;
        while done < n {
            let p = unsafe { addr_of!((*self.dma).events) };
            self.plat.invalidate_dcache(p as usize, EVENT_BYTES);
            let ev = unsafe { read_volatile((p as *const u32).add(self.lpos / 4)) };
            self.lpos = (self.lpos + 4) % EVENT_BYTES;
            self.event(ev)?;
            self.w(GEVNTCOUNT, 4);
            done += 4;
        }
        if core::mem::take(&mut self.bus_reset) {
            Err(Error::Reset)
        } else {
            Ok(())
        }
    }

    pub fn configured(&self) -> bool {
        self.configured
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

    /// One bulk transfer on EP1 (one TRB). Returns the bytes moved (an OUT
    /// transfer ends early on a short packet).
    fn bulk(
        &mut self,
        phys: usize,
        buf: usize,
        len: usize,
        active: bool,
        expired: &mut dyn FnMut() -> bool,
    ) -> Result<usize, Error> {
        if !self.ep1_enabled {
            return Err(Error::Reset);
        }
        self.start_trans(phys, buf, len, trb::NORMAL)?;
        let mut polls = 0;
        loop {
            if let Err(e) = self.poll_inner() {
                // A reset already ended the transfer; anything else ends it.
                self.end_transfer(phys)?;
                return Err(e);
            }
            if let Some((status, left)) = self.ep[phys].done.take() {
                if status & event::STATUS_BUSERR != 0 {
                    return Err(Error::Transfer(status));
                }
                return Ok(len - (left as usize).min(len));
            }
            if active {
                polls += 1;
                if polls == self.cfg.transfer_polls {
                    self.end_transfer(phys)?;
                    return Err(Error::Timeout);
                }
            }
            if expired() {
                self.end_transfer(phys)?;
                return Err(Error::Timeout);
            }
        }
    }

    /// Send `data` (at most 512 bytes) on EP1 IN.
    pub fn send(&mut self, data: &[u8]) -> Result<(), Error> {
        self.check()?;
        let p = unsafe { addr_of_mut!((*self.dma).small) } as *mut u8;
        let n = data.len().min(BULK_GRANULE);
        for (i, v) in data[..n].iter().enumerate() {
            unsafe { write_volatile(p.add(i), *v) };
        }
        let r = self.bulk(EP1_IN, p as usize, n, true, &mut || false);
        let r = match r {
            Ok(m) if m != n => Err(Error::Transfer(m as u32)),
            r => r.map(|_| ()),
        };
        self.remember(r)
    }

    /// Receive one command (a single transfer of at most 512 bytes) on
    /// EP1 OUT into `out`; returns its length. Waits as long as the host
    /// takes (the host drives the session).
    pub fn recv(&mut self, out: &mut [u8]) -> Result<usize, Error> {
        self.recv_until(out, || false)
    }

    /// [`recv`](Self::recv) that gives up with `Err(Timeout)` (terminal,
    /// the controller stops) when `expired()` returns true.
    pub fn recv_until(
        &mut self,
        out: &mut [u8],
        mut expired: impl FnMut() -> bool,
    ) -> Result<usize, Error> {
        self.check()?;
        let p = unsafe { addr_of_mut!((*self.dma).small) } as *mut u8;
        let r = self.bulk(EP1_OUT, p as usize, BULK_GRANULE, false, &mut expired);
        let n = self.remember(r)?;
        self.plat.invalidate_dcache(p as usize, BULK_GRANULE);
        let n = n.min(out.len());
        for (i, o) in out[..n].iter_mut().enumerate() {
            *o = unsafe { read_volatile(p.add(i)) };
        }
        Ok(n)
    }

    /// Receive exactly `len` bytes into memory at `buf`: whole 512-byte
    /// blocks straight into the buffer in TRBs of up to [`XFER_MAX`], the
    /// tail through the bounce buffer, so nothing past `buf + len` is
    /// written.
    pub fn recv_into(&mut self, buf: usize, len: usize) -> Result<(), Error> {
        self.check()?;
        let whole = len / BULK_GRANULE * BULK_GRANULE;
        let mut done = 0;
        let mut next_report = 64 << 20;
        while done < whole {
            let chunk = (whole - done).min(XFER_MAX);
            self.plat.flush_dcache(buf + done, chunk);
            let r = self.bulk(EP1_OUT, buf + done, chunk, true, &mut || false);
            let got = self.remember(r)?;
            self.plat.invalidate_dcache(buf + done, chunk);
            if got != chunk {
                return self.remember(Err(Error::Transfer((done + got) as u32)));
            }
            done += chunk;
            if done >= next_report {
                self.io.log(format_args!("[usb] {} MiB\n", done >> 20));
                next_report += 64 << 20;
            }
        }
        let tail = len - whole;
        if tail != 0 {
            let p = unsafe { addr_of_mut!((*self.dma).small) } as *mut u8;
            let r = self.bulk(EP1_OUT, p as usize, BULK_GRANULE, true, &mut || false);
            let got = self.remember(r)?;
            self.plat.invalidate_dcache(p as usize, BULK_GRANULE);
            if got != tail {
                return self.remember(Err(Error::Transfer((whole + got) as u32)));
            }
            for i in 0..tail {
                // SAFETY: the caller owns [buf, buf + len).
                unsafe { write_volatile((buf + whole + i) as *mut u8, read_volatile(p.add(i))) };
            }
        }
        Ok(())
    }

    /// Leave the bus and quiesce: end every transfer, Run/Stop off until
    /// halted, events masked. Nothing is DMA'd after this returns Ok.
    pub fn disconnect(&mut self) -> Result<(), Error> {
        let r = (|| {
            for phys in [EP1_IN, EP1_OUT, EP0_IN, EP0_OUT] {
                self.end_transfer(phys)?;
            }
            self.halt()
        })();
        self.w(DEVTEN, 0);
        let v = self.r(GEVNTSIZ);
        self.w(GEVNTSIZ, v | GEVNTSIZ_INTMASK);
        self.configured = false;
        self.ep1_enabled = false;
        self.remember(r)
    }

    /// GSNPSID as read at init.
    pub fn revision(&self) -> u32 {
        self.revision.0
    }

    /// The core and revision read at init.
    pub fn rev(&self) -> Revision {
        self.revision
    }

    /// The register access, for the board.
    pub fn io(&mut self) -> &mut IO {
        &mut self.io
    }
}

/// `fastboot` over this controller (feature `fastboot`).
#[cfg(feature = "fastboot")]
impl<IO: RegIo, P: Dwc3Platform> fastboot::Transport for Udc<'_, IO, P> {
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
    fn log(&mut self, what: &str, line: &[u8]) {
        let s = core::str::from_utf8(line).unwrap_or("?");
        self.io.log(format_args!("[fastboot] {what} {s}\n"));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    extern crate std;
    use std::format;
    #[test]
    fn layout_matches_the_controller() {
        assert_eq!(core::mem::size_of::<Trb>(), 16);
        assert_eq!(core::mem::align_of::<Trb>(), 16);
        assert_eq!(core::mem::align_of::<Dma>(), 4096);
        assert_eq!(EVENT_BYTES % 4, 0);
        assert_eq!(XFER_MAX % BULK_GRANULE, 0);
        assert!(XFER_MAX < trb::SIZE_MASK as usize);
    }
    #[test]
    fn event_encoding() {
        // ep1 IN XferComplete with the short status.
        let e = event::ep(3, event::XFERCOMPLETE, event::STATUS_SHORT);
        assert_eq!((e >> 1) & 0x1f, 3);
        assert_eq!((e >> 6) & 0xf, 1);
        assert_eq!((e >> 12) & 0xf, 2);
        assert_eq!(event::dev(event::DEV_RESET), 0x101);
    }
    #[test]
    fn revisions() {
        let r300 = Revision(Revision::DWC3_300A);
        let r330 = Revision(Revision::DWC3_330A);
        assert_eq!(format!("{r300}"), "DWC_usb3 3.00a");
        assert_eq!(format!("{r330}"), "DWC_usb3 3.30a");
        assert_eq!(format!("{}", Revision(0x5533_194a)), "DWC_usb3 1.94a");
        assert!(r300.valid() && r330.valid());
        assert!(r300.dwc3_prior(Revision::DWC3_310A));
        assert!(!r330.dwc3_prior(Revision::DWC3_310A));
        assert!(!r300.dwc3_at_most(Revision::DWC3_194A));
        assert!(Revision(Revision::DWC3_194A).dwc3_at_most(Revision::DWC3_194A));
        // DWC_usb31/32 are never "prior to" a DWC_usb3 revision.
        let r31 = Revision(0x3331_0000);
        assert!(r31.valid() && !r31.dwc3_prior(Revision::DWC3_330A));
        assert!(!Revision(0x4f54_280a).valid());
    }
    #[test]
    fn devten_is_the_uboot_set() {
        assert_eq!(DEVTEN_USED, 0x1e1f);
    }
}
