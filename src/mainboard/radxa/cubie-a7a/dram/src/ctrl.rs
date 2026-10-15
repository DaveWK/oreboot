//! The two DRAM controllers (Synopsys DesignWare DDRCTL with LPDDR5
//! support, one per 16-bit channel) and Allwinner's MCTL_COM glue.
//!
//! The A733 manual lists the address ranges only ("DRAMC Common"
//! 0x0A020000, "DRAM Controller0/1" 0x0A100000/0x0A500000, "DRAM PHY"
//! 0x0A900000). Inside a controller's 4 MiB window:
//!
//! | offset      | block                                             |
//! |-------------|---------------------------------------------------|
//! | `+0x000000` | frequency set 0 timing registers (pstate 0)       |
//! | `+0x010000` | DDRC main registers (MSTR, MRCTRL, DFI, SWCTL...) |
//! | `+0x030000` | address map (ADDRMAP0..12)                        |
//! | `+0x100000` | frequency set 1 (pstate 1), and so on to set 3    |
//!
//! Register names follow the DesignWare DDRCTL register set where the
//! offset and the use match it (MSTR0, STAT, MRCTRL0/1, MRSTAT, PWRCTL,
//! DFIMISC, DFISTAT, SWCTL, SWSTAT, ODTMAP, ADDRMAPn); the rest are named
//! by offset with what the init does with them.

use crate::hw::{Error, Hw};
use crate::para::Para;
use dwc_ddrctl::addrmap::Geometry;

pub const MCTL_COM: usize = 0x0a02_0000;
const CTRL0: usize = 0x0a10_0000;
const CTRL_STRIDE: usize = 0x40_0000;

/// Base of channel `ch`'s controller window.
pub const fn ctrl(ch: usize) -> usize {
    CTRL0 + ch * CTRL_STRIDE
}
/// DDRC main register `off` of channel `ch`.
pub const fn ddrc(ch: usize, off: usize) -> usize {
    ctrl(ch) + 0x1_0000 + off
}
/// Frequency-set (pstate) `ps` timing register `off` of channel `ch`.
pub const fn freq(ch: usize, ps: usize, off: usize) -> usize {
    ctrl(ch) + ps * 0x10_0000 + off
}
/// ADDRMAPn of channel `ch`.
pub const fn addrmap(ch: usize, n: usize) -> usize {
    ctrl(ch) + 0x3_0000 + 4 * n
}

/// The A733's controllers for the shared DesignWare DDRCTL code.
pub const MAP: dwc_ddrctl::Map = dwc_ddrctl::Map {
    main: ddrc,
    freq,
    addrmap,
};

// DDRC main registers: the DesignWare ones (and their fields) are the
// shared crate's; the others are named by offset.
pub use dwc_ddrctl::regs::{
    DFILPCFG0, DFIMISC, DFIPHYMSTR, DFISTAT, DFIUPD0, DFI_FREQUENCY_MASK, DFI_INIT_COMPLETE_EN,
    DFI_INIT_START, MRCTRL0, MRCTRL1, MRSTAT, MSTR0, MSTR4, ODTMAP, PWRCTL, STAT, SWCTL, SWSTAT,
};
pub const DDRC_184: usize = 0x184;
pub const DDRC_18C: usize = 0x18c;
pub const DDRC_208: usize = 0x208;
pub const DDRC_220: usize = 0x220;
pub const DDRC_224: usize = 0x224;
pub const DDRC_280: usize = 0x280;
pub const DDRC_288: usize = 0x288;
pub const DDRC_300: usize = 0x300;
pub const DDRC_308: usize = 0x308;
pub const DDRC_380: usize = 0x380;
/// Allwinner request bit [0] around controller reprogramming; +0xb88
/// reports its acknowledges.
pub const DDRC_B84: usize = 0xb84;
pub const DDRC_B88: usize = 0xb88;
pub const DDRC_D00: usize = 0xd00;

/// Quasi-dynamic register writes: clear SWCTL.sw_done, write, set it and
/// wait for SWSTAT.sw_done_ack.
pub fn sw_done<H: Hw>(hw: &mut H, ch: usize) -> Result<(), Error> {
    dwc_ddrctl::sw_done(hw, &MAP, ch, 100_000).map_err(|t| Error::Timeout(t.0))
}

/// MCTL_COM set-up before the controllers: channel enables, the two AXI
/// ports' arbitration and the PHY's R_PRCM isolation. `two_channels`
/// selects the dual-channel bits.
pub fn com_pre_init<H: Hw>(hw: &mut H, p: &Para) {
    let two = p.channels() == 2;
    let ctrl_reg = MCTL_COM + 0x10;
    hw.clr32(ctrl_reg, 0x14);
    hw.clr32(ctrl_reg, 0x28);
    hw.clrset32(ctrl_reg, 0x3f3f, if two { 0x2728 } else { 0x2208 });
    hw.clrset32(
        MCTL_COM + 0x14,
        3,
        (if two { 2 } else { 0 }) | (if p.is_lpddr5() { 1 } else { 0 }),
    );
    // Port arbitration for the two AXI masters, then enable bit [2].
    hw.write32(MCTL_COM + 0x21c, 0x103);
    hw.write32(MCTL_COM + 0x220, 0x13ff);
    hw.write32(MCTL_COM + 0x224, 0x103);
    hw.write32(MCTL_COM + 0x228, 0x13ff);
    hw.write32(MCTL_COM + 0x21c, 0x107);
    hw.write32(MCTL_COM + 0x224, 0x107);
    // R_PRCM +0x250 [0]: release the DRAM PHY's retention/isolation.
    hw.set32(0x0701_0250, 1);
    hw.udelay(10_000);
}

/// MCTL_COM after the controllers: route both channels and release the
/// PHY interface ([1] then [0]).
pub fn com_post_init<H: Hw>(hw: &mut H, p: &Para) {
    let ctrl_reg = MCTL_COM + 0x10;
    hw.clrset32(ctrl_reg, 0x10, if p.channels() == 2 { 0x14 } else { 0x04 });
    hw.clrset32(ctrl_reg, 1, 2);
    hw.udelay(1);
    hw.set32(ctrl_reg, 3);
    hw.udelay(1);
}

/// Per-channel bus set-up: DDRC +0x380 and the four frequency sets'
/// +0xc00.
pub fn bus_config<H: Hw>(hw: &mut H, ch: usize) {
    hw.write32(ddrc(ch, DDRC_380), 0x8001_3414);
    for ps in 0..4 {
        hw.write32(freq(ch, ps, 0xc00), 0xff);
    }
}

/// MSTR0: LPDDR5 with BL16 (the only type this driver runs), active ranks,
/// bus width (para2[0]: half bus).
pub fn controller_config<H: Hw>(hw: &mut H, p: &Para, ch: usize) {
    let ranks = if p.dual_rank() { 2 } else { 1 };
    hw.write32(
        ddrc(ch, MSTR0),
        dwc_ddrctl::mstr0_lpddr5_bl16(ranks, p.para2() & 1 != 0),
    );
}

/// ODTMAP: each rank's ODT follows its own writes (two ranks), or rank 0
/// only.
pub fn controller_odt<H: Hw>(hw: &mut H, p: &Para, ch: usize) {
    let ranks = if p.dual_rank() { 2 } else { 1 };
    hw.write32(ddrc(ch, ODTMAP), dwc_ddrctl::odtmap(ranks));
}

/// Controller set-up before the PHY: everything up to and including the
/// timing registers of every frequency set.
pub fn com_init<H: Hw>(hw: &mut H, p: &mut Para, ch: usize) -> Result<(), Error> {
    validate_address_map(p, ch)?;
    controller_config(hw, p, ch);
    controller_odt(hw, p, ch);
    hw.clrset32(ddrc(ch, MSTR4), 0x100, 1);
    hw.clr32(ddrc(ch, DDRC_18C), 0x3f);
    hw.clrset32(ddrc(ch, PWRCTL), 0xff, 0x800);
    hw.clrset32(ddrc(ch, DFIUPD0), 0x8000, 0xc000_0000);
    hw.modify32(ddrc(ch, DDRC_280), |v| (v & !(1 << 29)) | (1 << 31));
    hw.set32(ddrc(ch, DDRC_288), 1);
    hw.clrset32(ddrc(ch, DDRC_300), 0x00ff_00ff, 0x00c0_00c0);
    hw.set32(ddrc(ch, DDRC_308), 1);
    hw.read32(ddrc(ch, DFILPCFG0));
    hw.write32(ddrc(ch, DFILPCFG0), 0);
    hw.clrset32(ddrc(ch, DFIPHYMSTR), 0xff00_0001, 0x2500_0000);
    hw.set32(ddrc(ch, DDRC_208), 1);
    hw.clrset32(ddrc(ch, DFIMISC), 0x3_0000, 0x1_0015);
    set_address_map(hw, p, ch)?;
    crate::timing::set_channel_timing(hw, p, ch)?;
    hw.clr32(ddrc(ch, DDRC_B84), 1);
    hw.write32(ddrc(ch, SWCTL), 0);
    hw.clrset32(ddrc(ch, DFIMISC), 0x3_0011, 0x1_0004);
    sw_done(hw, ch)?;
    hw.write32(ddrc(ch, SWCTL), 0);
    hw.clrset32(ddrc(ch, DFIMISC), 0x3_0001, 0x1_0014);
    sw_done(hw, ch)
}

/// The parameter set's geometry as the shared ADDRMAP writer takes it:
/// rank field para2[15:12], non-binary density tpr13[18:16].
pub fn geometry(p: &Para) -> Geometry {
    Geometry {
        col_bits: p.col_bits(),
        row_bits: p.row_bits(),
        bank_bits: p.bank_bits(),
        bg_bits: p.bg_bits(),
        rank_field: p.rank_bits(),
        dual_rank: p.dual_rank(),
        nonbinary: (p.tpr13() >> 16) & 7,
        lpddr5: p.is_lpddr5(),
    }
}

/// Encodings implemented by the ADDRMAP writer (`dwc_ddrctl::addrmap`). This
/// validates its software representation, not the fitted DRAM geometry or
/// PHY topology.
pub fn validate_geometry(p: &Para) -> Result<(), Error> {
    dwc_ddrctl::addrmap::validate(&geometry(p)).map_err(Error::Config)
}

fn validate_address_map(p: &Para, ch: usize) -> Result<(), Error> {
    validate_geometry(p)?;
    if ch >= p.channels() {
        return Err(Error::Config("inactive ADDRMAP controller instance"));
    }
    Ok(())
}

/// ADDRMAPn (and the LPDDR5 +0x220/+0x224 tail) of channel `ch`.
pub fn set_address_map<H: Hw>(hw: &mut H, p: &Para, ch: usize) -> Result<(), Error> {
    validate_address_map(p, ch)?;
    dwc_ddrctl::addrmap::set_address_map(hw, &MAP, ch, &geometry(p)).map_err(Error::Config)
}
