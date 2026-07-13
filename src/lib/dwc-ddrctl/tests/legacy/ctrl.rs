// Verbatim copy of ore-edk-boot-radxa-a7a ed42550, oreboot tree a803f4f780d7,
// src/mainboard/radxa/cubie-a7a/dram/src/ctrl.rs (equivalence oracle; not built into firmware).
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

use crate::hw::{wait, Error, Hw};
use crate::para::Para;

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

// DDRC main registers.
pub const MSTR0: usize = 0x000;
pub const MSTR4: usize = 0x010;
pub const STAT: usize = 0x014;
pub const MRCTRL0: usize = 0x080;
pub const MRCTRL1: usize = 0x084;
pub const MRSTAT: usize = 0x090;
pub const PWRCTL: usize = 0x180;
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
pub const DFILPCFG0: usize = 0x500;
pub const DFIUPD0: usize = 0x508;
pub const DFIMISC: usize = 0x510;
pub const DFISTAT: usize = 0x514;
pub const DFIPHYMSTR: usize = 0x518;
/// Allwinner request bit [0] around controller reprogramming; +0xb88
/// reports its acknowledges.
pub const DDRC_B84: usize = 0xb84;
pub const DDRC_B88: usize = 0xb88;
pub const SWCTL: usize = 0xc80;
pub const SWSTAT: usize = 0xc84;
pub const ODTMAP: usize = 0xc9c;
pub const DDRC_D00: usize = 0xd00;

/// DFIMISC fields.
pub const DFI_INIT_COMPLETE_EN: u32 = 1 << 0;
pub const DFI_INIT_START: u32 = 1 << 5;
pub const DFI_FREQUENCY_MASK: u32 = 0x1f << 8;

/// Quasi-dynamic register writes: clear SWCTL.sw_done, write, set it and
/// wait for SWSTAT.sw_done_ack.
pub fn sw_done<H: Hw>(hw: &mut H, ch: usize) -> Result<(), Error> {
    hw.write32(ddrc(ch, SWCTL), 1);
    wait(hw, "SWSTAT.sw_done_ack", 100_000, |hw| {
        hw.read32(ddrc(ch, SWSTAT)) & 1 != 0
    })
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

/// MSTR0: device type and burst length, active ranks, bus width.
pub fn controller_config<H: Hw>(hw: &mut H, p: &Para, ch: usize) {
    // [3] LPDDR5 with BL16 ([19:16] = 8): the only type this driver runs.
    const MSTR0_LPDDR5_BL16: u32 = 0x0008_0008;
    let active_ranks = if p.dual_rank() { 3 } else { 1 };
    let half_bus = p.para2() & 1;
    hw.write32(
        ddrc(ch, MSTR0),
        MSTR0_LPDDR5_BL16 | (active_ranks << 24) | (half_bus << 12),
    );
}

/// ODTMAP: each rank's ODT follows its own writes (two ranks), or rank 0
/// only.
pub fn controller_odt<H: Hw>(hw: &mut H, p: &Para, ch: usize) {
    hw.write32(ddrc(ch, ODTMAP), if p.dual_rank() { 0x303 } else { 0x1 });
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
    crate::timing::set_channel_timing(hw, p, ch);
    hw.clr32(ddrc(ch, DDRC_B84), 1);
    hw.write32(ddrc(ch, SWCTL), 0);
    hw.clrset32(ddrc(ch, DFIMISC), 0x3_0011, 0x1_0004);
    sw_done(hw, ch)?;
    hw.write32(ddrc(ch, SWCTL), 0);
    hw.clrset32(ddrc(ch, DFIMISC), 0x3_0001, 0x1_0014);
    sw_done(hw, ch)
}

/// ADDRMAPn: which HIF (host interface word address) bit drives each
/// column, bank, bank group, row and rank bit. Each field holds the HIF bit
/// minus that field's base; 0x1f/0x3f mark an unused bit.
///
/// With C column, BG bank-group and BA bank bits the HIF word address is
/// laid out, from bit 0 up: column bits 0..2 (fixed), bank group, column
/// bits 3.., bank, row, rank. The fields below follow the vendor init for
/// each geometry (col 8..11, row 14..17) of the A733 parameter sets.
/// Encodings implemented by the existing ADDRMAP writer. This validates its
/// software representation, not the fitted DRAM geometry or PHY topology.
pub fn validate_geometry(p: &Para) -> Result<(), Error> {
    if !(8..=11).contains(&p.col_bits())
        || !(14..=18).contains(&p.row_bits())
        || !matches!(p.bank_bits(), 2 | 3)
        || p.bg_bits() > 2
        || !matches!(p.rank_bits(), 0 | 1 | 7)
        || (p.tpr13() >> 16) & 7 > 2
    {
        return Err(Error::Config("geometry outside implemented ADDRMAP fields"));
    }
    Ok(())
}

fn validate_address_map(p: &Para, ch: usize) -> Result<(), Error> {
    validate_geometry(p)?;
    if ch >= p.channels() {
        return Err(Error::Config("inactive ADDRMAP controller instance"));
    }
    Ok(())
}

pub fn set_address_map<H: Hw>(hw: &mut H, p: &Para, ch: usize) -> Result<(), Error> {
    validate_address_map(p, ch)?;
    let col = p.col_bits();
    let row = p.row_bits();
    let ba = p.bank_bits();
    let bg = p.bg_bits();
    let rank_bits = (p.para2() >> 12) & 7;
    // Non-binary density (6/12 GB sets): tpr13[18:16] is 1 or 2.
    let dual_rank_adj = if rank_bits == 7 {
        2
    } else {
        (rank_bits == 1) as u32
    };
    let rep = |v: u32, n: u32| (0..n).fold(0, |acc, i| acc | (v << (8 * i)));

    // ADDRMAP6: column bits 3..6 (bit 3 at HIF 3 + 0, the others shifted
    // past the bank group bits).
    hw.write32(addrmap(ch, 6), rep(bg, 4) & !0xff);
    // ADDRMAP5: column bits 7..10.
    let am5 = match col {
        8 => 0x1f1f_1f00 | bg,
        9 => 0x1f1f_0000 | rep(bg, 2),
        10 => 0x1f00_0000 | rep(bg, 3),
        _ => rep(bg, 4),
    };
    hw.write32(addrmap(ch, 5), am5);
    // ADDRMAP4: bank group bits 0..1.
    let am4 = match bg {
        2 => 0x0101,
        1 => 0x3f01,
        _ => 0x3f3f,
    };
    hw.write32(addrmap(ch, 4), am4);
    // ADDRMAP3: bank bits 0..2, after the column and bank group bits.
    let b = col - 3 + bg;
    let am3 = if ba == 3 {
        rep(b, 3)
    } else {
        0x3f_0000 | rep(b, 2)
    };
    hw.write32(addrmap(ch, 3), am3);
    // ADDRMAP8..11: row bits 0..13, after column, bank group and bank.
    let r = col + bg + ba - 6;
    hw.write32(addrmap(ch, 11), rep(r, 2));
    hw.write32(addrmap(ch, 10), rep(r, 4));
    hw.write32(addrmap(ch, 9), rep(r, 4));
    hw.write32(addrmap(ch, 8), rep(r, 4));
    // ADDRMAP7: row bits 14..17; ADDRMAP1: the rank bit. A non-binary
    // density (tpr13[18:16]) puts the top row bit and the rank bit higher.
    let density = (p.tpr13() >> 16) & 7;
    let nb15 = matches!(density, 1 | 2);
    let nb = density == 1;
    let base = col + bg + ba;
    let x = r + dual_rank_adj;
    let mut rank_hif: Option<u32> = None;
    let am7 = match row {
        14 => 0x1f1f_1f1f,
        15 if nb15 => {
            hw.write32(addrmap(ch, 8), rep(r, 3) | (x << 24));
            rank_hif = Some(base + 7);
            0x1f1f_1f00 | x
        }
        15 => 0x1f1f_1f00 | r,
        16 if nb => {
            rank_hif = Some(base + 8);
            0x1f1f_0000 | rep(x, 2)
        }
        16 => 0x1f1f_0000 | rep(r, 2),
        17 if nb => {
            rank_hif = Some(base + 9);
            0x1f00_0000 | r | (x << 8) | (x << 16)
        }
        17 => 0x1f00_0000 | rep(r, 3),
        _ if nb => {
            rank_hif = Some(base + row - 8);
            rep(r, 2) | (x << 16) | (x << 24)
        }
        _ => rep(r, 4),
    };
    hw.write32(addrmap(ch, 7), am7);
    if row == 16 && nb && p.is_lpddr5() {
        // ADDRMAP12: non-binary device density.
        hw.write32(addrmap(ch, 12), 3);
    }
    if let Some(v) = rank_hif {
        hw.write32(addrmap(ch, 1), v | 0x3f00);
    }
    if p.dual_rank() {
        if !nb15 {
            hw.write32(addrmap(ch, 1), (base + row - 6) | 0x3f00);
        }
    } else {
        hw.write32(addrmap(ch, 1), 0x3f3f);
    }
    if p.is_lpddr5() {
        hw.write32(ddrc(ch, DDRC_220), 0x0a09_0e01);
        hw.write32(ddrc(ch, DDRC_224), 0);
    }
    Ok(())
}
