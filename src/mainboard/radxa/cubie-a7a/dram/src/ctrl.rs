// SPDX-License-Identifier: GPL-2.0-only
//! The two DRAM controllers (Synopsys DesignWare DDRCTL with LPDDR5
//! support, one per 16-bit channel) and Allwinner's MCTL_COM glue.
//!
//! Address ranges: DRAMC Common 0x0A020000, DRAM Controller0/1
//! 0x0A100000/0x0A500000, DRAM PHY 0x0A900000. Inside a controller's 4 MiB window:
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

/// HIF bit of ADDRMAP1's chip-select bit 0 field at value 0 (DesignWare
/// DDRCTL: each ADDRMAP field holds its HIF bit minus the field's base).
const CS_BIT0_BASE: u32 = 6;

/// Where the address map puts the ranks in the system address space: the
/// stages that reach the DRAM through the controller (write training, the
/// memory tests) see a rank only at the addresses that select it.
///
/// One HIF word is one 16-bit word of one channel; the two channels make
/// one 32-bit system word, so HIF bit n is bit n + 2 of the DRAM offset
/// (system address minus the DRAM base), n + 1 with one half-width channel.
///
/// - Binary densities: the rank bit is the top HIF bit. CXMT 4 GB: offset
///   bit 31, rank 1 at 0xc0000000-0x13fffffff.
/// - Non-binary densities: the vendor map puts the two top row bits above
///   the rank bit, so that the invalid quarter (both top row bits set) is
///   the top of the address space and the usable part is contiguous (the
///   6 GB board runs Linux on 6144 contiguous MiB with this map). CXMT
///   6 GB: offset bit 30, the ranks alternate every GiB and rank 1 starts
///   at 0x80000000.
///
/// Derived from the map's encodings (the unit tests check that the field
/// bases give a dense layout for every parameter set); not yet measured on
/// a board.
///
/// The default write stage decodes the live map with this type.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RankMap {
    /// Bit of the DRAM offset that selects the rank; `None` with one rank.
    pub rank_bit: Option<u32>,
    /// Usable size in bytes (from [`crate::size::boot_size_mib`]).
    pub size: u64,
}

impl RankMap {
    /// From the ADDRMAP1 write of the shared writer for `p`'s geometry;
    /// `size` is the usable size in bytes (the caller's
    /// [`crate::size::boot_size_mib`]).
    pub fn of(p: &Para, size: u64) -> Result<Self, Error> {
        use dwc_ddrctl::addrmap::{address_map, Target};
        let w = address_map(&geometry(p)).map_err(Error::Config)?;
        let mut cs = 0x3f;
        for &(t, v) in w.as_slice() {
            if t == Target::AddrMap(1) {
                cs = v & 0x3f;
            }
        }
        // As size::checked_size_mib: a 32-bit system word unless a single
        // half-width channel is populated.
        let half_width = p.para2() & 0xf != 0;
        let shift = if !half_width || p.channels() == 2 {
            2
        } else {
            1
        };
        Ok(Self {
            rank_bit: Self::bit_from_addrmap1(cs, shift),
            size,
        })
    }

    /// The DRAM offset bit that selects the rank, from an ADDRMAP1 value
    /// (as written, or read back from a controller): `None` when the
    /// chip-select field is unused (one rank). `shift` is 2 for a 32-bit
    /// system word (two 16-bit channels), 1 for one half-width channel.
    pub const fn bit_from_addrmap1(addrmap1: u32, shift: u32) -> Option<u32> {
        let cs = addrmap1 & 0x3f;
        if cs == 0x3f {
            None
        } else {
            Some(cs + CS_BIT0_BASE + shift)
        }
    }

    /// The rank that DRAM offset `off` selects.
    pub fn rank(&self, off: u64) -> u32 {
        match self.rank_bit {
            Some(b) => ((off >> b) & 1) as u32,
            None => 0,
        }
    }

    /// The lowest block of `rank`: its DRAM offset and length, or `None`
    /// when the rank is not fitted or lies above the usable size.
    pub fn first_block(&self, rank: u32) -> Option<(u64, u64)> {
        let (start, len) = match (self.rank_bit, rank) {
            (None, 0) => (0, self.size),
            (Some(b), 0 | 1) => ((rank as u64) << b, 1u64 << b),
            _ => return None,
        };
        if start >= self.size {
            return None;
        }
        Some((start, len.min(self.size - start)))
    }

    /// Bytes of `rank` below DRAM offset `limit` (what a 32-bit stage
    /// reaches with `limit` = 4 GiB minus the DRAM base).
    pub fn reach(&self, rank: u32, limit: u64) -> u64 {
        let end = limit.min(self.size);
        let Some(b) = self.rank_bit else {
            return if rank == 0 { end } else { 0 };
        };
        if rank > 1 {
            return 0;
        }
        let block = 1u64 << b;
        let mut n = 0;
        let mut at = (rank as u64) << b;
        while at < end {
            n += block.min(end - at);
            at += 2 * block;
        }
        n
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use crate::sets::{CXMT_4G, CXMT_6G, HYNIX_4G, PARAM_SETS};
    use dwc_ddrctl::addrmap::{address_map, Target};
    use std::collections::BTreeMap;
    use std::vec::Vec;

    const GIB: u64 = 1 << 30;

    fn set(i: usize) -> Para {
        Para::from_sysconfig(&PARAM_SETS[i].para)
    }

    fn rank_map(p: &Para) -> RankMap {
        let mib = crate::size::boot_size_mib(p).unwrap();
        RankMap::of(p, (mib as u64) << 20).unwrap()
    }

    /// The final ADDRMAPn values for `p`.
    fn addrmaps(p: &Para) -> BTreeMap<usize, u32> {
        let mut m = BTreeMap::new();
        for &(t, v) in address_map(&geometry(p)).unwrap().as_slice() {
            if let Target::AddrMap(n) = t {
                m.insert(n, v);
            }
        }
        m
    }

    /// (name, HIF bit) of every mapped address bit, from the ADDRMAP
    /// fields: register, shift, width, base (DesignWare DDRCTL layout; an
    /// all-ones field is unused). Column bits 0..2 are fixed at HIF 0..2.
    fn hif_bits(p: &Para) -> Vec<(std::string::String, u32)> {
        const FIELDS: [(&str, usize, u32, u32, u32); 30] = [
            ("cs0", 1, 0, 6, 6),
            ("ba0", 3, 0, 6, 3),
            ("ba1", 3, 8, 6, 4),
            ("ba2", 3, 16, 6, 5),
            ("bg0", 4, 0, 6, 3),
            ("bg1", 4, 8, 6, 4),
            ("col7", 5, 0, 5, 7),
            ("col8", 5, 8, 5, 8),
            ("col9", 5, 16, 5, 9),
            ("col10", 5, 24, 5, 10),
            ("col3", 6, 0, 4, 3),
            ("col4", 6, 8, 4, 4),
            ("col5", 6, 16, 4, 5),
            ("col6", 6, 24, 4, 6),
            ("row14", 7, 0, 5, 20),
            ("row15", 7, 8, 5, 21),
            ("row16", 7, 16, 5, 22),
            ("row17", 7, 24, 5, 23),
            ("row10", 8, 0, 5, 16),
            ("row11", 8, 8, 5, 17),
            ("row12", 8, 16, 5, 18),
            ("row13", 8, 24, 5, 19),
            ("row6", 9, 0, 5, 12),
            ("row7", 9, 8, 5, 13),
            ("row8", 9, 16, 5, 14),
            ("row9", 9, 24, 5, 15),
            ("row2", 10, 0, 5, 8),
            ("row3", 10, 8, 5, 9),
            ("row4", 10, 16, 5, 10),
            ("row5", 10, 24, 5, 11),
        ];
        let m = addrmaps(p);
        let mut out: Vec<(std::string::String, u32)> =
            (0..3).map(|c| (std::format!("col{c}"), c)).collect();
        let mut field = |name: &str, reg: usize, shift: u32, width: u32, base: u32| {
            let mask = (1u32 << width) - 1;
            let v = (m[&reg] >> shift) & mask;
            if v != mask {
                out.push((name.into(), v + base));
            }
        };
        for (name, reg, shift, width, base) in FIELDS {
            field(name, reg, shift, width, base);
        }
        field("row0", 11, 0, 5, 6);
        field("row1", 11, 8, 5, 7);
        out
    }

    #[test]
    fn every_sets_address_map_is_a_dense_hif_layout() {
        // Checks the field bases used here and in RankMap: with a wrong
        // base two address bits would share a HIF bit or leave a gap.
        for (i, s) in PARAM_SETS.iter().enumerate() {
            let p = set(i);
            let bits = hif_bits(&p);
            let want =
                p.col_bits() + p.bg_bits() + p.bank_bits() + p.row_bits() + p.dual_rank() as u32;
            assert_eq!(bits.len() as u32, want, "{}: {bits:?}", s.name);
            let mut hif: Vec<u32> = bits.iter().map(|b| b.1).collect();
            hif.sort_unstable();
            assert_eq!(hif, (0..want).collect::<Vec<_>>(), "{}: {bits:?}", s.name);
        }
    }

    #[test]
    fn cxmt_4gb_address_map_and_ranks() {
        let p = set(CXMT_4G);
        let m = addrmaps(&p);
        assert_eq!(
            (m[&1], m[&3], m[&4], m[&5], m[&6], m[&7]),
            (
                0x3f17,
                0x003f_0909,
                0x0101,
                0x1f02_0202,
                0x0202_0200,
                0x1f1f_1f08
            )
        );
        assert!(!m.contains_key(&12));
        let r = rank_map(&p);
        assert_eq!(
            r,
            RankMap {
                rank_bit: Some(31),
                size: 4 * GIB
            }
        );
        assert_eq!((r.rank(0), r.rank(2 * GIB - 4), r.rank(2 * GIB)), (0, 0, 1));
        // System addresses: rank 0 0x40000000-0xbfffffff, rank 1 from
        // 0xc0000000; a 32-bit stage reaches all of rank 0 and half of
        // rank 1.
        assert_eq!(r.first_block(0), Some((0, 2 * GIB)));
        assert_eq!(r.first_block(1), Some((2 * GIB, 2 * GIB)));
        assert_eq!(RankMap::bit_from_addrmap1(m[&1], 2), Some(31));
        assert_eq!(RankMap::bit_from_addrmap1(0x3f3f, 2), None);
        assert_eq!((r.reach(0, 3 * GIB), r.reach(1, 3 * GIB)), (2 * GIB, GIB));
        // The window bt0 tests today (to 0xbfffffff) has none of rank 1.
        assert_eq!(r.reach(1, 2 * GIB), 0);
    }

    #[test]
    fn cxmt_6gb_address_map_and_ranks() {
        let p = set(CXMT_6G);
        let m = addrmaps(&p);
        // Against the 4 GB map: ADDRMAP7 (row 14 and 15 one HIF bit up),
        // ADDRMAP1 (the rank bit below them) and ADDRMAP12 (non-binary
        // density) differ; nothing else.
        assert_eq!(
            (m[&1], m[&3], m[&4], m[&5], m[&6], m[&7], m[&12]),
            (
                0x3f16,
                0x003f_0909,
                0x0101,
                0x1f02_0202,
                0x0202_0200,
                0x1f1f_0909,
                3
            )
        );
        let m4 = addrmaps(&set(CXMT_4G));
        for n in [3, 4, 5, 6, 8, 9, 10, 11] {
            assert_eq!(m[&n], m4[&n], "ADDRMAP{n}");
        }
        // Rows 0..13 at HIF 14..27, the rank at 28, rows 14 and 15 at 29
        // and 30.
        let bits = hif_bits(&p);
        let at = |n: &str| bits.iter().find(|b| b.0 == n).unwrap().1;
        assert_eq!(
            (at("row13"), at("cs0"), at("row14"), at("row15")),
            (27, 28, 29, 30)
        );

        let r = rank_map(&p);
        assert_eq!(
            r,
            RankMap {
                rank_bit: Some(30),
                size: 6 * GIB
            }
        );
        // The ranks alternate every GiB: system 0x40000000 rank 0,
        // 0x80000000 rank 1, 0xc0000000 rank 0, 0x100000000 rank 1, ...
        for g in 0..6 {
            assert_eq!(r.rank(g * GIB), (g & 1) as u32);
            assert_eq!(r.rank(g * GIB + GIB - 4), (g & 1) as u32);
        }
        assert_eq!(r.first_block(0), Some((0, GIB)));
        assert_eq!(r.first_block(1), Some((GIB, GIB)));
        assert_eq!(RankMap::bit_from_addrmap1(m[&1], 2), Some(30));
        // 3 GiB per rank; a 32-bit stage reaches 2 GiB of rank 0 and 1 GiB
        // of rank 1, and the window bt0 tests today (to 0xbfffffff) already
        // holds 1 GiB of each.
        assert_eq!(
            (r.reach(0, 8 * GIB), r.reach(1, 8 * GIB)),
            (3 * GIB, 3 * GIB)
        );
        assert_eq!((r.reach(0, 3 * GIB), r.reach(1, 3 * GIB)), (2 * GIB, GIB));
        assert_eq!((r.reach(0, 2 * GIB), r.reach(1, 2 * GIB)), (GIB, GIB));
        // The invalid quarter (rows 14 and 15 both set) is exactly the
        // offsets from 6 GiB up: the usable 6144 MiB are contiguous.
        let invalid = (1u64 << (at("row14") + 2)) | (1u64 << (at("row15") + 2));
        assert_eq!(invalid, 6 * GIB);
        assert_eq!(1u64 << (bits.len() as u32 + 2), 8 * GIB);
    }

    #[test]
    fn one_rank_has_no_rank_bit() {
        let r = rank_map(&set(HYNIX_4G));
        assert_eq!(
            r,
            RankMap {
                rank_bit: None,
                size: 4 * GIB
            }
        );
        assert_eq!(r.rank(3 * GIB), 0);
        assert_eq!(r.first_block(0), Some((0, 4 * GIB)));
        assert_eq!(r.first_block(1), None);
        assert_eq!((r.reach(0, 3 * GIB), r.reach(1, 3 * GIB)), (3 * GIB, 0));
    }
}
