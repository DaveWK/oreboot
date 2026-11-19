// SPDX-License-Identifier: GPL-2.0-only
//! The Cubie A7A's DRAM parameter sets (vendor parameters).
//!
//! From Radxa's `configs/cubie_a7a/sys_config.fex` (radxa/allwinner-device
//! 6d911d15, sha256 d0764c8b..., the revision radxa-pkg/u-boot-aw2501 builds
//! boot0 from). The 4 GB sets are unchanged at the newest device-a733-v1.4.6
//! tip (79d54f4), which retunes 8/12 GB to 2040 MHz together with boot0
//! v0.6034; those are not taken here. Vendor boot0 picks one at run time
//! (`select_mode = 3`: PH16/PH17 + GPADC channel 1).

pub struct ParamSet {
    pub name: &'static str,
    pub para: [u32; 32],
}

/// Build a Cubie A7A LPDDR5 set from the words that differ between them.
#[allow(clippy::too_many_arguments)]
const fn lpddr5(
    name: &'static str,
    clk: u32,
    para1: u32,
    para2: u32,
    mr11: u32,
    mr14: u32,
    mr17: u32,
    tpr2: u32,
    tpr10: u32,
    tpr13: u32,
) -> ParamSet {
    ParamSet {
        name,
        para: [
            clk,
            9,
            0x0e0e_0e0e,
            0x0f0f_0f0f,
            0xec03_0e0f, // clk, LPDDR5, odt/drive
            0,
            para1,
            para2, // para0..2
            0,
            0,
            0,
            0x6,
            0,
            0,
            0, // mr0..mr6
            mr11,
            0x44,
            0,
            mr14,
            0,
            mr17,
            0, // mr11, mr12, mr13, mr14, mr16, mr17, mr22
            0x4040,
            0,
            tpr2,
            0x3800,
            0x3514, // tpr0..3, tpr6
            tpr10,
            0,
            0,
            tpr13,
            0, // tpr10..14
        ],
    }
}

pub static PARAM_SETS: [ParamSet; 6] = [
    // [dram_para]  "cxmt 4g ddr"
    lpddr5(
        "CXMT 4GB",
        2400,
        0xa0fa,
        0x1001,
        0x13,
        0x2e,
        0x06,
        0x170b_0703,
        0x325f_0000,
        0x65,
    ),
    // [dram_para1] "cxmt 6g ddr"
    lpddr5(
        "CXMT 6GB",
        2400,
        0xa10a,
        0x1001,
        0x13,
        0x2e,
        0x06,
        0x170b_0703,
        0x325f_0000,
        0x1_0065,
    ),
    // [dram_para2] "cxmt 8g ddr"
    lpddr5(
        "CXMT 8GB",
        1800,
        0xa10a,
        0x1001,
        0x1b,
        0x1d,
        0x86,
        0x1108_0503,
        0xb25f_0000,
        0x65,
    ),
    // [dram_para3] "cxmt 12g ddr"
    lpddr5(
        "CXMT 12GB",
        1800,
        0xa11a,
        0x1001,
        0x12,
        0x34,
        0x86,
        0x1108_0503,
        0xb25f_0000,
        0x1_0065,
    ),
    // [dram_para7] "skhnix 4g ddr"
    lpddr5(
        "SK hynix 4GB",
        2400,
        0xa10a,
        0x0001,
        0x12,
        0x34,
        0x06,
        0x170b_0703,
        0x325f_0000,
        0x65,
    ),
    // Last resort for 4 GB boards: [dram_para] of device-a733-v1.4.8
    // (Nov 2025 release, "stock DRAM 1800 MHz" per linux-sunxi). Not
    // selected by the strap; only tried if both 2400 MHz sets fail.
    lpddr5(
        "4GB @1800 (v1.4.8)",
        1800,
        0xa10a,
        0x1001,
        0x12,
        0x34,
        0x86,
        0x1108_0503,
        0x325f_0000,
        0x1_0065,
    ),
];
pub const CXMT_4G: usize = 0;
pub const CXMT_6G: usize = 1;
pub const HYNIX_4G: usize = 4;
pub const SAFE_4G: usize = 5;

/// DDR-PARA-SEL-ADC strap voltages in mV from the A7A V1.10 schematic
/// (sheet 6) and the set each one selects. The SK hynix 4 GB board leaves
/// the divider unpopulated (pulled up to 1.8 V); 1569 mV is "DDR PARA 7".
/// Read on the boards: 163 mV (4 GB CXMT), 380-381 mV (6 GB CXMT).
pub const ADC_STRAPS: [(u32, usize); 6] = [
    (163, CXMT_4G),
    (382, CXMT_6G),
    (608, 2),
    (811, 3),
    (1569, HYNIX_4G),
    (1800, HYNIX_4G),
];

/// The strap entry nearest to the measured `mv`: its nominal voltage and
/// the index of the set in [`PARAM_SETS`] (the first entry on a tie).
#[inline]
pub fn by_strap_mv(mv: u32) -> (u32, usize) {
    *ADC_STRAPS
        .iter()
        .min_by_key(|(s, _)| s.abs_diff(mv))
        .unwrap()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::para::Para;

    #[test]
    fn the_boards_strap_readings_select_their_sets() {
        // Serial logs: the 4 GB board reads 163 mV, the 6 GB board 380 and
        // 381 mV.
        assert_eq!(by_strap_mv(163), (163, CXMT_4G));
        for mv in [380, 381, 382] {
            assert_eq!(by_strap_mv(mv), (382, CXMT_6G));
        }
        assert_eq!(PARAM_SETS[CXMT_4G].name, "CXMT 4GB");
        assert_eq!(PARAM_SETS[CXMT_6G].name, "CXMT 6GB");
    }

    #[test]
    fn every_strap_window_is_at_least_100_mv_wide_and_selects_a_set() {
        // The decision points are the midpoints between neighbours; a
        // reading within 50 mV of a nominal value never changes the set.
        for &(mv, idx) in ADC_STRAPS.iter() {
            assert!(idx < PARAM_SETS.len());
            for d in 0..=50 {
                assert_eq!(by_strap_mv(mv - d).1, idx, "{mv} - {d}");
                assert_eq!(by_strap_mv(mv + d).1, idx, "{mv} + {d}");
            }
        }
        // 4 GB / 6 GB boundary: (163 + 382) / 2 = 272.5 mV.
        assert_eq!(by_strap_mv(272).1, CXMT_4G);
        assert_eq!(by_strap_mv(273).1, CXMT_6G);
        assert_eq!(by_strap_mv(0).1, CXMT_4G);
        assert_eq!(by_strap_mv(u32::MAX).1, HYNIX_4G);
    }

    #[test]
    fn the_6gb_set_is_the_4gb_set_with_one_more_row_at_three_quarters() {
        let a = Para::from_sysconfig(&PARAM_SETS[CXMT_4G].para);
        let b = Para::from_sysconfig(&PARAM_SETS[CXMT_6G].para);
        // Geometry: 10 columns, 15 or 16 rows, 4 bank groups of 4 banks,
        // two ranks, two channels.
        assert_eq!(
            (a.col_bits(), a.row_bits(), a.bank_bits(), a.bg_bits()),
            (10, 15, 2, 2)
        );
        assert_eq!(
            (b.col_bits(), b.row_bits(), b.bank_bits(), b.bg_bits()),
            (10, 16, 2, 2)
        );
        assert!(a.dual_rank() && b.dual_rank());
        assert_eq!((a.channels(), b.channels()), (2, 2));
        assert_eq!(((a.tpr13() >> 16) & 7, (b.tpr13() >> 16) & 7), (0, 1));
        // Every other word is the same: clock, drive/ODT, mode registers,
        // pstate dividers, rails, tpr10.
        for i in 0..32 {
            if i != 6 && i != 30 {
                assert_eq!(a.w[i], b.w[i], "word {i}");
            }
        }
        assert_eq!(crate::size::boot_size_mib(&a), Ok(4096));
        assert_eq!(crate::size::boot_size_mib(&b), Ok(6144));
        // The vendor library's hand-over word on the 6 GB board (serial
        // log): para2 0x18001001 = size 6144 MiB with bit 31.
        assert_eq!((0x1800_1001u32 >> 16) & 0x7fff, 6144);
    }
}
