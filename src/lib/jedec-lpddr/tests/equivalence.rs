// SPDX-License-Identifier: GPL-2.0-only
//! The crate against verbatim copies of the board code it replaces.

#[path = "legacy/a733.rs"]
mod a733;
#[path = "../../dram-train/tests/legacy/k1_train_eye.rs"]
mod k1;

use jedec_lpddr::{geometry, lp4, lp5, timing};

#[test]
fn ns_to_ck_matches_a733_ns() {
    for t in (0..3_000_000u32)
        .step_by(997)
        .chain([0, 1, 7, 999, 1000, 1001, 200_000, 2_000_000])
    {
        for mhz in [1, 100, 200, 300, 400, 600, 800, 1066, 1600, 2133] {
            if t.checked_mul(mhz).is_some() {
                assert_eq!(timing::ns_to_ck(t, mhz), a733::ns(t, mhz));
                assert_eq!(timing::checked_ns_to_ck(t, mhz), Some(a733::ns(t, mhz)));
            } else {
                assert_eq!(timing::checked_ns_to_ck(t, mhz), None);
            }
        }
    }
}

#[test]
fn lp5_timings_match_a733() {
    use a733::lp5 as old;
    use lp5::timing as new;
    let pairs = [
        (new::T_FAW, old::T_FAW),
        (new::T_RRD, old::T_RRD),
        (new::T_RCD, old::T_RCD),
        (new::T_XP, old::T_XP),
        (new::T_WTR_L, old::T_WTR_L),
        (new::T_RTP, old::T_RTP),
        (new::T_RPAB, old::T_RPAB),
        (new::T_RAS, old::T_RAS),
        (new::T_REFI, old::T_REFI),
        (new::T_RFCAB, old::T_RFCAB),
        (new::T_INIT_200US, old::T_INIT_200US),
        (new::T_ZQCAL, old::T_ZQCAL),
        (new::T_XSR_EXTRA, old::T_XSR_EXTRA),
        (new::T_CKE, old::T_CKE),
        (new::T_ESCKE, old::T_ESCKE),
        (new::T_CCDMW_NS, old::T_CCDMW_NS),
        (new::T_CMD_1NS, old::T_CMD_1NS),
        (new::T_MRW_40, old::T_MRW_40),
        (new::T_ZQRESET_6US, old::T_ZQRESET_6US),
        (new::T_ZQLAT_30, old::T_ZQLAT_30),
        (new::T_ZQCS_50, old::T_ZQCS_50),
    ];
    for (n, o) in pairs {
        assert_eq!(n, o);
    }
}

/// The A733 checked_size_mib rebuilt from geometry: the bus is 32 bits
/// unless one half-width channel is populated.
fn size(p: &a733::Para) -> Result<u32, a733::Error> {
    let g = geometry::Geometry {
        row_bits: p.row_bits(),
        col_bits: p.col_bits(),
        bank_bits: p.bank_bits(),
        bg_bits: p.bg_bits(),
        rank_bits: p.rank_bits(),
    };
    let half_width = p.para2() & 0xf != 0;
    let bus = if !half_width || p.channels() == 2 {
        2
    } else {
        1
    };
    let mut mib = geometry::size_mib(g.address_bits(), bus).ok_or(a733::Error::Config("x"))?;
    if p.tpr13() & (7 << 16) != 0 && p.para2() >> 30 != 2 {
        mib = geometry::non_binary_mib(mib).ok_or(a733::Error::Config("x"))?;
    }
    if !(2..=0x7fff).contains(&mib) {
        return Err(a733::Error::Config("x"));
    }
    Ok(mib)
}

#[test]
fn geometry_matches_a733_size() {
    let mut n = 0;
    for row in 12..=18 {
        for col in 8..=11 {
            for bank in 2..=3 {
                for bg in 0..=2 {
                    for rank in 0..=1 {
                        for channels in 1..=2 {
                            for para2 in [0u32, 1, 0x8000_0000, 0x4000_0001] {
                                for tpr13 in [0u32, 1 << 16] {
                                    let p = a733::Para {
                                        row,
                                        bank,
                                        col,
                                        bg,
                                        rank,
                                        para2,
                                        channels,
                                        tpr13,
                                    };
                                    assert_eq!(
                                        size(&p).is_ok(),
                                        a733::checked_size_mib(&p).is_ok()
                                    );
                                    assert_eq!(size(&p).ok(), a733::checked_size_mib(&p).ok());
                                    n += 1;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    assert!(n > 5000);
}

#[test]
fn lp4_axis_matches_k1() {
    assert_eq!(lp4::VREF_AXIS_LEN, k1::TX_AXIS_LEN);
    for i in 0..lp4::VREF_AXIS_LEN {
        assert_eq!(lp4::vref_axis_op(i), k1::tx_axis_mr14(i));
    }
    for op in 0..=255u8 {
        assert_eq!(lp4::vref_axis_index(op), k1::tx_axis_index(op));
        assert_eq!(lp4::lp4x_vref_permille(op), k1::lp4x_mr14_permille(op));
    }
}
