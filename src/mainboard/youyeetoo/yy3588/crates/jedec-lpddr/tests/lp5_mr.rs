// SPDX-License-Identifier: GPL-2.0-only
//! lp5::mr: the A733 MR16 it replaces, the cited source's register values,
//! round trips.

#[path = "legacy/a733_mr16.rs"]
mod old;

use jedec_lpddr::lp5::mr::{self, BankOrg, Mr16, Mr18, Mr28, Rzq};
use jedec_lpddr::lp5::WckCkRatio;

fn reg(fields: &[(u32, u32, u8)]) -> u8 {
    fields
        .iter()
        .map(|&(off, w, v)| {
            assert!(u32::from(v) < (1 << w));
            v << off
        })
        .fold(0, |a, b| a | b)
}

#[test]
fn mr16_matches_a733() {
    for fsp_wr in 0..4u8 {
        for fsp_op in 0..4u8 {
            for cbt in 0..4u8 {
                for vrcg in [false, true] {
                    let new = Mr16 {
                        fsp_wr,
                        fsp_op,
                        cbt,
                        vrcg,
                        cbt_phase: false,
                    }
                    .encode();
                    assert_eq!(new, old::mr16(fsp_wr, fsp_op, cbt, vrcg));
                }
            }
        }
    }
    // The values the A733 CBT sequence writes (coldinit.rs Mrs2::from_msgblock).
    assert_eq!(
        old::mr16(1, 0, 2, true),
        Mr16 {
            fsp_wr: 1,
            fsp_op: 0,
            cbt: 2,
            vrcg: true,
            cbt_phase: false
        }
        .encode()
    );
    for op in 0..=255u8 {
        assert_eq!(Mr16::decode(op).encode(), op);
    }
}

#[test]
fn mr18_agrees_with_wck_ck_ratio() {
    // replace_mr18 keeps OP[5:0] and sets OP[7] (ratio) and OP[6] (leveling).
    for prior in 0..=255u8 {
        let Some(base) = Mr18::decode(prior & 0x3f) else {
            continue;
        };
        for leveling in [false, true] {
            for (ratio, two) in [(WckCkRatio::Two, true), (WckCkRatio::Four, false)] {
                let m = Mr18 {
                    leveling,
                    ratio_2_1: two,
                    ..base
                };
                assert_eq!(m.encode(), ratio.replace_mr18(prior, leveling));
            }
        }
    }
}

#[test]
fn source_values_encode_identically() {
    assert_eq!(
        mr::mr1(0b0011, false, 0),
        Some(reg(&[(3, 1, 0), (4, 4, 0b0011)]))
    );
    assert_eq!(
        mr::mr2(0b0011, 0b0011),
        Some(reg(&[(0, 4, 0b0011), (4, 4, 0b0011)]))
    );
    assert_eq!(
        mr::mr3(Rzq::Div2, BankOrg::B16, false, 0),
        Some(reg(&[(0, 3, 2), (3, 2, 2), (5, 1, 0)]))
    );
    assert_eq!(
        mr::mr10(0, 0, 0, 0),
        Some(reg(&[(2, 2, 0), (4, 2, 0), (6, 2, 0)]))
    );
    assert_eq!(
        mr::mr11(Rzq::Div2, false, Rzq::Div2),
        reg(&[(0, 3, 2), (3, 1, 0), (4, 3, 2)])
    );
    assert_eq!(
        mr::mr17(Rzq::Disabled, mr::MR17_DEFAULT_7_3),
        Some(reg(&[
            (0, 3, 0),
            (3, 1, 1),
            (4, 1, 1),
            (5, 1, 1),
            (6, 1, 0),
            (7, 1, 0)
        ]))
    );
    let mr18 = Mr18 {
        wck_odt: Rzq::Disabled,
        wck_fm: false,
        wck_always_on: false,
        raw5: false,
        leveling: false,
        ratio_2_1: true,
    };
    assert_eq!(
        mr18.encode(),
        reg(&[(0, 3, 0), (3, 1, 0), (4, 1, 0), (6, 1, 0), (7, 1, 1)])
    );
    assert_eq!(mr::mr20(0b01, 0), Some(reg(&[(0, 2, 0b01)])));
    assert_eq!(mr::MR22_LINK_ECC_OFF, 0);
    let mr28 = Mr28 {
        zq_reset: false,
        zq_stop: false,
        zq_interval: 0b01,
        zq_mode: false,
        raw: 0,
    };
    assert_eq!(
        mr28.encode(),
        Some(reg(&[(0, 1, 0), (1, 1, 0), (2, 2, 0b01), (5, 1, 0)]))
    );
}

#[test]
fn round_trips_and_ranges() {
    for op in 0..=255u8 {
        assert_eq!(Mr28::decode(op).encode(), Some(op));
        if let Some(m) = Mr18::decode(op) {
            assert_eq!(m.encode(), op);
        }
    }
    assert_eq!(mr::mr1(16, false, 0), None);
    assert_eq!(mr::mr2(0, 16), None);
    assert_eq!(mr::mr3(Rzq::Div1, BankOrg::Bg, true, 4), None);
    assert_eq!(mr::mr10(0, 4, 0, 0), None);
    assert_eq!(mr::mr17(Rzq::Div1, 32), None);
    assert_eq!(mr::mr20(4, 0), None);
    assert_eq!(Mr18::decode(7), None);
}
