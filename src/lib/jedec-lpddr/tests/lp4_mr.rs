// SPDX-License-Identifier: GPL-2.0-only
//! lp4::mr: round trips, the cited source's register values, the VREF axis.

use jedec_lpddr::lp4::mr::{nwr_code, rl_code, wl_a_code, Mr1, Mr11, Mr2, Mr3, Rzq, Vref};
use jedec_lpddr::lp4::{timing::FREQ_RANGES, vref_axis_index};

/// LiteDRAM `reg()`: OR of value << offset over (offset, width, value).
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
fn codes_follow_table_28() {
    // The source's maps: nWR {6,10,16,20,24,30,34,40}, RL {6,10,14,20,24,28,32,36},
    // WL set A {4,6,8,10,12,14,16,18} -> 0..=7.
    for (i, (nwr, (rl, wl))) in [6u8, 10, 16, 20, 24, 30, 34, 40]
        .into_iter()
        .zip(
            [6u8, 10, 14, 20, 24, 28, 32, 36]
                .into_iter()
                .zip([4u8, 6, 8, 10, 12, 14, 16, 18]),
        )
        .enumerate()
    {
        assert_eq!(nwr_code(nwr), Some(i as u8));
        assert_eq!(rl_code(rl), Some(i as u8));
        assert_eq!(wl_a_code(wl), Some(i as u8));
        assert_eq!(FREQ_RANGES[i].nwr, nwr);
    }
    assert_eq!(nwr_code(7), None);
}

#[test]
fn source_values_encode_identically() {
    // MR1 as the source builds it for BL16, nWR 16.
    let mr1 = Mr1 {
        bl: 0,
        wr_pre: true,
        rd_pre: false,
        nwr_code: nwr_code(16).unwrap(),
        rd_pst: false,
    };
    assert_eq!(
        mr1.encode(),
        Some(reg(&[
            (0, 2, 0),
            (2, 1, 1),
            (3, 1, 0),
            (4, 3, 2),
            (7, 1, 0)
        ]))
    );
    // MR2 for RL 14 / WL 8, set A, no write leveling.
    let mr2 = Mr2 {
        rl_code: rl_code(14).unwrap(),
        wl_code: wl_a_code(8).unwrap(),
        wl_set_b: false,
        write_leveling: false,
    };
    assert_eq!(
        mr2.encode(),
        Some(reg(&[(0, 3, 2), (3, 3, 2), (6, 1, 0), (7, 1, 0)]))
    );
    // MR3 defaults with PDDS RZQ/2; MR11 DQ and CA ODT RZQ/2.
    assert_eq!(
        Mr3::defaults(Rzq::Div2).encode(),
        Some(reg(&[
            (0, 1, 1),
            (1, 1, 0),
            (2, 1, 0),
            (3, 3, 2),
            (6, 1, 0),
            (7, 1, 0)
        ]))
    );
    assert_eq!(
        Mr11 {
            dq_odt: Rzq::Div2,
            ca_odt: Rzq::Div2
        }
        .encode(),
        reg(&[(0, 3, 2), (4, 3, 2)])
    );
    // MR12/MR14 30.4 % range 1 = setting 0b010101.
    assert_eq!(
        Vref {
            range1: true,
            setting: 0b010101
        }
        .encode(),
        Some(reg(&[(0, 6, 0b010101), (6, 1, 1)]))
    );
}

#[test]
fn round_trips_over_every_op() {
    for op in 0..=255u8 {
        if let Some(m) = Mr1::decode(op) {
            assert_eq!(m.encode(), Some(op));
        }
        assert_eq!(Mr2::decode(op).encode(), Some(op));
        if let Some(m) = Mr3::decode(op) {
            assert_eq!(m.encode(), Some(op));
        }
        if let Some(m) = Mr11::decode(op) {
            assert_eq!(m.encode(), op);
        }
        match Vref::decode(op) {
            Some(v) => {
                assert_eq!(v.encode(), Some(op));
                assert!(vref_axis_index(op).is_some());
            }
            None => assert_eq!(vref_axis_index(op), None),
        }
    }
    assert_eq!(
        Vref {
            range1: false,
            setting: 51
        }
        .encode(),
        None
    );
    assert_eq!(Mr1::decode(3), None);
    assert_eq!(Mr11::decode(0x07), None);
}
