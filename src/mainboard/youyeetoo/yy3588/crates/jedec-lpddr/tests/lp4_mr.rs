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

// The literal other-bit list covers OP7:6 and OP2:0; PDDS occupies OP5:3.
const MR3_OTHER_BITS: [u8; 32] = [
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47,
    0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0xc0, 0xc1, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7,
];

#[test]
fn mr3_reserved_pdds_refuses_encode_and_decode() {
    const RESERVED_DEFAULT: Option<u8> = Mr3::defaults(Rzq::Disabled).encode();
    const RESERVED_DECODE: Option<Mr3> = Mr3::decode(0x01);
    assert_eq!(RESERVED_DEFAULT, None, "MR3 PDDS000 must not encode");
    assert_eq!(RESERVED_DECODE, None, "MR3 PDDS000 must not decode");
    for other in MR3_OTHER_BITS {
        assert_eq!(
            Mr3 {
                pdds: Rzq::Disabled,
                other
            }
            .encode(),
            None,
            "MR3 PDDS000 encode other={other:#04x}"
        );
        assert_eq!(
            Mr3::decode(other),
            None,
            "MR3 PDDS000 decode op={other:#04x}"
        );
        assert_eq!(
            Mr3::decode(other | 0x38),
            None,
            "MR3 PDDS111 decode op={:#04x}",
            other | 0x38
        );
    }
}

#[test]
fn mr3_valid_pdds_preserves_other_bits_and_refuses_overlap() {
    // Numeric code/bit positions only; the revised 011 resistance label is unresolved.
    let allowed = [
        (Rzq::Div1, 0x08),
        (Rzq::Div2, 0x10),
        (Rzq::Div3, 0x18),
        (Rzq::Div4, 0x20),
        (Rzq::Div5, 0x28),
        (Rzq::Div6, 0x30),
    ];
    for (pdds, field) in allowed {
        for other in MR3_OTHER_BITS {
            let expected = field | other;
            let value = Mr3 { pdds, other };
            assert_eq!(value.encode(), Some(expected));
            assert_eq!(Mr3::decode(expected), Some(value));
        }
    }
    for pdds in [
        Rzq::Disabled,
        Rzq::Div1,
        Rzq::Div2,
        Rzq::Div3,
        Rzq::Div4,
        Rzq::Div5,
        Rzq::Div6,
    ] {
        for other in 0..=u8::MAX {
            if other & 0x38 != 0 {
                assert_eq!(Mr3 { pdds, other }.encode(), None);
            }
        }
    }
}

#[test]
fn mr11_disabled_zero_remains_register_specific() {
    let zero = Mr11 {
        dq_odt: Rzq::Disabled,
        ca_odt: Rzq::Disabled,
    };
    assert_eq!(zero.encode(), 0x00);
    assert_eq!(
        Mr11::decode(0x00),
        Some(zero),
        "MR11 disabled zero must decode"
    );
    let fields = [
        (Rzq::Disabled, 0x00u8),
        (Rzq::Div1, 0x01),
        (Rzq::Div2, 0x02),
        (Rzq::Div3, 0x03),
        (Rzq::Div4, 0x04),
        (Rzq::Div5, 0x05),
        (Rzq::Div6, 0x06),
    ];
    for (dq_odt, dq_code) in fields {
        for (ca_odt, ca_code) in fields {
            let value = Mr11 { dq_odt, ca_odt };
            let expected = dq_code | (ca_code << 4);
            assert_eq!(value.encode(), expected);
            assert_eq!(Mr11::decode(expected), Some(value));
        }
    }
}
