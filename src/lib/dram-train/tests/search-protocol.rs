// SPDX-License-Identifier: GPL-2.0-only
use dram_train::search::lpddr5::{DramVref, EncodingError, VrefCode, WckCkRatio};

#[test]
fn vref_exhaustive_roundtrip_and_unrelated_bit_preservation() {
    for raw in 0..=u8::MAX {
        if raw > 127 {
            assert_eq!(VrefCode::new(raw), Err(EncodingError::Range));
            continue;
        }
        let code = VrefCode::new(raw).unwrap();
        assert_eq!(code.basis_points(), 1000 + 50 * u16::from(raw));
        assert_eq!(VrefCode::from_basis_points(code.basis_points()), Ok(code));
        for prior in 0..=u8::MAX {
            let value = code.replace_field(prior);
            assert_eq!(value & 0x7f, raw);
            assert_eq!(value & 0x80, prior & 0x80);
        }
    }
    for value in 0..=u16::MAX {
        let expected = if !(1000..=7350).contains(&value) {
            Err(EncodingError::Range)
        } else if (value - 1000) % 50 != 0 {
            Err(EncodingError::Step)
        } else {
            Ok(VrefCode::new(((value - 1000) / 50) as u8).unwrap())
        };
        assert_eq!(VrefCode::from_basis_points(value), expected);
    }
    assert_eq!(DramVref::Ca.mode_register(), 12);
    assert_eq!(DramVref::DqLower.mode_register(), 14);
    assert_eq!(DramVref::DqUpper.mode_register(), 15);
}

#[test]
fn mr18_exhaustive_ratio_leveling_and_unrelated_bit_preservation() {
    for prior in 0..=u8::MAX {
        for ratio in [WckCkRatio::Two, WckCkRatio::Four] {
            for leveling in [false, true] {
                let value = ratio.replace_mr18(prior, leveling);
                assert_eq!(value & 0x3f, prior & 0x3f);
                assert_eq!(value >> 7, u8::from(ratio.value() == 2));
                assert_eq!((value >> 6) & 1, u8::from(leveling));
                assert_eq!(ratio.replace_mr18(value, leveling), value);
                assert_eq!(ratio.replace_mr18(value, !leveling) ^ value, 0x40);
            }
        }
    }
}
