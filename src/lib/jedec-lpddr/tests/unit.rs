// SPDX-License-Identifier: GPL-2.0-only
use jedec_lpddr::{geometry, lp4, lp5, timing};

#[test]
fn lp4_axis_is_monotonic_for_lpddr4_and_lpddr4x() {
    for f in [lp4::lp4_vref_permille, lp4::lp4x_vref_permille] {
        let mut prev = 0;
        for i in 0..lp4::VREF_AXIS_LEN {
            let v = f(lp4::vref_axis_op(i));
            assert!(v > prev, "axis point {i}");
            prev = v;
            assert_eq!(lp4::vref_axis_index(lp4::vref_axis_op(i)), Some(i));
        }
    }
    // Range 1 below setting 21 maps onto range 0's part of the axis.
    assert_eq!(lp4::vref_axis_index(0x40 | 5), Some(35));
    assert_eq!(lp4::vref_axis_index(51), None);
    assert_eq!(lp4::vref_axis_index(0x80), None);
    assert_eq!((lp4::cs_mask(1), lp4::cs_mask(2)), (1, 3));
}

#[test]
fn lp5_vref_codes_and_mr18() {
    let v = lp5::VrefCode::from_basis_points(4500).unwrap();
    assert_eq!((v.code(), v.basis_points()), (70, 4500));
    assert_eq!(
        lp5::VrefCode::from_basis_points(4525),
        Err(lp5::EncodingError::Step)
    );
    assert_eq!(lp5::DramVref::DqUpper.mode_register(), 15);
    assert_eq!(lp5::WckCkRatio::Four.replace_mr18(0xff, true), 0x7f);
}

#[test]
fn timing_and_geometry() {
    assert_eq!(timing::ns_to_ck(lp5::timing::T_RCD, 600), 11);
    assert_eq!(timing::checked_ns_to_ck(u32::MAX, 2), None);
    // 2 channels x16 (32-bit bus), 1 rank, LPDDR4X 16 Gb per channel: row 16,
    // col 10, 8 banks: 2 GiB (the OrangePi R2S).
    let g = geometry::Geometry {
        row_bits: 16,
        col_bits: 10,
        bank_bits: 3,
        bg_bits: 0,
        rank_bits: 0,
    };
    assert_eq!(geometry::size_mib(g.address_bits(), 2), Some(2048));
    assert_eq!(geometry::non_binary_mib(8192), Some(6144));
    assert_eq!(geometry::size_mib(10, 1), None);
}
