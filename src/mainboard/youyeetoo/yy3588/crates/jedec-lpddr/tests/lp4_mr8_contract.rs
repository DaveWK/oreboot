// SPDX-License-Identifier: GPL-2.0-only
//! Device-contract checks are distinct from legacy generic capacity math.
use jedec_lpddr::lp4::density::{channel_mib, ContractField, Mr8Capture, PartPolicy};

const PART: PartPolicy = PartPolicy::SamsungK4ube3d4abMgclRev10;

#[test]
fn every_capture_retains_all_raw_fields_even_on_refusal() {
    for raw in 0..=u8::MAX {
        let capture = Mr8Capture::from_raw(raw);
        assert_eq!(capture.raw(), raw);
        assert_eq!(capture.type_code(), raw & 3);
        assert_eq!(capture.density_code(), (raw >> 2) & 15);
        assert_eq!(capture.width_code(), raw >> 6);
        assert_eq!(
            (capture.width_code() << 6) | (capture.density_code() << 2) | capture.type_code(),
            raw
        );
        match capture.review(PART) {
            Ok(review) => assert_eq!(review.capture(), capture),
            Err(error) => assert_eq!(error.capture(), capture),
        }
    }
}

#[test]
fn exact_part_acceptance_matches_public_type_width_and_organization_contract() {
    // Page25 admits type00/10 and width00. Pages12/14 fix this part's
    // per-channel density to8Gb, encoded0100 by the page25 table.
    for raw in 0..=u8::MAX {
        let result = Mr8Capture::from_raw(raw).review(PART);
        assert_eq!(
            result.is_ok(),
            [0x10, 0x12].contains(&raw),
            "raw={raw:#04x}"
        );
        if let Ok(review) = result {
            assert_eq!(review.channel_mib(), 1024);
            assert_eq!(review.policy(), PART);
            assert_eq!(review.capture().raw(), raw);
        }
    }
}

#[test]
fn equal_capacity_cannot_hide_reserved_type_or_part_unsupported_width() {
    // Both were valid capacities in the legacy decoder, not admitted parts.
    for (raw, expected_mib, field) in [
        (0x11, 1024, ContractField::Type),
        (0x50, 2048, ContractField::Width),
    ] {
        assert_eq!(channel_mib(raw), Ok(expected_mib));
        let error = Mr8Capture::from_raw(raw).review(PART).unwrap_err();
        assert_eq!(error.field(), field);
        assert_eq!(error.capture().raw(), raw);
    }
}

#[test]
fn lpddr4x_type_is_preserved_and_not_globally_rejected() {
    let ordinary = Mr8Capture::from_raw(0x10).review(PART).unwrap();
    let low_voltage = Mr8Capture::from_raw(0x12).review(PART).unwrap();
    assert_eq!(ordinary.channel_mib(), low_voltage.channel_mib());
    assert_ne!(
        ordinary.capture().type_code(),
        low_voltage.capture().type_code()
    );
    assert_ne!(ordinary.capture().raw(), low_voltage.capture().raw());
}

#[test]
fn historical_four_gib_print_is_not_a_candidate_part_identity() {
    // Capacity alone cannot turn the recorded0x18 into this two-rank part.
    assert_eq!(channel_mib(0x18), Ok(2048));
    let error = Mr8Capture::from_raw(0x18).review(PART).unwrap_err();
    assert_eq!(error.field(), ContractField::Density);
    assert_eq!(error.capture().density_code(), 6);
    assert_eq!(error.capture().raw(), 0x18);
}
