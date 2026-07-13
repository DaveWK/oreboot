// SPDX-License-Identifier: GPL-2.0-only
//! Unit tests: the K1 board's extent test (ported) and the layout parameters.

use fit::{source_extent, Layout};

#[test]
fn actual_partition_wins_over_unavailable_raw_fallback() {
    assert_eq!(source_extent(Some((34, 64)), 100, 8192), Ok((34, 64)));
    assert_eq!(source_extent(None, 16384, 8192), Ok((8192, 8192)));
    for (found, cap, fallback) in [
        (None, 8191, 8192),
        (None, 0, 0),
        (Some((90, 11)), 100, 0),
        (Some((1, 0)), 100, 0),
        (Some((1, u64::MAX)), u64::MAX, 0),
    ] {
        assert!(source_extent(found, cap, fallback).is_err());
    }
}

#[test]
fn a_layout_without_a_reserved_slot_or_fixed_data_base_is_a_plain_parameter_set() {
    // A board without a variable-store tail (e.g. an AArch64 board loading
    // TF-A-less firmware + EDK2 + DT) sets reserved/next_size/data_base to
    // None; the crate makes no K1 assumption.
    let l = Layout {
        arch: b"arm64",
        max_fit: 8 << 20,
        max_metadata: 64 << 10,
        data_base: None,
        firmware_load: 0x4000_0000,
        next_load: 0x4020_0000,
        next_size: None,
        reserved: None,
        next_slot: 0x40_0000,
        dt_align: 0x1000,
        firmware_limit: 0x4020_0000,
        payload_limit: 0x8000_0000,
    };
    assert!(l.reserved.is_none() && l.data_base.is_none());
    assert_eq!(
        fit::plan(&l, &[], fit::Region { start: 0, size: 0 }),
        Err(fit::Error::Fdt(fit::fdt_checked::Error::Header))
    );
}
