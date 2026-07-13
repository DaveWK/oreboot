// SPDX-License-Identifier: GPL-2.0-only
//! LPDDR4 and LPDDR4X VREF settings.
//!
//! MR12 VREF(CA) and MR14 VREF(DQ) share one encoding: OP[5:0] is the setting
//! (0..=50, higher values reserved), OP[6] the range, OP[7] RFU. In both the
//! LPDDR4 and the LPDDR4X tables range 1 setting 21 is the first one above
//! range 0's top (setting 50), so range 0 settings 0..=50 followed by range 1
//! settings 21..=50 is monotonic in VDDQ percentage, and range 1 setting s
//! sits at about range 0 setting s + 30. [`vref_axis_op`] and
//! [`vref_axis_index`] map that axis, so a sweep can walk one index.
//!
//! First written for the SpacemiT K1 own training (ore-edk-boot-opi
//! `train_eye.rs`, from the JEDEC tables only, no vendor logic).

pub mod cmd;
pub mod density;
pub mod mr;
pub mod timing;

/// MR number of VREF(CA).
pub const MR_VREF_CA: u8 = 12;
/// MR number of VREF(DQ).
pub const MR_VREF_DQ: u8 = 14;
/// Highest valid setting in either range.
pub const VREF_SETTING_MAX: u8 = 50;
/// Range 1 setting `s` is about range 0 setting `s + VREF_RANGE1_OFFSET`.
pub const VREF_RANGE1_OFFSET: usize = 30;
/// Points of the monotonic VREF axis: range 0 0..=50, range 1 21..=50.
pub const VREF_AXIS_LEN: usize = VREF_SETTING_MAX as usize + 1 + VREF_RANGE1_OFFSET;
/// OP[6]: range 1.
pub const VREF_RANGE1: u8 = 0x40;

/// MR12/MR14 OP byte of axis point `i` (`i < VREF_AXIS_LEN`).
pub const fn vref_axis_op(i: usize) -> u8 {
    if i <= VREF_SETTING_MAX as usize {
        i as u8
    } else {
        VREF_RANGE1 | (i - VREF_RANGE1_OFFSET) as u8
    }
}

/// Axis point of an MR12/MR14 OP byte; `None` for reserved settings or OP[7]
/// set. Range 1 settings below 21 map to their range 0 equivalent.
pub const fn vref_axis_index(op: u8) -> Option<usize> {
    let s = (op & 0x3f) as usize;
    if op & 0x80 != 0 || s > VREF_SETTING_MAX as usize {
        None
    } else if op & VREF_RANGE1 == 0 {
        Some(s)
    } else {
        Some(s + VREF_RANGE1_OFFSET)
    }
}

/// LPDDR4X VREF of an OP byte in 0.1 % of VDDQ (JEDEC LPDDR4X: range 0
/// 15.0..45.0 %, range 1 32.9..62.9 %, 0.6 % steps). For logs; reserved
/// settings are not rejected.
pub const fn lp4x_vref_permille(op: u8) -> u32 {
    let s = (op & 0x3f) as u32;
    if op & VREF_RANGE1 == 0 {
        150 + 6 * s
    } else {
        329 + 6 * s
    }
}

/// LPDDR4 VREF of an OP byte in 0.1 % of VDD2 (JEDEC LPDDR4: range 0
/// 10.0..30.0 %, range 1 22.0..42.0 %, 0.4 % steps). For logs.
pub const fn lp4_vref_permille(op: u8) -> u32 {
    let s = (op & 0x3f) as u32;
    if op & VREF_RANGE1 == 0 {
        100 + 4 * s
    } else {
        220 + 4 * s
    }
}

/// Chip-select mask addressing the first `ranks` ranks (at least rank 0):
/// 1 rank 0b01, 2 ranks 0b11. The command field it goes into is the
/// controller's.
pub const fn cs_mask(ranks: u32) -> u32 {
    if ranks >= 2 {
        3
    } else {
        1
    }
}
