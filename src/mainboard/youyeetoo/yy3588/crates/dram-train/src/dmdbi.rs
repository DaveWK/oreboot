// SPDX-License-Identifier: GPL-2.0-only
//! Data mask (DM) and data bus inversion (DBI) stages.
//!
//! - DM check: fill with one byte, then write a second byte with DM
//!   asserted on alternate beats (the rising-strobe beats); the masked
//!   beats must keep the first byte, the others take the second.
//!   [`dm_classify`] tells a working mask from an ignored, inverted or
//!   always-asserted one and from a data error.
//! - DBI: the DC inversion rule ([`dbi_encode`]: a byte with more ones than
//!   allowed goes out inverted with DBI asserted), the calibration patterns
//!   that keep DQ still while only DBI toggles ([`DBI_TOGGLE`]: alternate
//!   all-ones and all-zeros beats) so the DBI lane can be deskewed like a
//!   DQ bit (it is bit [`DBI_BIT`] of a lane group in
//!   [`crate::deskew::per_bit`]), and the combined DQ+DBI window the final
//!   strobe position must sit in ([`combined_window`]).
//!
//! Method: AMD PG313 v1.1 p80-81 and p345-357 (DM check with two patterns,
//! DBI deskew with its own patterns, read DBI, the combined DQ+DBI window)
//! and PG456 v1.0 p159 (DBI calibrated alongside DQ because it moves the
//! strobe); cited, not copied. The DC inversion threshold is the LPDDR4/4X
//! and LPDDR5 rule (more than four ones in a byte); DDR4 inverts on zeros
//! ([`DbiRule::Zeros`]).

/// The first and second DM fill bytes.
pub const DM_FIRST: u8 = 0x55;
pub const DM_SECOND: u8 = 0xbb;
/// The DBI lane's index next to the 8 DQ bits of a byte.
pub const DBI_BIT: u8 = 8;
/// Logical bytes per beat that make DBI toggle every beat while the DQ
/// wires stay at one level (under either rule).
pub const DBI_TOGGLE: [u8; 2] = [0xff, 0x00];

/// Whether beat `t` is written with DM asserted (the rising-strobe beats).
pub const fn dm_masked(t: usize) -> bool {
    t & 1 == 0
}

/// The byte beat `t` must read back after the DM check writes.
pub const fn dm_expected(t: usize) -> u8 {
    if dm_masked(t) {
        DM_FIRST
    } else {
        DM_SECOND
    }
}

/// What a byte lane's DM check showed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Dm {
    /// Masked beats kept the first byte, the others took the second.
    Ok,
    /// Every beat took the second byte: DM was not applied.
    Ignored,
    /// The mask hit the other beats: polarity or strobe edge swapped.
    Inverted,
    /// Every beat kept the first byte: DM stuck asserted, or the second
    /// write was lost.
    AllMasked,
    /// Data that is neither byte on some beat: a data-path error, not DM.
    Data,
}

/// Classify each byte lane of a read-back (`bytes` lanes, `beats` beats,
/// beat-major) into `out` (one entry per lane). Returns false if a buffer
/// is too small.
pub fn dm_classify(got: &[u8], bytes: usize, beats: usize, out: &mut [Dm]) -> bool {
    if got.len() < bytes * beats || out.len() < bytes || beats < 2 {
        return false;
    }
    for (b, o) in out.iter_mut().enumerate().take(bytes) {
        let lane = |t: usize| got[t * bytes + b];
        let all = |f: &dyn Fn(usize) -> u8| (0..beats).all(|t| lane(t) == f(t));
        *o = if all(&dm_expected) {
            Dm::Ok
        } else if all(&|_| DM_SECOND) {
            Dm::Ignored
        } else if all(&|t| dm_expected(t + 1)) {
            Dm::Inverted
        } else if all(&|_| DM_FIRST) {
            Dm::AllMasked
        } else {
            Dm::Data
        };
    }
    true
}

/// Which bit value the DC inversion keeps scarce.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DbiRule {
    /// LPDDR4/4X, LPDDR5: invert when more than four bits are 1.
    Ones,
    /// DDR4: invert when more than four bits are 0.
    Zeros,
}

/// The wire byte and DBI level for a logical byte.
pub const fn dbi_encode(byte: u8, rule: DbiRule) -> (u8, bool) {
    let n = match rule {
        DbiRule::Ones => byte.count_ones(),
        DbiRule::Zeros => byte.count_zeros(),
    };
    if n > 4 {
        (!byte, true)
    } else {
        (byte, false)
    }
}

/// The logical byte from a wire byte and its DBI level.
pub const fn dbi_decode(wire: u8, dbi: bool) -> u8 {
    if dbi {
        !wire
    } else {
        wire
    }
}

/// The wire image of logical bytes: `wire[i]` and bit i of the returned
/// DBI levels (up to 64 bytes, one byte lane; longer input is cut).
pub fn wire_image(logical: &[u8], wire: &mut [u8], rule: DbiRule) -> u64 {
    let mut dbi = 0u64;
    for (i, (l, w)) in logical.iter().zip(wire.iter_mut()).take(64).enumerate() {
        let (x, d) = dbi_encode(*l, rule);
        *w = x;
        dbi |= u64::from(d) << i;
    }
    dbi
}

/// The strobe window shared by the DQ bits and DBI (inclusive delay
/// ranges): their intersection and its centre, or None if they do not
/// overlap.
pub fn combined_window(dq: (i32, i32), dbi: (i32, i32)) -> Option<(i32, i32, i32)> {
    let lo = dq.0.max(dbi.0);
    let hi = dq.1.min(dbi.1);
    (lo <= hi).then_some((lo, hi, lo + (hi - lo) / 2))
}
