// SPDX-License-Identifier: GPL-2.0-only
//! Knob-response checks, run before a stage takes a decision: does the axis
//! move the observable at all, is part of the axis a copy of another part,
//! is the result saturated, and is the hardware in the mode the stage
//! assumes. A failed check is an error with no decision.
//!
//! Background: the A7A m11 WCK2CK sweep showed no delay dependence (a mode
//! register stayed at its training value), and the R2S MR14 range bit made
//! axis points 51-80 a repeat of 21-50. Both passed host tests against
//! cooperative models.

use crate::result::ErrorCode;

/// What a check found.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SanityError {
    /// The two ends of the axis give the same failing (or closed) outcome.
    DeadKnob,
    /// Points `b..b + len` repeat points `a..a + len` (within one step).
    AliasedAxis { a: usize, b: usize, len: usize },
    /// All pass, all fail, or a result at the limit of its range.
    Saturated(Saturation),
    /// A caller-supplied mode or state check failed.
    ModeMismatch {
        what: &'static str,
        got: u32,
        want: u32,
    },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Saturation {
    AllPass,
    AllFail,
    AtLimit,
}

impl SanityError {
    /// The per-stage error code ([`crate::result::ErrorCode`]).
    pub const fn code(self) -> ErrorCode {
        match self {
            SanityError::DeadKnob => ErrorCode::DeadKnob,
            SanityError::AliasedAxis { .. } => ErrorCode::AliasedAxis,
            SanityError::Saturated(_) => ErrorCode::Saturated,
            SanityError::ModeMismatch { .. } => ErrorCode::ModeMismatch,
        }
    }
}

/// Two-point test of a pass/fail knob: `lo` and `hi` are the outcomes of
/// repeated independent launches at two settings that must differ (the
/// range ends; for a phase knob, settings half a period apart). Dead when
/// both ends give the same fail fraction and are not all passing.
pub fn two_point(lo: &[bool], hi: &[bool]) -> Result<(), SanityError> {
    let pass = |s: &[bool]| s.iter().filter(|&&p| p).count();
    if lo.is_empty() || hi.is_empty() {
        return Ok(());
    }
    let (pl, ph) = (pass(lo), pass(hi));
    let all_pass = pl == lo.len() && ph == hi.len();
    // Same pass fraction (cross-multiplied).
    if !all_pass && pl * hi.len() == ph * lo.len() {
        return Err(SanityError::DeadKnob);
    }
    Ok(())
}

/// An error-count curve that never changes is saturated: all pass (all
/// zero) or all fail.
pub fn curve(err: &[u16]) -> Result<(), SanityError> {
    let (Some(&max), Some(&min)) = (err.iter().max(), err.iter().min()) else {
        return Ok(());
    };
    if max == min {
        return Err(SanityError::Saturated(if max == 0 {
            Saturation::AllPass
        } else {
            Saturation::AllFail
        }));
    }
    Ok(())
}

/// A result at either limit of its range (`0` or `max`, the sunxi rule) is
/// a failure, not a value.
pub const fn limits(value: u32, min: u32, max: u32) -> Result<(), SanityError> {
    if value <= min || value >= max {
        Err(SanityError::Saturated(Saturation::AtLimit))
    } else {
        Ok(())
    }
}

/// A width profile measured over the whole axis that is the same at every
/// point (at least `min_points` of them) did not respond to the knob.
pub fn flat_profile(profile: &[Option<u8>], min_points: usize) -> Result<(), SanityError> {
    if profile.len() < min_points.max(2) || profile.iter().any(|w| w.is_none()) {
        return Ok(());
    }
    let first = profile[0];
    if profile.iter().all(|w| *w == first) {
        if first == Some(0) {
            return Err(SanityError::Saturated(Saturation::AllFail));
        }
        return Err(SanityError::DeadKnob);
    }
    Ok(())
}

fn shaped(seg: &[Option<u8>]) -> bool {
    let vals = seg.iter().flatten();
    match (vals.clone().max(), vals.min()) {
        (Some(a), Some(b)) => a - b >= 2,
        _ => false,
    }
}

/// Whether `b..b + len` repeats `a..a + len`: every point within one step,
/// at least 3 in 4 identical, and the segment has shape (a spread of at
/// least two steps; flat stretches prove nothing).
pub fn segments_alias(profile: &[Option<u8>], a: usize, b: usize, len: usize) -> bool {
    if len == 0 || a + len > profile.len() || b + len > profile.len() || a.max(b) < a.min(b) + len {
        return false;
    }
    let (sa, sb) = (&profile[a..a + len], &profile[b..b + len]);
    if !shaped(sa) || !shaped(sb) {
        return false;
    }
    let mut same = 0;
    for (x, y) in sa.iter().zip(sb) {
        match (x, y) {
            (Some(x), Some(y)) if x.abs_diff(*y) <= 1 => same += (x == y) as usize,
            _ => return false,
        }
    }
    same * 4 >= len * 3
}

/// Look for an aliased stretch: two disjoint segments of at least
/// `min_len` points where one repeats the other ([`segments_alias`]). The
/// longest such pair is reported.
pub fn alias(profile: &[Option<u8>], min_len: usize) -> Result<(), SanityError> {
    let n = profile.len();
    let min_len = min_len.max(4);
    let mut best: Option<(usize, usize, usize)> = None;
    for shift in min_len..n {
        // Longest stretch at this shift where points match within one step.
        let mut start = 0;
        while start + shift < n {
            let mut end = start;
            while end + shift < n
                && matches!((profile[end], profile[end + shift]), (Some(x), Some(y)) if x.abs_diff(y) <= 1)
            {
                end += 1;
            }
            let len = (end - start).min(shift);
            if len >= min_len
                && segments_alias(profile, start, start + shift, len)
                && best.is_none_or(|b| len > b.2)
            {
                best = Some((start, start + shift, len));
            }
            start = end.max(start + 1);
        }
    }
    match best {
        Some((a, b, len)) => Err(SanityError::AliasedAxis { a, b, len }),
        None => Ok(()),
    }
}

/// A caller-supplied mode check: `got & mask` must be `want`.
pub const fn mode(what: &'static str, got: u32, mask: u32, want: u32) -> Result<(), SanityError> {
    if got & mask == want {
        Ok(())
    } else {
        Err(SanityError::ModeMismatch {
            what,
            got: got & mask,
            want,
        })
    }
}
