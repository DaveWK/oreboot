// SPDX-License-Identifier: GPL-2.0-only
//! Complex-pattern stages: a victim/aggressor pattern in which each DQ of a
//! byte takes a turn as the victim, and a short-then-long search (coarse
//! steps with the short pattern, single steps with the long one) for the
//! read and write complex stages.
//!
//! Method: AMD PG150 v1.4 p862 (each DQ lane of a byte is the victim in
//! turn; a long run of victim/aggressor bursts), PG313 v1.1 p357-359 (read
//! and write complex stages start from the simple-stage result, try a short
//! burst before the long one), AM019 `RDCMPLX_*_SHORT`, `*_SHORT_FCRSE`
//! (10-step coarse with the short pattern) and `WRCMPLX_*` (cited, not
//! copied). AMD does not publish the pattern contents; the segments below
//! are ours.
//!
//! # Pattern
//!
//! For victim lane `v`, one block of beats; every other lane of the byte is
//! an aggressor and carries the aggressor bit. Blocks run for `v = 0, 1,
//! ..., lanes - 1`. Segments of 16 beats (multiples of BL8, BL16 and, per
//! block, BL32):
//!
//! | Segment | Victim | Aggressors | Stresses |
//! |---|---|---|---|
//! | odd mode | toggles | toggle opposite | crosstalk against the victim edge |
//! | lone one | a single 1 in 0s | toggle | inter-symbol interference, crosstalk |
//! | even mode | toggles | toggle with it | simultaneous switching |
//! | lone zero | a single 0 in 1s | toggle opposite | the other polarity |
//! | low frequency | 8 ones, 8 zeros | toggle | long runs on the victim |
//! | quiet victim | 0 | toggle | crosstalk onto a still victim |
//! | PRBS7 | PRBS7 (127 + 1 beats) | its complement | data-dependent ISI |
//!
//! The short pattern is the first two segments (32 beats per victim); the
//! long one is all of them (224 beats per victim, a multiple of 32).

/// Most lanes in a byte group (8 DQ plus DBI/DM, or 16 for x16 groups).
pub const MAX_LANES: u8 = 16;
/// Beats per segment.
pub const SEGMENT: usize = 16;
/// Beats per victim block of the short pattern.
pub const SHORT_BEATS: usize = 2 * SEGMENT;
/// Beats per victim block of the long pattern.
pub const LONG_BEATS: usize = 6 * SEGMENT + 128;

/// Which pattern.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Length {
    Short,
    Long,
}

/// PRBS7 (x^7 + x^6 + 1) from the all-ones seed, one period (bit t = beat
/// t), computed at build time.
const PRBS7: u128 = {
    let mut s: u8 = 0x7f;
    let mut bits: u128 = 0;
    let mut t = 0;
    while t < 127 {
        if s & 1 != 0 {
            bits |= 1 << t;
        }
        let b = ((s >> 6) ^ (s >> 5)) & 1;
        s = ((s << 1) | b) & 0x7f;
        t += 1;
    }
    bits
};

/// Bit `t` of the PRBS7 sequence; period 127.
pub const fn prbs7(t: usize) -> bool {
    PRBS7 >> (t % 127) & 1 != 0
}

/// The victim/aggressor pattern of one byte group.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Complex {
    pub lanes: u8,
    pub length: Length,
}

impl Complex {
    /// A pattern for `lanes` lanes (1..=[`MAX_LANES`]).
    pub fn new(lanes: u8, length: Length) -> Option<Self> {
        (1..=MAX_LANES)
            .contains(&lanes)
            .then_some(Self { lanes, length })
    }

    /// Beats per victim block.
    pub const fn block(&self) -> usize {
        match self.length {
            Length::Short => SHORT_BEATS,
            Length::Long => LONG_BEATS,
        }
    }

    /// Beats in the whole pattern (one block per victim lane).
    pub const fn len(&self) -> usize {
        self.block() * self.lanes as usize
    }

    pub const fn is_empty(&self) -> bool {
        false
    }

    /// The victim lane of beat `i`.
    pub fn victim(&self, i: usize) -> u8 {
        ((i % self.len()) / self.block()) as u8
    }

    /// (victim bit, aggressor bit) at beat `t` of a block.
    fn bits(t: usize) -> (bool, bool) {
        let seg = t / SEGMENT;
        let k = t % SEGMENT;
        let tog = k % 2 == 1;
        match seg {
            0 => (tog, !tog),
            1 => (k == 7, tog),
            2 => (tog, tog),
            3 => (k != 7, !tog),
            4 => (k < 8, tog),
            5 => (false, tog),
            _ => {
                let p = t - 6 * SEGMENT;
                let v = if p < 127 { prbs7(p) } else { false };
                (v, !v)
            }
        }
    }

    /// Lane bits of beat `i` (bit n = lane n).
    pub fn beat(&self, i: usize) -> u16 {
        let i = i % self.len();
        let v = self.victim(i);
        let (vb, ab) = Self::bits(i % self.block());
        let all = if self.lanes == 16 {
            0xffff
        } else {
            (1u16 << self.lanes) - 1
        };
        let mut word = if ab { all & !(1 << v) } else { 0 };
        if vb {
            word |= 1 << v;
        }
        word
    }

    /// Fill a bus image: `out` holds beats of `bytes` byte lanes each (one
    /// byte per lane, beat-major, as a burst lands on a `bytes`-wide bus).
    /// Byte lane `b` runs the pattern `b` blocks ahead, so at any beat the
    /// byte lanes have different victims (no lane is idle while another is
    /// tested). Only the low 8 lanes are used (`lanes` <= 8). Returns the
    /// bytes written (whole beats).
    pub fn fill_bus(&self, out: &mut [u8], bytes: usize, start_beat: usize) -> usize {
        if bytes == 0 || self.lanes > 8 {
            return 0;
        }
        let beats = out.len() / bytes;
        for t in 0..beats {
            for b in 0..bytes {
                let i = start_beat + t + b * self.block();
                out[t * bytes + b] = self.beat(i) as u8;
            }
        }
        beats * bytes
    }
}

/// Why a complex search failed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ComplexError<E> {
    /// The probe failed (hardware or timeout).
    Probe(E),
    /// The long pattern fails at the start point (the simple-stage centre):
    /// there is no long-pattern window around it.
    StartFails,
}

impl<E> From<E> for ComplexError<E> {
    fn from(e: E) -> Self {
        Self::Probe(e)
    }
}

/// One side of a complex window.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ComplexEdge {
    /// The outermost point (from the start) at which the long pattern
    /// passes; the start itself if nothing beyond it passes.
    pub last_pass: u32,
    /// The first point where the short pattern failed on the coarse walk;
    /// None if it passed up to the limit.
    pub short_fail: Option<u32>,
    pub probes_short: u32,
    pub probes_long: u32,
}

/// A complex window: both edges and the centre.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ComplexWindow {
    pub left: ComplexEdge,
    pub right: ComplexEdge,
    /// Midpoint of the two long-pattern pass edges (rounded down).
    pub center: u32,
    /// Long-pattern pass points, both edges included.
    pub width: u32,
}

/// Probe `point` with the long pattern; a fail counts only if `confirm`
/// re-probes all fail too.
fn confirmed_fail<E>(
    probe: &mut impl FnMut(u32, Length) -> Result<bool, E>,
    point: u32,
    confirm: u8,
    probes: &mut u32,
) -> Result<bool, E> {
    for _ in 0..=confirm {
        *probes += 1;
        if probe(point, Length::Long)? {
            return Ok(false);
        }
    }
    Ok(true)
}

/// One edge from `start` towards `limit` (either direction, inclusive):
/// step `coarse` points with the short pattern until it fails, then with
/// the long pattern from the last short pass: outward one point at a time
/// while the long pattern passes; or, when it fails there (the long pattern
/// is stricter, its edge can lie inside the last coarse pass), coarse steps
/// inward to a long pass and single steps outward from it. A
/// long-pattern fail is re-probed `confirm` times and only counts if all
/// agree. `probe(point, length)` measures one point.
pub fn complex_edge<E>(
    start: u32,
    limit: u32,
    coarse: u32,
    confirm: u8,
    probe: &mut impl FnMut(u32, Length) -> Result<bool, E>,
) -> Result<ComplexEdge, E> {
    let up = limit >= start;
    let span = start.abs_diff(limit);
    let at = |k: u32| if up { start + k } else { start - k };
    let coarse = coarse.max(1);
    let (mut ps, mut pl) = (0u32, 0u32);

    // Coarse walk with the short pattern.
    let mut k = 0;
    let mut short_fail = None;
    while k < span {
        let next = (k + coarse).min(span);
        ps += 1;
        if !probe(at(next), Length::Short)? {
            short_fail = Some(next);
            break;
        }
        k = next;
    }
    let hi = short_fail.unwrap_or(span);

    // Long pattern from the last short pass.
    let mut last = k;
    if k > 0 && confirmed_fail(probe, at(k), confirm, &mut pl)? {
        // The long edge lies inside: coarse steps inward with the long
        // pattern to a pass (single probes; the start passes, the caller
        // checked it), then single steps outward to the first confirmed
        // long fail.
        let mut j = k;
        loop {
            j = j.saturating_sub(coarse);
            if j == 0 {
                break;
            }
            if !confirmed_fail(probe, at(j), 0, &mut pl)? {
                break;
            }
        }
        last = j;
        let mut x = j + 1;
        while x <= k {
            if confirmed_fail(probe, at(x), confirm, &mut pl)? {
                break;
            }
            last = x;
            x += 1;
        }
    } else {
        // Outward while the long pattern passes, up to the short fail.
        let mut j = k + 1;
        while j <= hi {
            if short_fail == Some(j) || confirmed_fail(probe, at(j), confirm, &mut pl)? {
                break; // the short pattern already failed here, or the long
            }
            last = j;
            j += 1;
        }
    }
    Ok(ComplexEdge {
        last_pass: at(last),
        short_fail: short_fail.map(at),
        probes_short: ps,
        probes_long: pl,
    })
}

/// The complex window around `start` (normally the simple-stage centre),
/// searching down to `lo` and up to `hi`: the start must pass the long
/// pattern; then [`complex_edge`] on each side; the centre is the midpoint
/// of the long-pattern edges.
pub fn complex_window<E>(
    start: u32,
    lo: u32,
    hi: u32,
    coarse: u32,
    confirm: u8,
    mut probe: impl FnMut(u32, Length) -> Result<bool, E>,
) -> Result<ComplexWindow, ComplexError<E>> {
    let lo = lo.min(start);
    let hi = hi.max(start);
    if !probe(start, Length::Long)? {
        return Err(ComplexError::StartFails);
    }
    let left = complex_edge(start, lo, coarse, confirm, &mut probe)?;
    let right = complex_edge(start, hi, coarse, confirm, &mut probe)?;
    let (l, r) = (left.last_pass, right.last_pass);
    Ok(ComplexWindow {
        left,
        right,
        center: l + (r - l) / 2,
        width: r - l + 1,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn prbs7_has_period_127_and_64_ones() {
        let ones = (0..127).filter(|&t| prbs7(t)).count();
        assert_eq!(ones, 64);
        for t in 0..127 {
            assert_eq!(prbs7(t), prbs7(t + 127));
        }
        // Longest run of ones is 7 (the seed), of zeros 6.
        let mut run = (0, 0, false);
        let mut max = [0usize; 2];
        for t in 0..254 {
            let b = prbs7(t);
            run = if b == run.2 {
                (run.0 + 1, 0, b)
            } else {
                (1, 0, b)
            };
            max[b as usize] = max[b as usize].max(run.0);
        }
        assert_eq!(max, [6, 7]);
    }
}
