// SPDX-License-Identifier: GPL-2.0-only
//! Multi-rank merge, plausibility checks on engine results, and a bounded
//! retrain loop.
//!
//! - [`merge`]: one setting shared by several ranks (or phases, or byte
//!   lanes behind one delay) is the midpoint of the min/max range of their
//!   trained values, not the mean: the mean leans towards the ranks that
//!   happen to agree and can leave the outlier rank with less margin than
//!   the midpoint gives every rank. The spread is checked against a skew
//!   limit.
//! - [`plausible`]: a training engine's per-lane results are checked
//!   before use: values at or near a delay-line end (saturated), values
//!   near zero where zero is not a valid answer, and lanes far from the
//!   others (outliers, against the median). [`monotonic`] checks values
//!   that must rise (or fall) with an index, such as one delay across
//!   frequency points.
//! - [`retrain`]: run a training step again when its result is not
//!   plausible, a bounded number of times, rotating the start phase on each
//!   try so a retry does not repeat the same start.
//!
//! Method: AMD PG150 v1.4 p56 and p879-880 (ranks merged by the midpoint
//! of the min/max range; a skew limit), Microchip PolarFire `mss_ddr.c`
//! (MIT) around line 1586 (retrain with a rotated start phase, bounded);
//! cited, not copied.

/// The merged value and the spread of the inputs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Merged {
    pub value: i32,
    pub min: i32,
    pub max: i32,
}

impl Merged {
    pub const fn spread(&self) -> i32 {
        self.max - self.min
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MergeError {
    Empty,
    /// The values are further apart than the skew limit.
    Skew {
        min: i32,
        max: i32,
    },
}

/// The midpoint of the min/max range of `values`; `max_skew` bounds the
/// spread.
pub fn merge(values: &[i32], max_skew: i32) -> Result<Merged, MergeError> {
    let min = *values.iter().min().ok_or(MergeError::Empty)?;
    let max = *values.iter().max().ok_or(MergeError::Empty)?;
    if max - min > max_skew {
        return Err(MergeError::Skew { min, max });
    }
    Ok(Merged {
        value: min + (max - min) / 2,
        min,
        max,
    })
}

/// Limits a lane value is checked against.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Limits {
    /// The delay line's range.
    pub lo: i32,
    pub hi: i32,
    /// Within this many steps of an end counts as saturated.
    pub near_end: i32,
    /// Values with |v| at most this are suspect (0: off).
    pub near_zero: i32,
    /// More than this from the median is an outlier.
    pub outlier: i32,
}

/// Lane masks (bit n = lane n) of the checks that failed.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Plausibility {
    pub saturated: u64,
    pub near_zero: u64,
    pub outliers: u64,
    pub median: i32,
}

impl Plausibility {
    pub const fn ok(&self) -> bool {
        self.saturated == 0 && self.near_zero == 0 && self.outliers == 0
    }
    pub const fn bad(&self) -> u64 {
        self.saturated | self.near_zero | self.outliers
    }
}

/// Check up to 64 lane values (more are ignored).
pub fn plausible(values: &[i32], l: &Limits) -> Plausibility {
    let v = &values[..values.len().min(64)];
    let mut p = Plausibility::default();
    if v.is_empty() {
        return p;
    }
    let mut sorted = [0i32; 64];
    sorted[..v.len()].copy_from_slice(v);
    sorted[..v.len()].sort_unstable();
    let n = v.len();
    p.median = if n % 2 == 1 {
        sorted[n / 2]
    } else {
        sorted[n / 2 - 1] + (sorted[n / 2] - sorted[n / 2 - 1]) / 2
    };
    for (i, &x) in v.iter().enumerate() {
        if x <= l.lo + l.near_end || x >= l.hi - l.near_end {
            p.saturated |= 1 << i;
        }
        if l.near_zero > 0 && x.abs() <= l.near_zero {
            p.near_zero |= 1 << i;
        }
        if (x - p.median).abs() > l.outlier {
            p.outliers |= 1 << i;
        }
    }
    p
}

/// The first index where `values` stops rising (`rising`) or falling, or
/// None if it is monotonic (equal neighbours allowed).
pub fn monotonic(values: &[i32], rising: bool) -> Option<usize> {
    values
        .windows(2)
        .position(|w| if rising { w[1] < w[0] } else { w[1] > w[0] })
        .map(|i| i + 1)
}

/// Why [`retrain`] gave up.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RetrainError<E, T> {
    /// The step failed on the last try.
    Step(E),
    /// Every try gave an implausible result; the last one.
    Implausible(T),
    /// No phases or no tries.
    Config,
}

/// Run `step(phase)` up to `tries` times, the start phase rotating through
/// `phases` (try k uses `phases[k % len]`), until `ok(&result)`. Returns
/// the result and the try index that gave it.
pub fn retrain<T, E>(
    tries: u32,
    phases: &[u32],
    mut step: impl FnMut(u32) -> Result<T, E>,
    ok: impl Fn(&T) -> bool,
) -> Result<(T, u32), RetrainError<E, T>> {
    if tries == 0 || phases.is_empty() {
        return Err(RetrainError::Config);
    }
    let mut last = None;
    for k in 0..tries {
        match step(phases[k as usize % phases.len()]) {
            Ok(r) if ok(&r) => return Ok((r, k)),
            Ok(r) => last = Some(Ok(r)),
            Err(e) => last = Some(Err(e)),
        }
    }
    match last {
        Some(Ok(r)) => Err(RetrainError::Implausible(r)),
        Some(Err(e)) => Err(RetrainError::Step(e)),
        None => Err(RetrainError::Config),
    }
}
