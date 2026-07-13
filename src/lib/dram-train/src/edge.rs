// SPDX-License-Identifier: GPL-2.0-only
//! Transitions of a measured pass/fail curve.
//!
//! A sweep of `err.len()` points at delays `i * step` gives an error count
//! per point. Counts above half the maximum are "fail". [`decide`] reports
//! the edges and picks the first stable transition of the wanted polarity,
//! optionally preferring an independent hardware result that lies on one.
//!
//! First written for the A733 WCK2CK leveling (ore-edk-boot-radxa-a7a
//! `wl.rs`).

/// Edges reported in a [`Decision`].
pub const MAX_EDGES: usize = 8;
/// Edges considered when choosing (a longer curve's later edges are ignored).
pub const MAX_TRANSITIONS: usize = 64;

/// Which transition is the wanted one, walking up the delay axis.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Polarity {
    /// Fail then pass (e.g. the DRAM's sampled level becomes the expected one).
    FailToPass,
    /// Pass then fail.
    PassToFail,
}

/// The decision on one curve.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Decision {
    /// Fail threshold: counts above it are "fail" (half the maximum).
    pub threshold: u16,
    /// Points (index i: the change happens between i-1 and i) where pass/fail
    /// changes, and the fail state after each, first [`MAX_EDGES`].
    pub edges: [(u8, bool); MAX_EDGES],
    pub n_edges: u8,
    /// Chosen delay (`step` units times the point index, minus half a step),
    /// None without a usable transition.
    pub delay: Option<u16>,
    /// The hardware counter's delay lies on a measured transition.
    pub counter_agrees: bool,
}

impl Decision {
    pub const EMPTY: Decision = Decision {
        threshold: 0,
        edges: [(0, false); MAX_EDGES],
        n_edges: 0,
        delay: None,
        counter_agrees: false,
    };
}

/// The decision on a fail-to-pass curve with the default [`DecidePolicy`]
/// ([`DecidePolicy::DEFAULT`]): noise points and isolated flips ignored, a
/// clean curve required (periodic curves allowed), 4 stable points on each
/// side of the edge, and a hardware counter accepted only on such an edge.
/// [`decide_explain`] also says why there is no delay.
pub fn decide(err: &[u16], step: u16, counter: Option<u16>) -> Decision {
    decide_policy(
        err,
        step,
        counter,
        Polarity::FailToPass,
        &DecidePolicy::DEFAULT,
    )
}

/// The decision as moved from the A733 (`wl.rs`), before v1.0.0:
/// [`decide_with`] for a fail-to-pass transition with two stable points on
/// each side; the counter is accepted near any transition of the polarity.
pub fn decide_legacy(err: &[u16], step: u16, counter: Option<u16>) -> Decision {
    decide_with(err, step, counter, Polarity::FailToPass, 2)
}

/// Decide from error counts `err` (at most 256 points) measured at delays
/// `i * step`. A curve without both pass and fail points has no transition
/// and gives no delay. The first transition of `polarity` with at least
/// `stable` points of the same state on each side is taken (delay halfway
/// between its two points). `counter` is an independent hardware result: it
/// is chosen instead when within two steps of a measured transition of
/// `polarity`.
pub fn decide_with(
    err: &[u16],
    step: u16,
    counter: Option<u16>,
    polarity: Polarity,
    stable: usize,
) -> Decision {
    let mut d = Decision::EMPTY;
    let max = err.iter().copied().max().unwrap_or(0);
    let min = err.iter().copied().min().unwrap_or(0);
    if max == min {
        return d;
    }
    d.threshold = max / 2;
    let fail = |i: usize| err[i] > d.threshold;
    let wanted = |i: usize| match polarity {
        Polarity::FailToPass => !fail(i),
        Polarity::PassToFail => fail(i),
    };
    let mut all = [0usize; MAX_TRANSITIONS];
    let mut n_all = 0;
    for i in 1..err.len() {
        if fail(i) != fail(i - 1) {
            if (d.n_edges as usize) < MAX_EDGES {
                d.edges[d.n_edges as usize] = (i as u8, fail(i));
                d.n_edges += 1;
            }
            if n_all < all.len() {
                all[n_all] = i;
                n_all += 1;
            }
        }
    }
    let at = |i: usize| (i as u16 * step) - step / 2;
    if let Some(c) = counter {
        if all[..n_all]
            .iter()
            .any(|&i| wanted(i) && at(i).abs_diff(c) <= 2 * step)
        {
            d.counter_agrees = true;
            d.delay = Some(c);
            return d;
        }
    }
    for k in 0..n_all {
        let i = all[k];
        let prev = if k == 0 { 0 } else { all[k - 1] };
        let next = if k + 1 < n_all { all[k + 1] } else { err.len() };
        if wanted(i) && i >= prev + stable && next >= i + stable {
            d.delay = Some(at(i));
            break;
        }
    }
    d
}

/// Agreement of one quantity measured from several starting points (A733
/// WL m11: the PHY's TrainingCntr search run again from other start values;
/// the same question for any search that can be restarted).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Convergence {
    /// Runs that produced a value.
    pub results: u8,
    /// Runs without a value (failed or out of range).
    pub missing: u8,
    pub min: Option<u16>,
    pub max: Option<u16>,
    /// The agreed value (the lower median of the values) when every run
    /// produced one and `max - min <= tolerance`; None otherwise.
    pub agreed: Option<u16>,
}

/// Whether the values of several runs of the same search agree within
/// `tolerance`. Any missing run, or no run at all, means no agreement.
/// At most 255 runs are counted; more is a caller error (`None`).
pub fn converge(runs: &[Option<u16>], tolerance: u16) -> Option<Convergence> {
    if runs.len() > u8::MAX as usize {
        return None;
    }
    let mut c = Convergence {
        results: 0,
        missing: 0,
        min: None,
        max: None,
        agreed: None,
    };
    for r in runs {
        match *r {
            Some(v) => {
                c.results += 1;
                c.min = Some(c.min.map_or(v, |m| m.min(v)));
                c.max = Some(c.max.map_or(v, |m| m.max(v)));
            }
            None => c.missing += 1,
        }
    }
    if let (Some(lo), Some(hi)) = (c.min, c.max) {
        if c.missing == 0 && hi - lo <= tolerance {
            // Lower median without allocation: the value with at least half
            // of the runs at or below it.
            let need = runs.len().div_ceil(2);
            let mut best = hi;
            for v in runs.iter().flatten() {
                let at_or_below = runs.iter().flatten().filter(|w| *w <= v).count();
                if at_or_below >= need && *v < best {
                    best = *v;
                }
            }
            c.agreed = Some(best);
        }
    }
    Some(c)
}

// ---------------------------------------------------------------------------
// Sample voting and coarse-then-fine edge search. Older open prior art for
// both: LiteDRAM's read leveling (scan then centre on the window) and
// coreboot's Haswell raminit (coarse sweeps refined at the edges, majority
// votes); also AMD PG150 v1.4 p735, p752, p768 and PG353 v1.0 p202, p233,
// p247; UG586 v4.2 p130 (methods only): a
// point counts as pass or fail only when enough of its samples agree, a
// mixed point is noise ("X"), and an edge is bracketed with large steps
// before it is located with single steps.

/// The vote on one point.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Vote {
    Pass,
    Fail,
    /// Neither side reached `min_agree` samples.
    Noise,
}

/// Vote on a point measured `total` times with `passes` passing results:
/// pass when at least `min_agree` passed, fail when at least `min_agree`
/// failed, noise otherwise. `min_agree` = `total` requires all to agree.
/// The measurements must be independent launches (repeated measurements),
/// not compares inside one launch, which are correlated
/// ([`vote_launches`]).
pub const fn vote(passes: u16, total: u16, min_agree: u16) -> Vote {
    let fails = total.saturating_sub(passes);
    if passes >= min_agree && passes > fails {
        Vote::Pass
    } else if fails >= min_agree && fails > passes {
        Vote::Fail
    } else {
        Vote::Noise
    }
}

/// Pass and noise maps (bit i for point i, at most 128 points) of per-point
/// pass counts out of `total` samples each.
pub fn vote_maps(passes: &[u16], total: u16, min_agree: u16) -> (u128, u128) {
    let mut pass = 0u128;
    let mut noise = 0u128;
    for (i, &p) in passes.iter().enumerate().take(128) {
        match vote(p, total, min_agree) {
            Vote::Pass => pass |= 1 << i,
            Vote::Noise => noise |= 1 << i,
            Vote::Fail => {}
        }
    }
    (pass, noise)
}

/// Result of [`coarse_fine`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EdgeSearch {
    /// The first point (walking from `from` towards `to`) whose state differs
    /// from the state at `from`; None if the state never changes.
    pub edge: Option<u32>,
    /// The state at `from`.
    pub start_pass: bool,
    /// Probes made.
    pub probes: u32,
}

/// Find where a pass/fail state changes between `from` and `to` (either
/// direction, `to` inclusive): step `coarse` points at a time until the state
/// differs from the state at `from`, back off one coarse step, then step one
/// point at a time to the first changed point, and re-probe it `confirm`
/// times (each must agree, else the fine walk continues past it as noise).
/// `probe(i)` measures point i.
pub fn coarse_fine<E>(
    from: u32,
    to: u32,
    coarse: u32,
    confirm: u8,
    mut probe: impl FnMut(u32) -> Result<bool, E>,
) -> Result<EdgeSearch, E> {
    let up = to >= from;
    let span = from.abs_diff(to);
    let at = |k: u32| if up { from + k } else { from - k };
    let mut probes = 1;
    let start_pass = probe(from)?;
    let coarse = coarse.max(1);
    // Bracket: last coarse offset with the start state, first with the other.
    let mut k = 0;
    let mut bracket = None;
    while k < span {
        let next = (k + coarse).min(span);
        probes += 1;
        if probe(at(next))? != start_pass {
            bracket = Some((k, next));
            break;
        }
        k = next;
    }
    let (lo, hi) = match bracket {
        Some(b) => b,
        None => {
            return Ok(EdgeSearch {
                edge: None,
                start_pass,
                probes,
            })
        }
    };
    let mut j = lo + 1;
    while j <= hi {
        probes += 1;
        if probe(at(j))? != start_pass {
            let mut stable = true;
            for _ in 0..confirm {
                probes += 1;
                if probe(at(j))? == start_pass {
                    stable = false;
                    break;
                }
            }
            if stable {
                return Ok(EdgeSearch {
                    edge: Some(at(j)),
                    start_pass,
                    probes,
                });
            }
        }
        j += 1;
    }
    // The coarse point changed but no stable fine point did: take it.
    Ok(EdgeSearch {
        edge: Some(at(hi)),
        start_pass,
        probes,
    })
}

// ---------------------------------------------------------------------------
// Clean-curve decisions (the default since v1.0.0).
//
// Error counts per point are only votes if they come from independent
// launches: the A7A m11 data show that all compares inside one launch agree
// (whole-launch flips), so 32 compares of one launch are one vote. Sum
// several launches per point (or use [`vote_launches`]) before deciding.

/// How [`decide_policy`] reads a curve.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DecidePolicy {
    /// Points of the same state needed on each side of an edge, and the
    /// shortest run allowed inside the curve (shorter interior runs that are
    /// not isolated flips make the curve noisy).
    pub stable: usize,
    /// Interior runs of at most this many points between two runs of the
    /// other state are isolated flips: they count as noise and the runs
    /// around them join (0: none).
    pub flip: usize,
    /// A point whose error count lies strictly between `noise_percent` and
    /// 100 - `noise_percent` of the curve's maximum is noise ("X"); 0: no
    /// noise band (the half-maximum threshold only).
    pub noise_percent: u8,
    /// More noise points (band and flips) than this percentage of the curve
    /// make it noisy.
    pub max_noise_percent: u8,
    /// The error count of a point that fails every compare of every launch
    /// (compares x launches summed). The noise band is taken from it; None:
    /// from the curve's maximum, which cannot tell a point where one launch
    /// of three failed from one where all failed.
    pub full_scale: Option<u16>,
    /// Only edges within this many delay units of the counter (when there is
    /// one) are considered, and only that part of the curve has to be clean;
    /// None: the whole curve. Periodic curves (several periods of a phase
    /// sweep) are fine either way: any number of clean transitions is
    /// allowed.
    pub window: Option<u16>,
}

impl DecidePolicy {
    /// 4 points each side (AMD PG150 v1.4 p752 / UG586 v4.2 p130 ask for a
    /// stable run before an edge; method only), 1-2 point flips tolerated, a
    /// 25-75 % noise band, at most 10 % noise points, the whole curve.
    pub const DEFAULT: DecidePolicy = DecidePolicy {
        stable: 4,
        flip: 2,
        noise_percent: 25,
        max_noise_percent: 10,
        full_scale: None,
        window: None,
    };
}

impl Default for DecidePolicy {
    fn default() -> Self {
        DecidePolicy::DEFAULT
    }
}

/// Why [`decide_policy`] gave no delay, or how it chose one.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Verdict {
    /// An edge of the curve was taken.
    Edge,
    /// The counter, lying on a clean edge, was taken.
    Counter,
    /// Every point has the same state (apart from isolated flips): no
    /// response along the axis ([`crate::sanity::curve`]).
    Saturated(crate::sanity::Saturation),
    /// Too many noise points, or short runs that are not isolated flips.
    Noisy,
    /// The curve is clean but has no edge of the polarity with `stable`
    /// points on both sides (in the window).
    NoStableEdge,
}

impl Verdict {
    /// The per-stage error code of a verdict without a delay.
    pub const fn error(self) -> Option<crate::result::ErrorCode> {
        use crate::result::ErrorCode;
        match self {
            Verdict::Edge | Verdict::Counter => None,
            Verdict::Saturated(_) => Some(ErrorCode::Saturated),
            Verdict::Noisy | Verdict::NoStableEdge => Some(ErrorCode::NoTransition),
        }
    }
}

/// Vote over `launches` independent launches of one measurement: pass,
/// fail or noise with `min_agree` agreeing launches ([`vote`]).
pub fn vote_launches<E>(
    launches: u16,
    min_agree: u16,
    mut launch: impl FnMut() -> Result<bool, E>,
) -> Result<Vote, E> {
    let mut passes = 0;
    for _ in 0..launches {
        if launch()? {
            passes += 1;
        }
    }
    Ok(vote(passes, launches, min_agree))
}

/// Decide with a [`DecidePolicy`]; [`decide_explain`] without the verdict.
pub fn decide_policy(
    err: &[u16],
    step: u16,
    counter: Option<u16>,
    polarity: Polarity,
    p: &DecidePolicy,
) -> Decision {
    decide_explain(err, step, counter, polarity, p).0
}

/// Decide with a [`DecidePolicy`] and say why. Points are pass, fail or
/// noise (the noise band); interior runs of at most `flip` points between
/// two runs of the other state are noise too. The curve (or the window
/// around the counter) is clean when noise stays under `max_noise_percent`
/// and every other interior run has at least `stable` points; any number of
/// such transitions is allowed (periodic sweeps). An edge of `polarity` lies
/// between the last point of one run and the first of the next, at
/// `(last + first) * step / 2` (the legacy half-step point when adjacent).
/// `counter` is taken when within two steps of a clean edge; otherwise the
/// first edge (the nearest to the counter when there is one). `edges`,
/// `n_edges` and `threshold` are filled as by [`decide_with`].
pub fn decide_explain(
    err: &[u16],
    step: u16,
    counter: Option<u16>,
    polarity: Polarity,
    p: &DecidePolicy,
) -> (Decision, Verdict) {
    let mut d = decide_with(err, step, None, polarity, err.len() + 1);
    d.delay = None;
    d.counter_agrees = false;
    if let Err(crate::sanity::SanityError::Saturated(how)) = crate::sanity::curve(err) {
        return (d, Verdict::Saturated(how));
    }
    let max = p
        .full_scale
        .unwrap_or_else(|| err.iter().copied().max().unwrap_or(0));
    // The points to read: all, or the window around the counter.
    let (from, to) = match (p.window, counter) {
        (Some(w), Some(c)) => {
            let s = step.max(1) as u32;
            let lo = (c.saturating_sub(w) as u32 / s) as usize;
            let hi = ((c as u32 + w as u32) / s + 1) as usize;
            (lo.min(err.len()), hi.min(err.len()))
        }
        _ => (0, err.len()),
    };
    let lo_band = (max as u32 * p.noise_percent as u32 / 100) as u16;
    let hi_band = max - lo_band;
    let state = |i: usize| -> Option<bool> {
        let e = err[i];
        if p.noise_percent == 0 {
            Some(e > max / 2)
        } else if e <= lo_band {
            Some(false)
        } else if e >= hi_band {
            Some(true)
        } else {
            None
        }
    };
    // Runs of definite points: (fail, first, last, count).
    const R: usize = 2 * MAX_TRANSITIONS;
    let mut runs = [(false, 0usize, 0usize, 0usize); R];
    let mut n = 0;
    let mut noise = 0usize;
    for i in from..to {
        let Some(s) = state(i) else {
            noise += 1;
            continue;
        };
        if n > 0 && runs[n - 1].0 == s {
            runs[n - 1].2 = i;
            runs[n - 1].3 += 1;
        } else {
            if n == R {
                return (d, Verdict::Noisy);
            }
            runs[n] = (s, i, i, 1);
            n += 1;
        }
    }
    // Isolated flips: short interior runs between two runs of the other
    // state join their neighbours.
    let mut k = 1;
    while n >= 3 && k + 1 < n {
        if runs[k].3 <= p.flip && runs[k - 1].0 == runs[k + 1].0 {
            noise += runs[k].3;
            runs[k - 1].2 = runs[k + 1].2;
            runs[k - 1].3 += runs[k + 1].3;
            for j in k..n - 2 {
                runs[j] = runs[j + 2];
            }
            n -= 2;
        } else {
            k += 1;
        }
    }
    let span = to - from;
    if span == 0 || noise * 100 > span * p.max_noise_percent as usize {
        return (d, Verdict::Noisy);
    }
    if n < 2 {
        let how = match runs[..n].first() {
            Some(r) if r.0 => crate::sanity::Saturation::AllFail,
            _ => crate::sanity::Saturation::AllPass,
        };
        return (d, Verdict::Saturated(how));
    }
    if runs[1..n - 1].iter().any(|r| r.3 < p.stable) {
        return (d, Verdict::Noisy);
    }
    let wanted_after = matches!(polarity, Polarity::FailToPass);
    let at = |last: usize, first: usize| ((last + first) as u32 * step as u32 / 2) as u16;
    let mut chosen: Option<u16> = None;
    for k in 1..n {
        let (before, after) = (runs[k - 1], runs[k]);
        if after.0 == !wanted_after && before.3 >= p.stable && after.3 >= p.stable {
            let edge = at(before.2, after.1);
            if let Some(c) = counter {
                if edge.abs_diff(c) <= 2 * step {
                    d.counter_agrees = true;
                    d.delay = Some(c);
                    return (d, Verdict::Counter);
                }
            }
            let better = match (chosen, counter) {
                (None, _) => true,
                (Some(e), Some(c)) => edge.abs_diff(c) < e.abs_diff(c),
                (Some(_), None) => false,
            };
            if better {
                chosen = Some(edge);
            }
        }
    }
    d.delay = chosen;
    let v = if chosen.is_some() {
        Verdict::Edge
    } else {
        Verdict::NoStableEdge
    };
    (d, v)
}
