// SPDX-License-Identifier: GPL-2.0-only
//! One axis of a Vref x delay eye: sweep the axis (a Vref code, an MR14
//! VREF(DQ) setting...), at each point run the delay search on every rank
//! and keep the worst lane window, centre the axis at half height of that
//! profile ([`crate::eye::center`]), set the centre, re-run every rank for
//! the final per-lane windows and require a minimum margin.
//!
//! The delay search is the board's (a PHY training engine, a software
//! sweep...): it reports one window per lane for one rank at the current
//! axis point. First written as the K1's own LPDDR4X training
//! (ore-edk-boot-opi `train_own.rs` `train_fp`, read and write eye steps);
//! nothing here comes from the translated vendor training.
//!
//! Live-readback contract ([`crate::live`], Oct 15): after every
//! [`Axis::set`] the sweep calls [`Axis::verify_live`]. The engine never
//! runs at a point the axis did not confirm live; a stale point is
//! [`SweepError::NotLive`]. An axis without a live readback declares it
//! (the default), and [`Outcome::live`] says so in the report.

use crate::eye::{walk_out, worst, Eye};
pub use crate::live::{Live, Liveness};

/// An axis and the delay search measured along it, for `L` lanes.
pub trait Axis<const L: usize> {
    type Error;
    /// Ranks to measure at every point.
    fn ranks(&self) -> u32;
    /// Move to axis point `i` (program the Vref).
    fn set(&mut self, i: usize);
    /// Called right after every `set(i)`: read the live copy of the axis
    /// knob (the Vref the receivers use, the MR14 the DRAM holds as MRR
    /// returns it, ...), not the register that was written, and say
    /// whether it is at point `i`. A knob without a live readback answers
    /// [`Live::Unverifiable`] (the default); the report then says "knob not
    /// verifiable". Reads only.
    fn verify_live(&mut self, _i: usize) -> Result<Live, Self::Error> {
        Ok(Live::Unverifiable)
    }
    /// Run the delay search on `rank` at the current point; one window per
    /// lane (wider is better, 0 is closed).
    fn run(&mut self, rank: u32) -> Result<[u8; L], Self::Error>;
    /// The board's mode check (pstate, training mode registers...), run
    /// before and after the sweep when the policy's checks are on
    /// ([`crate::sanity::mode`]).
    fn mode(&mut self) -> Result<(), crate::sanity::SanityError> {
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Sweep {
    /// Axis point to start from (clamped to the axis).
    pub start: usize,
    /// Closed points in a row that end the sweep in one direction
    /// (`usize::MAX`: the whole axis).
    pub stop_after: usize,
    /// Smallest acceptable worst-lane window after the final re-run.
    pub min_window: u8,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Outcome<const L: usize> {
    pub eye: Eye,
    /// The chosen axis point (`eye.center`), set on the axis.
    pub point: usize,
    /// Per-lane window at `point`, the minimum over ranks.
    pub window: [u8; L],
    pub worst: u8,
    /// Axis moves confirmed live ("points verified live: N/M").
    pub live: Liveness,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SweepError<E> {
    /// The delay search failed.
    Search(E),
    /// No open point on the axis.
    Closed,
    /// The final worst window is below `min_window`.
    Margin { worst: u8 },
    /// A knob-response check failed ([`crate::sanity`]); no decision.
    Sanity(crate::sanity::SanityError),
    /// `set(point)` did not reach the knob's live copy
    /// ([`Axis::verify_live`]); nothing was measured there.
    NotLive { point: usize },
}

/// `set(i)` plus the live check.
fn move_to<A: Axis<L>, const L: usize>(
    a: &mut A,
    i: usize,
    live: &mut Liveness,
) -> Result<(), SweepError<A::Error>> {
    a.set(i);
    live.record(a.verify_live(i).map_err(SweepError::Search)?)
        .map_err(|_| SweepError::NotLive { point: i })
}

/// Worst window over all lanes and ranks at the current point.
fn width<A: Axis<L>, const L: usize>(a: &mut A) -> Result<u8, A::Error> {
    let mut w = u8::MAX;
    for rank in 0..a.ranks() {
        w = w.min(worst(&a.run(rank)?));
    }
    Ok(w)
}

/// Sweep `axis` into `profile` (one entry per axis point; `None` where not
/// measured), centre, set and re-run, with the default [`SweepPolicy`]
/// ([`SweepPolicy::DEFAULT`]: ratio-bounded search, plateau centring).
/// `profile` holds the measured profile afterwards, also on failure, for the
/// board's log.
pub fn sweep_axis<A: Axis<L>, const L: usize>(
    axis: &mut A,
    profile: &mut [Option<u8>],
    cfg: &Sweep,
) -> Result<Outcome<L>, SweepError<A::Error>> {
    sweep_axis_with(axis, profile, cfg, &SweepPolicy::DEFAULT).map(|(o, _)| o)
}

/// The sweep as moved from the K1 (`train_own.rs`), before v1.0.0: walk out
/// from `cfg.start` until `cfg.stop_after` closed points each way, centre at
/// half height ([`SweepPolicy::LEGACY`]).
pub fn sweep_axis_legacy<A: Axis<L>, const L: usize>(
    axis: &mut A,
    profile: &mut [Option<u8>],
    cfg: &Sweep,
) -> Result<Outcome<L>, SweepError<A::Error>> {
    let mut live = Liveness::default();
    walk_out(profile, cfg.start, cfg.stop_after, |i| {
        move_to(axis, i, &mut live)?;
        width(axis).map_err(SweepError::Search)
    })?;
    let eye = crate::eye::center_legacy(profile).map_err(|_| SweepError::Closed)?;
    move_to(axis, eye.center, &mut live)?;
    let mut window = [u8::MAX; L];
    for rank in 0..axis.ranks() {
        let w = axis.run(rank).map_err(SweepError::Search)?;
        for (a, b) in window.iter_mut().zip(w) {
            *a = (*a).min(b);
        }
    }
    let worst = worst(&window);
    if worst < cfg.min_window {
        return Err(SweepError::Margin { worst });
    }
    Ok(Outcome {
        eye,
        point: eye.center,
        window,
        worst,
        live,
    })
}

// ---------------------------------------------------------------------------
// Ratio-bounded Vref search (AMD PG353 v1.0 p38, Versal soft DDR4 VREF
// calibration; method only): start at the default point, widen around the
// largest window seen so far until the window falls below `ratio` of it on
// both sides, re-bounding whenever a larger window appears, and take the
// midpoint of the bounds. Each point is measured at most once.

/// Result of [`ratio_bounds`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Bounds {
    /// Lowest and highest point with a window of at least `threshold`,
    /// contiguous with `peak`.
    pub lb: usize,
    pub ub: usize,
    /// The midpoint of the bounds (the lower middle when their distance is
    /// odd).
    pub center: usize,
    pub peak: usize,
    pub w_max: u8,
    /// ceil(`ratio_permille` x `w_max` / 1000), at least 1.
    pub threshold: u8,
    /// Points measured.
    pub measured: usize,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BoundsError<E> {
    Measure(E),
    /// Empty axis.
    Empty,
    /// Every measured point is closed.
    Closed,
}

/// Bound the open region of an axis of `profile.len()` points around the
/// largest window, measuring lazily from `start`: walking out from the
/// largest window seen, a side ends after `confirm` (at least 1) points in a
/// row below `ratio_permille` / 1000 of it (PG353: "N x W_max"; `confirm` =
/// 1 is the literal method) or at the axis end; a larger window found on the
/// way becomes the new peak and the bounds are redone around it. The bounds
/// are the points of at least the threshold contiguous with the peak.
/// `profile` holds every measured window afterwards (None: not measured).
pub fn ratio_bounds<E>(
    profile: &mut [Option<u8>],
    start: usize,
    ratio_permille: u16,
    confirm: usize,
    mut measure: impl FnMut(usize) -> Result<u8, E>,
) -> Result<Bounds, BoundsError<E>> {
    for p in profile.iter_mut() {
        *p = None;
    }
    let len = profile.len();
    if len == 0 {
        return Err(BoundsError::Empty);
    }
    let start = start.min(len - 1);
    let confirm = confirm.max(1);
    let mut measured = 0usize;
    let mut get = |profile: &mut [Option<u8>], i: usize| -> Result<u8, E> {
        if let Some(w) = profile[i] {
            return Ok(w);
        }
        let w = measure(i)?;
        profile[i] = Some(w);
        measured += 1;
        Ok(w)
    };
    let w0 = get(profile, start).map_err(BoundsError::Measure)?;
    let (mut peak, mut w_max) = (start, w0);
    'redo: loop {
        let threshold = ((ratio_permille as u32 * w_max as u32).div_ceil(1000) as u8).max(1);
        for up in [true, false] {
            let mut below = 0;
            let mut i = peak;
            loop {
                let next = if up {
                    if i + 1 >= len {
                        break;
                    }
                    i + 1
                } else {
                    if i == 0 {
                        break;
                    }
                    i - 1
                };
                let w = get(profile, next).map_err(BoundsError::Measure)?;
                if w > w_max {
                    peak = next;
                    w_max = w;
                    continue 'redo;
                }
                below = if w < threshold { below + 1 } else { 0 };
                if below >= confirm {
                    break;
                }
                i = next;
            }
        }
        if w_max == 0 {
            return Err(BoundsError::Closed);
        }
        let open = |i: usize| matches!(profile[i], Some(w) if w >= threshold);
        let mut lb = peak;
        while lb > 0 && open(lb - 1) {
            lb -= 1;
        }
        let mut ub = peak;
        while ub + 1 < len && open(ub + 1) {
            ub += 1;
        }
        return Ok(Bounds {
            lb,
            ub,
            center: (lb + ub) / 2,
            peak,
            w_max,
            threshold,
            measured,
        });
    }
}

/// How [`sweep_axis_with`] searches the axis.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Search {
    /// Walk out from the start until `Sweep::stop_after` closed points each
    /// way ([`walk_out`]; the pre-v1.0.0 search).
    WalkOut,
    /// [`ratio_bounds`] with this ratio, ending a side after
    /// `Sweep::stop_after` points below it (at least 1). If every measured
    /// point is closed, the rest of the axis is measured.
    RatioBounds { ratio_permille: u16 },
}

/// How [`sweep_axis_with`] measures and centres.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SweepPolicy {
    pub search: Search,
    /// Centring of the profile ([`crate::eye::center_with`]).
    pub center: crate::eye::Policy,
    /// Measurements per axis point; the worst counts (at least 1).
    pub repeats: u8,
    /// Knob-response checks: the axis's mode check before and after, a
    /// fully measured flat axis is a dead knob, and an aliased stretch of at
    /// least `alias_min_len` points is an error (0: no alias check).
    pub checks: bool,
    pub alias_min_len: usize,
    /// What to pick on the plateau (since REQ-K1-03; default
    /// [`Objective::PlateauCentre`], the v1.0.0 behaviour).
    pub objective: Objective,
}

/// What [`sweep_axis_sums`] picks once the eye is centred.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Objective {
    /// The centring policy's point (v1.0.0).
    PlateauCentre,
    /// On the plateau of the largest worst-lane window, the point with the
    /// largest sum of lane windows (each lane's minimum over ranks and
    /// repeats); ties go to the point nearest the plateau-centre choice
    /// (the lower one on a tie). Needs a sums buffer ([`sweep_axis_sums`]).
    /// REQ-K1-03: a flat worst profile (R2S fp2 write: 9 on points 14-33)
    /// otherwise centres away from where the lanes are widest.
    MaxMinThenSum,
}

impl SweepPolicy {
    /// This policy with another [`Objective`].
    pub const fn with_objective(mut self, objective: Objective) -> SweepPolicy {
        self.objective = objective;
        self
    }
    /// The default since v1.0.0: ratio-bounded search at 900 / 1000 of the
    /// largest window and plateau centring ([`crate::eye::Policy::DEFAULT`]).
    /// 900 is the early-break regime (MT8183 uses 950): every recorded K1
    /// profile stays above half height across its axis, so 500 saved no
    /// engine run; 900 with K1's stop rule replays at 159 instead of 297 runs
    /// per boot with the same widths (orchestrator
    /// prior-art-dram-training-20261014/OLD-VS-NEW-COMPARISON.md section 3.1).
    /// A coarse-then-fine Vref pass (111 runs) was not taken: its coarse step
    /// can skip a narrow plateau (K1 RX: 4 codes).
    pub const DEFAULT: SweepPolicy = SweepPolicy {
        search: Search::RatioBounds {
            ratio_permille: 900,
        },
        center: crate::eye::Policy::DEFAULT,
        repeats: 1,
        checks: true,
        alias_min_len: 8,
        objective: Objective::PlateauCentre,
    };
    /// [`sweep_axis_legacy`]'s behaviour.
    pub const LEGACY: SweepPolicy = SweepPolicy {
        search: Search::WalkOut,
        center: crate::eye::Policy::LEGACY,
        repeats: 1,
        checks: false,
        alias_min_len: 0,
        objective: Objective::PlateauCentre,
    };
    pub const fn legacy() -> SweepPolicy {
        SweepPolicy::LEGACY
    }
}

impl Default for SweepPolicy {
    fn default() -> Self {
        SweepPolicy::DEFAULT
    }
}

/// Sweep with a [`SweepPolicy`]: each point measured `repeats` times keeping
/// the worst, searched and centred by the policy, then set and re-run. With
/// [`SweepPolicy::LEGACY`] it is [`sweep_axis_legacy`]. The second value
/// tells whether an edge of the eye lay outside the scan.
pub fn sweep_axis_with<A: Axis<L>, const L: usize>(
    axis: &mut A,
    profile: &mut [Option<u8>],
    cfg: &Sweep,
    policy: &SweepPolicy,
) -> Result<(Outcome<L>, bool), SweepError<A::Error>> {
    sweep_axis_sums(axis, profile, &mut [], cfg, policy)
}

/// [`sweep_axis_with`], also recording each measured point's sum of lane
/// windows (each lane's minimum over ranks and repeats) in `sums` (None:
/// not measured), for the board's log and for
/// [`Objective::MaxMinThenSum`], which needs `sums` at least as long as
/// `profile` (otherwise [`SweepError::Sanity`] with a mode mismatch).
pub fn sweep_axis_sums<A: Axis<L>, const L: usize>(
    axis: &mut A,
    profile: &mut [Option<u8>],
    sums: &mut [Option<u16>],
    cfg: &Sweep,
    policy: &SweepPolicy,
) -> Result<(Outcome<L>, bool), SweepError<A::Error>> {
    if policy.objective == Objective::MaxMinThenSum && sums.len() < profile.len() {
        return Err(SweepError::Sanity(
            crate::sanity::SanityError::ModeMismatch {
                what: "MaxMinThenSum needs a sums buffer as long as the profile",
                got: sums.len() as u32,
                want: profile.len() as u32,
            },
        ));
    }
    for s in sums.iter_mut() {
        *s = None;
    }
    if policy.checks {
        axis.mode().map_err(SweepError::Sanity)?;
    }
    let mut live = Liveness::default();
    // Each lane's minimum over ranks and repeats: the worst lane is the
    // same minimum as before; the sum is recorded.
    let mut point = |axis: &mut A, i: usize| -> Result<u8, SweepError<A::Error>> {
        move_to(axis, i, &mut live)?;
        let mut lanes = [u8::MAX; L];
        for _ in 0..policy.repeats.max(1) {
            for rank in 0..axis.ranks() {
                for (a, b) in lanes
                    .iter_mut()
                    .zip(axis.run(rank).map_err(SweepError::Search)?)
                {
                    *a = (*a).min(b);
                }
            }
        }
        if let Some(s) = sums.get_mut(i) {
            *s = Some(lanes.iter().map(|&w| u16::from(w)).sum());
        }
        Ok(worst(&lanes))
    };
    match policy.search {
        Search::WalkOut => walk_out(profile, cfg.start, cfg.stop_after, |i| point(axis, i))?,
        Search::RatioBounds { ratio_permille } => {
            let confirm = cfg.stop_after.min(profile.len()).max(1);
            match ratio_bounds(profile, cfg.start, ratio_permille, confirm, |i| {
                point(axis, i)
            }) {
                Ok(_) => {}
                Err(BoundsError::Measure(e)) => return Err(e),
                Err(BoundsError::Empty) => return Err(SweepError::Closed),
                Err(BoundsError::Closed) => {
                    for (i, w) in profile.iter_mut().enumerate() {
                        if w.is_none() {
                            *w = Some(point(axis, i)?);
                        }
                    }
                }
            }
        }
    }
    if policy.checks {
        crate::sanity::flat_profile(profile, 8).map_err(SweepError::Sanity)?;
        if policy.alias_min_len > 0 {
            crate::sanity::alias(profile, policy.alias_min_len).map_err(SweepError::Sanity)?;
        }
    }
    let c = crate::eye::center_with(profile, policy.center).map_err(|_| SweepError::Closed)?;
    let mut eye = c.eye;
    if policy.objective == Objective::MaxMinThenSum {
        let at = |i: usize| profile[i];
        // The plateau: the centring's band, or the longest run at the peak
        // worst (the first on a tie).
        let (lo, hi) = if c.plateau {
            (eye.lo, eye.hi)
        } else {
            let (mut lo, mut hi) = (eye.peak, eye.peak);
            let mut i = 0;
            while i < profile.len() {
                if at(i) == Some(eye.peak_width) {
                    let s = i;
                    while i + 1 < profile.len() && at(i + 1) == Some(eye.peak_width) {
                        i += 1;
                    }
                    if i - s > hi - lo {
                        (lo, hi) = (s, i);
                    }
                }
                i += 1;
            }
            (lo, hi)
        };
        let mut best: Option<(u16, usize)> = None;
        for i in lo..=hi {
            if at(i) != Some(eye.peak_width) && !c.plateau {
                continue;
            }
            let Some(sum) = sums.get(i).copied().flatten() else {
                continue;
            };
            let better = match best {
                None => true,
                Some((bs, bi)) => {
                    sum > bs || (sum == bs && i.abs_diff(eye.center) < bi.abs_diff(eye.center))
                }
            };
            if better {
                best = Some((sum, i));
            }
        }
        if let Some((_, i)) = best {
            eye.center = i;
            eye.center_width = at(i).unwrap_or(0);
        }
    }
    move_to(axis, eye.center, &mut live)?;
    let mut window = [u8::MAX; L];
    for rank in 0..axis.ranks() {
        let w = axis.run(rank).map_err(SweepError::Search)?;
        for (a, b) in window.iter_mut().zip(w) {
            *a = (*a).min(b);
        }
    }
    let worst = worst(&window);
    if worst < cfg.min_window {
        return Err(SweepError::Margin { worst });
    }
    if policy.checks {
        axis.mode().map_err(SweepError::Sanity)?;
    }
    Ok((
        Outcome {
            eye,
            point: eye.center,
            window,
            worst,
            live,
        },
        c.edge_outside_scan,
    ))
}
