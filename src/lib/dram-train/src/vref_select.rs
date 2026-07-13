// SPDX-License-Identifier: GPL-2.0-only
//! Choosing a Vref (or DFE, DCA) setting from noisy window measurements
//! with a fixed budget, and smoothing a measured profile before choosing.
//!
//! - [`successive_halving`]: measure every candidate once, keep the better
//!   half by mean window, measure those again, and so on, until one is left
//!   or the budget runs out. The replicates go to the candidates that can
//!   still win instead of being spread evenly (B2).
//! - [`median3`], [`mean3`]: three-point smoothing of a profile, and
//!   [`concave_peak`]: the maximum of a concave quadratic fitted to the
//!   points at or above half height (B3). Both cut single-point noise
//!   without extra measurements; the result feeds the usual centring.
//! - [`sweep_halving`]: successive halving over an [`crate::vref_sweep::Axis`]
//!   (each pull is the worst lane window over all ranks), the axis left at
//!   the winner.
//!
//! Selectable next to the v1.0.0 default (`vref_sweep::SweepPolicy::DEFAULT`,
//! the ratio-bounded sweep); proven on the recorded R2S and b0 profiles
//! (`tests/vref_select.rs`). Source: ALTERNATIVE-APPROACHES (prior art,
//! orchestrator) sections 3.1 and 3.2; successive halving and three-point
//! smoothing are textbook methods.

/// One candidate and its measurements so far.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Arm {
    pub point: u16,
    pub n: u8,
    pub sum: u16,
    pub min: u8,
}

impl Arm {
    pub const fn new(point: u16) -> Self {
        Self {
            point,
            n: 0,
            sum: 0,
            min: u8::MAX,
        }
    }
    /// Mean window x 256 (0 when not measured).
    pub const fn mean256(&self) -> u32 {
        if self.n == 0 {
            0
        } else {
            (self.sum as u32 * 256) / self.n as u32
        }
    }
    fn pull(&mut self, w: u8) {
        self.n = self.n.saturating_add(1);
        self.sum = self.sum.saturating_add(w as u16);
        self.min = self.min.min(w);
    }
}

/// What [`successive_halving`] chose.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Choice {
    /// The winning candidate's point.
    pub point: u16,
    pub mean256: u32,
    /// Measurements spent.
    pub runs: u32,
    pub rounds: u8,
    /// Candidates still in the race at the end (1 unless the budget ran out).
    pub finalists: usize,
}

/// Successive halving over `arms` (their points; measurements are added to
/// them) with at most `budget` measurements; a round runs only if the
/// budget covers it whole. Ranking is by mean window, then minimum; among
/// leaders tied on both, those nearest the middle of the tied group stay
/// in the race and the middle one (by point) wins, so a flat plateau gives
/// its centre. None if `arms` is empty or the budget does not cover
/// one round.
pub fn successive_halving<E>(
    arms: &mut [Arm],
    budget: u32,
    mut measure: impl FnMut(u16) -> Result<u8, E>,
) -> Result<Option<Choice>, E> {
    let mut alive = arms.len();
    let mut runs = 0u32;
    let mut rounds = 0u8;
    while alive > 0 {
        if runs + alive as u32 > budget {
            break;
        }
        for a in arms[..alive].iter_mut() {
            let w = measure(a.point)?;
            a.pull(w);
        }
        runs += alive as u32;
        rounds += 1;
        arms[..alive].sort_unstable_by(|x, y| {
            y.mean256()
                .cmp(&x.mean256())
                .then(y.min.cmp(&x.min))
                .then(x.point.cmp(&y.point))
        });
        // Among the leaders tied on mean and minimum, keep the ones nearest
        // the middle of their group, so a plateau loses its ends first.
        let (m0, n0) = (arms[0].mean256(), arms[0].min);
        let t = arms[..alive]
            .iter()
            .take_while(|a| a.mean256() == m0 && a.min == n0)
            .count();
        let (lo, hi) = arms[..t]
            .iter()
            .fold((u16::MAX, 0), |(l, h), a| (l.min(a.point), h.max(a.point)));
        let mid2 = lo as i32 + hi as i32;
        arms[..t].sort_unstable_by_key(|a| ((2 * a.point as i32 - mid2).abs(), a.point));
        if alive == 1 {
            break;
        }
        alive = alive.div_ceil(2);
    }
    if rounds == 0 {
        return Ok(None);
    }
    // The leaders tied on the best mean: take the middle one by point.
    let best = arms[0].mean256();
    let tied = arms[..alive.max(1)]
        .iter()
        .filter(|a| a.mean256() == best)
        .count();
    let tie = &mut arms[..tied];
    tie.sort_unstable_by_key(|a| a.point);
    let w = tie[(tied - 1) / 2];
    Ok(Some(Choice {
        point: w.point,
        mean256: w.mean256(),
        runs,
        rounds,
        finalists: alive,
    }))
}

/// Three-point median of `p` into `out` (the ends use their two points'
/// minimum; a missing neighbour leaves the point as it is).
pub fn median3(p: &[Option<u8>], out: &mut [Option<u8>]) {
    smooth(p, out, |a, b, c| a.min(b).max(a.max(b).min(c)))
}

/// Three-point mean (rounded down) of `p` into `out`, with the same edge
/// rules as [`median3`].
pub fn mean3(p: &[Option<u8>], out: &mut [Option<u8>]) {
    smooth(p, out, |a, b, c| {
        ((a as u16 + b as u16 + c as u16) / 3) as u8
    })
}

fn smooth(p: &[Option<u8>], out: &mut [Option<u8>], f: impl Fn(u8, u8, u8) -> u8) {
    let n = p.len().min(out.len());
    for i in 0..n {
        out[i] = match (
            i.checked_sub(1).and_then(|j| p[j]),
            p[i],
            p.get(i + 1).copied().flatten(),
        ) {
            (Some(a), Some(b), Some(c)) => Some(f(a, b, c)),
            (None, Some(b), Some(c)) if i == 0 => Some(b.min(c)),
            (Some(a), Some(b), None) if i + 1 == p.len() => Some(a.min(b)),
            (_, b, _) => b,
        };
    }
}

/// The maximum of a concave quadratic least-squares fit to the measured
/// points at or above half the profile's maximum, rounded and clamped to
/// those points' range. None with fewer than three such points or a fit
/// that is not concave.
pub fn concave_peak(p: &[Option<u8>]) -> Option<usize> {
    let max = p.iter().flatten().copied().max()?;
    let half = max.div_ceil(2);
    let pts = || {
        p.iter()
            .enumerate()
            .filter_map(move |(i, v)| v.filter(|&v| v >= half).map(|v| (i as i128, v as i128)))
    };
    let (mut s, mut t) = ([0i128; 5], [0i128; 3]);
    let (mut lo, mut hi, mut n) = (i128::MAX, i128::MIN, 0);
    for (x, y) in pts() {
        let mut xk = 1;
        for sk in s.iter_mut() {
            *sk += xk;
            xk *= x;
        }
        t[0] += y;
        t[1] += x * y;
        t[2] += x * x * y;
        lo = lo.min(x);
        hi = hi.max(x);
        n += 1;
    }
    if n < 3 {
        return None;
    }
    // Normal equations for y = a x^2 + b x + c:
    // [s4 s3 s2][a]   [t2]
    // [s3 s2 s1][b] = [t1]
    // [s2 s1 s0][c]   [t0]
    let det3 = |m: [[i128; 3]; 3]| {
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
    };
    let m = [[s[4], s[3], s[2]], [s[3], s[2], s[1]], [s[2], s[1], s[0]]];
    let d = det3(m);
    if d == 0 {
        return None;
    }
    let da = det3([[t[2], s[3], s[2]], [t[1], s[2], s[1]], [t[0], s[1], s[0]]]);
    let db = det3([[s[4], t[2], s[2]], [s[3], t[1], s[1]], [s[2], t[0], s[0]]]);
    // a = da/d < 0 for a concave fit; peak at x = -b / 2a = -db / (2 da).
    if (da > 0) == (d > 0) || da == 0 {
        return None;
    }
    let (mut num, mut den) = (-db, 2 * da);
    if den < 0 {
        num = -num;
        den = -den;
    }
    let x = (2 * num + den).div_euclid(2 * den); // nearest
    Some(x.clamp(lo, hi) as usize)
}

/// Successive halving over axis points `points` of a Vref axis: each pull
/// sets the point and measures the worst lane window over all ranks. The
/// axis is left at the winner. `arms` is scratch (at least `points.len()`).
pub fn sweep_halving<A: crate::vref_sweep::Axis<L>, const L: usize>(
    axis: &mut A,
    points: &[u16],
    arms: &mut [Arm],
    budget: u32,
) -> Result<Option<Choice>, A::Error> {
    let n = points.len().min(arms.len());
    for (a, &p) in arms.iter_mut().zip(points).take(n) {
        *a = Arm::new(p);
    }
    let r = successive_halving(&mut arms[..n], budget, |p| {
        axis.set(p as usize);
        let mut w = u8::MAX;
        for rank in 0..axis.ranks() {
            w = w.min(crate::eye::worst(&axis.run(rank)?));
        }
        Ok(w)
    })?;
    if let Some(c) = r {
        axis.set(c.point as usize);
    }
    Ok(r)
}
