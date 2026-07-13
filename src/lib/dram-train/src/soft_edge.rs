// SPDX-License-Identifier: GPL-2.0-only
//! Soft-information edges: instead of a pass/fail decision per delay
//! point, use the error fraction (errors out of compares) near an edge.
//! On the probit (Q) scale a Gaussian-jitter edge is a straight line in
//! the delay, so a least-squares line through the partly failing points
//! gives:
//! - the edge at 50 % errors, to a fraction of a tap;
//! - the jitter sigma from the slope (a noise figure per lane);
//! - the edge at any target error rate (a bathtub point), and from two
//!   edges the eye width at that rate ([`eye_width_at`]).
//!
//! Points that never or always fail carry no slope and are not fitted; an
//! edge without at least two partly failing points falls back to the hard
//! edge (between the last clean and the first all-failing point) with no
//! sigma. Deterministic-jitter edges (ISI, crosstalk) are not Gaussian:
//! use the fit for centring and diagnostics and keep the minimum-width
//! gate on measured points.
//!
//! Integer arithmetic only (no_std, no floats): the probit comes from a
//! table of the inverse normal CDF at k/256 (milli-Q), interpolated.
//! Source: ALTERNATIVE-APPROACHES (prior art, orchestrator) section 3.3;
//! the dual-Dirac / Q-scale method is the long-standing SerDes practice
//! (cited, not copied).

/// Inverse normal CDF at k/256, k = 1..=255, in thousandths (generated
/// with Python's `statistics.NormalDist().inv_cdf`, rounded).
#[rustfmt::skip]
const PROBIT: [i16; 255] = [
    -2660, -2418, -2266, -2154, -2064, -1987, -1921, -1863, -1810, -1762, -1717, -1676,
    -1637, -1601, -1567, -1534, -1503, -1473, -1445, -1418, -1392, -1366, -1342, -1318,
    -1295, -1273, -1251, -1230, -1209, -1189, -1170, -1150, -1132, -1113, -1095, -1078,
    -1060, -1043, -1026, -1010, -994, -978, -962, -947, -932, -917, -902, -887,
    -873, -858, -844, -831, -817, -803, -790, -776, -763, -750, -737, -725,
    -712, -699, -687, -674, -662, -650, -638, -626, -614, -602, -591, -579,
    -568, -556, -545, -533, -522, -511, -500, -489, -478, -467, -456, -445,
    -434, -424, -413, -402, -392, -381, -371, -360, -350, -339, -329, -319,
    -308, -298, -288, -278, -268, -257, -247, -237, -227, -217, -207, -197,
    -187, -177, -167, -157, -147, -138, -128, -118, -108, -98, -88, -78,
    -69, -59, -49, -39, -29, -20, -10, 0, 10, 20, 29, 39,
    49, 59, 69, 78, 88, 98, 108, 118, 128, 138, 147, 157,
    167, 177, 187, 197, 207, 217, 227, 237, 247, 257, 268, 278,
    288, 298, 308, 319, 329, 339, 350, 360, 371, 381, 392, 402,
    413, 424, 434, 445, 456, 467, 478, 489, 500, 511, 522, 533,
    545, 556, 568, 579, 591, 602, 614, 626, 638, 650, 662, 674,
    687, 699, 712, 725, 737, 750, 763, 776, 790, 803, 817, 831,
    844, 858, 873, 887, 902, 917, 932, 947, 962, 978, 994, 1010,
    1026, 1043, 1060, 1078, 1095, 1113, 1132, 1150, 1170, 1189, 1209, 1230,
    1251, 1273, 1295, 1318, 1342, 1366, 1392, 1418, 1445, 1473, 1503, 1534,
    1567, 1601, 1637, 1676, 1717, 1762, 1810, 1863, 1921, 1987, 2064, 2154,
    2266, 2418, 2660,
];

/// Q (thousandths) of an error fraction `e / n`, 0 < e < n.
pub fn probit_milli(e: u32, n: u32) -> Option<i32> {
    if n == 0 || e == 0 || e >= n {
        return None;
    }
    // Position on the table in 1/1024 steps of k.
    let pos = (u64::from(e) * 256 * 1024) / u64::from(n);
    let k = (pos / 1024) as usize; // 0..=255
    let frac = (pos % 1024) as i64;
    let at = |k: usize| -> i64 {
        match k {
            0 => i64::from(PROBIT[0]) - 400, // below 1/256: extend the tail
            256.. => i64::from(PROBIT[254]) + 400,
            _ => i64::from(PROBIT[k - 1]),
        }
    };
    let (a, b) = (at(k), at(k + 1));
    Some((a + (b - a) * frac / 1024) as i32)
}

/// One measured point: delay (taps), errors out of compares.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Point {
    pub delay: i32,
    pub errors: u32,
    pub compares: u32,
}

/// A fitted (or hard) edge.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SoftEdge {
    /// Delay at 50 % errors, thousandths of a tap.
    pub center_milli: i64,
    /// Jitter sigma, thousandths of a tap; None for a hard edge.
    pub sigma_milli: Option<i64>,
    /// Errors rise with the delay (a right edge of a window).
    pub rising: bool,
    /// Partly failing points fitted (0: hard edge).
    pub fitted: u32,
}

impl SoftEdge {
    /// The delay (thousandths of a tap) where the error rate is the one
    /// whose Q is `q_milli` towards the inside of the window (e.g. 7034 for
    /// 1e-12): moved by q x sigma away from the failing side. A hard edge
    /// stays where it is.
    pub fn at_q(&self, q_milli: i64) -> i64 {
        let shift = self.sigma_milli.map_or(0, |s| s * q_milli / 1000);
        if self.rising {
            self.center_milli - shift
        } else {
            self.center_milli + shift
        }
    }
}

/// Fit one edge from `points` (any order, distinct delays). None if the
/// points show no edge (never failing, always failing, or no transition).
pub fn fit(points: &[Point]) -> Option<SoftEdge> {
    // Direction: compare the error fraction at the lowest and highest delay.
    let lo = points.iter().min_by_key(|p| p.delay)?;
    let hi = points.iter().max_by_key(|p| p.delay)?;
    let frac = |p: &Point| (u64::from(p.errors) << 16) / u64::from(p.compares.max(1));
    let rising = frac(hi) > frac(lo);
    if frac(hi) == frac(lo) {
        return None;
    }
    let (mut n, mut sx, mut sq, mut sxx, mut sxq) = (0i128, 0i128, 0i128, 0i128, 0i128);
    for p in points {
        if let Some(q) = probit_milli(p.errors, p.compares) {
            let (x, q) = (i128::from(p.delay), i128::from(q));
            n += 1;
            sx += x;
            sq += q;
            sxx += x * x;
            sxq += x * q;
        }
    }
    let den = n * sxq - sx * sq; // slope numerator (x D)
    let d = n * sxx - sx * sx;
    if n >= 2 && d != 0 && den != 0 && (den > 0) == rising {
        // Q = a x + b, a = den / d (milli-Q per tap); centre where Q = 0.
        let num = -(sq * sxx - sx * sxq); // -b x d
        let center_milli = (num * 1000 / den) as i64;
        let sigma_milli = (1_000_000 * d / den.abs()) as i64;
        return Some(SoftEdge {
            center_milli,
            sigma_milli: Some(sigma_milli),
            rising,
            fitted: n as u32,
        });
    }
    // Hard edge: between the last point on the clean side and the first
    // point at the failing level.
    let mut sorted: [Option<Point>; 64] = [None; 64];
    let m = points.len().min(64);
    for (s, p) in sorted.iter_mut().zip(points) {
        *s = Some(*p);
    }
    let pts = &mut sorted[..m];
    pts.sort_unstable_by_key(|p| p.map(|p| p.delay));
    let clean = |p: &Point| {
        if rising {
            p.errors == 0
        } else {
            p.errors == p.compares
        }
    };
    let mut last = None;
    for p in pts.iter().flatten() {
        if clean(p) {
            last = Some(p.delay);
        } else if let Some(l) = last {
            return Some(SoftEdge {
                center_milli: (i64::from(l) + i64::from(p.delay)) * 500,
                sigma_milli: None,
                rising,
                fitted: 0,
            });
        }
    }
    None
}

/// The eye width (thousandths of a tap) between a left edge (errors fall
/// with the delay) and a right edge (errors rise) at the error rate whose
/// Q is `q_milli`; 0 if closed at that rate.
pub fn eye_width_at(left: &SoftEdge, right: &SoftEdge, q_milli: i64) -> i64 {
    (right.at_q(q_milli) - left.at_q(q_milli)).max(0)
}

/// Q (thousandths) of common target error rates.
pub const Q_1E_6: i64 = 4753;
pub const Q_1E_9: i64 = 5998;
pub const Q_1E_12: i64 = 7034;
