// SPDX-License-Identifier: GPL-2.0-only
//! Cross-pstate seeding by delay physics: a trained delay is a fixed
//! physical part (flight time, pad and insertion delay, in ps) plus a part
//! locked to the clock (whole UIs chosen by latency settings). On a PHY
//! that stores delays in UI fractions (coarse UI plus fine UI/64 on the
//! DWC PHYs), the same ps delay is a different code at each pstate. Fit
//! the two terms on trained pstates, predict the code at the others, and
//! search only a narrow bracket around the prediction, falling back to the
//! full range when the result is not inside the bracket.
//!
//! Model, in UI units: `code / fine_per_ui = t_ps / ui_ps + n_ui`, with
//! `n_ui` an integer. From two or more trained pstates both terms are
//! fitted (`n_ui` rounded to an integer, then `t_ps` by least squares);
//! from one, `n_ui` must be given (the board's latency settings).
//!
//! Source: ALTERNATIVE-APPROACHES (prior art, orchestrator) section 5.1;
//! the DWC PhyInit skip-train formulas are the same idea for one point.

/// One trained pstate: its UI in ps and the trained code of one delay.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Trained {
    pub ui_ps: f64,
    pub code: i32,
}

/// The fitted delay model.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Model {
    /// The fixed physical part, ps.
    pub t_ps: f64,
    /// Whole UIs locked to the clock.
    pub n_ui: i32,
    pub fine_per_ui: u32,
}

fn round(x: f64) -> i64 {
    if x >= 0.0 {
        (x + 0.5) as i64
    } else {
        (x - 0.5) as i64
    }
}

impl Model {
    /// From one trained pstate with a known UI-locked part.
    pub fn from_one(t: Trained, n_ui: i32, fine_per_ui: u32) -> Option<Model> {
        if t.ui_ps <= 0.0 || fine_per_ui == 0 {
            return None;
        }
        let y = f64::from(t.code) / f64::from(fine_per_ui);
        Some(Model {
            t_ps: (y - f64::from(n_ui)) * t.ui_ps,
            n_ui,
            fine_per_ui,
        })
    }

    /// From two or more trained pstates with different UIs.
    pub fn fit(points: &[Trained], fine_per_ui: u32) -> Option<Model> {
        if points.len() < 2 || fine_per_ui == 0 || points.iter().any(|p| p.ui_ps <= 0.0) {
            return None;
        }
        let f = f64::from(fine_per_ui);
        let (mut n, mut sx, mut sy, mut sxx, mut sxy) = (0.0, 0.0, 0.0, 0.0, 0.0);
        for p in points {
            let (x, y) = (1.0 / p.ui_ps, f64::from(p.code) / f);
            n += 1.0;
            sx += x;
            sy += y;
            sxx += x * x;
            sxy += x * y;
        }
        let d = n * sxx - sx * sx;
        if d.abs() < 1e-18 {
            return None; // all the same UI
        }
        let slope = (n * sxy - sx * sy) / d;
        let n_ui = round((sy - slope * sx) / n) as i32;
        // Refit the physical part with the integer UI term: least squares
        // of y - n = t x through the origin, t = sum(x (y - n)) / sum(x^2).
        let num: f64 = points
            .iter()
            .map(|p| (f64::from(p.code) / f - f64::from(n_ui)) / p.ui_ps)
            .sum();
        let den: f64 = points.iter().map(|p| 1.0 / (p.ui_ps * p.ui_ps)).sum();
        Some(Model {
            t_ps: num / den,
            n_ui,
            fine_per_ui,
        })
    }

    /// The predicted code at a pstate with this UI.
    pub fn code(&self, ui_ps: f64) -> i32 {
        round((self.t_ps / ui_ps + f64::from(self.n_ui)) * f64::from(self.fine_per_ui)) as i32
    }

    /// The prediction and a bracket of `margin_ps` each side (at least one
    /// code), clamped to `[lo, hi]`.
    pub fn seed(&self, ui_ps: f64, margin_ps: f64, lo: i32, hi: i32) -> Seed {
        let c = self.code(ui_ps).clamp(lo, hi);
        let m = (round(margin_ps / ui_ps * f64::from(self.fine_per_ui)) as i32).max(1);
        Seed {
            code: c,
            lo: (c - m).max(lo),
            hi: (c + m).min(hi),
        }
    }
}

/// A predicted code and the bracket to search.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Seed {
    pub code: i32,
    pub lo: i32,
    pub hi: i32,
}

/// How [`search_seeded`] ended.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Seeded {
    /// Found inside the bracket.
    InBracket(i32),
    /// Not inside the bracket (none, or on its edge): the full range gave
    /// this (None: not found there either).
    Full(Option<i32>),
}

/// Search the bracket first; a result on the bracket's boundary (it may lie
/// beyond) or none at all falls back to the full range `[lo, hi]`.
/// `search(lo, hi)` returns the trained code in that range, if any.
pub fn search_seeded<E>(
    seed: Seed,
    lo: i32,
    hi: i32,
    mut search: impl FnMut(i32, i32) -> Result<Option<i32>, E>,
) -> Result<Seeded, E> {
    if let Some(c) = search(seed.lo, seed.hi)? {
        let inside = (c > seed.lo || seed.lo == lo) && (c < seed.hi || seed.hi == hi);
        if inside {
            return Ok(Seeded::InBracket(c));
        }
    }
    Ok(Seeded::Full(search(lo, hi)?))
}
