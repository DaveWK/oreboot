// SPDX-License-Identifier: GPL-2.0-only
//! The delay-line ruler and the total-delay API.
//!
//! A fine delay line's step size varies by silicon, voltage and
//! temperature. Measure once how many taps make a known time (a quarter
//! clock, or a whole clock period, from two edges of a clock-rate signal),
//! then express every limit (45 degrees, one UI, one clock) and every
//! reported margin through that ruler, in taps, ps, UI or degrees, instead
//! of assuming a nominal tap size.
//!
//! Delays are set as one total, which the board's delay chain splits into
//! whole-UI (or whole-cycle) stages and a fine remainder ([`split`],
//! [`join`]); training code then moves totals and never has to carry
//! between the coarse and fine fields by hand.
//!
//! Method: AMD PG150 v1.4 p734 and p918 (taps per quarter clock measured by
//! BISC; ps = taps x (tCK/4) / taps-per-quarter), UG1085 p455 and UG1087
//! `MDLR0.IPRD/TPRD` (period measurement), `DXnLCDLR1.WDQD` with
//! `DXnGTR0.WDQSL` (a total delay as a UI pipeline plus a fine delay);
//! cited, not copied.

use crate::result::{Margin, Unit};

/// Taps of a delay line against time, at one clock period.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Ruler {
    /// Clock period, ps.
    pub tck_ps: u32,
    /// Taps in one clock period (x 1000, so a fractional tap count from a
    /// quarter-clock measurement keeps its precision).
    pub milli_taps_per_tck: u64,
}

impl Ruler {
    /// From taps measured over one quarter clock (BISC style).
    pub fn from_quarter(tck_ps: u32, taps_per_quarter: u32) -> Option<Self> {
        (tck_ps > 0 && taps_per_quarter > 0).then(|| Self {
            tck_ps,
            milli_taps_per_tck: 4000 * u64::from(taps_per_quarter),
        })
    }

    /// From taps measured over one clock period (`MDLR0` style).
    pub fn from_period(tck_ps: u32, taps_per_tck: u32) -> Option<Self> {
        (tck_ps > 0 && taps_per_tck > 0).then(|| Self {
            tck_ps,
            milli_taps_per_tck: 1000 * u64::from(taps_per_tck),
        })
    }

    /// From the taps of two consecutive same-direction edges of a
    /// clock-rate signal found by a delay sweep (one period apart).
    pub fn from_edges(tck_ps: u32, first: u32, second: u32) -> Option<Self> {
        Self::from_period(tck_ps, second.checked_sub(first)?)
    }

    /// Picoseconds of `taps` (rounded to nearest).
    pub fn ps(&self, taps: i32) -> i32 {
        let num = i64::from(taps) * i64::from(self.tck_ps) * 1000;
        let den = self.milli_taps_per_tck as i64;
        ((num + num.signum() * den / 2) / den) as i32
    }

    /// Taps of `ps` (rounded to nearest).
    pub fn taps(&self, ps: i32) -> i32 {
        let num = i64::from(ps) * self.milli_taps_per_tck as i64;
        let den = i64::from(self.tck_ps) * 1000;
        ((num + num.signum() * den / 2) / den) as i32
    }

    /// Taps of `deg` degrees of the clock (360 = one period).
    pub fn degrees(&self, deg: u32) -> u32 {
        ((self.milli_taps_per_tck * u64::from(deg) + 180_000) / 360_000) as u32
    }

    /// Taps of one UI (half a clock at double data rate; `ui_per_tck` 2).
    pub fn ui(&self, ui_per_tck: u32) -> u32 {
        let n = u64::from(ui_per_tck.max(1)) * 1000;
        ((self.milli_taps_per_tck + n / 2) / n) as u32
    }

    /// A margin line in steps converted to ps through the ruler.
    pub fn margin_ps(&self, m: Margin) -> Margin {
        if m.unit != Unit::Step {
            return m;
        }
        Margin {
            unit: Unit::Ps,
            lo: self.ps(m.lo),
            hi: self.ps(m.hi),
            center: self.ps(m.center),
            ..m
        }
    }
}

/// A total delay as whole stages plus a fine remainder.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Parts {
    /// Whole stages (UI or cycles of the pipeline).
    pub stages: u32,
    /// Fine taps, below `stage_taps` (or at most `fine_max`).
    pub fine: u32,
}

/// Split `total` taps into whole stages of `stage_taps` and a fine
/// remainder. If the fine line can hold more than one stage (`fine_max`
/// at least `stage_taps`) the stages are still taken whole first, so the
/// fine part stays small and the same total always gives the same parts.
/// None if a stage is 0 taps or the stages would exceed `max_stages`.
pub fn split(total: u32, stage_taps: u32, max_stages: u32) -> Option<Parts> {
    if stage_taps == 0 {
        return None;
    }
    let stages = total / stage_taps;
    (stages <= max_stages).then_some(Parts {
        stages,
        fine: total % stage_taps,
    })
}

/// The total of a stage/fine pair.
pub const fn join(p: Parts, stage_taps: u32) -> u32 {
    p.stages * stage_taps + p.fine
}

/// A delay set as a total; the board writes the parts.
pub trait TotalDelay {
    type Error;
    /// Taps per stage of this delay chain.
    fn stage_taps(&self) -> u32;
    fn max_stages(&self) -> u32;
    fn write_parts(&mut self, p: Parts) -> Result<(), Self::Error>;
}

/// Why a total delay could not be set.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DelayError<E> {
    Board(E),
    /// Beyond the last stage.
    Range,
}

/// Set `total` taps on `d` (split, then write the parts).
pub fn set_total<D: TotalDelay>(d: &mut D, total: u32) -> Result<Parts, DelayError<D::Error>> {
    let p = split(total, d.stage_taps(), d.max_stages()).ok_or(DelayError::Range)?;
    d.write_parts(p).map_err(DelayError::Board)?;
    Ok(p)
}
