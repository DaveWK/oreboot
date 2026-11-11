// SPDX-License-Identifier: GPL-2.0-only
//! DQS / WCK2DQ oscillator tracking.
//!
//! The DRAM's interval oscillator counts during a run time set by the
//! controller; the count gives the DRAM's internal strobe-to-data delay
//! (tDQS2DQ on LPDDR4/4X, tWCK2DQ on LPDDR5), which drifts with
//! temperature and voltage. Tracking reads it periodically, records the
//! drift since boot (min/max), and moves one rank's write delay by the
//! drift when it exceeds a threshold, while traffic runs on the other rank.
//! Gate tracking needs a read strobe at least every 20 us: without traffic
//! a dummy read is inserted.
//!
//! The count-to-delay relation is delay = run time / (divisor x count),
//! divisor 2 for the LPDDR4 DQS oscillator and the LPDDR5 WCK2DQ
//! oscillators. [`Osc`] keeps the divisor and run time as plain parameters
//! (a board with another oscillator still fits); for LPDDR4/4X they come
//! from `jedec_lpddr::lp4::mr` through [`Osc::lp4`] / [`Osc::lp4_mr23`]
//! (MR23 run time, `DQS_OSC_DIVISOR`), and [`Tracker::update_mr18_19`]
//! takes the count as the two mode registers read back. For LPDDR5
//! the WCK2DQI/WCK2DQO oscillators come from `jedec_lpddr::lp5::mr` through
//! [`Osc::lp5`] / [`Osc::lp5_mr`] (MR37/MR40 run time, the same coding as
//! MR23; `WCK2DQ_OSC_DIVISOR`) and [`Tracker::update_osc_regs`] (the count
//! as MR35/MR36 or MR38/MR39 read back, `Wck2DqOsc::registers`).
//!
//! Method: AMD PG313 v1.1 p71-72 and p361-362 (write DQS oscillator
//! tracking per rank while the other rank carries traffic; oscillator value
//! 0 and tDQS2DQ out of range are errors), PG456 v1.0 p36 and p159
//! (a dummy read when no read strobe for 20 us), AM019 `LPQDQSOSCTRACK_*`
//! (tDQS2DQ and the write delay tracked with min and max since boot);
//! cited, not copied.

use crate::result::Drift;
use crate::ruler::Ruler;
use jedec_lpddr::lp4::mr::{dqs_osc_count, OscRunTime, DQS_OSC_DIVISOR};
use jedec_lpddr::lp5::mr::WCK2DQ_OSC_DIVISOR;

/// Gate tracking needs a read strobe at least this often (us).
pub const DUMMY_READ_INTERVAL_US: u64 = 20;

/// Oscillator settings.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Osc {
    pub tck_ps: u32,
    /// Oscillator run time, clocks.
    pub runtime_tck: u32,
    /// delay = run time / (divisor x count).
    pub divisor: u32,
    /// Plausible delay range, ps.
    pub min_ps: u32,
    pub max_ps: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OscError {
    /// The oscillator returned 0 (not run, or not read back).
    Zero,
    /// The computed delay is outside the plausible range.
    Range { ps: u32 },
    /// No such rank.
    Rank,
}

impl Osc {
    /// LPDDR4/4X: the DQS oscillator at `tck_ps` with the MR23 run time;
    /// None for [`OscRunTime::StopByMpc`] (then the controller's measured
    /// run time goes into a plain `Osc`).
    pub const fn lp4(tck_ps: u32, run_time: OscRunTime, min_ps: u32, max_ps: u32) -> Option<Osc> {
        match run_time.clocks() {
            None => None,
            Some(runtime_tck) => Some(Osc {
                tck_ps,
                runtime_tck,
                divisor: DQS_OSC_DIVISOR,
                min_ps,
                max_ps,
            }),
        }
    }

    /// LPDDR4/4X from the MR23 byte as programmed (None for a reserved
    /// code or stop-by-MPC).
    pub const fn lp4_mr23(tck_ps: u32, mr23: u8, min_ps: u32, max_ps: u32) -> Option<Osc> {
        match OscRunTime::decode(mr23) {
            None => None,
            Some(r) => Osc::lp4(tck_ps, r, min_ps, max_ps),
        }
    }

    /// LPDDR5: a WCK2DQ oscillator (input or output path, the same
    /// arithmetic) at `tck_ps` with the MR37/MR40 run time; None for
    /// stop-by-MPC.
    pub const fn lp5(tck_ps: u32, run_time: OscRunTime, min_ps: u32, max_ps: u32) -> Option<Osc> {
        match run_time.clocks() {
            None => None,
            Some(runtime_tck) => Some(Osc {
                tck_ps,
                runtime_tck,
                divisor: WCK2DQ_OSC_DIVISOR,
                min_ps,
                max_ps,
            }),
        }
    }

    /// LPDDR5 from the MR37 or MR40 byte as programmed (None for a reserved
    /// code or stop-by-MPC).
    pub const fn lp5_mr(tck_ps: u32, run_time_mr: u8, min_ps: u32, max_ps: u32) -> Option<Osc> {
        match OscRunTime::decode(run_time_mr) {
            None => None,
            Some(r) => Osc::lp5(tck_ps, r, min_ps, max_ps),
        }
    }

    /// The delay (ps, rounded) for an oscillator count.
    pub fn delay_ps(&self, count: u32) -> Result<u32, OscError> {
        if count == 0 {
            return Err(OscError::Zero);
        }
        let run = u64::from(self.runtime_tck) * u64::from(self.tck_ps);
        let den = u64::from(self.divisor.max(1)) * u64::from(count);
        let ps = ((run + den / 2) / den) as u32;
        if ps < self.min_ps || ps > self.max_ps {
            return Err(OscError::Range { ps });
        }
        Ok(ps)
    }
}

/// Which way the write delay moves when the internal delay grows.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Sense {
    /// Delay DQ by the growth (DQ follows the slower strobe tree).
    DqFollows,
    /// Move the strobe earlier by the growth.
    StrobeLeads,
}

/// One rank's tracking state.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RankTrack {
    /// The delay when the write path was trained, ps.
    pub trained_ps: u32,
    /// The trained write delay, taps.
    pub trained_taps: i32,
    /// The write delay now set, taps.
    pub taps: i32,
    /// The oscillator delay since boot (ps).
    pub drift: Drift,
    /// The write delay since boot (taps).
    pub taps_drift: Drift,
}

/// Tracking for up to `R` ranks.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Tracker<const R: usize> {
    pub ranks: [RankTrack; R],
    pub osc: Osc,
    pub ruler: Ruler,
    /// Move the write delay only when it is this many taps off.
    pub threshold_taps: i32,
    pub sense: Sense,
}

impl<const R: usize> Tracker<R> {
    /// Start tracking with the delays measured right after training.
    pub fn new(
        osc: Osc,
        ruler: Ruler,
        trained: [(u32, i32); R],
        threshold_taps: i32,
        sense: Sense,
    ) -> Self {
        Self {
            ranks: trained.map(|(ps, taps)| RankTrack {
                trained_ps: ps,
                trained_taps: taps,
                taps,
                drift: Drift::new(ps as i32),
                taps_drift: Drift::new(taps),
            }),
            osc,
            ruler,
            threshold_taps: threshold_taps.max(1),
            sense,
        }
    }

    /// [`Tracker::update`] with the LPDDR4/4X count read back as MR18 (low
    /// byte) and MR19 (high byte).
    pub fn update_mr18_19(
        &mut self,
        rank: usize,
        mr18: u8,
        mr19: u8,
    ) -> Result<Option<i32>, OscError> {
        self.update(rank, u32::from(dqs_osc_count(mr18, mr19)))
    }

    /// [`Tracker::update`] with the count read back as its LSB and MSB
    /// registers (LPDDR5 MR35/MR36 or MR38/MR39, `Wck2DqOsc::registers`).
    pub fn update_osc_regs(
        &mut self,
        rank: usize,
        lsb: u8,
        msb: u8,
    ) -> Result<Option<i32>, OscError> {
        self.update(rank, u32::from(jedec_lpddr::lp5::mr::osc_count(lsb, msb)))
    }

    /// A new oscillator count for `rank`: record the drift and return the
    /// write delay to set if it moved by the threshold or more (the caller
    /// sets it while traffic runs on another rank, see [`schedule`]).
    pub fn update(&mut self, rank: usize, count: u32) -> Result<Option<i32>, OscError> {
        let ps = self.osc.delay_ps(count)?;
        let osc_ruler = self.ruler;
        let sense = self.sense;
        let threshold = self.threshold_taps;
        let r = self.ranks.get_mut(rank).ok_or(OscError::Rank)?;
        r.drift.update(ps as i32);
        let growth = osc_ruler.taps(ps as i32 - r.trained_ps as i32);
        let want = match sense {
            Sense::DqFollows => r.trained_taps + growth,
            Sense::StrobeLeads => r.trained_taps - growth,
        };
        if (want - r.taps).abs() >= threshold {
            r.taps = want;
            r.taps_drift.update(want);
            Ok(Some(want))
        } else {
            Ok(None)
        }
    }
}

/// One tracking slot.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Slot {
    /// The rank whose oscillator is read and whose write delay may change.
    pub rank: u8,
    /// Traffic may continue on the other ranks meanwhile; with one rank,
    /// the change waits for an idle window (refresh) instead.
    pub needs_idle: bool,
}

/// The `tick`-th tracking slot over `ranks` ranks (round robin).
pub const fn schedule(ranks: u8, tick: u32) -> Slot {
    let n = if ranks == 0 { 1 } else { ranks };
    Slot {
        rank: (tick % n as u32) as u8,
        needs_idle: n == 1,
    }
}

/// Whether a dummy read is due: no read strobe since `last_read_us` for
/// [`DUMMY_READ_INTERVAL_US`] (or `interval_us` if smaller and non-zero).
pub const fn dummy_read_due(last_read_us: u64, now_us: u64, interval_us: u64) -> bool {
    let i = if interval_us == 0 || interval_us > DUMMY_READ_INTERVAL_US {
        DUMMY_READ_INTERVAL_US
    } else {
        interval_us
    };
    now_us.wrapping_sub(last_read_us) >= i
}
