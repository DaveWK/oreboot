// SPDX-License-Identifier: GPL-2.0-only
//! The live-readback contract on the Vref axis (K1 own training, the A7A
//! lesson of m13): the sweep never runs the engine at a point the axis did
//! not confirm live, and an axis without a readback says so in the result.
use dram_train::vref_sweep::{
    sweep_axis_legacy, sweep_axis_with, Axis, Live, Liveness, Sweep, SweepError, SweepPolicy,
};

/// A Vref knob with a written register and a live copy (the DRAM's MR14 as
/// MRR returns it, or the PHY's Vref DAC).
struct Vref {
    written: usize,
    live: usize,
    /// The write reaches the live copy.
    takes: bool,
    /// The axis can read the live copy.
    readback: bool,
    runs: u32,
    sets: u32,
}
impl Axis<4> for Vref {
    type Error = ();
    fn ranks(&self) -> u32 {
        1
    }
    fn set(&mut self, i: usize) {
        self.sets += 1;
        self.written = i;
        if self.takes {
            self.live = i;
        }
    }
    fn verify_live(&mut self, i: usize) -> Result<Live, ()> {
        Ok(if !self.readback {
            Live::Unverifiable
        } else if self.live == i {
            Live::Live
        } else {
            Live::Stale
        })
    }
    fn run(&mut self, _: u32) -> Result<[u8; 4], ()> {
        self.runs += 1;
        // The eye is at the LIVE Vref.
        let w = match self.live {
            6..=14 => 10,
            4..=5 | 15..=16 => 6,
            _ => 0,
        };
        Ok([w; 4])
    }
}
fn vref(takes: bool, readback: bool) -> Vref {
    Vref {
        written: 10,
        live: 10,
        takes,
        readback,
        runs: 0,
        sets: 0,
    }
}
fn cfg() -> Sweep {
    Sweep {
        start: 10,
        stop_after: 2,
        min_window: 4,
    }
}

#[test]
fn every_axis_move_is_confirmed_when_the_knob_takes() {
    let mut a = vref(true, true);
    let mut profile = [None; 21];
    let (o, _) = sweep_axis_with(&mut a, &mut profile, &cfg(), &SweepPolicy::DEFAULT).unwrap();
    assert_eq!(o.point, 10);
    assert_eq!(
        o.live,
        Liveness {
            applied: a.sets,
            live: a.sets,
            unverifiable: 0
        }
    );
    assert!(o.live.all_verified());
}

#[test]
fn a_write_that_does_not_take_stops_before_the_engine_runs() {
    // The written register follows every set; the live Vref stays at 10.
    // The first move goes to the start point 10 itself, which is live by
    // luck; the next point is not.
    let mut a = vref(false, true);
    let mut profile = [None; 21];
    let e = sweep_axis_with(&mut a, &mut profile, &cfg(), &SweepPolicy::DEFAULT).unwrap_err();
    let SweepError::NotLive { point } = e else {
        panic!("{e:?}")
    };
    assert_ne!(point, 10, "the stale point is named");
    assert_eq!(
        a.runs, 1,
        "the engine ran only at the one point that was live"
    );
    // The legacy sweep obeys the same contract.
    let mut a = vref(false, true);
    let e = sweep_axis_legacy(&mut a, &mut profile, &cfg()).unwrap_err();
    assert!(matches!(e, SweepError::NotLive { .. }), "{e:?}");
    assert_eq!(a.runs, 1);
}

#[test]
fn an_axis_without_readback_is_declared() {
    let mut a = vref(true, false);
    let mut profile = [None; 21];
    let (o, _) = sweep_axis_with(&mut a, &mut profile, &cfg(), &SweepPolicy::DEFAULT).unwrap();
    assert_eq!(o.live.applied, a.sets);
    assert_eq!(o.live.unverifiable, a.sets);
    assert_eq!(o.live.live, 0);
    assert_eq!(
        format!("{}", o.live),
        format!("points verified live: 0/{} (knob not verifiable)", a.sets)
    );
    // Without a readback the write that does not take is NOT caught: the
    // sweep reports a flat profile around the live point instead (the
    // sanity checks may still refuse it; here the knob check fires).
    let mut a = vref(false, false);
    let r = sweep_axis_with(&mut a, &mut profile, &cfg(), &SweepPolicy::DEFAULT);
    assert!(matches!(r, Err(SweepError::Sanity(_))), "{r:?}");
}
