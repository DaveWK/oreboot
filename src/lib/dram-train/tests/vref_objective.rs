// SPDX-License-Identifier: GPL-2.0-only
//! REQ-K1-03: `Objective::MaxMinThenSum` on the R2S fp2 write axis, from the
//! recorded Oct 15 boots (orchestrator serial-logs/r2s/20261015-*):
//!
//! - the m2 own-training worst-window profile (m2-own-retry.log): 9 on
//!   points 14-33 (MR14 0x0e-0x21), 8 on 11-13 and 34-36;
//! - the per-DQ write windows each boot logged at the MR14 it chose
//!   (`fp2 write-win`), the latest boot per MR14: 0x15 b1-baseline, 0x17
//!   m2-own-retry, 0x18 m2-own, 0x19 m2-return, 0x1a hw1-A, 0x1c
//!   m1-baseline, 0x1d g-warm1.
//!
//! The m2 boot is modelled as those recorded lane windows, each lowered so
//! the point's worst is m2's (one lane at the profile's worst), and every
//! unrecorded point as all lanes at the profile's worst. The axis index is
//! the MR14 code (K1 `tx_axis_mr14`).

use dram_train::sanity::SanityError;
use dram_train::vref_sweep::{
    sweep_axis_sums, sweep_axis_with, Axis, Objective, Sweep, SweepError, SweepPolicy,
};

const POINTS: usize = 41;

fn m2_worst(i: usize) -> u8 {
    match i {
        14..=33 => 9,
        11..=13 | 34..=36 => 8,
        _ => 7,
    }
}

/// (MR14, per-DQ write windows) as logged.
const RECORDED: [(usize, &str); 7] = [
    (0x15, "11 10 10 10 10 10 9 11 10 10 10 10 10 10 10 10 10 10 10 10 9 10 9 11 9 9 10 9 10 9 10 10"),
    (0x17, "11 10 10 10 10 10 10 11 10 10 10 10 10 11 10 10 11 10 10 10 9 10 10 11 9 10 10 10 11 9 10 10"),
    (0x18, "11 10 11 10 10 10 10 11 10 10 10 10 10 11 10 10 10 10 10 10 9 11 10 11 9 9 10 9 11 9 10 10"),
    (0x19, "11 10 11 11 11 10 10 11 10 10 10 11 10 11 10 11 11 10 10 10 10 10 10 11 9 10 10 10 11 9 10 10"),
    (0x1a, "11 10 11 11 11 10 10 10 10 10 11 11 11 11 10 11 11 10 10 10 10 10 10 11 9 10 11 10 11 9 10 11"),
    (0x1c, "11 10 11 10 10 10 11 11 10 10 10 10 11 11 11 10 10 10 10 10 10 10 10 10 10 11 10 11 10 10 10 10"),
    (0x1d, "11 10 11 10 10 10 11 11 10 10 10 10 10 11 11 10 10 10 10 10 10 10 10 10 10 10 10 11 10 10 10 10"),
];

fn lanes(i: usize) -> [u8; 32] {
    let w = m2_worst(i);
    match RECORDED.iter().find(|(m, _)| *m == i) {
        Some((_, text)) => {
            let mut l = [0u8; 32];
            for (d, v) in l.iter_mut().zip(text.split_whitespace()) {
                *d = v.parse().unwrap();
            }
            // This boot's worst at the point: lower the weakest lane to the
            // profile's worst where the recorded lanes are all wider.
            let min = *l.iter().min().unwrap();
            if min > w {
                let k = l.iter().position(|&x| x == min).unwrap();
                l[k] = w;
            }
            l
        }
        None => [w; 32],
    }
}

struct Write {
    point: usize,
    runs: u32,
}
impl Axis<32> for Write {
    type Error = ();
    fn ranks(&self) -> u32 {
        1
    }
    fn set(&mut self, i: usize) {
        self.point = i;
    }
    fn run(&mut self, _rank: u32) -> Result<[u8; 32], ()> {
        self.runs += 1;
        Ok(lanes(self.point))
    }
}

fn cfg() -> Sweep {
    // K1: start at MR14 0x19, stop after 3 points below the ratio.
    Sweep {
        start: 0x19,
        stop_after: 3,
        min_window: 8,
    }
}

fn sum(i: usize) -> u16 {
    lanes(i).iter().map(|&x| u16::from(x)).sum()
}

#[test]
fn the_default_centres_on_the_flat_plateau() {
    let mut profile = [None; POINTS];
    let mut a = Write { point: 0, runs: 0 };
    let (o, _) = sweep_axis_with(&mut a, &mut profile, &cfg(), &SweepPolicy::DEFAULT).unwrap();
    assert_eq!((o.point, o.worst), (0x17, 9), "m2 chose 23 (MR14 0x17)");
}

#[test]
fn max_min_then_sum_moves_to_the_widest_lanes() {
    let policy = SweepPolicy::DEFAULT.with_objective(Objective::MaxMinThenSum);
    let mut profile = [None; POINTS];
    let mut sums = [None; POINTS];
    let mut a = Write { point: 0, runs: 0 };
    let (o, _) = sweep_axis_sums(&mut a, &mut profile, &mut sums, &cfg(), &policy).unwrap();
    // Still on the plateau of the largest worst window...
    assert_eq!(profile[o.point], Some(9));
    // ... at its largest lane sum, in the MR14 0x1a-0x1d region where the
    // recorded boots' lanes are widest (0x1c/0x1d gave worst 10 on every
    // boot that chose them), not at 0x17.
    assert!((0x1a..=0x1d).contains(&o.point), "{:#x}", o.point);
    assert!(sum(o.point) > sum(0x17) + 4);
    for i in 14..=33 {
        if let Some(s) = sums[i] {
            assert!(s <= sums[o.point].unwrap(), "point {i}");
        }
    }
    // The sums the boards log: every measured point has one.
    for i in 0..POINTS {
        assert_eq!(profile[i].is_some(), sums[i].is_some(), "point {i}");
    }
    // With the plain record of each MR14 (no lane lowered), 0x1a carries
    // the single largest sum of one boot (332) ahead of 0x1c (328): sums
    // alone do not separate 0x1a from 0x1c; repeats or the halving in
    // vref_select (P3-A1) settle that on the board.
    assert_eq!(o.point, 0x1a);
}

#[test]
fn the_default_is_unchanged_and_sums_are_only_recorded() {
    let mut p1 = [None; POINTS];
    let mut p2 = [None; POINTS];
    let mut sums = [None; POINTS];
    let mut a = Write { point: 0, runs: 0 };
    let mut b = Write { point: 0, runs: 0 };
    let r1 = sweep_axis_with(&mut a, &mut p1, &cfg(), &SweepPolicy::DEFAULT).unwrap();
    let r2 = sweep_axis_sums(&mut b, &mut p2, &mut sums, &cfg(), &SweepPolicy::DEFAULT).unwrap();
    assert_eq!((r1, p1, a.runs), (r2, p2, b.runs));
    assert_eq!(sums[0x17], Some(sum(0x17)));
}

#[test]
fn ties_go_to_the_point_nearest_the_centring_choice() {
    // A flat plateau with equal sums everywhere: the plateau centre.
    struct Flat(usize);
    impl Axis<4> for Flat {
        type Error = ();
        fn ranks(&self) -> u32 {
            2
        }
        fn set(&mut self, i: usize) {
            self.0 = i;
        }
        fn run(&mut self, _: u32) -> Result<[u8; 4], ()> {
            Ok(if (5..=15).contains(&self.0) {
                [6; 4]
            } else {
                [2; 4]
            })
        }
    }
    let policy = SweepPolicy::DEFAULT.with_objective(Objective::MaxMinThenSum);
    let (mut p, mut s) = ([None; 21], [None; 21]);
    let mut c = Sweep {
        start: 10,
        stop_after: usize::MAX,
        min_window: 1,
    };
    let (o, _) = sweep_axis_sums(&mut Flat(0), &mut p, &mut s, &c, &policy).unwrap();
    let (d, _) = sweep_axis_with(&mut Flat(0), &mut [None; 21], &c, &SweepPolicy::DEFAULT).unwrap();
    assert_eq!(o.point, d.point);
    // Without a sums buffer the objective cannot work: said so, not ignored.
    c.start = 10;
    assert!(matches!(
        sweep_axis_with(&mut Flat(0), &mut [None; 21], &c, &policy),
        Err(SweepError::Sanity(SanityError::ModeMismatch { .. }))
    ));
}
