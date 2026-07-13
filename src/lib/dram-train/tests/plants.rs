// SPDX-License-Identifier: GPL-2.0-only
//! Acceptance bench for the v1.0.0 defaults: recorded-data replays and
//! adversarial plant models (seeded, property style, no extra crates). The
//! defaults must give a correct decision or an explicit error on every one.
//!
//! Plants: dead knob, aliased axis, sticky status bits, correlated
//! within-launch noise, periodic WL curves, +-1 step noise at about 8 % per
//! sample. Replays: R2S b0/f profiles, A7A m10/m11 WL curves and m11 CA maps
//! (the same data as the orchestrator's prior-art-dram-training-20261014
//! old-vs-new scripts).

#[path = "recorded/fixtures.rs"]
mod rec;

use dram_train::edge::{self, decide_explain, DecidePolicy, Polarity, Verdict};
use dram_train::eye::{center, center_legacy};
use dram_train::io::mock::Mock;
use dram_train::launch::{bounded_launch, LaunchError};
use dram_train::sanity::{self, SanityError, Saturation};
use dram_train::vref_sweep::{self, ratio_bounds, sweep_axis, Sweep, SweepError};

/// xorshift64*, seeded per test.
struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        self.0.wrapping_mul(0x2545_f491_4f6c_dd1d)
    }
    fn below(&mut self, n: u64) -> u64 {
        self.next() % n
    }
    fn chance(&mut self, p_permille: u64) -> bool {
        self.below(1000) < p_permille
    }
}

/// A Vref axis answering from a table, with an optional per-sample noise.
struct Table<'a> {
    t: &'a [u8],
    at: usize,
    rng: Option<(Rng, u64)>,
    lanes: [usize; 4],
}
impl vref_sweep::Axis<4> for Table<'_> {
    type Error = ();
    fn ranks(&self) -> u32 {
        1
    }
    fn set(&mut self, i: usize) {
        self.at = i;
    }
    fn run(&mut self, _: u32) -> Result<[u8; 4], ()> {
        let mut w = self.t[self.at];
        if let Some((r, p)) = self.rng.as_mut() {
            if r.chance(*p) {
                w = if r.below(2) == 0 {
                    w.saturating_add(1)
                } else {
                    w.saturating_sub(1)
                };
            }
        }
        // Lane 0 carries the worst value; the others are wider. `lanes`
        // permutes them.
        let v = [w, w + 1, w + 2, w + 3];
        Ok([
            v[self.lanes[0]],
            v[self.lanes[1]],
            v[self.lanes[2]],
            v[self.lanes[3]],
        ])
    }
}

fn full_axis() -> Sweep {
    Sweep {
        start: 0,
        stop_after: usize::MAX,
        min_window: 1,
    }
}

// --- Replays -------------------------------------------------------------------

#[test]
fn replay_r2s_aliased_write_axis_is_an_error() {
    // As recorded (81 points, 51-80 a repeat of 21-50): measured in full, the
    // default refuses with AliasedAxis.
    for prof in [
        &rec::R2S_FP0_WRITE,
        &rec::R2S_FP1_WRITE,
        &rec::R2S_FP2_WRITE,
    ] {
        let mut p = [None; 81];
        let r = sweep_axis(
            &mut Table {
                t: prof,
                at: 0,
                rng: None,
                lanes: [0, 1, 2, 3],
            },
            &mut p,
            &full_axis(),
        );
        match r {
            Err(SweepError::Sanity(SanityError::AliasedAxis { a, b, len })) => {
                assert!(b >= 51 && len >= 8 && b - a == 30, "{a} {b} {len}");
            }
            other => panic!("{other:?}"),
        }
    }
    // The corrected axis (0-50) decides, on the peak plateau.
    for (prof, want) in [
        (&rec::R2S_FP0_WRITE[..51], (22, 23)),
        (&rec::R2S_FP1_WRITE[..51], (25, 16)),
        (&rec::R2S_FP2_WRITE[..51], (23, 9)),
    ] {
        let mut p = vec![None; 51];
        let o = sweep_axis(
            &mut Table {
                t: prof,
                at: 0,
                rng: None,
                lanes: [0, 1, 2, 3],
            },
            &mut p,
            &full_axis(),
        )
        .unwrap();
        assert_eq!((o.point, o.worst), want);
    }
}

#[test]
fn replay_r2s_tables_raise_no_false_alarm() {
    let tables: [&[u8]; 9] = [
        &rec::R2S_FP0_READ,
        &rec::R2S_FP1_READ,
        &rec::R2S_FP2_READ,
        &rec::B0_FP0_RX.0,
        &rec::B0_FP0_TX.0,
        &rec::B0_FP1_RX.0,
        &rec::B0_FP1_TX.0,
        &rec::B0_FP2_RX.0,
        &rec::B0_FP2_TX.0,
    ];
    for t in tables {
        let mut p = vec![None; t.len()];
        let o = sweep_axis(
            &mut Table {
                t,
                at: 0,
                rng: None,
                lanes: [0, 1, 2, 3],
            },
            &mut p,
            &full_axis(),
        )
        .unwrap_or_else(|e| panic!("{t:?}: {e:?}"));
        assert_eq!(o.worst, *t.iter().max().unwrap(), "{t:?}");
    }
}

#[test]
fn replay_a7a_wl_m10_m11() {
    // m10: every curve flat zero; m11: the recorded curves.
    let m10 = [0u16; 84];
    let (d, v) = decide_explain(
        &m10,
        16,
        Some(512),
        Polarity::FailToPass,
        &DecidePolicy::DEFAULT,
    );
    assert_eq!(
        (d.delay, v),
        (None, Verdict::Saturated(Saturation::AllPass))
    );
    assert_eq!(
        sanity::curve(&m10),
        Err(SanityError::Saturated(Saturation::AllPass))
    );
    for &(name, err, counter, _, _) in &rec::A7A_WL {
        let (d, v) = decide_explain(
            err,
            16,
            counter,
            Polarity::FailToPass,
            &DecidePolicy::DEFAULT,
        );
        assert_eq!(d.delay, None, "{name}");
        assert!(v.error().is_some(), "{name}: {v:?}");
    }
}

#[test]
fn replay_a7a_ca_maps_pass_the_ui_gate() {
    let (ui, step) = (416_667u64, 26_042u64);
    for &(block, bit, map, _, _) in &rec::A7A_CBT {
        assert!(
            dram_train::window::min_run_ui(map, 128, true, 500, ui, step).is_some(),
            "CBT {block} bit {bit}"
        );
    }
}

// --- Plants --------------------------------------------------------------------

#[test]
fn plant_dead_knob() {
    let mut r = Rng(0x0dead);
    for _ in 0..500 {
        // Vref axis with no response: a constant width over the whole axis.
        let w = 1 + r.below(30) as u8;
        let n = [16usize, 32, 81][r.below(3) as usize];
        let t = vec![w; n];
        let mut p = vec![None; n];
        let res = sweep_axis(
            &mut Table {
                t: &t,
                at: 0,
                rng: None,
                lanes: [0, 1, 2, 3],
            },
            &mut p,
            &full_axis(),
        );
        assert_eq!(res, Err(SweepError::Sanity(SanityError::DeadKnob)));
        // Pass/fail knob: the same outcome distribution at both ends.
        let fail = r.chance(500);
        let k = 3 + r.below(4) as usize;
        let lo = vec![!fail; k];
        let hi = vec![!fail; k];
        if fail {
            assert_eq!(sanity::two_point(&lo, &hi), Err(SanityError::DeadKnob));
        } else {
            assert_eq!(sanity::two_point(&lo, &hi), Ok(()));
        }
        // An error curve independent of the delay, with correlated
        // whole-launch flips, 3 launches summed per point: never a decision.
        let base = if fail { 32u16 } else { 0 };
        let q = [10u64, 50, 180][r.below(3) as usize];
        let err: Vec<u16> = (0..84)
            .map(|_| {
                (0..3)
                    .map(|_| if r.chance(q) { 32 - base } else { base })
                    .sum()
            })
            .collect();
        let (d, v) = decide_explain(
            &err,
            16,
            Some(512),
            Polarity::FailToPass,
            &DecidePolicy {
                full_scale: Some(96),
                ..DecidePolicy::DEFAULT
            },
        );
        assert_eq!(d.delay, None, "q {q}: {v:?} {err:?}");
    }
}

#[test]
fn plant_aliased_axis() {
    let mut r = Rng(0xa11a5);
    let mut errors = 0;
    for _ in 0..300 {
        // A unimodal eye on n real points, then a copy of `len` of them.
        let n = 40 + r.below(20) as usize;
        let peak = 10 + r.below(n as u64 - 20) as usize;
        let top = 8 + r.below(16) as u8;
        let real: Vec<u8> = (0..n)
            .map(|i| top.saturating_sub((i.abs_diff(peak) / 3) as u8))
            .collect();
        let len = 12 + r.below(10) as usize;
        let from = peak.saturating_sub(len / 2).min(n - len);
        let mut t = real.clone();
        t.extend_from_slice(&real[from..from + len]);
        let mut p = vec![None; t.len()];
        match sweep_axis(
            &mut Table {
                t: &t,
                at: 0,
                rng: None,
                lanes: [0, 1, 2, 3],
            },
            &mut p,
            &full_axis(),
        ) {
            Err(SweepError::Sanity(SanityError::AliasedAxis { .. })) => errors += 1,
            Ok(o) => assert!(o.point < n, "decision in the copy: {} of {n}", o.point),
            Err(e) => panic!("{e:?}"),
        }
    }
    assert!(errors > 250, "{errors} of 300 flagged");
}

#[test]
fn plant_sticky_status_bits() {
    let mut io = Mock::<4>::default();
    // Done stuck at 1 and no witness of a run: refused.
    let l = bounded_launch(&mut io, 100, |_| {}, |_| true, |_| false, |_| {});
    assert_eq!(l.result, Err(LaunchError::NoBusyObserved));
    // Done stuck at 0: a timeout, not a hang.
    let l = bounded_launch(&mut io, 100, |_| {}, |_| false, |_| true, |_| {});
    assert_eq!(l.result, Err(LaunchError::Timeout));
    // A sticky "saturated" engine result is a failure, not a value.
    assert_eq!(
        sanity::limits(0x3f, 0, 0x3f),
        Err(SanityError::Saturated(Saturation::AtLimit))
    );
    assert_eq!(sanity::limits(0x20, 0, 0x3f), Ok(()));
    // A mode register left at its training value.
    assert_eq!(
        sanity::mode("TrainingParam", 0x300, 0xffff, 0),
        Err(SanityError::ModeMismatch {
            what: "TrainingParam",
            got: 0x300,
            want: 0
        })
    );
}

/// A WL plant: fail-to-pass edges of a periodic phase sweep (half period
/// `half` points, phase `ph`), 32 compares per launch that all flip
/// together with probability q (per mille), `launches` summed per point.
fn wl_curve(r: &mut Rng, ph: usize, half: usize, q: u64, launches: usize) -> Vec<u16> {
    (0..84)
        .map(|i| {
            let fail = ((i + ph) / half).is_multiple_of(2);
            (0..launches)
                .map(|_| {
                    let f = fail ^ r.chance(q);
                    if f {
                        32
                    } else {
                        0
                    }
                })
                .sum()
        })
        .collect()
}

fn true_edges(ph: usize, half: usize) -> Vec<u16> {
    (1..84)
        .filter(|&i| {
            ((i - 1 + ph) / half).is_multiple_of(2) && !((i + ph) / half).is_multiple_of(2)
        })
        .map(|i| (i as u16) * 16 - 8)
        .collect()
}

#[test]
fn plant_periodic_wl_and_correlated_noise() {
    let mut r = Rng(0x9e7);
    let (mut decided, mut wrong) = (0, 0);
    for trial in 0..4000 {
        let half = 16;
        let ph = r.below(32) as usize;
        let q = [0u64, 10, 50][trial % 3];
        let err = wl_curve(&mut r, ph, half, q, 3);
        let edges = true_edges(ph, half);
        let pol = DecidePolicy {
            full_scale: Some(96),
            ..DecidePolicy::DEFAULT
        };
        let (d, v) = decide_explain(&err, 16, None, Polarity::FailToPass, &pol);
        match d.delay {
            Some(x) => {
                decided += 1;
                if !edges.iter().any(|e| e.abs_diff(x) <= 16) {
                    wrong += 1;
                }
            }
            None => assert!(v.error().is_some()),
        }
        if q == 0 && !edges.is_empty() {
            // A clean curve with a stable edge is always decided.
            let first = edges[0] as usize / 16;
            if first >= 4 && first + 4 <= 84 {
                assert!(d.delay.is_some(), "clean curve refused: ph {ph}");
            }
        }
    }
    assert_eq!(wrong, 0, "{wrong} wrong of {decided}");
    // q = 0 and 1 %: nearly all decided; 5 %: often refused as noisy (14 %
    // of the points have mixed launches), never wrong.
    assert!(decided > 2600, "{decided} decided");
    // Voting over the 32 compares of one launch adds nothing: a single
    // launch with whole-launch flips is either 0 or 32 everywhere.
    let one = wl_curve(&mut r, 3, 16, 50, 1);
    assert!(one.iter().all(|&e| e == 0 || e == 32));
}

#[test]
fn plant_one_step_noise_at_8_percent() {
    // Truth profiles; the measured width is truth +-1 with 8 % per sample.
    let truths: [&[u8]; 6] = [
        &rec::R2S_FP0_WRITE[..51],
        &rec::R2S_FP2_WRITE[..51],
        &rec::R2S_FP2_READ,
        &rec::B0_FP0_RX.0,
        &rec::B0_FP2_TX.0,
        &rec::B0_FP2_RX.0,
    ];
    let mut r = Rng(0x8);
    for t in truths {
        let peak = *t.iter().max().unwrap();
        let (mut new_sum, mut old_sum, mut two_below) = (0u64, 0u64, 0);
        let trials = 2000;
        for _ in 0..trials {
            let m: Vec<Option<u8>> = t
                .iter()
                .map(|&w| {
                    Some(if r.chance(80) {
                        if r.below(2) == 0 {
                            w.saturating_add(1)
                        } else {
                            w.saturating_sub(1)
                        }
                    } else {
                        w
                    })
                })
                .collect();
            let n = center(&m).unwrap();
            let o = center_legacy(&m).unwrap();
            // The pick lies in the measured open region.
            assert!(m[n.center].unwrap() > 0);
            new_sum += t[n.center] as u64;
            old_sum += t[o.center] as u64;
            if t[n.center] + 2 <= peak {
                two_below += 1;
            }
        }
        let (new_mean, old_mean) = (
            new_sum as f64 / trials as f64,
            old_sum as f64 / trials as f64,
        );
        assert!(
            two_below * 200 <= trials,
            "{two_below} picks 2+ below the peak on {t:?}"
        );
        assert!(
            new_mean + 0.3 >= old_mean,
            "{t:?}: new {new_mean:.2} old {old_mean:.2}"
        );
    }
}

#[test]
fn properties_bounds_and_lane_order() {
    let mut r = Rng(0x1a7e);
    for _ in 0..500 {
        let n = 16 + r.below(70) as usize;
        let peak = r.below(n as u64) as usize;
        let top = 2 + r.below(25) as u8;
        let t: Vec<u8> = (0..n)
            .map(|i| top.saturating_sub((i.abs_diff(peak) / (1 + r.below(4) as usize)) as u8))
            .collect();
        // ratio_bounds never measures a point twice or beyond the axis.
        let mut p = vec![None; n];
        let mut count = 0;
        let start = r.below(n as u64) as usize;
        let _ = ratio_bounds(&mut p, start, 900, 2, |i| {
            count += 1;
            Ok::<u8, ()>(t[i])
        });
        assert!(count <= n);
        // The result does not depend on the order of the lanes.
        let cfg = Sweep {
            start,
            stop_after: 3,
            min_window: 0,
        };
        let mut a = vec![None; n];
        let mut b = vec![None; n];
        let ra = sweep_axis(
            &mut Table {
                t: &t,
                at: 0,
                rng: None,
                lanes: [0, 1, 2, 3],
            },
            &mut a,
            &cfg,
        );
        let rb = sweep_axis(
            &mut Table {
                t: &t,
                at: 0,
                rng: None,
                lanes: [3, 1, 0, 2],
            },
            &mut b,
            &cfg,
        );
        assert_eq!(
            ra.map(|o| (o.point, o.worst)),
            rb.map(|o| (o.point, o.worst))
        );
        assert_eq!(a, b);
    }
    let _ = edge::vote(1, 1, 1);
}
