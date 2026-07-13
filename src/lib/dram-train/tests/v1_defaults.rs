// SPDX-License-Identifier: GPL-2.0-only
//! The v1.0.0 defaults (plateau centring, ratio-bounded Vref search, voted
//! decisions, coarse-then-fine edges and margin scans) against recorded
//! hardware data and synthetic edge cases, next to the legacy entry points.

#[path = "recorded/fixtures.rs"]
mod rec;

use dram_train::edge::{self, coarse_fine, decide, decide_legacy, vote, vote_maps, Vote};
use dram_train::eye::{self, center, center_legacy, center_with, CenterError, Policy};
use dram_train::margin::{self, Direction, Granularity, Range, Request, Target};
use dram_train::result::{Axis, Drift, ErrorCode, ErrorRecord, Margin, StageLog, Status, Unit};
use dram_train::vref_sweep::{self, ratio_bounds, sweep_axis, sweep_axis_legacy, Sweep};
use dram_train::window::{intersect, longest_run, min_run};
use dram_train::wrlat::{self, FrameShift, NoMatchCause};

fn opt(w: &[u8]) -> Vec<Option<u8>> {
    w.iter().map(|&x| Some(x)).collect()
}

fn width_at(w: &[u8], i: usize) -> u8 {
    w[i]
}

// --- R2S f run (our K1 training, 2400 MT/s) ---------------------------------

#[test]
fn r2s_f_write_eyes_take_the_plateau() {
    for (name, prof, logged) in [
        ("fp0", &rec::R2S_FP0_WRITE[..], rec::R2S_FP0_WRITE_LOGGED),
        ("fp1", &rec::R2S_FP1_WRITE[..], rec::R2S_FP1_WRITE_LOGGED),
        ("fp2", &rec::R2S_FP2_WRITE[..], rec::R2S_FP2_WRITE_LOGGED),
    ] {
        let p = opt(prof);
        // The legacy centre is what the board logged: the scan midpoint.
        let old = center_legacy(&p).unwrap();
        assert_eq!((old.center, old.center_width), logged, "{name} legacy");
        let new = center(&p).unwrap();
        let peak = *prof.iter().max().unwrap();
        assert!(
            new.center_width >= peak - 1 && new.center_width > old.center_width,
            "{name}: new {} (width {}), legacy {} (width {}), peak {peak}",
            new.center,
            new.center_width,
            old.center,
            old.center_width
        );
        let c = center_with(&p, Policy::DEFAULT).unwrap();
        assert!(c.edge_outside_scan && c.plateau, "{name}");
    }
    // fp2 is the REQ-K1-01 case: width 9 instead of the logged 8, in the
    // middle of the longest run of 9s (15..=32).
    let e = center(&opt(&rec::R2S_FP2_WRITE)).unwrap();
    assert_eq!((e.center, e.center_width), (23, 9));
    assert_eq!((e.lo, e.hi), (15, 32));
    // The same on the corrected axis (points 0..=50; 51..=80 re-measured
    // 21..=50 because of the MR14 range bit): fp0 22 (23), fp1 25 (16),
    // fp2 23 (9).
    for (prof, want) in [
        (&rec::R2S_FP0_WRITE[..51], (22, 23)),
        (&rec::R2S_FP1_WRITE[..51], (25, 16)),
        (&rec::R2S_FP2_WRITE[..51], (23, 9)),
    ] {
        let e = center(&opt(prof)).unwrap();
        assert_eq!((e.center, e.center_width), want);
    }
}

#[test]
fn r2s_f_read_eyes_stay_on_the_peak() {
    for (name, prof, logged) in [
        ("fp0", &rec::R2S_FP0_READ[..], rec::R2S_FP0_READ_LOGGED),
        ("fp1", &rec::R2S_FP1_READ[..], rec::R2S_FP1_READ_LOGGED),
        ("fp2", &rec::R2S_FP2_READ[..], rec::R2S_FP2_READ_LOGGED),
    ] {
        let p = opt(prof);
        let old = center_legacy(&p).unwrap();
        assert_eq!((old.center, old.center_width), logged, "{name} legacy");
        let new = center(&p).unwrap();
        assert!(
            new.center_width >= old.center_width
                && new.center_width >= *prof.iter().max().unwrap() - 1,
            "{name}: new {} width {}",
            new.center,
            new.center_width
        );
    }
}

#[test]
fn b0_rx_profile_centres_on_the_plateau() {
    let prof = [8u8, 9, 9, 10, 10, 10, 10, 10, 9, 9, 8, 7, 7, 4, 0, 0];
    let e = center(&opt(&prof)).unwrap();
    assert!((5..=6).contains(&e.center), "{e:?}");
    assert_eq!(width_at(&prof, e.center), 10);
}

#[test]
fn ratio_search_finds_the_same_region_with_fewer_points() {
    for prof in [
        &rec::R2S_FP0_WRITE[..],
        &rec::R2S_FP2_WRITE[..],
        &rec::R2S_FP2_READ[..],
    ] {
        let mut p = vec![None; prof.len()];
        let b = ratio_bounds(&mut p, 20.min(prof.len() / 2), 500, 1, |i| {
            Ok::<u8, ()>(prof[i])
        })
        .unwrap();
        let full = center_legacy(&opt(prof)).unwrap();
        assert_eq!((b.lb, b.ub), (full.lo, full.hi));
        assert_eq!(b.w_max, full.peak_width);
    }
    // A closed tail is not measured beyond the first point under the bound.
    let prof = [0u8, 0, 0, 2, 6, 9, 10, 9, 6, 2, 0, 0, 0, 0, 0, 0];
    let mut p = [None; 16];
    let b = ratio_bounds(&mut p, 6, 500, 1, |i| Ok::<u8, ()>(prof[i])).unwrap();
    assert_eq!((b.lb, b.ub, b.center, b.peak), (4, 8, 6, 6));
    assert!(b.measured < prof.len());
    // A larger window found on the way moves the bounds.
    let prof = [3u8, 4, 5, 6, 20, 6, 5, 4];
    let mut p = [None; 8];
    let b = ratio_bounds(&mut p, 0, 500, 2, |i| Ok::<u8, ()>(prof[i])).unwrap();
    assert_eq!((b.peak, b.w_max, b.lb, b.ub), (4, 20, 4, 4));
}

// --- Synthetic edge cases ----------------------------------------------------

#[test]
fn synthetic_edge_cases() {
    // Flat: the plateau is the whole scan; the middle.
    let e = center(&opt(&[7; 11])).unwrap();
    assert_eq!(e.center, 5);
    // Plateau touching the start: centre of the plateau, not of the scan.
    let p = [10u8, 10, 10, 10, 9, 8, 7, 7, 6, 6];
    let e = center(&opt(&p)).unwrap();
    assert_eq!(e.center, 1);
    assert_eq!(center_legacy(&opt(&p)).unwrap().center, 4);
    // Plateau touching the end.
    let p = [6u8, 6, 7, 7, 8, 9, 10, 10, 10, 10];
    assert_eq!(center(&opt(&p)).unwrap().center, 7);
    // A narrow peak above a wide plateau: the peak run, not the plateau.
    let p = [9u8, 9, 9, 9, 10, 10, 10, 9, 9, 9, 9, 9, 9, 9];
    assert_eq!(center(&opt(&p)).unwrap().center, 5);
    // Noise +-1 on a plateau: stays inside it.
    let p = [5u8, 9, 10, 9, 10, 10, 9, 10, 9, 6, 5, 5];
    let e = center(&opt(&p)).unwrap();
    assert!((1..=8).contains(&e.center) && p[e.center] >= 9, "{e:?}");
    // All closed.
    assert_eq!(center(&opt(&[0; 8])), Err(eye::EyeError::Closed));
    // A single-point peak inside the scan: unchanged from legacy.
    let p = [0u8, 0, 1, 2, 9, 2, 1, 0, 0];
    assert_eq!(center(&opt(&p)), center_legacy(&opt(&p)));
    assert_eq!(center(&opt(&p)).unwrap().center, 4);
    // A single-point peak at the scan end.
    let p = [9u8, 3, 1, 0];
    assert_eq!(center(&opt(&p)).unwrap().center, 0);
    // require_edges reports the open end.
    let r = center_with(
        &opt(&[9, 9, 9, 4, 0]),
        Policy {
            require_edges: true,
            ..Policy::DEFAULT
        },
    );
    assert!(matches!(r, Err(CenterError::EdgeOutsideScan(_))));
}

#[test]
fn default_center_equals_legacy_when_both_edges_are_seen() {
    // Deterministic pseudo-random profiles with closed ends.
    let mut x: u32 = 0x1234_5678;
    for _ in 0..2000 {
        let mut p = [Some(0u8); 24];
        for w in p[2..22].iter_mut() {
            x ^= x << 13;
            x ^= x >> 17;
            x ^= x << 5;
            *w = Some((x % 20) as u8);
        }
        assert_eq!(center(&p), center_legacy(&p));
    }
}

// --- Sweep -------------------------------------------------------------------

struct Recorded<'a>(&'a [u8], usize);
impl vref_sweep::Axis<2> for Recorded<'_> {
    type Error = ();
    fn ranks(&self) -> u32 {
        1
    }
    fn set(&mut self, i: usize) {
        self.1 = i;
    }
    fn run(&mut self, _: u32) -> Result<[u8; 2], ()> {
        Ok([self.0[self.1]; 2])
    }
}

#[test]
fn sweep_default_on_the_r2s_profiles() {
    let cfg = Sweep {
        start: 20,
        stop_after: 3,
        min_window: 1,
    };
    let mut p = [None; 81];
    let new = sweep_axis(&mut Recorded(&rec::R2S_FP2_WRITE, 0), &mut p, &cfg).unwrap();
    let mut q = [None; 81];
    let old = sweep_axis_legacy(&mut Recorded(&rec::R2S_FP2_WRITE, 0), &mut q, &cfg).unwrap();
    assert_eq!((new.point, new.worst), (23, 9));
    assert_eq!((old.point, old.worst), (40, 8));
}

// --- A7A recorded data -------------------------------------------------------

#[test]
fn a7a_m11_cbt_maps_give_the_logged_windows() {
    for &(block, bit, map, start, len) in &rec::A7A_CBT {
        let w = longest_run(map, 128);
        assert_eq!(
            (w.start * 4, w.len * 4),
            (start, len),
            "CBT {block} bit {bit}"
        );
        assert!(min_run(map, 128, 10, true).is_some());
    }
    for &(block, start, len, c) in &rec::A7A_CBT_COMMON {
        let maps = rec::A7A_CBT.iter().filter(|r| r.0 == block).map(|r| r.2);
        let w = longest_run(intersect(maps, 128), 128);
        assert_eq!(
            (w.start * 4, w.len * 4, w.center(128) * 4),
            (start, len, c),
            "CBT {block}"
        );
    }
}

#[test]
fn a7a_m11_wl_noise_curves_are_rejected_by_default() {
    for &(name, err, counter, logged, agreed) in &rec::A7A_WL {
        let old = decide_legacy(err, 16, counter);
        assert_eq!(
            (old.delay, old.counter_agrees),
            (logged, agreed),
            "{name} legacy"
        );
        // m11 found no delay dependence in these sweeps: the default must not
        // turn them into a decision, with or without the counter.
        let new = decide(err, 16, counter);
        assert_eq!((new.delay, new.counter_agrees), (None, false), "{name}");
    }
}

#[test]
fn voted_decisions_on_clean_and_noisy_curves() {
    // A clean fail-to-pass curve: decided at the legacy point.
    let mut err = [32u16; 40];
    for e in err[17..].iter_mut() {
        *e = 0;
    }
    assert_eq!(
        decide(&err, 16, None).delay,
        decide_legacy(&err, 16, None).delay
    );
    assert_eq!(decide(&err, 16, None).delay, Some(17 * 16 - 8));
    // Noise points (half the samples failing) around the edge: the middle.
    err[16] = 16;
    err[17] = 16;
    assert_eq!(decide(&err, 16, None).delay, Some((15 + 18) * 16 / 2));
    // The counter is taken on a stable edge.
    assert_eq!(decide(&err, 16, Some(260)).delay, Some(260));
    // Too short a run before the edge: no decision.
    let err = [32u16, 32, 0, 0, 0, 0, 0, 0];
    assert_eq!(decide(&err, 16, None).delay, None);
    assert_eq!(vote(9, 10, 8), Vote::Pass);
    assert_eq!(vote(1, 10, 8), Vote::Fail);
    assert_eq!(vote(5, 10, 8), Vote::Noise);
    let (pass, noise) = vote_maps(&[10, 10, 5, 0], 10, 8);
    assert_eq!((pass, noise), (0b0011, 0b0100));
}

#[test]
fn coarse_then_fine_finds_the_exact_edge() {
    for edge_at in 1..100u32 {
        let mut probes = 0;
        let r = coarse_fine(0, 127, 10, 1, |i| {
            probes += 1;
            Ok::<bool, ()>(i < edge_at)
        })
        .unwrap();
        assert_eq!(r.edge, Some(edge_at));
        assert!(r.start_pass);
        assert!(probes <= 1 + 13 + 10 + 1, "{probes}");
    }
    // Downwards, and no edge.
    let r = coarse_fine(60, 0, 10, 0, |i| Ok::<bool, ()>(i > 33)).unwrap();
    assert_eq!(r.edge, Some(33));
    let r = coarse_fine(60, 127, 10, 0, |_| Ok::<bool, ()>(true)).unwrap();
    assert_eq!(r.edge, None);
    // A one-point glitch inside the bracket is skipped by the confirmation.
    let mut seen = std::collections::HashMap::new();
    let r = coarse_fine(0, 50, 10, 1, |i| {
        let n = seen.entry(i).or_insert(0);
        *n += 1;
        Ok::<bool, ()>(if i == 12 { *n > 1 } else { i < 15 })
    })
    .unwrap();
    assert_eq!(r.edge, Some(15));
}

// --- Margin scan -----------------------------------------------------------------

struct Eye2d {
    vref: i32,
    delay: i32,
    tests: u32,
    restored: bool,
}
impl Target for Eye2d {
    type Error = ();
    fn set_vref(&mut self, _: &Request, v: i32) -> Result<(), ()> {
        self.vref = v;
        Ok(())
    }
    fn set_delay(&mut self, _: &Request, d: i32) -> Result<(), ()> {
        self.delay = d;
        Ok(())
    }
    fn test(&mut self, _: &Request) -> Result<bool, ()> {
        self.tests += 1;
        // A diamond eye centred at delay 50, Vref 20.
        Ok((self.delay - 50).abs() + 2 * (self.vref - 20).abs() < 30)
    }
    fn restore(&mut self, _: &Request) -> Result<(), ()> {
        self.restored = true;
        Ok(())
    }
}

#[test]
fn margin_scan_default_matches_the_full_scan_with_fewer_tests() {
    let req = Request {
        dir: Direction::Write,
        unit: Granularity::Byte,
        ch: 0,
        rank: 0,
        select: 1,
        delay: Range {
            start: 0,
            step: 1,
            count: 100,
        },
        vref: Range {
            start: 10,
            step: 2,
            count: 11,
        },
        trained: 50,
        coarse: 10,
    };
    let mut a = [0u128; 11];
    let mut ta = Eye2d {
        vref: 0,
        delay: 0,
        tests: 0,
        restored: false,
    };
    let sa = margin::scan(&mut ta, &req, &mut a).unwrap();
    let mut b = [0u128; 11];
    let mut tb = Eye2d {
        vref: 0,
        delay: 0,
        tests: 0,
        restored: false,
    };
    let sb = margin::scan_full(&mut tb, &req, &mut b).unwrap();
    assert_eq!(a, b);
    assert_eq!(sa, sb);
    assert!(ta.restored && tb.restored);
    assert!(ta.tests * 2 < tb.tests, "{} vs {}", ta.tests, tb.tests);
    assert_eq!(sa.best_row, Some(5));
    let m = margin::row_margin(&req, &a, 5, Axis::WriteEye, Unit::Step).unwrap();
    assert_eq!((m.lo, m.hi, m.center), (21, 79, 50));
    assert_eq!(
        m.to_string(),
        "[margin] axis=write_eye ch=0 rank=0 lane=1 unit=step lo=21 hi=79 center=50"
    );
}

// --- Result schema and write latency ---------------------------------------------

#[test]
fn stage_log_and_drift() {
    let mut log = StageLog::<8>::new();
    assert!(log.plan(1, Some(Axis::WriteLeveling)) && log.plan(2, Some(Axis::ReadEye)));
    log.begin(1);
    log.done(1);
    log.begin(2);
    log.fail(ErrorRecord {
        stage: 2,
        code: ErrorCode::RangeRight,
        ch: 0,
        rank: 1,
        byte: 3,
        bit: Some(5),
    });
    assert_eq!(log.pointer().unwrap().status, Status::Failed);
    assert_eq!(log.error.unwrap().byte, 3);
    assert_eq!(
        log.entries()[0].to_string(),
        "[stage] id=1 axis=write_leveling status=done warn=0x0000"
    );
    let mut d = Drift::new(10);
    d.update(13);
    d.update(8);
    assert_eq!((d.min, d.max, d.excursion()), (8, 13, 3));
    let m = Margin {
        axis: Axis::ReadEye,
        ch: 1,
        rank: 0,
        lane: 2,
        unit: Unit::Ps,
        lo: -40,
        hi: 60,
        center: 10,
    };
    assert_eq!(m.width(), 101);
}

#[test]
fn write_latency_frame() {
    let mut buf = [0u8; 48];
    assert_eq!(wrlat::fill(&mut buf, 16), Some(48));
    assert_eq!(wrlat::frame_shift(&buf, 16), FrameShift::Aligned);
    // Two beats early: the frame shows up two beats before its slot.
    let mut early = [0u8; 48];
    early.copy_from_slice(&buf);
    early.rotate_left(2);
    assert_eq!(wrlat::frame_shift(&early, 16), FrameShift::Early(2));
    assert_eq!(FrameShift::cycles(2, 2), Some(1));
    let mut late = buf;
    late.rotate_right(4);
    assert_eq!(wrlat::frame_shift(&late, 16), FrameShift::Late(4));
    assert_eq!(wrlat::frame_shift(&[0u8; 48], 16), FrameShift::NoMatch);
    let r1 = [1u8, 2, 3];
    let r2 = [1u8, 2, 4];
    assert_eq!(
        wrlat::classify_no_match(&[&r1, &r1]),
        NoMatchCause::WritePath
    );
    assert_eq!(
        wrlat::classify_no_match(&[&r1, &r2]),
        NoMatchCause::ReadPath
    );
}

#[test]
fn legacy_policies_reproduce_the_old_entry_points() {
    let p = opt(&rec::R2S_FP2_WRITE);
    assert_eq!(
        center_with(&p, Policy::legacy()).unwrap().eye,
        center_legacy(&p).unwrap()
    );
    let _ = edge::DecidePolicy::DEFAULT;
}

// --- Replays from OLD-VS-NEW-COMPARISON.md ------------------------------------

#[test]
fn b0_vendor_tables() {
    // (table, NEW pick expected, legacy pick expected): NEW reaches the
    // vendor's best width on every table.
    for (name, (t, vendor, best), new_pick, old_pick) in [
        ("fp0 rx", rec::B0_FP0_RX, 6, 7),
        ("fp0 tx", rec::B0_FP0_TX, 3, 7),
        ("fp1 rx", rec::B0_FP1_RX, 6, 6),
        ("fp1 tx", rec::B0_FP1_TX, 6, 7),
        ("fp2 rx", rec::B0_FP2_RX, 5, 6),
        ("fp2 tx", rec::B0_FP2_TX, 8, 7),
    ] {
        let p = opt(&t);
        assert_eq!(t[vendor], best, "{name}");
        let e = center(&p).unwrap();
        assert_eq!((e.center, e.center_width), (new_pick, best), "{name} new");
        assert_eq!(center_legacy(&p).unwrap().center, old_pick, "{name} legacy");
    }
}

#[test]
fn f_read_single_point_peak_keeps_the_peak() {
    // fp2 read: 11 at code 6 only; the 1-tap band is 3..=7 and the widest of
    // its middle and neighbours is the peak.
    let e = center(&opt(&rec::R2S_FP2_READ)).unwrap();
    assert_eq!((e.center, e.center_width), (6, 11));
}

#[test]
fn k1_vref_search_cost_at_900_permille() {
    // Engine runs on the recorded K1 profiles with K1's stop rules (RX: the
    // whole axis, TX: 3 points), against the full 297 per boot.
    let mut runs = 0;
    for (prof, start, stop) in [
        (&rec::R2S_FP0_READ[..], 0usize, usize::MAX),
        (&rec::R2S_FP1_READ[..], 0, usize::MAX),
        (&rec::R2S_FP2_READ[..], 0, usize::MAX),
        (&rec::R2S_FP0_WRITE[..], 4, 3),
        (&rec::R2S_FP1_WRITE[..], 4, 3),
        (&rec::R2S_FP2_WRITE[..], 4, 3),
    ] {
        let mut p = vec![None; prof.len()];
        let cfg = Sweep {
            start,
            stop_after: stop,
            min_window: 1,
        };
        let mut a = Recorded(prof, 0);
        let o = sweep_axis(&mut a, &mut p, &cfg).unwrap();
        let measured = p.iter().filter(|w| w.is_some()).count();
        runs += measured + 1;
        let mut q = vec![None; prof.len()];
        let full = sweep_axis_legacy(&mut Recorded(prof, 0), &mut q, &cfg).unwrap();
        assert!(o.worst >= full.worst, "{} < {}", o.worst, full.worst);
    }
    assert!(runs < 200, "{runs} engine runs per boot");
}

#[test]
fn periodic_and_flipped_wl_curves() {
    // A clean phase sweep over about 2.6 periods: 5 transitions.
    let mut err = [0u16; 84];
    for (i, e) in err.iter_mut().enumerate() {
        *e = if (i + 8) % 32 < 16 { 32 } else { 0 };
    }
    let (d, v) = edge::decide_explain(
        &err,
        16,
        None,
        edge::Polarity::FailToPass,
        &edge::DecidePolicy::DEFAULT,
    );
    assert_eq!(v, edge::Verdict::Edge);
    assert_eq!(d.delay, Some((7 + 8) * 16 / 2));
    // With the counter near the second fail-to-pass edge, that one.
    let d = decide(&err, 16, Some(39 * 16));
    assert!(d.counter_agrees);
    // One isolated flip in a long run does not kill the curve.
    let mut flipped = err;
    flipped[20] = 32;
    let (d, v) = edge::decide_explain(
        &flipped,
        16,
        None,
        edge::Polarity::FailToPass,
        &edge::DecidePolicy::DEFAULT,
    );
    assert_eq!((d.delay, v), (Some((7 + 8) * 16 / 2), edge::Verdict::Edge));
    // The m11 noise curves: refused with a reason.
    for &(name, err, counter, _, _) in &rec::A7A_WL {
        let (d, v) = edge::decide_explain(
            err,
            16,
            counter,
            edge::Polarity::FailToPass,
            &edge::DecidePolicy::DEFAULT,
        );
        assert_eq!(d.delay, None, "{name}");
        assert!(
            matches!(v, edge::Verdict::Saturated(_) | edge::Verdict::Noisy),
            "{name}: {v:?}"
        );
    }
    // A window around the counter reads only that part of the curve.
    let mut noisy_far = err;
    for i in (60..84).step_by(2) {
        noisy_far[i] ^= 32;
    }
    let p = edge::DecidePolicy {
        window: Some(10 * 16),
        ..edge::DecidePolicy::DEFAULT
    };
    let (d, v) = edge::decide_explain(&noisy_far, 16, Some(120), edge::Polarity::FailToPass, &p);
    assert_eq!((d.delay, v), (Some(120), edge::Verdict::Counter));
    let (_, v) = edge::decide_explain(
        &noisy_far,
        16,
        Some(120),
        edge::Polarity::FailToPass,
        &edge::DecidePolicy::DEFAULT,
    );
    assert_eq!(v, edge::Verdict::Noisy);
}

#[test]
fn launch_votes() {
    let mut seq = [true, false, true].into_iter();
    assert_eq!(
        edge::vote_launches(3, 2, || Ok::<bool, ()>(seq.next().unwrap())),
        Ok(Vote::Pass)
    );
}

#[test]
fn ui_gates_on_the_a7a_ca_maps() {
    use dram_train::window::{gate_points, min_run_ui};
    // A733 pstate 1: UI 417 ps, CA common step 4 units of 6.5 ps = 26 ps.
    let (ui, step) = (416_667u64, 26_042u64);
    assert_eq!(gate_points(500, ui, step), 8);
    let literal_15 = rec::A7A_CBT
        .iter()
        .filter(|r| min_run(r.2, 128, 15, true).is_none())
        .count();
    assert_eq!(literal_15, 5);
    // Half a UI passes every recorded window.
    assert!(rec::A7A_CBT
        .iter()
        .all(|r| min_run_ui(r.2, 128, true, 500, ui, step).is_some()));
}
