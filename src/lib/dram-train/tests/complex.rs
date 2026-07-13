// SPDX-License-Identifier: GPL-2.0-only
//! P3-M1: the victim/aggressor pattern and the short-then-long complex
//! search against eye plants (the long pattern's eye inside the short one).

use dram_train::complex::{
    complex_edge, complex_window, prbs7, Complex, ComplexError, Length, LONG_BEATS, SEGMENT,
    SHORT_BEATS,
};

#[test]
fn every_lane_takes_a_turn_as_victim() {
    for lanes in [1u8, 4, 8, 9, 16] {
        for length in [Length::Short, Length::Long] {
            let p = Complex::new(lanes, length).unwrap();
            assert_eq!(p.len(), p.block() * lanes as usize);
            assert_eq!(p.block() % 32, 0, "whole BL32 blocks");
            let mask: u32 = if lanes == 16 {
                0xffff
            } else {
                (1 << lanes) - 1
            };
            for v in 0..lanes {
                let first = v as usize * p.block();
                assert_eq!(p.victim(first), v);
                // Odd mode: the victim opposes every aggressor on each beat.
                for t in 0..SEGMENT {
                    let w = u32::from(p.beat(first + t));
                    let vb = w >> v & 1;
                    let others = w & mask & !(1 << v);
                    let want = if vb == 1 { 0 } else { mask & !(1 << v) };
                    assert_eq!(others, want, "lanes {lanes} victim {v} beat {t}");
                    assert_eq!(w & !mask, 0);
                }
                // Every lane toggles in the block (no stuck lane in the data).
                let mut seen1 = 0u32;
                let mut seen0 = 0u32;
                for t in 0..p.block() {
                    let w = u32::from(p.beat(first + t));
                    seen1 |= w;
                    seen0 |= !w & mask;
                }
                assert_eq!((seen1, seen0), (mask, mask));
            }
        }
    }
    assert!(Complex::new(0, Length::Short).is_none());
    assert!(Complex::new(17, Length::Short).is_none());
    assert_eq!((SHORT_BEATS, LONG_BEATS), (32, 224));
}

#[test]
fn the_long_pattern_holds_lone_bits_runs_and_prbs7() {
    let p = Complex::new(8, Length::Long).unwrap();
    let v = 3u8;
    let base = v as usize * p.block();
    let bit = |t: usize| p.beat(base + t) >> v & 1;
    // Lone one: exactly one 1 in the second segment.
    assert_eq!((SEGMENT..2 * SEGMENT).filter(|&t| bit(t) == 1).count(), 1);
    // Lone zero: exactly one 0 in the fourth.
    assert_eq!(
        (3 * SEGMENT..4 * SEGMENT).filter(|&t| bit(t) == 0).count(),
        1
    );
    // Quiet victim under toggling aggressors.
    assert!((5 * SEGMENT..6 * SEGMENT).all(|t| bit(t) == 0));
    // PRBS7 on the victim, its complement on the aggressors.
    for k in 0..127 {
        let t = 6 * SEGMENT + k;
        assert_eq!(bit(t) == 1, prbs7(k));
        let agg = p.beat(base + t) >> ((v + 1) % 8) & 1;
        assert_eq!(agg == 1, !prbs7(k));
    }
    // The short pattern is the long one's first two segments.
    let s = Complex::new(8, Length::Short).unwrap();
    for t in 0..SHORT_BEATS {
        assert_eq!(s.beat(base / p.block() * s.block() + t), p.beat(base + t));
    }
}

#[test]
fn bus_fill_gives_each_byte_lane_another_victim() {
    let p = Complex::new(8, Length::Short).unwrap();
    let mut bus = vec![0u8; 4 * p.len()];
    assert_eq!(p.fill_bus(&mut bus, 4, 0), bus.len());
    for t in 0..p.len() {
        let victims: Vec<u8> = (0..4).map(|b| p.victim(t + b * p.block())).collect();
        let mut d = victims.clone();
        d.dedup();
        assert_eq!(d.len(), 4, "beat {t}: {victims:?}");
        assert_eq!(bus[4 * t], p.beat(t) as u8);
    }
    assert_eq!(p.fill_bus(&mut bus, 0, 0), 0);
    assert_eq!(
        Complex::new(9, Length::Short)
            .unwrap()
            .fill_bus(&mut bus, 4, 0),
        0
    );
}

/// An eye plant: the short pattern passes on [ls, rs], the long one on
/// [ll, rl] inside it; `flaky` points fail the long pattern once.
struct Eye {
    short: (u32, u32),
    long: (u32, u32),
    flaky: Vec<u32>,
    probes: Vec<(u32, Length)>,
}
impl Eye {
    fn new(short: (u32, u32), long: (u32, u32)) -> Self {
        Self {
            short,
            long,
            flaky: Vec::new(),
            probes: Vec::new(),
        }
    }
    fn probe(&mut self, p: u32, l: Length) -> Result<bool, ()> {
        self.probes.push((p, l));
        let (a, b) = match l {
            Length::Short => self.short,
            Length::Long => self.long,
        };
        if l == Length::Long {
            if let Some(i) = self.flaky.iter().position(|&f| f == p) {
                self.flaky.remove(i);
                return Ok(false);
            }
        }
        Ok(a <= p && p <= b)
    }
}

#[test]
fn the_window_is_the_long_patterns_eye() {
    for (short, long, start) in [
        ((20, 180), (35, 160), 100),
        ((20, 180), (21, 179), 100),
        ((20, 180), (95, 104), 100), // long eye much narrower than a coarse step
        ((0, 255), (3, 250), 128),   // limits inside the short eye
        ((50, 60), (50, 60), 55),
    ] {
        let mut eye = Eye::new(short, long);
        let w = complex_window(start, 0, 255, 10, 2, |p, l| eye.probe(p, l)).unwrap();
        assert_eq!(
            (w.left.last_pass, w.right.last_pass),
            long,
            "short {short:?} long {long:?}"
        );
        assert_eq!(w.width, long.1 - long.0 + 1);
        assert_eq!(w.center, long.0 + (long.1 - long.0) / 2);
        // Far fewer probes than a single-step sweep of 256 points.
        assert!(
            eye.probes.len() < 60,
            "{short:?} {long:?}: {} probes",
            eye.probes.len()
        );
    }
}

#[test]
fn a_flaky_long_fail_is_confirmed_away() {
    // 160 fails twice: once on the inward coarse walk (single probes), once
    // on the outward fine walk, where only a confirmed fail ends the edge.
    let mut eye = Eye::new((20, 180), (35, 160));
    eye.flaky = vec![160, 160];
    let w = complex_window(100, 0, 255, 10, 2, |p, l| eye.probe(p, l)).unwrap();
    assert_eq!(w.right.last_pass, 160);
    // Without confirmation the flaky fail moves the edge inward by one.
    let mut eye = Eye::new((20, 180), (35, 160));
    eye.flaky = vec![160, 160];
    let e = complex_edge(100, 255, 10, 0, &mut |p, l| eye.probe(p, l)).unwrap();
    assert_eq!(e.last_pass, 159);
}

#[test]
fn start_must_pass_the_long_pattern() {
    let mut eye = Eye::new((20, 180), (120, 160));
    assert_eq!(
        complex_window(100, 0, 255, 10, 1, |p, l| eye.probe(p, l)),
        Err(ComplexError::StartFails)
    );
    let r: Result<_, ComplexError<&str>> =
        complex_window(100, 0, 255, 10, 1, |_, _| Err("timeout"));
    assert_eq!(r, Err(ComplexError::Probe("timeout")));
}

#[test]
fn the_coarse_walk_uses_only_the_short_pattern() {
    let mut eye = Eye::new((20, 180), (35, 160));
    let e = complex_edge(100, 255, 10, 0, &mut |p, l| eye.probe(p, l)).unwrap();
    assert_eq!(e.short_fail, Some(190));
    assert_eq!(e.probes_short, 9); // 110, 120, ..., 190
    let shorts: Vec<u32> = eye
        .probes
        .iter()
        .filter(|(_, l)| *l == Length::Short)
        .map(|(p, _)| *p)
        .collect();
    assert_eq!(shorts, (110..=190).step_by(10).collect::<Vec<_>>());
    // The long walk starts at the last short pass (180) and goes inward.
    assert_eq!(eye.probes[9], (180, Length::Long));
    assert_eq!(e.last_pass, 160);
    // Downwards, mirrored.
    let mut eye = Eye::new((20, 180), (35, 160));
    let e = complex_edge(100, 0, 10, 0, &mut |p, l| eye.probe(p, l)).unwrap();
    assert_eq!((e.last_pass, e.short_fail), (35, Some(10)));
    // An eye open to the limit.
    let mut eye = Eye::new((0, 255), (0, 255));
    let e = complex_edge(100, 255, 10, 0, &mut |p, l| eye.probe(p, l)).unwrap();
    assert_eq!((e.last_pass, e.short_fail), (255, None));
}
