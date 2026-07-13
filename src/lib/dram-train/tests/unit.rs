// SPDX-License-Identifier: GPL-2.0-only
//! Board-neutral behaviour of the training algorithms, with the uses of the
//! K1 (Vref eye along an LPDDR4X MR14 axis), the A733 (circular CA windows,
//! WCK2CK curves, ACSM launches) and the next boards (linear delay lines,
//! other lane and point counts) in mind.

use dram_train::io::{mock::Mock, wait, RegIo, Timeout};
use dram_train::window::{self, Window};
use dram_train::{edge, eye, launch};
use jedec_lpddr::lp4;

#[test]
fn circular_window_wraps_the_seam() {
    // Points 60..63 and 0..3 of a 64-point circular sweep pass.
    let map: u128 = 0xf000_0000_0000_000f;
    let w = window::longest_run(map, 64);
    assert_eq!(w, Window { start: 60, len: 8 });
    assert_eq!(w.center(64), 0);
    assert!(w.contains(2, 64) && w.contains(61, 64) && !w.contains(10, 64));
    // The linear view of the same map sees two runs of 4.
    assert_eq!(
        window::longest_run_linear(map, 64),
        Window { start: 0, len: 4 }
    );
}

#[test]
fn run_near_prefers_the_window_holding_the_reference() {
    // Two equal windows (CA passes once per 256 units): 4..9 and 36..41.
    let map: u128 = (0x3f << 4) | (0x3f << 36);
    assert_eq!(window::run_near(map, 64, 38), Window { start: 36, len: 6 });
    assert_eq!(window::run_near(map, 64, 5), Window { start: 4, len: 6 });
    // Outside both: the nearer centre.
    assert_eq!(window::run_near(map, 64, 30), Window { start: 36, len: 6 });
    assert_eq!(window::run_near(0, 64, 3), Window::EMPTY);
    assert_eq!(
        window::run_near(u128::MAX, 128, 3),
        Window { start: 0, len: 128 }
    );
}

#[test]
fn intersection_across_blocks_and_any_point_count() {
    for n in [16u32, 32, 64, 100, 128] {
        let a = window::full_mask(n) & !0b111;
        let b = window::full_mask(n) >> 2;
        let w = window::intersect_near([a, b], n, n / 2);
        assert_eq!(
            w,
            Window {
                start: 3,
                len: n - 5
            }
        );
    }
    assert_eq!(window::intersect([0b1100u128, 0b0110], 4), 0b0100);
}

#[test]
fn pass_bits_any_pattern_count_and_polarity() {
    // Two patterns, inverted polarity, 16 lanes.
    let echo = [0xffff, 0x0000];
    assert_eq!(window::pass_bits(&echo, &[true, false], 0xffff), 0xffff);
    assert_eq!(window::pass_bits(&echo, &[false, true], 0xffff), 0);
    // Lane 3 stuck high.
    assert_eq!(
        window::pass_bits(&[0x0008, 0x0008], &[false, false], 0x000f),
        0x0007
    );
}

#[test]
fn eye_centres_at_half_height_not_at_the_peak() {
    // A skewed eye over a LPDDR4X MR14 axis: peak at 30, half height 14..45.
    let mut w = [None; lp4::VREF_AXIS_LEN];
    for (i, x) in w.iter_mut().enumerate() {
        *x = Some(match i {
            14..=29 => 10 + (i - 14) as u8,
            30 => 26,
            31..=45 => 25 - ((i - 31) / 2) as u8,
            _ => 0,
        });
    }
    let e = eye::center(&w).unwrap();
    assert_eq!((e.peak, e.peak_width, e.level), (30, 26, 13));
    assert_eq!((e.lo, e.hi), (17, 45));
    assert_eq!(e.center, 31);
    assert_eq!(lp4::vref_axis_op(e.center), 31);
    assert_eq!(eye::center(&[None, None]), Err(eye::EyeError::Empty));
    assert_eq!(eye::center(&[Some(0), None]), Err(eye::EyeError::Closed));
}

#[test]
fn walk_out_stops_after_closed_points_and_covers_a_missed_eye() {
    let profile = |i: usize| -> Result<u8, ()> { Ok(if (20..30).contains(&i) { 5 } else { 0 }) };
    let mut out = [None; 50];
    eye::walk_out(&mut out, 25, 3, profile).unwrap();
    assert_eq!(out.iter().filter(|w| w.is_some()).count(), 10 + 3 + 3);
    // Started outside the eye with nothing open: everything is measured.
    let mut out = [None; 50];
    eye::walk_out(&mut out, 45, 2, |_| Ok::<u8, ()>(0)).unwrap();
    assert!(out.iter().all(|w| *w == Some(0)));
    // A measurement error stops the walk.
    let mut out = [None; 50];
    assert_eq!(
        eye::walk_out(&mut out, 25, 3, |i| if i == 27 { Err(i) } else { Ok(1) }),
        Err(27)
    );
}

#[test]
fn edge_polarity_and_stability_legacy() {
    // fail fail pass pass pass fail fail
    let err = [90, 90, 1, 1, 1, 90, 90];
    let d = edge::decide_legacy(&err, 16, None);
    assert_eq!(d.threshold, 45);
    assert_eq!(&d.edges[..d.n_edges as usize], &[(2, false), (5, true)]);
    assert_eq!(d.delay, Some(2 * 16 - 8));
    let d = edge::decide_with(&err, 16, None, edge::Polarity::PassToFail, 2);
    assert_eq!(d.delay, Some(5 * 16 - 8));
    // A one-point glitch is not stable.
    let glitch = [90, 1, 90, 90, 1, 1];
    assert_eq!(
        edge::decide_legacy(&glitch, 16, None).delay,
        Some(4 * 16 - 8)
    );
    // The hardware counter wins when it sits on a wanted transition.
    assert_eq!(edge::decide_legacy(&err, 16, Some(30)).delay, Some(30));
    assert!(edge::decide_legacy(&err, 16, Some(30)).counter_agrees);
    assert_eq!(
        edge::decide_legacy(&[5, 5, 5], 16, None),
        edge::Decision::EMPTY
    );
}

#[test]
fn bounded_launch_paths() {
    let mut io: Mock<8> = Mock::default();
    // Done at the first poll with a witness: ok.
    let l = launch::bounded_launch(&mut io, 100, |_| {}, |_| true, |_| true, |_| {});
    assert_eq!((l.result, l.polls), (Ok(()), 0));
    // ... without one: not observed.
    let l = launch::bounded_launch(&mut io, 100, |_| {}, |_| true, |_| false, |_| {});
    assert_eq!(l.result, Err(launch::LaunchError::NoBusyObserved));
    // Busy for three polls: ok, witness not needed.
    let mut left = 3;
    let l = launch::bounded_launch(
        &mut io,
        100,
        |_| {},
        |_| {
            left -= 1;
            left < 0
        },
        |_| false,
        |io| io.write32(0x10, 0),
    );
    assert_eq!((l.result, l.polls), (Ok(()), 3));
    // Never done: time budget, and stop still runs.
    io.set(0x20, 1);
    let l = launch::bounded_launch(
        &mut io,
        50,
        |_| {},
        |_| false,
        |_| true,
        |io| io.write32(0x20, 0),
    );
    assert_eq!(l.result, Err(launch::LaunchError::Timeout));
    assert_eq!(io.get(0x20), 0);
}

#[test]
fn wait_is_bounded_with_a_frozen_clock() {
    let mut io: Mock<4> = Mock::default();
    io.tick_us = 0;
    let mut n = 0u32;
    let r = wait(&mut io, "never", 10, |_| {
        n += 1;
        false
    });
    assert_eq!(r, Err(Timeout("never")));
    assert_eq!(n, dram_train::io::WAIT_MAX_POLLS);
    io.tick_us = 3;
    assert_eq!(wait(&mut io, "x", 0, |_| true), Ok(()));
    assert_eq!(wait(&mut io, "x", 0, |_| false), Err(Timeout("x")));
}

#[test]
fn reg_io_read_modify_write_helpers() {
    let mut io: Mock<4> = Mock::default();
    io.write32(0x40, 0xf0f0);
    io.set32(0x40, 0x1);
    io.clr32(0x40, 0xf000);
    io.clrset32(0x40, 0x00f0, 0x0a00);
    assert_eq!(io.read32(0x40), 0x0a01);
}
