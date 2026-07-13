// SPDX-License-Identifier: GPL-2.0-only
//! P3-M7: margin-check options (pattern mode, step count, bit select, the
//! Vref of the check, abort and status, the restore verified).

use dram_train::margin::{
    scan, scan_with_options, CheckStatus, Direction, Granularity, Options, Pattern, Range, Request,
    Target,
};

/// A diamond eye centred at delay 50, Vref 20; the complex pattern shrinks
/// it by 8 delay points. `restore` puts back delay 50 and Vref 20 unless
/// `broken_restore`.
#[derive(Default)]
struct Eye {
    vref: i32,
    delay: i32,
    tests: u32,
    restored: u32,
    pattern: Pattern,
    bits: Option<u16>,
    broken_restore: bool,
}
impl Target for Eye {
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
        let shrink = if self.pattern == Pattern::Complex {
            8
        } else {
            0
        };
        Ok((self.delay - 50).abs() + 2 * (self.vref - 20).abs() < 30 - shrink)
    }
    fn restore(&mut self, _: &Request) -> Result<(), ()> {
        self.restored += 1;
        if !self.broken_restore {
            self.delay = 50;
            self.vref = 20;
        }
        Ok(())
    }
    fn configure(&mut self, _: &Request, o: &Options) -> Result<(), ()> {
        self.pattern = o.pattern;
        self.bits = o.bits;
        Ok(())
    }
}

fn req() -> Request {
    Request {
        dir: Direction::Read,
        unit: Granularity::Nibble,
        ch: 0,
        rank: 1,
        select: 3,
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
    }
}

fn row(lo: u32, hi: u32) -> u128 {
    (lo..=hi).fold(0, |m, i| m | 1 << i)
}

#[test]
fn defaults_are_the_v1_scan() {
    let (mut a, mut b) = ([0u128; 11], [0u128; 11]);
    let s = scan(&mut Eye::default(), &req(), &mut a).unwrap();
    let mut e = Eye::default();
    let r = scan_with_options(&mut e, &req(), &Options::default(), &mut b, || false).unwrap();
    assert_eq!((a, s), (b, r.scan));
    assert_eq!(r.status, CheckStatus::Done);
    assert_eq!(r.rows_done, 11);
    assert_eq!(r.points, e.tests);
    assert_eq!(r.restored_passes, None);
}

#[test]
fn the_complex_pattern_and_bit_select_reach_the_board() {
    let mut m = [0u128; 11];
    let mut e = Eye::default();
    let o = Options {
        pattern: Pattern::Complex,
        bits: Some(0b0101),
        vref: Some(20),
        ..Options::default()
    };
    let r = scan_with_options(&mut e, &req(), &o, &mut m, || false).unwrap();
    assert_eq!((e.pattern, e.bits), (Pattern::Complex, Some(0b0101)));
    // One row at Vref 20: the complex eye is 50 +/- 21.
    assert_eq!(r.rows_done, 1);
    assert_eq!(m[0], row(29, 71));
}

#[test]
fn the_step_count_bounds_the_scan_around_the_trained_point() {
    let mut m = [0u128; 11];
    let mut e = Eye::default();
    let o = Options {
        steps: Some(10),
        vref: Some(20),
        full: true,
        ..Options::default()
    };
    let r = scan_with_options(&mut e, &req(), &o, &mut m, || false).unwrap();
    assert_eq!(m[0], row(40, 60));
    assert_eq!(r.points, 21);
}

#[test]
fn an_abort_keeps_the_finished_rows_and_restores() {
    let mut m = [0u128; 11];
    let mut e = Eye::default();
    let mut polls = 0;
    let r = scan_with_options(&mut e, &req(), &Options::default(), &mut m, || {
        polls += 1;
        polls > 60
    })
    .unwrap();
    assert_eq!(r.status, CheckStatus::Aborted);
    assert!(r.rows_done > 0 && r.rows_done < 11, "{}", r.rows_done);
    assert_eq!(r.scan.rows, r.rows_done);
    assert_eq!(r.points, 60);
    assert_eq!(e.restored, 1);
}

#[test]
fn the_restore_is_verified_on_request() {
    let o = Options {
        verify_restore: true,
        ..Options::default()
    };
    let mut m = [0u128; 11];
    let r = scan_with_options(&mut Eye::default(), &req(), &o, &mut m, || false).unwrap();
    assert_eq!(r.restored_passes, Some(true));
    let mut bad = Eye {
        broken_restore: true,
        ..Eye::default()
    };
    let r = scan_with_options(&mut bad, &req(), &o, &mut m, || false).unwrap();
    assert_eq!(
        r.restored_passes,
        Some(false),
        "left at the last scan point"
    );
}
