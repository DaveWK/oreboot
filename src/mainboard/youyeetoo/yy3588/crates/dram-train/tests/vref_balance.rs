// SPDX-License-Identifier: GPL-2.0-only
//! P3-M8: Vref by balancing the 0- and 1-windows, against a swing plant.

use dram_train::vref_balance::{balance, Axis, BalanceError};

/// The 0-window grows and the 1-window shrinks as the reference rises
/// above the swing centre `centre` (in tenths of a point, so the balance
/// can fall between two points); `closed` points read nothing.
struct Swing {
    centre10: i32,
    slope: i32,
    base: i32,
    point: usize,
    visits: Vec<usize>,
    closed: Option<usize>,
}
impl Swing {
    fn new(centre10: i32) -> Self {
        Self {
            centre10,
            slope: 10,
            base: 40,
            point: 0,
            visits: Vec::new(),
            closed: None,
        }
    }
}
impl Axis for Swing {
    type Error = ();
    fn set(&mut self, p: usize) -> Result<(), ()> {
        self.point = p;
        Ok(())
    }
    fn windows(&mut self) -> Result<(u16, u16), ()> {
        self.visits.push(self.point);
        if self.closed == Some(self.point) {
            return Ok((0, 0));
        }
        let off = (self.point as i32 * 10 - self.centre10) * self.slope / 10;
        let z = (self.base + off).max(0) as u16;
        let o = (self.base - off).max(0) as u16;
        Ok((z, o))
    }
}

#[test]
fn it_walks_to_the_balance_from_either_side() {
    for start in [0usize, 10, 25, 40, 63] {
        let mut s = Swing::new(250);
        let b = balance(&mut s, start, 64, 2, 100).unwrap();
        assert_eq!(b.point, 25, "start {start}");
        assert_eq!(b.zero, b.one);
        assert!(!b.straddled);
        assert_eq!(s.point, 25, "left at the result");
        assert_eq!(
            b.steps as usize,
            start.abs_diff(25) + 1,
            "only the way there"
        );
    }
}

#[test]
fn a_balance_between_two_points_takes_the_better_one() {
    // Centre at 25.4: point 25 is closer.
    for (centre10, want) in [(254, 25), (257, 26)] {
        for start in [10usize, 40] {
            let mut s = Swing::new(centre10);
            let b = balance(&mut s, start, 64, 0, 100).unwrap();
            assert!(b.straddled);
            assert_eq!(b.point, want, "centre {centre10} start {start}");
            assert_eq!(s.point, want);
        }
    }
}

#[test]
fn failures() {
    let mut s = Swing::new(700);
    assert!(matches!(
        balance(&mut s, 10, 64, 2, 100),
        Err(BalanceError::OutOfRange { point: 63, .. })
    ));
    let mut s = Swing::new(-50);
    assert!(matches!(
        balance(&mut s, 10, 64, 2, 100),
        Err(BalanceError::OutOfRange { point: 0, .. })
    ));
    let mut s = Swing::new(250);
    s.closed = Some(20);
    assert_eq!(
        balance(&mut s, 10, 64, 2, 100),
        Err(BalanceError::Closed { point: 20 })
    );
    let mut s = Swing::new(250);
    assert_eq!(balance(&mut s, 0, 64, 2, 5), Err(BalanceError::NoBalance));
    assert_eq!(balance(&mut s, 0, 0, 2, 5), Err(BalanceError::NoBalance));
}
