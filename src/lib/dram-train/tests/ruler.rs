// SPDX-License-Identifier: GPL-2.0-only
//! P3-M9: the delay-line ruler and the total-delay API.

use dram_train::result::{Axis, Margin, Unit};
use dram_train::ruler::{join, set_total, split, DelayError, Parts, Ruler, TotalDelay};

#[test]
fn quarter_period_and_edge_rulers_agree() {
    // LPDDR4X-3200: tCK 625 ps; a line of 64 taps per quarter clock.
    let q = Ruler::from_quarter(625, 64).unwrap();
    let p = Ruler::from_period(625, 256).unwrap();
    let e = Ruler::from_edges(625, 100, 356).unwrap();
    assert_eq!(q, p);
    assert_eq!(p, e);
    assert!(Ruler::from_edges(625, 356, 100).is_none());
    assert!(Ruler::from_quarter(0, 64).is_none());
    assert!(Ruler::from_period(625, 0).is_none());
}

#[test]
fn conversions_round_trip_and_round_to_nearest() {
    let r = Ruler::from_quarter(625, 64).unwrap(); // 2.44 ps per tap
    assert_eq!(r.ps(256), 625);
    assert_eq!(r.ps(-256), -625);
    assert_eq!(r.ps(1), 2);
    assert_eq!(r.ps(3), 7); // 7.32
    assert_eq!(r.taps(625), 256);
    assert_eq!(r.taps(-100), -41); // 40.96
    for t in -300..300 {
        assert!((r.taps(r.ps(t)) - t).abs() <= 1, "{t}");
    }
    assert_eq!(r.degrees(45), 32);
    assert_eq!(r.degrees(90), 64);
    assert_eq!(r.degrees(360), 256);
    assert_eq!(r.ui(2), 128);
    // A fractional quarter (a 37-tap quarter: 148 per period).
    let f = Ruler::from_quarter(938, 37).unwrap();
    assert_eq!(f.degrees(45), 19); // 18.5 rounds up
    assert_eq!(f.ui(2), 74);
    assert_eq!(f.ps(148), 938);
}

#[test]
fn margins_convert_to_ps() {
    let r = Ruler::from_quarter(625, 64).unwrap();
    let m = Margin {
        axis: Axis::ReadEye,
        ch: 0,
        rank: 0,
        lane: 2,
        unit: Unit::Step,
        lo: -40,
        hi: 52,
        center: 6,
    };
    let p = r.margin_ps(m);
    assert_eq!((p.unit, p.lo, p.hi, p.center), (Unit::Ps, -98, 127, 15));
    assert_eq!(r.margin_ps(p), p, "already in ps");
}

#[test]
fn totals_split_into_stages_and_fine() {
    assert_eq!(
        split(300, 128, 8),
        Some(Parts {
            stages: 2,
            fine: 44
        })
    );
    assert_eq!(
        join(
            Parts {
                stages: 2,
                fine: 44
            },
            128
        ),
        300
    );
    for total in 0..2000 {
        let p = split(total, 128, 16).unwrap();
        assert!(p.fine < 128);
        assert_eq!(join(p, 128), total);
    }
    assert_eq!(split(5000, 128, 16), None);
    assert_eq!(split(5, 0, 16), None);
}

struct Chain {
    written: Vec<Parts>,
}
impl TotalDelay for Chain {
    type Error = ();
    fn stage_taps(&self) -> u32 {
        128
    }
    fn max_stages(&self) -> u32 {
        3
    }
    fn write_parts(&mut self, p: Parts) -> Result<(), ()> {
        self.written.push(p);
        Ok(())
    }
}

#[test]
fn set_total_writes_the_parts() {
    let mut c = Chain {
        written: Vec::new(),
    };
    assert_eq!(
        set_total(&mut c, 400),
        Ok(Parts {
            stages: 3,
            fine: 16
        })
    );
    assert_eq!(set_total(&mut c, 600), Err(DelayError::Range));
    assert_eq!(
        c.written,
        [Parts {
            stages: 3,
            fine: 16
        }]
    );
}
