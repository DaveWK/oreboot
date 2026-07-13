// SPDX-License-Identifier: GPL-2.0-only
//! P3-A2: loopback instruments and PRBS checks on a PHY plant.

use dram_train::gate::Class;
use dram_train::loopback::{
    knob_check, period, rising_edges, sweep, Checker, Generator, Loopback, Prbs,
};
use dram_train::ruler::Ruler;
use dram_train::sanity::SanityError;

/// A transmit delay line of `taps_per_period` taps per clock period looped
/// back to a receiver sampling at a fixed point; jitter makes samples
/// within `jitter` taps of an edge noisy. `dead`: the delay register does
/// nothing (not the live copy).
struct Phy {
    taps_per_period: u32,
    phase: u32,
    jitter: u32,
    dead: bool,
    delay: u32,
}
impl Loopback for Phy {
    type Error = ();
    fn set_tx_delay(&mut self, _: u8, t: u32) -> Result<(), ()> {
        if !self.dead {
            self.delay = t;
        }
        Ok(())
    }
    fn sample(&mut self, _: u8, n: u16) -> Result<u16, ()> {
        let p = self.taps_per_period;
        let t = (self.delay + self.phase) % p;
        let near = |e: u32| t.abs_diff(e).min(p - t.abs_diff(e)) < self.jitter;
        Ok(if near(0) || near(p / 2) {
            n / 2
        } else if t < p / 2 {
            n
        } else {
            0
        })
    }
}

#[test]
fn the_period_calibrates_the_delay_line() {
    for (taps, phase) in [(256u32, 0u32), (256, 77), (148, 30), (300, 290)] {
        let mut phy = Phy {
            taps_per_period: taps,
            phase,
            jitter: 3,
            dead: false,
            delay: 0,
        };
        let mut s = vec![Class::Noise; 2 * taps as usize + 40];
        sweep(&mut phy, 0, 16, &mut s).unwrap();
        knob_check(&s).unwrap();
        let (a, b) = period(&s).unwrap();
        assert_eq!(b - a, taps, "taps {taps} phase {phase}");
        let r = Ruler::from_edges(625, a, b).unwrap();
        assert_eq!(r, Ruler::from_period(625, taps).unwrap());
    }
}

#[test]
fn a_dead_delay_register_is_named() {
    let mut phy = Phy {
        taps_per_period: 256,
        phase: 50,
        jitter: 3,
        dead: true,
        delay: 0,
    };
    let mut s = vec![Class::Noise; 600];
    sweep(&mut phy, 0, 16, &mut s).unwrap();
    assert_eq!(knob_check(&s), Err(SanityError::DeadKnob));
    assert_eq!(period(&s), None);
    let mut e = [0u32; 4];
    assert_eq!(rising_edges(&s, &mut e), 0);
}

#[test]
fn prbs_sequences_have_their_period() {
    for p in [Prbs::P7, Prbs::P15] {
        let mut g = Generator::new(p, 1);
        let start = g;
        let mut n = 0u64;
        loop {
            g.next_bit();
            n += 1;
            if g == start {
                break;
            }
        }
        assert_eq!(n, p.period(), "{p:?}");
    }
    assert_eq!(Prbs::P31.period(), (1 << 31) - 1);
    // A zero seed would lock up: it becomes 1.
    assert_eq!(Generator::new(Prbs::P7, 0), Generator::new(Prbs::P7, 1));
}

#[test]
fn the_checker_locks_by_itself_and_counts_errors() {
    for p in [Prbs::P7, Prbs::P15, Prbs::P23, Prbs::P31] {
        let mut g = Generator::new(p, 0x1234_5678);
        // Skip some bits: the checker needs no alignment.
        for _ in 0..1000 {
            g.next_bit();
        }
        let mut c = Checker::new(p);
        for _ in 0..5000 {
            c.push(g.next_bit());
        }
        assert!(c.locked());
        assert_eq!(c.errors, 0, "{p:?} clean");
        // Three isolated wrong bits: each counts once per tap plus one.
        for k in 0..3000 {
            let b = g.next_bit();
            c.push(if k % 1000 == 500 { !b } else { b });
        }
        assert_eq!(c.errors, 9, "{p:?}");
        assert_eq!(c.bits, 8000 - p.taps().0 as u64);
    }
    // A stuck lane never looks like PRBS.
    let mut c = Checker::new(Prbs::P7);
    for _ in 0..1000 {
        c.push(true);
    }
    assert!(c.errors > 900);
}
