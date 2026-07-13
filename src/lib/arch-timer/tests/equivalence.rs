// SPDX-License-Identifier: GPL-2.0-only
//! The crate against a verbatim copy of the A733 board's time.rs.

use arch_timer::{Counter, Timer};

#[allow(dead_code, clippy::all)]
mod a7a {
    include!("legacy/a7a_time.rs");
}

/// The legacy udelay's deadline expression, written out verbatim from
/// `udelay` (its `ticks()` cannot be fed on the host).
fn legacy_deadline(start: u64, us: u64) -> u64 {
    start + us * (a7a::TIMER_HZ / 1_000_000)
}

struct Script {
    t: u64,
    step: u64,
}
impl Counter for Script {
    fn ticks(&mut self) -> u64 {
        let v = self.t;
        self.t += self.step;
        v
    }
}

#[test]
fn host_counter_and_zero_delay_match() {
    assert_eq!(a7a::ticks(), arch_timer::ticks());
    a7a::udelay(0); // terminates on the host (ticks() == 0)
    let t = Timer::new(a7a::TIMER_HZ);
    assert_eq!(t.udelay(&mut arch_timer::ArchCounter, 0), 0);
}

#[test]
fn deadlines_match_the_a733_expression() {
    let t = Timer::new(a7a::TIMER_HZ);
    let mut x: u64 = 0x5eed_7173;
    for _ in 0..200_000 {
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        let start = x >> 8;
        let us = (x & 0xff_ffff) % 10_000_000;
        assert_eq!(t.deadline(start, us), legacy_deadline(start, us));
    }
}

#[test]
fn udelay_polls_until_the_deadline() {
    let t = Timer::new(24_000_000);
    let mut c = Script { t: 1000, step: 6 };
    // 10 us = 240 ticks from 1000: reads 1006..1234 are below 1240 -> 39 polls.
    assert_eq!(t.udelay(&mut c, 10), 39);
    assert_eq!(t.ticks_to_us(240), 10);
}
