// SPDX-License-Identifier: GPL-2.0-only
//! The stress test against fault-injection plants: each pattern class
//! catches the faults it is meant for (stuck cells, stuck data lines,
//! aliased address lines, coupling faults in both address orders, faults a
//! cache would hide), and the classes that are not meant for a fault are
//! shown to miss it, which is why the profile has them all.

use std::collections::HashMap;

use memtest::stress::{splitmix64, traffic, DEFAULT_SEED};
use memtest::{
    ErrorLog, Guard, Memory, Plan, Record, Region, Report, Runner, Step, Stress, PROFILE_FAST,
};

#[derive(Clone, Copy, Debug)]
enum Coupling {
    /// A rising transition of the aggressor's bit inverts the victim's bit.
    Invert,
    /// A rising transition of the aggressor's bit forces the victim's bit.
    Force(bool),
}

#[derive(Default)]
struct Plant {
    base: u64,
    dram: Vec<u64>,
    /// (address, bit, value): one stuck cell.
    stuck: Vec<(u64, u8, bool)>,
    /// (bit, value): a data line stuck on every word.
    line: Option<(u8, bool)>,
    /// Address bit the decoder ignores.
    alias: Option<u32>,
    /// (aggressor, victim, bit, kind).
    coupling: Vec<(u64, u64, u8, Coupling)>,
    /// A write-back cache in front of DRAM (word granular, unbounded).
    cache: Option<HashMap<u64, (u64, bool)>>,
    /// Whether the cache-maintenance hooks do anything.
    maintain: bool,
    dropped_dirty: u64,
    ticks: u64,
    kicks: u64,
    cleans: u64,
    invals: u64,
}

impl Plant {
    fn new(base: u64, bytes: u64) -> Self {
        Self {
            base,
            dram: vec![0; (bytes / 8) as usize],
            maintain: true,
            ..Default::default()
        }
    }
    fn cached(mut self, maintain: bool) -> Self {
        self.cache = Some(HashMap::new());
        self.maintain = maintain;
        self
    }
    fn cell(&self, addr: u64) -> usize {
        let a = match self.alias {
            Some(k) => addr & !(1u64 << k),
            None => addr,
        };
        ((a - self.base) / 8) as usize
    }
    fn dram_write(&mut self, addr: u64, v: u64) {
        let i = self.cell(addr);
        let old = self.dram[i];
        let mut v = v;
        for &(a, bit, val) in &self.stuck {
            if self.cell(a) == i {
                v = v & !(1 << bit) | (u64::from(val) << bit);
            }
        }
        self.dram[i] = v;
        for &(agg, vic, bit, kind) in &self.coupling.clone() {
            if self.cell(agg) == i && old & (1 << bit) == 0 && v & (1 << bit) != 0 {
                let j = self.cell(vic);
                match kind {
                    Coupling::Invert => self.dram[j] ^= 1 << bit,
                    Coupling::Force(val) => {
                        self.dram[j] = self.dram[j] & !(1 << bit) | (u64::from(val) << bit)
                    }
                }
            }
        }
    }
    fn dram_read(&self, addr: u64) -> u64 {
        let v = self.dram[self.cell(addr)];
        match self.line {
            Some((bit, val)) => v & !(1 << bit) | (u64::from(val) << bit),
            None => v,
        }
    }
}

impl Memory for Plant {
    fn write(&mut self, addr: u64, v: u64) {
        match self.cache.as_mut() {
            Some(c) => {
                c.insert(addr, (v, true));
            }
            None => self.dram_write(addr, v),
        }
    }
    fn read(&mut self, addr: u64) -> u64 {
        if let Some(&(v, _)) = self.cache.as_ref().and_then(|c| c.get(&addr)) {
            return v;
        }
        let v = self.dram_read(addr);
        if let Some(c) = self.cache.as_mut() {
            c.insert(addr, (v, false));
        }
        v
    }
    fn complete(&mut self) {}
    fn ticks(&mut self) -> u64 {
        self.ticks += 1;
        self.ticks
    }
    fn kick(&mut self) {
        self.kicks += 1;
    }
    fn clean_invalidate(&mut self, addr: u64, len: u64) {
        self.cleans += 1;
        if !self.maintain {
            return;
        }
        if let Some(mut c) = self.cache.take() {
            let mut a = addr;
            while a < addr + len {
                if let Some((v, dirty)) = c.remove(&a) {
                    if dirty {
                        self.dram_write(a, v);
                    }
                }
                a += 8;
            }
            self.cache = Some(c);
        }
    }
    fn invalidate(&mut self, addr: u64, len: u64) {
        self.invals += 1;
        if !self.maintain {
            return;
        }
        if let Some(c) = self.cache.as_mut() {
            let mut a = addr;
            while a < addr + len {
                if let Some((_, true)) = c.remove(&a) {
                    self.dropped_dirty += 1;
                }
                a += 8;
            }
        }
    }
}

const BASE: u64 = 0x4000_0000;
const BYTES: u64 = 64 << 10;

/// Two ranges with a reserved hole, like a real plan.
fn plan() -> Plan {
    Plan::new(
        Region::new(BASE, BYTES).unwrap(),
        &[Region::new(BASE + 0x3000, 0x1000).unwrap()],
    )
    .unwrap()
}

fn errors_of(p: &mut Plant, plan: &Plan, step: Step) -> u64 {
    let mut log = ErrorLog::<4>::default();
    Runner::new(p, 512, 100).unwrap().step(plan, step, &mut log);
    log.errors
}

const CHECKER: [Step; 2] = [
    Step::Check(Stress::Checkerboard { inverted: false }),
    Step::Check(Stress::Checkerboard { inverted: true }),
];
const WALK: [Step; 2] = [
    Step::Check(Stress::Walk { ones: true }),
    Step::Check(Stress::Walk { ones: false }),
];
const LINES: [Step; 1] = [Step::Lines { window: 1 << 20 }];
const MARCH: [Step; 1] = [Step::March { background: 0 }];
const OWN: [Step; 1] = [Step::Check(Stress::OwnAddress)];
const RANDOM: [Step; 1] = [Step::Check(Stress::Random { seed: DEFAULT_SEED })];

fn class_catches(make: &dyn Fn() -> Plant, class: &[Step]) -> bool {
    let plan = plan();
    let mut p = make();
    class
        .iter()
        .map(|s| errors_of(&mut p, &plan, *s))
        .sum::<u64>()
        > 0
}

#[test]
fn a_good_memory_passes_every_step() {
    let plan = plan();
    for cached in [false, true] {
        let mut p = Plant::new(BASE, BYTES);
        if cached {
            p = p.cached(true);
        }
        let mut log = ErrorLog::<8>::default();
        let (n, why) =
            Runner::new(&mut p, 512, 100)
                .unwrap()
                .run(&plan, &PROFILE_FAST, &mut log, |_| Guard::Continue);
        assert_eq!((n, why, log.errors), (PROFILE_FAST.len() as u32, "done", 0));
        assert_eq!(p.dropped_dirty, 0, "invalidate only after clean");
        assert!(p.kicks > 1);
        assert!(p.cleans > 0 && p.invals > 0);
    }
}

#[test]
fn stuck_cells_both_values_every_bit_class() {
    for bit in [0u8, 1, 17, 31, 32, 63] {
        for val in [false, true] {
            for addr in [BASE, BASE + 0x1238, BASE + BYTES - 8] {
                let make = || {
                    let mut p = Plant::new(BASE, BYTES);
                    p.stuck.push((addr, bit, val));
                    p
                };
                for (name, class) in [
                    ("checkerboard pair", &CHECKER[..]),
                    ("walk pair", &WALK[..]),
                    ("march", &MARCH[..]),
                    ("lines", &LINES[..]),
                ] {
                    assert!(
                        class_catches(&make, class),
                        "{name}: stuck {val} bit {bit} at {addr:#x}"
                    );
                }
            }
        }
    }
}

#[test]
fn stuck_data_lines_are_named_in_the_log() {
    for bit in [0u8, 7, 8, 31, 32, 63] {
        for val in [false, true] {
            let make = || {
                let mut p = Plant::new(BASE, BYTES);
                p.line = Some((bit, val));
                p
            };
            for class in [&CHECKER[..], &WALK[..], &LINES[..], &MARCH[..]] {
                assert!(class_catches(&make, class), "line {bit}={val}");
            }
            let mut p = make();
            let mut log = ErrorLog::<4>::default();
            Runner::new(&mut p, 512, 100)
                .unwrap()
                .lines(&plan(), 4096, &mut log);
            assert_eq!(log.bits, 1 << bit, "only the stuck line fails");
            assert_eq!(log.len, 4);
            assert!(log.errors > 4);
        }
    }
}

#[test]
fn aliased_address_lines_need_an_address_pattern() {
    // Address bits 4.. inside the 64 KiB plan: bit 3 is the word parity.
    for k in [4u32, 6, 9, 12, 15] {
        let make = || {
            let mut p = Plant::new(BASE, BYTES);
            p.alias = Some(k);
            p
        };
        assert!(class_catches(&make, &OWN), "own address, line {k}");
        assert!(class_catches(&make, &RANDOM), "random, line {k}");
        assert!(class_catches(&make, &MARCH), "march, line {k}");
        // A checkerboard writes the same value to both aliases (same word
        // parity): it cannot see this fault.
        assert!(!class_catches(&make, &CHECKER), "checkerboard, line {k}");
    }
}

#[test]
fn coupling_faults_need_the_march_in_both_orders() {
    let below = BASE + 0x100;
    let above = BASE + 0x8000;
    for (agg, vic) in [(below, above), (above, below)] {
        for kind in [
            Coupling::Invert,
            Coupling::Force(true),
            Coupling::Force(false),
        ] {
            for bit in [0u8, 33] {
                let make = || {
                    let mut p = Plant::new(BASE, BYTES);
                    p.coupling.push((agg, vic, bit, kind));
                    p
                };
                assert!(
                    class_catches(&make, &MARCH),
                    "march: {kind:?} bit {bit}, aggressor {agg:#x} victim {vic:#x}"
                );
            }
        }
        // A fill writes the victim after an aggressor below it, which
        // hides the fault from every fill-then-verify pattern.
        if agg < vic {
            let make = || {
                let mut p = Plant::new(BASE, BYTES);
                p.coupling.push((agg, vic, 0, Coupling::Invert));
                p
            };
            for class in [&OWN[..], &CHECKER[..], &WALK[..], &RANDOM[..]] {
                assert!(!class_catches(&make, class), "{class:?}");
            }
        }
    }
}

#[test]
fn a_cache_hides_faults_unless_the_board_maintains_it() {
    let make = |maintain| {
        let mut p = Plant::new(BASE, BYTES).cached(maintain);
        p.stuck.push((BASE + 0x2000, 5, true));
        p.line = Some((40, false));
        p
    };
    let plan = plan();
    for step in PROFILE_FAST {
        let mut p = make(false);
        assert_eq!(errors_of(&mut p, &plan, step), 0, "{step:?} hidden");
        let mut p = make(true);
        assert!(errors_of(&mut p, &plan, step) > 0, "{step:?} seen");
        assert_eq!(p.dropped_dirty, 0);
    }
}

#[test]
fn slices_cover_the_plan_and_merge_like_one_run() {
    let plan = plan();
    for n in 1..=7 {
        let mut got: Vec<Region> = Vec::new();
        let mut bytes = 0;
        for k in 0..n {
            let s = plan.slice(n, k, 256).unwrap();
            for r in s.ranges() {
                assert!(r.start < r.end);
                if r.start != BASE && r.start != BASE + 0x4000 {
                    assert_eq!((r.start - BASE) % 256, 0, "aligned cut");
                }
                got.push(*r);
            }
            bytes += s.bytes;
        }
        assert_eq!(bytes, plan.bytes, "n={n}");
        got.sort_by_key(|r| r.start);
        for w in got.windows(2) {
            assert!(w[0].end <= w[1].start, "disjoint");
        }
    }
    assert!(plan.slice(0, 0, 64).is_err());
    assert!(plan.slice(2, 2, 64).is_err());
    assert!(plan.slice(2, 0, 24).is_err());

    // Four harts, one log each, merged: the same errors as one run.
    let fault = |p: &mut Plant| {
        p.stuck.push((BASE + 0x40, 3, true));
        p.stuck.push((BASE + 0xf000, 60, false));
    };
    let mut whole = Plant::new(BASE, BYTES);
    fault(&mut whole);
    let mut one = ErrorLog::<16>::default();
    Runner::new(&mut whole, 512, 100)
        .unwrap()
        .run(&plan, &PROFILE_FAST, &mut one, |_| Guard::Continue);
    let mut harts = Plant::new(BASE, BYTES);
    fault(&mut harts);
    let mut merged = ErrorLog::<16>::default();
    for k in 0..4 {
        let s = plan.slice(4, k, 512).unwrap();
        let mut log = ErrorLog::<16>::default();
        Runner::new(&mut harts, 512, 100)
            .unwrap()
            .run(&s, &PROFILE_FAST, &mut log, |_| Guard::Continue);
        merged.merge(&log);
    }
    assert_eq!(merged.errors, one.errors);
    assert_eq!(merged.bits, one.bits);
    assert_eq!((merged.low, merged.high), (BASE + 0x40, BASE + 0xf000));
}

#[test]
fn split_fill_and_verify_catch_aliases_across_slices() {
    // Address line 15 folds the upper half of the plan onto the lower: two
    // harts that each fill and verify their own half see nothing; filling
    // every slice before verifying any (a barrier between) catches it.
    let plan = Plan::new(Region::new(BASE, BYTES).unwrap(), &[]).unwrap();
    let make = || {
        let mut p = Plant::new(BASE, BYTES);
        p.alias = Some(15);
        p
    };
    let pat = Stress::OwnAddress;
    let halves = [plan.slice(2, 0, 64).unwrap(), plan.slice(2, 1, 64).unwrap()];
    let mut p = make();
    let mut log = Report::default();
    for h in &halves {
        Runner::new(&mut p, 512, 100)
            .unwrap()
            .check(h, pat, &mut log);
    }
    assert_eq!(log.errors, 0, "each half alone is self-consistent");
    let mut p = make();
    for h in &halves {
        Runner::new(&mut p, 512, 100).unwrap().fill(h, pat);
    }
    let mut log = Report::default();
    for h in &halves {
        Runner::new(&mut p, 512, 100)
            .unwrap()
            .verify(h, pat, &mut log);
    }
    assert_eq!(log.errors, BYTES / 16);
}

#[test]
fn error_log_is_bounded_and_keeps_the_first() {
    let mut log = ErrorLog::<3>::default();
    for i in 0..10u64 {
        log.record(BASE + 8 * i, 0, 1 << i);
    }
    assert_eq!(log.errors, 10);
    assert_eq!(log.entries().len(), 3);
    assert_eq!(log.entries()[2].addr, BASE + 16);
    assert_eq!(log.entries()[2].xor(), 4);
    assert_eq!(log.bits, 0x3ff);
    assert_eq!((log.low, log.high), (BASE, BASE + 72));
}

#[test]
fn patterns_are_address_pure_and_reproducible() {
    let r = Stress::Random { seed: 7 };
    assert_eq!(r.value(BASE), splitmix64(BASE ^ 7));
    assert_eq!(r.value(BASE), Stress::Random { seed: 7 }.value(BASE));
    assert_ne!(r.value(BASE), Stress::Random { seed: 8 }.value(BASE));
    // Own address: unique words, both halves carry the address.
    let mut seen = std::collections::HashSet::new();
    for a in (0..1u64 << 34).step_by(0x1000_0008) {
        let v = Stress::OwnAddress.value(a);
        assert_eq!(v & 0xffff_ffff, a & 0xffff_ffff);
        assert_eq!(v >> 32, (!a ^ (a >> 32)) & 0xffff_ffff);
        assert!(seen.insert(v));
    }
    assert_eq!(
        Stress::Checkerboard { inverted: false }.value(8),
        0xaaaa_aaaa_aaaa_aaaa
    );
    assert_eq!(Stress::Walk { ones: false }.value(8 * 65), !2);
    assert_eq!(
        Stress::Line {
            bit: 63,
            ones: true
        }
        .value(123 * 8),
        1 << 63
    );
}

#[test]
fn run_obeys_its_guard_and_the_profile_traffic_is_a_few_minutes() {
    let plan = plan();
    let mut p = Plant::new(BASE, BYTES);
    // Bit 3 of word BASE + 8 stuck at 1: own address and the
    // checkerboard write a 1 there, the inverted checkerboard a 0.
    p.stuck.push((BASE + 8, 3, true));
    let mut log = ErrorLog::<2>::default();
    let (n, why) =
        Runner::new(&mut p, 512, 100)
            .unwrap()
            .run(&plan, &PROFILE_FAST, &mut log, |s| {
                if s.errors > 0 {
                    Guard::Stop("errors")
                } else {
                    Guard::Continue
                }
            });
    assert_eq!(
        (n, why),
        (3, "errors"),
        "stops after the inverted checkerboard"
    );
    assert!(Runner::new(&mut p, 100, 1).is_err());

    // 2 GiB: 22 sweeps (44 GiB) plus the line walks on one 1 MiB window.
    let two = Plan::new(Region::new(0, 2 << 30).unwrap(), &[]).unwrap();
    assert_eq!(traffic(&PROFILE_FAST, &two), (44u64 << 30) + (256 << 20));
}

#[test]
fn descending_march_elements_visit_every_word() {
    // A plant that counts reads per word: the march reads every word five
    // times, whatever the block size and range ends.
    struct Count(HashMap<u64, u32>, Vec<u64>);
    impl Memory for Count {
        fn write(&mut self, _: u64, _: u64) {}
        fn read(&mut self, a: u64) -> u64 {
            *self.0.entry(a).or_default() += 1;
            self.1.push(a);
            0
        }
        fn complete(&mut self) {}
        fn ticks(&mut self) -> u64 {
            0
        }
        fn kick(&mut self) {}
    }
    let plan = Plan::new(
        Region::new(BASE + 0x48, 0x1230).unwrap(),
        &[Region::new(BASE + 0x400, 0x88).unwrap()],
    )
    .unwrap();
    for block in [64, 192, 4096] {
        let mut c = Count(HashMap::new(), Vec::new());
        let mut log = Report::default();
        Runner::new(&mut c, block, 1)
            .unwrap()
            .march(&plan, 0, &mut log);
        assert_eq!(c.0.len() as u64, plan.bytes / 8);
        assert!(c.0.values().all(|&n| n == 5));
        // The fourth element (first descending one) starts at the top.
        let words = (plan.bytes / 8) as usize;
        assert_eq!(c.1[2 * words], plan.ranges()[1].end - 8);
        assert_eq!(c.1[3 * words - 1], plan.ranges()[0].start);
    }
}
