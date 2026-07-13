// SPDX-License-Identifier: GPL-2.0-only
//! Destructive DRAM tests for firmware: a planner that keeps reserved ranges
//! (the running firmware, loaded images, the handoff DTB) out of the test,
//! data patterns, full passes over the plan, a bounded `quick` smoke check
//! for every boot, and a guarded `soak` that stops itself on a time budget,
//! a temperature limit or the first errors.
//!
//! The platform supplies word access through [`Memory`] (uncached or
//! otherwise coherent accesses, write completion, a monotonic tick counter
//! and the watchdog kick); MMU or cache set-up for speed stays board code.
//!
//! From ore-edk-boot-radxa-a7a oreboot 4904019: `common/src/memtest.rs`
//! (regions, plan, [`Pattern::Address`]/[`Pattern::Inverted`]/
//! [`Pattern::Xorshift`], [`pass`], kept exactly; the A7A handoff check stays
//! in the board, which calls [`Plan::new`]) and `dram/src/size.rs`
//! `quick_test` ([`quick`], same accesses and errors). New here: walking
//! ones, moving inversions, and [`soak`] (doc/CONVERGENCE.md section 5a #2).
//!
//! New in v1.1.0 (K1 K0.9, shared part): [`stress`], the full-coverage
//! stress test (own address, checkerboard, walks, per-line walks, seeded
//! random, March C- in both directions), cached runs with per-block cache
//! maintenance through [`Memory`], plan slices for several harts, a bounded
//! [`ErrorLog`], and the recommended [`PROFILE_FAST`].
// Region/Plan/pass code is kept as the A7A board had it (equivalence-tested
// against tests/legacy/); this newer-clippy style lint is allowed.
#![allow(unknown_lints)] // the lint below is newer than the runners' clippy (Rust 1.98)
#![allow(clippy::manual_is_multiple_of)]
#![no_std]
#[cfg(test)]
extern crate std;

pub mod arch;
pub mod stress;

pub use stress::{ErrorLog, Mismatch, Record, Runner, Step, Stress, PROFILE_FAST};

use dram_train::io::RegIo;

/// Most testable ranges a plan keeps (DRAM split around reserved ranges).
pub const MAX_REGIONS: usize = 8;
/// Most reserved ranges [`Plan::new`] takes.
pub const MAX_RESERVED: usize = MAX_REGIONS - 1;

/// A byte range `[start, end)` of 8-byte words.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct Region {
    pub start: u64,
    pub end: u64,
}
impl Region {
    /// A non-empty, 8-byte aligned range of `size` bytes at `start`.
    pub fn new(start: u64, size: u64) -> Result<Self, &'static str> {
        let end = start.checked_add(size).ok_or("range overflow")?;
        if size == 0 || start % 8 != 0 || end % 8 != 0 {
            return Err("empty or unaligned range");
        }
        Ok(Self { start, end })
    }
    /// `[addr, addr + size)` rounded outward to whole words (for a DTB or
    /// other byte-granular range that must be kept).
    pub fn covering(addr: u64, size: u64) -> Result<Self, &'static str> {
        let end = addr
            .checked_add(size)
            .and_then(|v| v.checked_add(7))
            .ok_or("range overflow")?
            & !7;
        if size == 0 {
            return Err("empty range");
        }
        Ok(Self {
            start: addr & !7,
            end,
        })
    }
    pub fn overlaps(self, other: Self) -> bool {
        self.start < other.end && other.start < self.end
    }
    pub fn bytes(self) -> u64 {
        self.end - self.start
    }
}

/// The ranges a test may touch: DRAM minus the reserved ranges.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Plan {
    pub regions: [Region; MAX_REGIONS],
    pub count: usize,
    pub bytes: u64,
}
impl Plan {
    /// Plan a test of `dram` that leaves out `reserved` (each must lie in
    /// `dram`; they must not overlap). Errors: "too many reserved ranges",
    /// "reserved range outside DRAM", "reserved ranges overlap", "no testable
    /// DRAM".
    pub fn new(dram: Region, reserved: &[Region]) -> Result<Self, &'static str> {
        if reserved.len() > MAX_RESERVED {
            return Err("too many reserved ranges");
        }
        let mut excluded = [Region::default(); MAX_RESERVED];
        excluded[..reserved.len()].copy_from_slice(reserved);
        let excluded = &mut excluded[..reserved.len()];
        excluded.sort_unstable_by_key(|r| r.start);
        let mut cursor = dram.start;
        let mut plan = Self {
            regions: [Region::default(); MAX_REGIONS],
            count: 0,
            bytes: 0,
        };
        for (i, r) in excluded.iter().copied().enumerate() {
            if r.start < dram.start || r.end > dram.end || r.start >= r.end {
                return Err("reserved range outside DRAM");
            }
            if i > 0 && r.overlaps(excluded[i - 1]) {
                return Err("reserved ranges overlap");
            }
            if r.start > cursor {
                plan.push(Region {
                    start: cursor,
                    end: r.start,
                });
            }
            cursor = r.end;
        }
        if cursor < dram.end {
            plan.push(Region {
                start: cursor,
                end: dram.end,
            });
        }
        if plan.bytes == 0 {
            return Err("no testable DRAM");
        }
        Ok(plan)
    }
    /// Plan a test of several DRAM windows (a SoC that maps DRAM in more
    /// than one range, e.g. the K1 at 0 and above 4 GiB), leaving out
    /// `reserved`; each reserved range must lie in one window. For one
    /// window this is [`Plan::new`]. Errors as [`Plan::new`], plus "too many
    /// DRAM windows" and "DRAM windows overlap".
    pub fn windows(dram: &[Region], reserved: &[Region]) -> Result<Self, &'static str> {
        if dram.len() > MAX_REGIONS {
            return Err("too many DRAM windows");
        }
        for (i, a) in dram.iter().enumerate() {
            if dram[..i].iter().any(|b| a.overlaps(*b)) {
                return Err("DRAM windows overlap");
            }
        }
        for r in reserved {
            if !dram
                .iter()
                .any(|w| r.start >= w.start && r.end <= w.end && r.start < r.end)
            {
                return Err("reserved range outside DRAM");
            }
        }
        let mut plan = Self {
            regions: [Region::default(); MAX_REGIONS],
            count: 0,
            bytes: 0,
        };
        for w in dram {
            let mut inside = [Region::default(); MAX_RESERVED];
            let mut n = 0;
            for r in reserved {
                if r.start >= w.start && r.end <= w.end {
                    if n == MAX_RESERVED {
                        return Err("too many reserved ranges");
                    }
                    inside[n] = *r;
                    n += 1;
                }
            }
            match Plan::new(*w, &inside[..n]) {
                Ok(sub) => {
                    for r in sub.ranges() {
                        if plan.count == MAX_REGIONS {
                            return Err("too many DRAM windows");
                        }
                        plan.push(*r);
                    }
                }
                Err("no testable DRAM") => {}
                Err(e) => return Err(e),
            }
        }
        if plan.bytes == 0 {
            return Err("no testable DRAM");
        }
        Ok(plan)
    }
    fn push(&mut self, r: Region) {
        self.bytes += r.end - r.start;
        self.regions[self.count] = r;
        self.count += 1;
    }
    pub fn ranges(&self) -> &[Region] {
        &self.regions[..self.count]
    }
}

/// What each word of a pass holds.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Pattern {
    /// The word's own address: address-line faults and aliasing.
    Address,
    /// Its complement: every bit both ways.
    Inverted,
    /// A xorshift64 sequence over the whole plan: data-dependent and
    /// coupling faults.
    Xorshift,
    /// One set bit walking with the word index: stuck and bridged data lines.
    WalkingOnes,
    /// The walking ones inverted.
    WalkingZeros,
}
impl Pattern {
    pub const ALL: [Pattern; 5] = [
        Pattern::Address,
        Pattern::Inverted,
        Pattern::Xorshift,
        Pattern::WalkingOnes,
        Pattern::WalkingZeros,
    ];
    pub fn name(self) -> &'static str {
        match self {
            Self::Address => "address",
            Self::Inverted => "~address",
            Self::Xorshift => "xorshift",
            Self::WalkingOnes => "walking-1",
            Self::WalkingZeros => "walking-0",
        }
    }
    fn value(self, addr: u64, state: &mut u64) -> u64 {
        match self {
            Self::Address => addr,
            Self::Inverted => !addr,
            Self::Xorshift => {
                *state ^= *state << 13;
                *state ^= *state >> 7;
                *state ^= *state << 17;
                *state
            }
            Self::WalkingOnes => 1 << ((addr / 8) % 64),
            Self::WalkingZeros => !(1 << ((addr / 8) % 64)),
        }
    }
}

/// Error count and the first mismatch (address, expected, read).
#[derive(Default, Debug, Clone, Copy, PartialEq, Eq)]
pub struct Report {
    pub errors: u64,
    pub first: Option<(u64, u64, u64)>,
}
impl Report {
    fn record(&mut self, addr: u64, want: u64, got: u64) {
        self.errors = self.errors.saturating_add(1);
        if self.first.is_none() {
            self.first = Some((addr, want, got));
        }
    }
}

/// Implementations must restrict accesses to owned test memory. `complete`
/// drains writes before verification; watchdog time uses a monotonic counter.
pub trait Memory {
    fn write(&mut self, addr: u64, value: u64);
    fn read(&mut self, addr: u64) -> u64;
    fn complete(&mut self);
    fn ticks(&mut self) -> u64;
    fn kick(&mut self);
    /// Write back and invalidate the cache lines of `[addr, addr + len)`
    /// (K1 Zicbom `cbo.flush`, A7A `DC CIVAC`), then a barrier. The
    /// [`stress::Runner`] calls it after writing each block; a no-op when
    /// the accesses are uncached (the default).
    fn clean_invalidate(&mut self, _addr: u64, _len: u64) {}
    /// Invalidate the cache lines of `[addr, addr + len)` without writing
    /// them back (K1 `cbo.inval`, A7A `DC IVAC`), then a barrier. Called
    /// before reading each block; only on blocks already cleaned.
    fn invalidate(&mut self, _addr: u64, _len: u64) {}
}
impl<T: Memory + ?Sized> Memory for &mut T {
    fn write(&mut self, addr: u64, value: u64) {
        (**self).write(addr, value)
    }
    fn read(&mut self, addr: u64) -> u64 {
        (**self).read(addr)
    }
    fn complete(&mut self) {
        (**self).complete()
    }
    fn ticks(&mut self) -> u64 {
        (**self).ticks()
    }
    fn kick(&mut self) {
        (**self).kick()
    }
    fn clean_invalidate(&mut self, addr: u64, len: u64) {
        (**self).clean_invalidate(addr, len)
    }
    fn invalidate(&mut self, addr: u64, len: u64) {
        (**self).invalidate(addr, len)
    }
}

/// Write *all* regions before reading any back, to catch inter-region aliases.
/// Check elapsed time every 512 words (4 KiB), kick at least once per sweep and
/// when the supplied tick interval elapses. A bus stall is recovered by WDT.
pub fn pass(
    m: &mut impl Memory,
    plan: &Plan,
    pattern: Pattern,
    kick_ticks: u64,
    report: &mut Report,
) {
    let mut last = m.ticks();
    m.kick();
    for writing in [true, false] {
        let mut state = 0x9e37_79b9_7f4a_7c15;
        for r in plan.ranges() {
            let mut addr = r.start;
            while addr < r.end {
                let value = pattern.value(addr, &mut state);
                if writing {
                    m.write(addr, value);
                } else {
                    let got = m.read(addr);
                    if got != value {
                        report.record(addr, value, got);
                    }
                }
                addr += 8;
                if addr & 4095 == 0 || addr == r.end {
                    let now = m.ticks();
                    if now.wrapping_sub(last) >= kick_ticks {
                        m.kick();
                        last = now;
                    }
                }
            }
        }
        m.complete();
        m.kick();
        last = m.ticks();
    }
}

/// Moving inversions (March C- style) with `background`: write it everywhere;
/// ascending, read it and write its complement; descending, read the
/// complement and write it back; then read it again. Finds faults that depend
/// on the neighbours' write order, which a single write-then-read pass
/// cannot. Kicks as [`pass`] does.
pub fn moving_inversions(
    m: &mut impl Memory,
    plan: &Plan,
    background: u64,
    kick_ticks: u64,
    report: &mut Report,
) {
    let mut last = m.ticks();
    m.kick();
    let mut tick = |m: &mut dyn MemoryTicks, addr: u64| {
        if addr & 4095 == 0 {
            let now = m.ticks();
            if now.wrapping_sub(last) >= kick_ticks {
                m.kick();
                last = now;
            }
        }
    };
    for r in plan.ranges() {
        let mut a = r.start;
        while a < r.end {
            m.write(a, background);
            a += 8;
            tick(m, a);
        }
    }
    m.complete();
    for r in plan.ranges() {
        let mut a = r.start;
        while a < r.end {
            let got = m.read(a);
            if got != background {
                report.record(a, background, got);
            }
            m.write(a, !background);
            a += 8;
            tick(m, a);
        }
    }
    m.complete();
    for r in plan.ranges().iter().rev() {
        let mut a = r.end;
        while a > r.start {
            a -= 8;
            let got = m.read(a);
            if got != !background {
                report.record(a, !background, got);
            }
            m.write(a, background);
            tick(m, a);
        }
    }
    m.complete();
    for r in plan.ranges() {
        let mut a = r.start;
        while a < r.end {
            let got = m.read(a);
            if got != background {
                report.record(a, background, got);
            }
            a += 8;
            tick(m, a);
        }
    }
    m.kick();
}

/// The tick/kick part of [`Memory`], object-safe for the kick helper.
trait MemoryTicks {
    fn ticks(&mut self) -> u64;
    fn kick(&mut self);
}
impl<T: Memory> MemoryTicks for T {
    fn ticks(&mut self) -> u64 {
        Memory::ticks(self)
    }
    fn kick(&mut self) {
        Memory::kick(self)
    }
}

/// Why [`quick`] failed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum QuickError {
    /// Size or word count outside the bounded boot footprint.
    Config(&'static str),
    /// A word read back wrong.
    Mismatch {
        addr: usize,
        expected: u32,
        got: u32,
    },
}

/// Most words [`quick`] writes per region (16 KiB). Bounds boot work; this is
/// not full-RAM coverage or a timing margin.
pub const MAX_QUICK_WORDS: usize = 4096;

/// The boot smoke check: `words` incrementing 32-bit patterns at the start
/// of DRAM (`base`) and at its middle (`base + mib / 2` MiB), then both read
/// back, middle first. Catches a dead or swapped channel, rank or high
/// address line, not marginal timing. `mib` must be 2..=4096 (all addresses
/// stay 32-bit) and `words` 1..=[`MAX_QUICK_WORDS`].
pub fn quick<H: RegIo>(hw: &mut H, base: usize, mib: u32, words: usize) -> Result<(), QuickError> {
    if !(2..=4096).contains(&mib) || words == 0 || words > MAX_QUICK_WORDS {
        return Err(QuickError::Config(
            "quick-test size/word count outside bounded boot footprint",
        ));
    }
    let half = ((mib / 2) as usize) << 20;
    for i in 0..words {
        hw.write32(base + 4 * i, 0x0123_4567u32.wrapping_add(i as u32));
        hw.write32(base + half + 4 * i, 0xfedc_ba98u32.wrapping_add(i as u32));
    }
    for i in 0..words {
        for (b, pat) in [(base + half, 0xfedc_ba98u32), (base, 0x0123_4567)] {
            let addr = b + 4 * i;
            let expected = pat.wrapping_add(i as u32);
            let got = hw.read32(addr);
            if got != expected {
                return Err(QuickError::Mismatch {
                    addr,
                    expected,
                    got,
                });
            }
        }
    }
    Ok(())
}

/// What a soak's guard sees between steps.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SoakStatus {
    /// Steps done (one step: one pattern pass or one moving-inversions run).
    pub steps: u32,
    pub errors: u64,
    /// Ticks since the soak started.
    pub elapsed: u64,
}

/// The guard's answer.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Guard {
    Continue,
    /// Stop; the reason is recorded (e.g. "time", "temperature", "errors").
    Stop(&'static str),
}

/// Result of [`soak`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SoakReport {
    pub report: Report,
    pub steps: u32,
    pub stop: &'static str,
}

/// Repeat every [`Pattern`] and a moving-inversions run (background
/// alternating 0 and 0x5555...) until `guard` says stop. The guard runs
/// before every step, so a thermal or time limit acts within one pass; a
/// soak never runs unguarded. `max_steps` bounds it regardless.
pub fn soak(
    m: &mut impl Memory,
    plan: &Plan,
    kick_ticks: u64,
    max_steps: u32,
    mut guard: impl FnMut(&SoakStatus) -> Guard,
) -> SoakReport {
    let start = m.ticks();
    let mut report = Report::default();
    let cycle = Pattern::ALL.len() as u32 + 1;
    for steps in 0..max_steps {
        let status = SoakStatus {
            steps,
            errors: report.errors,
            elapsed: m.ticks().wrapping_sub(start),
        };
        if let Guard::Stop(why) = guard(&status) {
            return SoakReport {
                report,
                steps,
                stop: why,
            };
        }
        let k = steps % cycle;
        if (k as usize) < Pattern::ALL.len() {
            pass(m, plan, Pattern::ALL[k as usize], kick_ticks, &mut report);
        } else {
            let bg = if (steps / cycle) % 2 == 0 {
                0
            } else {
                0x5555_5555_5555_5555
            };
            moving_inversions(m, plan, bg, kick_ticks, &mut report);
        }
    }
    SoakReport {
        report,
        steps: max_steps,
        stop: "steps",
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::{vec, vec::Vec};

    struct Ram {
        words: Vec<u64>,
        fault: Option<u64>,
        alias: bool,
        ticks: u64,
        kicks: u64,
        barriers: u64,
    }
    impl Ram {
        fn new(words: usize) -> Self {
            Self {
                words: vec![0; words],
                fault: None,
                alias: false,
                ticks: 0,
                kicks: 0,
                barriers: 0,
            }
        }
        fn i(&self, a: u64) -> usize {
            (if self.alias { (a / 8) % 4 } else { a / 8 }) as usize
        }
    }
    impl Memory for Ram {
        fn write(&mut self, a: u64, v: u64) {
            let i = self.i(a);
            self.words[i] = v;
        }
        fn read(&mut self, a: u64) -> u64 {
            self.words[self.i(a)] ^ u64::from(self.fault == Some(a))
        }
        fn complete(&mut self) {
            self.barriers += 1;
        }
        fn ticks(&mut self) -> u64 {
            self.ticks += 10;
            self.ticks
        }
        fn kick(&mut self) {
            self.kicks += 1;
        }
    }
    fn plan2() -> Plan {
        Plan::new(Region::new(0, 64).unwrap(), &[Region::new(32, 8).unwrap()]).unwrap()
    }
    #[test]
    fn planner_splits_around_reserved_ranges() {
        let dram = Region::new(0x4000_0000, 4 << 30).unwrap();
        let el3 = Region::new(0x4000_0000, 2 << 20).unwrap();
        let payload = Region::new(0x4100_0000, 4 << 20).unwrap();
        let dtb = Region::covering(0x4200_0003, 9).unwrap();
        let p = Plan::new(dram, &[payload, dtb, el3]).unwrap();
        assert_eq!(
            p.ranges(),
            &[
                Region {
                    start: 0x4020_0000,
                    end: 0x4100_0000
                },
                Region {
                    start: 0x4140_0000,
                    end: 0x4200_0000
                },
                Region {
                    start: 0x4200_0010,
                    end: 0x1_4000_0000
                },
            ]
        );
        assert_eq!(p.bytes, (4 << 30) - (6 << 20) - 16);
        assert_eq!(Plan::new(dram, &[dram]), Err("no testable DRAM"));
        assert_eq!(
            Plan::new(dram, &[el3, Region::new(0x4010_0000, 8 << 20).unwrap()]),
            Err("reserved ranges overlap")
        );
        assert_eq!(
            Plan::new(dram, &[Region::new(0, 8).unwrap()]),
            Err("reserved range outside DRAM")
        );
        assert_eq!(Plan::new(dram, &[el3; 8]), Err("too many reserved ranges"));
    }
    #[test]
    fn every_pattern_finds_a_stuck_bit_and_an_alias() {
        for pat in Pattern::ALL {
            for (fault, alias) in [(None, false), (Some(16), false), (None, true)] {
                let mut m = Ram::new(8);
                m.fault = fault;
                m.alias = alias;
                let mut r = Report::default();
                pass(&mut m, &plan2(), pat, 10, &mut r);
                assert_eq!(
                    r.errors > 0,
                    fault.is_some() || alias,
                    "{pat:?} {fault:?} {alias}"
                );
            }
        }
    }
    #[test]
    fn moving_inversions_finds_faults_in_both_directions() {
        for (fault, alias) in [(None, false), (Some(48), false), (None, true)] {
            let mut m = Ram::new(8);
            m.fault = fault;
            m.alias = alias;
            let mut r = Report::default();
            moving_inversions(&mut m, &plan2(), 0, 10, &mut r);
            assert_eq!(r.errors > 0, fault.is_some() || alias);
            assert_eq!(m.barriers, 3);
        }
    }
    #[test]
    fn soak_obeys_its_guard_and_its_bound() {
        let mut m = Ram::new(8);
        let s = soak(&mut m, &plan2(), 10, 100, |st| {
            if st.steps == 7 {
                Guard::Stop("time")
            } else {
                Guard::Continue
            }
        });
        assert_eq!((s.steps, s.stop, s.report.errors), (7, "time", 0));
        m.fault = Some(8);
        let s = soak(&mut m, &plan2(), 10, 100, |st| {
            if st.errors > 0 {
                Guard::Stop("errors")
            } else {
                Guard::Continue
            }
        });
        assert_eq!((s.steps, s.stop), (1, "errors"));
        let s = soak(&mut Ram::new(8), &plan2(), 10, 3, |_| Guard::Continue);
        assert_eq!((s.steps, s.stop), (3, "steps"));
    }
    struct Bus {
        mem: std::collections::HashMap<usize, u32>,
        stuck: Option<usize>,
        log: Vec<(bool, usize)>,
    }
    impl RegIo for Bus {
        fn read32(&mut self, a: usize) -> u32 {
            self.log.push((false, a));
            self.mem.get(&a).copied().unwrap_or(0) ^ u32::from(self.stuck == Some(a))
        }
        fn write32(&mut self, a: usize, v: u32) {
            self.log.push((true, a));
            self.mem.insert(a, v);
        }
        fn read16(&mut self, _: usize) -> u16 {
            unreachable!()
        }
        fn write16(&mut self, _: usize, _: u16) {
            unreachable!()
        }
        fn udelay(&mut self, _: u32) {}
        fn now_us(&mut self) -> u64 {
            0
        }
        fn log(&mut self, _: core::fmt::Arguments) {}
    }
    #[test]
    fn quick_writes_both_halves_and_reports_the_first_mismatch() {
        let bus = |stuck| Bus {
            mem: Default::default(),
            stuck,
            log: Vec::new(),
        };
        let mut b = bus(None);
        for (mib, words) in [(1, 1), (4097, 1), (2, 0), (2, MAX_QUICK_WORDS + 1)] {
            assert!(matches!(
                quick(&mut b, 0x4000_0000, mib, words),
                Err(QuickError::Config(_))
            ));
        }
        assert!(b.log.is_empty(), "no access before the bounds check");
        assert_eq!(quick(&mut b, 0x4000_0000, 4096, 16), Ok(()));
        assert_eq!(b.log.len(), 64);
        assert_eq!(b.log[1], (true, 0x4000_0000 + (2048 << 20)));
        assert_eq!(b.log[32], (false, 0x4000_0000 + (2048 << 20)));
        let mut b = bus(Some(0x4000_0008));
        assert_eq!(
            quick(&mut b, 0x4000_0000, 8, 4),
            Err(QuickError::Mismatch {
                addr: 0x4000_0008,
                expected: 0x0123_4569,
                got: 0x0123_4568
            })
        );
    }
}
