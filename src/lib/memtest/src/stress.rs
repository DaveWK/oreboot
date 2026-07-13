// SPDX-License-Identifier: GPL-2.0-only
//! Full-coverage stress test: address-pure 64-bit patterns, march tests in
//! both address directions, per-data-line walks, cached runs with explicit
//! cache maintenance per block, plan slices for several harts or cores, and
//! a bounded error log (address, expected, read, XOR).
//!
//! Every pattern value is a function of the word's address (and a seed), so
//! any slice of a plan can be filled and verified on its own and a failure
//! can be reproduced from the address alone.
//!
//! # Caches
//!
//! [`Memory`] accesses may be cached. The runner calls
//! [`Memory::clean_invalidate`] on each block after writing it (the data
//! reaches DRAM and leaves the cache) and [`Memory::invalidate`] on each
//! block before reading it (no line, prefetched or stale, answers for DRAM).
//! Boards implement them with their cache-maintenance instructions (K1:
//! Zicbom `cbo.flush`/`cbo.inval`; A7A: `DC CIVAC`/`DC IVAC`, each by cache
//! line, then a barrier). With caches off they stay the default no-ops.
//!
//! # Several harts
//!
//! [`Plan::slice`] splits a plan into `n` disjoint, aligned slices. Each
//! hart runs the same [`Step`] list on its own slice with its own
//! [`ErrorLog`]; the logs are merged with [`ErrorLog::merge`]. To catch
//! address aliases that cross slices, run [`Runner::fill`] on every hart,
//! meet at a barrier, then [`Runner::verify`].
//!
//! # Recommended profile
//!
//! [`PROFILE_FAST`] is the "few minutes for 2 GiB" set, in this order:
//!
//! | # | Step | Sweeps | Catches |
//! |---|---|---|---|
//! | 1 | own address, both halves | 2 | address lines, aliasing, decoder faults (first: fails fast on a wrong map) |
//! | 2 | checkerboard, inverted checkerboard | 4 | every cell at 0 and 1, adjacent-bit and adjacent-word shorts |
//! | 3 | walking ones, walking zeros | 4 | stuck or bridged data lines, every lane against the others |
//! | 4 | per-data-line walk on 1 MiB per range | 128 x 1 MiB | one data line high (low) with all others low (high), all 64 lines |
//! | 5 | seeded random | 2 | data-dependent faults; the seed is in the step |
//! | 6 | March C- (background 0) | 10 | coupling faults in both address orders, transition faults |
//!
//! That is 22 full sweeps plus 256 MiB of line walks: about 44 GiB of
//! traffic for 2 GiB. On the K1 or A733 with caches on and per-block
//! maintenance this is a few minutes on one hart and scales with the harts
//! given slices. Longer soaks append [`Step::March`] with background
//! `0x5555_5555_5555_5555` and [`Step::Check`] of [`Stress::Random`] with new
//! seeds.

use crate::{Guard, Memory, Plan, Region, Report, SoakStatus};

/// Default seed of [`Stress::Random`] in [`PROFILE_FAST`].
pub const DEFAULT_SEED: u64 = 0x6f72_652d_6564_6b32;

/// 64-bit stress patterns; each word's value depends only on its address.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Stress {
    /// Own address in both halves: the low 32 bits are the address, the
    /// high 32 bits its complement XOR the address bits 32..63. Every word
    /// differs, and both halves of the bus carry the address.
    OwnAddress,
    /// `0x5555...` on even words, `0xaaaa...` on odd ones (inverted: the
    /// opposite): every bit differs from its neighbour bits and from the
    /// same bit in the neighbour words.
    Checkerboard { inverted: bool },
    /// One bit set (clear for `ones: false`) walking with the word index:
    /// each run of 64 words puts each data line high against the others.
    Walk { ones: bool },
    /// Every word has only `bit` set (`ones`) or only `bit` clear: one data
    /// line against all others, everywhere.
    Line { bit: u8, ones: bool },
    /// A seeded hash of the address (splitmix64): reproducible from the
    /// seed, independent of slicing.
    Random { seed: u64 },
}

impl Stress {
    pub fn name(self) -> &'static str {
        match self {
            Self::OwnAddress => "own-address",
            Self::Checkerboard { inverted: false } => "checkerboard",
            Self::Checkerboard { inverted: true } => "~checkerboard",
            Self::Walk { ones: true } => "walking-1",
            Self::Walk { ones: false } => "walking-0",
            Self::Line { ones: true, .. } => "line-1",
            Self::Line { ones: false, .. } => "line-0",
            Self::Random { .. } => "random",
        }
    }

    /// The value of the word at `addr`.
    #[inline(always)]
    pub fn value(self, addr: u64) -> u64 {
        match self {
            Self::OwnAddress => {
                let lo = addr & 0xffff_ffff;
                let hi = (!lo ^ (addr >> 32)) & 0xffff_ffff;
                (hi << 32) | lo
            }
            Self::Checkerboard { inverted } => {
                let v = if (addr / 8) % 2 == 0 {
                    0x5555_5555_5555_5555
                } else {
                    0xaaaa_aaaa_aaaa_aaaa
                };
                if inverted {
                    !v
                } else {
                    v
                }
            }
            Self::Walk { ones } => {
                let v = 1u64 << ((addr / 8) % 64);
                if ones {
                    v
                } else {
                    !v
                }
            }
            Self::Line { bit, ones } => {
                let v = 1u64 << (bit % 64);
                if ones {
                    v
                } else {
                    !v
                }
            }
            Self::Random { seed } => splitmix64(addr ^ seed),
        }
    }
}

/// splitmix64 finaliser of `x + golden`.
#[inline(always)]
pub fn splitmix64(x: u64) -> u64 {
    let mut z = x.wrapping_add(0x9e37_79b9_7f4a_7c15);
    z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
    z ^ (z >> 31)
}

/// One step of a run.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Step {
    /// Fill the plan with the pattern, then verify it.
    Check(Stress),
    /// March C- with `background` and its complement.
    March { background: u64 },
    /// [`Stress::Line`] for all 64 lines, ones then zeros, over the first
    /// `window` bytes of each range (data-line faults do not depend on the
    /// address, so a window is enough).
    Lines { window: u64 },
}

impl Step {
    pub fn name(self) -> &'static str {
        match self {
            Self::Check(s) => s.name(),
            Self::March { .. } => "march-c-",
            Self::Lines { .. } => "lines",
        }
    }

    /// Bytes this step reads and writes on `plan` (traffic, for estimates).
    pub fn traffic(self, plan: &Plan) -> u64 {
        match self {
            Self::Check(_) => 2 * plan.bytes,
            Self::March { .. } => 10 * plan.bytes,
            Self::Lines { window } => {
                let w: u64 = plan.ranges().iter().map(|r| r.bytes().min(window)).sum();
                2 * 128 * w
            }
        }
    }
}

/// The recommended "few minutes for 2 GiB" profile (see the module docs).
pub const PROFILE_FAST: [Step; 8] = [
    Step::Check(Stress::OwnAddress),
    Step::Check(Stress::Checkerboard { inverted: false }),
    Step::Check(Stress::Checkerboard { inverted: true }),
    Step::Check(Stress::Walk { ones: true }),
    Step::Check(Stress::Walk { ones: false }),
    Step::Lines { window: 1 << 20 },
    Step::Check(Stress::Random { seed: DEFAULT_SEED }),
    Step::March { background: 0 },
];

/// Bytes a step list reads and writes on `plan`.
pub fn traffic(steps: &[Step], plan: &Plan) -> u64 {
    steps.iter().map(|s| s.traffic(plan)).sum()
}

/// A word that read back wrong.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Mismatch {
    pub addr: u64,
    pub expected: u64,
    pub got: u64,
}
impl Mismatch {
    /// The failing bits.
    pub fn xor(self) -> u64 {
        self.expected ^ self.got
    }
}

/// Where mismatches go: the [`Report`] of the original passes, or an
/// [`ErrorLog`].
pub trait Record {
    fn record(&mut self, addr: u64, expected: u64, got: u64);
    fn errors(&self) -> u64;
}
impl Record for Report {
    fn record(&mut self, addr: u64, expected: u64, got: u64) {
        Report::record(self, addr, expected, got)
    }
    fn errors(&self) -> u64 {
        self.errors
    }
}

/// Bounded error log: the first `N` mismatches in full, and over all of
/// them the count, the OR of the failing bits and the address range.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ErrorLog<const N: usize> {
    pub errors: u64,
    pub kept: [Mismatch; N],
    pub len: usize,
    /// OR of every mismatch's XOR: the data lines that ever failed.
    pub bits: u64,
    /// Lowest and highest failing address.
    pub low: u64,
    pub high: u64,
}
impl<const N: usize> Default for ErrorLog<N> {
    fn default() -> Self {
        Self {
            errors: 0,
            kept: [Mismatch::default(); N],
            len: 0,
            bits: 0,
            low: u64::MAX,
            high: 0,
        }
    }
}
impl<const N: usize> ErrorLog<N> {
    pub fn entries(&self) -> &[Mismatch] {
        &self.kept[..self.len]
    }
    /// Fold another log (another hart's slice) into this one.
    pub fn merge<const M: usize>(&mut self, other: &ErrorLog<M>) {
        let errors = self.errors.saturating_add(other.errors);
        for m in other.entries() {
            self.keep(*m);
        }
        self.errors = errors;
        self.bits |= other.bits;
        self.low = self.low.min(other.low);
        self.high = self.high.max(other.high);
    }
    fn keep(&mut self, m: Mismatch) {
        if self.len < N {
            self.kept[self.len] = m;
            self.len += 1;
        }
    }
}
impl<const N: usize> Record for ErrorLog<N> {
    fn record(&mut self, addr: u64, expected: u64, got: u64) {
        let m = Mismatch {
            addr,
            expected,
            got,
        };
        self.keep(m);
        self.errors = self.errors.saturating_add(1);
        self.bits |= m.xor();
        self.low = self.low.min(addr);
        self.high = self.high.max(addr);
    }
    fn errors(&self) -> u64 {
        self.errors
    }
}

impl Plan {
    /// Slice `k` of `n` (0-based): the plan's bytes split into `n` parts at
    /// offsets rounded down to `align` (a power of two, at least 8; use the
    /// cache-maintenance block). The slices are disjoint and together are
    /// the plan; a slice may be empty when the plan is small.
    pub fn slice(&self, n: usize, k: usize, align: u64) -> Result<Plan, &'static str> {
        if n == 0 || k >= n {
            return Err("slice index outside 0..n");
        }
        if align < 8 || !align.is_power_of_two() {
            return Err("slice alignment must be a power of two of at least 8");
        }
        let cut = |i: usize| -> u64 {
            if i == n {
                self.bytes
            } else {
                ((self.bytes as u128 * i as u128 / n as u128) as u64) & !(align - 1)
            }
        };
        let (from, to) = (cut(k), cut(k + 1));
        let mut out = Plan {
            regions: [Region::default(); crate::MAX_REGIONS],
            count: 0,
            bytes: 0,
        };
        let mut off = 0;
        for r in self.ranges() {
            let (s, e) = (off, off + r.bytes());
            let (a, b) = (from.max(s), to.min(e));
            if a < b {
                out.regions[out.count] = Region {
                    start: r.start + (a - s),
                    end: r.start + (b - s),
                };
                out.count += 1;
                out.bytes += b - a;
            }
            off = e;
        }
        Ok(out)
    }
}

/// Runs stress steps on one plan (or slice) with per-block cache
/// maintenance and watchdog kicks.
pub struct Runner<'a, M: Memory> {
    pub mem: &'a mut M,
    /// Bytes per cache-maintenance block (a multiple of 64; e.g. 64 KiB).
    pub block: u64,
    /// Kick the watchdog when this many ticks passed (checked per block).
    pub kick_ticks: u64,
    last: u64,
}

impl<'a, M: Memory> Runner<'a, M> {
    pub fn new(mem: &'a mut M, block: u64, kick_ticks: u64) -> Result<Self, &'static str> {
        if block < 64 || block % 64 != 0 {
            return Err("block must be a non-zero multiple of 64 bytes");
        }
        let last = mem.ticks();
        mem.kick();
        Ok(Self {
            mem,
            block,
            kick_ticks,
            last,
        })
    }

    fn pace(&mut self) {
        let now = self.mem.ticks();
        if now.wrapping_sub(self.last) >= self.kick_ticks {
            self.mem.kick();
            self.last = now;
        }
    }

    /// Blocks of `r` in ascending or descending order.
    fn blocks(&self, r: Region, up: bool) -> Blocks {
        Blocks {
            r,
            block: self.block,
            next: if up { r.start } else { r.end },
            up,
        }
    }

    /// Write `pat` over the plan (every range, ascending).
    pub fn fill(&mut self, plan: &Plan, pat: Stress) {
        // One loop per pattern kind, so the per-word value is not a match.
        match pat {
            Stress::OwnAddress => self.fill_with(plan, |a| Stress::OwnAddress.value(a)),
            Stress::Checkerboard { inverted } => {
                self.fill_with(plan, |a| Stress::Checkerboard { inverted }.value(a))
            }
            Stress::Walk { ones } => self.fill_with(plan, |a| Stress::Walk { ones }.value(a)),
            Stress::Line { .. } => {
                let v = pat.value(0);
                self.fill_with(plan, |_| v)
            }
            Stress::Random { seed } => self.fill_with(plan, |a| splitmix64(a ^ seed)),
        }
    }

    fn fill_with(&mut self, plan: &Plan, f: impl Fn(u64) -> u64) {
        for r in plan.ranges() {
            for b in self.blocks(*r, true) {
                let mut a = b.start;
                while a < b.end {
                    self.mem.write(a, f(a));
                    a += 8;
                }
                self.mem.clean_invalidate(b.start, b.bytes());
                self.pace();
            }
        }
        self.mem.complete();
    }

    /// Read the plan back against `pat`.
    pub fn verify(&mut self, plan: &Plan, pat: Stress, log: &mut impl Record) {
        match pat {
            Stress::OwnAddress => self.verify_with(plan, |a| Stress::OwnAddress.value(a), log),
            Stress::Checkerboard { inverted } => {
                self.verify_with(plan, |a| Stress::Checkerboard { inverted }.value(a), log)
            }
            Stress::Walk { ones } => {
                self.verify_with(plan, |a| Stress::Walk { ones }.value(a), log)
            }
            Stress::Line { .. } => {
                let v = pat.value(0);
                self.verify_with(plan, |_| v, log)
            }
            Stress::Random { seed } => self.verify_with(plan, |a| splitmix64(a ^ seed), log),
        }
    }

    fn verify_with(&mut self, plan: &Plan, f: impl Fn(u64) -> u64, log: &mut impl Record) {
        for r in plan.ranges() {
            for b in self.blocks(*r, true) {
                self.mem.invalidate(b.start, b.bytes());
                let mut a = b.start;
                while a < b.end {
                    let want = f(a);
                    let got = self.mem.read(a);
                    if got != want {
                        log.record(a, want, got);
                    }
                    a += 8;
                }
                self.pace();
            }
        }
    }

    /// Fill, then verify (one hart, or slices that need no barrier).
    pub fn check(&mut self, plan: &Plan, pat: Stress, log: &mut impl Record) {
        self.fill(plan, pat);
        self.verify(plan, pat, log);
    }

    /// One march element over the plan: per word, read and expect
    /// `expect` (if any), then write `write` (if any).
    fn element(
        &mut self,
        plan: &Plan,
        up: bool,
        expect: Option<u64>,
        write: Option<u64>,
        log: &mut impl Record,
    ) {
        let n = plan.count;
        for i in 0..n {
            let r = plan.regions[if up { i } else { n - 1 - i }];
            for b in self.blocks(r, up) {
                if expect.is_some() {
                    self.mem.invalidate(b.start, b.bytes());
                }
                let mut k = 0;
                let words = b.bytes() / 8;
                while k < words {
                    let a = if up {
                        b.start + 8 * k
                    } else {
                        b.end - 8 * (k + 1)
                    };
                    if let Some(want) = expect {
                        let got = self.mem.read(a);
                        if got != want {
                            log.record(a, want, got);
                        }
                    }
                    if let Some(v) = write {
                        self.mem.write(a, v);
                    }
                    k += 1;
                }
                if write.is_some() {
                    self.mem.clean_invalidate(b.start, b.bytes());
                }
                self.pace();
            }
        }
        self.mem.complete();
    }

    /// March C-: up(w0); up(r0,w1); up(r1,w0); down(r0,w1); down(r1,w0);
    /// up(r0), with 0 = `background` and 1 its complement. Finds stuck-at,
    /// transition and coupling faults whichever side of the victim the
    /// aggressor is on, and address faults a single fill-verify misses.
    pub fn march(&mut self, plan: &Plan, background: u64, log: &mut impl Record) {
        let (z, o) = (background, !background);
        self.element(plan, true, None, Some(z), log);
        self.element(plan, true, Some(z), Some(o), log);
        self.element(plan, true, Some(o), Some(z), log);
        self.element(plan, false, Some(z), Some(o), log);
        self.element(plan, false, Some(o), Some(z), log);
        self.element(plan, true, Some(z), None, log);
    }

    /// [`Stress::Line`] for all 64 lines, ones then zeros, on the first
    /// `window` bytes of each range.
    pub fn lines(&mut self, plan: &Plan, window: u64, log: &mut impl Record) {
        let mut w = *plan;
        w.bytes = 0;
        for r in w.regions[..w.count].iter_mut() {
            r.end = r.start + r.bytes().min(window & !7);
            w.bytes += r.bytes();
        }
        for ones in [true, false] {
            for bit in 0..64 {
                self.check(&w, Stress::Line { bit, ones }, log);
            }
        }
    }

    /// One step.
    pub fn step(&mut self, plan: &Plan, step: Step, log: &mut impl Record) {
        match step {
            Step::Check(p) => self.check(plan, p, log),
            Step::March { background } => self.march(plan, background, log),
            Step::Lines { window } => self.lines(plan, window, log),
        }
    }

    /// The steps in order; `guard` runs before each (time, temperature,
    /// errors), as in [`crate::soak`]. Returns the steps done and the stop
    /// reason ("done" when all ran).
    pub fn run(
        &mut self,
        plan: &Plan,
        steps: &[Step],
        log: &mut impl Record,
        mut guard: impl FnMut(&SoakStatus) -> Guard,
    ) -> (u32, &'static str) {
        let start = self.mem.ticks();
        for (i, s) in steps.iter().enumerate() {
            let status = SoakStatus {
                steps: i as u32,
                errors: log.errors(),
                elapsed: self.mem.ticks().wrapping_sub(start),
            };
            if let Guard::Stop(why) = guard(&status) {
                return (i as u32, why);
            }
            self.step(plan, *s, log);
        }
        self.mem.kick();
        (steps.len() as u32, "done")
    }
}

/// Blocks of a region (the last one may be short), either direction.
struct Blocks {
    r: Region,
    block: u64,
    next: u64,
    up: bool,
}
impl Iterator for Blocks {
    type Item = Region;
    fn next(&mut self) -> Option<Region> {
        if self.up {
            if self.next >= self.r.end {
                return None;
            }
            let end = (self.next - self.next % self.block + self.block).min(self.r.end);
            let b = Region {
                start: self.next,
                end,
            };
            self.next = end;
            Some(b)
        } else {
            if self.next <= self.r.start {
                return None;
            }
            let start = ((self.next - 1) - (self.next - 1) % self.block).max(self.r.start);
            let b = Region {
                start,
                end: self.next,
            };
            self.next = start;
            Some(b)
        }
    }
}
