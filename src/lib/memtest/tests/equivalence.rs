// SPDX-License-Identifier: GPL-2.0-only
//! The crate against the verbatim A7A board code (tests/legacy/):
//! - the A7A plan (Plan::from_handoff) against the same policy written on
//!   memtest::Plan::new, the way the board will call it, over random handoffs;
//! - pass() over random plans and the three A7A patterns: same Memory calls;
//! - quick() against size.rs quick_test: same accesses and results.
#[path = "legacy/a7a_memtest.rs"]
mod legacy;
#[path = "legacy/a7a_quick_test.rs"]
mod legacy_quick;

use memtest::{Plan, Region};

// stub: the A7A memory map constants and handoff fields the legacy planner uses.
pub mod mem {
    pub const DRAM_BASE: usize = 0x4000_0000;
    pub const EL3_BASE: usize = DRAM_BASE;
    pub const EL3_SIZE: usize = 0x0020_0000;
}
pub struct Handoff {
    pub dram_base: u64,
    pub dram_size: u64,
    pub payload_addr: u64,
    pub payload_size: u64,
    pub dtb_addr: u64,
    pub dtb_size: u64,
}

/// The A7A policy on the shared planner.
fn a7a_plan(h: &Handoff) -> Result<Plan, &'static str> {
    if h.dram_base != mem::DRAM_BASE as u64 || h.dram_size > 4 << 30 {
        return Err("unexpected A7A DRAM map");
    }
    let dram = Region::new(h.dram_base, h.dram_size)?;
    let el3 = Region::new(mem::EL3_BASE as u64, mem::EL3_SIZE as u64)?;
    let payload = Region::new(h.payload_addr, h.payload_size)?;
    if h.dtb_addr != 0 || h.dtb_size != 0 {
        if h.dtb_addr
            .checked_add(h.dtb_size)
            .and_then(|v| v.checked_add(7))
            .is_none()
        {
            return Err("DTB overflow");
        }
        if h.dtb_addr == 0 || h.dtb_size == 0 {
            return Err("invalid DTB range");
        }
        let dtb = Region::covering(h.dtb_addr, h.dtb_size)?;
        return Plan::new(dram, &[el3, payload, dtb]);
    }
    Plan::new(dram, &[el3, payload])
}

struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }
    fn below(&mut self, n: u64) -> u64 {
        self.next() % n
    }
    fn pick(&mut self, v: &[u64]) -> u64 {
        v[self.below(v.len() as u64) as usize]
    }
}

#[test]
fn a7a_plans_match() {
    let mut r = Rng(0x3e3e_7e57_0000_0001);
    let mut ok = 0;
    for _ in 0..200_000 {
        let h = Handoff {
            dram_base: r.pick(&[0x4000_0000, 0x4000_0000, 0x4000_0000, 0]),
            dram_size: r.pick(&[
                4 << 30,
                2 << 30,
                1 << 30,
                0,
                u64::MAX,
                6 << 30,
                (4 << 30) - 8,
            ]),
            payload_addr: r
                .pick(&[
                    0x4100_0000,
                    0x4020_0000,
                    0x4000_0000,
                    0x1_3ff0_0000,
                    u64::MAX - 7,
                    0x4100_0004,
                ])
                .wrapping_add(8 * r.below(4)),
            payload_size: r.pick(&[4 << 20, 0, 1, 8, 1 << 30]),
            dtb_addr: r.pick(&[
                0,
                0,
                0x4200_0003,
                0x4100_0000,
                0x4010_0000,
                u64::MAX,
                0x4100_1000,
            ]),
            dtb_size: r.pick(&[0, 9, 8, 0x1_0000, u64::MAX]),
        };
        let a = a7a_plan(&h);
        let l = legacy::Plan::from_handoff(&legacy_handoff(&h));
        match (&a, &l) {
            (Ok(a), Ok(l)) => {
                ok += 1;
                assert_eq!(a.bytes, l.bytes);
                let la: Vec<_> = l.ranges().iter().map(|x| (x.start, x.end)).collect();
                let aa: Vec<_> = a.ranges().iter().map(|x| (x.start, x.end)).collect();
                assert_eq!(aa, la);
            }
            (Err(a), Err(l)) => assert_eq!(a, l),
            _ => panic!(
                "{:?} vs {:?}",
                a.as_ref().map(|p| p.bytes),
                l.as_ref().map(|p| p.bytes)
            ),
        }
    }
    assert!(ok > 1000, "only {ok} plans");
}

fn legacy_handoff(h: &Handoff) -> Handoff {
    Handoff { ..*h }
}
impl Copy for Handoff {}
impl Clone for Handoff {
    fn clone(&self) -> Self {
        *self
    }
}

#[derive(Default)]
struct Trace {
    words: std::collections::HashMap<u64, u64>,
    stuck: Option<u64>,
    t: u64,
    log: Vec<(u8, u64, u64)>,
}
impl Trace {
    fn w(&mut self, a: u64, v: u64) {
        self.log.push((0, a, v));
        self.words.insert(a, v);
    }
    fn r(&mut self, a: u64) -> u64 {
        let v = self.words.get(&a).copied().unwrap_or(0) ^ u64::from(self.stuck == Some(a));
        self.log.push((1, a, v));
        v
    }
}
impl memtest::Memory for Trace {
    fn write(&mut self, a: u64, v: u64) {
        self.w(a, v)
    }
    fn read(&mut self, a: u64) -> u64 {
        self.r(a)
    }
    fn complete(&mut self) {
        self.log.push((2, 0, 0))
    }
    fn ticks(&mut self) -> u64 {
        self.t += 7;
        self.log.push((3, self.t, 0));
        self.t
    }
    fn kick(&mut self) {
        self.log.push((4, 0, 0))
    }
}
impl legacy::Memory for Trace {
    fn write(&mut self, a: u64, v: u64) {
        self.w(a, v)
    }
    fn read(&mut self, a: u64) -> u64 {
        self.r(a)
    }
    fn complete(&mut self) {
        self.log.push((2, 0, 0))
    }
    fn ticks(&mut self) -> u64 {
        self.t += 7;
        self.log.push((3, self.t, 0));
        self.t
    }
    fn kick(&mut self) {
        self.log.push((4, 0, 0))
    }
}

#[test]
fn passes_match() {
    let mut r = Rng(0x9a55_0000_0000_0001);
    for _ in 0..400 {
        let base = 8 * r.below(1024);
        let size = 8 * (1 + r.below(2048));
        let dram = Region::new(base, size).unwrap();
        let mut reserved = Vec::new();
        for _ in 0..r.below(3) {
            let s = base + 8 * r.below(size / 8);
            let n = 8 * (1 + r.below(64));
            if let Ok(x) = Region::new(s, n) {
                if x.end <= dram.end && !reserved.iter().any(|y: &Region| y.overlaps(x)) {
                    reserved.push(x);
                }
            }
        }
        let Ok(plan) = Plan::new(dram, &reserved) else {
            continue;
        };
        let mut lp = legacy::Plan {
            regions: [legacy::Region::default(); 4],
            count: 0,
            bytes: 0,
        };
        if plan.count > 4 {
            continue;
        }
        for (i, x) in plan.ranges().iter().enumerate() {
            lp.regions[i] = legacy::Region {
                start: x.start,
                end: x.end,
            };
        }
        lp.count = plan.count;
        lp.bytes = plan.bytes;
        let stuck = if r.below(2) == 0 {
            Some(plan.ranges()[0].start)
        } else {
            None
        };
        let kick = [1, 50, 1000][r.below(3) as usize];
        for (np, lpat) in [
            (memtest::Pattern::Address, legacy::Pattern::Address),
            (memtest::Pattern::Inverted, legacy::Pattern::Inverted),
            (memtest::Pattern::Xorshift, legacy::Pattern::Xorshift),
        ] {
            let mut a = Trace {
                stuck,
                ..Default::default()
            };
            let mut b = Trace {
                stuck,
                ..Default::default()
            };
            let mut ra = memtest::Report::default();
            let mut rb = legacy::Report::default();
            memtest::pass(&mut a, &plan, np, kick, &mut ra);
            legacy::pass(&mut b, &lp, lpat, kick, &mut rb);
            assert_eq!(a.log, b.log);
            assert_eq!((ra.errors, ra.first), (rb.errors, rb.first));
            assert_eq!(np.name(), lpat.name());
        }
    }
}

#[derive(Default)]
struct Bus {
    mem: std::collections::HashMap<usize, u32>,
    stuck: Option<usize>,
    log: Vec<(bool, usize, u32)>,
}
impl Bus {
    fn r(&mut self, a: usize) -> u32 {
        let v = self.mem.get(&a).copied().unwrap_or(0) ^ u32::from(self.stuck == Some(a));
        self.log.push((false, a, v));
        v
    }
    fn w(&mut self, a: usize, v: u32) {
        self.log.push((true, a, v));
        self.mem.insert(a, v);
    }
}
impl dram_train::io::RegIo for Bus {
    fn read32(&mut self, a: usize) -> u32 {
        self.r(a)
    }
    fn write32(&mut self, a: usize, v: u32) {
        self.w(a, v)
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
impl legacy_quick::Hw for Bus {
    fn read32(&mut self, a: usize) -> u32 {
        self.r(a)
    }
    fn write32(&mut self, a: usize, v: u32) {
        self.w(a, v)
    }
}

#[test]
fn quick_matches_quick_test() {
    let mut r = Rng(0x9c1c_0000_0000_0001);
    for _ in 0..3000 {
        let mib = r.pick(&[0, 1, 2, 3, 64, 1024, 2048, 4095, 4096, 4097]) as u32;
        let words = r.pick(&[0, 1, 2, 17, 64, 4096, 4097]) as usize;
        let stuck = match r.below(3) {
            0 => None,
            1 => Some(0x4000_0000 + 4 * r.below(words.max(1) as u64) as usize),
            _ => Some(
                0x4000_0000
                    + ((mib as usize / 2) << 20)
                    + 4 * r.below(words.max(1) as u64) as usize,
            ),
        };
        let mut a = Bus {
            stuck,
            ..Default::default()
        };
        let mut b = Bus {
            stuck,
            ..Default::default()
        };
        let ra = memtest::quick(&mut a, 0x4000_0000, mib, words);
        let rb = legacy_quick::quick_test(&mut b, mib, words);
        assert_eq!(a.log, b.log, "mib {mib} words {words}");
        match (ra, rb) {
            (Ok(()), Ok(())) => {}
            (Err(memtest::QuickError::Config(x)), Err(legacy_quick::Error::Config(y))) => {
                assert_eq!(x, y)
            }
            (
                Err(memtest::QuickError::Mismatch {
                    addr,
                    expected,
                    got,
                }),
                Err(legacy_quick::Error::Test {
                    addr: a2,
                    expected: e2,
                    got: g2,
                }),
            ) => {
                assert_eq!((addr, expected, got), (a2, e2, g2))
            }
            (x, y) => panic!("{x:?} vs {y:?}"),
        }
    }
}
