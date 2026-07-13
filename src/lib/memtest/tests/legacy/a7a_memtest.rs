// SPDX-License-Identifier: GPL-2.0-only
// Verbatim copy of ore-edk-boot-radxa-a7a oreboot 4904019
// src/mainboard/radxa/cubie-a7a/common/src/memtest.rs, the oracle for the
// equivalence tests (its unit tests removed). `crate::{mem, Handoff}` are
// stubs in tests/equivalence.rs with the A7A values.
#![allow(dead_code, clippy::all)]
//! Destructive memory-test planning and execution, shared with host controls.
//! Only explicit, validated DRAM ranges are accessed; EL3 and loaded images
//! are excluded. The platform supplies uncached accesses and write completion.

use crate::{mem, Handoff};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct Region {
    pub start: u64,
    pub end: u64,
}
impl Region {
    fn new(start: u64, size: u64) -> Result<Self, &'static str> {
        let end = start.checked_add(size).ok_or("range overflow")?;
        if size == 0 || start % 8 != 0 || end % 8 != 0 {
            return Err("empty or unaligned range");
        }
        Ok(Self { start, end })
    }
    fn overlaps(self, other: Self) -> bool {
        self.start < other.end && other.start < self.end
    }
}

pub struct Plan {
    pub regions: [Region; 4],
    pub count: usize,
    pub bytes: u64,
}
impl Plan {
    pub fn from_handoff(h: &Handoff) -> Result<Self, &'static str> {
        if h.dram_base != mem::DRAM_BASE as u64 || h.dram_size > 4 << 30 {
            return Err("unexpected A7A DRAM map");
        }
        let dram = Region::new(h.dram_base, h.dram_size)?;
        let el3 = Region::new(mem::EL3_BASE as u64, mem::EL3_SIZE as u64)?;
        let payload = Region::new(h.payload_addr, h.payload_size)?;
        let mut excluded = [el3, payload, Region::default()];
        let mut n = 2;
        if h.dtb_addr != 0 || h.dtb_size != 0 {
            // Preserve the supplied tree even though main currently generates
            // its handoff DTB in EL3 memory. Round outward to whole test words.
            let end = h
                .dtb_addr
                .checked_add(h.dtb_size)
                .and_then(|v| v.checked_add(7))
                .ok_or("DTB overflow")?
                & !7;
            if h.dtb_addr == 0 || h.dtb_size == 0 {
                return Err("invalid DTB range");
            }
            excluded[2] = Region {
                start: h.dtb_addr & !7,
                end,
            };
            n = 3;
        }
        excluded[..n].sort_unstable_by_key(|r| r.start);
        let mut cursor = dram.start;
        let mut plan = Self {
            regions: [Region::default(); 4],
            count: 0,
            bytes: 0,
        };
        for (i, r) in excluded[..n].iter().copied().enumerate() {
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
    fn push(&mut self, r: Region) {
        self.bytes += r.end - r.start;
        self.regions[self.count] = r;
        self.count += 1;
    }
    pub fn ranges(&self) -> &[Region] {
        &self.regions[..self.count]
    }
}

#[derive(Clone, Copy)]
pub enum Pattern {
    Address,
    Inverted,
    Xorshift,
}
impl Pattern {
    pub fn name(self) -> &'static str {
        match self {
            Self::Address => "address",
            Self::Inverted => "~address",
            Self::Xorshift => "xorshift",
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
        }
    }
}

#[derive(Default, Debug)]
pub struct Report {
    pub errors: u64,
    pub first: Option<(u64, u64, u64)>,
}

/// Implementations must restrict accesses to owned test memory. `complete`
/// drains writes before verification; watchdog time uses a monotonic counter.
pub trait Memory {
    fn write(&mut self, addr: u64, value: u64);
    fn read(&mut self, addr: u64) -> u64;
    fn complete(&mut self);
    fn ticks(&mut self) -> u64;
    fn kick(&mut self);
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
                        report.errors = report.errors.saturating_add(1);
                        if report.first.is_none() {
                            report.first = Some((addr, value, got));
                        }
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
