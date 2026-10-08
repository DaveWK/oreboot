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

#[cfg(test)]
mod tests {
    use super::*;
    fn handoff() -> Handoff {
        Handoff {
            magic: crate::HANDOFF_MAGIC,
            version: crate::HANDOFF_VERSION,
            dram_base: 0x40000000,
            dram_size: 4 << 30,
            soc_version: 0,
            hosc_mhz: 26,
            bt0_done_ticks: 0,
            payload_addr: 0x41000000,
            payload_size: 4 << 20,
            dtb_addr: 0,
            dtb_size: 0,
            cpu_mhz: [0; 3],
            dram_mhz: 400,
        }
    }
    #[test]
    fn exact_four_gib_map_and_protected_images() {
        let mut h = handoff();
        h.dtb_addr = 0x42000003;
        h.dtb_size = 9;
        let p = Plan::from_handoff(&h).unwrap();
        assert_eq!(
            p.ranges(),
            &[
                Region {
                    start: 0x40200000,
                    end: 0x41000000
                },
                Region {
                    start: 0x41400000,
                    end: 0x42000000
                },
                Region {
                    start: 0x42000010,
                    end: 0x140000000
                }
            ]
        );
        assert_eq!(p.bytes, (4 << 30) - (6 << 20) - 16);
    }
    #[test]
    fn reject_bad_handoffs() {
        for field in 0..11 {
            let mut h = handoff();
            match field {
                0 => h.dram_base = 0,
                1 => h.dram_size = 0,
                2 => h.dram_size = u64::MAX,
                3 => h.payload_addr = u64::MAX - 7,
                4 => h.payload_size = 0,
                5 => h.payload_addr = 0x40000000,
                6 => h.payload_addr = 0x140000000,
                7 => h.payload_size = 1,
                8 => h.dtb_addr = 1,
                9 => {
                    h.dtb_addr = h.payload_addr;
                    h.dtb_size = 8
                }
                _ => {
                    h.dtb_addr = u64::MAX;
                    h.dtb_size = 8
                }
            }
            assert!(Plan::from_handoff(&h).is_err(), "field {field}");
        }
    }
    struct Ram {
        words: [u64; 8],
        fault: bool,
        alias: bool,
        ticks: u64,
        kicks: u64,
        barriers: u64,
    }
    impl Memory for Ram {
        fn write(&mut self, a: u64, v: u64) {
            let i = if self.alias { (a / 8) % 4 } else { a / 8 };
            self.words[i as usize] = v;
        }
        fn read(&mut self, a: u64) -> u64 {
            let i = if self.alias { (a / 8) % 4 } else { a / 8 };
            self.words[i as usize] ^ u64::from(self.fault && a == 16)
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
    #[test]
    fn real_passes_find_faults_and_cross_region_aliases() {
        let p = Plan {
            regions: [
                Region { start: 0, end: 32 },
                Region { start: 32, end: 64 },
                Region::default(),
                Region::default(),
            ],
            count: 2,
            bytes: 64,
        };
        for (fault, alias) in [(false, false), (true, false), (false, true)] {
            let mut m = Ram {
                words: [0; 8],
                fault,
                alias,
                ticks: 0,
                kicks: 0,
                barriers: 0,
            };
            let mut r = Report::default();
            for pat in [Pattern::Address, Pattern::Inverted, Pattern::Xorshift] {
                pass(&mut m, &p, pat, 10, &mut r);
            }
            assert_eq!(r.errors > 0, fault || alias);
            assert_eq!(m.barriers, 6);
            assert!(m.kicks >= 15);
            if fault {
                assert_eq!(r.errors, 3);
                assert_eq!(r.first, Some((16, 16, 17)));
            }
        }
    }
}
