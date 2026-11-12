//! FEL lab entry (feature `fel`, entry 3 at `LINK_BASE + 0x48`): runs a
//! list of register operations the host placed at [`CMD`], writes the
//! results to [`OUT`] and returns to FEL. For DRAM measurements after entry
//! 1 (PHY CSR dumps, single CSR changes, mode register reads through the
//! controller, memory tests) without rebuilding or retraining.
//!
//! Commands are 32-bit little-endian words: an opcode followed by its
//! arguments. The list ends with [`op::END`] or after [`CMD_WORDS`] words.
//! The watchdog runs during the list (a hang resets the board back into
//! FEL) and is stopped before returning.
//!
//! | Op | Arguments | Output words |
//! |----|-----------|--------------|
//! | 1 W16 | csr, value | - |
//! | 2 R16 | csr | value |
//! | 3 W32 | address, value | - |
//! | 4 R32 | address | value |
//! | 5 DELAY | microseconds | - |
//! | 6 MEMTEST | base, bytes, seed | errors, first address, wanted, got, OR of all xors, ms |
//! | 7 MRR | channel, rank, mr | value, or 0xdead_0000 \| error |
//! | 8 R16N | csr, count | count values |
//! | 9 FILL | base, bytes, seed | ms |
//! | 10 CHECK | base, bytes, seed | as MEMTEST |
//! | 11 MRW | channel, rank, mr, value | 0, or 0xdead_0000 \| error |
//!
//! PHY CSRs are given as their APB address (0x20000 = MASTER); the CPU
//! address is `0x0A900000 + 2 * csr`. The CSR mux (MicroContMuxSel) is not
//! changed implicitly: write CSR 0xd0000 = 0 first for host access.

use util::mmio::{read16, read32, write16, write32};

use crate::{recovery, soc};

pub const CMD: usize = 0x000e_0000;
pub const CMD_WORDS: usize = 1024;
pub const OUT: usize = 0x000e_1000;
pub const OUT_WORDS: usize = (0x000e_e000 - OUT) / 4;

pub mod op {
    pub const END: u32 = 0;
    pub const W16: u32 = 1;
    pub const R16: u32 = 2;
    pub const W32: u32 = 3;
    pub const R32: u32 = 4;
    pub const DELAY: u32 = 5;
    pub const MEMTEST: u32 = 6;
    pub const MRR: u32 = 7;
    pub const R16N: u32 = 8;
    pub const FILL: u32 = 9;
    pub const CHECK: u32 = 10;
    pub const MRW: u32 = 11;
}

const PHY_BASE: usize = 0x0a90_0000;

struct LabHw;

impl cubie_a7a_dram::RegIo for LabHw {
    fn read32(&mut self, addr: usize) -> u32 {
        read32(addr)
    }
    fn write32(&mut self, addr: usize, val: u32) {
        write32(addr, val)
    }
    fn read16(&mut self, addr: usize) -> u16 {
        read16(addr)
    }
    fn write16(&mut self, addr: usize, val: u16) {
        write16(addr, val)
    }
    fn udelay(&mut self, us: u32) {
        soc::udelay(us as u64)
    }
    fn now_us(&mut self) -> u64 {
        soc::ticks() / (soc::TIMER_HZ / 1_000_000)
    }
    fn log(&mut self, _args: core::fmt::Arguments) {}
    fn kick_watchdog(&mut self) {
        recovery::kick();
    }
}

impl cubie_a7a_dram::Hw for LabHw {
    fn set_voltage(&mut self, _rail: cubie_a7a_dram::Rail, _mv: u32) -> bool {
        false
    }
}

struct Out {
    n: usize,
}

impl Out {
    fn push(&mut self, v: u32) {
        if self.n < OUT_WORDS {
            write32(OUT + 4 * self.n, v);
            self.n += 1;
        }
    }
}

fn pattern(addr: usize, seed: u32) -> u32 {
    let a = addr as u32;
    let x = a.wrapping_mul(0x9e37_79b1) ^ seed;
    x ^ (x >> 15)
}

fn ms_since(t0: u64) -> u32 {
    ((soc::ticks() - t0) / (soc::TIMER_HZ / 1000)) as u32
}

fn fill(base: usize, bytes: usize, seed: u32) {
    let mut a = base;
    let end = base + bytes;
    while a < end {
        let chunk = (end - a).min(1 << 20);
        let stop = a + chunk;
        while a < stop {
            unsafe { core::ptr::write_volatile(a as *mut u32, pattern(a, seed)) };
            a += 4;
        }
        recovery::kick();
    }
}

/// Errors, first address, wanted, got, OR of all xors.
fn check(base: usize, bytes: usize, seed: u32) -> [u32; 5] {
    let mut r = [0u32; 5];
    let mut a = base;
    let end = base + bytes;
    while a < end {
        let chunk = (end - a).min(1 << 20);
        let stop = a + chunk;
        while a < stop {
            let want = pattern(a, seed);
            let got = unsafe { core::ptr::read_volatile(a as *const u32) };
            if got != want {
                if r[0] == 0 {
                    r[1] = a as u32;
                    r[2] = want;
                    r[3] = got;
                }
                r[0] = r[0].saturating_add(1);
                r[4] |= got ^ want;
            }
            a += 4;
        }
        recovery::kick();
    }
    r
}

/// Run the host's command list. Returns (ops run, output words, error).
pub fn run() -> (u32, u32, u32) {
    let mut out = Out { n: 0 };
    let mut ops = 0;
    let mut i = 0;
    let arg = |k: usize| read32(CMD + 4 * k);
    let mut hw = LabHw;
    while i < CMD_WORDS {
        let opc = arg(i);
        let need = match opc {
            op::END => return (ops, out.n as u32, 0),
            op::W16 | op::W32 | op::R16N => 2,
            op::R16 | op::R32 | op::DELAY => 1,
            op::MEMTEST | op::MRR | op::FILL | op::CHECK => 3,
            op::MRW => 4,
            _ => return (ops, out.n as u32, 0x100 | opc),
        };
        if i + need >= CMD_WORDS {
            return (ops, out.n as u32, 0x200);
        }
        let a = |k: usize| arg(i + 1 + k);
        match opc {
            op::W16 => write16(PHY_BASE + 2 * a(0) as usize, a(1) as u16),
            op::R16 => out.push(read16(PHY_BASE + 2 * a(0) as usize) as u32),
            op::W32 => write32(a(0) as usize, a(1)),
            op::R32 => out.push(read32(a(0) as usize)),
            op::DELAY => {
                let mut us = a(0);
                while us > 0 {
                    let step = us.min(1_000_000);
                    soc::udelay(step as u64);
                    recovery::kick();
                    us -= step;
                }
            }
            op::MEMTEST | op::CHECK => {
                let (base, bytes, seed) = (a(0) as usize, a(1) as usize & !3, a(2));
                let t0 = soc::ticks();
                if opc == op::MEMTEST {
                    fill(base, bytes, seed);
                }
                let r = check(base, bytes, seed);
                for v in r {
                    out.push(v);
                }
                out.push(ms_since(t0));
            }
            op::FILL => {
                let t0 = soc::ticks();
                fill(a(0) as usize, a(1) as usize & !3, a(2));
                out.push(ms_since(t0));
            }
            op::MRR => {
                let v = match cubie_a7a_dram::mr::read(&mut hw, a(0) as usize, a(1), a(2)) {
                    Ok(v) => v,
                    Err(cubie_a7a_dram::Error::Timeout(_)) => 0xdead_0001,
                    Err(cubie_a7a_dram::Error::Config(_)) => 0xdead_0002,
                    Err(_) => 0xdead_0003,
                };
                out.push(v);
            }
            op::MRW => {
                let r = cubie_a7a_dram::mr::write_rank(&mut hw, a(0) as usize, a(1), a(2), a(3));
                out.push(match r {
                    Ok(()) => 0,
                    Err(cubie_a7a_dram::Error::Timeout(_)) => 0xdead_0001,
                    Err(cubie_a7a_dram::Error::Config(_)) => 0xdead_0002,
                    Err(_) => 0xdead_0003,
                });
            }
            op::R16N => {
                let (c, n) = (a(0) as usize, a(1) as usize);
                for k in 0..n {
                    out.push(read16(PHY_BASE + 2 * (c + k)) as u32);
                }
            }
            _ => unreachable!(),
        }
        ops += 1;
        i += 1 + need;
    }
    (ops, out.n as u32, 0x200)
}
