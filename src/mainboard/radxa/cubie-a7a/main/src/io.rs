//! [`regio::RegIo`] for the shared crates main drives (PCIe, PSCI, GIC,
//! PPU): MMIO, the arch counter, and log lines through `println!`.

use core::fmt;
use regio::RegIo;
use util::mmio::{read16, read32, write16, write32};

#[derive(Clone, Copy, Debug, Default)]
pub struct Io;

impl RegIo for Io {
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
        cubie_a7a_common::time::udelay(us as u64)
    }
    fn now_us(&mut self) -> u64 {
        cubie_a7a_common::time::ticks() / (cubie_a7a_common::time::TIMER_HZ / 1_000_000)
    }
    fn log(&mut self, args: fmt::Arguments) {
        println!("[main] {args}");
    }
}
