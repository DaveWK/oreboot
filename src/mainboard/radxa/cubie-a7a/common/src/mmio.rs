//! [`regio::RegIo`] on the SoC's MMIO, for the shared crates that drive
//! registers through it (UART). Delays are the arch counter's; there is no
//! log sink here (this is what the logger itself writes through).

use core::fmt;
use regio::RegIo;
use util::mmio::{read16, read32, write16, write32};

/// Plain MMIO.
#[derive(Clone, Copy, Debug, Default)]
pub struct Mmio;

impl RegIo for Mmio {
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
        crate::time::udelay(us as u64)
    }
    fn now_us(&mut self) -> u64 {
        crate::time::ticks() / (crate::time::TIMER_HZ / 1_000_000)
    }
    fn log(&mut self, _args: fmt::Arguments) {}
}
