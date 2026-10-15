// SPDX-License-Identifier: GPL-2.0-only
//! The K1 bt0's register access for the ore-edk2-boot crates
//! (`regio::RegIo`): volatile MMIO of each width (byte registers such as
//! SDHCI's are accessed alone), a busy-wait and a microsecond clock from the
//! core's cycle counter, and the bt0 log.
//!
//! The crates bt0 uses for storage, USB and the UART keep their own spin and
//! poll budgets; `udelay`/`now_us` only serve crates that time out in
//! microseconds. bt0 leaves the core clock as the BootROM set it, pll1_d4
//! (614.4 MHz, `CPU_C0` mux 0), which `CPU_MHZ` assumes.

use log::print;
use util::mmio;

const CPU_MHZ: u64 = 614;

pub struct K1Io;

impl regio::RegIo for K1Io {
    fn read32(&mut self, addr: usize) -> u32 {
        mmio::read32(addr)
    }
    fn write32(&mut self, addr: usize, val: u32) {
        mmio::write32(addr, val)
    }
    fn read16(&mut self, addr: usize) -> u16 {
        mmio::read16(addr)
    }
    fn write16(&mut self, addr: usize, val: u16) {
        mmio::write16(addr, val)
    }
    fn read8(&mut self, addr: usize) -> u8 {
        mmio::read8(addr)
    }
    fn write8(&mut self, addr: usize, val: u8) {
        mmio::write8(addr, val)
    }
    fn udelay(&mut self, us: u32) {
        let t0 = startup::riscv::cycles();
        while startup::riscv::cycles().wrapping_sub(t0) < us as u64 * CPU_MHZ {
            core::hint::spin_loop();
        }
    }
    fn now_us(&mut self) -> u64 {
        startup::riscv::cycles() / CPU_MHZ
    }
    fn log(&mut self, args: core::fmt::Arguments) {
        print!("{args}");
    }
}
