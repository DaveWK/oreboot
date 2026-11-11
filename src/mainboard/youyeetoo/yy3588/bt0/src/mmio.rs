// SPDX-License-Identifier: GPL-2.0-only
//! MMIO register access for bt0: volatile word, half-word and byte
//! accesses (SDHCI byte registers need real byte accesses), the ARM
//! generic counter (24 MHz, CNTFRQ set by start) as the microsecond clock,
//! and the console as the log.

use crate::uart::Uart2;
use core::fmt::{self, Write};
use regio::RegIo;

pub struct Mmio {
    pub uart: Uart2,
}

/// RK3588 secure timer 0 (STIMER), the source of the ARM generic counter.
const STIMER: usize = 0xfd8c_8000;

/// Start the counter behind CNTPCT_EL0 if nothing before bt0 did. Out of
/// the BootROM (own DDR init, bt0 as the init image) it stands still and
/// every [`RegIo::udelay`] would spin forever; after the rkbin DDR init it
/// already runs and is left alone. Sequence and register offsets: U-Boot
/// arch/arm/mach-rockchip/rk3588/rk3588.c rockchip_stimer_init()
/// (CONFIG_ROCKCHIP_STIMER_BASE 0xfd8c8000): load count 0x14 / 0x18 all
/// ones, then control 0x4 bit 0.
pub fn stimer_start() {
    // SAFETY: fixed SoC registers, single core, no other user yet.
    unsafe {
        if core::ptr::read_volatile((STIMER + 0x4) as *const u32) & 1 != 0 {
            return;
        }
        core::ptr::write_volatile((STIMER + 0x14) as *mut u32, 0xffff_ffff);
        core::ptr::write_volatile((STIMER + 0x18) as *mut u32, 0xffff_ffff);
        core::ptr::write_volatile((STIMER + 0x4) as *mut u32, 1);
    }
}

fn cntpct() -> u64 {
    let v: u64;
    unsafe { core::arch::asm!("isb", "mrs {}, cntpct_el0", out(reg) v) };
    v
}

impl RegIo for Mmio {
    fn read32(&mut self, addr: usize) -> u32 {
        unsafe { core::ptr::read_volatile(addr as *const u32) }
    }
    fn write32(&mut self, addr: usize, val: u32) {
        unsafe { core::ptr::write_volatile(addr as *mut u32, val) }
    }
    fn read16(&mut self, addr: usize) -> u16 {
        unsafe { core::ptr::read_volatile(addr as *const u16) }
    }
    fn write16(&mut self, addr: usize, val: u16) {
        unsafe { core::ptr::write_volatile(addr as *mut u16, val) }
    }
    fn read8(&mut self, addr: usize) -> u8 {
        unsafe { core::ptr::read_volatile(addr as *const u8) }
    }
    fn write8(&mut self, addr: usize, val: u8) {
        unsafe { core::ptr::write_volatile(addr as *mut u8, val) }
    }
    fn udelay(&mut self, us: u32) {
        let t = cntpct() + us as u64 * 24;
        while cntpct() < t {}
    }
    fn now_us(&mut self) -> u64 {
        cntpct() / 24
    }
    fn log(&mut self, args: fmt::Arguments) {
        let _ = self.uart.write_fmt(args);
    }
}

impl Write for Mmio {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        self.uart.write_str(s)
    }
}
