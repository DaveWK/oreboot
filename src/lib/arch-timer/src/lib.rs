// SPDX-License-Identifier: GPL-2.0-only
//! The Arm generic timer's physical counter and delays on it.
//!
//! Moved from ore-edk-boot-radxa-a7a ed42550 (oreboot a803f4f780d7)
//! `src/mainboard/radxa/cubie-a7a/common/src/time.rs`, where the A733's
//! 24 MHz SYS_CLK24M was a constant: here the counter frequency is a
//! parameter of [`Timer`], and the counter is a trait so the delay logic
//! runs in host tests. RISC-V cycle and time counters are in
//! `crates/startup` (`riscv`).
#![no_std]

/// A free-running tick counter.
pub trait Counter {
    fn ticks(&mut self) -> u64;
}

/// The architected physical counter of the CPU this runs on: CNTPCT through
/// `mrrc p15` on AArch32, `cntpct_el0` on AArch64 (both after an `isb`).
/// Other targets (host tests) read 0.
#[derive(Clone, Copy, Debug, Default)]
pub struct ArchCounter;

impl Counter for ArchCounter {
    #[inline]
    fn ticks(&mut self) -> u64 {
        ticks()
    }
}

#[cfg(target_arch = "arm")]
pub fn ticks() -> u64 {
    let lo: u32;
    let hi: u32;
    // CNTPCT
    unsafe { core::arch::asm!("isb", "mrrc p15, 0, {}, {}, c14", out(reg) lo, out(reg) hi) };
    ((hi as u64) << 32) | lo as u64
}

#[cfg(target_arch = "aarch64")]
pub fn ticks() -> u64 {
    let t: u64;
    unsafe { core::arch::asm!("isb", "mrs {}, cntpct_el0", out(reg) t) };
    t
}

/// Host builds have no architected counter.
#[cfg(not(any(target_arch = "arm", target_arch = "aarch64")))]
pub fn ticks() -> u64 {
    0
}

/// A counter running at `hz` ticks per second.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Timer {
    pub hz: u64,
}

impl Timer {
    pub const fn new(hz: u64) -> Self {
        Self { hz }
    }

    /// Ticks in `us` microseconds (whole ticks per microsecond, as the A733
    /// code computed it; exact for MHz-multiple frequencies such as 24 MHz).
    pub const fn us_to_ticks(&self, us: u64) -> u64 {
        us * (self.hz / 1_000_000)
    }

    /// Whole microseconds in `ticks`.
    pub const fn ticks_to_us(&self, ticks: u64) -> u64 {
        ticks / (self.hz / 1_000_000)
    }

    /// The counter value `us` microseconds after `start`.
    pub const fn deadline(&self, start: u64, us: u64) -> u64 {
        start + self.us_to_ticks(us)
    }

    /// Busy-wait `us` microseconds on `c`; returns the number of counter
    /// reads after the first (the polls).
    pub fn udelay<C: Counter + ?Sized>(&self, c: &mut C, us: u64) -> u64 {
        let t1 = self.deadline(c.ticks(), us);
        let mut polls = 0;
        while c.ticks() < t1 {
            polls += 1;
        }
        polls
    }
}
