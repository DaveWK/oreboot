//! Arch counter delays for both stages (AArch32 bt0, AArch64 main).

/// The arch timer runs from the 24 MHz SYS_CLK24M on every A733 board.
pub const TIMER_HZ: u64 = 24_000_000;

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

/// Host builds (`make test`) have no arch counter.
#[cfg(not(any(target_arch = "arm", target_arch = "aarch64")))]
pub fn ticks() -> u64 {
    0
}

pub fn udelay(us: u64) {
    let t1 = ticks() + us * (TIMER_HZ / 1_000_000);
    while ticks() < t1 {}
}
