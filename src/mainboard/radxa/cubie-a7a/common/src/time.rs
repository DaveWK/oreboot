//! Arch counter delays for both stages (AArch32 bt0, AArch64 main), on the
//! shared `arch-timer` crate.

/// The arch timer runs from the 24 MHz SYS_CLK24M on every A733 board.
pub const TIMER_HZ: u64 = 24_000_000;

pub use arch_timer::ticks;

const TIMER: arch_timer::Timer = arch_timer::Timer::new(TIMER_HZ);

pub fn udelay(us: u64) {
    TIMER.udelay(&mut arch_timer::ArchCounter, us);
}
