//! EL3-only, MMU/data-cache-off bakery backend. No exclusive instructions.
//!
//! All entrants are one execution stream per physical CPU: SMC entry masks
//! DAIF, EL3 does not enable asynchronous handlers, and cold/warm startup uses
//! that core's own stack. OS memory excludes the EL3 region containing these
//! slots/CPU records. If EL3 ever enables translation or D-cache this backend
//! must be replaced with a coherent/cache-maintained implementation first.
use core::{arch::asm, cell::UnsafeCell};
use cubie_a7a_common::{
    bakery::{self, Access},
    mem, Handoff,
};

#[repr(C, align(64))]
struct Slot(UnsafeCell<u32>);
// Only CPU i writes slot i. Aligned scalar assembly accesses and SY barriers
// implement the bakery Access contract without Rust shared references to data.
unsafe impl Sync for Slot {}
static SLOTS: [Slot; 8] = [const { Slot(UnsafeCell::new(0)) }; 8];
pub struct Uncached;
static ACCESS: Uncached = Uncached;
impl Access for Uncached {
    fn participants(&self) -> usize {
        SLOTS.len()
    }
    fn load(&self, i: usize) -> u32 {
        unsafe { load32(SLOTS[i].0.get()) }
    }
    fn store(&self, i: usize, v: u32) {
        unsafe { store32(SLOTS[i].0.get(), v) }
    }
    fn barrier(&self) {
        barrier();
    }
    fn ticks(&self) -> u64 {
        cubie_a7a_common::time::ticks()
    }
    fn relax(&self) {
        unsafe {
            asm!("yield", options(nomem, nostack));
        }
    }
}

pub fn barrier() {
    unsafe {
        asm!("dsb sy", options(nostack));
    }
}
/// Caller guarantees aligned, EL3-owned memory for the entire access.
pub unsafe fn load32(p: *const u32) -> u32 {
    let v: u32;
    asm!("dmb sy", "ldr {v:w}, [{p}]", "dmb sy", p=in(reg) p, v=out(reg) v,
         options(nostack));
    v
}
/// Caller guarantees aligned, EL3-owned memory and serialized writers.
pub unsafe fn store32(p: *mut u32, v: u32) {
    asm!("dmb sy", "str {v:w}, [{p}]", "dsb sy", p=in(reg) p, v=in(reg) v,
         options(nostack));
}

pub fn acquire() -> Result<bakery::Guard<'static, Uncached>, ()> {
    let sctlr: u64;
    unsafe {
        asm!("mrs {}, sctlr_el3", out(reg) sctlr, options(nomem, nostack));
    }
    if sctlr & ((1 << 0) | (1 << 2)) != 0 {
        return Err(());
    }
    // Eight contending CPU_ON calls can each spend at most 2*100ms polling
    // the PPU. Two seconds bounds acquisition while allowing that queue.
    bakery::acquire(&ACCESS, crate::current_core(), crate::soc::TIMER_HZ * 2).map_err(|_| ())
}

pub fn handoff() -> Handoff {
    unsafe { core::ptr::read_volatile(mem::HANDOFF_ADDR as *const Handoff) }
}
pub fn standby() {
    unsafe {
        asm!("dsb sy", "wfi", options(nostack));
    }
}
