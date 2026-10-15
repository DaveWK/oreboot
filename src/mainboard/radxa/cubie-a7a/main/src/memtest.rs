//! Opt-in destructive DRAM test before any DMA peripheral starts.
//! EL3, page tables/stacks/log/handoff and both loaded images are excluded.
//! MMU maps DRAM explicitly Normal Non-cacheable (MAIR=0x44), MMIO Device.
//! Entry requires M=C=0; all translation/control registers are restored.

use ::memtest::arch::aarch64::{l1_table, El3FlatMap};
use core::arch::asm;
use cubie_a7a_common::{
    memtest::{self, Memory, Pattern, Report},
    Handoff,
};

/// The level-1 table of the flat map (shared `memtest::arch::aarch64`).
#[repr(C, align(4096))]
struct Table([u64; 512]);
static mut L1: Table = Table([0; 512]);

/// The 1 GiB block main runs from (0x4000_0000), kept executable.
const EXEC_BLOCK: usize = 1;

struct Dram;
impl Memory for Dram {
    fn write(&mut self, a: u64, v: u64) {
        unsafe { (a as *mut u64).write_volatile(v) }
    }
    fn read(&mut self, a: u64) -> u64 {
        unsafe { (a as *const u64).read_volatile() }
    }
    fn complete(&mut self) {
        unsafe { asm!("dsb sy") }
    }
    fn ticks(&mut self) -> u64 {
        crate::ticks()
    }
    fn kick(&mut self) {
        crate::soc::watchdog_kick()
    }
}
pub fn run(h: &Handoff) -> Result<(), &'static str> {
    let plan = memtest::plan_from_handoff(h)?;
    for r in plan.ranges() {
        println!("[main] memtest: test {:#x}..{:#x}", r.start, r.end);
    }
    // SAFETY: only this function touches L1, on the boot core before
    // anything else runs; it outlives the map, which is dropped below.
    let l1 = unsafe { &mut *core::ptr::addr_of_mut!(L1) };
    l1_table(&mut l1.0, h.dram_base + h.dram_size, Some(EXEC_BLOCK))?;
    let map = unsafe { El3FlatMap::enter(&l1.0)? };
    let mut report = Report::default();
    for pat in [Pattern::Address, Pattern::Inverted, Pattern::Xorshift] {
        let t = crate::ticks();
        let before = report.errors;
        memtest::pass(&mut Dram, &plan, pat, crate::soc::TIMER_HZ, &mut report);
        println!(
            "[main] memtest: {} {} bytes {} ms {} errors",
            pat.name(),
            plan.bytes,
            (crate::ticks() - t) / (crate::soc::TIMER_HZ / 1000),
            report.errors - before
        );
    }
    drop(map);
    if let Some((a, w, g)) = report.first {
        println!(
            "[main] memtest: first error @ {:#x}: wanted {:#x}, got {:#x}; total {}",
            a, w, g, report.errors
        );
        Err("DRAM integrity failure; payload withheld")
    } else {
        Ok(())
    }
}
