//! Opt-in destructive DRAM test before any DMA peripheral starts.
//! EL3, page tables/stacks/log/handoff and both loaded images are excluded.
//! MMU maps DRAM explicitly Normal Non-cacheable (MAIR=0x44), MMIO Device.
//! Entry requires M=C=0; all translation/control registers are restored.

use core::arch::asm;
use cubie_a7a_common::{
    memtest::{self, Memory, Pattern, Report},
    Handoff,
};

#[repr(C, align(4096))]
struct Table([u64; 512]);
static mut L1: Table = Table([0; 512]);
struct Map {
    sctlr: u64,
    mair: u64,
    tcr: u64,
    ttbr: u64,
}
impl Map {
    fn enter(end: u64) -> Result<Self, &'static str> {
        let (sctlr, mair, tcr, ttbr): (u64, u64, u64, u64);
        unsafe {
            asm!("mrs {s}, sctlr_el3", "mrs {m}, mair_el3", "mrs {t}, tcr_el3", "mrs {b}, ttbr0_el3", s=out(reg)sctlr,m=out(reg)mair,t=out(reg)tcr,b=out(reg)ttbr);
        }
        if sctlr & 5 != 0 {
            return Err("MMU or D-cache already enabled");
        }
        let l1 = unsafe { &mut *core::ptr::addr_of_mut!(L1) };
        l1.0.fill(0);
        l1.0[0] = 1 | (1 << 10) | (1 << 54); // Device nGnRnE, XN
        for gib in 1..end.div_ceil(1 << 30) as usize {
            l1.0[gib] = (gib as u64) << 30 | 1 | (1 << 10) | (3 << 8) | (1 << 2);
            if gib != 1 {
                l1.0[gib] |= 1 << 54;
            }
        }
        // 39-bit VA / L1, 4 KiB granule, uncached walks, 40-bit PA, RES1.
        let test_tcr = 25u64 | (2 << 16) | (1 << 23) | (1 << 31);
        unsafe {
            asm!("dsb sy", "msr mair_el3, {m}", "msr tcr_el3, {t}", "msr ttbr0_el3, {b}", "isb", "tlbi alle3", "dsb sy", "isb", "msr sctlr_el3, {s}", "isb", m=in(reg)0x4400u64,t=in(reg)test_tcr,b=in(reg)l1.0.as_ptr() as u64,s=in(reg)(sctlr|1));
        }
        Ok(Self {
            sctlr,
            mair,
            tcr,
            ttbr,
        })
    }
}
impl Drop for Map {
    fn drop(&mut self) {
        unsafe {
            asm!("dsb sy", "msr sctlr_el3, {s}", "isb", "tlbi alle3", "dsb sy", "isb", "msr ttbr0_el3, {b}", "msr tcr_el3, {t}", "msr mair_el3, {m}", "isb", s=in(reg)self.sctlr,b=in(reg)self.ttbr,t=in(reg)self.tcr,m=in(reg)self.mair);
        }
    }
}
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
    let map = Map::enter(h.dram_base + h.dram_size)?;
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
