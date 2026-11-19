// SPDX-License-Identifier: GPL-2.0-only
//! Opt-in cached full-burst DRAM test in AArch64 EL3 (`memtest-cached`).
//!
//! Uses the same 39-bit EL3 flat map as `memtest.rs` (block 0 MMIO Device-nGnRnE,
//! DRAM blocks Normal) with MAIR attr1 = 0xff (inner/outer write-back
//! read/write-allocate) and `SCTLR_EL3.M | .C | .I` enabled during the test and
//! restored afterwards. Cache maintenance uses `DC CIVAC` after writes and
//! `DC IVAC` before reads (`dsb sy` after each) so every read is a line fill and
//! every write-back is a gapless burst across both ranks and above 4 GiB.

use ::memtest::arch::aarch64::{l1_table, TCR};
use ::memtest::{Memory, Runner, PROFILE_FAST};
use core::arch::asm;
use cubie_a7a_common::{mem, uart::Uart, Handoff};
use util::mmio::read32;

use crate::memtest_cached_core::{
    build_copy_plan, cached_plan_from_handoff, effective_rank_bit, rank_bit_from_addrmap1,
    rank_of_addr, run_bit37_pass, run_copy_leg, Bit37Shape, CopyLegStats, ErrorDecode, PassReport,
    ABOVE_4G, ADDRMAP1_CH0, BIT37_SEED_1000, BIT37_SEED_1010, COPY_BLOCK_BYTES, COPY_SEED_BASE,
    DUAL_CH_SHIFT, MAIR_WB, SCTLR_C, SCTLR_I, SCTLR_M, STRESS_BLOCK_BYTES,
};

/// Level-1 table of the cached EL3 flat map (4 KiB aligned).
#[repr(C, align(4096))]
struct Table([u64; 512]);
static mut L1_CACHED: Table = Table([0; 512]);

/// The 1 GiB block `main` executes from (`0x4000_0000`), kept executable.
const EXEC_BLOCK: usize = 1;

fn dcache_line_size() -> u64 {
    let ctr: u64;
    // SAFETY: CTR_EL0 is always readable at EL3.
    unsafe { asm!("mrs {}, ctr_el0", out(reg) ctr) };
    4u64 << ((ctr >> 16) & 0xf)
}

/// Active cached EL3 flat map; restores SCTLR/TTBR0/TCR/MAIR on drop after
/// cleaning and invalidating EL3's own memory range back to DRAM (PoC).
struct CachedEl3FlatMap {
    sctlr: u64,
    mair: u64,
    tcr: u64,
    ttbr: u64,
    line_size: u64,
}

impl CachedEl3FlatMap {
    /// Enter the cached EL3 flat map using `table` (filled by `l1_table`,
    /// 4 KiB aligned, kept alive while this guard lives).
    ///
    /// # Safety
    /// EL3 on the boot core with no secondary cores running; `table` must
    /// outlive the returned guard and map `EXEC_BLOCK` executable.
    unsafe fn enter(table: &[u64; 512], line_size: u64) -> Result<Self, &'static str> {
        let (sctlr, mair, tcr, ttbr): (u64, u64, u64, u64);
        asm!(
            "mrs {s}, sctlr_el3",
            "mrs {m}, mair_el3",
            "mrs {t}, tcr_el3",
            "mrs {b}, ttbr0_el3",
            s = out(reg) sctlr,
            m = out(reg) mair,
            t = out(reg) tcr,
            b = out(reg) ttbr,
        );
        if sctlr & (SCTLR_M | SCTLR_C) != 0 {
            return Err("MMU or D-cache already enabled");
        }
        if !(table.as_ptr() as usize).is_multiple_of(4096) {
            return Err("table not 4 KiB aligned");
        }
        let next_sctlr = sctlr | SCTLR_M | SCTLR_C | SCTLR_I;
        asm!(
            "dsb sy",
            "msr mair_el3, {m}",
            "msr tcr_el3, {t}",
            "msr ttbr0_el3, {b}",
            "isb",
            "tlbi alle3",
            "ic iallu",
            "dsb sy",
            "isb",
            "msr sctlr_el3, {s}",
            "isb",
            m = in(reg) MAIR_WB,
            t = in(reg) TCR,
            b = in(reg) table.as_ptr() as u64,
            s = in(reg) next_sctlr,
        );
        Ok(Self {
            sctlr,
            mair,
            tcr,
            ttbr,
            line_size,
        })
    }
}

impl Drop for CachedEl3FlatMap {
    fn drop(&mut self) {
        let el3_start = mem::EL3_BASE as u64;
        let el3_end = (mem::EL3_BASE + mem::EL3_SIZE) as u64;
        // Clear C while keeping M set so no new D-cache allocations happen
        // while DC CIVAC flushes EL3's stack, .data, .bss and L1 table to PoC;
        // all values stay in registers inside one asm! block (no stack writes).
        let sctlr_no_c = (self.sctlr | SCTLR_M | SCTLR_I) & !SCTLR_C;
        // SAFETY: restores the EL3 translation/control registers saved in enter
        // after draining all dirty EL3 lines to DRAM.
        unsafe {
            asm!(
                "dsb sy",
                "msr sctlr_el3, {no_c}",
                "isb",
                "1:",
                "dc civac, {p}",
                "add {p}, {p}, {step}",
                "cmp {p}, {end}",
                "b.lo 1b",
                "dsb sy",
                "msr sctlr_el3, {s}",
                "isb",
                "tlbi alle3",
                "ic iallu",
                "dsb sy",
                "isb",
                "msr ttbr0_el3, {b}",
                "msr tcr_el3, {t}",
                "msr mair_el3, {m}",
                "isb",
                no_c = in(reg) sctlr_no_c,
                p = inout(reg) el3_start => _,
                end = in(reg) el3_end,
                step = in(reg) self.line_size,
                s = in(reg) self.sctlr,
                b = in(reg) self.ttbr,
                t = in(reg) self.tcr,
                m = in(reg) self.mair,
            );
        }
    }
}

struct CachedDram {
    line_size: u64,
}

impl Memory for CachedDram {
    fn write(&mut self, a: u64, v: u64) {
        // SAFETY: callers only pass validated addresses inside the test plan.
        unsafe { (a as *mut u64).write_volatile(v) }
    }

    fn read(&mut self, a: u64) -> u64 {
        // SAFETY: callers only pass validated addresses inside the test plan.
        unsafe { (a as *const u64).read_volatile() }
    }

    fn complete(&mut self) {
        // SAFETY: barrier instruction.
        unsafe { asm!("dsb sy", options(nostack, preserves_flags)) }
    }

    fn ticks(&mut self) -> u64 {
        crate::ticks()
    }

    fn kick(&mut self) {
        crate::soc::watchdog_kick()
    }

    fn clean_invalidate(&mut self, addr: u64, len: u64) {
        if len == 0 {
            return;
        }
        let mask = self.line_size - 1;
        let mut p = addr & !mask;
        let end = addr.saturating_add(len);
        while p < end {
            // SAFETY: DC CIVAC cleans and invalidates to PoC at EL3.
            unsafe { asm!("dc civac, {}", in(reg) p, options(nostack, preserves_flags)) };
            p = p.saturating_add(self.line_size);
        }
        // SAFETY: barrier waits for all DC CIVAC operations to complete.
        unsafe { asm!("dsb sy", options(nostack, preserves_flags)) };
    }

    fn invalidate(&mut self, addr: u64, len: u64) {
        if len == 0 {
            return;
        }
        let mask = self.line_size - 1;
        let mut p = addr & !mask;
        let end = addr.saturating_add(len);
        // Clean+invalidate any partial edge line so neighbouring bytes are
        // never discarded; use DC IVAC for every fully covered cache line.
        if addr & mask != 0 {
            // SAFETY: DC CIVAC on a partial leading line preserves other bytes.
            unsafe { asm!("dc civac, {}", in(reg) p, options(nostack, preserves_flags)) };
            p = p.saturating_add(self.line_size);
        }
        let aligned_end = end & !mask;
        while p < aligned_end {
            // SAFETY: DC IVAC on a fully covered test line discards cached copy.
            unsafe { asm!("dc ivac, {}", in(reg) p, options(nostack, preserves_flags)) };
            p = p.saturating_add(self.line_size);
        }
        if end & mask != 0 && aligned_end >= p {
            // SAFETY: DC CIVAC on a partial trailing line preserves other bytes.
            unsafe {
                asm!("dc civac, {}", in(reg) aligned_end, options(nostack, preserves_flags));
            }
        }
        // SAFETY: barrier waits for all DC IVAC/CIVAC operations to complete.
        unsafe { asm!("dsb sy", options(nostack, preserves_flags)) };
    }
}

fn elapsed_ms(start_ticks: u64) -> u64 {
    crate::ticks().wrapping_sub(start_ticks) / (crate::soc::TIMER_HZ / 1000)
}

fn print_gib_counts(report: &PassReport) {
    print!("gib=[");
    for (i, &c) in report.gib_counts().iter().enumerate() {
        if i > 0 {
            print!(" ");
        }
        print!("{i}:{c}");
    }
    if report.out_of_range > 0 {
        print!(" oor:{}", report.out_of_range);
    }
    print!("]");
}

fn print_first_error(name: &str, err: &ErrorDecode) {
    print!(
        "[main] memtest-cached: pass {name} first @ {:#x} (addr%8={}",
        err.addr, err.addr_mod_8
    );
    if let Some(b) = err.first_bit {
        print!(
            " byte_addr%8={} w32%8={}",
            b.byte_addr_mod_8, b.word32_addr_mod_8
        );
    }
    print!(
        "): wanted {:#018x} got {:#018x} xor {:#018x} bits=[",
        err.wanted, err.got, err.xor
    );
    for (i, &bit) in err.failing_bits().iter().enumerate() {
        if i > 0 {
            print!(",");
        }
        print!("{bit}");
    }
    print!("]");
    if let Some(b) = err.first_bit {
        let ch = if b.channel == 0 { 'A' } else { 'B' };
        println!(
            " beat={}/4 burst_beat={}/16 ch={ch}",
            b.beat_in_word, b.burst_beat
        );
    } else {
        println!();
    }
}

fn print_pass_summary(name: &str, ms: u64, report: &PassReport, copy: Option<CopyLegStats>) {
    print!("[main] memtest-cached: pass {name} ms={ms}");
    if let Some(st) = copy {
        print!(
            " blocks={} mib={} hash_mismatches={}",
            st.blocks,
            st.bytes >> 20,
            st.hash_mismatches
        );
    }
    print!(" errors={} ", report.errors);
    print_gib_counts(report);
    println!();
    if let Some(err) = report.first.as_ref() {
        print_first_error(name, err);
    }
}

pub fn run(h: &Handoff) -> Result<(), &'static str> {
    let plan = cached_plan_from_handoff(h)?;
    let am1 = read32(ADDRMAP1_CH0);
    let rank_bit = effective_rank_bit(rank_bit_from_addrmap1(am1, DUAL_CH_SHIFT), h.dram_size);
    let line_size = dcache_line_size();

    println!(
        "[main] memtest-cached: DRAM {:#x}..{:#x} ({} MiB), ADDRMAP1 {:#x}, rank_bit {:?}, dcache {} B, MAIR {:#x}",
        h.dram_base,
        h.dram_base + h.dram_size,
        h.dram_size >> 20,
        am1,
        rank_bit,
        line_size,
        MAIR_WB
    );
    let gib_count = h.dram_size.div_ceil(1u64 << 30);
    for g in 0..gib_count {
        let start = h.dram_base + (g << 30);
        let end = (start + (1u64 << 30)).min(h.dram_base + h.dram_size);
        let rk = rank_of_addr(start, h.dram_base, rank_bit);
        let hi = if start >= ABOVE_4G { " (>4GiB)" } else { "" };
        println!("[main] memtest-cached: gib {g}: {start:#x}..{end:#x} r{rk}{hi}");
    }
    for r in plan.ranges() {
        println!(
            "[main] memtest-cached: test {:#x}..{:#x} ({} MiB)",
            r.start,
            r.end,
            r.bytes() >> 20
        );
    }

    // SAFETY: boot core only, single-threaded before payload entry; L1_CACHED
    // outlives the flat map guard dropped before returning.
    let l1 = unsafe { &mut *core::ptr::addr_of_mut!(L1_CACHED) };
    l1_table(&mut l1.0, h.dram_base + h.dram_size, Some(EXEC_BLOCK))?;
    let map = unsafe { CachedEl3FlatMap::enter(&l1.0, line_size)? };

    let t_total = crate::ticks();
    let mut total = PassReport::new(h.dram_base, h.dram_size);
    let mut dram = CachedDram { line_size };

    // Phase 1: stress::Runner with PROFILE_FAST, 64 KiB blocks.
    {
        let mut runner = Runner::new(&mut dram, STRESS_BLOCK_BYTES, crate::soc::TIMER_HZ)?;
        for step in PROFILE_FAST {
            let t0 = crate::ticks();
            let mut rep = PassReport::new(h.dram_base, h.dram_size);
            runner.step(&plan, step, &mut rep);
            print_pass_summary(step.name(), elapsed_ms(t0), &rep, None);
            total.merge(&rep);
        }
    }

    // Phase 2: bit-37 1-0-x-0 beat-shape pattern passes (1010 then 1000).
    for (shape, seed) in [
        (Bit37Shape::OneZeroOneZero, BIT37_SEED_1010),
        (Bit37Shape::OneZeroZeroZero, BIT37_SEED_1000),
    ] {
        let t0 = crate::ticks();
        let mut rep = PassReport::new(h.dram_base, h.dram_size);
        run_bit37_pass(
            &mut dram,
            &plan,
            STRESS_BLOCK_BYTES,
            crate::soc::TIMER_HZ,
            seed,
            shape,
            &mut rep,
        )?;
        print_pass_summary(shape.name(), elapsed_ms(t0), &rep, None);
        total.merge(&rep);
    }

    // Phase 3: 16 MiB cross-rank and above-4-GiB copy passes with SHA-256 compare.
    let copy_plan = build_copy_plan(&plan, h.dram_base, rank_bit, COPY_BLOCK_BYTES);
    for (leg_idx, leg) in [
        &copy_plan.r0_to_r1,
        &copy_plan.r1_to_r0,
        &copy_plan.r1_above4g,
    ]
    .iter()
    .enumerate()
    {
        if leg.count == 0 {
            println!(
                "[main] memtest-cached: pass {} skipped (no eligible 16 MiB pairs for rank_bit {:?})",
                leg.name, rank_bit
            );
            continue;
        }
        let t0 = crate::ticks();
        let mut rep = PassReport::new(h.dram_base, h.dram_size);
        let seed_base = COPY_SEED_BASE.wrapping_add((leg_idx as u64) << 16);
        let stats = run_copy_leg(
            &mut dram,
            leg,
            STRESS_BLOCK_BYTES,
            crate::soc::TIMER_HZ,
            seed_base,
            &mut rep,
        )?;
        print_pass_summary(leg.name, elapsed_ms(t0), &rep, Some(stats));
        total.merge(&rep);
    }

    drop(map);

    let total_ms = elapsed_ms(t_total);
    let verdict = if total.errors == 0 { "PASS" } else { "FAIL" };
    print!(
        "[main] memtest-cached: {verdict} total_errors={} total_ms={total_ms} ",
        total.errors
    );
    print_gib_counts(&total);
    println!();
    if let Some(err) = total.first.as_ref() {
        print_first_error("total", err);
    }
    Uart::attach(crate::soc::UART0_BASE).drain();

    if total.errors > 0 {
        Err("DRAM integrity failure; payload withheld")
    } else {
        Ok(())
    }
}
