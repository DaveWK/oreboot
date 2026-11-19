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
use ::memtest::{Memory, Runner, Step, PROFILE_FAST};
use core::arch::asm;
use cubie_a7a_common::{uart::Uart, Handoff};
use util::mmio::read32;

use crate::memtest_cached_core::{
    build_copy_plan, cached_plan_from_handoff, evaluate_verdict,
    plan_covers_both_ranks_and_above_4g, rank_of_addr, run_bit37_pass, run_copy_leg,
    validate_live_rank_map, wb_mapped_range, Bit37Shape, CopyLegStats, ErrorDecode, LiveRankMap,
    PassReport, RunCoverage, UnlocalizedMismatch, Verdict, ABOVE_4G, ADDRMAP1_CH0, BIT37_SEED_1000,
    BIT37_SEED_1010, COPY_BLOCK_BYTES, COPY_SEED_BASE, MAIR_WB, SCTLR_C, SCTLR_I, SCTLR_M,
    STRESS_BLOCK_BYTES,
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
/// cleaning and invalidating every Normal WB cacheable-mapped block to PoC.
struct CachedEl3FlatMap {
    sctlr: u64,
    mair: u64,
    tcr: u64,
    ttbr: u64,
    line_size: u64,
    wb_start: u64,
    wb_end: u64,
}

impl CachedEl3FlatMap {
    /// Enter the cached EL3 flat map using `table` (filled by `l1_table`,
    /// 4 KiB aligned, kept alive while this guard lives).
    ///
    /// # Safety
    /// EL3 on the boot core with no secondary cores running; `table` must
    /// outlive the returned guard and map `EXEC_BLOCK` executable.
    unsafe fn enter(table: &[u64; 512], line_size: u64) -> Result<Self, &'static str> {
        let (wb_start, wb_end) = wb_mapped_range(table)?;
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
            wb_start,
            wb_end,
        })
    }

    /// Tear down the cached EL3 flat map in one register-only `asm!` block:
    /// 1. Clear both `SCTLR_EL3.C` and `SCTLR_EL3.I` (keeping `M=1` so the
    ///    flat VA=PA map stays active). Per Arm ARM `SCTLR_EL3.I` ("Stage 1
    ///    instruction access cacheability"), EL3 instruction fetches to Normal
    ///    memory are treated as Non-cacheable when `I=0`, stopping both data-
    ///    and instruction-side refills into L1/L2/L3.
    /// 2. `DC CIVAC` every cache line across `[wb_start, wb_end)` — all Normal
    ///    WB DRAM blocks mapped by `L1_CACHED` — reaching PoC across L1/L2/L3.
    /// 3. Hold `C=0, I=0` through the final maintenance barrier (`dsb sy; isb`),
    ///    then `ic iallu; dsb sy; isb` to invalidate all instruction caches,
    ///    then restore original `SCTLR_EL3` (`M=0, C=0`, saved `I`), `isb`,
    ///    `tlbi alle3; dsb sy; isb`, and restore `TTBR0/TCR/MAIR; isb`.
    #[inline(never)]
    fn teardown(&self) {
        let sctlr_no_ci = (self.sctlr | SCTLR_M) & !(SCTLR_C | SCTLR_I);
        let wb_start = self.wb_start;
        let wb_end = self.wb_end;
        let step = self.line_size;
        let s = self.sctlr;
        let b = self.ttbr;
        let t = self.tcr;
        let m = self.mair;
        // SAFETY: all values are loaded into registers before the first
        // `msr sctlr_el3`; `options(nostack)` and no memory operands ensure
        // zero stack or other loads/stores occur across the transition. Only
        // the boot core is running (`main.rs` prepared all other cores off).
        unsafe {
            asm!(
                "dsb sy",
                "msr sctlr_el3, {no_ci}",
                "isb",
                "1:",
                "dc civac, {p}",
                "add {p}, {p}, {step}",
                "cmp {p}, {end}",
                "b.lo 1b",
                "dsb sy",
                "isb",
                "ic iallu",
                "dsb sy",
                "isb",
                "msr sctlr_el3, {s}",
                "isb",
                "tlbi alle3",
                "dsb sy",
                "isb",
                "msr ttbr0_el3, {b}",
                "msr tcr_el3, {t}",
                "msr mair_el3, {m}",
                "isb",
                no_ci = in(reg) sctlr_no_ci,
                p = inout(reg) wb_start => _,
                end = in(reg) wb_end,
                step = in(reg) step,
                s = in(reg) s,
                b = in(reg) b,
                t = in(reg) t,
                m = in(reg) m,
                options(nostack),
            );
        }
    }
}

impl Drop for CachedEl3FlatMap {
    fn drop(&mut self) {
        // Reload WDT0 (Device-nGnRnE block 0) before the full-DRAM DC CIVAC
        // sweep so the entire teardown has a fresh 16 s watchdog window.
        crate::soc::watchdog_kick();
        self.teardown();
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

fn print_hex32(bytes: &[u8; 32]) {
    for &b in bytes {
        print!("{b:02x}");
    }
}

fn print_unlocalized_mismatch(name: &str, count: u64, u: &UnlocalizedMismatch) {
    print!(
        "[main] memtest-cached: pass {name} unlocalized block-hash mismatch (count={count}): leg={} block={:#x} len={:#x} src_sha256=",
        u.leg, u.block_base, u.block_len
    );
    print_hex32(&u.src_digest);
    print!(" dst_sha256=");
    print_hex32(&u.dst_digest);
    println!();
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
    if let Some(u) = report.first_unlocalized.as_ref() {
        print_unlocalized_mismatch(name, report.unlocalized_count, u);
    }
}

pub fn run(h: &Handoff) -> Result<(), &'static str> {
    let plan = cached_plan_from_handoff(h)?;
    let am1 = read32(ADDRMAP1_CH0);
    let rank_map = validate_live_rank_map(am1, h.dram_size);
    let line_size = dcache_line_size();

    let live_map = match rank_map {
        Ok(m) => m,
        Err(reason) => {
            println!(
                "[main] memtest-cached: FAIL reason=\"{reason}\" ADDRMAP1={am1:#x} dram_size={:#x} copy_bytes=[r0->r1:0 r1->r0:0 r1>4g:0]",
                h.dram_size
            );
            Uart::attach(crate::soc::UART0_BASE).drain();
            return Err(reason);
        }
    };
    let rank_bit = live_map.rank_bit();
    let plan_dual_full = match live_map {
        LiveRankMap::DualRank { rank_bit: b } => {
            plan_covers_both_ranks_and_above_4g(&plan, h.dram_base, b)
        }
        LiveRankMap::SingleRank => false,
    };

    println!(
        "[main] memtest-cached: DRAM {:#x}..{:#x} ({} MiB), ADDRMAP1 {:#x}, rank_map {:?}, dcache {} B, MAIR {:#x}",
        h.dram_base,
        h.dram_base + h.dram_size,
        h.dram_size >> 20,
        am1,
        live_map,
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
    let mut cov = RunCoverage::default();
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
            match step {
                Step::March { .. } => cov.march_ran_both_ranks = plan_dual_full,
                Step::Check(_) | Step::Lines { .. } => cov.stress_ran_both_ranks = plan_dual_full,
            }
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
    cov.bit37_ran_both_ranks = plan_dual_full;

    // Phase 3: 16 MiB cross-rank and above-4-GiB copy passes with SHA-256 compare.
    let copy_plan = build_copy_plan(&plan, h.dram_base, rank_bit, COPY_BLOCK_BYTES);
    let mut leg_bytes = [0u64; 3];
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
                "[main] memtest-cached: pass {} skipped (no eligible 16 MiB pairs for rank_map {:?})",
                leg.name, live_map
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
        leg_bytes[leg_idx] = stats.bytes;
        print_pass_summary(leg.name, elapsed_ms(t0), &rep, Some(stats));
        total.merge(&rep);
    }
    cov.copy_r0_to_r1_bytes = leg_bytes[0];
    cov.copy_r1_to_r0_bytes = leg_bytes[1];
    cov.copy_r1_above4g_bytes = leg_bytes[2];

    drop(map);

    let total_ms = elapsed_ms(t_total);
    let (verdict, reason) = evaluate_verdict(Ok(live_map), total.errors, &cov);
    print!(
        "[main] memtest-cached: {} total_errors={} total_ms={total_ms} copy_bytes=[r0->r1:{} r1->r0:{} r1>4g:{}] ",
        verdict.as_str(),
        total.errors,
        cov.copy_r0_to_r1_bytes,
        cov.copy_r1_to_r0_bytes,
        cov.copy_r1_above4g_bytes,
    );
    print_gib_counts(&total);
    println!(" ({reason})");
    if let Some(err) = total.first.as_ref() {
        print_first_error("total", err);
    }
    if let Some(u) = total.first_unlocalized.as_ref() {
        print_unlocalized_mismatch("total", total.unlocalized_count, u);
    }
    Uart::attach(crate::soc::UART0_BASE).drain();

    match verdict {
        Verdict::Pass | Verdict::Limited => Ok(()),
        Verdict::Fail => Err(reason),
    }
}
