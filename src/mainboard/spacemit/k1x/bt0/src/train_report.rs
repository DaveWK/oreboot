// SPDX-License-Identifier: GPL-2.0-only
//! Per-point DRAM training report (cargo feature `train-report`) and full
//! memory test (`train-memtest`), for comparing our own training with the
//! translated vendor training on the same board, side by side.
//!
//! After either training finishes a frequency point, the same lines are
//! printed: training kind, CPU cycles the training took, the RX Vref code and
//! registers, the MR14 value and what each rank/channel reads back, and the
//! per-DQ read and write windows the engines left (worst and mean). After
//! point 2 (2400 MT/s, the operating point) the total training time and
//! cycles since reset, then (train-memtest) a test of all DRAM of the board
//! profile. Lines start with `[k1r]`. Only reads registers; the memory test
//! writes DRAM before anything is loaded there.

use crate::train_engine::{Engine, HwPhy, Phy};
use crate::train_eye::{lp4x_mr14_permille, worst};
use core::sync::atomic::{AtomicU64, Ordering};
use log::{print, println};
use util::mmio::read32;

const TRAINING: &str = if cfg!(feature = "own-training") {
    "own"
} else {
    "translated"
};

static TOTAL: AtomicU64 = AtomicU64::new(0);

/// CPU cycle counter (mcycle); 0 off target (host tests).
pub use startup::riscv::cycles;

/// Report one trained frequency point; `start` is `cycles()` before training.
pub fn after_fp(
    ddrc: usize,
    ranks: u32,
    fp: u32,
    start: u64,
    mrr: fn(u32, u32, u32) -> Option<u32>,
) {
    let spent = cycles().wrapping_sub(start);
    TOTAL.fetch_add(spent, Ordering::Relaxed);
    let mut phy = HwPhy::new(ddrc, fp, ranks, mrr);
    let [a, b] = phy.rx_vref_regs();
    let mr14 = phy.tx_mr14();
    print!(
        "[k1r] fp{fp} training={TRAINING} cycles={spent} rx-code={} rxA=0x{:08x} rxB=0x{:08x} mr14=0x{mr14:02x} ({} permille) readback",
        phy.rx_vref(),
        read32(a),
        read32(b),
        lp4x_mr14_permille(mr14)
    );
    for rank in 0..ranks {
        for ch in 0..2 {
            match phy.mr_read(14, ch, rank) {
                Ok(v) => print!(" r{rank}c{ch}=0x{v:02x}"),
                Err(_) => print!(" r{rank}c{ch}=timeout"),
            }
        }
    }
    println!();
    for (name, engine) in [("read", Engine::Read), ("write", Engine::Write)] {
        let w = phy.window(engine);
        let sum: u32 = w.iter().map(|&v| v as u32).sum();
        print!("[k1r] fp{fp} {name}-win:");
        for v in w {
            print!(" {v}");
        }
        println!(
            " worst={} mean={}.{:02}",
            worst(&w),
            sum / 32,
            (sum % 32) * 100 / 32
        );
    }
    if fp == 2 {
        println!(
            "[k1r] DRAM trained ({TRAINING}): training cycles total={} mcycle since reset={}",
            TOTAL.load(Ordering::Relaxed),
            cycles()
        );
        #[cfg(feature = "train-memtest")]
        memtest();
    }
}

/// The DRAM windows of the board profile (K1_DRAM_MIB): 2 GiB at 0, and on
/// 4 GiB boards another 2 GiB at 4 GiB. The first page is reserved (address
/// 0 is a null pointer to Rust).
#[cfg(feature = "train-memtest")]
pub fn memtest_plan(dram_mib: u32) -> Result<memtest::Plan, &'static str> {
    use memtest::{Plan, Region};
    let low = Region::new(0, 0x8000_0000)?;
    let page = Region::new(0, 0x1000)?;
    match dram_mib {
        4096 => Plan::windows(&[low, Region::new(0x1_0000_0000, 0x8000_0000)?], &[page]),
        2048 => Plan::windows(&[low], &[page]),
        _ => Plan::windows(&[Region::new(0, 0x20_1000)?], &[page]),
    }
}

/// Test all DRAM of the board profile with oreboot's pattern test
/// (`util::mem::test`); a failure panics, so bt0 stops before anything is
/// loaded into bad memory.
#[cfg(feature = "train-memtest")]
fn memtest() {
    let plan = match memtest_plan(startup::env::u32(option_env!("K1_DRAM_MIB"))) {
        Ok(p) => p,
        Err(e) => {
            println!("[k1r] memtest plan: {e}");
            return;
        }
    };
    let start = cycles();
    for r in plan.ranges() {
        println!("[k1r] memtest 0x{:09x} + 0x{:08x}", r.start, r.bytes());
        util::mem::test(r.start as usize, r.bytes() as usize);
    }
    println!(
        "[k1r] memtest PASS ({TRAINING} training), cycles={}",
        cycles().wrapping_sub(start)
    );
}
