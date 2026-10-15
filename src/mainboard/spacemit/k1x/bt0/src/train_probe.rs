// SPDX-License-Identifier: GPL-2.0-only
//! Hardware probe for our own K1 DRAM training (cargo feature `train-probe`).
//!
//! Step 1 of doc/K1-OWN-TRAINING-PLAN.md needs measurements that only the
//! board can give: what the engines do with their parameter registers at
//! hardware values, which RX Vref field drives which DQ, whether the MR14
//! field we program reaches the DRAM, and whether the engines need any of the
//! set-up steps the translated training performs (lookup T7). This build
//! prints them on the UART before our training runs at each frequency point,
//! then runs our training.
//!
//! Every experiment restores the registers it changed. Engine timeouts are
//! reported, not fatal, and use a short poll budget. Lines start with
//! `[k1p]`; doc/K1-OWN-TRAINING-PLAN.md "Probe log" explains each one. The
//! per-point report and the full memory test come from `train_report.rs`
//! (features `train-report`, `train-memtest`, both enabled by `train-probe`).

use crate::train_engine::{
    Engine, Error, HwPhy, Phy, DPHY_CS_STRIDE, DPHY_DATAPHY, DPHY_OTHER, DPHY_SUBPHY,
    MC_FP_MR_VALUES, MC_FP_SEL, PHY_PARAM, PHY_STATUS, POLL_LIMIT, TRAINING_SUMMARY,
};
use crate::train_eye::{tx_axis_index, tx_axis_mr14};
use crate::train_own::Report;
use log::{print, println};
use util::mmio::{read32, write32};

/// Poll budget during experiments (a tenth of the normal one).
const PROBE_POLL: u32 = 1_000_000;
/// Values tried in each byte of each engine parameter register.
const GRID: [u8; 7] = [0x00, 0x01, 0x08, 0x20, 0x40, 0x80, 0xff];
/// Set-up register the translation writes before training (lookup T7).
const OTHER_28: usize = DPHY_OTHER + 0x28;
/// RX Vref bit the translation clears around the read engine (lookup T7).
const RX_VREF_BIT4: u32 = 1 << 4;
/// Per-DQ registers, read back raw for the log (lookup T8): 8 words per byte
/// lane from these offsets in each lane's rank block.
const LANE_READBACK: [usize; 2] = [0x00, 0x30];

fn dump(phy: &HwPhy, when: &str) {
    let fp = phy.fp;
    print!("[k1p] fp{fp} {when}:");
    for (name, off) in [
        ("mc8", 0x0008),
        ("fpsel", MC_FP_SEL),
        ("mrval", MC_FP_MR_VALUES),
        ("p13e8", PHY_PARAM[0]),
        ("p13ec", PHY_PARAM[1]),
        ("p13f8", PHY_PARAM[2]),
        ("phy13fc", PHY_STATUS),
        ("sum58000", TRAINING_SUMMARY),
        ("oth50028", OTHER_28),
    ] {
        print!(" {name}=0x{:08x}", read32(phy.ddrc + off));
    }
    let [a, b] = phy.rx_vref_regs();
    println!(" rxA=0x{:08x} rxB=0x{:08x}", read32(a), read32(b));
}

/// One engine run on one rank; prints status and (read/write) windows.
/// Returns (done, worst window).
fn trial(phy: &HwPhy, e: Engine, rank: u32, label: &str, verbose: bool) -> (bool, Option<u8>) {
    let before = read32(phy.ddrc + PHY_STATUS);
    let r = phy.trigger(e, rank);
    let summary = read32(phy.ddrc + TRAINING_SUMMARY);
    let w = match e {
        Engine::Read | Engine::Write => Some(phy.window(e)),
        _ => None,
    };
    let worst = w.map(|w| crate::train_eye::worst(&w));
    if verbose {
        match r {
            Ok(s) => print!(
                "[k1p] {label} {} r{rank}: 0x{before:08x} -> done 0x{s:08x}",
                e.name()
            ),
            Err(s) => print!(
                "[k1p] {label} {} r{rank}: 0x{before:08x} -> TIMEOUT 0x{s:08x}",
                e.name()
            ),
        }
        print!(" sum 0x{summary:08x}");
        if let Some(w) = w {
            print!(" win");
            for v in w {
                print!(" {v}");
            }
        }
        println!();
    }
    (r.is_ok(), worst)
}

fn lane_readback(phy: &HwPhy, rank: u32) {
    let pp = phy.pp();
    for off in LANE_READBACK {
        print!("[k1p] fp{} r{rank} lane+0x{off:02x}:", phy.fp);
        for sub in DPHY_SUBPHY {
            for dp in DPHY_DATAPHY {
                let base = pp + sub + dp + rank as usize * DPHY_CS_STRIDE + off;
                for k in 0..8 {
                    print!(" {:08x}", read32(base + k * 4));
                }
                print!(" |");
            }
        }
        println!();
    }
}

/// Byte lanes' DQ nibbles in the two RX Vref registers (register index,
/// shift); GPL: [23:16] rx_ref_d1, [31:24] rx_ref_d2, DQ in the low nibble.
const RX_LANES: [(usize, u32); 4] = [(0, 16), (0, 24), (1, 16), (1, 24)];

/// Characterisation at frequency point 0, before our training.
fn characterise(phy: &mut HwPhy) {
    let ddrc = phy.ddrc;
    let ranks = phy.ranks;
    let rx_regs = phy.rx_vref_regs();
    let rx_orig = [read32(rx_regs[0]), read32(rx_regs[1])];
    let mr14_orig = phy.tx_mr14();
    let params_orig = PHY_PARAM.map(|r| read32(ddrc + r));
    let other_orig = read32(ddrc + OTHER_28);
    phy.poll_limit = PROBE_POLL;

    // 1. Every engine on every rank with hardware parameter values.
    for rank in 0..ranks {
        for e in Engine::ALL {
            trial(phy, e, rank, "plain", true);
        }
        lane_readback(phy, rank);
    }

    // 2. Which DQ each RX Vref lane field drives: one lane at code 0.
    let (_, base) = trial(phy, Engine::Read, 0, "lane-ref", true);
    for (lane, (reg, shift)) in RX_LANES.iter().enumerate() {
        write32(rx_regs[*reg], rx_orig[*reg] & !(0xf << shift));
        print!(
            "[k1p] lane {lane} (reg {reg} bits {}..{}) at 0:",
            shift + 3,
            shift
        );
        trial(phy, Engine::Read, 0, "lane", true);
        write32(rx_regs[*reg], rx_orig[*reg]);
    }
    let _ = base;

    // 3. Does the DQS nibble matter: all DQS fields at 0 and at 15.
    for dqs in [0u32, 15] {
        for (i, r) in rx_regs.iter().enumerate() {
            write32(*r, (rx_orig[i] & !0xf0f0_0000) | dqs << 20 | dqs << 28);
        }
        trial(
            phy,
            Engine::Read,
            0,
            if dqs == 0 { "dqs0" } else { "dqs15" },
            true,
        );
    }
    for (i, r) in rx_regs.iter().enumerate() {
        write32(*r, rx_orig[i]);
    }

    // 4. MR14 field mapping: program a different value, read MR14 back.
    let test = tx_axis_index(mr14_orig)
        .map(|i| tx_axis_mr14((i + 2).min(50)))
        .unwrap_or(0x1b);
    for op in [test, mr14_orig] {
        phy.set_tx_mr14(op);
        print!(
            "[k1p] mr14 sent 0x{op:02x} (mrval 0x{:08x}):",
            read32(ddrc + MC_FP_MR_VALUES)
        );
        for rank in 0..ranks {
            for ch in 0..2 {
                match phy.mr_read(14, ch, rank) {
                    Ok(v) => print!(" r{rank}c{ch}=0x{v:02x}"),
                    Err(_) => print!(" r{rank}c{ch}=MRR-timeout"),
                }
            }
        }
        match phy.mr_read(12, 0, 0) {
            Ok(v) => {
                println!(" mr12=0x{v:02x}");
            }
            Err(_) => {
                println!(" mr12=MRR-timeout");
            }
        }
    }

    // 5. Set-up steps the translation performs (lookup T7), one at a time.
    write32(ddrc + OTHER_28, 0x200);
    trial(phy, Engine::Read, 0, "oth28=0x200", true);
    trial(phy, Engine::Write, 0, "oth28=0x200", true);
    write32(ddrc + OTHER_28, other_orig);
    write32(rx_regs[0], rx_orig[0] & !RX_VREF_BIT4);
    trial(phy, Engine::Read, 0, "rxA-bit4=0", true);
    write32(rx_regs[0], rx_orig[0]);

    // 6. Parameter registers: each byte over GRID, all four engines on rank 0.
    for (i, reg) in PHY_PARAM.iter().enumerate() {
        let orig = params_orig[i];
        let mut timeouts_in_row = 0;
        'reg: for byte in 0..4 {
            for v in GRID {
                let val = (orig & !(0xff << (byte * 8))) | (v as u32) << (byte * 8);
                write32(ddrc + reg, val);
                print!("[k1p] sweep 0x{reg:04x} b{byte}=0x{v:02x} (0x{val:08x}):");
                let mut any_timeout = false;
                for e in Engine::ALL {
                    let (done, worst) = trial(phy, e, 0, "", false);
                    let status = read32(ddrc + PHY_STATUS);
                    any_timeout |= !done;
                    match (done, worst) {
                        (false, _) => print!(" {}=TIMEOUT", e.name()),
                        (true, Some(w)) => print!(" {}={w}", e.name()),
                        (true, None) => print!(" {}=done/0x{status:02x}", e.name()),
                    }
                }
                println!();
                timeouts_in_row = if any_timeout { timeouts_in_row + 1 } else { 0 };
                if timeouts_in_row >= 2 {
                    println!("[k1p] sweep 0x{reg:04x}: two timeouts in a row, skipping the rest");
                    break 'reg;
                }
            }
        }
        write32(ddrc + reg, orig);
    }

    // Restore everything for our training.
    write32(ddrc + OTHER_28, other_orig);
    for (i, r) in rx_regs.iter().enumerate() {
        write32(*r, rx_orig[i]);
    }
    phy.set_tx_mr14(mr14_orig);
    phy.poll_limit = POLL_LIMIT;
    println!("[k1p] characterisation done; registers restored");
}

/// Called before our training at every frequency point.
pub fn before_training(phy: &mut HwPhy) {
    dump(phy, "before");
    if phy.fp == 0 {
        characterise(phy);
        dump(phy, "restored");
    }
}

/// Called after our training at every frequency point.
pub fn after_training(phy: &mut HwPhy, _r: &Result<Report, Error>) {
    dump(phy, "after");
}
