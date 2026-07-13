// Verbatim copy of ore-edk-boot-opi 78479e21ba
// oreboot/src/mainboard/spacemit/k1x/bt0/src/train_own.rs (equivalence oracle; never built into firmware).
// SPDX-License-Identifier: GPL-2.0-only
//! Our own K1 LPDDR4/4X training for one frequency point (cargo feature
//! `own-training`; the translated vendor training stays the default build).
//!
//! Written for ore-edk-boot-opi from JEDEC LPDDR4/4X and generic DDR training
//! practice; no logic or parameter is taken from SpacemiT's training image or
//! its translation (doc/K1-OWN-TRAINING-PLAN.md). Register knowledge and its
//! sources are in `train_engine.rs` and doc/K1-DDR-REGISTERS.md.
//!
//! Sequence, in the usual LPDDR4 order (write leveling, then read timing
//! before write timing, because the write eye is measured by reading back):
//!   1. write leveling on every rank;
//!   2. read gate on every rank, which must report pass;
//!   3. read eye: every PHY RX Vref code, the read engine on every rank at
//!      each, the worst DQ window per code; the Vref at the centre of the eye
//!      at half height (`train_eye::center`); the engine re-run there;
//!   4. write eye: DRAM VREF(DQ) via MR14 along the JEDEC axis, searched
//!      outwards from the current value until the eye closes on both sides;
//!      centred and re-run the same way; MR14 read back from every rank and
//!      channel.
//! The engines' parameter registers keep their hardware values unless a
//! build sets them (`PARAMS`); choosing our own values is step 1's hardware
//! work. Pass/fail is decided on the measured windows here and on the memory
//! test and soak, not on the engines' own summary status.

use crate::train_engine::{Engine, Error, Phy};
use crate::train_eye::{
    center, lp4x_mr14_permille, tx_axis_index, tx_axis_mr14, walk_out, worst, Eye, RX_CODES,
    TX_AXIS_LEN,
};
use log::{print, println};

#[derive(Debug, Clone, Copy)]
pub struct Config {
    /// Smallest acceptable worst-DQ window after the final re-run.
    pub min_window: u8,
    /// Closed points in a row that end the write-eye search in a direction.
    pub tx_stop_after: usize,
    /// Start of the write-eye search when MR14 holds a reserved value.
    pub tx_default_mr14: u8,
}

/// The build's configuration. MR14 0x19 is the GPL init value for LPDDR4X.
pub const CONFIG: Config = Config {
    min_window: 1,
    tx_stop_after: 3,
    tx_default_mr14: 0x19,
};

const fn hex_digit(c: u8) -> u32 {
    match c {
        b'0'..=b'9' => (c - b'0') as u32,
        b'a'..=b'f' => (c - b'a' + 10) as u32,
        b'A'..=b'F' => (c - b'A' + 10) as u32,
        _ => panic!("K1_TRAIN_P13xx must be hexadecimal"),
    }
}

/// Parse an optional build-time hex value ("0x" prefix optional).
pub const fn env_hex(v: Option<&str>) -> Option<u32> {
    match v {
        None => None,
        Some(s) => {
            let b = s.as_bytes();
            let mut i = 0;
            if b.len() > 2 && b[0] == b'0' && (b[1] == b'x' || b[1] == b'X') {
                i = 2;
            }
            if i == b.len() || b.len() - i > 8 {
                panic!("K1_TRAIN_P13xx must be 1 to 8 hex digits");
            }
            let mut n = 0u32;
            while i < b.len() {
                n = (n << 4) | hex_digit(b[i]);
                i += 1;
            }
            Some(n)
        }
    }
}

/// Engine parameter register values set at build time, in the order of
/// `train_engine::PHY_PARAM` (0x13e8, 0x13ec, 0x13f8). `None` keeps the
/// hardware value. Values come from our probe measurements, never from the
/// vendor image.
pub const PARAMS: [Option<u32>; 3] = [
    env_hex(option_env!("K1_TRAIN_P13E8")),
    env_hex(option_env!("K1_TRAIN_P13EC")),
    env_hex(option_env!("K1_TRAIN_P13F8")),
];

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Report {
    pub rx: Eye,
    pub rx_code: u8,
    pub rx_worst: u8,
    pub tx: Eye,
    pub mr14: u8,
    pub tx_worst: u8,
    /// MR14 read back from a rank/channel differed from what was sent.
    pub mr14_mismatch: bool,
}

/// Worst window over all DQ and ranks with the engine run on every rank.
fn eye_width<P: Phy>(phy: &mut P, engine: Engine) -> Result<u8, Error> {
    let mut w = u8::MAX;
    for rank in 0..phy.ranks() {
        phy.run(engine, rank)?;
        w = w.min(worst(&phy.window(engine)));
    }
    Ok(w)
}

/// Re-run on every rank; per-DQ minimum over ranks.
fn final_window<P: Phy>(phy: &mut P, engine: Engine) -> Result<[u8; 32], Error> {
    let mut m = [u8::MAX; 32];
    for rank in 0..phy.ranks() {
        phy.run(engine, rank)?;
        for (a, b) in m.iter_mut().zip(phy.window(engine)) {
            *a = (*a).min(b);
        }
    }
    Ok(m)
}

fn print_profile(fp: u32, what: &str, w: &[Option<u8>], label: impl Fn(usize) -> u32) {
    println!("[k1t] fp{fp} {what} eye profile (point:Vref worst-window):");
    let mut n = 0;
    for (i, w) in w.iter().enumerate() {
        if let Some(w) = w {
            print!(" {i:02}:{:03}={w:3}", label(i));
            n += 1;
            if n % 8 == 0 {
                println!();
            }
        }
    }
    if n % 8 != 0 {
        println!();
    }
}

fn print_window(fp: u32, what: &str, m: &[u8; 32]) {
    print!("[k1t] fp{fp} {what} per-DQ window:");
    for v in m {
        print!(" {v}");
    }
    println!();
}

fn print_eye(fp: u32, what: &str, e: &Eye) {
    println!(
        "[k1t] fp{fp} {what} eye: open {}..{} at level {} (peak {} at {}), chosen {} width {}",
        e.lo, e.hi, e.level, e.peak_width, e.peak, e.center, e.center_width
    );
}

/// Train the current frequency point.
pub fn train_fp<P: Phy>(phy: &mut P, cfg: &Config) -> Result<Report, Error> {
    let fp = phy.fp();
    let ranks = phy.ranks();
    println!(
        "[k1t] fp{fp}: own training, {ranks} rank(s); start RX code {}, MR14 0x{:02x}",
        phy.rx_vref(),
        phy.tx_mr14()
    );

    for rank in 0..ranks {
        phy.run(Engine::WriteLeveling, rank)?;
    }
    for rank in 0..ranks {
        phy.run(Engine::ReadGate, rank)?;
    }
    println!("[k1t] fp{fp}: write leveling and read gate pass");

    // Read eye over the full RX Vref range.
    let mut rx = [None; RX_CODES];
    walk_out(&mut rx, 0, usize::MAX, |code| {
        phy.set_rx_vref(code as u8);
        eye_width(phy, Engine::Read)
    })?;
    print_profile(fp, "read", &rx, |i| i as u32);
    let rx_eye = center(&rx).map_err(|_| Error::Closed { fp, which: 0 })?;
    print_eye(fp, "read", &rx_eye);
    let rx_code = rx_eye.center as u8;
    phy.set_rx_vref(rx_code);
    let rx_final = final_window(phy, Engine::Read)?;
    print_window(fp, "read", &rx_final);
    let rx_worst = worst(&rx_final);
    if rx_worst < cfg.min_window {
        return Err(Error::Margin {
            fp,
            which: 0,
            worst: rx_worst,
        });
    }

    // Write eye along the JEDEC MR14 axis, outwards from the current value.
    let start = tx_axis_index(phy.tx_mr14())
        .or(tx_axis_index(cfg.tx_default_mr14))
        .unwrap_or(0);
    let mut tx = [None; TX_AXIS_LEN];
    walk_out(&mut tx, start, cfg.tx_stop_after, |i| {
        phy.set_tx_mr14(tx_axis_mr14(i));
        eye_width(phy, Engine::Write)
    })?;
    print_profile(fp, "write", &tx, |i| lp4x_mr14_permille(tx_axis_mr14(i)));
    let tx_eye = center(&tx).map_err(|_| Error::Closed { fp, which: 1 })?;
    print_eye(fp, "write", &tx_eye);
    let mr14 = tx_axis_mr14(tx_eye.center);
    phy.set_tx_mr14(mr14);
    let tx_final = final_window(phy, Engine::Write)?;
    print_window(fp, "write", &tx_final);
    let tx_worst = worst(&tx_final);
    if tx_worst < cfg.min_window {
        return Err(Error::Margin {
            fp,
            which: 1,
            worst: tx_worst,
        });
    }

    // Check that every die got our MR14 (JEDEC MRR of MR14). A mismatch is
    // reported, not fatal: the windows above were measured either way.
    let mut mr14_mismatch = false;
    for rank in 0..ranks {
        for ch in 0..2 {
            match phy.mr_read(14, ch, rank) {
                Ok(v) if v == mr14 => {}
                Ok(v) => {
                    mr14_mismatch = true;
                    println!("[k1t] WARNING fp{fp} rank {rank} ch {ch}: MR14 reads 0x{v:02x}, sent 0x{mr14:02x}");
                }
                Err(e) => {
                    mr14_mismatch = true;
                    println!("[k1t] WARNING fp{fp} rank {rank} ch {ch}: MR14 read failed: {e:?}");
                }
            }
        }
    }
    println!(
        "[k1t] fp{fp}: done; RX code {rx_code} worst {rx_worst}; MR14 0x{mr14:02x} ({} per mille VDDQ, LPDDR4X) worst {tx_worst}",
        lp4x_mr14_permille(mr14)
    );
    Ok(Report {
        rx: rx_eye,
        rx_code,
        rx_worst,
        tx: tx_eye,
        mr14,
        tx_worst,
        mr14_mismatch,
    })
}
