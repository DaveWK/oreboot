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
//!   3. read eye: the PHY RX Vref axis (all codes), the read engine on every
//!      rank at each point, the worst DQ window per point; centred, set and
//!      re-run by the shared `dram_train::vref_sweep`;
//!   4. write eye: DRAM VREF(DQ) via MR14 along the JEDEC axis, searched
//!      outwards from the current value, centred and re-run the same way;
//!      MR14 read back from every rank and channel.
//! The sweep policy is ore-edk2-boot's v1.0.0 default for both eyes
//! (`SweepPolicy::DEFAULT`: ratio-bounded search at 900/1000 of the peak,
//! plateau centring when the eye is wider than the scan, knob-response
//! checks); `SweepPolicy::legacy()` (the pre-v1.0.0 walk-out and half-height
//! centring) is used only by the equivalence tests.
//! The engines' parameter registers keep their hardware values unless a
//! build sets them (`PARAMS`). Pass/fail is decided on the measured windows
//! here and on the memory test and soak, not on the engines' own summary
//! status.

use crate::train_engine::{Engine, Error, Phy};
use crate::train_eye::{
    lp4x_mr14_permille, tx_axis_index, tx_axis_mr14, worst, Eye, RX_CODES, TX_AXIS_LEN,
};
use dram_train::vref_sweep::{sweep_axis_with, Axis, Sweep, SweepError, SweepPolicy};
use log::{print, println};

#[derive(Debug, Clone, Copy)]
pub struct Config {
    /// Smallest acceptable worst-DQ window after the final re-run.
    pub min_window: u8,
    /// Search and centring of both Vref axes (dram_train::vref_sweep).
    pub policy: SweepPolicy,
    /// Closed points in a row that end the write-eye search in a direction.
    pub tx_stop_after: usize,
    /// Start of the write-eye search when MR14 holds a reserved value.
    pub tx_default_mr14: u8,
}

/// The build's configuration. MR14 0x19 is the GPL init value for LPDDR4X.
pub const CONFIG: Config = Config {
    min_window: 1,
    policy: SweepPolicy::DEFAULT,
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

/// One Vref axis of a K1 training engine: `set` programs axis point `i`, a
/// run is the engine on one rank and its 32 per-DQ windows.
struct EngineAxis<'a, P: Phy> {
    phy: &'a mut P,
    engine: Engine,
    set: fn(&mut P, usize),
}

impl<P: Phy> Axis<32> for EngineAxis<'_, P> {
    type Error = Error;
    fn ranks(&self) -> u32 {
        self.phy.ranks()
    }
    fn set(&mut self, i: usize) {
        (self.set)(self.phy, i)
    }
    fn run(&mut self, rank: u32) -> Result<[u8; 32], Error> {
        self.phy.run(self.engine, rank)?;
        Ok(self.phy.window(self.engine))
    }
}

fn set_rx<P: Phy>(phy: &mut P, i: usize) {
    phy.set_rx_vref(i as u8)
}

fn set_tx<P: Phy>(phy: &mut P, i: usize) {
    phy.set_tx_mr14(tx_axis_mr14(i))
}

/// Sweep one eye; `which` 0 read, 1 write (for errors and logs).
fn eye<P: Phy>(
    phy: &mut P,
    engine: Engine,
    set: fn(&mut P, usize),
    profile: &mut [Option<u8>],
    sweep: &Sweep,
    policy: &SweepPolicy,
    which: u8,
) -> Result<(Eye, [u8; 32], bool), Error> {
    let fp = phy.fp();
    let mut axis = EngineAxis { phy, engine, set };
    match sweep_axis_with(&mut axis, profile, sweep, policy) {
        Ok((o, outside)) => Ok((o.eye, o.window, outside)),
        Err(SweepError::Search(e)) => Err(e),
        Err(SweepError::Closed) => Err(Error::Closed { fp, which }),
        Err(SweepError::Margin { worst }) => Err(Error::Margin { fp, which, worst }),
        Err(SweepError::Sanity(e)) => {
            println!("[k1t] fp{fp} eye {which}: knob-response check failed: {e:?}");
            Err(Error::Sanity { fp, which })
        }
    }
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

fn print_eye(fp: u32, what: &str, e: &Eye, outside: bool) {
    println!(
        "[k1t] fp{fp} {what} eye: open {}..{} at level {} (peak {} at {}), chosen {} width {}{}",
        e.lo,
        e.hi,
        e.level,
        e.peak_width,
        e.peak,
        e.center,
        e.center_width,
        if outside {
            "; an eye edge lies outside the scan"
        } else {
            ""
        }
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

    // Read eye over the full RX Vref range, from the current code.
    let mut rx = [None; RX_CODES];
    let sweep = Sweep {
        start: phy.rx_vref() as usize,
        stop_after: usize::MAX,
        min_window: cfg.min_window,
    };
    let r = eye(phy, Engine::Read, set_rx, &mut rx, &sweep, &cfg.policy, 0);
    print_profile(fp, "read", &rx, |i| i as u32);
    let (rx_eye, rx_final, rx_outside) = r?;
    print_eye(fp, "read", &rx_eye, rx_outside);
    print_window(fp, "read", &rx_final);
    let rx_code = rx_eye.center as u8;
    let rx_worst = worst(&rx_final);

    // Write eye along the JEDEC MR14 axis, outwards from the current value.
    let start = tx_axis_index(phy.tx_mr14())
        .or(tx_axis_index(cfg.tx_default_mr14))
        .unwrap_or(0);
    let mut tx = [None; TX_AXIS_LEN];
    let sweep = Sweep {
        start,
        stop_after: cfg.tx_stop_after,
        min_window: cfg.min_window,
    };
    let r = eye(phy, Engine::Write, set_tx, &mut tx, &sweep, &cfg.policy, 1);
    print_profile(fp, "write", &tx, |i| lp4x_mr14_permille(tx_axis_mr14(i)));
    let (tx_eye, tx_final, tx_outside) = r?;
    print_eye(fp, "write", &tx_eye, tx_outside);
    print_window(fp, "write", &tx_final);
    let mr14 = tx_axis_mr14(tx_eye.center);
    let tx_worst = worst(&tx_final);

    // Check that every die got our MR14 (MRR of MR14). A mismatch is
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
