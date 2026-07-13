// SPDX-License-Identifier: GPL-2.0-only
//! dram_train::vref_sweep against the K1's own training it came from
//! (ore-edk-boot-opi 78479e21ba train_own.rs, verbatim in tests/legacy/k1/):
//! the read and write eye steps composed on `sweep_axis` give the same
//! report or error and the same sequence of PHY calls, on simulated eyes.
#![allow(dead_code, unused_imports, clippy::all)]

// The verbatim K1 files name oreboot's `log` and `util` crates; this test
// crate stands in for both (no extra packages, so the oreboot workspace's
// Cargo.lock is unaffected).
extern crate self as log;
extern crate self as util;

#[macro_export]
macro_rules! print {
    () => {};
    ($($t:tt)*) => {{ let _ = format_args!($($t)*); }};
}
#[macro_export]
macro_rules! println {
    () => {};
    ($($t:tt)*) => {{ let _ = format_args!($($t)*); }};
}
pub mod mmio {
    pub fn read32(_: usize) -> u32 {
        0
    }
    pub fn write32(_: usize, _: u32) {}
}
pub fn nop_delay(_: usize) {}

#[path = "legacy/k1/train_engine.rs"]
mod train_engine;
#[path = "legacy/k1/train_eye.rs"]
mod train_eye;
#[path = "legacy/k1/train_own.rs"]
mod train_own;

use dram_train::vref_sweep::{sweep_axis, sweep_axis_legacy, Axis, Sweep, SweepError};
use train_engine::{Engine, Error, Phy};

#[derive(Clone, Debug, PartialEq)]
enum Call {
    SetRx(u8),
    SetTx(u8),
    Run(Engine, u32),
    Mrr(u32, u32, u32),
}

/// A simulated K1 PHY: read windows peak at `rx_peak`, write windows at
/// MR14 axis point `tx_peak`, with per-lane and per-rank skew; optional
/// engine failure after `fail_after` runs.
#[derive(Clone)]
struct Sim {
    fp: u32,
    ranks: u32,
    rx: u8,
    mr14: u8,
    rx_peak: i32,
    tx_peak: i32,
    height: i32,
    slope: i32,
    skew: [i32; 32],
    runs: u32,
    fail_after: Option<u32>,
    mrr_lies: bool,
    last: [u8; 32],
    calls: Vec<Call>,
}

impl Sim {
    fn new(seed: u64) -> Sim {
        let mut s = seed | 1;
        let mut next = move || {
            s ^= s << 13;
            s ^= s >> 7;
            s ^= s << 17;
            s
        };
        let mut skew = [0; 32];
        for k in skew.iter_mut() {
            *k = (next() % 5) as i32;
        }
        Sim {
            fp: (next() % 3) as u32,
            ranks: 1 + (next() % 2) as u32,
            rx: (next() % 16) as u8,
            mr14: [0x19, 0x0c, 0x40 | 30, 0x7f, 0x33][(next() % 5) as usize],
            rx_peak: (next() % 16) as i32,
            tx_peak: (next() % 81) as i32,
            height: (next() % 40) as i32 - 4,
            slope: 1 + (next() % 6) as i32,
            skew,
            runs: 0,
            fail_after: if next() % 7 == 0 {
                Some((next() % 40) as u32)
            } else {
                None
            },
            mrr_lies: next() % 5 == 0,
            last: [0; 32],
            calls: vec![],
        }
    }
    fn tx_index(&self) -> i32 {
        train_eye::tx_axis_index(self.mr14).map_or(-100, |i| i as i32)
    }
}

impl Phy for Sim {
    fn fp(&self) -> u32 {
        self.fp
    }
    fn ranks(&self) -> u32 {
        self.ranks
    }
    fn rx_vref(&self) -> u8 {
        self.rx
    }
    fn set_rx_vref(&mut self, code: u8) {
        self.calls.push(Call::SetRx(code));
        self.rx = code & 0xf;
    }
    fn tx_mr14(&self) -> u8 {
        self.mr14
    }
    fn set_tx_mr14(&mut self, op: u8) {
        self.calls.push(Call::SetTx(op));
        self.mr14 = op & 0x7f;
    }
    fn run(&mut self, engine: Engine, rank: u32) -> Result<(), Error> {
        self.calls.push(Call::Run(engine, rank));
        self.runs += 1;
        if let Some(n) = self.fail_after {
            if self.runs > n {
                return Err(Error::Timeout {
                    engine,
                    rank,
                    last: 7,
                });
            }
        }
        let (pos, peak) = match engine {
            Engine::Write => (self.tx_index(), self.tx_peak),
            _ => (self.rx as i32, self.rx_peak),
        };
        for (dq, w) in self.last.iter_mut().enumerate() {
            let v = self.height - (pos - peak).abs() * self.slope - self.skew[dq] - rank as i32;
            *w = v.clamp(0, 255) as u8;
        }
        Ok(())
    }
    fn window(&self, _: Engine) -> [u8; 32] {
        self.last
    }
    fn mr_read(&mut self, mr: u32, ch: u32, rank: u32) -> Result<u8, Error> {
        self.calls.push(Call::Mrr(mr, ch, rank));
        Ok(if self.mrr_lies && ch == 1 {
            self.mr14 ^ 1
        } else {
            self.mr14
        })
    }
}

/// The K1 axes on the crate's trait.
struct Rx<'a>(&'a mut Sim);
impl Axis<32> for Rx<'_> {
    type Error = Error;
    fn ranks(&self) -> u32 {
        self.0.ranks
    }
    fn set(&mut self, i: usize) {
        self.0.set_rx_vref(i as u8);
    }
    fn run(&mut self, rank: u32) -> Result<[u8; 32], Error> {
        self.0.run(Engine::Read, rank)?;
        Ok(self.0.window(Engine::Read))
    }
}
struct Tx<'a>(&'a mut Sim);
impl Axis<32> for Tx<'_> {
    type Error = Error;
    fn ranks(&self) -> u32 {
        self.0.ranks
    }
    fn set(&mut self, i: usize) {
        self.0.set_tx_mr14(train_eye::tx_axis_mr14(i));
    }
    fn run(&mut self, rank: u32) -> Result<[u8; 32], Error> {
        self.0.run(Engine::Write, rank)?;
        Ok(self.0.window(Engine::Write))
    }
}

/// The K1 board step on the crate: the same sequence as train_own::train_fp.
fn train_fp_on_crate(phy: &mut Sim, cfg: &train_own::Config) -> Result<train_own::Report, Error> {
    let fp = phy.fp;
    for rank in 0..phy.ranks {
        phy.run(Engine::WriteLeveling, rank)?;
    }
    for rank in 0..phy.ranks {
        phy.run(Engine::ReadGate, rank)?;
    }
    let map = |which: u8| {
        move |e: SweepError<Error>| match e {
            SweepError::Search(e) => e,
            SweepError::Closed => Error::Closed { fp, which },
            SweepError::Margin { worst } => Error::Margin { fp, which, worst },
            SweepError::Sanity(e) => unreachable!("the legacy sweep runs no checks: {e:?}"),
            SweepError::NotLive { point } => {
                unreachable!("this axis has no live readback: point {point}")
            }
        }
    };
    let mut rx = [None; train_eye::RX_CODES];
    let r = sweep_axis_legacy(
        &mut Rx(phy),
        &mut rx,
        &Sweep {
            start: 0,
            stop_after: usize::MAX,
            min_window: cfg.min_window,
        },
    )
    .map_err(map(0))?;
    let start = train_eye::tx_axis_index(phy.tx_mr14())
        .or(train_eye::tx_axis_index(cfg.tx_default_mr14))
        .unwrap_or(0);
    let mut tx = [None; train_eye::TX_AXIS_LEN];
    let t = sweep_axis_legacy(
        &mut Tx(phy),
        &mut tx,
        &Sweep {
            start,
            stop_after: cfg.tx_stop_after,
            min_window: cfg.min_window,
        },
    )
    .map_err(map(1))?;
    let mr14 = train_eye::tx_axis_mr14(t.point);
    let mut mr14_mismatch = false;
    for rank in 0..phy.ranks {
        for ch in 0..2 {
            match phy.mr_read(14, ch, rank) {
                Ok(v) if v == mr14 => {}
                _ => mr14_mismatch = true,
            }
        }
    }
    Ok(train_own::Report {
        rx: train_eye::Eye {
            lo: r.eye.lo,
            hi: r.eye.hi,
            center: r.eye.center,
            peak: r.eye.peak,
            peak_width: r.eye.peak_width,
            level: r.eye.level,
            center_width: r.eye.center_width,
        },
        rx_code: r.point as u8,
        rx_worst: r.worst,
        tx: train_eye::Eye {
            lo: t.eye.lo,
            hi: t.eye.hi,
            center: t.eye.center,
            peak: t.eye.peak,
            peak_width: t.eye.peak_width,
            level: t.eye.level,
            center_width: t.eye.center_width,
        },
        mr14,
        tx_worst: t.worst,
        mr14_mismatch,
    })
}

#[test]
fn k1_eye_steps_on_the_crate_match_train_own() {
    let (mut ok, mut err) = (0, 0);
    for seed in 1..6000u64 {
        for cfg in [
            train_own::CONFIG,
            train_own::Config {
                min_window: 6,
                tx_stop_after: 1,
                tx_default_mr14: 0x40 | 25,
            },
        ] {
            let mut a = Sim::new(seed);
            let mut b = a.clone();
            let old = train_own::train_fp(&mut a, &cfg);
            let new = train_fp_on_crate(&mut b, &cfg);
            assert_eq!(old, new, "seed {seed}");
            assert_eq!(a.calls, b.calls, "seed {seed}");
            if old.is_ok() {
                ok += 1
            } else {
                err += 1
            }
        }
    }
    assert!(
        ok > 1000 && err > 1000,
        "both outcomes covered: {ok} ok, {err} errors"
    );
}

#[test]
fn sweep_reports_closed_and_margin_and_keeps_the_profile() {
    struct Flat(u8);
    impl Axis<4> for Flat {
        type Error = ();
        fn ranks(&self) -> u32 {
            2
        }
        fn set(&mut self, _: usize) {}
        fn run(&mut self, _: u32) -> Result<[u8; 4], ()> {
            Ok([self.0; 4])
        }
    }
    let mut p = [None; 5];
    let cfg = Sweep {
        start: 2,
        stop_after: 2,
        min_window: 3,
    };
    assert_eq!(
        sweep_axis(&mut Flat(0), &mut p, &cfg),
        Err(SweepError::Closed)
    );
    assert!(p.iter().all(|w| *w == Some(0)));
    assert_eq!(
        sweep_axis(&mut Flat(2), &mut p, &cfg),
        Err(SweepError::Margin { worst: 2 })
    );
    let o = sweep_axis(&mut Flat(9), &mut p, &cfg).unwrap();
    assert_eq!((o.point, o.worst, o.window), (2, 9, [9; 4]));
}
