// SPDX-License-Identifier: GPL-2.0-only
//! Equivalence of `sdhci` with the SpacemiT K1 bt0 driver it was moved from
//! (`tests/legacy/k1_mmc.rs`, verbatim): both drive the same controller and
//! card model. Data operations retain exact register traces and outcomes.
//! Clock preflight deliberately moves and reduces capabilities reads: their
//! values, counts and new positions are checked explicitly before comparing
//! all remaining accesses. Unknown-clock refusal has separate controls. The crate runs
//! with the K1 platform below, which is the K1 board's side of the split.

extern crate self as util;

mod model;
use model::{access, reset, with, Access, Card, APMU_SDH0, APMU_SDH2};

pub mod mmio {
    use super::model::access;
    pub fn read8(a: usize) -> u8 {
        access(a, 1, None) as u8
    }
    pub fn read16(a: usize) -> u16 {
        access(a, 2, None) as u16
    }
    pub fn read32(a: usize) -> u32 {
        access(a, 4, None)
    }
    pub fn write8(a: usize, v: u8) {
        access(a, 1, Some(v as u32));
    }
    pub fn write16(a: usize, v: u16) {
        access(a, 2, Some(v as u32));
    }
    pub fn write32(a: usize, v: u32) {
        access(a, 4, Some(v));
    }
}
mod cache {
    pub fn flush(_: usize, _: usize) {}
}
mod sd_capacity {
    pub use storage::sd::sectors;
}
#[allow(dead_code, unused_parens, clippy::all)]
#[path = "legacy/k1_mmc.rs"]
mod legacy;

use regio::RegIo;
use sdhci::{Error, Kind, Partition, Sdhci, SdhciPlatform};

/// `RegIo` on the model, with real byte and halfword accesses.
struct Io;
impl RegIo for Io {
    fn read32(&mut self, a: usize) -> u32 {
        access(a, 4, None)
    }
    fn write32(&mut self, a: usize, v: u32) {
        access(a, 4, Some(v));
    }
    fn read16(&mut self, a: usize) -> u16 {
        access(a, 2, None) as u16
    }
    fn write16(&mut self, a: usize, v: u16) {
        access(a, 2, Some(v as u32));
    }
    fn read8(&mut self, a: usize) -> u8 {
        access(a, 1, None) as u8
    }
    fn write8(&mut self, a: usize, v: u8) {
        access(a, 1, Some(v as u32));
    }
    fn udelay(&mut self, _: u32) {}
    fn now_us(&mut self) -> u64 {
        0
    }
    fn log(&mut self, a: core::fmt::Arguments) {
        LOG.with(|l| l.borrow_mut().push(a.to_string()));
    }
}

std::thread_local! {
    static LOG: std::cell::RefCell<Vec<String>> = const { std::cell::RefCell::new(Vec::new()) };
}

/// The K1 board's platform side (APMU clock/reset, PHY, pads, TX hold,
/// source clock tree), as the K1 driver did it.
struct K1;
const OP_EXT: usize = 0x108;
const LEGACY_CTRL: usize = 0x10c;
const MMC_CTRL: usize = 0x114;
const TX_CFG: usize = 0x11c;
const PHY_CTRL: usize = 0x160;
const PHY_PADCFG: usize = 0x178;
impl SdhciPlatform for K1 {
    fn enable<IO: RegIo>(&mut self, io: &mut IO, _: usize, kind: Kind) {
        let v = io.read32(APMU_SDH0);
        io.write32(APMU_SDH0, v | (1 << 3) | (1 << 0));
        let r = if kind == Kind::Emmc {
            APMU_SDH2
        } else {
            APMU_SDH0
        };
        let v = io.read32(r);
        io.write32(r, v | (1 << 4) | (1 << 1));
    }
    fn init_quirks<IO: RegIo>(&mut self, io: &mut IO, b: usize, kind: Kind) {
        if kind == Kind::Emmc {
            io.set32(b + MMC_CTRL, 1 << 12);
            io.set32(b + PHY_CTRL, 0x3);
            io.set32(b + PHY_PADCFG, 1 << 5);
        }
        io.set32(b + TX_CFG, 1 << 30);
        io.set32(b + OP_EXT, (1 << 11) | (1 << 12));
    }
    fn source_khz<IO: RegIo>(&mut self, io: &mut IO, _: usize, kind: Kind) -> Result<u32, Error> {
        let (reg, third) = match kind {
            Kind::Emmc => (io.read32(APMU_SDH2), 819_200),
            Kind::Sd => (io.read32(APMU_SDH0), 600_000),
        };
        let parents = [409_600, 614_400, 375_000, third, 223_400, 189_000, 106_800];
        let mux = ((reg >> 5) & 7) as usize;
        let div = ((reg >> 8) & 7) + 1;
        parents
            .get(mux)
            .map(|parent| parent / div)
            .ok_or(Error::ClockSource)
    }
    fn tune_for_writes<IO: RegIo>(&mut self, io: &mut IO, b: usize, kind: Kind) {
        // The K1 driver logs these before tuning.
        io.read32(if kind == Kind::Emmc {
            APMU_SDH2
        } else {
            APMU_SDH0
        });
        for r in [PHY_CTRL, PHY_PADCFG, LEGACY_CTRL, TX_CFG, OP_EXT] {
            io.read32(b + r);
        }
        if kind == Kind::Emmc {
            io.set32(b + PHY_CTRL, 0x3);
            io.clrset32(b + PHY_PADCFG, 0x7, 4 | (1 << 5));
        }
        io.set32(b + LEGACY_CTRL, 1 << 6);
        self.set_tx_hold(io, b, true);
    }
    fn set_tx_hold<IO: RegIo>(&mut self, io: &mut IO, b: usize, on: bool) {
        let v = io.read32(b + TX_CFG) & !(1 << 30);
        io.write32(b + TX_CFG, v | if on { 1 << 30 } else { 0 });
    }
}

fn base(kind: Kind) -> usize {
    match kind {
        Kind::Emmc => model::EMMC_BASE,
        Kind::Sd => model::SD_BASE,
    }
}

/// One operation of a scenario.
#[derive(Clone, Copy, Debug)]
enum Op {
    Select(Partition),
    Capacity(Partition),
    Write(u32, usize),
    WriteDma(u32, usize),
    Read(u32, usize),
    ReadSector(u32),
    Width(u8),
    Khz(u32),
    Tune,
    TrimZeroes,
    Trim(u32, usize),
    Widen,
    TxHold(bool),
    Range(u32, usize),
}

fn pattern(seed: u32, n: usize) -> Vec<u32> {
    (0..n * 128)
        .map(|i| seed.wrapping_mul(0x9e37_79b9) ^ (i as u32).wrapping_mul(0x85eb_ca6b))
        .collect()
}

/// Trace, results and card data (partition, lba) -> sector.
type Outcome = (Vec<Vec<Access>>, Vec<String>, Vec<((u8, u32), Vec<u8>)>);

/// Run a scenario with one driver; returns the trace, the results and the
/// model's card data.
fn run<D>(
    card: Card,
    setup: impl Fn(&mut model::Model),
    init: impl FnOnce() -> Result<D, String>,
    step: impl Fn(&mut D, Op, &mut Vec<u32>) -> String,
    ops: &[Op],
) -> Outcome {
    reset(card);
    with(|m| setup(m));
    let mut results = Vec::new();
    let mut phases = Vec::new();
    match init() {
        Err(e) => {
            results.push(e);
            phases.push(with(|m| std::mem::take(&mut m.trace)));
        }
        Ok(mut d) => {
            results.push("init ok".into());
            phases.push(with(|m| std::mem::take(&mut m.trace)));
            for (i, op) in ops.iter().enumerate() {
                let mut buf = pattern(i as u32 + 1, 4);
                if let Op::WriteDma(..) = op {
                    let p = buf.as_ptr() as usize;
                    with(|m| m.dma_source = p);
                }
                let r = step(&mut d, *op, &mut buf);
                phases.push(with(|m| std::mem::take(&mut m.trace)));
                results.push(format!(
                    "{op:?}: {r} {:08x}",
                    buf.iter().fold(0u32, |a, w| a.rotate_left(5) ^ w)
                ));
            }
        }
    }
    let (trace, mut data) = with(|m| {
        (
            std::mem::take(&mut m.trace),
            m.data
                .iter()
                .map(|(k, v)| (*k, v.to_vec()))
                .collect::<Vec<_>>(),
        )
    });
    data.sort();
    assert!(trace.is_empty());
    (phases, results, data)
}

fn legacy_kind(k: Kind) -> legacy::Kind {
    match k {
        Kind::Emmc => legacy::Kind::Emmc,
        Kind::Sd => legacy::Kind::Sd,
    }
}
fn legacy_part(p: Partition) -> legacy::Partition {
    match p {
        Partition::User => legacy::Partition::User,
        Partition::Boot0 => legacy::Partition::Boot0,
        Partition::Boot1 => legacy::Partition::Boot1,
    }
}

fn compare(card: Card, setup: impl Fn(&mut model::Model) + Copy, ops: &[Op]) {
    let kind = if card == Card::Emmc {
        Kind::Emmc
    } else {
        Kind::Sd
    };
    let old = run(
        card,
        setup,
        || legacy::Mmc::init(legacy_kind(kind)).map_err(|e| format!("{e:?}")),
        |m, op, buf| {
            let p = buf.as_mut_ptr() as usize;
            match op {
                Op::Select(x) => format!("{:?}", m.select(legacy_part(x))),
                Op::Capacity(x) => format!("{:?}", m.capacity_sectors(legacy_part(x))),
                Op::Write(l, n) => format!("{:?}", m.write(l, n, p)),
                Op::WriteDma(l, n) => format!("{:?}", m.write_dma(l, n, p)),
                Op::Read(l, n) => format!("{:?}", m.read(l, n, p)),
                Op::ReadSector(l) => format!("{:?}", m.read_sector(l, p)),
                Op::Width(w) => format!("{:?}", m.set_width(w)),
                Op::Khz(k) => format!("{:?}", m.set_khz(k)),
                Op::Tune => format!("{:?}", m.tune_for_writes()),
                Op::TrimZeroes => format!("{:?}", m.trim_zeroes()),
                Op::Trim(l, n) => format!("{:?}", m.trim(l, n)),
                Op::Widen => format!("{:?}", m.widen()),
                Op::TxHold(on) => format!("{:?}", m.set_tx_hold(on)),
                Op::Range(l, n) => format!("{:?}", m.read(l, n, p)),
            }
        },
        ops,
    );
    let new = run(
        card,
        setup,
        || Sdhci::init(Io, K1, base(kind), kind).map_err(|e| format!("{e:?}")),
        |m, op, buf| {
            let p = buf.as_mut_ptr() as usize;
            match op {
                Op::Select(x) => format!("{:?}", m.select(x)),
                Op::Capacity(x) => format!("{:?}", m.capacity_sectors(x)),
                Op::Write(l, n) => format!("{:?}", m.write(l, n, p)),
                Op::WriteDma(l, n) => format!("{:?}", m.write_dma(l, n, p)),
                Op::Read(l, n) => format!("{:?}", m.read(l, n, p)),
                Op::ReadSector(l) => format!("{:?}", m.read_sector(l, p)),
                Op::Width(w) => format!("{:?}", m.set_width(w)),
                Op::Khz(k) => format!("{:?}", m.set_khz(k)),
                Op::Tune => format!("{:?}", m.tune_for_writes().expect("known test source")),
                Op::TrimZeroes => format!("{:?}", m.trim_zeroes()),
                Op::Trim(l, n) => format!("{:?}", m.trim(l, n)),
                Op::Widen => format!("{:?}", m.widen()),
                Op::TxHold(on) => format!("{:?}", m.set_tx_hold(on)),
                Op::Range(l, n) => format!("{:?}", m.read(l, n, p)),
            }
        },
        ops,
    );
    assert_eq!(old.1, new.1, "results differ");
    assert_eq!(old.2, new.2, "card contents differ");
    assert_eq!(old.0.len(), new.0.len());
    for (phase, (old_trace, new_trace)) in old.0.iter().zip(&new.0).enumerate() {
        let clock_op = phase == 0 || matches!(ops[phase - 1], Op::Khz(_) | Op::Tune);
        if clock_op {
            // Only the explicit capabilities transaction changes are excluded.
            // Every write and every other read remains compared, in order.
            let caps = Access::R(4, base(kind) + 0x40, 200 << 8);
            let old_count = if phase == 0 {
                if old.1[0] == "init ok" {
                    3
                } else {
                    2
                }
            } else {
                2
            };
            let new_count = if phase == 0 {
                if new.1[0] == "init ok" {
                    2
                } else {
                    1
                }
            } else {
                1
            };
            assert_eq!(old_trace.iter().filter(|x| **x == caps).count(), old_count);
            assert_eq!(new_trace.iter().filter(|x| **x == caps).count(), new_count);
            assert_eq!(new_trace[if phase == 0 { 4 } else { 0 }], caps);
            let old_rest: Vec<_> = old_trace.iter().filter(|x| **x != caps).collect();
            let new_rest: Vec<_> = new_trace.iter().filter(|x| **x != caps).collect();
            assert_eq!(
                old_rest, new_rest,
                "non-capabilities trace in phase {phase}"
            );
        } else {
            assert_eq!(old_trace, new_trace, "data operation phase {phase}");
        }
    }
}

#[test]
fn emmc_boot_partition_flasher_session() {
    use Op::*;
    compare(
        Card::Emmc,
        |_| {},
        &[
            Capacity(Partition::User),
            Capacity(Partition::Boot0),
            Select(Partition::Boot0),
            Write(0, 1),
            Write(4, 3),
            Read(0, 4),
            Select(Partition::User),
            Width(8),
            Khz(52_000),
            Tune,
            TxHold(false),
            TxHold(true),
            TrimZeroes,
            Trim(10, 5),
            Trim(0, 0),
            ReadSector(11),
            Widen,
            Range(u32::MAX, 2),
        ],
    );
}

#[test]
fn emmc_sdma_write_across_a_boundary() {
    use Op::*;
    compare(
        Card::Emmc,
        |m| m.dma_boundary_once = true,
        &[
            WriteDma(100, 3),
            WriteDma(200, 1),
            Read(100, 3),
            Read(200, 1),
        ],
    );
}

#[test]
fn emmc_widen_falls_back_to_four_bits() {
    use Op::*;
    compare(
        Card::Emmc,
        |m| {
            m.fail_cmd = Some(8);
            m.fail_times = 1;
        },
        &[Widen, ReadSector(0)],
    );
}

#[test]
fn emmc_failed_read_then_recovers() {
    use Op::*;
    compare(
        Card::Emmc,
        |m| {
            m.fail_cmd = Some(17);
            m.fail_times = 1;
        },
        &[Read(0, 2), Read(0, 2), Write(1, 1), ReadSector(1)],
    );
}

#[test]
fn emmc_init_fails_when_cmd1_fails() {
    compare(
        Card::Emmc,
        |m| {
            m.fail_cmd = Some(1);
            m.fail_times = 1;
        },
        &[],
    );
}

#[test]
fn sdhc_session() {
    use Op::*;
    compare(
        Card::SdHc,
        |_| {},
        &[
            Capacity(Partition::User),
            Capacity(Partition::Boot0),
            Select(Partition::Boot0),
            Width(4),
            Widen,
            Write(0, 2),
            WriteDma(8, 2),
            Read(0, 2),
            Read(8, 2),
            Khz(50_000),
            Tune,
            Trim(0, 1),
            Range(15_523_839, 2),
        ],
    );
}

#[test]
fn sd_v1_byte_addressed_session() {
    use Op::*;
    compare(
        Card::SdV1,
        |_| {},
        &[
            Capacity(Partition::User),
            Write(3, 2),
            Read(3, 2),
            Range((1 << 23) - 1, 2),
        ],
    );
}

#[test]
fn actual_initial_clock_uses_platform_source_instead_of_assumed_200mhz() {
    for (card, kind) in [(Card::Emmc, Kind::Emmc), (Card::SdHc, Kind::Sd)] {
        reset(card);
        with(|m| m.caps_mhz = 0);
        let mut card = Sdhci::init(Io, K1, base(kind), kind).unwrap();
        assert_eq!(card.source_khz(), Ok(409_600));
        let clocks = with(|m| {
            m.trace
                .iter()
                .copied()
                .filter(|a| matches!(a,Access::W(2,p,_) if *p==base(kind)+0x2c))
                .collect::<Vec<_>>()
        });
        assert_eq!(
            clocks,
            vec![
                Access::W(2, base(kind) + 0x2c, 0),
                Access::W(2, base(kind) + 0x2c, 0x81),
                Access::W(2, base(kind) + 0x2c, 0x85),
                Access::W(2, base(kind) + 0x2c, 0),
                Access::W(2, base(kind) + 0x2c, 0x901),
                Access::W(2, base(kind) + 0x2c, 0x905)
            ]
        );
    }
}
#[test]
fn unknown_source_refuses_before_card_clock_or_pad_writes() {
    for kind in [Kind::Emmc, Kind::Sd] {
        reset(if kind == Kind::Emmc {
            Card::Emmc
        } else {
            Card::SdHc
        });
        let mut card = Sdhci::init(Io, K1, base(kind), kind).unwrap();
        with(|m| {
            m.caps_mhz = 0;
            m.write(
                if kind == Kind::Emmc {
                    APMU_SDH2
                } else {
                    APMU_SDH0
                },
                4,
                7 << 5,
            );
            m.trace.clear();
        });
        assert_eq!(card.set_khz(52_000), Err(Error::ClockSource));
        assert_eq!(card.tune_for_writes(), Err(Error::ClockSource));
        with(|m| assert!(m.trace.iter().all(|a| matches!(a, Access::R(..)))));
    }
}
#[test]
fn init_refuses_unknown_source_after_enable_before_quirks() {
    for kind in [Kind::Emmc, Kind::Sd] {
        reset(if kind == Kind::Emmc {
            Card::Emmc
        } else {
            Card::SdHc
        });
        let reg = if kind == Kind::Emmc {
            APMU_SDH2
        } else {
            APMU_SDH0
        };
        with(|m| {
            m.caps_mhz = 0;
            m.write(reg, 4, 7 << 5);
            m.trace.clear();
        });
        assert!(matches!(
            Sdhci::init(Io, K1, base(kind), kind),
            Err(Error::ClockSource)
        ));
        with(|m| {
            assert_eq!(m.trace.len(), 6);
            assert!(m
                .trace
                .iter()
                .all(|a| !matches!(a,Access::W(_,p,_) if *p>=base(kind)&&*p<base(kind)+0x200)));
        });
    }
}
#[test]
fn invalid_clock_rates_refuse_without_writes_and_large_rate_does_not_overflow() {
    reset(Card::Emmc);
    let mut card = Sdhci::init(Io, K1, base(Kind::Emmc), Kind::Emmc).unwrap();
    for rate in [0, 1, 97] {
        with(|m| m.trace.clear());
        assert_eq!(card.set_khz(rate), Err(Error::ClockRate));
        with(|m| assert!(m.trace.iter().all(|a| matches!(a, Access::R(..)))));
    }
    assert_eq!(card.set_khz(u32::MAX), Ok(100_000));
}
#[test]
fn missing_platform_clock_is_not_a_default_200mhz() {
    struct Plain;
    impl SdhciPlatform for Plain {}
    reset(Card::Emmc);
    with(|m| m.caps_mhz = 0);
    assert!(matches!(
        Sdhci::init(Io, Plain, base(Kind::Emmc), Kind::Emmc),
        Err(Error::ClockSource)
    ));
    with(|m| assert_eq!(m.trace, vec![Access::R(4, base(Kind::Emmc) + 0x40, 0)]));
}
#[test]
fn init_logs_the_resolved_source() {
    reset(Card::Emmc);
    with(|m| m.caps_mhz = 0);
    LOG.with(|l| l.borrow_mut().clear());
    Sdhci::init(Io, K1, base(Kind::Emmc), Kind::Emmc).unwrap();
    LOG.with(|l| {
        assert_eq!(
            l.borrow()[0],
            "[mmc] eMMC clock source 409600 kHz, bus power 1.8 V on (0x0b)\n"
        )
    });
}

#[test]
fn zero_from_a_platform_cannot_be_used_as_a_known_source() {
    struct Zero;
    impl SdhciPlatform for Zero {
        fn source_khz<IO: RegIo>(&mut self, _: &mut IO, _: usize, _: Kind) -> Result<u32, Error> {
            Ok(0)
        }
    }
    reset(Card::Emmc);
    with(|m| m.caps_mhz = 0);
    assert!(matches!(
        Sdhci::init(Io, Zero, base(Kind::Emmc), Kind::Emmc),
        Err(Error::ClockSource)
    ));
    with(|m| assert_eq!(m.trace, vec![Access::R(4, base(Kind::Emmc) + 0x40, 0)]));
}
