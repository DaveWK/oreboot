// SPDX-License-Identifier: GPL-2.0-only
//! `psci_el3::gicv3` against the board's gic.rs (tests/legacy/a7a_gic.rs)
//! on one modelled GIC-600: distributor RWP delays, redistributor power
//! group transitions and RDPD retries, WAKER ChildrenAsleep delays, GICR_TYPER
//! affinities and Last. Both run the boot sequence (distributor, this CPU's
//! interface, CPU_OFF) over thousands of seeded configurations; the MMIO and
//! system-register traces and the results must be identical.
#![allow(dead_code, clippy::all)]

use std::cell::RefCell;
use std::collections::HashMap;

use psci_el3::gicv3::{self, Gic, GicError, IccEl3};
use regio::RegIo;

extern crate self as util;

#[derive(Clone, Debug, PartialEq, Eq)]
enum Ev {
    R(usize, u32),
    W(usize, u32),
    Sre(u64),
    Ctlr(u64),
    Igrpen1(u64),
}

const GICD: usize = 0x0340_0000;
const GICR: usize = 0x0346_0000;
const STRIDE: usize = 0x2_0000;

#[derive(Clone, Default)]
struct Cfg {
    it_lines: u32,
    rwp_delay: u32,
    transition: [u32; 8],
    rdpd_retries: [u32; 8],
    aff: [u32; 8],
    last: usize,
    waker_delay: u32,
}

#[derive(Default)]
struct Model {
    cfg: Cfg,
    regs: HashMap<usize, u32>,
    rwp_left: u32,
    trans_left: [u32; 8],
    rdpd_left: [u32; 8],
    waker_left: u32,
    trace: Vec<Ev>,
}

impl Model {
    fn new(cfg: Cfg) -> Self {
        Model {
            trans_left: cfg.transition,
            rdpd_left: cfg.rdpd_retries,
            cfg,
            ..Default::default()
        }
    }
    fn rd_index(addr: usize) -> Option<(usize, usize)> {
        if addr >= GICR && addr < GICR + 8 * STRIDE {
            Some(((addr - GICR) / STRIDE, (addr - GICR) % STRIDE))
        } else {
            None
        }
    }
    fn read(&mut self, addr: usize) -> u32 {
        let v = if addr == GICD + gicv3::gicd::CTLR {
            let busy = if self.rwp_left > 0 {
                self.rwp_left -= 1;
                gicv3::gicd::CTLR_RWP
            } else {
                0
            };
            self.regs.get(&addr).copied().unwrap_or(0) | busy
        } else if addr == GICD + gicv3::gicd::TYPER {
            self.cfg.it_lines
        } else if let Some((i, off)) = Self::rd_index(addr) {
            match off {
                o if o == gicv3::gicr::PWRR => {
                    if self.trans_left[i] > 0 {
                        self.trans_left[i] -= 1;
                        gicv3::gicr::PWRR_RDGPD
                    } else if self.regs.contains_key(&addr) && self.rdpd_left[i] > 0 {
                        self.rdpd_left[i] -= 1;
                        gicv3::gicr::PWRR_RDPD
                    } else if self.regs.contains_key(&addr) {
                        0
                    } else {
                        gicv3::gicr::PWRR_RDPD
                    }
                }
                o if o == gicv3::gicr::TYPER => {
                    if i == self.cfg.last {
                        gicv3::gicr::TYPER_LAST as u32
                    } else {
                        0
                    }
                }
                o if o == gicv3::gicr::TYPER + 4 => self.cfg.aff[i],
                o if o == gicv3::gicr::WAKER => {
                    let base = self.regs.get(&addr).copied().unwrap_or(
                        gicv3::gicr::WAKER_PROCESSOR_SLEEP | gicv3::gicr::WAKER_CHILDREN_ASLEEP,
                    );
                    if base & gicv3::gicr::WAKER_PROCESSOR_SLEEP == 0 {
                        if self.waker_left > 0 {
                            self.waker_left -= 1;
                            base | gicv3::gicr::WAKER_CHILDREN_ASLEEP
                        } else {
                            base & !gicv3::gicr::WAKER_CHILDREN_ASLEEP
                        }
                    } else {
                        base
                    }
                }
                _ => self.regs.get(&addr).copied().unwrap_or(0),
            }
        } else {
            self.regs.get(&addr).copied().unwrap_or(0)
        };
        self.trace.push(Ev::R(addr, v));
        v
    }
    fn write(&mut self, addr: usize, v: u32) {
        self.trace.push(Ev::W(addr, v));
        if addr == GICD + gicv3::gicd::CTLR {
            self.rwp_left = self.cfg.rwp_delay;
        }
        if let Some((_, off)) = Self::rd_index(addr) {
            if off == gicv3::gicr::WAKER {
                self.waker_left = self.cfg.waker_delay;
            }
        }
        self.regs.insert(addr, v);
    }
}

thread_local! { static MODEL: RefCell<Model> = RefCell::new(Model::default()); }

// The board's util::mmio and asm replacements, on the model.
pub mod mmio {
    pub fn read32(a: usize) -> u32 {
        super::MODEL.with(|m| m.borrow_mut().read(a))
    }
    pub fn write32(a: usize, v: u32) {
        super::MODEL.with(|m| m.borrow_mut().write(a, v))
    }
    pub fn read64le(a: usize) -> u64 {
        let l = read32(a) as u64;
        let h = read32(a + 4) as u64;
        (h << 32) | l
    }
}
fn icc_sre(v: u64) {
    MODEL.with(|m| m.borrow_mut().trace.push(Ev::Sre(v)))
}
fn icc_ctlr(v: u64) {
    MODEL.with(|m| m.borrow_mut().trace.push(Ev::Ctlr(v)))
}
fn icc_igrpen1(v: u64) {
    MODEL.with(|m| m.borrow_mut().trace.push(Ev::Igrpen1(v)))
}

#[path = "legacy/a7a_gic.rs"]
mod legacy;

struct Io;
impl RegIo for Io {
    fn read32(&mut self, a: usize) -> u32 {
        mmio::read32(a)
    }
    fn write32(&mut self, a: usize, v: u32) {
        mmio::write32(a, v)
    }
    fn read16(&mut self, _: usize) -> u16 {
        unreachable!()
    }
    fn write16(&mut self, _: usize, _: u16) {
        unreachable!()
    }
    fn udelay(&mut self, _: u32) {}
    fn now_us(&mut self) -> u64 {
        0
    }
    fn log(&mut self, _: core::fmt::Arguments) {}
}
struct Icc;
impl IccEl3 for Icc {
    fn write_sre(&mut self, v: u64) {
        icc_sre(v)
    }
    fn write_ctlr(&mut self, v: u64) {
        icc_ctlr(v)
    }
    fn write_igrpen1(&mut self, v: u64) {
        icc_igrpen1(v)
    }
}

const G: Gic = Gic {
    gicd: GICD,
    gicr: GICR,
    gicr_stride: STRIDE,
    max_cpus: 8,
};

#[derive(Debug, PartialEq, Eq)]
struct Outcome {
    irqs: u32,
    cpu: Result<usize, ()>,
    trace: Vec<Ev>,
}

fn run_legacy(cfg: &Cfg, mpidr: u64, core: usize) -> Outcome {
    MODEL.with(|m| *m.borrow_mut() = Model::new(cfg.clone()));
    let irqs = legacy::init_distributor();
    let cpu = legacy::init_cpu(mpidr, core);
    if let Ok(rd) = cpu {
        legacy::cpu_off(rd);
    }
    let trace = MODEL.with(|m| std::mem::take(&mut m.borrow_mut().trace));
    Outcome { irqs, cpu, trace }
}

fn run_crate(cfg: &Cfg, mpidr: u64, core: usize) -> Outcome {
    MODEL.with(|m| *m.borrow_mut() = Model::new(cfg.clone()));
    let irqs = gicv3::init_distributor(&mut Io, &G).expect("model always settles");
    let cpu = match gicv3::init_cpu(&mut Io, &mut Icc, &G, mpidr, core) {
        Ok(rd) => Ok(rd),
        Err(GicError::NoRedistributor { mpidr: m }) => {
            assert_eq!(m, mpidr);
            Err(())
        }
        Err(e) => panic!("{e:?}"),
    };
    if let Ok(rd) = cpu {
        gicv3::cpu_off(&mut Io, &mut Icc, rd);
    }
    let trace = MODEL.with(|m| std::mem::take(&mut m.borrow_mut().trace));
    Outcome { irqs, cpu, trace }
}

struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }
    fn below(&mut self, n: u64) -> u64 {
        self.next() % n
    }
}

#[test]
fn boot_sequence_matches_the_board_on_seeded_configurations() {
    let mut r = Rng(0x9e37_79b9_7f4a_7c15);
    let mut found = 0;
    let mut scanned = 0;
    let mut missing = 0;
    for _ in 0..3000 {
        let mut cfg = Cfg {
            it_lines: r.below(32) as u32,
            rwp_delay: r.below(4) as u32,
            last: r.below(8) as usize,
            waker_delay: r.below(4) as u32,
            ..Default::default()
        };
        // Affinities: identity (A733: Aff1 = core) or a permutation.
        let mut order: Vec<u32> = (0..8).collect();
        if r.below(2) == 1 {
            for i in (1..8).rev() {
                let j = r.below(i as u64 + 1) as usize;
                order.swap(i, j);
            }
        }
        for i in 0..8 {
            cfg.transition[i] = r.below(3) as u32;
            cfg.rdpd_retries[i] = r.below(3) as u32;
            cfg.aff[i] = order[i] << 8;
        }
        let core = r.below(8) as usize;
        let mut mpidr = (core as u64) << 8;
        if r.below(10) == 0 {
            mpidr |= 1 << 16; // an Aff2 no redistributor reports
        }
        let a = run_legacy(&cfg, mpidr, core);
        let b = run_crate(&cfg, mpidr, core);
        assert_eq!(a, b, "core {core} mpidr {mpidr:#x}");
        match a.cpu {
            Ok(rd) if rd == GICR + core * STRIDE => found += 1,
            Ok(_) => scanned += 1,
            Err(()) => missing += 1,
        }
    }
    assert!(
        found > 0 && scanned > 0 && missing > 0,
        "{found} {scanned} {missing}"
    );
}

#[test]
fn affinity_encoding_matches_the_board() {
    let mut r = Rng(0x1234_5678);
    for _ in 0..100_000 {
        let m = r.next();
        let board = (((m >> 32) & 0xff) << 24 | (m & 0x00ff_ffff)) as u32;
        assert_eq!(gicv3::mpidr_to_gicr_affinity(m), board);
    }
}
