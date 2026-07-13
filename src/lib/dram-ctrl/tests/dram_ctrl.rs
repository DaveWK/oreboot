// SPDX-License-Identifier: GPL-2.0-only
//! dram-ctrl against the A733 controller code (verbatim copies kept in
//! crates/dwc-ddrctl/tests/legacy, ore-edk-boot-radxa-a7a ed42550) and its
//! own invariants for every rank/density combination.
#![allow(dead_code, unused_imports, clippy::all)]

#[path = "../../dwc-ddrctl/tests/legacy/ctrl.rs"]
mod ctrl;
#[path = "../../dwc-ddrctl/tests/legacy/hw.rs"]
mod hw;
#[path = "../../dwc-ddrctl/tests/legacy/mr.rs"]
mod mr;
#[path = "../../dwc-ddrctl/tests/legacy/para.rs"]
mod para;
#[path = "../../dwc-ddrctl/tests/legacy/post.rs"]
mod post;
#[path = "../../dwc-ddrctl/tests/legacy/timing.rs"]
mod timing;
mod phy {
    use crate::hw::Hw;
    pub fn dmem_access<H: Hw>(_: &mut H, _: bool) {}
    pub fn mb_byte<H: Hw>(_: &mut H, _: usize) -> u32 {
        0
    }
    pub fn read_latency_csrs<H: Hw>(_: &mut H) -> (u32, u32) {
        (0, 0)
    }
}

use dram_ctrl::hif::{active_ranks_mask, Bit, Layout};
use dram_ctrl::pack::{pack, Entry, PackError, Src};
use dram_ctrl::steps::{run, MrPort, NoMr, Step, StepError};
use dram_ctrl::values::{Name, Values};
use jedec_lpddr::geometry::{non_binary_mib, size_mib, Geometry, Topology};
use regtrace::{compare, CompareOpts, Ev, Recorder};

/// The legacy Hw on a regtrace Recorder, answering the controller polls.
struct L(Recorder);
impl L {
    fn new() -> L {
        L(polling_recorder())
    }
}
fn polling_recorder() -> Recorder {
    Recorder {
        responder: Some(Box::new(|a, regs| {
            for ch in 0..2 {
                if a == ctrl::ddrc(ch, ctrl::SWSTAT) as u64 {
                    return Some(
                        regs.get(&(ctrl::ddrc(ch, ctrl::SWCTL) as u64))
                            .copied()
                            .unwrap_or(0)
                            & 1,
                    );
                }
                if a == ctrl::ddrc(ch, ctrl::DFISTAT) as u64
                    || a == ctrl::ddrc(ch, ctrl::STAT) as u64
                {
                    return Some(1);
                }
            }
            None
        })),
        ..Recorder::default()
    }
}
impl hw::Hw for L {
    fn read32(&mut self, a: usize) -> u32 {
        regio::RegIo::read32(&mut self.0, a)
    }
    fn write32(&mut self, a: usize, v: u32) {
        regio::RegIo::write32(&mut self.0, a, v)
    }
    fn read16(&mut self, a: usize) -> u16 {
        regio::RegIo::read16(&mut self.0, a)
    }
    fn write16(&mut self, a: usize, v: u16) {
        regio::RegIo::write16(&mut self.0, a, v)
    }
    fn udelay(&mut self, us: u32) {
        regio::RegIo::udelay(&mut self.0, us)
    }
    fn now_us(&mut self) -> u64 {
        regio::RegIo::now_us(&mut self.0)
    }
    fn set_voltage(&mut self, _: hw::Rail, _: u32) -> bool {
        true
    }
    fn log(&mut self, _: core::fmt::Arguments) {}
}

/// The A733 board's DRAMSET1TMG0/1/2/4 values on the crate's names: the
/// formulas of its timing.rs, from the JEDEC LPDDR5 values it uses.
fn a733_values(mhz: u32, long_rl: bool) -> Values {
    let t = |n| timing::ns(n, mhz);
    let faw = t(20);
    let rrd = t(5).max(2);
    let rcd = t(18).max(2);
    let xp = t(7).max(3);
    let wtr = t(if long_rl { 14 } else { 12 }).max(4);
    let rtp = t(8).max(2);
    let rp = t(21).max(2);
    let ras = t(42).max(3);
    let refi_x32 = t(3906) >> 5;
    let rl = if long_rl { 14 } else { 13 };
    let wl_term = if long_rl { 22 } else { 21 };
    let mut v = Values::default();
    v.set(Name::Ras, ras)
        .set(Name::RasMax, (refi_x32 * 9) >> 5)
        .set(Name::Faw, faw)
        .set(Name::Wr2Pre, wl_term + 10)
        .set(Name::Rc, ras + rp)
        .set(Name::Rd2Pre, rtp)
        .set(Name::Xp, xp)
        .set(Name::Wr2Rd, wtr + 11)
        .set(Name::Rd2Wr, rl + 3)
        .set(Name::ReadLatency, rl)
        .set(Name::WriteLatency, 7)
        .set(Name::Rp, rp)
        .set(Name::Rrd, rrd)
        .set(Name::Ccd, 4)
        .set(Name::Rcd, rcd);
    v
}

#[test]
fn dwc_table_packs_what_the_a733_timing_writer_writes() {
    // The A733 controller clocks (dram_clk 600 MHz and below, per pstate
    // divided by tpr2): every value fits its field there.
    for clk in [100u32, 200, 300, 400, 466, 533, 600] {
        for long in [false, true] {
            let mut words = [0u32; 32];
            words[0] = clk;
            words[1] = para::TYPE_LPDDR5;
            words[24] = 0x0302_0100;
            words[27] = (long as u32) << 31;
            words[30] = 5;
            let mut p = para::Para::from_sysconfig(&words);
            let mut a = L::new();
            timing::set_channel_timing(&mut a, &mut p, 0);
            for ps in 0..4usize {
                let mhz = p.clk() / (((p.tpr2() >> (8 * ps)) & 0x1f) + 1);
                let packed = pack(&dram_ctrl::dwc::DRAMSET1TMG, &a733_values(mhz, long)).unwrap();
                for &(off, v) in packed.as_slice() {
                    assert_eq!(
                        a.0.regs.get(&(ctrl::freq(0, ps, off) as u64)).copied(),
                        Some(v),
                        "clk {clk} long {long} pstate {ps} reg {off:#x}"
                    );
                }
            }
        }
    }
}

#[test]
fn an_overflowing_timing_is_refused_not_spilled() {
    // At a 1066 MHz controller clock tRC (42 + 21 ns) is 68 cycles: fits.
    // At 4266 MHz it would be 270 and spill into RD2PRE in a plain OR.
    let v = a733_values(4266, false);
    assert!(matches!(
        pack(&dram_ctrl::dwc::DRAMSET1TMG, &v),
        Err(PackError::Overflow {
            reg: 4,
            value: 270,
            ..
        })
    ));
    assert!(pack(&dram_ctrl::dwc::DRAMSET1TMG, &a733_values(1066, false)).is_ok());
}

#[test]
fn pack_refuses_missing_and_overflowing_values() {
    let t = [Entry {
        reg: 4,
        field: dwc_ddrctl::timing::Field::new(8, 4),
        src: Src::Value(Name::Rp),
    }];
    assert_eq!(
        pack(&t, &Values::default()),
        Err(PackError::Missing {
            reg: 4,
            name: Name::Rp
        })
    );
    let mut v = Values::default();
    v.set(Name::Rp, 16);
    assert!(matches!(
        pack(&t, &v),
        Err(PackError::Overflow {
            reg: 4,
            value: 16,
            ..
        })
    ));
    v.set(Name::Rp, 15);
    assert_eq!(pack(&t, &v).unwrap().get(4), Some(0xf00));
    let c = [Entry {
        reg: 0,
        field: dwc_ddrctl::timing::Field::new(24, 8),
        src: Src::Const(7),
    }];
    assert_eq!(pack(&c, &v).unwrap().get(0), Some(7 << 24));
}

#[test]
fn from_lp4_rounds_jedec_cycles_up_to_controller_cycles() {
    use jedec_lpddr::lp4::timing::MT53E256M16D1_1866 as T;
    for ck in [200u32, 533, 800, 933, 1066] {
        for ratio in [1u32, 2, 4] {
            let v = Values::from_lp4(&T, ck, ratio).unwrap();
            let c = |m: jedec_lpddr::lp4::timing::Min| m.cycles(ck).div_ceil(ratio);
            assert_eq!(v.get(Name::Rp), Some(c(T.t_rp)));
            assert_eq!(v.get(Name::Rcd), Some(c(T.t_rcd)));
            assert_eq!(v.get(Name::Rc), Some(c(T.t_ras) + c(T.t_rp)));
            assert_eq!(v.get(Name::Faw), Some(c(T.t_faw)));
            assert_eq!(v.get(Name::Refi), Some(T.t_refi.cycles(ck) / ratio));
            // Never shorter than the JEDEC time.
            let ctrl_mhz = ck as f64 / ratio as f64;
            assert!(
                v.get(Name::Rcd).unwrap() as f64 / ctrl_mhz * 1000.0
                    >= T.t_rcd.ps as f64 / 1000.0 - 1e-9
            );
            assert_eq!(v.get(Name::Wr2Pre), None, "latencies are the board's");
        }
    }
    assert!(Values::from_lp4(&T, 933, 3).is_none());
}

/// The A733 DFI initialisation of one channel as a step list.
fn a733_dfi_steps() -> Vec<Step> {
    let d = |off| ctrl::ddrc(0, off);
    let t = 100_000;
    vec![
        Step::Write {
            addr: d(ctrl::SWCTL),
            val: 0,
        },
        Step::Modify {
            addr: d(ctrl::DFIMISC),
            clear: 0,
            set: 0x1001,
        },
        Step::Modify {
            addr: d(ctrl::DFIMISC),
            clear: 0,
            set: ctrl::DFI_INIT_START,
        },
        Step::Write {
            addr: d(ctrl::SWCTL),
            val: 1,
        },
        Step::Poll {
            addr: d(ctrl::SWSTAT),
            mask: 1,
            want: 1,
            timeout_us: t,
            what: "SWSTAT.sw_done_ack",
        },
        Step::Poll {
            addr: d(ctrl::DFISTAT),
            mask: 1,
            want: 1,
            timeout_us: t,
            what: "DFISTAT.dfi_init_complete",
        },
        Step::Delay { us: 50 },
        Step::Modify {
            addr: 0x0701_0250,
            clear: 0,
            set: 1,
        },
        Step::Delay { us: 2000 },
        Step::Note("leave self-refresh"),
        Step::Write {
            addr: d(ctrl::SWCTL),
            val: 0,
        },
        Step::Modify {
            addr: d(ctrl::DFIMISC),
            clear: ctrl::DFI_INIT_START,
            set: 0,
        },
        Step::Modify {
            addr: d(ctrl::PWRCTL),
            clear: 0x800,
            set: 0,
        },
        Step::Poll {
            addr: d(ctrl::STAT),
            mask: 0x30,
            want: 0,
            timeout_us: t,
            what: "STAT.selfref_type",
        },
        Step::Poll {
            addr: d(ctrl::STAT),
            mask: 3,
            want: 1,
            timeout_us: t,
            what: "STAT.operating_mode",
        },
        Step::Delay { us: 50 },
        Step::Modify {
            addr: d(ctrl::DFIMISC),
            clear: ctrl::DFI_INIT_COMPLETE_EN,
            set: 0,
        },
        Step::Write {
            addr: d(ctrl::SWCTL),
            val: 1,
        },
        Step::Poll {
            addr: d(ctrl::SWSTAT),
            mask: 1,
            want: 1,
            timeout_us: t,
            what: "SWSTAT.sw_done_ack",
        },
        Step::Delay { us: 2 },
    ]
}

#[test]
fn step_list_reproduces_the_a733_dfi_init() {
    let mut a = L::new();
    post::dfi_init(&mut a, 1).unwrap();
    let mut b = polling_recorder();
    assert_eq!(run(&mut b, &a733_dfi_steps(), &mut NoMr, true), Ok(20));
    // Same accesses and delays; the runner's log lines are not compared.
    assert_eq!(
        compare(
            &a.0.evs,
            &b.evs,
            &CompareOpts {
                fold_polls: false,
                ..CompareOpts::default()
            }
        ),
        None
    );
    assert!(b
        .evs
        .iter()
        .any(|e| matches!(e, Ev::Print(p) if p.contains("leave self-refresh"))));
}

#[test]
fn step_runner_names_the_failing_step() {
    let mut io = Recorder {
        tick_us: 10,
        ..Recorder::default()
    };
    let steps = [
        Step::Write { addr: 0x10, val: 1 },
        Step::Poll {
            addr: 0x14,
            mask: 1,
            want: 1,
            timeout_us: 100,
            what: "ready",
        },
        Step::Write { addr: 0x18, val: 2 },
    ];
    assert_eq!(
        run(&mut io, &steps, &mut NoMr, false),
        Err(StepError {
            index: 1,
            what: "ready"
        })
    );
    assert!(!io.regs.contains_key(&0x18), "nothing after the failure");
    struct Mr(Vec<(u32, u32, u32)>);
    impl<H: regio::RegIo + ?Sized> MrPort<H> for Mr {
        fn mrw(&mut self, _: &mut H, rank: u32, mr: u32, val: u32) -> Result<(), &'static str> {
            self.0.push((rank, mr, val));
            if mr == 99 {
                Err("MR timeout")
            } else {
                Ok(())
            }
        }
    }
    let mut m = Mr(vec![]);
    let s = [
        Step::Mrw {
            rank: 1,
            mr: 13,
            val: 0x40,
        },
        Step::Mrw {
            rank: 0,
            mr: 99,
            val: 0,
        },
    ];
    assert_eq!(
        run(&mut io, &s, &mut m, false),
        Err(StepError {
            index: 1,
            what: "MR timeout"
        })
    );
    assert_eq!(m.0, vec![(1, 13, 0x40), (0, 99, 0)]);
    assert_eq!(
        run(&mut io, &s[..1], &mut NoMr, false),
        Err(StepError {
            index: 0,
            what: "no mode-register path"
        })
    );
}

/// LPDDR4 x16 channel geometries per JEDEC LPDDR4/4X density (rows; 10 columns, 8
/// banks), binary and the 3/4-row non-binary ones.
const LP4: [(u32, u32, bool); 7] = [
    (256, 14, false),
    (384, 15, true),
    (512, 15, false),
    (768, 16, true),
    (1024, 16, false),
    (1536, 17, true),
    (2048, 17, false),
];

#[test]
fn layouts_for_every_rank_and_density() {
    let mut n = 0;
    for (mib, rows, non_binary) in LP4 {
        for ranks in [1u8, 2] {
            for channels in [1u8, 2, 4] {
                let topo = Topology::new(ranks, channels, mib).unwrap();
                let g = Geometry {
                    row_bits: rows,
                    col_bits: 10,
                    bank_bits: 3,
                    bg_bits: 0,
                    rank_bits: topo.rank_bits(),
                };
                let l = Layout::new(&g).unwrap();
                // Every address bit once, packed from HIF bit 0.
                assert_eq!(l.bits().len() as u32, g.address_bits());
                for (i, b) in l.bits().iter().enumerate() {
                    assert_eq!(l.hif(*b), Some(i as u32));
                }
                assert_eq!(l.hif(Bit::Col(0)), Some(0));
                assert_eq!(l.hif(Bit::Col(3)), Some(3));
                assert_eq!(l.hif(Bit::Bank(0)), Some(10));
                assert_eq!(l.hif(Bit::Row(0)), Some(13));
                // The rank bit sits above all of one rank's bits.
                assert_eq!(l.rank_hif(), (ranks == 2).then_some(rows + 13));
                // The layout's size is the topology's (x16 per channel).
                let bus_log2 = 1 + (channels as u32).trailing_zeros();
                let size = size_mib(g.address_bits(), bus_log2).unwrap();
                let size = if non_binary {
                    non_binary_mib(size).unwrap()
                } else {
                    size
                };
                assert_eq!(
                    Some(size),
                    topo.total_mib(),
                    "{mib} MiB x{ranks} ranks x{channels} ch"
                );
                assert_eq!(active_ranks_mask(ranks), if ranks == 2 { 3 } else { 1 });
                n += 1;
            }
        }
    }
    assert_eq!(n, 7 * 2 * 3);
    let g = Geometry {
        row_bits: 16,
        col_bits: 10,
        bank_bits: 2,
        bg_bits: 2,
        rank_bits: 1,
    };
    let l = Layout::new(&g).unwrap();
    assert_eq!(
        (
            l.hif(Bit::BankGroup(0)),
            l.hif(Bit::Col(3)),
            l.hif(Bit::Bank(0))
        ),
        (Some(3), Some(5), Some(12))
    );
    assert!(Layout::new(&Geometry { col_bits: 2, ..g }).is_none());
}
