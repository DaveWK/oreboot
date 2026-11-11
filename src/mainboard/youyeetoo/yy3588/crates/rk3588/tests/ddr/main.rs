// SPDX-License-Identifier: GPL-2.0-only
//! Host tests of crate::ddr: golden sequences of the init phases written
//! down from the TRM text (independent of the code), ordering rules of the
//! later phases, the 4-channel flow against a behavioural model, detection
//! logic, timing against jedec-lpddr, and a regtrace snapshot of one
//! channel's complete register program.

mod hw;

use hw::{Hw, Part};
use jedec_lpddr::lp4::timing as jt;
use regtrace::{CompareOpts, Ev};
use rk3588::ddr::{self, detect, fallback, seq, timing, train, Config, Error, Known, YY3588_PARTS};
use rk3588::msch::{self, Layout};
use rk3588::osreg;
use rk3588::regs::{
    base, ctl, ddr_cru, phy, wm, CENTER_GRF_SOC_CON4, OS_REG2, OS_REG3, OS_REG4, OS_REG5,
};

/// x16, 8 Gb per channel (1 GiB per channel and rank).
const MR8_8GB: u8 = 0x10;
/// x16, 16 Gb per channel.
const MR8_16GB: u8 = 0x18;

fn part(ranks: u8, mr8: u8) -> Part {
    Part {
        ranks,
        mr5: 0x01,
        mr8,
    }
}

/// The probe-pass configuration (the parts of these tests vary).
fn cfg() -> Config {
    Config {
        rails: false,
        ..Config::yy3588_probe()
    }
}

/// The YY3588 as identified on hardware: x8 byte-mode dies, MR8 0x58,
/// two ranks, Micron (MR5 0xff).
fn yy3588_part() -> Part {
    Part {
        ranks: 2,
        mr5: 0xff,
        mr8: 0x58,
    }
}

fn writes_and_delays(evs: &[Ev]) -> Vec<Ev> {
    evs.iter()
        .filter(|e| !matches!(e, Ev::R(..)))
        .cloned()
        .collect()
}

// ---------------------------------------------------------------- golden

/// TRM P2 §2.6.1 steps 3-13 for channel 1, written down from the TRM and
/// the P1 CRU/GRF field tables, not from the code.
#[test]
fn golden_reset_and_clock_ch1() {
    let mut hw = Hw::new([part(1, MR8_8GB); 4]);
    seq::reset_and_clock(&mut hw, 1);
    let cru = 0xFD80_4000u64; // DDR1CRU
    let main = 0xFD7C_0A54u64; // CRU_SOFTRST_CON21
    let grf = 0xFD59_C134u64; // DDR01_GRF CHB_PHY_CON0
    let phyb = 0xFE0D_0000u64;
    let w = |a, v| Ev::W(32, a, v);
    let expect = vec![
        w(cru + 0x300, 0x0001_0000), // clk_ddrphy2x_ch1_sel = D1APLL (24 MHz)
        w(cru + 0x800, 0x0038_0000), // PHY/CRU pclk, osc clock ungated
        w(cru + 0xA00, 0x0016_0016), // presetn_ddrphy, resetn_ddrphy2x, div_rst_n asserted
        w(main, 0x011D_011D),        // presetn/resetn_dfi/sbr/core/aresetn upctl_ch1 asserted
        w(grf, 0x0008_0000),         // ddrphy2xclkgate_enable = 0
        Ev::Delay(1),
        w(cru + 0xA00, 0x0010_0000), // 3. PRESETn of DDRPHY released
        w(main, 0x0001_0000),        // 3. presetn of DDRCTL released
        w(cru + 0xA00, 0x0004_0000), // 4. RESETn released
        Ev::Delay(1),
        w(grf, 0x0008_0008), // 5. clock gate enable
        Ev::Delay(1),
        w(cru + 0xA00, 0x0002_0000), // 6. div_rst_n released
        Ev::Delay(1),
        w(main, 0x0118_0118), // 7. sbr, core, aresetn asserted
        w(main, 0x0004_0000), // 8. rst_n (resetn_dfi_ch1) released
        Ev::Delay(1),
        w(grf, 0x0008_0000),         // 9.
        w(cru + 0xA00, 0x0002_0002), // 10.
        w(phyb, 0x4400_5011),        // 11. ctrl_ddr_mode = 0 (already)
        w(phyb + 0xA94, 0x9),        // 11. ctrl_phy_clk_2x = 1, ctrl_phy_mode = 1
        w(cru + 0xA00, 0x0002_0000), // 12.
        w(grf, 0x0008_0008),         // 13.
        Ev::Delay(1),
    ];
    assert_eq!(writes_and_delays(&hw.evs), expect);
}

/// Step 29 (frequency change) for channel 2 with the self-refresh guard.
#[test]
fn golden_freq_change_ch2() {
    let mut hw = Hw::new([part(1, MR8_8GB); 4]);
    // Pretend the channel is in normal mode after step 25.
    let b = base::DDRCTL[2];
    hw.regs.insert(b + ctl::DFIMISC, 1);
    // Drive the model into "inited" through a STAT read.
    use regio::RegIo;
    assert_eq!(hw.read32(b + ctl::STAT) & 7, 1);
    hw.evs.clear();
    seq::freq_change(&mut hw, 2, true, true).unwrap();
    let ws: Vec<(usize, u32)> = hw.writes();
    let cru = base::DDR_CRU[2];
    let grf = base::DDR23_GRF + 0x130;
    let pos = |a: usize, v: u32| {
        ws.iter()
            .position(|&(x, y)| x == a && y == v)
            .unwrap_or(usize::MAX)
    };
    let sr_in = pos(b + ctl::PWRCTL, ctl::PWRCTL_SELFREF_SW);
    let start = pos(b + ctl::DFIMISC, ctl::DFIMISC_INIT_START);
    let gate_off = pos(grf, 0x0008_0000);
    let div_on = pos(cru + ddr_cru::SOFTRST_CON00, 0x0002_0002);
    let sel = pos(cru + ddr_cru::CLKSEL_CON00, 0x0001_0001);
    let div_off = pos(cru + ddr_cru::SOFTRST_CON00, 0x0002_0000);
    let gate_on = pos(grf, 0x0008_0008);
    let stop = pos(b + ctl::DFIMISC, 0);
    let en = ws
        .iter()
        .rposition(|&(x, y)| x == b + ctl::DFIMISC && y == 1)
        .unwrap();
    let sr_out = ws
        .iter()
        .rposition(|&(x, y)| x == b + ctl::PWRCTL && y == 0)
        .unwrap();
    let order = [
        sr_in, start, gate_off, div_on, sel, div_off, gate_on, stop, en, sr_out,
    ];
    assert!(order.windows(2).all(|p| p[0] < p[1]), "{order:?}");
}

// --------------------------------------------------------- ordering rules

fn one_channel(parts: [Part; 4], ch: usize, c: &Config) -> Hw {
    let mut hw = Hw::new(parts);
    let g = detect::geometry(parts[ch].mr8).unwrap();
    let p = timing::plan(c, parts[ch].ranks, g.density_gb);
    seq::init_channel(&mut hw, ch, &p, &g, c).unwrap();
    hw
}

#[test]
fn controller_registers_before_core_reset_release() {
    let hw = one_channel([part(2, MR8_8GB); 4], 0, &cfg());
    let b = base::DDRCTL[0];
    let release = hw
        .find_write(0, |a, v| a == base::CRU + 0xA50 && v == 0x1180_0000)
        .unwrap();
    for off in [
        ctl::MSTR0,
        ctl::INITTMG0,
        ctl::ODTMAP,
        ctl::ADDRMAP_BASE + 4,
        ctl::DFITMG0,
        ctl::INITMR0,
    ] {
        let i = hw.find_write(0, |a, _| a == b + off).unwrap();
        assert!(i < release, "{off:#x} written after the core reset release");
    }
    // ...and after the APB reset release (step 3).
    let apb = hw
        .find_write(0, |a, v| a == base::CRU + 0xA50 && v == 0x0008_0000)
        .unwrap();
    assert!(hw.find_write(0, |a, _| a == b + ctl::MSTR0).unwrap() > apb);
    // Two ranks: MSTR0 active_ranks = 3, ADDRMAP1 has a CS bit (not 0x3f).
    assert_eq!(hw.get(b + ctl::MSTR0), 0x0308_0002);
    assert_ne!(hw.get(b + ctl::ADDRMAP_BASE + 4) & 0x3f, 0x3f);
}

#[test]
fn dfi_init_and_dll_order() {
    let hw = one_channel([part(1, MR8_8GB); 4], 3, &cfg());
    let b = base::DDRCTL[3];
    let pb = base::DDRPHY[3];
    let dll_off = hw
        .find_write(0, |a, v| {
            a == pb + phy::MDLL_CON0 && v & phy::MDLL_DLL_ON == 0
        })
        .unwrap();
    let start = hw
        .find_write(0, |a, v| {
            a == b + ctl::DFIMISC && v & ctl::DFIMISC_INIT_START != 0
        })
        .unwrap();
    let zq = hw
        .find_write(0, |a, v| {
            a == pb + phy::ZQ_CON0 && v & phy::ZQ_MANUAL_STR != 0
        })
        .unwrap();
    let sel = hw
        .find_write(0, |a, v| {
            a == base::DDR_CRU[3] + ddr_cru::CLKSEL_CON00 && v == 0x0001_0001
        })
        .unwrap();
    let dll_on = hw
        .find_write(sel, |a, v| {
            a == pb + phy::MDLL_CON0 && v & phy::MDLL_DLL_ON != 0
        })
        .unwrap();
    let sched = hw
        .find_write(0, |a, v| {
            a == pb + phy::LP_CON0 && v & phy::LP_SCHEDULER_EN != 0
        })
        .unwrap();
    assert!(dll_off < start && start < zq && zq < sel && sel < dll_on && dll_on < sched);
    // Every DFIMISC write is framed by SWCTL 0 ... 1.
    for (i, e) in hw.evs.iter().enumerate() {
        if let Ev::W(_, a, _) = e {
            if *a as usize == b + ctl::DFIMISC {
                let last_swctl = hw.evs[..i].iter().rev().find_map(|e| match e {
                    Ev::W(_, a, v) if *a as usize == b + ctl::SWCTL => Some(*v),
                    _ => None,
                });
                assert_eq!(
                    last_swctl,
                    Some(0),
                    "DFIMISC write {i} outside a quasi-dynamic frame"
                );
            }
        }
    }
    // MR11/12/14/22 written once for the single rank, after the DLL lock.
    let mrs: Vec<u8> = hw.mrw.iter().map(|m| m.2).collect();
    assert_eq!(mrs, [11, 12, 14, 22]);
    // RK3576-RFC guard: dfi_init_complete held from the GRF around the DLL.
    let grf = base::DDR23_GRF + 0x134;
    let hold = hw
        .find_write(0, |a, v| a == grf && v == wm(4, 1, 1))
        .unwrap();
    let rel = hw
        .find_write(hold, |a, v| a == grf && v == wm(4, 1, 0))
        .unwrap();
    assert!(hold < dll_on && dll_on < rel);
}

#[test]
fn phy_scheduler_gets_dram_latencies_and_mr_opcodes() {
    let c = cfg();
    let hw = one_channel([part(1, MR8_8GB); 4], 0, &c);
    let pb = base::DDRPHY[0];
    let p = timing::plan(&c, 1, 8);
    assert_eq!(hw.get(pb + phy::SCHD_TIME_CON0) & 0x3f, 10); // WL at 1056 MHz
    assert_eq!(hw.get(pb + phy::GNR_CON0) & 0x3f, 20); // RL
    let op0 = hw.get(pb + phy::LP4_MR_OP0);
    assert_eq!(op0 & 0xff, p.mr1 as u32);
    assert_eq!((op0 >> 8) & 0xff, p.mr2 as u32);
    assert_eq!(op0 >> 24, c.mr12 as u32);
    // One rank: GATE_CON0.rank_en = 1.
    assert_eq!((hw.get(pb + phy::GATE_CON0) >> 24) & 3, 1);
}

#[test]
fn training_settings_and_launch() {
    let c = cfg();
    let mut hw = one_channel([part(2, MR8_8GB); 4], 1, &c);
    let p = timing::plan(&c, 2, 8);
    hw.evs.clear();
    train::prepare(&mut hw, 1, &p, &c);
    let r = train::run(&mut hw, 1, 3, &c).unwrap();
    assert!(r.ok());
    let pb = base::DDRPHY[1];
    assert_eq!(hw.get(pb + phy::CAL_RD_PATTERN_CON0), 0xEEEE_5555);
    assert_eq!(hw.get(pb + phy::DVFS_CON) & 0xfff, 1056);
    assert_ne!(hw.get(pb + phy::CAL_CON0) & phy::CAL_WRTRN_DBI_CAL_EN, 0);
    assert_eq!(hw.get(pb + phy::CAL_CON5) & 0x3ff, 0b11); // < 1066 MHz: low-freq cycle mode
    assert_eq!((hw.get(pb + phy::CAL_CON4) >> 6) & 0x1f, 4);
    let launches: Vec<u32> = hw
        .writes()
        .into_iter()
        .filter(|(a, _)| *a == pb + phy::SCHD_TRAIN_CON0)
        .map(|(_, v)| v)
        .collect();
    assert_eq!(launches, [0xff78, 0xff79, 0xff78]);
}

// ------------------------------------------------------- 4-channel flow

fn decode_osreg(hw: &Hw) -> (osreg::Pair, osreg::Pair) {
    (
        osreg::decode(hw.get(OS_REG2), hw.get(OS_REG3)),
        osreg::decode(hw.get(OS_REG4), hw.get(OS_REG5)),
    )
}

#[test]
fn four_channels_two_ranks_8gib() {
    let mut hw = Hw::new([part(2, MR8_8GB); 4]);
    let info = ddr::init(&mut hw, &cfg()).unwrap();
    assert_eq!(info.total_mib, 8192);
    for c in 0..4 {
        assert!(info.ch[c].present);
        assert_eq!(info.ch[c].ranks, 2);
        assert_eq!(info.ch[c].mr5, 0x01);
        // DDRSCH DeviceSize: 1 GiB ranks = 16 granules each.
        assert_eq!(hw.get(base::DDRSCH[c] + 0xC), 0x1010);
        // Each rank got MR11/12/14/22 in the real pass.
        let n = hw.mrw.iter().filter(|m| m.0 == c && m.2 == 14).count();
        assert_eq!(n, 4, "probe + real pass, 2 ranks each");
    }
    let (a, b) = decode_osreg(&hw);
    assert_eq!(a.size_mib() + b.size_mib(), 8192);
    assert_eq!(a.dram_type, osreg::LPDDR4X);
    assert!(ddr::handoff_present(&mut hw).is_some());
    // Interleave left for the OS; the linear layout was used for the tests.
    assert_eq!(hw.get(CENTER_GRF_SOC_CON4) & (0x1f << 4), 0);
    // 2 GiB per channel: linear code 0b00010 (P2 Table 1-5).
    assert!(hw
        .writes()
        .contains(&(CENTER_GRF_SOC_CON4, wm(4, 5, 0b00010))));
}

/// The YY3588 board with its default configuration: 4 x 8 GiB of x8
/// byte-mode dies, 32 GiB. The OS_REG words our init writes must equal
/// what the rkbin DDR init left on the first hardware run (identification
/// and handoff format, the test oracle; no training value is involved).
#[test]
fn yy3588_board_32gib() {
    let mut hw = Hw::new([yy3588_part(); 4]);
    let c = Config {
        rails: false,
        ..Config::yy3588_default()
    };
    assert_eq!(c.known, Some(YY3588_PARTS));
    assert_eq!(c.ck_mhz, 1056);
    let info = ddr::init(&mut hw, &c).unwrap();
    assert_eq!(info.total_mib, 32768);
    for ch in 0..4 {
        let i = info.ch[ch];
        assert!(i.present);
        assert_eq!((i.ranks, i.mr5, i.mr8), (2, 0xff, 0x58));
        let g = i.geometry.unwrap();
        assert_eq!(
            (
                g.row,
                g.col,
                g.bank_bits,
                g.byte_mode,
                g.rank_mib,
                g.density_gb
            ),
            (18, 10, 3, true, 4096, 16)
        );
        // DDRSCH DeviceSize: 4 GiB ranks = 64 granules each.
        assert_eq!(hw.get(base::DDRSCH[ch] + 0xC), 0x4040);
        // Both ranks active; R17 in ACT-2 (18 row bits).
        assert_eq!(hw.get(base::DDRCTL[ch] + ctl::MSTR0) >> 24 & 3, 3);
        assert_ne!(
            hw.get(base::DDRPHY[ch] + phy::CAL_CON4) & phy::CAL_CON4_LP4_HIGH_DENSITY,
            0
        );
        // tRFCab 380 ns (16 Gb per die channel) at 1056 MHz = 402 clocks.
        assert_eq!(hw.get(base::DDRCTL[ch] + ctl::RFSHSET1TMG1) & 0xfff, 402);
    }
    // No probe pass: one MSTR0 write per channel.
    let mstr = hw
        .writes()
        .iter()
        .filter(|(a, _)| *a == base::DDRCTL[0] + ctl::MSTR0)
        .count();
    assert_eq!(mstr, 1);
    assert_eq!(
        [
            hw.get(OS_REG2),
            hw.get(OS_REG3),
            hw.get(OS_REG4),
            hw.get(OS_REG5)
        ],
        [0x3a56_1a56, 0x3000_10f5, 0x3a56_1a56, 0x3000_10f5]
    );
    let (a, b) = decode_osreg(&hw);
    assert_eq!(a.size_mib() + b.size_mib(), 32768);
    assert!(ddr::handoff_present(&mut hw).is_some());
    // Per-channel tests through the 4 GiB linear layout (code 0b00001),
    // then the interleave for the OS.
    assert!(hw
        .writes()
        .contains(&(CENTER_GRF_SOC_CON4, wm(4, 5, 0b00001))));
    assert_eq!(hw.get(CENTER_GRF_SOC_CON4) & (0x1f << 4), 0);
    assert!(hw
        .text
        .contains("8192 MiB channels tested through 4096 MiB linear windows"));
    assert!(hw.text.contains("ddr ch3: mr5=0xff (Micron)"));
}

/// Known parts that are not what answers: refused after the real pass.
#[test]
fn known_parts_are_checked_by_mrr() {
    let mut hw = Hw::new([part(2, 0x50); 4]);
    let c = Config {
        rails: false,
        ..Config::yy3588_default()
    };
    assert_eq!(
        ddr::init(&mut hw, &c),
        Err(Error::Geometry { ch: 0, mr8: 0x50 })
    );
}

/// An 8 GiB channel that is only half fitted fails its per-channel test
/// in the 4 GiB window or the whole-memory test, never reaches the handoff.
#[test]
fn half_fitted_32gib_board_is_caught() {
    let mut hw = Hw::new([yy3588_part(); 4]);
    hw.dram_bytes /= 2;
    let c = Config {
        rails: false,
        fallback: false,
        ..Config::yy3588_default()
    };
    let r = ddr::init(&mut hw, &c);
    assert!(
        matches!(r, Err(Error::Memtest { .. }) | Err(Error::Alias { .. })),
        "{r:?}"
    );
    assert_eq!(hw.get(OS_REG2), 0);
}

#[test]
fn single_rank_detected_by_mrr() {
    let mut hw = Hw::new([part(1, MR8_16GB); 4]);
    let info = ddr::init(&mut hw, &cfg()).unwrap();
    assert_eq!(info.total_mib, 8192);
    assert!(info.ch.iter().all(|c| c.ranks == 1));
    assert_eq!(hw.get(base::DDRCTL[0] + ctl::MSTR0) >> 24 & 3, 1);
    assert_eq!(hw.get(base::DDRSCH[2] + 0xC), 32);
}

#[test]
fn rank1_rejected_when_gate_probe_fails() {
    // MR8 of rank 1 answers (e.g. a floating bus that happens to match)
    // but gate training on rank 1 fails: one rank.
    let mut hw = Hw::new([part(2, MR8_8GB); 4]);
    hw.gate_rank1 = Some(false);
    // The model's DRAM is sized for 2 ranks; the 1-rank result must still
    // pass the tests (smaller than fitted is safe).
    let info = ddr::init(&mut hw, &cfg()).unwrap();
    assert!(info.ch.iter().all(|c| c.ranks == 1));
    assert_eq!(info.total_mib, 4096);
}

#[test]
fn known_parts_skip_the_probe_pass() {
    let mut hw = Hw::new([part(2, MR8_8GB); 4]);
    let c = Config {
        known: Some(Known {
            ranks: 2,
            mr8: MR8_8GB,
        }),
        ..cfg()
    };
    ddr::init(&mut hw, &c).unwrap();
    let mstr = hw
        .writes()
        .iter()
        .filter(|(a, _)| *a == base::DDRCTL[0] + ctl::MSTR0)
        .count();
    assert_eq!(mstr, 1);
}

#[test]
fn dead_channel_cannot_interleave_but_can_run_linear() {
    let parts = [
        part(1, MR8_8GB),
        part(1, MR8_8GB),
        part(0, MR8_8GB),
        part(1, MR8_8GB),
    ];
    let mut hw = Hw::new(parts);
    assert_eq!(ddr::init(&mut hw, &cfg()), Err(Error::Asymmetric));
    let mut hw = Hw::new(parts);
    let c = Config {
        layout: Layout::Linear { mib: 1024 },
        ..cfg()
    };
    let info = ddr::init(&mut hw, &c).unwrap();
    assert!(!info.ch[2].present);
    assert_eq!(info.total_mib, 3072);
    // OS_REG4/5 then describe channel 3 alone.
    let (a, b) = decode_osreg(&hw);
    assert_eq!((a.channels, b.channels), (2, 1));
    assert_eq!(a.size_mib() + b.size_mib(), 3072);
}

#[test]
fn training_failure_is_an_error_without_fallback() {
    let mut hw = Hw::new([part(1, MR8_8GB); 4]);
    hw.train_fail[2] = 0x0100;
    let c = Config {
        fallback: false,
        ..cfg()
    };
    assert_eq!(
        ddr::init(&mut hw, &c),
        Err(Error::Training {
            ch: 2,
            fail: 0x0100
        })
    );
}

#[test]
fn dfi_latency_found_by_the_fallback_sweep() {
    // Reads are good only for RL - dfi_t_rddata_en in 3..=5: the default
    // (2) fails, the sweep must settle in the middle of the window (4).
    let mut hw = Hw::new([part(1, MR8_8GB); 4]);
    hw.good_rd_sub = Some((3, 5));
    let info = ddr::init(&mut hw, &cfg()).unwrap();
    for c in 0..4 {
        let d = info.ch[c].dfi.expect("swept");
        assert_eq!(d.rddata_en_sub, 4);
    }
}

#[test]
fn oversized_geometry_is_caught() {
    // The parts claim 16 Gb per channel, but only half is fitted: the
    // per-channel or the aliasing test must fail, never the handoff.
    let mut hw = Hw::new([part(1, MR8_16GB); 4]);
    hw.dram_bytes /= 2;
    let c = Config {
        fallback: false,
        ..cfg()
    };
    let r = ddr::init(&mut hw, &c);
    assert!(
        matches!(r, Err(Error::Memtest { .. }) | Err(Error::Alias { .. })),
        "{r:?}"
    );
}

#[test]
fn bad_frequency_and_3_4_density_refused() {
    let mut hw = Hw::new([part(1, MR8_8GB); 4]);
    assert_eq!(
        ddr::init(
            &mut hw,
            &Config {
                ck_mhz: 400,
                ..cfg()
            }
        ),
        Err(Error::BadFrequency)
    );
    assert_eq!(
        ddr::init(
            &mut hw,
            &Config {
                ck_mhz: 1700,
                ..cfg()
            }
        ),
        Err(Error::BadFrequency)
    );
    // 12 Gb per channel (MR8 density code 5): 3/4 rows, not supported yet.
    let mut hw = Hw::new([part(1, 0x14); 4]);
    assert_eq!(
        ddr::init(&mut hw, &cfg()),
        Err(Error::Geometry { ch: 0, mr8: 0x14 })
    );
}

// --------------------------------------------------------------- detect

#[test]
fn mr8_geometry_matches_rk3568_rule() {
    for mr8 in 0..=255u8 {
        if let Some(g) = detect::geometry(mr8) {
            let (row, r34) = detect::rk3568_rows(mr8);
            assert_eq!((g.row, g.row_3_4), (row, r34), "mr8 {mr8:#04x}");
            // Size = 2^(row + col + banks) x 2 bytes (16-bit channel), 3/4 aside.
            let full = 1u64 << (g.row + g.col + g.bank_bits + 1 - 20);
            let mib = if g.row_3_4 { full * 3 / 4 } else { full };
            assert_eq!(g.rank_mib, mib, "mr8 {mr8:#04x}");
        }
    }
}

#[test]
fn mr8_examples() {
    let g = detect::geometry(MR8_8GB).unwrap();
    assert_eq!(
        (g.row, g.rank_mib, g.density_gb, g.byte_mode),
        (16, 1024, 8, false)
    );
    let g = detect::geometry(MR8_16GB).unwrap();
    assert_eq!((g.row, g.rank_mib, g.density_gb), (17, 2048, 16));
    // x8 byte-mode dies, 8 Gb each: a 16-bit channel of two dies = 16 Gb.
    let g = detect::geometry(0x50).unwrap();
    assert_eq!(
        (g.row, g.rank_mib, g.density_gb, g.byte_mode),
        (17, 2048, 8, true)
    );
    assert!(detect::geometry(0xff).is_none());
    // Density code 0: 2 Gb per channel, 14 rows, 256 MiB.
    let g = detect::geometry(0x00).unwrap();
    assert_eq!((g.row, g.rank_mib), (14, 256));
}

#[test]
fn rank_decision() {
    let ids = |r1| detect::Ids {
        mr8: MR8_8GB,
        mr8_rank1: r1,
        ..Default::default()
    };
    assert_eq!(detect::ranks(&ids(MR8_8GB), 2, None), 2);
    assert_eq!(detect::ranks(&ids(MR8_8GB), 2, Some(true)), 2);
    assert_eq!(detect::ranks(&ids(MR8_8GB), 2, Some(false)), 1);
    assert_eq!(detect::ranks(&ids(0x00), 2, None), 1);
    assert_eq!(detect::ranks(&ids(0xff), 2, Some(true)), 1);
    assert_eq!(detect::ranks(&ids(MR8_8GB), 1, None), 1);
    // An undriven bus: MR8 0x00 is valid, MR5 0x00 is not.
    assert!(!detect::present(&detect::Ids {
        mr5: 0,
        mr8: 0,
        ..Default::default()
    }));
    assert!(detect::present(&detect::Ids {
        mr5: 0xff,
        mr8: MR8_8GB,
        ..Default::default()
    }));
}

#[test]
fn layout_codes_and_device_size() {
    assert_eq!(msch::linear_code(1024), Some(0b00011));
    assert_eq!(msch::linear_code(4096), Some(0b00001));
    assert_eq!(msch::linear_code(8192), None);
    assert_eq!(msch::test_window(8192), Some(4096));
    assert_eq!(msch::test_window(2048), Some(2048));
    assert_eq!(msch::test_window(16384), None);
    assert_eq!(ddr::upper_half_test_base(32768), Some(0x4_0000_0000));
    assert_eq!(ddr::upper_half_test_base(8192), Some(0x1_0000_0000));
    assert_eq!(ddr::upper_half_test_base(4096), None);
    assert_eq!(msch::device_size(4096, 2), 0x4040);
    assert_eq!(msch::device_size(1024, 2), 0x1010);
    assert_eq!(msch::device_size(2048, 1), 0x20);
    assert_eq!(ddr::linear_base(3, 2048), 0x1_8000_0000);
}

// --------------------------------------------------------------- timing

#[test]
fn timing_follows_jedec_lpddr_across_the_range() {
    for f in (533..=1600).step_by(17) {
        let c = Config { ck_mhz: f, ..cfg() };
        for dens in [4u8, 8, 16] {
            let p = timing::plan(&c, 2, dens);
            let (_, row) = jt::freq_range(f).unwrap();
            assert_eq!(
                (p.rl, p.wl, p.nwr),
                (row.rl as u32, row.wl_a as u32, row.nwr as u32)
            );
            let t = p.dramset1tmg;
            // TMG2 latencies.
            assert_eq!((t[2] >> 16) & 0x7f, p.rl);
            assert_eq!((t[2] >> 24) & 0x7f, p.wl);
            // TMG0 wr2pre = WL + BL/2 + nWR + 1 (p.69).
            assert_eq!(t[0] >> 24, p.wl + 8 + p.nwr + 1);
            // tRFCab from jedec-lpddr, tREFI never exceeded.
            let rfc = jt::refresh(dens).unwrap().t_rfc_ab.cycles(f);
            assert_eq!(p.rfshset1tmg1, rfc);
            let refi = jt::Max {
                ps: timing::T_REFI_PS,
            }
            .cycles(f);
            assert!(32 * (p.rfshset1tmg0 & 0xfff) <= refi);
            // tRCD >= 18 ns, tRP >= 21 ns (in ps: cycles * 1e6 / f).
            let ns = |cyc: u32| cyc as u64 * 1_000_000 / f as u64;
            assert!(ns(t[4] >> 24) >= 18_000);
            assert!(ns(t[4] & 0x7f) >= 21_000);
            // Every field fits its TRM width (no saturation).
            assert!(!p.overflow, "{f} MHz, {dens} Gb");
            assert_eq!(p.initmr[0] & 0xffff, p.mr2 as u32);
            assert_eq!(p.initmr[0] >> 16, p.mr1 as u32);
        }
    }
}

#[test]
fn pll_covers_the_supported_range_in_steps() {
    for f in [800, 933, 1056, 1068, 1200, 1392, 1560, 1600] {
        let s = rk3588::pll::settings(f).unwrap();
        assert_eq!(s.fout_mhz(), f);
    }
}

// ------------------------------------------------------------- fallback

#[test]
fn sweep_centres_and_restores() {
    let mut hw = Hw::new([part(1, MR8_8GB); 4]);
    // All pass: centre of the whole axis.
    let mut last = 0u32;
    let r = fallback::sweep(&mut hw, 64, 0x14, |_, i| last = i, |_| true);
    assert_eq!(r.map(|x| x.0), Some(32));
    assert_eq!(last, 32);
    // None pass: the knob goes back to its restore value.
    let r = fallback::sweep(&mut hw, 64, 0x14, |_, i| last = i, |_| false);
    assert!(r.is_none());
    assert_eq!(last, 0x14);
}

#[test]
fn rx_vref_sweep_writes_both_slices() {
    let mut hw = Hw::new([part(1, MR8_8GB); 4]);
    let pb = base::DDRPHY[0];
    let v = fallback::rx_vref(&mut hw, 0, |io| {
        use regio::RegIo;
        let c = io.read32(pb + phy::ZQ_CON9) & 0x3f;
        (20..=40).contains(&c)
    });
    assert_eq!(v, Some(30));
    assert_eq!(hw.get(pb + phy::ZQ_CON9) & 0x3f3f, (30 << 8) | 30);
}

// ------------------------------------------------------------- snapshot

/// The complete register program of channel 0 (2 ranks, 8 Gb, default
/// config): a regtrace snapshot so any change to the sequence is a
/// visible, reviewed diff. Regenerate with RK3588_DDR_BLESS=1.
#[test]
fn snapshot_channel0_program() {
    let c = cfg();
    let mut hw = Hw::new([part(2, MR8_8GB); 4]);
    let g = detect::geometry(MR8_8GB).unwrap();
    let info = ddr::bring_up_channel(&mut hw, 0, &c, 2, &g).unwrap();
    assert!(info.train.ok());
    let evs = writes_and_delays(&hw.evs);
    let text: String = evs.iter().map(|e| format!("{e}\n")).collect();
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/ddr_ch0_program.trace"
    );
    if std::env::var_os("RK3588_DDR_BLESS").is_some() {
        let head =
            "# RK3588 DDR channel 0, 2 ranks x 8 Gb, Config::yy3588_default (writes and delays)\n";
        std::fs::write(path, format!("{head}{text}")).unwrap();
    }
    let reference = regtrace::parse(&std::fs::read_to_string(path).unwrap()).unwrap();
    if let Some(d) = regtrace::compare(&reference, &evs, &CompareOpts::default()) {
        panic!("channel 0 program changed (RK3588_DDR_BLESS=1 to accept):\n{d}");
    }
}
