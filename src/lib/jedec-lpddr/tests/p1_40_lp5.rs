// SPDX-License-Identifier: GPL-2.0-only
//! P1-40: the LPDDR5 timings, mode-register layouts, code tables and
//! MPC/FIFO/RDC/CAS encodings, against each other and against the older
//! tables of this crate.

use jedec_lpddr::lp4;
use jedec_lpddr::lp5::cmd::{self, mpc, CaEdges, Cas};
use jedec_lpddr::lp5::mr::{self, *};
use jedec_lpddr::lp5::timing::{self as t, cbt, leveling, BankMode, Step};
use jedec_lpddr::lp5::WckCkRatio::{self, Four, Two};

#[test]
fn mr_round_trips() {
    for op in 0..=255u8 {
        if let Some(m) = Mr10::decode(op) {
            assert_eq!(m.encode(), Some(op));
        }
        if let Some(m) = Mr11::decode(op) {
            assert_eq!(m.encode(), op);
        }
        assert_eq!(VrefMr::decode(op).encode(), Some(op));
        if let Some(m) = Mr13::decode(op) {
            assert_eq!(m.encode(), op);
        }
        if let Some(m) = Mr17::decode(op) {
            assert_eq!(m.encode(), op);
        }
        if let Some(m) = Mr19::decode(op) {
            assert_eq!(m.encode(), Some(op));
        }
        if let Some(m) = Mr20::decode(op) {
            assert_eq!(m.encode(), op);
        }
        assert_eq!(Mr24::decode(op).encode(), Some(op));
        if let Some(m) = Mr41::decode(op) {
            assert_eq!(m.encode(), Some(op));
        }
        if let Some(m) = Mr46::decode(op) {
            assert_eq!(m.encode(), op);
        }
        assert_eq!(Mr58::decode(op).encode(), op);
        assert_eq!(dfe_offsets_encode(dfe_offsets_decode(op)), Some(op));
    }
}

#[test]
fn reserved_codes_rejected() {
    assert_eq!(Mr10::decode(0b0000_1100), None); // WCK PST 11b
    assert_eq!(Mr10::decode(0b1100_0000), None); // RDQS PST 11b
    assert_eq!(Mr11::decode(0x07), None);
    assert_eq!(Mr13::decode(0x03), None);
    assert_eq!(Mr13::decode(0x08), None);
    assert_eq!(Mr19::decode(0x03), None);
    assert_eq!(Mr19::decode(0x08), None);
    assert_eq!(Mr19::decode(0xc0), None);
    assert_eq!(Mr20::decode(0x0c), None);
    assert_eq!(Mr41::decode(0xe0), None);
    assert_eq!(Mr46::decode(0x08), None);
    assert_eq!(dfe_offsets_encode([0, 4, 0, 0]), None);
}

#[test]
fn defaults() {
    assert_eq!(Mr10::decode(0).unwrap().wck_pst, WckPst::T2_5);
    assert_eq!(Mr10::decode(0).unwrap().rdqs_pst, RdqsPst::T0_5);
    // MR17: OP[5:3] = 1 means disabled; the reset state disables CK, CS, CA.
    let m = Mr17::decode(MR17_DEFAULT).unwrap();
    assert!(m.ck_odt_disable && m.cs_odt_disable && m.ca_odt_disable);
    assert_eq!(m.soc_odt, Rzq::Disabled);
    assert_eq!(
        MR17_DEFAULT,
        mr::mr17(Rzq::Disabled, MR17_DEFAULT_7_3).unwrap()
    );
    // MR18: CKR 1 = 2:1 is the reset value, and WckCkRatio agrees.
    assert!(Mr18::decode(MR18_DEFAULT).unwrap().ratio_2_1);
    assert_eq!(WckCkRatio::Two.replace_mr18(0, false), MR18_DEFAULT);
    assert_eq!(WckCkRatio::Four.replace_mr18(MR18_DEFAULT, false), 0);
    assert_eq!(Mr20::decode(MR20_DEFAULT).unwrap().rdqs, RdqsMode::TrueOnly);
    assert_eq!(Mr41::decode(MR41_DEFAULT).unwrap().nt_dq_odt, Rzq::Div3);
    assert_eq!(VREF_DEFAULT_CODE, 80);
    assert_eq!(
        jedec_lpddr::lp5::VrefCode::new(VREF_DEFAULT_CODE)
            .unwrap()
            .basis_points(),
        5000
    );
    assert_eq!(
        RDC_DEFAULTS,
        [(31, 0x55), (32, 0x55), (33, 0x5a), (34, 0x3c)]
    );
}

#[test]
fn mr19_and_mr41_dvfsc() {
    let m = Mr19::decode(0b0110_0110).unwrap();
    assert_eq!(m.dvfsc, Dvfsc::Enhanced);
    assert!(m.dvfsq_0v3 && m.osc_fm_supported && !m.osc_hf);
    assert_eq!(m.cs_odt(), Rzq::Div2);
    assert_eq!(Mr19::decode(0).unwrap().cs_odt(), Rzq::Div3);
    use DvfscSupport::*;
    assert!(DvfscOnly.allows(Dvfsc::Dvfsc) && !DvfscOnly.allows(Dvfsc::Enhanced));
    assert!(EnhancedOnly.allows(Dvfsc::Enhanced) && !EnhancedOnly.allows(Dvfsc::Dvfsc));
    assert!(Both.allows(Dvfsc::Dvfsc) && Both.allows(Dvfsc::Enhanced));
    let m41 = Mr41::decode(0b0110_0101).unwrap();
    assert!(m41.per_pin_dfe);
    assert_eq!(m41.dvfsc_support(), Some(Both));
    assert_eq!(m41.nt_dq_odt, Rzq::Div3);
}

#[test]
fn dca_codes() {
    for s in -7..=7i8 {
        assert_eq!(dca_step(dca_code(s).unwrap()), Some(s));
    }
    assert_eq!(dca_code(-1), Some(1));
    assert_eq!(dca_code(1), Some(9));
    assert_eq!(dca_code(8), None);
    assert_eq!(dca_step(8), None);
    let m = Mr30::from_steps(-7, 7).unwrap();
    assert_eq!(m.encode(), Some(0x7f));
    assert_eq!(m.steps(), Some((-7, 7)));
}

#[test]
fn rdc_bit_order() {
    let b = rdc_burst(0x27, 0x00);
    assert_eq!(
        &b[..8],
        &[true, true, true, false, false, true, false, false]
    );
    let inv = rdc_pin_burst(0x27, 0x00, true, false);
    assert!(inv[8..].iter().all(|&x| x));
    assert!(rdc_pin_burst(0xff, 0xff, true, true).iter().all(|&x| !x));
}

#[test]
fn oscillators_and_per_pin_dfe() {
    assert_eq!(Wck2DqOsc::Input.registers(), (35, 36, 37));
    assert_eq!(Wck2DqOsc::Output.registers(), (38, 39, 40));
    assert_eq!(Wck2DqOsc::Input.mpc(), (0x81, 0x82));
    assert_eq!(Wck2DqOsc::Output.mpc(), (0x83, 0x84));
    assert_eq!(osc_count(0x34, 0x12), 0x1234);
    assert_eq!(OscRunTime::decode(0x80), Some(OscRunTime::Clocks(4096)));
    assert_eq!(dfe_offset_register(0), Some((70, 0)));
    assert_eq!(dfe_offset_register(15), Some((73, 3)));
    assert_eq!(dfe_offset_register(16), None);
    assert_eq!(dfe_offsets_encode([1, 2, 3, 0]), Some(0b00_11_10_01));
}

#[test]
fn vref_limits() {
    assert_eq!(dq_vref_mv(1600, false), (75, 350));
    assert_eq!(dq_vref_mv(2400, false), (75, 225));
    assert_eq!(dq_vref_mv(3201, false), (75, 180));
    assert_eq!(dq_vref_mv(800, true), (75, 300));
    assert_eq!(CA_VREF_MV, (75, 350));
    let v = VrefMr::decode(0xd0);
    assert_eq!((v.code, v.op7), (0x50, true));
}

#[test]
fn commands() {
    assert_eq!(
        cmd::mpc(mpc::ZQCAL_START),
        CaEdges {
            rise: 0x70,
            fall: 0x05
        }
    );
    assert_eq!(
        cmd::mpc(0x06),
        CaEdges {
            rise: 0x30,
            fall: 0x06
        }
    );
    assert_eq!(mpc::ALL, [0x81, 0x82, 0x83, 0x84, 0x85, 0x86]);
    assert_eq!(
        (cmd::WFF.rise, cmd::RFF.rise, cmd::RDC.rise),
        (0x60, 0x20, 0x50)
    );
    // The A7A reference rows: CAS(WS_FS) 4Ch, CAS(WS_WR|WS_RD|WS_FS) 7Ch.
    let fs = Cas {
        ws_fs: true,
        ..Cas::default()
    };
    assert_eq!(fs.encode().unwrap().rise, 0x4c);
    let all = Cas {
        ws_wr: true,
        ws_rd: true,
        ws_fs: true,
        ..Cas::default()
    };
    assert_eq!(all.encode().unwrap().rise, 0x7c);
    for fall in 0..128u8 {
        for ws in 0..8u8 {
            let e = CaEdges {
                rise: 0x0c | ws << 4,
                fall,
            };
            assert_eq!(Cas::decode(e).unwrap().encode(), Some(e));
        }
    }
    assert_eq!(Cas::decode(cmd::RDC), None);
    assert_eq!(
        Cas {
            dc: 16,
            ..Cas::default()
        }
        .encode(),
        None
    );
}

#[test]
fn lp5_mrw_mrd_differ_from_lp4() {
    assert_eq!((t::T_MRW.nck, t::T_MRW.ps), (5, 10_000));
    assert_eq!((t::T_MRD.nck, t::T_MRD.ps), (5, 14_000));
    assert_ne!(t::T_MRW, lp4::timing::T_MRW);
    assert_ne!(t::T_MRD, lp4::timing::T_MRD);
    // At 400 MHz CK the nCK side decides: 5 vs LPDDR4's 10.
    assert_eq!(t::T_MRW.cycles(400), 5);
    assert_eq!(lp4::timing::T_MRW.cycles(400), 10);
    assert_eq!(t::t_mrr_nck(Four), 4);
    assert_eq!(t::t_mrr_nck(Two), 8);
    assert_eq!(t::t_mrri_nck(18), 20);
}

#[test]
fn half_ck_and_wck_minimums() {
    // 200 ns + 0.5 tCK at 600 MHz: 120 + 0.5 -> 121 nCK.
    assert_eq!(t::t_fc(Step::Short).cycles(600), 121);
    assert_eq!(t::t_fc(Step::Long).cycles(600), 151);
    assert_eq!(t::t_vref(Step::Long), t::t_fc(Step::Long));
    assert_eq!(t::T_VREF_WEAK_PS, 1_000_000_000);
    // max(25 ns, 2.5 tWCK) at 2400 MHz WCK: 60 cycles; at 40 MHz: 3.
    assert_eq!(leveling::T_WCK_INT.wck_cycles(2400), 60);
    assert_eq!(leveling::T_WCK_INT.wck_cycles(40), 3);
    assert_eq!(cbt::T_WCK2DQ7H.wck_cycles(400), 12);
    assert_eq!(cbt::t_cacd_nck(600), 12);
    assert_eq!(cbt::t_ckprecs_nck(600), 2 + 5);
    assert_eq!(cbt::T_XCBT.cycles(800), 200);
    assert_eq!(leveling::T_WCKTGGL_HALF_WCK, 15);
}

#[test]
fn wck2ck_range() {
    assert_eq!(t::t_wck2ck(Four).ps(2400), (-208, 208));
    assert_eq!(t::t_wck2ck(Two).ps(1600), (-156, 156));
    assert_eq!(leveling::T_WCK2CK_LEVELING, t::t_wck2ck(Four));
}

#[test]
fn zq_init_and_ranges() {
    assert_eq!(t::t_zqcal_ps(1), Some(1_500_000));
    assert_eq!(t::t_zqcal_ps(8), Some(3_000_000));
    assert_eq!(t::t_zqcal_ps(16), Some(6_000_000));
    assert_eq!(t::t_zqcal_ps(17), None);
    assert_eq!(t::T_ZQLAT.cycles(600), 18);
    assert_eq!(t::T_ZQLAT.cycles(100), 4);
    assert_eq!(t::INIT.t_init1_ns, 200_000);
    assert_eq!(t::INIT.t_init3_ns, 2_000_000);
    assert_eq!(t::t_wck2dqi(true, 6400), t::PsRange { min: 300, max: 700 });
    assert_eq!(t::t_wck2dqi(true, 7500), t::PsRange { min: 250, max: 600 });
    assert_eq!(t::t_wck2dqo(false).max, 1900);
    assert_eq!(t::wck_dca_range(true).step_dps, (15, 60));
    assert_eq!(t::read_dca_range(Dvfsc::Enhanced).total_dps, (100, 700));
    assert_eq!(t::dfe_range(true).max_mv, (35, 50, 65));
    assert_eq!(t::dfe_range(false).step_mv, (5, 10, 15));
    assert_eq!(Mr24::dfe_max_code(false), 3);
}

#[test]
fn code_tables_match_the_frequency_ranges() {
    for (i, r) in t::FREQ_RANGES_4_1.iter().enumerate() {
        assert_eq!(r.mr as usize, i);
        assert_eq!((r.wl[0], r.wl[1]), t::WL_4_1[i]);
        let (a, b, c, n) = t::RL_4_1[i];
        assert_eq!((r.rl, r.n_rbtp), ([a, b, c], n));
        assert_eq!(r.n_wr, t::NWR_4_1[i].x16);
        assert_eq!(r.dr_max, t::NWR_4_1[i].dr_max);
    }
    for (i, r) in t::FREQ_RANGES_2_1.iter().enumerate() {
        assert_eq!((r.wl[0], r.wl[1]), t::WL_2_1[i]);
        let (a, b, c, n) = t::RL_2_1[i];
        assert_eq!((r.rl, r.n_rbtp), ([a, b, c], n));
    }
    // A7A at 4800 Mb/s, 4:1: code 1000b, WL 7/12, RL 13/14/15, nWR 21.
    let c = t::code_4_1(4800).unwrap() as usize;
    assert_eq!(c, 0b1000);
    assert_eq!(t::WL_4_1[c], (7, 12));
    assert_eq!(t::RL_4_1[c], (13, 14, 15, 3));
    assert_eq!(t::NWR_4_1[c].x16, 21);
    assert_eq!(t::code_4_1(8533), Some(13));
    assert_eq!(t::code_4_1(8534), None);
}

#[test]
fn bank_mode_table() {
    use mr::Dvfsc as D;
    assert_eq!(
        t::bank_mode_max_rate(BankMode::Bg, Four, D::HighSpeed, true),
        Some(8533)
    );
    assert_eq!(
        t::bank_mode_max_rate(BankMode::B8, Four, D::HighSpeed, true),
        None
    );
    assert_eq!(
        t::bank_mode_max_rate(BankMode::B16, Two, D::Enhanced, true),
        Some(3200)
    );
    assert_eq!(
        t::bank_mode_max_rate(BankMode::B16, Two, D::Enhanced, false),
        None
    );
    assert_eq!(
        t::bank_mode_max_rate(BankMode::B8, Two, D::Dvfsc, false),
        Some(1600)
    );
    assert_eq!(
        t::bank_mode_max_rate(BankMode::Bg, Four, D::Dvfsc, false),
        None
    );
}
