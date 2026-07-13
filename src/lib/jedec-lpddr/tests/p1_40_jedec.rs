// SPDX-License-Identifier: GPL-2.0-only
//! P1-40: the JEDEC LPDDR4/4X timings, MR4/18/19/22/23, the MPC opcodes and
//! the named LPDDR5 capability bits.

#![allow(clippy::unusual_byte_groupings)] // MR8 fields: OP[7:6]_OP[5:2]_OP[1:0].

use jedec_lpddr::lp4::cmd::mpc;
use jedec_lpddr::lp4::mr::{
    dqs_osc_count, dqs_osc_split, Mr22, Mr4, OscRunTime, RD_DQ_CAL_DEFAULTS, T_TSI_MS,
};
use jedec_lpddr::lp4::timing::{self as t, VrefStep};
use jedec_lpddr::lp5::mr::bits;
use jedec_lpddr::pending;

#[test]
fn max_forms_pick_the_larger_side() {
    // tXP = max(7.5 ns, 5 nCK): 5 nCK wins at 400 MHz (2.5 ns), 7.5 ns at
    // 2133 MHz (16 cycles, rounded up).
    assert_eq!(t::T_XP.cycles(400), 5);
    assert_eq!(t::T_XP.cycles(2133), 16);
    assert_eq!(t::T_CKE.cycles(1600), 12);
    assert_eq!(t::T_CKELPD.cycles(200), 3);
    assert_eq!(t::T_MRW.cycles(800), 10);
    assert_eq!(t::T_MRW.cycles(1600), 16);
    assert_eq!(t::T_MRD.cycles(1600), 23);
    assert_eq!(t::T_ZQLAT.cycles(1600), 48);
    assert_eq!(t::T_ZQLAT.cycles(100), 8);
    assert_eq!(t::T_ZQRESET.cycles(1600), 80);
    assert_eq!(t::T_ZQCAL.cycles(1600), 1600);
    assert_eq!(t::T_CKFSPE, t::T_CKFSPX);
    assert_eq!(t::T_SDO.cycles(400), 12);
    assert_eq!(t::T_SDO.cycles(1600), 32);
}

#[test]
fn vref_and_fsp_settling() {
    assert_eq!(t::t_vref(VrefStep::Short).ps, 100_000);
    assert_eq!(t::t_vref(VrefStep::Middle).ps, 200_000);
    assert_eq!(t::t_vref(VrefStep::Long).ps, 250_000);
    assert_eq!(t::t_vref(VrefStep::Weak).ps, 1_000_000_000);
    assert_eq!(t::t_vref_ca_cbt_x16(VrefStep::Short).unwrap().ps, 80_000);
    assert_eq!(t::t_vref_ca_cbt_x16(VrefStep::Middle), None);
    assert_eq!(
        t::t_vref_ca_cbt_x8_mode2(VrefStep::Middle).unwrap().ps,
        200_000
    );
    assert_eq!(t::t_vref_ca_cbt_x8_mode2(VrefStep::Weak), None);
    assert_eq!(t::T_VRCG_ENABLE.ps, 200_000);
    assert_eq!(t::T_VRCG_DISABLE.ps, 100_000);
    assert_eq!(t::t_fc(VrefStep::Middle).unwrap().ps, 200_000);
    assert_eq!(t::t_fc(VrefStep::Long).unwrap().ps, 250_000);
    let x = t::t_xcbt(VrefStep::Long).unwrap();
    assert_eq!((x.nck, x.ps), (5, 250_000));
    assert_eq!(t::t_xcbt(VrefStep::Short).unwrap().cycles(10), 5);
}

#[test]
fn mr4_round_trip() {
    for op in 0..=255u8 {
        assert_eq!(Mr4::decode(op).encode(), Some(op));
    }
    let m = Mr4::decode(0x83);
    assert!(m.tuf && m.refresh_rate == 3 && !m.ppre && m.thermal_offset == 0);
    assert_eq!(Mr4::decode(0x60).thermal_offset, 3);
    assert_eq!(
        Mr4 {
            refresh_rate: 8,
            sr_abort: false,
            ppre: false,
            thermal_offset: 0,
            tuf: false
        }
        .encode(),
        None
    );
    assert_eq!(T_TSI_MS, 32);
}

#[test]
fn dqs_osc_count_bytes() {
    assert_eq!(dqs_osc_count(0x34, 0x12), 0x1234);
    for c in [0u16, 1, 0xff, 0x100, 0xbeef, 0xffff] {
        let (lo, hi) = dqs_osc_split(c);
        assert_eq!(dqs_osc_count(lo, hi), c);
    }
}

#[test]
fn mr22_round_trip() {
    for op in 0..=255u8 {
        assert_eq!(Mr22::decode(op).encode(), Some(op));
    }
    let m = Mr22::decode(0b0010_0110);
    assert_eq!(m.soc_odt, 6);
    assert!(m.ca_odt_disable && !m.cs_odt_disable && !m.ck_odt_disable);
    assert_eq!(Mr22::decode(0xc0).x8_odt_disable, 3);
}

#[test]
fn mr23_run_time() {
    assert_eq!(OscRunTime::decode(0), Some(OscRunTime::StopByMpc));
    assert_eq!(OscRunTime::decode(1), Some(OscRunTime::Clocks(16)));
    assert_eq!(OscRunTime::decode(63), Some(OscRunTime::Clocks(1008)));
    assert_eq!(OscRunTime::decode(0x40), Some(OscRunTime::Clocks(2048)));
    assert_eq!(OscRunTime::decode(0x80), Some(OscRunTime::Clocks(4096)));
    assert_eq!(OscRunTime::decode(0xc0), Some(OscRunTime::Clocks(8192)));
    assert_eq!(OscRunTime::decode(0x41), None);
    for op in 0..=255u8 {
        if let Some(r) = OscRunTime::decode(op) {
            assert_eq!(r.encode(), Some(op));
        }
    }
    assert_eq!(OscRunTime::Clocks(17).encode(), None);
    assert_eq!(OscRunTime::Clocks(1024).encode(), None);
}

#[test]
fn rd_dq_cal_defaults() {
    assert_eq!(
        RD_DQ_CAL_DEFAULTS,
        [(32, 0x5a), (40, 0x3c), (15, 0x55), (20, 0x55)]
    );
}

#[test]
fn mpc_opcodes() {
    assert_eq!(mpc::ALL, [0x41, 0x43, 0x47, 0x4b, 0x4d, 0x4f, 0x51]);
    let cas2: Vec<u8> = mpc::ALL
        .into_iter()
        .filter(|&o| mpc::needs_cas2(o))
        .collect();
    assert_eq!(cas2, [mpc::RD_FIFO, mpc::RD_DQ_CAL, mpc::WR_FIFO]);
}

#[test]
fn lp5_capability_bits() {
    assert!(bits::has(0x80, bits::MR24_DFE_SUPPORTED));
    assert!(bits::has(0x08, bits::MR24_READ_DCA_SUPPORTED));
    assert!(!bits::has(0x80, bits::MR24_READ_DCA_SUPPORTED));
    assert!(bits::has(0x40, bits::MR26_RDQS_T_TRAINING));
    assert!(bits::has(0x20, bits::MR19_WCK2DQ_OSC_FM));
    assert!(bits::has(0x04, bits::MR13_VRO));
    assert_eq!(bits::mr41_dvfsc(0b0000_0110), 3);
    assert_eq!(bits::mr41_dvfsc(0b0000_1010), 1);
    assert!(bits::has(0x08, bits::MR41_E_DVFSC_ODT));
}

#[test]
fn lp4_pending_is_empty() {
    assert!(pending::LP4.is_empty());
    assert_eq!(pending::LP5.len(), 3);
}

mod lp5x {
    use jedec_lpddr::lp5::mr::bits::{self, DvfscSupport, Vdd2Rail};
    use jedec_lpddr::lp5::mr::{BankOrg, DeviceType, IoWidth, Mr8};
    use jedec_lpddr::lp5::timing::{
        bank_mode_allowed, refresh, refresh_edvfsc, t_pbr2act_ps, wck_clock, BankMode, WCK_CLOCK,
        WCK_DUTY_ABS, WCK_DUTY_AVG,
    };
    use jedec_lpddr::lp5::WckCkRatio::{Four, Two};

    #[test]
    fn mr8_density_type_width() {
        let m = Mr8::decode(0b01_0101_10);
        assert_eq!(m.io_width, Some(IoWidth::X8));
        assert_eq!(m.density_gb, Some(12));
        assert_eq!(m.device_type_kind(), Some(DeviceType::VendorLpddr5x9600));
        let m = Mr8::decode(0b00_0100_01);
        assert_eq!(
            (m.io_width, m.density_gb, m.device_type_kind()),
            (Some(IoWidth::X16), Some(8), Some(DeviceType::Lpddr5x))
        );
        assert_eq!(Mr8::decode(0b00_0110_00).density_gb, Some(16));
        assert_eq!(
            Mr8::decode(0b00_0110_00).device_type_kind(),
            Some(DeviceType::Lpddr5)
        );
        assert_eq!(Mr8::decode(0b00_0110_11).device_type_kind(), None);
        assert_eq!(Mr8::decode(0b10_0111_11).io_width, None);
        assert_eq!(Mr8::decode(0b00_1001_00).density_gb, None);
    }

    #[test]
    fn mr3_bank_org_codes() {
        assert_eq!(BankOrg::from_mr3(0b0000_0000), Some(BankOrg::Bg));
        assert_eq!(BankOrg::from_mr3(0b0000_1000), Some(BankOrg::B8));
        assert_eq!(BankOrg::from_mr3(0b0001_0000), Some(BankOrg::B16));
        assert_eq!(BankOrg::from_mr3(0b0001_1000), None);
        for b in [BankOrg::Bg, BankOrg::B8, BankOrg::B16] {
            assert_eq!(BankOrg::from_code(b as u8), Some(b));
        }
    }

    #[test]
    fn single_bits() {
        assert!(bits::has(0x80, bits::MR0_PER_PIN_DFE));
        assert_eq!(Vdd2Rail::from_mr13(0x80), Vdd2Rail::Single);
        assert_eq!(Vdd2Rail::from_mr13(0x7f), Vdd2Rail::Dual);
        assert_eq!(Vdd2Rail::Single.vdd2l_mv(), (1010, 1120));
        assert_eq!(Vdd2Rail::Dual.vdd2l_mv(), (870, 970));
        assert_eq!(bits::mr19_dvfsc(0b1110_0110), 0b10);
        assert_eq!(
            DvfscSupport::from_mr41(0b0000),
            Some(DvfscSupport::DvfscOnly)
        );
        assert_eq!(
            DvfscSupport::from_mr41(0b0010),
            Some(DvfscSupport::EnhancedOnly)
        );
        assert_eq!(DvfscSupport::from_mr41(0b0100), Some(DvfscSupport::Both));
        assert_eq!(DvfscSupport::from_mr41(0b0110), None);
    }

    #[test]
    fn bank_mode_limits() {
        assert!(bank_mode_allowed(BankMode::Bg, Four, false, 6400));
        assert!(!bank_mode_allowed(BankMode::Bg, Two, false, 3200));
        assert!(!bank_mode_allowed(BankMode::Bg, Four, true, 1600));
        assert!(bank_mode_allowed(BankMode::B16, Two, false, 3200));
        assert!(!bank_mode_allowed(BankMode::B16, Two, false, 3733));
        assert!(!bank_mode_allowed(BankMode::B16, Two, false, 4267));
        assert!(bank_mode_allowed(BankMode::B16, Four, true, 1600));
        assert!(!bank_mode_allowed(BankMode::B16, Four, true, 2133));
        assert!(bank_mode_allowed(BankMode::B8, Four, false, 6400));
        assert!(!bank_mode_allowed(BankMode::B8, Two, false, 4267));
        assert!(!bank_mode_allowed(BankMode::B8, Four, true, 3200));
    }

    #[test]
    fn refresh_values() {
        let (r, m) = refresh_edvfsc(16).unwrap();
        assert_eq!(
            (r.t_rfc_ab_ns, r.t_rfc_pb_ns, r.t_pbr2pbr_ns),
            (300, 150, 100)
        );
        assert_eq!((m.t_rfm_ab_ns, m.t_rfm_pb_ns), (300, 190));
        assert_eq!(refresh_edvfsc(8), None);
        let r = refresh(12).unwrap();
        assert_eq!(
            (r.t_rfc_ab_ns, r.t_rfc_pb_ns, r.t_pbr2pbr_ns),
            (280, 140, 90)
        );
        assert_eq!(t_pbr2act_ps(12, BankMode::Bg), Some(7_500));
        assert_eq!(t_pbr2act_ps(12, BankMode::B16), Some(7_500));
        assert_eq!(t_pbr2act_ps(12, BankMode::B8), Some(10_000));
        assert_eq!(t_pbr2act_ps(16, BankMode::Bg), None);
    }

    #[test]
    fn wck_clock_limits() {
        assert_eq!(wck_clock(800).unwrap().t_wck_avg_min_ps, 1250);
        assert_eq!(wck_clock(1067).unwrap().t_jit_per_ps, 56);
        assert_eq!(wck_clock(1000), None);
        // tWCK(avg) min is about one period: within 0.5% of 1e6 / f.
        for w in WCK_CLOCK {
            let period = 1_000_000 / w.wck_mhz;
            assert!(w.t_wck_avg_min_ps.abs_diff(period) * 200 <= period, "{w:?}");
        }
        assert_eq!(WCK_DUTY_AVG, (460, 540));
        assert_eq!(WCK_DUTY_ABS, (430, 570));
    }
}
