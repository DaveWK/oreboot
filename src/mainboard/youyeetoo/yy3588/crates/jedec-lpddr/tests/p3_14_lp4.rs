// SPDX-License-Identifier: GPL-2.0-only
//! The MPC CA encoding and sequence, the MR4 refresh-rate meaning
//! and cache band, the DQS oscillator delay, the read-DQ calibration
//! pattern per DQ.

use jedec_lpddr::lp4::cmd::{self, mpc, CaCycles, CmdError, CAS2};
use jedec_lpddr::lp4::mr::{
    dqs_osc_count, dqs_osc_delay_ps, Mr4, OscRunTime, RdDqCal, RefreshRate, DQS_OSC_DIVISOR,
    RD_DQ_CAL_DEFAULTS,
};

#[test]
fn mpc_follows_the_truth_table_row() {
    // `L L L L L OP6 | OP0-5`: ZQCAL-START 0x4f -> CA5 = 1, then 0x0f.
    assert_eq!(
        cmd::mpc(mpc::ZQCAL_START),
        Ok(CaCycles {
            first: 0x20,
            second: 0x0f
        })
    );
    // An opcode below 0x40 leaves CA5 low.
    assert_eq!(
        cmd::mpc(0x06),
        Ok(CaCycles {
            first: 0x00,
            second: 0x06
        })
    );
    assert_eq!(cmd::mpc(0x80), Err(CmdError::Opcode));
    // Every opcode round-trips through the decoder; non-MPC first cycles
    // (MRW-1, MRR-1, CAS-2) are refused.
    for op in 0..=0x7fu8 {
        let c = cmd::mpc(op).unwrap();
        assert_eq!(cmd::mpc_opcode(c), Some(op));
    }
    assert_eq!(cmd::mpc_opcode(cmd::mrw(1, 2).unwrap()[0]), None);
    assert_eq!(cmd::mpc_opcode(cmd::mrr(1).unwrap()[0]), None);
    assert_eq!(
        cmd::mpc_opcode(CaCycles {
            first: CAS2,
            second: 0
        }),
        None
    );
}

#[test]
fn fifo_and_dq_cal_commands_take_cas2() {
    for op in mpc::ALL {
        let (seq, n) = cmd::mpc_sequence(op).unwrap();
        assert_eq!(seq[0], cmd::mpc(op).unwrap());
        if mpc::needs_cas2(op) {
            assert_eq!(n, 2);
            assert_eq!(
                seq[1],
                CaCycles {
                    first: CAS2,
                    second: 0
                }
            );
        } else {
            assert_eq!(n, 1);
            assert_eq!(seq[1], CaCycles::EMPTY);
        }
    }
    assert_eq!(cmd::mpc_sequence(0xff), Err(CmdError::Opcode));
}

#[test]
fn mr4_codes_mean_refresh_multiples_and_bands() {
    let table = [
        (0, RefreshRate::LowLimitExceeded, None, false),
        (1, RefreshRate::X4, Some(16), false),
        (2, RefreshRate::X2, Some(8), false),
        (3, RefreshRate::X1, Some(4), false),
        (4, RefreshRate::Half, Some(2), false),
        (5, RefreshRate::Quarter, Some(1), false),
        (6, RefreshRate::QuarterDerated, Some(1), true),
        (7, RefreshRate::HighLimitExceeded, None, false),
    ];
    for (code, rate, quarters, derate) in table {
        assert_eq!(RefreshRate::from_code(code), rate);
        assert_eq!(rate.code(), code);
        assert_eq!(rate.refi_quarters(), quarters);
        assert_eq!(rate.derate(), derate);
        assert_eq!(rate.out_of_range(), quarters.is_none());
        // The band is the code whatever the other MR4 bits say.
        for rest in [0x00u8, 0x08, 0x10, 0x60, 0x80, 0xf8] {
            assert_eq!(Mr4::band(rest | code), code);
            assert_eq!(Mr4::decode(rest | code).refresh_rate(), rate);
        }
    }
    // Monotonic in temperature: a hotter DRAM asks for more refresh.
    let q = |c: u8| RefreshRate::from_code(c).refi_quarters();
    assert!(q(1) > q(2) && q(2) > q(3) && q(3) > q(4) && q(4) > q(5));
}

#[test]
fn oscillator_count_gives_tdqs2dq() {
    assert_eq!(DQS_OSC_DIVISOR, 2);
    // 8192 clocks at 625 ps (1600 MHz CK, 3200 MT/s): count 8000 -> 320 ps.
    assert_eq!(dqs_osc_delay_ps(8192, 625, 8000), Some(320));
    assert_eq!(dqs_osc_delay_ps(8192, 625, 6400), Some(400));
    assert_eq!(dqs_osc_delay_ps(8192, 625, 0), None);
    // Through the mode registers: MR18 low, MR19 high.
    let (lo, hi) = (0x40u8, 0x1fu8);
    assert_eq!(dqs_osc_count(lo, hi), 8000);
    let run = OscRunTime::decode(0xc0).unwrap();
    assert_eq!(run.clocks(), Some(8192));
    assert_eq!(OscRunTime::StopByMpc.clocks(), None);
    assert_eq!(
        dqs_osc_delay_ps(run.clocks().unwrap(), 625, dqs_osc_count(lo, hi)),
        Some(320)
    );
}

#[test]
fn read_dq_cal_pattern_per_dq() {
    let d = RdDqCal::DEFAULT;
    assert_eq!(d.mrs(), RD_DQ_CAL_DEFAULTS);
    assert_eq!(RdDqCal::from_mrs(0x5a, 0x3c, 0x55, 0x55), d);
    // Default inversion 0x55: even DQs inverted, odd ones not, both bytes.
    assert_eq!(d.expected(1), Some(0x3c5a));
    assert_eq!(d.expected(0), Some(!0x3c5au16));
    assert_eq!(d.expected(9), Some(0x3c5a));
    assert_eq!(d.expected(8), Some(!0x3c5au16));
    assert_eq!(d.expected(16), None);
    for dq in 0..16u8 {
        assert_eq!(d.inverted(dq), dq % 2 == 0);
    }
    // Separate masks per byte.
    let m = RdDqCal::from_mrs(0xff, 0x00, 0x00, 0x80);
    assert_eq!(m.expected(7), Some(0x00ff));
    assert_eq!(m.expected(15), Some(!0x00ffu16));
    assert!(jedec_lpddr::pending::LP4
        .iter()
        .any(|p| p.module == "lp4::mr::RdDqCal"));
}
