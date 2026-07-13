// SPDX-License-Identifier: GPL-2.0-only
//! P1-39: every value filled from the free sources, against the cited
//! numbers (orchestrator references/datasheets/README.md section 3).

// MR8 literals are grouped by field (OP[7:6], OP[5:2], OP[1:0]).
#![allow(clippy::unusual_byte_groupings)]

use jedec_lpddr::lp4::{cmd as lp4cmd, mr as lp4mr, timing as lp4t};
use jedec_lpddr::lp5::{mr as lp5mr, timing as lp5t};

#[test]
fn lp4_refresh_per_density_jedec() {
    // coreboot lp4x.go fec3d608 lines 280-309.
    for (gb, ab, pb) in [
        (3, 180, 90),
        (4, 180, 90),
        (6, 280, 140),
        (8, 280, 140),
        (12, 380, 190),
        (16, 380, 190),
    ] {
        let r = lp4t::refresh(gb).unwrap();
        assert_eq!(
            (r.t_rfc_ab.ps, r.t_rfc_pb.ps),
            (ab * 1000, pb * 1000),
            "{gb} Gb"
        );
        assert_eq!((r.t_rfc_ab.nck, r.t_rfc_pb.nck), (0, 0));
    }
    for gb in [0, 1, 2, 5, 7, 24, 32] {
        assert_eq!(lp4t::refresh(gb), None);
    }
}

#[test]
fn lp4_core_and_init_timings_longsys_tables_12_13() {
    assert_eq!(lp4t::T_RPAB, lp4t::Min { nck: 4, ps: 21_000 });
    assert_eq!(lp4t::T_RPPB, lp4t::Min { nck: 4, ps: 18_000 });
    let rfc = lp4t::refresh(8).unwrap().t_rfc_ab; // 280 ns (Table 13 too)
    let xsr = lp4t::t_xsr(rfc);
    assert_eq!(
        xsr,
        lp4t::Min {
            nck: 2,
            ps: 287_500
        }
    );
    // At 1200 MHz (2400 MT/s): 287.5 ns = 345 CK; tRPab 21 ns = 26 CK.
    assert_eq!(xsr.cycles(1200), 345);
    assert_eq!(lp4t::T_RPAB.cycles(1200), 26);
    assert_eq!(
        lp4t::T_RPPB.cycles(100),
        4,
        "the 4 nCK floor at a slow clock"
    );
    let i = lp4t::INIT;
    assert_eq!(
        (
            i.t_init0_max_ms,
            i.t_init1_us,
            i.t_init2_ns,
            i.t_init3_ms,
            i.t_init4_nck,
            i.t_init5_us
        ),
        (20, 200, 10, 2, 5, 2)
    );
}

#[test]
fn lp4_mr13_layout_round_trip() {
    for op in 0..=255u8 {
        assert_eq!(lp4mr::Mr13::decode(op).encode(), op);
    }
    let m = lp4mr::Mr13::decode(0b1100_1001);
    assert!(m.fsp_op && m.fsp_wr && m.vrcg && m.cbt && !m.dmd && !m.pro);
    assert_eq!(
        lp4mr::Mr13 {
            dmd: true,
            ..lp4mr::Mr13::decode(0)
        }
        .encode(),
        0x20
    );
    assert_eq!(
        lp4mr::Mr13 {
            pro: true,
            ..lp4mr::Mr13::decode(0)
        }
        .encode(),
        0x10
    );
}

/// The LiteDRAM truth-table rows as strings, CA0..CA5 left to right.
fn row(desc: &str, ma: u8, op: u8) -> u8 {
    let mut v = 0u8;
    for (bit, t) in desc.split_whitespace().enumerate() {
        let b = match t {
            "H" => 1,
            "L" | "V" => 0,
            _ if t.starts_with("OP") => (op >> t[2..].parse::<u8>().unwrap()) & 1,
            _ if t.starts_with("MA") => (ma >> t[2..].parse::<u8>().unwrap()) & 1,
            _ if t.starts_with('C') => 0,
            _ => panic!("{t}"),
        };
        v |= b << bit;
    }
    v
}

#[test]
fn lp4_mrw_mrr_ca_encoding_litedram_truth_table() {
    // litedram/phy/lpddr4/commands.py lines 164-166, 173.
    for ma in 0..64u8 {
        for op in 0..=255u8 {
            let c = lp4cmd::mrw(ma, op).unwrap();
            assert_eq!(c[0].first, row("L H H L L OP7", ma, op));
            assert_eq!(c[0].second, row("MA0 MA1 MA2 MA3 MA4 MA5", ma, op));
            assert_eq!(c[1].first, row("L H H L H OP6", ma, op));
            assert_eq!(c[1].second, row("OP0 OP1 OP2 OP3 OP4 OP5", ma, op));
        }
        let r = lp4cmd::mrr(ma).unwrap();
        assert_eq!(r[0].first, row("L H H H L V", ma, 0));
        assert_eq!(r[0].second, ma);
        assert_eq!(r[1].first, row("L H L L H C8", ma, 0));
        assert_eq!(r[1].second, row("C2 C3 C4 C5 C6 C7", ma, 0));
    }
    assert_eq!(lp4cmd::mrw(64, 0), Err(lp4cmd::CmdError::Address));
    assert_eq!(lp4cmd::mrr(64), Err(lp4cmd::CmdError::Address));
}

#[test]
fn lp5_refresh_per_density_jedec() {
    // ramulator2 lpddr5.py lines 171-232.
    let rows = [
        (2, 130, 60, 60),
        (3, 180, 90, 90),
        (4, 180, 90, 90),
        (6, 210, 120, 90),
        (8, 210, 120, 90),
        (12, 280, 140, 90),
        (16, 280, 140, 90),
        (24, 380, 190, 90),
        (32, 380, 190, 90),
    ];
    for (gb, ab, pb, p2p) in rows {
        let r = lp5t::refresh(gb).unwrap();
        assert_eq!(
            (r.t_rfc_ab_ns, r.t_rfc_pb_ns, r.t_pbr2pbr_ns),
            (ab, pb, p2p),
            "{gb} Gb"
        );
    }
    // coreboot lp5.go (JEDEC LPDDR5/5X) agrees from 4 Gb up.
    for (gb, ab, pb) in [
        (4, 180, 90),
        (6, 210, 120),
        (8, 210, 120),
        (12, 280, 140),
        (16, 280, 140),
        (24, 380, 190),
        (32, 380, 190),
    ] {
        let r = lp5t::refresh(gb).unwrap();
        assert_eq!((r.t_rfc_ab_ns, r.t_rfc_pb_ns), (ab, pb));
    }
    assert_eq!(lp5t::refresh(5), None);
}

#[test]
fn lp5_rrd_faw_and_ccd() {
    assert_eq!(lp5t::rrd_faw(lp5t::BankMode::Bg), (5, 20));
    assert_eq!(lp5t::rrd_faw(lp5t::BankMode::B16), (5, 20));
    assert_eq!(lp5t::rrd_faw(lp5t::BankMode::B8), (10, 40));
    assert_eq!(lp5t::CCD_BG_4TO1_BL16, (2, 4));
    // gem5 LPDDR5_6400_1x16_BG_BL16: tCCD_L 5 ns at tCK 1.25 ns = 4 nCK.
    assert_eq!(5_000 / 1_250, lp5t::CCD_BG_4TO1_BL16.1);
}

#[test]
fn lp5_mr8_partial_mr26_mr30() {
    // Micron Y52Q p.29: x16, 8 Gb, LPDDR5X (8533).
    let m = lp5mr::Mr8::decode(0b00_0100_01);
    assert_eq!(
        (m.io_width, m.density_gb, m.device_type),
        (Some(lp5mr::IoWidth::X16), Some(8), 1)
    );
    // Y52P p.39 / Y42M p.34: x8, 16 Gb.
    let m = lp5mr::Mr8::decode(0b01_0110_01);
    assert_eq!(
        (m.io_width, m.density_gb),
        (Some(lp5mr::IoWidth::X8), Some(16))
    );
    assert_eq!(
        lp5mr::Mr8::decode(0b10_1001_00).density_gb,
        None,
        "reserved density codes decode to None"
    );
    assert_eq!(lp5mr::Mr8::decode(0b10_0101_00).io_width, None);
    for op in 0..=255u8 {
        assert_eq!(lp5mr::Mr26::decode(op).encode(), op);
        assert_eq!(lp5mr::Mr30::decode(op).encode(), Some(op));
    }
    let d = lp5mr::Mr26::decode(0b0010_0001);
    assert!(d.dcmu1 && d.start && !d.dcmu0 && !d.flip);
    assert_eq!(
        lp5mr::Mr30 {
            dca_upper: 16,
            dca_lower: 0
        }
        .encode(),
        None
    );
    assert_eq!(
        lp5mr::Mr30::decode(0xa5),
        lp5mr::Mr30 {
            dca_upper: 0xa,
            dca_lower: 5
        }
    );
}

#[test]
fn pending_list_no_longer_holds_the_filled_values() {
    let all: Vec<_> = jedec_lpddr::pending::LP4
        .iter()
        .chain(jedec_lpddr::pending::LP5)
        .map(|p| p.item)
        .collect();
    for filled in [
        "tRFCab and tRFCpb per density",
        "tRPab and tRPpb",
        "tXSR",
        "tCCD",
        "MRW/MRR command (CA) encoding",
    ] {
        assert!(!all.contains(&filled), "{filled} still listed");
    }
    // tXP and tWCK2CK left the list in P1-40 (only tWCK2CK's ps caps stay).
    assert!(!all.contains(&"tWCK2CK") && !all.contains(&"tXP"));
    assert!(all.contains(&"tWCK2CK and tWCK2CK_Leveling picosecond caps (TBD in JEDEC)"));
}
