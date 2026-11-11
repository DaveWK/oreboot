// SPDX-License-Identifier: GPL-2.0-only
//! OS_REG handoff decode/encode (crate::osreg, ported from U-Boot).
use rk3588::osreg::{self, Geometry};

fn lp4x_1g_channel() -> Geometry {
    // 8 Gb per 16-bit channel: 16 rows, 10 columns, 8 banks = 1 GiB.
    Geometry {
        rank: 1,
        col: 10,
        bk: 3,
        cs0_row: 16,
        cs1_row: 16,
        bw: 1,
        dbw: 1,
    }
}

#[test]
fn round_trip_4gib_board() {
    let g = lp4x_1g_channel();
    let (r2, r3) = osreg::encode(osreg::LPDDR4X, &[g, g]);
    let p = osreg::decode(r2, r3);
    assert_eq!(p.version, 3);
    assert_eq!(p.dram_type, osreg::LPDDR4X);
    assert_eq!(p.channels, 2);
    assert_eq!(p.size_mib(), 2048);
    assert!(osreg::plausible(&p));
    assert_eq!(p.ch[0].cs0_row, 16);
    assert_eq!(p.ch[0].bw, 1);
}

#[test]
fn two_ranks_and_17_rows() {
    // 16 Gb per channel (17 rows), two ranks: 4 GiB per channel.
    let g = Geometry {
        rank: 2,
        col: 10,
        bk: 3,
        cs0_row: 17,
        cs1_row: 17,
        bw: 1,
        dbw: 1,
    };
    let (r2, r3) = osreg::encode(osreg::LPDDR4X, &[g, g]);
    let p = osreg::decode(r2, r3);
    assert_eq!(p.ch[1].rank, 2);
    assert_eq!(p.ch[1].cs0_row, 17);
    assert_eq!(p.size_mib(), 8192);
}

#[test]
fn hand_decoded_word() {
    // reg2 written by hand from the U-Boot field list: one channel pair,
    // ch0 and ch1 each rank 1 (0), col 10 (1), 8 banks (0), cs0_row 16
    // (code 3), cs1_row 16 (3), bw 16 bit (1), dbw 1; type LPDDR4X = 8:
    // low bits 0, extended bits 1 in reg3 [13:12]; two channels (bit 12);
    // chinfo [29:28] = both channels described.
    let ch = (1 << 9) | (3 << 6) | (3 << 4) | (1 << 2) | 1;
    let reg2 = ch | (ch << 16) | (1 << 12) | (3 << 28);
    let reg3 = (3 << 28) | (1 << 12) | (1 << 2) | 1;
    let p = osreg::decode(reg2, reg3);
    assert_eq!(osreg::type_name(p.dram_type), "LPDDR4X");
    assert_eq!(p.size_mib(), 2048);
    assert_eq!(
        osreg::encode(osreg::LPDDR4X, &[lp4x_1g_channel(), lp4x_1g_channel()]),
        (reg2, reg3)
    );
}

#[test]
fn cold_boot_zeros_are_not_a_handoff() {
    assert!(!osreg::plausible(&osreg::decode(0, 0)));
    assert!(!osreg::plausible(&osreg::decode(0xffff_ffff, 0xffff_ffff)));
}

#[test]
fn display_is_parseable() {
    let g = lp4x_1g_channel();
    let (r2, r3) = osreg::encode(osreg::LPDDR4X, &[g, g]);
    let text = format!("{}", osreg::decode(r2, r3));
    let first = text.lines().next().unwrap();
    assert!(first.starts_with("osreg version=3 type=LPDDR4X channels=2 size_mib=2048"));
    for line in text.lines() {
        for kv in line.split_whitespace().skip(1) {
            assert!(kv.contains('='), "{kv}");
        }
    }
}

/// The YY3588 (first hardware run): what the rkbin DDR init left in
/// OS_REG2..5 (both pairs equal), decoded, and our encoder producing the
/// same words from the identified geometry: LPDDR4X, two channels per
/// pair, 2 ranks, 10 columns, 8 banks, 18 rows on both ranks, 16-bit
/// channels, x8 dies (DBW field 2). The words are a handoff format and
/// identification, not training data.
#[test]
fn yy3588_words_round_trip() {
    let (r2, r3) = (0x3a56_1a56, 0x3000_10f5);
    let p = osreg::decode(r2, r3);
    assert_eq!((p.version, p.dram_type, p.channels), (3, osreg::LPDDR4X, 2));
    for c in &p.ch {
        assert_eq!(
            (c.rank, c.cs0_col, c.cs1_col, c.bk, c.cs0_row, c.cs1_row, c.bw, c.dbw),
            (2, 10, 10, 3, 18, 18, 1, osreg::DBW_X8)
        );
        assert!(!c.row_3_4);
        assert_eq!(c.size_mib, 8192);
    }
    assert_eq!(p.size_mib(), 16384);
    assert!(osreg::plausible(&p));
    let g = Geometry {
        rank: 2,
        col: 10,
        bk: 3,
        cs0_row: 18,
        cs1_row: 18,
        bw: 1,
        dbw: osreg::DBW_X8,
    };
    assert_eq!(osreg::encode(osreg::LPDDR4X, &[g, g]), (r2, r3));
}
