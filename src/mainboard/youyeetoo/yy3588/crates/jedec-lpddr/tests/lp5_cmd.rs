// SPDX-License-Identifier: GPL-2.0-only
//! lp5::cmd against the A733 board code it replaces and the cited truth table.

#[path = "legacy/a733_cmd.rs"]
mod old;

use jedec_lpddr::lp5::cmd::{self, CaEdges, CmdError};

#[test]
fn mrw_matches_a733_payload_and_rows() {
    for ma in 0..=127u8 {
        for op in 0..=255u8 {
            let [w1, w2] = cmd::mrw(ma, op).unwrap();
            for cs in 1..=3u8 {
                let p = old::MrWritePayload::new(ma, op, cs).unwrap().fields();
                assert_eq!((p[0].1 & 0x7f) as u8, w1.rise);
                assert_eq!(((p[0].1 >> 7) & 0x7f) as u8, w1.fall);
                assert_eq!((p[1].1 & 0x7f) as u8, w2.rise);
                assert_eq!(((p[1].1 >> 7) & 0x7f) as u8, w2.fall);
                let rows = old::mrw(cs, ma, op);
                let ca = |r: &old::Row| (r.0[0] & 0x7f) as u8;
                assert_eq!(
                    [ca(&rows[0]), ca(&rows[1]), ca(&rows[2]), ca(&rows[3])],
                    [w1.rise, w1.fall, w2.rise, w2.fall]
                );
                let ddr = old::mrw_ddr(cs, ma, op);
                let lohi = |r: &old::Row| ((r.0[0] & 0x7f) as u8, ((r.0[0] >> 7) & 0x7f) as u8);
                assert_eq!(lohi(&ddr[0]), (w1.rise, w1.fall));
                assert_eq!(lohi(&ddr[1]), (w2.rise, w2.fall));
            }
        }
    }
}

#[test]
fn address_range() {
    assert_eq!(cmd::mrw(128, 0), Err(CmdError::Address));
    assert_eq!(cmd::mrr(128), Err(CmdError::Address));
    assert!(old::MrWritePayload::new(128, 0, 1).is_err());
}

/// LiteDRAM 74522f715d `litedram/phy/lpddr5/commands.py` truth-table strings
/// (CA0..CA6, rising edge) for the commands this module encodes.
fn rise_from(table: &str, op7: bool) -> u8 {
    let mut v = 0;
    for (bit, t) in table.split_whitespace().enumerate() {
        let high = match t {
            "H" => true,
            "L" => false,
            "OP7" => op7,
            other => panic!("unexpected {other}"),
        };
        v |= (high as u8) << bit;
    }
    v
}

#[test]
fn rising_edges_match_the_cited_truth_table() {
    assert_eq!(rise_from("L L L H H L H", false), cmd::MRW1);
    assert_eq!(rise_from("L L L H H L L", false), cmd::MRR);
    for op in [0u8, 0x7f, 0x80, 0xff] {
        let w2 = cmd::mrw(5, op).unwrap()[1];
        assert_eq!(rise_from("L L L H L L OP7", op & 0x80 != 0), w2.rise);
        assert_eq!(w2.fall, op & 0x7f);
    }
    assert_eq!(
        cmd::mrr(0x2a),
        Ok(CaEdges {
            rise: 0x18,
            fall: 0x2a
        })
    );
}
