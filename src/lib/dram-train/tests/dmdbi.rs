// SPDX-License-Identifier: GPL-2.0-only
//! P3-M5: the DM check against mask faults, the DBI rule and patterns, DBI
//! deskewed as a ninth lane, and the combined DQ+DBI window.

use dram_train::deskew::{per_bit, Deskew, Params, Target};
use dram_train::dmdbi::{
    combined_window, dbi_decode, dbi_encode, dm_classify, dm_expected, dm_masked, wire_image,
    DbiRule, Dm, DBI_BIT, DBI_TOGGLE, DM_FIRST, DM_SECOND,
};

#[derive(Clone, Copy, Debug)]
enum Mask {
    Works,
    Ignored,
    OtherEdge,
    Stuck,
    SecondLost,
    BadBit,
}

/// Run the DM check on a 4-lane bus whose lane 2 has `fault`.
fn dm_run(fault: Mask) -> [Dm; 4] {
    let (bytes, beats) = (4, 16);
    let mut mem = vec![0u8; bytes * beats];
    // First write: DM off.
    mem.fill(DM_FIRST);
    // Second write: DM on the masked beats.
    for t in 0..beats {
        for b in 0..bytes {
            let masked = dm_masked(t);
            let (apply, data) = match (b, fault) {
                (2, Mask::Ignored) => (true, DM_SECOND),
                (2, Mask::OtherEdge) => (masked, DM_SECOND),
                (2, Mask::Stuck) | (2, Mask::SecondLost) => (false, DM_SECOND),
                (2, Mask::BadBit) => (!masked, DM_SECOND ^ 0x08),
                _ => (!masked, DM_SECOND),
            };
            if apply {
                mem[t * bytes + b] = data;
            }
        }
    }
    let mut out = [Dm::Data; 4];
    assert!(dm_classify(&mem, bytes, beats, &mut out));
    out
}

#[test]
fn the_dm_check_names_each_mask_fault() {
    for (fault, want) in [
        (Mask::Works, Dm::Ok),
        (Mask::Ignored, Dm::Ignored),
        (Mask::OtherEdge, Dm::Inverted),
        (Mask::Stuck, Dm::AllMasked),
        (Mask::SecondLost, Dm::AllMasked),
        (Mask::BadBit, Dm::Data),
    ] {
        let r = dm_run(fault);
        assert_eq!(r[2], want, "{fault:?}");
        for b in [0, 1, 3] {
            assert_eq!(r[b], Dm::Ok, "{fault:?} lane {b}");
        }
    }
    assert_eq!(dm_expected(0), DM_FIRST);
    assert_eq!(dm_expected(1), DM_SECOND);
    assert!(!dm_classify(&[0; 8], 4, 4, &mut [Dm::Ok; 4]));
}

#[test]
fn dbi_round_trips_and_keeps_the_scarce_value_scarce() {
    for rule in [DbiRule::Ones, DbiRule::Zeros] {
        for x in 0..=255u8 {
            let (w, d) = dbi_encode(x, rule);
            assert_eq!(dbi_decode(w, d), x);
            let scarce = match rule {
                DbiRule::Ones => w.count_ones(),
                DbiRule::Zeros => w.count_zeros(),
            };
            assert!(scarce <= 4, "{rule:?} {x:#x}");
        }
    }
    assert_eq!(dbi_encode(0x1f, DbiRule::Ones), (0xe0, true));
    assert_eq!(dbi_encode(0x0f, DbiRule::Ones), (0x0f, false));
}

#[test]
fn the_toggle_pattern_moves_only_dbi() {
    for rule in [DbiRule::Ones, DbiRule::Zeros] {
        let logical: Vec<u8> = (0..16).map(|t| DBI_TOGGLE[t % 2]).collect();
        let mut wire = [0u8; 16];
        let dbi = wire_image(&logical, &mut wire, rule);
        assert!(wire.iter().all(|&w| w == wire[0]), "{rule:?}: DQ still");
        for t in 1..16 {
            assert_ne!(dbi >> t & 1, dbi >> (t - 1) & 1, "{rule:?}: DBI toggles");
        }
    }
}

/// Nine lanes (8 DQ + DBI) with skewed edges, as in the deskew plant.
struct Lanes {
    edge: [u16; 9],
    delay: [u16; 9],
    strobe: u16,
}
impl Target for Lanes {
    type Error = ();
    fn set_strobe(&mut self, d: u16) -> Result<(), ()> {
        self.strobe = d;
        Ok(())
    }
    fn set_bit(&mut self, b: u8, d: u16) -> Result<(), ()> {
        self.delay[b as usize] = d;
        Ok(())
    }
    fn passes(&mut self, b: u8) -> Result<bool, ()> {
        Ok(self.strobe >= self.edge[b as usize] + self.delay[b as usize])
    }
}

#[test]
fn dbi_deskews_as_the_ninth_lane_and_can_be_the_latest() {
    let mut l = Lanes {
        edge: [10, 12, 9, 11, 10, 13, 12, 11, 17],
        delay: [0; 9],
        strobe: 0,
    };
    let p = Params {
        bits: 9,
        strobe_start: 0,
        strobe_max: 100,
        bit_max: 63,
        confirm: 0,
    };
    let r: Deskew<9> = per_bit(&mut l, &p).unwrap();
    assert_eq!(r.strobe, 17);
    assert_eq!(r.latest, Some(DBI_BIT));
    for b in 0..9 {
        assert_eq!(l.edge[b] + r.bit[b], 17);
    }
}

#[test]
fn the_strobe_sits_in_the_combined_window() {
    assert_eq!(combined_window((10, 50), (20, 60)), Some((20, 50, 35)));
    assert_eq!(combined_window((10, 50), (10, 50)), Some((10, 50, 30)));
    assert_eq!(combined_window((10, 20), (30, 60)), None);
}
