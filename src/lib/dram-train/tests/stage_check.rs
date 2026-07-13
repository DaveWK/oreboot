// SPDX-License-Identifier: GPL-2.0-only
//! P3-M2: the per-stage sanity check (fixed word per stage, negative check,
//! failing-nibble record) against fault plants.

use dram_train::result::ErrorCode;
use dram_train::stage_check::{
    byte_at, check, compare, fill, negative, word, CheckError, BEATS, KEPT, STAGE_WORDS,
};

/// A bus with optional stuck data bits, a one-beat slip, or a compare that
/// is stuck passing (the read returns what was written whatever happens).
#[derive(Default)]
struct Bus {
    stuck: Vec<(usize, u8, bool)>, // (byte lane, bit, value)
    slip: bool,
    bad: bool,
    stuck_passing: bool,
    restored: u32,
}
impl Bus {
    fn write_read(&mut self, bytes: usize, exp: &[u8], got: &mut [u8]) -> Result<(), ()> {
        got.copy_from_slice(exp);
        if self.stuck_passing {
            return Ok(());
        }
        if self.slip || self.bad {
            // One beat late: beat t reads what beat t-1 wrote.
            for t in (1..BEATS).rev() {
                for b in 0..bytes {
                    got[t * bytes + b] = exp[(t - 1) * bytes + b];
                }
            }
        }
        for &(b, bit, v) in &self.stuck {
            for t in 0..BEATS {
                let x = &mut got[t * bytes + b];
                *x = *x & !(1 << bit) | (u8::from(v) << bit);
            }
        }
        Ok(())
    }
}

#[test]
fn each_stage_and_rank_has_its_own_word() {
    let mut seen = std::collections::HashSet::new();
    for s in 0..5 {
        for r in 0..4 {
            assert!(seen.insert(word(s, r)), "stage {s} rank {r}");
        }
    }
    assert_eq!(word(5, 0), word(0, 0), "stages cycle");
    assert_eq!(word(2, 0), STAGE_WORDS[2]);
    for w in STAGE_WORDS {
        // Every nibble bit is 0 on some beat and 1 on another (any stuck
        // data bit fails), and a one-beat slip changes the word.
        for j in 0..4 {
            let vals: Vec<u32> = (0..8).map(|t| (w >> (4 * t + j)) & 1).collect();
            assert!(vals.contains(&0) && vals.contains(&1), "{w:#x} bit {j}");
        }
        assert_ne!(w.rotate_left(4), w);
        for t in 0..BEATS {
            let b = byte_at(w, t);
            assert_eq!(b >> 4, !b & 0xf, "high nibble is the inverse");
        }
    }
}

#[test]
fn a_good_path_passes_for_every_bus_width() {
    for bytes in 1..=8 {
        for stage in 0..5 {
            let (mut e, mut g) = ([0u8; 64], [0u8; 64]);
            let mut bus = Bus::default();
            let c = check(stage, 1, bytes, &mut e, &mut g, |x, y| {
                bus.write_read(bytes, x, y)
            })
            .unwrap();
            assert!(c.passed());
        }
    }
    let (mut e, mut g) = ([0u8; 16], [0u8; 64]);
    assert_eq!(
        check(0, 0, 4, &mut e, &mut g, |_, _| Ok::<(), ()>(())),
        Err(CheckError::Config)
    );
    assert_eq!(fill(&mut [0u8; 64], 9, 0, 0), None);
}

#[test]
fn a_stuck_bit_is_named_down_to_the_bit() {
    for (lane, bit, v) in [(0usize, 0u8, true), (2, 5, false), (3, 7, true)] {
        let bytes = 4;
        let (mut e, mut g) = ([0u8; 32], [0u8; 32]);
        let mut bus = Bus {
            stuck: vec![(lane, bit, v)],
            ..Default::default()
        };
        let r = check(3, 0, bytes, &mut e, &mut g, |x, y| {
            bus.write_read(bytes, x, y)
        });
        let Err(CheckError::Mismatch(c)) = r else {
            panic!("stuck bit passed")
        };
        let nib = (2 * lane + usize::from(bit >= 4)) as u8;
        assert_eq!(c.nibbles, 1 << nib);
        assert_eq!(c.bits, 1u64 << (8 * lane + bit as usize));
        assert_eq!(c.kept, 1);
        assert_eq!(c.first[0].nibble, nib);
        assert_ne!(c.first[0].expected, c.first[0].got);
        let rec = c.error(7, 1, 0, ErrorCode::PostCheck).unwrap();
        assert_eq!(
            (rec.stage, rec.ch, rec.byte, rec.bit, rec.code),
            (7, 1, lane as u8, Some(bit), ErrorCode::PostCheck)
        );
    }
}

#[test]
fn the_first_three_failing_nibbles_are_kept_and_all_are_mapped() {
    let bytes = 8;
    let mut e = [0u8; 64];
    fill(&mut e, bytes, 1, 0).unwrap();
    let mut g = e;
    for b in [1usize, 2, 4, 6, 7] {
        g[3 * bytes + b] ^= 0x10; // high nibble of byte lane b, beat 3
    }
    let c = compare(&e, &g, bytes);
    assert_eq!(
        c.nibbles,
        (1 << 3) | (1 << 5) | (1 << 9) | (1 << 13) | (1 << 15)
    );
    assert_eq!(c.kept, KEPT);
    assert_eq!(c.first.map(|f| f.nibble), [3, 5, 9], "lowest nibbles first");
    for f in c.first {
        assert_eq!(f.expected ^ f.got, 1 << 12, "beat 3, bit 0 of the nibble");
    }
    // Two bits of one byte: no single bit named.
    let mut g2 = e;
    g2[0] ^= 0x03;
    let c = compare(&e, &g2, bytes);
    assert_eq!(c.error(0, 0, 0, ErrorCode::PostCheck).unwrap().bit, None);
}

#[test]
fn a_beat_slip_fails_every_stage() {
    for stage in 0..5 {
        let (mut e, mut g) = ([0u8; 16], [0u8; 16]);
        let mut bus = Bus {
            slip: true,
            ..Default::default()
        };
        let r = check(stage, 0, 2, &mut e, &mut g, |x, y| bus.write_read(2, x, y));
        assert!(matches!(r, Err(CheckError::Mismatch(c)) if c.nibbles == 0b1111));
    }
}

#[test]
fn the_negative_check_needs_a_failing_bad_setting_and_always_restores() {
    use std::cell::RefCell;
    let bus = RefCell::new(Bus::default());
    let set = |on: bool| -> Result<(), ()> {
        let mut b = bus.borrow_mut();
        b.bad = on;
        if !on {
            b.restored += 1;
        }
        Ok(())
    };
    let wr = |x: &[u8], y: &mut [u8]| bus.borrow_mut().write_read(4, x, y);
    let (mut e, mut g) = ([0u8; 32], [0u8; 32]);
    let c = negative(2, 0, 4, &mut e, &mut g, set, wr).unwrap();
    assert!(!c.passed());
    assert_eq!(bus.borrow().restored, 1);
    assert!(!bus.borrow().bad);

    // A compare stuck passing (or a dead knob): the negative check says so.
    bus.borrow_mut().stuck_passing = true;
    let set = |on: bool| -> Result<(), ()> {
        let mut b = bus.borrow_mut();
        b.bad = on;
        if !on {
            b.restored += 1;
        }
        Ok(())
    };
    let wr = |x: &[u8], y: &mut [u8]| bus.borrow_mut().write_read(4, x, y);
    assert_eq!(
        negative(2, 0, 4, &mut e, &mut g, set, wr),
        Err(CheckError::NegativePassed)
    );
    assert_eq!(bus.borrow().restored, 2);

    // An access error still restores.
    let restored = RefCell::new(0);
    let r = negative(
        0,
        0,
        4,
        &mut e,
        &mut g,
        |on| {
            if !on {
                *restored.borrow_mut() += 1;
            }
            Ok(())
        },
        |_, _| Err("timeout"),
    );
    assert_eq!(r, Err(CheckError::Io("timeout")));
    assert_eq!(*restored.borrow(), 1);
}
