// SPDX-License-Identifier: GPL-2.0-only
//! Per-bit centre alignment with the re-measure hook, on a linear
//! plant (a bit's window moves exactly with its delay), a plant with a
//! weak delay line (each tap moves the window less than a tap, so the
//! first move falls short and the re-measure round corrects it), one with
//! interaction between bits, and one that saturates.

use dram_train::deskew::{per_bit_centre, CentreError, CentreParams, Centred, Measure};

/// Window of bit b at delay 0 is `base[b]`; its delay moves it by
/// `gain` taps per tap (fixed-point, 1.0 = 256); `couple` shifts every
/// window by that fraction of the mean of the other bits' delays.
struct Plant {
    base: Vec<(i32, i32)>,
    delay: Vec<u16>,
    gain: i32,
    couple: i32,
    windows_measured: u32,
    dead: Option<u8>,
}

impl Plant {
    fn new(base: &[(i32, i32)]) -> Plant {
        Plant {
            base: base.to_vec(),
            delay: vec![0; base.len()],
            gain: 256,
            couple: 0,
            windows_measured: 0,
            dead: None,
        }
    }
    fn shift(&self, bit: usize) -> i32 {
        let own = i32::from(self.delay[bit]) * self.gain / 256;
        let others: i32 = self
            .delay
            .iter()
            .enumerate()
            .filter(|(i, _)| *i != bit)
            .map(|(_, d)| i32::from(*d))
            .sum();
        let n = (self.delay.len() as i32 - 1).max(1);
        own + others / n * self.couple / 256
    }
}

impl Measure for Plant {
    type Error = &'static str;
    fn window(&mut self, bit: u8) -> Result<Option<(i32, i32)>, &'static str> {
        self.windows_measured += 1;
        if self.dead == Some(bit) {
            return Ok(None);
        }
        let (lo, hi) = self.base[usize::from(bit)];
        let s = self.shift(usize::from(bit));
        Ok(Some((lo + s, hi + s)))
    }
    fn set_bit(&mut self, bit: u8, delay: u16) -> Result<(), &'static str> {
        self.delay[usize::from(bit)] = delay;
        Ok(())
    }
}

const P: CentreParams = CentreParams {
    bits: 4,
    bit_max: 63,
    tolerance: 0,
    rounds: 2,
};

fn centres<const N: usize>(c: &Centred<N>, n: usize) -> Vec<i32> {
    c.bits[..n].iter().map(|b| b.after).collect()
}

#[test]
fn linear_plant_lines_up_in_one_move() {
    // Centres 10, 14, 7, 12 -> target 14 (the latest bit, which stays put).
    let mut pl = Plant::new(&[(5, 15), (9, 19), (3, 11), (8, 16)]);
    let c: Centred<8> = per_bit_centre(&mut pl, &P, &[0; 4]).unwrap();
    assert_eq!(c.target, 14);
    assert_eq!(centres(&c, 4), [14, 14, 14, 14]);
    assert_eq!(pl.delay, [4, 0, 7, 2]);
    assert_eq!(c.bits[1].delay, 0, "the latest bit is not moved");
    assert_eq!((c.saturated, c.unverified, c.rounds), (0, 0, 0));
    assert!(c.bits[..4].iter().all(|b| b.verified));
    assert_eq!(c.min_width, 8);
    // One measurement per bit before, one after (the hook): 8 in all.
    assert_eq!(pl.windows_measured, 8);
}

#[test]
fn a_weak_delay_line_is_corrected_by_the_re_measure_round() {
    // 0.75 tap per tap: a bit 8 taps short moves 6 on the first try.
    let mut pl = Plant::new(&[(0, 10), (8, 18)]);
    pl.gain = 192;
    let p = CentreParams { bits: 2, ..P };
    let c: Centred<2> = per_bit_centre(&mut pl, &p, &[0, 0]).unwrap();
    assert_eq!(c.target, 13);
    assert_eq!(centres(&c, 2), [13, 13]);
    // 8 taps -> 6 (centre 11), +2 -> 10 taps -> 7 (centre 12), +1 -> 11
    // taps -> 8 (centre 13): two correction rounds.
    assert_eq!(c.rounds, 2);
    assert_eq!(pl.delay[0], 11);
    assert_eq!(c.unverified, 0);
    // With no rounds allowed the bit stays short and is reported.
    let mut pl = Plant::new(&[(0, 10), (8, 18)]);
    pl.gain = 192;
    let p0 = CentreParams { rounds: 0, ..p };
    let c: Centred<2> = per_bit_centre(&mut pl, &p0, &[0, 0]).unwrap();
    assert_eq!(c.unverified, 0b01);
    assert!(!c.bits[0].verified && c.bits[1].verified);
    assert_eq!(c.bits[0].after, 11);
    // A tolerance of 2 accepts it.
    let mut pl = Plant::new(&[(0, 10), (8, 18)]);
    pl.gain = 192;
    let pt = CentreParams { tolerance: 2, ..p0 };
    let c: Centred<2> = per_bit_centre(&mut pl, &pt, &[0, 0]).unwrap();
    assert_eq!(c.unverified, 0);
}

#[test]
fn interaction_between_bits_converges_within_the_rounds() {
    // Each bit's window also moves by a quarter of the others' mean delay.
    let mut pl = Plant::new(&[(0, 10), (20, 30), (4, 14), (10, 20)]);
    pl.couple = 64;
    let p = CentreParams {
        tolerance: 1,
        rounds: 6,
        ..P
    };
    let c: Centred<4> = per_bit_centre(&mut pl, &p, &[0; 4]).unwrap();
    // The latest bit itself drifts later as the others are delayed, so the
    // target grows past the first measurement's 25 and the bits follow it.
    assert!(c.target >= 25, "{c:?}");
    for b in &c.bits {
        assert!((b.after - c.target).abs() <= 1, "{c:?}");
    }
    assert_eq!(c.unverified, 0);
    assert!(c.rounds >= 1 && c.rounds <= 6, "{}", c.rounds);
    assert_eq!(pl.delay[1], 0, "the latest bit is never moved");
}

#[test]
fn saturation_and_errors_are_reported() {
    let mut pl = Plant::new(&[(0, 10), (40, 50)]);
    let p = CentreParams {
        bits: 2,
        bit_max: 20,
        ..P
    };
    let c: Centred<2> = per_bit_centre(&mut pl, &p, &[0, 0]).unwrap();
    assert_eq!(c.saturated, 0b01);
    assert_eq!(c.bits[0].delay, 20);
    assert_eq!(c.bits[0].after, 25);
    assert!(!c.bits[0].verified);
    assert_eq!(c.unverified, 0b01);
    // Start delays are kept and set first.
    let mut pl = Plant::new(&[(0, 10), (0, 10)]);
    let c: Centred<2> = per_bit_centre(&mut pl, &CentreParams { bits: 2, ..P }, &[3, 9]).unwrap();
    assert_eq!(c.target, 14);
    assert_eq!(pl.delay, [9, 9]);
    assert_eq!((c.bits[0].before, c.bits[0].after), (8, 14));
    // Config errors.
    let mut pl = Plant::new(&[(0, 10), (0, 10)]);
    assert_eq!(
        per_bit_centre::<_, 1>(&mut pl, &CentreParams { bits: 2, ..P }, &[0, 0]).err(),
        Some(CentreError::Config)
    );
    assert_eq!(
        per_bit_centre::<_, 2>(&mut pl, &CentreParams { bits: 2, ..P }, &[0]).err(),
        Some(CentreError::Config)
    );
    assert_eq!(
        per_bit_centre::<_, 2>(&mut pl, &CentreParams { bits: 2, ..P }, &[0, 64]).err(),
        Some(CentreError::Config)
    );
    // A bit with no window.
    pl.dead = Some(1);
    assert_eq!(
        per_bit_centre::<_, 2>(&mut pl, &CentreParams { bits: 2, ..P }, &[0, 0]).err(),
        Some(CentreError::NoWindow(1))
    );
}
