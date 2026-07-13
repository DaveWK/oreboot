// SPDX-License-Identifier: GPL-2.0-only
//! PHY loopback and PRBS checks: DRAM-free instruments.
//!
//! With the PHY's transmit path looped back to its own receivers (pad- or
//! core-side loopback), the transmit delay lines can be measured with no
//! DRAM in the loop:
//! - [`sweep`] / [`knob_check`]: does moving a transmit delay move the
//!   edge the receiver sees? A flat sweep is a dead knob (the register is
//!   not the live copy, or the lane is not driven).
//! - [`period`]: sweep the transmit delay against a fixed receive sampling
//!   point while a clock-like pattern toggles; two consecutive rising edges
//!   are one period apart, which calibrates the delay line
//!   ([`crate::ruler::Ruler::from_edges`]).
//! - [`Prbs`], [`Generator`], [`Checker`]: the standard PRBS sequences
//!   and a self-synchronising checker (it seeds itself from the first
//!   received bits, so it needs no alignment with the generator) for a
//!   pad or delay-line self-test before DRAM init; a single wrong bit
//!   counts once per feedback tap plus one.
//!
//! The PHY registers (loopback enables and modes, the PRBS checker and its
//! error counters, the tristate overrides that keep a strobe driven) are
//! board code. Source: ALTERNATIVE-APPROACHES (prior art, orchestrator)
//! section 6.1; the PRBS polynomials are the ITU-T O.150 ones.

use crate::gate::{classify, Class};

/// The PRBS polynomials x^n + x^m + 1.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Prbs {
    P7,
    P15,
    P23,
    P31,
}

impl Prbs {
    /// (n, m) of x^n + x^m + 1.
    pub const fn taps(self) -> (u32, u32) {
        match self {
            Prbs::P7 => (7, 6),
            Prbs::P15 => (15, 14),
            Prbs::P23 => (23, 18),
            Prbs::P31 => (31, 28),
        }
    }
    /// Sequence period, 2^n - 1.
    pub const fn period(self) -> u64 {
        (1u64 << self.taps().0) - 1
    }
}

/// A PRBS generator (Fibonacci form) from a non-zero seed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Generator {
    prbs: Prbs,
    state: u32,
}

impl Generator {
    pub fn new(prbs: Prbs, seed: u32) -> Self {
        let n = prbs.taps().0;
        let mask = if n == 32 { u32::MAX } else { (1u32 << n) - 1 };
        let s = seed & mask;
        Self {
            prbs,
            state: if s == 0 { 1 } else { s },
        }
    }
    pub fn next_bit(&mut self) -> bool {
        let (n, m) = self.prbs.taps();
        let b = ((self.state >> (n - 1)) ^ (self.state >> (m - 1))) & 1;
        let mask = (1u32 << n) - 1;
        self.state = ((self.state << 1) | b) & mask;
        b != 0
    }
}

/// A self-synchronising PRBS checker: the first n bits seed it, after
/// that each bit is predicted from the n bits before it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Checker {
    prbs: Prbs,
    history: u32,
    seeded: u32,
    pub bits: u64,
    pub errors: u64,
}

impl Checker {
    pub fn new(prbs: Prbs) -> Self {
        Self {
            prbs,
            history: 0,
            seeded: 0,
            bits: 0,
            errors: 0,
        }
    }
    /// Feed one received bit.
    pub fn push(&mut self, bit: bool) {
        let (n, m) = self.prbs.taps();
        let mask = (1u32 << n) - 1;
        if self.seeded >= n {
            let want = ((self.history >> (n - 1)) ^ (self.history >> (m - 1))) & 1 != 0;
            self.bits += 1;
            if want != bit {
                self.errors += 1;
            }
        } else {
            self.seeded += 1;
        }
        self.history = ((self.history << 1) | bit as u32) & mask;
    }
    /// The checker has seen enough bits to predict.
    pub fn locked(&self) -> bool {
        self.seeded >= self.prbs.taps().0
    }
}

/// The board's loopback: one lane's transmit delay and the receiver's
/// samples of a toggling pattern at a fixed receive point.
pub trait Loopback {
    type Error;
    fn set_tx_delay(&mut self, lane: u8, taps: u32) -> Result<(), Self::Error>;
    /// Ones out of `n` samples at the current setting.
    fn sample(&mut self, lane: u8, n: u16) -> Result<u16, Self::Error>;
}

/// Sweep `lane`'s transmit delay over `0..out.len()` taps, classifying each
/// point (stable 0, stable 1, noise) from `n` samples.
pub fn sweep<L: Loopback>(lb: &mut L, lane: u8, n: u16, out: &mut [Class]) -> Result<(), L::Error> {
    for (t, c) in out.iter_mut().enumerate() {
        lb.set_tx_delay(lane, t as u32)?;
        *c = classify(lb.sample(lane, n)?, n);
    }
    Ok(())
}

/// The taps (index of the first stable 1 after a stable 0) of every
/// rising edge in a sweep, into `out`; returns how many.
pub fn rising_edges(sweep: &[Class], out: &mut [u32]) -> usize {
    let mut k = 0;
    let mut last = None;
    for (t, &c) in sweep.iter().enumerate() {
        match c {
            Class::Zero => last = Some(Class::Zero),
            Class::One => {
                if last == Some(Class::Zero) && k < out.len() {
                    out[k] = t as u32;
                    k += 1;
                }
                last = Some(Class::One);
            }
            Class::Noise => {}
        }
    }
    k
}

/// Whether the knob moves anything: a sweep with at least one stable 0 and
/// one stable 1 (Ok), or a flat one ([`crate::sanity::SanityError::DeadKnob`]).
pub fn knob_check(sweep: &[Class]) -> Result<(), crate::sanity::SanityError> {
    let zero = sweep.contains(&Class::Zero);
    let one = sweep.contains(&Class::One);
    if zero && one {
        Ok(())
    } else {
        Err(crate::sanity::SanityError::DeadKnob)
    }
}

/// Two consecutive rising edges of a clock-like loopback pattern: the
/// taps of one period (first, second), for
/// [`crate::ruler::Ruler::from_edges`]. None if the sweep holds fewer than
/// two rising edges.
pub fn period(sweep: &[Class]) -> Option<(u32, u32)> {
    let mut e = [0u32; 2];
    (rising_edges(sweep, &mut e) == 2).then_some((e[0], e[1]))
}
