// SPDX-License-Identifier: GPL-2.0-only
//! A CPU data check of the trained bus with the complex victim/aggressor
//! pattern ([`crate::complex`]), for PHYs whose training engines report
//! windows only (the K1): write the whole pattern through the controller,
//! read it back, compare per byte lane, and report every failing data bit.
//!
//! What it checks: that the settings training chose (read gate, read and
//! write Vref and delays) carry data the controller writes and reads back
//! under crosstalk, inter-symbol interference and PRBS data, on every bit.
//! It is the gate cross-check and the complex-pattern stage of the training in
//! one data test, run once per frequency point after training. A failure
//! is a training failure; the caller stops before anything is loaded into
//! DRAM.
//!
//! What it does not check: the beat order on the wires. The CPU writes
//! words; the controller forms bursts (BL16 on a 32-bit LPDDR4X bus is 64
//! bytes), so beat `t` of the pattern lands on the bus as the controller's
//! address mapping puts it, normally in order within a burst. The data
//! stress stands; the per-beat timing of the engine-driven pattern does not.
//!
//! Method: the AMD complex stages (PG313 v1.1 p357-359, cited in
//! `complex.rs`) run their pattern through the PHY's own engine; here the
//! same pattern goes through the controller from the CPU, as a memory test
//! would. The layout (byte lane `b` runs the pattern `b` victim blocks
//! ahead) is [`Complex::fill_bus`]'s.

use crate::complex::Complex;

/// A failed check.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Fail {
    /// Bit `8 * b + n` set: data bit `n` of byte lane `b` read back wrong
    /// at least once.
    pub bits: u64,
    /// Beats that read back wrong.
    pub failing_beats: u32,
    /// The first failing beat: (beat, expected, read).
    pub first: (u32, u64, u64),
}

impl Fail {
    /// Failing bits of byte lane `b`.
    pub const fn lane(&self, b: usize) -> u8 {
        (self.bits >> (8 * b)) as u8
    }
}

/// The pattern word of beat `t` on a `bytes`-wide bus (byte lane `b` in bits
/// `8b..8b+7`), byte lane `b` running `b` blocks ahead.
pub fn word(c: &Complex, bytes: usize, t: usize) -> u64 {
    let mut w = 0u64;
    for b in 0..bytes.min(8) {
        let v = c.beat(t + b * c.block()) as u8;
        w |= u64::from(v) << (8 * b);
    }
    w
}

/// Beats in one pass: the whole pattern (one block per victim lane).
pub const fn beats(c: &Complex) -> usize {
    c.len()
}

/// Write every beat with `write(beat, word)`, call `fence` once, read every
/// beat back with `read(beat)` and compare. `bytes` is the bus width in
/// bytes (1..=8; the pattern's lane count is the bits per byte lane the
/// caller chose when building `c`, normally 8).
pub fn run<E>(
    c: &Complex,
    bytes: usize,
    mut write: impl FnMut(usize, u64) -> Result<(), E>,
    mut fence: impl FnMut(),
    mut read: impl FnMut(usize) -> Result<u64, E>,
) -> Result<Result<(), Fail>, E> {
    let bytes = bytes.clamp(1, 8);
    let mask = if bytes == 8 {
        u64::MAX
    } else {
        (1u64 << (8 * bytes)) - 1
    };
    let n = beats(c);
    for t in 0..n {
        write(t, word(c, bytes, t))?;
    }
    fence();
    let mut fail: Option<Fail> = None;
    for t in 0..n {
        let want = word(c, bytes, t);
        let got = read(t)? & mask;
        let diff = (want ^ got) & mask;
        if diff != 0 {
            let f = fail.get_or_insert(Fail {
                bits: 0,
                failing_beats: 0,
                first: (t as u32, want, got),
            });
            f.bits |= diff;
            f.failing_beats += 1;
        }
    }
    Ok(fail.map_or(Ok(()), Err))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::complex::Length;

    fn model(flip: u64) -> Result<Result<(), Fail>, ()> {
        let c = Complex::new(8, Length::Long).unwrap();
        let mem = core::cell::RefCell::new([0u64; 8 * 224]);
        run(
            &c,
            4,
            |t, w| {
                mem.borrow_mut()[t] = w;
                Ok(())
            },
            || {},
            |t| Ok(mem.borrow()[t] ^ flip),
        )
    }

    #[test]
    fn a_faithful_bus_passes_and_the_pattern_covers_every_bit() {
        assert_eq!(model(0), Ok(Ok(())));
        let c = Complex::new(8, Length::Long).unwrap();
        assert_eq!(beats(&c), 8 * 224);
        let (mut ones, mut zeros) = (0u64, 0u64);
        for t in 0..beats(&c) {
            let w = word(&c, 4, t);
            ones |= w;
            zeros |= !w & 0xffff_ffff;
        }
        assert_eq!(ones, 0xffff_ffff);
        assert_eq!(zeros, 0xffff_ffff);
        // Byte lanes run different victims at a beat (no idle lane).
        let w = word(&c, 4, 0);
        assert_ne!(w & 0xff, (w >> 8) & 0xff);
    }

    #[test]
    fn a_stuck_bit_names_its_byte_lane_and_bit() {
        let f = model(1 << 21).unwrap().unwrap_err();
        assert_eq!(f.bits, 1 << 21);
        assert_eq!(f.lane(2), 1 << 5);
        assert_eq!(f.lane(0), 0);
        assert_eq!(
            f.failing_beats as usize,
            beats(&Complex::new(8, Length::Long).unwrap())
        );
        assert_eq!(f.first.0, 0);
        assert_eq!(f.first.1 ^ f.first.2, 1 << 21);
        // Bits outside the bus width are masked off.
        assert_eq!(model(1 << 40), Ok(Ok(())));
    }

    #[test]
    fn access_errors_propagate() {
        let c = Complex::new(8, Length::Short).unwrap();
        let r: Result<Result<(), Fail>, &str> = run(&c, 4, |_, _| Err("bus"), || {}, |_| Ok(0));
        assert_eq!(r, Err("bus"));
    }
}
