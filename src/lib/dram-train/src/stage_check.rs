// SPDX-License-Identifier: GPL-2.0-only
//! Sanity check after every stage: write and read back a fixed pattern that
//! differs per stage (so a stage that only works for the previous stage's
//! data is caught), check that a known-bad setting fails (the compare is
//! not stuck passing), and on a failure keep the expected and read data of
//! the first failing nibbles and a bitmap of all failing nibbles.
//!
//! Method: AMD PG150 v1.4 p887-896 (a different fixed pattern after each
//! stage; the expected and read data of the first three failing nibbles and
//! a failing-nibble bitmap) and PG353 v1.0 p236 (a negative sanity check),
//! cited, not copied. The five stage words are the ones PG150 names; the
//! rank variants (a nibble rotation per rank) and the bus layout are ours.
//!
//! Layout: each nibble lane carries the 32-bit stage word over 8 beats, beat
//! `t` holding bits `4t..4t+3`; both nibbles of a byte lane carry it, the
//! high nibble inverted so the two halves of a byte never match.

use crate::result::{ErrorCode, ErrorRecord};

/// The stage words (PG150 p887-888), used in turn by stage index.
pub const STAGE_WORDS: [u32; 5] = [
    0x399c_4e27,
    0x3587_d5dc,
    0x919c_d315,
    0x4e25_62e5,
    0x2c6c_9aaa,
];
/// Beats per pattern (the 32-bit word over a 4-bit nibble lane).
pub const BEATS: usize = 8;
/// Failing nibbles kept with their data.
pub const KEPT: usize = 3;

/// The word for `stage` on `rank`: the stage's word rotated by one nibble
/// per rank.
pub const fn word(stage: u8, rank: u8) -> u32 {
    STAGE_WORDS[stage as usize % STAGE_WORDS.len()].rotate_left(4 * (rank as u32 % 8))
}

/// The byte of one byte lane at beat `t` for `word`.
pub const fn byte_at(word: u32, t: usize) -> u8 {
    let lo = ((word >> (4 * (t % BEATS))) & 0xf) as u8;
    lo | ((!lo & 0xf) << 4)
}

/// Fill a bus image of `bytes` byte lanes (beat-major, `BEATS` beats) for
/// `stage` and `rank`. Returns the bytes written, or None if `out` is
/// smaller than `BEATS * bytes` or `bytes` is 0 or above 8.
pub fn fill(out: &mut [u8], bytes: usize, stage: u8, rank: u8) -> Option<usize> {
    let n = BEATS * bytes;
    if bytes == 0 || bytes > 8 || out.len() < n {
        return None;
    }
    let w = word(stage, rank);
    for t in 0..BEATS {
        for b in 0..bytes {
            out[t * bytes + b] = byte_at(w, t);
        }
    }
    Some(n)
}

/// One failing nibble lane: its index (`2 * byte + high`) and the 32 bits
/// it should have carried and did carry.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct NibbleFail {
    pub nibble: u8,
    pub expected: u32,
    pub got: u32,
}

/// The result of one compare.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Compare {
    /// Bit n set: nibble lane n failed (up to 16 nibbles = 8 byte lanes).
    pub nibbles: u16,
    /// Bit n set: data bit n of the bus failed (8 bits per byte lane).
    pub bits: u64,
    /// The first [`KEPT`] failing nibbles, lowest index first.
    pub first: [NibbleFail; KEPT],
    pub kept: usize,
}

impl Compare {
    pub fn passed(&self) -> bool {
        self.nibbles == 0
    }

    /// The first failure as an error record (stage, channel, rank given;
    /// byte of the first failing nibble; `bit` when one bit failed in that
    /// byte). None when the compare passed.
    pub fn error(&self, stage: u8, ch: u8, rank: u8, code: ErrorCode) -> Option<ErrorRecord> {
        if self.passed() {
            return None;
        }
        let byte = self.first[0].nibble / 2;
        let lane = (self.bits >> (8 * byte as u32)) & 0xff;
        let bit = (lane.count_ones() == 1).then(|| lane.trailing_zeros() as u8);
        Some(ErrorRecord {
            stage,
            code,
            ch,
            rank,
            byte,
            bit,
        })
    }
}

/// Compare a read-back bus image with the expected one (`bytes` lanes,
/// `BEATS` beats each).
pub fn compare(expected: &[u8], got: &[u8], bytes: usize) -> Compare {
    let mut c = Compare::default();
    let bytes = bytes.min(8);
    let beats = BEATS
        .min(expected.len() / bytes.max(1))
        .min(got.len() / bytes.max(1));
    let nib = |img: &[u8], b: usize, hi: bool| -> u32 {
        let mut w = 0u32;
        for t in 0..beats {
            let v = img[t * bytes + b];
            let n = if hi { v >> 4 } else { v & 0xf };
            w |= u32::from(n) << (4 * t);
        }
        w
    };
    for b in 0..bytes {
        for t in 0..beats {
            let x = expected[t * bytes + b] ^ got[t * bytes + b];
            c.bits |= u64::from(x) << (8 * b);
        }
        for hi in [false, true] {
            let (e, g) = (nib(expected, b, hi), nib(got, b, hi));
            if e != g {
                let n = (2 * b + hi as usize) as u8;
                c.nibbles |= 1 << n;
                if c.kept < KEPT {
                    c.first[c.kept] = NibbleFail {
                        nibble: n,
                        expected: e,
                        got: g,
                    };
                    c.kept += 1;
                }
            }
        }
    }
    c
}

/// Why a stage check failed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CheckError<E> {
    /// The write/read access failed.
    Io(E),
    /// The buffers are too small or the lane count is out of range.
    Config,
    /// The pattern did not read back.
    Mismatch(Compare),
    /// The known-bad setting passed: the compare (or the knob) does not see
    /// a broken path.
    NegativePassed,
}

/// Run the stage check: fill `exp` for `stage`/`rank`, call
/// `write_read(exp, got)` (write the image, read it back into `got`),
/// compare. `exp` and `got` must hold `BEATS * bytes` bytes.
pub fn check<E>(
    stage: u8,
    rank: u8,
    bytes: usize,
    exp: &mut [u8],
    got: &mut [u8],
    mut write_read: impl FnMut(&[u8], &mut [u8]) -> Result<(), E>,
) -> Result<Compare, CheckError<E>> {
    let n = fill(exp, bytes, stage, rank).ok_or(CheckError::Config)?;
    if got.len() < n {
        return Err(CheckError::Config);
    }
    write_read(&exp[..n], &mut got[..n]).map_err(CheckError::Io)?;
    let c = compare(&exp[..n], &got[..n], bytes);
    if c.passed() {
        Ok(c)
    } else {
        Err(CheckError::Mismatch(c))
    }
}

/// The negative check: `set_bad(true)` applies a known-bad setting (for
/// example a delay far outside the window), the stage check must then fail;
/// `set_bad(false)` restores the trained setting whatever happened. Ok with
/// the failing compare; `NegativePassed` if the bad setting passed.
pub fn negative<E>(
    stage: u8,
    rank: u8,
    bytes: usize,
    exp: &mut [u8],
    got: &mut [u8],
    mut set_bad: impl FnMut(bool) -> Result<(), E>,
    write_read: impl FnMut(&[u8], &mut [u8]) -> Result<(), E>,
) -> Result<Compare, CheckError<E>> {
    set_bad(true).map_err(CheckError::Io)?;
    let r = check(stage, rank, bytes, exp, got, write_read);
    let restored = set_bad(false).map_err(CheckError::Io);
    match r {
        Err(CheckError::Mismatch(c)) => restored.map(|_| c),
        Ok(_) => restored.and(Err(CheckError::NegativePassed)),
        Err(e) => Err(e),
    }
}
