// SPDX-License-Identifier: GPL-2.0-only
//! The plain training cache: a versioned record of a board's trained
//! settings, keyed on what the training depends on, so a warm boot can
//! restore it instead of training again.
//!
//! - Key (identity only, no temperature): board ID, crate version, rank
//!   count, density, MR5-MR8 of every rank (manufacturer, revisions,
//!   density and I/O width), and the frequency points.
//! - Payload: the board's own bytes (controller and PHY registers, MR
//!   values), opaque here. Margins may be stored alongside for logs and
//!   acceptance records only: nothing on the boot path reads them.
//! - Integrity: a CRC-32 over key and payload, computed by the function
//!   the board passes (normally `storage::crc::crc32`, so this crate needs
//!   no new dependency).
//! - [`boot`]: restore when the stored key matches and the record is
//!   intact, then run the board's warm stages (`dram_ctrl::stages` with
//!   `Boot::CacheRestore`: the read gate) and a plain data test; otherwise
//!   train in full. The caller stores a new record after a full training.
//!
//! # The boot-path rules
//!
//! These keep the restore decision free of margin testing (orchestrator
//! `prior-art-dram-training-20261014/PATENT-COMPARISON-A5.md`, against
//! Intel US 11,894,084 and AMD US 11,176,986; Dave, Oct 15):
//! - [`Restore::data_test`] is a plain data test at the restored settings,
//!   on every rank, channel and byte lane: write and read a pattern with no
//!   delay, Vref or voltage offset, no window or margin measurement, and
//!   any error is a failure ([`DataTest::Errors`], no counts, no
//!   threshold). It never compares with stored margins.
//! - The reasons for a full training are a closed list ([`Why`]): forced,
//!   every Nth boot, record error, key mismatch, failed apply, a
//!   functional warm-stage failure ([`WarmFailure`]: no gate found, an
//!   engine timeout), a data-test mismatch. Never a margin delta, a
//!   window width or an eye size. ([`crate::cache_temp`], P3-A5, adds a
//!   temperature-band mismatch, which only invalidates.)
//! - Stored margins never feed the decision; margins are measured, if at
//!   all, in an acceptance or diagnostic mode whose result is only logged.
//!
//! A future suspend/resume item (training restored across a system
//! suspend) needs a review of AMD US 11,176,986 and US 11,682,445 first.
//!
//! The temperature-aware extension (an MR4 band that only invalidates, a
//! recovery copy) is [`crate::cache_temp`] (P3-A5). Sources: prior-art REPORT
//! 5.1, 5.4, 5.5 (orchestrator); AMD PG150 v1.4 p71-75 (save/restore
//! re-runs the gate).

/// Record magic and format version.
pub const MAGIC: [u8; 8] = *b"OREDTC01";
pub const FORMAT: u16 = 1;
/// Most ranks and frequency points a key holds.
pub const MAX_RANKS: usize = 4;
pub const MAX_FREQS: usize = 8;
/// Bytes of an encoded key.
pub const KEY_BYTES: usize = 4 + 4 + 1 + 1 + 4 * MAX_RANKS + 1 + 2 * MAX_FREQS;
/// Bytes before the key: magic, format, payload length, CRC.
pub const HEADER_BYTES: usize = 8 + 2 + 4 + 4;

/// What a trained result depends on.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Key {
    pub board: u32,
    /// The training code's version (e.g. major << 16 | minor << 8 | patch).
    pub crate_version: u32,
    pub ranks: u8,
    /// Density code (the board's, e.g. MR8 density field).
    pub density: u8,
    /// MR5, MR6, MR7, MR8 per rank (unused ranks 0).
    pub mr5_8: [[u8; 4]; MAX_RANKS],
    pub freqs: u8,
    /// Frequency points in MHz (the first `freqs`).
    pub freq_mhz: [u16; MAX_FREQS],
}

impl Key {
    fn encode(&self, out: &mut [u8]) {
        out[0..4].copy_from_slice(&self.board.to_le_bytes());
        out[4..8].copy_from_slice(&self.crate_version.to_le_bytes());
        out[8] = self.ranks;
        out[9] = self.density;
        for (r, m) in self.mr5_8.iter().enumerate() {
            out[10 + 4 * r..14 + 4 * r].copy_from_slice(m);
        }
        let f = 10 + 4 * MAX_RANKS;
        out[f] = self.freqs;
        for (i, v) in self.freq_mhz.iter().enumerate() {
            out[f + 1 + 2 * i..f + 3 + 2 * i].copy_from_slice(&v.to_le_bytes());
        }
    }

    fn decode(b: &[u8]) -> Key {
        let u32_at = |i: usize| u32::from_le_bytes([b[i], b[i + 1], b[i + 2], b[i + 3]]);
        let mut mr5_8 = [[0u8; 4]; MAX_RANKS];
        for (r, m) in mr5_8.iter_mut().enumerate() {
            m.copy_from_slice(&b[10 + 4 * r..14 + 4 * r]);
        }
        let f = 10 + 4 * MAX_RANKS;
        let mut freq_mhz = [0u16; MAX_FREQS];
        for (i, v) in freq_mhz.iter_mut().enumerate() {
            *v = u16::from_le_bytes([b[f + 1 + 2 * i], b[f + 2 + 2 * i]]);
        }
        Key {
            board: u32_at(0),
            crate_version: u32_at(4),
            ranks: b[8],
            density: b[9],
            mr5_8,
            freqs: b[f],
            freq_mhz,
        }
    }

    /// The first field that differs from `other`, for the log.
    pub fn mismatch(&self, other: &Key) -> Option<Field> {
        if self.board != other.board {
            Some(Field::Board)
        } else if self.crate_version != other.crate_version {
            Some(Field::CrateVersion)
        } else if self.ranks != other.ranks {
            Some(Field::Ranks)
        } else if self.density != other.density {
            Some(Field::Density)
        } else if self.mr5_8 != other.mr5_8 {
            Some(Field::ModeRegisters)
        } else if self.freqs != other.freqs || self.freq_mhz != other.freq_mhz {
            Some(Field::Frequencies)
        } else {
            None
        }
    }
}

/// A key field.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Field {
    Board,
    CrateVersion,
    Ranks,
    Density,
    ModeRegisters,
    Frequencies,
}

/// Why a record cannot be used.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CacheError {
    /// The output is too small for the record.
    Space,
    /// No record (wrong magic: erased or never written).
    Empty,
    /// Another record format.
    Format(u16),
    /// The length field does not fit the stored bytes.
    Length,
    /// The CRC does not match.
    Crc,
}

/// Encode a record of `key` and `payload` into `out`; returns its length.
pub fn encode(
    key: &Key,
    payload: &[u8],
    crc: fn(&[u8]) -> u32,
    out: &mut [u8],
) -> Result<usize, CacheError> {
    let n = HEADER_BYTES + KEY_BYTES + payload.len();
    if out.len() < n || payload.len() > u32::MAX as usize {
        return Err(CacheError::Space);
    }
    out[0..8].copy_from_slice(&MAGIC);
    out[8..10].copy_from_slice(&FORMAT.to_le_bytes());
    out[10..14].copy_from_slice(&(payload.len() as u32).to_le_bytes());
    key.encode(&mut out[HEADER_BYTES..HEADER_BYTES + KEY_BYTES]);
    out[HEADER_BYTES + KEY_BYTES..n].copy_from_slice(payload);
    let c = crc(&out[HEADER_BYTES..n]);
    out[14..18].copy_from_slice(&c.to_le_bytes());
    Ok(n)
}

/// Decode a stored record: its key and payload.
pub fn decode(stored: &[u8], crc: fn(&[u8]) -> u32) -> Result<(Key, &[u8]), CacheError> {
    if stored.len() < HEADER_BYTES + KEY_BYTES || stored[0..8] != MAGIC {
        return Err(CacheError::Empty);
    }
    let format = u16::from_le_bytes([stored[8], stored[9]]);
    if format != FORMAT {
        return Err(CacheError::Format(format));
    }
    let len = u32::from_le_bytes([stored[10], stored[11], stored[12], stored[13]]) as usize;
    let n = HEADER_BYTES
        .checked_add(KEY_BYTES)
        .and_then(|h| h.checked_add(len))
        .filter(|&n| n <= stored.len())
        .ok_or(CacheError::Length)?;
    let want = u32::from_le_bytes([stored[14], stored[15], stored[16], stored[17]]);
    if crc(&stored[HEADER_BYTES..n]) != want {
        return Err(CacheError::Crc);
    }
    Ok((
        Key::decode(&stored[HEADER_BYTES..HEADER_BYTES + KEY_BYTES]),
        &stored[HEADER_BYTES + KEY_BYTES..n],
    ))
}

/// Why a boot trained in full: a closed list (see the module docs).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Why {
    Forced,
    /// Every Nth boot (the periodic full training).
    Periodic,
    Record(CacheError),
    Key(Field),
    /// Applying the stored payload failed.
    Apply,
    /// A warm stage failed functionally.
    Warm(WarmFailure),
    /// The plain data test after the restore saw an error.
    DataTest,
    /// The temperature band differs from the record's beyond the
    /// tolerance, or could not be read (`now: None`); P3-A5 only
    /// ([`crate::cache_temp`]). It only ever invalidates.
    Band {
        stored: u8,
        now: Option<u8>,
    },
}

/// The only ways a warm stage may fail: functional failures, never a
/// width or margin threshold.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WarmFailure {
    /// The read gate search found no gate.
    NoGate,
    /// A training engine or sequencer did not complete in time.
    Timeout,
}

/// The result of the plain data test: clean, or any error at all.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DataTest {
    Clean,
    Errors,
}

/// How this boot got its training.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Outcome {
    Restored,
    /// Trained in full; the caller stores a new record.
    Trained(Why),
}

/// Policy inputs of [`boot`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Policy {
    /// Train in full regardless (a user or recovery flag).
    pub forced: bool,
    /// Boots since the last full training, and train in full when it
    /// reaches `every` (0: never periodic).
    pub boots_since_train: u32,
    pub every: u32,
}

/// The board's side of [`boot`]. The boot-path rules in the module docs
/// bind every implementation.
pub trait Restore {
    type Error;
    /// Write the stored payload to the hardware.
    fn apply(&mut self, payload: &[u8]) -> Result<(), Self::Error>;
    /// Run the warm stages (`dram_ctrl::stages::plan` with
    /// `Boot::CacheRestore`: the read gate). Fail only functionally.
    fn warm(&mut self) -> Result<(), WarmFailure>;
    /// The plain data test at the restored settings, every rank, channel
    /// and byte lane, no offsets and no margin measurement; any error is
    /// [`DataTest::Errors`]. An access error counts as `Errors`.
    fn data_test(&mut self) -> Result<DataTest, Self::Error>;
    /// Train in full.
    fn train(&mut self) -> Result<(), Self::Error>;
}

/// The boot decision and its steps (see the module docs). Errors of
/// `train` are returned; errors on the restore path fall back to training.
pub fn boot<R: Restore>(
    stored: &[u8],
    key: &Key,
    crc: fn(&[u8]) -> u32,
    policy: Policy,
    board: &mut R,
) -> Result<Outcome, R::Error> {
    let why = (|| {
        if policy.forced {
            return Err(Why::Forced);
        }
        if policy.every != 0 && policy.boots_since_train + 1 >= policy.every {
            return Err(Why::Periodic);
        }
        let (k, payload) = decode(stored, crc).map_err(Why::Record)?;
        if let Some(f) = k.mismatch(key) {
            return Err(Why::Key(f));
        }
        board.apply(payload).map_err(|_| Why::Apply)?;
        board.warm().map_err(Why::Warm)?;
        match board.data_test() {
            Ok(DataTest::Clean) => Ok(()),
            _ => Err(Why::DataTest),
        }
    })();
    match why {
        Ok(()) => Ok(Outcome::Restored),
        Err(w) => {
            board.train()?;
            Ok(Outcome::Trained(w))
        }
    }
}

/// Kani proofs (P3-A7): decoding any stored bytes never panics, and an
/// accepted record's payload lies inside them.
#[cfg(kani)]
mod proofs {
    use super::*;

    fn zero_crc(_: &[u8]) -> u32 {
        0
    }

    #[kani::proof]
    #[kani::unwind(70)]
    fn decode_any_bytes() {
        let stored: [u8; 64] = kani::any();
        let n: usize = kani::any();
        kani::assume(n <= 64);
        if let Ok((_, payload)) = decode(&stored[..n], zero_crc) {
            assert!(payload.len() + HEADER_BYTES + KEY_BYTES <= n);
        }
    }
}
