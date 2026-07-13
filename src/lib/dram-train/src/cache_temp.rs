// SPDX-License-Identifier: GPL-2.0-only
//! The temperature-aware training cache (P3-A5) on top of the plain cache
//! ([`crate::cache`]), built to the design-around Dave chose (Oct 15) from
//! the orchestrator's `prior-art-dram-training-20261014/PATENT-COMPARISON-A5.md`
//! (rows D1, D4, D5, D6, D7, with D8/D9 as planned):
//!
//! - **Temperature only invalidates (D1).** The record holds the MR4
//!   temperature band at training time. A band differing from the current
//!   one by more than the policy's tolerance (default: any change) means a
//!   full retrain. Temperature never selects a stored set (no per-band
//!   records; Micron US 12,027,195) and never adjusts a value (no
//!   temperature-corrected delays; AMD US 12,602,162).
//! - **One record plus a recovery copy (D9).** A rolling copy (rewritten
//!   after every full training) and a golden copy (written once, at
//!   acceptance). A copy is chosen by integrity and identity only: the
//!   rolling copy if intact with a matching key, else the golden one. The
//!   band is checked after that choice, so it can never pick a copy.
//! - **The boot-path check is a plain data test (D4)** at the restored
//!   settings, on all ranks, channels and byte lanes, as in
//!   [`crate::cache::Restore::data_test`]. No offsets, no window widths,
//!   no comparison with stored margins.
//! - **A closed list of functional retrain reasons (D5):** record, key,
//!   band, failed apply, no gate found or engine timeout, data-test
//!   mismatch, plus the forced flag and the periodic retrain
//!   ([`crate::cache::Why`]). Never a width threshold.
//! - **Margins are for logs and acceptance only.** A record may carry a
//!   log section (margins, notes); [`boot`] never reads it and never hands
//!   it to the board on the boot path; [`log_of`] reads it for
//!   diagnostics.
//! - **Retrain in the same boot (D6):** [`boot`] calls the board's full
//!   training itself; there is no reboot path.
//! - **Mode-register re-init stays CPU-driven (D7):**
//!   [`crate::cache::Restore::apply`] must re-initialise the DRAM with
//!   mode-register writes issued one command at a time by firmware (K1
//!   direct, A7A `MRCTRL0`), never by loading a stored command list into a
//!   controller power-state or self-refresh sequencer. The payload holds
//!   register values, not command programs.
//!
//! Host tests (`tests/cache_temp.rs`) enforce what code can: every band
//! change beyond the tolerance invalidates, the band never selects a copy,
//! the decision does not depend on the log section, and the retrain runs
//! within the same call.

use crate::cache::{decode, encode, CacheError, DataTest, Key, Outcome, Policy, Restore, Why};

/// Inner layout tag inside the plain record's payload.
const TAG: [u8; 2] = *b"T1";
/// Bytes before the board payload: tag, band, payload length.
const INNER: usize = 2 + 1 + 4;

/// The board's MR4 band reading (in addition to [`Restore`]).
pub trait Band: Restore {
    /// The current temperature band (the board's coding of the MR4
    /// refresh-rate field, e.g. the hottest rank's).
    fn band(&mut self) -> Result<u8, Self::Error>;
}

/// Encode a record: `key`, the band at training time, the board payload
/// (registers and MR values) and an optional log section (margins and
/// notes for logs and acceptance; never read on the boot path).
pub fn encode_temp(
    key: &Key,
    band: u8,
    payload: &[u8],
    log: &[u8],
    crc: fn(&[u8]) -> u32,
    scratch: &mut [u8],
    out: &mut [u8],
) -> Result<usize, CacheError> {
    let n = INNER + payload.len() + log.len();
    if scratch.len() < n || payload.len() > u32::MAX as usize {
        return Err(CacheError::Space);
    }
    scratch[..2].copy_from_slice(&TAG);
    scratch[2] = band;
    scratch[3..7].copy_from_slice(&(payload.len() as u32).to_le_bytes());
    scratch[INNER..INNER + payload.len()].copy_from_slice(payload);
    scratch[INNER + payload.len()..n].copy_from_slice(log);
    encode(key, &scratch[..n], crc, out)
}

/// (band, board payload, log section) of a decoded inner payload.
fn split(inner: &[u8]) -> Result<(u8, &[u8], &[u8]), CacheError> {
    if inner.len() < INNER || inner[..2] != TAG {
        return Err(CacheError::Format(0));
    }
    let len = u32::from_le_bytes([inner[3], inner[4], inner[5], inner[6]]) as usize;
    let end = INNER
        .checked_add(len)
        .filter(|&e| e <= inner.len())
        .ok_or(CacheError::Length)?;
    Ok((inner[2], &inner[INNER..end], &inner[end..]))
}

/// The log section of a stored record, for diagnostics and acceptance
/// (not for the boot decision).
pub fn log_of(stored: &[u8], crc: fn(&[u8]) -> u32) -> Result<&[u8], CacheError> {
    let (_, inner) = decode(stored, crc)?;
    split(inner).map(|(_, _, log)| log)
}

/// Which copy a restore used.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Source {
    Rolling,
    Golden,
}

/// [`boot`]'s result: the plain outcome and, for a restore, the copy.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TempOutcome {
    pub outcome: Outcome,
    pub copy: Option<Source>,
}

/// The stored copies.
#[derive(Clone, Copy, Debug)]
pub struct Copies<'a> {
    pub rolling: &'a [u8],
    pub golden: Option<&'a [u8]>,
}

/// The temperature-aware boot decision (see the module docs).
/// `band_tolerance`: band steps allowed before the record is invalid (0:
/// any change invalidates). Errors of `train` are returned.
pub fn boot<R: Band>(
    copies: Copies<'_>,
    key: &Key,
    crc: fn(&[u8]) -> u32,
    policy: Policy,
    band_tolerance: u8,
    board: &mut R,
) -> Result<TempOutcome, R::Error> {
    let why = (|| -> Result<Source, Why> {
        if policy.forced {
            return Err(Why::Forced);
        }
        if policy.every != 0 && policy.boots_since_train + 1 >= policy.every {
            return Err(Why::Periodic);
        }
        // Choose a copy by integrity and identity only.
        fn pick<'a>(
            stored: &'a [u8],
            key: &Key,
            crc: fn(&[u8]) -> u32,
        ) -> Result<(u8, &'a [u8]), Why> {
            let (k, inner) = decode(stored, crc).map_err(Why::Record)?;
            if let Some(f) = k.mismatch(key) {
                return Err(Why::Key(f));
            }
            let (band, payload, _log) = split(inner).map_err(Why::Record)?;
            Ok((band, payload))
        }
        let (copy, band, payload) = match pick(copies.rolling, key, crc) {
            Ok((b, p)) => (Source::Rolling, b, p),
            Err(first) => match copies.golden.map(|g| pick(g, key, crc)) {
                Some(Ok((b, p))) => (Source::Golden, b, p),
                _ => return Err(first),
            },
        };
        // Then the band: it can only invalidate.
        let now = board.band().map_err(|_| Why::Band {
            stored: band,
            now: None,
        })?;
        if now.abs_diff(band) > band_tolerance {
            return Err(Why::Band {
                stored: band,
                now: Some(now),
            });
        }
        board.apply(payload).map_err(|_| Why::Apply)?;
        board.warm().map_err(Why::Warm)?;
        match board.data_test() {
            Ok(DataTest::Clean) => Ok(copy),
            _ => Err(Why::DataTest),
        }
    })();
    match why {
        Ok(copy) => Ok(TempOutcome {
            outcome: Outcome::Restored,
            copy: Some(copy),
        }),
        Err(w) => {
            board.train()?;
            Ok(TempOutcome {
                outcome: Outcome::Trained(w),
                copy: None,
            })
        }
    }
}
