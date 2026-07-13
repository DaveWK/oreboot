// SPDX-License-Identifier: GPL-2.0-only
//! The live-readback contract of a search backend (A7A m13, Oct 15): a
//! point counts as applied only when the backend has read the knob's LIVE
//! copy and found the point there, or has declared the knob unverifiable.
//!
//! Why: six hours of A7A read-training sweeps (m13l-m13r) swept nothing.
//! The board's `apply(point)` wrote RxEnDly/RxClk CSRs that are
//! double-buffered on that PHY and only go live after latch pulses
//! (PptRxEnEvnt after RxEnDly, ForceInternalUpdate after RxClk). The CSR
//! read back the written value; the live copy, readable through the
//! SelDbCurDlyTmngInfo/DbCurrentDlyTimingInfo selector, stayed at the old
//! value. The generic search trusted the write.
//!
//! The rule for every backend (`search::Backend::verify_live`,
//! `vref_sweep::Axis::verify_live`): after programming a point, read the
//! copy the data path uses, not the CSR that was written, and answer
//! [`Live::Live`] or [`Live::Stale`]; a knob with no live readback answers
//! [`Live::Unverifiable`] and the report says so. The search engines treat
//! `Stale` as a hard error naming the knob and the point, never as a point
//! to continue from, and count the rest in a [`Liveness`]:
//! "points verified live: N/M".

use core::fmt;

/// What the backend found in the knob's live copy after programming a point.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Live {
    /// The live copy holds the point.
    Live,
    /// The live copy holds something else: the write did not take (a
    /// double-buffered CSR without its latch, a wrong address, a read-only
    /// field...). The search stops here.
    Stale,
    /// The backend has no live readback for this knob. Declared, not
    /// assumed: the report names it.
    Unverifiable,
}

/// The error of [`Liveness::record`]: the point was not in the live copy.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Stale;

/// Point programmings of one search and how many were confirmed live.
/// `applied == live + unverifiable` always holds: a stale point ends the
/// search before it is counted.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct Liveness {
    /// Points programmed (every `launch`/`apply`/`set`).
    pub applied: u32,
    /// Of those, confirmed in the live copy.
    pub live: u32,
    /// Of those, on a knob the backend declared unverifiable.
    pub unverifiable: u32,
}

impl Liveness {
    /// Count one programming; `Err(Stale)` on a stale point (the caller
    /// turns it into its own error, naming knob and point).
    pub fn record(&mut self, l: Live) -> Result<(), Stale> {
        match l {
            Live::Stale => Err(Stale),
            Live::Live => {
                self.applied = self.applied.saturating_add(1);
                self.live = self.live.saturating_add(1);
                Ok(())
            }
            Live::Unverifiable => {
                self.applied = self.applied.saturating_add(1);
                self.unverifiable = self.unverifiable.saturating_add(1);
                Ok(())
            }
        }
    }

    /// Every programming was confirmed live (none unverifiable).
    pub const fn all_verified(&self) -> bool {
        self.unverifiable == 0 && self.live == self.applied
    }

    /// Sum with another search's counts (a session over several stages).
    pub fn merge(&mut self, o: Liveness) {
        self.applied = self.applied.saturating_add(o.applied);
        self.live = self.live.saturating_add(o.live);
        self.unverifiable = self.unverifiable.saturating_add(o.unverifiable);
    }
}

/// `points verified live: 7/7`; `points verified live: 0/7 (knob not
/// verifiable)`; `points verified live: 5/7 (2 not verifiable)`.
impl fmt::Display for Liveness {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "points verified live: {}/{}", self.live, self.applied)?;
        if self.unverifiable != 0 && self.unverifiable == self.applied {
            write!(f, " (knob not verifiable)")
        } else if self.unverifiable != 0 {
            write!(f, " ({} not verifiable)", self.unverifiable)
        } else {
            Ok(())
        }
    }
}

/// Kani proofs (P3-A7): the counts stay consistent and a stale point is
/// never counted.
#[cfg(kani)]
mod proofs {
    use super::*;

    #[kani::proof]
    #[kani::unwind(5)]
    fn liveness_counts_are_consistent() {
        let mut l = Liveness::default();
        for _ in 0..4 {
            let before = l;
            let r = l.record(match kani::any::<u8>() % 3 {
                0 => Live::Live,
                1 => Live::Stale,
                _ => Live::Unverifiable,
            });
            if r.is_err() {
                assert!(l == before);
            }
            assert!(l.applied == l.live + l.unverifiable);
            assert!(l.all_verified() == (l.unverifiable == 0 && l.live == l.applied));
        }
    }
}
