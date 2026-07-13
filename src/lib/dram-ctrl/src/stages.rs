// SPDX-License-Identifier: GPL-2.0-only
//! Training stage lists by data rate, and which stages a warm boot re-runs.
//!
//! A board lists its training stages once, each with the data rates it runs
//! at (for example command bus training only above a rate, per-bit deskew
//! at or above a rate, complex patterns above a rate), whether it runs on
//! every rank or only the first, and whether it must run again when the
//! rest of the training comes from the training cache or after a
//! self-refresh exit. [`plan`] picks the stages for one boot.
//!
//! The cache restore rule: restored delays are written back as they were
//! trained, but the read DQS gate depends on the round-trip delay, which
//! moves with temperature and voltage since the cache was written; so a
//! restore re-runs only the gate (and the stages a board marks the same
//! way), and so does a self-refresh exit.
//!
//! Method: AMD PG313 v1.1 p72-82 and section 6 (stage list, rate
//! thresholds, first-rank-only write complex, gate re-run on self-refresh
//! exit), PG150 v1.4 p71-75 (save/restore re-runs the gate), PG456 v1.0
//! p159-160; cited, not copied. [`LPDDR4_EXAMPLE`] carries the PG313 rate
//! thresholds as an example; each board keeps its own list.

/// When a stage runs, by data rate (MT/s).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Rate {
    Always,
    /// Above this rate (exclusive).
    Above(u32),
    /// At or above this rate.
    AtLeast(u32),
    /// Below this rate (exclusive).
    Below(u32),
}

impl Rate {
    pub const fn allows(self, mtps: u32) -> bool {
        match self {
            Rate::Always => true,
            Rate::Above(r) => mtps > r,
            Rate::AtLeast(r) => mtps >= r,
            Rate::Below(r) => mtps < r,
        }
    }
}

/// Which ranks a stage runs on.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Ranks {
    All,
    /// Only the first rank (the others use its result).
    First,
}

/// Why this boot trains.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Boot {
    /// Full training.
    Cold,
    /// Delays restored from the training cache.
    CacheRestore,
    /// Leaving self-refresh (or deep power-down with retention).
    SelfRefreshExit,
}

/// One stage of a board's list.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Stage<S> {
    pub id: S,
    pub rate: Rate,
    pub ranks: Ranks,
    /// Runs again on [`Boot::CacheRestore`] and [`Boot::SelfRefreshExit`].
    pub warm: bool,
}

impl<S> Stage<S> {
    pub const fn new(id: S, rate: Rate) -> Self {
        Self {
            id,
            rate,
            ranks: Ranks::All,
            warm: false,
        }
    }
    pub const fn first_rank(mut self) -> Self {
        self.ranks = Ranks::First;
        self
    }
    pub const fn warm(mut self) -> Self {
        self.warm = true;
        self
    }
}

/// The stages to run for `rank` (0 = first) at `mtps` on a `boot`, in list
/// order, into `out`. Returns how many; stops when `out` is full.
pub fn plan<S: Copy>(stages: &[Stage<S>], mtps: u32, boot: Boot, rank: u8, out: &mut [S]) -> usize {
    let mut n = 0;
    for s in stages {
        if n == out.len() {
            break;
        }
        let rate = s.rate.allows(mtps);
        let rank_ok = s.ranks == Ranks::All || rank == 0;
        let boot_ok = boot == Boot::Cold || s.warm;
        if rate && rank_ok && boot_ok {
            out[n] = s.id;
            n += 1;
        }
    }
    n
}

/// Common training stages, for boards that want names rather than their
/// own ids.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Train {
    CommandBus,
    Gate,
    WriteLeveling,
    ReadDeskew,
    ReadCentre,
    WriteDeskew,
    WriteCentre,
    DataMask,
    WriteLatency,
    ReadDbi,
    ReadComplex,
    WriteComplex,
    Sanity,
}

/// The PG313 LPDDR4 stage order and rate thresholds, as an example list:
/// command bus training above 1866, deskew at or above 1600, read DBI above
/// 1600, complex stages above 2133 MT/s, write complex on the first rank,
/// the gate re-run on warm boots.
pub const LPDDR4_EXAMPLE: [Stage<Train>; 13] = [
    Stage::new(Train::CommandBus, Rate::Above(1866)),
    Stage::new(Train::Gate, Rate::Always).warm(),
    Stage::new(Train::WriteLeveling, Rate::Always),
    Stage::new(Train::ReadDeskew, Rate::AtLeast(1600)),
    Stage::new(Train::ReadCentre, Rate::Always),
    Stage::new(Train::WriteDeskew, Rate::AtLeast(1600)),
    Stage::new(Train::WriteCentre, Rate::Always),
    Stage::new(Train::DataMask, Rate::Always),
    Stage::new(Train::WriteLatency, Rate::Always),
    Stage::new(Train::ReadDbi, Rate::Above(1600)),
    Stage::new(Train::ReadComplex, Rate::Above(2133)),
    Stage::new(Train::WriteComplex, Rate::Above(2133)).first_rank(),
    Stage::new(Train::Sanity, Rate::Always).warm(),
];
