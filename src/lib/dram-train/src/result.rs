// SPDX-License-Identifier: GPL-2.0-only
//! The shared training-result schema: a per-stage log, one error record with
//! its location, per-edge left/right/final values, drift since boot, and the
//! margin line every board prints.
//!
//! Shape: the Versal LPDDR4 calibration data map (AMD AM019 v1.2,
//! `ddrmc_lpddr4_xram___CAL_SEQUENCE_STATUS_*`, `CAL_ERROR*`, `*_LEFT`,
//! `*_RIGHT`, `*_FINAL`, `*_MIN`/`*_MAX` tracking) and the PG353 v1.0 p38-42
//! status scheme; error kinds: the ZynqMP PUB training error classes
//! (UG1085 v2.5 p467-477). Names and structure only; no text is copied.
//!
//! A margin line has the fields of a `training` entry of the
//! `ore-edk2-boot/acceptance/1` record (doc/CONVERGENCE.md section 5b):
//! `axis ch rank lane unit lo hi center`, so the acceptance parser reads it
//! without a per-board adapter.

use core::fmt;

/// A trained axis, named as in the acceptance record.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Axis {
    Ca,
    CaVref,
    WriteLeveling,
    Wck2Ck,
    ReadGate,
    ReadEye,
    WriteEye,
    ReadVref,
    WriteVref,
    WriteLatency,
    ReadDeskew,
    WriteDeskew,
    Dqs2Dq,
}

impl Axis {
    pub const fn name(self) -> &'static str {
        match self {
            Axis::Ca => "ca",
            Axis::CaVref => "ca_vref",
            Axis::WriteLeveling => "write_leveling",
            Axis::Wck2Ck => "wck2ck",
            Axis::ReadGate => "read_gate",
            Axis::ReadEye => "read_eye",
            Axis::WriteEye => "write_eye",
            Axis::ReadVref => "read_vref",
            Axis::WriteVref => "write_vref",
            Axis::WriteLatency => "write_latency",
            Axis::ReadDeskew => "read_deskew",
            Axis::WriteDeskew => "write_deskew",
            Axis::Dqs2Dq => "dqs2dq",
        }
    }
}

/// A stage's status (the AM019 3-bit codes).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Status {
    NotStarted = 0b000,
    InProgress = 0b011,
    Skipped = 0b100,
    Done = 0b110,
    Failed = 0b111,
}

/// What went wrong in a stage (the ZynqMP PUB classes).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ErrorCode {
    /// The check before the search failed (nothing works at the start).
    PreCheck,
    /// The strobe or data is already too early at the range start.
    Early,
    /// The delay range ended while searching the left edge.
    RangeLeft,
    /// The delay range ended while searching the right edge.
    RangeRight,
    /// The check after setting the result failed.
    PostCheck,
    /// A hardware engine did not finish in time.
    Timeout,
    /// The window is narrower than required.
    Margin,
    /// The pass/fail curve has no usable transition.
    NoTransition,
    /// A read-back of a written setting (mode register...) differs.
    Readback,
    /// The knob did not move the observable ([`crate::sanity`]).
    DeadKnob,
    /// Part of the axis repeats another part.
    AliasedAxis,
    /// All pass, all fail, or a result at its limit.
    Saturated,
    /// The hardware is not in the mode the stage assumes.
    ModeMismatch,
}

/// Warnings that do not stop training.
pub mod warn {
    /// An eye edge was not found in the range; the centre may be skewed.
    pub const EDGE_NOT_FOUND: u16 = 1 << 0;
    /// Some samples disagreed (noise points) inside the window.
    pub const NOISE: u16 = 1 << 1;
    /// The result was saturated at a delay-line end.
    pub const SATURATED: u16 = 1 << 2;
    /// The result is an outlier against the other lanes.
    pub const OUTLIER: u16 = 1 << 3;
}

/// Where an error happened. `bit` is None for a byte-wide result.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ErrorRecord {
    pub stage: u8,
    pub code: ErrorCode,
    pub ch: u8,
    pub rank: u8,
    pub byte: u8,
    pub bit: Option<u8>,
}

/// One stage entry of the log.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct StageEntry {
    /// The board's stage id (a step id of its step list).
    pub stage: u8,
    pub axis: Option<Axis>,
    pub status: Status,
    pub warnings: u16,
}

/// A fixed-size stage log with the first error (no allocation).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct StageLog<const N: usize> {
    pub entries: [StageEntry; N],
    pub len: usize,
    pub error: Option<ErrorRecord>,
}

impl<const N: usize> Default for StageLog<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> StageLog<N> {
    const EMPTY: StageEntry = StageEntry {
        stage: 0,
        axis: None,
        status: Status::NotStarted,
        warnings: 0,
    };
    pub const fn new() -> Self {
        StageLog {
            entries: [Self::EMPTY; N],
            len: 0,
            error: None,
        }
    }
    fn find(&mut self, stage: u8) -> Option<&mut StageEntry> {
        self.entries[..self.len]
            .iter_mut()
            .find(|e| e.stage == stage)
    }
    /// Plan a stage (NotStarted), as the sequence about to run.
    pub fn plan(&mut self, stage: u8, axis: Option<Axis>) -> bool {
        if self.find(stage).is_some() {
            return true;
        }
        if self.len == N {
            return false;
        }
        self.entries[self.len] = StageEntry {
            stage,
            axis,
            ..Self::EMPTY
        };
        self.len += 1;
        true
    }
    fn set(&mut self, stage: u8, status: Status) -> bool {
        if !self.plan(stage, None) {
            return false;
        }
        if let Some(e) = self.find(stage) {
            e.status = status;
        }
        true
    }
    pub fn begin(&mut self, stage: u8) -> bool {
        self.set(stage, Status::InProgress)
    }
    pub fn done(&mut self, stage: u8) -> bool {
        self.set(stage, Status::Done)
    }
    pub fn skip(&mut self, stage: u8) -> bool {
        self.set(stage, Status::Skipped)
    }
    pub fn warn(&mut self, stage: u8, w: u16) {
        if let Some(e) = self.find(stage) {
            e.warnings |= w;
        }
    }
    /// Mark the stage failed; the first error is kept.
    pub fn fail(&mut self, e: ErrorRecord) -> bool {
        if self.error.is_none() {
            self.error = Some(e);
        }
        self.set(e.stage, Status::Failed)
    }
    /// The last stage that started (where training stopped).
    pub fn pointer(&self) -> Option<StageEntry> {
        self.entries[..self.len]
            .iter()
            .rev()
            .find(|e| e.status != Status::NotStarted)
            .copied()
    }
    pub fn entries(&self) -> &[StageEntry] {
        &self.entries[..self.len]
    }
}

/// Left, right and final value of one edge (rising or falling strobe...),
/// in the axis's own steps.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct EdgeValues {
    pub left: i32,
    pub right: i32,
    pub fin: i32,
}

impl EdgeValues {
    /// Margins left and right of the final value.
    pub const fn margins(self) -> (i32, i32) {
        (self.fin - self.left, self.right - self.fin)
    }
}

/// Minimum and maximum of a tracked value since boot, and its boot value.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Drift {
    pub boot: i32,
    pub min: i32,
    pub max: i32,
}

impl Drift {
    pub const fn new(boot: i32) -> Drift {
        Drift {
            boot,
            min: boot,
            max: boot,
        }
    }
    pub fn update(&mut self, v: i32) {
        self.min = self.min.min(v);
        self.max = self.max.max(v);
    }
    /// Largest distance from the boot value.
    pub const fn excursion(self) -> u32 {
        let a = self.boot.abs_diff(self.min);
        let b = self.boot.abs_diff(self.max);
        if a > b {
            a
        } else {
            b
        }
    }
}

/// The unit of a margin line.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Unit {
    /// The axis's own steps (taps, codes).
    Step,
    Ps,
    Mv,
}

impl Unit {
    pub const fn name(self) -> &'static str {
        match self {
            Unit::Step => "step",
            Unit::Ps => "ps",
            Unit::Mv => "mv",
        }
    }
}

/// One margin: a `training` entry of the acceptance record.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Margin {
    pub axis: Axis,
    pub ch: u8,
    pub rank: u8,
    pub lane: u8,
    pub unit: Unit,
    pub lo: i32,
    pub hi: i32,
    pub center: i32,
}

impl Margin {
    pub const fn width(self) -> i32 {
        self.hi - self.lo + 1
    }
}

/// `[margin] axis=read_eye ch=0 rank=0 lane=1 unit=step lo=3 hi=17 center=10`
impl fmt::Display for Margin {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "[margin] axis={} ch={} rank={} lane={} unit={} lo={} hi={} center={}",
            self.axis.name(),
            self.ch,
            self.rank,
            self.lane,
            self.unit.name(),
            self.lo,
            self.hi,
            self.center
        )
    }
}

/// `[stage] id=3 axis=read_eye status=done warn=0x0001`
impl fmt::Display for StageEntry {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let status = match self.status {
            Status::NotStarted => "not_started",
            Status::InProgress => "in_progress",
            Status::Skipped => "skipped",
            Status::Done => "done",
            Status::Failed => "failed",
        };
        write!(
            f,
            "[stage] id={} axis={} status={} warn={:#06x}",
            self.stage,
            self.axis.map_or("-", Axis::name),
            status,
            self.warnings
        )
    }
}

/// Kani proofs (P3-A7): the stage log never panics, never grows past its
/// capacity, and keeps one entry per stage.
#[cfg(kani)]
mod proofs {
    use super::*;

    #[kani::proof]
    #[kani::unwind(6)]
    fn stage_log_stays_bounded() {
        let mut log = StageLog::<3>::new();
        for _ in 0..4 {
            let stage: u8 = kani::any::<u8>() % 5;
            match kani::any::<u8>() % 5 {
                0 => {
                    log.plan(stage, None);
                }
                1 => {
                    log.begin(stage);
                }
                2 => {
                    log.done(stage);
                }
                3 => {
                    log.skip(stage);
                }
                _ => log.warn(stage, kani::any()),
            }
            assert!(log.len <= 3);
            for i in 0..log.len {
                for j in 0..i {
                    assert!(log.entries[i].stage != log.entries[j].stage);
                }
            }
        }
    }
}
