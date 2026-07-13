// SPDX-License-Identifier: GPL-2.0-only
//! Original, allocation-free CPU training search/control primitives
//! (formerly the A7A `tools/cpu-training` crate `a733-cpu-training`).
//!
//! This module contains no CSR addresses, default topology, analog conversion,
//! physical command issue, or PMU implementation. A backend must supply those
//! from reviewed platform evidence. Grid coordinates are backend-defined
//! discrete settings, never claimed picoseconds, volts, or physical margins.
//! No board backend exists yet; [`Unbound`] refuses every request.
//!
//! Live-readback contract (Oct 15, A7A m13; [`crate::live`]): a backend
//! programs a point in `launch`/`apply` and the engines then call
//! [`Backend::verify_live`]. A point is treated as applied only when the
//! backend confirms it in the knob's live copy or has declared the knob
//! unverifiable; a stale point is [`Cause::NotLive`], naming the request
//! and the point. Every result carries a [`Liveness`] count.

pub use crate::live::{Live, Liveness};

pub mod command;
pub mod feedback;
pub mod full_session;
pub mod interval_table;
pub mod line;
pub use jedec_lpddr::lp5 as lpddr5;
pub mod matrix;
pub mod phase;
pub mod session;
pub mod vref_mr;

/// Frequency set-points a request may name (LPDDR4/4X and LPDDR5 controllers
/// in use have at most four).
pub const MAX_PSTATES: u8 = 4;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Stage {
    Ca,
    Wck2Ck,
    ReadGate,
    ReadCenter,
    WriteCenter,
    ReadVref,
    WriteVref,
    Dfe,
    Dca,
}
impl Stage {
    pub const ALL: [Self; 9] = [
        Self::Ca,
        Self::Wck2Ck,
        Self::ReadGate,
        Self::ReadCenter,
        Self::WriteCenter,
        Self::ReadVref,
        Self::WriteVref,
        Self::Dfe,
        Self::Dca,
    ];
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Request {
    pub stage: Stage,
    pub pstate: u8,
    pub channel: u8,
    pub rank: u8,
    pub lane: u8,
    /// All required bits must have fresh observations and no failure.
    pub required_mask: u64,
}
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct Point {
    pub x: u8,
    pub y: u8,
}
#[derive(Clone, Copy, Debug)]
pub struct Domain {
    pub width: u8,
    pub height: u8,
    pub circular_x: bool,
    pub circular_y: bool,
    pub min_width: u8,
    pub min_height: u8,
    pub scan_repeats: u8,
    pub verify_repeats: u8,
    pub polls_per_probe: u32,
    /// Shared launch+poll budget for the whole stage, not a wall-clock time.
    pub probe_operations: u32,
}
impl Domain {
    pub fn valid(self) -> bool {
        (1..=64).contains(&self.width)
            && (1..=64).contains(&self.height)
            && self.min_width > 0
            && self.min_width <= self.width
            && self.min_height > 0
            && self.min_height <= self.height
            && self.scan_repeats > 0
            && self.verify_repeats > 0
            && self.polls_per_probe > 0
            && self.probe_operations > 0
    }
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Eye {
    pub start: Point,
    pub width: u8,
    pub height: u8,
    pub center: Point,
    /// Boundary contact means the physical eye may extend outside this scan.
    pub boundary_contact: bool,
}

/// At most 512 bytes. No heap, inferred lanes or guessed hardware values.
pub struct Grid {
    rows: [u64; 64],
}
impl Default for Grid {
    fn default() -> Self {
        Self { rows: [0; 64] }
    }
}
impl Grid {
    pub fn set(&mut self, p: Point, pass: bool) -> Result<(), Cause> {
        if p.x >= 64 || p.y >= 64 {
            return Err(Cause::InvalidPlan);
        }
        let mask = 1u64 << p.x;
        if pass {
            self.rows[p.y as usize] |= mask;
        } else {
            self.rows[p.y as usize] &= !mask;
        }
        Ok(())
    }
    pub fn get(&self, p: Point) -> bool {
        p.x < 64 && p.y < 64 && self.rows[p.y as usize] & (1u64 << p.x) != 0
    }
    /// Exhaustively maximize the area of an all-pass axis-aligned rectangle
    /// in grid cells, then its smaller dimension, then width; deterministic
    /// center/start ordering breaks ties. This is an exact discrete objective,
    /// not a claim of optimum voltage/phase margin in physical units.
    /// Wrapped axes search seams without counting any cell more than once.
    pub fn select(&self, d: Domain) -> Result<Eye, Cause> {
        if !d.valid() {
            return Err(Cause::InvalidPlan);
        }
        let w = usize::from(d.width);
        let h = usize::from(d.height);
        let mut best: Option<Eye> = None;
        for top in 0..h {
            let mut mask = u64::MAX;
            let max_height = if d.circular_y { h } else { h - top };
            for height in 1..=max_height {
                mask &= self.rows[(top + height - 1) % h];
                if height < usize::from(d.min_height) {
                    continue;
                }
                let mut run = 0usize;
                for i in 0..if d.circular_x { 2 * w } else { w } {
                    if mask & (1u64 << (i % w)) == 0 {
                        run = 0;
                        continue;
                    }
                    run = (run + 1).min(w);
                    if run < usize::from(d.min_width) {
                        continue;
                    }
                    let left = (i + 1 - run) % w;
                    let eye = Eye {
                        start: Point {
                            x: left as u8,
                            y: top as u8,
                        },
                        width: run as u8,
                        height: height as u8,
                        center: Point {
                            x: ((left + (run - 1) / 2) % w) as u8,
                            y: ((top + (height - 1) / 2) % h) as u8,
                        },
                        boundary_contact: (!d.circular_x && (left == 0 || left + run == w))
                            || (!d.circular_y && (top == 0 || top + height == h)),
                    };
                    if best.is_none_or(|old| better(eye, old)) {
                        best = Some(eye);
                    }
                }
            }
        }
        best.ok_or(Cause::NoWindow)
    }
}
fn better(a: Eye, b: Eye) -> bool {
    let score = |e: Eye| {
        (
            u16::from(e.width) * u16::from(e.height),
            e.width.min(e.height),
            e.width,
        )
    };
    score(a) > score(b)
        || (score(a) == score(b)
            && (a.center.y, a.center.x, a.start.y, a.start.x)
                < (b.center.y, b.center.x, b.start.y, b.start.x))
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BackendError {
    Unsupported,
    Fault,
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Cause {
    InvalidPlan,
    /// The backend programmed `point` for `request`'s knob and read another
    /// value in the knob's live copy ([`Backend::verify_live`]). The write
    /// did not take (a double-buffered CSR without its latch pulse...);
    /// nothing was measured at that point.
    NotLive {
        request: Request,
        point: Point,
    },
    Unsupported,
    BackendFault,
    Budget,
    Timeout,
    StaleObservation,
    MissingObservation,
    NoWindow,
    Verification,
}
impl From<BackendError> for Cause {
    fn from(e: BackendError) -> Self {
        match e {
            BackendError::Unsupported => Self::Unsupported,
            BackendError::Fault => Self::BackendFault,
        }
    }
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Failure {
    pub cause: Cause,
    pub rollback_failed: bool,
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Measurement {
    pub ticket: u32,
    pub observed_mask: u64,
    pub failed_mask: u64,
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Poll {
    Pending,
    Ready(Measurement),
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Probe {
    Scan(u8),
    Verify(u8),
}

/// Every method must be bounded by the backend, including reset/recovery.
/// domain() must reject unsupported stage/topology/state combinations BEFORE
/// any hardware operation. begin() owns saving and entering a training mode;
/// if it fails, it must restore any partial changes itself. apply() installs
/// the selected settings, while finish() leaves the training mode. rollback()
/// restores the pre-stage configuration on every post-begin failure.
///
/// Scan/verify are separate requests: a hardware backend must generate actual
/// traffic/patterns, collect fresh per-bit observations and honor dependencies.
/// Invented CSR readiness or a loopback of selected settings is not a backend.
///
/// Backends must prove the point is live or declare it unverifiable
/// ([`Backend::verify_live`]): reading back the CSR that was written is not
/// proof (A7A m13: double-buffered RxEnDly/RxClk read back the new value
/// while the live gate stayed put).
pub trait Backend {
    type Snapshot;
    fn domain(&self, req: Request) -> Result<Domain, BackendError>;
    fn begin(&mut self, req: Request) -> Result<Self::Snapshot, BackendError>;
    fn launch(&mut self, req: Request, p: Point, probe: Probe) -> Result<u32, BackendError>;
    fn poll(&mut self, ticket: u32) -> Result<Poll, BackendError>;
    fn apply(&mut self, req: Request, p: Point) -> Result<(), BackendError>;
    fn finish(&mut self, req: Request) -> Result<(), BackendError>;
    fn rollback(&mut self, req: Request, snapshot: Self::Snapshot) -> Result<(), BackendError>;
    /// Called by the engines right after every `launch` and `apply` that
    /// programmed `p`: read the copy of the knob the data path uses (the
    /// live delay through its selector, the mode register through MRR, the
    /// Vref DAC the receiver sees...), not the CSR that was written, and
    /// answer whether it holds `p`. A knob without a live readback answers
    /// [`Live::Unverifiable`], which the default does; the report then says
    /// "knob not verifiable". Reads only; no writes, no latch pulses (those
    /// belong in `launch`/`apply`).
    fn verify_live(&mut self, _req: Request, _p: Point) -> Result<Live, BackendError> {
        Ok(Live::Unverifiable)
    }
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ResultState {
    pub request: Request,
    pub eye: Eye,
    pub probe_operations: u32,
    /// Point programmings confirmed live ("points verified live: N/M").
    pub live: Liveness,
}
struct Budget {
    left: u32,
    used: u32,
}
impl Budget {
    fn take(&mut self) -> Result<(), Cause> {
        if self.left == 0 {
            return Err(Cause::Budget);
        }
        self.left -= 1;
        self.used += 1;
        Ok(())
    }
}
/// After a `launch`/`apply` programmed `p`: the live check. A stale point is
/// the hard error [`Cause::NotLive`]; the read costs no probe budget.
fn confirm_live<B: Backend>(
    b: &mut B,
    req: Request,
    p: Point,
    live: &mut Liveness,
) -> Result<(), Cause> {
    live.record(b.verify_live(req, p)?)
        .map_err(|_| Cause::NotLive {
            request: req,
            point: p,
        })
}
fn sample<B: Backend>(
    b: &mut B,
    req: Request,
    p: Point,
    probe: Probe,
    d: Domain,
    budget: &mut Budget,
    live: &mut Liveness,
) -> Result<bool, Cause> {
    budget.take()?;
    let ticket = b.launch(req, p, probe)?;
    confirm_live(b, req, p, live)?;
    for _ in 0..d.polls_per_probe {
        budget.take()?;
        match b.poll(ticket)? {
            Poll::Pending => {}
            Poll::Ready(m) => {
                if m.ticket != ticket {
                    return Err(Cause::StaleObservation);
                }
                if m.observed_mask & req.required_mask != req.required_mask {
                    return Err(Cause::MissingObservation);
                }
                return Ok(m.failed_mask & req.required_mask == 0);
            }
        }
    }
    Err(Cause::Timeout)
}
pub fn train<B: Backend>(b: &mut B, req: Request) -> Result<ResultState, Failure> {
    let fail = |cause| Failure {
        cause,
        rollback_failed: false,
    };
    if req.pstate >= MAX_PSTATES || req.required_mask == 0 {
        return Err(fail(Cause::InvalidPlan));
    }
    let d = b.domain(req).map_err(|e| fail(e.into()))?;
    train_in_domain(b, req, d)
}
pub(crate) fn train_in_domain<B: Backend>(
    b: &mut B,
    req: Request,
    d: Domain,
) -> Result<ResultState, Failure> {
    train_using(b, req, d, |grid, domain| grid.select(domain))
}
pub(crate) fn train_using<B: Backend, F: FnOnce(&Grid, Domain) -> Result<Eye, Cause>>(
    b: &mut B,
    req: Request,
    d: Domain,
    choose: F,
) -> Result<ResultState, Failure> {
    let fail = |cause| Failure {
        cause,
        rollback_failed: false,
    };
    if req.pstate >= MAX_PSTATES || req.required_mask == 0 {
        return Err(fail(Cause::InvalidPlan));
    }
    if !d.valid() {
        return Err(fail(Cause::InvalidPlan));
    }
    let snapshot = b.begin(req).map_err(|e| fail(e.into()))?;
    let mut budget = Budget {
        left: d.probe_operations,
        used: 0,
    };
    let mut live = Liveness::default();
    let operation = (|| {
        let mut grid = Grid::default();
        for y in 0..d.height {
            for x in 0..d.width {
                let p = Point { x, y };
                let mut pass = true;
                // Always run all repeats; don't hide instability by early exit.
                for n in 0..d.scan_repeats {
                    pass &= sample(b, req, p, Probe::Scan(n), d, &mut budget, &mut live)?;
                }
                grid.set(p, pass)?;
            }
        }
        let eye = choose(&grid, d)?;
        b.apply(req, eye.center)?;
        confirm_live(b, req, eye.center, &mut live)?;
        for n in 0..d.verify_repeats {
            if !sample(
                b,
                req,
                eye.center,
                Probe::Verify(n),
                d,
                &mut budget,
                &mut live,
            )? {
                return Err(Cause::Verification);
            }
        }
        b.finish(req)?;
        Ok(ResultState {
            request: req,
            eye,
            probe_operations: budget.used,
            live,
        })
    })();
    operation.map_err(|cause| Failure {
        cause,
        rollback_failed: b.rollback(req, snapshot).is_err(),
    })
}

/// Explicit unavailable adapter for a board without a reviewed backend. No
/// reads or writes. This keeps a caller from interpreting the host algorithms
/// as a hardware driver.
pub struct Unbound;
impl Backend for Unbound {
    type Snapshot = ();
    fn domain(&self, _: Request) -> Result<Domain, BackendError> {
        Err(BackendError::Unsupported)
    }
    fn begin(&mut self, _: Request) -> Result<(), BackendError> {
        Err(BackendError::Unsupported)
    }
    fn launch(&mut self, _: Request, _: Point, _: Probe) -> Result<u32, BackendError> {
        Err(BackendError::Unsupported)
    }
    fn poll(&mut self, _: u32) -> Result<Poll, BackendError> {
        Err(BackendError::Unsupported)
    }
    fn apply(&mut self, _: Request, _: Point) -> Result<(), BackendError> {
        Err(BackendError::Unsupported)
    }
    fn finish(&mut self, _: Request) -> Result<(), BackendError> {
        Err(BackendError::Unsupported)
    }
    fn rollback(&mut self, _: Request, _: ()) -> Result<(), BackendError> {
        Err(BackendError::Unsupported)
    }
}
