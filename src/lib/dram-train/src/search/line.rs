// SPDX-License-Identifier: GPL-2.0-only
//! Original constant-memory one-dimensional training over up to256 explicit
//! settings. Coordinates are backend-defined codes, not inferred physical units.
//! Static PMU acquisition loops of128/212 iterations motivated removing a64-cell
//! software limit; that correspondence does not prove phase range or period.
use crate::search::{
    confirm_live, sample, Backend, BackendError, Budget, Cause, Domain, Failure, Liveness, Point,
    Probe, Request, Stage,
};

#[derive(Clone, Copy, Debug)]
pub struct LineDomain {
    pub points: u16,
    pub circular: bool,
    pub min_span: u16,
    pub scan_repeats: u8,
    pub verify_repeats: u8,
    pub polls_per_probe: u32,
    pub probe_operations: u32,
}
impl LineDomain {
    pub fn valid(self) -> bool {
        (1..=256).contains(&self.points)
            && self.min_span > 0
            && self.min_span <= self.points
            && self.scan_repeats > 0
            && self.verify_repeats > 0
            && self.polls_per_probe > 0
            && self.probe_operations > 0
    }
    fn probe_limits(self) -> Domain {
        // sample() only uses polls_per_probe. A one-cell Domain carries the
        // existing bounded probe contract; it is never used to limit this line.
        Domain {
            width: 1,
            height: 1,
            circular_x: false,
            circular_y: false,
            min_width: 1,
            min_height: 1,
            scan_repeats: self.scan_repeats,
            verify_repeats: self.verify_repeats,
            polls_per_probe: self.polls_per_probe,
            probe_operations: self.probe_operations,
        }
    }
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct LineEye {
    pub start: u16,
    pub span: u16,
    pub center: u8,
    pub boundary_contact: bool,
}
/// Streaming longest stable common interval. A single pass needs constant
/// memory; circular finalization joins only the final suffix and first prefix.
/// Ties choose the smallest center, then start. An all-pass circle chooses
/// center0 explicitly because every phase is equally supported by this data.
pub struct Interval {
    points: u16,
    min_span: u16,
    circular: bool,
    seen: u16,
    prefix: u16,
    run: u16,
    best: Option<LineEye>,
}
impl Interval {
    pub fn new(d: LineDomain) -> Result<Self, Cause> {
        if !d.valid() {
            return Err(Cause::InvalidPlan);
        }
        Ok(Self {
            points: d.points,
            min_span: d.min_span,
            circular: d.circular,
            seen: 0,
            prefix: 0,
            run: 0,
            best: None,
        })
    }
    fn consider(&mut self, start: u16, span: u16) {
        if span < self.min_span {
            return;
        }
        let center = ((start + (span - 1) / 2) % self.points) as u8;
        let eye = LineEye {
            start,
            span,
            center,
            boundary_contact: !self.circular && (start == 0 || start + span == self.points),
        };
        if self.best.is_none_or(|old| {
            span > old.span || (span == old.span && (center, start) < (old.center, old.start))
        }) {
            self.best = Some(eye);
        }
    }
    pub fn push(&mut self, pass: bool) -> Result<(), Cause> {
        if self.seen >= self.points {
            return Err(Cause::InvalidPlan);
        }
        if pass {
            if self.seen == self.prefix {
                self.prefix += 1;
            }
            self.run += 1;
            self.consider(self.seen + 1 - self.run, self.run);
        } else {
            self.run = 0;
        }
        self.seen += 1;
        Ok(())
    }
    pub fn finish(mut self) -> Result<LineEye, Cause> {
        if self.seen != self.points {
            return Err(Cause::InvalidPlan);
        }
        if self.circular && self.prefix > 0 && self.run > 0 {
            let span = (self.prefix + self.run).min(self.points);
            let start = if span == self.points {
                (self.points - (span - 1) / 2) % self.points
            } else {
                self.points - self.run
            };
            self.consider(start, span);
        }
        self.best.ok_or(Cause::NoWindow)
    }
}
/// The ordinary Backend entry/launch/poll/apply/exit/recovery contract remains
/// mandatory. launch/apply x is the explicit setting index, y0. line_domain()
/// must reject unsupported actual hardware paths before any hardware operation.
pub trait LineBackend: Backend {
    fn line_domain(&self, request: Request) -> Result<LineDomain, BackendError>;
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct LineResult {
    pub request: Request,
    pub eye: LineEye,
    pub probe_operations: u32,
    /// Point programmings confirmed live ("points verified live: N/M").
    pub live: Liveness,
}
/// Original WCK/ordered-latency/read/write centering policy. Physical training
/// entry, actual measurement, index encoding and transitions remain unbound.
/// CA/Vref/DFE/DCA require their reviewed multidimensional/candidate policies.
pub fn train_line<B: LineBackend>(
    backend: &mut B,
    request: Request,
) -> Result<LineResult, Failure> {
    let fail = |cause| Failure {
        cause,
        rollback_failed: false,
    };
    if request.pstate >= crate::search::MAX_PSTATES
        || request.required_mask == 0
        || !matches!(
            request.stage,
            Stage::Wck2Ck | Stage::ReadGate | Stage::ReadCenter | Stage::WriteCenter
        )
    {
        return Err(fail(Cause::InvalidPlan));
    }
    let d = backend.line_domain(request).map_err(|e| fail(e.into()))?;
    train_line_using(backend, request, d)
}
pub(crate) fn train_line_using<B: LineBackend>(
    backend: &mut B,
    request: Request,
    d: LineDomain,
) -> Result<LineResult, Failure> {
    let fail = |cause| Failure {
        cause,
        rollback_failed: false,
    };
    if request.pstate >= crate::search::MAX_PSTATES
        || request.required_mask == 0
        || !matches!(
            request.stage,
            Stage::Wck2Ck | Stage::ReadGate | Stage::ReadCenter | Stage::WriteCenter
        )
    {
        return Err(fail(Cause::InvalidPlan));
    }
    if !d.valid()
        || (request.stage == Stage::Wck2Ck && !d.circular)
        || (request.stage == Stage::ReadGate && d.circular)
    {
        return Err(fail(Cause::InvalidPlan));
    }
    let mut interval = Interval::new(d).map_err(fail)?;
    let snapshot = backend.begin(request).map_err(|e| fail(e.into()))?;
    let mut budget = Budget {
        left: d.probe_operations,
        used: 0,
    };
    let limits = d.probe_limits();
    let mut live = Liveness::default();
    let operation = (|| {
        for x in 0..d.points {
            let p = Point { x: x as u8, y: 0 };
            let mut pass = true;
            for repeat in 0..d.scan_repeats {
                pass &= sample(
                    backend,
                    request,
                    p,
                    Probe::Scan(repeat),
                    limits,
                    &mut budget,
                    &mut live,
                )?;
            }
            interval.push(pass)?;
        }
        let eye = interval.finish()?;
        let p = Point {
            x: eye.center,
            y: 0,
        };
        backend.apply(request, p)?;
        confirm_live(backend, request, p, &mut live)?;
        for repeat in 0..d.verify_repeats {
            if !sample(
                backend,
                request,
                p,
                Probe::Verify(repeat),
                limits,
                &mut budget,
                &mut live,
            )? {
                return Err(Cause::Verification);
            }
        }
        backend.finish(request)?;
        Ok(LineResult {
            request,
            eye,
            probe_operations: budget.used,
            live,
        })
    })();
    operation.map_err(|cause| Failure {
        cause,
        rollback_failed: backend.rollback(request, snapshot).is_err(),
    })
}
impl LineBackend for crate::search::Unbound {
    fn line_domain(&self, _: Request) -> Result<LineDomain, BackendError> {
        Err(BackendError::Unsupported)
    }
}
