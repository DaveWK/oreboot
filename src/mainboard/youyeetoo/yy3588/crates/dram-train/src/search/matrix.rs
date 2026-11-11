// SPDX-License-Identifier: GPL-2.0-only
//! Original streaming joint timing/Vref search, up to256 explicit codes per
//! axis. No whole matrix is stored, no physical units/part limits are inferred.
use crate::search::{
    confirm_live, sample, Backend, BackendError, Budget, Cause, Domain, Failure, Liveness, Point,
    Probe, Request, Stage,
};
#[derive(Clone, Copy, Debug)]
pub struct MatrixDomain {
    pub width: u16,
    pub height: u16,
    pub circular_timing: bool,
    pub min_width: u16,
    pub min_height: u16,
    pub scan_repeats: u8,
    pub verify_repeats: u8,
    pub polls_per_probe: u32,
    pub probe_operations: u32,
}
impl MatrixDomain {
    pub fn valid(self) -> bool {
        (1..=256).contains(&self.width)
            && (1..=256).contains(&self.height)
            && self.min_width > 0
            && self.min_width <= self.width
            && self.min_height > 0
            && self.min_height <= self.height
            && self.scan_repeats > 0
            && self.verify_repeats > 0
            && self.polls_per_probe > 0
            && self.probe_operations > 0
    }
    fn limits(self) -> Domain {
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
pub struct MatrixEye {
    pub start: Point,
    pub width: u16,
    pub height: u16,
    pub center: Point,
    pub boundary_contact: bool,
}
#[derive(Clone, Copy, Default)]
struct Bar {
    start: u16,
    height: u16,
}
/// Streaming largest all-pass rectangle: vertical run heights plus a monotone
/// histogram stack per row. Vref is linear; timing may wrap. Repeating a timing
/// row twice finds seams, capping width at the domain to avoid duplicate cells.
/// Full-width circular ties choose center0 explicitly. Objective matches Grid:
/// area, smaller dimension, width, then deterministic center/start ordering.
pub struct Rectangle {
    domain: MatrixDomain,
    heights: [u16; 256],
    x: u16,
    rows: u16,
    best: Option<MatrixEye>,
}
impl Rectangle {
    pub fn new(domain: MatrixDomain) -> Result<Self, Cause> {
        if !domain.valid() {
            return Err(Cause::InvalidPlan);
        }
        Ok(Self {
            domain,
            heights: [0; 256],
            x: 0,
            rows: 0,
            best: None,
        })
    }
    pub fn push(&mut self, pass: bool) -> Result<(), Cause> {
        if self.rows >= self.domain.height {
            return Err(Cause::InvalidPlan);
        }
        let h = &mut self.heights[usize::from(self.x)];
        *h = if pass { *h + 1 } else { 0 };
        self.x += 1;
        if self.x == self.domain.width {
            self.select_row();
            self.rows += 1;
            self.x = 0;
        }
        Ok(())
    }
    fn consider(&mut self, start: u16, end: u16, height: u16) {
        let d = self.domain;
        let width = (end - start).min(d.width);
        if width < d.min_width || height < d.min_height {
            return;
        }
        let left = if d.circular_timing && width == d.width {
            (d.width - (width - 1) / 2) % d.width
        } else {
            (end - width) % d.width
        };
        let top = self.rows + 1 - height;
        let eye = MatrixEye {
            start: Point {
                x: left as u8,
                y: top as u8,
            },
            width,
            height,
            center: Point {
                x: ((left + (width - 1) / 2) % d.width) as u8,
                y: (top + (height - 1) / 2) as u8,
            },
            boundary_contact: (!d.circular_timing && (left == 0 || left + width == d.width))
                || top == 0
                || top + height == d.height,
        };
        let score = |e: MatrixEye| {
            (
                u32::from(e.width) * u32::from(e.height),
                e.width.min(e.height),
                e.width,
            )
        };
        if self.best.is_none_or(|old| {
            score(eye) > score(old)
                || (score(eye) == score(old)
                    && (eye.center.y, eye.center.x, eye.start.y, eye.start.x)
                        < (old.center.y, old.center.x, old.start.y, old.start.x))
        }) {
            self.best = Some(eye);
        }
    }
    fn select_row(&mut self) {
        // Strictly increasing positive heights imply at most256 stack entries,
        // even while traversing512 timing positions in a circular domain.
        let mut bars = [Bar::default(); 256];
        let mut len = 0usize;
        let end = if self.domain.circular_timing {
            2 * self.domain.width
        } else {
            self.domain.width
        };
        for i in 0..=end {
            let height = if i == end {
                0
            } else {
                self.heights[usize::from(i % self.domain.width)]
            };
            let mut start = i;
            while len > 0 && bars[len - 1].height > height {
                len -= 1;
                let bar = bars[len];
                self.consider(bar.start, i, bar.height);
                start = bar.start;
            }
            if height > 0 && (len == 0 || bars[len - 1].height < height) {
                bars[len] = Bar { start, height };
                len += 1;
            }
        }
    }
    pub fn finish(&self) -> Result<MatrixEye, Cause> {
        if self.rows != self.domain.height || self.x != 0 {
            return Err(Cause::InvalidPlan);
        }
        self.best.ok_or(Cause::NoWindow)
    }
}
pub trait MatrixBackend: Backend {
    /// Read-only reviewed domain. Default unsupported does not narrow/downsample.
    fn matrix_domain(&self, _: Request) -> Result<MatrixDomain, BackendError> {
        Err(BackendError::Unsupported)
    }
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct MatrixResult {
    pub request: Request,
    pub eye: MatrixEye,
    pub probe_operations: u32,
    /// Point programmings confirmed live ("points verified live: N/M").
    pub live: Liveness,
}
pub fn train<B: MatrixBackend>(b: &mut B, r: Request) -> Result<MatrixResult, Failure> {
    let fail = |cause| Failure {
        cause,
        rollback_failed: false,
    };
    if !valid_request(r) {
        return Err(fail(Cause::InvalidPlan));
    }
    let d = b.matrix_domain(r).map_err(|e| fail(e.into()))?;
    train_using(b, r, d)
}
fn valid_request(r: Request) -> bool {
    r.pstate < 4
        && r.required_mask != 0
        && matches!(r.stage, Stage::Ca | Stage::ReadVref | Stage::WriteVref)
}
pub(crate) fn train_using<B: MatrixBackend>(
    b: &mut B,
    r: Request,
    d: MatrixDomain,
) -> Result<MatrixResult, Failure> {
    let fail = |cause| Failure {
        cause,
        rollback_failed: false,
    };
    if !valid_request(r) || !d.valid() {
        return Err(fail(Cause::InvalidPlan));
    }
    let mut rectangle = Rectangle::new(d).map_err(fail)?;
    let old = b.begin(r).map_err(|e| fail(e.into()))?;
    let mut budget = Budget {
        left: d.probe_operations,
        used: 0,
    };
    let limits = d.limits();
    let mut live = Liveness::default();
    let operation = (|| {
        for y in 0..d.height {
            for x in 0..d.width {
                let p = Point {
                    x: x as u8,
                    y: y as u8,
                };
                let mut pass = true;
                for n in 0..d.scan_repeats {
                    pass &= sample(b, r, p, Probe::Scan(n), limits, &mut budget, &mut live)?;
                }
                rectangle.push(pass)?;
            }
        }
        let eye = rectangle.finish()?;
        b.apply(r, eye.center)?;
        confirm_live(b, r, eye.center, &mut live)?;
        for n in 0..d.verify_repeats {
            if !sample(
                b,
                r,
                eye.center,
                Probe::Verify(n),
                limits,
                &mut budget,
                &mut live,
            )? {
                return Err(Cause::Verification);
            }
        }
        b.finish(r)?;
        Ok(MatrixResult {
            request: r,
            eye,
            probe_operations: budget.used,
            live,
        })
    })();
    operation.map_err(|cause| Failure {
        cause,
        rollback_failed: b.rollback(r, old).is_err(),
    })
}
impl MatrixBackend for crate::search::Unbound {}
