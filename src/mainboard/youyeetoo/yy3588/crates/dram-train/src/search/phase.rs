// SPDX-License-Identifier: GPL-2.0-only
//! Original CPU decision policies. Axes must be supplied with reviewed physical
//! meaning by a backend; these policies do not establish A733 encodings/units,
//! commands, DRAM training modes, traffic, feedback or stage dependency order.
use crate::search::{
    train_using, Backend, Cause, Domain, Eye, Failure, Grid, Point, Request, ResultState, Stage,
};

#[derive(Clone, Copy, Debug)]
pub enum Policy<'a> {
    /// Timing x DRAM Vref: maximize an all-bit, repeat-stable joint window.
    Ca,
    /// A complete circular phase domain, one row.
    Wck2Ck,
    /// A linear, ordered latency domain, one row. The backend must correctly
    /// linearize coarse/fine settings and reject unavailable/discontinuous bins.
    ReadGate,
    ReadCenter {
        circular: bool,
    },
    WriteCenter {
        circular: bool,
    },
    /// Timing x receiver Vref, with reviewed receiver-Vref units and codes.
    ReadVref,
    /// Timing x DRAM Vref, with reviewed DRAM-part limits and MR issue flow.
    WriteVref,
    /// Each row is one independently encoded coefficient/tap candidate. Codes
    /// are not assumed adjacent or monotonic. Costs only break equal-eye ties.
    Dfe {
        candidate_costs: &'a [u16],
    },
    /// Each row is an independently encoded duty-adjustment candidate.
    Dca {
        candidate_costs: &'a [u16],
    },
}
impl Policy<'_> {
    pub const fn stage(self) -> Stage {
        match self {
            Self::Ca => Stage::Ca,
            Self::Wck2Ck => Stage::Wck2Ck,
            Self::ReadGate => Stage::ReadGate,
            Self::ReadCenter { .. } => Stage::ReadCenter,
            Self::WriteCenter { .. } => Stage::WriteCenter,
            Self::ReadVref => Stage::ReadVref,
            Self::WriteVref => Stage::WriteVref,
            Self::Dfe { .. } => Stage::Dfe,
            Self::Dca { .. } => Stage::Dca,
        }
    }
    pub fn validate(self, d: Domain) -> Result<(), Cause> {
        if !d.valid() {
            return Err(Cause::InvalidPlan);
        }
        let valid = match self {
            Self::Wck2Ck => d.height == 1 && d.circular_x && !d.circular_y,
            Self::ReadGate => d.height == 1 && !d.circular_x && !d.circular_y,
            Self::ReadCenter { circular } | Self::WriteCenter { circular } => {
                d.height == 1 && d.circular_x == circular && !d.circular_y
            }
            Self::Ca | Self::ReadVref | Self::WriteVref => !d.circular_y,
            Self::Dfe { candidate_costs } | Self::Dca { candidate_costs } => {
                candidate_costs.len() == usize::from(d.height) && d.min_height == 1 && !d.circular_y
            }
        };
        if valid {
            Ok(())
        } else {
            Err(Cause::InvalidPlan)
        }
    }
    pub fn select(self, grid: &Grid, d: Domain) -> Result<Eye, Cause> {
        self.validate(d)?;
        match self {
            Self::Dfe { candidate_costs } | Self::Dca { candidate_costs } => {
                select_candidate(grid, d, candidate_costs)
            }
            _ => grid.select(d),
        }
    }
}
fn select_candidate(grid: &Grid, d: Domain, costs: &[u16]) -> Result<Eye, Cause> {
    // Each coefficient/duty candidate is an independent timing line. Reuse
    // the constant-memory interval scan rather than a second512-byte Grid.
    let row_domain = crate::search::line::LineDomain {
        points: u16::from(d.width),
        circular: d.circular_x,
        min_span: u16::from(d.min_width),
        scan_repeats: d.scan_repeats,
        verify_repeats: d.verify_repeats,
        polls_per_probe: d.polls_per_probe,
        probe_operations: d.probe_operations,
    };
    let mut best: Option<Eye> = None;
    for row in 0..d.height {
        let mut interval = crate::search::line::Interval::new(row_domain)?;
        for x in 0..d.width {
            interval.push(grid.get(Point { x, y: row }))?;
        }
        let line = match interval.finish() {
            Ok(eye) => eye,
            Err(Cause::NoWindow) => continue,
            Err(error) => return Err(error),
        };
        let eye = Eye {
            start: Point {
                x: line.start as u8,
                y: row,
            },
            width: line.span as u8,
            height: 1,
            center: Point {
                x: line.center,
                y: row,
            },
            boundary_contact: line.boundary_contact,
        };
        let better = best.is_none_or(|old| {
            eye.width > old.width
                || (eye.width == old.width
                    && (costs[usize::from(row)], eye.center.x, row, eye.start.x)
                        < (
                            costs[usize::from(old.center.y)],
                            old.center.x,
                            old.center.y,
                            old.start.x,
                        ))
        });
        if better {
            best = Some(eye);
        }
    }
    best.ok_or(Cause::NoWindow)
}
/// Execute a specific CPU decision policy using the ordinary bounded scan,
/// all-bit/repeat intersection, application, fresh verification and recovery.
/// Policy/domain mismatches are rejected before begin(). Physical phase entry,
/// measurement production and exit remain the backend's reviewed responsibility.
pub fn train_phase<B: Backend>(
    backend: &mut B,
    request: Request,
    policy: Policy<'_>,
) -> Result<ResultState, Failure> {
    let fail = |cause| Failure {
        cause,
        rollback_failed: false,
    };
    if request.stage != policy.stage()
        || request.pstate >= crate::search::MAX_PSTATES
        || request.required_mask == 0
    {
        return Err(fail(Cause::InvalidPlan));
    }
    let domain = backend.domain(request).map_err(|e| fail(e.into()))?;
    policy.validate(domain).map_err(fail)?;
    train_using(backend, request, domain, |grid, d| policy.select(grid, d))
}
