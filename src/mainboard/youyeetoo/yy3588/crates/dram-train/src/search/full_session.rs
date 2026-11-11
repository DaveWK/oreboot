// SPDX-License-Identifier: GPL-2.0-only
//! Mixed line/grid/candidate orchestration with explicit reviewed applicability.
//! No physical topology, stage order, encodings or exclusion evidence is invented.
use crate::search::{
    line::{self, LineBackend, LineDomain, LineResult},
    matrix::{self, MatrixBackend, MatrixDomain, MatrixResult},
    phase,
    session::{self, Applicability, Plan, SessionFailure, SessionResult},
    BackendError, Cause, Domain, Failure, Liveness, Request, ResultState, Stage,
};

#[derive(Clone, Copy, Debug)]
pub enum LinePolicy {
    Wck2Ck,
    ReadGate,
    ReadCenter { circular: bool },
    WriteCenter { circular: bool },
}
impl LinePolicy {
    fn stage(self) -> Stage {
        match self {
            Self::Wck2Ck => Stage::Wck2Ck,
            Self::ReadGate => Stage::ReadGate,
            Self::ReadCenter { .. } => Stage::ReadCenter,
            Self::WriteCenter { .. } => Stage::WriteCenter,
        }
    }
    fn validate(self, d: LineDomain) -> Result<(), Cause> {
        let circular = match self {
            Self::Wck2Ck => true,
            Self::ReadGate => false,
            Self::ReadCenter { circular } | Self::WriteCenter { circular } => circular,
        };
        if d.valid() && d.circular == circular {
            Ok(())
        } else {
            Err(Cause::InvalidPlan)
        }
    }
}
#[derive(Clone, Copy, Debug)]
pub enum MatrixPolicy {
    Ca,
    ReadVref,
    WriteVref,
}
impl MatrixPolicy {
    fn stage(self) -> Stage {
        match self {
            Self::Ca => Stage::Ca,
            Self::ReadVref => Stage::ReadVref,
            Self::WriteVref => Stage::WriteVref,
        }
    }
}
#[derive(Clone, Copy, Debug)]
pub enum Policy<'a> {
    Grid(phase::Policy<'a>),
    Line(LinePolicy),
    Matrix(MatrixPolicy),
}
impl Policy<'_> {
    fn stage(self) -> Stage {
        match self {
            Self::Grid(p) => p.stage(),
            Self::Line(p) => p.stage(),
            Self::Matrix(p) => p.stage(),
        }
    }
    fn domain<B: LineBackend + MatrixBackend>(self, b: &B, r: Request) -> Result<Space, Cause> {
        if self.stage() != r.stage {
            return Err(Cause::InvalidPlan);
        }
        match self {
            Self::Matrix(_) => {
                let d = b.matrix_domain(r)?;
                if !d.valid() {
                    return Err(Cause::InvalidPlan);
                }
                Ok(Space::Matrix(d))
            }
            Self::Grid(p) => {
                let d = b.domain(r)?;
                p.validate(d)?;
                Ok(Space::Grid(d))
            }
            Self::Line(p) => {
                let d = b.line_domain(r)?;
                p.validate(d)?;
                Ok(Space::Line(d))
            }
        }
    }
}
enum Space {
    Grid(Domain),
    Line(LineDomain),
    Matrix(MatrixDomain),
}
impl Space {
    fn allowance(&self) -> u64 {
        u64::from(match self {
            Self::Grid(d) => d.probe_operations,
            Self::Line(d) => d.probe_operations,
            Self::Matrix(d) => d.probe_operations,
        })
    }
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Trained {
    Grid(ResultState),
    Line(LineResult),
    Matrix(MatrixResult),
}
impl Trained {
    pub fn request(self) -> Request {
        match self {
            Self::Grid(r) => r.request,
            Self::Line(r) => r.request,
            Self::Matrix(r) => r.request,
        }
    }
    pub fn probe_operations(self) -> u32 {
        match self {
            Self::Grid(r) => r.probe_operations,
            Self::Line(r) => r.probe_operations,
            Self::Matrix(r) => r.probe_operations,
        }
    }
    pub fn live(self) -> Liveness {
        match self {
            Self::Grid(r) => r.live,
            Self::Line(r) => r.live,
            Self::Matrix(r) => r.live,
        }
    }
}
/// Same whole-session snapshot/transition/recovery responsibilities as the
/// grid session. preflight MUST check each evidence identity and applicability
/// against reviewed actual topology/part/state data, with no hardware access.
/// Every method must be bounded. Failed begin restores partial changes itself.
/// finish_session must consume both result variants without narrowing span256.
pub trait Backend: LineBackend + MatrixBackend {
    type SessionSnapshot;
    fn preflight(
        &self,
        plan: &Plan<'_>,
        rules: &[Applicability<'_>],
        policies: &[Policy<'_>],
    ) -> Result<(), BackendError>;
    fn begin_session(&mut self) -> Result<Self::SessionSnapshot, BackendError>;
    fn switch_pstate(&mut self, state: u8) -> Result<(), BackendError>;
    fn finish_session(&mut self, results: &[Option<Trained>]) -> Result<(), BackendError>;
    fn rollback_session(&mut self, old: Self::SessionSnapshot) -> Result<(), BackendError>;
}
fn failure(cause: Cause, step: Option<usize>) -> SessionFailure {
    SessionFailure {
        cause,
        step,
        stage_rollback_failed: false,
        session_rollback_failed: false,
    }
}
/// Each step has exactly one supplied policy, no generic fallback or automatic
/// downsampling. Validate coverage/dependencies, reviewed exclusions, policies,
/// domains and summed allowance BEFORE entry. Revalidate after each transition
/// and use that exact active-state domain for the stage. Outputs are invalidated
/// after any failure, including commit; both recovery failures remain visible.
pub fn run<B: Backend>(
    b: &mut B,
    plan: &Plan<'_>,
    rules: &[Applicability<'_>],
    policies: &[Policy<'_>],
    outputs: &mut [Option<Trained>],
) -> Result<SessionResult, SessionFailure> {
    outputs.fill(None);
    session::validate_reviewed(plan, rules).map_err(|e| failure(e, None))?;
    if outputs.len() != plan.steps.len() || policies.len() != plan.steps.len() {
        return Err(failure(Cause::InvalidPlan, None));
    }
    b.preflight(plan, rules, policies)
        .map_err(|e| failure(e.into(), None))?;
    let mut allowance = 0u64;
    for (i, step) in plan.steps.iter().enumerate() {
        let space = policies[i]
            .domain(b, step.request)
            .map_err(|e| failure(e, Some(i)))?;
        allowance = allowance
            .checked_add(space.allowance())
            .ok_or_else(|| failure(Cause::Budget, Some(i)))?;
        if allowance > plan.operation_budget {
            return Err(failure(Cause::Budget, Some(i)));
        }
    }
    let old = b.begin_session().map_err(|e| failure(e.into(), None))?;
    let mut current = None;
    let mut receipt = SessionResult {
        steps: 0,
        transitions: 0,
        probe_operations: 0,
        live: Liveness::default(),
    };
    let operation = (|| {
        for (i, step) in plan.steps.iter().enumerate() {
            let r = step.request;
            if current != Some(r.pstate) {
                b.switch_pstate(r.pstate)
                    .map_err(|e| failure(e.into(), Some(i)))?;
                current = Some(r.pstate);
                receipt.transitions += 1;
            }
            let space = policies[i].domain(b, r).map_err(|e| failure(e, Some(i)))?;
            if space.allowance() > plan.operation_budget - receipt.probe_operations {
                return Err(failure(Cause::Budget, Some(i)));
            }
            let result = match (policies[i], space) {
                (Policy::Grid(p), Space::Grid(d)) => {
                    crate::search::train_using(b, r, d, |grid, domain| p.select(grid, domain))
                        .map(Trained::Grid)
                }
                (Policy::Line(_), Space::Line(d)) => {
                    line::train_line_using(b, r, d).map(Trained::Line)
                }
                (Policy::Matrix(_), Space::Matrix(d)) => {
                    matrix::train_using(b, r, d).map(Trained::Matrix)
                }
                _ => unreachable!(),
            }
            .map_err(
                |Failure {
                     cause,
                     rollback_failed,
                 }| {
                    let mut e = failure(cause, Some(i));
                    e.stage_rollback_failed = rollback_failed;
                    e
                },
            )?;
            receipt.probe_operations += u64::from(result.probe_operations());
            receipt.live.merge(result.live());
            receipt.steps += 1;
            outputs[i] = Some(result);
        }
        b.finish_session(outputs)
            .map_err(|e| failure(e.into(), None))?;
        Ok(receipt)
    })();
    operation.map_err(|mut e: SessionFailure| {
        outputs.fill(None);
        e.session_rollback_failed = b.rollback_session(old).is_err();
        e
    })
}
