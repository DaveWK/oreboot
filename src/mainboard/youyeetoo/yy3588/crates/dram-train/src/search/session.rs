// SPDX-License-Identifier: GPL-2.0-only
//! Original all-pstate orchestration. An explicit plan declares targets and
//! dependencies; it never infers physical topology or a JEDEC/A733 stage order.
use crate::search::{
    train_using, Backend, BackendError, Cause, Failure, Liveness, Request, ResultState, Stage,
};

pub const ALL_STAGES: u16 = (1 << 9) - 1;
pub const fn stage_bit(stage: Stage) -> u16 {
    match stage {
        Stage::Ca => 1,
        Stage::Wck2Ck => 2,
        Stage::ReadGate => 4,
        Stage::ReadCenter => 8,
        Stage::WriteCenter => 16,
        Stage::ReadVref => 32,
        Stage::WriteVref => 64,
        Stage::Dfe => 128,
        Stage::Dca => 256,
    }
}
#[derive(Clone, Copy, Debug)]
pub struct Target {
    pub pstate: u8,
    pub channel: u8,
    pub rank: u8,
    pub lane: u8,
    pub required_mask: u64,
    /// Required stages for exactly this target. No automatic lane expansion.
    pub stages: u16,
}
impl Target {
    fn location(self, req: Request) -> bool {
        (self.pstate, self.channel, self.rank, self.lane)
            == (req.pstate, req.channel, req.rank, req.lane)
    }
    fn matches(self, req: Request) -> bool {
        self.location(req) && self.required_mask == req.required_mask
    }
}
#[derive(Clone, Copy, Debug)]
pub struct Step {
    pub request: Request,
    /// These stages must already have completed for exactly the same target.
    /// Zero is permitted only if the reviewed plan establishes no dependency.
    pub prerequisites: u16,
}
pub struct Plan<'a> {
    pub targets: &'a [Target],
    pub steps: &'a [Step],
    /// Maximum total declared launch+poll allowance. Not elapsed time.
    pub operation_budget: u64,
}

/// Every method must be bounded, including preflight and recovery. preflight
/// must validate physical topology, stage ordering, encodings and transitions
/// without hardware access. begin_session snapshots all settings needed for
/// whole-plan recovery and restores partial changes itself if it returns Err.
/// switch_pstate must perform the reviewed transition, never a guessed divisor.
/// finish_session checks/activates the complete trained result set. A failure
/// anywhere after begin_session restores the whole snapshot, including states
/// completed earlier. Per-stage rollback remains independently enforced.
pub trait SessionBackend: Backend {
    type SessionSnapshot;
    fn preflight(&self, plan: &Plan<'_>) -> Result<(), BackendError>;
    fn begin_session(&mut self) -> Result<Self::SessionSnapshot, BackendError>;
    fn switch_pstate(&mut self, pstate: u8) -> Result<(), BackendError>;
    fn finish_session(&mut self, results: &[Option<ResultState>]) -> Result<(), BackendError>;
    fn rollback_session(&mut self, snapshot: Self::SessionSnapshot) -> Result<(), BackendError>;
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SessionFailure {
    pub cause: Cause,
    pub step: Option<usize>,
    pub stage_rollback_failed: bool,
    pub session_rollback_failed: bool,
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SessionResult {
    pub steps: usize,
    pub transitions: usize,
    pub probe_operations: u64,
    /// Point programmings of every step, confirmed live or declared
    /// unverifiable ("points verified live: N/M").
    pub live: Liveness,
}
fn failure(cause: Cause, step: Option<usize>) -> SessionFailure {
    SessionFailure {
        cause,
        step,
        stage_rollback_failed: false,
        session_rollback_failed: false,
    }
}
/// Validate exact requested coverage and dependency order. Global coverage
/// requires all nine stage labels in each of four states; this only proves
/// plan coverage, not algorithm implementation or actual physical topology.
/// Exact reviewed applicability for the correspondingly indexed plan target.
/// Required and excluded labels partition ALL_STAGES. Exclusion evidence is an
/// identity for review by backend preflight, not proof supplied by this core.
#[derive(Clone, Copy, Debug)]
pub struct Applicability<'a> {
    pub required: u16,
    pub excluded: u16,
    pub evidence: &'a str,
}
pub fn validate(plan: &Plan<'_>) -> Result<(), Cause> {
    validate_using(plan, None)
}
/// Production coverage requires an explicit contract for every listed target,
/// all four states, and evidence for every exclusion. No implicit exemption.
pub fn validate_reviewed(plan: &Plan<'_>, rules: &[Applicability<'_>]) -> Result<(), Cause> {
    validate_using(plan, Some(rules))
}
fn validate_using(plan: &Plan<'_>, rules: Option<&[Applicability<'_>]>) -> Result<(), Cause> {
    if rules.is_some_and(|r| r.len() != plan.targets.len()) {
        return Err(Cause::InvalidPlan);
    }
    if plan.targets.is_empty() || plan.steps.is_empty() || plan.operation_budget == 0 {
        return Err(Cause::InvalidPlan);
    }
    let mut coverage = [0u16; 4];
    for (i, target) in plan.targets.iter().copied().enumerate() {
        if target.pstate >= crate::search::MAX_PSTATES
            || target.required_mask == 0
            || target.stages == 0
            || target.stages & !ALL_STAGES != 0
        {
            return Err(Cause::InvalidPlan);
        }
        if let Some(rules) = rules {
            let rule = rules[i];
            if rule.required != target.stages
                || rule.required & rule.excluded != 0
                || rule.required | rule.excluded != ALL_STAGES
                || (rule.excluded != 0 && rule.evidence.trim().is_empty())
            {
                return Err(Cause::InvalidPlan);
            }
        }
        for prior in &plan.targets[..i] {
            if (prior.pstate, prior.channel, prior.rank, prior.lane)
                == (target.pstate, target.channel, target.rank, target.lane)
            {
                return Err(Cause::InvalidPlan);
            }
        }
        let mut seen = 0;
        for step in plan.steps {
            if target.location(step.request) {
                let bit = stage_bit(step.request.stage);
                if !target.matches(step.request)
                    || bit & target.stages == 0
                    || seen & bit != 0
                    || step.prerequisites & !seen != 0
                {
                    return Err(Cause::InvalidPlan);
                }
                seen |= bit;
            }
        }
        if seen != target.stages {
            return Err(Cause::InvalidPlan);
        }
        coverage[target.pstate as usize] |= seen | rules.map_or(0, |r| r[i].excluded);
    }
    for step in plan.steps {
        if !plan
            .targets
            .iter()
            .any(|target| target.matches(step.request))
        {
            return Err(Cause::InvalidPlan);
        }
    }
    if coverage != [ALL_STAGES; 4] {
        return Err(Cause::InvalidPlan);
    }
    Ok(())
}
/// Outputs are valid only on Ok. On failure every output is cleared, preventing
/// a successfully completed prefix from being mistaken for a complete plan.
pub fn run<B: SessionBackend>(
    backend: &mut B,
    plan: &Plan<'_>,
    outputs: &mut [Option<ResultState>],
) -> Result<SessionResult, SessionFailure> {
    run_using(backend, plan, outputs, |_| Ok(Decision::Generic))
}
/// Run every step through an explicit named phase policy. Missing policies are
/// Unsupported, mismatches are InvalidPlan, both checked before session entry.
/// No generic fallback can silently substitute for an unavailable phase.
pub fn run_phases<
    'a,
    B: SessionBackend,
    F: Fn(Request) -> Option<crate::search::phase::Policy<'a>>,
>(
    backend: &mut B,
    plan: &Plan<'_>,
    outputs: &mut [Option<ResultState>],
    policies: F,
) -> Result<SessionResult, SessionFailure> {
    run_using(backend, plan, outputs, |req| {
        policies(req).map(Decision::Phase).ok_or(Cause::Unsupported)
    })
}
enum Decision<'a> {
    Generic,
    Phase(crate::search::phase::Policy<'a>),
}
impl Decision<'_> {
    fn validate(&self, request: Request, domain: crate::search::Domain) -> Result<(), Cause> {
        match self {
            Self::Generic => Ok(()),
            Self::Phase(policy) if policy.stage() == request.stage => policy.validate(domain),
            _ => Err(Cause::InvalidPlan),
        }
    }
    fn select(
        &self,
        grid: &crate::search::Grid,
        domain: crate::search::Domain,
    ) -> Result<crate::search::Eye, Cause> {
        match self {
            Self::Generic => grid.select(domain),
            Self::Phase(policy) => policy.select(grid, domain),
        }
    }
}
fn run_using<'a, B: SessionBackend, F: Fn(Request) -> Result<Decision<'a>, Cause>>(
    backend: &mut B,
    plan: &Plan<'_>,
    outputs: &mut [Option<ResultState>],
    decisions: F,
) -> Result<SessionResult, SessionFailure> {
    outputs.fill(None);
    validate(plan).map_err(|e| failure(e, None))?;
    if outputs.len() != plan.steps.len() {
        return Err(failure(Cause::InvalidPlan, None));
    }
    backend
        .preflight(plan)
        .map_err(|e| failure(e.into(), None))?;
    let mut allowance = 0u64;
    for (i, step) in plan.steps.iter().enumerate() {
        let domain = backend
            .domain(step.request)
            .map_err(|e| failure(e.into(), Some(i)))?;
        if !domain.valid() {
            return Err(failure(Cause::InvalidPlan, Some(i)));
        }
        decisions(step.request)
            .and_then(|decision| decision.validate(step.request, domain))
            .map_err(|cause| failure(cause, Some(i)))?;
        allowance = allowance
            .checked_add(u64::from(domain.probe_operations))
            .ok_or_else(|| failure(Cause::Budget, Some(i)))?;
        if allowance > plan.operation_budget {
            return Err(failure(Cause::Budget, Some(i)));
        }
    }
    let snapshot = backend
        .begin_session()
        .map_err(|e| failure(e.into(), None))?;
    let mut current = None;
    let mut receipt = SessionResult {
        steps: 0,
        transitions: 0,
        probe_operations: 0,
        live: Liveness::default(),
    };
    let operation = (|| {
        for (i, step) in plan.steps.iter().enumerate() {
            if current != Some(step.request.pstate) {
                backend
                    .switch_pstate(step.request.pstate)
                    .map_err(|e| failure(e.into(), Some(i)))?;
                current = Some(step.request.pstate);
                receipt.transitions += 1;
            }
            // Revalidate the domain in the active state against the remaining
            // budget, then use exactly this domain for the stage.
            let domain = backend
                .domain(step.request)
                .map_err(|e| failure(e.into(), Some(i)))?;
            if u64::from(domain.probe_operations) > plan.operation_budget - receipt.probe_operations
            {
                return Err(failure(Cause::Budget, Some(i)));
            }
            let decision = decisions(step.request).map_err(|cause| failure(cause, Some(i)))?;
            decision
                .validate(step.request, domain)
                .map_err(|cause| failure(cause, Some(i)))?;
            let result = train_using(backend, step.request, domain, |grid, d| {
                decision.select(grid, d)
            })
            .map_err(
                |Failure {
                     cause,
                     rollback_failed,
                 }| {
                    let mut error = failure(cause, Some(i));
                    error.stage_rollback_failed = rollback_failed;
                    error
                },
            )?;
            receipt.probe_operations += u64::from(result.probe_operations);
            receipt.live.merge(result.live);
            receipt.steps += 1;
            outputs[i] = Some(result);
        }
        backend
            .finish_session(outputs)
            .map_err(|e| failure(e.into(), None))?;
        Ok(receipt)
    })();
    operation.map_err(|mut error: SessionFailure| {
        outputs.fill(None);
        error.session_rollback_failed = backend.rollback_session(snapshot).is_err();
        error
    })
}
