// SPDX-License-Identifier: GPL-2.0-only
use dram_train::search::{
    full_session::{self as full, LinePolicy, Trained},
    line::{LineBackend, LineDomain},
    phase,
    session::{self, Applicability, Plan, Step, Target},
    *,
};
fn index(r: Request) -> usize {
    r.pstate as usize * 9 + Stage::ALL.iter().position(|s| *s == r.stage).unwrap()
}
static COSTS: [u16; 4] = [3, 2, 1, 0];
fn policy(stage: Stage) -> full::Policy<'static> {
    use full::Policy::{Grid, Line};
    match stage {
        Stage::Ca => Grid(phase::Policy::Ca),
        Stage::Wck2Ck => Line(LinePolicy::Wck2Ck),
        Stage::ReadGate => Line(LinePolicy::ReadGate),
        Stage::ReadCenter => Line(LinePolicy::ReadCenter { circular: false }),
        Stage::WriteCenter => Line(LinePolicy::WriteCenter { circular: true }),
        Stage::ReadVref => Grid(phase::Policy::ReadVref),
        Stage::WriteVref => Grid(phase::Policy::WriteVref),
        Stage::Dfe => Grid(phase::Policy::Dfe {
            candidate_costs: &COSTS,
        }),
        Stage::Dca => Grid(phase::Policy::Dca {
            candidate_costs: &COSTS,
        }),
    }
}
fn setup(
    exclude: bool,
) -> (
    Vec<Target>,
    Vec<Step>,
    Vec<Applicability<'static>>,
    Vec<full::Policy<'static>>,
) {
    let mut targets = Vec::new();
    let mut steps = Vec::new();
    let mut rules = Vec::new();
    let mut policies = Vec::new();
    for state in 0..4 {
        let excluded = if exclude && state == 2 {
            session::stage_bit(Stage::Dfe) | session::stage_bit(Stage::Dca)
        } else {
            0
        };
        let required = session::ALL_STAGES ^ excluded;
        targets.push(Target {
            pstate: state,
            channel: 0,
            rank: 0,
            lane: 0,
            required_mask: 3,
            stages: required,
        });
        rules.push(Applicability {
            required,
            excluded,
            evidence: "test-review:synthetic-part-state-contract",
        });
        let mut prior = 0;
        for stage in Stage::ALL {
            let bit = session::stage_bit(stage);
            if required & bit == 0 {
                continue;
            }
            steps.push(Step {
                request: Request {
                    stage,
                    pstate: state,
                    channel: 0,
                    rank: 0,
                    lane: 0,
                    required_mask: 3,
                },
                prerequisites: prior,
            });
            policies.push(policy(stage));
            prior |= bit;
        }
    }
    (targets, steps, rules, policies)
}
fn is_line(r: Request) -> bool {
    matches!(
        r.stage,
        Stage::Wck2Ck | Stage::ReadGate | Stage::ReadCenter | Stage::WriteCenter
    )
}
fn points(r: Request) -> u16 {
    [256, 212, 128, 256][r.pstate as usize]
}
fn full_circle(r: Request) -> bool {
    r.stage == Stage::Wck2Ck && r.pstate == 0
}
fn line_start(r: Request) -> u16 {
    70 + (index(r) * 3 % 30) as u16
}
fn expected(r: Request) -> Point {
    if is_line(r) {
        Point {
            x: if full_circle(r) {
                0
            } else {
                (line_start(r) + 2) as u8
            },
            y: 0,
        }
    } else {
        Point {
            x: (index(r) % 8) as u8,
            y: ((index(r) / 8) % 4) as u8,
        }
    }
}
struct Sim {
    matrix_allpass: bool,
    matrix_mode: bool,
    settings: [Point; 36],
    current: u8,
    request: Option<Request>,
    point: Point,
    probe: Probe,
    ticket: u32,
    begun: bool,
    recovered: bool,
    transitions: Vec<u8>,
    verify_fault: Option<usize>,
    transition_fault: Option<u8>,
    finish_fault: bool,
    rollback_fault: bool,
    local_rollback_fault: bool,
    unsupported: Option<usize>,
    drift: bool,
}
impl Default for Sim {
    fn default() -> Self {
        Self {
            matrix_allpass: false,
            matrix_mode: false,
            settings: [Point { x: 250, y: 250 }; 36],
            current: 2,
            request: None,
            point: Point { x: 0, y: 0 },
            probe: Probe::Scan(0),
            ticket: 0,
            begun: false,
            recovered: false,
            transitions: Vec::new(),
            verify_fault: None,
            transition_fault: None,
            finish_fault: false,
            rollback_fault: false,
            local_rollback_fault: false,
            unsupported: None,
            drift: false,
        }
    }
}
fn joint(r: Request) -> bool {
    matches!(r.stage, Stage::Ca | Stage::ReadVref | Stage::WriteVref)
}
impl Sim {
    fn expected(&self, r: Request) -> Point {
        if self.matrix_mode && joint(r) && self.matrix_allpass {
            Point { x: 0, y: 63 }
        } else if self.matrix_mode && joint(r) {
            Point {
                x: 180 + (index(r) % 32) as u8,
                y: 70 + (index(r) % 24) as u8,
            }
        } else {
            expected(r)
        }
    }
}
impl Backend for Sim {
    type Snapshot = Point;
    fn domain(&self, r: Request) -> Result<Domain, BackendError> {
        assert!(
            !is_line(r) && !(self.matrix_mode && joint(r)),
            "wide domain silently narrowed to grid"
        );
        if self.unsupported == Some(index(r)) {
            return Err(BackendError::Unsupported);
        }
        Ok(Domain {
            width: 8,
            height: 4,
            min_width: 1,
            min_height: 1,
            circular_x: false,
            circular_y: false,
            scan_repeats: 2,
            verify_repeats: 2,
            polls_per_probe: 1,
            probe_operations: 2000,
        })
    }
    fn begin(&mut self, r: Request) -> Result<Point, BackendError> {
        assert_eq!(self.current, r.pstate);
        self.request = Some(r);
        Ok(self.settings[index(r)])
    }
    fn launch(&mut self, r: Request, p: Point, probe: Probe) -> Result<u32, BackendError> {
        assert_eq!(self.request, Some(r));
        self.point = p;
        self.probe = probe;
        self.ticket += 1;
        Ok(self.ticket)
    }
    fn poll(&mut self, ticket: u32) -> Result<Poll, BackendError> {
        let r = self.request.unwrap();
        let x = u16::from(self.point.x);
        let failed = if full_circle(r) || (self.matrix_mode && joint(r) && self.matrix_allpass) {
            0
        } else if is_line(r) {
            // Independent bit windows [start,start+4] and [start+1,start+5].
            let start = line_start(r);
            u64::from(!(start..=start + 4).contains(&x))
                | (u64::from(!(start + 1..=start + 5).contains(&x)) << 1)
        } else if self.point == self.expected(r) {
            0
        } else {
            3
        };
        let verify_fail =
            self.verify_fault == Some(index(r)) && matches!(self.probe, Probe::Verify(_));
        Ok(Poll::Ready(Measurement {
            ticket,
            observed_mask: 3,
            failed_mask: if verify_fail { 3 } else { failed },
        }))
    }
    fn apply(&mut self, r: Request, p: Point) -> Result<(), BackendError> {
        self.settings[index(r)] = p;
        Ok(())
    }
    fn finish(&mut self, _: Request) -> Result<(), BackendError> {
        Ok(())
    }
    fn rollback(&mut self, r: Request, p: Point) -> Result<(), BackendError> {
        self.settings[index(r)] = p;
        if self.local_rollback_fault {
            Err(BackendError::Fault)
        } else {
            Ok(())
        }
    }
}
impl LineBackend for Sim {
    fn line_domain(&self, r: Request) -> Result<LineDomain, BackendError> {
        assert!(is_line(r));
        if self.unsupported == Some(index(r)) {
            return Err(BackendError::Unsupported);
        }
        let mut circular = matches!(r.stage, Stage::Wck2Ck | Stage::WriteCenter);
        if self.drift && self.begun && index(r) == 12 {
            circular = !circular;
        }
        Ok(LineDomain {
            points: points(r),
            circular,
            min_span: 3,
            scan_repeats: 2,
            verify_repeats: 2,
            polls_per_probe: 1,
            probe_operations: 2000,
        })
    }
}
impl dram_train::search::matrix::MatrixBackend for Sim {
    fn matrix_domain(
        &self,
        r: Request,
    ) -> Result<dram_train::search::matrix::MatrixDomain, BackendError> {
        if !self.matrix_mode || !joint(r) {
            return Err(BackendError::Unsupported);
        }
        Ok(dram_train::search::matrix::MatrixDomain {
            width: 256,
            height: 128,
            circular_timing: true,
            min_width: 1,
            min_height: 1,
            scan_repeats: 2,
            verify_repeats: 2,
            polls_per_probe: 1,
            probe_operations: 300000,
        })
    }
}
impl full::Backend for Sim {
    type SessionSnapshot = ([Point; 36], u8);
    fn preflight(
        &self,
        _: &Plan<'_>,
        rules: &[Applicability<'_>],
        _: &[full::Policy<'_>],
    ) -> Result<(), BackendError> {
        // Only this explicit synthetic contract is supported by this mock.
        if rules
            .iter()
            .any(|r| !r.evidence.starts_with("test-review:"))
        {
            Err(BackendError::Unsupported)
        } else {
            Ok(())
        }
    }
    fn begin_session(&mut self) -> Result<Self::SessionSnapshot, BackendError> {
        self.begun = true;
        Ok((self.settings, self.current))
    }
    fn switch_pstate(&mut self, p: u8) -> Result<(), BackendError> {
        self.current = p;
        self.transitions.push(p);
        if self.transition_fault == Some(p) {
            Err(BackendError::Fault)
        } else {
            Ok(())
        }
    }
    fn finish_session(&mut self, outputs: &[Option<Trained>]) -> Result<(), BackendError> {
        for result in outputs {
            let r = result.unwrap();
            assert_eq!(
                self.settings[index(r.request())],
                self.expected(r.request())
            );
        }
        assert!(outputs
            .iter()
            .any(|r| matches!(r, Some(Trained::Line(l)) if l.eye.span == 256)));
        if self.finish_fault {
            Err(BackendError::Fault)
        } else {
            Ok(())
        }
    }
    fn rollback_session(&mut self, old: Self::SessionSnapshot) -> Result<(), BackendError> {
        self.settings = old.0;
        self.current = old.1;
        self.recovered = true;
        if self.rollback_fault {
            Err(BackendError::Fault)
        } else {
            Ok(())
        }
    }
}
#[test]
fn mixed_full_plan_preserves_long_results_and_all_four_state_settings() {
    let (targets, steps, rules, policies) = setup(false);
    let plan = Plan {
        targets: &targets,
        steps: &steps,
        operation_budget: 72000,
    };
    let mut sim = Sim::default();
    let mut out = vec![None; steps.len()];
    let receipt = full::run(&mut sim, &plan, &rules, &policies, &mut out).unwrap();
    assert_eq!(receipt.steps, 36);
    assert_eq!(receipt.transitions, 4);
    assert_eq!(sim.transitions, [0, 1, 2, 3]);
    assert_eq!(
        receipt.probe_operations,
        out.iter()
            .map(|r| u64::from(r.unwrap().probe_operations()))
            .sum()
    );
    for (i, result) in out.iter().enumerate() {
        let r = result.unwrap();
        assert_eq!(r.request(), steps[i].request);
        assert_eq!(sim.settings[index(r.request())], expected(r.request()));
        match r {
            Trained::Matrix(_) => panic!("unrequested matrix"),
            Trained::Line(l) => {
                assert!(is_line(l.request));
                assert_eq!(l.eye.span, if full_circle(l.request) { 256 } else { 4 });
                assert_eq!(l.eye.center, expected(l.request).x);
            }
            Trained::Grid(g) => {
                assert!(!is_line(g.request));
                assert_eq!(g.eye.center, expected(g.request));
            }
        }
    }
}
#[test]
fn explicit_reviewed_exclusions_skip_only_inapplicable_target_phases() {
    let (targets, steps, rules, policies) = setup(true);
    let plan = Plan {
        targets: &targets,
        steps: &steps,
        operation_budget: 68000,
    };
    assert_eq!(session::validate(&plan), Err(Cause::InvalidPlan));
    assert_eq!(session::validate_reviewed(&plan, &rules), Ok(()));
    let mut sim = Sim::default();
    let old = sim.settings;
    let mut out = vec![None; steps.len()];
    let receipt = full::run(&mut sim, &plan, &rules, &policies, &mut out).unwrap();
    assert_eq!(receipt.steps, 34);
    assert_eq!(sim.settings[25], old[25]);
    assert_eq!(sim.settings[26], old[26]);
    for (i, p) in sim.settings.iter().enumerate() {
        if ![25, 26].contains(&i) {
            assert_ne!(*p, old[i]);
        }
    }
}
#[test]
fn mixed_later_phase_transition_commit_and_domain_failures_restore_entire_plan() {
    let (targets, steps, rules, policies) = setup(false);
    let plan = Plan {
        targets: &targets,
        steps: &steps,
        operation_budget: 72000,
    };
    // Each of36 phases fails fresh verification after selected settings applied.
    for case in 0..41 {
        let mut sim = Sim::default();
        let mut expected_cause = Cause::Verification;
        match case {
            0..=35 => sim.verify_fault = Some(case),
            36 => {
                sim.transition_fault = Some(3);
                expected_cause = Cause::BackendFault;
            }
            37 => {
                sim.finish_fault = true;
                expected_cause = Cause::BackendFault;
            }
            38 => {
                sim.verify_fault = Some(20);
                sim.local_rollback_fault = true;
            }
            39 => {
                sim.verify_fault = Some(20);
                sim.rollback_fault = true;
            }
            40 => {
                sim.drift = true;
                expected_cause = Cause::InvalidPlan;
            }
            _ => unreachable!(),
        }
        let old = (sim.settings, sim.current);
        let mut out = vec![None; steps.len()];
        let error = full::run(&mut sim, &plan, &rules, &policies, &mut out).unwrap_err();
        assert_eq!(error.cause, expected_cause, "case{case}");
        assert_eq!(error.stage_rollback_failed, case == 38);
        assert_eq!(error.session_rollback_failed, case == 39);
        assert_eq!((sim.settings, sim.current), old);
        assert!(sim.recovered && out.iter().all(Option::is_none));
    }
}
#[test]
fn missing_required_unsupported_policy_and_unreviewed_exclusion_fail_before_entry() {
    for case in 0..10 {
        let (mut targets, mut steps, mut rules, mut policies) = setup(true);
        let mut budget = 68000;
        let mut sim = Sim::default();
        let mut cause = Cause::InvalidPlan;
        match case {
            0 => {
                rules[2].evidence = "";
            }
            1 => {
                rules[2].evidence = "unreviewed:test";
                cause = Cause::Unsupported;
            }
            2 => {
                rules[2].excluded = 0;
            }
            3 => {
                rules[2].required = session::ALL_STAGES;
            }
            4 => {
                targets[2].stages |= session::stage_bit(Stage::Dfe);
            }
            5 => {
                steps.remove(1);
                policies.remove(1);
            }
            6 => {
                policies[1] = policy(Stage::Ca);
            }
            7 => {
                policies.pop();
            }
            8 => {
                sim.unsupported = Some(31);
                cause = Cause::Unsupported;
            }
            9 => {
                budget = 1;
                cause = Cause::Budget;
            }
            _ => unreachable!(),
        }
        let plan = Plan {
            targets: &targets,
            steps: &steps,
            operation_budget: budget,
        };
        let mut out = vec![None; steps.len()];
        let error = full::run(&mut sim, &plan, &rules, &policies, &mut out).unwrap_err();
        assert_eq!(error.cause, cause, "case{case}");
        assert!(!sim.begun && !sim.recovered);
        assert!(out.iter().all(Option::is_none));
    }
}

#[test]
fn wide_joint_results_travel_through_complete_mixed_session_without_narrowing() {
    let (targets, steps, rules, mut policies) = setup(false);
    for (i, step) in steps.iter().enumerate() {
        let p = match step.request.stage {
            Stage::Ca => Some(full::MatrixPolicy::Ca),
            Stage::ReadVref => Some(full::MatrixPolicy::ReadVref),
            Stage::WriteVref => Some(full::MatrixPolicy::WriteVref),
            _ => None,
        };
        if let Some(p) = p {
            policies[i] = full::Policy::Matrix(p);
        }
    }
    let plan = Plan {
        targets: &targets,
        steps: &steps,
        operation_budget: 4000000,
    };
    for (fail, allpass) in [(false, false), (true, false), (false, true)] {
        let mut sim = Sim {
            matrix_allpass: allpass,
            matrix_mode: true,
            ..Sim::default()
        };
        if fail {
            sim.verify_fault = Some(24);
        }
        let old = (sim.settings, sim.current);
        let mut out = vec![None; steps.len()];
        let result = full::run(&mut sim, &plan, &rules, &policies, &mut out);
        if fail {
            assert_eq!(result.unwrap_err().cause, Cause::Verification);
            assert!(sim.recovered && out.iter().all(Option::is_none));
            assert_eq!((sim.settings, sim.current), old);
        } else {
            assert_eq!(result.unwrap().steps, 36);
            assert_eq!(
                out.iter()
                    .filter(|r| matches!(r, Some(Trained::Matrix(_))))
                    .count(),
                12
            );
            for r in out.into_iter().flatten() {
                if let Trained::Matrix(m) = r {
                    assert_eq!(m.eye.center, sim.expected(m.request));
                    if allpass {
                        assert_eq!((m.eye.width, m.eye.height), (256, 128));
                    } else {
                        assert!(m.eye.center.x >= 180 && m.eye.center.y >= 70);
                    }
                }
            }
        }
    }
}
