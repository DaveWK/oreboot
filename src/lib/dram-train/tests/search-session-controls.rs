// SPDX-License-Identifier: GPL-2.0-only
use dram_train::search::{session::*, *};
fn index(r: Request) -> usize {
    r.pstate as usize * 9 + Stage::ALL.iter().position(|s| *s == r.stage).unwrap()
}
fn setup() -> (Vec<Target>, Vec<Step>) {
    let mut targets = Vec::new();
    let mut steps = Vec::new();
    // This order is a synthetic test plan, never a proposed hardware sequence.
    for pstate in 0..4 {
        targets.push(Target {
            pstate,
            channel: 0,
            rank: 0,
            lane: 0,
            required_mask: 3,
            stages: ALL_STAGES,
        });
        let mut prior = 0;
        for stage in Stage::ALL {
            steps.push(Step {
                request: Request {
                    stage,
                    pstate,
                    channel: 0,
                    rank: 0,
                    lane: 0,
                    required_mask: 3,
                },
                prerequisites: prior,
            });
            prior |= stage_bit(stage);
        }
    }
    (targets, steps)
}
struct Sim {
    settings: [Option<Point>; 36],
    current: u8,
    request: Option<Request>,
    point: Point,
    ticket: u32,
    probe: Probe,
    phase_domains: bool,
    begun: bool,
    recovered: bool,
    transitions: Vec<u8>,
    verify_fault: Option<usize>,
    transition_fault: Option<u8>,
    finish_fault: bool,
    recovery_fault: bool,
    preflight_fault: bool,
    local_recovery_fault: bool,
}
impl Default for Sim {
    fn default() -> Self {
        Self {
            settings: [Some(Point { x: 3, y: 3 }); 36],
            current: 2,
            request: None,
            point: Point { x: 0, y: 0 },
            ticket: 0,
            probe: Probe::Scan(0),
            phase_domains: false,
            begun: false,
            recovered: false,
            transitions: Vec::new(),
            verify_fault: None,
            transition_fault: None,
            finish_fault: false,
            recovery_fault: false,
            preflight_fault: false,
            local_recovery_fault: false,
        }
    }
}
impl Backend for Sim {
    type Snapshot = Option<Point>;
    fn domain(&self, r: Request) -> Result<Domain, BackendError> {
        let one_d = self.phase_domains
            && matches!(
                r.stage,
                Stage::Wck2Ck | Stage::ReadGate | Stage::ReadCenter | Stage::WriteCenter
            );
        Ok(Domain {
            width: 4,
            height: if one_d { 1 } else { 4 },
            circular_x: self.phase_domains && r.stage == Stage::Wck2Ck,
            circular_y: false,
            min_width: 1,
            min_height: 1,
            scan_repeats: 2,
            verify_repeats: 2,
            polls_per_probe: 1,
            probe_operations: 100,
        })
    }
    fn begin(&mut self, r: Request) -> Result<Self::Snapshot, BackendError> {
        assert_eq!(self.current, r.pstate);
        self.request = Some(r);
        Ok(self.settings[index(r)])
    }
    fn launch(&mut self, r: Request, p: Point, probe: Probe) -> Result<u32, BackendError> {
        assert_eq!(self.request, Some(r));
        self.ticket += 1;
        self.point = p;
        self.probe = probe;
        Ok(self.ticket)
    }
    fn poll(&mut self, ticket: u32) -> Result<Poll, BackendError> {
        let i = index(self.request.unwrap());
        let expected = Point {
            x: (i % 4) as u8,
            y: if self.phase_domains
                && matches!(
                    self.request.unwrap().stage,
                    Stage::Wck2Ck | Stage::ReadGate | Stage::ReadCenter | Stage::WriteCenter
                )
            {
                0
            } else {
                ((i / 4) % 4) as u8
            },
        };
        let fail = self.point != expected
            || (self.verify_fault == Some(i) && matches!(self.probe, Probe::Verify(_)));
        Ok(Poll::Ready(Measurement {
            ticket,
            observed_mask: 3,
            failed_mask: if fail { 3 } else { 0 },
        }))
    }
    fn apply(&mut self, r: Request, p: Point) -> Result<(), BackendError> {
        self.settings[index(r)] = Some(p);
        Ok(())
    }
    fn finish(&mut self, _: Request) -> Result<(), BackendError> {
        Ok(())
    }
    fn rollback(&mut self, r: Request, old: Self::Snapshot) -> Result<(), BackendError> {
        self.settings[index(r)] = old;
        if self.local_recovery_fault {
            Err(BackendError::Fault)
        } else {
            Ok(())
        }
    }
}
impl SessionBackend for Sim {
    type SessionSnapshot = ([Option<Point>; 36], u8);
    fn preflight(&self, _: &Plan<'_>) -> Result<(), BackendError> {
        if self.preflight_fault {
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
    fn finish_session(&mut self, r: &[Option<ResultState>]) -> Result<(), BackendError> {
        assert!(r.iter().all(Option::is_some));
        if self.finish_fault {
            Err(BackendError::Fault)
        } else {
            Ok(())
        }
    }
    fn rollback_session(&mut self, s: Self::SessionSnapshot) -> Result<(), BackendError> {
        self.settings = s.0;
        self.current = s.1;
        self.recovered = true;
        if self.recovery_fault {
            Err(BackendError::Fault)
        } else {
            Ok(())
        }
    }
}
#[test]
fn full_plan_retains_independently_varied_settings_and_exact_state_transitions() {
    let (targets, steps) = setup();
    let plan = Plan {
        targets: &targets,
        steps: &steps,
        operation_budget: 3600,
    };
    let mut sim = Sim::default();
    let mut outputs = [None; 36];
    let got = run(&mut sim, &plan, &mut outputs).unwrap();
    assert_eq!(
        (got.steps, got.transitions, got.probe_operations),
        (36, 4, 36 * 68)
    );
    // Sim has no live readback: every programming is declared unverifiable.
    assert!(got.live.applied > 0 && got.live.unverifiable == got.live.applied);
    assert_eq!(sim.transitions, [0, 1, 2, 3]);
    assert!(!sim.recovered);
    for (i, r) in outputs.into_iter().enumerate() {
        let r = r.unwrap();
        assert_eq!(r.request, steps[i].request);
        let point = Point {
            x: (i % 4) as u8,
            y: ((i / 4) % 4) as u8,
        };
        assert_eq!(r.eye.center, point);
        assert_eq!(sim.settings[i], Some(point));
    }
}
#[test]
fn later_stage_transition_and_commit_failures_restore_whole_prior_configuration() {
    let (targets, steps) = setup();
    let plan = Plan {
        targets: &targets,
        steps: &steps,
        operation_budget: 3600,
    };
    for case in 0..5 {
        let mut sim = Sim::default();
        match case {
            0 => sim.verify_fault = Some(20),
            1 => sim.transition_fault = Some(3),
            2 => sim.finish_fault = true,
            3 => {
                sim.verify_fault = Some(20);
                sim.local_recovery_fault = true;
            }
            4 => {
                sim.verify_fault = Some(20);
                sim.recovery_fault = true;
            }
            _ => unreachable!(),
        }
        let old = (sim.settings, sim.current);
        let mut outputs = [None; 36];
        let error = run(&mut sim, &plan, &mut outputs).unwrap_err();
        assert_eq!(
            error.cause,
            if [0, 3, 4].contains(&case) {
                Cause::Verification
            } else {
                Cause::BackendFault
            }
        );
        assert_eq!(error.stage_rollback_failed, case == 3);
        assert_eq!(error.session_rollback_failed, case == 4);
        assert_eq!((sim.settings, sim.current), old);
        assert!(sim.recovered && outputs.iter().all(Option::is_none));
    }
}
#[test]
fn invalid_coverage_dependencies_or_budget_and_unsupported_preflight_do_no_mutation() {
    for case in 0..9 {
        let (mut targets, mut steps) = setup();
        let mut sim = Sim::default();
        let mut budget = 3600;
        match case {
            0 => {
                steps.pop();
            } // missing DCA in state3
            1 => {
                steps.push(steps[0]);
            } // duplicate target/stage
            2 => {
                steps[0].prerequisites = stage_bit(Stage::Dca);
            }
            3 => {
                steps[0].request.lane = 1;
            } // unlisted target
            4 => {
                steps[0].request.required_mask = 1;
            }
            5 => {
                targets[0].stages = 1 << 15;
            }
            6 => {
                targets[0].pstate = 4;
            }
            7 => {
                budget -= 1;
            }
            8 => {
                sim.preflight_fault = true;
            }
            _ => unreachable!(),
        }
        let plan = Plan {
            targets: &targets,
            steps: &steps,
            operation_budget: budget,
        };
        let mut outputs = vec![None; steps.len()];
        let e = run(&mut sim, &plan, &mut outputs).unwrap_err();
        assert_eq!(
            e.cause,
            match case {
                7 => Cause::Budget,
                8 => Cause::Unsupported,
                _ => Cause::InvalidPlan,
            }
        );
        assert!(!sim.begun && sim.transitions.is_empty() && !sim.recovered);
        assert!(outputs.iter().all(Option::is_none));
    }
}

#[test]
fn full_session_executes_named_phase_policies_and_rejects_missing_policy_before_entry() {
    use dram_train::search::phase::Policy;
    let (targets, steps) = setup();
    let plan = Plan {
        targets: &targets,
        steps: &steps,
        operation_budget: 3600,
    };
    let costs = [3, 2, 1, 0];
    let policies = |request: Request| {
        Some(match request.stage {
            Stage::Ca => Policy::Ca,
            Stage::Wck2Ck => Policy::Wck2Ck,
            Stage::ReadGate => Policy::ReadGate,
            Stage::ReadCenter => Policy::ReadCenter { circular: false },
            Stage::WriteCenter => Policy::WriteCenter { circular: false },
            Stage::ReadVref => Policy::ReadVref,
            Stage::WriteVref => Policy::WriteVref,
            Stage::Dfe => Policy::Dfe {
                candidate_costs: &costs,
            },
            Stage::Dca => Policy::Dca {
                candidate_costs: &costs,
            },
        })
    };
    let mut sim = Sim {
        phase_domains: true,
        ..Sim::default()
    };
    let mut outputs = [None; 36];
    let receipt = run_phases(&mut sim, &plan, &mut outputs, policies).unwrap();
    assert_eq!(receipt.steps, 36);
    assert_eq!(sim.transitions, [0, 1, 2, 3]);
    for (i, result) in outputs.iter().enumerate() {
        assert_eq!(result.unwrap().eye.center, sim.settings[i].unwrap());
    }
    for mismatch in [false, true] {
        let mut sim = Sim {
            phase_domains: true,
            ..Sim::default()
        };
        let error = run_phases(&mut sim, &plan, &mut outputs, |r| {
            if r.stage == Stage::Dca {
                if mismatch {
                    Some(Policy::Ca)
                } else {
                    None
                }
            } else {
                policies(r)
            }
        })
        .unwrap_err();
        assert_eq!(
            error.cause,
            if mismatch {
                Cause::InvalidPlan
            } else {
                Cause::Unsupported
            }
        );
        assert!(!sim.begun && sim.transitions.is_empty());
        assert!(outputs.iter().all(Option::is_none));
    }
}
