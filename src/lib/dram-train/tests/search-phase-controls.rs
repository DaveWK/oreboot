// SPDX-License-Identifier: GPL-2.0-only
use dram_train::search::{phase::*, *};
fn domain(height: u8, circular_x: bool) -> Domain {
    Domain {
        width: 8,
        height,
        circular_x,
        circular_y: false,
        min_width: 2,
        min_height: 1,
        scan_repeats: 2,
        verify_repeats: 2,
        polls_per_probe: 1,
        probe_operations: 1000,
    }
}
fn candidates() -> Grid {
    let mut grid = Grid::default();
    for (row, lo, hi) in [(0, 0, 4), (1, 2, 6), (2, 3, 4)] {
        for x in lo..=hi {
            grid.set(Point { x, y: row }, true).unwrap();
        }
    }
    grid
}
#[test]
fn coefficient_and_duty_candidates_are_independent_not_continuous_voltage_rows() {
    let grid = candidates();
    let d = domain(3, false);
    let joint = grid.select(d).unwrap();
    assert_eq!((joint.width, joint.height), (3, 2));
    for policy in [
        Policy::Dfe {
            candidate_costs: &[4, 1, 0],
        },
        Policy::Dca {
            candidate_costs: &[4, 1, 0],
        },
    ] {
        let got = policy.select(&grid, d).unwrap();
        assert_eq!((got.width, got.height), (5, 1));
        assert_eq!(got.center, Point { x: 4, y: 1 });
        assert!(!got.boundary_contact);
    }
    // Lower cost only breaks an equal timing-eye tie; a narrow zero-cost eye
    // must never win over a wider eye. No monotonic coefficient assumption.
    let got = Policy::Dfe {
        candidate_costs: &[0, 9, 0],
    }
    .select(&grid, d)
    .unwrap();
    assert_eq!(got.center, Point { x: 2, y: 0 });
    assert!(got.boundary_contact);
    assert_eq!(
        Policy::Dfe {
            candidate_costs: &[1, 2]
        }
        .select(&grid, d),
        Err(Cause::InvalidPlan)
    );
    let mut bad = d;
    bad.min_height = 2;
    assert_eq!(
        Policy::Dca {
            candidate_costs: &[1, 2, 3]
        }
        .select(&grid, bad),
        Err(Cause::InvalidPlan)
    );
}
#[test]
fn phase_axis_contracts_reject_unsupported_domains() {
    assert!(Policy::Wck2Ck.validate(domain(1, true)).is_ok());
    assert_eq!(
        Policy::Wck2Ck.validate(domain(1, false)),
        Err(Cause::InvalidPlan)
    );
    assert_eq!(
        Policy::ReadGate.validate(domain(1, true)),
        Err(Cause::InvalidPlan)
    );
    assert_eq!(
        Policy::ReadGate.validate(domain(2, false)),
        Err(Cause::InvalidPlan)
    );
    assert_eq!(
        Policy::ReadCenter { circular: false }.validate(domain(1, true)),
        Err(Cause::InvalidPlan)
    );
    let mut d = domain(4, false);
    d.circular_y = true;
    for policy in [Policy::Ca, Policy::ReadVref, Policy::WriteVref] {
        assert_eq!(policy.validate(d), Err(Cause::InvalidPlan));
    }
}
struct Sim {
    external: Grid,
    d: Domain,
    point: Point,
    ticket: u32,
    probe: Probe,
    begun: bool,
    recovered: bool,
    verify_fault: bool,
}
impl Backend for Sim {
    type Snapshot = Point;
    fn domain(&self, _: Request) -> Result<Domain, BackendError> {
        Ok(self.d)
    }
    fn begin(&mut self, _: Request) -> Result<Point, BackendError> {
        self.begun = true;
        Ok(self.point)
    }
    fn launch(&mut self, _: Request, p: Point, k: Probe) -> Result<u32, BackendError> {
        self.point = p;
        self.probe = k;
        self.ticket += 1;
        Ok(self.ticket)
    }
    fn poll(&mut self, ticket: u32) -> Result<Poll, BackendError> {
        let failed = !self.external.get(self.point)
            || (self.verify_fault && matches!(self.probe, Probe::Verify(_)));
        Ok(Poll::Ready(Measurement {
            ticket,
            observed_mask: 3,
            failed_mask: if failed { 3 } else { 0 },
        }))
    }
    fn apply(&mut self, _: Request, p: Point) -> Result<(), BackendError> {
        self.point = p;
        Ok(())
    }
    fn finish(&mut self, _: Request) -> Result<(), BackendError> {
        Ok(())
    }
    fn rollback(&mut self, _: Request, old: Point) -> Result<(), BackendError> {
        self.point = old;
        self.recovered = true;
        Ok(())
    }
}
#[test]
fn named_phase_policies_apply_and_verify_distinct_synthetic_settings_in_four_states() {
    for state in 0..4 {
        let costs = [if state % 2 == 0 { 0 } else { 4 }, 1, 0];
        let policies = [
            Policy::Ca,
            Policy::Wck2Ck,
            Policy::ReadGate,
            Policy::ReadCenter { circular: false },
            Policy::WriteCenter { circular: false },
            Policy::ReadVref,
            Policy::WriteVref,
            Policy::Dfe {
                candidate_costs: &costs,
            },
            Policy::Dca {
                candidate_costs: &costs,
            },
        ];
        for policy in policies {
            let stage = policy.stage();
            let mut grid = Grid::default();
            let (d, expected) = match stage {
                Stage::Ca | Stage::ReadVref | Stage::WriteVref => {
                    for y in 1..=2 {
                        for x in 2..=5 {
                            grid.set(Point { x, y }, true).unwrap();
                        }
                    }
                    (domain(4, false), Point { x: 3, y: 1 })
                }
                Stage::Dfe | Stage::Dca => {
                    grid = candidates();
                    (
                        domain(3, false),
                        if state % 2 == 0 {
                            Point { x: 2, y: 0 }
                        } else {
                            Point { x: 4, y: 1 }
                        },
                    )
                }
                Stage::Wck2Ck => {
                    for x in [6, 7, 0, 1] {
                        grid.set(Point { x, y: 0 }, true).unwrap();
                    }
                    (domain(1, true), Point { x: 7, y: 0 })
                }
                _ => {
                    for x in 2..=5 {
                        grid.set(Point { x, y: 0 }, true).unwrap();
                    }
                    (domain(1, false), Point { x: 3, y: 0 })
                }
            };
            let mut sim = Sim {
                external: grid,
                d,
                point: Point { x: 7, y: 7 },
                ticket: 0,
                probe: Probe::Scan(0),
                begun: false,
                recovered: false,
                verify_fault: false,
            };
            let req = Request {
                stage,
                pstate: state,
                channel: 0,
                rank: 0,
                lane: 0,
                required_mask: 3,
            };
            let result = train_phase(&mut sim, req, policy).unwrap();
            assert_eq!(result.eye.center, expected);
            assert_eq!(sim.point, expected);
            assert!(sim.begun && !sim.recovered);
            let prior = sim.point;
            sim.verify_fault = true;
            assert_eq!(
                train_phase(&mut sim, req, policy).unwrap_err().cause,
                Cause::Verification
            );
            assert_eq!(sim.point, prior);
            assert!(sim.recovered);
        }
    }
}
#[test]
fn candidate_selection_matches_independent_cell_by_cell_interval_oracle() {
    let mut random = 0xa733u32;
    for n in 0..320 {
        let mut grid = Grid::default();
        let d = domain(3, n % 2 == 0);
        let mut costs = [0; 3];
        for y in 0..3 {
            for x in 0..8 {
                random ^= random << 13;
                random ^= random >> 17;
                random ^= random << 5;
                grid.set(Point { x, y }, !random.is_multiple_of(3)).unwrap();
            }
            costs[y as usize] = (random % 13) as u16;
        }
        let mut expected: Option<Eye> = None;
        for y in 0..3 {
            for start in 0..8 {
                for len in 2..=8 {
                    if !d.circular_x && start + len > 8 {
                        continue;
                    }
                    if !(0..len).all(|i| {
                        grid.get(Point {
                            x: (start + i) % 8,
                            y,
                        })
                    }) {
                        continue;
                    }
                    let eye = Eye {
                        start: Point { x: start, y },
                        width: len,
                        height: 1,
                        center: Point {
                            x: (start + (len - 1) / 2) % 8,
                            y,
                        },
                        boundary_contact: !d.circular_x && (start == 0 || start + len == 8),
                    };
                    let better = expected.is_none_or(|old| {
                        len > old.width
                            || (len == old.width
                                && (costs[y as usize], eye.center.x, y, start)
                                    < (
                                        costs[old.center.y as usize],
                                        old.center.x,
                                        old.center.y,
                                        old.start.x,
                                    ))
                    });
                    if better {
                        expected = Some(eye);
                    }
                }
            }
        }
        assert_eq!(
            Policy::Dfe {
                candidate_costs: &costs
            }
            .select(&grid, d),
            expected.ok_or(Cause::NoWindow)
        );
    }
}
#[test]
fn unbound_phase_policies_remain_unsupported_without_access() {
    let mut backend = Unbound;
    for policy in [
        Policy::Ca,
        Policy::Wck2Ck,
        Policy::ReadGate,
        Policy::ReadCenter { circular: false },
        Policy::WriteCenter { circular: false },
        Policy::ReadVref,
        Policy::WriteVref,
        Policy::Dfe {
            candidate_costs: &[],
        },
        Policy::Dca {
            candidate_costs: &[],
        },
    ] {
        let r = Request {
            stage: policy.stage(),
            pstate: 0,
            channel: 0,
            rank: 0,
            lane: 0,
            required_mask: 1,
        };
        assert_eq!(
            train_phase(&mut backend, r, policy).unwrap_err().cause,
            Cause::Unsupported
        );
    }
}

#[test]
fn max64_candidates_keep_full_circle_cost_ties_and_seam_windows() {
    let mut grid = Grid::default();
    let mut costs = [9u16; 64];
    costs[53] = 0;
    for y in 0..64 {
        let start = y % 60;
        for x in start..start + 3 {
            grid.set(Point { x, y }, true).unwrap();
        }
    }
    for y in [12, 53] {
        for x in 0..64 {
            grid.set(Point { x, y }, true).unwrap();
        }
    }
    for circular in [false, true] {
        let d = Domain {
            width: 64,
            ..domain(64, circular)
        };
        for policy in [
            Policy::Dfe {
                candidate_costs: &costs,
            },
            Policy::Dca {
                candidate_costs: &costs,
            },
        ] {
            let got = policy.select(&grid, d).unwrap();
            assert_eq!(got.width, 64);
            assert_eq!(got.height, 1);
            assert_eq!(
                got.center,
                Point {
                    x: if circular { 0 } else { 31 },
                    y: 53
                }
            );
            assert_eq!(
                got.start,
                Point {
                    x: if circular { 33 } else { 0 },
                    y: 53
                }
            );
            assert_eq!(got.boundary_contact, !circular);
        }
    }
    for y in [12, 53] {
        for x in 0..64 {
            grid.set(Point { x, y }, false).unwrap();
        }
    }
    for x in [60, 61, 62, 63, 0, 1, 2, 3, 4, 5, 6] {
        grid.set(Point { x, y: 20 }, true).unwrap();
    }
    // row20's existing20..22 island stays disjoint from the seam window.
    let got = Policy::Dfe {
        candidate_costs: &costs,
    }
    .select(
        &grid,
        Domain {
            width: 64,
            ..domain(64, true)
        },
    )
    .unwrap();
    assert_eq!(
        (got.width, got.center, got.start),
        (11, Point { x: 1, y: 20 }, Point { x: 60, y: 20 })
    );
}
