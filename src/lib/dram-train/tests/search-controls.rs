use dram_train::search::*;
fn domain() -> Domain {
    Domain {
        width: 8,
        height: 6,
        circular_x: false,
        circular_y: false,
        min_width: 2,
        min_height: 2,
        scan_repeats: 3,
        verify_repeats: 2,
        polls_per_probe: 4,
        probe_operations: 10000,
    }
}
fn req() -> Request {
    Request {
        stage: Stage::ReadCenter,
        pstate: 1,
        channel: 0,
        rank: 0,
        lane: 0,
        required_mask: 3,
    }
}
#[derive(Default)]
struct Fake {
    point: Option<Point>,
    old: Option<Point>,
    ticket: u32,
    probe: Option<Probe>,
    scans: u32,
    verifies: u32,
    applied: bool,
    finished: bool,
    restored: bool,
    fault: u8,
    pending: u32,
    polls: u32,
    d: Option<Domain>,
    skew: u8,
}
impl Backend for Fake {
    type Snapshot = Option<Point>;
    fn domain(&self, _: Request) -> Result<Domain, BackendError> {
        Ok(self.d.unwrap_or_else(domain))
    }
    fn begin(&mut self, _: Request) -> Result<Self::Snapshot, BackendError> {
        self.old = self.point;
        Ok(self.point)
    }
    fn launch(&mut self, _: Request, p: Point, k: Probe) -> Result<u32, BackendError> {
        if self.fault == 8 {
            return Err(BackendError::Fault);
        }
        self.ticket += 1;
        self.point = Some(p);
        self.probe = Some(k);
        self.polls = 0;
        match k {
            Probe::Scan(_) => self.scans += 1,
            Probe::Verify(_) => self.verifies += 1,
        };
        Ok(self.ticket)
    }
    fn poll(&mut self, t: u32) -> Result<Poll, BackendError> {
        self.polls += 1;
        if self.fault == 1 || self.polls <= self.pending {
            return Ok(Poll::Pending);
        }
        let p = self.point.unwrap();
        // Two independently specified bit windows. Their common rectangle is
        // x=2+skew..5, y=2..4; never consult the search algorithm's chosen eye.
        let bit0 = (1 + self.skew..=5).contains(&p.x) && (1..=4).contains(&p.y);
        let bit1 = (2 + self.skew..=6).contains(&p.x) && (2..=5).contains(&p.y);
        let mut failed = u64::from(!bit0) | (u64::from(!bit1) << 1);
        if self.fault == 4 && matches!(self.probe, Some(Probe::Verify(_))) {
            failed = 3
        }
        if self.fault == 5 && matches!(self.probe, Some(Probe::Scan(1))) {
            failed = 3
        }
        Ok(Poll::Ready(Measurement {
            ticket: if self.fault == 2 { t - 1 } else { t },
            observed_mask: if self.fault == 3 { 1 } else { 3 },
            failed_mask: failed,
        }))
    }
    fn apply(&mut self, _: Request, p: Point) -> Result<(), BackendError> {
        self.point = Some(p);
        self.applied = true;
        if self.fault == 6 {
            Err(BackendError::Fault)
        } else {
            Ok(())
        }
    }
    fn finish(&mut self, _: Request) -> Result<(), BackendError> {
        self.finished = true;
        if self.fault == 9 {
            Err(BackendError::Fault)
        } else {
            Ok(())
        }
    }
    fn rollback(&mut self, _: Request, s: Self::Snapshot) -> Result<(), BackendError> {
        self.point = s;
        self.restored = true;
        if self.fault == 7 {
            Err(BackendError::Fault)
        } else {
            Ok(())
        }
    }
}
#[test]
fn all_four_states_and_stage_requests_have_independently_varied_outputs() {
    let mut cases = 0;
    for state in 0..4 {
        for (stage_id, stage) in Stage::ALL.into_iter().enumerate() {
            for lane in 0..2 {
                let skew = (state + lane + (stage_id as u8)) % 2;
                let mut f = Fake {
                    skew,
                    pending: u32::from(state % 2),
                    ..Fake::default()
                };
                let r = Request {
                    stage,
                    pstate: state,
                    lane,
                    ..req()
                };
                let got = train(&mut f, r).unwrap();
                assert_eq!(got.request, r);
                assert_eq!(got.eye.start, Point { x: 2 + skew, y: 2 });
                assert_eq!((got.eye.width, got.eye.height), (4 - skew, 3));
                assert_eq!(got.eye.center, Point { x: 3 + skew, y: 3 });
                assert_eq!(f.point, Some(got.eye.center));
                assert!(f.applied && f.finished && !f.restored);
                assert_eq!(f.scans, 8 * 6 * 3);
                assert_eq!(f.verifies, 2);
                cases += 1;
            }
        }
    }
    assert_eq!(cases, 72);
}
#[test]
fn independently_injected_failures_restore_the_old_settings() {
    for (fault, cause) in [
        (1, Cause::Timeout),
        (2, Cause::StaleObservation),
        (3, Cause::MissingObservation),
        (4, Cause::Verification),
        (5, Cause::NoWindow),
        (6, Cause::BackendFault),
        (8, Cause::BackendFault),
        (9, Cause::BackendFault),
    ] {
        let prior = Some(Point { x: 7, y: 0 });
        let mut f = Fake {
            point: prior,
            fault,
            ..Fake::default()
        };
        assert_eq!(
            train(&mut f, req()),
            Err(Failure {
                cause,
                rollback_failed: false
            })
        );
        assert!(f.restored);
        assert_eq!(f.point, prior);
    }
}
#[test]
fn exhausted_budget_and_failed_rollback_are_not_passes() {
    let mut d = domain();
    d.probe_operations = 1;
    for fault in [0, 7] {
        let mut f = Fake {
            fault,
            d: Some(d),
            ..Fake::default()
        };
        assert_eq!(
            train(&mut f, req()),
            Err(Failure {
                cause: Cause::Budget,
                rollback_failed: fault == 7
            })
        );
        assert!(f.restored);
    }
}
#[test]
fn exact_poll_boundary_and_budget_are_enforced() {
    let mut f = Fake {
        pending: 3,
        ..Fake::default()
    };
    let ok = train(&mut f, req()).unwrap();
    assert_eq!(ok.probe_operations, (8 * 6 * 3 + 2) * 5);
    let mut f = Fake {
        pending: 4,
        ..Fake::default()
    };
    assert_eq!(train(&mut f, req()).unwrap_err().cause, Cause::Timeout);
    let mut d = domain();
    d.probe_operations = ok.probe_operations;
    let mut f = Fake {
        pending: 3,
        d: Some(d),
        ..Fake::default()
    };
    assert!(train(&mut f, req()).is_ok());
    d.probe_operations -= 1;
    let mut f = Fake {
        pending: 3,
        d: Some(d),
        ..Fake::default()
    };
    assert_eq!(train(&mut f, req()).unwrap_err().cause, Cause::Budget);
}
#[test]
fn unbound_path_is_explicitly_unsupported_without_any_mmio() {
    for pstate in 0..4 {
        for stage in Stage::ALL {
            assert_eq!(
                train(
                    &mut Unbound,
                    Request {
                        stage,
                        pstate,
                        ..req()
                    }
                ),
                Err(Failure {
                    cause: Cause::Unsupported,
                    rollback_failed: false
                })
            );
        }
    }
}
#[test]
fn invalid_domains_requests_and_grid_coordinates_do_not_begin() {
    let mut f = Fake::default();
    assert_eq!(
        train(&mut f, Request { pstate: 4, ..req() })
            .unwrap_err()
            .cause,
        Cause::InvalidPlan
    );
    assert_eq!(
        train(
            &mut f,
            Request {
                required_mask: 0,
                ..req()
            }
        )
        .unwrap_err()
        .cause,
        Cause::InvalidPlan
    );
    for d in [
        Domain {
            width: 0,
            ..domain()
        },
        Domain {
            height: 65,
            ..domain()
        },
        Domain {
            min_width: 9,
            ..domain()
        },
        Domain {
            min_height: 0,
            ..domain()
        },
        Domain {
            scan_repeats: 0,
            ..domain()
        },
        Domain {
            verify_repeats: 0,
            ..domain()
        },
        Domain {
            polls_per_probe: 0,
            ..domain()
        },
        Domain {
            probe_operations: 0,
            ..domain()
        },
    ] {
        let mut f = Fake {
            d: Some(d),
            ..Fake::default()
        };
        assert_eq!(train(&mut f, req()).unwrap_err().cause, Cause::InvalidPlan);
        assert!(!f.restored && f.scans == 0);
    }
    assert_eq!(
        Grid::default().set(Point { x: 64, y: 0 }, true),
        Err(Cause::InvalidPlan)
    );
}
#[test]
fn seam_windows_and_disjoint_islands_are_not_merged() {
    let mut g = Grid::default();
    let d = Domain {
        width: 8,
        height: 1,
        circular_x: true,
        min_height: 1,
        ..domain()
    };
    for x in [6, 7, 0, 1, 3] {
        g.set(Point { x, y: 0 }, true).unwrap();
    }
    let w = g.select(d).unwrap();
    assert_eq!((w.start.x, w.width, w.center.x), (6, 4, 7));
    let w = g
        .select(Domain {
            circular_x: false,
            ..d
        })
        .unwrap();
    assert_eq!(w.width, 2);
    let mut g = Grid::default();
    let d = Domain {
        width: 8,
        height: 8,
        circular_x: true,
        circular_y: true,
        ..domain()
    };
    for y in [6, 7, 0, 1] {
        for x in [7, 0, 1] {
            g.set(Point { x, y }, true).unwrap();
        }
    }
    let w = g.select(d).unwrap();
    assert_eq!(
        (w.start, w.width, w.height, w.center),
        (Point { x: 7, y: 6 }, 3, 4, Point { x: 0, y: 7 })
    );
}
#[test]
fn width64_and_narrow_windows_and_clipped_boundaries_are_explicit() {
    let mut g = Grid::default();
    let d = Domain {
        width: 64,
        height: 1,
        min_height: 1,
        circular_x: true,
        ..domain()
    };
    for x in 0..64 {
        g.set(Point { x, y: 0 }, true).unwrap();
    }
    assert_eq!(g.select(d).unwrap().width, 64);
    let mut g = Grid::default();
    g.set(Point { x: 3, y: 0 }, true).unwrap();
    assert_eq!(g.select(d), Err(Cause::NoWindow));
    let mut g = Grid::default();
    for x in 0..4 {
        for y in 0..3 {
            g.set(Point { x, y }, true).unwrap();
        }
    }
    assert!(g.select(domain()).unwrap().boundary_contact);
}
// Independent oracle enumerates EVERY rectangle and EACH constituent cell.
// It does not reuse the production bitmask/run algorithm.
fn oracle(g: &Grid, d: Domain) -> Option<(u16, u8, u8)> {
    let mut best = None;
    for y in 0..d.height {
        for x in 0..d.width {
            for height in 1..=d.height {
                for width in 1..=d.width {
                    if width < d.min_width
                        || height < d.min_height
                        || (!d.circular_x && u16::from(x) + u16::from(width) > u16::from(d.width))
                        || (!d.circular_y && u16::from(y) + u16::from(height) > u16::from(d.height))
                    {
                        continue;
                    }
                    let pass = (0..height).all(|dy| {
                        (0..width).all(|dx| {
                            g.get(Point {
                                x: (x + dx) % d.width,
                                y: (y + dy) % d.height,
                            })
                        })
                    });
                    let score = (
                        u16::from(width) * u16::from(height),
                        width.min(height),
                        width,
                    );
                    if pass && best.is_none_or(|s| score > s) {
                        best = Some(score)
                    }
                }
            }
        }
    }
    best
}
#[test]
fn rectangle_objective_matches_independent_exhaustive_oracle() {
    let mut rng = 0x81b3a549u32;
    for circular_x in [false, true] {
        for circular_y in [false, true] {
            for _ in 0..80 {
                let d = Domain {
                    width: 5,
                    height: 4,
                    min_width: 1,
                    min_height: 1,
                    circular_x,
                    circular_y,
                    ..domain()
                };
                let mut g = Grid::default();
                for y in 0..4 {
                    for x in 0..5 {
                        rng ^= rng << 13;
                        rng ^= rng >> 17;
                        rng ^= rng << 5;
                        g.set(Point { x, y }, !rng.is_multiple_of(3)).unwrap();
                    }
                }
                let expected = oracle(&g, d);
                let got = g.select(d).ok().map(|w| {
                    (
                        u16::from(w.width) * u16::from(w.height),
                        w.width.min(w.height),
                        w.width,
                    )
                });
                assert_eq!(got, expected);
            }
        }
    }
}
