// SPDX-License-Identifier: GPL-2.0-only
use dram_train::search::{line::*, *};
fn domain(points: u16, circular: bool) -> LineDomain {
    LineDomain {
        points,
        circular,
        min_span: 3,
        scan_repeats: 3,
        verify_repeats: 2,
        polls_per_probe: 4,
        probe_operations: 20000,
    }
}
fn oracle(bits: &[bool], d: LineDomain) -> Result<LineEye, Cause> {
    let mut best: Option<LineEye> = None;
    for start in 0..d.points {
        for span in d.min_span..=d.points {
            if !d.circular && start + span > d.points {
                continue;
            }
            if !(0..span).all(|offset| bits[usize::from((start + offset) % d.points)]) {
                continue;
            }
            let eye = LineEye {
                start,
                span,
                center: ((start + (span - 1) / 2) % d.points) as u8,
                boundary_contact: !d.circular && (start == 0 || start + span == d.points),
            };
            if best.is_none_or(|old| {
                span > old.span
                    || (span == old.span && (eye.center, start) < (old.center, old.start))
            }) {
                best = Some(eye);
            }
        }
    }
    best.ok_or(Cause::NoWindow)
}
#[test]
fn long_streaming_intervals_match_independent_exhaustive_cell_oracle() {
    let mut rng = 0xa733u32;
    let mut cases = 0;
    for points in [1, 2, 63, 64, 65, 127, 128, 211, 212, 255, 256] {
        for circular in [false, true] {
            for fixture in 0..8 {
                let mut d = domain(points, circular);
                d.min_span = 1;
                let bits: Vec<bool> = (0..points)
                    .map(|_| {
                        rng ^= rng << 13;
                        rng ^= rng >> 17;
                        rng ^= rng << 5;
                        match fixture {
                            0 => false,
                            1 => true,
                            _ => !rng.is_multiple_of(3),
                        }
                    })
                    .collect();
                let mut stream = Interval::new(d).unwrap();
                for bit in &bits {
                    stream.push(*bit).unwrap();
                }
                assert_eq!(
                    stream.finish(),
                    oracle(&bits, d),
                    "points{points}/circular{circular}/fixture{fixture}"
                );
                cases += 1;
            }
        }
    }
    assert_eq!(cases, 176);
    assert!(core::mem::size_of::<Interval>() <= 64);
}
#[test]
fn exact_input_count_full_circle_and_clipped_boundaries_are_explicit() {
    let d = domain(256, true);
    let mut stream = Interval::new(d).unwrap();
    for _ in 0..256 {
        stream.push(true).unwrap();
    }
    assert_eq!(stream.push(true), Err(Cause::InvalidPlan));
    assert_eq!(
        stream.finish(),
        Ok(LineEye {
            start: 129,
            span: 256,
            center: 0,
            boundary_contact: false
        })
    );
    let d = domain(212, false);
    let mut stream = Interval::new(d).unwrap();
    for _ in 0..211 {
        stream.push(true).unwrap();
    }
    assert_eq!(stream.finish(), Err(Cause::InvalidPlan));
    let mut stream = Interval::new(d).unwrap();
    for _ in 0..212 {
        stream.push(true).unwrap();
    }
    assert_eq!(
        stream.finish(),
        Ok(LineEye {
            start: 0,
            span: 212,
            center: 105,
            boundary_contact: true
        })
    );
}
struct Sim {
    d: LineDomain,
    start: u16,
    span: u16,
    point: Point,
    old: Point,
    ticket: u32,
    probe: Probe,
    polls: u32,
    pending: u32,
    begun: bool,
    recovered: bool,
    fault: u8,
    rollback_fault: bool,
    scans: u32,
    verifies: u32,
}
impl Backend for Sim {
    type Snapshot = Point;
    fn domain(&self, _: Request) -> Result<Domain, BackendError> {
        panic!("64-cell grid path used for long line")
    }
    fn begin(&mut self, _: Request) -> Result<Point, BackendError> {
        self.begun = true;
        self.old = self.point;
        Ok(self.point)
    }
    fn launch(&mut self, _: Request, p: Point, k: Probe) -> Result<u32, BackendError> {
        self.point = p;
        self.probe = k;
        self.polls = 0;
        self.ticket += 1;
        match k {
            Probe::Scan(_) => self.scans += 1,
            Probe::Verify(_) => self.verifies += 1,
        };
        Ok(self.ticket)
    }
    fn poll(&mut self, ticket: u32) -> Result<Poll, BackendError> {
        self.polls += 1;
        if self.fault == 1 || self.polls <= self.pending {
            return Ok(Poll::Pending);
        }
        let x = u16::from(self.point.x);
        let n = self.d.points;
        let bit0 = (x + n - self.start) % n < self.span;
        let bit1 = (x + n - ((self.start + 2) % n)) % n < self.span;
        let mut failed = u64::from(!bit0) | (u64::from(!bit1) << 1);
        if (self.fault == 4 && matches!(self.probe, Probe::Verify(_)))
            || (self.fault == 5 && matches!(self.probe, Probe::Scan(1)))
        {
            failed = 3;
        }
        Ok(Poll::Ready(Measurement {
            ticket: if self.fault == 2 { ticket - 1 } else { ticket },
            observed_mask: if self.fault == 3 { 1 } else { 3 },
            failed_mask: failed,
        }))
    }
    fn apply(&mut self, _: Request, p: Point) -> Result<(), BackendError> {
        self.point = p;
        if self.fault == 6 {
            Err(BackendError::Fault)
        } else {
            Ok(())
        }
    }
    fn finish(&mut self, _: Request) -> Result<(), BackendError> {
        Ok(())
    }
    fn rollback(&mut self, _: Request, p: Point) -> Result<(), BackendError> {
        self.point = p;
        self.recovered = true;
        if self.rollback_fault {
            Err(BackendError::Fault)
        } else {
            Ok(())
        }
    }
}
impl LineBackend for Sim {
    fn line_domain(&self, _: Request) -> Result<LineDomain, BackendError> {
        Ok(self.d)
    }
}
fn sim(points: u16, circular: bool) -> Sim {
    Sim {
        d: domain(points, circular),
        start: points / 3,
        span: 27,
        point: Point { x: 255, y: 0 },
        old: Point { x: 0, y: 0 },
        ticket: 0,
        probe: Probe::Scan(0),
        polls: 0,
        pending: 0,
        begun: false,
        recovered: false,
        fault: 0,
        rollback_fault: false,
        scans: 0,
        verifies: 0,
    }
}
fn request(stage: Stage, pstate: u8) -> Request {
    Request {
        stage,
        pstate,
        channel: 1,
        rank: 1,
        lane: 3,
        required_mask: 3,
    }
}
#[test]
fn independently_varied_128_212_256_point_windows_train_and_apply_in_four_states() {
    let mut cases = 0;
    for points in [128, 212, 256] {
        for pstate in 0..4 {
            for (i, stage) in [
                Stage::Wck2Ck,
                Stage::ReadGate,
                Stage::ReadCenter,
                Stage::WriteCenter,
            ]
            .into_iter()
            .enumerate()
            {
                let circular = stage == Stage::Wck2Ck;
                let mut b = sim(points, circular);
                b.start = if circular {
                    points - 12 - u16::from(pstate)
                } else {
                    points / 3 + u16::from(pstate) + i as u16
                };
                b.span = 25 + u16::from(pstate);
                b.pending = u32::from(pstate % 2);
                let r = request(stage, pstate);
                let got = train_line(&mut b, r).unwrap();
                let start = (b.start + 2) % points;
                let span = b.span - 2;
                let center = ((start + (span - 1) / 2) % points) as u8;
                assert_eq!(got.request, r);
                assert_eq!(
                    (got.eye.start, got.eye.span, got.eye.center),
                    (start, span, center)
                );
                assert_eq!(b.point, Point { x: center, y: 0 });
                assert!(b.begun && !b.recovered);
                assert_eq!((b.scans, b.verifies), (u32::from(points) * 3, 2));
                assert_eq!(
                    got.probe_operations,
                    (u32::from(points) * 3 + 2) * (2 + b.pending)
                );
                cases += 1;
            }
        }
    }
    assert_eq!(cases, 48);
}
#[test]
fn long_line_faults_and_exact_budget_restore_prior_settings() {
    for (fault, cause) in [
        (1, Cause::Timeout),
        (2, Cause::StaleObservation),
        (3, Cause::MissingObservation),
        (4, Cause::Verification),
        (5, Cause::NoWindow),
        (6, Cause::BackendFault),
    ] {
        let mut b = sim(212, false);
        b.fault = fault;
        let old = b.point;
        let e = train_line(&mut b, request(Stage::ReadCenter, 2)).unwrap_err();
        assert_eq!(e.cause, cause);
        assert!(!e.rollback_failed);
        assert!(b.recovered);
        assert_eq!(b.point, old);
    }
    let mut b = sim(256, false);
    b.pending = 3;
    let r = request(Stage::WriteCenter, 3);
    let used = train_line(&mut b, r).unwrap().probe_operations;
    let mut b = sim(256, false);
    b.pending = 3;
    b.d.probe_operations = used;
    assert!(train_line(&mut b, r).is_ok());
    let mut b = sim(256, false);
    b.pending = 3;
    b.d.probe_operations = used - 1;
    b.rollback_fault = true;
    let e = train_line(&mut b, r).unwrap_err();
    assert_eq!(e.cause, Cause::Budget);
    assert!(e.rollback_failed && b.recovered);
    assert_eq!(b.point, Point { x: 255, y: 0 });
}
#[test]
fn invalid_long_line_domains_and_actual_a733_do_not_begin() {
    for case in 0..6 {
        let mut b = sim(212, false);
        let mut r = request(Stage::ReadGate, 0);
        match case {
            0 => b.d.points = 257,
            1 => b.d.min_span = 213,
            2 => b.d.scan_repeats = 0,
            3 => b.d.circular = true,
            4 => r.pstate = 4,
            5 => r.stage = Stage::Dfe,
            _ => unreachable!(),
        }
        assert_eq!(train_line(&mut b, r).unwrap_err().cause, Cause::InvalidPlan);
        assert!(!b.begun);
    }
    for pstate in 0..4 {
        for stage in [
            Stage::Wck2Ck,
            Stage::ReadGate,
            Stage::ReadCenter,
            Stage::WriteCenter,
        ] {
            assert_eq!(
                train_line(&mut Unbound, request(stage, pstate))
                    .unwrap_err()
                    .cause,
                Cause::Unsupported
            );
        }
    }
}
