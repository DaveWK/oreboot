// SPDX-License-Identifier: GPL-2.0-only
use dram_train::search::{
    matrix::{train as train_matrix, MatrixBackend, MatrixDomain, MatrixEye, Rectangle},
    *,
};
fn domain(w: u16, h: u16, circular: bool) -> MatrixDomain {
    MatrixDomain {
        width: w,
        height: h,
        circular_timing: circular,
        min_width: 1,
        min_height: 1,
        scan_repeats: 2,
        verify_repeats: 2,
        polls_per_probe: 2,
        probe_operations: 300000,
    }
}
fn oracle(cells: &[bool], d: MatrixDomain) -> Result<MatrixEye, Cause> {
    let mut best = None;
    for top in 0..d.height {
        for left in 0..d.width {
            for h in d.min_height..=d.height - top {
                for w in d.min_width..=d.width {
                    if !d.circular_timing && left + w > d.width {
                        continue;
                    }
                    if !(0..h).all(|y| {
                        (0..w)
                            .all(|x| cells[usize::from((top + y) * d.width + (left + x) % d.width)])
                    }) {
                        continue;
                    }
                    let eye = MatrixEye {
                        start: Point {
                            x: left as u8,
                            y: top as u8,
                        },
                        width: w,
                        height: h,
                        center: Point {
                            x: ((left + (w - 1) / 2) % d.width) as u8,
                            y: (top + (h - 1) / 2) as u8,
                        },
                        boundary_contact: (!d.circular_timing
                            && (left == 0 || left + w == d.width))
                            || top == 0
                            || top + h == d.height,
                    };
                    let score = |e: MatrixEye| {
                        (
                            u32::from(e.width) * u32::from(e.height),
                            e.width.min(e.height),
                            e.width,
                        )
                    };
                    if best.is_none_or(|old: MatrixEye| {
                        score(eye) > score(old)
                            || (score(eye) == score(old)
                                && (eye.center.y, eye.center.x, eye.start.y, eye.start.x)
                                    < (old.center.y, old.center.x, old.start.y, old.start.x))
                    }) {
                        best = Some(eye);
                    }
                }
            }
        }
    }
    best.ok_or(Cause::NoWindow)
}
#[test]
fn streaming_histograms_match_independent_exhaustive_cell_rectangle_oracle() {
    let mut rng = 0xa7335u32;
    for case in 0..640 {
        let mut next = || {
            rng ^= rng << 13;
            rng ^= rng >> 17;
            rng ^= rng << 5;
            rng
        };
        let mut d = domain(
            (next() % 8 + 1) as u16,
            (next() % 7 + 1) as u16,
            case % 2 == 0,
        );
        d.min_width = (next() % u32::from(d.width) + 1) as u16;
        d.min_height = (next() % u32::from(d.height) + 1) as u16;
        let cells: Vec<bool> = (0..d.width * d.height)
            .map(|_| match case % 16 {
                0 => true,
                1 => false,
                _ => !next().is_multiple_of(3),
            })
            .collect();
        let mut r = Rectangle::new(d).unwrap();
        for bit in &cells {
            r.push(*bit).unwrap();
        }
        assert_eq!(r.finish(), oracle(&cells, d), "case{case}");
    }
    assert!(core::mem::size_of::<Rectangle>() <= 560);
}
#[test]
fn full256_dimensions_seam_clipping_and_max_histogram_stack_are_exact() {
    for circular in [false, true] {
        let d = domain(256, 256, circular);
        let mut r = Rectangle::new(d).unwrap();
        for _ in 0..65536 {
            r.push(true).unwrap();
        }
        assert_eq!(r.push(true), Err(Cause::InvalidPlan));
        let eye = r.finish().unwrap();
        assert_eq!((eye.width, eye.height), (256, 256));
        assert_eq!(
            eye.center,
            Point {
                x: if circular { 0 } else { 127 },
                y: 127
            }
        );
        assert!(eye.boundary_contact);
        let mut r = Rectangle::new(d).unwrap();
        // Last row has strictly increasing histogram heights1..256.
        for y in 0..256 {
            for x in 0..256 {
                r.push(y + x >= 255).unwrap();
            }
        }
        let eye = r.finish().unwrap();
        assert_eq!((eye.width, eye.height), (129, 128));
        assert_eq!(eye.center, Point { x: 191, y: 191 });
    }
    let d = domain(256, 128, true);
    let mut r = Rectangle::new(d).unwrap();
    for y in 0..128 {
        for x in 0..256 {
            r.push(!(6..250).contains(&x) && (10..20).contains(&y))
                .unwrap();
        }
    }
    let eye = r.finish().unwrap();
    assert_eq!((eye.width, eye.height), (12, 10));
    assert_eq!(eye.start, Point { x: 250, y: 10 });
    assert_eq!(eye.center, Point { x: 255, y: 14 });
    assert!(!eye.boundary_contact);
    let mut r = Rectangle::new(d).unwrap();
    r.push(true).unwrap();
    assert_eq!(r.finish(), Err(Cause::InvalidPlan));
}
struct Sim {
    d: MatrixDomain,
    point: Point,
    ticket: u32,
    probe: Probe,
    old: Point,
    begun: bool,
    recovered: bool,
    fault: u8,
    rollback_fault: bool,
    scans: u32,
    verifies: u32,
}
impl Sim {
    fn new(d: MatrixDomain) -> Self {
        Self {
            d,
            point: Point { x: 9, y: 9 },
            ticket: 0,
            probe: Probe::Scan(0),
            old: Point { x: 9, y: 9 },
            begun: false,
            recovered: false,
            fault: 0,
            rollback_fault: false,
            scans: 0,
            verifies: 0,
        }
    }
}
impl Backend for Sim {
    type Snapshot = Point;
    fn domain(&self, _: Request) -> Result<Domain, BackendError> {
        panic!("joint domain silently downsampled")
    }
    fn begin(&mut self, _: Request) -> Result<Point, BackendError> {
        self.begun = true;
        self.old = self.point;
        Ok(self.old)
    }
    fn launch(&mut self, _: Request, p: Point, probe: Probe) -> Result<u32, BackendError> {
        self.point = p;
        self.probe = probe;
        self.ticket += 1;
        if matches!(probe, Probe::Scan(_)) {
            self.scans += 1;
        } else {
            self.verifies += 1;
        }
        Ok(self.ticket)
    }
    fn poll(&mut self, ticket: u32) -> Result<Poll, BackendError> {
        if self.fault == 1 {
            return Ok(Poll::Pending);
        }
        let offset = (u16::from(self.point.x) + self.d.width - 250 % self.d.width) % self.d.width;
        // Two independently supplied bit windows: timing[0..11] vs[2..13],
        // Vref[60..68] vs[61..69]. Common rectangle timing[2..11],Vref[61..68].
        let mut failed = u64::from(offset >= 12 || !(60..=68).contains(&self.point.y))
            | (u64::from(!(2..14).contains(&offset) || !(61..=69).contains(&self.point.y)) << 1);
        if self.fault == 4 && matches!(self.probe, Probe::Verify(_)) {
            failed = 3;
        }
        if self.fault == 5 && self.probe == Probe::Scan(1) {
            failed = 3;
        }
        Ok(Poll::Ready(Measurement {
            ticket: if self.fault == 2 { ticket + 1 } else { ticket },
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
        if self.fault == 7 {
            Err(BackendError::Fault)
        } else {
            Ok(())
        }
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
impl MatrixBackend for Sim {
    fn matrix_domain(&self, _: Request) -> Result<MatrixDomain, BackendError> {
        Ok(self.d)
    }
}
fn request(stage: Stage, pstate: u8) -> Request {
    Request {
        stage,
        pstate,
        channel: 0,
        rank: 0,
        lane: 0,
        required_mask: 3,
    }
}
#[test]
fn independent_bit_joint_windows_apply_fresh_results_in_all_four_states() {
    for state in 0..4 {
        for stage in [Stage::Ca, Stage::ReadVref, Stage::WriteVref] {
            let d = domain(256, 128, true);
            let mut b = Sim::new(d);
            let got = train_matrix(&mut b, request(stage, state)).unwrap();
            assert_eq!(got.eye.start, Point { x: 252, y: 61 });
            assert_eq!(got.eye.center, Point { x: 0, y: 64 });
            assert_eq!((got.eye.width, got.eye.height), (10, 8));
            assert_eq!(b.point, got.eye.center);
            assert_eq!(b.scans, 65536);
            assert_eq!(b.verifies, 2);
            assert_eq!(got.probe_operations, 131076);
        }
    }
}
#[test]
fn joint_probe_faults_and_budget_restore_pretraining_settings() {
    for (fault, cause) in [
        (1, Cause::Timeout),
        (2, Cause::StaleObservation),
        (3, Cause::MissingObservation),
        (4, Cause::Verification),
        (5, Cause::NoWindow),
        (6, Cause::BackendFault),
        (7, Cause::BackendFault),
        (8, Cause::Budget),
    ] {
        let mut d = domain(256, 128, true);
        if fault == 8 {
            d.probe_operations = 131075;
        }
        let mut b = Sim::new(d);
        b.fault = fault;
        b.rollback_fault = fault == 4;
        let old = b.point;
        let got = train_matrix(&mut b, request(Stage::Ca, 0)).unwrap_err();
        assert_eq!(got.cause, cause);
        assert_eq!(got.rollback_failed, fault == 4);
        assert!(b.recovered);
        assert_eq!(b.point, old);
    }
    for case in 0..6 {
        let mut d = domain(256, 128, true);
        let mut r = request(Stage::Ca, 0);
        match case {
            0 => d.width = 257,
            1 => d.height = 0,
            2 => d.min_height = 129,
            3 => d.scan_repeats = 0,
            4 => r.pstate = 4,
            5 => r.stage = Stage::Dfe,
            _ => unreachable!(),
        }
        let mut b = Sim::new(d);
        assert_eq!(
            train_matrix(&mut b, r).unwrap_err().cause,
            Cause::InvalidPlan
        );
        assert!(!b.begun);
    }
    for state in 0..4 {
        for stage in [Stage::Ca, Stage::ReadVref, Stage::WriteVref] {
            assert_eq!(
                train_matrix(&mut Unbound, request(stage, state))
                    .unwrap_err()
                    .cause,
                Cause::Unsupported
            );
        }
    }
}
