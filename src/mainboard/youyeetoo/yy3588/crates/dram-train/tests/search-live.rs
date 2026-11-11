// SPDX-License-Identifier: GPL-2.0-only
//! The live-readback contract of the search engines (A7A m13, Oct 15): a
//! backend whose knob is double-buffered and never latched reads the written
//! value back from its CSR while the live copy stays put. The engines must
//! stop at the first such point, name it, and never report an eye from it.
use dram_train::search::line::{train_line, LineBackend, LineDomain};
use dram_train::search::*;

fn domain() -> Domain {
    Domain {
        width: 8,
        height: 6,
        circular_x: false,
        circular_y: false,
        min_width: 2,
        min_height: 2,
        scan_repeats: 2,
        verify_repeats: 2,
        polls_per_probe: 2,
        probe_operations: 10000,
    }
}
fn req() -> Request {
    Request {
        stage: Stage::ReadCenter,
        pstate: 1,
        channel: 0,
        rank: 1,
        lane: 2,
        required_mask: 1,
    }
}

/// A PHY knob with a CSR and a live copy. `latch`: the backend's
/// launch/apply fire the latch pulse, so the live copy follows the CSR.
/// `readback`: the backend can read the live copy at all.
#[derive(Default)]
struct Knob {
    csr: Point,
    live: Point,
    latch: bool,
    readback: bool,
    ticket: u32,
    scans: u32,
    verifies: u32,
    applied: u32,
    restored: bool,
    finished: bool,
}
impl Knob {
    fn program(&mut self, p: Point) {
        self.csr = p;
        if self.latch {
            self.live = p;
        }
    }
}
impl Backend for Knob {
    type Snapshot = Point;
    fn domain(&self, _: Request) -> Result<Domain, BackendError> {
        Ok(domain())
    }
    fn begin(&mut self, _: Request) -> Result<Point, BackendError> {
        Ok(self.live)
    }
    fn launch(&mut self, _: Request, p: Point, k: Probe) -> Result<u32, BackendError> {
        self.program(p);
        match k {
            Probe::Scan(_) => self.scans += 1,
            Probe::Verify(_) => self.verifies += 1,
        }
        self.ticket += 1;
        Ok(self.ticket)
    }
    fn poll(&mut self, t: u32) -> Result<Poll, BackendError> {
        // The data path samples at the LIVE point.
        let p = self.live;
        // (y 0..=4 so the line engine, which scans y = 0, sees the eye too.)
        let pass = (2..=5).contains(&p.x) && p.y <= 4;
        Ok(Poll::Ready(Measurement {
            ticket: t,
            observed_mask: 1,
            failed_mask: u64::from(!pass),
        }))
    }
    fn apply(&mut self, _: Request, p: Point) -> Result<(), BackendError> {
        self.program(p);
        self.applied += 1;
        Ok(())
    }
    fn finish(&mut self, _: Request) -> Result<(), BackendError> {
        self.finished = true;
        Ok(())
    }
    fn rollback(&mut self, _: Request, s: Point) -> Result<(), BackendError> {
        self.restored = true;
        self.live = s;
        self.csr = s;
        Ok(())
    }
    fn verify_live(&mut self, _: Request, p: Point) -> Result<Live, BackendError> {
        if !self.readback {
            return Ok(Live::Unverifiable);
        }
        // Reading the CSR back would say "p" here too; the contract is the
        // live copy.
        Ok(if self.live == p {
            Live::Live
        } else {
            Live::Stale
        })
    }
}
impl LineBackend for Knob {
    fn line_domain(&self, _: Request) -> Result<LineDomain, BackendError> {
        Ok(LineDomain {
            points: 8,
            circular: false,
            min_span: 2,
            scan_repeats: 2,
            verify_repeats: 1,
            polls_per_probe: 2,
            probe_operations: 10000,
        })
    }
}

#[test]
fn a_latched_knob_is_verified_at_every_programming() {
    let mut k = Knob {
        latch: true,
        readback: true,
        ..Knob::default()
    };
    let got = train(&mut k, req()).unwrap();
    assert_eq!(got.eye.center, Point { x: 3, y: 2 });
    // Every scan launch, the apply and every verify launch were programmed
    // and confirmed live.
    let programmed = k.scans + k.applied + k.verifies;
    assert_eq!(got.live.applied, programmed);
    assert_eq!(
        got.live,
        Liveness {
            applied: programmed,
            live: programmed,
            unverifiable: 0
        }
    );
    assert!(got.live.all_verified());
    assert_eq!(
        format!("{}", got.live),
        format!("points verified live: {programmed}/{programmed}")
    );
    assert!(k.finished && !k.restored);
}

#[test]
fn an_unlatched_knob_stops_the_search_at_its_first_point_and_names_it() {
    // The live copy starts at (4, 3), inside the eye: without the live check
    // every scan would "pass" there and the engine would report an eye of
    // the whole grid (m13l-m13r: six hours of sweeps that swept nothing).
    let start = Point { x: 4, y: 3 };
    let mut k = Knob {
        csr: start,
        live: start,
        latch: false,
        readback: true,
        ..Knob::default()
    };
    let r = req();
    assert_eq!(
        train(&mut k, r),
        Err(Failure {
            cause: Cause::NotLive {
                request: r,
                point: Point { x: 0, y: 0 }
            },
            rollback_failed: false
        })
    );
    // One launch, no measurement counted, no apply, rolled back.
    assert_eq!((k.scans, k.applied, k.verifies), (1, 0, 0));
    assert!(k.restored && !k.finished);
    assert_eq!(k.live, start);
    // The line engine too.
    let mut k = Knob {
        csr: start,
        live: start,
        readback: true,
        ..Knob::default()
    };
    let e = train_line(&mut k, r).unwrap_err();
    assert_eq!(
        e.cause,
        Cause::NotLive {
            request: r,
            point: Point { x: 0, y: 0 }
        }
    );
    assert!(k.restored);
}

#[test]
fn a_knob_without_readback_is_declared_not_assumed() {
    let mut k = Knob {
        latch: true,
        readback: false,
        ..Knob::default()
    };
    let got = train(&mut k, req()).unwrap();
    let programmed = k.scans + k.applied + k.verifies;
    assert_eq!(
        got.live,
        Liveness {
            applied: programmed,
            live: 0,
            unverifiable: programmed
        }
    );
    assert!(!got.live.all_verified());
    assert_eq!(
        format!("{}", got.live),
        format!("points verified live: 0/{programmed} (knob not verifiable)")
    );
    // A backend that latches but cannot read back still trains: the
    // unlatched failure above is caught only by a readback.
    let mut k = Knob {
        latch: false,
        readback: false,
        live: Point { x: 4, y: 3 },
        ..Knob::default()
    };
    let got = train(&mut k, req()).unwrap();
    assert_eq!(got.live.live, 0);
    assert_eq!(got.live.unverifiable, got.live.applied);
    // The line result carries the same count.
    let mut k = Knob {
        latch: true,
        readback: true,
        ..Knob::default()
    };
    let got = train_line(&mut k, req()).unwrap();
    assert_eq!(got.live.applied, k.scans + k.applied + k.verifies);
    assert!(got.live.all_verified());
}

#[test]
fn a_stale_final_apply_is_caught_after_a_clean_scan() {
    // Scans latch (the backend pulses in launch) but apply forgets the
    // pulse: the chosen centre never reaches the live gate.
    struct ApplyForgets(Knob);
    impl Backend for ApplyForgets {
        type Snapshot = Point;
        fn domain(&self, r: Request) -> Result<Domain, BackendError> {
            self.0.domain(r)
        }
        fn begin(&mut self, r: Request) -> Result<Point, BackendError> {
            self.0.begin(r)
        }
        fn launch(&mut self, r: Request, p: Point, k: Probe) -> Result<u32, BackendError> {
            self.0.launch(r, p, k)
        }
        fn poll(&mut self, t: u32) -> Result<Poll, BackendError> {
            self.0.poll(t)
        }
        fn apply(&mut self, _: Request, p: Point) -> Result<(), BackendError> {
            self.0.csr = p;
            self.0.applied += 1;
            Ok(())
        }
        fn finish(&mut self, r: Request) -> Result<(), BackendError> {
            self.0.finish(r)
        }
        fn rollback(&mut self, r: Request, s: Point) -> Result<(), BackendError> {
            self.0.rollback(r, s)
        }
        fn verify_live(&mut self, r: Request, p: Point) -> Result<Live, BackendError> {
            self.0.verify_live(r, p)
        }
    }
    let mut k = ApplyForgets(Knob {
        latch: true,
        readback: true,
        ..Knob::default()
    });
    let r = req();
    let e = train(&mut k, r).unwrap_err();
    assert_eq!(
        e.cause,
        Cause::NotLive {
            request: r,
            point: Point { x: 3, y: 2 }
        }
    );
    assert_eq!(
        k.0.verifies, 0,
        "no verify launch at a point that is not live"
    );
    assert!(k.0.restored);
}
