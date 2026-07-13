// SPDX-License-Identifier: GPL-2.0-only
use dram_train::search::{
    command::{Cause, Plan, Port, Transaction, MAX_STATUS_READS},
    BackendError, Unbound,
};
use std::cell::Cell;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum Op {
    Parameter(u16),
    Settle,
    Launch,
    Read,
    Clear,
}
struct Model {
    preflight_calls: Cell<u32>,
    supported: bool,
    operations: Vec<Op>,
    fail: Option<Op>,
    clear_fails: bool,
    active: bool,
    nonzero_at: u16,
    raw: u16,
    reads: u16,
}
impl Model {
    fn new(nonzero_at: u16, raw: u16) -> Self {
        Self {
            preflight_calls: Cell::new(0),
            supported: true,
            operations: Vec::new(),
            fail: None,
            clear_fails: false,
            active: false,
            nonzero_at,
            raw,
            reads: 0,
        }
    }
    fn operation(&mut self, op: Op) -> Result<(), BackendError> {
        self.operations.push(op);
        if self.fail == Some(op) {
            Err(BackendError::Fault)
        } else {
            Ok(())
        }
    }
}
impl Port for Model {
    fn preflight(&self, _: Plan) -> Result<(), BackendError> {
        self.preflight_calls.set(self.preflight_calls.get() + 1);
        if self.supported {
            Ok(())
        } else {
            Err(BackendError::Unsupported)
        }
    }
    fn write_parameter(&mut self, p: u16) -> Result<(), BackendError> {
        self.operation(Op::Parameter(p))
    }
    fn settle_parameter(&mut self) -> Result<(), BackendError> {
        self.operation(Op::Settle)
    }
    fn set_launch(&mut self, active: bool) -> Result<(), BackendError> {
        if active {
            self.active = true; // Fault may be reported after the write took effect.
            self.operation(Op::Launch)
        } else {
            self.operation(Op::Clear)?;
            if self.clear_fails {
                return Err(BackendError::Fault);
            }
            self.active = false;
            Ok(())
        }
    }
    fn read_status(&mut self) -> Result<u16, BackendError> {
        self.operation(Op::Read)?;
        self.reads += 1;
        Ok(if self.nonzero_at != 0 && self.reads >= self.nonzero_at {
            self.raw
        } else {
            0
        })
    }
}
fn plan(reads: u16) -> Plan {
    Plan {
        parameter: 0x8123,
        status_reads: reads,
    }
}

#[test]
fn exact_synchronous_order_and_raw_status_retention() {
    let mut t = Transaction::new(Model::new(3, 0x8000));
    let raw = t.observe(plan(10)).unwrap();
    assert_eq!(
        (raw.raw_status, raw.reads, raw.preceding_zero_reads),
        (0x8000, 3, 2)
    );
    assert_eq!(
        t.port().operations,
        [
            Op::Parameter(0x8123),
            Op::Settle,
            Op::Launch,
            Op::Read,
            Op::Read,
            Op::Read,
            Op::Clear
        ]
    );
    assert!(!t.port().active);
    assert!(!t.quarantined());
}

#[test]
fn every_nonzero_word_is_unclassified_including_preexisting_status() {
    // Independent raw values include unknown error/reserved bits. A status
    // already nonzero on the first read is not silently made fresh or passing.
    for word in 1..=u16::MAX {
        let mut t = Transaction::new(Model::new(1, word));
        let raw = t.observe(plan(1)).unwrap();
        assert_eq!(
            (raw.raw_status, raw.reads, raw.preceding_zero_reads),
            (word, 1, 0)
        );
        assert_eq!(t.port().operations.last(), Some(&Op::Clear));
    }
}

#[test]
fn no_extra_poll_at_either_budget_boundary() {
    for at in [1, 2, MAX_STATUS_READS - 1, MAX_STATUS_READS] {
        let mut t = Transaction::new(Model::new(at, 0x1200));
        let raw = t.observe(plan(MAX_STATUS_READS)).unwrap();
        assert_eq!(raw.reads, at);
        assert_eq!(t.port().operations.len(), usize::from(at) + 4);
        assert_eq!(t.port().reads, at);
    }
    let mut t = Transaction::new(Model::new(0, 0));
    let f = t.observe(plan(MAX_STATUS_READS)).unwrap_err();
    assert_eq!(f.cause, Cause::Timeout);
    assert!(f.clear_attempted);
    assert_eq!(f.clear_error, None);
    assert_eq!(t.port().reads, MAX_STATUS_READS);
    assert_eq!(t.port().operations.len(), usize::from(MAX_STATUS_READS) + 4);
    assert!(!t.port().active);
}

#[test]
fn invalid_and_unsupported_requests_have_no_port_mutation() {
    for limit in [0, MAX_STATUS_READS + 1, u16::MAX] {
        let mut t = Transaction::new(Model::new(1, 1));
        let f = t.observe(plan(limit)).unwrap_err();
        assert_eq!(f.cause, Cause::InvalidPlan);
        assert!(!f.clear_attempted);
        assert_eq!(t.port().preflight_calls.get(), 0);
        assert!(t.port().operations.is_empty());
    }
    let mut m = Model::new(1, 1);
    m.supported = false;
    let mut t = Transaction::new(m);
    let f = t.observe(plan(2)).unwrap_err();
    assert_eq!(f.cause, Cause::Port(BackendError::Unsupported));
    assert!(!f.clear_attempted);
    assert!(t.port().operations.is_empty());
    let mut physical = Transaction::new(Unbound);
    assert_eq!(
        physical.observe(plan(2)).unwrap_err().cause,
        Cause::Port(BackendError::Unsupported)
    );
}

#[test]
fn prelaunch_failures_do_not_launch_or_claim_recovery() {
    for op in [Op::Parameter(0x8123), Op::Settle] {
        let mut m = Model::new(1, 1);
        m.fail = Some(op);
        let mut t = Transaction::new(m);
        let f = t.observe(plan(2)).unwrap_err();
        assert_eq!(f.cause, Cause::Port(BackendError::Fault));
        assert!(!f.clear_attempted);
        assert!(!t.port().operations.contains(&Op::Launch));
        assert!(!t.port().operations.contains(&Op::Clear));
        assert!(!t.port().active);
    }
}

#[test]
fn uncertain_launch_and_poll_fault_attempt_exactly_one_clear() {
    for op in [Op::Launch, Op::Read] {
        let mut m = Model::new(1, 1);
        m.fail = Some(op);
        let mut t = Transaction::new(m);
        let f = t.observe(plan(2)).unwrap_err();
        assert_eq!(f.cause, Cause::Port(BackendError::Fault));
        assert!(f.clear_attempted);
        assert_eq!(f.clear_error, None);
        assert_eq!(
            t.port()
                .operations
                .iter()
                .filter(|x| **x == Op::Clear)
                .count(),
            1
        );
        assert!(!t.port().active);
    }
}

#[test]
fn failed_clear_invalidates_raw_output_and_quarantines_without_retry() {
    for outcome in [0, 1] {
        let mut m = Model::new(outcome, 1);
        m.clear_fails = true;
        let mut t = Transaction::new(m);
        let f = t.observe(plan(2)).unwrap_err();
        assert_eq!(
            f.cause,
            if outcome == 0 {
                Cause::Timeout
            } else {
                Cause::Port(BackendError::Fault)
            }
        );
        assert!(f.clear_attempted);
        assert_eq!(f.clear_error, Some(BackendError::Fault));
        assert!(t.quarantined());
        assert!(t.port().active);
        let before = t.port().operations.clone();
        let preflight = t.port().preflight_calls.get();
        let retry = t.observe(plan(2)).unwrap_err();
        assert_eq!(retry.cause, Cause::Quarantined);
        assert!(!retry.clear_attempted);
        assert_eq!(t.port().operations, before);
        assert_eq!(t.port().preflight_calls.get(), preflight);
    }
}
