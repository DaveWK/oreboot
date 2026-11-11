// SPDX-License-Identifier: GPL-2.0-only
use dram_train::search::{
    lpddr5::{DramVref, VrefCode},
    vref_mr::{Cause, Port, Setting, Step, Target, Transaction, MAX_SETTINGS},
    BackendError, Unbound,
};
use std::cell::Cell;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum Op {
    Read(usize),
    Write(usize, u8),
    Settle(usize),
}
struct Model {
    values: [u8; MAX_SETTINGS],
    operations: Vec<Op>,
    faults: Vec<usize>,
    corrupt: Vec<usize>,
    supported: bool,
    preflights: Cell<u32>,
}
fn index(t: Target) -> usize {
    usize::from(t.channel) * 6
        + usize::from(t.rank) * 3
        + match t.register {
            DramVref::Ca => 0,
            DramVref::DqLower => 1,
            DramVref::DqUpper => 2,
        }
}
impl Model {
    fn new() -> Self {
        Self {
            values: [0x83; MAX_SETTINGS],
            operations: vec![],
            faults: vec![],
            corrupt: vec![],
            supported: true,
            preflights: Cell::new(0),
        }
    }
    fn op(&mut self, op: Op) -> Result<usize, BackendError> {
        let call = self.operations.len();
        self.operations.push(op);
        if self.faults.contains(&call) {
            Err(BackendError::Fault)
        } else {
            Ok(call)
        }
    }
}
impl Port for Model {
    fn preflight(&self, _: &[Setting]) -> Result<(), BackendError> {
        self.preflights.set(self.preflights.get() + 1);
        if self.supported {
            Ok(())
        } else {
            Err(BackendError::Unsupported)
        }
    }
    fn read(&mut self, target: Target) -> Result<u8, BackendError> {
        let i = index(target);
        let call = self.op(Op::Read(i))?;
        Ok(self.values[i] ^ u8::from(self.corrupt.contains(&call)))
    }
    fn write(&mut self, target: Target, value: u8) -> Result<(), BackendError> {
        let i = index(target);
        self.values[i] = value; // Fault can follow a real change.
        self.op(Op::Write(i, value)).map(|_| ())
    }
    fn settle(&mut self, target: Target) -> Result<(), BackendError> {
        self.op(Op::Settle(index(target))).map(|_| ())
    }
}
fn setting(channel: u8, rank: u8, register: DramVref, code: u8) -> Setting {
    Setting {
        target: Target {
            channel,
            rank,
            register,
        },
        value: VrefCode::new(code).unwrap(),
        minimum: 0,
        maximum: 127,
    }
}
fn pair() -> [Setting; 2] {
    [
        setting(0, 0, DramVref::Ca, 42),
        setting(1, 1, DramVref::DqUpper, 63),
    ]
}

#[test]
fn exact_snapshot_before_write_order_and_unrelated_bit_preservation() {
    let mut model = Model::new();
    model.values[11] = 0x15;
    let mut t = Transaction::new(model);
    t.apply(&pair()).unwrap();
    assert_eq!(
        t.port().operations,
        [
            Op::Read(0),
            Op::Read(11),
            Op::Write(0, 0xaa),
            Op::Settle(0),
            Op::Read(0),
            Op::Write(11, 0x3f),
            Op::Settle(11),
            Op::Read(11)
        ]
    );
    assert_eq!((t.port().values[0], t.port().values[11]), (0xaa, 0x3f));
    assert_eq!(t.port().preflights.get(), 1);
    assert!(!t.quarantined());
}
#[test]
fn every_original_byte_and_vref_code_preserves_only_bit7() {
    for register in [DramVref::Ca, DramVref::DqLower, DramVref::DqUpper] {
        for original in 0..=255u8 {
            for code in 0..=127u8 {
                let s = setting(1, 0, register, code);
                let i = index(s.target);
                let mut model = Model::new();
                model.values[i] = original;
                let mut t = Transaction::new(model);
                t.apply(&[s]).unwrap();
                assert_eq!(t.port().values[i], (original / 128) * 128 + code);
                assert_eq!(t.port().operations.len(), 4);
            }
        }
    }
}
#[test]
fn malformed_late_fields_and_duplicate_targets_stop_before_port_access() {
    let baseline = pair();
    let mut cases = vec![
        vec![],
        vec![baseline[0]; MAX_SETTINGS + 1],
        vec![baseline[0]; 2],
    ];
    for field in 0..5 {
        let mut p = baseline;
        match field {
            0 => p[1].target.channel = 2,
            1 => p[1].target.rank = 2,
            2 => p[1].minimum = 64,
            3 => p[1].maximum = 128,
            _ => {
                p[1].minimum = 10;
                p[1].maximum = 9;
            }
        }
        cases.push(p.to_vec());
    }
    for p in cases {
        let mut t = Transaction::new(Model::new());
        let f = t.apply(&p).unwrap_err();
        assert_eq!(f.cause, Cause::InvalidPlan);
        assert_eq!(f.attempted, 0);
        assert!(t.port().operations.is_empty());
        assert_eq!(t.port().preflights.get(), 0);
    }
}
#[test]
fn unsupported_port_and_each_snapshot_failure_make_no_writes() {
    let mut t = Transaction::new(Unbound);
    assert_eq!(
        t.apply(&pair()).unwrap_err().cause,
        Cause::Preflight(BackendError::Unsupported)
    );
    let mut model = Model::new();
    model.supported = false;
    let mut t = Transaction::new(model);
    assert!(matches!(
        t.apply(&pair()).unwrap_err().cause,
        Cause::Preflight(_)
    ));
    assert!(t.port().operations.is_empty());
    for at in 0..2 {
        let mut model = Model::new();
        model.faults = vec![at];
        let mut t = Transaction::new(model);
        let f = t.apply(&pair()).unwrap_err();
        assert_eq!(
            f.cause,
            Cause::Operation {
                index: at as u8,
                step: Step::Snapshot,
                error: BackendError::Fault
            }
        );
        assert_eq!(f.attempted, 0);
        assert_eq!(t.port().values, [0x83; MAX_SETTINGS]);
        assert!(t
            .port()
            .operations
            .iter()
            .all(|op| matches!(op, Op::Read(_))));
    }
}
#[test]
fn every_apply_fault_restores_uncertain_writes_and_stops_the_prefix() {
    let normal = [
        Op::Read(0),
        Op::Read(11),
        Op::Write(0, 0xaa),
        Op::Settle(0),
        Op::Read(0),
        Op::Write(11, 0xbf),
        Op::Settle(11),
        Op::Read(11),
    ];
    for at in 2..8 {
        let mut model = Model::new();
        model.faults = vec![at];
        let mut t = Transaction::new(model);
        let f = t.apply(&pair()).unwrap_err();
        let count = if at < 5 { 1 } else { 2 };
        assert_eq!(f.attempted, if count == 1 { 1 } else { 3 });
        let mut expected = normal[..=at].to_vec();
        for i in [0, 11].into_iter().take(count) {
            expected.extend([Op::Write(i, 0x83), Op::Settle(i), Op::Read(i)]);
        }
        assert_eq!(t.port().operations, expected);
        assert_eq!(t.port().values, [0x83; MAX_SETTINGS]);
        assert!(!t.quarantined());
        assert!(f.restore.iter().all(|r| *r == Default::default()));
    }
}
#[test]
fn readback_mismatch_is_a_failure_and_restores_original_bytes() {
    let mut model = Model::new();
    model.corrupt = vec![7];
    let mut t = Transaction::new(model);
    let f = t.apply(&pair()).unwrap_err();
    assert_eq!(
        f.cause,
        Cause::Readback {
            index: 1,
            expected: 0xbf,
            observed: 0xbe
        }
    );
    assert_eq!(f.attempted, 3);
    assert_eq!(t.port().values, [0x83; MAX_SETTINGS]);
    assert_eq!(t.port().operations.len(), 14);
    assert!(!t.quarantined());
}
#[test]
fn all_restore_write_and_read_errors_survive_and_quarantine_retry() {
    for faults in [vec![7, 8, 11], vec![7, 10, 13]] {
        let writes = faults[1] == 8;
        let mut model = Model::new();
        model.faults = faults;
        let mut t = Transaction::new(model);
        let f = t.apply(&pair()).unwrap_err();
        for r in &f.restore[..2] {
            if writes {
                assert_eq!(r.write, Some(BackendError::Fault));
            } else {
                assert_eq!(r.readback, Some(BackendError::Fault));
            }
        }
        assert_eq!(t.port().operations.len(), 14);
        assert!(t.quarantined());
        assert_eq!(t.apply(&pair()).unwrap_err().cause, Cause::Quarantined);
        assert_eq!(t.port().operations.len(), 14);
        assert_eq!(t.port().preflights.get(), 1);
    }
}
#[test]
fn failed_restore_settle_never_authorizes_read_but_attempts_other_targets() {
    let mut model = Model::new();
    model.faults = vec![7, 9, 11];
    let mut t = Transaction::new(model);
    let f = t.apply(&pair()).unwrap_err();
    assert_eq!(
        &t.port().operations[8..],
        [
            Op::Write(0, 0x83),
            Op::Settle(0),
            Op::Write(11, 0x83),
            Op::Settle(11)
        ]
    );
    for r in &f.restore[..2] {
        assert_eq!(r.settle, Some(BackendError::Fault));
        assert_eq!(r.readback, None);
    }
    assert!(t.quarantined());
}
#[test]
fn restore_readback_mismatches_are_retained_independently() {
    let mut model = Model::new();
    model.corrupt = vec![7, 10, 13];
    let mut t = Transaction::new(model);
    let f = t.apply(&pair()).unwrap_err();
    assert_eq!(f.restore[0].mismatch, Some(0x82));
    assert_eq!(f.restore[1].mismatch, Some(0x82));
    assert!(t.quarantined());
    assert_eq!(t.port().operations.len(), 14);
}
#[test]
fn all_twelve_unique_mrs_have_exact_bounded_full_byte_operations() {
    let mut settings = vec![];
    for channel in 0..2 {
        for rank in 0..2 {
            for register in [DramVref::Ca, DramVref::DqLower, DramVref::DqUpper] {
                settings.push(setting(channel, rank, register, 64));
            }
        }
    }
    let mut model = Model::new();
    model.faults = vec![47];
    let mut t = Transaction::new(model);
    let f = t.apply(&settings).unwrap_err();
    assert_eq!(f.attempted, 0xfff);
    assert_eq!(t.port().operations.len(), 84);
    assert_eq!(t.port().values, [0x83; MAX_SETTINGS]);
    assert!(!t.quarantined());
}

#[test]
fn out_of_part_range_snapshot_is_never_written_back_or_partially_applied() {
    for at in 0..2 {
        let mut settings = pair();
        settings[at].minimum = 10;
        let mut t = Transaction::new(Model::new());
        let f = t.apply(&settings).unwrap_err();
        assert_eq!(
            f.cause,
            Cause::SnapshotRange {
                index: at as u8,
                observed: 0x83
            }
        );
        assert_eq!(f.attempted, 0);
        assert_eq!(t.port().operations.len(), at + 1);
        assert!(t
            .port()
            .operations
            .iter()
            .all(|op| matches!(op, Op::Read(_))));
    }
}
