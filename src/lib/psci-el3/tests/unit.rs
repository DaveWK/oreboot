// SPDX-License-Identifier: GPL-2.0-only
//! The dispatcher on a board other than the A733: four cores numbered by
//! Aff0, a plain mutex lock, workaround 3 required, and the full CPU_OFF
//! path through `cpu_off_finish`.

use psci_el3::{fid, ret, Cpu, LockDenied, Platform, PowerOnError, Psci, State};
use std::cell::{Cell, RefCell};
use std::sync::{Mutex, MutexGuard};

#[derive(Default)]
struct Board {
    lock: Mutex<()>,
    deny: Cell<bool>,
    core: Cell<usize>,
    on_err: Cell<Option<PowerOnError>>,
    events: RefCell<Vec<String>>,
    a76: bool,
}
impl Board {
    fn ev(&self, s: String) {
        self.events.borrow_mut().push(s);
    }
}
impl Platform for Board {
    type Guard<'a> = MutexGuard<'a, ()>;
    fn acquire(&self) -> Result<Self::Guard<'_>, LockDenied> {
        if self.deny.get() {
            return Err(LockDenied);
        }
        Ok(self.lock.try_lock().expect("lock is not re-entered"))
    }
    unsafe fn load32(&self, p: *const u32) -> u32 {
        p.read_volatile()
    }
    unsafe fn store32(&self, p: *mut u32, v: u32) {
        p.write_volatile(v)
    }
    fn barrier(&self) {
        self.ev("barrier".into());
    }
    fn current_core(&self) -> usize {
        self.core.get()
    }
    fn core_index(&self, mpidr: u64) -> Option<usize> {
        let aff0 = (mpidr & 0xff) as usize;
        (mpidr & !0xff == 0 && aff0 < 4).then_some(aff0)
    }
    fn valid_ns_entry(&self, entry: u64) -> bool {
        entry.is_multiple_of(4) && (0x8000_0000..0x1_0000_0000).contains(&entry)
    }
    fn cpu_power_on(&self, idx: usize) -> Result<(), PowerOnError> {
        self.ev(format!("on {idx}"));
        self.on_err.get().map_or(Ok(()), Err)
    }
    fn cpu_power_off_prepare(&self, idx: usize) {
        self.ev(format!("off-prepare {idx}"));
    }
    fn gic_cpu_off(&self, gicr: usize) {
        self.ev(format!("gic-off {gicr:#x}"));
    }
    fn standby(&self) {
        self.ev("standby".into());
    }
    fn cpu_off_finish(&self) -> i64 {
        self.ev("off-finish".into());
        0x77
    }
    fn system_off(&self) -> i64 {
        self.ev("system-off".into());
        0x55
    }
    fn system_reset(&self) -> i64 {
        self.ev("system-reset".into());
        0x66
    }
    fn workaround_3_required(&self) -> bool {
        self.a76
    }
    fn sip(&self, f: u32) -> Option<i64> {
        (f == 0x8200_0010).then_some(42)
    }
    fn log(&self, args: std::fmt::Arguments) {
        self.ev(format!("log {args}"));
    }
}

fn smc(ps: &Psci<4>, b: &Board, f: u32, a1: u64, a2: u64, a3: u64) -> i64 {
    let mut x = [f as u64, a1, a2, a3];
    ps.handle_smc(b, &mut x);
    x[0] as i64
}

#[test]
fn cpu_on_off_cycle_on_aff0_numbering() {
    let b = Board::default();
    let ps = Psci::<4>::new();
    ps.set_cpu(
        &b,
        0,
        Cpu {
            state: State::On,
            gicr: 0x1000,
            ..psci_el3::CPU_INIT
        },
    );
    // Aff1 numbering (the A733's) is not this board's.
    assert_eq!(
        smc(&ps, &b, fid::CPU_ON, 0x100, 0x8000_0000, 0),
        ret::INVALID_PARAMETERS
    );
    assert_eq!(
        smc(&ps, &b, fid::CPU_ON, 4, 0x8000_0000, 0),
        ret::INVALID_PARAMETERS
    );
    assert_eq!(
        smc(&ps, &b, fid::CPU_ON, 2, 0x4000_0000, 0),
        ret::INVALID_ADDRESS
    );
    assert_eq!(smc(&ps, &b, fid::CPU_ON, 2, 0x8000_0010, 9), ret::SUCCESS);
    assert_eq!(
        smc(&ps, &b, fid::CPU_ON, 2, 0x8000_0010, 9),
        ret::ON_PENDING
    );
    assert_eq!(
        smc(&ps, &b, fid::AFFINITY_INFO, 2, 0, 0),
        ret::AFF_ON_PENDING
    );
    let c = ps.cpu(&b, 2);
    assert_eq!((c.entry, c.context), (0x8000_0010, 9));
    // The secondary publishes On (the board's warm path does this).
    ps.set_cpu(
        &b,
        2,
        Cpu {
            state: State::On,
            gicr: 0x2000,
            ..c
        },
    );
    assert_eq!(
        smc(&ps, &b, fid::CPU_ON, 2, 0x8000_0010, 9),
        ret::ALREADY_ON
    );
    assert_eq!(smc(&ps, &b, fid::AFFINITY_INFO, 2, 0, 0), ret::AFF_ON);
    assert_eq!(
        smc(&ps, &b, fid::AFFINITY_INFO, 2, 1, 0),
        ret::INVALID_PARAMETERS
    );
    // CPU_OFF from core 2: GIC first, then platform, barrier, then finish.
    b.events.borrow_mut().clear();
    b.core.set(2);
    assert_eq!(smc(&ps, &b, fid::CPU_OFF, 0, 0, 0), 0x77);
    assert_eq!(
        *b.events.borrow(),
        ["gic-off 0x2000", "off-prepare 2", "barrier", "off-finish"]
    );
    assert_eq!(smc(&ps, &b, fid::AFFINITY_INFO, 2, 0, 0), ret::AFF_OFF);
    // A core that is not On cannot turn itself off.
    assert_eq!(smc(&ps, &b, fid::CPU_OFF, 0, 0, 0), ret::DENIED);
}

#[test]
fn power_on_failures() {
    let b = Board::default();
    let ps = Psci::<4>::new();
    b.on_err.set(Some(PowerOnError::OffTimeout));
    assert_eq!(
        smc(&ps, &b, fid::CPU_ON, 1, 0x8000_0000, 0),
        ret::INTERNAL_FAILURE
    );
    assert_eq!(ps.cpu(&b, 1).state, State::Off);
    assert!(b
        .events
        .borrow()
        .iter()
        .any(|e| e == "log [main] PSCI CPU_ON cpu1: OffTimeout\n"));
    b.on_err.set(Some(PowerOnError::OnTimeout));
    assert_eq!(
        smc(&ps, &b, fid::CPU_ON, 1, 0x8000_0000, 0),
        ret::INTERNAL_FAILURE
    );
    assert_eq!(ps.cpu(&b, 1).state, State::OnPending);
    b.deny.set(true);
    assert_eq!(smc(&ps, &b, fid::CPU_ON, 3, 0x8000_0000, 0), ret::DENIED);
    assert_eq!(ps.prepare_cpu_off(&b, 1), Err(ret::DENIED));
    assert_eq!(
        smc(&ps, &b, fid::AFFINITY_INFO, 1, 0, 0),
        ret::AFF_ON_PENDING
    );
}

#[test]
fn system_suspend_features_and_sip() {
    let b = Board {
        a76: true,
        ..Board::default()
    };
    let ps = Psci::<4>::new();
    assert_eq!(smc(&ps, &b, fid::SYSTEM_OFF, 0, 0, 0), 0x55);
    assert_eq!(smc(&ps, &b, fid::SYSTEM_RESET, 0, 0, 0), 0x66);
    assert_eq!(
        smc(&ps, &b, fid::SYSTEM_OFF | fid::SMC64, 0, 0, 0),
        ret::NOT_SUPPORTED
    );
    assert_eq!(
        smc(&ps, &b, fid::CPU_SUSPEND, 0xffff_ffff_0000_0000, 0, 0),
        ret::SUCCESS
    );
    assert_eq!(
        smc(&ps, &b, fid::CPU_SUSPEND | fid::SMC64, 1 << 32, 0, 0),
        ret::SUCCESS
    );
    assert_eq!(
        smc(&ps, &b, fid::CPU_SUSPEND, 1, 0, 0),
        ret::INVALID_PARAMETERS
    );
    assert_eq!(
        b.events.borrow().iter().filter(|e| *e == "standby").count(),
        2
    );
    let arch = |q: u32| smc(&ps, &b, fid::SMCCC_ARCH_FEATURES, q as u64, 0, 0);
    assert_eq!(arch(fid::SMCCC_ARCH_WORKAROUND_3), ret::SUCCESS);
    assert_eq!(arch(fid::SMCCC_ARCH_WORKAROUND_1), ret::WA_UNAFFECTED);
    assert_eq!(arch(fid::SMCCC_ARCH_WORKAROUND_2), ret::NOT_REQUIRED);
    assert_eq!(arch(fid::PSCI_VERSION), ret::NOT_SUPPORTED);
    assert_eq!(psci_el3::features(fid::CPU_SUSPEND | fid::SMC64), 0);
    assert_eq!(psci_el3::features(fid::SYSTEM_RESET), ret::SUCCESS);
    assert_eq!(
        psci_el3::features(fid::SMCCC_ARCH_FEATURES),
        ret::NOT_SUPPORTED
    );
    assert_eq!(smc(&ps, &b, fid::MIGRATE_INFO_TYPE, 0, 0, 0), 2);
    assert_eq!(smc(&ps, &b, 0x8200_0010, 0, 0, 0), 42);
    assert_eq!(smc(&ps, &b, 0x8200_0011, 0, 0, 0), -1);
    assert_eq!(
        b.events.borrow().last().unwrap(),
        "log [main] unknown SMC function 0x82000011\n"
    );
}

#[test]
fn aff1_numbering() {
    use psci_el3::aff1_core_index as ix;
    assert_eq!(ix::<8>(0x700), Some(7));
    assert_eq!(ix::<8>(0x8000_0000 | 0x300), Some(3)); // MPIDR bit 31 (RES1) ignored
    assert_eq!(ix::<8>(0x800), None);
    assert_eq!(ix::<8>(0x101), None);
    assert_eq!(ix::<8>(0x1_0100), None);
    assert_eq!(ix::<8>(1 << 32 | 0x100), None);
    assert_eq!(ix::<4>(0x400), None);
}
