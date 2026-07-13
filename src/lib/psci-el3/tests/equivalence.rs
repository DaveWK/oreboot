// SPDX-License-Identifier: GPL-2.0-only
//! The board's own PSCI host harness (ore-edk-boot-radxa-a7a ed42550
//! tests/psci-host.rs.in + scripts/test-psci.py, nine cases), run against
//! the verbatim board psci.rs AND the crate with an A733 Platform on the
//! same modelled platform effects (bakery lock over eight slots, SoC power
//! hooks, GIC, standby). Every case asserts the board's expected results;
//! the per-case return sequences of both runs are also compared.
#![allow(static_mut_refs, dead_code, clippy::all)]

use std::cell::Cell;
use std::sync::atomic::{AtomicU32, AtomicU64, AtomicU8, AtomicUsize, Ordering::SeqCst};
use std::sync::{Arc, Barrier, Mutex};
use std::thread;

extern crate self as cubie_a7a_common;
pub use ::bakery;
#[path = "legacy/a7a_mem.rs"]
mod mem_file;
pub use mem_file::*;
#[derive(Clone, Copy)]
pub struct Handoff {
    pub dram_base: u64,
    pub dram_size: u64,
}
pub mod recovery {
    pub const SIP_BOOT_OK: u32 = 0x82000001;
}
pub mod fwlog {
    pub const SIP_FWLOG_BASE: u32 = 0x82000003;
    pub const CUR_BASE: usize = 0x40140000;
}
pub struct Context {
    pub x: [u64; 31],
}
thread_local! { static CORE: Cell<usize> = const { Cell::new(0) }; }
fn current_core() -> usize {
    CORE.with(Cell::get)
}
fn warm_entry_addr() -> usize {
    0x41000000
}
fn is_cortex_a76() -> bool {
    false
}
fn boot_confirmed() -> i64 {
    11
}
fn system_off() -> i64 {
    12
}
fn system_reset() -> i64 {
    13
}
fn cpu_off() -> i64 {
    psci::prepare_cpu_off(current_core()).err().unwrap_or(0)
}
pub mod psci_lock {
    use super::*;
    pub struct Memory;
    pub static ACCESS: Memory = Memory;
    pub static SLOTS: [AtomicU32; 8] = [const { AtomicU32::new(0) }; 8];
    static TICKS: AtomicU64 = AtomicU64::new(0);
    pub static FAIL: AtomicU32 = AtomicU32::new(0);
    pub static STANDBY: AtomicU32 = AtomicU32::new(0);
    impl bakery::Access for Memory {
        fn participants(&self) -> usize {
            8
        }
        fn load(&self, i: usize) -> u32 {
            SLOTS[i].load(SeqCst)
        }
        fn store(&self, i: usize, v: u32) {
            SLOTS[i].store(v, SeqCst);
        }
        fn barrier(&self) {
            barrier();
        }
        fn ticks(&self) -> u64 {
            TICKS.fetch_add(1, SeqCst)
        }
        fn relax(&self) {
            thread::yield_now();
        }
    }
    pub fn acquire() -> Result<bakery::Guard<'static, Memory>, ()> {
        if FAIL.load(SeqCst) != 0 {
            return Err(());
        }
        bakery::acquire(&ACCESS, current_core(), 100_000_000).map_err(|_| ())
    }
    pub fn barrier() {
        std::sync::atomic::fence(SeqCst);
    }
    pub unsafe fn load32(p: *const u32) -> u32 {
        AtomicU32::from_ptr(p.cast_mut()).load(SeqCst)
    }
    pub unsafe fn store32(p: *mut u32, v: u32) {
        AtomicU32::from_ptr(p).store(v, SeqCst);
    }
    pub fn handoff() -> Handoff {
        Handoff {
            dram_base: 0x40000000,
            dram_size: 4 << 30,
        }
    }
    pub fn standby() {
        STANDBY.fetch_add(1, SeqCst);
    }
    pub fn released() {
        assert!(SLOTS.iter().all(|x| x.load(SeqCst) == 0));
    }
}
pub mod soc {
    use super::*;
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    pub enum PowerOnError {
        OffTimeout,
        OnTimeout,
    }
    pub static CALLS: AtomicUsize = AtomicUsize::new(0);
    pub static FAIL: AtomicUsize = AtomicUsize::new(0);
    pub static OFFS: AtomicUsize = AtomicUsize::new(0);
    pub fn cpu_power_on(n: usize, entry: usize) -> Result<(), PowerOnError> {
        assert_eq!(entry, 0x41000000);
        assert_eq!(call(0xc4000004, n as u64 * 256, 0, 0), 2);
        CALLS.fetch_add(1, SeqCst);
        for _ in 0..200 {
            thread::yield_now();
        }
        match FAIL.load(SeqCst) {
            1 => Err(PowerOnError::OffTimeout),
            2 => Err(PowerOnError::OnTimeout),
            _ => Ok(()),
        }
    }
    pub fn cpu_power_off_prepare(n: usize) {
        // Production must still publish On until all platform preparation finishes.
        assert_eq!(call(0xc4000004, n as u64 * 256, 0, 0), 0);
        assert_eq!(gic::OFFS.load(SeqCst), 1);
        OFFS.fetch_add(1, SeqCst);
    }
}
pub mod gic {
    use super::*;
    pub static OFFS: AtomicUsize = AtomicUsize::new(0);
    pub fn cpu_off(rd: usize) {
        assert_eq!(rd, 0x3400000);
        OFFS.fetch_add(1, SeqCst);
    }
}
#[path = "legacy/a7a_psci.rs"]
pub mod psci;

// ---- the crate on the same platform effects ----------------------------------

/// The A733 Platform a board would write, on the modelled effects.
pub struct A733;
impl psci_el3::Platform for A733 {
    type Guard<'a> = bakery::Guard<'static, psci_lock::Memory>;
    fn acquire(&self) -> Result<Self::Guard<'_>, psci_el3::LockDenied> {
        psci_lock::acquire().map_err(|()| psci_el3::LockDenied)
    }
    unsafe fn load32(&self, p: *const u32) -> u32 {
        psci_lock::load32(p)
    }
    unsafe fn store32(&self, p: *mut u32, v: u32) {
        psci_lock::store32(p, v)
    }
    fn barrier(&self) {
        psci_lock::barrier()
    }
    fn current_core(&self) -> usize {
        current_core()
    }
    fn core_index(&self, mpidr: u64) -> Option<usize> {
        psci_el3::aff1_core_index::<8>(mpidr)
    }
    fn valid_ns_entry(&self, entry: u64) -> bool {
        let h = psci_lock::handoff();
        mem::valid_payload_entry(h.dram_base, h.dram_size, entry)
    }
    fn cpu_power_on(&self, idx: usize) -> Result<(), psci_el3::PowerOnError> {
        soc::cpu_power_on(idx, warm_entry_addr()).map_err(|e| match e {
            soc::PowerOnError::OffTimeout => psci_el3::PowerOnError::OffTimeout,
            soc::PowerOnError::OnTimeout => psci_el3::PowerOnError::OnTimeout,
        })
    }
    fn cpu_power_off_prepare(&self, idx: usize) {
        soc::cpu_power_off_prepare(idx)
    }
    fn gic_cpu_off(&self, gicr: usize) {
        gic::cpu_off(gicr)
    }
    fn standby(&self) {
        psci_lock::standby()
    }
    fn cpu_off_finish(&self) -> i64 {
        0 // the board's final WFI; the harness returns success
    }
    fn system_off(&self) -> i64 {
        system_off()
    }
    fn system_reset(&self) -> i64 {
        system_reset()
    }
    fn workaround_3_required(&self) -> bool {
        is_cortex_a76()
    }
    fn sip(&self, fid: u32) -> Option<i64> {
        match fid {
            recovery::SIP_BOOT_OK => Some(boot_confirmed()),
            fwlog::SIP_FWLOG_BASE => Some(fwlog::CUR_BASE as i64),
            _ => None,
        }
    }
    fn log(&self, _: std::fmt::Arguments) {}
}

/// Which implementation `call` (and so the SoC shim's nested checks) uses.
static MODE: AtomicU8 = AtomicU8::new(0);
static mut CRATE: Option<&'static psci_el3::Psci<8>> = None;
static LOG: Mutex<Vec<(String, i64)>> = Mutex::new(Vec::new());

fn call(fid: u32, x1: u64, x2: u64, x3: u64) -> i64 {
    let mut c = Context { x: [0; 31] };
    c.x[..4].copy_from_slice(&[fid as u64, x1, x2, x3]);
    if MODE.load(SeqCst) == 0 {
        psci::handle_smc(&mut c);
    } else {
        unsafe { CRATE.unwrap() }.handle_smc(&A733, &mut c.x);
    }
    c.x[0] as i64
}
/// (state, entry, context, gicr) of a core.
fn cpu(i: usize) -> (u32, u64, u64, usize) {
    if MODE.load(SeqCst) == 0 {
        let c = psci::cpu(i);
        (c.state as u32, c.entry, c.context, c.gicr)
    } else {
        let c = unsafe { CRATE.unwrap() }.cpu(&A733, i);
        (c.state as u32, c.entry, c.context, c.gicr)
    }
}
fn set_cpu(i: usize, (state, entry, context, gicr): (u32, u64, u64, usize)) {
    if MODE.load(SeqCst) == 0 {
        let state = [psci::State::Off, psci::State::On, psci::State::OnPending][state as usize];
        psci::set_cpu(
            i,
            psci::Cpu {
                state,
                entry,
                context,
                gicr,
            },
        );
    } else {
        let state = [
            psci_el3::State::Off,
            psci_el3::State::On,
            psci_el3::State::OnPending,
        ][state as usize];
        unsafe { CRATE.unwrap() }.set_cpu(
            &A733,
            i,
            psci_el3::Cpu {
                state,
                entry,
                context,
                gicr,
            },
        );
    }
}
fn prepare_cpu_off(i: usize) -> Result<(), i64> {
    if MODE.load(SeqCst) == 0 {
        psci::prepare_cpu_off(i)
    } else {
        unsafe { CRATE.unwrap() }.prepare_cpu_off(&A733, i)
    }
}
/// Record a result for the cross-run comparison and return it.
fn r(what: &str, v: i64) -> i64 {
    LOG.lock().unwrap().push((what.to_string(), v));
    v
}

fn reset(mode: u8) {
    MODE.store(mode, SeqCst);
    for s in &psci_lock::SLOTS {
        s.store(0, SeqCst);
    }
    psci_lock::FAIL.store(0, SeqCst);
    psci_lock::STANDBY.store(0, SeqCst);
    soc::CALLS.store(0, SeqCst);
    soc::FAIL.store(0, SeqCst);
    soc::OFFS.store(0, SeqCst);
    gic::OFFS.store(0, SeqCst);
    CORE.with(|c| c.set(0));
    if mode == 0 {
        for i in 0..8 {
            set_cpu(i, (0, 0, 0, 0)); // the board's table is a static: clear it
        }
    } else {
        unsafe { CRATE = Some(Box::leak(Box::new(psci_el3::Psci::new()))) };
    }
}

/// One case of the board's harness, with its assertions.
fn case(name: &str) {
    match name {
        "concurrent" => {
            let gate = Arc::new(Barrier::new(7));
            let ts: Vec<_> = (0..7)
                .map(|i| {
                    let g = gate.clone();
                    let mode = MODE.load(SeqCst);
                    thread::spawn(move || {
                        MODE.store(mode, SeqCst);
                        CORE.with(|c| c.set(i));
                        g.wait();
                        call(
                            0xc4000003,
                            0x700,
                            0x40200000 + i as u64 * 4,
                            0x12340000 + i as u64,
                        )
                    })
                })
                .collect();
            let rs: Vec<_> = ts.into_iter().map(|t| t.join().unwrap()).collect();
            assert_eq!(rs.iter().filter(|&&r| r == 0).count(), 1);
            assert_eq!(rs.iter().filter(|&&r| r == -5).count(), 6);
            assert_eq!(soc::CALLS.load(SeqCst), 1);
            let c = cpu(7);
            assert!(c.0 == 2);
            assert_eq!((c.1 - 0x40200000) / 4, c.2 - 0x12340000);
            r(
                "concurrent-ok",
                rs.iter().filter(|&&r| r == 0).count() as i64,
            );
        }
        "off-timeout" | "on-timeout" => {
            let fail = if name == "off-timeout" { 1 } else { 2 };
            soc::FAIL.store(fail, SeqCst);
            assert_eq!(r("on", call(0xc4000003, 0x100, 0x40200000, 77)), -6);
            assert_eq!(
                r("aff", call(0xc4000004, 0x100, 0, 0)),
                if fail == 1 { 1 } else { 2 }
            );
            soc::FAIL.store(0, SeqCst);
            assert_eq!(
                r("on2", call(0xc4000003, 0x100, 0x40200004, 88)),
                if fail == 1 { 0 } else { -5 }
            );
            assert_eq!(soc::CALLS.load(SeqCst), if fail == 1 { 2 } else { 1 });
            if fail == 2 {
                CORE.with(|c| c.set(1));
                let mut c = cpu(1);
                assert_eq!(c.1, 0x40200000);
                assert_eq!(c.2, 77);
                c.0 = 1;
                set_cpu(1, c);
                assert_eq!(r("aff2", call(0xc4000004, 0x100, 0, 0)), 0);
            }
        }
        "off-order" => {
            set_cpu(0, (1, 0, 0, 0x3400000));
            prepare_cpu_off(0).unwrap();
            assert_eq!(soc::OFFS.load(SeqCst), 1);
            assert_eq!(r("aff", call(0xc4000004, 0, 0, 0)), 1);
            psci_lock::released();
            assert_eq!(r("on", call(0xc4000003, 0, 0x40200000, 3)), 0);
        }
        "denied" => {
            set_cpu(0, (1, 0, 0, 0x3400000));
            psci_lock::FAIL.store(1, SeqCst);
            assert_eq!(r("on", call(0xc4000003, 0x100, 0x40200000, 0)), -3);
            assert_eq!(prepare_cpu_off(0), Err(-3));
            assert_eq!(soc::CALLS.load(SeqCst), 0);
            assert_eq!(soc::OFFS.load(SeqCst), 0);
            assert_eq!(gic::OFFS.load(SeqCst), 0);
            assert_eq!(r("aff", call(0xc4000004, 0, 0, 0)), 0); // observer never takes lock
            psci_lock::FAIL.store(0, SeqCst);
        }
        "standby" => {
            assert_eq!(
                r(
                    "s0",
                    call(0x84000001, 0, 0xffffffffffffffff, 0xffffffffffffffff)
                ),
                0
            );
            assert_eq!(r("s1", call(0xc4000001, 0, 3, 4)), 0);
            for state in [1, 0x10000, 0x1010000, 0x1000000, 0x20000, 0x80000000] {
                assert_eq!(r("s", call(0xc4000001, state, 0x40200000, 1)), -2);
            }
            // These arguments are uint32_t even in the SMC64 calling convention.
            assert_eq!(r("s2", call(0xc4000001, 0xdead000000000000, 3, 4)), 0);
            assert_eq!(r("aff", call(0xc4000004, 0x100, 0xbeef000000000000, 0)), 1);
            assert_eq!(psci_lock::STANDBY.load(SeqCst), 3);
        }
        "smc64" => {
            for fid in [
                0x80000000u32,
                0x80000001,
                0x84000000,
                0x84000002,
                0x84000006,
                0x84000008,
                0x84000009,
                0x8400000a,
                0x82000002,
                0x82000003,
            ] {
                assert_eq!(r("f64", call(fid | 0x40000000, 0, 0, 0)), -1);
                assert_eq!(
                    r("feat", call(0x8400000a, (fid | 0x40000000) as u64, 0, 0)),
                    -1
                );
            }
            for fid in [0xc4000001u64, 0xc4000003, 0xc4000004] {
                assert_eq!(r("feat", call(0x8400000a, fid, 0, 0)), 0);
            }
            assert_eq!(r("ver", call(0x84000000, 0, 0, 0)), 0x10000);
            assert_eq!(r("smccc", call(0x80000000, 0, 0, 0)), 0x10001);
        }
        "smc32" => {
            assert_eq!(
                r(
                    "on",
                    call(
                        0x84000003,
                        0xdead000000000100,
                        0xbeef000040200000,
                        0xabcd000012345678
                    )
                ),
                0
            );
            let c = cpu(1);
            assert_eq!(c.1, 0x40200000);
            assert_eq!(c.2, 0x12345678);
            assert_eq!(
                r(
                    "aff",
                    call(0x84000004, 0xdead000000000100, 0xbeef000000000000, 0)
                ),
                2
            );
        }
        "bounds" => {
            for target in [1, 0x800, 0x10000, 1 << 32] {
                assert_eq!(r("tgt", call(0xc4000003, target, 0x40200000, 0)), -2);
            }
            for entry in [0, 0x401ffffc, 0x40200001, 0x140000000, 0xfffffffffffffffc] {
                assert_eq!(r("ent", call(0xc4000003, 0x100, entry, 0)), -9);
            }
            assert_eq!(r("aff", call(0xc4000004, 0x100, 1, 0)), -2);
            assert_eq!(soc::CALLS.load(SeqCst), 0);
        }
        _ => panic!("unknown case"),
    }
    psci_lock::released();
}

/// Beyond the board's cases: every function ID's answer, and the SiP and
/// unknown-function paths, through both implementations.
fn sweep() {
    let mut fids: Vec<u32> = vec![0x82000001, 0x82000003, 0x82000005, 0x86000000, 0xc2000001];
    for base in [0x8000_0000u32, 0x8400_0000] {
        for f in 0..0x20 {
            fids.push(base + f);
            fids.push((base + f) | 0x4000_0000);
        }
    }
    for f in [0x8000_8000u32, 0x8000_7fff, 0x8000_3fff, 0x8000_1234] {
        fids.push(f);
    }
    for fid in fids {
        if matches!(fid & !0x4000_0000, 0x8400_0002 | 0x8400_0001 | 0x8400_0003) {
            continue; // state-changing; covered by the cases
        }
        r("fid", call(fid, 0, 0, 0));
        for q in [
            0x8000_0000u32,
            0x8000_0001,
            0x8000_8000,
            0x8000_7fff,
            0x8000_3fff,
            0x8000_0002,
        ] {
            r("arch", call(0x8000_0001, q as u64, 0, 0));
        }
        r("feat", call(0x8400_000a, fid as u64, 0, 0));
    }
}

#[test]
fn the_board_harness_passes_on_both_and_they_agree() {
    let cases = [
        "concurrent",
        "off-timeout",
        "on-timeout",
        "off-order",
        "denied",
        "standby",
        "smc64",
        "smc32",
        "bounds",
    ];
    let mut runs: Vec<Vec<(String, i64)>> = Vec::new();
    for mode in [0u8, 1] {
        LOG.lock().unwrap().clear();
        for c in cases {
            reset(mode);
            case(c);
        }
        reset(mode);
        sweep();
        runs.push(std::mem::take(&mut *LOG.lock().unwrap()));
    }
    assert!(runs[0].len() > 300);
    assert_eq!(runs[0], runs[1]);
}
