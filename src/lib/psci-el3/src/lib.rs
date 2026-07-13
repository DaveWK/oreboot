// SPDX-License-Identifier: GPL-2.0-only
//! PSCI 1.0 and SMCCC 1.1 services for the non-secure world, at EL3.
//!
//! From ore-edk-boot-radxa-a7a `main/src/psci.rs` (A733). The function
//! dispatch (SMC32/SMC64 forms and argument widths), the per-core state
//! table (Off, On, OnPending with entry, context and GIC redistributor),
//! CPU_ON/CPU_OFF/AFFINITY_INFO/CPU_SUSPEND standby, the feature queries and
//! the SMCCC workaround answers are here, in the board's order and with its
//! return codes. The [`Platform`] supplies the rest: the lock that guards the
//! state records (a bakery lock, `crates/bakery`, on boards without coherent
//! atomics at EL3), ordered access to the state words, the calling core and
//! the MPIDR mapping, which entry points are valid, the SoC power-on and
//! power-down hooks (the A733 hotplug/PPU registers), the GIC, standby, system
//! off and reset, SiP calls, and logging.
//!
//! Around it: [`el3`] (EL3/EL2 register values, the lower-EL context, trap
//! classification, per-core EL3 set-up with the CPU errata as data, the drop
//! to EL2 and, with feature `vectors`, the EL3 vector table) and [`gicv3`]
//! (GICv3/GIC-600 secure set-up over `regio`), both from the A733 main stage.
#![no_std]

pub mod el3;
pub mod gicv3;

use core::cell::UnsafeCell;
use core::fmt;
use core::ptr::{addr_of, addr_of_mut, read_volatile, write_volatile};

/// Function IDs (SMC32 form; SMC64 adds [`fid::SMC64`]).
pub mod fid {
    pub const SMCCC_VERSION: u32 = 0x8000_0000;
    pub const SMCCC_ARCH_FEATURES: u32 = 0x8000_0001;
    pub const SMCCC_ARCH_WORKAROUND_1: u32 = 0x8000_8000;
    pub const SMCCC_ARCH_WORKAROUND_2: u32 = 0x8000_7fff;
    pub const SMCCC_ARCH_WORKAROUND_3: u32 = 0x8000_3fff;
    pub const PSCI_VERSION: u32 = 0x8400_0000;
    pub const CPU_SUSPEND: u32 = 0x8400_0001;
    pub const CPU_OFF: u32 = 0x8400_0002;
    pub const CPU_ON: u32 = 0x8400_0003;
    pub const AFFINITY_INFO: u32 = 0x8400_0004;
    pub const MIGRATE_INFO_TYPE: u32 = 0x8400_0006;
    pub const SYSTEM_OFF: u32 = 0x8400_0008;
    pub const SYSTEM_RESET: u32 = 0x8400_0009;
    pub const PSCI_FEATURES: u32 = 0x8400_000a;
    /// SMC64 calling convention bit.
    pub const SMC64: u32 = 0x4000_0000;
}
use fid::*;

/// PSCI / SMCCC return codes.
pub mod ret {
    pub const SUCCESS: i64 = 0;
    pub const NOT_SUPPORTED: i64 = -1;
    pub const NOT_REQUIRED: i64 = -2;
    /// SMCCC_ARCH_FEATURES for a workaround: implemented, not needed here.
    pub const WA_UNAFFECTED: i64 = 1;
    pub const INVALID_PARAMETERS: i64 = -2;
    pub const DENIED: i64 = -3;
    pub const ALREADY_ON: i64 = -4;
    pub const ON_PENDING: i64 = -5;
    pub const INTERNAL_FAILURE: i64 = -6;
    pub const INVALID_ADDRESS: i64 = -9;
    pub const AFF_ON: i64 = 0;
    pub const AFF_OFF: i64 = 1;
    pub const AFF_ON_PENDING: i64 = 2;
}
use ret::*;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u32)]
pub enum State {
    Off = 0,
    On = 1,
    OnPending = 2,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(C)]
pub struct Cpu {
    pub state: State,
    pub entry: u64,
    pub context: u64,
    /// GIC redistributor frame, once known.
    pub gicr: usize,
}

pub const CPU_INIT: Cpu = Cpu {
    state: State::Off,
    entry: 0,
    context: 0,
    gicr: 0,
};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PowerOnError {
    /// The core never reached its powered-off state: nothing was requested.
    OffTimeout,
    /// The power-on request timed out: the core may still come up.
    OnTimeout,
}

/// The platform lock could not be taken; the call answers DENIED.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LockDenied;

/// The SoC and the EL3 environment.
pub trait Platform {
    type Guard<'a>
    where
        Self: 'a;
    /// The lock over the CPU records (the board's bakery lock).
    fn acquire(&self) -> Result<Self::Guard<'_>, LockDenied>;
    /// Ordered, aligned 32-bit load of EL3-owned memory.
    ///
    /// # Safety
    /// `p` is an aligned state word of this crate's table.
    unsafe fn load32(&self, p: *const u32) -> u32;
    /// Ordered, aligned 32-bit store; writers are serialised by the lock.
    ///
    /// # Safety
    /// As for [`Platform::load32`].
    unsafe fn store32(&self, p: *mut u32, v: u32);
    fn barrier(&self);
    fn current_core(&self) -> usize;
    /// Table index for an MPIDR affinity value from the OS
    /// ([`aff1_core_index`] for a single cluster numbered by Aff1).
    fn core_index(&self, mpidr: u64) -> Option<usize>;
    /// A non-secure entry point the OS may pass to CPU_ON.
    fn valid_ns_entry(&self, entry: u64) -> bool;
    /// Power core `idx` on at the warm entry.
    fn cpu_power_on(&self, idx: usize) -> Result<(), PowerOnError>;
    /// Platform preparation before core `idx` is published Off.
    fn cpu_power_off_prepare(&self, idx: usize);
    fn gic_cpu_off(&self, gicr: usize);
    fn standby(&self);
    /// After `prepare_cpu_off` succeeded for the calling core: power it
    /// down (does not return on hardware).
    fn cpu_off_finish(&self) -> i64;
    fn system_off(&self) -> i64;
    fn system_reset(&self) -> i64;
    /// CVE-2022-23960 (SMCCC_ARCH_WORKAROUND_3) needed on the calling core
    /// (its EL3 entry path flushes the branch history on every SMC).
    fn workaround_3_required(&self) -> bool;
    /// A board SiP call, or None for an unknown function.
    fn sip(&self, fid: u32) -> Option<i64>;
    fn log(&self, args: fmt::Arguments);
}

/// Core index from an MPIDR on a single cluster numbered by Aff1 (A733).
pub fn aff1_core_index<const N: usize>(mpidr: u64) -> Option<usize> {
    let aff0 = mpidr & 0xff;
    let aff1 = ((mpidr >> 8) & 0xff) as usize;
    let aff2 = (mpidr >> 16) & 0xff;
    let aff3 = (mpidr >> 32) & 0xff;
    (aff0 == 0 && aff2 == 0 && aff3 == 0 && aff1 < N).then_some(aff1)
}

/// The per-core state of `N` cores. Whole records are read and written only
/// under the platform lock; the state word also has ordered scalar access,
/// so AFFINITY_INFO reads exactly one published state without the lock.
pub struct Psci<const N: usize> {
    cpus: UnsafeCell<[Cpu; N]>,
}
// SAFETY: every access goes through the platform's lock or its ordered
// scalar state-word access (see the type's documentation).
unsafe impl<const N: usize> Sync for Psci<N> {}

impl<const N: usize> Default for Psci<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> Psci<N> {
    pub const fn new() -> Self {
        Self {
            cpus: UnsafeCell::new([CPU_INIT; N]),
        }
    }
    fn rec(&self, idx: usize) -> *mut Cpu {
        assert!(idx < N);
        // SAFETY: in bounds; no reference to the record is formed.
        unsafe { (self.cpus.get() as *mut Cpu).add(idx) }
    }
    fn state<P: Platform>(&self, p: &P, idx: usize) -> State {
        // SAFETY: the record's aligned state word.
        match unsafe { p.load32(addr_of!((*self.rec(idx)).state).cast()) } {
            0 => State::Off,
            1 => State::On,
            2 => State::OnPending,
            _ => panic!("invalid EL3 CPU state"),
        }
    }
    // Requires a live guard; do not expose an unlocked aggregate access.
    fn read_cpu<P: Platform>(&self, p: &P, idx: usize) -> Cpu {
        let r = self.rec(idx);
        // SAFETY: under the lock; fields of an in-bounds record.
        unsafe {
            Cpu {
                state: self.state(p, idx),
                entry: read_volatile(addr_of!((*r).entry)),
                context: read_volatile(addr_of!((*r).context)),
                gicr: read_volatile(addr_of!((*r).gicr)),
            }
        }
    }
    fn write_cpu<P: Platform>(&self, p: &P, idx: usize, c: Cpu) {
        let r = self.rec(idx);
        // SAFETY: under the lock; the state word is published last.
        unsafe {
            write_volatile(addr_of_mut!((*r).entry), c.entry);
            write_volatile(addr_of_mut!((*r).context), c.context);
            write_volatile(addr_of_mut!((*r).gicr), c.gicr);
            p.store32(addr_of_mut!((*r).state).cast(), c.state as u32);
        }
    }
    pub fn cpu<P: Platform>(&self, p: &P, idx: usize) -> Cpu {
        let _guard = p.acquire().expect("EL3 CPU-state lock unavailable");
        self.read_cpu(p, idx)
    }
    pub fn set_cpu<P: Platform>(&self, p: &P, idx: usize, c: Cpu) {
        let _guard = p.acquire().expect("EL3 CPU-state lock unavailable");
        self.write_cpu(p, idx, c);
    }

    /// Platform preparation completes before Off is published. CPU_ON then
    /// waits for the core to be off before issuing a request. The caller
    /// releases nothing before its final WFI: the guard is dropped here.
    pub fn prepare_cpu_off<P: Platform>(&self, p: &P, idx: usize) -> Result<(), i64> {
        let _guard = p.acquire().map_err(|_| DENIED)?;
        let mut c = self.read_cpu(p, idx);
        if c.state != State::On {
            return Err(DENIED);
        }
        if c.gicr != 0 {
            p.gic_cpu_off(c.gicr);
        }
        p.cpu_power_off_prepare(idx);
        p.barrier();
        c.state = State::Off;
        self.write_cpu(p, idx, c);
        Ok(())
    }

    fn cpu_on<P: Platform>(&self, p: &P, target: u64, entry: u64, context: u64) -> i64 {
        let Some(idx) = p.core_index(target) else {
            return INVALID_PARAMETERS;
        };
        if !p.valid_ns_entry(entry) {
            return INVALID_ADDRESS;
        }
        let Ok(_guard) = p.acquire() else {
            return DENIED;
        };
        let mut c = self.read_cpu(p, idx);
        match c.state {
            State::On => return ALREADY_ON,
            State::OnPending => return ON_PENDING,
            State::Off => {}
        }
        c.entry = entry;
        c.context = context;
        c.state = State::OnPending;
        self.write_cpu(p, idx, c);
        match p.cpu_power_on(idx) {
            Ok(()) => SUCCESS,
            Err(why) => {
                p.log(format_args!("[main] PSCI CPU_ON cpu{idx}: {why:?}\n"));
                if why == PowerOnError::OffTimeout {
                    c.state = State::Off;
                    self.write_cpu(p, idx, c);
                }
                // ON timeout is ambiguous: retain Pending so a late secondary
                // can complete, and no second caller can reprogram the core.
                INTERNAL_FAILURE
            }
        }
    }

    fn affinity_info<P: Platform>(&self, p: &P, target: u64, lowest_level: u64) -> i64 {
        if lowest_level != 0 {
            return INVALID_PARAMETERS;
        }
        match p.core_index(target).map(|i| self.state(p, i)) {
            Some(State::On) => AFF_ON,
            Some(State::Off) => AFF_OFF,
            Some(State::OnPending) => AFF_ON_PENDING,
            None => INVALID_PARAMETERS,
        }
    }

    /// Handle an SMC from the non-secure world; `x` holds x0..x3 (at least
    /// four registers) and gets the result in x0.
    pub fn handle_smc<P: Platform>(&self, p: &P, x: &mut [u64]) {
        let fid = x[0] as u32;
        // SMC32 callers only own the low 32 bits of the arguments.
        let arg = |i: usize| {
            if fid & SMC64 != 0 {
                x[i]
            } else {
                x[i] & 0xffff_ffff
            }
        };
        let Some(base) = canonical(fid) else {
            x[0] = NOT_SUPPORTED as u64;
            return;
        };
        let ret: i64 = match base {
            SMCCC_VERSION => 0x1_0001,
            SMCCC_ARCH_FEATURES => arch_features(p, arg(1) as u32),
            // The BHB flush already ran on entry; nothing else to do.
            SMCCC_ARCH_WORKAROUND_1 | SMCCC_ARCH_WORKAROUND_3 => SUCCESS,
            PSCI_VERSION => 0x1_0000,
            CPU_SUSPEND => cpu_suspend(p, arg(1) as u32 as u64),
            CPU_OFF => match self.prepare_cpu_off(p, p.current_core()) {
                Err(code) => code,
                Ok(()) => p.cpu_off_finish(),
            },
            CPU_ON => self.cpu_on(p, arg(1), arg(2), arg(3)),
            AFFINITY_INFO => self.affinity_info(p, arg(1), arg(2) as u32 as u64),
            MIGRATE_INFO_TYPE => 2, // no Trusted OS
            SYSTEM_OFF => p.system_off(),
            SYSTEM_RESET => p.system_reset(),
            PSCI_FEATURES => features(arg(1) as u32),
            other => match p.sip(other) {
                Some(r) => r,
                None => {
                    p.log(format_args!("[main] unknown SMC function 0x{fid:08x}\n"));
                    -1
                }
            },
        };
        x[0] = ret as u64;
    }
}

/// Only original-format, level-zero, state-ID-zero standby is implemented;
/// power-down states need a real save/restore and are rejected.
fn cpu_suspend<P: Platform>(p: &P, power_state: u64) -> i64 {
    if power_state != 0 {
        return INVALID_PARAMETERS;
    }
    p.standby();
    SUCCESS
}

/// SMCCC_ARCH_FEATURES, answered per calling core as TF-A does.
pub fn arch_features<P: Platform>(p: &P, fid: u32) -> i64 {
    match fid {
        SMCCC_VERSION | SMCCC_ARCH_FEATURES => SUCCESS,
        // CVE-2017-5715: the board's cores are not affected.
        SMCCC_ARCH_WORKAROUND_1 => WA_UNAFFECTED,
        // CVE-2018-3639: not required (SSBS or unaffected).
        SMCCC_ARCH_WORKAROUND_2 => NOT_REQUIRED,
        SMCCC_ARCH_WORKAROUND_3 if p.workaround_3_required() => SUCCESS,
        SMCCC_ARCH_WORKAROUND_3 => WA_UNAFFECTED,
        _ => NOT_SUPPORTED,
    }
}

/// The SMC32 form of a function ID; SMC64 only for the functions with
/// 64-bit arguments.
pub fn canonical(fid: u32) -> Option<u32> {
    if fid & SMC64 == 0 {
        return Some(fid);
    }
    let base = fid & !SMC64;
    matches!(base, CPU_SUSPEND | CPU_ON | AFFINITY_INFO).then_some(base)
}

/// PSCI_FEATURES.
pub fn features(fid: u32) -> i64 {
    let Some(base) = canonical(fid) else {
        return NOT_SUPPORTED;
    };
    match base {
        // bit 1 clear: original power_state format
        CPU_SUSPEND => 0,
        PSCI_VERSION | CPU_OFF | CPU_ON | AFFINITY_INFO | MIGRATE_INFO_TYPE | SYSTEM_OFF
        | SYSTEM_RESET | PSCI_FEATURES => SUCCESS,
        SMCCC_VERSION => SUCCESS,
        _ => NOT_SUPPORTED,
    }
}
