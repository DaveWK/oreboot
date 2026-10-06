//! PSCI 1.0 and SMCCC 1.1 services for the non-secure world.
//!
//! Cores are powered on and off through the A733 hotplug/PPU registers
//! ([`crate::soc::cpu_power_on`]); a powered-on core starts at `start` in
//! EL3 and is sent to its requested entry point by `secondary_main`.

use core::ptr::{addr_of, addr_of_mut, read_volatile, write_volatile};

use cubie_a7a_common::mem;

use crate::{soc, Context};

const SMCCC_VERSION: u32 = 0x8000_0000;
const SMCCC_ARCH_FEATURES: u32 = 0x8000_0001;
const PSCI_VERSION: u32 = 0x8400_0000;
const CPU_SUSPEND: u32 = 0x8400_0001;
const CPU_OFF: u32 = 0x8400_0002;
const CPU_ON: u32 = 0x8400_0003;
const AFFINITY_INFO: u32 = 0x8400_0004;
const MIGRATE_INFO_TYPE: u32 = 0x8400_0006;
const SYSTEM_OFF: u32 = 0x8400_0008;
const SYSTEM_RESET: u32 = 0x8400_0009;
const PSCI_FEATURES: u32 = 0x8400_000a;
/// SMC64 calling convention bit.
const SMC64: u32 = 0x4000_0000;

const SUCCESS: i64 = 0;
const NOT_SUPPORTED: i64 = -1;
const INVALID_PARAMETERS: i64 = -2;
const ALREADY_ON: i64 = -4;
const ON_PENDING: i64 = -5;
const INTERNAL_FAILURE: i64 = -6;
const INVALID_ADDRESS: i64 = -9;

const AFF_ON: i64 = 0;
const AFF_OFF: i64 = 1;
const AFF_ON_PENDING: i64 = 2;

pub const MAX_CPUS: usize = 8;

#[derive(Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum State {
    Off = 0,
    On = 1,
    OnPending = 2,
}

#[derive(Clone, Copy)]
#[repr(C)]
pub struct Cpu {
    pub state: State,
    pub entry: u64,
    pub context: u64,
    /// Redistributor frame, once known.
    pub gicr: usize,
}

const CPU_INIT: Cpu = Cpu {
    state: State::Off,
    entry: 0,
    context: 0,
    gicr: 0,
};

// The MMU and caches are off in EL3, so plain volatile accesses are
// coherent between cores; each field has a single writer at a time.
static mut CPUS: [Cpu; MAX_CPUS] = [CPU_INIT; MAX_CPUS];

pub fn cpu(idx: usize) -> Cpu {
    unsafe { read_volatile(addr_of!(CPUS[idx])) }
}

pub fn set_cpu(idx: usize, c: Cpu) {
    unsafe { write_volatile(addr_of_mut!(CPUS[idx]), c) }
}

/// Core index for an MPIDR affinity value as passed by the OS
/// (Aff1 = core number on the A733's single DynamIQ cluster).
pub fn core_index(mpidr: u64) -> Option<usize> {
    let aff0 = mpidr & 0xff;
    let aff1 = ((mpidr >> 8) & 0xff) as usize;
    let aff2 = (mpidr >> 16) & 0xff;
    let aff3 = (mpidr >> 32) & 0xff;
    (aff0 == 0 && aff2 == 0 && aff3 == 0 && aff1 < MAX_CPUS).then_some(aff1)
}

fn valid_ns_entry(addr: u64) -> bool {
    addr >= mem::PAYLOAD_RAM_BASE as u64 && addr % 4 == 0
}

fn cpu_on(target: u64, entry: u64, context: u64) -> i64 {
    let Some(idx) = core_index(target) else {
        return INVALID_PARAMETERS;
    };
    if !valid_ns_entry(entry) {
        return INVALID_ADDRESS;
    }
    let mut c = cpu(idx);
    match c.state {
        State::On => return ALREADY_ON,
        State::OnPending => return ON_PENDING,
        State::Off => {}
    }
    c.entry = entry;
    c.context = context;
    c.state = State::OnPending;
    set_cpu(idx, c);
    match soc::cpu_power_on(idx, crate::warm_entry_addr()) {
        Ok(()) => SUCCESS,
        Err(why) => {
            println!("[main] PSCI CPU_ON cpu{idx}: {why}");
            c.state = State::Off;
            set_cpu(idx, c);
            INTERNAL_FAILURE
        }
    }
}

fn affinity_info(target: u64, lowest_level: u64) -> i64 {
    if lowest_level != 0 {
        return INVALID_PARAMETERS;
    }
    match core_index(target).map(|i| cpu(i).state) {
        Some(State::On) => AFF_ON,
        Some(State::Off) => AFF_OFF,
        Some(State::OnPending) => AFF_ON_PENDING,
        None => INVALID_PARAMETERS,
    }
}

fn features(fid: u32) -> i64 {
    match fid & !SMC64 {
        PSCI_VERSION | CPU_OFF | CPU_ON | AFFINITY_INFO | MIGRATE_INFO_TYPE | SYSTEM_OFF
        | SYSTEM_RESET | PSCI_FEATURES => SUCCESS,
        SMCCC_VERSION => SUCCESS,
        _ => NOT_SUPPORTED,
    }
}

/// Handle an SMC from the non-secure world; results go to x0..x3.
pub fn handle_smc(ctx: &mut Context) {
    let fid = ctx.x[0] as u32;
    // SMC32 callers only own the low 32 bits of the arguments.
    let arg = |i: usize| {
        if fid & SMC64 != 0 {
            ctx.x[i]
        } else {
            ctx.x[i] & 0xffff_ffff
        }
    };
    let ret: i64 = match fid & !SMC64 {
        SMCCC_VERSION => 0x1_0001,
        SMCCC_ARCH_FEATURES => NOT_SUPPORTED,
        PSCI_VERSION => 0x1_0000,
        CPU_SUSPEND => NOT_SUPPORTED,
        CPU_OFF => crate::cpu_off(),
        CPU_ON => cpu_on(arg(1), arg(2), arg(3)),
        AFFINITY_INFO => affinity_info(arg(1), arg(2)),
        MIGRATE_INFO_TYPE => 2, // no Trusted OS
        SYSTEM_OFF => crate::system_off(),
        SYSTEM_RESET => crate::system_reset(),
        PSCI_FEATURES => features(arg(1) as u32),
        _ => {
            println!("[main] unknown SMC function 0x{fid:08x}");
            -1
        }
    };
    ctx.x[0] = ret as u64;
}
