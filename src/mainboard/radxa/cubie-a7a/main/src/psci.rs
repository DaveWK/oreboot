//! PSCI 1.0 and SMCCC 1.1 services for the non-secure world: the shared
//! `psci-el3` dispatcher and per-core state table with the A733 as its
//! [`Platform`].
//!
//! Cores are powered on and off through the A733 hotplug/PPU registers
//! ([`crate::soc::cpu_power_on`]); a powered-on core starts at `start` in
//! EL3 and is sent to its requested entry point by `secondary_main`. The
//! state records are guarded by the EL3 bakery lock ([`psci_lock`]).

use core::fmt;

use cubie_a7a_common::{bakery, fwlog, mem, recovery};
use psci_el3::{aff1_core_index, el3::Context, LockDenied, Platform, PowerOnError, Psci};

pub use psci_el3::{Cpu, State};

use crate::{psci_lock, soc};

pub const MAX_CPUS: usize = 8;

static PSCI: Psci<MAX_CPUS> = Psci::new();

/// The A733 under the PSCI dispatcher.
pub struct A733;

impl Platform for A733 {
    type Guard<'a> = bakery::Guard<'static, psci_lock::Uncached>;
    fn acquire(&self) -> Result<Self::Guard<'_>, LockDenied> {
        psci_lock::acquire().map_err(|_| LockDenied)
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
        crate::current_core()
    }
    /// Aff1 = core number on the A733's single DynamIQ cluster.
    fn core_index(&self, mpidr: u64) -> Option<usize> {
        aff1_core_index::<MAX_CPUS>(mpidr)
    }
    fn valid_ns_entry(&self, entry: u64) -> bool {
        // The handoff remains in EL3-owned RAM for the whole uptime.
        let h = psci_lock::handoff();
        mem::valid_payload_entry(h.dram_base, h.dram_size, entry)
    }
    fn cpu_power_on(&self, idx: usize) -> Result<(), PowerOnError> {
        soc::cpu_power_on(idx, crate::warm_entry_addr()).map_err(|e| match e {
            soc::PowerOnError::OffTimeout => PowerOnError::OffTimeout,
            soc::PowerOnError::OnTimeout => PowerOnError::OnTimeout,
        })
    }
    fn cpu_power_off_prepare(&self, idx: usize) {
        soc::cpu_power_off_prepare(idx)
    }
    fn gic_cpu_off(&self, gicr: usize) {
        crate::gic::cpu_off(gicr)
    }
    fn standby(&self) {
        psci_lock::standby()
    }
    fn cpu_off_finish(&self) -> i64 {
        crate::cpu_off_finish()
    }
    fn system_off(&self) -> i64 {
        crate::system_off()
    }
    fn system_reset(&self) -> i64 {
        crate::system_reset()
    }
    /// CVE-2022-23960: the A76 needs the branch history flushed, which the
    /// EL3 entry path does on every SMC.
    fn workaround_3_required(&self) -> bool {
        crate::is_cortex_a76()
    }
    fn sip(&self, fid: u32) -> Option<i64> {
        match fid {
            recovery::SIP_BOOT_OK => Some(crate::boot_confirmed()),
            fwlog::SIP_FWLOG_BASE => Some(fwlog::CUR_BASE as i64),
            _ => None,
        }
    }
    fn log(&self, args: fmt::Arguments) {
        print!("{args}");
    }
}

pub fn cpu(idx: usize) -> Cpu {
    PSCI.cpu(&A733, idx)
}

pub fn set_cpu(idx: usize, c: Cpu) {
    PSCI.set_cpu(&A733, idx, c)
}

/// Platform preparation completes before Off is published (the dispatcher's
/// CPU_OFF path; also used by the host controls, tests/psci-host.rs.in).
#[allow(dead_code)]
pub fn prepare_cpu_off(idx: usize) -> Result<(), i64> {
    PSCI.prepare_cpu_off(&A733, idx)
}

/// Handle an SMC from the non-secure world; results go to x0..x3.
pub fn handle_smc(ctx: &mut Context) {
    PSCI.handle_smc(&A733, &mut ctx.x[..4])
}
