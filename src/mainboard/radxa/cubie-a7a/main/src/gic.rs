//! GICv3 (GIC-600) set-up at EL3 on the shared `psci-el3` gicv3 code: the
//! distributor once, each CPU's redistributor and EL3 CPU interface, and
//! quiescing a CPU before PSCI CPU_OFF. The A733's GIC is the [`GIC`] value.

use cubie_a7a_common::mmio::Mmio;
use psci_el3::gicv3::{self, ArchIcc, Gic, GIC600_GICR_STRIDE};

pub use psci_el3::gicv3::GicError;

pub const GICD_BASE: usize = 0x0340_0000;
pub const GICR_BASE: usize = 0x0346_0000;

/// The A733's GIC-600: one redistributor per core, eight cores.
const GIC: Gic = Gic {
    gicd: GICD_BASE,
    gicr: GICR_BASE,
    gicr_stride: GIC600_GICR_STRIDE,
    max_cpus: 8,
};

/// Distributor: once, on the boot CPU. Returns the number of interrupt
/// lines.
pub fn init_distributor() -> Result<u32, GicError> {
    gicv3::init_distributor(&mut Mmio, &GIC)
}

/// Per-CPU part: redistributor wake-up, SGIs/PPIs to Group 1 NS, and the
/// EL3 CPU interface. Returns the redistributor base.
pub fn init_cpu(mpidr: u64, core: usize) -> Result<usize, GicError> {
    gicv3::init_cpu(&mut Mmio, &mut ArchIcc, &GIC, mpidr, core)
}

/// Quiesce this CPU's interface before it stops (PSCI CPU_OFF).
pub fn cpu_off(rd: usize) {
    gicv3::cpu_off(&mut Mmio, &mut ArchIcc, rd)
}
