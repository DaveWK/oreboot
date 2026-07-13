// SPDX-License-Identifier: GPL-2.0-only
//! GICv3 (Arm GIC-600) secure-side set-up at EL3.
//!
//! From ore-edk-boot-radxa-a7a `main/src/gic.rs` (A733): every interrupt is
//! handed to the non-secure world as Group 1 non-secure, affinity routing is
//! on, and the system-register interface is enabled for lower ELs. The
//! distributor and redistributor bases, the redistributor stride and the
//! number of redistributors are parameters ([`Gic`]); MMIO goes through
//! [`regio::RegIo`] and the three EL3 ICC system registers through
//! [`IccEl3`], so host tests can model both. Polls are bounded
//! ([`POLL_LIMIT`]) where the board code spun forever.

use regio::RegIo;

/// Distributor register offsets and bits.
pub mod gicd {
    pub const CTLR: usize = 0x0000;
    pub const TYPER: usize = 0x0004;
    pub const IGROUPR: usize = 0x0080;
    pub const ICENABLER: usize = 0x0180;
    pub const IPRIORITYR: usize = 0x0400;
    pub const IGRPMODR: usize = 0x0d00;

    pub const CTLR_EN_GRP0: u32 = 1 << 0;
    pub const CTLR_EN_GRP1NS: u32 = 1 << 1;
    pub const CTLR_EN_GRP1S: u32 = 1 << 2;
    pub const CTLR_ARE_S: u32 = 1 << 4;
    pub const CTLR_ARE_NS: u32 = 1 << 5;
    pub const CTLR_RWP: u32 = 1 << 31;
}

/// Redistributor register offsets and bits (RD_base, then SGI_base at +64K).
pub mod gicr {
    pub const TYPER: usize = 0x0008;
    pub const WAKER: usize = 0x0014;
    pub const PWRR: usize = 0x0024;
    pub const SGI: usize = 0x1_0000;
    pub const IGROUPR0: usize = SGI + 0x0080;
    pub const ICENABLER0: usize = SGI + 0x0180;
    pub const IPRIORITYR: usize = SGI + 0x0400;
    pub const IGRPMODR0: usize = SGI + 0x0d00;

    pub const WAKER_PROCESSOR_SLEEP: u32 = 1 << 1;
    pub const WAKER_CHILDREN_ASLEEP: u32 = 1 << 2;
    pub const TYPER_LAST: u64 = 1 << 4;

    /// GIC-600 power register.
    pub const PWRR_RDPD: u32 = 1 << 0;
    pub const PWRR_RDGPD: u32 = 1 << 2;
    pub const PWRR_RDGPO: u32 = 1 << 3;
}

/// RD_base + SGI_base frames, 64K each (GIC-600 has no VLPI frames).
pub const GIC600_GICR_STRIDE: usize = 0x2_0000;
/// Priority of every interrupt the secure side configures.
pub const DEFAULT_PRIO: u32 = 0xa0a0_a0a0;
/// Polls of one condition before giving up.
pub const POLL_LIMIT: u32 = 10_000_000;

/// ICC_SRE_EL3: SRE | DFB | DIB | Enable (lower ELs may use SRE).
pub const ICC_SRE_EL3_VAL: u64 = 0xf;
/// ICC_CTLR_EL3: defaults (no EOImode split, PMHE off).
pub const ICC_CTLR_EL3_VAL: u64 = 0;
/// ICC_IGRPEN1_EL3: Group 1 non-secure and secure enabled.
pub const ICC_IGRPEN1_EL3_ON: u64 = 0x3;

/// Where the GIC is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Gic {
    pub gicd: usize,
    pub gicr: usize,
    pub gicr_stride: usize,
    /// Redistributors to scan (one per core).
    pub max_cpus: usize,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GicError {
    /// A register did not reach its state within [`POLL_LIMIT`] polls.
    Timeout(&'static str),
    /// No redistributor reports this MPIDR's affinity.
    NoRedistributor { mpidr: u64 },
}

/// The EL3 CPU-interface system registers.
pub trait IccEl3 {
    /// ICC_SRE_EL3, followed by an ISB.
    fn write_sre(&mut self, v: u64);
    /// ICC_CTLR_EL3.
    fn write_ctlr(&mut self, v: u64);
    /// ICC_IGRPEN1_EL3, followed by an ISB.
    fn write_igrpen1(&mut self, v: u64);
}

/// The real registers (AArch64 EL3).
#[cfg(target_arch = "aarch64")]
pub struct ArchIcc;

#[cfg(target_arch = "aarch64")]
impl IccEl3 for ArchIcc {
    fn write_sre(&mut self, v: u64) {
        unsafe { core::arch::asm!("msr S3_6_C12_C12_5, {}", "isb", in(reg) v) };
    }
    fn write_ctlr(&mut self, v: u64) {
        unsafe { core::arch::asm!("msr S3_6_C12_C12_4, {}", in(reg) v) };
    }
    fn write_igrpen1(&mut self, v: u64) {
        unsafe { core::arch::asm!("msr S3_6_C12_C12_7, {}", "isb", in(reg) v) };
    }
}

/// A 64-bit register as two 32-bit reads, low word first (oreboot
/// `util::mmio::read64le`).
fn read64<I: RegIo + ?Sized>(io: &mut I, addr: usize) -> u64 {
    let l = io.read32(addr) as u64;
    let h = io.read32(addr + 4) as u64;
    (h << 32) | l
}

fn poll<I: RegIo + ?Sized>(
    io: &mut I,
    what: &'static str,
    mut done: impl FnMut(&mut I) -> bool,
) -> Result<(), GicError> {
    for _ in 0..POLL_LIMIT {
        if done(io) {
            return Ok(());
        }
    }
    Err(GicError::Timeout(what))
}

fn wait_rwp<I: RegIo + ?Sized>(io: &mut I, g: &Gic) -> Result<(), GicError> {
    poll(io, "GICD_CTLR.RWP", |io| {
        io.read32(g.gicd + gicd::CTLR) & gicd::CTLR_RWP == 0
    })
}

/// Distributor: once, on the boot CPU. Returns the number of interrupt
/// lines (32 * (ITLinesNumber + 1)).
pub fn init_distributor<I: RegIo + ?Sized>(io: &mut I, g: &Gic) -> Result<u32, GicError> {
    use gicd::*;
    let d = g.gicd;
    io.write32(d + CTLR, 0);
    wait_rwp(io, g)?;
    io.write32(d + CTLR, CTLR_ARE_S | CTLR_ARE_NS);
    wait_rwp(io, g)?;

    let it_lines = (io.read32(d + TYPER) & 0x1f) as usize;
    let num_irqs = 32 * (it_lines + 1);
    // SPIs (32..): disabled, Group 1 non-secure, default priority.
    for n in 1..=it_lines {
        io.write32(d + ICENABLER + 4 * n, !0);
        io.write32(d + IGROUPR + 4 * n, !0);
        io.write32(d + IGRPMODR + 4 * n, 0);
    }
    for n in (32..num_irqs).step_by(4) {
        io.write32(d + IPRIORITYR + n, DEFAULT_PRIO);
    }
    wait_rwp(io, g)?;
    io.write32(
        d + CTLR,
        CTLR_ARE_S | CTLR_ARE_NS | CTLR_EN_GRP0 | CTLR_EN_GRP1NS | CTLR_EN_GRP1S,
    );
    wait_rwp(io, g)?;
    Ok(num_irqs as u32)
}

/// GIC-600: power up a redistributor (GICR_PWRR.RDPD) before using it.
pub fn power_on<I: RegIo + ?Sized>(io: &mut I, rd: usize) -> Result<(), GicError> {
    use gicr::*;
    for _ in 0..POLL_LIMIT {
        // Wait for the power group not to be in transition.
        poll(io, "GICR_PWRR transition", |io| {
            let p = io.read32(rd + PWRR);
            (p & PWRR_RDGPD != 0) == (p & PWRR_RDGPO != 0)
        })?;
        io.write32(rd + PWRR, 0);
        if io.read32(rd + PWRR) & PWRR_RDPD == 0 {
            return Ok(());
        }
    }
    Err(GicError::Timeout("GICR_PWRR.RDPD"))
}

/// The GICR_TYPER affinity value (bits [63:32]) of an MPIDR.
pub const fn mpidr_to_gicr_affinity(mpidr: u64) -> u32 {
    ((((mpidr >> 32) & 0xff) << 24) | (mpidr & 0x00ff_ffff)) as u32
}

/// This CPU's redistributor: first guess by core index, else scan until
/// GICR_TYPER.Last. Each candidate is powered on before its TYPER is read.
pub fn find_redistributor<I: RegIo + ?Sized>(
    io: &mut I,
    g: &Gic,
    mpidr: u64,
    core: usize,
) -> Result<Option<usize>, GicError> {
    let want = mpidr_to_gicr_affinity(mpidr);
    let guess = g.gicr + core * g.gicr_stride;
    power_on(io, guess)?;
    if (read64(io, guess + gicr::TYPER) >> 32) as u32 == want {
        return Ok(Some(guess));
    }
    for i in 0..g.max_cpus {
        let rd = g.gicr + i * g.gicr_stride;
        power_on(io, rd)?;
        let typer = read64(io, rd + gicr::TYPER);
        if (typer >> 32) as u32 == want {
            return Ok(Some(rd));
        }
        if typer & gicr::TYPER_LAST != 0 {
            break;
        }
    }
    Ok(None)
}

/// Per-CPU part: redistributor wake-up, SGIs/PPIs to Group 1 non-secure,
/// and the EL3 CPU interface. Returns the redistributor base.
pub fn init_cpu<I: RegIo + ?Sized, C: IccEl3 + ?Sized>(
    io: &mut I,
    icc: &mut C,
    g: &Gic,
    mpidr: u64,
    core: usize,
) -> Result<usize, GicError> {
    use gicr::*;
    let rd = find_redistributor(io, g, mpidr, core)?.ok_or(GicError::NoRedistributor { mpidr })?;
    let w = io.read32(rd + WAKER);
    io.write32(rd + WAKER, w & !WAKER_PROCESSOR_SLEEP);
    poll(io, "GICR_WAKER.ChildrenAsleep", |io| {
        io.read32(rd + WAKER) & WAKER_CHILDREN_ASLEEP == 0
    })?;

    io.write32(rd + ICENABLER0, !0);
    io.write32(rd + IGROUPR0, !0);
    io.write32(rd + IGRPMODR0, 0);
    for n in (0..32).step_by(4) {
        io.write32(rd + IPRIORITYR + n, DEFAULT_PRIO);
    }

    icc.write_sre(ICC_SRE_EL3_VAL);
    icc.write_ctlr(ICC_CTLR_EL3_VAL);
    icc.write_igrpen1(ICC_IGRPEN1_EL3_ON);
    Ok(rd)
}

/// Quiesce this CPU's interface before it stops (PSCI CPU_OFF).
pub fn cpu_off<I: RegIo + ?Sized, C: IccEl3 + ?Sized>(io: &mut I, icc: &mut C, rd: usize) {
    icc.write_igrpen1(0);
    let w = io.read32(rd + gicr::WAKER);
    io.write32(rd + gicr::WAKER, w | gicr::WAKER_PROCESSOR_SLEEP);
}
