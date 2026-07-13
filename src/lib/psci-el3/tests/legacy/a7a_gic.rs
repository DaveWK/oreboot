// SPDX-License-Identifier: GPL-2.0-only
// Copy of ore-edk-boot-radxa-a7a ed42550 (oreboot tree a803f4f780d7)
// src/mainboard/radxa/cubie-a7a/main/src/gic.rs, the oracle for the gicv3
// equivalence tests. Verbatim except the four `asm!` system-register writes,
// replaced by calls to the test's recorder (lines marked `HOST:`); `util::`
// and `crate::` resolve to the test crate root.
//! GICv3 (Arm GIC-600) secure-side setup: everything is handed to the
//! non-secure world as Group 1 NS, affinity routing on, and the system
//! register interface enabled for lower ELs.

#[allow(unused_imports)]
use core::arch::asm;
use util::mmio::{read32, read64le, write32};

pub const GICD_BASE: usize = 0x0340_0000;
pub const GICR_BASE: usize = 0x0346_0000;
/// RD_base + SGI_base frames, 64K each (GIC-600 has no VLPI frames).
const GICR_STRIDE: usize = 0x2_0000;
const MAX_CPUS: usize = 8;

const GICD_CTLR: usize = 0x0000;
const GICD_TYPER: usize = 0x0004;
const GICD_IGROUPR: usize = 0x0080;
const GICD_ICENABLER: usize = 0x0180;
const GICD_IPRIORITYR: usize = 0x0400;
const GICD_IGRPMODR: usize = 0x0d00;

const CTLR_EN_GRP0: u32 = 1 << 0;
const CTLR_EN_GRP1NS: u32 = 1 << 1;
const CTLR_EN_GRP1S: u32 = 1 << 2;
const CTLR_ARE_S: u32 = 1 << 4;
const CTLR_ARE_NS: u32 = 1 << 5;
const CTLR_RWP: u32 = 1 << 31;

const GICR_TYPER: usize = 0x0008;
const GICR_WAKER: usize = 0x0014;
const GICR_PWRR: usize = 0x0024;
const GICR_SGI: usize = 0x1_0000;
const GICR_IGROUPR0: usize = GICR_SGI + 0x0080;
const GICR_ICENABLER0: usize = GICR_SGI + 0x0180;
const GICR_IPRIORITYR: usize = GICR_SGI + 0x0400;
const GICR_IGRPMODR0: usize = GICR_SGI + 0x0d00;

const WAKER_PROCESSOR_SLEEP: u32 = 1 << 1;
const WAKER_CHILDREN_ASLEEP: u32 = 1 << 2;
const TYPER_LAST: u64 = 1 << 4;

const PWRR_RDPD: u32 = 1 << 0;
const PWRR_RDGPD: u32 = 1 << 2;
const PWRR_RDGPO: u32 = 1 << 3;

const DEFAULT_PRIO: u32 = 0xa0a0_a0a0;

fn wait_rwp() {
    while read32(GICD_BASE + GICD_CTLR) & CTLR_RWP != 0 {}
}

/// Distributor: once, on the boot CPU.
pub fn init_distributor() -> u32 {
    write32(GICD_BASE + GICD_CTLR, 0);
    wait_rwp();
    write32(GICD_BASE + GICD_CTLR, CTLR_ARE_S | CTLR_ARE_NS);
    wait_rwp();

    let it_lines = (read32(GICD_BASE + GICD_TYPER) & 0x1f) as usize;
    let num_irqs = 32 * (it_lines + 1);
    // SPIs (32..): disabled, Group 1 non-secure, default priority.
    for n in 1..=it_lines {
        write32(GICD_BASE + GICD_ICENABLER + 4 * n, !0);
        write32(GICD_BASE + GICD_IGROUPR + 4 * n, !0);
        write32(GICD_BASE + GICD_IGRPMODR + 4 * n, 0);
    }
    for n in (32..num_irqs).step_by(4) {
        write32(GICD_BASE + GICD_IPRIORITYR + n, DEFAULT_PRIO);
    }
    wait_rwp();
    write32(
        GICD_BASE + GICD_CTLR,
        CTLR_ARE_S | CTLR_ARE_NS | CTLR_EN_GRP0 | CTLR_EN_GRP1NS | CTLR_EN_GRP1S,
    );
    wait_rwp();
    num_irqs as u32
}

/// GIC-600: power up a redistributor (GICR_PWRR.RDPD) before using it.
fn power_on(rd: usize) {
    loop {
        // wait for the power group not to be in transition
        loop {
            let p = read32(rd + GICR_PWRR);
            if (p & PWRR_RDGPD != 0) == (p & PWRR_RDGPO != 0) {
                break;
            }
        }
        write32(rd + GICR_PWRR, 0);
        if read32(rd + GICR_PWRR) & PWRR_RDPD == 0 {
            break;
        }
    }
}

fn mpidr_to_gicr_affinity(mpidr: u64) -> u32 {
    (((mpidr >> 32) & 0xff) << 24 | (mpidr & 0x00ff_ffff)) as u32
}

/// Find this CPU's redistributor: first guess by core index, else scan.
fn find_redistributor(mpidr: u64, core: usize) -> Option<usize> {
    let want = mpidr_to_gicr_affinity(mpidr);
    let guess = GICR_BASE + core * GICR_STRIDE;
    power_on(guess);
    if (read64le(guess + GICR_TYPER) >> 32) as u32 == want {
        return Some(guess);
    }
    for i in 0..MAX_CPUS {
        let rd = GICR_BASE + i * GICR_STRIDE;
        power_on(rd);
        let typer = read64le(rd + GICR_TYPER);
        if (typer >> 32) as u32 == want {
            return Some(rd);
        }
        if typer & TYPER_LAST != 0 {
            break;
        }
    }
    None
}

/// Per-CPU part: redistributor wake-up, SGIs/PPIs to Group 1 NS, and the
/// EL3 CPU interface.
pub fn init_cpu(mpidr: u64, core: usize) -> Result<usize, ()> {
    let rd = find_redistributor(mpidr, core).ok_or(())?;
    write32(
        rd + GICR_WAKER,
        read32(rd + GICR_WAKER) & !WAKER_PROCESSOR_SLEEP,
    );
    while read32(rd + GICR_WAKER) & WAKER_CHILDREN_ASLEEP != 0 {}

    write32(rd + GICR_ICENABLER0, !0);
    write32(rd + GICR_IGROUPR0, !0);
    write32(rd + GICR_IGRPMODR0, 0);
    for n in (0..32).step_by(4) {
        write32(rd + GICR_IPRIORITYR + n, DEFAULT_PRIO);
    }

    {
        // ICC_SRE_EL3: SRE | DFB | DIB | Enable (lower ELs may use SRE)
        crate::icc_sre(0xfu64); // HOST: was asm!("msr S3_6_C12_C12_5, {}", "isb", ...)
                                // ICC_CTLR_EL3: defaults (no EOImode split, PMHE off)
        crate::icc_ctlr(0u64); // HOST: was asm!("msr S3_6_C12_C12_4, {}", ...)
                               // ICC_IGRPEN1_EL3: enable Group 1 NS (and S) at the CPU interface
        crate::icc_igrpen1(0x3u64); // HOST: was asm!("msr S3_6_C12_C12_7, {}", "isb", ...)
    }
    Ok(rd)
}

/// Quiesce this CPU's interface before it stops (PSCI CPU_OFF).
pub fn cpu_off(rd: usize) {
    crate::icc_igrpen1(0u64); // HOST: was asm!("msr S3_6_C12_C12_7, {}", "isb", in(reg) 0u64)
    write32(
        rd + GICR_WAKER,
        read32(rd + GICR_WAKER) | WAKER_PROCESSOR_SLEEP,
    );
}
