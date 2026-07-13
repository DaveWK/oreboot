// SPDX-License-Identifier: GPL-2.0-only
// Excerpt of ore-edk-boot-radxa-a7a ed42550 (oreboot tree a803f4f780d7)
// src/mainboard/radxa/cubie-a7a/main/src/main.rs, the oracle for the el3
// equivalence tests: the EL3/EL2 constants and Context, el3_sync_lower,
// el3_unhandled, midr, is_cortex_a76, cpu_errata and el3_cpu_init, verbatim
// except the inline assembly and the calls out of this file, replaced by
// the test's recorder (lines marked `HOST:`). `crate::`, `soc::` and
// `println!` resolve in the test crate.
#![allow(dead_code)]
use crate::soc;
/// SCR_EL3: NS, RES1[5:4], HCE, SIF, RW (lower ELs are AArch64).
const SCR_EL3_VAL: u64 = (1 << 0) | (0b11 << 4) | (1 << 8) | (1 << 9) | (1 << 10);
/// SCTLR_EL2 RES1 bits; MMU and caches off.
const SCTLR_EL2_VAL: u64 = 0x30c5_0830;
/// HCR_EL2.RW: EL1 is AArch64.
const HCR_EL2_VAL: u64 = 1 << 31;
/// CPTR_EL2 RES1 bits; no FP/SIMD traps.
const CPTR_EL2_VAL: u64 = 0x33ff;
/// CNTHCTL_EL2: EL1 access to the physical counter and timer.
const CNTHCTL_EL2_VAL: u64 = 0x3;
/// SPSR_EL3 for the payload: EL2h, DAIF masked.
const SPSR_EL2H: u64 = 0x3c9;

/// Saved lower-EL context on SMC entry, see `el3_vectors`.
#[repr(C)]
pub struct Context {
    pub x: [u64; 31],
    pub elr: u64,
    pub spsr: u64,
    _pad: u64,
}

#[unsafe(no_mangle)]
extern "C-unwind" fn el3_sync_lower(ctx: &mut Context) {
    // HOST: was extern "C" (the test catches the halt)
    let esr: u64;
    esr = crate::model_esr(); // HOST: was asm!("mrs {}, esr_el3", out(reg) esr)
    match esr >> 26 {
        0x17 | 0x13 => crate::psci_handle_smc(ctx), // HOST: was psci::handle_smc(ctx)
        ec => {
            println!(
                "[main] unexpected trap from lower EL: EC {ec:#x} ESR {esr:#x} ELR {:#x}",
                ctx.elr
            );
            crate::halt_sync(ec) // HOST: was halt()
        }
    }
}

#[unsafe(no_mangle)]
extern "C-unwind" fn el3_unhandled(kind: u64, esr: u64, elr: u64, far: u64) -> ! {
    // HOST: was extern "C" (the test catches the halt)
    const NAMES: [&str; 4] = ["sync", "IRQ", "FIQ", "SError"];
    let from = ["EL3/SP0", "EL3/SPx", "lower/A64", "lower/A32"][(kind / 4) as usize & 3];
    println!(
        "[main] EXCEPTION {} from {from}: ESR {esr:#x} ELR {elr:#x} FAR {far:#x}",
        NAMES[(kind % 4) as usize]
    );
    crate::halt_unhandled(NAMES[(kind % 4) as usize], from) // HOST: was halt()
}

const MIDR_PARTNUM_A76: u64 = 0xd0b;

fn midr() -> u64 {
    let v: u64;
    v = crate::model_midr(); // HOST: was asm!("mrs {}, midr_el1", out(reg) v)
    v
}

pub fn is_cortex_a76() -> bool {
    (midr() >> 4) & 0xfff == MIDR_PARTNUM_A76
}

/// Cortex-A76 errata that need EL3 at every core reset, per TF-A
/// (lib/cpus/aarch64/cortex_a76.S). This board's A76s are r4p1. The A55s
/// are r2p0, which no A55 erratum handled by TF-A covers.
fn cpu_errata() {
    let midr = midr();
    if (midr >> 4) & 0xfff != MIDR_PARTNUM_A76 {
        return;
    }
    let rev = ((midr >> 16) & 0xf0) | (midr & 0xf); // 0xVR, e.g. 0x41
    if (0x30..=0x41).contains(&rev) {
        // Erratum 1946160: IMPDEF instruction patching slots 3, 4 and 5.
        for (slot, a, b) in [
            (3u64, 0x10_e390_0002u64, 0x10_fff0_0083u64),
            (4, 0x10_e380_0082, 0x10_fff0_0083),
            (5, 0x10_e380_0200, 0x10_fff0_03e0),
        ] {
            crate::patch_slot(slot, a, b, 0x2_0010_03ffu64); // HOST: was the four msr S3_6_C15_C8_{0,2,3,1} writes
        }
        crate::isb(); // HOST: was asm!("isb")
    }
}

/// Per-core EL3 register setup, cold and warm boot.
fn el3_cpu_init() {
    cpu_errata();
    // HOST: was one asm! block: msr scr_el3, cptr_el3 = xzr, mdcr_el3 = xzr, cntfrq_el0, isb
    crate::write_scr(SCR_EL3_VAL);
    crate::write_cptr(0);
    crate::write_mdcr(0);
    crate::write_cntfrq(soc::TIMER_HZ);
    crate::isb();
}

// HOST: test accessors (not in the board file).
pub(crate) fn el3_cpu_init_for_test() {
    el3_cpu_init()
}
pub(crate) fn el3_sync_lower_for_test(ctx: &mut Context) {
    el3_sync_lower(ctx)
}
pub(crate) fn el3_unhandled_for_test(kind: u64, esr: u64, elr: u64, far: u64) -> ! {
    el3_unhandled(kind, esr, elr, far)
}
pub(crate) fn new_context() -> Context {
    Context {
        x: [0; 31],
        elr: 0,
        spsr: 0,
        _pad: 0,
    }
}
pub(crate) mod consts {
    pub const SCR_EL3_VAL: u64 = super::SCR_EL3_VAL;
    pub const SCTLR_EL2_VAL: u64 = super::SCTLR_EL2_VAL;
    pub const HCR_EL2_VAL: u64 = super::HCR_EL2_VAL;
    pub const CPTR_EL2_VAL: u64 = super::CPTR_EL2_VAL;
    pub const CNTHCTL_EL2_VAL: u64 = super::CNTHCTL_EL2_VAL;
    pub const SPSR_EL2H: u64 = super::SPSR_EL2H;
}
