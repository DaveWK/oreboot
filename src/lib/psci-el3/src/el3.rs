// SPDX-License-Identifier: GPL-2.0-only
//! The generic Arm EL3 runtime around the PSCI dispatcher.
//!
//! From ore-edk-boot-radxa-a7a `main/src/main.rs` (A733): the EL3 and EL2
//! register values, the lower-EL [`Context`] the SMC vector saves, trap
//! classification, the per-core EL3 set-up ([`el3_cpu_init`]) with the CPU
//! errata as data ([`ERRATA`]), the drop to EL2 ([`enter_nonsecure`]) and,
//! with feature `vectors`, the EL3 exception vector table. System registers
//! go through [`El3Regs`] so host tests can model them; [`ArchEl3`] is the
//! real implementation on AArch64.
//!
//! Board-side: the reset entry (`start`: stacks, cold/warm boot state,
//! primary/secondary entry), the counter frequency, the SoC security set-up,
//! power domains, reset vectors and the bodies of `el3_sync_lower` /
//! `el3_unhandled` that the vector table calls.

/// SCR_EL3: NS, RES1[5:4], HCE, SIF, RW (lower ELs are AArch64).
pub const SCR_EL3_VAL: u64 = (1 << 0) | (0b11 << 4) | (1 << 8) | (1 << 9) | (1 << 10);
/// SCTLR_EL3: RES1 bits + I-cache; MMU, D-cache, alignment checks off (the
/// value the reset entry writes).
pub const SCTLR_EL3_VAL: u64 = 0x30c5_1830;
/// SCTLR_EL2 RES1 bits; MMU and caches off.
pub const SCTLR_EL2_VAL: u64 = 0x30c5_0830;
/// HCR_EL2.RW: EL1 is AArch64.
pub const HCR_EL2_VAL: u64 = 1 << 31;
/// CPTR_EL2 RES1 bits; no FP/SIMD traps.
pub const CPTR_EL2_VAL: u64 = 0x33ff;
/// CNTHCTL_EL2: EL1 access to the physical counter and timer.
pub const CNTHCTL_EL2_VAL: u64 = 0x3;
/// SPSR_EL3 for the payload: EL2h, DAIF masked.
pub const SPSR_EL2H: u64 = 0x3c9;

/// Saved lower-EL context on SMC entry (layout fixed by the vector code:
/// x0..x30 at 0, ELR at 248, SPSR at 256; 272 bytes).
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Context {
    pub x: [u64; 31],
    pub elr: u64,
    pub spsr: u64,
    pub _pad: u64,
}

/// Size of [`Context`] on the EL3 stack.
pub const CONTEXT_SIZE: usize = 272;

/// What a synchronous trap from a lower EL is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Trap {
    /// SMC from AArch64 (EC 0x17) or AArch32 (EC 0x13): PSCI/SMCCC.
    Smc,
    /// Anything else, with its exception class.
    Other(u64),
}

/// Classify ESR_EL3 of a synchronous exception from a lower EL.
pub const fn classify(esr: u64) -> Trap {
    match esr >> 26 {
        0x17 | 0x13 => Trap::Smc,
        ec => Trap::Other(ec),
    }
}

/// Name and origin of an unhandled vector entry (`kind` = vector index
/// 0..15), as the board reports it.
pub const fn unhandled_kind(kind: u64) -> (&'static str, &'static str) {
    const NAMES: [&str; 4] = ["sync", "IRQ", "FIQ", "SError"];
    const FROM: [&str; 4] = ["EL3/SP0", "EL3/SPx", "lower/A64", "lower/A32"];
    (NAMES[(kind % 4) as usize], FROM[(kind / 4) as usize & 3])
}

/// MIDR_EL1 part number of the Cortex-A76.
pub const MIDR_PARTNUM_A76: u16 = 0xd0b;
/// MIDR_EL1 part number of the Cortex-A55.
pub const MIDR_PARTNUM_A55: u16 = 0xd05;

/// The part number field of MIDR_EL1.
pub const fn part_num(midr: u64) -> u16 {
    ((midr >> 4) & 0xfff) as u16
}

/// Variant and revision as 0xVR (e.g. r4p1 = 0x41).
pub const fn variant_revision(midr: u64) -> u8 {
    (((midr >> 16) & 0xf0) | (midr & 0xf)) as u8
}

pub const fn is_cortex_a76(midr: u64) -> bool {
    part_num(midr) == MIDR_PARTNUM_A76
}

/// One IMPDEF instruction-patching slot write (S3_6_C15_C8_0..3).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PatchSlot {
    pub slot: u64,
    pub opcode: u64,
    pub mask: u64,
    pub control: u64,
}

/// A CPU erratum EL3 must work around at every core reset.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Erratum {
    pub id: u32,
    pub part: u16,
    /// Affected 0xVR range, inclusive.
    pub rev_min: u8,
    pub rev_max: u8,
    pub patches: &'static [PatchSlot],
}

impl Erratum {
    pub const fn applies(&self, midr: u64) -> bool {
        let rev = variant_revision(midr);
        part_num(midr) == self.part && rev >= self.rev_min && rev <= self.rev_max
    }
}

const A76_1946160_CONTROL: u64 = 0x2_0010_03ff;

/// The errata handled, per TF-A `lib/cpus/aarch64/cortex_a76.S`. The A733's
/// A76s are r4p1; its A55s are r2p0, which no A55 erratum handled by TF-A
/// covers.
pub const ERRATA: &[Erratum] = &[Erratum {
    // Cortex-A76 erratum 1946160: IMPDEF instruction patching slots 3-5.
    id: 1_946_160,
    part: MIDR_PARTNUM_A76,
    rev_min: 0x30,
    rev_max: 0x41,
    patches: &[
        PatchSlot {
            slot: 3,
            opcode: 0x10_e390_0002,
            mask: 0x10_fff0_0083,
            control: A76_1946160_CONTROL,
        },
        PatchSlot {
            slot: 4,
            opcode: 0x10_e380_0082,
            mask: 0x10_fff0_0083,
            control: A76_1946160_CONTROL,
        },
        PatchSlot {
            slot: 5,
            opcode: 0x10_e380_0200,
            mask: 0x10_fff0_03e0,
            control: A76_1946160_CONTROL,
        },
    ],
}];

/// The EL3 system registers the per-core set-up touches.
pub trait El3Regs {
    fn midr(&mut self) -> u64;
    /// One patching slot: S3_6_C15_C8_0 = slot, _2 = opcode, _3 = mask,
    /// _1 = control, in that order.
    fn patch_slot(&mut self, p: &PatchSlot);
    fn isb(&mut self);
    fn write_scr(&mut self, v: u64);
    fn write_cptr(&mut self, v: u64);
    fn write_mdcr(&mut self, v: u64);
    fn write_cntfrq(&mut self, v: u64);
}

/// Apply every erratum of [`ERRATA`] that matches this core; an ISB after
/// each erratum's patch writes.
pub fn apply_errata<R: El3Regs + ?Sized>(r: &mut R) {
    let midr = r.midr();
    for e in ERRATA.iter().filter(|e| e.applies(midr)) {
        for p in e.patches {
            r.patch_slot(p);
        }
        r.isb();
    }
}

/// Per-core EL3 register set-up, cold and warm boot: errata, SCR_EL3,
/// CPTR_EL3 = 0, MDCR_EL3 = 0, CNTFRQ_EL0 = `counter_hz`, ISB.
pub fn el3_cpu_init<R: El3Regs + ?Sized>(r: &mut R, counter_hz: u64) {
    apply_errata(r);
    r.write_scr(SCR_EL3_VAL);
    r.write_cptr(0);
    r.write_mdcr(0);
    r.write_cntfrq(counter_hz);
    r.isb();
}

/// The real registers (AArch64 EL3).
#[cfg(target_arch = "aarch64")]
pub struct ArchEl3;

#[cfg(target_arch = "aarch64")]
impl El3Regs for ArchEl3 {
    fn midr(&mut self) -> u64 {
        let v: u64;
        unsafe { core::arch::asm!("mrs {}, midr_el1", out(reg) v) };
        v
    }
    fn patch_slot(&mut self, p: &PatchSlot) {
        unsafe {
            core::arch::asm!(
                "msr S3_6_C15_C8_0, {slot}",
                "msr S3_6_C15_C8_2, {a}",
                "msr S3_6_C15_C8_3, {b}",
                "msr S3_6_C15_C8_1, {c}",
                slot = in(reg) p.slot,
                a = in(reg) p.opcode,
                b = in(reg) p.mask,
                c = in(reg) p.control,
            )
        };
    }
    fn isb(&mut self) {
        unsafe { core::arch::asm!("isb") };
    }
    fn write_scr(&mut self, v: u64) {
        unsafe { core::arch::asm!("msr scr_el3, {}", in(reg) v) };
    }
    fn write_cptr(&mut self, v: u64) {
        unsafe { core::arch::asm!("msr cptr_el3, {}", in(reg) v) };
    }
    fn write_mdcr(&mut self, v: u64) {
        unsafe { core::arch::asm!("msr mdcr_el3, {}", in(reg) v) };
    }
    fn write_cntfrq(&mut self, v: u64) {
        unsafe { core::arch::asm!("msr cntfrq_el0, {}", in(reg) v) };
    }
}

/// Drop to EL2 at `entry` with `x0 = arg0`, `x1..x3 = 0` (EL2 MMU and
/// caches off, EL1 AArch64, no FP traps, EL1 counter access).
///
/// # Safety
/// `entry` must be valid EL2 code; EL3 state is abandoned.
#[cfg(target_arch = "aarch64")]
pub unsafe fn enter_nonsecure(entry: u64, arg0: u64) -> ! {
    unsafe {
        core::arch::asm!(
            "msr sctlr_el2, {sctlr}",
            "msr hcr_el2, {hcr}",
            "msr cptr_el2, {cptr}",
            "msr cnthctl_el2, {cnthctl}",
            "msr cntvoff_el2, xzr",
            "msr spsr_el3, {spsr}",
            "msr elr_el3, {entry}",
            "ic iallu",
            "dsb sy",
            "isb",
            "eret",
            sctlr = in(reg) SCTLR_EL2_VAL,
            hcr = in(reg) HCR_EL2_VAL,
            cptr = in(reg) CPTR_EL2_VAL,
            cnthctl = in(reg) CNTHCTL_EL2_VAL,
            spsr = in(reg) SPSR_EL2H,
            entry = in(reg) entry,
            in("x0") arg0,
            in("x1") 0u64,
            in("x2") 0u64,
            in("x3") 0u64,
            options(noreturn)
        )
    }
}

// EL3 exception vectors (feature `vectors`). SMCs from EL2/EL1 (AArch64 or
// AArch32) save the full GPR context ([`Context`]) on SP_EL3 and call the
// board's `el3_sync_lower(ctx: &mut Context)`; anything else calls the
// board's `el3_unhandled(kind, esr, elr, far) -> !`. Before EL3 code runs,
// the branch history is overwritten with 24 taken branches (CVE-2022-23960,
// Spectre-BHB; TF-A wa_cve_2022_23960_bhb, k = 24 for the A76; harmless on
// cores that do not need it).
#[cfg(all(target_arch = "aarch64", feature = "vectors"))]
core::arch::global_asm!(
    r#"
    .section .text.vectors, "ax"
    .balign 0x800
    .global el3_vectors
el3_vectors:
    .irp idx, 0, 1, 2, 3, 4, 5, 6, 7
    .balign 0x80
    mov     x0, #\idx
    b       el3_unhandled_entry
    .endr
    .balign 0x80                // 0x400: lower EL, AArch64, synchronous
    b       el3_lower_sync
    .balign 0x80
    mov     x0, #9
    b       el3_unhandled_entry
    .balign 0x80
    mov     x0, #10
    b       el3_unhandled_entry
    .balign 0x80
    mov     x0, #11
    b       el3_unhandled_entry
    .balign 0x80                // 0x600: lower EL, AArch32, synchronous
    b       el3_lower_sync
    .irp idx, 13, 14, 15
    .balign 0x80
    mov     x0, #\idx
    b       el3_unhandled_entry
    .endr

el3_lower_sync:
    sub     sp, sp, #272
    stp     x0, x1, [sp, #0]
    mov     x0, #24
1:  b       2f
2:  subs    x0, x0, #1
    b.ne    1b
    dsb     sy
    isb
    stp     x2, x3, [sp, #16]
    stp     x4, x5, [sp, #32]
    stp     x6, x7, [sp, #48]
    stp     x8, x9, [sp, #64]
    stp     x10, x11, [sp, #80]
    stp     x12, x13, [sp, #96]
    stp     x14, x15, [sp, #112]
    stp     x16, x17, [sp, #128]
    stp     x18, x19, [sp, #144]
    stp     x20, x21, [sp, #160]
    stp     x22, x23, [sp, #176]
    stp     x24, x25, [sp, #192]
    stp     x26, x27, [sp, #208]
    stp     x28, x29, [sp, #224]
    mrs     x0, elr_el3
    stp     x30, x0, [sp, #240]
    mrs     x0, spsr_el3
    str     x0, [sp, #256]
    mov     x0, sp
    bl      el3_sync_lower
    ldr     x0, [sp, #256]
    msr     spsr_el3, x0
    ldp     x30, x0, [sp, #240]
    msr     elr_el3, x0
    ldp     x0, x1, [sp, #0]
    ldp     x2, x3, [sp, #16]
    ldp     x4, x5, [sp, #32]
    ldp     x6, x7, [sp, #48]
    ldp     x8, x9, [sp, #64]
    ldp     x10, x11, [sp, #80]
    ldp     x12, x13, [sp, #96]
    ldp     x14, x15, [sp, #112]
    ldp     x16, x17, [sp, #128]
    ldp     x18, x19, [sp, #144]
    ldp     x20, x21, [sp, #160]
    ldp     x22, x23, [sp, #176]
    ldp     x24, x25, [sp, #192]
    ldp     x26, x27, [sp, #208]
    ldp     x28, x29, [sp, #224]
    add     sp, sp, #272
    eret

el3_unhandled_entry:
    mrs     x1, esr_el3
    mrs     x2, elr_el3
    mrs     x3, far_el3
    bl      el3_unhandled
    "#
);
