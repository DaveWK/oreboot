//! oreboot main for the Radxa Cubie A7A (Allwinner A733).
//!
//! Entered in AArch64 EL3 through RVBAR after bt0's warm reset. Sets up the
//! secure side (SCR_EL3, GICv3, peripheral security), builds a device tree
//! describing the memory the payload may use, and drops to EL2 into the
//! payload (EDK2). It then stays resident at the bottom of DRAM to serve
//! PSCI calls.
#![no_std]
#![no_main]

#[macro_use]
extern crate log;

use core::arch::{asm, global_asm, naked_asm};
use core::panic::PanicInfo;
use core::ptr::{addr_of_mut, read_volatile, write_volatile};

use cubie_a7a_common::{
    mem,
    uart::{init_logger, Uart},
    Handoff, HANDOFF_MAGIC, HANDOFF_VERSION,
};

mod fdt;
mod gic;
mod gmac;
#[cfg(feature = "memtest")]
mod memtest;
mod pcie;
mod pmic;
mod ppu;
mod psci;
mod psci_lock;
mod soc;
mod usb;

use psci::{Cpu, State};

const STACK_SIZE: usize = 16 * 1024;

/// Boot CPU cold boot vs. any later entry (PSCI warm boot). Lives in .data
/// (non-zero initializer) so clearing .bss does not touch it.
const BOOT_COLD: u32 = 0xc01d_b007;
const BOOT_WARM: u32 = 0x0000_600d;
static mut BOOT_STATE: u32 = BOOT_COLD;

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

/// # Safety
/// Reset entry (RVBAR) for every core: cold boot and PSCI warm boot.
#[unsafe(naked)]
#[unsafe(export_name = "start")]
#[unsafe(link_section = ".text.entry")]
pub unsafe extern "C" fn start() -> ! {
    naked_asm!(
        "mrs    x19, mpidr_el1",
        "ubfx   x20, x19, #8, #8",          // core index = Aff1
        "cmp    x20, #{max_cpus}",
        "b.hs   3f",
        "adr    x0, _stacks_top",
        "mov    x1, #{stack_size}",
        "msub   x0, x20, x1, x0",
        "mov    sp, x0",
        "adr    x0, el3_vectors",
        "msr    vbar_el3, x0",
        // SCTLR_EL3: RES1 bits + I-cache; MMU, D-cache, alignment checks off
        "movz   x0, #0x1830",
        "movk   x0, #0x30c5, lsl #16",
        "msr    sctlr_el3, x0",
        "isb",
        "adr    x0, {boot_state}",
        "ldr    w1, [x0]",
        "movz   w2, #{cold_lo}",
        "movk   w2, #{cold_hi}, lsl #16",
        "cmp    w1, w2",
        "b.ne   2f",
        "adr    x0, _sbss",
        "adr    x1, _ebss",
        "1:",
        "cmp    x0, x1",
        "b.hs   4f",
        "str    xzr, [x0], #8",
        "b      1b",
        "4:",
        "mov    x0, x19",
        "mov    x1, x20",
        "bl     {primary}",
        "2:",
        "mov    x0, x19",
        "mov    x1, x20",
        "bl     {secondary}",
        "3:",
        "wfi",
        "b      3b",
        max_cpus = const psci::MAX_CPUS,
        stack_size = const STACK_SIZE,
        boot_state = sym BOOT_STATE,
        cold_lo = const BOOT_COLD & 0xffff,
        cold_hi = const BOOT_COLD >> 16,
        primary = sym primary_main,
        secondary = sym secondary_main,
    )
}

pub fn warm_entry_addr() -> usize {
    start as *const () as usize
}

// EL3 exception vectors. SMCs from EL2/EL1 (AArch64 or AArch32) save the
// full GPR context on SP_EL3 and go to `el3_sync_lower`; anything else is
// reported and halts.
global_asm!(
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
    // CVE-2022-23960 (Spectre-BHB): overwrite the branch history with 24
    // taken branches before EL3 code runs (TF-A wa_cve_2022_23960_bhb,
    // k = 24 for the A76). Harmless on the A55.
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

#[unsafe(no_mangle)]
extern "C" fn el3_sync_lower(ctx: &mut Context) {
    let esr: u64;
    unsafe { asm!("mrs {}, esr_el3", out(reg) esr) };
    match esr >> 26 {
        0x17 | 0x13 => psci::handle_smc(ctx),
        ec => {
            println!(
                "[main] unexpected trap from lower EL: EC {ec:#x} ESR {esr:#x} ELR {:#x}",
                ctx.elr
            );
            halt()
        }
    }
}

#[unsafe(no_mangle)]
extern "C" fn el3_unhandled(kind: u64, esr: u64, elr: u64, far: u64) -> ! {
    const NAMES: [&str; 4] = ["sync", "IRQ", "FIQ", "SError"];
    let from = ["EL3/SP0", "EL3/SPx", "lower/A64", "lower/A32"][(kind / 4) as usize & 3];
    println!(
        "[main] EXCEPTION {} from {from}: ESR {esr:#x} ELR {elr:#x} FAR {far:#x}",
        NAMES[(kind % 4) as usize]
    );
    halt()
}

fn halt() -> ! {
    loop {
        unsafe { asm!("wfi") };
    }
}

fn current_el() -> u64 {
    let el: u64;
    unsafe { asm!("mrs {}, CurrentEL", out(reg) el) };
    (el >> 2) & 3
}

fn ticks() -> u64 {
    let t: u64;
    unsafe { asm!("isb", "mrs {}, cntpct_el0", out(reg) t) };
    t
}

fn current_core() -> usize {
    let mpidr: u64;
    unsafe { asm!("mrs {}, mpidr_el1", out(reg) mpidr) };
    ((mpidr >> 8) & 0xff) as usize
}

const MIDR_PARTNUM_A76: u64 = 0xd0b;

fn midr() -> u64 {
    let v: u64;
    unsafe { asm!("mrs {}, midr_el1", out(reg) v) };
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
            unsafe {
                asm!(
                    "msr S3_6_C15_C8_0, {slot}",
                    "msr S3_6_C15_C8_2, {a}",
                    "msr S3_6_C15_C8_3, {b}",
                    "msr S3_6_C15_C8_1, {c}",
                    slot = in(reg) slot,
                    a = in(reg) a,
                    b = in(reg) b,
                    c = in(reg) 0x2_0010_03ffu64,
                )
            };
        }
        unsafe { asm!("isb") };
    }
}

/// Per-core EL3 register setup, cold and warm boot.
fn el3_cpu_init() {
    cpu_errata();
    unsafe {
        asm!(
            "msr scr_el3, {scr}",
            "msr cptr_el3, xzr",
            "msr mdcr_el3, xzr",
            "msr cntfrq_el0, {freq}",
            "isb",
            scr = in(reg) SCR_EL3_VAL,
            freq = in(reg) soc::TIMER_HZ,
        )
    };
}

/// Drop to EL2 at `entry` with `x0 = arg0`, `x1..x3 = 0`.
fn enter_nonsecure(entry: u64, arg0: u64) -> ! {
    unsafe {
        asm!(
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

fn build_dtb(h: &Handoff) -> usize {
    static mut FDT: fdt::Fdt = fdt::Fdt::new();
    // SAFETY: only the boot CPU runs this, once.
    let f = unsafe { &mut *addr_of_mut!(FDT) };
    let (mem_base, mem_size) = mem::payload_region(h.dram_base, h.dram_size)
        .expect("bt0 handed over an invalid DRAM region");

    f.begin_node("");
    f.prop_str("compatible", "radxa,cubie-a7a\0allwinner,sun60i-a733");
    f.prop_str("model", "Radxa Cubie A7A");
    f.prop_u32("#address-cells", 2);
    f.prop_u32("#size-cells", 2);

    f.begin_node("chosen");
    f.prop_str("stdout-path", "serial0:115200n8");
    f.end_node();

    f.begin_node("aliases");
    f.prop_str("serial0", "/serial@2500000");
    f.end_node();

    f.begin_node("memory@40200000");
    f.prop_str("device_type", "memory");
    f.prop_u64_cells("reg", &[mem_base, mem_size]);
    f.end_node();

    f.begin_node("psci");
    f.prop_str("compatible", "arm,psci-1.0\0arm,psci-0.2");
    f.prop_str("method", "smc");
    f.end_node();

    // What bt0 set up, for the payload (EDK2's SMBIOS tables). Not a
    // binding Linux knows; it ignores the node.
    f.begin_node("firmware");
    f.begin_node("oreboot");
    f.prop_str("compatible", "oreboot,cubie-a7a");
    f.prop_u32_cells("cpu-clock-mhz", &h.cpu_mhz);
    f.prop_u32("dram-clock-mhz", h.dram_mhz);
    if let Some(mac) = gmac::mac_address() {
        // GMAC0's address; EDK2 uses it and copies it into the OS DTB.
        f.prop("mac-address", &mac);
    }
    // Host controllers usb::init() brought up, which EDK2 registers.
    let usb = usb::ready();
    let mut list = [0u8; 18];
    let mut n = 0;
    for (up, name) in [
        (usb.ehci0, "ehci0\0"),
        (usb.ehci1, "ehci1\0"),
        (usb.xhci, "xhci\0"),
    ] {
        if up {
            list[n..n + name.len()].copy_from_slice(name.as_bytes());
            n += name.len();
        }
    }
    if n != 0 {
        f.prop("usb-ready", &list[..n]);
    }
    // Child nodes after all properties: the trained PCIe link, if any.
    pcie::fdt_node(f);
    f.end_node();
    f.end_node();

    f.begin_node("serial@2500000");
    f.prop_str("compatible", "snps,dw-apb-uart");
    f.prop_u64_cells("reg", &[soc::UART0_BASE as u64, 0x400]);
    f.prop_u32("reg-shift", 2);
    f.prop_u32("reg-io-width", 4);
    f.prop_u32("clock-frequency", soc::UART_CLOCK_HZ);
    f.prop_str("status", "okay");
    f.end_node();

    f.end_node();

    // SAFETY: DTB_ADDR..+DTB_MAX_SIZE is reserved for this in the EL3 region.
    let out =
        unsafe { core::slice::from_raw_parts_mut(mem::DTB_ADDR as *mut u8, mem::DTB_MAX_SIZE) };
    f.finish(out)
}

extern "C" fn primary_main(mpidr: u64, core: usize) -> ! {
    cubie_a7a_common::fwlog::attach();
    init_logger(Uart::attach(soc::UART0_BASE));
    let t0 = ticks();
    println!();
    println!(
        "oreboot 🦀 main | Radxa Cubie A7A | AArch64 EL{}",
        current_el()
    );

    let h: Handoff = unsafe { read_volatile(mem::HANDOFF_ADDR as *const Handoff) };
    if h.magic != HANDOFF_MAGIC || h.version != HANDOFF_VERSION {
        panic!("no valid handoff from bt0 at {:#x}", mem::HANDOFF_ADDR);
    }
    if !mem::valid_payload_entry(h.dram_base, h.dram_size, h.payload_addr) {
        panic!("bt0 handed over an invalid payload entry or DRAM extent");
    }
    println!(
        "[main] DRAM {} MiB @ {:#x}, warm reset took {} us",
        h.dram_size >> 20,
        h.dram_base,
        (t0 - h.bt0_done_ticks) / (soc::TIMER_HZ / 1_000_000)
    );

    el3_cpu_init();
    soc::security_setup();
    // Like TF-A at boot: make sure no other core is left armed to run.
    for other in (0..psci::MAX_CPUS).filter(|&n| n != core) {
        soc::cpu_power_off_prepare(other);
    }
    let irqs = gic::init_distributor();
    let gicr = match gic::init_cpu(mpidr, core) {
        Ok(rd) => rd,
        Err(()) => panic!("no GIC redistributor for MPIDR {mpidr:#x}"),
    };
    println!("[main] GICv3: {irqs} interrupt lines, cpu{core} redistributor @ {gicr:#x}");
    psci::set_cpu(
        core,
        Cpu {
            state: State::On,
            entry: 0,
            context: 0,
            gicr,
        },
    );

    if h.payload_addr == 0 {
        panic!("bt0 loaded no payload");
    }
    #[cfg(feature = "memtest")]
    memtest::run(&h).unwrap_or_else(|e| panic!("memtest failed: {e}"));
    if h.dtb_addr != 0 {
        println!("[main] note: DTB from boot media is not used yet, generating one");
    }
    if cfg!(feature = "gmac") {
        gmac::init();
    }
    // Before usb::init(): sets the shared serdes config clock to 100 MHz.
    if cfg!(feature = "pcie") {
        pcie::init();
    }
    if cfg!(feature = "usb") {
        usb::init();
    }
    let dtb_size = build_dtb(&h);
    println!(
        "[main] DTB @ {:#x} ({dtb_size} bytes), payload memory from {:#x}",
        mem::DTB_ADDR,
        mem::PAYLOAD_RAM_BASE
    );
    println!(
        "[main] PSCI 1.0 ready; entering payload @ {:#x} in EL2",
        h.payload_addr
    );

    unsafe { write_volatile(addr_of_mut!(BOOT_STATE), BOOT_WARM) };
    enter_nonsecure(h.payload_addr, mem::DTB_ADDR as u64)
}

extern "C" fn secondary_main(mpidr: u64, core: usize) -> ! {
    el3_cpu_init();
    let mut c = psci::cpu(core);
    if c.state != State::OnPending {
        // Not requested through CPU_ON: stay out of the way.
        halt()
    }
    match gic::init_cpu(mpidr, core) {
        Ok(rd) => c.gicr = rd,
        Err(()) => {
            println!("[main] cpu{core}: no GIC redistributor");
            halt()
        }
    }
    c.state = State::On;
    psci::set_cpu(core, c);
    enter_nonsecure(c.entry, c.context)
}

/// PSCI CPU_OFF on the calling core: quiesce the GIC interface, arm the
/// hotplug controller and WFI; the PPU then powers the core down. CPU_ON
/// later restarts it from reset at `start`.
pub fn cpu_off() -> i64 {
    let core = current_core();
    if let Err(code) = psci::prepare_cpu_off(core) {
        return code;
    }
    loop {
        unsafe { asm!("dsb sy", "wfi") };
    }
}

/// SiP call from EDK2's boot manager: this boot worked, so clear bt0's
/// attempt marker and stop the watchdog it armed (see
/// `cubie_a7a_common::recovery`). Only the first call logs.
pub fn boot_confirmed() -> i64 {
    use cubie_a7a_common::recovery::{RTC_GPR, WDT0_BASE};
    use util::mmio::{read32, write32};
    if read32(RTC_GPR) != 0 {
        println!("[main] boot confirmed by the payload; watchdog off");
    }
    write32(RTC_GPR, 0);
    write32(WDT0_BASE + 0x14, 0x16aa_0000); // WDT_MODE: disable
    0
}

pub fn system_reset() -> i64 {
    println!("[main] PSCI SYSTEM_RESET");
    Uart::attach(soc::UART0_BASE).drain();
    soc::watchdog_reset()
}

pub fn system_off() -> i64 {
    println!("[main] PSCI SYSTEM_OFF");
    Uart::attach(soc::UART0_BASE).drain();
    let why = match pmic::power_off() {
        // The rails take a moment to drop.
        Ok(()) => {
            let t = ticks() + soc::TIMER_HZ;
            while ticks() < t {}
            "PMIC still on after 1 s"
        }
        Err(e) => e,
    };
    println!("[main] power-off failed ({why}), halting");
    halt()
}

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    if let Some(l) = info.location() {
        println!("[main] panic at {}:{}", l.file(), l.line());
    }
    println!("[main]   {}", info.message());
    halt()
}
