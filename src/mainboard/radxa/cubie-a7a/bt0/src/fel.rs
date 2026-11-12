//! USB FEL boot (feature `fel`): bt0 started by the mask ROM's FEL mode
//! over USB (xfel), for a board that cannot boot from its SD card.
//!
//! The host (scripts/fel-boot.sh in radxa-a7a-workspace) writes this build to
//! [`LINK_BASE`] and calls two entries with `xfel exec`:
//!
//! | Address           | Entry                                                |
//! |-------------------|------------------------------------------------------|
//! | `LINK_BASE + 0x40` | 1: clocks, UART, PMIC, DRAM, CPU boost; back to FEL |
//! | `LINK_BASE + 0x44` | 2: check the images the host placed, enter main     |
//!
//! Between the two the host writes oreboot main, the EDK2 FD (and a DTB, if
//! the layout has one) to their load addresses in DRAM and the image's boot
//! layout table frame to [`TABLE`]. Entry 2 runs the SD path's layout
//! checks and CRCs on them in place and hands over to main exactly as the
//! SD boot does (handoff block, watchdog, warm reset into AArch64).
//!
//! Both entries keep the mask ROM's state: callee-saved registers, sp, lr,
//! CPSR (mode, masks), SCTLR and VBAR are saved on entry and put back
//! before returning, and the D-cache is cleaned and invalidated by set/way
//! before bt0 turns it off, in case the ROM ran with it on. Nothing below
//! 0x90000 is touched (the ROM's FEL memory is in 0x47000..0x87000): image,
//! bss and stack sit at 0x90000..0xdf000 (build.rs), then [`MAILBOX`],
//! below xfel's helper area at 0xee800; the SD path's SRAM log buffer
//! (0xf0000) and vendor recovery are left out of this build. The one bus
//! clock bt0's clock setup changes that the ROM's USB could depend on
//! ([`AHB`]) is put back for FEL and set again in entry 2. The watchdog
//! runs during entry 1 (a hang resets the board, which comes back in FEL
//! because the SD card does not boot) and is stopped before returning.
//!
//! The mailbox tells the host what happened ([`MAILBOX`] layout below).

use core::arch::global_asm;

use cubie_a7a_common::{crc32, layout, uart::Uart};
use util::mmio::{read32, write32};

use crate::{recovery, soc, Bringup, UART0_BASE};

/// Where build.rs links the FEL build.
pub const LINK_BASE: usize = 0x0009_0000;

/// Host interface, 4 KiB above bt0's stack (32-bit words, little endian):
///
/// | Offset | Written by | Content                                        |
/// |--------|------------|------------------------------------------------|
/// | 0x000  | bt0        | magic `"A7AF"`                                 |
/// | 0x004  | bt0        | state, [`state`]                               |
/// | 0x008  | bt0        | DRAM size in MiB                               |
/// | 0x00c  | bt0        | DRAM clock in MHz                              |
/// | 0x010  | bt0        | the ROM's SCTLR at entry                       |
/// | 0x014  | bt0        | the ROM's CPSR at entry                        |
/// | 0x018  | bt0        | the ROM's VBAR at entry                        |
/// | 0x200  | host       | the image's 512-byte layout table frame        |
pub const MAILBOX: usize = 0x000d_f000;
const MB_MAGIC: usize = MAILBOX;
const MB_STATE: usize = MAILBOX + 0x04;
const MB_DRAM_MB: usize = MAILBOX + 0x08;
const MB_DRAM_MHZ: usize = MAILBOX + 0x0c;
const MB_ROM_SCTLR: usize = MAILBOX + 0x10;
const MB_ROM_CPSR: usize = MAILBOX + 0x14;
const MB_ROM_VBAR: usize = MAILBOX + 0x18;
/// The layout table frame the host copied from the image.
pub const TABLE: usize = MAILBOX + 0x200;
const MAGIC: u32 = u32::from_le_bytes(*b"A7AF");

/// Mailbox states. The host clears the mailbox before entry 1.
pub mod state {
    pub const INIT_RUNNING: u32 = 0x10;
    /// Entry 1 done: DRAM up, back in FEL.
    pub const DRAM_UP: u32 = 0x11;
    pub const BOOT_RUNNING: u32 = 0x20;
    /// Entry 2 accepted the images and is entering main (no return).
    pub const BOOT_HANDOVER: u32 = 0x21;
    /// Entry 2 refused and returned to FEL: entry 1 has not run.
    pub const BOOT_NO_INIT: u32 = 0xe0;
    /// ... the layout table frame is invalid.
    pub const BOOT_TABLE: u32 = 0xe1;
    /// ... an image does not match its CRC.
    pub const BOOT_CRC: u32 = 0xe2;
    /// Entry 3 (lab) running / done.
    pub const LAB_RUNNING: u32 = 0x30;
    pub const LAB_DONE: u32 = 0x31;
    /// Entry 1 halted (DRAM init or test failed) and returned to FEL.
    pub const HALTED: u32 = 0xef;
}

/// CCU+0x518, the bus clock register bt0's clock setup (ccu_control.rs
/// `init`) sets to source 0, divider 1.
const AHB: usize = soc::CCU_BASE + 0x518;
const AHB_BT0_CLEAR: u32 = (3 << 24) | 31;

static mut BRINGUP: Option<Bringup> = None;
/// The ROM's AHB value at entry 1, put back by [`halt_to_fel`] too.
static mut ROM_AHB: u32 = 0;

// The entries. The ROM calls them like a function (blx, AArch32, secure
// SVC) and expects them to return. fel_save keeps its state; it is in
// .data, so entry 1's bss clear leaves it alone.
global_asm!(
    r#"
    .section .data.fel_save, "aw"
    .balign 4
    .global fel_save
fel_save:
    .space  64                  @ sp, lr, cpsr, sctlr, vbar, r4-r11

    .section .text.entry, "ax"
    .arm
    .global start
start:
    b       fel_entry1          @ LINK_BASE + 0x40
    b       fel_entry2          @ LINK_BASE + 0x44
    b       fel_entry3          @ LINK_BASE + 0x48 (lab)

    .section .text.fel, "ax"
    .arm
fel_entry1:
    mov     r3, #1
    b       fel_common
fel_entry2:
    mov     r3, #0
    b       fel_common
fel_entry3:
    mov     r3, #2
fel_common:
    ldr     r12, =fel_save
    str     sp, [r12, #0]
    str     lr, [r12, #4]
    mrs     r0, cpsr
    str     r0, [r12, #8]
    mrc     p15, 0, r0, c1, c0, 0
    str     r0, [r12, #12]
    mrc     p15, 0, r0, c12, c0, 0
    str     r0, [r12, #16]
    add     r0, r12, #20
    stmia   r0, {{r4-r11}}
    mov     r12, r3             @ entry flag (r12: not used below)
    cpsid   if                  @ keep the ROM's mode, mask IRQ and FIQ

    @ SCTLR as bt0's SD entry sets it: MMU, alignment check and D-cache
    @ off, low vectors via VBAR, I-cache and branch prediction on.
    mrc     p15, 0, r0, c1, c0, 0
    bic     r0, r0, #0x7
    bic     r0, r0, #(1 << 13)
    orr     r0, r0, #(1 << 12)
    orr     r0, r0, #(1 << 11)
    mcr     p15, 0, r0, c1, c0, 0
    isb

    @ Clean and invalidate the data caches by set/way up to the LoC (ARM
    @ ARM B2.2.7 example): if the ROM ran with the D-cache on, FEL's
    @ writes (this image, the host's images before entry 2) reach memory.
    @ Uses r0-r11 only; no stack while the caches change.
    dmb
    mrc     p15, 1, r0, c0, c0, 1       @ CLIDR
    ands    r3, r0, #0x7000000
    mov     r3, r3, lsr #23             @ LoC * 2
    beq     5f
    mov     r10, #0                     @ level * 2
1:  add     r2, r10, r10, lsr #1
    mov     r1, r0, lsr r2
    and     r1, r1, #7                  @ cache type at this level
    cmp     r1, #2
    blt     4f                          @ no data cache
    mcr     p15, 2, r10, c0, c0, 0      @ CSSELR
    isb
    mrc     p15, 1, r1, c0, c0, 0       @ CCSIDR
    and     r2, r1, #7
    add     r2, r2, #4                  @ log2(line length)
    ldr     r4, =0x3ff
    ands    r4, r4, r1, lsr #3          @ ways - 1
    clz     r5, r4
    ldr     r7, =0x7fff
    ands    r7, r7, r1, lsr #13         @ sets - 1
2:  mov     r9, r4
3:  orr     r11, r10, r9, lsl r5
    orr     r11, r11, r7, lsl r2
    mcr     p15, 0, r11, c7, c14, 2     @ DCCISW
    subs    r9, r9, #1
    bge     3b
    subs    r7, r7, #1
    bge     2b
4:  add     r10, r10, #2
    cmp     r3, r10
    bgt     1b
5:  mov     r10, #0
    mcr     p15, 2, r10, c0, c0, 0      @ CSSELR back to L1
    dsb     sy
    mov     r0, #0
    mcr     p15, 0, r0, c8, c7, 0       @ TLBIALL
    mcr     p15, 0, r0, c7, c5, 0       @ ICIALLU
    mcr     p15, 0, r0, c7, c5, 6       @ BPIALL
    dsb     sy
    isb

    ldr     r0, =bt0_vectors
    mcr     p15, 0, r0, c12, c0, 0      @ VBAR
    ldr     sp, =_stack_top
    cmp     r12, #2
    beq     9f
    cmp     r12, #0
    beq     7f
    @ entry 1: clear .bss
    ldr     r0, =_sbss
    ldr     r1, =_ebss
    mov     r2, #0
6:  cmp     r0, r1
    strlo   r2, [r0], #4
    blo     6b
    bl      fel_init
    b       8f
7:  bl      fel_boot                    @ returns only if it refused
    b       8f
9:  bl      fel_lab
    .global fel_return
fel_return:
8:  ldr     r12, =fel_save
    ldr     r0, [r12, #16]
    mcr     p15, 0, r0, c12, c0, 0      @ the ROM's VBAR
    mov     r0, #0
    dsb     sy
    mcr     p15, 0, r0, c8, c7, 0       @ TLBIALL
    mcr     p15, 0, r0, c7, c5, 0       @ ICIALLU
    mcr     p15, 0, r0, c7, c5, 6       @ BPIALL
    dsb     sy
    isb
    ldr     r0, [r12, #12]
    mcr     p15, 0, r0, c1, c0, 0       @ the ROM's SCTLR
    isb
    ldr     r0, [r12, #8]
    msr     cpsr_fsxc, r0               @ the ROM's mode and masks
    ldr     sp, [r12, #0]
    ldr     lr, [r12, #4]
    add     r0, r12, #20
    ldmia   r0, {{r4-r11}}
    bx      lr
    .ltorg
    "#
);

unsafe extern "C" {
    static fel_save: [u32; 16];
    fn fel_return() -> !;
}

/// bt0's halt in the FEL build: instead of the watchdog reset, record
/// [`state::HALTED`] and return to FEL with the ROM's state (the stack is
/// abandoned; fel_return restores everything from fel_save), so the host
/// can still inspect the hardware after a failed init.
pub fn halt_to_fel() -> ! {
    println!(
        "[bt0] FEL: halted; back to FEL (state {:#x})",
        state::HALTED
    );
    Uart::attach(UART0_BASE).drain();
    recovery::stop_watchdog();
    let rom_ahb = unsafe { *core::ptr::addr_of!(ROM_AHB) };
    if rom_ahb != 0 {
        write32(AHB, rom_ahb);
    }
    write32(MB_STATE, state::HALTED);
    unsafe { fel_return() }
}

/// (sp, lr, cpsr, sctlr, vbar) as the ROM called us.
fn rom_state() -> [u32; 5] {
    let p = core::ptr::addr_of!(fel_save).cast::<u32>();
    core::array::from_fn(|i| unsafe { core::ptr::read_volatile(p.add(i)) })
}

/// Entry 1: everything bt0 does before the boot medium, then back to FEL.
#[unsafe(no_mangle)]
extern "C" fn fel_init() {
    let [_, _, cpsr, sctlr, vbar] = rom_state();
    write32(MB_STATE, state::INIT_RUNNING);
    write32(MB_ROM_SCTLR, sctlr);
    write32(MB_ROM_CPSR, cpsr);
    write32(MB_ROM_VBAR, vbar);
    write32(MB_MAGIC, MAGIC);
    let rom_ahb = read32(AHB);
    unsafe { *core::ptr::addr_of_mut!(ROM_AHB) = rom_ahb };

    // Halts (watchdog reset, back into FEL) if DRAM does not come up.
    let b = crate::bring_up();
    unsafe { *core::ptr::addr_of_mut!(BRINGUP) = Some(b) };

    println!(
        "[bt0] FEL: mask ROM state at entry: CPSR {cpsr:#010x} SCTLR {sctlr:#010x} VBAR {vbar:#010x}{}",
        if sctlr & 1 != 0 { " (MMU was on)" } else { "" }
    );
    let ours = read32(AHB);
    if ours != rom_ahb {
        println!("[bt0] FEL: CCU+0x518 {ours:#010x} -> {rom_ahb:#010x} (the ROM's) until entry 2");
        write32(AHB, rom_ahb);
    }
    write32(MB_DRAM_MB, b.dram_mb);
    write32(MB_DRAM_MHZ, b.dram_mhz);
    println!(
        "[bt0] FEL: DRAM up, {} ms; back to FEL. Next: images to DRAM, layout table to {TABLE:#x}, entry 2 at {:#x}",
        (soc::ticks() - b.t_start) / (soc::TIMER_HZ / 1000),
        LINK_BASE + 0x44
    );
    Uart::attach(UART0_BASE).drain();
    recovery::stop_watchdog();
    write32(MB_STATE, state::DRAM_UP);
}

/// Entry 2: check what the host placed and enter main (no return), or
/// refuse with a mailbox state and return to FEL.
#[unsafe(no_mangle)]
extern "C" fn fel_boot() {
    write32(MB_STATE, state::BOOT_RUNNING);
    let Some(b) = (unsafe { *core::ptr::addr_of!(BRINGUP) }) else {
        // No UART either: entry 1 never ran in this FEL session.
        write32(MB_STATE, state::BOOT_NO_INIT);
        return;
    };
    println!("[bt0] FEL: entry 2: checking the images in DRAM");
    // SAFETY: the mailbox is ours (build.rs keeps bt0 below it).
    let frame = unsafe { core::slice::from_raw_parts(TABLE as *const u8, 512) };
    let table = match layout::decode(frame) {
        Ok(t) => t,
        Err(error) => return refuse(state::BOOT_TABLE, "layout table frame", error),
    };
    let mut handoff = crate::handoff(&b);
    let mut main_entry = None;
    let mut bad_crc = false;
    let placed = layout::load_entries(&table, b.dram_end as u64, |e, addr, _padded| {
        let t0 = soc::time_us();
        // SAFETY: load_entries validated the extent against DRAM.
        let data = unsafe { core::slice::from_raw_parts(addr as *const u8, e.size as usize) };
        let crc = crc32(data);
        if e.crc32 != 0 && crc != e.crc32 {
            println!(
                "[bt0] FEL: kind {} at 0x{addr:08x}: CRC {crc:08x}, table {:08x}",
                e.kind, e.crc32
            );
            bad_crc = true;
            return;
        }
        println!(
            "[bt0] FEL: kind {} ({} KiB) at 0x{addr:08x}{} ({} ms)",
            e.kind,
            e.size / 1024,
            if e.crc32 != 0 { ", CRC ok" } else { "" },
            (soc::time_us() - t0) / 1000
        );
        if let Some(entry) = crate::place(&mut handoff, e, addr) {
            main_entry = Some(entry);
        }
    });
    if let Err(error) = placed {
        return refuse(state::BOOT_TABLE, "boot layout", error);
    }
    if bad_crc {
        return refuse(state::BOOT_CRC, "images", "CRC mismatch");
    }
    let Some(main_entry) = main_entry else {
        return refuse(state::BOOT_TABLE, "boot layout", "no oreboot main");
    };

    // The ROM's interrupts off, or the warm reset never happens (rmr_switch).
    quiesce_rom_interrupts();
    // As the SD boot from here: bt0's bus clock, the attempt marker and the
    // watchdog (EDK2's boot manager confirms the boot), the warm reset.
    soc::clr_set_bits(AHB, AHB_BT0_CLEAR, 0);
    recovery::begin_attempt();
    write32(MB_STATE, state::BOOT_HANDOVER);
    crate::enter_main(&b, handoff, main_entry)
}

/// Mailbox words of the lab entry (entry 3, `lab.rs`).
const MB_LAB_OPS: usize = MAILBOX + 0x20;
const MB_LAB_OUT: usize = MAILBOX + 0x24;
const MB_LAB_ERR: usize = MAILBOX + 0x28;

/// Entry 3: the host's lab command list (`lab.rs`), watchdog armed.
#[unsafe(no_mangle)]
extern "C" fn fel_lab() {
    write32(MB_STATE, state::LAB_RUNNING);
    crate::recovery::arm_watchdog();
    let (ops, n, err) = crate::lab::run();
    write32(MB_LAB_OPS, ops);
    write32(MB_LAB_OUT, n);
    write32(MB_LAB_ERR, err);
    recovery::stop_watchdog();
    write32(MB_STATE, state::LAB_DONE);
}

/// GIC-600 distributor and CPU0's redistributor SGI/PPI frame (main gic.rs).
const GICD: usize = 0x0340_0000;
const GICR0_SGI: usize = 0x0346_0000 + 0x1_0000;
const GICD_TYPER: usize = 0x004;
const GIC_ICENABLER: usize = 0x180;
const GIC_ISPENDR: usize = 0x200;
const GIC_ICPENDR: usize = 0x280;
const GICD_CTLR_RWP: u32 = 1 << 31;

/// The mask ROM runs FEL with IRQs on (CPSR.I clear, VBAR 0x17000): its USB
/// is interrupt driven, and an interrupt still pending at the GIC keeps WFI
/// from ever entering standby, which the warm reset waits for (first FEL
/// trial: "warm reset into AArch64", then nothing until the
/// watchdog reset the board back into FEL). Turn the distributor off, every
/// SPI, SGI and PPI enable and pending bit with it, and the generic timers;
/// main programs the GIC from scratch (gic::init_distributor).
fn quiesce_rom_interrupts() {
    let ctlr = read32(GICD);
    write32(GICD, 0);
    let mut reads = 0;
    while read32(GICD) & GICD_CTLR_RWP != 0 && reads < 1_000_000 {
        reads += 1;
    }
    let lines = (read32(GICD + GICD_TYPER) & 0x1f) as usize;
    let mut pending = [0u32; 4];
    let mut spis = 0;
    for n in 1..=lines {
        let p = read32(GICD + GIC_ISPENDR + 4 * n);
        if p != 0 {
            for b in 0..32 {
                if p & (1 << b) != 0 && spis < pending.len() {
                    pending[spis] = (32 * n + b) as u32;
                    spis += 1;
                }
            }
        }
        write32(GICD + GIC_ICENABLER + 4 * n, !0);
        write32(GICD + GIC_ICPENDR + 4 * n, !0);
    }
    let ppi = read32(GICR0_SGI + GIC_ISPENDR);
    write32(GICR0_SGI + GIC_ICENABLER, !0);
    write32(GICR0_SGI + GIC_ICPENDR, !0);
    unsafe {
        core::arch::asm!(
            "mcr p15, 0, {z}, c14, c2, 1", // CNTP_CTL
            "mcr p15, 0, {z}, c14, c3, 1", // CNTV_CTL
            "dsb sy",
            "isb",
            z = in(reg) 0u32,
        )
    };
    println!(
        "[bt0] FEL: GIC off for the hand-over (GICD_CTLR was {ctlr:#x}; pending SPIs {:?}, SGI/PPI {ppi:#x})",
        &pending[..spis]
    );
}

fn refuse(code: u32, what: &str, why: impl core::fmt::Debug) {
    println!("[bt0] FEL: entry 2 refused, {what}: {why:?}; back to FEL (state {code:#x})");
    Uart::attach(UART0_BASE).drain();
    write32(MB_STATE, code);
}
