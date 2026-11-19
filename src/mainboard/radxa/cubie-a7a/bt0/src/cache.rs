//! MMU and D-cache while fastboot runs (feature `fastboot`, not the DRAM
//! lab mode): the identity map of `cubie_a7a_fastboot::mmu::FASTBOOT_MAP`
//! (SRAM and DRAM write-back, peripherals device, the EHCI schedule's MiB
//! non-cacheable), so the CPU's FIFO copies and the bundle checks run
//! against cached DRAM instead of word-by-word uncached accesses.
//!
//! [`off`] cleans and invalidates the whole D-cache to the point of
//! coherency and turns MMU and caches off: before the hand-over to main,
//! before any return to FEL (the ROM and the host then see SRAM and DRAM
//! as written), and it is what every such path calls first. The only DMA
//! master that touches cached memory is EHCI1 (`fetch:usb0`); it gets
//! [`clean_inv`] before and [`inv`] after a transfer into a cached buffer,
//! [`clean`] before one out of it. MUSB, SMHC and SPI0 are PIO.
use core::arch::{asm, global_asm};
use core::sync::atomic::{AtomicBool, Ordering};

use cubie_a7a_fastboot::mmu;

/// The first-level table: 16 KiB, 16 KiB aligned, in DRAM above the
/// DRAM extension (0x5900_0000 + 1 MiB).
const TABLE: usize = 0x5a00_0000;

static ON: AtomicBool = AtomicBool::new(false);

pub fn on() -> bool {
    ON.load(Ordering::Relaxed)
}

// Clean and invalidate every data cache level to the LoC by set/way (ARM
// ARM B2.2.7), with the D-cache already off for new allocations (the
// caller's pushes are cleaned out by the loop itself; nothing below
// touches memory between SCTLR.C off and the end of the loop).
global_asm!(
    r#"
    .section .text.cache, "ax"
    .arm
    .global bt0_dcache_off_clean_all
bt0_dcache_off_clean_all:
    push    {{r4-r11, lr}}
    mrc     p15, 0, r0, c1, c0, 0
    bic     r0, r0, #(1 << 2)           @ SCTLR.C off
    mcr     p15, 0, r0, c1, c0, 0
    isb
    dmb
    mrc     p15, 1, r0, c0, c0, 1       @ CLIDR
    ands    r3, r0, #0x7000000
    mov     r3, r3, lsr #23             @ LoC * 2
    beq     5f
    mov     r10, #0
1:  add     r2, r10, r10, lsr #1
    mov     r1, r0, lsr r2
    and     r1, r1, #7
    cmp     r1, #2
    blt     4f
    mcr     p15, 2, r10, c0, c0, 0      @ CSSELR
    isb
    mrc     p15, 1, r1, c0, c0, 0       @ CCSIDR
    and     r2, r1, #7
    add     r2, r2, #4
    ldr     r4, =0x3ff
    ands    r4, r4, r1, lsr #3
    clz     r5, r4
    ldr     r7, =0x7fff
    ands    r7, r7, r1, lsr #13
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
    mcr     p15, 2, r10, c0, c0, 0
    dsb     sy
    isb
    pop     {{r4-r11, pc}}
    .ltorg
    "#
);

unsafe extern "C" {
    fn bt0_dcache_off_clean_all();
}

fn sctlr() -> u32 {
    let v: u32;
    unsafe { asm!("mrc p15, 0, {}, c1, c0, 0", out(reg) v) };
    v
}

/// Build the map and turn MMU, D-cache and I-cache on. DRAM must be up.
pub fn on_fastboot() {
    if on() {
        return;
    }
    // SAFETY: DRAM bt0 tested and owns; 16 KiB at a 16 KiB boundary.
    let t = unsafe { core::slice::from_raw_parts_mut(TABLE as *mut u32, 4096) };
    if mmu::table(&mmu::FASTBOOT_MAP, t).is_err() {
        return;
    }
    unsafe {
        asm!(
            "dsb sy",
            "mcr p15, 0, {z}, c8, c7, 0",  // TLBIALL
            "mcr p15, 0, {z}, c7, c5, 0",  // ICIALLU
            "mcr p15, 0, {z}, c7, c5, 6",  // BPIALL
            "mcr p15, 0, {z}, c2, c0, 2",  // TTBCR = 0 (TTBR0 only)
            "mcr p15, 0, {t}, c2, c0, 0",  // TTBR0 (walks non-cacheable)
            "mcr p15, 0, {d}, c3, c0, 0",  // DACR: domain 0 client
            "dsb sy",
            "isb",
            z = in(reg) 0u32,
            t = in(reg) TABLE as u32,
            d = in(reg) 1u32,
        );
        // M, C, Z, I on; A (alignment) stays off.
        let s = sctlr() | 1 | (1 << 2) | (1 << 11) | (1 << 12);
        asm!("mcr p15, 0, {}, c1, c0, 0", "isb", in(reg) s);
    }
    ON.store(true, Ordering::Relaxed);
}

/// Clean everything to memory, then MMU and D-cache off (I-cache stays on,
/// as bt0 runs it). Safe to call when already off.
pub fn off() {
    if !on() {
        return;
    }
    unsafe {
        bt0_dcache_off_clean_all();
        let s = sctlr() & !1;
        asm!(
            "mcr p15, 0, {s}, c1, c0, 0",
            "isb",
            "mcr p15, 0, {z}, c8, c7, 0",
            "mcr p15, 0, {z}, c7, c5, 0",
            "mcr p15, 0, {z}, c7, c5, 6",
            "dsb sy",
            "isb",
            s = in(reg) s,
            z = in(reg) 0u32,
        );
    }
    ON.store(false, Ordering::Relaxed);
}

fn line() -> usize {
    let ctr: u32;
    unsafe { asm!("mrc p15, 0, {}, c0, c0, 1", out(reg) ctr) };
    4 << ((ctr >> 16) & 0xf)
}

fn by_line(start: usize, len: usize, op: impl Fn(usize)) {
    if !on() || len == 0 {
        return;
    }
    let (s, e) = mmu::lines(start, len, line());
    let mut a = s;
    while a < e {
        op(a);
        a += line();
    }
    unsafe { asm!("dsb sy") };
}

/// Write dirty lines of the range back (before a DMA master reads it).
pub fn clean(start: usize, len: usize) {
    by_line(start, len, |a| unsafe {
        asm!("mcr p15, 0, {}, c7, c10, 1", in(reg) a)
    });
}

/// Write back and drop the range's lines (before a DMA master writes it:
/// no dirty line can be evicted over the incoming data).
pub fn clean_inv(start: usize, len: usize) {
    by_line(start, len, |a| unsafe {
        asm!("mcr p15, 0, {}, c7, c14, 1", in(reg) a)
    });
}

/// Drop the range's lines (after a DMA master wrote it). The range must
/// have been [`clean_inv`]ed before the transfer, so partial lines at its
/// ends hold no other dirty data by then.
pub fn inv(start: usize, len: usize) {
    by_line(start, len, |a| unsafe {
        asm!("mcr p15, 0, {}, c7, c6, 1", in(reg) a)
    });
}
