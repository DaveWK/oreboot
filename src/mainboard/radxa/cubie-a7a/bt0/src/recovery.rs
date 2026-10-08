//! Boot-attempt tracking and the vendor SPL fallback
//! (see `cubie_a7a_common::recovery`).
//!
//! The vendor eGON SPL is linked to run at 0x47000, exactly where bt0
//! itself runs, so it is staged in SHARED_SRAM above bt0 and a small
//! position-independent stub copies it down and jumps to it with the CPU in
//! the state the BROM leaves (secure SVC, MMU off). The stub sits above the
//! staging area, so the downward copy never overwrites it.

use core::arch::global_asm;

use cubie_a7a_common::{layout, recovery};
use util::mmio::{read32, write32};

use crate::smhc::{Smhc, BLOCK_SIZE};

/// Where the BROM loads eGON images and the vendor SPL expects to run.
const SPL_BASE: usize = 0x0004_7000;
/// SHARED_SRAM (0x74000..0xf3fff) above bt0's image, bss and stack (bt0's
/// linker script ends them below 0x90000). The stub sits above the largest
/// SPL (0x4a000) and below fwlog's early buffer at 0xf0000. The copy runs
/// upwards with the destination below the source, so overlapping is safe.
const STAGING: usize = 0x0009_0000;
const STUB: usize = 0x000e_0000;
const SPL_MAX: usize = STUB - STAGING;
/// eGON header: boot_media byte (sunxi `struct boot_file_head`), written by
/// the BROM: 0 MMC0 at 8 KiB, 0x10 MMC0 at 128 KiB.
const BOOT_MEDIA: usize = 0x28;
const BOOTED_FROM_MMC0_HIGH: u8 = 0x10;

const WDT_CTRL: usize = recovery::WDT0_BASE + 0x0c;
const WDT_CFG: usize = recovery::WDT0_BASE + 0x10;
const WDT_MODE: usize = recovery::WDT0_BASE + 0x14;
const WDT_KEY: u32 = 0x16aa_0000;
/// WDT_MODE interval 0xb: 512k cycles of DCXO/750, ~15 s with the A7A's
/// 26 MHz crystal.
const WDT_INTERVAL_16S: u32 = 0xb << 4;

/// True if the last boot set the attempt marker and never confirmed.
/// Clears the marker, so a failed fallback leads back to oreboot.
pub fn previous_boot_failed() -> bool {
    let failed = read32(recovery::RTC_GPR) == recovery::ATTEMPT;
    write32(recovery::RTC_GPR, 0);
    failed
}

/// Mark this boot as in progress and arm WDT0 to reset the whole system if
/// nothing confirms it in time.
pub fn begin_attempt() {
    write32(recovery::RTC_GPR, recovery::ATTEMPT);
    arm_watchdog();
}

/// Reset the whole system in ~16 s unless kicked or stopped. bt0 arms it
/// first thing, so a hang before the UART works still ends in a reset.
pub fn arm_watchdog() {
    write32(WDT_CFG, WDT_KEY | 1); // reset the whole system
    write32(WDT_MODE, WDT_KEY | WDT_INTERVAL_16S | 1);
    kick();
}

/// Disable WDT0 (before handing over to the vendor recovery chain).
fn stop_watchdog() {
    write32(WDT_MODE, WDT_KEY);
}

/// Restart the watchdog count, e.g. before handing over to the next stage.
pub fn kick() {
    write32(WDT_CTRL, (0x0a57 << 1) | 1);
}

global_asm!(
    r#"
    .section .text.recovery_stub, "ax"
    .arm
    .global recovery_stub_start
    .global recovery_stub_end
    .balign 4
recovery_stub_start:
    // r0 = source, r1 = destination, r2 = length (multiple of 4)
1:  ldr     r3, [r0], #4
    str     r3, [r1], #4
    subs    r2, r2, #4
    bgt     1b
    mov     r0, #0
    mcr     p15, 0, r0, c12, c0, 0      // VBAR = 0, as the BROM leaves it
    mcr     p15, 0, r0, c7, c5, 0       // ICIALLU
    mcr     p15, 0, r0, c7, c5, 6       // BPIALL
    dsb     sy
    isb
    mov     r3, #0x47000
    bx      r3
recovery_stub_end:
    "#
);

unsafe extern "C" {
    static recovery_stub_start: u8;
    static recovery_stub_end: u8;
}

/// Load the vendor SPL named in the layout table and run it. Returns only
/// if the card carries none or it cannot be read.
pub fn boot_vendor_spl(sd: &Smhc, table: &layout::Table) {
    let Some(e) = table.entries[..table.count as usize]
        .iter()
        .find(|e| e.kind == layout::KIND_VENDOR_SPL)
    else {
        println!("[bt0] recovery: no vendor SPL on this card");
        return;
    };
    let len = (e.size as usize).next_multiple_of(BLOCK_SIZE);
    if len > SPL_MAX || e.offset as usize % BLOCK_SIZE != 0 {
        println!(
            "[bt0] recovery: vendor SPL entry is unusable ({} bytes)",
            e.size
        );
        return;
    }
    // SAFETY: STAGING..STAGING+SPL_MAX is unused SRAM above bt0.
    let buf = unsafe { core::slice::from_raw_parts_mut(STAGING as *mut u32, len / 4) };
    if let Err(err) = sd.read_blocks(e.offset / BLOCK_SIZE as u32, buf) {
        println!("[bt0] recovery: reading the vendor SPL failed: {err:?}");
        return;
    }
    if &buf[1..3] != [u32::from_le_bytes(*b"eGON"), u32::from_le_bytes(*b".BT0")] {
        println!("[bt0] recovery: no eGON header in the vendor SPL entry");
        return;
    }
    if e.crc32 != 0 {
        // SAFETY: same buffer, viewed as bytes.
        let bytes = unsafe { core::slice::from_raw_parts(STAGING as *const u8, e.size as usize) };
        if cubie_a7a_common::crc32(bytes) != e.crc32 {
            println!("[bt0] recovery: vendor SPL CRC mismatch");
            return;
        }
    }

    // The vendor SPL (sunxi U-Boot SPL) reads the boot source from the
    // eGON header's boot_media byte, which the BROM fills in SRAM when it
    // loads an image. MMC0_HIGH means "loaded from 128 KiB", and the SPL
    // then looks for U-Boot (the FIT that follows it on the vendor card)
    // 120 KiB further than for an 8 KiB load. The card copy holds 0
    // (MMC0, 8 KiB), so pass on the 128 KiB source explicitly. bt0's own
    // header shows what the BROM wrote for this boot.
    // SAFETY: bt0's eGON header is at SPL_BASE; the staged copy is ours.
    let brom = unsafe { core::ptr::read_volatile((SPL_BASE + BOOT_MEDIA) as *const u8) };
    let staged = unsafe { &mut *((STAGING + BOOT_MEDIA) as *mut u8) };
    let card = *staged;
    *staged = BOOTED_FROM_MMC0_HIGH;
    println!(
        "[bt0] recovery: starting the vendor SPL ({} KiB); boot_media card {card:#x}, BROM for bt0 {brom:#x}, passing {BOOTED_FROM_MMC0_HIGH:#x}",
        e.size / 1024
    );
    // The vendor chain is the known-good recovery path and boots without
    // a watchdog from the BROM; a 16 s reset could cut a slow but healthy
    // boot short and keep the board cycling. Stop WDT0 for it.
    stop_watchdog();
    crate::Uart::attach(crate::UART0_BASE).drain();
    // SAFETY: stub symbols bound a small position-independent routine;
    // STUB is unused SRAM above the staging area.
    unsafe {
        let start = &recovery_stub_start as *const u8;
        let n = (&recovery_stub_end as *const u8).offset_from(start) as usize;
        core::ptr::copy_nonoverlapping(start, STUB as *mut u8, n);
        // D-cache is off; make sure no stale I-cache line covers the stub.
        core::arch::asm!("dsb sy", "mcr p15, 0, {z}, c7, c5, 0", "isb", z = in(reg) 0u32);
        let stub: extern "C" fn(usize, usize, usize) -> ! = core::mem::transmute(STUB);
        stub(STAGING, SPL_BASE, len)
    }
}
