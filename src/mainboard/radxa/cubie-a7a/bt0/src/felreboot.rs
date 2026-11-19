// SPDX-License-Identifier: GPL-2.0-only

//! Back to the mask ROM's FEL from a medium boot: on the "reboot efex"
//! flag, and after a failed boot from the SD card.
//!
//! The flag is a one-shot value in an RTC general purpose register, which
//! keeps its value across a WDT0 system reset. The convention is the
//! vendor firmware's (Allwinner U-Boot `rtc_set_fel_flag`, boot0
//! `rtc_probe_fel_flag`): 0x5AA5A55A in RTC GP_DATA_REG2 (0x07090108).
//! Whoever wants the next boot to land in the mask ROM's FEL mode writes it
//! and resets the SoC (WDT0); bt0 checks it first thing, before it writes
//! its own diagnostics into REG0..3 (soc.rs) and before the PMIC, DRAM or
//! any boot medium, clears it (one shot), prints one line and jumps to the
//! mask ROM's FEL entry at 0x20 (the vendor boot0's FEL_BASE for this SoC)
//! with the ROM's VBAR, SCTLR, CPSR and stack as the ROM called bt0 (saved
//! by `start`, main.rs). Every media build has it (SD/eMMC, SPI NOR); the
//! FEL build is in FEL anyway.
//!
//! The GP registers are kept while VDD_RTC stays up, so on a board whose
//! RTC domain is battery-backed a power cycle need not clear a flag that
//! nobody consumed; the flag is one-shot either way (bt0 clears it before
//! it acts on it).
//!
//! The same entry answers a failed boot from the SD card, as the vendor
//! boot0 does on a boot error: the failed attempt ends in a watchdog reset
//! (recovery.rs), and the next bt0 finds the attempt marker and goes to
//! FEL instead of trying again (main.rs `bring_up_pre`).
//!
//! [`enter_fel`] is only for that early point of a start, before
//! `ccu::init`, the PMIC and DRAM: the ROM's FEL sets its USB up on the
//! clocks the ROM left, and bt0's clock setup changes the AHB clock among
//! others (fel.rs puts it back before every return to FEL). Later code
//! that fails halts and lets the watchdog reset the SoC, which leads here
//! with the ROM's clocks.

use util::mmio::{read32, write32};

/// RTC GP_DATA_REG2.
pub const FLAG_REG: usize = crate::soc::RTC_BASE + 0x108;
/// The vendor's EFEX flag.
pub const FEL_FLAG: u32 = 0x5aa5_a55a;
/// The mask ROM's FEL entry (vendor FEL_BASE).
const ROM_FEL_ENTRY: u32 = 0x20;

/// The ROM's state as it branched to bt0 (`start` writes it before it sets
/// up its own VBAR, SCTLR and stack): VBAR, SCTLR, SP, CPSR. In .data, so
/// the .bss clear does not touch it.
#[unsafe(no_mangle)]
pub static mut ROM_ENTRY_STATE: [u32; 4] = [0xffff_ffff; 4];

/// Read the flag and, if it is set, clear it (read back until it reads 0,
/// as the vendor does). Called before anything else writes REG2.
pub fn take() -> bool {
    if read32(FLAG_REG) != FEL_FLAG {
        return false;
    }
    for _ in 0..100 {
        write32(FLAG_REG, 0);
        if read32(FLAG_REG) == 0 {
            break;
        }
    }
    true
}

/// Back to the mask ROM's FEL: watchdog off, the UART drained, then the
/// ROM's VBAR/SCTLR/CPSR/SP and a branch to its FEL entry. The caller has
/// printed why. Only before bt0's clock setup (module documentation).
pub fn enter_fel() -> ! {
    crate::recovery::stop_watchdog();
    cubie_a7a_common::uart::Uart::attach(cubie_a7a_common::uart::UART0_BASE).drain();
    let s = unsafe { core::ptr::read_volatile(core::ptr::addr_of!(ROM_ENTRY_STATE)) };
    unsafe {
        core::arch::asm!(
            "cpsid  if",
            "dsb    sy",
            "mcr    p15, 0, {vbar}, c12, c0, 0",
            "mcr    p15, 0, {zero}, c8, c7, 0",  // TLBIALL
            "mcr    p15, 0, {zero}, c7, c5, 0",  // ICIALLU
            "mcr    p15, 0, {zero}, c7, c5, 6",  // BPIALL
            "dsb    sy",
            "isb",
            "mcr    p15, 0, {sctlr}, c1, c0, 0",
            "isb",
            "msr    cpsr_fsxc, {cpsr}",
            "mov    sp, {sp}",
            "bx     {entry}",
            vbar = in(reg) s[0],
            sctlr = in(reg) s[1],
            sp = in(reg) s[2],
            cpsr = in(reg) s[3],
            entry = in(reg) ROM_FEL_ENTRY,
            zero = in(reg) 0u32,
            options(noreturn)
        )
    }
}
