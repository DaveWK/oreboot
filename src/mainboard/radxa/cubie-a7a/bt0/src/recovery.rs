// SPDX-License-Identifier: GPL-2.0-only
//! Boot-attempt tracking and the boot watchdog
//! (see `cubie_a7a_common::recovery`).
//!
//! bt0 arms WDT0 first thing and marks every boot attempt in an RTC
//! register; whatever fails or hangs before EDK2's boot manager confirms
//! the boot ends in a watchdog reset with the marker still set. The next
//! bt0 finds it before it sets up the bus clocks, the PMIC or DRAM: an SD
//! boot then returns to the mask ROM's FEL (main.rs `bring_up_pre`,
//! felreboot.rs), the eMMC and SPI NOR boots report it and try again.

use bootlog::progress;
use cubie_a7a_common::recovery;
use util::mmio::{read32, write32};

const WDT_CTRL: usize = recovery::WDT0_BASE + 0x0c;
const WDT_CFG: usize = recovery::WDT0_BASE + 0x10;
const WDT_MODE: usize = recovery::WDT0_BASE + 0x14;
const WDT_KEY: u32 = 0x16aa_0000;
/// WDT_MODE interval 0xb: 512k cycles of DCXO/750, ~15 s with the A7A's
/// 26 MHz crystal.
const WDT_INTERVAL_16S: u32 = 0xb << 4;

/// The attempt marker: RTC GP_DATA_REG6, kept across watchdog resets.
struct RtcGpr;

impl progress::Marker for RtcGpr {
    fn read(&mut self) -> u32 {
        read32(recovery::RTC_GPR)
    }
    fn write(&mut self, v: u32) {
        write32(recovery::RTC_GPR, v)
    }
}

/// True if the last boot set the attempt marker and never confirmed.
/// Clears the marker, so the start after that one is a normal attempt
/// again (shared `bootlog::progress`).
pub fn previous_boot_failed() -> bool {
    progress::previous_boot_failed(&mut RtcGpr, recovery::ATTEMPT)
}

/// Mark this boot as in progress and arm WDT0 to reset the whole system if
/// nothing confirms it in time.
pub fn begin_attempt() {
    progress::begin_attempt(&mut RtcGpr, recovery::ATTEMPT);
    arm_watchdog();
}

/// Reset the whole system in ~16 s unless kicked or stopped. bt0 arms it
/// first thing, so a hang before the UART works still ends in a reset.
pub fn arm_watchdog() {
    write32(WDT_CFG, WDT_KEY | 1); // reset the whole system
    write32(WDT_MODE, WDT_KEY | WDT_INTERVAL_16S | 1);
    kick();
}

/// Disable WDT0 (before going back to the mask ROM's FEL).
pub fn stop_watchdog() {
    write32(WDT_MODE, WDT_KEY);
}

/// Restart the watchdog count, e.g. before handing over to the next stage.
pub fn kick() {
    write32(WDT_CTRL, (0x0a57 << 1) | 1);
}
