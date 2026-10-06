//! Allwinner A733 (sun60iw2) addresses and early SoC setup.
//!
//! Register addresses and sequences follow SyterKit's sun60iw2 support
//! (boards/radxa-cubie-a7a, drivers/clk/clk-sun60iw2.c), which in turn is
//! derived from the Allwinner BSP boot0.

use core::arch::asm;
use util::mmio::{read32, write32};

pub const PIO_BASE: usize = 0x0200_0000;
pub const CCU_BASE: usize = 0x0200_2000;
pub const WDT0_BASE: usize = 0x0205_0000;
pub const SYSCTRL_BASE: usize = 0x0300_0000;
pub const SID_BASE: usize = 0x0300_6000;
pub const SMHC0_BASE: usize = 0x0402_0000;
pub const R_PRCM_BASE: usize = 0x0701_0000;
pub const R_PIO_BASE: usize = 0x0702_5000;
pub const R_TWI0_BASE: usize = 0x0708_3000;
pub const RTC_BASE: usize = 0x0709_0000;
pub const CPU_SUBSYS_CTRL_BASE: usize = 0x0800_0000;
pub const CPU_PLL_CFG_BASE: usize = 0x0887_0000;

/// Reset vector base address for CPU n after a (warm) reset, low/high word:
/// CPUCFG + 0x4/0x8 + (n + 1) * 0x1000. U-Boot's A733 support uses
/// SUNXI_RVBAR_ADDRESS = 0x0800_1004 for CPU0 and dlan17's A733 TF-A the
/// same per-core layout. (SyterKit's old sun60iw2 header said 0x0800_0040,
/// copied from the A523; that is not used.)
pub const RVBAR_LO_CPU0: usize = CPU_SUBSYS_CTRL_BASE + 0x1004;
pub const RVBAR_HI_CPU0: usize = CPU_SUBSYS_CTRL_BASE + 0x1008;

const SOC_VER_REG: usize = SYSCTRL_BASE + 0x24;
const PLL_LDO_REG: usize = CPU_SUBSYS_CTRL_BASE + 0x144;
const GPIO_POW_MODE_REG: usize = PIO_BASE + 0x340;

#[inline]
pub fn set_bits(addr: usize, bits: u32) {
    write32(addr, read32(addr) | bits);
}

#[inline]
pub fn clr_bits(addr: usize, bits: u32) {
    write32(addr, read32(addr) & !bits);
}

#[inline]
pub fn clr_set_bits(addr: usize, clr: u32, set: u32) {
    write32(addr, (read32(addr) & !clr) | set);
}

// ---------------------------------------------------------------------------
// Arch timer. The generic counter runs from the 24 MHz oscillator domain.

pub const TIMER_HZ: u64 = 24_000_000;

#[inline]
pub fn ticks() -> u64 {
    let lo: u32;
    let hi: u32;
    // CNTPCT
    unsafe { asm!("isb", "mrrc p15, 0, {}, {}, c14", out(reg) lo, out(reg) hi) };
    ((hi as u64) << 32) | lo as u64
}

pub fn time_us() -> u64 {
    ticks() / (TIMER_HZ / 1_000_000)
}

pub fn udelay(us: u64) {
    let t1 = ticks() + us * (TIMER_HZ / 1_000_000);
    while ticks() < t1 {}
}

pub fn mdelay(ms: u64) {
    udelay(ms * 1000);
}

/// CNTFRQ is only writable from the highest exception level, which is where
/// the BROM leaves us (secure AArch32 SVC with EL3 in AArch32).
pub fn set_cntfrq() {
    unsafe { asm!("mcr p15, 0, {}, c14, c0, 0", in(reg) TIMER_HZ as u32) };
}

// ---------------------------------------------------------------------------

/// Silicon revision: 0 = A, 1 = B, ...
pub fn soc_version() -> u32 {
    read32(SOC_VER_REG) & 0x7
}

/// Raw DCXO frequency field, XO_CTRL0_REG[15:14] (24 / 19.2 / 26 MHz).
pub fn xo_freq_field() -> u32 {
    (read32(RTC_BASE + 0x160) >> 14) & 0x3
}

/// The A733 runs from either a 24 MHz or a 26 MHz crystal (26 MHz on the
/// Cubie A7A). Only the CPU PLLs are computed from it; peripherals use the
/// fixed 24 MHz REFPLL. Decoded like SyterKit (bit 15).
pub fn hosc_mhz() -> u32 {
    if read32(RTC_BASE + 0x160) & (1 << 15) != 0 {
        26
    } else {
        24
    }
}

/// Port of SyterKit's `board_common_init()` for the Cubie A7A.
pub fn board_common_init(version: u32) {
    // Revision B: GPIO bank power mode (IO voltage selection).
    if version == 1 {
        write32(GPIO_POW_MODE_REG, 0x0115_5550);
    }
    // PLL LDO voltage per silicon revision; written twice like the vendor.
    let ldo = match version {
        0 => Some(0xa707_0025),
        1 => Some(0xa706_0025),
        _ => None,
    };
    if let Some(v) = ldo {
        write32(PLL_LDO_REG, v);
        write32(PLL_LDO_REG, v);
    }
}

/// Read the 128-bit chip SID from the eFuse SRAM mirror.
pub fn chip_sid() -> [u32; 4] {
    let sram = SID_BASE + 0x200;
    [
        read32(sram),
        read32(sram + 4),
        read32(sram + 8),
        read32(sram + 12),
    ]
}

/// Reset through WDT0. First the soft-reset register used by SyterKit's
/// A7A port; if that does nothing, arm the watchdog with the shortest
/// timeout the way U-Boot's sun55i/A733 driver does (CFG 0x10, MODE 0x14).
pub fn watchdog_reset() -> ! {
    const KEY: u32 = 0x16aa_0000;
    write32(WDT0_BASE + 0x08, KEY | 1);
    mdelay(10);
    clr_set_bits(WDT0_BASE + 0x10, 0x3, KEY | 1); // reset the whole system
    write32(WDT0_BASE + 0x14, KEY | 1); // enable, 0.5 s
    write32(WDT0_BASE + 0x0c, (0x0a57 << 1) | 1); // restart the counter
    loop {
        unsafe { asm!("wfi") };
    }
}
