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
pub const RTC_BASE: usize = 0x0709_0000;
pub const CPU_SUBSYS_CTRL_BASE: usize = 0x0800_0000;
pub const CPU_PLL_CFG_BASE: usize = 0x0887_0000;

/// Reset vector base address for CPU n after a (warm) reset, low/high word:
/// CPUCFG + 0x4/0x8 + (n + 1) * 0x1000 (A733 user manual 5.1.6:
/// CLU0_RVBARADDRn_L/H; same in U-Boot's SUNXI_RVBAR_ADDRESS and dlan17's
/// TF-A). SyterKit's old sun60iw2 header said 0x0800_0040, copied from the
/// A523; that is not used.
pub const RVBAR_LO_CPU0: usize = CPU_SUBSYS_CTRL_BASE + 0x1004;
pub const RVBAR_HI_CPU0: usize = CPU_SUBSYS_CTRL_BASE + 0x1008;
/// CLU0_CPU0_CTRL_REG: bit 0 AA64nAA32, sampled at CPU0 reset.
pub const CPU0_CTRL: usize = CPU_SUBSYS_CTRL_BASE + 0x1000;

const SOC_VER_REG: usize = SYSCTRL_BASE + 0x24;
const PLL_LDO_REG: usize = CPU_SUBSYS_CTRL_BASE + 0x144;

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

pub use cubie_a7a_common::time::{ticks, udelay, TIMER_HZ};

pub fn time_us() -> u64 {
    ticks() / (TIMER_HZ / 1_000_000)
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

/// RTC general purpose registers 0..3 as a diagnostic channel: they keep
/// their value across a watchdog reset, and the vendor firmware leaves all
/// eight at 0, so a boot whose UART never worked can be read back from the
/// next OS (devmem 0x07090100..0x0709010c). 0: PLL_REF_CTRL as the BROM left
/// it, 1: APB_UART_CLK as the BROM left it, 2: XO_CTRL0, 3: progress.
const DIAG_GPR: usize = RTC_BASE + 0x100;

pub fn diag(idx: usize, v: u32) {
    write32(DIAG_GPR + 4 * idx, v);
}

/// Progress marker in diagnostic register 3: 0xa7a0_00NN.
pub fn stage(n: u32) {
    diag(3, 0xa7a0_0000 | n);
}

pub fn xo_ctrl0() -> u32 {
    read32(RTC_BASE + 0x160)
}

/// Silicon revision: 0 = A, 1 = B, ...
pub fn soc_version() -> u32 {
    read32(SOC_VER_REG) & 0x7
}

/// Raw DCXO status field, XO_CTRL0_REG[15:14].
pub fn xo_freq_field() -> u32 {
    (read32(RTC_BASE + 0x160) >> 14) & 0x3
}

/// DCXO frequency in kHz from XO_CTRL0_REG[15:14] (A733 user manual
/// 9.1.7.16: 00 = 24M, 01 = 19.2M, 10 = 26M, 11 = 24M). The Cubie A7A has a
/// 26 MHz crystal. Only the CPU PLLs are computed from it; peripherals use
/// the fixed 24 MHz REFPLL. (SyterKit tested bit 15 alone, which reads the
/// 0b11 = 24 MHz encoding as 26 MHz.)
pub fn hosc_khz() -> u32 {
    match xo_freq_field() {
        0b01 => 19_200,
        0b10 => 26_000,
        _ => 24_000,
    }
}

pub fn hosc_mhz() -> u32 {
    hosc_khz() / 1000
}

/// Early SoC setup, as the vendor SPL leaves it on the Cubie A7A.
///
/// SyterKit's `board_common_init()` treats SYS_CFG 0x24 = 1 as revision B and
/// then lowers the PLL LDO to 0xa706_0025 and rewrites the GPIO power mode
/// (0x0115_5550). This board's chip reads 0x101 there, yet the vendor SPL
/// only ever writes 0xa707_0025 (it has no other value) and leaves the power
/// mode to Linux. Do the same.
pub fn board_common_init(_version: u32) {
    // Written twice like the vendor.
    write32(PLL_LDO_REG, 0xa707_0025);
    write32(PLL_LDO_REG, 0xa707_0025);
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
