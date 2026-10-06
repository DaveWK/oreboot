//! A733 specifics needed by the EL3 runtime.
//!
//! Core power control and the SPC port count follow Yixun Lan's A733 TF-A
//! port (github.com/dlan17/trusted-firmware-a, branch A733 @ f4c0b0db:
//! plat/allwinner/sun60i_a733, common/sunxi_cpu_new_ops.c), which was
//! reverse-engineered from the vendor BL31 and tested with CPU on/off.

use util::mmio::{read32, write32};

pub const CCU_BASE: usize = 0x0200_2000;
pub const WDT0_BASE: usize = 0x0205_0000;
pub const SPC_BASE: usize = 0x0205_4000;
pub const UART0_BASE: usize = 0x0250_0000;
pub const R_CPUCFG_BASE: usize = 0x0705_0000;
pub const CPUCFG_BASE: usize = 0x0800_0000;
pub const TIMER_HZ: u64 = 24_000_000;
/// UART module clock: SYS_CLK24M, fixed 24 MHz from the REFPLL.
pub const UART_CLOCK_HZ: u32 = 24_000_000;

/// Hand all peripherals, bus clocks and PLLs to the non-secure world, as
/// TF-A's `sunxi_security_setup()` does.
///
/// 14 SPC ports, 0x10 apart: dlan17 TF-A, "by REing stock ATF". The CCU
/// security switch is at 0x1f00 per SyterKit's sun60iw2 CCU header; TF-A's
/// A733 header still says 0x0f00 (copied from the H616), but on the A733
/// that offset is the SPI0 clock register (U-Boot clk_a733.c). The R_PRCM
/// switch offset is unknown for the A733 and left alone.
pub fn security_setup() {
    const SPC_PORTS: usize = 14;
    for p in 0..SPC_PORTS {
        write32(SPC_BASE + 0x10 * p + 0x4, 0xffff_ffff);
    }
    // CCU_SEC_SWITCH_REG: MBUS, bus clocks and PLLs non-secure
    write32(CCU_BASE + 0x1f00, 0x7);
}

// Per-core registers (n = core index = MPIDR.Aff1).
const fn rvbar_lo(n: usize) -> usize {
    CPUCFG_BASE + 0x4 + (n + 1) * 0x1000
}
const fn rvbar_hi(n: usize) -> usize {
    CPUCFG_BASE + 0x8 + (n + 1) * 0x1000
}
/// Bit 0: core resets into AArch64.
const fn initarch(n: usize) -> usize {
    CPUCFG_BASE + 0x1000 + n * 0x1000
}
const fn hotplug_control(n: usize) -> usize {
    R_CPUCFG_BASE + 0x200 + n * 4
}
const fn hotplug_powermode(n: usize) -> usize {
    R_CPUCFG_BASE + 0x220 + n * 4
}
/// DynamIQ PPU power status of core n (PPU frame n + 1).
const fn ppu_pwsr(n: usize) -> usize {
    R_CPUCFG_BASE + (n + 1) * 0x1000 + 0x1008
}

const GIC_WAKEUP_DISABLE: u32 = 1 << 1;
const POWER_ON: u32 = 1 << 0;
const HOTPLUG_EN: u32 = 1 << 0;
const AARCH64: u32 = 1 << 0;
const PPU_STATE_MASK: u32 = 0xf;
const PPU_STATE_OFF: u32 = 0;
const PPU_STATE_ON: u32 = 8;

fn set_bits(a: usize, b: u32) {
    write32(a, read32(a) | b);
}

fn clr_bits(a: usize, b: u32) {
    write32(a, read32(a) & !b);
}

fn ticks() -> u64 {
    let t: u64;
    unsafe { core::arch::asm!("isb", "mrs {}, cntpct_el0", out(reg) t) };
    t
}

fn wait_ppu(n: usize, state: u32) -> Result<(), &'static str> {
    let deadline = ticks() + TIMER_HZ / 10; // 100 ms
    while read32(ppu_pwsr(n)) & PPU_STATE_MASK != state {
        if ticks() > deadline {
            return Err(if state == PPU_STATE_ON {
                "core did not reach PPU ON"
            } else {
                "core is not in PPU OFF"
            });
        }
    }
    Ok(())
}

/// Power up core `n` so that it starts in AArch64 EL3 at `entry`
/// (TF-A `sunxi_cpu_on()` plus the RVBAR programming from
/// `plat_setup_psci_ops()`).
pub fn cpu_power_on(n: usize, entry: usize) -> Result<(), &'static str> {
    write32(rvbar_lo(n), entry as u32);
    write32(rvbar_hi(n), (entry as u64 >> 32) as u32);
    set_bits(initarch(n), AARCH64);

    wait_ppu(n, PPU_STATE_OFF)?;
    set_bits(hotplug_powermode(n), POWER_ON);
    set_bits(hotplug_control(n), HOTPLUG_EN);
    let r = wait_ppu(n, PPU_STATE_ON);
    clr_bits(hotplug_control(n), HOTPLUG_EN);
    clr_bits(hotplug_powermode(n), POWER_ON);
    clr_bits(hotplug_control(n), GIC_WAKEUP_DISABLE);
    r
}

/// Prepare the calling core `n` to power down at its next WFI
/// (TF-A `sunxi_cpu_off()`).
pub fn cpu_power_off_prepare(n: usize) {
    clr_bits(hotplug_powermode(n), POWER_ON);
    set_bits(hotplug_control(n), GIC_WAKEUP_DISABLE);
}

/// Reset through WDT0: the soft-reset register first (SyterKit), then the
/// watchdog armed with its shortest timeout (U-Boot sun55i/A733 driver).
pub fn watchdog_reset() -> ! {
    const KEY: u32 = 0x16aa_0000;
    write32(WDT0_BASE + 0x08, KEY | 1);
    let t = ticks() + TIMER_HZ / 100;
    while ticks() < t {}
    write32(WDT0_BASE + 0x10, (read32(WDT0_BASE + 0x10) & !0x3) | KEY | 1);
    write32(WDT0_BASE + 0x14, KEY | 1);
    write32(WDT0_BASE + 0x0c, (0x0a57 << 1) | 1);
    loop {
        unsafe { core::arch::asm!("wfi") };
    }
}
