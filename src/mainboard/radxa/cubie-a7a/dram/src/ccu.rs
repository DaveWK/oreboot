//! DRAM clocks and resets in the A733 CCU (A733 User Manual 4.1.6) and the
//! system-control resistor calibration bits the DRAM pads use.

use crate::hw::Hw;

const CCU: usize = 0x0200_2000;
const PLL_DDR_CTRL: usize = CCU + 0x020;
const NSI_CLK: usize = CCU + 0x580;
const NSI_BGR: usize = CCU + 0x584;
const MBUS_CLK: usize = CCU + 0x588;
const DRAM0_CLK: usize = CCU + 0xc00;
const DRAM0_BGR: usize = CCU + 0xc0c;

// PLL_DDR_CTRL_REG
const PLL_EN: u32 = 1 << 31;
const PLL_LDO_EN: u32 = 1 << 30;
const PLL_LOCK_ENABLE: u32 = 1 << 29;
const PLL_LOCK: u32 = 1 << 28;
const PLL_OUTPUT_GATE: u32 = 1 << 27;
/// Bit 24 is reserved in the manual; the vendor init always sets it (it is
/// set at reset on the A7A, 0x4130_4700 before init).
const PLL_BIT24: u32 = 1 << 24;
const PLL_P0_MASK: u32 = 7 << 20;
const PLL_N_MASK: u32 = 0xff << 8;
const PLL_M1: u32 = 1 << 1;

// Module clock registers (NSI, MBUS, DRAM0): gate, config update, source.
const CLK_GATING: u32 = 1 << 31;
const NSI_RST: u32 = 1 << 30;
const CLK_UPD: u32 = 1 << 27;
const CLK_SEL_MASK: u32 = 7 << 24;
const fn clk_sel(s: u32) -> u32 {
    s << 24
}
const CLK_DIV_MASK: u32 = 0x1f;
// Bus gating/reset registers.
const BGR_RST: u32 = 1 << 16;
const BGR_GATING: u32 = 1 << 0;

/// MBUS and NSI from PERI0_600M (MBUS_CLK_SEL 1, NSI_CLK_SEL 3).
const MBUS_SRC_PERI0_600M: u32 = 1;
const NSI_SRC_PERI0_600M: u32 = 3;
/// DRAM0_CLK = PLL_DDR / 4 (the PHY's reference; the PHY PLL makes the
/// DRAM clock from it).
const DRAM0_CLK_DIV: u32 = 4;

/// MCTL_COM +0x10 bits [13], [10:9] and [8]: the vendor init sets [8] and
/// clears the others around the SMC glitch workaround, then swaps them back.
const MCTL_COM_CTRL: usize = 0x0a02_0010;

/// Program PLL_DDR to `mhz` (24 MHz steps: N = mhz / 24, M1 = P0 = 1) and
/// wait for lock. Returns the rate in MHz.
pub fn set_pll_ddr<H: Hw>(hw: &mut H, mhz: u32) -> u32 {
    let n = mhz / 24;
    let v = hw.read32(PLL_DDR_CTRL);
    hw.write32(
        PLL_DDR_CTRL,
        (v & !(PLL_P0_MASK | PLL_N_MASK | PLL_M1)) | PLL_BIT24 | ((n - 1) << 8),
    );
    let v = hw.read32(PLL_DDR_CTRL);
    hw.write32(PLL_DDR_CTRL, v | PLL_EN | PLL_LDO_EN);
    hw.write32(PLL_DDR_CTRL, v | PLL_EN | PLL_LDO_EN | PLL_LOCK_ENABLE);
    hw.udelay(20);
    // The manual's 0.1 ms lock time; the vendor init polls without limit.
    while hw.read32(PLL_DDR_CTRL) & PLL_LOCK == 0 {}
    hw.write32(
        PLL_DDR_CTRL,
        v | PLL_EN | PLL_LDO_EN | PLL_LOCK_ENABLE | PLL_OUTPUT_GATE,
    );
    24 * n
}

/// Put the DRAM controller, MBUS and NSI through reset and clock them:
/// PLL_DDR at `mhz`, DRAM0_CLK = PLL_DDR / 4, MBUS and NSI from PERI0_600M.
pub fn sys_init<H: Hw>(hw: &mut H, mhz: u32) {
    // Gate MBUS, NSI and DRAM0 and hold them in reset.
    hw.clrset32(MBUS_CLK, CLK_GATING, CLK_UPD);
    hw.clr32(NSI_BGR, BGR_GATING);
    hw.clr32(NSI_BGR, BGR_RST);
    hw.clr32(DRAM0_BGR, BGR_RST);
    hw.clr32(DRAM0_BGR, BGR_GATING);
    hw.set32(DRAM0_BGR, BGR_RST);
    hw.set32(DRAM0_BGR, BGR_GATING);
    hw.clrset32(MCTL_COM_CTRL, 0x2600, 0x0100);
    // (The vendor init calls the secure monitor's glitch workaround here; it
    // has nothing to do without a secure monitor.)
    hw.clrset32(MCTL_COM_CTRL, 0x0100, 0x2600);
    hw.clr32(PLL_DDR_CTRL, PLL_EN);
    hw.clrset32(DRAM0_CLK, CLK_GATING, CLK_UPD);
    hw.udelay(5);

    set_pll_ddr(hw, mhz);

    hw.clrset32(DRAM0_CLK, CLK_DIV_MASK, (DRAM0_CLK_DIV - 1) | CLK_UPD);
    hw.clrset32(DRAM0_CLK, CLK_SEL_MASK, clk_sel(0) | CLK_UPD);
    hw.set32(DRAM0_BGR, BGR_RST);
    hw.set32(DRAM0_BGR, BGR_GATING);
    hw.clrset32(
        MBUS_CLK,
        CLK_SEL_MASK,
        CLK_GATING | CLK_UPD | clk_sel(MBUS_SRC_PERI0_600M),
    );
    hw.set32(NSI_BGR, BGR_RST);
    hw.set32(NSI_BGR, BGR_GATING);
    hw.clrset32(
        NSI_CLK,
        CLK_SEL_MASK,
        CLK_GATING | NSI_RST | CLK_UPD | clk_sel(NSI_SRC_PERI0_600M),
    );
    hw.set32(DRAM0_CLK, CLK_GATING | CLK_UPD);
    hw.udelay(5);
}

/// SYS_CFG +0x160/+0x168: [8] enables the pad resistor calibration and
/// [7:0] of +0x168 clear its trim (as in mainline U-Boot's A523 driver,
/// which clears [5:0]).
pub fn pad_calibration<H: Hw>(hw: &mut H) {
    hw.set32(0x0300_0160, 1 << 8);
    hw.clr32(0x0300_0168, 0xff);
}
