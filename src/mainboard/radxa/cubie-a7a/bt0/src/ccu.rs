//! Clock control (CCU and CPU PLLs). Port of SyterKit's sun60iw2
//! `sunxi_clk_init()`; the DRAM PLL is left to the vendor DRAM library.

use util::mmio::read32;

use crate::soc::{
    clr_bits, clr_set_bits, hosc_khz, set_bits, udelay, CCU_BASE, CPU_PLL_CFG_BASE, R_PRCM_BASE,
};

// PLL control registers whose reference select (bit 24) is switched over.
const PLL_REGS: [usize; 13] = [
    0x020, // DDR
    0x0a0, // PERI0
    0x0c0, // PERI1
    0x0e0, // GPU0
    0x120, // VIDEO0
    0x140, // VIDEO1
    0x160, // VIDEO2
    0x220, // VE0
    0x240, // VE1
    0x260, // AUDIO0
    0x280, // AUDIO1
    0x2a0, // NPU
    0x2e0, // DE
];
const PLL_FREF_SEL: u32 = 1 << 24;

const PLL_EN: u32 = 1 << 31;
const PLL_LDO_EN: u32 = 1 << 30;
const PLL_LOCK_EN: u32 = 1 << 29;
const PLL_LOCK: u32 = 1 << 28;
const PLL_OUTPUT_GATE: u32 = 1 << 27;
const PLL_UPDATE: u32 = 1 << 26;
const PLL_FACTOR_MASK: u32 = (0x3 << 20) | (0xf << 16) | (0xff << 8) | 0xf;

const APB1_CLK_REG: usize = CCU_BASE + 0x518;
/// UART module clocks come from APB_UART (BSP ccu-sun60iw2.c), not APB1.
const APB_UART_CLK_REG: usize = CCU_BASE + 0x538;
const NSI_CLK_REG: usize = CCU_BASE + 0x580;
const NSI_GATE: u32 = 1 << 31;
const NSI_UPD: u32 = 1 << 27;
const NSI_SEL_PERI0_600M: u32 = 3 << 24;

// Bus clock gating / reset registers used by bt0.
pub const UART0_BGR: usize = CCU_BASE + 0xe00;
pub const SMHC0_CLK: usize = CCU_BASE + 0xd00;
pub const SMHC0_BGR: usize = CCU_BASE + 0xd0c;
pub const R_TWI0_BGR: usize = R_PRCM_BASE + 0x19c;
const BGR_GATE: u32 = 1 << 0;
const BGR_RESET: u32 = 1 << 16;

fn pll_factors(m0: u32, n: u32, m1: u32, p: u32) -> u32 {
    (m0 << 20) | (p << 16) | (n << 8) | m1
}

fn wait_locked(addr: usize) {
    while read32(addr) & PLL_UPDATE != 0 {}
    for _ in 0..3 {
        while read32(addr) & PLL_LOCK == 0 {}
    }
}

fn enable_pll(addr: usize, m0: u32, n: u32, m1: u32, p: u32) {
    set_bits(addr, PLL_LDO_EN);
    set_bits(addr, PLL_OUTPUT_GATE);
    clr_set_bits(addr, PLL_FACTOR_MASK, pll_factors(m0, n, m1, p));
    udelay(20);
    set_bits(addr, PLL_EN);
    set_bits(addr, PLL_LOCK_EN);
    set_bits(addr, PLL_UPDATE);
    wait_locked(addr);
    udelay(20);
}

fn set_pll(addr: usize, m0: u32, n: u32, m1: u32, p: u32) {
    // Run the core from the oscillator while the PLL relocks.
    clr_bits(addr + 0x1c, 0x3 << 24);
    clr_bits(addr, PLL_OUTPUT_GATE | PLL_LOCK_EN);
    clr_set_bits(addr, PLL_FACTOR_MASK, pll_factors(m0, n, m1, p));
    udelay(20);
    set_bits(addr, PLL_LOCK_EN);
    set_bits(addr, PLL_UPDATE);
    wait_locked(addr);
    udelay(20);
    set_bits(addr, PLL_OUTPUT_GATE);
    udelay(20);
}

/// CPU_L (A55), CPU_B (A76) and DSU PLLs to ~1 GHz / ~750 MHz, then switch
/// the clusters over to them.
fn set_pll_cpux(hosc_khz: u32) {
    let n = |mhz: u32| (mhz * 1000).div_ceil(hosc_khz);
    // SyterKit's targets: multiples of 24 MHz, or of 26 MHz otherwise.
    let (cpu, dsu) = if hosc_khz == 26_000 { (1014, 780) } else { (1008, 744) };
    for (off, mhz) in [(0x1000, cpu), (0x2000, cpu), (0x3000, dsu)] {
        let addr = CPU_PLL_CFG_BASE + off;
        enable_pll(addr, 0, n(480), 0, 0);
        set_pll(addr, 0, n(mhz), 0, 0);
    }
    udelay(20);
    for off in [0x101c, 0x201c, 0x301c] {
        clr_set_bits(
            CPU_PLL_CFG_BASE + off,
            (0x7 << 24) | (0x3 << 16),
            3 << 24, // source: own PLL, divider P = 1
        );
        udelay(20);
    }
}

pub fn init() {
    for r in PLL_REGS {
        set_bits(CCU_BASE + r, PLL_FREF_SEL);
    }
    set_pll_cpux(hosc_khz());
    // APB1 from the 24 MHz oscillator, M = 1.
    clr_set_bits(APB1_CLK_REG, (0x3 << 24) | 0x1f, 0);
    // NSI (bus interconnect): divider 6 first, then source PERI0 600M.
    clr_set_bits(NSI_CLK_REG, NSI_GATE | 0x1f, 5 | NSI_UPD);
    while read32(NSI_CLK_REG) & NSI_UPD != 0 {}
    clr_set_bits(NSI_CLK_REG, 0x7 << 24, NSI_SEL_PERI0_600M | NSI_GATE | NSI_UPD);
    while read32(NSI_CLK_REG) & NSI_UPD != 0 {}
}

/// APB_UART from sys24M (mux 0), M = 1, so the UARTs run at 24 MHz no
/// matter what the BROM left there.
pub fn uart_clock_24m() {
    clr_set_bits(APB_UART_CLK_REG, (0x7 << 24) | 0x1f, 0);
}

/// Ungate a peripheral's bus clock and release its reset.
pub fn bus_enable(bgr: usize) {
    clr_bits(bgr, BGR_RESET);
    clr_bits(bgr, BGR_GATE);
    udelay(5);
    set_bits(bgr, BGR_RESET);
    set_bits(bgr, BGR_GATE);
    udelay(5);
}

pub fn dump() {
    let r = |off| read32(CPU_PLL_CFG_BASE + off);
    println!(
        "[bt0] PLL CPU_L {:08x} CPU_B {:08x} DSU {:08x} PERI0 {:08x}",
        r(0x1000),
        r(0x2000),
        r(0x3000),
        read32(CCU_BASE + 0x0a0)
    );
}
