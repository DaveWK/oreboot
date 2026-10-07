//! Clock control (CCU and CPU PLLs). Port of SyterKit's sun60iw2
//! `sunxi_clk_init()`; the DRAM PLL is left to the vendor DRAM library.

use util::mmio::{read32, write32};

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
const PLL_REF_CTRL: usize = CCU_BASE;
/// PLL_REF from the 26 MHz crystal as the vendor firmware leaves it
/// (0xf867_5f00 under Linux): 26 MHz * 96 / 104 = 24 MHz.
const PLL_REF_26M_FACTORS: u32 = (103 << 16) | (95 << 8);
const PLL_REF_FACTOR_MASK: u32 = (0x7f << 16) | (0xff << 8) | (1 << 1);
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

/// PLL multiplier for `mhz` from the CPU PLL reference, rounded down so a
/// 24 MHz board never overshoots a frequency characterised at 26 MHz.
fn cpu_pll_n(hosc_khz: u32, mhz: u32) -> u32 {
    mhz * 1000 / hosc_khz
}

const CPU_L_PLL: usize = CPU_PLL_CFG_BASE + 0x1000;
const CPU_B_PLL: usize = CPU_PLL_CFG_BASE + 0x2000;
const DSU_PLL: usize = CPU_PLL_CFG_BASE + 0x3000;

/// CPU_L (A55), CPU_B (A76) and DSU PLLs to ~1 GHz / ~750 MHz, then switch
/// the clusters over to them. These run at the PMIC's 0.8 V reset voltage;
/// [`set_cpu_clocks`] raises them once the rails are up.
fn set_pll_cpux(hosc_khz: u32) {
    // SyterKit's targets: multiples of 24 MHz, or of 26 MHz otherwise.
    let (cpu, dsu) = if hosc_khz == 26_000 { (1014, 780) } else { (1008, 744) };
    for (addr, mhz) in [(CPU_L_PLL, cpu), (CPU_B_PLL, cpu), (DSU_PLL, dsu)] {
        enable_pll(addr, 0, (480 * 1000_u32).div_ceil(hosc_khz), 0, 0);
        set_pll(addr, 0, cpu_pll_n(hosc_khz, mhz), 0, 0);
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

/// Retune running CPU PLLs (frequencies in MHz). The caller must already
/// have raised the rails for the new operating point.
pub fn set_cpu_clocks(cpu_l: u32, cpu_b: u32, dsu: u32) {
    let hosc = hosc_khz();
    for (addr, mhz) in [(DSU_PLL, dsu), (CPU_L_PLL, cpu_l), (CPU_B_PLL, cpu_b)] {
        set_pll(addr, 0, cpu_pll_n(hosc, mhz), 0, 0);
        // set_pll() parked the cluster on the oscillator; back to its PLL.
        clr_set_bits(addr + 0x1c, (0x7 << 24) | (0x3 << 16), 3 << 24);
        udelay(20);
    }
}

/// Current CPU_L, CPU_B and DSU frequencies in MHz, from the PLL factors.
pub fn cpu_clocks() -> [u32; 3] {
    let hosc = hosc_khz();
    [CPU_L_PLL, CPU_B_PLL, DSU_PLL].map(|a| {
        let v = read32(a);
        let n = (v >> 8) & 0xff;
        let p = ((v >> 16) & 0xf) + 1;
        let m = (((v >> 20) & 0x3) + 1) * ((v & 0xf) + 1);
        hosc * n / (p * m) / 1000
    })
}

pub fn init() {
    for r in PLL_REGS {
        set_bits(CCU_BASE + r, PLL_FREF_SEL);
    }
    set_pll_cpux(hosc_khz());
    // APB1 from the 24 MHz oscillator, M = 1.
    clr_set_bits(APB1_CLK_REG, (0x3 << 24) | 0x1f, 0);
    // NSI (bus interconnect) at 600 MHz like the vendor boot0: divider 1
    // first, then source PERI0 600M. SyterKit's divider 6 left it at
    // 100 MHz, and the DRAM library only changes the source.
    clr_set_bits(NSI_CLK_REG, NSI_GATE | 0x1f, NSI_UPD);
    while read32(NSI_CLK_REG) & NSI_UPD != 0 {}
    clr_set_bits(NSI_CLK_REG, 0x7 << 24, NSI_SEL_PERI0_600M | NSI_GATE | NSI_UPD);
    while read32(NSI_CLK_REG) & NSI_UPD != 0 {}
}

/// APB_UART from sys24M (mux 0), M = 1, so the UARTs run at 24 MHz no
/// matter what the BROM left there.
/// Make sure PLL_REF, the source of SYS_CLK24M (UART, APB, generic timer),
/// runs at 24 MHz before the UART is set up for 24 MHz; the vendor firmware
/// runs it this way. Returns the value the BROM left.
///
/// The output gate is never cleared, in case SYS_CLK24M already runs from
/// it. No timer delays: the timer may run from the clock being fixed.
pub fn ref_pll_init() -> u32 {
    let found = read32(PLL_REF_CTRL);
    if hosc_khz() != 26_000 {
        return found;
    }
    let on = PLL_EN | PLL_LDO_EN | PLL_LOCK_EN | PLL_LOCK | PLL_OUTPUT_GATE;
    if found & on == on && found & PLL_REF_FACTOR_MASK == PLL_REF_26M_FACTORS {
        return found;
    }
    let mut v = (found & !PLL_REF_FACTOR_MASK) | PLL_REF_26M_FACTORS | PLL_LDO_EN;
    write32(PLL_REF_CTRL, v);
    spin(10_000);
    v |= PLL_EN | PLL_LOCK_EN;
    write32(PLL_REF_CTRL, v);
    for _ in 0..1_000_000 {
        if read32(PLL_REF_CTRL) & PLL_LOCK != 0 {
            break;
        }
    }
    write32(PLL_REF_CTRL, v | PLL_OUTPUT_GATE);
    spin(10_000);
    found
}

fn spin(n: u32) {
    for _ in 0..n {
        core::hint::spin_loop();
    }
}

pub fn apb_uart_clk() -> u32 {
    read32(APB_UART_CLK_REG)
}

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
    let [l, b, d] = cpu_clocks();
    println!(
        "[bt0] CPU_L {l} MHz, CPU_B {b} MHz, DSU {d} MHz; PERI0 {:08x} NSI {:08x} MBUS {:08x}",
        read32(CCU_BASE + 0x0a0),
        read32(NSI_CLK_REG),
        read32(CCU_BASE + 0x588)
    );
}
