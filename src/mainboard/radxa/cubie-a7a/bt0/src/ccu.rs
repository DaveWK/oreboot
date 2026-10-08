//! Clock control wrappers over bounded production A733 control logic.
//! Peripheral bus settling uses the same bounded production timer logic.
use crate::soc::{clr_set_bits, hosc_khz, CCU_BASE, CPU_PLL_CFG_BASE, R_PRCM_BASE};
use util::mmio::{read32, write32};
#[path = "ccu_control.rs"]
mod control;
pub use control::Error;
struct Native;
impl control::Hw for Native {
    fn read32(&mut self, a: usize) -> u32 {
        read32(a)
    }
    fn write32(&mut self, a: usize, v: u32) {
        write32(a, v)
    }
    fn ticks(&mut self) -> u64 {
        crate::soc::ticks()
    }
}
const APB_UART_CLK_REG: usize = CCU_BASE + 0x538;
const NSI_CLK_REG: usize = CCU_BASE + 0x580;
const CPU_L_PLL: usize = CPU_PLL_CFG_BASE + 0x1000;
const CPU_B_PLL: usize = CPU_PLL_CFG_BASE + 0x2000;
const DSU_PLL: usize = CPU_PLL_CFG_BASE + 0x3000;
// Bus clock gating / reset registers used by bt0.
pub const UART0_BGR: usize = CCU_BASE + 0xe00;
pub const SMHC0_CLK: usize = CCU_BASE + 0xd00;
pub const SMHC0_BGR: usize = CCU_BASE + 0xd0c;
pub const R_TWI0_BGR: usize = R_PRCM_BASE + 0x19c;

pub fn ref_pll_init() -> Result<u32, Error> {
    control::reference(&mut Native, hosc_khz())
}
pub fn init() -> Result<(), Error> {
    control::init(&mut Native, hosc_khz())
}
pub fn delay_us(us: u32) -> Result<(), Error> {
    control::delay(&mut Native, us)
}
pub fn set_cpu_clocks(l: u32, b: u32, d: u32) -> Result<(), Error> {
    control::retune(&mut Native, hosc_khz(), [l, b, d])
}
/// Factor-derived MHz; physical measurement and current mux must be checked
/// separately. Used only after successful clock init/retune by current callers.
pub fn cpu_clocks() -> [u32; 3] {
    let hosc = hosc_khz();
    [CPU_L_PLL, CPU_B_PLL, DSU_PLL].map(|a| {
        let v = read32(a);
        let n = (v >> 8) & 255;
        let p = ((v >> 16) & 15) + 1;
        let m = (((v >> 20) & 3) + 1) * ((v & 15) + 1);
        hosc * n / (p * m) / 1000
    })
}
pub fn apb_uart_clk() -> u32 {
    read32(APB_UART_CLK_REG)
}

pub fn uart_clock_24m() {
    clr_set_bits(APB_UART_CLK_REG, (0x7 << 24) | 0x1f, 0);
}

/// Ungate a peripheral's bus clock and release its reset.
pub fn bus_enable(bgr: usize) -> Result<(), Error> {
    control::bus_enable(&mut Native, bgr)
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
