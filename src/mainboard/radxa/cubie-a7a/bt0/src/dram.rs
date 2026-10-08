//! LPDDR5 bring-up: parameter set selection, the init with fallbacks, and
//! the quick checks around it.
//!
//! The init itself runs either through the open driver (crate
//! `cubie-a7a-dram`, the default: no vendor code, the PHY's firmware loaded
//! as data) or, with the `dramlib` feature, through Allwinner's closed
//! library as firmware de106497 did.

use util::mmio::read32;

use cubie_a7a_dram::sets::{ParamSet, CXMT_4G, HYNIX_4G, PARAM_SETS, SAFE_4G};

use crate::axp::Axp;
use crate::soc;

#[cfg(not(feature = "dramlib"))]
mod open;
#[cfg(feature = "dramlib")]
mod vendor;

#[cfg(not(feature = "dramlib"))]
use open as backend;
#[cfg(feature = "dramlib")]
use vendor as backend;

#[cfg(feature = "dramlib")]
pub use vendor::report_previous_trace;

#[cfg(not(feature = "dramlib"))]
pub fn report_previous_trace() {}

/// DDR-PARA-SEL-ADC strap voltages from the A7A V1.10 schematic (sheet 6)
/// and the set each one selects. The SK hynix 4 GB board leaves the
/// divider unpopulated (pulled up to 1.8 V); 1569 mV is "DDR PARA 7".
const ADC_STRAPS: [(u32, usize); 6] = [
    (163, CXMT_4G),
    (382, 1),
    (608, 2),
    (811, 3),
    (1569, HYNIX_4G),
    (1800, HYNIX_4G),
];
const DDR_SEL_ADC_CHANNEL: u32 = 1;

/// Choose the parameter set like vendor boot0: read the PH16/PH17 strap
/// GPIOs (both 0 on every A7A variant in the schematic) and the GPADC
/// strap voltage, and take the nearest table entry.
pub fn select() -> &'static ParamSet {
    use crate::gpio::{self, Port, Pull};
    for pin in [16, 17] {
        gpio::set_function(Port::H, pin, 0);
        gpio::set_pull(Port::H, pin, Pull::Up);
    }
    soc::udelay(100);
    let gpios = (gpio::read(Port::H, 17) as u32) << 1 | gpio::read(Port::H, 16) as u32;
    for pin in [16, 17] {
        gpio::set_pull(Port::H, pin, Pull::None);
    }
    if gpios != 0 {
        println!(
            "[bt0] DRAM: strap GPIOs PH17:PH16 = {gpios:02b}, not an A7A set; using the ADC only"
        );
    }
    let Some(mv) = crate::gpadc::read_mv(DDR_SEL_ADC_CHANNEL) else {
        println!(
            "[bt0] DRAM: GPADC did not convert; assuming {}",
            PARAM_SETS[CXMT_4G].name
        );
        return &PARAM_SETS[CXMT_4G];
    };
    let &(strap, idx) = ADC_STRAPS
        .iter()
        .min_by_key(|(s, _)| s.abs_diff(mv))
        .unwrap();
    println!(
        "[bt0] DRAM: strap {mv} mV (nearest {strap} mV) -> {}",
        PARAM_SETS[idx].name
    );
    &PARAM_SETS[idx]
}

static mut PMIC: Option<Axp<'static>> = None;

/// The PMIC handed to [`init`], for whatever runs after DRAM.
pub fn pmic() -> Option<&'static Axp<'static>> {
    unsafe { (*core::ptr::addr_of!(PMIC)).as_ref() }
}

/// Bring up DRAM with `set`. For the 4 GB sets, retry with the other
/// vendor's set and then the conservative 1800 MHz set if init fails.
/// Returns the DRAM size in MiB and the clock in MHz of the set that
/// trained, or `None`.
pub fn init(pmic: Option<Axp<'static>>, set: &'static ParamSet) -> Option<(u32, u32)> {
    unsafe { *core::ptr::addr_of_mut!(PMIC) = pmic };
    let fallback: &[&'static ParamSet] = if core::ptr::eq(set, &PARAM_SETS[CXMT_4G]) {
        &[&PARAM_SETS[HYNIX_4G], &PARAM_SETS[SAFE_4G]]
    } else if core::ptr::eq(set, &PARAM_SETS[HYNIX_4G]) {
        &[&PARAM_SETS[CXMT_4G], &PARAM_SETS[SAFE_4G]]
    } else {
        &[]
    };
    for s in core::iter::once(set).chain(fallback.iter().copied()) {
        println!(
            "[bt0] DRAM: init with {} parameters ({} MHz)",
            s.name, s.para[0]
        );
        if let Some(mib) = backend::init_one(&s.para) {
            return Some((mib, s.para[0]));
        }
        println!("[bt0] DRAM: {} failed", s.name);
    }
    None
}

/// Quick sanity check: unique pattern at power-of-two strides, then read
/// back, catching dead or aliased address lines.
pub fn test(base: usize, size: usize) -> Result<(), (usize, u32, u32)> {
    let pattern = |a: usize| (a as u32) ^ 0x5a5a_a5a5;
    let mut off = 0x1000;
    let mut addrs = [0usize; 32];
    let mut n = 0;
    addrs[n] = base;
    n += 1;
    while off < size && n < addrs.len() {
        addrs[n] = base + off - 4;
        n += 1;
        off <<= 1;
    }
    for &a in &addrs[..n] {
        util::mmio::write32(a, pattern(a));
    }
    for &a in &addrs[..n] {
        let v = util::mmio::read32(a);
        if v != pattern(a) {
            return Err((a, pattern(a), v));
        }
    }
    Ok(())
}

/// Clock, reset and PHY state around DRAM init. Only registers the library
/// itself touches before training, each read once.
pub fn dump_clocks(when: &str) {
    const REGS: [(&str, usize); 10] = [
        ("PLL_REF", 0x0200_2000),
        ("PLL_DDR", 0x0200_2020),
        ("PLL_DDR_PAT0", 0x0200_2028),
        ("NSI_CLK", 0x0200_2580),
        ("NSI_BGR", 0x0200_2584),
        ("MBUS_CLK", 0x0200_2588),
        ("DRAM_CLK", 0x0200_2c00),
        ("DRAM_BGR", 0x0200_2c0c),
        ("R_PRCM+250", 0x0701_0250),
        ("SYS+160", 0x0300_0160),
    ];
    print!("[dram] {when}:");
    for (name, addr) in REGS {
        print!(" {name}={:08x}", read32(addr));
    }
    println!();
}
