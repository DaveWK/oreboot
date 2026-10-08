//! General purpose ADC, used once to read the DRAM-variant strap.
//!
//! GPADC0 sits at 0x0252_1000 on the A733 (vendor BSP device trees for
//! Linux 5.15/6.6/6.18, alexcaoys/allwinner-bsp; SyterKit's old sun60iw2
//! header said 0x0200_9000, the A523 address). Its registers are the
//! sun20i/A523 layout (mainline sun20i-gpadc-iio); the sequence follows
//! SyterKit's sun252iw1 DRAM-select code. Clocks: GPADC0_24M at CCU 0xfc0
//! (mux 0 = sys24M), bus gate/reset at 0xfc4 (BSP ccu-sun60iw2.c).

use util::mmio::{read32, write32};

use crate::ccu;
use crate::soc::{clr_set_bits, set_bits, time_us, udelay, CCU_BASE};

const GPADC_BASE: usize = 0x0252_1000;
const GP_SR_CON: usize = GPADC_BASE + 0x00;
const GP_CTRL: usize = GPADC_BASE + 0x04;
const GP_CS_EN: usize = GPADC_BASE + 0x08;
/// Data-ready interrupt enable/status (GP_DATA_INTC/INTS; 0x20/0x30 are
/// the low-threshold compare registers).
const GP_DATA_INTC: usize = GPADC_BASE + 0x28;
const GP_DATA_INTS: usize = GPADC_BASE + 0x38;
const GP_CH_DATA: usize = GPADC_BASE + 0x80;

const GPADC0_24M_CLK: usize = CCU_BASE + 0xfc0;
const GPADC0_BGR: usize = CCU_BASE + 0xfc4;

/// 12-bit converter referenced to 1.8 V.
const FULL_SCALE_MV: u32 = 1800;

/// Average a few conversions of `channel`, in millivolts. `None` if the
/// converter never produces data.
pub fn read_mv(channel: u32) -> Option<u32> {
    // module clock: gate on, SYS_CLK24M, M = 1
    write32(GPADC0_24M_CLK, 1 << 31);
    ccu::bus_enable(GPADC0_BGR).unwrap_or_else(|error| crate::clock_failure(error));

    // acquire time and sample-rate divider as in the vendor sequence
    clr_set_bits(GP_SR_CON, 0xffff_ffff, (0x1df << 16) | 0x164);
    // GP_MODE 0b10 = continuous conversion, auto calibration, then ADC_EN
    clr_set_bits(GP_CTRL, 0x3 << 18, (0x2 << 18) | (1 << 23));
    set_bits(GP_CTRL, 1 << 16);
    set_bits(GP_CS_EN, 1 << channel);
    udelay(50);
    set_bits(GP_DATA_INTC, 1 << channel);

    const SAMPLES: u32 = 8;
    let mut sum = 0;
    for _ in 0..SAMPLES {
        write32(GP_DATA_INTS, 1 << channel);
        let t0 = time_us();
        while read32(GP_DATA_INTS) & (1 << channel) == 0 {
            if time_us() - t0 > 10_000 {
                return None;
            }
        }
        sum += read32(GP_CH_DATA + 4 * channel as usize) & 0xfff;
    }
    // leave the converter off again
    clr_set_bits(GP_CTRL, 1 << 16, 0);
    Some(sum / SAMPLES * FULL_SCALE_MV / 0xfff)
}
