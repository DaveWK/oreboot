//! DRAM supply rails.

use crate::hw::{Error, Hw, Rail};
use crate::para::Para;

/// Switch on VDD2H/VDD2L (1080 mV, or 1200 mV with tpr0[16]) and VDDQ
/// (tpr3[15:8] x 10 mV, default 500 mV). Returns VDDQ in mV.
pub fn set_rails<H: Hw>(hw: &mut H, p: &Para) -> Result<u32, Error> {
    let vddq = match ((p.tpr3() >> 8) & 0xff) * 10 {
        0 => 500,
        v => v,
    };
    let vdd2 = if p.tpr0() & (1 << 16) != 0 {
        1200
    } else {
        1080
    };
    let ok = hw.set_voltage(Rail::Vdd2h, vdd2)
        & hw.set_voltage(Rail::Vdd2l, vdd2)
        & hw.set_voltage(Rail::Vddq, vddq);
    if !ok {
        hw.log(format_args!("[dram] PMIC refused a DRAM rail\n"));
        return Err(Error::Power);
    }
    Ok(vddq)
}
