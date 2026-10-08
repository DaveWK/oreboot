//! DRAM supply rails.

use crate::hw::{Error, Hw, Rail};
use crate::para::Para;

/// Switch on VDD2H (1080 mV, or 1200 mV with tpr0[16]), VDD2L at
/// 900 mV and VDDQ (tpr3[15:8] x 10 mV, default 500 mV).
/// A7A schematic V1.10 sheets4/5 routes ELDO1 to VCC09-VDD2L and gives
/// 0.9 V in the LPDDR5 rail table; it is separate from DCDC7/VDD2H.
/// The inherited VDD2H/VDDQ parameter policy is not a measured part limit.
/// Returns VDDQ in mV.
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
        & hw.set_voltage(Rail::Vdd2l, 900)
        & hw.set_voltage(Rail::Vddq, vddq);
    if !ok {
        hw.log(format_args!("[dram] PMIC refused a DRAM rail\n"));
        return Err(Error::Power);
    }
    Ok(vddq)
}
