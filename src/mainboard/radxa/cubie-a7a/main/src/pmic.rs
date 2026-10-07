//! AXP318W (SyterKit: AXP8191) power controls on R_TWI0, for PSCI.
//!
//! bt0 already muxed PL0/PL1 and ungated R_TWI0, so only the controller is
//! reprogrammed here. REG 55H per the AXP318W datasheet V1.0 (section 6.4):
//! bit 7 software power-off, bit 6 software restart; both self-clear.

use cubie_a7a_common::{twi::Twi, R_TWI0_BASE};

const ADDR: u8 = 0x36;
const REG_POWER_CTL: u8 = 0x55;
const POWER_OFF: u8 = 1 << 7;

/// Switch every rail off. Returns only if the PMIC did not take the write.
pub fn power_off() -> Result<(), &'static str> {
    let twi = Twi::new(R_TWI0_BASE);
    let v = twi.read_reg(ADDR, REG_POWER_CTL).map_err(|_| "i2c read")?;
    twi.write_reg(ADDR, REG_POWER_CTL, v | POWER_OFF)
        .map_err(|_| "i2c write")?;
    Ok(())
}
