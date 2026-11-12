// SPDX-License-Identifier: GPL-2.0-only
//! The AXP318W (AXP8191) PMIC's hardware over-temperature power-off.
//!
//! The PMIC compares its die temperature with two warning levels
//! (REG 54H[1:0]) and, with REG 53H[3] set, powers the board off when the
//! die exceeds level 2. That needs no software at run time. The reset
//! defaults already enable it at level 2 = 125 °C (REG 54H = 0b01); bt0
//! lowers the levels to 105/115 °C, makes sure detection (REG 54H[2]) and
//! the power-off (REG 53H[3]) are on, reads both back, reports whether the
//! last power-off was a die over-temperature (REG 51H[5], kept until the
//! next power-on reset) and reads the die temperature once (Tdie ADC, REG
//! 67H/68H).
//!
//! This is the PMIC's own die temperature, not the SoC's: it trips when the
//! PMIC itself overheats (board ambient, regulator load). The A733 thermal
//! sensor's shutdown threshold only raises an interrupt and is handled
//! separately.

/// Power-off sources of the last power-off (REG 51H), read only.
pub const SHUTDOWN_SOURCE: u8 = 0x51;
/// REG 51H[5]: the last power-off was a die over-temperature.
pub const SOURCE_DIE_OTP: u8 = 1 << 5;
/// Power-off source enables 2 (REG 53H).
pub const POWEROFF_CTL2: u8 = 0x53;
/// REG 53H[3]: power off when the die exceeds warning level 2.
pub const POWEROFF_DIE_OTP: u8 = 1 << 3;
/// Discharge and over-temperature settings (REG 54H).
pub const DIE_OTP_CTL: u8 = 0x54;
/// REG 54H[2]: die temperature detection.
pub const DIE_DETECT: u8 = 1 << 2;
/// REG 54H[1:0]: warning levels.
pub const LEVEL_MASK: u8 = 0b11;
/// Tdie ADC: REG 67H bit 7 enable, bits 5:0 data[13:8]; REG 68H data[7:0].
pub const TDIE_HI: u8 = 0x67;
pub const TDIE_LO: u8 = 0x68;
pub const TDIE_ENABLE: u8 = 1 << 7;

/// REG 54H[1:0] warning levels (level 1 / level 2, °C).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Levels {
    L105_115 = 0,
    L115_125 = 1,
    L125_135 = 2,
    L135_145 = 3,
}

impl Levels {
    pub const fn from_bits(b: u8) -> Levels {
        match b & LEVEL_MASK {
            0 => Levels::L105_115,
            1 => Levels::L115_125,
            2 => Levels::L125_135,
            _ => Levels::L135_145,
        }
    }
    /// Level 2, the power-off threshold, in °C.
    pub const fn power_off_c(self) -> u32 {
        105 + 10 * (self as u32) + 10
    }
}

/// The levels bt0 sets: power-off at 115 °C die temperature.
pub const TARGET: Levels = Levels::L105_115;

/// What [`enable`] found, wrote and read back.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Report {
    /// REG 51H as found.
    pub source: u8,
    /// REG 53H and REG 54H before, as written and as read back.
    pub ctl2: [u8; 3],
    pub otp: [u8; 3],
    /// Levels read back.
    pub levels: Levels,
    /// Detection, power-off and the target levels all read back as set.
    pub armed: bool,
    /// Tdie ADC raw (14 bits) and in m°C, when the reads worked.
    pub tdie_raw: Option<u16>,
    pub tdie_mc: Option<i32>,
}

impl Report {
    /// The last power-off was a die over-temperature.
    pub const fn last_off_was_otp(&self) -> bool {
        self.source & SOURCE_DIE_OTP != 0
    }
}

/// Die temperature in m°C from the 14-bit Tdie ADC code:
/// temp = (721 * 5 - ADC) / 1.91 / 5 °C = (3605 - ADC) / 9.55 °C.
pub const fn tdie_mc(raw: u16) -> i32 {
    (3605 - raw as i32) * 100_000 / 955
}

/// Arm the die over-temperature power-off at [`TARGET`] through `rd`/`wr`
/// (PMIC register read/write) and report. `delay_us` waits for the Tdie
/// conversion. A failed register access stops with that error; nothing
/// is written before every register to be changed has been read.
pub fn enable<E>(
    mut rd: impl FnMut(u8) -> Result<u8, E>,
    mut wr: impl FnMut(u8, u8) -> Result<(), E>,
    mut delay_us: impl FnMut(u32),
) -> Result<Report, E> {
    let source = rd(SHUTDOWN_SOURCE)?;
    let ctl2_before = rd(POWEROFF_CTL2)?;
    let otp_before = rd(DIE_OTP_CTL)?;
    let ctl2 = ctl2_before | POWEROFF_DIE_OTP;
    let otp = (otp_before & !LEVEL_MASK) | DIE_DETECT | TARGET as u8;
    // Thresholds and detection first, then the power-off enable.
    if otp != otp_before {
        wr(DIE_OTP_CTL, otp)?;
    }
    if ctl2 != ctl2_before {
        wr(POWEROFF_CTL2, ctl2)?;
    }
    let otp_back = rd(DIE_OTP_CTL)?;
    let ctl2_back = rd(POWEROFF_CTL2)?;
    let levels = Levels::from_bits(otp_back);
    let armed = ctl2_back & POWEROFF_DIE_OTP != 0 && otp_back & DIE_DETECT != 0 && levels == TARGET;

    // One die temperature reading (the ADC enable is left as found).
    let hi = rd(TDIE_HI)?;
    if hi & TDIE_ENABLE == 0 {
        wr(TDIE_HI, hi | TDIE_ENABLE)?;
        delay_us(2_000);
    }
    let hi2 = rd(TDIE_HI)?;
    let lo = rd(TDIE_LO)?;
    let raw = ((hi2 as u16 & 0x3f) << 8) | lo as u16;
    if hi & TDIE_ENABLE == 0 {
        wr(TDIE_HI, hi)?;
    }
    Ok(Report {
        source,
        ctl2: [ctl2_before, ctl2, ctl2_back],
        otp: [otp_before, otp, otp_back],
        levels,
        armed,
        tdie_raw: Some(raw),
        tdie_mc: Some(tdie_mc(raw)),
    })
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use std::{vec, vec::Vec};

    struct Pmic {
        regs: [u8; 256],
        writes: Vec<(u8, u8)>,
        /// Writes to these registers are ignored (read-only in a fault).
        stuck: Vec<u8>,
        fail_at: Option<u8>,
    }

    impl Pmic {
        /// The AXP318W reset defaults of the registers concerned.
        fn reset() -> Self {
            let mut regs = [0u8; 256];
            regs[POWEROFF_CTL2 as usize] = 0b1010_1011; // REG 53H reset value
            regs[DIE_OTP_CTL as usize] = 0b0000_1101; // discharge, detect, 01
            regs[TDIE_LO as usize] = 0x26; // with 0x0d: 0x0d26 = 3366, 25 °C
            regs[TDIE_HI as usize] = 0x0d;
            Self {
                regs,
                writes: vec![],
                stuck: vec![],
                fail_at: None,
            }
        }
        fn run(&mut self) -> Result<Report, u8> {
            let p = core::cell::RefCell::new(self);
            enable(
                |r| {
                    let p = p.borrow();
                    if p.fail_at == Some(r) {
                        return Err(r);
                    }
                    Ok(p.regs[r as usize])
                },
                |r, v| {
                    let mut p = p.borrow_mut();
                    p.writes.push((r, v));
                    if !p.stuck.contains(&r) {
                        p.regs[r as usize] = v;
                    }
                    Ok(())
                },
                |_| {},
            )
        }
    }

    #[test]
    fn from_reset_defaults_the_levels_drop_to_105_115_and_read_back() {
        let mut p = Pmic::reset();
        let r = p.run().unwrap();
        assert!(r.armed);
        assert_eq!(r.levels, Levels::L105_115);
        assert_eq!(r.levels.power_off_c(), 115);
        assert_eq!(r.otp, [0x0d, 0x0c, 0x0c]);
        // REG 53H already had the power-off on: not written.
        assert_eq!(r.ctl2, [0xab, 0xab, 0xab]);
        assert!(!p.writes.iter().any(|w| w.0 == POWEROFF_CTL2));
        // Discharge (REG 54H[3]) untouched.
        assert_eq!(p.regs[DIE_OTP_CTL as usize] & 0x08, 0x08);
        assert!(!r.last_off_was_otp());
    }

    #[test]
    fn a_disabled_power_off_and_detection_are_switched_on_thresholds_first() {
        let mut p = Pmic::reset();
        p.regs[POWEROFF_CTL2 as usize] = 0b1010_0011;
        p.regs[DIE_OTP_CTL as usize] = 0b0000_1011;
        let r = p.run().unwrap();
        assert!(r.armed);
        let order: Vec<u8> = p.writes.iter().map(|w| w.0).take(2).collect();
        assert_eq!(order, [DIE_OTP_CTL, POWEROFF_CTL2]);
        assert_eq!(p.regs[POWEROFF_CTL2 as usize], 0b1010_1011);
        assert_eq!(p.regs[DIE_OTP_CTL as usize], 0b0000_1100);
    }

    #[test]
    fn a_write_that_does_not_stick_is_not_armed() {
        let mut p = Pmic::reset();
        p.stuck.push(DIE_OTP_CTL);
        let r = p.run().unwrap();
        assert!(!r.armed);
        assert_eq!(r.levels, Levels::L115_125);
    }

    #[test]
    fn the_last_power_off_source_is_reported() {
        let mut p = Pmic::reset();
        p.regs[SHUTDOWN_SOURCE as usize] = SOURCE_DIE_OTP;
        assert!(p.run().unwrap().last_off_was_otp());
    }

    #[test]
    fn a_read_error_stops_before_any_write() {
        let mut p = Pmic::reset();
        p.fail_at = Some(DIE_OTP_CTL);
        assert_eq!(p.run(), Err(DIE_OTP_CTL));
        assert!(p.writes.is_empty());
    }

    #[test]
    fn the_die_temperature_is_converted_and_the_adc_enable_put_back() {
        let mut p = Pmic::reset();
        let r = p.run().unwrap();
        assert_eq!(r.tdie_raw, Some(0x0d26));
        assert_eq!(r.tdie_mc, Some(25_026));
        assert_eq!(p.regs[TDIE_HI as usize] & TDIE_ENABLE, 0);
        assert_eq!(tdie_mc(3605), 0);
        // 115 °C is about code 2507.
        assert!((114_500..115_500).contains(&tdie_mc(2507)));
    }
}
