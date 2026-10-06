//! X-Powers AXP8191 PMIC (as used with the A733), runtime address 0x36.
//! Register map and voltage step tables follow SyterKit's axp8191.c; the
//! LDO enable bits agree with mainline's AXP318W regulator description.

use crate::twi::{Error, Twi};

pub const ADDR: u8 = 0x36;

// NOTE: SyterKit names register 0x0e AXP8191_CHIP_ID and compares it to
// AXP8191_IC_TYPE (0x03); we do the same read but only report the value.
const REG_PROBE: u8 = 0x0e;
const PROBE_EXPECTED: u8 = 0x03;
const DCDC_ON_OFF_CTL1: u8 = 0x10;
const DCDC_ON_OFF_CTL2: u8 = 0x11;
const DC2OUT_VOL: u8 = 0x13;
const DC9OUT_VOL: u8 = 0x1a;
const LDO_ON_OFF_CTL1: u8 = 0x20;
const LDO_ON_OFF_CTL2: u8 = 0x21;

/// (min mV, max mV, step mV)
type Range = (u32, u32, u32);

struct Rail {
    name: &'static str,
    vol_reg: u8,
    vol_mask: u8,
    en_reg: u8,
    en_bit: u8,
    ranges: &'static [Range],
}

const fn rail(
    name: &'static str,
    vol_reg: u8,
    vol_mask: u8,
    en_reg: u8,
    en_bit: u8,
    ranges: &'static [Range],
) -> Rail {
    Rail {
        name,
        vol_reg,
        vol_mask,
        en_reg,
        en_bit,
        ranges,
    }
}

const LDO: &[Range] = &[(500, 3400, 100)];

const LOW: Range = (500, 1200, 10);
const MID: Range = (1220, 1540, 20);

static RAILS: &[Rail] = &[
    rail("dcdc1", 0x12, 0x1f, DCDC_ON_OFF_CTL1, 0, &[(1000, 3800, 100)]),
    rail("dcdc2", 0x13, 0x7f, DCDC_ON_OFF_CTL1, 1, &[LOW, MID]),
    rail("dcdc3", 0x14, 0x7f, DCDC_ON_OFF_CTL1, 2, &[LOW, MID]),
    rail("dcdc4", 0x15, 0x7f, DCDC_ON_OFF_CTL1, 3, &[LOW, MID]),
    rail(
        "dcdc5",
        0x16,
        0x7f,
        DCDC_ON_OFF_CTL1,
        4,
        &[LOW, MID, (1800, 2400, 20), (2440, 2760, 100)],
    ),
    rail(
        "dcdc6",
        0x17,
        0x7f,
        DCDC_ON_OFF_CTL1,
        5,
        &[LOW, MID, (1800, 2400, 20), (2440, 2760, 40)],
    ),
    rail("dcdc7", 0x18, 0x7f, DCDC_ON_OFF_CTL1, 6, &[LOW, (1220, 1840, 20)]),
    rail("dcdc8", 0x19, 0x7f, DCDC_ON_OFF_CTL1, 7, &[LOW, MID, (1900, 3400, 100)]),
    rail("dcdc9", 0x1a, 0x7f, DCDC_ON_OFF_CTL2, 0, &[LOW, MID, (1900, 3400, 100)]),
    rail("bldo1", 0x2a, 0x1f, LDO_ON_OFF_CTL1, 6, LDO),
    rail("cldo2", 0x30, 0x1f, LDO_ON_OFF_CTL2, 4, LDO),
    rail("cldo5", 0x33, 0x1f, LDO_ON_OFF_CTL2, 7, LDO),
];

/// LDOs the vendor configuration expects on for EDK2's devices. Radxa's
/// device trees and U-Boot `[power_sply]` set no voltage for them (the
/// rails run at the PMIC's factory setting), so these are only switched on,
/// and only when the programmed voltage is the one the consumers need.
const BOARD_LDOS: [(&str, u32, &str); 3] = [
    ("bldo1", 1800, "PCIe 1.8 V, PE bank I/O"),
    ("cldo2", 1800, "HDMI 1.8 V"),
    ("cldo5", 1800, "SERDES 1.8 V, SD 1.8 V signalling"),
];

/// Selector value for `mv` in a piecewise-linear step table. Values in a
/// gap between ranges round down to the top of the lower range.
fn selector(ranges: &[Range], mv: u32) -> u32 {
    let mut base = 0;
    for &(min, max, step) in ranges {
        let mv_c = mv.max(min);
        if mv_c <= max {
            return base + (mv_c - min) / step;
        }
        base += (max - min) / step + 1;
        // gap to the next range: clamp to this range's max
        if let Some(next) = ranges.iter().find(|r| r.0 > max) {
            if mv < next.0 {
                return base - 1;
            }
        }
    }
    base - 1
}

/// Inverse of `selector`: millivolts for selector value `sel`.
fn millivolts(ranges: &[Range], sel: u32) -> u32 {
    let mut base = 0;
    for &(min, max, step) in ranges {
        let n = (max - min) / step + 1;
        if sel < base + n {
            return min + (sel - base) * step;
        }
        base += n;
    }
    let &(_, max, _) = ranges.last().unwrap();
    max
}

pub struct Axp<'a> {
    twi: &'a Twi,
}

impl<'a> Axp<'a> {
    pub fn new(twi: &'a Twi) -> Self {
        Self { twi }
    }

    /// Port of `pmu_axp8191_init()`: turn on DCDC4 and enable DVM on
    /// DCDC2..DCDC9.
    pub fn init(&self) -> Result<(), Error> {
        let id = self.twi.read_reg(ADDR, REG_PROBE)?;
        println!(
            "[bt0] AXP8191 @0x{ADDR:02x}: reg 0x{REG_PROBE:02x} = 0x{id:02x}{}",
            if id == PROBE_EXPECTED { "" } else { " (unexpected)" }
        );
        let v = self.twi.read_reg(ADDR, DCDC_ON_OFF_CTL1)?;
        self.twi.write_reg(ADDR, DCDC_ON_OFF_CTL1, v | 0x08)?;
        for reg in DC2OUT_VOL..=DC9OUT_VOL {
            let v = self.twi.read_reg(ADDR, reg)?;
            self.twi.write_reg(ADDR, reg, v | 0x80)?;
        }
        Ok(())
    }

    /// Switch on the LDOs in `BOARD_LDOS` without changing their voltage.
    /// A rail whose programmed voltage differs from the expected one is
    /// reported and left alone rather than driven into its consumers.
    pub fn enable_board_ldos(&self) {
        for (name, want_mv, what) in BOARD_LDOS {
            let rail = RAILS.iter().find(|r| r.name == name).unwrap();
            let regs = (
                self.twi.read_reg(ADDR, rail.vol_reg),
                self.twi.read_reg(ADDR, rail.en_reg),
            );
            let (Ok(vol), Ok(en)) = regs else {
                println!("[bt0] PMIC: {name}: register read failed");
                continue;
            };
            let mv = millivolts(rail.ranges, (vol & rail.vol_mask) as u32);
            let on = en & (1 << rail.en_bit) != 0;
            if mv != want_mv {
                println!(
                    "[bt0] PMIC: {name} ({what}) is set to {mv} mV, expected {want_mv} mV; left {}",
                    if on { "on" } else { "off" }
                );
                continue;
            }
            if on {
                println!("[bt0] PMIC: {name} ({what}) on at {mv} mV");
                continue;
            }
            match self.twi.write_reg(ADDR, rail.en_reg, en | 1 << rail.en_bit) {
                Ok(()) => {
                    println!("[bt0] PMIC: {name} ({what}) switched on at {mv} mV");
                }
                Err(e) => {
                    println!("[bt0] PMIC: {name}: enable failed: {e:?}");
                }
            }
        }
    }

    /// Set rail `name` to `mv` (0 = leave voltage) and switch it on/off
    /// (`None` = leave the enable bit), like SyterKit's `axp_set_vol()`.
    pub fn set_voltage(&self, name: &str, mv: u32, on: Option<bool>) -> Result<(), &'static str> {
        let rail = RAILS
            .iter()
            .find(|r| r.name == name)
            .ok_or("unknown rail")?;
        let rd = |reg| self.twi.read_reg(ADDR, reg).map_err(|_| "i2c read");
        let wr = |reg, v| self.twi.write_reg(ADDR, reg, v).map_err(|_| "i2c write");
        if mv > 0 {
            let sel = selector(rail.ranges, mv) as u8 & rail.vol_mask;
            wr(rail.vol_reg, (rd(rail.vol_reg)? & !rail.vol_mask) | sel)?;
        }
        if let Some(on) = on {
            let v = rd(rail.en_reg)?;
            let bit = 1 << rail.en_bit;
            wr(rail.en_reg, if on { v | bit } else { v & !bit })?;
        }
        Ok(())
    }
}
