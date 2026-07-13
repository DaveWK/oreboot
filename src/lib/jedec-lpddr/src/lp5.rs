// SPDX-License-Identifier: GPL-2.0-only
//! LPDDR5 mode-register value helpers and device timings.
//! These do not issue MRWs. (Formerly the A7A `tools/cpu-training` module
//! `lpddr5`.)
//!
//! Public protocol correspondence: LiteDRAM commit
//! 74522f715d7b10163ef6f6caa09887a90874be83, litedram/init.py,
//! get_lpddr5_phy_init_sequence(). That is an FPGA implementation, not evidence
//! for any SoC's CSR encodings, DRAM-part limits, timing, feedback or entry/exit
//! flow.
//!
//! Values not here yet are listed as **pending spec** in [`crate::pending::LP5`].

pub mod cmd;
pub mod mr;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum EncodingError {
    Range,
    Step,
}

/// Seven-bit LPDDR5 CA/DQ Vref code. The protocol transfer function is
/// 10.0% + code * 0.5%. This is a code/percentage conversion, not measured voltage.
/// The exact DRAM part and operating conditions must establish allowed limits.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct VrefCode(u8);
impl VrefCode {
    pub fn new(code: u8) -> Result<Self, EncodingError> {
        if code > 127 {
            Err(EncodingError::Range)
        } else {
            Ok(Self(code))
        }
    }
    /// Percent in basis points: 1000 represents 10.00%; steps are exactly 50.
    /// Reject off-grid input rather than silently rounding a training setting.
    pub fn from_basis_points(value: u16) -> Result<Self, EncodingError> {
        if !(1000..=7350).contains(&value) {
            return Err(EncodingError::Range);
        }
        let delta = value - 1000;
        if !delta.is_multiple_of(50) {
            return Err(EncodingError::Step);
        }
        Self::new((delta / 50) as u8)
    }
    pub const fn code(self) -> u8 {
        self.0
    }
    pub const fn basis_points(self) -> u16 {
        1000 + 50 * self.0 as u16
    }
    /// Replace only the Vref field, retaining the caller's bit7 value.
    /// The caller must establish that bit7 and the selected MR are appropriate.
    pub const fn replace_field(self, prior_mr: u8) -> u8 {
        (prior_mr & 0x80) | self.0
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DramVref {
    Ca,
    DqLower,
    DqUpper,
}
impl DramVref {
    pub const fn mode_register(self) -> u8 {
        match self {
            Self::Ca => 12,
            Self::DqLower => 14,
            Self::DqUpper => 15,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum WckCkRatio {
    Two,
    Four,
}
impl WckCkRatio {
    pub const fn value(self) -> u8 {
        match self {
            Self::Two => 2,
            Self::Four => 4,
        }
    }
    /// Replace MR18 ratio(bit7) and WCK2CK-leveling(bit6) only.
    /// No synchronization, MRW, clock transition or measurement is implied.
    pub const fn replace_mr18(self, prior_mr: u8, leveling: bool) -> u8 {
        let ratio = match self {
            Self::Two => 0x80,
            Self::Four => 0,
        };
        (prior_mr & 0x3f) | ratio | if leveling { 0x40 } else { 0 }
    }
}

pub mod timing;
