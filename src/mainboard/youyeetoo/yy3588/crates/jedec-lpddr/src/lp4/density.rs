// SPDX-License-Identifier: GPL-2.0-only
//! LPDDR4/4X density from mode register 8 (JEDEC LPDDR4/4X MR8: OP[5:2] density,
//! OP[7:6] I/O width), per channel, and the total over every rank and
//! channel read.
//!
//! First written for the SpacemiT K1 bt0 (`channel_size`,
//! `ddr_get_density`): the decode and the "every rank and
//! channel reports the same MR8" rule. The MR8 reads (controller specific)
//! and the board's accepted totals stay board code.

/// Why an MR8 value or set was refused.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DensityError {
    /// OP[5:2] above 6: a density this decode does not know.
    Density,
    /// OP[7:6] neither x16 (0) nor byte mode x8 (1).
    Width,
    /// Not every rank and channel reported the same MR8.
    Mismatch,
    /// No MR8 value given.
    Empty,
}

/// Legacy generic capacity arithmetic, not a device-type or part-policy check.
/// MiB of one channel of a die with this MR8 value: 256, 384, 512, 768,
/// 1024, 1536, 2048 MiB for OP[5:2] = 0..=6 on a x16 channel, twice that in
/// byte mode (OP[7:6] = 1).
pub const fn channel_mib(mr8: u8) -> Result<u32, DensityError> {
    let size = match (mr8 >> 2) & 15 {
        0 => 256,
        1 => 384,
        2 => 512,
        3 => 768,
        4 => 1024,
        5 => 1536,
        6 => 2048,
        _ => return Err(DensityError::Density),
    };
    match mr8 >> 6 {
        0 => Ok(size),
        1 => Ok(size * 2),
        _ => Err(DensityError::Width),
    }
}

/// Total MiB over the MR8 values read from every rank and channel; they must
/// all be equal (one row/column geometry for the controller).
pub fn total_mib(mr8s: &[u8]) -> Result<u32, DensityError> {
    let first = *mr8s.first().ok_or(DensityError::Empty)?;
    let mut total = 0u32;
    for &v in mr8s {
        if v != first {
            return Err(DensityError::Mismatch);
        }
        total += channel_mib(v)?;
    }
    Ok(total)
}

/// An observed MR8 byte, before interpreting a part or admitting geometry.
/// Every bit is retained, including type/width codes a policy may refuse.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Mr8Capture {
    raw: u8,
}

impl Mr8Capture {
    pub const fn from_raw(raw: u8) -> Self {
        Self { raw }
    }
    pub const fn raw(self) -> u8 {
        self.raw
    }
    pub const fn type_code(self) -> u8 {
        self.raw & 3
    }
    pub const fn density_code(self) -> u8 {
        (self.raw >> 2) & 15
    }
    pub const fn width_code(self) -> u8 {
        self.raw >> 6
    }

    /// Compare with an explicitly selected documented part contract.
    /// This does not identify the fitted part, voltage, rank wiring or board.
    /// No policy is inferred from capacity, and no default policy is supplied.
    pub const fn review(self, policy: PartPolicy) -> Result<ReviewedMr8, ContractError> {
        match policy {
            PartPolicy::SamsungK4ube3d4abMgclRev10 => {
                if self.type_code() != 0 && self.type_code() != 2 {
                    return Err(ContractError {
                        capture: self,
                        field: ContractField::Type,
                    });
                }
                if self.width_code() != 0 {
                    return Err(ContractError {
                        capture: self,
                        field: ContractField::Width,
                    });
                }
                if self.density_code() != 4 {
                    return Err(ContractError {
                        capture: self,
                        field: ContractField::Density,
                    });
                }
                Ok(ReviewedMr8 {
                    capture: self,
                    policy,
                    channel_mib: 1024,
                })
            }
        }
    }
}

/// Independently documented candidate policies; none is a board default.
/// Selecting one requires separate evidence of the actual fitted device.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PartPolicy {
    /// Samsung K4UBE3D4AB-MGCL, Rev1.0 (August2021), pages12/14/25:
    /// two x16 channels per16Gb die,8Gb perchannel; MR8 type00/10,
    /// width00, density0100 for that organization. Other table densities
    /// describe the wider family, not this exact package organization.
    /// Public manufacturer-authored document, SHA256
    /// 7bb0223d5490109c2ebd9e8f25641e1d2dccd2df0952b04c02553fd078a06e04:
    /// <https://www.ftcelectronics.com/datasheets-6a/k4ube3d4ab-mgcl.pdf>.
    /// Type codes are not permission to select or change a supply voltage.
    SamsungK4ube3d4abMgclRev10,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ContractField {
    Type,
    Width,
    Density,
}

/// A failed part-contract comparison preserves the complete observed byte.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ContractError {
    capture: Mr8Capture,
    field: ContractField,
}

impl ContractError {
    pub const fn capture(self) -> Mr8Capture {
        self.capture
    }
    pub const fn field(self) -> ContractField {
        self.field
    }
}

/// A per-channel device-contract match, not admitted board geometry.
/// It carries the original capture and selected policy for later evidence.
/// Rank/channel population, actual part identity, rails and controller mapping
/// remain external; no runtime initialization or board profile is selected.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ReviewedMr8 {
    capture: Mr8Capture,
    policy: PartPolicy,
    channel_mib: u32,
}

impl ReviewedMr8 {
    pub const fn capture(self) -> Mr8Capture {
        self.capture
    }
    pub const fn policy(self) -> PartPolicy {
        self.policy
    }
    pub const fn channel_mib(self) -> u32 {
        self.channel_mib
    }
}
