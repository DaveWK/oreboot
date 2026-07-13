// SPDX-License-Identifier: GPL-2.0-only
//! LPDDR4/4X density from mode register 8 (JEDEC LPDDR4/4X MR8: OP[5:2] density,
//! OP[7:6] I/O width), per channel, and the total over every rank and
//! channel read.
//!
//! Moved from the SpacemiT K1 bt0 (ore-edk-boot-opi `dram.rs`
//! `channel_size`, `ddr_get_density`): the decode and the "every rank and
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
