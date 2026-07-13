// SPDX-License-Identifier: GPL-2.0-only
//! Frequency-set (pstate) timing registers: offsets within a frequency
//! block ([`crate::Map::freq`]) and a checked field packer.
//!
//! The DRAMSET1TMGn registers start at offset 0 of each frequency block
//! (DDRCTL with frequency sets; the uMCTL2 DRAMTMGn of single-frequency
//! controllers have the same field layout at another base). The values come
//! from the JEDEC timings in controller cycles (`jedec_lpddr::timing::ns_to_ck`)
//! and the SoC's choices; this module only places them.

/// DRAMSET1TMGn, n = 0..=14, of a frequency block.
pub const fn dramset1tmg(n: usize) -> usize {
    4 * n
}

/// A field of a register: bit position and width.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Field {
    pub shift: u32,
    pub width: u32,
}

impl Field {
    pub const fn new(shift: u32, width: u32) -> Self {
        Field { shift, width }
    }
    pub const fn mask(self) -> u32 {
        (if self.width >= 32 {
            u32::MAX
        } else {
            (1u32 << self.width) - 1
        }) << self.shift
    }
}

/// A value does not fit its field.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Overflow {
    pub field: Field,
    pub value: u32,
}

/// OR the values into their fields; a value wider than its field is an
/// error, never truncated.
pub fn pack(fields: &[(Field, u32)]) -> Result<u32, Overflow> {
    let mut v = 0u32;
    for &(f, x) in fields {
        if f.width < 32 && x >> f.width != 0 {
            return Err(Overflow { field: f, value: x });
        }
        v |= x << f.shift;
    }
    Ok(v)
}

/// Replace the fields in `old`, keeping the other bits.
pub fn update(old: u32, fields: &[(Field, u32)]) -> Result<u32, Overflow> {
    let clear = fields.iter().fold(0, |m, (f, _)| m | f.mask());
    Ok((old & !clear) | pack(fields)?)
}

/// The DRAMSET1TMG0..5 field layout (one byte per field, as the A733 init
/// programs them).
pub mod tmg {
    use super::Field;
    /// DRAMSET1TMG0: tRAS(min), tRAS(max)/1024, tFAW, write-to-precharge.
    pub const RAS_MIN: Field = Field::new(0, 8);
    pub const RAS_MAX: Field = Field::new(8, 8);
    pub const FAW: Field = Field::new(16, 8);
    pub const WR2PRE: Field = Field::new(24, 8);
    /// DRAMSET1TMG1: tRC, read-to-precharge, tXP.
    pub const RC: Field = Field::new(0, 8);
    pub const RD2PRE: Field = Field::new(8, 8);
    pub const XP: Field = Field::new(16, 8);
    /// DRAMSET1TMG2: write-to-read, read-to-write, read latency, write latency.
    pub const WR2RD: Field = Field::new(0, 8);
    pub const RD2WR: Field = Field::new(8, 8);
    pub const READ_LATENCY: Field = Field::new(16, 8);
    pub const WRITE_LATENCY: Field = Field::new(24, 8);
    /// DRAMSET1TMG4: tRP, tRRD, tCCD, tRCD.
    pub const RP: Field = Field::new(0, 8);
    pub const RRD: Field = Field::new(8, 8);
    pub const CCD: Field = Field::new(16, 8);
    pub const RCD: Field = Field::new(24, 8);
}
