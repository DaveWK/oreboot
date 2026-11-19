// SPDX-License-Identifier: GPL-2.0-only
//! A733 two-channel write-stage probes. Channel n drives DBYTE 2n and 2n+1
//! (the board's `rdtrain`/`wrtrain` DBYTE layout). Timing group = rank.
use super::{pattern, Folds, BLOCKS, DBYTES};
use crate::ctrl::{addrmap, ddrc, RankMap, MSTR0};
use crate::hw::RegIo;

/// Original bt0 write-stage footprint: one 128 B block on either side of
/// each possible interleave bit (128 B through 8 KiB), 1 MiB into a stripe.
pub const BLOCK_BYTES: usize = 128;
const OFF: [u32; BLOCKS / 2] = [0, 0x80, 0x100, 0x200, 0x400, 0x800, 0x1000, 0x2000];
const INSET: u32 = 1 << 20;

/// MSTR0 and ADDRMAP1 of both controllers, as they are programmed.
pub fn read_live<H: RegIo>(hw: &mut H) -> ([u32; 2], [u32; 2]) {
    (
        [0, 1].map(|ch| hw.read32(ddrc(ch, MSTR0))),
        [0, 1].map(|ch| hw.read32(addrmap(ch, 1))),
    )
}

pub struct Plan {
    pub blocks: [usize; BLOCKS],
    /// Bit (TG * DBYTES + DBYTE), independent of the discovery result.
    pub required: u8,
}

impl Plan {
    /// `ctrl::controller_config` writes the detected Para's rank count to
    /// MSTR0.active_ranks (`dwc_ddrctl::mstr0_lpddr5_bl16`: bits 25:24).
    /// Both channels must agree; never downgrade an unmappable dual rank.
    pub fn from_live(base: u32, mstr: [u32; 2], am1: [u32; 2]) -> Option<Self> {
        let active = mstr.map(|v| (v >> 24) & 3);
        // RankMap: two 16-bit channels make a 32-bit system word (shift 2).
        let bits = am1.map(|v| RankMap::bit_from_addrmap1(v, 2));
        if active[0] != active[1] || bits[0] != bits[1] {
            return None;
        }
        let bytes = (1 << DBYTES) - 1;
        let (upper, required) = match (active[0], bits[0]) {
            // Two distinct probe banks in rank zero (1 and 2 MiB).
            (1, None) => (INSET, bytes),
            (3, Some(bit)) => {
                let stripe = 1u32.checked_shl(bit)?;
                if INSET + OFF[OFF.len() - 1] + BLOCK_BYTES as u32 > stripe {
                    return None;
                }
                (stripe, bytes | (bytes << DBYTES))
            }
            _ => return None,
        };
        let lo = base.checked_add(INSET)?;
        let hi = lo.checked_add(upper)?;
        // The exclusive loop end must be representable by 32-bit bt0.
        hi.checked_add(OFF[OFF.len() - 1] + BLOCK_BYTES as u32)?;
        let blocks = core::array::from_fn(|i| {
            ((if i < OFF.len() { lo } else { hi }) + OFF[i % OFF.len()]) as usize
        });
        Some(Self { blocks, required })
    }

    /// Store the seed's pattern to every block, then read every block
    /// back: per block the OR of `got ^ want`, and the mask of the blocks
    /// compared.
    pub fn compare<H: RegIo>(&self, hw: &mut H, seed: u32) -> (Folds, u32) {
        for &b in self.blocks.iter() {
            for a in (b..b + BLOCK_BYTES).step_by(4) {
                hw.write32(a, pattern(a, seed));
            }
        }
        let mut f = [0u32; BLOCKS];
        let mut compared = 0;
        for (k, &b) in self.blocks.iter().enumerate() {
            for a in (b..b + BLOCK_BYTES).step_by(4) {
                f[k] |= hw.read32(a) ^ pattern(a, seed);
            }
            compared |= 1 << k;
        }
        (f, compared)
    }
}
