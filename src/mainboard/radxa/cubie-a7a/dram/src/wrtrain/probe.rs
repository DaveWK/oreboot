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

/// Why the live configuration gets no probe plan. Every refusal is decided
/// from register reads alone: no store has been made.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Refusal {
    /// The two controllers hold different rank enables or rank maps.
    Channels,
    /// One enabled rank and no rank bit. The registers repeat the parameter
    /// set the strap selected, so they cannot tell a single-rank board from
    /// a dual-rank board with the wrong set, and rank 1 would go untested.
    SingleRank,
    /// Not rank enables and a rank bit the stage has a probe plan for.
    Geometry,
}

impl Refusal {
    /// For the log.
    pub const fn reason(self) -> &'static str {
        match self {
            Self::Channels => "the two controllers hold different rank configurations",
            Self::SingleRank => {
                "single rank is not positively identified and is not supported by the blob-free path"
            }
            Self::Geometry => "unsupported rank layout",
        }
    }
}

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
    /// `ctrl::controller_config` writes the selected Para's rank count to
    /// MSTR0.active_ranks (`dwc_ddrctl::mstr0_lpddr5_bl16`: bits 25:24).
    /// Both channels must agree; never downgrade an unmappable dual rank.
    /// Both timing groups are always required.
    pub fn from_live(base: u32, mstr: [u32; 2], am1: [u32; 2]) -> Result<Self, Refusal> {
        let active = mstr.map(|v| (v >> 24) & 3);
        // RankMap: two 16-bit channels make a 32-bit system word (shift 2).
        let bits = am1.map(|v| RankMap::bit_from_addrmap1(v, 2));
        if active[0] != active[1] || bits[0] != bits[1] {
            return Err(Refusal::Channels);
        }
        let stripe = match (active[0], bits[0]) {
            (1, None) => return Err(Refusal::SingleRank),
            (3, Some(bit)) => 1u32.checked_shl(bit).ok_or(Refusal::Geometry)?,
            _ => return Err(Refusal::Geometry),
        };
        let reach = OFF[OFF.len() - 1] + BLOCK_BYTES as u32;
        if INSET + reach > stripe {
            return Err(Refusal::Geometry);
        }
        // The exclusive loop end must be representable by 32-bit bt0.
        let hi = base
            .checked_add(INSET)
            .and_then(|lo| lo.checked_add(stripe))
            .filter(|hi| hi.checked_add(reach).is_some())
            .ok_or(Refusal::Geometry)?;
        let lo = hi - stripe;
        let blocks = core::array::from_fn(|i| {
            ((if i < OFF.len() { lo } else { hi }) + OFF[i % OFF.len()]) as usize
        });
        let bytes = (1 << DBYTES) - 1;
        Ok(Self {
            blocks,
            required: bytes | (bytes << DBYTES),
        })
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

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use crate::ctrl;
    use crate::hw::{Hw, Rail};
    use crate::para::Para;
    use crate::sets::{HYNIX_4G, PARAM_SETS};
    use core::fmt;
    use std::collections::HashMap;
    use std::vec::Vec;

    const BASE: u32 = 0x4000_0000;

    /// Registers and DRAM as one address space: reads return the last
    /// value written, and every store is recorded.
    #[derive(Default)]
    struct Fake {
        stores: Vec<(usize, u32)>,
        mem: HashMap<usize, u32>,
    }
    impl RegIo for Fake {
        fn read32(&mut self, addr: usize) -> u32 {
            *self.mem.get(&addr).unwrap_or(&0)
        }
        fn write32(&mut self, addr: usize, val: u32) {
            self.stores.push((addr, val));
            self.mem.insert(addr, val);
        }
        fn read16(&mut self, _: usize) -> u16 {
            panic!("no PHY access in the probe plan")
        }
        fn write16(&mut self, _: usize, _: u16) {
            panic!("no PHY access in the probe plan")
        }
        fn udelay(&mut self, _: u32) {}
        fn now_us(&mut self) -> u64 {
            0
        }
        fn log(&mut self, _: fmt::Arguments) {}
    }
    impl Hw for Fake {
        fn set_voltage(&mut self, _: Rail, _: u32) -> bool {
            true
        }
    }

    impl Fake {
        /// Both controllers holding these MSTR0 and ADDRMAP1 values.
        fn with(mstr: u32, am1: u32) -> Self {
            let mut hw = Self::default();
            for ch in 0..2 {
                hw.mem.insert(ddrc(ch, MSTR0), mstr);
                hw.mem.insert(addrmap(ch, 1), am1);
            }
            hw
        }
        /// Both controllers as the bring-up programs them for set `i`
        /// (`ctrl::com_init`'s MSTR0 and address map writes).
        fn programmed(i: usize) -> Self {
            let p = Para::from_sysconfig(&PARAM_SETS[i].para);
            let mut hw = Self::default();
            for ch in 0..2 {
                ctrl::controller_config(&mut hw, &p, ch);
                ctrl::set_address_map(&mut hw, &p, ch).unwrap();
            }
            hw.stores.clear();
            hw
        }
    }

    /// The stage's front as bt0 runs it: read the live registers, plan,
    /// and only with a plan compare once.
    fn stage(hw: &mut Fake) -> Result<Plan, Refusal> {
        let (mstr, am1) = read_live(hw);
        let plan = Plan::from_live(BASE, mstr, am1)?;
        plan.compare(hw, 0x5eed_0001);
        Ok(plan)
    }

    #[test]
    fn probe119r2_dual_rank_board_with_a_single_rank_configuration_is_refused() {
        // One enabled rank and no chip-select bit: what the registers of a
        // dual-rank board hold when the strap picked a single-rank set.
        // Rank 1 would go untested, so there is no plan and no store.
        let mut hw = Fake::with(1 << 24, 0x3f3f);
        assert_eq!(stage(&mut hw).err(), Some(Refusal::SingleRank));
        assert_eq!(hw.stores, []);
    }

    #[test]
    fn probe119r2_genuine_single_rank_set_is_refused_with_the_log_line() {
        let mut hw = Fake::programmed(HYNIX_4G);
        let (mstr, am1) = read_live(&mut hw);
        assert_eq!(mstr.map(|v| (v >> 24) & 3), [1, 1]);
        assert_eq!(am1.map(|v| v & 0x3f), [0x3f, 0x3f]);
        let refusal = stage(&mut hw).err().unwrap();
        assert_eq!(refusal, Refusal::SingleRank);
        assert_eq!(
            refusal.reason(),
            "single rank is not positively identified and is not supported by the blob-free path"
        );
        assert_eq!(hw.stores, []);
    }
}
