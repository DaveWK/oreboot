// SPDX-License-Identifier: GPL-2.0-only
//! A733 two-channel write-stage probes. Channel n drives DBYTE 2n and 2n+1
//! (the board's `rdtrain`/`wrtrain` DBYTE layout). Timing group = rank.
//!
//! # What the probes may overwrite
//!
//! The stage stores to DRAM, so it assumes that nothing lives there yet:
//! the write stage runs inside bt0's DRAM bring-up, before anything is
//! loaded. bt0, its bss and its stack are in SRAM (`bt0/build.rs`, the
//! `SRAM` region of the linker script); a FEL boot loads main and the
//! payload only after entry 1 has returned with DRAM up (workspace
//! `scripts/fel-boot.sh`, "Entry 1" then "Images to DRAM"), and the SD,
//! eMMC and NOR boots read them after `dram::init` (`bt0/src/main.rs`).
//! The lower bank is at the address main later builds its device tree at
//! (`cubie_a7a_common::mem::DTB_ADDR`), so a caller that ran the stage
//! after the images were placed would corrupt them: keep it before.
//!
//! No block touches main's image, the firmware log areas (the previous
//! boot's log can survive a warm reset), the bt0 → main handoff, the EDK2
//! FD window or the fastboot buffers; the unit tests hold the table.
use super::{pattern, Folds, BLOCKS, DBYTES};
use crate::ctrl::{addrmap, ddrc, RankMap, MSTR0};
use crate::hw::RegIo;

/// Original bt0 write-stage footprint: one 128 B block on either side of
/// each possible interleave bit (128 B through 8 KiB), 1 MiB into a stripe.
pub const BLOCK_BYTES: usize = 128;
const OFF: [u32; BLOCKS / 2] = [0, 0x80, 0x100, 0x200, 0x400, 0x800, 0x1000, 0x2000];
const INSET: u32 = 1 << 20;

/// A rank layout the stage has a probe plan for: the DRAM offset bit that
/// selects the rank and the usable size the boot reports with that map.
struct Layout {
    rank_bit: u32,
    usable_mib: u32,
}

/// The two dual-rank boards on the bench, as `ctrl::RankMap` decodes their
/// address maps (ctrl.rs tests `cxmt_4gb_address_map_and_ranks` and
/// `cxmt_6gb_address_map_and_ranks`; the unit tests here derive both rows
/// from the parameter sets again):
///
/// - CXMT 4 GB: ADDRMAP1 chip-select field 0x17, the rank is the top
///   offset bit (31), 4096 MiB.
/// - CXMT 6 GB, non-binary: field 0x16, the rank bit (30) lies below the
///   two top row bits, 6144 MiB of which the usable part is contiguous.
const SUPPORTED: [Layout; 2] = [
    Layout {
        rank_bit: 31,
        usable_mib: 4096,
    },
    Layout {
        rank_bit: 30,
        usable_mib: 6144,
    },
];

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
    /// Not the rank enables and rank bit of a supported layout.
    Geometry,
    /// A probe block is not wholly inside usable DRAM.
    Bounds,
    /// The boot's usable size is not the supported layout's.
    Size,
}

impl Refusal {
    /// For the log.
    pub const fn reason(self) -> &'static str {
        match self {
            Self::Channels => "the two controllers hold different rank configurations",
            Self::SingleRank => {
                "single rank is not positively identified and is not supported by the blob-free path"
            }
            Self::Geometry => "not a supported dual-rank CXMT 4 GB or 6 GB rank layout",
            Self::Bounds => "a probe block is outside usable DRAM",
            Self::Size => "the usable size does not match the decoded rank layout",
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

/// The checked probe blocks. Built only by [`Plan::from_live`], and
/// [`Plan::compare`] is the stage's only store to DRAM: no block is
/// written that has not passed the checks.
pub struct Plan {
    blocks: [usize; BLOCKS],
    /// Bit (TG * DBYTES + DBYTE), independent of the discovery result.
    required: u8,
}

impl Plan {
    /// `ctrl::controller_config` writes the selected Para's rank count to
    /// MSTR0.active_ranks (`dwc_ddrctl::mstr0_lpddr5_bl16`: bits 25:24).
    /// Both channels must agree; never downgrade an unmappable dual rank.
    ///
    /// `usable_mib` is the size the boot reports for the same parameter set
    /// (`size::boot_size_mib`, what `init_one` returns and bt0 hands to
    /// main as the end of DRAM); `base` is the DRAM base. Both timing
    /// groups are always required.
    pub fn from_live(
        base: u32,
        usable_mib: u32,
        mstr: [u32; 2],
        am1: [u32; 2],
    ) -> Result<Self, Refusal> {
        let active = mstr.map(|v| (v >> 24) & 3);
        // RankMap: two 16-bit channels make a 32-bit system word (shift 2).
        let bits = am1.map(|v| RankMap::bit_from_addrmap1(v, 2));
        if active[0] != active[1] || bits[0] != bits[1] {
            return Err(Refusal::Channels);
        }
        let layout = match (active[0], bits[0]) {
            (1, None) => return Err(Refusal::SingleRank),
            (3, Some(bit)) => SUPPORTED
                .iter()
                .find(|l| l.rank_bit == bit)
                .ok_or(Refusal::Geometry)?,
            _ => return Err(Refusal::Geometry),
        };
        let lo = base as u64 + INSET as u64;
        let hi = lo + (1u64 << layout.rank_bit);
        // Exclusive end of usable DRAM, and of what 32-bit bt0 addresses.
        let end = (base as u64 + ((usable_mib as u64) << 20)).min(1 << 32);
        let mut blocks = [0; BLOCKS];
        for (i, block) in blocks.iter_mut().enumerate() {
            let start = (if i < OFF.len() { lo } else { hi }) + OFF[i % OFF.len()] as u64;
            if start + BLOCK_BYTES as u64 > end {
                return Err(Refusal::Bounds);
            }
            *block = start as usize;
        }
        if usable_mib != layout.usable_mib {
            return Err(Refusal::Size);
        }
        let bytes = (1 << DBYTES) - 1;
        Ok(Self {
            blocks,
            required: bytes | (bytes << DBYTES),
        })
    }

    /// First address of every block: eight in rank 0, then eight in rank 1.
    pub fn blocks(&self) -> &[usize; BLOCKS] {
        &self.blocks
    }

    /// Bit (TG * DBYTES + DBYTE) of every pair the stage must measure.
    pub fn required(&self) -> u8 {
        self.required
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
    use crate::sets::{CXMT_4G, CXMT_6G, HYNIX_4G, PARAM_SETS, SAFE_4G};
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
        /// (`ctrl::com_init`'s MSTR0 and address map writes), and the
        /// usable size the boot reports for it.
        fn programmed(i: usize) -> (Self, u32) {
            let p = Para::from_sysconfig(&PARAM_SETS[i].para);
            let mut hw = Self::default();
            for ch in 0..2 {
                ctrl::controller_config(&mut hw, &p, ch);
                ctrl::set_address_map(&mut hw, &p, ch).unwrap();
            }
            hw.stores.clear();
            (hw, crate::size::boot_size_mib(&p).unwrap())
        }
    }

    /// The stage's front as bt0 runs it: read the live registers, plan,
    /// and only with a plan compare once.
    fn stage(hw: &mut Fake, usable_mib: u32) -> Result<Plan, Refusal> {
        let (mstr, am1) = read_live(hw);
        let plan = Plan::from_live(BASE, usable_mib, mstr, am1)?;
        plan.compare(hw, 0x5eed_0001);
        Ok(plan)
    }

    fn bank(first: usize) -> [usize; BLOCKS / 2] {
        OFF.map(|off| first + off as usize)
    }

    #[test]
    fn probe119r2_dual_rank_board_with_a_single_rank_configuration_is_refused() {
        // One enabled rank and no chip-select bit: what the registers of a
        // dual-rank board hold when the strap picked a single-rank set.
        // Rank 1 would go untested, so there is no plan and no store.
        let mut hw = Fake::with(1 << 24, 0x3f3f);
        assert_eq!(stage(&mut hw, 4096).err(), Some(Refusal::SingleRank));
        assert_eq!(hw.stores, []);
    }

    #[test]
    fn probe119r2_genuine_single_rank_set_is_refused_with_the_log_line() {
        let (mut hw, mib) = Fake::programmed(HYNIX_4G);
        let (mstr, am1) = read_live(&mut hw);
        assert_eq!(mstr.map(|v| (v >> 24) & 3), [1, 1]);
        assert_eq!(am1.map(|v| v & 0x3f), [0x3f, 0x3f]);
        let refusal = stage(&mut hw, mib).err().unwrap();
        assert_eq!(refusal, Refusal::SingleRank);
        assert_eq!(
            refusal.reason(),
            "single rank is not positively identified and is not supported by the blob-free path"
        );
        assert_eq!(hw.stores, []);
    }

    #[test]
    fn probe119r2_unsupported_chip_select_field_is_refused_with_zero_stores() {
        // Two ranks enabled, rank bit 24: the upper bank would be at
        // 0x41100000, inside the EDK2 FD window.
        let mut hw = Fake::with(3 << 24, 0x3f10);
        assert_eq!(stage(&mut hw, 4096).err(), Some(Refusal::Geometry));
        assert_eq!(hw.stores, []);
        // Every other chip-select field than the two supported ones.
        for cs in (0..0x3f).filter(|cs| ![0x16, 0x17].contains(cs)) {
            for mib in [4096, 6144] {
                let mut hw = Fake::with(3 << 24, 0x3f00 | cs);
                assert_eq!(stage(&mut hw, mib).err(), Some(Refusal::Geometry));
                assert_eq!(hw.stores, [], "chip-select field {cs:#x}");
            }
        }
        // A rank bit with one rank enabled, two ranks without one, the
        // reserved rank enables, and channels that disagree.
        for (mstr, am1) in [(1, 0x3f17), (3, 0x3f3f), (0, 0x3f17), (2, 0x3f17)] {
            let mut hw = Fake::with(mstr << 24, am1);
            assert_eq!(stage(&mut hw, 4096).err(), Some(Refusal::Geometry));
            assert_eq!(hw.stores, []);
        }
        let mut hw = Fake::with(3 << 24, 0x3f17);
        hw.mem.insert(addrmap(1, 1), 0x3f16);
        assert_eq!(stage(&mut hw, 4096).err(), Some(Refusal::Channels));
        assert_eq!(hw.stores, []);
    }

    #[test]
    fn probe119r2_block_past_the_usable_extent_is_refused_with_zero_stores() {
        // The 4 GB map with 2 GiB usable: rank 1's bank starts where DRAM
        // ends. Likewise the 6 GB map with 1 GiB.
        for (am1, mib) in [(0x3f17, 2048), (0x3f16, 1024)] {
            let mut hw = Fake::with(3 << 24, am1);
            assert_eq!(stage(&mut hw, mib).err(), Some(Refusal::Bounds));
            assert_eq!(hw.stores, []);
        }
        // The 6 GB map's upper bank is 0x80100000..0x80102080: with
        // 1025 MiB DRAM ends at its first byte.
        let mut hw = Fake::with(3 << 24, 0x3f16);
        assert_eq!(stage(&mut hw, 1025).err(), Some(Refusal::Bounds));
        assert_eq!(hw.stores, []);
        // Inside usable DRAM but not the layout's size: still no plan.
        for (am1, mib) in [
            (0x3f17, 6144),
            (0x3f16, 4096),
            (0x3f16, 1026),
            (0x3f17, 12288),
        ] {
            let mut hw = Fake::with(3 << 24, am1);
            assert_eq!(stage(&mut hw, mib).err(), Some(Refusal::Size));
            assert_eq!(hw.stores, []);
        }
        // A block that 32-bit bt0 cannot address.
        let mut hw = Fake::with(3 << 24, 0x3f17);
        let (mstr, am1) = read_live(&mut hw);
        assert_eq!(
            Plan::from_live(0x8000_0000, 4096, mstr, am1).err(),
            Some(Refusal::Bounds)
        );
        assert_eq!(hw.stores, []);
    }

    #[test]
    fn probe119r2_each_supported_geometry_yields_exactly_the_expected_blocks() {
        for (set, mib, upper) in [
            (CXMT_4G, 4096, 0xc010_0000usize),
            (CXMT_6G, 6144, 0x8010_0000),
        ] {
            let (mut hw, size) = Fake::programmed(set);
            assert_eq!(size, mib);
            let plan = stage(&mut hw, size).unwrap_or_else(|r| panic!("{}: {r:?}", r.reason()));
            let mut want = Vec::from(bank(0x4010_0000));
            want.extend(bank(upper));
            assert_eq!(plan.blocks()[..], want[..]);
            assert_eq!(plan.required(), 0xff);
            // One compare stores every word of every block once, in
            // order, and nothing else.
            let words: Vec<usize> = want
                .iter()
                .flat_map(|&b| (b..b + BLOCK_BYTES).step_by(4))
                .collect();
            assert_eq!(words.len(), BLOCKS * BLOCK_BYTES / 4);
            let stored: Vec<usize> = hw.stores.iter().map(|s| s.0).collect();
            assert_eq!(stored, words);
            for &(a, v) in &hw.stores {
                assert_eq!(v, pattern(a, 0x5eed_0001));
            }
            // Every block is whole inside usable DRAM.
            let end = BASE as u64 + ((size as u64) << 20);
            for &b in plan.blocks() {
                assert!(b as u64 >= BASE as u64 && b as u64 + BLOCK_BYTES as u64 <= end);
            }
            // The readback of an intact memory is clean and complete.
            let (folds, compared) = plan.compare(&mut hw, 7);
            assert_eq!((folds, compared), ([0; BLOCKS], 0xffff));
        }
    }

    #[test]
    fn probe119r2_supported_layouts_are_the_two_cxmt_rank_maps() {
        // The table against ctrl::RankMap for the two parameter sets.
        for (layout, set) in SUPPORTED.iter().zip([CXMT_4G, CXMT_6G]) {
            let p = Para::from_sysconfig(&PARAM_SETS[set].para);
            let mib = crate::size::boot_size_mib(&p).unwrap();
            let map = RankMap::of(&p, (mib as u64) << 20).unwrap();
            assert!(p.dual_rank());
            assert_eq!(map.rank_bit, Some(layout.rank_bit));
            assert_eq!(mib, layout.usable_mib);
            // The upper bank is in rank 1 and the lower in rank 0.
            let upper = INSET as u64 + (1u64 << layout.rank_bit);
            assert_eq!((map.rank(INSET as u64), map.rank(upper)), (0, 1));
        }
    }

    #[test]
    fn probe119r2_every_parameter_set_has_a_plan_or_a_refusal() {
        // (set, two ranks, usable MiB, live MSTR0 ranks, live chip-select
        // field, outcome).
        let want = [
            ("CXMT 4GB", true, 4096, 3, 0x17, Ok(0xc010_0000usize)),
            ("CXMT 6GB", true, 6144, 3, 0x16, Ok(0x8010_0000)),
            ("CXMT 8GB", true, 8192, 3, 0x18, Err(Refusal::Geometry)),
            ("CXMT 12GB", true, 12288, 3, 0x17, Err(Refusal::Size)),
            (
                "SK hynix 4GB",
                false,
                4096,
                1,
                0x3f,
                Err(Refusal::SingleRank),
            ),
            ("4GB @1800 (v1.4.8)", true, 6144, 3, 0x16, Ok(0x8010_0000)),
        ];
        assert_eq!(PARAM_SETS[SAFE_4G].name, want[5].0);
        for (i, set) in PARAM_SETS.iter().enumerate() {
            let p = Para::from_sysconfig(&set.para);
            let (mut hw, mib) = Fake::programmed(i);
            let (mstr, am1) = read_live(&mut hw);
            let got = stage(&mut hw, mib);
            if got.is_err() {
                assert_eq!(hw.stores, [], "{}", set.name);
            }
            assert_eq!(
                (
                    set.name,
                    p.dual_rank(),
                    mib,
                    (mstr[0] >> 24) & 3,
                    am1[0] & 0x3f,
                    got.map(|plan| plan.blocks()[BLOCKS / 2])
                ),
                want[i]
            );
        }
    }

    #[test]
    fn probe119r2_probe_blocks_against_the_reserved_ranges() {
        // What lives in DRAM once the boot has gone on (first, exclusive
        // end): cubie-a7a-common `mem` and `fwlog`, bt0 `fastboot.rs` and
        // `extcheck.rs`. The stage runs before any of it is placed.
        const MAIN: (usize, usize) = (0x4000_0000, 0x4008_0000);
        const DTB: (usize, usize) = (0x4010_0000, 0x4011_0000);
        const LOGS: (usize, usize) = (0x4014_0000, 0x401c_0000);
        const HANDOFF: (usize, usize) = (0x401f_f000, 0x4020_0000);
        const EDK2_FD: (usize, usize) = (0x4100_0000, 0x4140_0000);
        const DOWNLOAD: (usize, usize) = (0x4800_0000, 0x5001_0000);
        const EXTENSION: (usize, usize) = (0x5900_0000, 0x5910_0000);
        let inside = |b: usize, r: (usize, usize)| b >= r.0 && b + BLOCK_BYTES <= r.1;
        let clear = |b: usize, r: (usize, usize)| b + BLOCK_BYTES <= r.0 || b >= r.1;
        for (am1, mib) in [(0x3f17, 4096), (0x3f16, 6144)] {
            let plan = Plan::from_live(BASE, mib, [3 << 24; 2], [am1; 2]).unwrap();
            for (k, &b) in plan.blocks().iter().enumerate() {
                // The lower bank is where main later builds the device
                // tree: the stage must stay before the images are loaded.
                assert_eq!(inside(b, DTB), k < BLOCKS / 2, "block {k} at {b:#x}");
                for r in [MAIN, LOGS, HANDOFF, EDK2_FD, DOWNLOAD, EXTENSION] {
                    assert!(clear(b, r), "block {k} at {b:#x} in {r:x?}");
                }
            }
        }
    }
}
