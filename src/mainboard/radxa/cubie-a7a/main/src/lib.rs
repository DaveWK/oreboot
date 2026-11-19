// SPDX-License-Identifier: GPL-2.0-only
//! Host test harness for the cached memtest; the firmware uses its own entry.
#![no_std]

#[cfg(test)]
use cubie_a7a_common::{mem, sha256, Handoff, HANDOFF_MAGIC, HANDOFF_VERSION};
#[cfg(test)]
mod memtest_cached_core;

#[cfg(test)]
mod tests {
    use super::memtest_cached_core::{
        rank_bit_from_addrmap1, validate_live_rank_map, LiveRankMap, DUAL_CH_SHIFT,
    };
    use cubie_a7a_dram::ctrl::RankMap;

    #[test]
    fn live_rank_bit_and_supported_layouts_agree_with_write_stage() {
        for gib in [4, 6] {
            for cs in 0..=0x3f {
                for upper in [0, 0x3f] {
                    let raw = upper << 8 | cs;
                    assert_eq!(
                        rank_bit_from_addrmap1(raw, DUAL_CH_SHIFT),
                        RankMap::bit_from_addrmap1(raw, 2),
                        "ADDRMAP1={raw:#x}, {gib} GiB",
                    );
                    let expected = match (gib, cs) {
                        (4, 0x17) => Some(LiveRankMap::DualRank { rank_bit: 31 }),
                        (6, 0x16) => Some(LiveRankMap::DualRank { rank_bit: 30 }),
                        (_, 0x3f) => Some(LiveRankMap::SingleRank), // LIMITED only
                        _ => None,
                    };
                    assert_eq!(validate_live_rank_map(raw, gib << 30).ok(), expected);
                }
            }
        }
    }
}
