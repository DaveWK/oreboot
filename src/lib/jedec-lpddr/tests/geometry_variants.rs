// SPDX-License-Identifier: GPL-2.0-only
//! geometry::Topology over every rank, channel and LPDDR4 density variant.

use jedec_lpddr::geometry::{Topology, TopologyError};
use jedec_lpddr::lp4::density::{channel_mib, total_mib, DensityError};

#[test]
fn every_variant_agrees_with_the_mr8_total() {
    for mr8 in 0..=255u8 {
        for ranks in [1u8, 2] {
            for channels in [1u8, 2, 4] {
                let t = Topology::lp4(ranks, channels, mr8);
                match channel_mib(mr8) {
                    Ok(mib) => {
                        let t = t.unwrap();
                        assert_eq!(t.channel_mib, mib);
                        let reads = vec![mr8; usize::from(ranks) * usize::from(channels)];
                        assert_eq!(t.total_mib(), total_mib(&reads).ok());
                        assert_eq!(
                            t.rank_mib().unwrap() * u32::from(ranks),
                            t.total_mib().unwrap()
                        );
                        assert_eq!(1u32 << t.rank_bits(), u32::from(ranks));
                    }
                    Err(e) => assert_eq!(t, Err(TopologyError::Mr8(e))),
                }
            }
        }
    }
}

#[test]
fn board_variants() {
    // OrangePi R2S: one rank, two x16 channels of 1 GiB (MR8 OP[5:2] = 4).
    let r2s = Topology::lp4(1, 2, 4 << 2).unwrap();
    assert_eq!((r2s.total_mib(), r2s.rank_bits()), (Some(2048), 0));
    // OrangePi RV2 4 GiB: two such ranks.
    let rv2 = Topology::lp4(2, 2, 4 << 2).unwrap();
    assert_eq!((rv2.total_mib(), rv2.rank_bits()), (Some(4096), 1));
    // An 8 GiB two-rank variant: 2 GiB channels (OP[5:2] = 6).
    assert_eq!(Topology::lp4(2, 2, 6 << 2).unwrap().total_mib(), Some(8192));
}

#[test]
fn refusals() {
    assert_eq!(Topology::new(0, 2, 1024), Err(TopologyError::Ranks));
    assert_eq!(Topology::new(3, 2, 1024), Err(TopologyError::Ranks));
    assert_eq!(Topology::new(1, 3, 1024), Err(TopologyError::Channels));
    assert_eq!(Topology::new(1, 2, 0), Err(TopologyError::Size));
    assert_eq!(Topology::new(2, 4, u32::MAX / 4), Err(TopologyError::Size));
    assert_eq!(
        Topology::lp4(1, 2, 0x1c),
        Err(TopologyError::Mr8(DensityError::Density))
    );
}
