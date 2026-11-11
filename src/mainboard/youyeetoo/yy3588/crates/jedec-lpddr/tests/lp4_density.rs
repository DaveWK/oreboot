// SPDX-License-Identifier: GPL-2.0-only
//! lp4::density against the K1 board code it replaces.

#[path = "legacy/k1_density.rs"]
mod old;

use jedec_lpddr::lp4::density::{channel_mib, total_mib, DensityError};

#[test]
fn channel_mib_matches_k1_for_every_mr8() {
    for v in 0..=255u8 {
        assert_eq!(
            channel_mib(v).ok(),
            old::channel_size_of(v as u32).ok(),
            "MR8 {v:#x}"
        );
    }
}

/// The K1 board keeps its accepted totals; with that rule on top, the crate
/// gives ddr_get_density's result for every rank count and MR8 set.
fn k1_policy(mr8s: &[u8]) -> Result<u32, ()> {
    let t = total_mib(mr8s).map_err(|_| ())?;
    if matches!(t, 1024 | 2048 | 4096 | 8192) {
        Ok(t)
    } else {
        Err(())
    }
}

#[test]
fn total_matches_k1_ddr_get_density() {
    let mut seed = 0x1234_5678u32;
    let mut next = || {
        seed ^= seed << 13;
        seed ^= seed >> 17;
        seed ^= seed << 5;
        seed
    };
    let mut cases: Vec<[[u32; 2]; 2]> = Vec::new();
    for v in 0..=255u32 {
        cases.push([[v, v], [v, v]]);
    }
    for _ in 0..20_000 {
        let pick = |n: u32| [0x00u32, 0x08, 0x10, 0x18, 0x40, 0x48, 0xc0, 0x1c][(n & 7) as usize];
        cases.push([[pick(next()), pick(next())], [pick(next()), pick(next())]]);
    }
    for t in cases {
        for cs_num in 1..=2u32 {
            old::MR8.with(|m| *m.borrow_mut() = t);
            let want = old::ddr_get_density_of(cs_num).ok();
            let mut reads = Vec::new();
            for rank in t.iter().take(cs_num as usize) {
                reads.push(rank[0] as u8);
                reads.push(rank[1] as u8);
            }
            // ddr_get_density compares every read with rank 0 channel 0.
            assert_eq!(k1_policy(&reads).ok(), want, "{t:x?} cs {cs_num}");
        }
    }
}

#[test]
fn errors() {
    assert_eq!(channel_mib(0x1c), Err(DensityError::Density));
    assert_eq!(channel_mib(0x80), Err(DensityError::Width));
    assert_eq!(total_mib(&[]), Err(DensityError::Empty));
    assert_eq!(total_mib(&[0x18, 0x10]), Err(DensityError::Mismatch));
    assert_eq!(total_mib(&[0x18, 0x18, 0x18, 0x18]), Ok(4 * 2048));
}
