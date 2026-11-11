// SPDX-License-Identifier: GPL-2.0-only
//! P3-M11: rank merge by midrange, plausibility checks, bounded retrain.

use dram_train::ranks::{
    merge, monotonic, plausible, retrain, Limits, MergeError, Merged, RetrainError,
};

#[test]
fn the_midrange_gives_every_rank_the_same_margin() {
    // Four ranks with eye centres 100, 102, 104 and 140 (one late rank),
    // each eye +/-40 wide.
    let c = [100, 102, 104, 140];
    let m = merge(&c, 64).unwrap();
    assert_eq!(
        m,
        Merged {
            value: 120,
            min: 100,
            max: 140
        }
    );
    let margin = |set: i32| c.iter().map(|x| 40 - (set - x).abs()).min().unwrap();
    let mean = c.iter().sum::<i32>() / 4; // 111
    assert_eq!(margin(m.value), 20);
    assert_eq!(margin(mean), 11, "the mean starves the late rank");
    assert_eq!(m.spread(), 40);
    assert_eq!(merge(&c, 39), Err(MergeError::Skew { min: 100, max: 140 }));
    assert_eq!(merge(&[], 10), Err(MergeError::Empty));
    assert_eq!(merge(&[7], 0).unwrap().value, 7);
}

#[test]
fn implausible_lanes_are_named() {
    let l = Limits {
        lo: 0,
        hi: 511,
        near_end: 2,
        near_zero: 1,
        outlier: 30,
    };
    let good = [200, 210, 205, 198, 215, 207, 202, 211];
    assert!(plausible(&good, &l).ok());
    let bad = [200, 510, 205, 0, 290, 207, 1, 211];
    let p = plausible(&bad, &l);
    assert_eq!(p.saturated, (1 << 1) | (1 << 3) | (1 << 6));
    assert_eq!(p.near_zero, (1 << 3) | (1 << 6));
    assert_eq!(p.outliers, (1 << 1) | (1 << 3) | (1 << 4) | (1 << 6));
    assert_eq!(p.median, 206);
    assert_eq!(p.bad(), 0b0101_1010);
    // Odd count median; empty input.
    assert_eq!(plausible(&[3, 1, 2], &l).median, 2);
    assert!(plausible(&[], &l).ok());
}

#[test]
fn monotonic_finds_the_first_turn() {
    assert_eq!(monotonic(&[10, 20, 20, 35], true), None);
    assert_eq!(monotonic(&[10, 20, 15, 35], true), Some(2));
    assert_eq!(monotonic(&[40, 30, 30, 10], false), None);
    assert_eq!(monotonic(&[40, 30, 31], false), Some(2));
    assert_eq!(monotonic(&[], true), None);
}

#[test]
fn retrain_rotates_the_start_phase_and_is_bounded() {
    // The engine locks wrongly when started at phase 0 or 90.
    let mut seen = Vec::new();
    let r = retrain(
        4,
        &[0, 90, 180, 270],
        |ph| {
            seen.push(ph);
            Ok::<_, ()>(if ph < 180 { 511 } else { 200 })
        },
        |&v| v < 500,
    );
    assert_eq!(r, Ok((200, 2)));
    assert_eq!(seen, [0, 90, 180]);
    // Never plausible: the last result comes back with the error.
    let r = retrain(3, &[0, 90], |_| Ok::<_, ()>(511), |&v| v < 500);
    assert_eq!(r, Err(RetrainError::Implausible(511)));
    // The step itself fails on the last try.
    let mut k = 0;
    let r = retrain(
        2,
        &[0],
        |_| {
            k += 1;
            if k == 2 {
                Err("timeout")
            } else {
                Ok(511)
            }
        },
        |&v| v < 500,
    );
    assert_eq!(r, Err(RetrainError::Step("timeout")));
    assert_eq!(
        retrain(0, &[0], |_| Ok::<i32, ()>(1), |_| true),
        Err(RetrainError::Config)
    );
}
