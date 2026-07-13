// SPDX-License-Identifier: GPL-2.0-only
//! P3-M6: stage lists by data rate, first-rank stages and the warm-boot
//! (cache restore, self-refresh exit) rule.

use dram_ctrl::stages::{plan, Boot, Rate, Stage, Train, LPDDR4_EXAMPLE};
use Train::*;

fn run(mtps: u32, boot: Boot, rank: u8) -> Vec<Train> {
    let mut out = [Gate; 16];
    let n = plan(&LPDDR4_EXAMPLE, mtps, boot, rank, &mut out);
    out[..n].to_vec()
}

#[test]
fn rates_select_the_stages() {
    assert_eq!(
        run(1066, Boot::Cold, 0),
        [
            Gate,
            WriteLeveling,
            ReadCentre,
            WriteCentre,
            DataMask,
            WriteLatency,
            Sanity
        ]
    );
    // 1600: deskew (at or above) but not read DBI (above).
    assert_eq!(
        run(1600, Boot::Cold, 0),
        [
            Gate,
            WriteLeveling,
            ReadDeskew,
            ReadCentre,
            WriteDeskew,
            WriteCentre,
            DataMask,
            WriteLatency,
            Sanity
        ]
    );
    // 1866: command bus training starts only above it.
    assert!(!run(1866, Boot::Cold, 0).contains(&CommandBus));
    assert!(run(1867, Boot::Cold, 0).contains(&CommandBus));
    assert!(!run(2133, Boot::Cold, 0).contains(&ReadComplex));
    assert_eq!(
        run(2400, Boot::Cold, 0),
        LPDDR4_EXAMPLE.map(|s| s.id).to_vec()
    );
}

#[test]
fn write_complex_runs_on_the_first_rank_only() {
    let r1 = run(3200, Boot::Cold, 1);
    assert!(!r1.contains(&WriteComplex));
    assert!(r1.contains(&ReadComplex));
    assert_eq!(r1.len(), LPDDR4_EXAMPLE.len() - 1);
}

#[test]
fn warm_boots_rerun_only_the_gate_and_the_check() {
    for boot in [Boot::CacheRestore, Boot::SelfRefreshExit] {
        for mtps in [1066, 2400, 4266] {
            assert_eq!(run(mtps, boot, 0), [Gate, Sanity], "{boot:?} {mtps}");
            assert_eq!(run(mtps, boot, 1), [Gate, Sanity]);
        }
    }
}

#[test]
fn plan_stops_when_the_output_is_full_and_rates_compare_right() {
    let mut out = [Gate; 3];
    assert_eq!(plan(&LPDDR4_EXAMPLE, 2400, Boot::Cold, 0, &mut out), 3);
    assert_eq!(out, [CommandBus, Gate, WriteLeveling]);
    assert!(Rate::Below(1600).allows(1599) && !Rate::Below(1600).allows(1600));
    assert!(Rate::AtLeast(1600).allows(1600) && !Rate::Above(1600).allows(1600));
    // Board-specific ids work the same way.
    let mine = [
        Stage::new(7u8, Rate::Always).warm(),
        Stage::new(9u8, Rate::Above(3000)),
    ];
    let mut o = [0u8; 2];
    assert_eq!(plan(&mine, 3200, Boot::Cold, 0, &mut o), 2);
    assert_eq!(plan(&mine, 3200, Boot::CacheRestore, 0, &mut o), 1);
    assert_eq!(o[0], 7);
}
