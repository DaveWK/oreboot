// SPDX-License-Identifier: GPL-2.0-only
//! Board adapters of memtest against the code they came from: the A7A
//! main-stage flat map (ore-edk-boot-radxa-a7a oreboot tree a803f4f780d7,
//! main/src/memtest.rs `Map::enter`), the K1 memtest windows
//! (ore-edk-boot-opi 78479e21ba bt0/src/train_report.rs `memtest`).

// The A7A fill loop is kept as copied.
#![allow(clippy::needless_range_loop)]

use memtest::arch::aarch64::{l1_table, MAIR, TCR};
use memtest::{Plan, Region};

/// The table fill of A7A `Map::enter`, copied verbatim (its statements
/// between `let l1 = ...` and the TCR line; the rest of that function is
/// AArch64 register access).
fn a7a_fill(l1: &mut [u64; 512], end: u64) {
    l1.fill(0);
    l1[0] = 1 | (1 << 10) | (1 << 54); // Device nGnRnE, XN
    for gib in 1..end.div_ceil(1 << 30) as usize {
        l1[gib] = (gib as u64) << 30 | 1 | (1 << 10) | (3 << 8) | (1 << 2);
        if gib != 1 {
            l1[gib] |= 1 << 54;
        }
    }
}

#[test]
fn flat_map_matches_the_a7a_main_stage() {
    // The A733's DRAM starts at 1 GiB, where main runs: block 1 executable.
    for end in (0..=64u64)
        .map(|q| q << 28)
        .chain([0x1_4000_0001, 0x7_ffff_ffff])
    {
        let (mut a, mut b) = ([0u64; 512], [0u64; 512]);
        a7a_fill(&mut a, end);
        l1_table(&mut b, end, Some(1)).unwrap();
        assert_eq!(a, b, "end {end:#x}");
    }
    assert_eq!(MAIR, 0x4400);
    assert_eq!(TCR, 25u64 | (2 << 16) | (1 << 23) | (1 << 31));
    let mut t = [0u64; 512];
    assert!(l1_table(&mut t, 513 << 30, None).is_err());
    l1_table(&mut t, 4 << 30, None).unwrap();
    assert!(
        t[1..4].iter().all(|e| e & (1 << 54) != 0),
        "no exec block: all XN"
    );
}

/// The K1 memtest's regions per board profile, verbatim match arms.
fn k1_regions(mib: u32) -> [(usize, usize); 2] {
    match mib {
        4096 => [(0x1000, 0x8000_0000 - 0x1000), (0x1_0000_0000, 0x8000_0000)],
        2048 => [(0x1000, 0x8000_0000 - 0x1000), (0, 0)],
        _ => [(0x1000, 0x20_0000), (0, 0)],
    }
}

#[test]
fn k1_windows_plan_the_k1_memtest_regions() {
    // K1 DRAM windows: up to 2 GiB at 0, the rest above 4 GiB; the first
    // page (address 0) is left out.
    let r = |s: u64, e: u64| Region::new(s, e - s).unwrap();
    for (mib, windows) in [
        (2048u32, vec![r(0, 0x8000_0000)]),
        (
            4096,
            vec![r(0, 0x8000_0000), r(0x1_0000_0000, 0x1_8000_0000)],
        ),
    ] {
        let plan = Plan::windows(&windows, &[r(0, 0x1000)]).unwrap();
        let got: Vec<(usize, usize)> = plan
            .ranges()
            .iter()
            .map(|g| (g.start as usize, (g.end - g.start) as usize))
            .collect();
        let want: Vec<(usize, usize)> = k1_regions(mib).into_iter().filter(|x| x.1 != 0).collect();
        assert_eq!(got, want, "{mib} MiB");
    }
}

#[test]
fn windows_of_one_is_plan_new() {
    let mut s = 0x9e37_79b9_7f4a_7c15u64;
    let mut next = move || {
        s ^= s << 13;
        s ^= s >> 7;
        s ^= s << 17;
        s
    };
    let page = |x: u64| x & !0xfff;
    for _ in 0..20000 {
        let start = page(next() % (1 << 33));
        let size = page(next() % (1 << 32)) + 0x1000;
        let dram = match Region::new(start, size) {
            Ok(d) => d,
            Err(_) => continue,
        };
        let n = (next() % 9) as usize;
        let res: Vec<Region> = (0..n)
            .filter_map(|_| {
                let a = start.saturating_sub(0x1000) + page(next() % (size + 0x2000));
                Region::new(a, page(next() % (size / 2 + 0x2000)) + 0x1000).ok()
            })
            .collect();
        let a = Plan::new(dram, &res);
        let b = Plan::windows(&[dram], &res);
        match (a, b) {
            (Ok(a), Ok(b)) => {
                assert_eq!(a.ranges(), b.ranges());
                assert_eq!(a.bytes, b.bytes);
            }
            (Err(x), Err(y)) => {
                // Plan::new checks per-range bounds first; windows rejects a
                // range outside every window up front: same message there.
                assert!(x == y || (y == "reserved range outside DRAM"), "{x} vs {y}");
            }
            (a, b) => panic!("{:?} vs {:?}", a.map(|p| p.bytes), b.map(|p| p.bytes)),
        }
    }
}

#[test]
fn windows_reject_overlap_and_misplaced_reservations() {
    let r = |s: u64, e: u64| Region::new(s, e - s).unwrap();
    assert_eq!(
        Plan::windows(&[r(0, 0x2000), r(0x1000, 0x3000)], &[]).err(),
        Some("DRAM windows overlap")
    );
    assert_eq!(
        Plan::windows(&[r(0, 0x2000), r(0x4000, 0x6000)], &[r(0x1000, 0x5000)]).err(),
        Some("reserved range outside DRAM")
    );
    assert_eq!(
        Plan::windows(&[r(0, 0x1000)], &[r(0, 0x1000)]).err(),
        Some("no testable DRAM")
    );
}
