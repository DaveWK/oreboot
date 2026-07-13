// SPDX-License-Identifier: GPL-2.0-only
//! lp5::timing latency tables.

use jedec_lpddr::lp5::mr;
use jedec_lpddr::lp5::timing::{ck_mhz, freq_range, freq_ranges, FREQ_RANGES_2_1, FREQ_RANGES_4_1};
use jedec_lpddr::lp5::WckCkRatio;

#[test]
fn tables_are_contiguous_and_coded_by_row() {
    for t in [&FREQ_RANGES_2_1[..], &FREQ_RANGES_4_1[..]] {
        assert_eq!(t[0].dr_min, 40);
        for (i, r) in t.iter().enumerate() {
            assert_eq!(r.mr as usize, i);
            assert_eq!(r.n_wr_op as usize, i);
            assert!(r.wl[1] >= r.wl[0] && r.rl[2] >= r.rl[0]);
            // MR1/MR2 from the row encode the codes where the source puts them.
            assert_eq!(mr::mr1(r.mr, false, 0), Some(r.mr << 4));
            assert_eq!(mr::mr2(r.mr, r.n_wr_op), Some(r.mr | r.n_wr_op << 4));
        }
        for w in t.windows(2) {
            assert_eq!(w[0].dr_max, w[1].dr_min);
            assert!(w[0].rl[0] < w[1].rl[0] && w[0].n_wr < w[1].n_wr);
        }
    }
    assert_eq!(FREQ_RANGES_2_1[5].dr_max, 3200);
    assert_eq!(FREQ_RANGES_4_1[11].dr_max, 6400);
}

#[test]
fn lookups() {
    use WckCkRatio::{Four, Two};
    assert_eq!(freq_ranges(Two).len(), 6);
    assert_eq!(freq_range(Two, 40), None);
    assert_eq!(freq_range(Two, 533).map(|r| r.mr), Some(0));
    assert_eq!(freq_range(Two, 534).map(|r| r.mr), Some(1));
    assert_eq!(freq_range(Two, 3201), None);
    // A733 Cubie A7A: dram_clk 2400 MHz WCK = 4800 Mbps at WCK:CK 4:1.
    let r = freq_range(Four, 4800).unwrap();
    assert_eq!((r.mr, r.wl[0], r.rl[0], r.n_wr), (0b1000, 7, 13, 21));
    assert_eq!(ck_mhz(Four, 4800), 600);
    assert_eq!(ck_mhz(Two, 3200), 800);
    assert_eq!(freq_range(Four, 6401), None);
}
