// SPDX-License-Identifier: GPL-2.0-only
//! lp4::timing tables and conversions.

use jedec_lpddr::lp4::timing::{freq_range, ps_to_ck, Min, FREQ_RANGES, MT53E256M16D1_1866};

#[test]
fn table_28_is_contiguous_and_monotonic() {
    for w in FREQ_RANGES.windows(2) {
        assert_eq!(w[0].f_max, w[1].f_min);
        assert!(w[0].rl < w[1].rl && w[0].wl_a < w[1].wl_a && w[0].nwr < w[1].nwr);
        assert!(w[0].rl_dbi >= w[0].rl && w[0].wl_b >= w[0].wl_a);
    }
    assert_eq!(FREQ_RANGES[0].f_min, 10);
    assert_eq!(FREQ_RANGES[7].f_max, 2133);
}

#[test]
fn freq_range_lookup_edges() {
    assert_eq!(freq_range(10), None);
    assert_eq!(freq_range(11).map(|r| r.0), Some(0));
    assert_eq!(freq_range(266).map(|r| r.0), Some(0));
    assert_eq!(freq_range(267).map(|r| r.0), Some(1));
    // K1 frequency points: 1200, 1600 and 2400 MT/s = 600, 800, 1200 MHz CK.
    assert_eq!(
        freq_range(600).map(|r| (r.0, r.1.rl, r.1.wl_a)),
        Some((2, 14, 8))
    );
    assert_eq!(
        freq_range(800).map(|r| (r.0, r.1.rl, r.1.wl_a)),
        Some((2, 14, 8))
    );
    assert_eq!(
        freq_range(1200).map(|r| (r.0, r.1.rl, r.1.nwr)),
        Some((4, 24, 24))
    );
    assert_eq!(freq_range(2133).map(|r| r.0), Some(7));
    assert_eq!(freq_range(2134), None);
}

#[test]
fn min_cycles_take_the_larger_bound() {
    assert_eq!(ps_to_ck(1000, 1000), 1);
    assert_eq!(ps_to_ck(1001, 1000), 2);
    assert_eq!(ps_to_ck(3_906_250, 800), 3125);
    let t = MT53E256M16D1_1866;
    // tFAW max(32 nCK, 40 ns): 32 at 600 MHz (24 ns-cycles), 75 at 1866 MHz.
    assert_eq!(t.t_faw.cycles(600), 32);
    assert_eq!(t.t_faw.cycles(1866), 75);
    assert_eq!(t.t_rcd.cycles(100), 4);
    assert_eq!(t.t_rcd.cycles(1866), 34);
    // tREFI is a maximum: 3906.25 ns at 1866 MHz is 7289.06 cycles, so 7289.
    assert_eq!(t.t_refi.cycles(1866), 7289);
    assert_eq!(t.t_refi.cycles(800), 3125);
    assert_eq!(Min::ns(0, 180).cycles(1200), 216);
}
