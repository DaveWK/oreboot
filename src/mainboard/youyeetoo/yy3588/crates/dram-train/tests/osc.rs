// SPDX-License-Identifier: GPL-2.0-only
//! P3-M13: oscillator count to delay, drift since boot, per-rank write
//! delay tracking, the rank schedule and the dummy-read rule.

use dram_train::osc::{
    dummy_read_due, schedule, Osc, OscError, Sense, Slot, Tracker, DUMMY_READ_INTERVAL_US,
};
use dram_train::ruler::Ruler;

const OSC: Osc = Osc {
    tck_ps: 625,
    runtime_tck: 512 * 16, // a run time of 8192 clocks
    divisor: 2,
    min_ps: 200,
    max_ps: 800,
};

#[test]
fn counts_give_the_delay() {
    // 8192 x 625 ps / (2 x 8000) = 320 ps.
    assert_eq!(OSC.delay_ps(8000), Ok(320));
    assert_eq!(OSC.delay_ps(6400), Ok(400));
    assert_eq!(OSC.delay_ps(0), Err(OscError::Zero));
    assert_eq!(OSC.delay_ps(2000), Err(OscError::Range { ps: 1280 }));
    assert_eq!(OSC.delay_ps(20000), Err(OscError::Range { ps: 128 }));
}

fn tracker(sense: Sense) -> Tracker<2> {
    // 64 taps per quarter clock at 625 ps: 2.44 ps per tap.
    let ruler = Ruler::from_quarter(625, 64).unwrap();
    Tracker::new(OSC, ruler, [(320, 100), (330, 120)], 4, sense)
}

#[test]
fn the_write_delay_follows_drift_beyond_the_threshold() {
    let mut t = tracker(Sense::DqFollows);
    // 320 -> 325 ps: 2 taps, below the threshold of 4.
    let c325 = (8192 * 625 / (2 * 325)) as u32;
    assert_eq!(t.update(0, c325), Ok(None));
    // 320 -> 340 ps: 8 taps, the DQ delay follows.
    let c340 = (8192 * 625 / (2 * 340)) as u32;
    assert_eq!(t.update(0, c340), Ok(Some(108)));
    assert_eq!(t.ranks[0].taps, 108);
    // Back towards 320: 2 taps off the set value, no change; then back.
    let c335 = (8192 * 625 / (2 * 335)) as u32;
    assert_eq!(t.update(0, c335), Ok(None));
    let c320 = 8000;
    assert_eq!(t.update(0, c320), Ok(Some(100)));
    let r = t.ranks[0];
    assert_eq!((r.drift.boot, r.drift.min, r.drift.max), (320, 320, 340));
    assert_eq!((r.taps_drift.min, r.taps_drift.max), (100, 108));
    assert_eq!(r.drift.excursion(), 20);
    // Rank 1 is untouched.
    assert_eq!(t.ranks[1].taps, 120);
    // The other sense moves the other way.
    let mut s = tracker(Sense::StrobeLeads);
    assert_eq!(s.update(0, c340), Ok(Some(92)));
}

#[test]
fn errors_leave_the_state_alone() {
    let mut t = tracker(Sense::DqFollows);
    let before = t;
    assert_eq!(t.update(0, 0), Err(OscError::Zero));
    assert_eq!(t.update(2, 8000), Err(OscError::Rank));
    assert_eq!(t, before);
}

#[test]
fn ranks_take_turns_and_one_rank_needs_idle() {
    let s: Vec<Slot> = (0..4).map(|k| schedule(2, k)).collect();
    assert_eq!(s.iter().map(|x| x.rank).collect::<Vec<_>>(), [0, 1, 0, 1]);
    assert!(s.iter().all(|x| !x.needs_idle));
    assert_eq!(
        schedule(1, 7),
        Slot {
            rank: 0,
            needs_idle: true
        }
    );
    assert_eq!(schedule(0, 3).rank, 0);
}

#[test]
fn a_dummy_read_every_20_us_without_traffic() {
    assert_eq!(DUMMY_READ_INTERVAL_US, 20);
    assert!(!dummy_read_due(100, 119, 0));
    assert!(dummy_read_due(100, 120, 0));
    assert!(
        dummy_read_due(100, 110, 10),
        "a shorter interval is allowed"
    );
    assert!(dummy_read_due(u64::MAX - 5, 15, 0), "timer wrap");
}

#[test]
fn lp4_oscillator_from_the_mode_registers() {
    // MR23 0xc0 = 8192 clocks, divisor 2 from jedec-lpddr.
    let o = Osc::lp4_mr23(625, 0xc0, 200, 800).unwrap();
    assert_eq!(o, OSC);
    assert_eq!(
        Osc::lp4_mr23(625, 0x00, 200, 800),
        None,
        "stop by MPC: no run time"
    );
    assert_eq!(Osc::lp4_mr23(625, 0x41, 200, 800), None, "reserved code");
    assert_eq!(
        Osc::lp4(
            625,
            jedec_lpddr::lp4::mr::OscRunTime::Clocks(2048),
            200,
            800
        )
        .unwrap()
        .runtime_tck,
        2048
    );
    // The count as MR18/MR19 read back drives the same tracking.
    let mut a = tracker(Sense::DqFollows);
    let mut b = tracker(Sense::DqFollows);
    let c340 = (8192 * 625 / (2 * 340)) as u32;
    let (lo, hi) = jedec_lpddr::lp4::mr::dqs_osc_split(c340 as u16);
    assert_eq!(a.update(0, c340), b.update_mr18_19(0, lo, hi));
    assert_eq!(a.ranks, b.ranks);
    assert_eq!(b.update_mr18_19(0, 0, 0), Err(OscError::Zero));
}

#[test]
fn lp5_wck2dq_oscillator_from_the_mode_registers() {
    // MR37 (WCK2DQI) or MR40 (WCK2DQO) run time, the MR23 coding;
    // divisor 2 from jedec-lpddr; the count as MR35/MR36 read back.
    use jedec_lpddr::lp5::mr::{osc_count, wck2dq_osc_delay_ps, Wck2DqOsc, WCK2DQ_OSC_DIVISOR};
    assert_eq!(WCK2DQ_OSC_DIVISOR, 2);
    let o = Osc::lp5_mr(625, 0xc0, 200, 800).unwrap();
    assert_eq!(o, OSC, "the same arithmetic as the LPDDR4 oscillator");
    assert_eq!(Osc::lp5_mr(625, 0x00, 200, 800), None);
    assert_eq!(Osc::lp5_mr(625, 0x81, 200, 800), None);
    assert_eq!(o.delay_ps(8000), Ok(320));
    assert_eq!(wck2dq_osc_delay_ps(8192, 625, 8000), Some(320));
    assert_eq!(wck2dq_osc_delay_ps(8192, 625, 0), None);
    let (lsb_mr, msb_mr, run_mr) = Wck2DqOsc::Input.registers();
    assert_eq!((lsb_mr, msb_mr, run_mr), (35, 36, 37));
    let mut a = tracker(Sense::StrobeLeads);
    let mut b = tracker(Sense::StrobeLeads);
    let c340 = (8192 * 625 / (2 * 340)) as u32;
    let (lsb, msb) = (c340 as u8, (c340 >> 8) as u8);
    assert_eq!(osc_count(lsb, msb), c340 as u16);
    assert_eq!(a.update(1, c340), b.update_osc_regs(1, lsb, msb));
    assert_eq!(a.ranks, b.ranks);
    assert_eq!(b.update_osc_regs(0, 0, 0), Err(OscError::Zero));
}
