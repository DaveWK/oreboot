// SPDX-License-Identifier: GPL-2.0-only
//! Signed-axis boundaries, observed width and bounded remeasurement.
use dram_train::deskew::{
    centre_from_edges, per_bit_centre, CentreFrom, CentreParams, Edges, Measure,
};

struct Windows {
    windows: Vec<(i32, i32)>,
    delays: Vec<u16>,
    moves: Vec<(u8, u16)>,
    reads: usize,
}
impl Windows {
    fn new(windows: &[(i32, i32)]) -> Self {
        Self {
            windows: windows.to_vec(),
            delays: vec![0; windows.len()],
            moves: Vec::new(),
            reads: 0,
        }
    }
}
impl Measure for Windows {
    type Error = ();
    fn window(&mut self, bit: u8) -> Result<Option<(i32, i32)>, ()> {
        self.reads += 1;
        let b = usize::from(bit);
        let d = i64::from(self.delays[b]);
        let (lo, hi) = self.windows[b];
        Ok(Some((
            i32::try_from(i64::from(lo) + d).unwrap(),
            i32::try_from(i64::from(hi) + d).unwrap(),
        )))
    }
    fn set_bit(&mut self, bit: u8, delay: u16) -> Result<(), ()> {
        self.delays[usize::from(bit)] = delay;
        self.moves.push((bit, delay));
        Ok(())
    }
}
const P: CentreParams = CentreParams {
    bits: 1,
    bit_max: u16::MAX,
    tolerance: 0,
    rounds: 0,
};

#[test]
fn midpoint_keeps_extreme_and_negative_half_taps() {
    for (lo, hi, centre) in [
        (i32::MAX - 4, i32::MAX, i32::MAX - 2),
        (i32::MIN, i32::MIN + 4, i32::MIN + 2),
        (-4, -1, -3),
        (-1, 0, -1),
        (0, 1, 0),
    ] {
        let mut m = Windows::new(&[(lo, hi)]);
        let out = per_bit_centre::<_, 1>(&mut m, &P, &[0]).unwrap();
        assert_eq!(
            (out.target, out.bits[0].before, out.bits[0].after),
            (centre, centre, centre)
        );
        assert!(out.bits[0].verified);
        assert_eq!(m.moves, [(0, 0)]);
    }
}

#[test]
fn full_signed_axis_width_is_reported_exactly() {
    let mut m = Windows::new(&[(i32::MIN, i32::MAX)]);
    let out = per_bit_centre::<_, 1>(&mut m, &P, &[0]).unwrap();
    assert_eq!(out.target, -1);
    assert_eq!(i64::from(out.min_width), i64::from(u32::MAX));
}

#[test]
fn distant_centres_saturate_without_wrapping_or_false_verification() {
    let mut m = Windows::new(&[(i32::MIN, i32::MIN), (i32::MAX, i32::MAX)]);
    let p = CentreParams { bits: 2, ..P };
    let out = per_bit_centre::<_, 2>(&mut m, &p, &[0, 0]).unwrap();
    assert_eq!(out.target, i32::MAX);
    assert_eq!(m.delays, [u16::MAX, 0]);
    assert_eq!(out.bits[0].after, i32::MIN + i32::from(u16::MAX));
    assert_eq!((out.saturated, out.unverified), (1, 1));
    assert!(!out.bits[0].verified && out.bits[1].verified);
}

struct Sequence {
    windows: std::collections::VecDeque<(i32, i32)>,
    moves: Vec<(u8, u16)>,
}
impl Measure for Sequence {
    type Error = ();
    fn window(&mut self, _: u8) -> Result<Option<(i32, i32)>, ()> {
        Ok(self.windows.pop_front())
    }
    fn set_bit(&mut self, bit: u8, delay: u16) -> Result<(), ()> {
        self.moves.push((bit, delay));
        Ok(())
    }
}
#[test]
fn reversed_window_is_refused_at_initial_or_repeat_measurement() {
    for windows in [vec![(5, 4)], vec![(0, 0), (5, 4)]] {
        let mut m = Sequence {
            windows: windows.into(),
            moves: Vec::new(),
        };
        assert!(per_bit_centre::<_, 1>(&mut m, &P, &[0]).is_err());
        assert_eq!(m.moves, [(0, 0)], "only the requested initial setting");
    }
}
#[test]
fn extreme_remeasurement_is_not_falsely_verified() {
    for (start, after) in [(0, i32::MIN), (i32::MIN, i32::MAX)] {
        let mut m = Sequence {
            windows: [(start, start), (after, after)].into(),
            moves: Vec::new(),
        };
        let out = per_bit_centre::<_, 1>(&mut m, &P, &[0]).unwrap();
        assert_eq!(out.bits[0].after, after);
        assert_eq!(out.unverified, 1);
        assert!(!out.bits[0].verified);
    }
}
#[test]
fn maximum_round_count_is_bounded_with_a_stuck_window() {
    let mut windows = vec![(0, 0)];
    windows.extend(std::iter::repeat_n((-1, -1), 256));
    let mut m = Sequence {
        windows: windows.into(),
        moves: Vec::new(),
    };
    let p = CentreParams {
        rounds: u8::MAX,
        ..P
    };
    let out = per_bit_centre::<_, 1>(&mut m, &p, &[0]).unwrap();
    assert_eq!(
        (out.rounds, out.unverified, out.bits[0].delay),
        (255, 1, 255)
    );
    assert!(m.windows.is_empty());
}
#[test]
fn partial_edge_midpoint_spans_the_signed_axis() {
    let e = Edges {
        rise_left: Some(i32::MIN),
        rise_right: Some(i32::MAX),
        ..Edges::default()
    };
    assert_eq!(
        centre_from_edges(e, 90, i32::MIN, i32::MAX),
        Some((-1, CentreFrom::Window))
    );
    let e = Edges {
        rise_right: Some(i32::MAX),
        ..Edges::default()
    };
    assert_eq!(
        centre_from_edges(e, 90, i32::MIN, i32::MAX),
        Some((-1, CentreFrom::EdgeAndLimit))
    );
}
#[test]
fn unrepresentable_previous_window_centre_is_refused() {
    for e in [
        Edges {
            fall_left: Some(i32::MIN),
            fall_right: Some(i32::MIN),
            ..Edges::default()
        },
        Edges {
            rise_right: Some(i32::MIN),
            fall_left: Some(i32::MIN),
            ..Edges::default()
        },
        Edges {
            fall_left: Some(i32::MIN),
            ..Edges::default()
        },
        Edges {
            fall_right: Some(i32::MIN),
            ..Edges::default()
        },
    ] {
        assert_eq!(centre_from_edges(e, 2, i32::MIN, i32::MIN), None);
    }
}
