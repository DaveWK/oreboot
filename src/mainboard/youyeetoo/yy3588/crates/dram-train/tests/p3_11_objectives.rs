// SPDX-License-Identifier: GPL-2.0-only
//! The optional objectives. `vref_sweep::Objective::MaxSum` picks
//! the point with the largest lane-window sum wherever it lies (the
//! `min_window` gate still applies); `search::Objective::Centroid` reports
//! the pass set's centroid beside the rectangle and applies it only when it
//! passes inside the rectangle.

use dram_train::search::{Cause, Centres, Domain, Grid, Objective, Point};

fn domain(w: u8, h: u8) -> Domain {
    Domain {
        width: w,
        height: h,
        circular_x: false,
        circular_y: false,
        min_width: 1,
        min_height: 1,
        scan_repeats: 1,
        verify_repeats: 1,
        polls_per_probe: 1,
        probe_operations: 1,
    }
}

fn grid(rows: &[&str]) -> Grid {
    let mut g = Grid::default();
    for (y, r) in rows.iter().enumerate() {
        for (x, c) in r.chars().enumerate() {
            g.set(
                Point {
                    x: x as u8,
                    y: y as u8,
                },
                c == '#',
            )
            .unwrap();
        }
    }
    g
}

#[test]
fn centroid_beside_the_rectangle() {
    // A rectangle 3 wide, 2 high plus a tail of passes to the right on one
    // row: the centroid leans right of the rectangle centre.
    let g = grid(&["...........", ".###.......", ".###.####..", "..........."]);
    let d = domain(11, 4);
    let c: Centres = g.centres(d).unwrap();
    assert_eq!(
        (c.rectangle.start, c.rectangle.width, c.rectangle.height),
        (Point { x: 1, y: 1 }, 3, 2)
    );
    assert_eq!(c.rectangle.center, Point { x: 2, y: 1 });
    assert_eq!(c.passing_points, 10);
    // x: (1+2+3)*2 + 5+6+7+8 = 38 / 10 = 3.8 -> 4; y: (1*3 + 2*7) = 17 / 10 -> 2.
    assert_eq!(c.centroid, Point { x: 4, y: 2 });
    assert!(!c.centroid_passes, "x=4,y=2 is the gap");
    assert!(!c.centroid_in_rectangle);
    // Objective::Centroid then keeps the rectangle centre.
    let e = g.select_with(d, Objective::Centroid).unwrap();
    assert_eq!(e.center, c.rectangle.center);
    assert_eq!(
        g.select_with(d, Objective::Rectangle).unwrap(),
        g.select(d).unwrap()
    );
}

#[test]
fn a_passing_centroid_inside_the_rectangle_is_applied() {
    // A 5x3 rectangle with extra passes on its right edge rows: the centroid
    // moves one to the right of the rectangle centre and still passes.
    let g = grid(&["#####..", "#####..", "#####..", ".....##", ".....##"]);
    let d = domain(7, 5);
    let c = g.centres(d).unwrap();
    assert_eq!(c.rectangle.center, Point { x: 2, y: 1 });
    // x: 15*2 + 4*5.5 = 30+22 = 52 / 19 = 2.7 -> 3; y: 5*(0+1+2) + 2*(3+4) =
    // 15+14 = 29/19 -> 2 (1.53).
    assert_eq!(c.centroid, Point { x: 3, y: 2 });
    assert!(c.centroid_passes && c.centroid_in_rectangle);
    let e = g.select_with(d, Objective::Centroid).unwrap();
    assert_eq!(e.center, Point { x: 3, y: 2 });
    assert_eq!((e.start, e.width, e.height), (Point { x: 0, y: 0 }, 5, 3));
    assert_eq!(Objective::default(), Objective::Rectangle);
}

#[test]
fn centroid_errors() {
    let g = grid(&["....", "...."]);
    assert_eq!(g.centres(domain(4, 2)).err(), Some(Cause::NoWindow));
    let mut d = domain(4, 2);
    d.circular_x = true;
    assert_eq!(
        grid(&["##..", "...."]).centres(d).err(),
        Some(Cause::InvalidPlan)
    );
}
