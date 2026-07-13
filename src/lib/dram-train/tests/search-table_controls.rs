// SPDX-License-Identifier: GPL-2.0-only
use dram_train::search::{
    interval_table::{Contract, Table},
    matrix::MatrixEye,
    Cause, Point,
};
fn empty() -> [u8; 132] {
    let mut r = [0; 132];
    for row in 0..64 {
        r[2 * row] = 255;
    }
    r[128..].copy_from_slice(&[0x6a, 0x95, 0xa5, 0x5a]);
    r
}
fn contract(width: u16) -> Contract {
    Contract {
        width,
        empty: [255, 0],
    }
}
#[test]
fn explicit_contract_rejects_malformed_rows_and_ignores_metadata() {
    let mut r = empty();
    assert!(matches!(
        Table::new(&r, contract(0)),
        Err(Cause::InvalidPlan)
    ));
    assert!(matches!(
        Table::new(&r, contract(257)),
        Err(Cause::InvalidPlan)
    ));
    assert!(matches!(
        Table::new(
            &r,
            Contract {
                width: 128,
                empty: [0, 0]
            }
        ),
        Err(Cause::InvalidPlan)
    ));
    r[0] = 7;
    r[1] = 6;
    assert!(matches!(
        Table::new(&r, contract(128)),
        Err(Cause::InvalidPlan)
    ));
    r[0] = 0;
    r[1] = 128;
    assert!(matches!(
        Table::new(&r, contract(128)),
        Err(Cause::InvalidPlan)
    ));
    for row in 0..64 {
        r[2 * row] = 127;
        r[2 * row + 1] = 0;
    }
    assert!(matches!(
        Table::new(&r, contract(128)),
        Err(Cause::InvalidPlan)
    ));
    assert!(Table::new(
        &r,
        Contract {
            width: 128,
            empty: [127, 0]
        }
    )
    .is_ok());
    r[128..].copy_from_slice(&[255; 4]);
    assert!(Table::new(
        &r,
        Contract {
            width: 128,
            empty: [127, 0]
        }
    )
    .is_ok());
}
#[test]
fn empty_is_not_a_result_and_no_input_bytes_change() {
    let r = empty();
    let before = r;
    let t = Table::new(&r, contract(128)).unwrap();
    assert_eq!(t.geometry(), Err(Cause::NoWindow));
    assert_eq!(t.select(1, 1), Err(Cause::NoWindow));
    assert!(!t.contains(Point { x: 0, y: 0 }));
    assert_eq!(r, before);
    assert_eq!(t.select(0, 1), Err(Cause::InvalidPlan));
    assert_eq!(t.select(1, 65), Err(Cause::InvalidPlan));
}
#[test]
fn centroid_gap_is_detected_and_safe_rectangle_is_used() {
    let mut r = empty();
    r[0] = 0;
    r[1] = 2;
    r[126] = 100;
    r[127] = 102;
    let t = Table::new(&r, contract(128)).unwrap();
    let g = t.geometry().unwrap();
    assert_eq!(g.passing_points, 6);
    assert_eq!(g.centroid, Point { x: 51, y: 32 });
    assert!(!g.centroid_passes);
    let e = t.select(1, 1).unwrap();
    assert_eq!(e.center, Point { x: 1, y: 0 });
    assert!(t.contains(e.center));
    assert_eq!((e.width, e.height), (3, 1));
    assert_eq!(t.select(1, 2), Err(Cause::NoWindow));
}
#[test]
fn inclusive_full_width_and_half_up_rounding_do_not_narrow() {
    let mut r = empty();
    for row in 0..64 {
        r[2 * row] = 0;
        r[2 * row + 1] = 255;
    }
    let t = Table::new(&r, contract(256)).unwrap();
    let g = t.geometry().unwrap();
    assert_eq!(g.passing_points, 16384);
    assert_eq!(g.centroid, Point { x: 128, y: 32 });
    assert!(g.centroid_passes);
    let e = t.select(256, 64).unwrap();
    assert_eq!((e.width, e.height), (256, 64));
    assert_eq!(e.center, Point { x: 127, y: 31 });
    let mut r = empty();
    r[0] = 0;
    r[1] = 0;
    r[2] = 1;
    r[3] = 1;
    let t = Table::new(&r, contract(2)).unwrap();
    let g = t.geometry().unwrap();
    assert_eq!(g.centroid, Point { x: 1, y: 1 });
    assert!(g.centroid_passes);
    assert!(!t.contains(Point { x: 2, y: 1 }));
    assert!(!t.contains(Point { x: 1, y: 64 }));
}
fn better(a: MatrixEye, b: MatrixEye) -> bool {
    let score = |e: MatrixEye| {
        (
            u32::from(e.width) * u32::from(e.height),
            e.width.min(e.height),
            e.width,
        )
    };
    score(a) > score(b)
        || (score(a) == score(b)
            && (a.center.y, a.center.x, a.start.y, a.start.x)
                < (b.center.y, b.center.x, b.start.y, b.start.x))
}
#[test]
fn independently_enumerated_point_and_rectangle_oracles() {
    let mut random = 0x733b55u32;
    for case in 0..160 {
        let w = 1 + case % 9;
        let h = 1 + case % 8;
        let mut r = empty();
        let mut cells = Vec::new();
        for row in 0..h {
            random ^= random << 13;
            random ^= random >> 17;
            random ^= random << 5;
            if random.is_multiple_of(4) {
                continue;
            }
            let a = random % w;
            let b = (random >> 8) % w;
            let lo = a.min(b);
            let hi = a.max(b);
            r[2 * row as usize] = lo as u8;
            r[2 * row as usize + 1] = hi as u8;
            for x in lo..=hi {
                cells.push(Point {
                    x: x as u8,
                    y: row as u8,
                });
            }
        }
        let t = Table::new(&r, contract(w as u16)).unwrap();
        for y in 0..64 {
            for x in 0..w {
                let p = Point { x: x as u8, y };
                assert_eq!(t.contains(p), cells.contains(&p));
            }
        }
        if cells.is_empty() {
            assert_eq!(t.geometry(), Err(Cause::NoWindow));
            assert_eq!(t.select(1, 1), Err(Cause::NoWindow));
            continue;
        }
        let n = cells.len() as u32;
        let sx = cells.iter().map(|p| u32::from(p.x)).sum::<u32>();
        let sy = cells.iter().map(|p| u32::from(p.y)).sum::<u32>();
        let c = Point {
            x: ((2 * sx + n) / (2 * n)) as u8,
            y: ((2 * sy + n) / (2 * n)) as u8,
        };
        let g = t.geometry().unwrap();
        assert_eq!(
            (g.passing_points, g.centroid, g.centroid_passes),
            (n, c, cells.contains(&c))
        );
        let mut best = None;
        for top in 0..h {
            for left in 0..w {
                for height in 1..=h - top {
                    for width in 1..=w - left {
                        let mut valid = true;
                        for y in top..top + height {
                            for x in left..left + width {
                                valid &= cells.contains(&Point {
                                    x: x as u8,
                                    y: y as u8,
                                });
                            }
                        }
                        if !valid {
                            continue;
                        }
                        let e = MatrixEye {
                            start: Point {
                                x: left as u8,
                                y: top as u8,
                            },
                            width: width as u16,
                            height: height as u16,
                            center: Point {
                                x: (left + (width - 1) / 2) as u8,
                                y: (top + (height - 1) / 2) as u8,
                            },
                            boundary_contact: left == 0
                                || left + width == w
                                || top == 0
                                || top + height == 64,
                        };
                        if best.is_none_or(|old| better(e, old)) {
                            best = Some(e);
                        }
                    }
                }
            }
        }
        assert_eq!(t.select(1, 1).unwrap(), best.unwrap(), "case{case}");
    }
}
