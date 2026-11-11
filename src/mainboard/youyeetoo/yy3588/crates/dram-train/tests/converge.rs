// SPDX-License-Identifier: GPL-2.0-only
//! dram_train::edge::converge: agreement of restarted searches.

use dram_train::edge::converge;

#[test]
fn agreeing_runs_give_the_lower_median() {
    let c = converge(&[Some(520), Some(512), Some(516)], 16).unwrap();
    assert_eq!(
        (c.results, c.missing, c.min, c.max),
        (3, 0, Some(512), Some(520))
    );
    assert_eq!(c.agreed, Some(516));
    assert_eq!(
        converge(&[Some(512), Some(520)], 8).unwrap().agreed,
        Some(512)
    );
    assert_eq!(converge(&[Some(7)], 0).unwrap().agreed, Some(7));
}

#[test]
fn spread_beyond_tolerance_or_a_missing_run_disagrees() {
    let c = converge(&[Some(448), Some(512), Some(576)], 64).unwrap();
    assert_eq!(c.agreed, None);
    assert_eq!((c.min, c.max), (Some(448), Some(576)));
    let c = converge(&[Some(512), None, Some(512)], 64).unwrap();
    assert_eq!((c.results, c.missing, c.agreed), (2, 1, None));
    let c = converge(&[], 64).unwrap();
    assert_eq!((c.results, c.agreed), (0, None));
    assert!(converge(&[Some(1); 256], 0).is_none());
}

#[test]
fn agreement_is_independent_of_order() {
    let mut s = 0x1234_5678u32;
    let mut next = || {
        s ^= s << 13;
        s ^= s >> 17;
        s ^= s << 5;
        s
    };
    for _ in 0..5000 {
        let n = 1 + (next() % 6) as usize;
        let mut v: Vec<Option<u16>> = (0..n).map(|_| Some(500 + (next() % 40) as u16)).collect();
        let tol = (next() % 48) as u16;
        let a = converge(&v, tol).unwrap();
        v.reverse();
        let b = converge(&v, tol).unwrap();
        assert_eq!(a, b);
        let mut sorted: Vec<u16> = v.iter().flatten().copied().collect();
        sorted.sort();
        if let Some(x) = a.agreed {
            assert_eq!(x, sorted[(n - 1) / 2]);
            assert!(sorted[n - 1] - sorted[0] <= tol);
        } else {
            assert!(sorted[n - 1] - sorted[0] > tol);
        }
    }
}
