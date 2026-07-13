// SPDX-License-Identifier: GPL-2.0-only
//! P3-A1: successive halving and profile smoothing on the recorded R2S and
//! b0 Vref profiles with run-to-run noise.

#[path = "recorded/fixtures.rs"]
mod rec;

use dram_train::vref_select::{concave_peak, mean3, median3, successive_halving, Arm};

struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        self.0.wrapping_mul(0x2545_f491_4f6c_dd1d)
    }
    fn below(&mut self, n: u64) -> u64 {
        self.next() % n
    }
}

/// One noisy reading: the true window, 1 lower 30 % of the time and 2 lower
/// 5 % (the R2S fp2 b0/f comparison: about one tap of run-to-run noise).
fn read(r: &mut Rng, truth: u8) -> u8 {
    match r.below(100) {
        0..=4 => truth.saturating_sub(2),
        5..=34 => truth.saturating_sub(1),
        _ => truth,
    }
}

/// The middle of the points tied on the best value.
fn argmax_mid(v: &[Option<u8>]) -> usize {
    let best = v.iter().flatten().max().copied().unwrap();
    let tied: Vec<usize> = (0..v.len()).filter(|&i| v[i] == Some(best)).collect();
    tied[(tied.len() - 1) / 2]
}

fn profiles() -> Vec<(&'static str, Vec<u8>)> {
    vec![
        ("r2s fp2 write", rec::R2S_FP2_WRITE[..51].to_vec()),
        ("r2s fp1 write", rec::R2S_FP1_WRITE[..51].to_vec()),
        ("r2s fp2 read", rec::R2S_FP2_READ.to_vec()),
        ("b0 fp0 rx", rec::B0_FP0_RX.0.to_vec()),
        ("b0 fp0 tx", rec::B0_FP0_TX.0.to_vec()),
        ("b0 fp2 rx", rec::B0_FP2_RX.0.to_vec()),
        ("b0 fp2 tx", rec::B0_FP2_TX.0.to_vec()),
    ]
}

#[derive(Default, Debug, Clone, Copy)]
struct Score {
    width: u64,
    best: u64,
    runs: u64,
}

fn bench(truth: &[u8], seeds: u64) -> [Score; 5] {
    let n = truth.len();
    let top = *truth.iter().max().unwrap();
    let mut s = [Score::default(); 5];
    for seed in 1..=seeds {
        let mut r = Rng(seed * 0x9e37_79b9 + 1);
        let pick = |k: usize, i: usize, runs: usize, s: &mut [Score; 5]| {
            s[k].width += truth[i] as u64;
            s[k].best += (truth[i] == top) as u64;
            s[k].runs += runs as u64;
        };
        // One reading each.
        let one: Vec<Option<u8>> = truth.iter().map(|&t| Some(read(&mut r, t))).collect();
        pick(0, argmax_mid(&one), n, &mut s);
        // Two readings each, the worst counts (the uniform x2 proposal).
        let two: Vec<Option<u8>> = one
            .iter()
            .zip(truth)
            .map(|(a, &t)| Some(a.unwrap().min(read(&mut r, t))))
            .collect();
        pick(1, argmax_mid(&two), 2 * n, &mut s);
        // Successive halving with the uniform x2 budget.
        let mut arms: Vec<Arm> = (0..n as u16).map(Arm::new).collect();
        let c = successive_halving(&mut arms, 2 * n as u32, |p| {
            Ok::<_, ()>(read(&mut r, truth[p as usize]))
        })
        .unwrap()
        .unwrap();
        pick(2, c.point as usize, c.runs as usize, &mut s);
        // One reading each, median-of-three smoothed.
        let mut m = vec![None; n];
        median3(&one, &mut m);
        pick(3, argmax_mid(&m), n, &mut s);
        // One reading each, concave fit (median argmax if no fit).
        let i = concave_peak(&one).unwrap_or_else(|| argmax_mid(&m));
        pick(4, i, n, &mut s);
    }
    s
}

/// Bench, 400 seeds per profile (Oct 15; best = share of picks on a true
/// maximum, w = mean true window at the pick):
///
/// | Profile | one | x2-min | halving (<= 2N runs) | median3 | concave |
/// |---|---|---|---|---|---|
/// | r2s fp2 read | 77 % | 68 % | 96 % | 36 % | 31 % |
/// | b0 fp0 rx | 69 % | 60 % | 94 % | 34 % | 0 % |
/// | b0 fp0 tx | 99 % | 95 % | 100 % | 87 % | 49 % |
/// | b0 fp2 tx | 98 % | 92 % | 99 % | 83 % | 38 % |
/// | r2s fp1/fp2 write, b0 fp2 rx | 100 % | 100 % | 100 % | 100 % | 100 % |
///
/// Successive halving wins on every profile with the uniform x2 budget;
/// keeping the worst of two readings is worse than one reading under
/// downward noise. Smoothing and the concave fit lose here: the recorded
/// profiles have one- or two-point peaks above a broad plateau, which the
/// smoothing flattens; they stay selectable, not defaults.
#[test]
fn halving_picks_the_true_best_most_often_on_recorded_profiles() {
    for (name, p) in profiles() {
        let s = bench(&p, 400);
        let pct = |k: usize| s[k].best * 100 / 400;
        assert!(pct(2) >= pct(0).max(pct(1)), "{name}: {s:?}");
        assert!(s[2].width >= s[0].width.max(s[1].width), "{name}");
        assert!(s[2].runs <= 400 * 2 * p.len() as u64, "{name}: budget");
        if name == "r2s fp2 read" || name == "b0 fp0 rx" {
            assert!(pct(2) >= 90 && pct(1) <= 70, "{name}: {s:?}");
        }
    }
}

#[test]
fn halving_rounds_budget_and_plateau_ties() {
    // 16 candidates, budget 31: rounds of 16, 8, 4, 2, 1.
    let truth = [5u8, 6, 7, 8, 9, 9, 9, 9, 9, 8, 7, 6, 5, 4, 3, 2];
    let mut arms: Vec<Arm> = (0..16).map(Arm::new).collect();
    let c = successive_halving(&mut arms, 31, |p| Ok::<_, ()>(truth[p as usize]))
        .unwrap()
        .unwrap();
    assert_eq!((c.runs, c.rounds, c.finalists), (31, 5, 1));
    assert!(truth[c.point as usize] == 9);
    // Budget for two rounds only: the plateau's middle among the leaders.
    let mut arms: Vec<Arm> = (0..16).map(Arm::new).collect();
    let c = successive_halving(&mut arms, 24, |p| Ok::<_, ()>(truth[p as usize]))
        .unwrap()
        .unwrap();
    assert_eq!((c.runs, c.rounds, c.finalists), (24, 2, 4));
    assert!(
        (5..=7).contains(&c.point),
        "the plateau 4..=8 lost its ends: {c:?}"
    );
    // No round fits, or nothing to choose.
    let mut arms: Vec<Arm> = (0..16).map(Arm::new).collect();
    assert_eq!(
        successive_halving(&mut arms, 15, |_| Ok::<_, ()>(1)),
        Ok(None)
    );
    assert_eq!(
        successive_halving(&mut [], 15, |_| Ok::<_, ()>(1)),
        Ok(None)
    );
    // A measurement error ends the search.
    let mut arms: Vec<Arm> = (0..4).map(Arm::new).collect();
    assert_eq!(
        successive_halving(&mut arms, 8, |_| Err::<u8, _>("t")),
        Err("t")
    );
}

#[test]
fn smoothing_and_the_concave_fit() {
    let p: Vec<Option<u8>> = [4, 6, 15, 8, 9, 10, 9, 8, 6, 4]
        .iter()
        .map(|&v| Some(v))
        .collect();
    let mut m = vec![None; p.len()];
    median3(&p, &mut m);
    assert_eq!(m[2], Some(8), "a one-point spike is removed");
    assert_eq!(m[5], Some(9));
    assert_eq!((m[0], m[9]), (Some(4), Some(4)), "ends: the lower of two");
    mean3(&p, &mut m);
    assert_eq!(m[5], Some(9));
    // Missing points pass through.
    let q = [Some(5), None, Some(7), Some(9), Some(8)];
    let mut m = [None; 5];
    median3(&q, &mut m);
    assert_eq!(m, [Some(5), None, Some(7), Some(8), Some(8)]);
    // A concave profile peaks between points: the fit finds it.
    let c: Vec<Option<u8>> = (0..21)
        .map(|x: i32| Some((100 - (x - 13) * (x - 13) / 2).max(0) as u8))
        .collect();
    assert_eq!(concave_peak(&c), Some(13));
    // Not concave, or too few points.
    let v: Vec<Option<u8>> = (0..9)
        .map(|x: i32| Some(((x - 4) * (x - 4) + 10) as u8))
        .collect();
    assert_eq!(concave_peak(&v), None);
    assert_eq!(concave_peak(&[Some(5), Some(6)]), None);
    assert_eq!(concave_peak(&[None, None]), None);
}

#[test]
#[ignore]
fn print_bench() {
    for (name, p) in profiles() {
        let s = bench(&p, 400);
        println!("{name:14} {:?}", s.map(|x| (x.best * 100 / 400, x.width)));
    }
}
