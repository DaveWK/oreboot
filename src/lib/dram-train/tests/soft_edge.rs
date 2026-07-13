// SPDX-License-Identifier: GPL-2.0-only
//! P3-A4: soft-information edges against Gaussian-jitter edge plants.

use dram_train::soft_edge::{eye_width_at, fit, probit_milli, Point, Q_1E_12};

/// Normal CDF (Abramowitz and Stegun 7.1.26 erf, error below 1.5e-7).
fn phi(z: f64) -> f64 {
    let x = z.abs() / std::f64::consts::SQRT_2;
    let t = 1.0 / (1.0 + 0.327_591_1 * x);
    let y = 1.0
        - (((((1.061_405_429 * t - 1.453_152_027) * t) + 1.421_413_741) * t - 0.284_496_736) * t
            + 0.254_829_592)
            * t
            * (-x * x).exp();
    let erf = if z >= 0.0 { y } else { -y };
    0.5 * (1.0 + erf)
}

struct Rng(u64);
impl Rng {
    fn unit(&mut self) -> f64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        (self.0.wrapping_mul(0x2545_f491_4f6c_dd1d) >> 11) as f64 / (1u64 << 53) as f64
    }
}

/// Error counts of a right edge (errors rise with the delay) at `mu` with
/// jitter `sigma`, `n` compares per point; `rng`: binomial sampling,
/// otherwise the expected counts.
fn edge(
    mu: f64,
    sigma: f64,
    n: u32,
    delays: std::ops::Range<i32>,
    rng: Option<&mut Rng>,
) -> Vec<Point> {
    let mut rng = rng;
    delays
        .map(|d| {
            let p = phi((d as f64 - mu) / sigma);
            let errors = match rng.as_deref_mut() {
                Some(r) => (0..n).filter(|_| r.unit() < p).count() as u32,
                None => (p * n as f64).round() as u32,
            };
            Point {
                delay: d,
                errors,
                compares: n,
            }
        })
        .collect()
}

#[test]
fn the_probit_table() {
    assert_eq!(probit_milli(1, 2), Some(0));
    assert_eq!(probit_milli(8, 16), Some(0));
    let q = probit_milli(1, 16).unwrap(); // -1.534
    assert!((q + 1534).abs() <= 2, "{q}");
    assert_eq!(probit_milli(15, 16), Some(-probit_milli(1, 16).unwrap()));
    assert_eq!(probit_milli(0, 16), None);
    assert_eq!(probit_milli(16, 16), None);
    let mut last = i32::MIN;
    for e in 1..1000 {
        let q = probit_milli(e, 1000).unwrap();
        assert!(q >= last, "monotone at {e}");
        last = q;
    }
}

#[test]
fn exact_counts_give_the_centre_and_sigma() {
    for (mu, sigma) in [(100.3, 2.0), (57.75, 4.5), (20.5, 1.2)] {
        let pts = edge(mu, sigma, 256, (mu as i32 - 15)..(mu as i32 + 16), None);
        let e = fit(&pts).unwrap();
        assert!(e.rising);
        let c = e.center_milli as f64 / 1000.0;
        let s = e.sigma_milli.unwrap() as f64 / 1000.0;
        assert!((c - mu).abs() < 0.1, "mu {mu}: {c}");
        assert!((s - sigma).abs() / sigma < 0.1, "sigma {sigma}: {s}");
        assert!(e.fitted >= 3);
    }
}

#[test]
fn sixteen_compares_with_sampling_noise_beat_the_pass_fail_edge() {
    // 200 edges, 16 compares per point (one A733 DTSM launch), binomial
    // noise: the fitted centre against the pass/fail edge (the midpoint
    // between the last point under half errors and the next).
    let mut r = Rng(0x1234_5678);
    let (mut soft, mut hard, mut worst) = (0.0f64, 0.0f64, 0.0f64);
    for k in 0..200 {
        let mu = 60.0 + (k % 10) as f64 * 0.1;
        let pts = edge(mu, 2.5, 16, 45..76, Some(&mut r));
        let e = fit(&pts).unwrap();
        let es = (e.center_milli as f64 / 1000.0 - mu).abs();
        let h = pts.windows(2).find(|w| w[1].errors * 2 > 16).unwrap();
        let eh = ((h[0].delay + h[1].delay) as f64 / 2.0 - mu).abs();
        soft += es;
        hard += eh;
        worst = worst.max(es);
    }
    let (soft, hard) = (soft / 200.0, hard / 200.0);
    println!("mean centre error: fitted {soft:.3} taps, pass/fail {hard:.3} taps, worst fitted {worst:.3}");
    assert!(soft < hard, "fitted {soft} vs pass/fail {hard}");
    assert!(soft < 0.35 && worst < 1.0, "{soft} {worst}");
}

#[test]
fn a_falling_edge_and_the_eye_at_a_target_rate() {
    // Left edge: errors fall with the delay (mirror of a right edge).
    let left: Vec<Point> = edge(-40.0, 2.0, 256, -55..-24, None)
        .into_iter()
        .map(|p| Point {
            delay: -p.delay,
            ..p
        })
        .collect();
    let l = fit(&left).unwrap();
    assert!(!l.rising);
    assert!((l.center_milli - 40_000).abs() < 100);
    let r = fit(&edge(80.0, 2.0, 256, 65..96, None)).unwrap();
    // 40 taps between the 50 % points; at 1e-12 each edge moves 7.03
    // sigma inwards: about 40 - 2 x 14.07.
    let w = eye_width_at(&l, &r, Q_1E_12) as f64 / 1000.0;
    assert!((w - (40.0 - 2.0 * 7.034 * 2.0)).abs() < 1.5, "{w}");
    assert_eq!(eye_width_at(&l, &r, 30_000), 0, "closed at an absurd rate");
}

#[test]
fn hard_edges_and_no_edge() {
    // A step with no partial points: the midpoint, no sigma.
    let step: Vec<Point> = (0..20)
        .map(|d| Point {
            delay: d,
            errors: if d >= 12 { 16 } else { 0 },
            compares: 16,
        })
        .collect();
    let e = fit(&step).unwrap();
    assert_eq!((e.center_milli, e.sigma_milli, e.fitted), (11_500, None, 0));
    assert_eq!(e.at_q(Q_1E_12), 11_500);
    // A flat fraction (dead knob): no edge.
    let flat: Vec<Point> = (0..20)
        .map(|d| Point {
            delay: d,
            errors: 8,
            compares: 16,
        })
        .collect();
    assert_eq!(fit(&flat), None);
    assert_eq!(fit(&[]), None);
}
