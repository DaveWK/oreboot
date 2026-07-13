// SPDX-License-Identifier: GPL-2.0-only
//! P3-M3: the DQS gate pattern match against a returning-strobe plant
//! (undriven bus, preamble, toggling burst, noise around every edge).

use dram_train::gate::{gapped_reads, search, Class, Gate, GateError, Params, Probe};

const FPC: u32 = 32;
const CYCLE: u32 = 4 * FPC;

fn params() -> Params {
    Params {
        coarse_per_cycle: 4,
        fine_per_coarse: FPC as u16,
        coarse_steps: 24,
        samples: 16,
        min_zero: 3,
        retries: 3,
        start_latency: 2,
        max_latency: 12,
        post_cycles: 1,
    }
}

/// The strobe as the gate sees it, in fine steps from latency 0.
#[derive(Clone)]
struct Strobe {
    /// Preamble start.
    t0: u32,
    /// Preamble length.
    pre: u32,
    /// High time of each cycle.
    high: u32,
    /// Toggling cycles.
    burst: u32,
    /// Half-width of the noise around each edge.
    jitter: u32,
    /// A spurious high pulse [a, b) before the preamble (ringing).
    ring: Option<(u32, u32)>,
    /// Override: every sample this class.
    stuck: Option<Class>,
    probes: u32,
}

impl Strobe {
    fn new(t0: u32) -> Self {
        Self {
            t0,
            pre: 2 * CYCLE,
            high: CYCLE / 2,
            burst: 4,
            jitter: 5,
            ring: None,
            stuck: None,
            probes: 0,
        }
    }
    /// The third edge (second rise).
    fn third(&self) -> u32 {
        self.t0 + self.pre + CYCLE
    }
    fn level(&self, t: u32) -> Class {
        if let Some(c) = self.stuck {
            return c;
        }
        let start = self.t0 + self.pre;
        let end = start + self.burst * CYCLE;
        let mut edges = vec![self.t0, end];
        for k in 0..self.burst {
            edges.push(start + k * CYCLE);
            edges.push(start + k * CYCLE + self.high);
        }
        if edges.iter().any(|&e| t.abs_diff(e) < self.jitter) {
            return Class::Noise;
        }
        if t < self.t0 {
            return match self.ring {
                Some((a, b)) if (a..b).contains(&t) => Class::One,
                _ => Class::Noise, // undriven
            };
        }
        if t < start {
            return Class::Zero;
        }
        if t >= end {
            return Class::Noise;
        }
        if (t - start) % CYCLE < self.high {
            Class::One
        } else {
            Class::Zero
        }
    }
}

impl Probe for Strobe {
    type Error = ();
    fn sample(&mut self, lat: u8, c: u16, f: u16, n: u16) -> Result<u16, ()> {
        self.probes += 1;
        let t = u32::from(lat) * CYCLE + u32::from(c) * FPC + u32::from(f);
        Ok(match self.level(t) {
            Class::Zero => 0,
            Class::One => n,
            Class::Noise => n / 2,
        })
    }
}

fn abs(g: &Gate) -> u32 {
    u32::from(g.latency) * CYCLE + u32::from(g.coarse) * FPC + u32::from(g.fine)
}

#[test]
fn the_gate_centres_on_the_third_edge_at_every_phase() {
    for t0 in (5 * CYCLE..6 * CYCLE).step_by(3) {
        let mut s = Strobe::new(t0);
        let mut scratch = [Class::Noise; 32];
        let g = search(&mut s, &params(), &mut scratch).unwrap();
        assert_eq!(abs(&g), s.third(), "t0 {t0}: {g:?}");
        assert_eq!(g.noise, 2 * 5 - 1, "the noise region is the edge jitter");
        assert_eq!(g.read_latency, g.latency + 1);
        assert!(g.coarse < 24 && u32::from(g.fine) < FPC);
    }
}

#[test]
fn ringing_before_the_preamble_is_not_an_edge() {
    let t0 = 5 * CYCLE + 40;
    let mut s = Strobe::new(t0);
    s.ring = Some((t0 - 3 * FPC, t0 - FPC));
    let mut scratch = [Class::Noise; 24];
    let g = search(&mut s, &params(), &mut scratch).unwrap();
    assert_eq!(abs(&g), s.third());
}

#[test]
fn narrow_high_pulses_need_the_45_degree_retry() {
    // A high time of 24 fine steps (68 degrees) leaves 14 steps of stable 1
    // between the edge noise: a 90-degree grid can miss every one of them.
    let mut retried = 0;
    let mut missed = 0;
    for t0 in 5 * CYCLE..6 * CYCLE {
        let mut s = Strobe::new(t0);
        s.high = 24;
        let mut scratch = [Class::Noise; 24];
        match search(&mut s, &params(), &mut scratch) {
            Ok(g) => {
                assert_eq!(abs(&g), s.third(), "t0 {t0}");
                if g.retries > 0 {
                    retried += 1;
                }
            }
            // Pulses between 45 and 90 degrees after a grid point stay
            // hidden: the search gives up, it never locks on a wrong edge.
            Err(e) => {
                assert_eq!(e, GateError::MaxLatency);
                missed += 1;
            }
        }
    }
    assert!(retried > 20, "{retried} phases found by a retry");
    assert!(missed < 32, "{missed} phases missed");
}

#[test]
fn latency_steps_up_until_the_strobe_is_in_the_sweep() {
    let t0 = 9 * CYCLE + 7;
    let mut s = Strobe::new(t0);
    let mut scratch = [Class::Noise; 24];
    let g = search(&mut s, &params(), &mut scratch).unwrap();
    assert_eq!(abs(&g), s.third());
    assert!(g.latency_steps > 0);
    assert_eq!(g.latency, params().start_latency + g.latency_steps);
}

#[test]
fn the_pg150_failure_classes() {
    let mut scratch = [Class::Noise; 24];
    for (stuck, want) in [
        (Class::Zero, GateError::AllZero),
        (Class::One, GateError::AllOne),
        (Class::Noise, GateError::MaxLatency),
    ] {
        let mut s = Strobe::new(0);
        s.stuck = Some(stuck);
        assert_eq!(search(&mut s, &params(), &mut scratch), Err(want));
    }
    // Too late for the maximum latency.
    let mut s = Strobe::new(40 * CYCLE);
    assert_eq!(
        search(&mut s, &params(), &mut scratch),
        Err(GateError::MaxLatency)
    );
    let mut p = params();
    p.coarse_steps = 25;
    assert_eq!(
        search(&mut Strobe::new(0), &p, &mut scratch),
        Err(GateError::Config)
    );
}

#[test]
fn gapped_reads_must_all_come_back() {
    assert_eq!(gapped_reads(10, Ok::<u32, ()>), Ok(Ok(())));
    assert_eq!(gapped_reads(10, |_| Ok::<u32, ()>(9)), Ok(Err(9)));
    assert_eq!(
        gapped_reads(10, |_| Err::<u32, _>("timeout")),
        Err("timeout")
    );
}
