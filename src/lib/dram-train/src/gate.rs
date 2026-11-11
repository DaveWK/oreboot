// SPDX-License-Identifier: GPL-2.0-only
//! DQS gate training as a pattern match: sweep the gate over the returning
//! read strobe in coarse steps, classify each position as stable 0, stable
//! 1 or noise from repeated samples, lock onto the third DQS edge after a
//! stable-0 preamble (skipping the preamble and spurious first edges),
//! confirm the stable 0 before it, then sweep fine through that edge's
//! noise region and centre the gate in it. When no position matches, retry
//! with the coarse grid shifted by 45 degrees, halving the shift on each
//! retry (duty-cycle distortion or aliasing can hide the high pulses from a
//! 90-degree grid); when the retries are spent, add one cycle of read
//! latency and start again, up to a maximum.
//!
//! Method: AMD PG150 v1.4 p736-740 (gate pattern match, 45-degree retry
//! halved per retry, stable 0 then centre in the noise, read latency +1),
//! PG353 v1.0 p202-204 (the third DQS edge), PG313 v1.1 p329 (the four
//! parts: third edge, stable 0, noise centre, ranks), AM019 `DQSGATE_*`;
//! the gapped-read check from PG150 p51 (cited, not copied).
//!
//! Positions are (latency, coarse, fine); a coarse step is
//! `fine_per_coarse` fine steps and a cycle is `coarse_per_cycle` coarse
//! steps. The board maps them to its delay lines.

/// What the gate saw at one position.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Class {
    Zero,
    One,
    Noise,
}

/// Classify `ones` out of `samples` (all 0, all 1, or mixed).
pub fn classify(ones: u16, samples: u16) -> Class {
    if ones == 0 {
        Class::Zero
    } else if ones >= samples {
        Class::One
    } else {
        Class::Noise
    }
}

/// Index of the third DQS edge in a coarse sweep: after a run of at least
/// `min_zero` stable zeros (the preamble), count changes of the stable value
/// (rising, falling, rising); noise between two stable values belongs to the
/// edge. Returns the index where the third edge starts (its first noise or
/// stable-1 position).
pub fn third_edge(classes: &[Class], min_zero: usize) -> Option<usize> {
    let mut i = 0;
    while i < classes.len() {
        // A preamble: min_zero stable zeros in a row.
        let mut z = 0;
        while i + z < classes.len() && classes[i + z] == Class::Zero {
            z += 1;
        }
        if z < min_zero.max(1) {
            i += z.max(1);
            continue;
        }
        let mut stable = Class::Zero;
        let mut pending = None;
        let mut edges = 0;
        let mut j = i + z;
        while j < classes.len() {
            match classes[j] {
                Class::Noise => {
                    pending.get_or_insert(j);
                }
                c if c != stable => {
                    edges += 1;
                    if edges == 3 {
                        return Some(pending.unwrap_or(j));
                    }
                    stable = c;
                    pending = None;
                }
                _ => pending = None,
            }
            j += 1;
        }
        return None;
    }
    None
}

/// Gate search settings.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Params {
    /// Coarse steps per cycle (4 for 90-degree taps).
    pub coarse_per_cycle: u16,
    /// Fine steps per coarse step.
    pub fine_per_coarse: u16,
    /// Coarse positions swept per latency (must cover the preamble and
    /// three edges).
    pub coarse_steps: u16,
    /// Samples per position.
    pub samples: u16,
    /// Stable zeros needed before the edges (the preamble).
    pub min_zero: u8,
    /// 45-degree retries (offset 45, 22.5, ... degrees) before latency + 1.
    pub retries: u8,
    pub start_latency: u8,
    pub max_latency: u8,
    /// Cycles added to the gate latency for the read latency (PG150 1,
    /// PG353 2).
    pub post_cycles: u8,
}

impl Params {
    fn cycle(&self) -> u32 {
        u32::from(self.coarse_per_cycle) * u32::from(self.fine_per_coarse)
    }
    /// Fine steps of 45 degrees.
    pub fn fine_45(&self) -> u32 {
        self.cycle() / 8
    }
}

/// The gate found.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Gate {
    pub latency: u8,
    pub coarse: u16,
    pub fine: u16,
    /// Read latency: `latency + post_cycles`.
    pub read_latency: u8,
    /// Coarse index (from the sweep start) of the third edge.
    pub edge_coarse: u16,
    /// Width of the noise region, fine steps.
    pub noise: u16,
    /// Grid offsets tried at the final latency (0: the first grid).
    pub retries: u8,
    /// Latencies tried before the final one.
    pub latency_steps: u8,
}

/// Why the gate search failed (the PG150 classes).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GateError<E> {
    Probe(E),
    /// Settings out of range (zero steps, start above max).
    Config,
    /// Every sample at every latency was 0: no strobe returns.
    AllZero,
    /// Every sample was 1: the strobe is stuck high.
    AllOne,
    /// No third edge up to the maximum latency.
    MaxLatency,
    /// The third edge was found but the fine sweep found no 0-to-1
    /// transition within one cycle.
    NoFineEdge,
}

/// Samples the gated strobe: `ones` out of `samples` reads at a position.
pub trait Probe {
    type Error;
    fn sample(
        &mut self,
        latency: u8,
        coarse: u16,
        fine: u16,
        samples: u16,
    ) -> Result<u16, Self::Error>;
}

fn at(p: &Params, t: u32) -> (u16, u16) {
    let f = u32::from(p.fine_per_coarse);
    ((t / f) as u16, (t % f) as u16)
}

/// The DQS gate search (see the module docs). `classes` is scratch space of
/// at least `coarse_steps` entries.
pub fn search<P: Probe>(
    hw: &mut P,
    p: &Params,
    classes: &mut [Class],
) -> Result<Gate, GateError<P::Error>> {
    let n = usize::from(p.coarse_steps);
    if n == 0
        || p.samples == 0
        || p.fine_per_coarse == 0
        || p.coarse_per_cycle == 0
        || p.start_latency > p.max_latency
        || classes.len() < n
    {
        return Err(GateError::Config);
    }
    let classes = &mut classes[..n];
    let fpc = u32::from(p.fine_per_coarse);
    let (mut any_zero, mut any_one, mut any_noise) = (false, false, false);
    let sample = |hw: &mut P, lat: u8, t: u32| -> Result<Class, GateError<P::Error>> {
        let (c, f) = at(p, t);
        let ones = hw.sample(lat, c, f, p.samples).map_err(GateError::Probe)?;
        Ok(classify(ones, p.samples))
    };
    for (steps, lat) in (p.start_latency..=p.max_latency).enumerate() {
        for retry in 0..=p.retries {
            let offset = if retry == 0 {
                0
            } else {
                (p.fine_45() >> (retry - 1)).max(1)
            };
            for (k, c) in classes.iter_mut().enumerate() {
                *c = sample(hw, lat, k as u32 * fpc + offset)?;
                match c {
                    Class::Zero => any_zero = true,
                    Class::One => any_one = true,
                    Class::Noise => any_noise = true,
                }
            }
            let Some(e) = third_edge(classes, usize::from(p.min_zero)) else {
                continue;
            };
            // Stable 0 one coarse step before the edge (the matcher puts a
            // stable value there; confirm it is 0 with a fresh sample).
            let before = (e as u32 - 1) * fpc + offset;
            if sample(hw, lat, before)? != Class::Zero {
                continue;
            }
            // Fine sweep from the stable 0 through the edge, up to a cycle.
            let mut start = None;
            let mut end = None;
            for d in 1..=p.cycle() {
                match sample(hw, lat, before + d)? {
                    Class::Zero => start = None,
                    Class::Noise => {
                        start.get_or_insert(d);
                    }
                    Class::One => {
                        end = Some(d);
                        start.get_or_insert(d);
                        break;
                    }
                }
            }
            let (Some(s), Some(e1)) = (start, end) else {
                return Err(GateError::NoFineEdge);
            };
            // Centre of the noise region [s, e1).
            let centre = before + s + (e1 - s) / 2;
            let (coarse, fine) = at(p, centre);
            return Ok(Gate {
                latency: lat,
                coarse,
                fine,
                read_latency: lat.saturating_add(p.post_cycles),
                edge_coarse: e as u16,
                noise: (e1 - s) as u16,
                retries: retry,
                latency_steps: steps as u8,
            });
        }
    }
    Err(match (any_zero, any_one, any_noise) {
        (true, false, false) => GateError::AllZero,
        (false, true, false) => GateError::AllOne,
        _ => GateError::MaxLatency,
    })
}

/// The gate sanity check: `reads(n)` issues `n` gapped reads and returns
/// how many read FIFO pops (or bursts) the PHY counted; exactly `n` must
/// come back. Returns the count on a mismatch.
pub fn gapped_reads<E>(
    n: u32,
    mut reads: impl FnMut(u32) -> Result<u32, E>,
) -> Result<Result<(), u32>, E> {
    let got = reads(n)?;
    Ok(if got == n { Ok(()) } else { Err(got) })
}

#[cfg(test)]
mod tests {
    use super::*;
    use Class::*;

    #[test]
    fn third_edge_skips_the_preamble_and_spurious_edges() {
        // Noise before the preamble, preamble, rise, fall, rise.
        let c = [
            Noise, One, Zero, Zero, Zero, Noise, One, Noise, Zero, Noise, One,
        ];
        assert_eq!(third_edge(&c, 3), Some(9));
        // A too-short zero run before the real preamble is skipped.
        let c = [Zero, One, Zero, Zero, Zero, One, Zero, One];
        assert_eq!(third_edge(&c, 3), Some(7));
        // Noise glitch inside a stable run is not an edge.
        let c = [Zero, Zero, Zero, One, Noise, One, Zero, One];
        assert_eq!(third_edge(&c, 3), Some(7));
        assert_eq!(third_edge(&[Zero; 8], 3), None);
        assert_eq!(third_edge(&[Zero, Zero, One, Zero], 2), None);
    }
}
