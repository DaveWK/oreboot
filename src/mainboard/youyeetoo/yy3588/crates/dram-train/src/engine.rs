// SPDX-License-Identifier: GPL-2.0-only
//! "Issue a pattern, compare, count": the one primitive every training
//! stage and margin check needs, as a trait a board implements on its own
//! engine (a PHY data-training unit, the A733 ACSM sequencer with the DTSM
//! compare, the K1 training engines, or plain CPU accesses), plus the loop
//! around it: repeat a write/read-compare pass, count failing passes and
//! failing lanes, stop on the Nth fail, keep the read data of the first
//! failing pass for the log.
//!
//! Model: the ZynqMP DDR PHY data-command unit (UG1087 `DCUAR`, `DCUDR`,
//! `DCURR` with its expected-data and read-data caches, compare enable,
//! stop-on-Nth-fail and loop count; UG1085 p428-478), cited, not copied.
//! [`MemEngine`] is the plain-access engine (and the host-test model).

/// A data beat: up to 64 lanes (bit n = lane n).
pub type Beat = u64;

/// The board's engine. One pass writes the loaded beats at (rank, addr)
/// and reads them back through the engine's compare.
pub trait Engine {
    type Error;
    /// Beats the expected-data cache holds.
    fn capacity(&self) -> usize;
    /// Load the expected data (`beats.len() <= capacity()`).
    fn load(&mut self, beats: &[Beat]) -> Result<(), Self::Error>;
    /// Write the loaded beats at (rank, addr).
    fn write(&mut self, rank: u8, addr: u64) -> Result<(), Self::Error>;
    /// Read them back and compare: the lanes that differed on any beat.
    fn read_compare(&mut self, rank: u8, addr: u64) -> Result<Beat, Self::Error>;
    /// The read data of the last pass, if the engine keeps it (a read
    /// cache); returns the beats copied.
    fn read_cache(&mut self, _out: &mut [Beat]) -> Result<usize, Self::Error> {
        Ok(0)
    }
}

/// One job.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Job {
    pub rank: u8,
    pub addr: u64,
    /// Passes to run (the DCU loop count); at least 1.
    pub passes: u32,
    /// Stop after this many failing passes (the DCU stop-on-Nth-fail).
    pub stop_after: Option<u32>,
    /// Lanes that count (others are ignored, e.g. DM or unused lanes).
    pub lanes: Beat,
    /// Write once and only read on later passes (a read-only loop), or
    /// write on every pass.
    pub write_each_pass: bool,
}

/// What a job saw.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Counts {
    pub passes: u32,
    /// Passes with at least one counted lane failing.
    pub fails: u32,
    /// OR of the failing lanes over all passes.
    pub lanes: Beat,
    /// Failing passes per lane (lanes 0..64).
    pub lane_fails: [u16; 64],
    pub first_fail: Option<u32>,
    /// Stopped by `stop_after`.
    pub stopped: bool,
    /// Beats of the first failing pass copied from the read cache.
    pub captured: usize,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EngineError<E> {
    Engine(E),
    /// More beats than the expected-data cache holds, or no beats.
    Capacity,
}

/// Run `job` with `expected` on `e`; with `capture`, the read data of the
/// first failing pass is copied into it.
pub fn run<E: Engine>(
    e: &mut E,
    job: &Job,
    expected: &[Beat],
    capture: &mut [Beat],
) -> Result<Counts, EngineError<E::Error>> {
    if expected.is_empty() || expected.len() > e.capacity() {
        return Err(EngineError::Capacity);
    }
    e.load(expected).map_err(EngineError::Engine)?;
    let mut c = Counts {
        passes: 0,
        fails: 0,
        lanes: 0,
        lane_fails: [0; 64],
        first_fail: None,
        stopped: false,
        captured: 0,
    };
    for pass in 0..job.passes.max(1) {
        if pass == 0 || job.write_each_pass {
            e.write(job.rank, job.addr).map_err(EngineError::Engine)?;
        }
        let bad = e
            .read_compare(job.rank, job.addr)
            .map_err(EngineError::Engine)?
            & job.lanes;
        c.passes += 1;
        if bad != 0 {
            c.fails += 1;
            c.lanes |= bad;
            for (l, n) in c.lane_fails.iter_mut().enumerate() {
                if bad >> l & 1 != 0 {
                    *n = n.saturating_add(1);
                }
            }
            if c.first_fail.is_none() {
                c.first_fail = Some(pass);
                c.captured = e.read_cache(capture).map_err(EngineError::Engine)?;
            }
            if job.stop_after.is_some_and(|n| c.fails >= n) {
                c.stopped = true;
                break;
            }
        }
    }
    Ok(c)
}

/// The engine on plain word accesses: `access(addr, Some(v))` writes,
/// `access(addr, None)` reads; one beat per 8-byte word from `addr`.
pub struct MemEngine<F, const N: usize> {
    pub access: F,
    expected: [Beat; N],
    len: usize,
    last: [Beat; N],
}

impl<F: FnMut(u64, Option<Beat>) -> Beat, const N: usize> MemEngine<F, N> {
    pub fn new(access: F) -> Self {
        Self {
            access,
            expected: [0; N],
            len: 0,
            last: [0; N],
        }
    }
}

impl<F: FnMut(u64, Option<Beat>) -> Beat, const N: usize> Engine for MemEngine<F, N> {
    type Error = core::convert::Infallible;
    fn capacity(&self) -> usize {
        N
    }
    fn load(&mut self, beats: &[Beat]) -> Result<(), Self::Error> {
        self.expected[..beats.len()].copy_from_slice(beats);
        self.len = beats.len();
        Ok(())
    }
    fn write(&mut self, _rank: u8, addr: u64) -> Result<(), Self::Error> {
        for i in 0..self.len {
            (self.access)(addr + 8 * i as u64, Some(self.expected[i]));
        }
        Ok(())
    }
    fn read_compare(&mut self, _rank: u8, addr: u64) -> Result<Beat, Self::Error> {
        let mut bad = 0;
        for i in 0..self.len {
            let v = (self.access)(addr + 8 * i as u64, None);
            self.last[i] = v;
            bad |= v ^ self.expected[i];
        }
        Ok(bad)
    }
    fn read_cache(&mut self, out: &mut [Beat]) -> Result<usize, Self::Error> {
        let n = out.len().min(self.len);
        out[..n].copy_from_slice(&self.last[..n]);
        Ok(n)
    }
}
