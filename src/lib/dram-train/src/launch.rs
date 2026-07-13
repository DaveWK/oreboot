// SPDX-License-Identifier: GPL-2.0-only
//! One bounded, witnessed launch of a hardware sequencer.
//!
//! A training engine or command sequencer is started, then polled until its
//! done flag sets or a time budget runs out. Some engines read "done" while
//! idle, so a done flag at the very first poll proves nothing: the launch
//! counts only if a witness shows it ran (e.g. a last-address register that
//! changed to this launch's stop row). The run control is always cleared
//! afterwards, also on failure.
//!
//! First written for the A733 ACSM launches (ore-edk-boot-radxa-a7a `wl.rs`
//! `WlExec::launch`).

use crate::io::RegIo;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LaunchError {
    /// Done did not set within the time budget.
    Timeout,
    /// Done was set at the first poll and the witness did not show a run.
    NoBusyObserved,
}

/// Result and cost of one launch.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Launched {
    pub result: Result<(), LaunchError>,
    /// Microseconds from start to the end of polling.
    pub us: u32,
    /// Polls that saw done clear.
    pub polls: u32,
}

/// Start with `start`, poll `done` until it holds or more than `timeout_us`
/// passed; if done held at the first poll, `witness` must confirm the run.
/// `stop` runs last in every case. Register access order: now_us, start,
/// (done, [witness], now_us)*, now_us, stop.
pub fn bounded_launch<R: RegIo + ?Sized>(
    io: &mut R,
    timeout_us: u64,
    start: impl FnOnce(&mut R),
    mut done: impl FnMut(&mut R) -> bool,
    witness: impl FnOnce(&mut R) -> bool,
    stop: impl FnOnce(&mut R),
) -> Launched {
    let mut witness = Some(witness);
    let t0 = io.now_us();
    start(io);
    let mut polls: u32 = 0;
    let result = loop {
        if done(io) {
            if polls == 0 {
                let ran = witness.take().is_some_and(|w| w(io));
                if !ran {
                    break Err(LaunchError::NoBusyObserved);
                }
            }
            break Ok(());
        }
        polls += 1;
        if io.now_us().wrapping_sub(t0) > timeout_us {
            break Err(LaunchError::Timeout);
        }
    };
    let us = io.now_us().wrapping_sub(t0) as u32;
    stop(io);
    Launched { result, us, polls }
}
