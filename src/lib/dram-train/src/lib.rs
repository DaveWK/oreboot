// SPDX-License-Identifier: GPL-2.0-only
//! Generic DRAM training for every ore-edk2-boot board.
//!
//! Every board trains the same way: sweep a delay or Vref axis, find the pass
//! window, centre in it, combine windows across ranks, phases and lanes, and
//! bound every launch so a stuck engine is an error, not a hang. This crate
//! holds those algorithms; board crates keep their register maps
//! (A733 Synopsys PHY/ACSM/PIE/DBYTE, K1 training engines, ...).
//!
//! - [`io`]: the one hardware-access trait ([`io::RegIo`]) and bounded polling.
//! - [`window`]: pass windows in circular or linear bitmaps.
//! - [`eye`]: eye centring along an axis of measured widths.
//! - [`edge`]: transitions of a measured pass/fail curve.
//! - [`launch`]: one bounded, witnessed launch of a hardware sequencer.
//! - [`dwc_mr`]: DesignWare DDRCTL mode-register access.
//! - [`vref_sweep`]: one Vref axis of an eye: sweep, centre, set, re-run, margin.
//! - [`vref_balance`]: Vref by balancing the 0- and 1-windows.
//! - [`vref_select`]: successive halving and profile smoothing for noisy
//!   Vref choices.
//! - [`search`]: bounded measurement search and transactional application.
//! - [`result`]: the training-result schema (stage log, error record, edge
//!   values, drift, margin lines in the acceptance-record shape).
//! - [`ruler`]: the delay-line ruler (taps, ps, UI, degrees) and total
//!   delays split into stages and fine taps.
//! - [`osc`]: DQS/WCK2DQ oscillator tracking (delay from the count, drift,
//!   per-rank write delay, rank schedule, dummy reads).
//! - [`pstate_seed`]: cross-pstate seeding by delay physics (fixed ps plus
//!   whole UIs) and a bracketed search with fall-back.
//! - [`ranks`]: rank merge by midrange, plausibility checks, bounded
//!   retrain with a rotating start phase.
//! - [`engine`]: the issue-pattern, compare, count primitive (an engine
//!   trait, the loop around it, a plain-access engine).
//! - [`loopback`]: DRAM-free loopback instruments (knob check, delay-line
//!   period) and PRBS generators and checkers.
//! - [`margin`]: a (delay x Vref) margin scan at bit, nibble or byte level.
//! - [`sanity`]: knob-response checks before a decision (dead knob, aliased
//!   axis, saturation, mode mismatch).
//! - [`wrlat`]: write-latency (cycle slip) check with a framed pattern.
//! - [`soft_edge`]: edges from error fractions (probit fit: sub-tap centre,
//!   jitter sigma, edges at a target error rate).
//! - [`stage_check`]: the sanity check after every stage (fixed word per
//!   stage, negative check, failing-nibble record).
//! - [`deskew`]: per-bit deskew to the latest bit, write centring by
//!   margins, the centre from partial edges.
//! - [`dmdbi`]: the DM check, the DBI rule and patterns, the combined
//!   DQ+DBI window.
//! - [`gate`]: DQS gate training as a pattern match (third DQS edge).
//! - [`cache`]: the plain training cache (identity key, CRC, restore with
//!   the warm stages and a check, fall-back to full training).
//! - [`cache_temp`]: the temperature-aware cache (band only invalidates,
//!   recovery copy, P3-A5).
//! - [`complex`]: victim/aggressor patterns and the short-then-long search
//!   of the read and write complex stages.
//!
//! Nothing here assumes a SoC: lane, rank, point and step counts, axis
//! lengths and pass polarity are parameters. LPDDR encodings are in
//! `jedec-lpddr`.
#![no_std]

pub mod cache;
pub mod cache_temp;
pub mod complex;
pub mod deskew;
pub mod dmdbi;
pub mod dwc_mr;
pub mod edge;
pub mod engine;
pub mod eye;
pub mod gate;
/// The register-access trait and bounded polling (`crates/regio`).
pub use regio as io;
pub mod launch;
pub mod live;
pub mod loopback;
pub mod margin;
pub mod osc;
pub mod pstate_seed;
pub mod ranks;
pub mod result;
pub mod ruler;
pub mod sanity;
pub mod search;
pub mod soft_edge;
pub mod stage_check;
pub mod vref_balance;
pub mod vref_select;
pub mod vref_sweep;
pub mod window;
pub mod wrlat;
