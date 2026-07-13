// SPDX-License-Identifier: GPL-2.0-only
//! JEDEC LPDDR helpers shared by every ore-edk2-boot board: mode-register and
//! Vref encodings, device timing in nanoseconds, and density arithmetic.
//!
//! Pure functions and constants, no hardware access and no SoC assumption:
//! the controller that issues a mode-register write, its command format and
//! the number of channels, ranks and lanes all belong to the board.
//!
//! - [`lp4`]: LPDDR4 and LPDDR4X: MR12/MR14 VREF
//!   axis, [`lp4::density`] (MR8), [`lp4::mr`] (MR1/2/3/11/12/14),
//!   [`lp4::timing`] (RL/WL/nWR frequency ranges, part timings).
//! - [`lp5`]: LPDDR5: Vref codes, WCK:CK ratio, [`lp5::cmd`]
//!   (MRW/MRR CA encoding), [`lp5::mr`] (mode registers), [`lp5::timing`]
//!   (latency tables per data rate, A733 device timings).
//! - [`timing`]: nanoseconds to clock cycles.
//! - [`geometry`]: density from address bits and bus width; ranks,
//!   channels and density as one [`geometry::Topology`].
#![no_std]

pub mod geometry;
pub mod lp4;
pub mod lp5;
/// JEDEC values not in this crate yet (pending spec), and why.
pub mod pending;
pub mod timing;
