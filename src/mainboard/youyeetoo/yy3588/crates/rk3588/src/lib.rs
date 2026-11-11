// SPDX-License-Identifier: GPL-2.0-only
//! Rockchip RK3588 first-stage support for the youyeetoo YY3588 bt0.
//!
//! - [`osreg`]: the PMU1 GRF OS_REG2..5 DRAM handoff (decode/encode).
//! - [`ddr`]: own LPDDR4/4X init and training (entry [`ddr::init`]);
//!   host-tested, not yet run on hardware (bt0 feature `own-ddr`).
//! - [`pll`], [`msch`], [`rk806`]: DDR PLL, DDR schedulers, PMIC rails.
//! - [`fit`], [`handoff`]: FIT walking and the TF-A BL31 handoff.
//! - [`dump`]: the dram-info report (text and binary record).
//!
//! Sources are cited per item; see crate::regs for the notation.

#![no_std]

pub mod ddr;
pub mod dump;
pub mod fit;
pub mod handoff;
pub mod msch;
pub mod osreg;
pub mod pll;
pub mod regs;
pub mod rk806;
