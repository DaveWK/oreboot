// SPDX-License-Identifier: GPL-2.0-only
//! Vref eye search and centring for our own K1 LPDDR4/4X training.
//!
//! The algorithms are the shared ones of ore-edk2-boot (pinned in
//! patches/ore-edk2-boot/pin.env): `dram_train::eye` (centring, worst window)
//! and `dram_train::vref_sweep` (one Vref axis of the eye, used by
//! `train_own.rs`), on the MR14 VREF(DQ) axis of `jedec_lpddr::lp4` (range 0
//! settings 0..=50 then range 1 settings 21..=50, monotonic in VDDQ
//! percentage). They were first written here, from JEDEC LPDDR4/4X and
//! generic eye-centring practice, and take no logic or parameter from
//! SpacemiT's training image or its translation (doc/K1-OWN-TRAINING-PLAN.md).
//! The names below keep the K1 code's vocabulary.

pub use dram_train::eye::{center, worst, Eye, EyeError};
/// VREF(DQ) of an LPDDR4X MR14 OP byte in 0.1 % of VDDQ. Log only.
pub use jedec_lpddr::lp4::lp4x_vref_permille as lp4x_mr14_permille;
/// Axis point of an MR14 OP byte; `None` for reserved settings or OP[7] set.
pub use jedec_lpddr::lp4::vref_axis_index as tx_axis_index;
/// MR14 OP byte of TX axis point `i` (`i < TX_AXIS_LEN`).
pub use jedec_lpddr::lp4::vref_axis_op as tx_axis_mr14;
/// TX Vref axis over MR14 VREF(DQ).
pub use jedec_lpddr::lp4::VREF_AXIS_LEN as TX_AXIS_LEN;

/// RX Vref codes: one 4-bit DQ field per byte lane in the PHY (GPL-2.0+
/// `lpddr4_silicon_init.c` `top_DDR_rx_ds_odt_vref`: "high 4 bit for DQS,
/// low 4 bit for DQ"). We scan all of them.
pub const RX_CODES: usize = 16;
