// SPDX-License-Identifier: GPL-2.0-only
//! Controller-neutral DRAM controller initialisation, the way the A733
//! driver does it, for any controller: the JEDEC timings of the part in
//! controller cycles ([`values`]), a register-field packing table per
//! controller ([`pack`]; the DesignWare DDRCTL table in [`dwc`] is the first
//! one), a bounded and logged init-step runner ([`steps`]) and the rank and
//! address-map bit layout of any rank count and density ([`hif`]), and
//! training stage lists by data rate with the warm-boot rule ([`stages`]).
//!
//! A board keeps its controller's register map and its init sequence (as
//! [`steps::Step`] lists and its own packing table) and computes nothing
//! by hand: every timing comes from `jedec-lpddr`, every rank or density
//! choice from `jedec_lpddr::geometry::Topology`.
#![no_std]

pub mod dwc;
pub mod hif;
pub mod pack;
pub mod stages;
pub mod steps;
pub mod values;
