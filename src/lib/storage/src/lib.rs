// SPDX-License-Identifier: GPL-2.0-only
//! Storage formats a first stage reads before any OS: a bounded primary-GPT
//! lookup ([`gpt`]), the SD CSD capacity decode ([`sd`]) and CRC-32
//! ([`crc`], from the A733 board code).
//!
//! From ore-edk-boot-opi c91bbc4e40 `oreboot/src/mainboard/spacemit/k1x/bt0/src/`
//! `gpt.rs` and `sd_capacity.rs`. The only change: the GPT fallback partition
//! type is a parameter ([`gpt::find_typed`]); [`gpt::find`] keeps the K1 one.
// The code is kept as the board had it (equivalence-tested against the
// verbatim copy in tests/legacy/); these newer-clippy style lints are allowed.
#![allow(unknown_lints)] // the lint below is newer than the runners' clippy (Rust 1.98)
#![allow(clippy::manual_is_multiple_of, clippy::manual_range_contains)]
#![no_std]
#[cfg(test)]
extern crate std;

pub mod crc;
pub mod gpt;
pub mod sd;
