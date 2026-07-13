// SPDX-License-Identifier: GPL-2.0-only
//! Architecture helpers that make a memory test fast without touching the
//! test itself (mapping DRAM so accesses are Normal memory, not Device).

pub mod aarch64;
