// SPDX-License-Identifier: GPL-2.0-only
//! The A7A contract (from the board's `mem` and `layout` constants), shared
//! by the tests.
#![allow(dead_code)]
use bootlayout::{Contract, KindRule, Place};

pub const KIND_MAIN: u32 = 1;
pub const KIND_PAYLOAD: u32 = 2;
pub const KIND_DTB: u32 = 3;
pub const KIND_VENDOR_SPL: u32 = 4;
pub const DRAM_BASE: u64 = 0x4000_0000;
pub const MAIN_BASE: u64 = DRAM_BASE;
pub const MAIN_MAX_SIZE: u64 = 0x0008_0000;
pub const PAYLOAD_RAM_BASE: u64 = DRAM_BASE + 0x0020_0000;
pub const PAYLOAD_BASE: u64 = 0x4100_0000;
pub const HANDOFF_ADDR: u64 = DRAM_BASE + 0x001f_f000;
pub const TABLE_OFFSET: u32 = 0x001f_f000;
pub const IMAGE_START: u64 = 0x0020_0000;
pub const IMAGE_END: u64 = 0x0100_0000;

pub const KINDS: [KindRule; 4] = [
    KindRule {
        kind: KIND_MAIN,
        place: Place::Exact {
            addr: MAIN_BASE,
            max_size: Some(MAIN_MAX_SIZE),
        },
    },
    KindRule {
        kind: KIND_PAYLOAD,
        place: Place::Exact {
            addr: PAYLOAD_BASE,
            max_size: None,
        },
    },
    KindRule {
        kind: KIND_DTB,
        place: Place::AtLeast(PAYLOAD_RAM_BASE),
    },
    KindRule {
        kind: KIND_VENDOR_SPL,
        place: Place::Recovery {
            addr: 0x47000,
            max_size: 0x4a000,
        },
    },
];

/// The A7A contract, with the loader's address width given.
pub fn a7a(max_address: u64) -> Contract<'static> {
    Contract {
        magic: *b"A7ABOOT1",
        version: 1,
        image_start: IMAGE_START,
        image_end: IMAGE_END,
        block_size: 512,
        dram_base: DRAM_BASE,
        max_address,
        kinds: &KINDS,
        required: &[KIND_MAIN, KIND_PAYLOAD],
    }
}
