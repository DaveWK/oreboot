// SPDX-License-Identifier: GPL-2.0-only
//! Checked external-data FIT planning and board-DT memory fixup for the K1
//! boot chain: the shared `fit` crate (ore-edk2-boot) with the K1 FIT
//! contract (`LAYOUT`) and DT memory slots (`MEMORY`). The crate's
//! equivalence tests run it with these values against this file's previous
//! code.

pub use fit::{read_extent, source_extent, Error, Image, Plan, Region};
type Result<T> = core::result::Result<T, Error>;

pub const MAX_FIT: usize = 4 * 1024 * 1024;
pub const MAX_METADATA: usize = 256 * 1024;
pub const NOR_BYTES: u64 = 16 * 1024 * 1024;
// Both in-tree FDFs: immutable compact FV, mutable variable/FTW tail and final
// erased page. The producer checks these constants against its actual FDF/FD.
pub const EDK2_CODE_SIZE: usize = 0x1ff000;
pub const EDK2_FD_SIZE: usize = 0x2c0000;

/// The K1 FIT: OpenSBI at 0 (`firmware`), EDK2 at 2 MiB (`standalone`) with
/// its exact code size at external data offset 0x1000 and the variable store
/// tail reserved behind it (`nvram-reserved`), the DT 64 KiB aligned after
/// the FD slot, everything below the 256 MiB staging area.
pub const LAYOUT: fit::Layout = fit::Layout {
    arch: b"riscv",
    max_fit: MAX_FIT,
    max_metadata: MAX_METADATA,
    data_base: Some(0x1000),
    firmware_load: 0,
    next_load: 0x20_0000,
    next_size: Some(EDK2_CODE_SIZE),
    reserved: Some(fit::Reserved {
        node: b"nvram-reserved",
        offset: EDK2_CODE_SIZE,
        size: EDK2_FD_SIZE - EDK2_CODE_SIZE,
    }),
    next_slot: EDK2_FD_SIZE,
    dt_align: 0x10000,
    firmware_limit: 0x20_0000,
    payload_limit: 0x1000_0000,
};

/// The K1 board DTs' memory nodes: up to 2 GiB at 0, the rest at 4 GiB.
pub const MEMORY: fit::MemoryLayout = fit::MemoryLayout {
    low: b"memory@0",
    high: b"memory@100000000",
    low_base: 0,
    low_max: 0x8000_0000,
    high_base: 0x1_0000_0000,
    sizes: &[0x4000_0000, 0x8000_0000, 0x1_0000_0000, 0x2_0000_0000],
};

/// Plan the default configuration of `metadata` against `staging`.
pub fn plan(metadata: &[u8], staging: Region) -> Result<Plan> {
    fit::plan(&LAYOUT, metadata, staging)
}

/// Rewrite both memory nodes of the board DT for `size` bytes of DRAM.
pub fn fix_memory(bytes: &mut [u8], size: u64) -> Result<()> {
    fit::fix_memory(&MEMORY, bytes, size)
}
