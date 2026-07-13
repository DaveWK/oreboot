// SPDX-License-Identifier: GPL-2.0-only
// Verbatim copies of the board code lp4::density replaces, the oracle of the
// equivalence tests. Source: ore-edk-boot-opi 78479e21ba,
// oreboot/src/mainboard/spacemit/k1x/bt0/src/dram.rs (channel_size,
// ddr_get_density). Only the lines marked "stub:" are not from the source.
#![allow(dead_code, clippy::all)]

// stub: the bt0 error type and its MR8 reads (from a table, per channel/rank).
#[derive(Debug, PartialEq, Eq)]
pub enum Error {
    Geometry,
}
type Result<T> = core::result::Result<T, Error>;
thread_local! { pub static MR8: core::cell::RefCell<[[u32; 2]; 2]> = core::cell::RefCell::new([[0; 2]; 2]); }
fn mode_register_read(mr: u32, ch: u32, cs: u32) -> Result<u32> {
    assert_eq!(mr, 8);
    Ok(MR8.with(|t| t.borrow()[cs as usize][ch as usize]))
}

// MR8 density in MiB per die; two independent 16-bit channels.
fn channel_size(mr8: u32) -> Result<u32> {
    let size = match (mr8 >> 2) & 15 {
        0 => 256,
        1 => 384,
        2 => 512,
        3 => 768,
        4 => 1024,
        5 => 1536,
        6 => 2048,
        _ => return Err(Error::Geometry),
    };
    match mr8 >> 6 {
        0 => Ok(size),
        1 => Ok(size * 2),
        _ => Err(Error::Geometry),
    }
}
fn ddr_get_density(cs_num: u32) -> Result<u32> {
    let first = mode_register_read(8, 0, 0)?;
    let mut size = 0;
    for cs in 0..cs_num {
        for ch in 0..2 {
            let mr8 = mode_register_read(8, ch, cs)?;
            // The controller uses one row/column geometry for both channels/ranks.
            if mr8 != first {
                return Err(Error::Geometry);
            }
            size += channel_size(mr8)?;
        }
    }
    if !matches!(size, 1024 | 2048 | 4096 | 8192) {
        return Err(Error::Geometry);
    }
    Ok(size)
}

// stub: test access to the private board functions above.
pub fn channel_size_of(mr8: u32) -> Result<u32> {
    channel_size(mr8)
}
pub fn ddr_get_density_of(cs_num: u32) -> Result<u32> {
    ddr_get_density(cs_num)
}
