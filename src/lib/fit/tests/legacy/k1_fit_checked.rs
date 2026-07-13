// SPDX-License-Identifier: GPL-2.0-only
// Verbatim copy of ore-edk-boot-opi 78479e21ba
// oreboot/src/mainboard/spacemit/k1x/bt0/src/fit_checked.rs, the oracle for the
// equivalence tests (its crate:: imports resolve to the test crate root).
#![allow(dead_code, clippy::all)]
//! Checked external-data FIT planning and board-DT memory fixup.
use crate::fdt_checked::{self as fdt, Fdt, Kind};
use crate::sha256;
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    Fdt(fdt::Error),
    Bounds,
    Image,
    Hash,
    Destination,
    Memory,
}
impl From<fdt::Error> for Error {
    fn from(e: fdt::Error) -> Self {
        Self::Fdt(e)
    }
}
type Result<T> = core::result::Result<T, Error>;
pub const MAX_FIT: usize = 4 * 1024 * 1024;
pub const MAX_METADATA: usize = 256 * 1024;
pub const NOR_BYTES: u64 = 16 * 1024 * 1024;
// Both in-tree FDFs: immutable compact FV, mutable variable/FTW tail and final
// erased page. The producer checks these constants against its actual FDF/FD.
pub const EDK2_CODE_SIZE: usize = 0x1ff000;
pub const EDK2_FD_SIZE: usize = 0x2c0000;
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Region {
    pub start: usize,
    pub size: usize,
}
impl Region {
    pub fn end(self) -> Result<usize> {
        self.start.checked_add(self.size).ok_or(Error::Bounds)
    }
    pub fn overlaps(self, other: Self) -> Result<bool> {
        Ok(self.start < other.end()? && other.start < self.end()?)
    }
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Image {
    pub data: Region,
    pub destination: Region,
    pub hash: [u8; 32],
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Plan {
    pub images: [Image; 3],
    pub end: usize,
    pub metadata: usize,
}
fn string_prop<'a>(f: &Fdt<'a>, n: usize, p: &[u8]) -> Result<&'a [u8]> {
    Ok(fdt::string(f.prop(n, p)?)?)
}
fn number_prop(f: &Fdt, n: usize, p: &[u8]) -> Result<usize> {
    Ok(fdt::number(f.prop(n, p)?)?)
}
fn align(n: usize, a: usize) -> Result<usize> {
    Ok(n.checked_add(a - 1).ok_or(Error::Bounds)? & !(a - 1))
}
fn image(
    f: &Fdt,
    name: &[u8],
    load: Option<usize>,
    data_base: usize,
    kind: &[u8],
) -> Result<Image> {
    let images = f.child(f.h.structure, b"images")?;
    let n = f.child(images, name)?;
    if string_prop(f, n, b"compression")? != b"none" || string_prop(f, n, b"type")? != kind {
        return Err(Error::Image);
    }
    if load.is_some() && string_prop(f, n, b"arch")? != b"riscv" {
        return Err(Error::Image);
    }
    // Only external offsets emitted by mkimage -E are supported. Reject a second
    // competing data representation instead of silently selecting one.
    for prop in [b"data" as &[u8], b"data-position"] {
        match f.prop(n, prop) {
            Err(fdt::Error::Missing) => {}
            Err(e) => return Err(e.into()),
            Ok(_) => return Err(Error::Image),
        }
    }
    let size = number_prop(f, n, b"data-size")?;
    let offset = number_prop(f, n, b"data-offset")?;
    if size == 0 {
        return Err(Error::Image);
    }
    let start = data_base.checked_add(offset).ok_or(Error::Bounds)?;
    let destination = match load {
        Some(expected) => {
            let dest = number_prop(f, n, b"load")?;
            if dest != expected || number_prop(f, n, b"entry")? != expected {
                return Err(Error::Destination);
            }
            dest
        }
        None => 0,
    };
    let hash_node = f.child(n, b"hash-1")?;
    if string_prop(f, hash_node, b"algo")? != b"sha256" {
        return Err(Error::Hash);
    }
    let hash = f
        .prop(hash_node, b"value")?
        .try_into()
        .map_err(|_| Error::Hash)?;
    Ok(Image {
        data: Region { start, size },
        destination: Region {
            start: destination,
            size,
        },
        hash,
    })
}
/// No source read or destination write is performed while constructing a plan.
pub fn plan(metadata: &[u8], staging: Region) -> Result<Plan> {
    let f = Fdt::new(metadata)?;
    if f.h.total > MAX_METADATA {
        return Err(Error::Bounds);
    }
    let config = f.child(f.h.structure, b"configurations")?;
    let default = string_prop(&f, config, b"default")?;
    let c = f.child(config, default)?;
    let data_base = align(f.h.total, 4)?;
    // FDF variable addresses are absolute NOR offsets. A second metadata page
    // would move the payload/variable slot despite otherwise valid extents.
    if data_base != 0x1000 {
        return Err(Error::Image);
    }
    let fw = image(
        &f,
        string_prop(&f, c, b"firmware")?,
        Some(0),
        data_base,
        b"firmware",
    )?;
    let next = image(
        &f,
        string_prop(&f, c, b"loadables")?,
        Some(0x20_0000),
        data_base,
        b"standalone",
    )?;
    let mut dt = image(&f, string_prop(&f, c, b"fdt")?, None, data_base, b"flat_dt")?;
    // Code-only selection must preserve the old FD reservation in both source
    // NOR layout and destination RAM. The variable tail is never executable or
    // copied; pinned SEC consumes only the compact FV, while variable services
    // use their NOR mapping or initialize a separate emulated RAM store.
    let reserved = f.child(f.child(f.h.structure, b"images")?, b"nvram-reserved")?;
    if next.data.start != data_base
        || next.data.size != EDK2_CODE_SIZE
        || number_prop(&f, reserved, b"data-offset")? != EDK2_CODE_SIZE
        || number_prop(&f, reserved, b"data-size")? != EDK2_FD_SIZE - EDK2_CODE_SIZE
        || string_prop(&f, reserved, b"compression")? != b"none"
    {
        return Err(Error::Image);
    }
    for field in [b"load" as &[u8], b"entry", b"data", b"data-position"] {
        match f.prop(reserved, field) {
            Err(fdt::Error::Missing) => {}
            Err(e) => return Err(e.into()),
            Ok(_) => return Err(Error::Image),
        }
    }
    let slot_end = data_base.checked_add(EDK2_FD_SIZE).ok_or(Error::Bounds)?;
    if fw.data.start < slot_end || dt.data.start < slot_end {
        return Err(Error::Image);
    }
    dt.destination.start = align(
        next.destination
            .start
            .checked_add(EDK2_FD_SIZE)
            .ok_or(Error::Bounds)?,
        0x10000,
    )?;
    let images = [fw, next, dt];
    let mut end = 0;
    // Fixed board layout: reserve the entire OpenSBI window; EDK2 cannot reach
    // staging, SRAM or any K1 MMIO aperture. DT stays below the 256 MiB staging.
    if fw.destination.end()? > 0x20_0000
        || next.destination.end()? > 0x1000_0000
        || dt.destination.end()? > 0x1000_0000
    {
        return Err(Error::Destination);
    }
    for i in 0..3 {
        end = end.max(images[i].data.end()?);
        if images[i].destination.overlaps(staging)? {
            return Err(Error::Destination);
        }
        for j in 0..i {
            if images[i].data.overlaps(images[j].data)?
                || images[i].destination.overlaps(images[j].destination)?
            {
                return Err(Error::Destination);
            }
        }
    }
    if end > MAX_FIT || align(end, 512)? > staging.size {
        return Err(Error::Bounds);
    }
    Ok(Plan {
        images,
        end,
        metadata: f.h.total,
    })
}
impl Plan {
    /// Validate every payload before copying any one of them. A later corrupt
    /// DT therefore cannot leave an executable payload partially installed.
    pub fn verify(&self, bytes: &[u8]) -> Result<()> {
        for image in self.images {
            let data = bytes
                .get(image.data.start..image.data.end()?)
                .ok_or(Error::Bounds)?;
            if sha256::digest(data) != image.hash {
                return Err(Error::Hash);
            }
        }
        let dt = self.images[2].data;
        Fdt::new(&bytes[dt.start..dt.end()?])?;
        Ok(())
    }
}
/// Checked bytes for a bounded sector read. GPT extents and actual medium
/// capacities are sectors; FIT metadata/data offsets are bytes.
pub fn read_extent(first: u32, sectors: u64, relative: u64, count: usize) -> Result<(u32, usize)> {
    if count == 0 {
        return Err(Error::Bounds);
    }
    let stop = relative.checked_add(count as u64).ok_or(Error::Bounds)?;
    let lba = (first as u64).checked_add(relative).ok_or(Error::Bounds)?;
    if stop > sectors || lba.checked_add(count as u64).ok_or(Error::Bounds)? > u32::MAX as u64 + 1 {
        return Err(Error::Bounds);
    }
    Ok((u32::try_from(lba).map_err(|_| Error::Bounds)?, count))
}
/// Rewrite both explicit 64-bit memory slots, removing the upper slot for R2S.
/// Validate the complete DT and required nodes before changing any byte.
pub fn fix_memory(bytes: &mut [u8], size: u64) -> Result<()> {
    if !matches!(
        size,
        0x4000_0000 | 0x8000_0000 | 0x1_0000_0000 | 0x2_0000_0000
    ) {
        return Err(Error::Memory);
    }
    let f = Fdt::new(bytes)?;
    let root = f.h.structure;
    if number_prop(&f, root, b"#address-cells")? != 2 || number_prop(&f, root, b"#size-cells")? != 2
    {
        return Err(Error::Memory);
    }
    let low = f.child(root, b"memory@0")?;
    let high = f.child(root, b"memory@100000000")?;
    for n in [low, high] {
        if f.prop(n, b"reg")?.len() != 16 || string_prop(&f, n, b"device_type")? != b"memory" {
            return Err(Error::Memory);
        }
    }
    let mut it = f.tokens();
    while let Some(t) = it.next()? {
        if let Kind::Begin(n) = t.kind {
            if t.depth == 2 && n.starts_with(b"memory") && t.at != low && t.at != high {
                return Err(Error::Memory);
            }
        }
    }
    let l = f.prop_offset(low, b"reg")?;
    let h = f.prop_offset(high, b"reg")?;
    let high_end = f.node_end(high)?;
    let low_size = size.min(0x8000_0000);
    let high_size = size - low_size;
    bytes[l..l + 8].copy_from_slice(&0u64.to_be_bytes());
    bytes[l + 8..l + 16].copy_from_slice(&low_size.to_be_bytes());
    if high_size == 0 {
        for p in (high..high_end).step_by(4) {
            bytes[p..p + 4].copy_from_slice(&4u32.to_be_bytes());
        }
    } else {
        bytes[h..h + 8].copy_from_slice(&0x1_0000_0000u64.to_be_bytes());
        bytes[h + 8..h + 16].copy_from_slice(&high_size.to_be_bytes());
    }
    Ok(())
}

/// Select a validated partition, evaluating the raw fallback only when absent.
/// Recheck its full extent against actual selected-medium capacity.
pub fn source_extent(
    found: Option<(u32, u64)>,
    capacity: u64,
    fallback: u32,
) -> Result<(u32, u64)> {
    let (first, sectors) = match found {
        Some(e) => e,
        None => (
            fallback,
            capacity.checked_sub(fallback as u64).ok_or(Error::Bounds)?,
        ),
    };
    if sectors == 0 || (first as u64).checked_add(sectors).ok_or(Error::Bounds)? > capacity {
        return Err(Error::Bounds);
    }
    Ok((first, sectors))
}
#[cfg(test)]
mod extent_tests {
    use super::*;
    #[test]
    fn actual_partition_wins_over_unavailable_raw_fallback() {
        assert_eq!(source_extent(Some((34, 64)), 100, 8192), Ok((34, 64)));
        assert_eq!(source_extent(None, 16384, 8192), Ok((8192, 8192)));
        for (found, cap, fallback) in [
            (None, 8191, 8192),
            (None, 0, 0),
            (Some((90, 11)), 100, 0),
            (Some((1, 0)), 100, 0),
            (Some((1, u64::MAX)), u64::MAX, 0),
        ] {
            assert!(source_extent(found, cap, fallback).is_err());
        }
    }
}
