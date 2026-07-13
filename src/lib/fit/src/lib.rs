// SPDX-License-Identifier: GPL-2.0-only
//! Checked external-data FIT (`mkimage -E`) planning, verification and the
//! board DT memory fixup.
//!
//! From ore-edk-boot-opi `bt0/src/fit_checked.rs` (K1). Every K1 constant
//! (image load addresses, EDK2 code and FD sizes, the variable-store slot,
//! destination limits, memory node names and sizes) is a [`Layout`] or
//! [`MemoryLayout`] field the board supplies; the checks and their order are
//! the board's, so a board layout gives the same results and errors
//! (`tests/equivalence.rs` against the verbatim K1 copy).
//!
//! A plan names three images, in this order: the firmware (e.g. OpenSBI
//! `fw_dynamic`), the next stage (e.g. EDK2) and the DT. No source byte is
//! read and no destination byte written while a plan is made.
#![no_std]

pub use fdt_checked;
use fdt_checked::{self as fdt, Fdt, Kind};

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
pub type Result<T> = core::result::Result<T, Error>;

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
    /// Firmware, next stage, DT.
    pub images: [Image; 3],
    pub end: usize,
    pub metadata: usize,
}

/// A data-only image node next to the next stage that reserves the rest of
/// its slot (K1: `nvram-reserved`, the EDK2 variable store tail of the FD):
/// it must sit at `offset`, be `size` bytes, uncompressed, and carry no
/// load, entry or embedded data.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Reserved {
    pub node: &'static [u8],
    pub offset: usize,
    pub size: usize,
}

/// The board's FIT contract.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Layout {
    /// `arch` of the loaded images (`riscv`).
    pub arch: &'static [u8],
    pub max_fit: usize,
    pub max_metadata: usize,
    /// Required start of external data (metadata rounded to 4), if fixed.
    pub data_base: Option<usize>,
    /// Load and entry address of the firmware image (`type = "firmware"`).
    pub firmware_load: usize,
    /// Load and entry address of the next stage (`type = "standalone"`).
    pub next_load: usize,
    /// With `reserved`: the next stage's exact size, at `data_base`.
    pub next_size: Option<usize>,
    pub reserved: Option<Reserved>,
    /// Size of the next stage's slot, in the source and in RAM: the
    /// firmware and DT data start after it, the DT is placed after it.
    pub next_slot: usize,
    pub dt_align: usize,
    /// End limits of the destinations.
    pub firmware_limit: usize,
    pub payload_limit: usize,
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
    l: &Layout,
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
    if load.is_some() && string_prop(f, n, b"arch")? != l.arch {
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

/// Plan the default configuration of `metadata` against `staging` (where the
/// whole FIT is read). No source read or destination write happens here.
pub fn plan(l: &Layout, metadata: &[u8], staging: Region) -> Result<Plan> {
    let f = Fdt::new(metadata)?;
    if f.h.total > l.max_metadata {
        return Err(Error::Bounds);
    }
    let config = f.child(f.h.structure, b"configurations")?;
    let default = string_prop(&f, config, b"default")?;
    let c = f.child(config, default)?;
    let data_base = align(f.h.total, 4)?;
    if let Some(expected) = l.data_base {
        if data_base != expected {
            return Err(Error::Image);
        }
    }
    let fw = image(
        l,
        &f,
        string_prop(&f, c, b"firmware")?,
        Some(l.firmware_load),
        data_base,
        b"firmware",
    )?;
    let next = image(
        l,
        &f,
        string_prop(&f, c, b"loadables")?,
        Some(l.next_load),
        data_base,
        b"standalone",
    )?;
    let mut dt = image(
        l,
        &f,
        string_prop(&f, c, b"fdt")?,
        None,
        data_base,
        b"flat_dt",
    )?;
    if let Some(r) = l.reserved {
        let reserved = f.child(f.child(f.h.structure, b"images")?, r.node)?;
        if next.data.start != data_base
            || Some(next.data.size) != l.next_size
            || number_prop(&f, reserved, b"data-offset")? != r.offset
            || number_prop(&f, reserved, b"data-size")? != r.size
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
    }
    let slot_end = data_base.checked_add(l.next_slot).ok_or(Error::Bounds)?;
    if fw.data.start < slot_end || dt.data.start < slot_end {
        return Err(Error::Image);
    }
    dt.destination.start = align(
        next.destination
            .start
            .checked_add(l.next_slot)
            .ok_or(Error::Bounds)?,
        l.dt_align,
    )?;
    let images = [fw, next, dt];
    let mut end = 0;
    if fw.destination.end()? > l.firmware_limit
        || next.destination.end()? > l.payload_limit
        || dt.destination.end()? > l.payload_limit
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
    if end > l.max_fit || align(end, 512)? > staging.size {
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

/// Checked sector range for a bounded read of `count` sectors at `relative`
/// within an extent of `sectors` starting at `first`.
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

/// Select a validated partition, evaluating the raw fallback only when absent.
/// Recheck its full extent against the medium's actual capacity.
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

/// The board DT's two explicit 64-bit memory slots (K1: `memory@0` up to
/// 2 GiB, `memory@100000000` for the rest) and the sizes a board may have.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MemoryLayout {
    pub low: &'static [u8],
    pub high: &'static [u8],
    pub low_base: u64,
    pub low_max: u64,
    pub high_base: u64,
    pub sizes: &'static [u64],
}

/// Rewrite both memory slots for `size`, turning the upper one into NOPs when
/// everything fits the lower one. The complete DT and both nodes are
/// validated before any byte changes.
pub fn fix_memory(m: &MemoryLayout, bytes: &mut [u8], size: u64) -> Result<()> {
    if !m.sizes.contains(&size) {
        return Err(Error::Memory);
    }
    let f = Fdt::new(bytes)?;
    let root = f.h.structure;
    if number_prop(&f, root, b"#address-cells")? != 2 || number_prop(&f, root, b"#size-cells")? != 2
    {
        return Err(Error::Memory);
    }
    let low = f.child(root, m.low)?;
    let high = f.child(root, m.high)?;
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
    let low_size = size.min(m.low_max);
    let high_size = size - low_size;
    bytes[l..l + 8].copy_from_slice(&m.low_base.to_be_bytes());
    bytes[l + 8..l + 16].copy_from_slice(&low_size.to_be_bytes());
    if high_size == 0 {
        for p in (high..high_end).step_by(4) {
            bytes[p..p + 4].copy_from_slice(&4u32.to_be_bytes());
        }
    } else {
        bytes[h..h + 8].copy_from_slice(&m.high_base.to_be_bytes());
        bytes[h + 8..h + 16].copy_from_slice(&high_size.to_be_bytes());
    }
    Ok(())
}
