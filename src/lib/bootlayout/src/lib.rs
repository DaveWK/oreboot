// SPDX-License-Identifier: GPL-2.0-only
//! Boot-stage layout table on a raw boot medium.
//!
//! From ore-edk-boot-radxa-a7a `common/src/lib.rs` `layout` (A7A, format
//! `A7ABOOT1` version 1). A 280-byte table at a fixed medium offset lists up
//! to eight stages (kind, medium offset, size, load address, CRC-32); the
//! loader decodes it (CRC-32 over the table with its CRC field zeroed),
//! validates every entry against the board's [`Contract`] and only then
//! loads the stages. The table layout is fixed; the magic, version, medium
//! window, DRAM base, address width, the kinds and where each may load are
//! the board's. `tools/bootlayout.py` writes tables.
#![no_std]

pub const MAX_ENTRIES: usize = 8;

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Entry {
    pub kind: u32,
    pub flags: u32,
    /// Byte offset on the medium, block aligned.
    pub offset: u32,
    /// Size in bytes.
    pub size: u32,
    pub load_addr: u64,
    /// CRC-32 (IEEE) of the image; 0 skips verification.
    pub crc32: u32,
    pub _reserved: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Table {
    pub magic: [u8; 8],
    pub version: u32,
    pub count: u32,
    /// CRC-32 over the whole table with this field set to 0.
    pub crc32: u32,
    pub _reserved: u32,
    pub entries: [Entry; MAX_ENTRIES],
}

pub const TABLE_SIZE: usize = core::mem::size_of::<Table>();
pub const CRC_FIELD_OFFSET: usize = 16;
const _: () = assert!(TABLE_SIZE == 24 + 32 * MAX_ENTRIES);
const _: () = assert!(TABLE_SIZE <= 512);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    Header,
    Checksum,
    UnsupportedFields,
    UnknownKind,
    DuplicateKind,
    MissingStage,
    EmptyImage,
    MediaRange,
    MemoryRange,
    LinkedAddress,
    Overlap,
}

/// Where a stage of a kind may load.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Place {
    /// Linked to `addr`; at most `max_size` bytes (padded) if given.
    Exact { addr: u64, max_size: Option<u64> },
    /// Anywhere in DRAM from `min` up.
    AtLeast(u64),
    /// Not loaded by this loader (e.g. a recovery stage another path runs):
    /// linked to `addr`, at most `max_size`; exempt from the DRAM and memory
    /// overlap checks.
    Recovery { addr: u64, max_size: u64 },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct KindRule {
    /// Below 32.
    pub kind: u32,
    pub place: Place,
}

/// The board's table contract.
#[derive(Clone, Copy, Debug)]
pub struct Contract<'a> {
    pub magic: [u8; 8],
    pub version: u32,
    /// Medium window the images occupy, and its block size.
    pub image_start: u64,
    pub image_end: u64,
    pub block_size: u64,
    pub dram_base: u64,
    /// Highest address the loader can hold (an AArch32 bt0:
    /// `u32::MAX as u64`; usually `usize::MAX as u64` of the loader's target).
    pub max_address: u64,
    pub kinds: &'a [KindRule],
    /// Kinds that must be present.
    pub required: &'a [u32],
}

impl Contract<'_> {
    fn rule(&self, kind: u32) -> Option<&KindRule> {
        if kind >= 32 {
            return None;
        }
        self.kinds.iter().find(|r| r.kind == kind)
    }
}

/// Decode the medium frame (any alignment). Checks the length, the header
/// and the table CRC; [`validate`] checks the entries.
pub fn decode(c: &Contract, bytes: &[u8]) -> Result<Table, Error> {
    if bytes.len() < TABLE_SIZE {
        return Err(Error::Header);
    }
    // SAFETY: complete repr(C) integer-only frame, possibly unaligned.
    let table: Table = unsafe { core::ptr::read_unaligned(bytes.as_ptr().cast()) };
    if table.magic != c.magic || table.version != c.version || table.count as usize > MAX_ENTRIES {
        return Err(Error::Header);
    }
    let mut checksum_bytes = [0u8; TABLE_SIZE];
    checksum_bytes.copy_from_slice(&bytes[..TABLE_SIZE]);
    checksum_bytes[CRC_FIELD_OFFSET..CRC_FIELD_OFFSET + 4].fill(0);
    if storage::crc::crc32(&checksum_bytes) != table.crc32 {
        return Err(Error::Checksum);
    }
    Ok(table)
}

/// The table's medium bytes with its CRC filled in (the writer's side; the
/// entries' own CRCs are the caller's).
pub fn encode(table: &Table) -> [u8; TABLE_SIZE] {
    let mut out = [0u8; TABLE_SIZE];
    let mut put = |at: usize, b: &[u8]| out[at..at + b.len()].copy_from_slice(b);
    put(0, &table.magic);
    put(8, &table.version.to_le_bytes());
    put(12, &table.count.to_le_bytes());
    put(20, &table._reserved.to_le_bytes());
    for (i, e) in table.entries.iter().enumerate() {
        let b = 24 + 32 * i;
        put(b, &e.kind.to_le_bytes());
        put(b + 4, &e.flags.to_le_bytes());
        put(b + 8, &e.offset.to_le_bytes());
        put(b + 12, &e.size.to_le_bytes());
        put(b + 16, &e.load_addr.to_le_bytes());
        put(b + 24, &e.crc32.to_le_bytes());
        put(b + 28, &e._reserved.to_le_bytes());
    }
    let crc = storage::crc::crc32(&out);
    out[CRC_FIELD_OFFSET..CRC_FIELD_OFFSET + 4].copy_from_slice(&crc.to_le_bytes());
    out
}

fn padded(c: &Contract, e: &Entry) -> Result<u64, Error> {
    if e.size == 0 {
        return Err(Error::EmptyImage);
    }
    Ok((u64::from(e.size) + c.block_size - 1) & !(c.block_size - 1))
}

fn media_range(c: &Contract, e: &Entry) -> Result<(u64, u64), Error> {
    let start = u64::from(e.offset);
    let end = start.checked_add(padded(c, e)?).ok_or(Error::MediaRange)?;
    if start < c.image_start || start % c.block_size != 0 || end > c.image_end {
        return Err(Error::MediaRange);
    }
    Ok((start, end))
}

fn memory_range(c: &Contract, e: &Entry, dram_end: u64) -> Result<(u64, u64), Error> {
    let start = e.load_addr;
    let end = start.checked_add(padded(c, e)?).ok_or(Error::MemoryRange)?;
    // Check the full 64-bit address against what the loader can hold.
    if start & 3 != 0 || start > c.max_address || end > c.max_address {
        return Err(Error::MemoryRange);
    }
    match c.rule(e.kind).map(|r| r.place) {
        Some(Place::Exact { addr, max_size }) => {
            if start != addr || max_size.is_some_and(|m| padded(c, e).is_ok_and(|p| p > m)) {
                return Err(Error::LinkedAddress);
            }
        }
        Some(Place::AtLeast(min)) => {
            if start < min {
                return Err(Error::MemoryRange);
            }
        }
        Some(Place::Recovery { addr, max_size }) => {
            if start != addr || padded(c, e)? > max_size {
                return Err(Error::LinkedAddress);
            }
            return Ok((start, end));
        }
        None => return Err(Error::UnknownKind),
    }
    if start < c.dram_base || end > dram_end {
        return Err(Error::MemoryRange);
    }
    Ok((start, end))
}

fn overlaps(a: (u64, u64), b: (u64, u64)) -> bool {
    a.0 < b.1 && b.0 < a.1
}

fn recovery(c: &Contract, kind: u32) -> bool {
    matches!(c.rule(kind).map(|r| r.place), Some(Place::Recovery { .. }))
}

/// Validate every entry before any stage is loaded. [`decode`] has checked
/// the table CRC; this checks the stage contract.
pub fn validate(c: &Contract, table: &Table, dram_end: u64) -> Result<(), Error> {
    if table.magic != c.magic || table.version != c.version || table.count as usize > MAX_ENTRIES {
        return Err(Error::Header);
    }
    if table._reserved != 0 {
        return Err(Error::UnsupportedFields);
    }
    let entries = &table.entries[..table.count as usize];
    let mut seen = 0u32;
    for (i, e) in entries.iter().enumerate() {
        if e.flags != 0 || e._reserved != 0 {
            return Err(Error::UnsupportedFields);
        }
        if c.rule(e.kind).is_none() {
            return Err(Error::UnknownKind);
        }
        let bit = 1 << e.kind;
        if seen & bit != 0 {
            return Err(Error::DuplicateKind);
        }
        seen |= bit;
        let media = media_range(c, e)?;
        let memory = memory_range(c, e, dram_end)?;
        for prior in &entries[..i] {
            if overlaps(media, media_range(c, prior)?)
                || (!recovery(c, e.kind)
                    && !recovery(c, prior.kind)
                    && overlaps(memory, memory_range(c, prior, dram_end)?))
            {
                return Err(Error::Overlap);
            }
        }
    }
    for &k in c.required {
        if k >= 32 || seen & (1 << k) == 0 {
            return Err(Error::MissingStage);
        }
    }
    Ok(())
}

/// Validate the whole table, then call `load(entry, load address, padded
/// size)` for each stage in table order; recovery stages are skipped. No
/// callback runs unless the entire table passes.
pub fn load_entries(
    c: &Contract,
    table: &Table,
    dram_end: u64,
    mut load: impl FnMut(&Entry, usize, usize),
) -> Result<(), Error> {
    validate(c, table, dram_end)?;
    for e in &table.entries[..table.count as usize] {
        if !recovery(c, e.kind) {
            // validate proved both conversions and the whole extent.
            load(e, e.load_addr as usize, padded(c, e)? as usize);
        }
    }
    Ok(())
}
