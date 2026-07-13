// SPDX-License-Identifier: GPL-2.0-only
// Verbatim copy of ore-edk-boot-opi c91bbc4e40
// oreboot/src/mainboard/spacemit/k1x/bt0/src/gpt.rs, the oracle for the
// equivalence tests (its own unit tests removed).
#![allow(dead_code, clippy::all)]
//! Bounded primary-GPT reader. All addresses and capacities are 512-byte sectors.
//! A corrupt protective GPT is never interpreted as a raw-FIT fallback.
pub const SECTOR: usize = 512;
const TYPE_FIRMWARE: [u8; 16] = [
    0x53, 0xb3, 0x54, 0x2e, 0x71, 0x12, 0x42, 0x48, 0x80, 0x6f, 0xe4, 0x36, 0xd6, 0xaf, 0x69, 0x85,
];
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PartitionExtent {
    pub first_lba: u32,
    pub sectors: u64,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GptError {
    Read,
    Header,
    HeaderCrc,
    Table,
    TableCrc,
    Partition,
    AddressWidth,
}
pub trait SectorReader {
    fn read_sector(&self, lba: u32, out: &mut [u8; SECTOR]) -> Result<(), GptError>;
}
fn u32le(b: &[u8], o: usize) -> u32 {
    u32::from_le_bytes(b[o..o + 4].try_into().unwrap())
}
fn u64le(b: &[u8], o: usize) -> u64 {
    u64::from_le_bytes(b[o..o + 8].try_into().unwrap())
}
pub fn crc_update(mut crc: u32, bytes: &[u8]) -> u32 {
    for b in bytes {
        crc ^= *b as u32;
        for _ in 0..8 {
            crc = (crc >> 1) ^ ((0u32.wrapping_sub(crc & 1)) & 0xedb88320);
        }
    }
    crc
}
/// Validate the complete header and entry-array before returning a candidate.
/// Name has precedence over firmware type, including when the type comes first.
pub fn find<R: SectorReader>(
    r: &R,
    name: &str,
    capacity: u64,
) -> Result<Option<PartitionExtent>, GptError> {
    if capacity < 3 || capacity > 1u64 << 32 {
        return Err(GptError::AddressWidth);
    }
    let mut h = [0u8; SECTOR];
    r.read_sector(1, &mut h)?;
    if &h[..8] != b"EFI PART" {
        r.read_sector(0, &mut h)?;
        if h[510..512] == [0x55, 0xaa] && (0..4).any(|i| h[446 + 16 * i + 4] == 0xee) {
            return Err(GptError::Header);
        }
        return Ok(None);
    }
    let hs = u32le(&h, 12) as usize;
    if u32le(&h, 8) != 0x10000
        || !(92..=512).contains(&hs)
        || u32le(&h, 20) != 0
        || u64le(&h, 24) != 1
    {
        return Err(GptError::Header);
    }
    let expected = u32le(&h, 16);
    h[16..20].fill(0);
    if !crc_update(!0, &h[..hs]) != expected {
        return Err(GptError::HeaderCrc);
    }
    let backup = u64le(&h, 32);
    let first = u64le(&h, 40);
    let last = u64le(&h, 48);
    let table = u64le(&h, 72);
    let count = u32le(&h, 80) as u64;
    let size = u32le(&h, 84) as u64;
    // Limit firmware parser work and stack independently of untrusted input.
    if count == 0 || count > 256 || size < 128 || !size.is_power_of_two() || size > 4096 {
        return Err(GptError::Table);
    }
    let bytes = count.checked_mul(size).ok_or(GptError::Table)?;
    if bytes > 128 * 1024 {
        return Err(GptError::Table);
    }
    let end = table
        .checked_add(bytes.div_ceil(512))
        .ok_or(GptError::Table)?;
    if backup
        .checked_sub(bytes.div_ceil(512))
        .filter(|b| last < *b)
        .is_none()
        || backup != capacity - 1
        || first > last
        || first < 2
        || last >= capacity
        || table < 2
        || end > first
    {
        return Err(GptError::Table);
    }
    if table > u32::MAX as u64 || end > 1u64 << 32 {
        return Err(GptError::AddressWidth);
    }
    let mut sector = [0u8; SECTOR];
    let mut crc = !0;
    let mut named = None;
    let mut typed = None;
    let mut seen = 0;
    let mut extents = [(0u32, 0u32); 256];
    let mut used = 0;
    for lba in table..end {
        r.read_sector(lba as u32, &mut sector)?;
        let left = (bytes - seen).min(512) as usize;
        crc = crc_update(crc, &sector[..left]);
        for off in (0..left).step_by(128) {
            let pos = seen + off as u64;
            if pos % size != 0 {
                continue;
            }
            let e = &sector[off..off + 128];
            if e[..16].iter().all(|b| *b == 0) {
                continue;
            }
            let start = u64le(e, 32);
            let finish = u64le(e, 40);
            if start < first || finish > last || start > finish {
                return Err(GptError::Partition);
            }
            if start > u32::MAX as u64 || finish > u32::MAX as u64 {
                return Err(GptError::AddressWidth);
            }
            if extents[..used]
                .iter()
                .any(|(a, b)| start <= *b as u64 && finish >= *a as u64)
            {
                return Err(GptError::Partition);
            }
            extents[used] = (start as u32, finish as u32);
            used += 1;
            let extent = PartitionExtent {
                first_lba: start as u32,
                sectors: finish - start + 1,
            };
            let mut chars = name.encode_utf16();
            let mut matches = true;
            for n in 0..36 {
                let actual = u16::from_le_bytes([e[56 + 2 * n], e[57 + 2 * n]]);
                let want = chars.next().unwrap_or(0);
                if actual != want {
                    matches = false;
                }
            }
            if chars.next().is_some() {
                matches = false;
            }
            if matches && named.is_none() {
                named = Some(extent);
            }
            if e[..16] == TYPE_FIRMWARE && typed.is_none() {
                typed = Some(extent);
            }
        }
        seen += left as u64;
    }
    if !crc != u32le(&h, 88) {
        return Err(GptError::TableCrc);
    }
    Ok(named.or(typed))
}
