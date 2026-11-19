// SPDX-License-Identifier: GPL-2.0-only

//! GUID partition table, read-only: the primary header at LBA 1 and its
//! entry array, both CRC-checked, for `fetch:usb0p<N>` (N as Linux numbers
//! them: the entry's index + 1).
use crate::crc32;

pub const SIGNATURE: &[u8; 8] = b"EFI PART";
/// Partitions exposed as `usb0p1` .. `usb0p16`.
pub const MAX_PARTS: usize = 16;

pub(crate) fn valid_block_size(size: u32) -> bool {
    matches!(size, 512 | 1024 | 2048 | 4096)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Header {
    pub entries_lba: u64,
    entry_size: usize,
    entries_crc: u32,
    entries_len: usize,
    read_len: usize,
}

impl Header {
    /// Bytes to read, including padding to the end of the last block.
    pub fn entries_read_len(&self) -> usize {
        self.read_len
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Part {
    pub first: u64,
    pub last: u64,
}

fn le64(b: &[u8], o: usize) -> u64 {
    let mut x = [0u8; 8];
    x.copy_from_slice(&b[o..o + 8]);
    u64::from_le_bytes(x)
}

fn le32(b: &[u8], o: usize) -> u32 {
    u32::from_le_bytes([b[o], b[o + 1], b[o + 2], b[o + 3]])
}

/// The header in `lba1` (exactly one logical block) of a disk of `blocks`
/// blocks. The CRC field in the scratch block is temporarily zeroed.
pub fn header(lba1: &mut [u8], blocks: u64) -> Result<Header, &'static str> {
    let block_size = u32::try_from(lba1.len()).map_err(|_| "GPT block size")?;
    if !valid_block_size(block_size) {
        return Err("GPT block size");
    }
    if blocks <= 1 || &lba1[..8] != SIGNATURE {
        return Err("no GPT");
    }
    let size = le32(lba1, 12) as usize;
    if !(92..=lba1.len()).contains(&size) {
        return Err("GPT header size");
    }
    let expected = le32(lba1, 16);
    lba1[16..20].fill(0);
    let actual = crc32(&lba1[..size]);
    lba1[16..20].copy_from_slice(&expected.to_le_bytes());
    if actual != expected {
        return Err("GPT header CRC");
    }
    let entries_lba = le64(lba1, 72);
    let count = le32(lba1, 80);
    let entry_size = le32(lba1, 84);
    if entry_size < 128 || !entry_size.is_multiple_of(8) || !(1..=1024).contains(&count) {
        return Err("GPT entry array");
    }
    let entries_len = count
        .checked_mul(entry_size)
        .ok_or("GPT entry array size")?;
    let read_len = entries_len
        .checked_next_multiple_of(block_size)
        .ok_or("GPT entry array size")?;
    let read_blocks = u64::from(read_len / block_size);
    if entries_lba < 2
        || entries_lba
            .checked_add(read_blocks)
            .is_none_or(|end| end > blocks)
    {
        return Err("GPT entry array outside the disk");
    }
    Ok(Header {
        entries_lba,
        entry_size: entry_size as usize,
        entries_crc: le32(lba1, 88),
        entries_len: entries_len as usize,
        read_len: read_len as usize,
    })
}

/// The used entries among the first [`MAX_PARTS`], by index, after the
/// array CRC check; extents past the disk are refused.
pub fn parts(
    h: &Header,
    array: &[u8],
    blocks: u64,
) -> Result<[Option<Part>; MAX_PARTS], &'static str> {
    let array = array.get(..h.entries_len).ok_or("short GPT entry array")?;
    if crc32(array) != h.entries_crc {
        return Err("GPT entry array CRC");
    }
    let mut out = [None; MAX_PARTS];
    for (o, e) in out.iter_mut().zip(array.chunks_exact(h.entry_size)) {
        if e[..16].iter().all(|&b| b == 0) {
            continue;
        }
        let (first, last) = (le64(e, 32), le64(e, 40));
        if first > last || last >= blocks {
            return Err("GPT entry outside the disk");
        }
        *o = Some(Part { first, last });
    }
    Ok(out)
}

#[cfg(test)]
pub(crate) fn build(blocks: u64, parts: &[(u64, u64)]) -> (std::vec::Vec<u8>, std::vec::Vec<u8>) {
    let mut array = std::vec![0u8; 128 * 128];
    for (i, &(f, l)) in parts.iter().enumerate() {
        let e = &mut array[i * 128..][..128];
        e[0] = 0xaf; // a type GUID
        e[32..40].copy_from_slice(&f.to_le_bytes());
        e[40..48].copy_from_slice(&l.to_le_bytes());
    }
    let mut h = std::vec![0u8; 512];
    h[..8].copy_from_slice(SIGNATURE);
    h[8..12].copy_from_slice(&0x0001_0000u32.to_le_bytes());
    h[12..16].copy_from_slice(&92u32.to_le_bytes());
    h[24..32].copy_from_slice(&1u64.to_le_bytes());
    h[32..40].copy_from_slice(&(blocks - 1).to_le_bytes());
    h[72..80].copy_from_slice(&2u64.to_le_bytes());
    h[80..84].copy_from_slice(&128u32.to_le_bytes());
    h[84..88].copy_from_slice(&128u32.to_le_bytes());
    h[88..92].copy_from_slice(&crc32(&array).to_le_bytes());
    fix_header_crc(&mut h);
    (h, array)
}

#[cfg(test)]
pub(crate) fn fix_header_crc(h: &mut [u8]) {
    h[16..20].fill(0);
    let c = crc32(&h[..le32(h, 12) as usize]);
    h[16..20].copy_from_slice(&c.to_le_bytes());
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn logical_block_geometry() {
        let (mut h, a) = build(1024, &[(50, 60)]);
        let hdr = header(&mut h, 1024).unwrap();
        assert!(parts(&hdr, &a, 1024).unwrap()[0].is_some());
        h.resize(4096, 0x5a);
        h[12..16].copy_from_slice(&1024u32.to_le_bytes());
        fix_header_crc(&mut h);
        let before = h.clone();
        let hdr = header(&mut h, 1024).unwrap();
        assert_eq!(h, before);
        assert!(parts(&hdr, &a, 1024).unwrap()[0].is_some());
    }

    #[test]
    fn rejects_non_power_of_two_block() {
        let (mut h, _) = build(1024, &[]);
        h.resize(520, 0);
        assert_eq!(header(&mut h, 1024), Err("GPT block size"));
    }

    #[test]
    fn rejects_header_larger_than_block() {
        let (mut h, _) = build(1024, &[]);
        h[12..16].copy_from_slice(&513u32.to_le_bytes());
        assert_eq!(header(&mut h, 1024), Err("GPT header size"));
    }

    #[test]
    fn rejects_entry_length_overflow_at_32_bits() {
        let (mut h, _) = build(u64::MAX, &[]);
        for (count, size) in [(2u32, 0x8000_0000u32), (1, 0xffff_fff8)] {
            h[80..84].copy_from_slice(&count.to_le_bytes());
            h[84..88].copy_from_slice(&size.to_le_bytes());
            fix_header_crc(&mut h);
            assert_eq!(header(&mut h, u64::MAX), Err("GPT entry array size"));
        }
    }

    #[test]
    fn rounded_array_must_fit_on_disk() {
        let (mut h, _) = build(1024, &[]);
        h[80..84].copy_from_slice(&5u32.to_le_bytes()); // 640 bytes, two sectors
        h[72..80].copy_from_slice(&1023u64.to_le_bytes());
        fix_header_crc(&mut h);
        assert_eq!(
            header(&mut h, 1024),
            Err("GPT entry array outside the disk")
        );
        assert!(header(&mut h, 1025).is_ok());
        h[72..80].copy_from_slice(&(u64::MAX - 1).to_le_bytes());
        fix_header_crc(&mut h);
        assert_eq!(
            header(&mut h, u64::MAX),
            Err("GPT entry array outside the disk")
        );
    }

    #[test]
    fn header_count_must_fit_bytes_actually_read() {
        let (mut h, a) = build(1024, &[(50, 60)]);
        let hdr = header(&mut h, 1024).unwrap();
        assert_eq!(
            parts(&hdr, &a[..a.len() - 1], 1024),
            Err("short GPT entry array")
        );
    }

    #[test]
    fn valid_table() {
        let (mut h, a) = build(1 << 21, &[(2048, 1050623), (1050624, 2097118)]);
        let hdr = header(&mut h, 1 << 21).unwrap();
        assert_eq!((hdr.entries_lba, hdr.entries_read_len()), (2, 16384));
        let p = parts(&hdr, &a, 1 << 21).unwrap();
        assert_eq!(
            p[0],
            Some(Part {
                first: 2048,
                last: 1050623
            })
        );
        assert_eq!(
            p[1],
            Some(Part {
                first: 1050624,
                last: 2097118
            })
        );
        assert!(p[2..].iter().all(|x| x.is_none()));
        // An empty table.
        let (mut h, a) = build(1 << 20, &[]);
        let hdr = header(&mut h, 1 << 20).unwrap();
        assert!(parts(&hdr, &a, 1 << 20)
            .unwrap()
            .iter()
            .all(|x| x.is_none()));
    }

    #[test]
    fn corrupt_tables() {
        let (mut h, mut a) = build(1 << 21, &[(2048, 4095)]);
        let hdr = header(&mut h, 1 << 21).unwrap();
        a[200] ^= 1;
        assert_eq!(parts(&hdr, &a, 1 << 21), Err("GPT entry array CRC"));
        h[30] ^= 1;
        assert_eq!(header(&mut h, 1 << 21), Err("GPT header CRC"));
        assert_eq!(header(&mut [0u8; 512], 1 << 21), Err("no GPT"));
        // An entry past the end of a (shorter) disk.
        let (mut h, a) = build(1 << 21, &[(2048, (1 << 21) - 1)]);
        let hdr = header(&mut h, 1 << 21).unwrap();
        assert_eq!(parts(&hdr, &a, 1 << 20), Err("GPT entry outside the disk"));
        let (mut h, a) = build(1 << 21, &[(5000, 4000)]);
        assert_eq!(
            parts(&header(&mut h, 1 << 21).unwrap(), &a, 1 << 21),
            Err("GPT entry outside the disk")
        );
    }
}
