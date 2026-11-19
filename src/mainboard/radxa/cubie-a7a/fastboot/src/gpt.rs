//! GUID partition table, read-only: the primary header at LBA 1 and its
//! entry array, both CRC-checked, for `fetch:usb0p<N>` (N as Linux numbers
//! them: the entry's index + 1).
use crate::crc32;

pub const SIGNATURE: &[u8; 8] = b"EFI PART";
/// Partitions exposed as `usb0p1` .. `usb0p16`.
pub const MAX_PARTS: usize = 16;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Header {
    pub entries_lba: u64,
    pub count: u32,
    pub entry_size: u32,
    pub entries_crc: u32,
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

/// The header in `lba1` (one block) of a disk of `blocks` blocks.
pub fn header(lba1: &[u8], blocks: u64) -> Result<Header, &'static str> {
    if lba1.len() < 92 || &lba1[..8] != SIGNATURE {
        return Err("no GPT");
    }
    let size = le32(lba1, 12) as usize;
    if !(92..=lba1.len()).contains(&size) {
        return Err("GPT header size");
    }
    let mut h = [0u8; 512];
    h[..size].copy_from_slice(&lba1[..size]);
    h[16..20].fill(0);
    if crc32(&h[..size]) != le32(lba1, 16) {
        return Err("GPT header CRC");
    }
    let hdr = Header {
        entries_lba: le64(lba1, 72),
        count: le32(lba1, 80),
        entry_size: le32(lba1, 84),
        entries_crc: le32(lba1, 88),
    };
    if hdr.entry_size < 128
        || !hdr.entry_size.is_multiple_of(8)
        || hdr.count == 0
        || hdr.count > 1024
        || hdr.entries_lba >= blocks
    {
        return Err("GPT entry array");
    }
    Ok(hdr)
}

/// Bytes of the entry array.
pub fn entries_len(h: &Header) -> usize {
    h.count as usize * h.entry_size as usize
}

/// The used entries among the first [`MAX_PARTS`], by index, after the
/// array CRC check; extents past the disk are refused.
pub fn parts(
    h: &Header,
    array: &[u8],
    blocks: u64,
) -> Result<[Option<Part>; MAX_PARTS], &'static str> {
    let n = entries_len(h);
    if array.len() < n || crc32(&array[..n]) != h.entries_crc {
        return Err("GPT entry array CRC");
    }
    let mut out = [None; MAX_PARTS];
    for (i, o) in out.iter_mut().enumerate().take(h.count as usize) {
        let e = &array[i * h.entry_size as usize..][..128];
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
    let c = crc32(&h[..92]);
    h[16..20].copy_from_slice(&c.to_le_bytes());
    (h, array)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn valid_table() {
        let (h, a) = build(1 << 21, &[(2048, 1050623), (1050624, 2097118)]);
        let hdr = header(&h, 1 << 21).unwrap();
        assert_eq!(
            (hdr.entries_lba, hdr.count, entries_len(&hdr)),
            (2, 128, 16384)
        );
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
        let (h, a) = build(1 << 20, &[]);
        let hdr = header(&h, 1 << 20).unwrap();
        assert!(parts(&hdr, &a, 1 << 20)
            .unwrap()
            .iter()
            .all(|x| x.is_none()));
    }

    #[test]
    fn corrupt_tables() {
        let (mut h, mut a) = build(1 << 21, &[(2048, 4095)]);
        let hdr = header(&h, 1 << 21).unwrap();
        a[200] ^= 1;
        assert_eq!(parts(&hdr, &a, 1 << 21), Err("GPT entry array CRC"));
        h[30] ^= 1;
        assert_eq!(header(&h, 1 << 21), Err("GPT header CRC"));
        assert_eq!(header(&[0u8; 512], 1 << 21), Err("no GPT"));
        // An entry past the end of a (shorter) disk.
        let (h, a) = build(1 << 21, &[(2048, (1 << 21) - 1)]);
        let hdr = header(&h, 1 << 21).unwrap();
        assert_eq!(parts(&hdr, &a, 1 << 20), Err("GPT entry outside the disk"));
        let (h, a) = build(1 << 21, &[(5000, 4000)]);
        assert_eq!(
            parts(&header(&h, 1 << 21).unwrap(), &a, 1 << 21),
            Err("GPT entry outside the disk")
        );
    }
}
