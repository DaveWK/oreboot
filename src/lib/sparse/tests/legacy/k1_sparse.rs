// SPDX-License-Identifier: GPL-2.0-only
// Verbatim copy of ore-edk-boot-opi c91bbc4e40
// oreboot/src/mainboard/spacemit/k1x/bt0/src/sparse.rs, the oracle for the
// equivalence tests (its own unit tests removed).
#![allow(dead_code, clippy::all)]
//! Android sparse-image validation and execution. No writer is called until the
//! complete downloaded image (including checksums and output extent) validates.
const MAGIC: u32 = 0xed26ff3a;
const RAW: u16 = 0xcac1;
const FILL: u16 = 0xcac2;
const SKIP: u16 = 0xcac3;
const CRC: u16 = 0xcac4;
fn u16le(b: &[u8], o: usize) -> u16 {
    u16::from_le_bytes(b[o..o + 2].try_into().unwrap())
}
fn u32le(b: &[u8], o: usize) -> u32 {
    u32::from_le_bytes(b[o..o + 4].try_into().unwrap())
}
fn crc_bytes(mut c: u32, b: &[u8]) -> u32 {
    for v in b {
        c ^= *v as u32;
        for _ in 0..8 {
            c = (c >> 1) ^ ((0u32.wrapping_sub(c & 1)) & 0xedb88320);
        }
    }
    c
}
// GF(2) CRC concatenation, logarithmic in length (reflected IEEE polynomial).
// Finished CRCs compose without visiting expanded fill/skip bytes.
fn matrix_times(m: &[u32; 32], mut v: u32) -> u32 {
    let mut out = 0;
    let mut n = 0;
    while v != 0 {
        if v & 1 != 0 {
            out ^= m[n];
        }
        v >>= 1;
        n += 1;
    }
    out
}
fn matrix_square(m: &[u32; 32]) -> [u32; 32] {
    core::array::from_fn(|i| matrix_times(m, m[i]))
}
fn combine(mut left: u32, right: u32, mut bytes: u64) -> u32 {
    if bytes == 0 {
        return left;
    }
    let mut matrix = [0u32; 32];
    matrix[0] = 0xedb88320;
    for i in 1..32 {
        matrix[i] = 1 << (i - 1);
    }
    // Convert one zero-bit operator to one zero-byte operator.
    for _ in 0..3 {
        matrix = matrix_square(&matrix);
    }
    while bytes != 0 {
        if bytes & 1 != 0 {
            left = matrix_times(&matrix, left);
        }
        bytes >>= 1;
        if bytes != 0 {
            matrix = matrix_square(&matrix);
        }
    }
    left ^ right
}
fn repeated_crc(pattern: &[u8; 512], mut count: u64) -> u32 {
    let mut result = 0;
    let mut crc = !crc_bytes(!0, pattern);
    let mut len = 512;
    while count != 0 {
        if count & 1 != 0 {
            result = combine(result, crc, len);
        }
        count >>= 1;
        if count != 0 {
            crc = combine(crc, crc, len);
            len *= 2;
        }
    }
    result
}
pub trait Writer {
    fn raw(&mut self, lba: u32, data: &[u8]) -> Result<(), &'static str>;
    fn fill(&mut self, lba: u32, sectors: u64, value: u32) -> Result<(), &'static str>;
}
struct Layout {
    header: usize,
    chunk_header: usize,
    block: u64,
    chunks: u32,
    total: u64,
    checksum: u32,
}
fn layout(b: &[u8], capacity: u64) -> Result<Layout, &'static str> {
    if b.len() < 28 || u32le(b, 0) != MAGIC || u16le(b, 4) != 1 {
        return Err("unsupported sparse header");
    }
    let h = u16le(b, 8) as usize;
    let c = u16le(b, 10) as usize;
    let block = u32le(b, 12) as u64;
    let total = u32le(b, 16) as u64;
    let chunks = u32le(b, 20);
    if h < 28
        || h > b.len()
        || c < 12
        || block == 0
        || block % 512 != 0
        || chunks as usize > (b.len() - h) / c
    {
        return Err("invalid sparse geometry");
    }
    let sectors = total
        .checked_mul(block / 512)
        .ok_or("sparse extent overflow")?;
    if capacity == 0 || sectors > capacity || sectors > 1u64 << 32 {
        return Err("image larger than supported partition");
    }
    Ok(Layout {
        header: h,
        chunk_header: c,
        block,
        chunks,
        total,
        checksum: u32le(b, 24),
    })
}
fn walk<F: FnMut(u16, u64, u64, &[u8]) -> Result<(), &'static str>>(
    b: &[u8],
    l: &Layout,
    mut f: F,
) -> Result<(), &'static str> {
    let mut off = l.header;
    let mut at = 0u64;
    for _ in 0..l.chunks {
        let end = off
            .checked_add(l.chunk_header)
            .filter(|e| *e <= b.len())
            .ok_or("truncated sparse chunk header")?;
        let kind = u16le(b, off);
        let count = u32le(b, off + 4) as u64;
        let total = u32le(b, off + 8) as usize;
        if u16le(b, off + 2) != 0 {
            return Err("nonzero sparse chunk reserved field");
        }
        let next = off
            .checked_add(total)
            .filter(|e| *e >= end && *e <= b.len())
            .ok_or("truncated sparse chunk body")?;
        let data = &b[end..next];
        let bytes = count.checked_mul(l.block).ok_or("sparse chunk overflow")?;
        match kind {
            RAW if data.len() as u64 == bytes => {}
            FILL if data.len() == 4 => {}
            SKIP if data.is_empty() => {}
            CRC if data.len() == 4 && count == 0 => {}
            _ => return Err("invalid sparse chunk type/size"),
        }
        if kind != CRC {
            at = at
                .checked_add(count)
                .filter(|a| *a <= l.total)
                .ok_or("sparse block total overflow")?;
        }
        f(kind, if kind == CRC { at } else { at - count }, count, data)?;
        off = next;
    }
    if off != b.len() || at != l.total {
        return Err("sparse totals/trailing bytes mismatch");
    }
    Ok(())
}
pub fn write<W: Writer>(b: &[u8], capacity: u64, w: &mut W) -> Result<(), &'static str> {
    let l = layout(b, capacity)?;
    // First structural pass identifies CRC chunks without trusting any later body.
    let mut crc_needed = l.checksum != 0;
    walk(b, &l, |kind, _, _, _| {
        crc_needed |= kind == CRC;
        Ok(())
    })?;
    if crc_needed {
        let mut crc = !0u32;
        walk(b, &l, |kind, _, count, data| {
            match kind {
                RAW => crc = crc_bytes(crc, data),
                FILL | SKIP => {
                    let mut pattern = [0u8; 512];
                    if kind == FILL {
                        for p in pattern.chunks_exact_mut(4) {
                            p.copy_from_slice(data);
                        }
                    }
                    let sectors = count * (l.block / 512);
                    crc = !combine(!crc, repeated_crc(&pattern, sectors), sectors * 512);
                }
                CRC => {
                    if !crc != u32le(data, 0) {
                        return Err("sparse chunk CRC mismatch");
                    }
                }
                _ => unreachable!(),
            }
            Ok(())
        })?;
        if l.checksum != 0 && !crc != l.checksum {
            return Err("sparse image CRC mismatch");
        }
    }
    walk(b, &l, |kind, at, count, data| {
        if count == 0 {
            return Ok(());
        }
        let lba = u32::try_from(at * (l.block / 512)).map_err(|_| "sparse LBA overflow")?;
        match kind {
            RAW => w.raw(lba, data),
            FILL => w.fill(lba, count * (l.block / 512), u32le(data, 0)),
            _ => Ok(()),
        }
    })
}
