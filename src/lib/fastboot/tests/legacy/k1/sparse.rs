// SPDX-License-Identifier: GPL-2.0-only
// Verbatim copy of ore-edk-boot-opi c91bbc4e40
// oreboot/src/mainboard/spacemit/k1x/bt0/src/sparse.rs (equivalence oracle).
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
#[cfg(test)]
mod tests {
    use super::*;
    #[derive(Default)]
    struct Sink {
        bytes: Vec<u8>,
        calls: usize,
    }
    impl Writer for Sink {
        fn raw(&mut self, lba: u32, b: &[u8]) -> Result<(), &'static str> {
            self.calls += 1;
            let o = lba as usize * 512;
            self.bytes[o..o + b.len()].copy_from_slice(b);
            Ok(())
        }
        fn fill(&mut self, lba: u32, n: u64, v: u32) -> Result<(), &'static str> {
            self.calls += 1;
            for p in self.bytes[lba as usize * 512..(lba as usize + n as usize) * 512]
                .chunks_exact_mut(4)
            {
                p.copy_from_slice(&v.to_le_bytes());
            }
            Ok(())
        }
    }
    fn p32(b: &mut [u8], o: usize, n: u32) {
        b[o..o + 4].copy_from_slice(&n.to_le_bytes());
    }
    fn image() -> Vec<u8> {
        let mut b = vec![0; 28];
        p32(&mut b, 0, MAGIC);
        b[4] = 1;
        b[8] = 28;
        b[10] = 12;
        p32(&mut b, 12, 512);
        p32(&mut b, 16, 3);
        p32(&mut b, 20, 3);
        for (k, count, d) in [
            (RAW, 1, vec![0x5a; 512]),
            (FILL, 1, vec![1, 2, 3, 4]),
            (SKIP, 1, vec![]),
        ] {
            b.extend_from_slice(&k.to_le_bytes());
            b.extend_from_slice(&[0, 0]);
            b.extend_from_slice(&(count as u32).to_le_bytes());
            b.extend_from_slice(&(12 + d.len() as u32).to_le_bytes());
            b.extend_from_slice(&d);
        }
        b
    }
    fn reject(b: &[u8], cap: u64) {
        let mut s = Sink {
            bytes: vec![0xcc; 2048],
            calls: 0,
        };
        assert!(write(b, cap, &mut s).is_err());
        assert_eq!(s.calls, 0);
        assert_eq!(s.bytes, vec![0xcc; 2048]);
    }
    #[test]
    fn valid_consumer() {
        let mut s = Sink {
            bytes: vec![0xcc; 2048],
            calls: 0,
        };
        write(&image(), 4, &mut s).unwrap();
        assert_eq!(s.bytes[..512], vec![0x5a; 512]);
        assert_eq!(s.bytes[512..1024], [1, 2, 3, 4].repeat(128));
        assert_eq!(s.bytes[1024..], vec![0xcc; 1024]);
        assert_eq!(s.calls, 2);
    }
    #[test]
    fn all_truncations_preflight() {
        let b = image();
        for n in 0..b.len() {
            reject(&b[..n], 4);
        }
    }
    #[test]
    fn capacity_and_address_width() {
        reject(&image(), 2);
        reject(&image(), 0);
        let mut b = image();
        p32(&mut b, 12, u32::MAX - 511);
        p32(&mut b, 16, u32::MAX);
        reject(&b, u64::MAX);
    }
    #[test]
    fn late_invalid_chunk_never_writes() {
        let mut b = image();
        let n = b.len();
        b[n - 12] = 0xff;
        reject(&b, 4);
    }
    #[test]
    fn incorrect_totals() {
        for count in [0, 2, 4, u32::MAX] {
            let mut b = image();
            p32(&mut b, 16, count);
            reject(&b, 4);
        }
    }
    #[test]
    fn bad_fill_skip_crc_lengths() {
        for at in [552, 568] {
            let mut b = image();
            p32(&mut b, at + 8, 13);
            reject(&b, 4);
        }
    }
    #[test]
    fn trailing_data() {
        let mut b = image();
        b.push(0);
        reject(&b, 4);
    }
    #[test]
    fn header_extensions() {
        let mut b = image();
        b.splice(28..28, [0; 4]);
        b[8] = 32;
        let mut s = Sink {
            bytes: vec![0; 2048],
            calls: 0,
        };
        write(&b, 4, &mut s).unwrap();
    }
    #[test]
    fn valid_crc_and_corrupt_crc() {
        let mut b = image();
        let expanded = [vec![0x5a; 512], [1, 2, 3, 4].repeat(128), vec![0; 512]].concat();
        let crc = !crc_bytes(!0, &expanded);
        p32(&mut b, 24, crc);
        p32(&mut b, 20, 4);
        b.extend_from_slice(&CRC.to_le_bytes());
        b.extend_from_slice(&[0, 0]);
        b.extend_from_slice(&0u32.to_le_bytes());
        b.extend_from_slice(&16u32.to_le_bytes());
        b.extend_from_slice(&crc.to_le_bytes());
        let mut s = Sink {
            bytes: vec![0; 2048],
            calls: 0,
        };
        write(&b, 4, &mut s).unwrap();
        *b.last_mut().unwrap() ^= 1;
        reject(&b, 4);
    }
    #[test]
    fn logarithmic_crc_independent_zlib_vectors() {
        // Expected values from system zlib crc32/crc32_combine64, not this code.
        let zero = [0; 512];
        let mut pat = [0; 512];
        for p in pat.chunks_exact_mut(4) {
            p.copy_from_slice(&[1, 2, 3, 4]);
        }
        for (n, z, p) in [
            (1048576, 0x6db88320, 0x02a7bf4f),
            (2147483771, 0x2185751f, 0x66a353d2),
            (4294967294, 0x97fe0594, 0x4aa60f4f),
        ] {
            assert_eq!(repeated_crc(&zero, n), z);
            assert_eq!(repeated_crc(&pat, n), p);
        }
        assert_eq!(repeated_crc(&zero, 2048), 0xa738ea1c);
    }
    #[test]
    fn enormous_valid_skip_crc_is_bounded() {
        let mut b = image();
        b.truncate(28);
        p32(&mut b, 16, 2147483771);
        p32(&mut b, 20, 1);
        p32(&mut b, 24, 0x2185751f);
        b.extend_from_slice(&SKIP.to_le_bytes());
        b.extend_from_slice(&[0, 0]);
        b.extend_from_slice(&2147483771u32.to_le_bytes());
        b.extend_from_slice(&12u32.to_le_bytes());
        let mut s = Sink::default();
        write(&b, 1 << 32, &mut s).unwrap();
        assert_eq!(s.calls, 0);
    }
    #[test]
    fn checksum_header_failure() {
        let mut b = image();
        p32(&mut b, 24, 1);
        reject(&b, 4);
    }
    #[test]
    fn zero_size_fill() {
        let mut b = image();
        p32(&mut b, 16, 2);
        p32(&mut b, 556, 0);
        let mut s = Sink {
            bytes: vec![0xcc; 2048],
            calls: 0,
        };
        write(&b, 4, &mut s).unwrap();
        assert_eq!(s.calls, 1);
    }
}
