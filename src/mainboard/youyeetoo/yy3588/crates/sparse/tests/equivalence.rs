// SPDX-License-Identifier: GPL-2.0-only
//! The crate against the verbatim K1 board copy (tests/legacy/k1_sparse.rs):
//! seeded-random valid and mutated sparse images must give the same result
//! and the same writer call trace.
#[path = "legacy/k1_sparse.rs"]
mod legacy;

#[derive(Debug, PartialEq)]
enum Call {
    Raw(u32, Vec<u8>),
    Fill(u32, u64, u32),
}
#[derive(Default)]
struct Trace(Vec<Call>, u32);
impl sparse::Writer for Trace {
    fn raw(&mut self, lba: u32, data: &[u8]) -> Result<(), &'static str> {
        self.0.push(Call::Raw(lba, data.to_vec()));
        if self.1 != 0 && self.0.len() as u32 == self.1 {
            return Err("injected");
        }
        Ok(())
    }
    fn fill(&mut self, lba: u32, sectors: u64, value: u32) -> Result<(), &'static str> {
        self.0.push(Call::Fill(lba, sectors, value));
        if self.1 != 0 && self.0.len() as u32 == self.1 {
            return Err("injected");
        }
        Ok(())
    }
}
impl legacy::Writer for Trace {
    fn raw(&mut self, lba: u32, data: &[u8]) -> Result<(), &'static str> {
        sparse::Writer::raw(self, lba, data)
    }
    fn fill(&mut self, lba: u32, sectors: u64, value: u32) -> Result<(), &'static str> {
        sparse::Writer::fill(self, lba, sectors, value)
    }
}

struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }
    fn below(&mut self, n: u64) -> u64 {
        self.next() % n
    }
}

fn crc(mut c: u32, b: &[u8]) -> u32 {
    for v in b {
        c ^= *v as u32;
        for _ in 0..8 {
            c = (c >> 1) ^ ((0u32.wrapping_sub(c & 1)) & 0xedb88320);
        }
    }
    c
}

/// A valid image of random RAW/FILL/SKIP chunks, optionally with a CRC chunk.
fn image(r: &mut Rng) -> Vec<u8> {
    let block = 512 * (1 + r.below(8) as usize);
    let n = 1 + r.below(6) as usize;
    let mut chunks = Vec::new();
    let mut expanded = Vec::new();
    let mut blocks = 0u32;
    for _ in 0..n {
        let count = 1 + r.below(3) as u32;
        blocks += count;
        let mut ch = Vec::new();
        match r.below(3) {
            0 => {
                let data: Vec<u8> = (0..block * count as usize)
                    .map(|_| r.next() as u8)
                    .collect();
                ch.extend(0xcac1u16.to_le_bytes());
                ch.extend([0, 0]);
                ch.extend(count.to_le_bytes());
                ch.extend((12 + data.len() as u32).to_le_bytes());
                ch.extend(&data);
                expanded.extend(&data);
            }
            1 => {
                let v = r.next() as u32;
                ch.extend(0xcac2u16.to_le_bytes());
                ch.extend([0, 0]);
                ch.extend(count.to_le_bytes());
                ch.extend(16u32.to_le_bytes());
                ch.extend(v.to_le_bytes());
                for _ in 0..block * count as usize / 4 {
                    expanded.extend(v.to_le_bytes());
                }
            }
            _ => {
                ch.extend(0xcac3u16.to_le_bytes());
                ch.extend([0, 0]);
                ch.extend(count.to_le_bytes());
                ch.extend(12u32.to_le_bytes());
                expanded.extend(vec![0u8; block * count as usize]);
            }
        }
        chunks.push(ch);
    }
    let with_crc = r.below(2) == 0;
    let mut nchunks = chunks.len() as u32;
    if with_crc {
        let mut ch = Vec::new();
        ch.extend(0xcac4u16.to_le_bytes());
        ch.extend([0, 0]);
        ch.extend(0u32.to_le_bytes());
        ch.extend(16u32.to_le_bytes());
        ch.extend((!crc(!0, &expanded)).to_le_bytes());
        chunks.push(ch);
        nchunks += 1;
    }
    let mut b = Vec::new();
    b.extend(0xed26ff3au32.to_le_bytes());
    b.extend(1u16.to_le_bytes());
    b.extend(0u16.to_le_bytes());
    b.extend(28u16.to_le_bytes());
    b.extend(12u16.to_le_bytes());
    b.extend((block as u32).to_le_bytes());
    b.extend(blocks.to_le_bytes());
    b.extend(nchunks.to_le_bytes());
    b.extend(if r.below(4) == 0 {
        (!crc(!0, &expanded)).to_le_bytes()
    } else {
        [0; 4]
    });
    for c in chunks {
        b.extend(c);
    }
    b
}

fn same(b: &[u8], capacity: u64, fail_at: u32) -> bool {
    let mut a = Trace(Vec::new(), fail_at);
    let mut l = Trace(Vec::new(), fail_at);
    let ra = sparse::write(b, capacity, &mut a);
    let rl = legacy::write(b, capacity, &mut l);
    assert_eq!(
        ra,
        rl,
        "result differs for {} bytes, capacity {capacity}",
        b.len()
    );
    assert_eq!(a.0, l.0, "writer calls differ");
    ra.is_ok()
}

#[test]
fn valid_and_mutated_images_match_the_board_code() {
    let mut r = Rng(0x5eed_1234_abcd_0001);
    let mut accepted = 0;
    for _ in 0..3000 {
        let img = image(&mut r);
        let capacity = [0, 1, 64, 1 << 20, 1 << 33][r.below(5) as usize];
        let fail_at = [0, 1, 2][r.below(3) as usize];
        accepted += same(&img, capacity, fail_at) as u32;
        // Mutations: flip a byte, truncate, extend.
        let mut m = img.clone();
        let i = r.below(m.len() as u64) as usize;
        m[i] ^= 1 << r.below(8);
        let _ = same(&m, 1 << 20, 0);
        let cut = r.below(img.len() as u64) as usize;
        let _ = same(&img[..cut], 1 << 20, 0);
        let mut e = img.clone();
        e.extend([0u8; 7]);
        let _ = same(&e, 1 << 20, 0);
    }
    assert!(accepted > 300, "only {accepted} images were written");
}
