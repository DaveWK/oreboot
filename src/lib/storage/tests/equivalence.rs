// SPDX-License-Identifier: GPL-2.0-only
//! The crate against the verbatim K1 board copies (tests/legacy/): random
//! valid and mutated GPT disks, and random CSD registers.
#[path = "legacy/k1_gpt.rs"]
mod legacy_gpt;
#[path = "legacy/k1_sd_capacity.rs"]
mod legacy_sd;

use std::collections::HashMap;
use storage::{gpt, sd};

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

struct Disk(HashMap<u32, [u8; 512]>);
impl gpt::SectorReader for Disk {
    fn read_sector(&self, lba: u32, out: &mut [u8; 512]) -> Result<(), gpt::GptError> {
        *out = *self.0.get(&lba).unwrap_or(&[0; 512]);
        Ok(())
    }
}
impl legacy_gpt::SectorReader for Disk {
    fn read_sector(&self, lba: u32, out: &mut [u8; 512]) -> Result<(), legacy_gpt::GptError> {
        *out = *self.0.get(&lba).unwrap_or(&[0; 512]);
        Ok(())
    }
}

fn put(b: &mut [u8], o: usize, v: &[u8]) {
    b[o..o + v.len()].copy_from_slice(v);
}

const NAMES: [&str; 4] = ["boot1", "rootfs", "boot", "esp"];

/// A valid GPT on a disk of `cap` sectors with up to 6 entries.
fn disk(r: &mut Rng, cap: u64) -> Disk {
    let count = [4u32, 8, 16, 128][r.below(4) as usize];
    let size = 128u32;
    let table = 2u64;
    let bytes = (count * size) as u64;
    let first = table + bytes.div_ceil(512);
    let last = cap - 1 - bytes.div_ceil(512) - 1;
    let mut entries = vec![0u8; bytes as usize];
    let mut lba = first;
    for i in 0..r.below(7).min(count as u64) as usize {
        let len = 1 + r.below(64);
        if lba + len > last {
            break;
        }
        let e = &mut entries[i * 128..i * 128 + 128];
        if r.below(3) == 0 {
            put(e, 0, &storage::gpt::TYPE_FIRMWARE);
        } else {
            put(e, 0, &[0x11; 16]);
        }
        put(e, 32, &lba.to_le_bytes());
        put(e, 40, &(lba + len - 1).to_le_bytes());
        let name = NAMES[r.below(4) as usize];
        for (k, c) in name.encode_utf16().enumerate() {
            put(e, 56 + 2 * k, &c.to_le_bytes());
        }
        lba += len + r.below(4);
    }
    let mut h = [0u8; 512];
    put(&mut h, 0, b"EFI PART");
    put(&mut h, 8, &0x10000u32.to_le_bytes());
    put(&mut h, 12, &92u32.to_le_bytes());
    put(&mut h, 24, &1u64.to_le_bytes());
    put(&mut h, 32, &(cap - 1).to_le_bytes());
    put(&mut h, 40, &first.to_le_bytes());
    put(&mut h, 48, &last.to_le_bytes());
    put(&mut h, 72, &table.to_le_bytes());
    put(&mut h, 80, &count.to_le_bytes());
    put(&mut h, 84, &size.to_le_bytes());
    put(&mut h, 88, &(!gpt::crc_update(!0, &entries)).to_le_bytes());
    let crc = !gpt::crc_update(!0, &h[..92]);
    put(&mut h, 16, &crc.to_le_bytes());
    let mut d = HashMap::new();
    d.insert(1, h);
    for (k, chunk) in entries.chunks(512).enumerate() {
        let mut s = [0u8; 512];
        s[..chunk.len()].copy_from_slice(chunk);
        d.insert(2 + k as u32, s);
    }
    Disk(d)
}

fn same(d: &Disk, name: &str, cap: u64) -> bool {
    let a = gpt::find(d, name, cap);
    let l = legacy_gpt::find(d, name, cap);
    let l = l
        .map(|o| {
            o.map(|e| gpt::PartitionExtent {
                first_lba: e.first_lba,
                sectors: e.sectors,
            })
        })
        .map_err(|e| format!("{e:?}"));
    assert_eq!(a.map_err(|e| format!("{e:?}")), l, "name {name} cap {cap}");
    matches!(gpt::find(d, name, cap), Ok(Some(_)))
}

#[test]
fn gpt_matches_the_board_code() {
    let mut r = Rng(0x6e70_7400_0000_0001);
    let mut found = 0;
    for _ in 0..2000 {
        let cap = 300 + r.below(5000);
        let mut d = disk(&mut r, cap);
        for name in NAMES
            .iter()
            .chain(["", "a-very-long-partition-name-over-36-characters"].iter())
        {
            found += same(&d, name, cap) as u32;
            let _ = same(&d, name, cap + 1);
        }
        // Mutate one byte of the header or the first table sector.
        let lba = 1 + r.below(2) as u32;
        let mut s = *d.0.get(&lba).unwrap_or(&[0; 512]);
        let i = r.below(512) as usize;
        s[i] ^= 1 << r.below(8);
        d.0.insert(lba, s);
        let _ = same(&d, "boot1", cap);
    }
    assert!(found > 1000, "only {found} lookups found a partition");
}

#[test]
fn sd_capacity_matches_the_board_code() {
    let mut r = Rng(0x5d00_c5d0_0000_0001);
    for i in 0..200_000 {
        let mut w = [
            r.next() as u32,
            r.next() as u32,
            r.next() as u32,
            r.next() as u32,
        ];
        // Bias the structure bits so both CSD versions are common.
        if i % 2 == 0 {
            w[3] = (w[3] & 0x3fff_ffff) | ((r.below(2) as u32) << 30) >> 8 << 8;
        }
        for ba in [false, true] {
            assert_eq!(sd::sectors(w, ba), legacy_sd::sectors(w, ba), "{w:x?} {ba}");
        }
    }
}
