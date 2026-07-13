// SPDX-License-Identifier: GPL-2.0-only
//! The crate with the A7A contract against the verbatim A7A board copy
//! (`legacy/a7a_layout.rs`): random valid and mutated tables through
//! decode (raw bytes, any CRC), validate and load_entries; same results,
//! errors and load calls. The host's usize is 64-bit, so the contract's
//! address limit is usize::MAX here (the AArch32 limit is a unit test).
mod common;
#[path = "legacy/a7a_layout.rs"]
mod legacy;

use bootlayout::{Entry, Table, MAX_ENTRIES, TABLE_SIZE};
use common::*;

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

fn random_table(r: &mut Rng) -> Table {
    let mut t = Table {
        magic: *b"A7ABOOT1",
        version: 1,
        count: 2,
        crc32: 0,
        _reserved: 0,
        entries: [Entry::default(); MAX_ENTRIES],
    };
    t.entries[0] = Entry {
        kind: KIND_MAIN,
        offset: 0x20_0000,
        size: 4096 + r.below(0x7_0000) as u32,
        load_addr: MAIN_BASE,
        ..Entry::default()
    };
    t.entries[1] = Entry {
        kind: KIND_PAYLOAD,
        offset: 0x30_0000,
        size: 1 + r.below(0x40_0000) as u32,
        load_addr: PAYLOAD_BASE,
        ..Entry::default()
    };
    if r.below(2) == 0 {
        t.entries[t.count as usize] = Entry {
            kind: KIND_DTB,
            offset: 0xc0_0000,
            size: 1 + r.below(0x1_0000) as u32,
            load_addr: PAYLOAD_RAM_BASE + r.below(0x40) * 0x1000,
            ..Entry::default()
        };
        t.count += 1;
    }
    if r.below(2) == 0 {
        t.entries[t.count as usize] = Entry {
            kind: KIND_VENDOR_SPL,
            offset: 0xe0_0000,
            size: 1 + r.below(0x4a000) as u32,
            load_addr: 0x47000,
            ..Entry::default()
        };
        t.count += 1;
    }
    // mutations
    for _ in 0..r.below(3) {
        let i = r.below(MAX_ENTRIES as u64) as usize;
        let e = &mut t.entries[i];
        match r.below(12) {
            0 => e.kind = r.below(7) as u32,
            1 => e.flags = r.below(2) as u32,
            2 => {
                e.offset = [0, TABLE_OFFSET, 0x30_0001, 0xffff_fe00, 0x20_0200][r.below(5) as usize]
            }
            3 => e.size = [0, 1, u32::MAX, u32::MAX - 510, 0x8_0001][r.below(5) as usize],
            4 => {
                e.load_addr = [
                    0,
                    1 << 32,
                    u64::MAX - 3,
                    HANDOFF_ADDR,
                    PAYLOAD_BASE + 4,
                    0x47000,
                ][r.below(6) as usize]
            }
            5 => e._reserved = 1,
            6 => t.count = r.below(MAX_ENTRIES as u64 + 2) as u32,
            7 => t._reserved = 1,
            8 => t.magic[r.below(8) as usize] ^= 1,
            9 => t.version = 2,
            10 => t.entries[i] = t.entries[0],
            _ => {}
        }
    }
    t
}

#[test]
fn tables_match_the_board_code() {
    let mut r = Rng(0x1234_5678_9abc_def1);
    let c = a7a(usize::MAX as u64);
    let mut loaded = 0;
    for i in 0..40_000 {
        let t = random_table(&mut r);
        let mut bytes = bootlayout::encode(&t).to_vec();
        if i % 7 == 0 {
            let at = r.below(TABLE_SIZE as u64) as usize;
            bytes[at] ^= 1 << r.below(8);
        }
        if i % 11 == 0 {
            bytes.truncate(r.below(TABLE_SIZE as u64) as usize);
        }
        let a = legacy::layout::decode(&bytes);
        let b = bootlayout::decode(&c, &bytes);
        assert_eq!(
            format!("{:?}", a.as_ref().map(|_| ())),
            format!("{:?}", b.as_ref().map(|_| ()))
        );
        let (Ok(ta), Ok(tb)) = (a, b) else { continue };
        assert_eq!(ta.crc32, tb.crc32);
        for dram_end in [0xc000_0000u64, PAYLOAD_BASE + 0x1000, 0x1_4000_0000] {
            let va = legacy::layout::validate(&ta, dram_end);
            let vb = bootlayout::validate(&c, &tb, dram_end);
            assert_eq!(format!("{va:?}"), format!("{vb:?}"));
            let mut ca = Vec::new();
            let mut cb = Vec::new();
            let la = legacy::layout::load_entries(&ta, dram_end, |e, a, p| ca.push((e.kind, a, p)));
            let lb = bootlayout::load_entries(&c, &tb, dram_end, |e, a, p| cb.push((e.kind, a, p)));
            assert_eq!(format!("{la:?}"), format!("{lb:?}"));
            assert_eq!(ca, cb);
            loaded += cb.len();
        }
    }
    assert!(loaded > 10_000, "too few loads ({loaded})");
}

#[test]
fn crate_encoding_matches_the_raw_table_bytes() {
    let mut r = Rng(5);
    for _ in 0..2000 {
        let t = random_table(&mut r);
        let enc = bootlayout::encode(&t);
        let mut raw = [0u8; TABLE_SIZE];
        // SAFETY: repr(C) integer frame, same as the board's read_unaligned.
        unsafe {
            core::ptr::copy_nonoverlapping(
                (&t as *const Table).cast::<u8>(),
                raw.as_mut_ptr(),
                TABLE_SIZE,
            )
        };
        raw[16..20].fill(0);
        let crc = legacy::crc32(&raw);
        raw[16..20].copy_from_slice(&crc.to_le_bytes());
        assert_eq!(enc, raw);
    }
}
