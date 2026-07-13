// SPDX-License-Identifier: GPL-2.0-only
//! The A7A board's own layout controls (common/src/lib.rs `load_controls`,
//! ore-edk-boot-radxa-a7a ed42550), ported to the crate with the A7A
//! contract, plus contract parameters the board code had as constants.
mod common;

use bootlayout::{
    decode, encode, load_entries, validate, Entry, Error, Table, CRC_FIELD_OFFSET, MAX_ENTRIES,
    TABLE_SIZE,
};
use common::*;

const END: u64 = 0xc000_0000;

fn c() -> bootlayout::Contract<'static> {
    a7a(usize::MAX as u64)
}

#[test]
fn frame_crc_length_and_unaligned_access() {
    let mut bytes = [0u8; TABLE_SIZE + 1];
    let frame = &mut bytes[1..];
    frame[..8].copy_from_slice(b"A7ABOOT1");
    frame[8..12].copy_from_slice(&1u32.to_le_bytes());
    let checksum = storage::crc::crc32(frame);
    frame[CRC_FIELD_OFFSET..CRC_FIELD_OFFSET + 4].copy_from_slice(&checksum.to_le_bytes());
    assert_eq!(decode(&c(), frame).unwrap().count, 0);
    assert!(matches!(
        decode(&c(), &frame[..TABLE_SIZE - 1]),
        Err(Error::Header)
    ));
    frame[24] ^= 1;
    assert!(matches!(decode(&c(), frame), Err(Error::Checksum)));
}
fn table() -> Table {
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
        offset: 0x200000,
        size: 4096,
        load_addr: MAIN_BASE,
        ..Entry::default()
    };
    t.entries[1] = Entry {
        kind: KIND_PAYLOAD,
        offset: 0x300000,
        size: 1024,
        load_addr: PAYLOAD_BASE,
        ..Entry::default()
    };
    t
}
fn rejected_before_load(t: &Table) {
    let mut calls = 0;
    assert!(load_entries(&c(), t, END, |_, _, _| calls += 1).is_err());
    assert_eq!(
        calls, 0,
        "malformed later entry must not follow an earlier copy"
    );
}
#[test]
fn linked_stages_and_optional_recovery() {
    let mut t = table();
    t.count = 3;
    t.entries[2] = Entry {
        kind: KIND_VENDOR_SPL,
        offset: 0x780000,
        size: 8192,
        load_addr: 0x47000,
        ..Entry::default()
    };
    let mut calls = 0;
    load_entries(&c(), &t, END, |e, addr, padded| {
        assert_ne!(e.kind, KIND_VENDOR_SPL);
        assert_eq!(addr as u64, e.load_addr);
        assert_eq!(padded as u64, u64::from(e.size));
        calls += 1;
    })
    .unwrap();
    assert_eq!(calls, 2);
}
#[test]
fn full_address_and_large_padding_never_alias_or_wrap() {
    let mut t = table();
    t.entries[1].load_addr += 1 << 32;
    rejected_before_load(&t);
    for size in [0, u32::MAX - 510, u32::MAX] {
        let mut t = table();
        t.entries[0].size = size;
        rejected_before_load(&t);
    }
    let mut t = table();
    t.entries[1].load_addr = u64::MAX - 3;
    rejected_before_load(&t);
}
#[test]
fn padded_transfer_extent_must_fit_dram() {
    let mut t = table();
    t.entries[1].size = 1;
    let end = t.entries[1].load_addr + 512;
    assert_eq!(validate(&c(), &t, end), Ok(()));
    let mut calls = 0;
    assert!(load_entries(&c(), &t, end - 1, |_, _, _| calls += 1).is_err());
    assert_eq!(calls, 0);
}
#[test]
fn media_and_memory_overlap_fail_before_first_copy() {
    let mut t = table();
    t.entries[1].offset = t.entries[0].offset + 512;
    rejected_before_load(&t);
    let mut t = table();
    t.count = 3;
    t.entries[2] = Entry {
        kind: KIND_DTB,
        offset: 0x800000,
        size: 1,
        load_addr: PAYLOAD_BASE + 512,
        ..Entry::default()
    };
    rejected_before_load(&t);
    t.entries[2].load_addr += 512;
    assert_eq!(validate(&c(), &t, END), Ok(())); // touching boundary allowed
    t.entries[2].load_addr = HANDOFF_ADDR;
    rejected_before_load(&t);
}
#[test]
fn duplicates_missing_and_unknown_stages_rejected() {
    let mut t = table();
    t.count = 1;
    rejected_before_load(&t);
    let mut t = table();
    t.count = 3;
    t.entries[2] = t.entries[0];
    rejected_before_load(&t);
    let mut t = table();
    t.count = 4;
    t.entries[2] = Entry {
        kind: KIND_DTB,
        offset: 0x800000,
        size: 512,
        load_addr: 0x48000000,
        ..Entry::default()
    };
    t.entries[3] = Entry {
        kind: KIND_DTB,
        offset: 0x900000,
        size: 512,
        load_addr: 0x49000000,
        ..Entry::default()
    };
    rejected_before_load(&t); // distinct extents: duplicate-kind gate matters
    for kind in [0, 5, u32::MAX] {
        let mut t = table();
        t.entries[1].kind = kind;
        rejected_before_load(&t);
    }
}
#[test]
fn version1_header_and_unknown_fields_rejected() {
    let mut t = table();
    t.magic[0] ^= 1;
    rejected_before_load(&t);
    let mut t = table();
    t.version += 1;
    rejected_before_load(&t);
    let mut t = table();
    t.count = MAX_ENTRIES as u32 + 1;
    rejected_before_load(&t);
    let mut t = table();
    t._reserved = 1;
    rejected_before_load(&t);
    let mut t = table();
    t.entries[1].flags = 1;
    rejected_before_load(&t);
    let mut t = table();
    t.entries[1]._reserved = 1;
    rejected_before_load(&t);
}
#[test]
fn media_gap_and_linker_address_contract() {
    for offset in [0, TABLE_OFFSET, 0x300001, 0xfffffe00] {
        let mut t = table();
        t.entries[1].offset = offset;
        rejected_before_load(&t);
    }
    let mut t = table();
    t.entries[0].size = MAIN_MAX_SIZE as u32 + 1;
    rejected_before_load(&t);
    let mut t = table();
    t.entries[0].load_addr += 4;
    rejected_before_load(&t);
    let mut t = table();
    t.entries[1].load_addr += 4;
    rejected_before_load(&t);
    let mut t = table();
    t.entries[1].size = (IMAGE_END - u64::from(t.entries[1].offset)) as u32;
    assert_eq!(validate(&c(), &t, END), Ok(()));
    t.entries[1].size += 1;
    rejected_before_load(&t);
}

// Beyond the board's controls: parameters it had as constants.

#[test]
fn aarch32_loader_address_limit() {
    // The A7A bt0 is AArch32: usize::try_from fails above 4 GiB there.
    let c32 = a7a(u32::MAX as u64);
    let mut t = table();
    assert_eq!(validate(&c32, &t, 0x1_4000_0000), Ok(()));
    t.count = 3;
    t.entries[2] = Entry {
        kind: KIND_DTB,
        offset: 0x800000,
        size: 512,
        load_addr: 0x1_0000_0000,
        ..Entry::default()
    };
    assert_eq!(validate(&c32, &t, 0x1_4000_0000), Err(Error::MemoryRange));
    assert_eq!(validate(&c(), &t, 0x1_4000_0000), Ok(()));
}

#[test]
fn another_board_contract() {
    use bootlayout::{Contract, KindRule, Place};
    const KINDS: [KindRule; 2] = [
        KindRule {
            kind: 1,
            place: Place::Exact {
                addr: 0x8000_0000,
                max_size: None,
            },
        },
        KindRule {
            kind: 7,
            place: Place::AtLeast(0x8100_0000),
        },
    ];
    let c = Contract {
        magic: *b"OTHER001",
        version: 3,
        image_start: 0x10_0000,
        image_end: 0x80_0000,
        block_size: 4096,
        dram_base: 0x8000_0000,
        max_address: u64::MAX,
        kinds: &KINDS,
        required: &[1],
    };
    let mut t = Table {
        magic: *b"OTHER001",
        version: 3,
        count: 2,
        crc32: 0,
        _reserved: 0,
        entries: [Entry::default(); MAX_ENTRIES],
    };
    t.entries[0] = Entry {
        kind: 1,
        offset: 0x10_0000,
        size: 5000,
        load_addr: 0x8000_0000,
        ..Entry::default()
    };
    t.entries[1] = Entry {
        kind: 7,
        offset: 0x10_2000,
        size: 1,
        load_addr: 0x8100_0000,
        ..Entry::default()
    };
    let bytes = encode(&t);
    let d = decode(&c, &bytes).unwrap();
    let mut calls = Vec::new();
    load_entries(&c, &d, 0x9000_0000, |e, a, p| calls.push((e.kind, a, p))).unwrap();
    assert_eq!(calls, [(1, 0x8000_0000, 8192), (7, 0x8100_0000, 4096)]);
    // the A7A contract rejects this table outright
    assert_eq!(decode(&a7a(u64::MAX), &bytes), Err(Error::Header));
}

#[test]
fn a_table_written_by_tools_bootlayout_py() {
    // tests/fixtures/a7a-table.bin: `tools/bootlayout.py write --magic
    // A7ABOOT1 --version 1` with main, payload and DTB entries.
    let bytes = include_bytes!("fixtures/a7a-table.bin");
    let t = decode(&c(), bytes).unwrap();
    assert_eq!(t.count, 3);
    assert_eq!(encode(&t).as_slice(), bytes.as_slice());
    let mut calls = Vec::new();
    load_entries(&c(), &t, END, |e, a, p| calls.push((e.kind, a, p))).unwrap();
    assert_eq!(
        calls,
        [
            (KIND_MAIN, MAIN_BASE as usize, 512),
            (KIND_PAYLOAD, PAYLOAD_BASE as usize, 3072),
            (KIND_DTB, PAYLOAD_RAM_BASE as usize, 512)
        ]
    );
}
