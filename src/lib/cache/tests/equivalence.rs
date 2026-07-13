// SPDX-License-Identifier: GPL-2.0-only
//! The block walk against the K1 cache.rs loop (tests/legacy/k1_cache.rs.txt
//! is the verbatim file; its RISC-V asm cannot run on the host, so its loop
//! is written out here unchanged, with the instruction replaced by a push).

use cache::{blocks, zicbom};

fn legacy_walk(addr: usize, len: usize) -> Vec<usize> {
    const BLOCK: usize = 64;
    let mut out = Vec::new();
    let mut a = addr & !(BLOCK - 1);
    while a < addr + len {
        out.push(a);
        a += BLOCK;
    }
    out
}

#[test]
fn walk_matches_the_k1_loop() {
    let mut x: u64 = 0x5eed_ca5e;
    for _ in 0..200_000 {
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        let addr = (x >> 20) as usize & 0xffff_ffff;
        let len = (x & 0x3fff) as usize;
        let ours: Vec<usize> = blocks(addr, len, zicbom::BLOCK).collect();
        assert_eq!(ours, legacy_walk(addr, len), "addr {addr:#x} len {len}");
    }
}

#[test]
fn verbatim_copy_is_present() {
    let text = include_str!("legacy/k1_cache.rs.txt");
    assert!(text.contains("while a < addr + len"));
    assert!(text.contains(".insn i 0x0f, 2, x0, {0}, 2"));
    assert!(text.contains(".insn i 0x0f, 2, x0, {0}, 0"));
}

#[test]
fn edges() {
    assert_eq!(blocks(0x1000, 0, 64).count(), 0);
    assert_eq!(blocks(0x1001, 1, 64).collect::<Vec<_>>(), [0x1000]);
    assert_eq!(blocks(0x103f, 2, 64).collect::<Vec<_>>(), [0x1000, 0x1040]);
    zicbom::flush(0x1000, 128);
    zicbom::invalidate(0x1000, 128);
}
