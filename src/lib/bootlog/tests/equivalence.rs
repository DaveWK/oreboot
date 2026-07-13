// SPDX-License-Identifier: GPL-2.0-only
//! The crate against the A7A board code (tests/legacy/a7a_fwlog.rs) on real
//! host memory: random sequences of init/putc on one 256 KiB area through the
//! board functions (raw addresses) and through bootlog::ring with the A7A
//! format; the two areas must stay byte-identical and agree on validity.
#[path = "legacy/a7a_fwlog.rs"]
mod legacy;

// stub: the A7A memory map the legacy module's const asserts use.
pub mod mem {
    pub const DTB_ADDR: usize = 0x4010_0000;
    pub const DTB_MAX_SIZE: usize = 0x0004_0000;
    pub const HANDOFF_ADDR: usize = 0x401c_0000;
}

use bootlog::{ring, Area};

struct Words(Vec<u32>);
impl Area for Words {
    fn read8(&self, o: usize) -> u8 {
        self.0[o / 4].to_le_bytes()[o % 4]
    }
    fn write8(&mut self, o: usize, v: u8) {
        let mut b = self.0[o / 4].to_le_bytes();
        b[o % 4] = v;
        self.0[o / 4] = u32::from_le_bytes(b);
    }
    fn read32(&self, o: usize) -> u32 {
        assert_eq!(o % 4, 0);
        self.0[o / 4]
    }
    fn write32(&mut self, o: usize, v: u32) {
        assert_eq!(o % 4, 0);
        self.0[o / 4] = v;
    }
}

#[test]
fn ring_operations_match_the_board_code() {
    assert_eq!(ring::A7A.magic, legacy::MAGIC);
    assert_eq!(ring::A7A.data_size, legacy::DATA_SIZE);
    assert_eq!(
        (ring::HEADER_SIZE, ring::VERSION),
        (legacy::HEADER_SIZE, legacy::VERSION)
    );
    let words = legacy::AREA_SIZE / 4;
    let mut x: u64 = 0xf00d_0000_0000_0001;
    for round in 0..40 {
        let mut old = vec![0u32; words];
        let mut new = Words(vec![0u32; words]);
        // Random initial contents half the time (a stale or foreign area).
        if round % 2 == 1 {
            for (i, w) in old.iter_mut().enumerate() {
                x ^= x << 13;
                x ^= x >> 7;
                x ^= x << 17;
                *w = x as u32;
                new.0[i] = x as u32;
            }
        }
        let base = old.as_mut_ptr() as usize;
        for _ in 0..2000 {
            x ^= x << 13;
            x ^= x >> 7;
            x ^= x << 17;
            match x % 50 {
                0 => {
                    let seq = (x >> 8) as u32;
                    legacy::init_header(base, seq);
                    ring::init_header(&mut new, &ring::A7A, seq);
                }
                1 => {
                    // A corrupted header byte.
                    let o = ((x >> 8) % 32) as usize;
                    let v = (x >> 20) as u8;
                    unsafe { *(base as *mut u8).add(o) = v };
                    new.write8(o, v);
                }
                _ => {
                    if legacy::header_valid(base) {
                        legacy::ring_putc(base, (x >> 8) as u8);
                        ring::putc(&mut new, &ring::A7A, (x >> 8) as u8);
                    }
                }
            }
            assert_eq!(
                legacy::header_valid(base),
                ring::header_valid(&new, &ring::A7A)
            );
        }
        assert!(old == new.0, "areas differ in round {round}");
    }
}
