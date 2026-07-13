// SPDX-License-Identifier: GPL-2.0-only
//! netid against the verbatim A733 ethernet.rs (its own known-ID test runs
//! through the copy).

mod sha256 {
    pub fn digest(d: &[u8]) -> [u8; 32] {
        ::sha256::digest(d)
    }
}

#[allow(dead_code, clippy::all)]
mod a7a {
    include!("legacy/a7a_ethernet.rs");
}

#[test]
fn matches_on_random_ids() {
    let mut x: u64 = 0x5eed_0e7e;
    for _ in 0..50_000 {
        let mut id = [0u32; 4];
        for w in id.iter_mut() {
            x ^= x << 13;
            x ^= x >> 7;
            x ^= x << 17;
            *w = x as u32;
        }
        let mac = netid::mac_from_chip_id(id);
        assert_eq!(mac, a7a::mac_from_chip_id(id));
        assert_eq!(mac[0] & 0x03, 0x02, "unicast, locally administered");
    }
}

#[test]
fn other_lengths() {
    let a = netid::mac_from_id_words([1u32, 2]);
    let b = netid::mac_from_id_words([1u32, 2, 0]);
    assert_ne!(a, b, "the length is part of the hash input");
    assert_eq!(netid::local_unicast([0xff; 6])[0], 0xfe);
}
