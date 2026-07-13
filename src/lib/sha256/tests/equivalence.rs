// SPDX-License-Identifier: GPL-2.0-only
//! The crate against both board copies, on every length 0..=600 and random
//! longer inputs.
#[path = "legacy/a7a_sha256.rs"]
mod a7a;
#[path = "legacy/k1_sha256.rs"]
mod k1;

#[test]
fn matches_both_board_copies() {
    let mut x: u64 = 0x5a5a_0123_4567_89ab;
    let mut data = Vec::new();
    for len in (0..=600).chain([4095, 4096, 4097, 65_536, 100_003]) {
        data.clear();
        for _ in 0..len {
            x ^= x << 13;
            x ^= x >> 7;
            x ^= x << 17;
            data.push(x as u8);
        }
        let d = sha256::digest(&data);
        assert_eq!(d, k1::digest(&data), "K1, length {len}");
        assert_eq!(d, a7a::digest(&data), "A7A, length {len}");
    }
}
