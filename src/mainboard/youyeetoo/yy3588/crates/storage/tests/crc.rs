// SPDX-License-Identifier: GPL-2.0-only
//! storage::crc against the verbatim A733 crc32 and the GPT bitwise form.

use storage::{crc, gpt};

#[allow(dead_code, clippy::all)]
mod a7a {
    include!("legacy/a7a_crc32.rs");
}

#[test]
fn check_value() {
    assert_eq!(crc::crc32(b"123456789"), 0xcbf4_3926);
}

#[test]
fn matches_a733_and_gpt_forms_on_random_buffers() {
    let mut x: u64 = 0x5eed_c3c3;
    let mut buf = vec![0u8; 4096];
    for round in 0..3000 {
        for b in buf.iter_mut() {
            x ^= x << 13;
            x ^= x >> 7;
            x ^= x << 17;
            *b = x as u8;
        }
        let len = (x as usize) % buf.len();
        let split = len / 2 + round % 7;
        let d = &buf[..len];
        let want = a7a::crc32(d);
        assert_eq!(crc::crc32(d), want);
        let s = split.min(len);
        assert_eq!(crc::update(crc::update(0, &d[..s]), &d[s..]), want);
        assert_eq!(
            crc::update(round as u32, d),
            a7a::crc32_update(round as u32, d)
        );
        assert_eq!(!gpt::crc_update(!0, d), want, "GPT bitwise form");
    }
}
