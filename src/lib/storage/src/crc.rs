// SPDX-License-Identifier: GPL-2.0-only
//! CRC-32 (IEEE 802.3, reflected polynomial 0xEDB88320), table driven.
//!
//! From ore-edk-boot-radxa-a7a ed42550 (oreboot a803f4f780d7)
//! `src/mainboard/radxa/cubie-a7a/common/src/lib.rs` `crc32`,
//! `crc32_update`. [`update`] takes and returns the finished CRC, so calls
//! chain: `update(update(0, a), b) == crc32(ab)`. The GPT code's raw,
//! uninverted bitwise form is [`crate::gpt::crc_update`].

/// CRC-32 of `data`.
pub fn crc32(data: &[u8]) -> u32 {
    update(0, data)
}

/// Continue the finished CRC `crc` over `data`.
pub fn update(crc: u32, data: &[u8]) -> u32 {
    let mut c = !crc;
    for &b in data {
        c = TABLE[((c ^ b as u32) & 0xff) as usize] ^ (c >> 8);
    }
    !c
}

const TABLE: [u32; 256] = {
    let mut t = [0u32; 256];
    let mut i = 0;
    while i < 256 {
        let mut c = i as u32;
        let mut k = 0;
        while k < 8 {
            c = if c & 1 != 0 {
                0xedb8_8320 ^ (c >> 1)
            } else {
                c >> 1
            };
            k += 1;
        }
        t[i] = c;
        i += 1;
    }
    t
};
