// SPDX-License-Identifier: GPL-2.0-only
//! A stable, locally administered Ethernet MAC address from a chip ID, for
//! boards without a programmed address.
//!
//! Moved from ore-edk-boot-radxa-a7a ed42550 (oreboot a803f4f780d7)
//! `src/mainboard/radxa/cubie-a7a/common/src/ethernet.rs`: the first six
//! bytes of SHA-256 over the ID words in little-endian order, multicast bit
//! cleared, locally administered bit set. The same chip always gets the same
//! address; it is not an OUI address. Where the ID lives (the A733 SID at
//! 0x0300_6200) stays in the board.
#![no_std]

/// Clear the multicast bit and set the locally administered bit.
pub const fn local_unicast(mut mac: [u8; 6]) -> [u8; 6] {
    mac[0] = (mac[0] & 0xfe) | 0x02;
    mac
}

/// The MAC address for an ID of any number of 32-bit words.
pub fn mac_from_id_words<const N: usize>(id: [u32; N]) -> [u8; 6] {
    // Up to 16 words (64 bytes), more than any chip ID in use.
    assert!(N <= 16, "chip ID longer than 16 words");
    let mut bytes = [0u8; 64];
    for (i, w) in id.iter().enumerate() {
        bytes[4 * i..4 * i + 4].copy_from_slice(&w.to_le_bytes());
    }
    let h = sha256::digest(&bytes[..4 * N]);
    local_unicast([h[0], h[1], h[2], h[3], h[4], h[5]])
}

/// The MAC address for a 128-bit (four-word) chip ID such as the A733 SID.
pub fn mac_from_chip_id(id: [u32; 4]) -> [u8; 6] {
    mac_from_id_words(id)
}
