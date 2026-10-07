//! Board Ethernet MAC address.
//!
//! The A7A's EEPROM carries no MAC address, so one is derived from the
//! 128-bit SID chip ID (efuse SRAM mirror at 0x0300_6200): the first six
//! bytes of SHA-256 over the four ID words in little-endian order, with the
//! multicast bit cleared and the locally-administered bit set. The same
//! board always gets the same address; it is not an OUI address.

use crate::sha256;

/// SID chip ID, four 32-bit words.
pub const SID_CHIP_ID: usize = 0x0300_6200;

pub fn mac_from_chip_id(id: [u32; 4]) -> [u8; 6] {
    let mut bytes = [0u8; 16];
    for (b, w) in bytes.chunks_exact_mut(4).zip(id) {
        b.copy_from_slice(&w.to_le_bytes());
    }
    let h = sha256::digest(&bytes);
    let mut mac = [h[0], h[1], h[2], h[3], h[4], h[5]];
    mac[0] = (mac[0] & 0xfe) | 0x02;
    mac
}

#[cfg(test)]
mod tests {
    #[test]
    fn known_id() {
        // Python: h = sha256(struct.pack('<4I', *id)).digest()[:6];
        //         h[0] = (h[0] & 0xfe) | 2
        let id = [0x1234_5678, 0x9abc_def0, 0x0fed_cba9, 0x8765_4321];
        assert_eq!(
            super::mac_from_chip_id(id),
            [0x3e, 0x4b, 0xba, 0xfe, 0x9c, 0x32]
        );
    }
}
