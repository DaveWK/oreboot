//! Board Ethernet MAC address.
//!
//! The A7A's EEPROM carries no MAC address, so one is derived from the
//! 128-bit SID chip ID (efuse SRAM mirror at 0x0300_6200): the first six
//! bytes of SHA-256 over the four ID words in little-endian order, with the
//! multicast bit cleared and the locally-administered bit set (shared
//! `netid::mac_from_chip_id`). The same board always gets the same address;
//! it is not an OUI address.

/// SID chip ID, four 32-bit words.
pub const SID_CHIP_ID: usize = 0x0300_6200;

pub use netid::mac_from_chip_id;

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
