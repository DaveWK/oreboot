// SPDX-License-Identifier: GPL-2.0-only
//! SD Physical Layer CSD v1/v2 capacity; input is SDHCI's four raw R2 registers.
//! SDHCI drops the CRC/end byte: canonical CSD bits127:8 are response bits119:0.
//! SDUC/extended addressing is not supported by this 32-bit sector driver.
pub fn sectors(response: [u32; 4], block_addressing: bool) -> Option<u64> {
    let csd = response
        .iter()
        .enumerate()
        .fold(0u128, |v, (i, w)| v | ((*w as u128) << (8 + 32 * i)));
    match csd >> 126 {
        0 if !block_addressing => {
            let read_bl_len = ((csd >> 80) & 15) as u32;
            if !(9..=11).contains(&read_bl_len) {
                return None;
            }
            let size = ((csd >> 62) & 0xfff) as u64 + 1;
            let mult = ((csd >> 47) & 7) as u32 + 2;
            let sectors = size.checked_shl(mult + read_bl_len - 9)?;
            (sectors > 0 && sectors <= 1 << 23).then_some(sectors) // byte-addressed CMD argument
        }
        1 if block_addressing => Some((((csd >> 48) & 0x3fffff) as u64 + 1) * 1024),
        _ => None,
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[allow(unused_imports)]
    use std::{vec, vec::Vec};
    fn raw(c: u128) -> [u32; 4] {
        [
            (c >> 8) as u32,
            (c >> 40) as u32,
            (c >> 72) as u32,
            (c >> 104) as u32,
        ]
    }
    #[test]
    fn sdhc_32g() {
        assert_eq!(
            sectors(raw((1 << 126) | (65535 << 48)), true),
            Some(67108864)
        );
    }
    #[test]
    fn sdxc_max() {
        assert_eq!(
            sectors(raw((1 << 126) | (0x3fffff << 48)), true),
            Some(1 << 32)
        );
    }
    #[test]
    fn sdsc_1g() {
        assert_eq!(
            sectors(raw((9 << 80) | (4095 << 62) | (7 << 47)), false),
            Some(2097152)
        );
    }
    #[test]
    fn linux_sdhci_mapping_independent_literal() {
        // Linux sdhci_read_rsp_136 reverses registers then shifts each word8.
        // Literal canonical CSD400e00325b590000edc87f800a404000: C_SIZE60872.
        // This is a source-derived synthetic literal, not a physical read.
        let raw = [0x800a4040, 0x00edc87f, 0x325b5900, 0x00400e00];
        let mut words = [raw[3], raw[2], raw[1], raw[0]];
        for i in 0..4 {
            words[i] <<= 8;
            if i != 3 {
                words[i] |= words[i + 1] >> 24;
            }
        }
        assert_eq!(words, [0x400e0032, 0x5b590000, 0xedc87f80, 0x0a404000]);
        let size = ((words[1] & 0x3f) as u64) << 16 | ((words[2] >> 16) as u64);
        assert_eq!(size, 60872);
        assert_eq!(sectors(raw, true), Some(62333952));
    }
    #[test]
    fn invalid_structure_or_addressing() {
        for structure in 0..4 {
            let c = raw((structure << 126) | (9 << 80));
            assert!(sectors(c, structure != 1).is_none());
        }
    }
    #[test]
    fn invalid_block_length() {
        for len in [0, 8, 12, 15] {
            assert!(sectors(raw(len << 80), false).is_none());
        }
    }
}
