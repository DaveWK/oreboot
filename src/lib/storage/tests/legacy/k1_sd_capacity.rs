// SPDX-License-Identifier: GPL-2.0-only
// Verbatim copy of ore-edk-boot-opi c91bbc4e40
// oreboot/src/mainboard/spacemit/k1x/bt0/src/sd_capacity.rs, the oracle for the
// equivalence tests (its own unit tests removed).
#![allow(dead_code, clippy::all)]
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
