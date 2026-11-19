//! fastboot for the Cubie A7A bt0, the parts that know no hardware:
//!
//! - [`proto`]: the protocol loop over two seams, the USB device controller
//!   ([`proto::Transport`]) and the board ([`proto::Device`]);
//! - [`bootimg`]: the Android boot image wrapper `fastboot boot` puts around
//!   a file that is not one;
//! - [`bundle`]: a downloaded copy of the boot medium (the layout table and
//!   the images it names) for `fastboot boot`;
//! - [`plan`]: what `boot` checks in a bundle before it answers OKAY
//!   (the board's layout table contract from `cubie-a7a-common`);
//! - [`nor`]: the SPI NOR partitions and the guards every NOR erase and
//!   program passes, over an abstract flash ([`nor::NorFlash`]);
//! - [`usb`]: chapter 9 descriptors and requests of the fastboot function.
//!
//! The structure follows the shared `fastboot` crate of the SpacemiT K1 bt0
//! (kept outside this tree:
//! [`proto::Transport`]/[`proto::Device`] seams, reply lines, terminal
//! download and hand-over errors), extended with `boot`, `oem`,
//! `reboot-bootloader` and the NOR guard. Everything here builds and is
//! tested on the host (`cargo test -p cubie-a7a-fastboot`).
#![no_std]
#[cfg(test)]
extern crate std;

pub mod bootimg;
pub mod bundle;
pub mod egon;
pub mod nor;
pub mod plan;
pub mod proto;
pub mod usb;

/// CRC-32 (IEEE 802.3, as zlib's `crc32`), bitwise and table-free (bt0's
/// SRAM is tight): the NOR arm token, over at most one partition.
pub fn crc32(data: &[u8]) -> u32 {
    let mut c = !0u32;
    for &b in data {
        c ^= b as u32;
        for _ in 0..8 {
            c = if c & 1 != 0 {
                (c >> 1) ^ 0xedb8_8320
            } else {
                c >> 1
            };
        }
    }
    !c
}

/// Little-endian u32 at `off`, or None past the end.
pub(crate) fn le32(b: &[u8], off: usize) -> Option<u32> {
    let s = b.get(off..off.checked_add(4)?)?;
    Some(u32::from_le_bytes([s[0], s[1], s[2], s[3]]))
}

#[cfg(test)]
mod tests {
    #[test]
    fn crc32_check_value() {
        assert_eq!(super::crc32(b"123456789"), 0xcbf4_3926);
        assert_eq!(super::crc32(b""), 0);
    }
    #[test]
    fn le32_bounds() {
        let b = [1, 2, 3, 4, 5];
        assert_eq!(super::le32(&b, 0), Some(0x0403_0201));
        assert_eq!(super::le32(&b, 1), Some(0x0504_0302));
        assert_eq!(super::le32(&b, 2), None);
        assert_eq!(super::le32(&b, usize::MAX), None);
    }
}
