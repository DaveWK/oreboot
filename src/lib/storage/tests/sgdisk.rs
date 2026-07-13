// SPDX-License-Identifier: GPL-2.0-only
//! An independent GPT producer: the K1 media control (ore-edk-boot-opi
//! 78479e21ba tests/run-media.py + tests/gpt-file.rs) on the crate.
//!
//! tests/fixtures/sgdisk-32mib-gpt.bin holds the primary (LBA 0-33) and
//! backup (last 33 sectors) GPT of a 32 MiB image made by GPT fdisk 1.0.10:
//!   truncate -s 32M sd.img
//!   sgdisk --clear --new=1:8192:16383 \
//!     --typecode=1:2E54B353-1271-4842-806F-E436D6AF6985 \
//!     --change-name=1:firmware sd.img
//! (all other sectors are zero). The test rebuilds the image in memory and
//! checks the name and type lookups, then corrupts it the same three ways.

use storage::gpt::{find, GptError, PartitionExtent, SectorReader};

const MIB32: usize = 32 << 20;
struct Disk(Vec<u8>);
impl SectorReader for Disk {
    fn read_sector(&self, lba: u32, out: &mut [u8; 512]) -> Result<(), GptError> {
        let a = lba as usize * 512;
        let s = self.0.get(a..a + 512).ok_or(GptError::Read)?;
        out.copy_from_slice(s);
        Ok(())
    }
}

fn image() -> Disk {
    let f = include_bytes!("fixtures/sgdisk-32mib-gpt.bin");
    assert_eq!(f.len(), (34 + 33) * 512);
    let mut d = vec![0u8; MIB32];
    d[..34 * 512].copy_from_slice(&f[..34 * 512]);
    d[MIB32 - 33 * 512..].copy_from_slice(&f[34 * 512..]);
    Disk(d)
}

const CAP: u64 = (MIB32 / 512) as u64;
const PART: PartitionExtent = PartitionExtent {
    first_lba: 8192,
    sectors: 8192,
};

#[test]
fn sgdisk_partition_by_name_and_by_type() {
    let d = image();
    assert_eq!(find(&d, "firmware", CAP), Ok(Some(PART)));
    // An unknown name falls back to the firmware partition type.
    assert_eq!(find(&d, "type-fallback", CAP), Ok(Some(PART)));
}

#[test]
fn sgdisk_header_crc_corruption_is_an_error() {
    let mut d = image();
    d.0[512 + 16..512 + 20].fill(0);
    assert!(find(&d, "firmware", CAP).is_err());
}

#[test]
fn sgdisk_missing_signature_is_an_error() {
    let mut d = image();
    d.0[512 + 16..512 + 20].fill(0);
    d.0[512..520].fill(0);
    assert!(find(&d, "firmware", CAP).is_err());
}

#[test]
fn sgdisk_without_protective_mbr_is_absent() {
    let mut d = image();
    d.0[512 + 16..512 + 20].fill(0);
    d.0[512..520].fill(0);
    d.0[..512].fill(0);
    assert_eq!(find(&d, "firmware", CAP), Ok(None));
}
