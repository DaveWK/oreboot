// SPDX-License-Identifier: GPL-2.0-only
//! Rockchip RK35xx BootROM idblock, header version 2, built from our own
//! first stage (oreboot bt0). It replaces rkbin's boot_merger and the
//! rkbin DDR init + SPL pair that edk2-rk3588 packs.
//!
//! The layout is the BootROM's documented v2 header (U-Boot `mkimage -T
//! rksd`, tools/rkcommon.c, is the public reference and the test oracle):
//!
//! | Offset | Size | Field |
//! |---|---|---|
//! | 0 | 4 | magic `RKNS` (0x534E4B52, little endian) |
//! | 4 | 4 | reserved |
//! | 8 | 4 | `size_and_nimage`: [31:16] image count, [15:0] hash offset in words |
//! | 12 | 4 | `boot_flag`: [3:0] hash type, 1 = SHA-256 |
//! | 16 | 104 | reserved |
//! | 120 | 4 x 88 | image entries |
//! | 472 | 1064 | reserved |
//! | 1536 | 512 | header hash (SHA-256 of bytes 0..1536, then zeros) |
//!
//! An image entry is `size_and_off` ([31:16] size, [15:0] offset, both in
//! 512-byte blocks from the header), `address` (0xFFFFFFFF: BootROM
//! default), `flag`, `counter` (1-based), 8 reserved bytes and a 64-byte
//! hash field (SHA-256 in the first 32 bytes). Image 0 is the "init" image
//! the BootROM loads into SRAM; image 1 is the optional "boot" image used
//! only when init returns to the BootROM. Both are padded to 2048 bytes.
//! The header always describes two images, as U-Boot writes it.

/// Header size: 4 blocks of 512 bytes; the first image starts here.
pub const HEADER_SIZE: usize = 2048;
/// Image size alignment.
pub const ALIGN: usize = 2048;
/// `RKNS`.
pub const MAGIC_V2: u32 = 0x534E_4B52;
const HASH_OFFSET: usize = 1536;
const ENTRY_OFFSET: usize = 120;
const ENTRY_SIZE: usize = 88;
const BLOCK: usize = 512;

/// Largest init image per SoC: the SRAM the BootROM loads it into, less
/// the BootROM's own 4 KiB (U-Boot's spl_infos table).
pub fn max_init_size(soc: &str) -> Option<usize> {
    match soc {
        "rk3568" => Some(0x10000 - 0x1000),
        "rk3576" => Some(0x80000 - 0x1000),
        "rk3588" => Some(0x100000 - 0x1000),
        _ => None,
    }
}

#[derive(Debug, PartialEq, Eq)]
pub enum Error {
    UnknownSoc,
    EmptyInit,
    InitTooLarge { size: usize, max: usize },
    TooManyBlocks,
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Error::UnknownSoc => write!(f, "unknown SoC (rk3568, rk3576, rk3588)"),
            Error::EmptyInit => write!(f, "the init image is empty"),
            Error::InitTooLarge { size, max } => {
                write!(f, "init image is {size:#x} bytes, more than {max:#x}")
            }
            Error::TooManyBlocks => write!(f, "image too large for a 16-bit block count"),
        }
    }
}

fn padded(data: &[u8]) -> Vec<u8> {
    let mut v = data.to_vec();
    v.resize(data.len().div_ceil(ALIGN) * ALIGN, 0);
    v
}

fn put32(buf: &mut [u8], at: usize, v: u32) {
    buf[at..at + 4].copy_from_slice(&v.to_le_bytes());
}

/// The complete idblock: header, init image and optional boot image, each
/// padded to 2048 bytes. Written at sector 64 of an SD/eMMC device.
pub fn build(soc: &str, init: &[u8], boot: Option<&[u8]>) -> Result<Vec<u8>, Error> {
    let max = max_init_size(soc).ok_or(Error::UnknownSoc)?;
    if init.is_empty() {
        return Err(Error::EmptyInit);
    }
    let init = padded(init);
    if init.len() > max {
        return Err(Error::InitTooLarge {
            size: init.len(),
            max,
        });
    }
    let boot = boot.map(padded).unwrap_or_default();

    let mut out = vec![0u8; HEADER_SIZE];
    put32(&mut out, 0, MAGIC_V2);
    put32(&mut out, 8, (2 << 16) | (HASH_OFFSET / 4) as u32);
    put32(&mut out, 12, 1);

    let mut offset_blocks = HEADER_SIZE / BLOCK;
    for (i, image) in [&init, &boot].into_iter().enumerate() {
        let blocks = image.len() / BLOCK;
        if blocks > 0xffff || offset_blocks > 0xffff {
            return Err(Error::TooManyBlocks);
        }
        let e = ENTRY_OFFSET + i * ENTRY_SIZE;
        put32(&mut out, e, ((blocks as u32) << 16) | offset_blocks as u32);
        put32(&mut out, e + 4, 0xFFFF_FFFF);
        put32(&mut out, e + 12, i as u32 + 1);
        out[e + 24..e + 56].copy_from_slice(&sha256::digest(image));
        offset_blocks += blocks;
    }
    let header_hash = sha256::digest(&out[..HASH_OFFSET]);
    out[HASH_OFFSET..HASH_OFFSET + 32].copy_from_slice(&header_hash);

    out.extend_from_slice(&init);
    out.extend_from_slice(&boot);
    Ok(out)
}

/// What [`build`] wrote, read back and checked: magic, hash type, image
/// extents inside `blob` and every hash. Returns the (offset, length) in
/// bytes of the init and boot images.
pub fn verify(blob: &[u8]) -> Result<[(usize, usize); 2], String> {
    if blob.len() < HEADER_SIZE {
        return Err("shorter than the header".into());
    }
    let get = |at: usize| u32::from_le_bytes(blob[at..at + 4].try_into().unwrap());
    if get(0) != MAGIC_V2 {
        return Err(format!("magic {:#010x}, want {MAGIC_V2:#010x}", get(0)));
    }
    if get(12) & 0xf != 1 {
        return Err("hash type is not SHA-256".into());
    }
    let words = (get(8) & 0xffff) as usize;
    if words * 4 != HASH_OFFSET {
        return Err(format!("hash offset {} words", words));
    }
    if sha256::digest(&blob[..HASH_OFFSET]) != blob[HASH_OFFSET..HASH_OFFSET + 32] {
        return Err("header hash mismatch".into());
    }
    let mut extents = [(0, 0); 2];
    for (i, ext) in extents.iter_mut().enumerate() {
        let e = ENTRY_OFFSET + i * ENTRY_SIZE;
        let so = get(e);
        let (off, len) = (
            ((so & 0xffff) as usize) * BLOCK,
            ((so >> 16) as usize) * BLOCK,
        );
        let data = blob
            .get(off..off + len)
            .ok_or(format!("image {i} outside the blob"))?;
        if sha256::digest(data) != blob[e + 24..e + 56] {
            return Err(format!("image {i} hash mismatch"));
        }
        *ext = (off, len);
    }
    Ok(extents)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn layout_and_round_trip() {
        let init: Vec<u8> = (0..5000u32).map(|i| i as u8).collect();
        let blob = build("rk3588", &init, None).unwrap();
        assert_eq!(blob.len(), HEADER_SIZE + 6144);
        assert_eq!(&blob[0..4], b"RKNS");
        let ext = verify(&blob).unwrap();
        assert_eq!(ext, [(2048, 6144), (8192, 0)]);
        assert_eq!(&blob[2048..2048 + init.len()], &init[..]);
    }

    #[test]
    fn limits() {
        assert_eq!(build("rk3399", &[1], None), Err(Error::UnknownSoc));
        assert_eq!(build("rk3588", &[], None), Err(Error::EmptyInit));
        let big = vec![0u8; 0x100000];
        assert!(matches!(
            build("rk3588", &big, None),
            Err(Error::InitTooLarge { .. })
        ));
    }

    #[test]
    fn tamper_is_detected() {
        let mut blob = build("rk3588", &[7u8; 100], Some(&[9u8; 10])).unwrap();
        blob[HEADER_SIZE + 3] ^= 1;
        assert_eq!(verify(&blob), Err("image 0 hash mismatch".into()));
    }
}
