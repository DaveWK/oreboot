//! The Android boot image around a `fastboot boot` download.
//!
//! `fastboot boot <file>` sends the file as it is when it starts with the
//! boot image magic, and otherwise wraps it as the kernel of a boot image it
//! builds (header version 0, 2048-byte pages, unless `--header-version` or
//! `--page-size` say otherwise). [`payload`] undoes that: the kernel of a
//! boot image, or the download itself.
//!
//! Header fields used (little endian): magic `ANDROID!` at 0, kernel size
//! at 8, page size at 36 (versions 0 to 2) and header version at 40; from
//! version 3 the page size is fixed at 4096 and offset 36 is reserved. The
//! kernel starts at the first page after the header page.
use crate::le32;

pub const MAGIC: [u8; 8] = *b"ANDROID!";
const HEADER_MIN: usize = 44;

/// The payload of a download: the kernel of a boot image, or all of it.
pub fn payload(image: &[u8]) -> Result<&[u8], &'static str> {
    if image.len() < MAGIC.len() || image[..MAGIC.len()] != MAGIC {
        return Ok(image);
    }
    if image.len() < HEADER_MIN {
        return Err("boot image header truncated");
    }
    let kernel = le32(image, 8).unwrap() as usize;
    let version = le32(image, 40).unwrap();
    let page = match version {
        0..=2 => le32(image, 36).unwrap() as usize,
        3 | 4 => 4096,
        _ => return Err("unknown boot image header version"),
    };
    if !page.is_power_of_two() || !(2048..=65536).contains(&page) {
        return Err("bad boot image page size");
    }
    if kernel == 0 {
        return Err("boot image without a kernel");
    }
    let end = page
        .checked_add(kernel)
        .ok_or("bad boot image kernel size")?;
    image
        .get(page..end)
        .ok_or("boot image kernel past the download")
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::vec::Vec;

    fn wrap(version: u32, page: u32, kernel: &[u8]) -> Vec<u8> {
        let p = if version >= 3 { 4096 } else { page as usize };
        let mut v = std::vec![0u8; p];
        v[..8].copy_from_slice(&MAGIC);
        v[8..12].copy_from_slice(&(kernel.len() as u32).to_le_bytes());
        if version < 3 {
            v[36..40].copy_from_slice(&page.to_le_bytes());
        }
        v[40..44].copy_from_slice(&version.to_le_bytes());
        v.extend_from_slice(kernel);
        // mkbootimg pads to whole pages
        v.resize(v.len().div_ceil(p) * p, 0);
        v
    }

    #[test]
    fn plain_downloads_pass_through() {
        assert_eq!(payload(b"A7ABOOT1...").unwrap(), b"A7ABOOT1...");
        assert_eq!(payload(b"ANDROI").unwrap(), b"ANDROI");
        assert_eq!(payload(b"").unwrap(), b"");
    }

    #[test]
    fn kernels_of_each_header_version() {
        let k: Vec<u8> = (0..5000u32).map(|i| i as u8).collect();
        for (v, p) in [(0, 2048), (1, 4096), (2, 2048), (3, 0), (4, 0)] {
            let img = wrap(v, p, &k);
            assert_eq!(payload(&img).unwrap(), &k[..], "v{v}");
        }
    }

    #[test]
    fn malformed_headers() {
        let k = [1u8; 100];
        let mut img = wrap(0, 2048, &k);
        assert!(payload(&img[..40]).is_err());
        img[40] = 9;
        assert_eq!(payload(&img), Err("unknown boot image header version"));
        let mut img = wrap(0, 2048, &k);
        img[36..40].copy_from_slice(&3000u32.to_le_bytes());
        assert_eq!(payload(&img), Err("bad boot image page size"));
        let mut img = wrap(0, 2048, &k);
        img[8..12].copy_from_slice(&0u32.to_le_bytes());
        assert_eq!(payload(&img), Err("boot image without a kernel"));
        let mut img = wrap(0, 2048, &k);
        img[8..12].copy_from_slice(&u32::MAX.to_le_bytes());
        assert_eq!(payload(&img), Err("boot image kernel past the download"));
        let img = wrap(0, 2048, &k);
        assert_eq!(
            payload(&img[..2048 + 50]),
            Err("boot image kernel past the download")
        );
    }
}
