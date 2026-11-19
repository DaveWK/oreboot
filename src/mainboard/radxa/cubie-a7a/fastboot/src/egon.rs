//! The eGON.BT0 header the mask ROM checks before it runs a boot0 from a
//! medium: magic at 4, checksum at 12, length at 16 (a multiple of 4: the
//! sum runs over 32-bit words), run address at 32. The checksum is the
//! 32-bit word sum over `length` bytes with the checksum word taken as the
//! stamp 0x5f0a6c39 (sunxi `mksunxiboot`, SyterKit `mksunxi`).
use crate::le32;

pub const MAGIC: [u8; 8] = *b"eGON.BT0";
pub const STAMP: u32 = 0x5f0a_6c39;
/// Where the mask ROM loads and runs a boot0 from a medium (bt0's SD and
/// SPI NOR builds); the FEL build is linked at 0x90000 instead.
pub const MEDIA_RUN_ADDR: u32 = 0x0004_7000;

/// The checks a boot0 image for a medium must pass: magic, length (whole
/// words, at least the header, at most `max` and the image), checksum, and
/// a run address of [`MEDIA_RUN_ADDR`] (so a FEL build is refused).
pub fn check(image: &[u8], max: u32) -> Result<(), &'static str> {
    if image.len() < 64 || image[4..12] != MAGIC {
        return Err("not an eGON.BT0 image");
    }
    let length = le32(image, 16).unwrap();
    if length < 64 || !length.is_multiple_of(4) || length > max || length as usize > image.len() {
        return Err("eGON length invalid for this partition");
    }
    if le32(image, 32).unwrap() != MEDIA_RUN_ADDR {
        return Err("eGON run address is not 0x47000 (FEL build?)");
    }
    if checksum(&image[..length as usize]) != le32(image, 12).unwrap() {
        return Err("eGON checksum mismatch");
    }
    Ok(())
}

/// The checksum the mask ROM computes over `image` (whole words).
pub fn checksum(image: &[u8]) -> u32 {
    image.chunks_exact(4).enumerate().fold(0u32, |sum, (i, w)| {
        let w = if i == 3 {
            STAMP
        } else {
            u32::from_le_bytes([w[0], w[1], w[2], w[3]])
        };
        sum.wrapping_add(w)
    })
}

#[cfg(test)]
pub(crate) fn make(len: usize, run: u32) -> std::vec::Vec<u8> {
    let mut v: std::vec::Vec<u8> = (0..len as u32).map(|i| (i * 7) as u8).collect();
    v[4..12].copy_from_slice(&MAGIC);
    v[16..20].copy_from_slice(&(len as u32).to_le_bytes());
    v[32..36].copy_from_slice(&run.to_le_bytes());
    let c = checksum(&v);
    v[12..16].copy_from_slice(&c.to_le_bytes());
    v
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn valid_image() {
        let v = make(0x1000, MEDIA_RUN_ADDR);
        assert_eq!(check(&v, 0x40000), Ok(()));
        // Trailing bytes past `length` are not summed.
        let mut w = v.clone();
        w.extend_from_slice(&[1, 2, 3]);
        assert_eq!(check(&w, 0x40000), Ok(()));
    }

    #[test]
    fn refusals() {
        let v = make(0x1000, MEDIA_RUN_ADDR);
        assert!(check(&v[..63], 0x40000).is_err());
        assert!(check(&v, 0xffc).is_err(), "longer than the partition");
        assert!(
            check(&v[..0x800], 0x40000).is_err(),
            "length past the image"
        );
        let mut w = v.clone();
        w[100] ^= 1;
        assert_eq!(check(&w, 0x40000), Err("eGON checksum mismatch"));
        let mut w = v.clone();
        w[4] = b'X';
        assert_eq!(check(&w, 0x40000), Err("not an eGON.BT0 image"));
        let mut w = v.clone();
        w[16] = 2; // length 0x1002: not whole words
        assert!(check(&w, 0x40000).is_err());
        let fel = make(0x1000, 0x90000);
        assert_eq!(
            check(&fel, 0x40000),
            Err("eGON run address is not 0x47000 (FEL build?)")
        );
    }
}
