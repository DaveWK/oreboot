// SPDX-License-Identifier: GPL-2.0-only

//! A `fastboot boot` bundle: a copy of the boot medium from some offset
//! on, holding the boot layout table frame and the images it names, as on
//! the SD card or SPI NOR. Any such cut works:
//!
//! - the whole medium image (the 8 MiB NOR image, base 0);
//! - the SD image's `firmware.bin` (base = `firmware.seek` * 512);
//! - the medium from the table on (base = the table offset), the smallest.
//!
//! The table names images by their offsets on the medium, so the bundle's
//! base follows from where its frame is: base = table offset - position.
//! Frames are sector aligned on the medium and so in every cut that starts
//! on a sector; [`locate`] tries every 512-byte position from the start up
//! to the table offset and takes the first that passes the board's frame
//! check (magic, version and CRC: bt0's `layout::decode`).

/// One medium sector, the alignment of the table frame.
pub const SECTOR: usize = 512;
pub const TABLE_MAGIC: [u8; 8] = *b"A7ABOOT1";

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Located {
    /// The frame's position in the bundle.
    pub pos: usize,
    /// The medium offset of the bundle's first byte.
    pub base: u32,
}

/// Find the layout table frame in `data`, a cut of the medium whose table
/// is at `table_offset`; `valid` checks a candidate frame of `frame` bytes.
pub fn locate(
    data: &[u8],
    table_offset: u32,
    frame: usize,
    mut valid: impl FnMut(&[u8]) -> bool,
) -> Option<Located> {
    let last = (table_offset as usize).min(data.len().checked_sub(frame)?);
    (0..=last)
        .step_by(SECTOR)
        .filter(|&pos| data[pos..pos + TABLE_MAGIC.len()] == TABLE_MAGIC)
        .find(|&pos| valid(&data[pos..pos + frame]))
        .map(|pos| Located {
            pos,
            base: table_offset - pos as u32,
        })
}

/// The `size` bytes at medium offset `offset` in a bundle starting at
/// medium offset `base`, if the bundle holds all of them.
pub fn image(data: &[u8], base: u32, offset: u32, size: u32) -> Option<&[u8]> {
    let start = offset.checked_sub(base)? as usize;
    data.get(start..start.checked_add(size as usize)?)
}

/// Do [a, a + alen) and [b, b + blen) share a byte? (Empty ranges never do.)
pub fn overlaps(a: u64, alen: u64, b: u64, blen: u64) -> bool {
    alen != 0 && blen != 0 && a < b.saturating_add(blen) && b < a.saturating_add(alen)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::vec;

    const TABLE: u32 = 0x1ff000;
    const FRAME: usize = 512;

    /// A medium with a frame whose byte 8 is `tag` (the test's "CRC").
    fn medium(tag: u8) -> std::vec::Vec<u8> {
        let mut m = vec![0xffu8; 0x300000];
        m[TABLE as usize..TABLE as usize + 8].copy_from_slice(&TABLE_MAGIC);
        m[TABLE as usize + 8] = tag;
        m
    }
    fn ok(f: &[u8]) -> bool {
        f[8] == 1
    }

    #[test]
    fn every_cut_finds_the_same_base() {
        let m = medium(1);
        for base in [0usize, 0x2000, 0x1fe000, TABLE as usize] {
            let cut = &m[base..];
            let l = locate(cut, TABLE, FRAME, ok).unwrap();
            assert_eq!(
                l,
                Located {
                    pos: TABLE as usize - base,
                    base: base as u32
                }
            );
            // main at 0x200000 in medium terms
            let main = image(cut, l.base, 0x200000, 16).unwrap();
            assert_eq!(main.as_ptr(), m[0x200000..].as_ptr());
        }
    }

    #[test]
    fn invalid_or_missing_frames() {
        let m = medium(2);
        assert_eq!(locate(&m, TABLE, FRAME, ok), None);
        // Cut past the frame or too short for one.
        let m = medium(1);
        assert_eq!(locate(&m[TABLE as usize + 512..], TABLE, FRAME, ok), None);
        assert_eq!(
            locate(&m[TABLE as usize..TABLE as usize + 511], TABLE, FRAME, ok),
            None
        );
        assert_eq!(locate(&[], TABLE, FRAME, ok), None);
        // Not sector aligned: not found.
        assert_eq!(locate(&m[0x100..], TABLE, FRAME, ok), None);
    }

    #[test]
    fn first_valid_frame_wins_and_bogus_magic_is_skipped() {
        let mut m = medium(1);
        // A bogus copy earlier (fails the check), then the real one.
        m[0x1000..0x1008].copy_from_slice(&TABLE_MAGIC);
        m[0x1008] = 7;
        assert_eq!(locate(&m, TABLE, FRAME, ok).unwrap().base, 0);
    }

    #[test]
    fn image_bounds() {
        let d = [0u8; 100];
        assert!(image(&d, 1000, 1000, 100).is_some());
        assert!(image(&d, 1000, 1000, 101).is_none());
        assert!(image(&d, 1000, 999, 1).is_none());
        assert!(image(&d, 1000, 1050, 50).is_some());
        assert!(image(&d, 0, u32::MAX, 2).is_none());
        assert_eq!(image(&d, 0, 100, 0).map(|s| s.len()), Some(0));
    }

    #[test]
    fn overlap() {
        assert!(overlaps(0, 10, 9, 1));
        assert!(!overlaps(0, 10, 10, 1));
        assert!(!overlaps(10, 1, 0, 10));
        assert!(overlaps(5, 1, 0, 10));
        assert!(!overlaps(0, 0, 0, 10));
        assert!(overlaps(u64::MAX - 1, 10, u64::MAX - 1, 1));
    }
}
