// SPDX-License-Identifier: GPL-2.0-only

//! `fetch` targets and their range arithmetic.
//!
//! A fetch names a partition of a read-only device (`nor`, a NOR partition,
//! `usb0`, `usb0p<N>`, `sd`), optionally narrowed to a window with
//! `@<hex offset>+<hex length>` (e.g. `usb0@1d00000000+4000000`), because
//! the stock tool only fetches whole partitions: `getvar partition-size`
//! of the windowed name is the window's length, and its chunk offsets are
//! relative to the window. All numbers are hex, `0x` optional.
use crate::proto::hex_u64;

/// A fetch window: offset and length.
pub type Window = Option<(u64, u64)>;

/// Split `name@off+len` into the name and the window.
pub fn split_window(name: &str) -> Result<(&str, Window), &'static str> {
    let Some((base, range)) = name.split_once('@') else {
        return Ok((name, None));
    };
    let (off, len) = range
        .split_once('+')
        .ok_or("window: <name>@<hex offset>+<hex length>")?;
    let off = hex_u64(off).ok_or("window offset is not hex")?;
    let len = hex_u64(len).ok_or("window length is not hex")?;
    if base.is_empty() || len == 0 {
        return Err("window: empty name or length");
    }
    Ok((base, Some((off, len))))
}

/// The size `getvar partition-size` reports for a target on a partition
/// of `size` bytes.
pub fn target_size(size: u64, window: Option<(u64, u64)>) -> Result<u64, &'static str> {
    match window {
        None => Ok(size),
        Some((off, len)) => match off.checked_add(len) {
            Some(end) if end <= size => Ok(len),
            _ => Err("window past the end of the partition"),
        },
    }
}

/// The byte range [start, start + len) inside a partition of `size` bytes
/// that a fetch of `offset`/`size` (relative to the window, if any)
/// covers; `size` None means to the end; `max` bounds the length.
pub fn resolve(
    part_size: u64,
    window: Option<(u64, u64)>,
    offset: u64,
    size: Option<u64>,
    max: u64,
) -> Result<(u64, u64), &'static str> {
    let (base, limit) = match window {
        None => (0, part_size),
        Some((off, _)) => (off, target_size(part_size, window)?),
    };
    if offset >= limit {
        return Err("fetch offset past the end");
    }
    let len = size.unwrap_or(limit - offset);
    if len == 0 || len > max {
        return Err("fetch size must be 1..=max-fetch-size");
    }
    if offset.checked_add(len).is_none_or(|e| e > limit) {
        return Err("fetch past the end");
    }
    Ok((base + offset, len))
}

/// The blocks of `block` bytes covering [start, start + len): first block,
/// block count, and how many bytes of the first block come before `start`.
pub fn blocks(start: u64, len: u64, block: u32) -> (u64, u64, usize) {
    let b = block as u64;
    let first = start / b;
    let end = (start + len).div_ceil(b);
    (first, end - first, (start % b) as usize)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn windows() {
        assert_eq!(split_window("usb0"), Ok(("usb0", None)));
        assert_eq!(
            split_window("usb0@1d00000000+4000000"),
            Ok(("usb0", Some((0x1d_0000_0000, 0x400_0000))))
        );
        assert_eq!(
            split_window("nor@0x1ff000+0x1000"),
            Ok(("nor", Some((0x1ff000, 0x1000))))
        );
        assert!(split_window("usb0@10").is_err());
        assert!(split_window("usb0@x+10").is_err());
        assert!(split_window("usb0@10+0").is_err());
        assert!(split_window("@10+1").is_err());
        assert_eq!(target_size(0x800000, Some((0x7ff000, 0x1000))), Ok(0x1000));
        assert!(target_size(0x800000, Some((0x7ff000, 0x1001))).is_err());
        assert!(target_size(0x800000, Some((u64::MAX, 2))).is_err());
    }

    #[test]
    fn resolve_chunks() {
        // The stock tool: whole partition in max-fetch-size chunks.
        let size = 0x80_0000;
        let mut off = 0;
        let mut n = 0;
        while off < size {
            let len = (size - off).min(0x10_0000);
            assert_eq!(
                resolve(size, None, off, Some(len), 0x10_0000),
                Ok((off, len))
            );
            off += len;
            n += 1;
        }
        assert_eq!(n, 8);
        // In a window: offsets are relative to it.
        let w = Some((0x1000_0000, 0x20_0000));
        assert_eq!(
            resolve(0x20_0000_0000, w, 0x10_0000, Some(0x10_0000), 0x10_0000),
            Ok((0x1010_0000, 0x10_0000))
        );
        assert!(resolve(0x20_0000_0000, w, 0x18_0000, Some(0x10_0000), 0x10_0000).is_err());
        // To the end, bounded by max.
        assert_eq!(
            resolve(0x3000, None, 0x2f00, None, 0x1000),
            Ok((0x2f00, 0x100))
        );
        assert!(resolve(0x3000, None, 0, None, 0x1000).is_err());
        assert!(resolve(0x3000, None, 0x3000, Some(1), 0x1000).is_err());
        assert!(resolve(0x3000, None, 0, Some(0), 0x1000).is_err());
        assert!(resolve(u64::MAX, None, u64::MAX - 1, Some(4), 0x1000).is_err());
    }

    #[test]
    fn block_cover() {
        assert_eq!(blocks(0, 512, 512), (0, 1, 0));
        assert_eq!(blocks(0, 513, 512), (0, 2, 0));
        assert_eq!(blocks(511, 2, 512), (0, 2, 511));
        assert_eq!(
            blocks(0x1d_0000_0000, 0x10_0000, 512),
            (0x1d_0000_0000 / 512, 2048, 0)
        );
        assert_eq!(blocks(4096 + 10, 4096, 4096), (1, 2, 10));
    }
}
