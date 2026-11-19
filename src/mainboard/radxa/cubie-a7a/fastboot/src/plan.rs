//! `fastboot boot` of a bundle on the Cubie A7A: everything checked before
//! the OKAY ([`check`]), and the copies after it ([`for_each_image`]).
//!
//! The layout table, its decoding and the per-kind load contract are bt0's
//! (`cubie_a7a_common::layout`, the same checks the SD, SPI NOR and FEL
//! boots run); this adds the bundle around them: where the frame is
//! ([`crate::bundle::locate`]), that every image the table loads lies inside
//! the bundle, matches its CRC and does not load over the download itself.
use cubie_a7a_common::{crc32, layout};

use crate::proto::Info;
use crate::{bootimg, bundle};

/// A bundle that passed every check.
#[derive(Clone, Copy, Debug)]
pub struct Checked {
    pub table: layout::Table,
    /// The bundle's medium offset.
    pub base: u32,
    /// The bundle within the download (after an Android boot image header).
    pub start: usize,
    pub len: usize,
}

/// Check the download `image`, which sits at `image_addr` in the board's
/// memory, as a bundle for a board whose tested DRAM ends at `dram_end`.
pub fn check(
    image: &[u8],
    image_addr: u64,
    dram_end: u64,
    info: &mut dyn Info,
) -> Result<Checked, &'static str> {
    let data = bootimg::payload(image)?;
    let start = data.as_ptr() as usize - image.as_ptr() as usize;
    let at = bundle::locate(data, layout::TABLE_OFFSET, bundle::SECTOR, |f| {
        layout::decode(f).is_ok()
    })
    .ok_or("no valid A7ABOOT1 layout table in the download")?;
    let table =
        layout::decode(&data[at.pos..at.pos + bundle::SECTOR]).map_err(|_| "layout table")?;
    layout::validate(&table, dram_end).map_err(|_| "layout table does not fit this board")?;
    info.info(format_args!(
        "layout table at bundle offset 0x{:x} (medium base 0x{:x})",
        at.pos, at.base
    ));
    let mut error = None;
    let mut main = false;
    layout::load_entries(&table, dram_end, |e, addr, padded| {
        if error.is_some() {
            return;
        }
        let Some(src) = bundle::image(data, at.base, e.offset, e.size) else {
            error = Some("an image lies past the end of the bundle");
            return;
        };
        if bundle::overlaps(addr as u64, padded as u64, image_addr, image.len() as u64) {
            error = Some("an image would load over the download buffer");
            return;
        }
        if e.crc32 != 0 && crc32(src) != e.crc32 {
            error = Some("an image does not match its CRC");
            return;
        }
        main |= e.kind == layout::KIND_MAIN;
        info.info(format_args!(
            "kind {}: {} bytes -> 0x{addr:08x}",
            e.kind, e.size
        ));
    })
    .map_err(|_| "boot layout invalid")?;
    if let Some(e) = error {
        return Err(e);
    }
    if !main {
        return Err("no oreboot main in the layout");
    }
    Ok(Checked {
        table,
        base: at.base,
        start,
        len: data.len(),
    })
}

/// Every image of a [`check`]ed bundle in `image` (the same download), in
/// the table's load order: `(entry, load address, bytes)`.
pub fn for_each_image(
    c: &Checked,
    image: &[u8],
    dram_end: u64,
    mut f: impl FnMut(&layout::Entry, usize, &[u8]),
) -> Result<(), &'static str> {
    let data = image.get(c.start..c.start + c.len).ok_or("bundle moved")?;
    let mut missing = false;
    layout::load_entries(&c.table, dram_end, |e, addr, _| {
        match bundle::image(data, c.base, e.offset, e.size) {
            Some(src) => f(e, addr, src),
            None => missing = true,
        }
    })
    .map_err(|_| "boot layout invalid")?;
    if missing {
        return Err("an image lies past the end of the bundle");
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use cubie_a7a_common::mem;
    use layout::{Entry, Table, MAX_ENTRIES, TABLE_SIZE};
    use std::{string::String, vec, vec::Vec};

    const DRAM_END: u64 = 0xc000_0000;
    const BUF: u64 = 0x4800_0000;

    struct Log(Vec<String>);
    impl Info for Log {
        fn info(&mut self, msg: core::fmt::Arguments) {
            self.0.push(std::format!("{msg}"));
        }
    }

    /// A medium cut from the table on: frame, main at 0x200000, the FD at
    /// 0x300000 (the FEL boot's layout, smaller images).
    fn medium(main_len: usize, fd_len: usize) -> Vec<u8> {
        let t0 = layout::TABLE_OFFSET as usize;
        let mut m = vec![0xffu8; 0x300000 + fd_len - t0];
        let main: Vec<u8> = (0..main_len).map(|i| (i * 3) as u8).collect();
        let fd: Vec<u8> = (0..fd_len).map(|i| (i * 5 + 1) as u8).collect();
        m[0x200000 - t0..0x200000 - t0 + main_len].copy_from_slice(&main);
        m[0x300000 - t0..].copy_from_slice(&fd);
        let mut t = Table {
            magic: layout::MAGIC,
            version: layout::VERSION,
            count: 2,
            crc32: 0,
            _reserved: 0,
            entries: [Entry::default(); MAX_ENTRIES],
        };
        t.entries[0] = Entry {
            kind: layout::KIND_MAIN,
            offset: 0x200000,
            size: main_len as u32,
            load_addr: mem::MAIN_BASE as u64,
            crc32: crc32(&main),
            ..Entry::default()
        };
        t.entries[1] = Entry {
            kind: layout::KIND_PAYLOAD,
            offset: 0x300000,
            size: fd_len as u32,
            load_addr: mem::PAYLOAD_BASE as u64,
            crc32: crc32(&fd),
            ..Entry::default()
        };
        let frame = bootlayout::encode(&t);
        m[..TABLE_SIZE].copy_from_slice(&frame[..TABLE_SIZE]);
        m
    }

    fn android(kernel: &[u8]) -> Vec<u8> {
        let mut v = vec![0u8; 2048];
        v[..8].copy_from_slice(b"ANDROID!");
        v[8..12].copy_from_slice(&(kernel.len() as u32).to_le_bytes());
        v[36..40].copy_from_slice(&2048u32.to_le_bytes());
        v.extend_from_slice(kernel);
        v.resize(v.len().div_ceil(2048) * 2048, 0);
        v
    }

    fn run(img: &[u8], addr: u64) -> Result<Checked, &'static str> {
        check(img, addr, DRAM_END, &mut Log(Vec::new()))
    }

    #[test]
    fn plain_and_wrapped_bundles() {
        let m = medium(5000, 0x2_0000);
        let c = run(&m, BUF).unwrap();
        assert_eq!((c.base, c.start, c.len), (layout::TABLE_OFFSET, 0, m.len()));
        let mut got = Vec::new();
        for_each_image(&c, &m, DRAM_END, |e, addr, src| {
            got.push((e.kind, addr, src.len(), src[1]))
        })
        .unwrap();
        assert_eq!(
            got,
            [
                (1, mem::MAIN_BASE, 5000, 3),
                (2, mem::PAYLOAD_BASE, 0x2_0000, 6)
            ]
        );
        let w = android(&m);
        let c = run(&w, BUF).unwrap();
        assert_eq!((c.start, c.len), (2048, m.len()));
        // A medium image from 0 (the NOR image): base 0.
        let mut full = vec![0u8; layout::TABLE_OFFSET as usize];
        full.extend_from_slice(&m);
        assert_eq!(run(&full, BUF).unwrap().base, 0);
    }

    #[test]
    fn refusals() {
        let m = medium(5000, 0x2_0000);
        // Truncated: the FD is cut short.
        assert_eq!(
            run(&m[..m.len() - 1], BUF).unwrap_err(),
            "an image lies past the end of the bundle"
        );
        // A corrupt image.
        let mut bad = m.clone();
        bad[0x200000 - layout::TABLE_OFFSET as usize + 10] ^= 1;
        assert_eq!(
            run(&bad, BUF).unwrap_err(),
            "an image does not match its CRC"
        );
        // A corrupt table frame: no valid frame at all.
        let mut bad = m.clone();
        bad[30] ^= 1;
        assert_eq!(
            run(&bad, BUF).unwrap_err(),
            "no valid A7ABOOT1 layout table in the download"
        );
        // The download sits where main loads.
        assert_eq!(
            run(&m, mem::MAIN_BASE as u64 - 16).unwrap_err(),
            "an image would load over the download buffer"
        );
        // Less DRAM than the payload needs.
        assert!(check(&m, BUF, mem::PAYLOAD_BASE as u64, &mut Log(Vec::new())).is_err());
        // Not a bundle at all, and a broken boot image.
        assert!(run(&[0u8; 4096], BUF).is_err());
        assert_eq!(
            run(b"ANDROID!", BUF).unwrap_err(),
            "boot image header truncated"
        );
    }

    /// A real boot bundle (`scripts/fastboot-boot.sh bundle` in
    /// radxa-a7a-workspace), when A7A_BUNDLE names it.
    #[test]
    fn real_bundle_if_given() {
        let Ok(path) = std::env::var("A7A_BUNDLE") else {
            return;
        };
        let m = std::fs::read(path).unwrap();
        let mut log = Log(Vec::new());
        let c = check(&m, BUF, DRAM_END, &mut log).unwrap();
        std::println!("{:#?}", log.0);
        let c2 = check(&android(&m), BUF, DRAM_END, &mut Log(Vec::new())).unwrap();
        assert_eq!(
            (c.base, c2.base, c2.start),
            (layout::TABLE_OFFSET, layout::TABLE_OFFSET, 2048)
        );
        let mut kinds = Vec::new();
        for_each_image(&c, &m, DRAM_END, |e, addr, src| {
            assert_eq!(crc32(src), e.crc32);
            kinds.push((e.kind, addr));
        })
        .unwrap();
        assert_eq!(kinds, [(1, mem::MAIN_BASE), (2, mem::PAYLOAD_BASE)]);
    }
}
