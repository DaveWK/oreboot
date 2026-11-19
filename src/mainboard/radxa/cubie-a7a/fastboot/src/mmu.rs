//! The identity map bt0 runs fastboot with (AArch32 short descriptors,
//! 1 MiB sections), and cache-line arithmetic for DMA maintenance.
//!
//! With the MMU and D-cache on, the CPU's FIFO copies land in cached DRAM
//! instead of going to DRAM word by word (bt0 otherwise runs with the
//! D-cache off). [`FASTBOOT_MAP`]:
//!
//! | Range | Memory | |
//! |---|---|---|
//! | 0x0000_0000..0x0010_0000 | SRAM (bt0, stack, mailbox, ROM FEL data) | normal, write-back |
//! | 0x0010_0000..0x4000_0000 | peripherals | device, never executable |
//! | 0x4000_0000..0x5800_0000 | DRAM: main, EDK2, download buffer | normal, write-back |
//! | 0x5800_0000..0x5810_0000 | DRAM: EHCI schedule, bounce and GPT buffers | normal, non-cacheable |
//! | 0x5810_0000..0xc000_0000 | DRAM (the extension at 0x5900_0000) | normal, write-back |
//! | 0xc000_0000.. | (no 32-bit DRAM) | fault |
//!
//! bt0 cleans and turns the cache and MMU off before any hand-over, return
//! to FEL or reset of the ROM's view (bt0 `cache.rs`); the DRAM lab mode
//! never turns them on.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Attr {
    Fault,
    /// Shareable device (TEX 000, C 0, B 1), execute never.
    Device,
    /// Normal, inner and outer write-back write-allocate (TEX 001, C 1, B 1).
    WriteBack,
    /// Normal, non-cacheable (TEX 001, C 0, B 0).
    NonCacheable,
}

#[derive(Clone, Copy, Debug)]
pub struct Region {
    pub start: u32,
    /// Exclusive; 0 means the end of the 32-bit space.
    pub end: u32,
    pub attr: Attr,
}

pub const SECTION: u32 = 1 << 20;

pub const FASTBOOT_MAP: [Region; 5] = [
    Region {
        start: 0,
        end: 0x0010_0000,
        attr: Attr::WriteBack,
    },
    Region {
        start: 0x0010_0000,
        end: 0x4000_0000,
        attr: Attr::Device,
    },
    Region {
        start: 0x4000_0000,
        end: 0x5800_0000,
        attr: Attr::WriteBack,
    },
    Region {
        start: 0x5800_0000,
        end: 0x5810_0000,
        attr: Attr::NonCacheable,
    },
    Region {
        start: 0x5810_0000,
        end: 0xc000_0000,
        attr: Attr::WriteBack,
    },
];

/// One section descriptor: full access (AP 11), domain 0, global.
pub const fn section(base: u32, attr: Attr) -> u32 {
    let b = base & !(SECTION - 1);
    let ap = 3 << 10;
    match attr {
        Attr::Fault => 0,
        Attr::Device => b | ap | (1 << 4) | (1 << 2) | (1 << 16) | 2,
        Attr::WriteBack => b | ap | (1 << 12) | (1 << 3) | (1 << 2) | 2,
        Attr::NonCacheable => b | ap | (1 << 12) | 2,
    }
}

/// The 4096-entry first-level table for `map` (sections not listed fault).
/// Regions must be section aligned and in order.
pub fn table(map: &[Region], out: &mut [u32]) -> Result<(), &'static str> {
    if out.len() != 4096 {
        return Err("table: 4096 entries");
    }
    out.fill(0);
    let mut prev_end: u64 = 0;
    for r in map {
        let end = if r.end == 0 { 1u64 << 32 } else { r.end as u64 };
        if r.start % SECTION != 0
            || end % SECTION as u64 != 0
            || (r.start as u64) < prev_end
            || end <= r.start as u64
        {
            return Err("table: regions unaligned or out of order");
        }
        let mut a = r.start as u64;
        while a < end {
            out[(a >> 20) as usize] = section(a as u32, r.attr);
            a += SECTION as u64;
        }
        prev_end = end;
    }
    Ok(())
}

/// The attribute `map` gives `addr`.
pub fn attr_of(map: &[Region], addr: u32) -> Attr {
    map.iter()
        .find(|r| addr >= r.start && (r.end == 0 || addr < r.end))
        .map_or(Attr::Fault, |r| r.attr)
}

/// [start, end) rounded out to whole cache lines of `line` bytes (a power
/// of two): what clean/invalidate by address must cover.
pub fn lines(start: usize, len: usize, line: usize) -> (usize, usize) {
    let s = start & !(line - 1);
    let e = (start + len + line - 1) & !(line - 1);
    (s, e)
}

/// Can an invalidate of [start, start + len) destroy someone else's data?
/// (Yes when the range does not start and end on line boundaries: the
/// partial lines must be cleaned first.)
pub fn needs_clean_first(start: usize, len: usize, line: usize) -> bool {
    !start.is_multiple_of(line) || !(start + len).is_multiple_of(line)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::vec;

    #[test]
    fn descriptors() {
        // Write-back normal: TEX 001, C, B, AP 11, section.
        assert_eq!(section(0x4000_0000, Attr::WriteBack), 0x4000_1c0e);
        // Device: B, XN, S, AP 11.
        assert_eq!(section(0x0200_0000, Attr::Device), 0x0201_0c16);
        assert_eq!(section(0x5800_0000, Attr::NonCacheable), 0x5800_1c02);
        assert_eq!(section(0x1234_5678, Attr::Fault), 0);
    }

    #[test]
    fn the_fastboot_map() {
        let mut t = vec![0u32; 4096];
        table(&FASTBOOT_MAP, &mut t).unwrap();
        let at = |a: u32| t[(a >> 20) as usize];
        // bt0 in SRAM runs (executable, cached); peripherals never execute.
        assert_eq!(at(0x0009_0000), section(0, Attr::WriteBack));
        for mmio in [
            0x0200_2000u32,
            0x0410_0000,
            0x0420_0000,
            0x0340_0000,
            0x0702_5000,
            0x0a90_0000,
            0x0254_0000,
        ] {
            assert_eq!(at(mmio) & 0x1f, 0x16, "{mmio:#x} device XN");
        }
        assert_eq!(attr_of(&FASTBOOT_MAP, 0x0410_0000), Attr::Device);
        // The EHCI schedule's MiB is uncached, the download buffer and the
        // DRAM extension are cached.
        assert_eq!(attr_of(&FASTBOOT_MAP, 0x5800_0100), Attr::NonCacheable);
        assert_eq!(attr_of(&FASTBOOT_MAP, 0x5801_0000), Attr::NonCacheable);
        assert_eq!(attr_of(&FASTBOOT_MAP, 0x4800_0000), Attr::WriteBack);
        assert_eq!(attr_of(&FASTBOOT_MAP, 0x5900_0000), Attr::WriteBack);
        assert_eq!(at(0xc000_0000), 0);
        assert_eq!(at(0xffff_0000), 0);
        // Every section an identity map.
        for (i, &d) in t.iter().enumerate() {
            if d != 0 {
                assert_eq!(d >> 20, i as u32);
            }
        }
        // Malformed maps are refused.
        let bad = [Region {
            start: 0x100,
            end: 0x10_0000,
            attr: Attr::Device,
        }];
        assert!(table(&bad, &mut t).is_err());
        let order = [FASTBOOT_MAP[2], FASTBOOT_MAP[0]];
        assert!(table(&order, &mut t).is_err());
        assert!(table(&FASTBOOT_MAP, &mut t[..10]).is_err());
    }

    #[test]
    fn line_rounding() {
        assert_eq!(lines(0x4800_0000, 0x1000, 64), (0x4800_0000, 0x4800_1000));
        assert_eq!(lines(0x4800_0010, 0x10, 64), (0x4800_0000, 0x4800_0040));
        assert_eq!(lines(0x4800_003f, 2, 64), (0x4800_0000, 0x4800_0080));
        assert!(!needs_clean_first(0x4800_0000, 0x1000, 64));
        assert!(needs_clean_first(0x4800_0004, 0x1000, 64));
        assert!(needs_clean_first(0x4800_0000, 13, 64));
    }
}
