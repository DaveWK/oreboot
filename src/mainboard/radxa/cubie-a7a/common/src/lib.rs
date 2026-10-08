//! Definitions shared by the Cubie A7A bt0 (AArch32) and main (AArch64)
//! stages, and mirrored by the host-side image tool
//! (`scripts/mkimage.py` in the board workspace).
//!
//! Boot flow:
//!
//! ```text
//! BROM ─eGON─▶ bt0 (SRAM, AArch32) ─RMR warm reset─▶ main (DRAM, AArch64 EL3)
//!                                                       │ eret, x0 = DTB
//!                                                       ▼
//!                                                     payload (EDK2, EL2)
//! ```
#![no_std]

pub mod bakery;
pub mod ethernet;
pub mod fwlog;
pub mod memtest;
pub mod sha256;
pub mod time;
pub mod twi;
pub mod uart;

/// R_TWI0, the PMIC's I2C bus.
pub const R_TWI0_BASE: usize = 0x0708_3000;

/// Memory map. DRAM starts at 0x4000_0000.
pub mod mem {
    pub const DRAM_BASE: usize = 0x4000_0000;

    /// oreboot main is resident here for the whole uptime (PSCI). This range
    /// is never reported to the payload as usable memory.
    pub const EL3_BASE: usize = DRAM_BASE;
    pub const EL3_SIZE: usize = 0x0020_0000;
    /// Link/load address of oreboot main.
    pub const MAIN_BASE: usize = EL3_BASE;
    pub const MAIN_MAX_SIZE: usize = 0x0008_0000;
    /// Device tree that main builds for the payload.
    pub const DTB_ADDR: usize = EL3_BASE + 0x0010_0000;
    pub const DTB_MAX_SIZE: usize = 0x0001_0000;
    /// bt0 → main handoff block, see [`super::Handoff`].
    pub const HANDOFF_ADDR: usize = EL3_BASE + 0x001f_f000;

    /// First byte the payload may use.
    pub const PAYLOAD_RAM_BASE: usize = EL3_BASE + EL3_SIZE;
    /// Usable non-secure RAM, excluding resident EL3 firmware. Reject a bad
    /// bt0 handoff instead of wrapping its advertised end/usable length.
    pub fn payload_region(dram_base: u64, dram_size: u64) -> Option<(u64, u64)> {
        if dram_base != DRAM_BASE as u64 {
            return None;
        }
        let base = PAYLOAD_RAM_BASE as u64;
        let end = dram_base.checked_add(dram_size)?;
        let size = end.checked_sub(base)?;
        (size != 0).then_some((base, size))
    }

    /// A PSCI entry must contain one aligned AArch64 instruction in usable RAM.
    pub fn valid_payload_entry(dram_base: u64, dram_size: u64, entry: u64) -> bool {
        let Some((base, size)) = payload_region(dram_base, dram_size) else {
            return false;
        };
        entry >= base && entry % 4 == 0 && entry.checked_add(4).is_some_and(|end| end <= base + size)
    }

    #[cfg(test)]
    mod region_tests {
        use super::*;
        const BASE: u64 = DRAM_BASE as u64;
        const SIZE: u64 = 4 << 30;
        #[test]
        fn usable_region_excludes_el3() {
            assert_eq!(payload_region(BASE, SIZE), Some((PAYLOAD_RAM_BASE as u64, SIZE - EL3_SIZE as u64)));
            assert!(valid_payload_entry(BASE, SIZE, PAYLOAD_RAM_BASE as u64));
            assert!(!valid_payload_entry(BASE, SIZE, BASE));
            assert!(!valid_payload_entry(BASE, SIZE, PAYLOAD_RAM_BASE as u64 - 4));
        }
        #[test]
        fn entry_bounds_include_last_instruction_only() {
            assert!(valid_payload_entry(BASE, SIZE, BASE + SIZE - 4));
            assert!(!valid_payload_entry(BASE, SIZE, BASE + SIZE));
            assert!(!valid_payload_entry(BASE, SIZE, u64::MAX - 3));
            assert!(!valid_payload_entry(BASE, SIZE, PAYLOAD_RAM_BASE as u64 + 1));
        }
        #[test]
        fn malformed_handoff_never_wraps() {
            for size in [0, EL3_SIZE as u64 - 1, EL3_SIZE as u64, u64::MAX] {
                assert_eq!(payload_region(BASE, size), None);
                assert!(!valid_payload_entry(BASE, size, PAYLOAD_BASE as u64));
            }
            assert_eq!(payload_region(BASE - 4, SIZE), None);
        }
    }

    /// Where the EDK2 FD is loaded and entered (its PcdFdBaseAddress).
    pub const PAYLOAD_BASE: usize = 0x4100_0000;
}

/// Boot media layout table, written by the image tool at a fixed offset on
/// the SD card. bt0 reads it to find what to load where.
pub mod layout {
    /// Byte offset of the table on the boot medium (sector 4088, just below
    /// 2 MiB). Everything between bt0 and here stays free so a vendor U-Boot
    /// FIT (0x6a000..0x18e000 on Radxa's dlan17 cards) can stay in place for
    /// [`KIND_VENDOR_SPL`] recovery.
    pub const TABLE_OFFSET: u32 = 0x001f_f000;
    pub const MAGIC: [u8; 8] = *b"A7ABOOT1";
    pub const VERSION: u32 = 1;
    pub const MAX_ENTRIES: usize = 8;

    /// AArch64 EL3 firmware: oreboot main. Entered via RVBAR after the warm
    /// reset into AArch64.
    pub const KIND_MAIN: u32 = 1;
    /// Non-secure payload entered at EL2 (EDK2 FD).
    pub const KIND_PAYLOAD: u32 = 2;
    /// Optional DTB handed to the payload instead of the generated one.
    pub const KIND_DTB: u32 = 3;
    /// A vendor eGON SPL, loaded to SRAM 0x47000 and run in place of bt0
    /// when the previous oreboot boot never confirmed ([`super::recovery`]).
    pub const KIND_VENDOR_SPL: u32 = 4;

    #[repr(C)]
    #[derive(Clone, Copy, Debug, Default)]
    pub struct Entry {
        pub kind: u32,
        pub flags: u32,
        /// Byte offset on the medium, 512-byte aligned.
        pub offset: u32,
        /// Size in bytes.
        pub size: u32,
        pub load_addr: u64,
        /// CRC-32 (IEEE) of the image; 0 skips verification.
        pub crc32: u32,
        pub _reserved: u32,
    }

    #[repr(C)]
    #[derive(Clone, Copy, Debug)]
    pub struct Table {
        pub magic: [u8; 8],
        pub version: u32,
        pub count: u32,
        /// CRC-32 over the whole table with this field set to 0.
        pub crc32: u32,
        pub _reserved: u32,
        pub entries: [Entry; MAX_ENTRIES],
    }

    /// Version1 image-builder contract: images occupy the firmware gap,
    /// after the table, before the first partition at16MiB.
    pub const IMAGE_START: u64 = 0x0020_0000;
    pub const IMAGE_END: u64 = 0x0100_0000;
    pub const BLOCK_SIZE: u64 = 512;

    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    pub enum Error {
        Header,
        Checksum,
        UnsupportedFields,
        UnknownKind,
        DuplicateKind,
        MissingStage,
        EmptyImage,
        MediaRange,
        MemoryRange,
        LinkedAddress,
        Overlap,
    }

    /// Decode the media frame without assuming its byte buffer has Table's
    /// u64 alignment. All fields are plain integers and the length is checked.
    pub fn decode(bytes: &[u8]) -> Result<Table, Error> {
        if bytes.len() < TABLE_SIZE {
            return Err(Error::Header);
        }
        // SAFETY: complete repr(C) integer-only frame, possibly unaligned.
        let table: Table = unsafe { core::ptr::read_unaligned(bytes.as_ptr().cast()) };
        if table.magic != MAGIC || table.version != VERSION || table.count as usize > MAX_ENTRIES {
            return Err(Error::Header);
        }
        let mut checksum_bytes = [0u8; TABLE_SIZE];
        checksum_bytes.copy_from_slice(&bytes[..TABLE_SIZE]);
        checksum_bytes[CRC_FIELD_OFFSET..CRC_FIELD_OFFSET + 4].fill(0);
        if super::crc32(&checksum_bytes) != table.crc32 {
            return Err(Error::Checksum);
        }
        Ok(table)
    }

    fn padded(e: &Entry) -> Result<u64, Error> {
        if e.size == 0 {
            return Err(Error::EmptyImage);
        }
        Ok((u64::from(e.size) + BLOCK_SIZE - 1) & !(BLOCK_SIZE - 1))
    }

    fn media_range(e: &Entry) -> Result<(u64, u64), Error> {
        let start = u64::from(e.offset);
        let end = start.checked_add(padded(e)?).ok_or(Error::MediaRange)?;
        if start < IMAGE_START || start % BLOCK_SIZE != 0 || end > IMAGE_END {
            return Err(Error::MediaRange);
        }
        Ok((start, end))
    }

    fn memory_range(e: &Entry, dram_end: u64) -> Result<(u64, u64), Error> {
        use super::mem;
        let start = e.load_addr;
        let end = start.checked_add(padded(e)?).ok_or(Error::MemoryRange)?;
        // Check the full64-bit address before converting to AArch32 usize.
        if start & 3 != 0 || usize::try_from(start).is_err() || usize::try_from(end).is_err() {
            return Err(Error::MemoryRange);
        }
        match e.kind {
            KIND_MAIN => {
                if start != mem::MAIN_BASE as u64 || padded(e)? > mem::MAIN_MAX_SIZE as u64 {
                    return Err(Error::LinkedAddress);
                }
            }
            KIND_PAYLOAD => {
                if start != mem::PAYLOAD_BASE as u64 {
                    return Err(Error::LinkedAddress);
                }
            }
            KIND_DTB => {
                if start < mem::PAYLOAD_RAM_BASE as u64 {
                    return Err(Error::MemoryRange);
                }
            }
            KIND_VENDOR_SPL => {
                // Existing optional recovery format; never loaded by this path.
                if start != 0x47000 || padded(e)? > 0x4a000 {
                    return Err(Error::LinkedAddress);
                }
                return Ok((start, end));
            }
            _ => return Err(Error::UnknownKind),
        }
        if start < mem::DRAM_BASE as u64 || end > dram_end {
            return Err(Error::MemoryRange);
        }
        Ok((start, end))
    }

    fn overlaps(a: (u64, u64), b: (u64, u64)) -> bool {
        a.0 < b.1 && b.0 < a.1
    }

    /// Validate every entry before any image is copied. The table CRC is
    /// checked by the media reader; this checks the image/loader contract.
    pub fn validate(table: &Table, dram_end: u64) -> Result<(), Error> {
        if table.magic != MAGIC || table.version != VERSION || table.count as usize > MAX_ENTRIES {
            return Err(Error::Header);
        }
        if table._reserved != 0 {
            return Err(Error::UnsupportedFields);
        }
        let entries = &table.entries[..table.count as usize];
        let mut seen = 0u32;
        for (i, e) in entries.iter().enumerate() {
            if e.flags != 0 || e._reserved != 0 {
                return Err(Error::UnsupportedFields);
            }
            if !(KIND_MAIN..=KIND_VENDOR_SPL).contains(&e.kind) {
                return Err(Error::UnknownKind);
            }
            let bit = 1 << e.kind;
            if seen & bit != 0 {
                return Err(Error::DuplicateKind);
            }
            seen |= bit;
            let media = media_range(e)?;
            let memory = memory_range(e, dram_end)?;
            for prior in &entries[..i] {
                if overlaps(media, media_range(prior)?)
                    || (e.kind != KIND_VENDOR_SPL
                        && prior.kind != KIND_VENDOR_SPL
                        && overlaps(memory, memory_range(prior, dram_end)?))
                {
                    return Err(Error::Overlap);
                }
            }
        }
        let required = (1 << KIND_MAIN) | (1 << KIND_PAYLOAD);
        if seen & required != required {
            return Err(Error::MissingStage);
        }
        Ok(())
    }

    /// Actual bt0 load orchestration: no callback can run until the entire
    /// table passes. Recovery-only entries are deliberately excluded.
    pub fn load_entries(
        table: &Table,
        dram_end: u64,
        mut load: impl FnMut(&Entry, usize, usize),
    ) -> Result<(), Error> {
        validate(table, dram_end)?;
        for e in &table.entries[..table.count as usize] {
            if e.kind != KIND_VENDOR_SPL {
                // validate proved both conversions and the whole extent.
                load(e, e.load_addr as usize, padded(e)? as usize);
            }
        }
        Ok(())
    }

    pub const TABLE_SIZE: usize = core::mem::size_of::<Table>();
    pub const CRC_FIELD_OFFSET: usize = 16;
    const _: () = assert!(TABLE_SIZE == 24 + 32 * MAX_ENTRIES);
    const _: () = assert!(TABLE_SIZE <= 512);

    #[cfg(test)]
    mod load_controls {
        use super::*;
        use crate::mem;
        const END: u64 = 0xc000_0000;
        #[test]
        fn frame_crc_length_and_unaligned_access() {
            let mut bytes = [0u8; TABLE_SIZE + 1];
            let frame = &mut bytes[1..];
            frame[..8].copy_from_slice(&MAGIC);
            frame[8..12].copy_from_slice(&VERSION.to_le_bytes());
            let checksum = crate::crc32(frame);
            frame[CRC_FIELD_OFFSET..CRC_FIELD_OFFSET + 4].copy_from_slice(&checksum.to_le_bytes());
            assert_eq!(decode(frame).unwrap().count, 0);
            assert!(matches!(decode(&frame[..TABLE_SIZE - 1]), Err(Error::Header)));
            frame[24] ^= 1;
            assert!(matches!(decode(frame), Err(Error::Checksum)));
        }
        fn table() -> Table {
            let mut t = Table {
                magic: MAGIC,
                version: VERSION,
                count: 2,
                crc32: 0,
                _reserved: 0,
                entries: [Entry::default(); MAX_ENTRIES],
            };
            t.entries[0] = Entry {
                kind: KIND_MAIN,
                offset: 0x200000,
                size: 4096,
                load_addr: mem::MAIN_BASE as u64,
                ..Entry::default()
            };
            t.entries[1] = Entry {
                kind: KIND_PAYLOAD,
                offset: 0x300000,
                size: 1024,
                load_addr: mem::PAYLOAD_BASE as u64,
                ..Entry::default()
            };
            t
        }
        fn rejected_before_load(t: &Table) {
            let mut calls = 0;
            assert!(load_entries(t, END, |_, _, _| calls += 1).is_err());
            assert_eq!(
                calls, 0,
                "malformed later entry must not follow an earlier copy"
            );
        }
        #[test]
        fn linked_stages_and_optional_recovery() {
            let mut t = table();
            t.count = 3;
            t.entries[2] = Entry {
                kind: KIND_VENDOR_SPL,
                offset: 0x780000,
                size: 8192,
                load_addr: 0x47000,
                ..Entry::default()
            };
            let mut calls = 0;
            load_entries(&t, END, |e, addr, padded| {
                assert_ne!(e.kind, KIND_VENDOR_SPL);
                assert_eq!(addr as u64, e.load_addr);
                assert_eq!(padded as u64, u64::from(e.size));
                calls += 1;
            })
            .unwrap();
            assert_eq!(calls, 2);
        }
        #[test]
        fn full_address_and_large_padding_never_alias_or_wrap() {
            let mut t = table();
            t.entries[1].load_addr += 1 << 32;
            rejected_before_load(&t);
            for size in [0, u32::MAX - 510, u32::MAX] {
                let mut t = table();
                t.entries[0].size = size;
                rejected_before_load(&t);
            }
            let mut t = table();
            t.entries[1].load_addr = u64::MAX - 3;
            rejected_before_load(&t);
        }
        #[test]
        fn padded_transfer_extent_must_fit_dram() {
            let mut t = table();
            t.entries[1].size = 1;
            let end = t.entries[1].load_addr + 512;
            assert_eq!(validate(&t, end), Ok(()));
            let mut calls = 0;
            assert!(load_entries(&t, end - 1, |_, _, _| calls += 1).is_err());
            assert_eq!(calls, 0);
        }
        #[test]
        fn media_and_memory_overlap_fail_before_first_copy() {
            let mut t = table();
            t.entries[1].offset = t.entries[0].offset + 512;
            rejected_before_load(&t);
            let mut t = table();
            t.count = 3;
            t.entries[2] = Entry {
                kind: KIND_DTB,
                offset: 0x800000,
                size: 1,
                load_addr: mem::PAYLOAD_BASE as u64 + 512,
                ..Entry::default()
            };
            rejected_before_load(&t);
            t.entries[2].load_addr += 512;
            assert_eq!(validate(&t, END), Ok(())); // touching boundary allowed
            t.entries[2].load_addr = mem::HANDOFF_ADDR as u64;
            rejected_before_load(&t);
        }
        #[test]
        fn duplicates_missing_and_unknown_stages_rejected() {
            let mut t = table();
            t.count = 1;
            rejected_before_load(&t);
            let mut t = table();
            t.count = 3;
            t.entries[2] = t.entries[0];
            rejected_before_load(&t);
            let mut t = table();
            t.count = 4;
            t.entries[2] = Entry {
                kind: KIND_DTB,
                offset: 0x800000,
                size: 512,
                load_addr: 0x48000000,
                ..Entry::default()
            };
            t.entries[3] = Entry {
                kind: KIND_DTB,
                offset: 0x900000,
                size: 512,
                load_addr: 0x49000000,
                ..Entry::default()
            };
            rejected_before_load(&t); // distinct extents: duplicate-kind gate matters
            for kind in [0, 5, u32::MAX] {
                let mut t = table();
                t.entries[1].kind = kind;
                rejected_before_load(&t);
            }
        }
        #[test]
        fn version1_header_and_unknown_fields_rejected() {
            let mut t = table();
            t.magic[0] ^= 1;
            rejected_before_load(&t);
            let mut t = table();
            t.version += 1;
            rejected_before_load(&t);
            let mut t = table();
            t.count = MAX_ENTRIES as u32 + 1;
            rejected_before_load(&t);
            let mut t = table();
            t._reserved = 1;
            rejected_before_load(&t);
            let mut t = table();
            t.entries[1].flags = 1;
            rejected_before_load(&t);
            let mut t = table();
            t.entries[1]._reserved = 1;
            rejected_before_load(&t);
        }
        #[test]
        fn media_gap_and_linker_address_contract() {
            for offset in [0, TABLE_OFFSET, 0x300001, 0xfffffe00] {
                let mut t = table();
                t.entries[1].offset = offset;
                rejected_before_load(&t);
            }
            let mut t = table();
            t.entries[0].size = mem::MAIN_MAX_SIZE as u32 + 1;
            rejected_before_load(&t);
            let mut t = table();
            t.entries[0].load_addr += 4;
            rejected_before_load(&t);
            let mut t = table();
            t.entries[1].load_addr += 4;
            rejected_before_load(&t);
            let mut t = table();
            t.entries[1].size = (IMAGE_END - u64::from(t.entries[1].offset)) as u32;
            assert_eq!(validate(&t, END), Ok(()));
            t.entries[1].size += 1;
            rejected_before_load(&t);
        }
    }

}

/// Boot-attempt tracking, so a broken oreboot/EDK2 build falls back to the
/// vendor bootloader by itself instead of needing a card reader.
///
/// bt0 writes [`ATTEMPT`] to an RTC general purpose register (kept across
/// watchdog resets while VDD_RTC is up) and arms WDT0. EDK2 confirms from
/// its boot manager with the [`SIP_BOOT_OK`] SMC; main then clears the
/// register and stops the watchdog. If bt0 finds [`ATTEMPT`] still set, the
/// previous boot failed or hung, and it runs the vendor SPL once instead.
pub mod recovery {
    /// RTC GP_DATA_REG6 (A733 manual 9.1.7.14).
    pub const RTC_GPR: usize = 0x0709_0118;
    pub const ATTEMPT: u32 = u32::from_le_bytes(*b"A7At");
    /// SMC32 fast call, SiP service range.
    pub const SIP_BOOT_OK: u32 = 0x8200_0001;
    pub const WDT0_BASE: usize = 0x0205_0000;
}

pub const HANDOFF_MAGIC: u32 = u32::from_le_bytes(*b"A7AH");
pub const HANDOFF_VERSION: u32 = 2;

/// Written by bt0 at [`mem::HANDOFF_ADDR`] before the warm reset, because
/// no register state survives it.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct Handoff {
    pub magic: u32,
    pub version: u32,
    pub dram_base: u64,
    pub dram_size: u64,
    /// SoC silicon revision (0 = A, 1 = B, ...).
    pub soc_version: u32,
    /// Main oscillator, 24 or 26 MHz.
    pub hosc_mhz: u32,
    /// Arch counter value when bt0 jumped, for boot time accounting.
    pub bt0_done_ticks: u64,
    pub payload_addr: u64,
    pub payload_size: u64,
    /// 0 if no DTB was supplied on the medium.
    pub dtb_addr: u64,
    pub dtb_size: u64,
    /// CPU_L, CPU_B and DSU clocks in MHz as bt0 left them.
    pub cpu_mhz: [u32; 3],
    /// DRAM clock in MHz of the parameter set that trained.
    pub dram_mhz: u32,
}

/// CRC-32 (IEEE 802.3, reflected, as used by zlib and Python's
/// `zlib.crc32`).
pub fn crc32(data: &[u8]) -> u32 {
    crc32_update(0, data)
}

pub fn crc32_update(crc: u32, data: &[u8]) -> u32 {
    let mut c = !crc;
    for &b in data {
        c = CRC_TABLE[((c ^ b as u32) & 0xff) as usize] ^ (c >> 8);
    }
    !c
}

const CRC_TABLE: [u32; 256] = {
    let mut t = [0u32; 256];
    let mut i = 0;
    while i < 256 {
        let mut c = i as u32;
        let mut k = 0;
        while k < 8 {
            c = if c & 1 != 0 {
                0xedb8_8320 ^ (c >> 1)
            } else {
                c >> 1
            };
            k += 1;
        }
        t[i] = c;
        i += 1;
    }
    t
};

#[cfg(test)]
mod tests {
    #[test]
    fn crc32_check_value() {
        assert_eq!(super::crc32(b"123456789"), 0xcbf4_3926);
    }
}
