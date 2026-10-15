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

pub mod ethernet;
pub mod fwlog;
pub mod memtest;
pub mod mmio;
pub mod time;
pub mod twi;
pub mod uart;

// ore-edk2-boot shared crates used as they are.
pub use bakery;
pub use sha256;

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
        entry >= base
            && entry % 4 == 0
            && entry.checked_add(4).is_some_and(|end| end <= base + size)
    }

    #[cfg(test)]
    mod region_tests {
        use super::*;
        const BASE: u64 = DRAM_BASE as u64;
        const SIZE: u64 = 4 << 30;
        #[test]
        fn usable_region_excludes_el3() {
            assert_eq!(
                payload_region(BASE, SIZE),
                Some((PAYLOAD_RAM_BASE as u64, SIZE - EL3_SIZE as u64))
            );
            assert!(valid_payload_entry(BASE, SIZE, PAYLOAD_RAM_BASE as u64));
            assert!(!valid_payload_entry(BASE, SIZE, BASE));
            assert!(!valid_payload_entry(
                BASE,
                SIZE,
                PAYLOAD_RAM_BASE as u64 - 4
            ));
        }
        #[test]
        fn entry_bounds_include_last_instruction_only() {
            assert!(valid_payload_entry(BASE, SIZE, BASE + SIZE - 4));
            assert!(!valid_payload_entry(BASE, SIZE, BASE + SIZE));
            assert!(!valid_payload_entry(BASE, SIZE, u64::MAX - 3));
            assert!(!valid_payload_entry(
                BASE,
                SIZE,
                PAYLOAD_RAM_BASE as u64 + 1
            ));
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

    use bootlayout::{Contract, KindRule, Place};
    /// The table format, decoding and the stage checks are the shared
    /// `bootlayout` crate; [`CONTRACT`] is the A7A's part.
    pub use bootlayout::{Entry, Error, Table, CRC_FIELD_OFFSET, MAX_ENTRIES, TABLE_SIZE};

    /// Version1 image-builder contract: images occupy the firmware gap,
    /// after the table, before the first partition at16MiB.
    pub const IMAGE_START: u64 = 0x0020_0000;
    pub const IMAGE_END: u64 = 0x0100_0000;
    pub const BLOCK_SIZE: u64 = 512;

    const KINDS: [KindRule; 4] = [
        KindRule {
            kind: KIND_MAIN,
            place: Place::Exact {
                addr: super::mem::MAIN_BASE as u64,
                max_size: Some(super::mem::MAIN_MAX_SIZE as u64),
            },
        },
        KindRule {
            kind: KIND_PAYLOAD,
            place: Place::Exact {
                addr: super::mem::PAYLOAD_BASE as u64,
                max_size: None,
            },
        },
        KindRule {
            kind: KIND_DTB,
            place: Place::AtLeast(super::mem::PAYLOAD_RAM_BASE as u64),
        },
        // Existing optional recovery format; never loaded by this path.
        KindRule {
            kind: KIND_VENDOR_SPL,
            place: Place::Recovery {
                addr: 0x47000,
                max_size: 0x4a000,
            },
        },
    ];

    /// The A7A table contract: magic, version, the firmware gap on the card,
    /// DRAM, this stage's address width, where each kind loads; main and the
    /// payload are required.
    pub const CONTRACT: Contract<'static> = Contract {
        magic: MAGIC,
        version: VERSION,
        image_start: IMAGE_START,
        image_end: IMAGE_END,
        block_size: BLOCK_SIZE,
        dram_base: super::mem::DRAM_BASE as u64,
        max_address: usize::MAX as u64,
        kinds: &KINDS,
        required: &[KIND_MAIN, KIND_PAYLOAD],
    };

    /// Decode the media frame (any alignment): length, header and the table
    /// CRC.
    pub fn decode(bytes: &[u8]) -> Result<Table, Error> {
        bootlayout::decode(&CONTRACT, bytes)
    }

    /// Validate every entry before any image is copied. The table CRC is
    /// checked by the media reader; this checks the image/loader contract.
    pub fn validate(table: &Table, dram_end: u64) -> Result<(), Error> {
        bootlayout::validate(&CONTRACT, table, dram_end)
    }

    /// Actual bt0 load orchestration: no callback can run until the entire
    /// table passes. Recovery-only entries are deliberately excluded.
    pub fn load_entries(
        table: &Table,
        dram_end: u64,
        load: impl FnMut(&Entry, usize, usize),
    ) -> Result<(), Error> {
        bootlayout::load_entries(&CONTRACT, table, dram_end, load)
    }

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
            assert!(matches!(
                decode(&frame[..TABLE_SIZE - 1]),
                Err(Error::Header)
            ));
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
/// `zlib.crc32`): the shared `storage::crc`.
pub use storage::crc::{crc32, update as crc32_update};

#[cfg(test)]
mod tests {
    #[test]
    fn crc32_check_value() {
        assert_eq!(super::crc32(b"123456789"), 0xcbf4_3926);
    }
}
