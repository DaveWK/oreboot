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

    pub const TABLE_SIZE: usize = core::mem::size_of::<Table>();
    pub const CRC_FIELD_OFFSET: usize = 16;
    const _: () = assert!(TABLE_SIZE == 24 + 32 * MAX_ENTRIES);
    const _: () = assert!(TABLE_SIZE <= 512);
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
