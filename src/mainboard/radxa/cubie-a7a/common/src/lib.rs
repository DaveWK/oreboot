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

pub mod uart;

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
    /// Where the EDK2 FD is loaded and entered (its PcdFdBaseAddress).
    pub const PAYLOAD_BASE: usize = 0x4100_0000;
}

/// Boot media layout table, written by the image tool at a fixed offset on
/// the SD card. bt0 reads it to find what to load where.
pub mod layout {
    /// Byte offset of the table on the boot medium (sector 2040, just below
    /// 1 MiB, after the room reserved for the bt0 eGON image at 8 KiB).
    pub const TABLE_OFFSET: u32 = 0x000f_f000;
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

pub const HANDOFF_MAGIC: u32 = u32::from_le_bytes(*b"A7AH");
pub const HANDOFF_VERSION: u32 = 1;

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
