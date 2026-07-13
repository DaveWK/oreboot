// SPDX-License-Identifier: GPL-2.0-only
// Verbatim copy of ore-edk-boot-radxa-a7a ed42550 (oreboot tree a803f4f780d7)
// src/mainboard/radxa/cubie-a7a/common/src/lib.rs module `mem` without its
// #[cfg(test)] module: the entry-point rule the board's PSCI uses.
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

    /// Where the EDK2 FD is loaded and entered (its PcdFdBaseAddress).
    pub const PAYLOAD_BASE: usize = 0x4100_0000;
}
