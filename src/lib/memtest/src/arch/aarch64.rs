// SPDX-License-Identifier: GPL-2.0-only
//! AArch64 EL3 flat map for memory tests: with the MMU off every access is
//! Device memory and a multi-GiB test takes minutes. One level-1 table of
//! 1 GiB blocks (39-bit VA) maps block 0 (MMIO) as Device-nGnRnE and
//! execute-never, every DRAM block as Normal non-cacheable (MAIR attr1 =
//! 0x44), execute-never except the block the firmware runs from. The
//! D-cache stays off, so every access reaches DRAM and leaving needs no
//! cache maintenance; the previous SCTLR/MAIR/TCR/TTBR0 are restored.
//!
//! From ore-edk-boot-radxa-a7a oreboot main/src/memtest.rs (`Map::enter`,
//! `Drop`), whose first version was the wip-memtest design (a30b661). The
//! table is computed by [`l1_table`] (host-tested); the register writes are
//! [`El3FlatMap`] (AArch64 targets only).

/// Level-1 block descriptor bits.
pub const BLOCK: u64 = 0b01;
pub const AF: u64 = 1 << 10;
pub const SH_INNER: u64 = 3 << 8;
pub const XN: u64 = 1 << 54;
/// AttrIndx 1 (MAIR attr1).
pub const ATTR1: u64 = 1 << 2;
/// MAIR: attr0 Device-nGnRnE (0x00), attr1 Normal inner/outer non-cacheable.
pub const MAIR: u64 = 0x4400;
/// TCR_EL3: T0SZ 25 (39-bit VA, level-1 start), non-cacheable walks, 4 KiB
/// granule, PS 40 bits, RES1 bits 23 and 31.
pub const TCR: u64 = 25 | (2 << 16) | (1 << 23) | (1 << 31);

/// Fill `table` for DRAM up to `end` (bytes): block 0 Device XN, blocks
/// 1..ceil(end / 1 GiB) Normal NC, XN except `exec` (the block holding the
/// running firmware, if any); the rest invalid. `end` must not exceed 512
/// GiB.
pub fn l1_table(table: &mut [u64; 512], end: u64, exec: Option<usize>) -> Result<(), &'static str> {
    let blocks = end.div_ceil(1 << 30);
    if blocks > 512 {
        return Err("DRAM end beyond the 39-bit flat map");
    }
    table.fill(0);
    table[0] = BLOCK | AF | XN;
    for (gib, e) in table.iter_mut().enumerate().take(blocks as usize).skip(1) {
        *e = (gib as u64) << 30 | BLOCK | AF | SH_INNER | ATTR1;
        if Some(gib) != exec {
            *e |= XN;
        }
    }
    Ok(())
}

/// The flat map, active while this value lives (EL3, MMU and D-cache off on
/// entry; restored on drop).
#[cfg(target_arch = "aarch64")]
pub struct El3FlatMap {
    sctlr: u64,
    mair: u64,
    tcr: u64,
    ttbr: u64,
}

#[cfg(target_arch = "aarch64")]
impl El3FlatMap {
    /// Switch to `table` (filled by [`l1_table`], 4 KiB aligned, kept alive
    /// by the caller while the map is active).
    ///
    /// # Safety
    /// EL3 on the boot core, nothing else running; `table` must outlive the
    /// returned value and must map the running code executable.
    pub unsafe fn enter(table: &[u64; 512]) -> Result<Self, &'static str> {
        use core::arch::asm;
        let (sctlr, mair, tcr, ttbr): (u64, u64, u64, u64);
        asm!("mrs {s}, sctlr_el3", "mrs {m}, mair_el3", "mrs {t}, tcr_el3", "mrs {b}, ttbr0_el3",
             s = out(reg) sctlr, m = out(reg) mair, t = out(reg) tcr, b = out(reg) ttbr);
        if sctlr & 5 != 0 {
            return Err("MMU or D-cache already enabled");
        }
        if table.as_ptr() as usize & 0xfff != 0 {
            return Err("table not 4 KiB aligned");
        }
        asm!("dsb sy", "msr mair_el3, {m}", "msr tcr_el3, {t}", "msr ttbr0_el3, {b}", "isb",
             "tlbi alle3", "dsb sy", "isb", "msr sctlr_el3, {s}", "isb",
             m = in(reg) MAIR, t = in(reg) TCR, b = in(reg) table.as_ptr() as u64, s = in(reg) (sctlr | 1));
        Ok(Self {
            sctlr,
            mair,
            tcr,
            ttbr,
        })
    }
}

#[cfg(target_arch = "aarch64")]
impl Drop for El3FlatMap {
    fn drop(&mut self) {
        // SAFETY: restores the registers read in enter, in the order the
        // A7A firmware does (MMU off first, then translation registers).
        unsafe {
            core::arch::asm!("dsb sy", "msr sctlr_el3, {s}", "isb", "tlbi alle3", "dsb sy", "isb",
                 "msr ttbr0_el3, {b}", "msr tcr_el3, {t}", "msr mair_el3, {m}", "isb",
                 s = in(reg) self.sctlr, b = in(reg) self.ttbr, t = in(reg) self.tcr, m = in(reg) self.mair);
        }
    }
}
