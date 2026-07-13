// SPDX-License-Identifier: GPL-2.0-only
//! Synopsys DesignWare DDR controller (DDRCTL with LPDDR5 support, the
//! uMCTL2 register conventions) for any SoC that integrates it.
//!
//! What is the controller's (register offsets and fields, quasi-dynamic
//! programming, the address map encoding, DRAMSET timing fields, the DFI
//! init handshake, the post-training turnaround fold, mode-register access)
//! lives here. What is the SoC's stays in the board: where each channel's
//! register blocks sit ([`Map`]), the glue around the controller (the A733's
//! MCTL_COM and its +0xb84/+0xb88 request), PHY isolation, and the values of
//! its parameter sets.
//!
//! First written for the Allwinner A733 (ore-edk-boot-radxa-a7a
//! `dram/src/{ctrl,timing,post,mr}.rs`).
#![no_std]

pub mod addrmap;
pub mod dfi;
pub mod regs;
pub mod timing;

/// Mode-register access (MRCTRL0/1, MRSTAT). Its source is
/// `dram_train::dwc_mr` until the oreboot overlay links this crate.
pub use dram_train::dwc_mr as mr;

use regio::{wait, RegIo, Timeout};

/// Where one SoC's controllers are: each channel's main register block, its
/// frequency-set (pstate) timing blocks and its address-map block.
#[derive(Clone, Copy, Debug)]
pub struct Map {
    /// Main register `off` of channel `ch` (MSTR0, STAT, MRCTRL, DFI*, SWCTL...).
    pub main: fn(ch: usize, off: usize) -> usize,
    /// Frequency-set `ps` register `off` of channel `ch` (DRAMSET1TMGn...).
    pub freq: fn(ch: usize, ps: usize, off: usize) -> usize,
    /// ADDRMAPn of channel `ch`.
    pub addrmap: fn(ch: usize, n: usize) -> usize,
}

/// Quasi-dynamic register writes are done: set SWCTL.sw_done and wait for
/// SWSTAT.sw_done_ack.
pub fn sw_done<H: RegIo + ?Sized>(
    io: &mut H,
    map: &Map,
    ch: usize,
    timeout_us: u64,
) -> Result<(), Timeout> {
    io.write32((map.main)(ch, regs::SWCTL), 1);
    wait(io, "SWSTAT.sw_done_ack", timeout_us, |io| {
        io.read32((map.main)(ch, regs::SWSTAT)) & 1 != 0
    })
}

/// MSTR0 for an LPDDR5 controller with burst length 16: active ranks (1 or
/// 2) and the half-bus-width bit.
pub const fn mstr0_lpddr5_bl16(ranks: u32, half_bus: bool) -> u32 {
    const LPDDR5_BL16: u32 = 0x0008_0008;
    let active = if ranks >= 2 { 3 } else { 1 };
    LPDDR5_BL16 | (active << 24) | ((half_bus as u32) << 12)
}

/// ODTMAP: each rank's ODT follows its own writes (two ranks) or rank 0 only.
pub const fn odtmap(ranks: u32) -> u32 {
    if ranks >= 2 {
        0x303
    } else {
        0x1
    }
}
