// SPDX-License-Identifier: GPL-2.0-only
//! DDR scheduler (DDRSCH) and channel address space, RK3588 TRM Part 2
//! Chapter 1 (§1.3.5 pp.18-20, §1.4 pp.21-26).
//!
//! The DDRSCH is the same scheduler family as the RK3568 "MSCH"
//! (ddrconf / ddrsize / ddrtiming / devtodev / ddrmode at the same
//! offsets; RK3568 RFC sdram_msch.h). Only DeviceConf and DeviceSize are
//! set here; the timing model registers keep their reset values (they
//! steer arbitration, the DDRCTL enforces the DRAM timings), and DdrMode
//! resets to the LPDDR4 16-bit, 32-byte burst setting already (p.25).
//!
//! DeviceConf 0 is D_R..R_RRBB_BCCC_CCCC_---- (8 banks, 10 columns, the
//! rank on top): the order the DDRCTL ADDRMAP of crate::ddr::ctl uses
//! (column, bank, row, rank) [I].
//!
//! Channel address space: CENTER_GRF_CON4 [8:4] = 0 interleaves the four
//! channels (the reset value); the other codes of Table 1-5 place channel
//! c at c x size ("linear"), which bt0 uses to test one channel at a time.

use crate::ddr::DramInfo;
use crate::regs::{base, ddrsch, wm, CENTER_GRF_SOC_CON4, CHANNEL_LAYOUT};
use regio::RegIo;

/// DeviceSize for ranks of `rank_mib` (granule 64 MiB, 8 bits per rank).
pub const fn device_size(rank_mib: u64, ranks: u8) -> u32 {
    let g = (rank_mib / 64) as u32 & 0xff;
    if ranks >= 2 {
        (g << 8) | g
    } else {
        g
    }
}

/// The CENTER_GRF_CON4 [8:4] code that places 4 channels of `mib` each
/// linearly (P2 Table 1-5); None when the table has no such size.
pub fn linear_code(mib: u64) -> Option<u32> {
    CHANNEL_LAYOUT
        .iter()
        .find(|(m, _)| *m == mib)
        .map(|(_, c)| *c)
}

/// The linear layout bt0 tests channels of `mib` each through: the
/// Table 1-5 size itself, or for 8 GiB channels the 4 GiB layout. Table
/// 1-5 prints 8 GB per channel for code 5'b00000, which is also the
/// 4-channel interleave code, so there is no linear 4-channel layout for
/// 8 GiB; with the 4 GiB code channel c answers at [c x 4 GiB, (c + 1) x
/// 4 GiB), which reaches the low 4 GiB of each channel, rank 0 with the
/// rank on top of the address map [I: the scheduler is taken to decode only
/// the channel base, not the channel size]. (Codes 5'b1xxxx map only
/// channels 2/3, 5'b10000 at 8 GB each: not used.)
pub fn test_window(mib: u64) -> Option<u64> {
    if linear_code(mib).is_some() {
        Some(mib)
    } else if mib == 8192 {
        Some(4096)
    } else {
        None
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Layout {
    /// 4-channel interleave (code 0).
    Interleave,
    /// Channel c at c x `mib` MiB.
    Linear { mib: u64 },
}

/// Program CENTER_GRF_CON4 [8:4].
pub fn set_layout<H: RegIo + ?Sized>(io: &mut H, l: Layout) -> Result<(), &'static str> {
    let code = match l {
        Layout::Interleave => 0,
        Layout::Linear { mib } => {
            linear_code(mib).ok_or("no linear layout for this channel size")?
        }
    };
    io.write32(CENTER_GRF_SOC_CON4, wm(4, 5, code));
    Ok(())
}

pub fn configure<H: RegIo + ?Sized>(io: &mut H, info: &DramInfo) {
    for (ch, c) in info.ch.iter().enumerate() {
        if let (true, Some(g)) = (c.present, c.geometry) {
            io.write32(base::DDRSCH[ch] + ddrsch::DEVICE_CONF, 0);
            io.write32(
                base::DDRSCH[ch] + ddrsch::DEVICE_SIZE,
                device_size(g.rank_mib, c.ranks),
            );
        }
    }
}
