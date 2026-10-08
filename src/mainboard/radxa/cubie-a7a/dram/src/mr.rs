//! LPDDR5 mode register access through the controller (MRCTRL0/1, MRSTAT).

use crate::ctrl::{ddrc, MRCTRL0, MRCTRL1, MRSTAT};
use crate::hw::{wait, Error, Hw};

const MR_WR: u32 = 1 << 31;
const MR_TYPE_READ: u32 = 1;
/// MRCTRL0.mr_rank: rank 0 is bit 4, rank 1 bit 5.
const fn rank_bit(rank: u32) -> u32 {
    1 << (4 + rank)
}
const TIMEOUT_US: u64 = 100_000;

fn idle<H: Hw>(hw: &mut H, ch: usize) -> Result<(), Error> {
    wait(hw, "MRSTAT.mr_wr_busy", TIMEOUT_US, |hw| {
        hw.read32(ddrc(ch, MRSTAT)) & 1 == 0
    })
}

fn done<H: Hw>(hw: &mut H, ch: usize) -> Result<(), Error> {
    wait(hw, "MRCTRL0.mr_wr", TIMEOUT_US, |hw| {
        hw.read32(ddrc(ch, MRCTRL0)) & MR_WR == 0
    })
}

/// Write MR `mr` of both ranks of channel `ch` (rank 0 gets `v0`, rank 1
/// `v1`).
pub fn write<H: Hw>(hw: &mut H, ch: usize, mr: u32, v0: u32, v1: u32) -> Result<(), Error> {
    for (rank, v) in [(0, v0), (1, v1)] {
        idle(hw, ch)?;
        hw.write32(ddrc(ch, MRCTRL0), rank_bit(rank));
        hw.write32(ddrc(ch, MRCTRL1), (mr << 8) | v);
        hw.write32(ddrc(ch, MRCTRL0), MR_WR | rank_bit(rank));
        done(hw, ch)?;
    }
    Ok(())
}

/// Read MR `mr` of `rank` of channel `ch`. The value arrives in MCTL_COM:
/// +0x2c [15:8] for channel 0, +0x40 [15:8] for channel 1.
pub fn read<H: Hw>(hw: &mut H, ch: usize, rank: u32, mr: u32) -> Result<u32, Error> {
    idle(hw, ch)?;
    hw.write32(ddrc(ch, MRCTRL0), rank_bit(rank) | MR_TYPE_READ);
    hw.write32(ddrc(ch, MRCTRL1), mr << 8);
    hw.write32(ddrc(ch, MRCTRL0), MR_WR | rank_bit(rank) | MR_TYPE_READ);
    done(hw, ch)?;
    let reg = if ch == 0 { 0x0a02_002c } else { 0x0a02_0040 };
    Ok((hw.read32(reg) >> 8) & 0xff)
}
