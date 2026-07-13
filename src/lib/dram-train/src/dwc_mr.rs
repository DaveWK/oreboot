// SPDX-License-Identifier: GPL-2.0-only
//! Mode-register access through a Synopsys DesignWare DDR controller
//! (uMCTL2/DDRCTL: MRCTRL0, MRCTRL1, MRSTAT).
//!
//! The register offsets are the DesignWare ones the board passes in
//! [`Ctrl`]; where each channel's controller sits and where an MRR result
//! lands are SoC-specific (A733: MCTL_COM +0x2c/+0x40 [15:8]).
//!
//! First written for the A733 (ore-edk-boot-radxa-a7a `mr.rs`).

use crate::io::{wait, RegIo, Timeout};

const MR_WR: u32 = 1 << 31;
const MR_TYPE_READ: u32 = 1;
/// MRCTRL0.mr_rank: rank 0 is bit 4, rank 1 bit 5.
const fn rank_bit(rank: u32) -> u32 {
    1 << (4 + rank)
}

/// One SoC's DesignWare controllers.
#[derive(Clone, Copy, Debug)]
pub struct Ctrl {
    /// Address of controller register `off` of channel `ch`.
    pub reg: fn(ch: usize, off: usize) -> usize,
    pub mrctrl0: usize,
    pub mrctrl1: usize,
    pub mrstat: usize,
    /// Where the MRR byte of channel `ch` lands: (address, bit shift).
    pub mrr_data: fn(ch: usize) -> (usize, u32),
    pub channels: usize,
    pub ranks: u32,
    pub timeout_us: u64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MrError {
    /// Channel, rank, MR or value outside the command fields.
    Config(&'static str),
    Timeout(&'static str),
}

impl From<Timeout> for MrError {
    fn from(t: Timeout) -> Self {
        MrError::Timeout(t.0)
    }
}

/// Validate the software command fields before any controller access. A legal
/// numeric channel/rank is not proof that it exists in the selected topology;
/// callers must separately establish that and the DRAM command timing.
fn validate(c: &Ctrl, ch: usize, rank: u32, mr: u32, value: u32) -> Result<(), MrError> {
    if ch >= c.channels || rank >= c.ranks || mr > 0xff || value > 0xff {
        return Err(MrError::Config(
            "MR channel/rank/address/data outside command fields",
        ));
    }
    Ok(())
}

fn idle<H: RegIo + ?Sized>(hw: &mut H, c: &Ctrl, ch: usize) -> Result<(), MrError> {
    let stat = (c.reg)(ch, c.mrstat);
    Ok(wait(hw, "MRSTAT.mr_wr_busy", c.timeout_us, |hw| {
        hw.read32(stat) & 1 == 0
    })?)
}

fn done<H: RegIo + ?Sized>(hw: &mut H, c: &Ctrl, ch: usize) -> Result<(), MrError> {
    let ctrl0 = (c.reg)(ch, c.mrctrl0);
    Ok(wait(hw, "MRCTRL0.mr_wr", c.timeout_us, |hw| {
        hw.read32(ctrl0) & MR_WR == 0
    })?)
}

/// Write one explicitly selected rank. Does not enter a training mode, infer
/// applicability, or prove MR timing.
pub fn write_rank<H: RegIo + ?Sized>(
    hw: &mut H,
    c: &Ctrl,
    ch: usize,
    rank: u32,
    mr: u32,
    value: u32,
) -> Result<(), MrError> {
    validate(c, ch, rank, mr, value)?;
    idle(hw, c, ch)?;
    hw.write32((c.reg)(ch, c.mrctrl0), rank_bit(rank));
    hw.write32((c.reg)(ch, c.mrctrl1), (mr << 8) | value);
    hw.write32((c.reg)(ch, c.mrctrl0), MR_WR | rank_bit(rank));
    done(hw, c, ch)
}

/// Write MR `mr` of ranks 0 and 1 of channel `ch` (rank 0 gets `v0`, rank 1
/// `v1`). All fields are validated before the first command. A rank-1
/// timeout does not undo rank 0: callers must retain that partial failure.
pub fn write<H: RegIo + ?Sized>(
    hw: &mut H,
    c: &Ctrl,
    ch: usize,
    mr: u32,
    v0: u32,
    v1: u32,
) -> Result<(), MrError> {
    validate(c, ch, 0, mr, v0)?;
    validate(c, ch, 1, mr, v1)?;
    write_rank(hw, c, ch, 0, mr, v0)?;
    write_rank(hw, c, ch, 1, mr, v1)
}

/// Read MR `mr` of `rank` of channel `ch`.
pub fn read<H: RegIo + ?Sized>(
    hw: &mut H,
    c: &Ctrl,
    ch: usize,
    rank: u32,
    mr: u32,
) -> Result<u32, MrError> {
    validate(c, ch, rank, mr, 0)?;
    idle(hw, c, ch)?;
    hw.write32((c.reg)(ch, c.mrctrl0), rank_bit(rank) | MR_TYPE_READ);
    hw.write32((c.reg)(ch, c.mrctrl1), mr << 8);
    hw.write32(
        (c.reg)(ch, c.mrctrl0),
        MR_WR | rank_bit(rank) | MR_TYPE_READ,
    );
    done(hw, c, ch)?;
    let (reg, shift) = (c.mrr_data)(ch);
    Ok((hw.read32(reg) >> shift) & 0xff)
}
