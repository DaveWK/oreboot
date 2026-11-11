// SPDX-License-Identifier: GPL-2.0-only
//! LPDDR4 mode-register write/read through the DDRCTL.
//!
//! TRM Part 2: MRCTRL0 mr_wr [31] (trigger), mrr_done_clr [24], mr_rank
//! [5:4] (1 = rank 0, 2 = rank 1), mr_type [0] (1 = read) (p.112);
//! MRCTRL1 "For LPDDR4/5, [15:8] MR Address, [7:0] MR data for writes"
//! (p.114); MRSTAT mrr_done [16], mr_wr_busy [0] (p.114); MRRDATA0 the
//! read data (p.115). The MR value of an LPDDR4 MRR is returned on DQ[7:0]
//! of each byte lane: the low byte of MRRDATA0 is taken.

use crate::regs::{base, ctl};
use regio::{wait, RegIo, Timeout};

const TIMEOUT_US: u64 = 10_000;

fn idle<H: RegIo + ?Sized>(io: &mut H, b: usize) -> Result<(), Timeout> {
    wait(io, "MR interface busy", TIMEOUT_US, |io| {
        io.read32(b + ctl::MRSTAT) & ctl::MRSTAT_WR_BUSY == 0
    })
}

/// MRW `op` to `ma` on `rank` (0 or 1) of channel `ch`.
pub fn write<H: RegIo + ?Sized>(
    io: &mut H,
    ch: usize,
    rank: u8,
    ma: u8,
    op: u8,
) -> Result<(), Timeout> {
    let b = base::DDRCTL[ch];
    idle(io, b)?;
    io.write32(b + ctl::MRCTRL1, ((ma as u32) << 8) | op as u32);
    let sel = (1u32 << rank) << 4;
    io.write32(b + ctl::MRCTRL0, sel);
    io.write32(b + ctl::MRCTRL0, sel | ctl::MRCTRL0_MR_WR);
    idle(io, b)
}

/// MRR of `ma` on `rank` of channel `ch`: the whole MRRDATA0 word. On a
/// 16-bit LPDDR4 channel the MR byte comes back on DQ[7:0] and DQ[15:8];
/// a board that swaps DQ bits inside a byte scrambles it, so callers keep
/// the raw word for the log.
pub fn read_raw<H: RegIo + ?Sized>(
    io: &mut H,
    ch: usize,
    rank: u8,
    ma: u8,
) -> Result<u32, Timeout> {
    let b = base::DDRCTL[ch];
    idle(io, b)?;
    io.write32(b + ctl::MRCTRL1, (ma as u32) << 8);
    let sel = ((1u32 << rank) << 4) | ctl::MRCTRL0_READ;
    io.write32(b + ctl::MRCTRL0, sel);
    io.write32(b + ctl::MRCTRL0, sel | ctl::MRCTRL0_MR_WR);
    let r = wait(io, "MRR done", TIMEOUT_US, |io| {
        io.read32(b + ctl::MRSTAT) & ctl::MRSTAT_MRR_DONE != 0
    });
    let v = io.read32(b + ctl::MRRDATA0);
    io.write32(b + ctl::MRCTRL0, ctl::MRCTRL0_MRR_DONE_CLR);
    io.write32(b + ctl::MRCTRL0, 0);
    r.map(|_| v)
}

/// MRR of `ma` on `rank` of channel `ch`: the MR byte (MRRDATA0 [7:0]).
pub fn read<H: RegIo + ?Sized>(io: &mut H, ch: usize, rank: u8, ma: u8) -> Result<u8, Timeout> {
    read_raw(io, ch, rank, ma).map(|v| v as u8)
}
