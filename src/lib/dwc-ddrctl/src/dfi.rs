// SPDX-License-Identifier: GPL-2.0-only
//! DFI initialisation after PHY training, and the training results the
//! controller takes back: the rank-to-rank turnaround differences (CDD) and
//! the read-data enable.

use crate::regs::*;
use crate::Map;
use regio::{wait, RegIo, Timeout};

/// DFI initialisation of `channels` controllers: start it, wait for the PHY
/// to report dfi_init_complete, let the SoC release the PHY's isolation
/// (`release_phy`, e.g. the A733's R_PRCM +0x250 [0]), leave software
/// self-refresh and wait for normal operation. `channels` must be 1..=`max`.
pub fn dfi_init<H: RegIo + ?Sized>(
    io: &mut H,
    map: &Map,
    channels: usize,
    max: usize,
    timeout_us: u64,
    mut release_phy: impl FnMut(&mut H),
) -> Result<(), DfiError> {
    if !(1..=max).contains(&channels) {
        return Err(DfiError::Config(
            "DFI channel count outside controller instances",
        ));
    }
    let m = |ch, off| (map.main)(ch, off);
    for ch in 0..channels {
        io.write32(m(ch, SWCTL), 0);
    }
    for ch in 0..channels {
        io.set32(m(ch, DFIMISC), 0x1001);
    }
    for ch in 0..channels {
        io.set32(m(ch, DFIMISC), DFI_INIT_START);
    }
    for ch in 0..channels {
        io.write32(m(ch, SWCTL), 1);
    }
    for ch in 0..channels {
        wait(io, "SWSTAT.sw_done_ack", timeout_us, |io| {
            io.read32(m(ch, SWSTAT)) & 1 != 0
        })?;
    }
    for ch in 0..channels {
        wait(io, "DFISTAT.dfi_init_complete", timeout_us, |io| {
            io.read32(m(ch, DFISTAT)) & 1 != 0
        })?;
    }
    io.udelay(50);
    release_phy(io);
    io.udelay(2000);
    for ch in 0..channels {
        io.write32(m(ch, SWCTL), 0);
    }
    for ch in 0..channels {
        io.clr32(m(ch, DFIMISC), DFI_INIT_START);
    }
    for ch in 0..channels {
        io.clr32(m(ch, PWRCTL), PWRCTL_SELFREF_SW);
    }
    for ch in 0..channels {
        wait(io, "STAT.selfref_type", timeout_us, |io| {
            io.read32(m(ch, STAT)) & STAT_SELFREF_TYPE == 0
        })?;
        wait(io, "STAT.operating_mode", timeout_us, |io| {
            io.read32(m(ch, STAT)) & STAT_OPERATING_MODE == OPERATING_MODE_NORMAL
        })?;
    }
    io.udelay(50);
    for ch in 0..channels {
        io.clr32(m(ch, DFIMISC), DFI_INIT_COMPLETE_EN);
    }
    for ch in 0..channels {
        io.write32(m(ch, SWCTL), 1);
    }
    for ch in 0..channels {
        wait(io, "SWSTAT.sw_done_ack", timeout_us, |io| {
            io.read32(m(ch, SWSTAT)) & 1 != 0
        })?;
    }
    io.udelay(2);
    Ok(())
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DfiError {
    Config(&'static str),
    Timeout(&'static str),
}

impl From<Timeout> for DfiError {
    fn from(t: Timeout) -> Self {
        DfiError::Timeout(t.0)
    }
}

/// Trained critical delay differences between ranks, each the worst case
/// over the rank/channel combinations, in controller cycles.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Cdd {
    pub rr: u32,
    pub rr_alt: u32,
    pub ww: u32,
    pub ww_alt: u32,
    pub rw: u32,
    pub wr: u32,
}

fn sat_add(field: u32, add: u32, max: u32) -> u32 {
    (field + add).min(max)
}

/// Fold `cdd` into pstate 0's turnaround timings of channel `ch` (the
/// frequency-block registers +0xd04, +0xd08, DRAMSET1TMG2, +0x60, +0x24,
/// +0x584), saturating each field. The caller frames this with its
/// quasi-dynamic sequence (SWCTL 0 ... [`crate::sw_done`]).
pub fn fold_cdd<H: RegIo + ?Sized>(io: &mut H, map: &Map, ch: usize, cdd: &Cdd) {
    let f = |off| (map.freq)(ch, 0, off);
    let pair = |v: u32, hi: u32, lo: u32| {
        (v & 0xffff_0000) | (sat_add((v >> 8) & 0xff, hi, 0xff) << 8) | sat_add(v & 0xff, lo, 0xff)
    };
    io.modify32(f(0xd04), |v| pair(v, cdd.rr, cdd.rr_alt));
    io.modify32(f(0xd08), |v| pair(v, cdd.ww, cdd.ww_alt));
    io.modify32(f(0x08), |v| pair(v, cdd.rw, cdd.wr));
    io.modify32(f(0x60), |v| {
        (v & !0xff_0000) | (sat_add((v >> 16) & 0xff, cdd.ww, 0xff) << 16)
    });
    io.modify32(f(0x24), |v| {
        (v & !0xff) | sat_add(v & 0xff, cdd.ww_alt, 0xff)
    });
    io.modify32(f(0x584), |v| {
        (v & !0x1f_0000) | (sat_add((v >> 16) & 0xff, cdd.rr, 31) << 16)
    });
}

/// Pstate 0's DFI read-data enable (+0x588 [21:16]) from the trained PHY
/// read latency (computed by the board from its PHY CSRs).
pub fn set_rddata_en<H: RegIo + ?Sized>(io: &mut H, map: &Map, ch: usize, rden: u32) {
    io.modify32((map.freq)(ch, 0, 0x588), |v| {
        (v & !0x3f_0000) | (rden << 16)
    });
}
