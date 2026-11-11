// SPDX-License-Identifier: GPL-2.0-only
//! The DRAM step, behind one interface for both bring-up paths.
//!
//! - DRAM already up (the stepping-stone image or the MaskROM USB loader:
//!   Rockchip's rkbin DDR init ran first): bt0 only reads the documented
//!   OS_REG2..5 geometry handoff (crate rk3588::osreg).
//! - Own init (feature `own-ddr`, off by default, work in progress): a call
//!   into `rk3588::ddr::init`, which writes OS_REG2..5 itself, then the
//!   same readback.
//!
//! Everything after this step (dram-info, FIT placement, the BL31
//! handoff) only sees [`Dram`].

use rk3588::osreg::Pair;
use rk3588::{ddr, osreg};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Source {
    /// OS_REG2..5 were valid at entry: something before bt0 initialised DRAM.
    Handoff,
    /// bt0 ran `rk3588::ddr::init` (feature `own-ddr`).
    #[cfg_attr(not(feature = "own-ddr"), allow(dead_code))]
    OwnInit,
}

#[derive(Clone, Copy, Debug)]
pub struct Dram {
    pub source: Source,
    /// OS_REG2/3 (channels 0/1) and OS_REG4/5 (channels 2/3).
    pub pairs: [Pair; 2],
}

impl Dram {
    pub fn total_bytes(&self) -> u64 {
        (self.pairs[0].size_mib() + self.pairs[1].size_mib()) << 20
    }
    pub fn type_name(&self) -> &'static str {
        osreg::type_name(self.pairs[0].dram_type)
    }
    pub fn channels(&self) -> usize {
        (self.pairs[0].channels + self.pairs[1].channels) as usize
    }
    pub fn ranks(&self) -> [u8; 4] {
        let [a, b] = &self.pairs;
        [
            a.ch[0].rank as u8,
            a.ch[1].rank as u8,
            b.ch[0].rank as u8,
            b.ch[1].rank as u8,
        ]
    }
}

/// Bring DRAM up (or find it up). Err carries a one-line reason.
pub fn bring_up<H: regio::RegIo + ?Sized>(io: &mut H) -> Result<Dram, &'static str> {
    if let Some((a, b)) = ddr::handoff_present(io) {
        return Ok(Dram {
            source: Source::Handoff,
            pairs: [a, b],
        });
    }
    own_init(io)
}

#[cfg(feature = "own-ddr")]
fn own_init<H: regio::RegIo + ?Sized>(io: &mut H) -> Result<Dram, &'static str> {
    ddr::init(io, &ddr::Config::yy3588_default()).map_err(|_| "own DDR init failed")?;
    let (a, b) =
        ddr::handoff_present(io).ok_or("own DDR init wrote an implausible OS_REG handoff")?;
    Ok(Dram {
        source: Source::OwnInit,
        pairs: [a, b],
    })
}

#[cfg(not(feature = "own-ddr"))]
fn own_init<H: regio::RegIo + ?Sized>(_io: &mut H) -> Result<Dram, &'static str> {
    Err("no OS_REG handoff and own DDR init is not built in (feature own-ddr, WIP)")
}
