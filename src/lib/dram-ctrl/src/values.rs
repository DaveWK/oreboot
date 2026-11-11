// SPDX-License-Identifier: GPL-2.0-only
//! Named timing quantities in controller cycles.

use jedec_lpddr::lp4::timing::Timings;

/// The timing quantities controllers program (in controller cycles).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Name {
    Ras,
    RasMax,
    Faw,
    Wr2Pre,
    Rc,
    Rd2Pre,
    Xp,
    Wr2Rd,
    Rd2Wr,
    ReadLatency,
    WriteLatency,
    Rp,
    Rrd,
    Ccd,
    Rcd,
    Rfc,
    Refi,
    Wtr,
    Rtp,
    Wr,
    Cke,
    Mrd,
    Mrw,
    ZqCal,
    Xsr,
}

pub const NAMES: usize = Name::Xsr as usize + 1;

/// A set of named values; unset ones are refused when packed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Values {
    v: [Option<u32>; NAMES],
}

impl Default for Values {
    fn default() -> Self {
        Self { v: [None; NAMES] }
    }
}

impl Values {
    pub fn get(&self, n: Name) -> Option<u32> {
        self.v[n as usize]
    }
    pub fn set(&mut self, n: Name, x: u32) -> &mut Self {
        self.v[n as usize] = Some(x);
        self
    }

    /// The JEDEC minima of an LPDDR4 part at DRAM clock `ck_mhz`, in cycles
    /// of a controller running at 1/`ratio` of it (DFI 1:1, 1:2, 1:4: each
    /// value rounded up to whole controller cycles); tRC = tRAS + tRP,
    /// tREFI rounded down. Latencies and turnarounds (Wr2Pre, Rd2Wr, RL,
    /// WL...) are the board's: they depend on its MR choices.
    pub fn from_lp4(t: &Timings, ck_mhz: u32, ratio: u32) -> Option<Values> {
        if !matches!(ratio, 1 | 2 | 4) || ck_mhz == 0 {
            return None;
        }
        let c = |n: u32| n.div_ceil(ratio);
        let mut v = Values::default();
        let ras = c(t.t_ras.cycles(ck_mhz));
        let rp = c(t.t_rp.cycles(ck_mhz));
        v.set(Name::Ras, ras)
            .set(Name::Rp, rp)
            .set(Name::Rc, ras + rp)
            .set(Name::Rcd, c(t.t_rcd.cycles(ck_mhz)))
            .set(Name::Wr, c(t.t_wr.cycles(ck_mhz)))
            .set(Name::Rfc, c(t.t_rfc.cycles(ck_mhz)))
            .set(Name::Faw, c(t.t_faw.cycles(ck_mhz)))
            .set(Name::Wtr, c(t.t_wtr.cycles(ck_mhz)))
            .set(Name::Rrd, c(t.t_rrd.cycles(ck_mhz)))
            .set(Name::Ccd, c(t.t_ccd.cycles(ck_mhz)))
            .set(Name::Refi, t.t_refi.cycles(ck_mhz) / ratio);
        Some(v)
    }
}
