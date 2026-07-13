// SPDX-License-Identifier: GPL-2.0-only
//! LPDDR4/4X mode-register encoders and decoders.
//!
//! Field layouts as transcribed in LiteDRAM
//! 74522f715d7b10163ef6f6caa09887a90874be83 `litedram/init.py`
//! (`get_lpddr4_phy_init_sequence`): MR1, MR2, MR3, MR11, MR12/MR14. Only
//! the meanings that source states are named; other settings are carried as
//! the raw bit. The latency codes are the row index of the JEDEC LPDDR4/4X
//! frequency ranges ([`super::timing::FREQ_RANGES`]). Not here, for lack of a cited layout:
//! MR4 (refresh rate), MR13 (FSP-WR/FSP-OP, VRCG, data mask; the source only
//! states that 0 means "data mask enabled, frequency set point 0"), MR22
//! (ODTE).

use super::timing::FREQ_RANGES;
use super::{VREF_RANGE1, VREF_SETTING_MAX};

/// Termination and drive strength codes (MR3 PDDS, MR11 DQ/CA ODT).
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Rzq {
    Disabled = 0,
    Div1 = 1,
    Div2 = 2,
    Div3 = 3,
    Div4 = 4,
    Div5 = 5,
    Div6 = 6,
}

impl Rzq {
    pub const fn from_code(c: u8) -> Option<Rzq> {
        Some(match c {
            0 => Rzq::Disabled,
            1 => Rzq::Div1,
            2 => Rzq::Div2,
            3 => Rzq::Div3,
            4 => Rzq::Div4,
            5 => Rzq::Div5,
            6 => Rzq::Div6,
            _ => return None,
        })
    }
}

/// MR1 OP[6:4]: the frequency-range row whose nWR is `nwr`.
pub const fn nwr_code(nwr: u8) -> Option<u8> {
    let mut i = 0;
    while i < FREQ_RANGES.len() {
        if FREQ_RANGES[i].nwr == nwr {
            return Some(i as u8);
        }
        i += 1;
    }
    None
}

/// MR2 OP[2:0]: the frequency-range row whose RL (DBI-RD disabled) is `rl`.
pub const fn rl_code(rl: u8) -> Option<u8> {
    let mut i = 0;
    while i < FREQ_RANGES.len() {
        if FREQ_RANGES[i].rl == rl {
            return Some(i as u8);
        }
        i += 1;
    }
    None
}

/// MR2 OP[5:3]: the frequency-range row whose WL set A is `wl`.
pub const fn wl_a_code(wl: u8) -> Option<u8> {
    let mut i = 0;
    while i < FREQ_RANGES.len() {
        if FREQ_RANGES[i].wl_a == wl {
            return Some(i as u8);
        }
        i += 1;
    }
    None
}

/// MR1. OP[1:0] burst length (0: BL16, 1: BL32, 2: on the fly), OP[2]
/// write preamble (1: 2 tCK), OP[3] read preamble (0: static), OP[6:4] nWR
/// code, OP[7] read postamble (0: 0.5 tCK).
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Mr1 {
    pub bl: u8,
    pub wr_pre: bool,
    pub rd_pre: bool,
    pub nwr_code: u8,
    pub rd_pst: bool,
}

impl Mr1 {
    pub const fn encode(self) -> Option<u8> {
        if self.bl > 2 || self.nwr_code > 7 {
            return None;
        }
        Some(
            self.bl
                | (self.wr_pre as u8) << 2
                | (self.rd_pre as u8) << 3
                | self.nwr_code << 4
                | (self.rd_pst as u8) << 7,
        )
    }
    pub const fn decode(op: u8) -> Option<Mr1> {
        if op & 3 == 3 {
            return None;
        }
        Some(Mr1 {
            bl: op & 3,
            wr_pre: op & 4 != 0,
            rd_pre: op & 8 != 0,
            nwr_code: (op >> 4) & 7,
            rd_pst: op & 0x80 != 0,
        })
    }
}

/// MR2. OP[2:0] RL code (DBI-RD disabled), OP[5:3] WL code, OP[6] WL set
/// (0: set A), OP[7] write leveling (0: disabled).
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Mr2 {
    pub rl_code: u8,
    pub wl_code: u8,
    pub wl_set_b: bool,
    pub write_leveling: bool,
}

impl Mr2 {
    pub const fn encode(self) -> Option<u8> {
        if self.rl_code > 7 || self.wl_code > 7 {
            return None;
        }
        Some(
            self.rl_code
                | self.wl_code << 3
                | (self.wl_set_b as u8) << 6
                | (self.write_leveling as u8) << 7,
        )
    }
    pub const fn decode(op: u8) -> Mr2 {
        Mr2 {
            rl_code: op & 7,
            wl_code: (op >> 3) & 7,
            wl_set_b: op & 0x40 != 0,
            write_leveling: op & 0x80 != 0,
        }
    }
}

/// MR3: OP[5:3] pull-down drive strength; OP[0] (1 by default), OP[1],
/// OP[2], OP[6], OP[7] (0 by default) as raw bits.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Mr3 {
    pub pdds: Rzq,
    /// OP[7:6] and OP[2:0] as written; OP[5:3] must be 0 here.
    pub other: u8,
}

impl Mr3 {
    /// The source's defaults: OP[0] = 1, the rest 0, with `pdds`.
    pub const fn defaults(pdds: Rzq) -> Mr3 {
        Mr3 { pdds, other: 1 }
    }
    pub const fn encode(self) -> Option<u8> {
        if self.other & 0x38 != 0 {
            return None;
        }
        Some(self.other | (self.pdds as u8) << 3)
    }
    pub const fn decode(op: u8) -> Option<Mr3> {
        match Rzq::from_code((op >> 3) & 7) {
            Some(pdds) => Some(Mr3 {
                pdds,
                other: op & !0x38,
            }),
            None => None,
        }
    }
}

/// MR11: OP[2:0] DQ ODT, OP[6:4] CA ODT.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Mr11 {
    pub dq_odt: Rzq,
    pub ca_odt: Rzq,
}

impl Mr11 {
    pub const fn encode(self) -> u8 {
        self.dq_odt as u8 | (self.ca_odt as u8) << 4
    }
    pub const fn decode(op: u8) -> Option<Mr11> {
        match (Rzq::from_code(op & 7), Rzq::from_code((op >> 4) & 7)) {
            (Some(dq_odt), Some(ca_odt)) if op & 0x88 == 0 => Some(Mr11 { dq_odt, ca_odt }),
            _ => None,
        }
    }
}

/// MR12 VREF(CA) / MR14 VREF(DQ): OP[5:0] setting (0..=50), OP[6] range.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Vref {
    pub range1: bool,
    pub setting: u8,
}

impl Vref {
    pub const fn encode(self) -> Option<u8> {
        if self.setting > VREF_SETTING_MAX {
            return None;
        }
        Some(self.setting | if self.range1 { VREF_RANGE1 } else { 0 })
    }
    pub const fn decode(op: u8) -> Option<Vref> {
        if op & 0x80 != 0 || op & 0x3f > VREF_SETTING_MAX {
            return None;
        }
        Some(Vref {
            range1: op & VREF_RANGE1 != 0,
            setting: op & 0x3f,
        })
    }
}

/// MR13 (LPDDR4): OP[7] FSP-OP, OP[6] FSP-WR, OP[5] DMD (data mask
/// disable), OP[4] PRO, OP[3] VRCG, OP[0] CBT; OP[2:1] carried raw (no
/// cited meaning). Layout: coreboot `src/vendorcode/mediatek/mt8195/include/
/// dramc_pi_api.h` fec3d608 lines 1610-1615 (`MR13_*`), BSD-3-Clause
/// (orchestrator `references/datasheets/open-source/coreboot_mt8195_
/// dramc_pi_api.h`, sha256 4a8d1e86...).
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Mr13 {
    pub fsp_op: bool,
    pub fsp_wr: bool,
    pub dmd: bool,
    pub pro: bool,
    pub vrcg: bool,
    pub cbt: bool,
    /// OP[2:1], raw.
    pub raw_2_1: u8,
}

/// MR number of MR13.
pub const MR13: u8 = 13;

impl Mr13 {
    pub const fn encode(self) -> u8 {
        (self.fsp_op as u8) << 7
            | (self.fsp_wr as u8) << 6
            | (self.dmd as u8) << 5
            | (self.pro as u8) << 4
            | (self.vrcg as u8) << 3
            | (self.raw_2_1 & 3) << 1
            | self.cbt as u8
    }
    pub const fn decode(op: u8) -> Mr13 {
        Mr13 {
            fsp_op: op & 0x80 != 0,
            fsp_wr: op & 0x40 != 0,
            dmd: op & 0x20 != 0,
            pro: op & 0x10 != 0,
            vrcg: op & 0x08 != 0,
            cbt: op & 0x01 != 0,
            raw_2_1: (op >> 1) & 3,
        }
    }
}

// ---------------------------------------------------------------------------
// P1-40: MR4, MR18/MR19, MR22, MR23, the read-DQ calibration patterns.
// Each layout: JEDEC LPDDR4/4X value.

/// MR number of MR4 (refresh rate, mostly read-only).
pub const MR4: u8 = 4;

/// MR4: OP[2:0] refresh rate, OP[3] SR abort (RFU on some parts), OP[4] PPRE
/// (post-package repair entry), OP[6:5] thermal offset, OP[7] TUF
/// (temperature update flag). JEDEC LPDDR4/4X value.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Mr4 {
    pub refresh_rate: u8,
    pub sr_abort: bool,
    pub ppre: bool,
    pub thermal_offset: u8,
    pub tuf: bool,
}

/// tTSI, the temperature sensor update interval behind MR4: 32 ms.
/// JEDEC LPDDR4/4X value.
pub const T_TSI_MS: u32 = 32;

impl Mr4 {
    pub const fn encode(self) -> Option<u8> {
        if self.refresh_rate > 7 || self.thermal_offset > 3 {
            return None;
        }
        Some(
            (self.tuf as u8) << 7
                | self.thermal_offset << 5
                | (self.ppre as u8) << 4
                | (self.sr_abort as u8) << 3
                | self.refresh_rate,
        )
    }
    pub const fn decode(op: u8) -> Mr4 {
        Mr4 {
            refresh_rate: op & 7,
            sr_abort: op & 0x08 != 0,
            ppre: op & 0x10 != 0,
            thermal_offset: (op >> 5) & 3,
            tuf: op & 0x80 != 0,
        }
    }
}

/// MR numbers of the DQS oscillator count: MR18 low byte, MR19 high byte.
pub const MR18: u8 = 18;
pub const MR19: u8 = 19;

/// The 16-bit DQS oscillator count from MR18 (low) and MR19 (high).
/// JEDEC LPDDR4/4X value.
pub const fn dqs_osc_count(mr18: u8, mr19: u8) -> u16 {
    (mr19 as u16) << 8 | mr18 as u16
}

/// (MR18, MR19) of a 16-bit DQS oscillator count.
pub const fn dqs_osc_split(count: u16) -> (u8, u8) {
    (count as u8, (count >> 8) as u8)
}

/// MR number of MR22 (LPDDR4X ODT control).
pub const MR22: u8 = 22;

/// MR22 (LPDDR4X): OP[2:0] SoC ODT code, OP[3] CK ODT disable, OP[4] CS ODT
/// disable, OP[5] CA ODT disable (1 = disabled, default 0 = enabled),
/// OP[7:6] x8 byte-mode ODT disable. Some SoC ODT codes are illegal when
/// MR3 OP[0] = 0; which ones is not encoded here (the caller checks its
/// part). JEDEC LPDDR4/4X value.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Mr22 {
    pub soc_odt: u8,
    pub ck_odt_disable: bool,
    pub cs_odt_disable: bool,
    pub ca_odt_disable: bool,
    pub x8_odt_disable: u8,
}

impl Mr22 {
    pub const fn encode(self) -> Option<u8> {
        if self.soc_odt > 7 || self.x8_odt_disable > 3 {
            return None;
        }
        Some(
            self.x8_odt_disable << 6
                | (self.ca_odt_disable as u8) << 5
                | (self.cs_odt_disable as u8) << 4
                | (self.ck_odt_disable as u8) << 3
                | self.soc_odt,
        )
    }
    pub const fn decode(op: u8) -> Mr22 {
        Mr22 {
            soc_odt: op & 7,
            ck_odt_disable: op & 0x08 != 0,
            cs_odt_disable: op & 0x10 != 0,
            ca_odt_disable: op & 0x20 != 0,
            x8_odt_disable: op >> 6,
        }
    }
}

/// MR number of MR23 (DQS oscillator run time).
pub const MR23: u8 = 23;

/// DQS oscillator run time (MR23). JEDEC LPDDR4/4X value.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum OscRunTime {
    /// 0: the oscillator runs until stopped by MPC.
    StopByMpc,
    /// n x 16 clocks (n = 1..=63), or 2048/4096/8192 clocks (OP[7:6] =
    /// 01/10/11).
    Clocks(u32),
}

impl OscRunTime {
    pub const fn encode(self) -> Option<u8> {
        match self {
            OscRunTime::StopByMpc => Some(0),
            OscRunTime::Clocks(2048) => Some(0x40),
            OscRunTime::Clocks(4096) => Some(0x80),
            OscRunTime::Clocks(8192) => Some(0xc0),
            OscRunTime::Clocks(c) => {
                if c % 16 == 0 && c >= 16 && c <= 63 * 16 {
                    Some((c / 16) as u8)
                } else {
                    None
                }
            }
        }
    }
    /// None for codes with OP[7:6] set and OP[5:0] not zero.
    pub const fn decode(op: u8) -> Option<OscRunTime> {
        match op >> 6 {
            0 => {
                if op == 0 {
                    Some(OscRunTime::StopByMpc)
                } else {
                    Some(OscRunTime::Clocks(op as u32 * 16))
                }
            }
            top => {
                if op & 0x3f != 0 {
                    None
                } else {
                    Some(OscRunTime::Clocks(1024 << top))
                }
            }
        }
    }
}

/// Read-DQ calibration pattern registers and their defaults: MR32 pattern A
/// 5Ah, MR40 pattern B 3Ch, MR15 lower-byte invert 55h, MR20 upper-byte
/// invert 55h. JEDEC LPDDR4/4X value.
pub const MR32: u8 = 32;
pub const MR40: u8 = 40;
pub const MR15: u8 = 15;
pub const MR20: u8 = 20;
pub const RD_DQ_CAL_DEFAULTS: [(u8, u8); 4] =
    [(MR32, 0x5a), (MR40, 0x3c), (MR15, 0x55), (MR20, 0x55)];
