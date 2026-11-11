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

// Manufacturer's two retained public MR3 figures agree that PDDS000 is RFU.
// This is register-specific: MR11 still permits disabled termination at zero.
// https://raw.githubusercontent.com/spacemit-com/docs-buildroot/163a8fccf8e7071f3177753d2e86e482e97970fe/zh/k1_buildroot/device/peripheral_driver/static/ddr_reg01.png
impl Mr3 {
    /// The source's defaults: OP[0] = 1, the rest 0, with `pdds`.
    pub const fn defaults(pdds: Rzq) -> Mr3 {
        Mr3 { pdds, other: 1 }
    }
    pub const fn encode(self) -> Option<u8> {
        if matches!(self.pdds, Rzq::Disabled) || self.other & 0x38 != 0 {
            return None;
        }
        Some(self.other | (self.pdds as u8) << 3)
    }
    pub const fn decode(op: u8) -> Option<Mr3> {
        match Rzq::from_code((op >> 3) & 7) {
            Some(Rzq::Disabled) | None => None,
            Some(pdds) => Some(Mr3 {
                pdds,
                other: op & !0x38,
            }),
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
/// dramc_pi_api.h` fec3d608 lines 1610-1615 (`MR13_*`), BSD-3-Clause.
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
// MR4, MR18/MR19, MR22, MR23, the read-DQ calibration patterns.
// Each layout: JEDEC LPDDR4/4X value. Their meaning: the MR4
// refresh-rate codes and the cache band, the oscillator count to tDQS2DQ,
// the RD-DQ-CAL expected pattern per DQ.

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

/// MR4 OP[2:0]: the refresh rate the DRAM asks for at its temperature,
/// as multiples of the base refresh intervals. JEDEC LPDDR4/4X value
/// .
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RefreshRate {
    /// 000: the low temperature operating limit is exceeded.
    LowLimitExceeded,
    /// 001: 4x tREFI (and tREFIpb, tREFW).
    X4,
    /// 010: 2x.
    X2,
    /// 011: 1x (up to 85 C).
    X1,
    /// 100: 0.5x, no derating.
    Half,
    /// 101: 0.25x, no derating.
    Quarter,
    /// 110: 0.25x and derate the AC timings (tRCD, tRP, tRAS... by tDERATE).
    QuarterDerated,
    /// 111: the high temperature operating limit is exceeded.
    HighLimitExceeded,
}

impl RefreshRate {
    pub const fn from_code(code: u8) -> RefreshRate {
        match code & 7 {
            0 => RefreshRate::LowLimitExceeded,
            1 => RefreshRate::X4,
            2 => RefreshRate::X2,
            3 => RefreshRate::X1,
            4 => RefreshRate::Half,
            5 => RefreshRate::Quarter,
            6 => RefreshRate::QuarterDerated,
            _ => RefreshRate::HighLimitExceeded,
        }
    }
    pub const fn code(self) -> u8 {
        self as u8
    }
    /// tREFI as a multiple of the base interval, in quarters (16 = 4x,
    /// 4 = 1x, 1 = 0.25x); None outside the operating limits.
    pub const fn refi_quarters(self) -> Option<u32> {
        match self {
            RefreshRate::X4 => Some(16),
            RefreshRate::X2 => Some(8),
            RefreshRate::X1 => Some(4),
            RefreshRate::Half => Some(2),
            RefreshRate::Quarter | RefreshRate::QuarterDerated => Some(1),
            _ => None,
        }
    }
    /// Whether the AC timings must be derated at this reading.
    pub const fn derate(self) -> bool {
        matches!(self, RefreshRate::QuarterDerated)
    }
    /// Whether the DRAM is outside its operating temperature range.
    pub const fn out_of_range(self) -> bool {
        matches!(
            self,
            RefreshRate::LowLimitExceeded | RefreshRate::HighLimitExceeded
        )
    }
}

impl Mr4 {
    /// The refresh-rate field's meaning.
    pub const fn refresh_rate(self) -> RefreshRate {
        RefreshRate::from_code(self.refresh_rate)
    }

    /// The temperature band of an MR4 reading for the training cache
    /// (`dram_train::cache_temp::Band`): the refresh-rate code itself,
    /// 0..=7, monotonic in temperature. The thermal offset, TUF and the
    /// other bits are not part of the band.
    pub const fn band(op: u8) -> u8 {
        op & 7
    }

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

/// The DQS interval oscillator toggles DQS at 1 / (2 x tDQS2DQ): the
/// count after a run time of `clocks` is run time / (2 x tDQS2DQ), so
/// tDQS2DQ = run time / (2 x count). JEDEC LPDDR4/4X value.
pub const DQS_OSC_DIVISOR: u32 = 2;

/// tDQS2DQ in ps from an oscillator count after `run_clocks` clocks of
/// `tck_ps` (rounded); None for a count of 0.
pub const fn dqs_osc_delay_ps(run_clocks: u32, tck_ps: u32, count: u16) -> Option<u32> {
    if count == 0 {
        return None;
    }
    let run = run_clocks as u64 * tck_ps as u64;
    let den = DQS_OSC_DIVISOR as u64 * count as u64;
    Some(((run + den / 2) / den) as u32)
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
    /// The run time in clocks; None for [`OscRunTime::StopByMpc`] (the
    /// controller measures it).
    pub const fn clocks(self) -> Option<u32> {
        match self {
            OscRunTime::StopByMpc => None,
            OscRunTime::Clocks(c) => Some(c),
        }
    }

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

/// The read-DQ calibration pattern (MPC RD-DQ-CAL) as programmed: pattern
/// A (MR32) and B (MR40), and the per-DQ inversion masks of the lower
/// (MR15, DQ0-7) and upper (MR20, DQ8-15) byte.
///
/// [`RdDqCal::expected`] gives one DQ's 16 beats as a u16, pattern A in
/// the low byte and B in the high byte, inverted when the DQ's mask bit is
/// set. The beat order on the bus (which bit of A comes first) is not
/// taken from a free source and is left to the board's engine check
/// (`crate::pending::LP4`); the engines compare whole bursts, so only the
/// inversion and the A/B split matter here.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct RdDqCal {
    pub pattern_a: u8,
    pub pattern_b: u8,
    pub invert_lower: u8,
    pub invert_upper: u8,
}

impl RdDqCal {
    /// The JEDEC defaults (`RD_DQ_CAL_DEFAULTS`).
    pub const DEFAULT: RdDqCal = RdDqCal {
        pattern_a: 0x5a,
        pattern_b: 0x3c,
        invert_lower: 0x55,
        invert_upper: 0x55,
    };

    /// From the four mode registers as read or programmed.
    pub const fn from_mrs(mr32: u8, mr40: u8, mr15: u8, mr20: u8) -> RdDqCal {
        RdDqCal {
            pattern_a: mr32,
            pattern_b: mr40,
            invert_lower: mr15,
            invert_upper: mr20,
        }
    }

    /// (MR, value) pairs to program this pattern, in the defaults' order.
    pub const fn mrs(self) -> [(u8, u8); 4] {
        [
            (MR32, self.pattern_a),
            (MR40, self.pattern_b),
            (MR15, self.invert_lower),
            (MR20, self.invert_upper),
        ]
    }

    /// Whether DQ `dq` (0..=15) returns the pattern inverted.
    pub const fn inverted(self, dq: u8) -> bool {
        let (mask, bit) = if dq < 8 {
            (self.invert_lower, dq)
        } else {
            (self.invert_upper, dq - 8)
        };
        mask >> bit & 1 != 0
    }

    /// The 16 beats expected on DQ `dq` (0..=15): A in bits 0-7, B in bits
    /// 8-15, inverted when the DQ's mask bit is set; None above DQ15.
    pub const fn expected(self, dq: u8) -> Option<u16> {
        if dq > 15 {
            return None;
        }
        let v = (self.pattern_b as u16) << 8 | self.pattern_a as u16;
        Some(if self.inverted(dq) { !v } else { v })
    }
}
