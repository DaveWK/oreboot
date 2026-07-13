// SPDX-License-Identifier: GPL-2.0-only
//! LPDDR5 device timings (JEDEC LPDDR5/5X, 8 Gb/16 Gb per channel dies), in ns,
//! as the Allwinner A733 parameter tables use them (lifted from the A7A
//! `timing.rs`). Convert with [`crate::timing::ns_to_ck`].
//!
//! Per data rate and WCK:CK ratio, [`FREQ_RANGES_2_1`] and
//! [`FREQ_RANGES_4_1`]: JEDEC LPDDR5/5X (DVFSC disabled,
//! read link ECC off) as transcribed in LiteDRAM
//! 74522f715d7b10163ef6f6caa09887a90874be83
//! `litedram/phy/lpddr5/basephy.py` `FREQUENCY_RANGES`.
//!
//! Per die density and bank mode ([`refresh`], [`rrd_faw`], [`CCD_BG_4TO1_BL16`]):
//! - tRFCab, tRFCpb, tpbR2pbR: JEDEC LPDDR5/5X as transcribed in
//!   ramulator2 `python/ramulator/dram/lpddr5.py` lines 171-232 (orchestrator
//!   `references/datasheets/open-source/ramulator2-lpddr5.py`, sha256
//!   d187d202..., MIT), cross-checked against JEDEC LPDDR5/5X in
//!   coreboot `util/spd_tools/src/spd_gen/lp5.go` fec3d608 lines 250-310
//!   (sha256 d059b31c..., GPL-2.0-or-later) and the Micron Y52Q 8 Gb die
//!   addenda (210/120/90 ns).
//! - tRRD, tFAW per bank mode: JEDEC LPDDR5/5X via ramulator2
//!   lines 149-171; gem5 `lpddr5.py` lines 165-169, 240-243 (sha256
//!   f0a758fb..., BSD-3-Clause) agree.
//! - tCCD (BG mode, WCK:CK 4:1, BL16): JEDEC LPDDR5/5X via ramulator2
//!   presets lines 252-273 (nCCDS 2, nCCDL 4); gem5's 6400 BL16 tCCD_L of
//!   5 ns at tCK 1.25 ns is the same 4 nCK.
//!
//! P1-40 (each a JEDEC LPDDR5 value): the bank-mode limits
//! ([`bank_mode_max_rate`]), the enhanced-DVFSC refresh of 16 Gb dies
//! ([`refresh_edvfsc`]), tPBR2ACT of 12 Gb dies ([`t_pbr2act_ps`]), the WCK
//! clock limits ([`WCK_CLOCK`]), the device timings at the end of this module
//! (tMRW/tMRD/tMRR/tMRRI, tVRCG, tFC, tCKFSPE/X, tVREF, tWCK2CK, [`leveling`],
//! [`cbt`], DCM/oscillator/RDQS, ZQ, [`INIT`], DCA and DFE ranges) and the
//! MR1/MR2 code tables ([`WL_4_1`], [`RL_4_1`], [`NWR_4_1`]), which extend
//! the frequency ranges to 8533 Mb/s.
//!
//! Not here: the tWCK2CK picosecond caps (TBD in JEDEC; `crate::pending::LP5`).
//! The 16B-mode codes 1100b/1101b at 3733 and 4267 Mb/s at 2:1 are a vendor
//! extension (JEDEC limits 16B to 3200 Mb/s), so they are not in the tables.

pub const T_FAW: u32 = 20;
pub const T_RRD: u32 = 5;
pub const T_RCD: u32 = 18;
pub const T_XP: u32 = 7;
pub const T_WTR_L: u32 = 12;
pub const T_RTP: u32 = 8; // 7.5 ns
pub const T_RPAB: u32 = 21;
pub const T_RAS: u32 = 42;
pub const T_REFI: u32 = 3906;
pub const T_RFCAB: u32 = 380;
/// Power-up: 200 us of RESET/CKE timing for the DFI init counters.
pub const T_INIT_200US: u32 = 200_000;
pub const T_ZQCAL: u32 = 260;
pub const T_XSR_EXTRA: u32 = 90;
pub const T_CKE: u32 = 3;
pub const T_ESCKE: u32 = 10;
pub const T_CCDMW_NS: u32 = 15;
pub const T_CMD_1NS: u32 = 1;
pub const T_MRW_40: u32 = 40;
pub const T_ZQRESET_6US: u32 = 6000;
pub const T_ZQLAT_30: u32 = 30;
pub const T_ZQCS_50: u32 = 50;

/// One row of the JEDEC LPDDR5/5X latency tables for one WCK:CK ratio. Latencies
/// in nCK; data rates in Mbps, the row covering (`dr_min`, `dr_max`].
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct FreqRange {
    /// MR1 OP[7:4] / MR2 OP[3:0] code.
    pub mr: u8,
    pub dr_min: u32,
    pub dr_max: u32,
    /// Write latency, set A and set B.
    pub wl: [u8; 2],
    /// tWCKENL_WR, set A and set B.
    pub t_wckenl_wr: [u8; 2],
    pub t_wckpre_static: u8,
    pub t_wckpre_toggle_wr: u8,
    /// Read latency, sets 0, 1, 2.
    pub rl: [u8; 3],
    /// tWCKENL_RD, sets 0, 1, 2.
    pub t_wckenl_rd: [u8; 3],
    pub t_wckpre_toggle_rd: u8,
    pub n_rbtp: u8,
    pub n_wr: u8,
    /// MR2 OP[7:4] code.
    pub n_wr_op: u8,
}

/// One row in the source's layout: MR code, (DR min, max], WL (A, B),
/// tWCKENL_WR (A, B), tWCKPRE_static, tWCKPRE_toggle_WR, RL (0, 1, 2),
/// tWCKENL_RD (0, 1, 2), tWCKPRE_toggle_RD, nRBTP, nWR, nWR code.
type Row = (
    u8,
    (u32, u32),
    (u8, u8),
    (u8, u8),
    u8,
    u8,
    (u8, u8, u8),
    (u8, u8, u8),
    u8,
    u8,
    u8,
    u8,
);

const fn row(r: Row) -> FreqRange {
    FreqRange {
        mr: r.0,
        dr_min: r.1 .0,
        dr_max: r.1 .1,
        wl: [r.2 .0, r.2 .1],
        t_wckenl_wr: [r.3 .0, r.3 .1],
        t_wckpre_static: r.4,
        t_wckpre_toggle_wr: r.5,
        rl: [r.6 .0, r.6 .1, r.6 .2],
        t_wckenl_rd: [r.7 .0, r.7 .1, r.7 .2],
        t_wckpre_toggle_rd: r.8,
        n_rbtp: r.9,
        n_wr: r.10,
        n_wr_op: r.11,
    }
}

/// WCK:CK 2:1.
pub const FREQ_RANGES_2_1: [FreqRange; 6] = [
    row((
        0b0000,
        (40, 533),
        (4, 4),
        (1, 1),
        1,
        3,
        (6, 6, 6),
        (0, 0, 0),
        6,
        0,
        5,
        0b0000,
    )),
    row((
        0b0001,
        (533, 1067),
        (4, 6),
        (0, 2),
        2,
        3,
        (8, 8, 8),
        (0, 0, 0),
        7,
        0,
        10,
        0b0001,
    )),
    row((
        0b0010,
        (1067, 1600),
        (6, 8),
        (1, 3),
        2,
        4,
        (10, 10, 12),
        (1, 1, 3),
        8,
        0,
        14,
        0b0010,
    )),
    row((
        0b0011,
        (1600, 2133),
        (8, 10),
        (2, 4),
        3,
        4,
        (12, 14, 14),
        (2, 4, 4),
        8,
        0,
        19,
        0b0011,
    )),
    row((
        0b0100,
        (2133, 2750),
        (8, 14),
        (1, 7),
        4,
        4,
        (16, 16, 18),
        (3, 3, 5),
        10,
        2,
        24,
        0b0100,
    )),
    row((
        0b0101,
        (2750, 3200),
        (10, 16),
        (3, 9),
        4,
        4,
        (18, 20, 20),
        (5, 7, 7),
        10,
        2,
        28,
        0b0101,
    )),
];

/// WCK:CK 4:1.
pub const FREQ_RANGES_4_1: [FreqRange; 12] = [
    row((
        0b0000,
        (40, 533),
        (2, 2),
        (0, 0),
        1,
        2,
        (3, 3, 3),
        (0, 0, 0),
        3,
        0,
        3,
        0b0000,
    )),
    row((
        0b0001,
        (533, 1067),
        (2, 3),
        (0, 1),
        1,
        2,
        (4, 4, 4),
        (0, 0, 0),
        4,
        0,
        5,
        0b0001,
    )),
    row((
        0b0010,
        (1067, 1600),
        (3, 4),
        (1, 2),
        1,
        2,
        (5, 5, 6),
        (1, 1, 2),
        4,
        0,
        7,
        0b0010,
    )),
    row((
        0b0011,
        (1600, 2133),
        (4, 5),
        (1, 2),
        2,
        2,
        (6, 7, 7),
        (1, 2, 2),
        4,
        0,
        10,
        0b0011,
    )),
    row((
        0b0100,
        (2133, 2750),
        (4, 7),
        (1, 4),
        2,
        2,
        (8, 8, 9),
        (2, 2, 3),
        5,
        1,
        12,
        0b0100,
    )),
    row((
        0b0101,
        (2750, 3200),
        (5, 8),
        (2, 5),
        2,
        2,
        (9, 10, 10),
        (3, 4, 4),
        5,
        1,
        14,
        0b0101,
    )),
    row((
        0b0110,
        (3200, 3733),
        (6, 9),
        (2, 5),
        3,
        2,
        (10, 11, 12),
        (3, 4, 5),
        5,
        2,
        16,
        0b0110,
    )),
    row((
        0b0111,
        (3733, 4267),
        (6, 11),
        (2, 7),
        3,
        2,
        (12, 13, 14),
        (4, 5, 6),
        6,
        2,
        19,
        0b0111,
    )),
    row((
        0b1000,
        (4267, 4800),
        (7, 12),
        (3, 8),
        3,
        2,
        (13, 14, 15),
        (5, 6, 7),
        6,
        3,
        21,
        0b1000,
    )),
    row((
        0b1001,
        (4800, 5500),
        (8, 14),
        (3, 9),
        4,
        2,
        (15, 16, 17),
        (6, 7, 8),
        6,
        4,
        24,
        0b1001,
    )),
    row((
        0b1010,
        (5500, 6000),
        (9, 15),
        (4, 10),
        4,
        2,
        (16, 17, 19),
        (6, 7, 9),
        7,
        4,
        26,
        0b1010,
    )),
    row((
        0b1011,
        (6000, 6400),
        (9, 16),
        (4, 11),
        4,
        2,
        (17, 18, 20),
        (7, 8, 10),
        7,
        4,
        28,
        0b1011,
    )),
];

/// The table of a WCK:CK ratio.
pub const fn freq_ranges(ratio: super::WckCkRatio) -> &'static [FreqRange] {
    match ratio {
        super::WckCkRatio::Two => &FREQ_RANGES_2_1,
        super::WckCkRatio::Four => &FREQ_RANGES_4_1,
    }
}

/// The row for a data rate in Mbps (DR in (`dr_min`, `dr_max`]).
pub const fn freq_range(ratio: super::WckCkRatio, data_rate: u32) -> Option<FreqRange> {
    let t = freq_ranges(ratio);
    let mut i = 0;
    while i < t.len() {
        if data_rate > t[i].dr_min && data_rate <= t[i].dr_max {
            return Some(t[i]);
        }
        i += 1;
    }
    None
}

/// CK frequency in MHz of a data rate in Mbps: DR = 2 x WCK, WCK = ratio x
/// CK (rounded down).
pub const fn ck_mhz(ratio: super::WckCkRatio, data_rate: u32) -> u32 {
    data_rate / (2 * ratio.value() as u32)
}

/// tRFCab, tRFCpb and tpbR2pbR (per-bank refresh to per-bank refresh) in ns.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Refresh {
    pub t_rfc_ab_ns: u32,
    pub t_rfc_pb_ns: u32,
    pub t_pbr2pbr_ns: u32,
}

/// [`Refresh`] for a die density of `gb` Gb (2, 3, 4, 6, 8, 12, 16, 24 or
/// 32), Enhanced DVFSC disabled: JEDEC LPDDR5/5X (ramulator2
/// `lpddr5.py` `_resolve_nRFC`, `_resolve_nRFCpb`, `_resolve_nPBR2PBR`).
pub const fn refresh(gb: u8) -> Option<Refresh> {
    let (ab, pb, pbr2pbr) = match gb {
        2 => (130, 60, 60),
        3 | 4 => (180, 90, 90),
        6 | 8 => (210, 120, 90),
        12 | 16 => (280, 140, 90),
        24 | 32 => (380, 190, 90),
        _ => return None,
    };
    Some(Refresh {
        t_rfc_ab_ns: ab,
        t_rfc_pb_ns: pb,
        t_pbr2pbr_ns: pbr2pbr,
    })
}

/// LPDDR5 bank architecture (MR3 BK/BG ORG).
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BankMode {
    /// 4 bank groups of 4 banks.
    Bg,
    /// 8 banks.
    B8,
    /// 16 banks.
    B16,
}

/// (tRRD, tFAW) in ns for a bank mode: BG and 16B 5 and 20 ns, 8B 10 and
/// 40 ns (JEDEC LPDDR5/5X via ramulator2 `_resolve_nRRDS`,
/// `_resolve_nFAW`; gem5 `LPDDR5_*_BG_*` and `*_8B_*` agree). tRRD_L
/// equals tRRD in BG mode.
pub const fn rrd_faw(mode: BankMode) -> (u32, u32) {
    match mode {
        BankMode::Bg | BankMode::B16 => (5, 20),
        BankMode::B8 => (10, 40),
    }
}

/// tCCD_S and tCCD_L in nCK for BG mode, WCK:CK 4:1, BL16: BL/n_min = 2
/// and BL/n_max = 4 (JEDEC LPDDR5/5X via ramulator2 presets).
pub const CCD_BG_4TO1_BL16: (u32, u32) = (2, 4);

// ---------------------------------------------------------------------------
// P1-40. Each value: JEDEC LPDDR5/5X value.

/// The highest data rate (Mb/s) of a bank mode at a WCK:CK ratio and DVFSC
/// mode, or None where the combination is not supported. LPDDR5 (speed
/// grade up to 6400): DVFSC off, 4:1: BG and 8B 6400, 16B 3200; 2:1: 8B and
/// 16B 3200, no BG; DVFSC on: 8B and 16B 1600, no BG; no E-DVFSC. LPDDR5X
/// (up to 8533, no 8B): off, 4:1: BG 8533, 16B 3200; 2:1: 16B 3200; DVFSC:
/// 16B 1600; E-DVFSC: 16B 3200. JEDEC LPDDR5 value. (Some parts allow 16B
/// up to 4267 Mb/s at 2:1; that is a vendor extension and not encoded.)
pub const fn bank_mode_max_rate(
    mode: BankMode,
    ratio: super::WckCkRatio,
    dvfsc: super::mr::Dvfsc,
    lp5x: bool,
) -> Option<u32> {
    use super::mr::Dvfsc;
    use super::WckCkRatio::{Four, Two};
    match (lp5x, dvfsc, mode, ratio) {
        (_, Dvfsc::HighSpeed, BankMode::Bg, Two) => None,
        (false, Dvfsc::HighSpeed, BankMode::Bg | BankMode::B8, Four) => Some(6400),
        (false, Dvfsc::HighSpeed, BankMode::B8, Two) => Some(3200),
        (true, Dvfsc::HighSpeed, BankMode::Bg, Four) => Some(8533),
        (_, Dvfsc::HighSpeed, BankMode::B16, _) => Some(3200),
        (false, Dvfsc::Dvfsc, BankMode::B8 | BankMode::B16, _) => Some(1600),
        (true, Dvfsc::Dvfsc, BankMode::B16, _) => Some(1600),
        (true, Dvfsc::Enhanced, BankMode::B16, _) => Some(3200),
        _ => None,
    }
}

/// Whether an LPDDR5 bank mode may be used at a WCK:CK ratio, DVFSC state
/// and data rate (Mb/s) ([`bank_mode_max_rate`]): BG needs WCK:CK 4:1 with
/// DVFSC off; 16B tops out at 3200 Mb/s, 1600 Mb/s with DVFSC on. JEDEC
/// LPDDR5 value.
pub const fn bank_mode_allowed(
    mode: BankMode,
    ratio: super::WckCkRatio,
    dvfsc: bool,
    data_rate: u32,
) -> bool {
    let d = if dvfsc {
        super::mr::Dvfsc::Dvfsc
    } else {
        super::mr::Dvfsc::HighSpeed
    };
    match bank_mode_max_rate(mode, ratio, d, false) {
        Some(max) => data_rate <= max,
        None => false,
    }
}

/// Refresh-management (RFM) times in ns.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Rfm {
    pub t_rfm_ab_ns: u32,
    pub t_rfm_pb_ns: u32,
}

/// [`Refresh`] and [`Rfm`] with enhanced DVFSC enabled. Only 16 Gb dies so
/// far: tRFCab 300 ns, tRFCpb 150 ns, tpbR2pbR 100 ns, tRFMab 300 ns,
/// tRFMpb 190 ns. JEDEC LPDDR5/5X value.
pub const fn refresh_edvfsc(gb: u8) -> Option<(Refresh, Rfm)> {
    match gb {
        16 => Some((
            Refresh {
                t_rfc_ab_ns: 300,
                t_rfc_pb_ns: 150,
                t_pbr2pbr_ns: 100,
            },
            Rfm {
                t_rfm_ab_ns: 300,
                t_rfm_pb_ns: 190,
            },
        )),
        _ => None,
    }
}

/// tPBR2ACT (per-bank refresh to activate) in ps. Only 12 Gb dies so far:
/// 7.5 ns, 10 ns in 8B mode. JEDEC LPDDR5/5X value (the same
/// value set gives the 12 Gb tRFCab 280, tRFCpb 140, tPBR2PBR 90 ns that
/// [`refresh`] already has).
pub const fn t_pbr2act_ps(gb: u8, mode: BankMode) -> Option<u32> {
    match (gb, mode) {
        (12, BankMode::B8) => Some(10_000),
        (12, _) => Some(7_500),
        _ => None,
    }
}

/// WCK clock limits at one WCK frequency.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct WckClock {
    pub wck_mhz: u32,
    /// tWCK(avg) minimum, ps.
    pub t_wck_avg_min_ps: u32,
    /// tJIT(per), ps (the limit is +/- this value).
    pub t_jit_per_ps: u32,
}

/// tWCK(avg) min 3.76 / 1.877 / 1.25 / 0.938 ns and tJIT(per) +/-190 / 95 /
/// 70 / 56 ps at WCK 266 / 533 / 800 / 1067 MHz. JEDEC LPDDR5/5X
/// value.
pub const WCK_CLOCK: [WckClock; 4] = [
    WckClock {
        wck_mhz: 266,
        t_wck_avg_min_ps: 3760,
        t_jit_per_ps: 190,
    },
    WckClock {
        wck_mhz: 533,
        t_wck_avg_min_ps: 1877,
        t_jit_per_ps: 95,
    },
    WckClock {
        wck_mhz: 800,
        t_wck_avg_min_ps: 1250,
        t_jit_per_ps: 70,
    },
    WckClock {
        wck_mhz: 1067,
        t_wck_avg_min_ps: 938,
        t_jit_per_ps: 56,
    },
];

/// The [`WCK_CLOCK`] row of a WCK frequency, if listed.
pub const fn wck_clock(wck_mhz: u32) -> Option<WckClock> {
    let mut i = 0;
    while i < WCK_CLOCK.len() {
        if WCK_CLOCK[i].wck_mhz == wck_mhz {
            return Some(WCK_CLOCK[i]);
        }
        i += 1;
    }
    None
}

/// WCK duty cycle, average and absolute, in per mille of tWCK (min, max):
/// 0.46-0.54 average, 0.43-0.57 absolute. JEDEC LPDDR5/5X value.
pub const WCK_DUTY_AVG: (u32, u32) = (460, 540);
pub const WCK_DUTY_ABS: (u32, u32) = (430, 570);

// ---------------------------------------------------------------------------
// P1-40: device timings. Each value: JEDEC LPDDR5 value. These are LPDDR5's
// own; where LPDDR4 has a timing of the same name the values differ (tMRW and
// tMRD are 5 nCK here, 10 nCK on LPDDR4).

/// A minimum of max(`ps`, `nck` CK cycles) (the LPDDR4 type).
pub use crate::lp4::timing::{ps_to_ck, Max, Min};

/// `ps` plus half a CK period (a minimum).
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct PsPlusHalfCk {
    pub ps: u32,
}

impl PsPlusHalfCk {
    /// CK cycles at `ck_mhz`, rounded up: ceil(ps / tCK + 0.5).
    pub const fn cycles(self, ck_mhz: u32) -> u32 {
        ((2 * self.ps as u64 * ck_mhz as u64 + 1_000_000).div_ceil(2_000_000)) as u32
    }
}

/// A minimum of max(`ps`, `nwck_x2` / 2 WCK cycles).
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct MinWck {
    /// WCK cycles times two (2.5 tWCK = 5).
    pub nwck_x2: u32,
    pub ps: u32,
}

impl MinWck {
    /// WCK cycles at `wck_mhz`, rounded up.
    pub const fn wck_cycles(self, wck_mhz: u32) -> u32 {
        let t = ps_to_ck(self.ps, wck_mhz);
        let n = self.nwck_x2.div_ceil(2);
        if t > n {
            t
        } else {
            n
        }
    }
}

/// A signed range in thousandths of tWCK.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct TwckRange {
    pub min_milli: i32,
    pub max_milli: i32,
}

impl TwckRange {
    /// The range in ps at `wck_mhz` (rounded toward zero).
    pub const fn ps(self, wck_mhz: u32) -> (i32, i32) {
        let t = 1_000_000_000i64 / wck_mhz as i64; // tWCK in fs
        (
            (self.min_milli as i64 * t / 1_000_000) as i32,
            (self.max_milli as i64 * t / 1_000_000) as i32,
        )
    }
}

/// A range in ps (min, max).
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct PsRange {
    pub min: u32,
    pub max: u32,
}

/// tMRW: max(10 ns, 5 nCK).
pub const T_MRW: Min = Min::ps(5, 10_000);
/// tMRD: max(14 ns, 5 nCK).
pub const T_MRD: Min = Min::ps(5, 14_000);
/// tMRR in nCK: 4 at WCK:CK 4:1, 8 at 2:1.
pub const fn t_mrr_nck(ratio: super::WckCkRatio) -> u32 {
    match ratio {
        super::WckCkRatio::Four => 4,
        super::WckCkRatio::Two => 8,
    }
}
/// tMRRI (after tXP, before MRR or RDC): tRCD + 2 nCK.
pub const fn t_mrri_nck(t_rcd_nck: u32) -> u32 {
    t_rcd_nck + 2
}

/// tVRCG_ENABLE: at most 150 ns from MRW MR16 OP[6] = 1 to a non-DES
/// command; tVRCG_DISABLE: at most 100 ns.
pub const T_VRCG_ENABLE: Max = Max { ps: 150_000 };
pub const T_VRCG_DISABLE: Max = Max { ps: 100_000 };

/// Vref step size of an FSP switch or Vref change: short (one step) or long
/// (two or more).
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Step {
    Short,
    Long,
}

/// tFC (FSP-OP switch): short 200 ns + 0.5 tCK, long 250 ns + 0.5 tCK, by
/// the VREF(CA) difference of the two FSPs; tCK is the one at the MRW.
pub const fn t_fc(step: Step) -> PsPlusHalfCk {
    match step {
        Step::Short => PsPlusHalfCk { ps: 200_000 },
        Step::Long => PsPlusHalfCk { ps: 250_000 },
    }
}
/// tCKFSPE / tCKFSPX: max(7.5 ns, 4 nCK).
pub const T_CKFSPE: Min = Min::ps(4, 7500);
pub const T_CKFSPX: Min = Min::ps(4, 7500);

/// tVREFCA / tVREFDQ with VRCG high current: short 200 ns + 0.5 tCK, long
/// 250 ns + 0.5 tCK (the same values bound tVREFCA in CBT mode 2).
pub const fn t_vref(step: Step) -> PsPlusHalfCk {
    t_fc(step)
}
/// tVREFCA_Weak / tVREFDQ_Weak (VRCG normal mode): 1 ms.
pub const T_VREF_WEAK_PS: u32 = 1_000_000_000;

/// tWCK2CK (WCK-to-CK phase offset): +/-0.5 tWCK at 4:1, +/-0.25 tWCK at
/// 2:1. The picosecond caps are TBD in JEDEC ([`crate::pending::LP5`]).
pub const fn t_wck2ck(ratio: super::WckCkRatio) -> TwckRange {
    match ratio {
        super::WckCkRatio::Four => TwckRange {
            min_milli: -500,
            max_milli: 500,
        },
        super::WckCkRatio::Two => TwckRange {
            min_milli: -250,
            max_milli: 250,
        },
    }
}

/// WCK2CK leveling timings. Leveling runs at WCK:CK 2:1 (MR18 OP[7] = 1)
/// whatever the operating ratio.
pub mod leveling {
    use super::{Min, MinWck, TwckRange};
    /// tWCK2CK_Leveling: the range of valid feedback, +/-0.5 tWCK (ps caps
    /// TBD in JEDEC).
    pub const T_WCK2CK_LEVELING: TwckRange = TwckRange {
        min_milli: -500,
        max_milli: 500,
    };
    /// tWLWCKON: WCK static at least 2 tCK before the entry MRW.
    pub const T_WLWCKON: Min = Min::ps(2, 0);
    /// tWLMRD: max(14 ns, 5 tCK) from the MRW to the first WCK edge.
    pub const T_WLMRD: Min = Min::ps(5, 14_000);
    /// tWLO: output delay, at most max(2 tCK, 20 ns).
    pub const T_WLO_MAX: Min = Min::ps(2, 20_000);
    /// tWCK_INT: max(25 ns, 2.5 tWCK) between toggle bursts.
    pub const T_WCK_INT: MinWck = MinWck {
        nwck_x2: 5,
        ps: 25_000,
    };
    /// tWLWCKOFF: max(14 ns, 5 tCK) after the exit.
    pub const T_WLWCKOFF: Min = Min::ps(5, 14_000);
    /// tWLDQOFF: DQ off at most max(14 ns, 5 tCK) after the exit.
    pub const T_WLDQOFF_MAX: Min = Min::ps(5, 14_000);
    /// tWCKTGGL: exactly 7.5 tWCK per phase detection, as half cycles.
    pub const T_WCKTGGL_HALF_WCK: u32 = 15;
}

/// Command bus training (CBT) timings, modes 1 and 2.
pub mod cbt {
    use super::{Min, MinWck, PsPlusHalfCk};
    /// tCBTWCKPRE_static: max(20 ns, 2 nCK) from CBT entry to WCK toggling.
    pub const T_CBTWCKPRE_STATIC: Min = Min::ps(2, 20_000);
    /// tWCK2DQ7H, tDQ7HWCK, tDQ7LWCK: max(5 ns, 12 nWCK).
    pub const T_WCK2DQ7H: MinWck = MinWck {
        nwck_x2: 24,
        ps: 5000,
    };
    pub const T_DQ7HWCK: MinWck = T_WCK2DQ7H;
    pub const T_DQ7LWCK: MinWck = T_WCK2DQ7H;
    /// tDQ7HCK, tDQ7LCK: max(5 ns, 3 nCK).
    pub const T_DQ7HCK: Min = Min::ps(3, 5000);
    pub const T_DQ7LCK: Min = Min::ps(3, 5000);
    /// tDQ7FSP: CA ODT change at most 20 ns after DQ7 high.
    pub const T_DQ7FSP_MAX_PS: u32 = 20_000;
    /// tXP inside tCKPRECS: max(7 ns, 3 nCK).
    pub const T_XP: Min = Min::ps(3, 7000);
    /// tCKPRECS: 2 tCK + tXP, in nCK at `ck_mhz`.
    pub const fn t_ckprecs_nck(ck_mhz: u32) -> u32 {
        2 + T_XP.cycles(ck_mhz)
    }
    /// tCKPSTCS: max(7.5 ns, 3 nCK).
    pub const T_CKPSTCS: Min = Min::ps(3, 7500);
    /// tCAENT (mode 1): 250 ns from DQ7 high to the first CA pattern.
    pub const T_CAENT_PS: u32 = 250_000;
    /// tADR: CA-to-DQ feedback at most 20 ns.
    pub const T_ADR_MAX_PS: u32 = 20_000;
    /// tCACD: RU(tADR / tCK) nCK between CA patterns.
    pub const fn t_cacd_nck(ck_mhz: u32) -> u32 {
        super::ps_to_ck(T_ADR_MAX_PS, ck_mhz)
    }
    /// tMRZ: 1.5 ns from DQ7 low to the DQ drivers off.
    pub const T_MRZ_PS: u32 = 1500;
    /// tXCBT: max(5 nCK, 250 ns) from CBT exit to the next command.
    pub const T_XCBT: Min = Min::ps(5, 250_000);
    /// tCBTWCKODTFIX, tCBTODTOFF, tCBTNTODTOFF: at most 20 ns.
    pub const T_CBT_ODT_MAX_PS: u32 = 20_000;
    /// Mode 2: tDQ72DQ, 250 ns from DQ7 high to DQ[6:0] Vref input.
    pub const T_DQ72DQ_PS: u32 = 250_000;
    /// Mode 2: tVREFCA long / short, at most 250 / 200 ns + 0.5 tCK.
    pub const T_VREFCA_LONG: PsPlusHalfCk = PsPlusHalfCk { ps: 250_000 };
    pub const T_VREFCA_SHORT: PsPlusHalfCk = PsPlusHalfCk { ps: 200_000 };
    /// Mode 2: tDStrain / tDHtrain around the DMI0 strobe: max(5 ns, 12
    /// nWCK).
    pub const T_DSTRAIN: MinWck = T_WCK2DQ7H;
    pub const T_DHTRAIN: MinWck = T_WCK2DQ7H;
    /// Mode 2: tCA2DMIL, 30 ns.
    pub const T_CA2DMIL_PS: u32 = 30_000;
    /// Mode 2: tCBTRTW, max(20 ns, 12 nWCK) from DMI low to DQ input.
    pub const T_CBTRTW: MinWck = MinWck {
        nwck_x2: 24,
        ps: 20_000,
    };
}

/// tDCMM: duty-cycle monitor measurement time, 2 us (WCK 1600-3200 MHz).
pub const T_DCMM_PS: u32 = 2_000_000;
/// tOSCAL: Rx offset calibration, at most 3 us (LPDDR5X).
pub const T_OSCAL_MAX_PS: u32 = 3_000_000;
/// tOSCODQI / tOSCODQO (oscillator stop to MRR) and tOSCINT (stop to next
/// start): max(40 ns, 8 nCK).
pub const T_OSCODQI: Min = Min::ps(8, 40_000);
pub const T_OSCODQO: Min = Min::ps(8, 40_000);
pub const T_OSCINT: Min = Min::ps(8, 40_000);
/// tERQE / tERQX / tRDQE_OD / tRDQX_OD (enhanced RDQS and RDQS toggle
/// entry/exit): at most max(35 ns, 4 nCK).
pub const T_ERQE_MAX: Min = Min::ps(4, 35_000);
pub const T_ERQX_MAX: Min = Min::ps(4, 35_000);
pub const T_RDQE_OD_MAX: Min = Min::ps(4, 35_000);
pub const T_RDQX_OD_MAX: Min = Min::ps(4, 35_000);
/// tRDQSTFE / tRDQSTFX (RD/WR-based RDQS_t training entry/exit): max(35 ns,
/// 4 nCK).
pub const T_RDQSTFE: Min = Min::ps(4, 35_000);
pub const T_RDQSTFX: Min = Min::ps(4, 35_000);

/// tZQCAL4 / tZQCAL8 / tZQCAL16 (ZQ start to latch, by the number of dies
/// on the resistor): 1.5 / 3 / 6 us. None for other die counts.
pub const fn t_zqcal_ps(dies: u32) -> Option<u32> {
    match dies {
        1..=4 => Some(1_500_000),
        5..=8 => Some(3_000_000),
        9..=16 => Some(6_000_000),
        _ => None,
    }
}
/// tZQLAT: max(30 ns, 4 nCK).
pub const T_ZQLAT: Min = Min::ps(4, 30_000);
/// tZQRESET: max(50 ns, 3 nCK).
pub const T_ZQRESET: Min = Min::ps(3, 50_000);
/// tZQSTOP: at most 30 ns.
pub const T_ZQSTOP_MAX_PS: u32 = 30_000;
/// tWCKSTOP after CAS(WS_OFF): max(6 ns, 2 nCK).
pub const T_WCKSTOP: Min = Min::ps(2, 6000);

/// Power-up and initialization. All minimums except `t_init0_max_ns`.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Init {
    /// tINIT0: maximum voltage ramp, ns (20 ms).
    pub t_init0_max_ns: u32,
    /// tINIT1: RESET_n low after the ramp, ns (200 us).
    pub t_init1_ns: u32,
    /// tINIT2: CS low before RESET_n high, ns.
    pub t_init2_ns: u32,
    /// tINIT3: CS low after RESET_n high, ns (2 ms).
    pub t_init3_ns: u32,
    /// tINIT4: stable clock before the first CS, tCK.
    pub t_init4_nck: u32,
    /// tINIT5: before the first MRW/MRR, ns (2 us).
    pub t_init5_ns: u32,
    /// tCKb: minimum boot clock period, ns.
    pub t_ckb_min_ns: u32,
    /// tPW_RESET: RESET_n pulse width, ns.
    pub t_pw_reset_ns: u32,
}

pub const INIT: Init = Init {
    t_init0_max_ns: 20_000_000,
    t_init1_ns: 200_000,
    t_init2_ns: 10,
    t_init3_ns: 2_000_000,
    t_init4_nck: 5,
    t_init5_ns: 2000,
    t_ckb_min_ns: 18,
    t_pw_reset_ns: 100,
};

/// tWCK2DQI (input oscillator path): high-frequency mode 300-700 ps up to
/// 6400 Mb/s, 250-600 ps above; low-frequency mode 300-900 ps.
pub const fn t_wck2dqi(high_frequency: bool, data_rate: u32) -> PsRange {
    if !high_frequency {
        PsRange { min: 300, max: 900 }
    } else if data_rate <= 6400 {
        PsRange { min: 300, max: 700 }
    } else {
        PsRange { min: 250, max: 600 }
    }
}
/// tWCK2DQO (output oscillator path): high-frequency 650-1600 ps,
/// low-frequency 650-1900 ps.
pub const fn t_wck2dqo(high_frequency: bool) -> PsRange {
    if high_frequency {
        PsRange {
            min: 650,
            max: 1600,
        }
    } else {
        PsRange {
            min: 650,
            max: 1900,
        }
    }
}

/// A duty-cycle adjuster range in tenths of a ps: the total range and one
/// step, each (min, max).
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct DcaRange {
    pub total_dps: (u32, u32),
    pub step_dps: (u32, u32),
}

/// WCK DCA (MR30): total 15-35 ps, step 2-5 ps; in E-DVFSC 10-42 / 1.5-6 ps.
pub const fn wck_dca_range(edvfsc: bool) -> DcaRange {
    if edvfsc {
        DcaRange {
            total_dps: (100, 420),
            step_dps: (15, 60),
        }
    } else {
        DcaRange {
            total_dps: (150, 350),
            step_dps: (20, 50),
        }
    }
}

/// Read DCA (MR69): total 15-35 ps with DVFSC off, 15-70 with DVFSC, 10-70
/// with E-DVFSC; step 2-5 / 2-10 / 1.5-10 ps.
pub const fn read_dca_range(dvfsc: super::mr::Dvfsc) -> DcaRange {
    match dvfsc {
        super::mr::Dvfsc::HighSpeed => DcaRange {
            total_dps: (150, 350),
            step_dps: (20, 50),
        },
        super::mr::Dvfsc::Dvfsc => DcaRange {
            total_dps: (150, 700),
            step_dps: (20, 100),
        },
        super::mr::Dvfsc::Enhanced => DcaRange {
            total_dps: (100, 700),
            step_dps: (15, 100),
        },
    }
}

/// A DFE range in mV, each (min, typ, max): the step, the first step and the
/// maximum feedback.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct DfeRange {
    pub step_mv: (u32, u32, u32),
    pub first_step_mv: (u32, u32, u32),
    pub max_mv: (u32, u32, u32),
}

/// DFE (MR24, only when WCK > 800 MHz): 3-step: step 5/10/15 mV, maximum
/// 15/30/45 mV; 7-step: step 4/7/10 mV, first step 4/10/15 mV, maximum
/// 35/50/65 mV.
pub const fn dfe_range(seven_step: bool) -> DfeRange {
    if seven_step {
        DfeRange {
            step_mv: (4, 7, 10),
            first_step_mv: (4, 10, 15),
            max_mv: (35, 50, 65),
        }
    } else {
        DfeRange {
            step_mv: (5, 10, 15),
            first_step_mv: (5, 10, 15),
            max_mv: (15, 30, 45),
        }
    }
}

/// MR1 OP[7:4] WL codes: (set A, set B) in nCK. WCK:CK 2:1 codes 0-5, 4:1
/// codes 0-13.
pub const WL_2_1: [(u8, u8); 6] = [(4, 4), (4, 6), (6, 8), (8, 10), (8, 14), (10, 16)];
pub const WL_4_1: [(u8, u8); 14] = [
    (2, 2),
    (2, 3),
    (3, 4),
    (4, 5),
    (4, 7),
    (5, 8),
    (6, 9),
    (6, 11),
    (7, 12),
    (8, 14),
    (9, 15),
    (9, 16),
    (11, 19),
    (12, 22),
];

/// MR2 OP[3:0] RL codes (link ECC, DVFSC and E-DVFSC off): (RL set 0, 1,
/// 2, nRBTP) in nCK. Set 0: no feature; set 1: byte mode or read DBI/data
/// copy; set 2: both.
pub const RL_2_1: [(u8, u8, u8, u8); 6] = [
    (6, 6, 6, 0),
    (8, 8, 8, 0),
    (10, 10, 12, 0),
    (12, 14, 14, 0),
    (16, 16, 18, 2),
    (18, 20, 20, 2),
];
pub const RL_4_1: [(u8, u8, u8, u8); 14] = [
    (3, 3, 3, 0),
    (4, 4, 4, 0),
    (5, 5, 6, 0),
    (6, 7, 7, 0),
    (8, 8, 9, 1),
    (9, 10, 10, 1),
    (10, 11, 12, 2),
    (12, 13, 14, 2),
    (13, 14, 15, 3),
    (15, 16, 17, 4),
    (16, 17, 19, 4),
    (17, 18, 20, 4),
    (20, 22, 24, 6),
    (23, 25, 26, 6),
];

/// One MR2 OP[7:4] nWR code at WCK:CK 4:1: the data-rate band (Mb/s,
/// upper end) and nWR in nCK for x16 and x8 without write link ECC, and
/// with it (None below 3200 Mb/s).
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct NwrCode {
    pub dr_max: u32,
    pub x16: u8,
    pub x8: u8,
    pub x16_ecc: Option<u8>,
    pub x8_ecc: Option<u8>,
}

const fn nwr(dr_max: u32, x16: u8, x8: u8, ecc: Option<(u8, u8)>) -> NwrCode {
    let (x16_ecc, x8_ecc) = match ecc {
        Some((a, b)) => (Some(a), Some(b)),
        None => (None, None),
    };
    NwrCode {
        dr_max,
        x16,
        x8,
        x16_ecc,
        x8_ecc,
    }
}

/// nWR codes 0-13 at 4:1.
pub const NWR_4_1: [NwrCode; 14] = [
    nwr(533, 3, 3, None),
    nwr(1067, 5, 5, None),
    nwr(1600, 7, 8, None),
    nwr(2133, 10, 10, None),
    nwr(2750, 12, 13, None),
    nwr(3200, 14, 15, None),
    nwr(3733, 16, 17, Some((18, 19))),
    nwr(4267, 19, 20, Some((21, 22))),
    nwr(4800, 21, 22, Some((23, 24))),
    nwr(5500, 24, 25, Some((27, 28))),
    nwr(6000, 26, 28, Some((29, 31))),
    nwr(6400, 28, 29, Some((31, 32))),
    nwr(7500, 32, 34, Some((36, 38))),
    nwr(8533, 37, 39, Some((41, 43))),
];

/// The 4:1 code (MR1 WL, MR2 RL and nWR use the same band index) for a data
/// rate in Mb/s, up to 8533.
pub const fn code_4_1(data_rate: u32) -> Option<u8> {
    let mut i = 0;
    while i < NWR_4_1.len() {
        if data_rate <= NWR_4_1[i].dr_max {
            return Some(i as u8);
        }
        i += 1;
    }
    None
}
