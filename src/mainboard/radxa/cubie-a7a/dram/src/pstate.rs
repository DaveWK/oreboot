// SPDX-License-Identifier: GPL-2.0-only
//! The operating point of the pstate our firmware-free stages train, and
//! the values that follow from it.
//!
//! The blob-free flow was built and proven at pstate 1 (WCK 1200 MHz,
//! 2400 MT/s); production runs pstate 0 (WCK 2400 MHz, 4800 MT/s). This
//! module is the one place that says what changes between them, so the
//! stages can take their pstate-dependent values from here instead of
//! pstate-1 literals. Build-time knob `A7A_PSTATE` (0..3; unset: 1) selects
//! the pstate of the firmware-free stages; unset, every generator produces
//! exactly what it did before (the pstate-1 values).
//!
//! Provenance per value:
//! - **PUBLIC**: the JEDEC LPDDR5/5X latency and timing tables as
//!   transcribed in `jedec_lpddr` (LiteDRAM 74522f7);
//!   the ACSM register formulas of the EIC7700X TRM Part 2.
//! - **MEASURED**: the ACSM REPEATCNT unit (the `acsm_probe` cases:
//!   a row pair with REPEATCNT N and flag F10 runs (N + 1) x 4 DfiClk; rows
//!   without F10 run once whatever their count; 3:1 between pstate 1 and
//!   pstate 3); the controller timing sets (our own code's values, read
//!   back on the board).
//! - **REFERENCE-DERIVED**: values of the reference at pstate 0
//!   against pstate 1 (bus events only): the low-speed ACSM clock register, the DLL gain, the
//!   WL measurement program. They are flagged where they are used.
//! - **VENDOR-BOARD-DATA**: the parameter set (`sets.rs`): dividers, MR
//!   inputs, rails.

use crate::para::Para;

const fn digit(s: Option<&str>, default: u32) -> u32 {
    match s {
        None => default,
        Some(t) => {
            let b = t.as_bytes();
            if b.len() != 1 || b[0] < b'0' || b[0] > b'3' {
                panic!("A7A_PSTATE must be 0, 1, 2 or 3");
            }
            (b[0] - b'0') as u32
        }
    }
}

/// The pstate the firmware-free stages train (env `A7A_PSTATE` at build
/// time; unset: 1, the pstate the blob-free flow was built at).
pub const ACTIVE: u32 = digit(option_env!("A7A_PSTATE"), 1);

/// Whether `A7A_PSTATE` was given (bt0 checks it against its other pstate
/// knobs).
pub const ACTIVE_FROM_ENV: bool = option_env!("A7A_PSTATE").is_some();

/// The DfiClk (= CK) per pstate of the Cubie A7A 4 GB sets (dram_clk 2400,
/// tpr2 0x170b0703: dividers 4, 8, 12, 24). VENDOR-BOARD-DATA; a unit test
/// checks it against `sets.rs`, and bt0's blobless backend refuses a set
/// whose DfiClk at [`ACTIVE`] differs (the generators use this table at
/// build time).
pub const BOARD_DFI_MHZ: [u32; 4] = [600, 300, 200, 100];
/// The DfiClk of [`ACTIVE`].
pub const ACTIVE_DFI_MHZ: u32 = BOARD_DFI_MHZ[ACTIVE as usize];
/// VDDQ of the 4 GB sets (tpr3 [15:8] = 0x38, 10 mV units).
/// VENDOR-BOARD-DATA (and the rail audit: 560 mV read back).
pub const BOARD_VDDQ_MV: u32 = 560;

/// One pstate's clocks. LPDDR5 at WCK:CK 4:1 (MR18 OP[7] = 0): WCK = 4 CK,
/// data rate = 2 WCK; the PHY's DfiClk is CK on this SoC (0x20004 reads
/// 600 at pstate 0, feature `av3h`; 300 at pstate 1).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Point {
    pub pstate: u32,
    pub dfi_mhz: u32,
}

impl Point {
    /// The board table's point ([`BOARD_DFI_MHZ`]).
    pub const fn board(pstate: u32) -> Point {
        Point {
            pstate,
            dfi_mhz: BOARD_DFI_MHZ[(pstate & 3) as usize],
        }
    }
    /// The point of `pstate` in parameter set `p` (dram_clk / (div + 1)).
    pub fn of(p: &Para, pstate: u32) -> Point {
        Point {
            pstate,
            dfi_mhz: p.pstate_mhz(pstate as usize) / 4,
        }
    }
    pub const fn ck_mhz(&self) -> u32 {
        self.dfi_mhz
    }
    pub const fn wck_mhz(&self) -> u32 {
        4 * self.dfi_mhz
    }
    /// Mb/s per pin.
    pub const fn data_rate(&self) -> u32 {
        8 * self.dfi_mhz
    }
    /// tCK in ps (rounded down).
    pub const fn tck_ps(&self) -> u32 {
        1_000_000 / self.dfi_mhz
    }
    /// One data UI in ps (rounded to nearest): 417 at pstate 1, 208 at 0.
    pub const fn ui_ps(&self) -> u32 {
        (1_000_000 + self.data_rate() / 2) / self.data_rate()
    }
}

// ---------------------------------------------------------------------------
// ACSM timing (MEASURED unit).

/// DfiClk per REPEATCNT step of a row pair with F10 (the `acsm_probe` cases:
/// T21 count 64 = 260 DfiClk per execution at pstates 1 and 3; T38 the
/// override 200 = 804).
pub const ACSM_DFICLK_PER_COUNT: u64 = 4;
/// REPEATCNT is 8 bits: one pair waits at most 256 steps.
pub const ACSM_COUNT_MAX: u32 = 255;

/// Duration of one execution of an F10 row pair with REPEATCNT `count` at
/// `dfi_mhz`, in ps (rounded down): (count + 1) x 4 DfiClk.
pub const fn acsm_wait_ps(count: u32, dfi_mhz: u32) -> u64 {
    (count as u64 + 1) * ACSM_DFICLK_PER_COUNT * 1_000_000 / dfi_mhz as u64
}

/// The smallest REPEATCNT whose F10 pair waits at least `t_ps` at
/// `dfi_mhz`. May exceed [`ACSM_COUNT_MAX`]: the caller then splits the
/// wait into pairs ([`acsm_split`]).
pub const fn acsm_count_for(t_ps: u64, dfi_mhz: u32) -> u32 {
    let units = (t_ps * dfi_mhz as u64).div_ceil(ACSM_DFICLK_PER_COUNT * 1_000_000);
    if units == 0 {
        0
    } else {
        (units - 1) as u32
    }
}

/// The REPEATCNT that keeps an F10 wait's duration when DfiClk goes from
/// `from_mhz` to `to_mhz` (rounded up, so never shorter). At the same clock
/// it returns `count` unchanged. The reference's WCK2CK measurement program
/// scales its time-based waits exactly this way from pstate 1 to pstate 0
/// (29 -> 59, 5 -> 11; REFERENCE-DERIVED check in the tests).
pub const fn acsm_scale(count: u32, from_mhz: u32, to_mhz: u32) -> u32 {
    let units = ((count as u64 + 1) * to_mhz as u64).div_ceil(from_mhz as u64);
    (units - 1) as u32
}

/// `total` REPEATCNT steps + 1 per pair split into F10 pairs of at most
/// 256 steps each: the counts of up to `N` pairs, and how many are used.
/// Returns None if `N` pairs cannot hold the wait.
pub const fn acsm_split<const N: usize>(t_ps: u64, dfi_mhz: u32) -> Option<([u32; N], usize)> {
    let mut units = (t_ps * dfi_mhz as u64).div_ceil(ACSM_DFICLK_PER_COUNT * 1_000_000);
    if units == 0 {
        units = 1;
    }
    let mut out = [0u32; N];
    let mut k = 0;
    while units > 0 {
        if k == N {
            return None;
        }
        let u = if units > 256 { 256 } else { units };
        out[k] = (u - 1) as u32;
        units -= u;
        k += 1;
    }
    Some((out, k))
}

// ---------------------------------------------------------------------------
// The low-speed ACSM clock of cold init, CBT, end of initialization and the
// WCK2CK leveling's MR18 edges.

/// ACSMLowSpeedClockEnable / ACSMLowSpeedClockDelay for one DfiClk.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LowSpeed {
    /// [6:0]: the ACSM clock is DfiClk / (2 x div), the AC bump clock
    /// (the slow CK the DRAM sees) DfiClk / (4 x div) with [9:8] = 1.
    pub div: u16,
    /// 0x100 | div: the row form of cold init (two rows per slow CK).
    pub enable: u16,
    /// 0x180 | div: ACSM at full speed, slow AC clock (the held-edge MR18
    /// form of WCK2CK leveling).
    pub enable_full: u16,
    /// ACSMLowSpeedClockDelay (DfiClk to the first clock edge).
    pub delay: u16,
}

/// The low-speed clock that keeps the slow CK at 40 ns (25 MHz) whatever
/// the DfiClk: div = ceil(DfiClk / 100 MHz), delay = div - 2 (0 at most).
/// REFERENCE-DERIVED rule: the reference writes 0x106/0x186 with delay 4 at
/// pstate 0 (600 MHz), 0x103/0x183 with delay 1 at pstate 1 (300 MHz),
/// 0x102/0x182 with delay 0 at pstate 2 (200 MHz). The register formula
/// (DfiClk / (2 x [6:0]), bump clock x 2 for [9:8] = 1) is PUBLIC (EIC7700X
/// TRM Part 2, ACSMLowSpeedClockEnable). The slow CK stays above JEDEC's
/// tCKb minimum of 18 ns at every pstate (test).
pub const fn low_speed(dfi_mhz: u32) -> LowSpeed {
    let mut div = dfi_mhz.div_ceil(100);
    if div == 0 {
        div = 1;
    }
    if div > 0x7f {
        div = 0x7f;
    }
    let div = div as u16;
    LowSpeed {
        div,
        enable: 0x100 | div,
        enable_full: 0x180 | div,
        delay: div.saturating_sub(2),
    }
}

/// The slow CK period of `ls` at `dfi_mhz`, in ps.
pub const fn slow_ck_ps(ls: LowSpeed, dfi_mhz: u32) -> u32 {
    4 * ls.div as u32 * 1_000_000 / dfi_mhz
}

/// The low-speed clock of [`ACTIVE`]: 0x103 / 0x183 / 1 at pstate 1 (the
/// values every stage used before this knob existed).
pub const ACTIVE_LOW_SPEED: LowSpeed = if ACTIVE == 1 {
    LowSpeed {
        div: 3,
        enable: 0x103,
        enable_full: 0x183,
        delay: 1,
    }
} else {
    low_speed(ACTIVE_DFI_MHZ)
};

/// DllGainCtl (0x20012) DevInit writes, per DfiClk. REFERENCE-DERIVED
/// (0x7353 at 600 MHz, 0x7342 at 300 MHz, 0x5331 at 200 MHz). None for a
/// clock the reference does not cover.
pub const fn dll_gain(dfi_mhz: u32) -> Option<u16> {
    match dfi_mhz {
        600 => Some(0x7353),
        300 => Some(0x7342),
        200 => Some(0x5331),
        _ => None,
    }
}

/// DllGainCtl at [`ACTIVE`] (0x7342, the pstate-1 value, where the reference
/// has none).
pub const ACTIVE_DLL_GAIN: u16 = match dll_gain(ACTIVE_DFI_MHZ) {
    Some(v) if ACTIVE != 3 => v,
    _ => 0x7342,
};

// ---------------------------------------------------------------------------
// Mode registers and latencies per pstate.

/// Where a value comes from (the module's classes, plus COMPUTED for a
/// value derived here from the others).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Source {
    Public,
    Measured,
    TraceDerived,
    VendorBoardData,
    Computed,
}

/// The latencies JEDEC LPDDR5 (4:1) gives one MR1/MR2 code, in nCK.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Latencies {
    pub code: u8,
    pub rl_set0: u8,
    /// Read DBI on (MR3 OP[6]) or byte mode: RL set 1.
    pub rl_set1: u8,
    pub wl_set_a: u8,
    pub wl_set_b: u8,
    pub nwr: u8,
    pub nrbtp: u8,
    /// tWCKENL_RD (set 0), tWCKPRE_static, tWCKPRE_toggle_RD.
    pub wckenl_rd: u8,
    pub wckpre_static: u8,
    pub wckpre_toggle_rd: u8,
    /// tWCKENL_WR (set A), tWCKPRE_toggle_WR.
    pub wckenl_wr: u8,
    pub wckpre_toggle_wr: u8,
}

/// The JEDEC row of MR1/MR2 code `code` (4:1). PUBLIC.
pub const fn latencies(code: u8) -> Option<Latencies> {
    use jedec_lpddr::lp5::timing::{FREQ_RANGES_4_1, NWR_4_1, RL_4_1, WL_4_1};
    let c = code as usize;
    if c >= FREQ_RANGES_4_1.len() {
        return None;
    }
    let f = FREQ_RANGES_4_1[c];
    let rl = RL_4_1[c];
    let wl = WL_4_1[c];
    Some(Latencies {
        code,
        rl_set0: rl.0,
        rl_set1: rl.1,
        wl_set_a: wl.0,
        wl_set_b: wl.1,
        nwr: NWR_4_1[c].x16,
        nrbtp: rl.3,
        wckenl_rd: f.t_wckenl_rd[0],
        wckpre_static: f.t_wckpre_static,
        wckpre_toggle_rd: f.t_wckpre_toggle_rd,
        wckenl_wr: f.t_wckenl_wr[0],
        wckpre_toggle_wr: f.t_wckpre_toggle_wr,
    })
}

/// The mode registers our firmware-free flow programs at one pstate, from
/// the message block (as the end of initialization takes them), with what
/// they mean. All MR inputs come from the parameter set through
/// `msgblock::build` and the DevInit bit rules (VENDOR-BOARD-DATA and
/// REFERENCE-DERIVED rules); their decoding is PUBLIC.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Params {
    pub point: Point,
    /// The code JEDEC requires at this data rate (the smallest allowed).
    pub code_required: u8,
    /// [MR1, MR2, MR3, MR10, MR11, MR12, MR13, MR14, MR15, MR17 (rank 0,
    /// rank 1), MR18, MR19, MR20, MR22, MR24, MR25, MR28, MR30, MR41] as the
    /// end of initialization writes them to channel A.
    pub mr1: u8,
    pub mr2: u8,
    pub mr3: u8,
    pub mr10: u8,
    pub mr11: u8,
    pub mr12: u8,
    pub mr13: u8,
    pub mr14: u8,
    pub mr15: u8,
    pub mr17: [u8; 2],
    pub mr18: u8,
    pub mr19: u8,
    pub mr20: u8,
    pub mr22: u8,
    pub mr24: u8,
    pub mr25: u8,
    pub mr28: u8,
    pub mr30: u8,
    pub mr41: u8,
    /// The latencies MR1/MR2 select (their codes must agree).
    pub lat: Latencies,
}

impl Params {
    /// The flow's values for `pstate` of parameter set `p` (`p` is fixed
    /// up as the trained path's timing step leaves it).
    pub fn of(p: &mut Para, pstate: u32) -> Option<Params> {
        use crate::phy::endinit::FspMrs;
        use crate::phy::wcklevel::{Channel, Rank};
        crate::timing::mr_fixup(p);
        let m = crate::blobless::message_block(p, pstate);
        let get = |r: Rank| FspMrs::from_message_block(&m, Channel::A, r);
        let r0 = get(Rank::R0);
        let r1 = get(Rank::R1);
        let v = |f: &FspMrs, ma: u8| f.mr.iter().find(|x| x.0 == ma).map(|x| x.1).unwrap_or(0);
        let point = Point::of(p, pstate);
        let mr1 = v(&r0, 1);
        let mr2 = v(&r0, 2);
        if mr1 >> 4 != mr2 & 0xf {
            return None;
        }
        Some(Params {
            point,
            code_required: jedec_lpddr::lp5::timing::code_4_1(point.data_rate())?,
            mr1,
            mr2,
            mr3: v(&r0, 3),
            mr10: v(&r0, 10),
            mr11: v(&r0, 11),
            mr12: v(&r0, 12),
            mr13: v(&r0, 13),
            mr14: v(&r0, 14),
            mr15: v(&r0, 15),
            mr17: [v(&r0, 17), v(&r1, 17)],
            mr18: v(&r0, 18),
            mr19: v(&r0, 19),
            mr20: v(&r0, 20),
            mr22: v(&r0, 22),
            mr24: v(&r0, 24),
            mr25: v(&r0, 25),
            mr28: v(&r0, 28),
            mr30: v(&r0, 30),
            mr41: v(&r0, 41),
            lat: latencies(mr2 & 0xf)?,
        })
    }

    /// The MR1/MR2 code is allowed at this data rate (a code at or above the
    /// required one: higher latencies at a lower rate are legal).
    pub const fn code_ok(&self) -> bool {
        self.lat.code >= self.code_required
    }
    /// The read latency the DRAM runs with MR3 as programmed.
    pub const fn rl(&self) -> u8 {
        if self.mr3 & 0x40 != 0 {
            self.lat.rl_set1
        } else {
            self.lat.rl_set0
        }
    }
    /// The write latency (MR3 OP[5]: set B).
    pub const fn wl(&self) -> u8 {
        if self.mr3 & 0x20 != 0 {
            self.lat.wl_set_b
        } else {
            self.lat.wl_set_a
        }
    }
    /// MR18 OP[3]: WCK high-frequency mode (WCK DCA, MR30, only applies
    /// there: JEDEC).
    pub const fn wck_high_frequency(&self) -> bool {
        self.mr18 & 0x08 != 0
    }
    /// The highest DQ Vref code (MR14/MR15) inside JEDEC's absolute limit
    /// at this WCK and VDDQ.
    pub const fn dq_vref_code_max(&self, vddq_mv: u32) -> u8 {
        dq_vref_code_max(self.point.wck_mhz(), vddq_mv)
    }
}

/// The highest MR14/MR15 code (10 % + 0.5 % per code of VDDQ) whose level
/// stays within JEDEC's DQ Vref maximum at `wck_mhz` (350 mV up to 1600
/// MHz, 225 mV up to 3200 MHz): 0x69 at pstate
/// 1, 0x3c at pstate 0 with VDDQ 560 mV. PUBLIC limit.
pub const fn dq_vref_code_max(wck_mhz: u32, vddq_mv: u32) -> u8 {
    let (_, max_mv) = jedec_lpddr::lp5::mr::dq_vref_mv(wck_mhz, false);
    // level = vddq * (100 + 5 code) / 1000  <=  max_mv
    let c = (max_mv * 1000 / vddq_mv).saturating_sub(100) / 5;
    if c > 0x7f {
        0x7f
    } else {
        c as u8
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use crate::sets::{CXMT_4G, CXMT_6G, HYNIX_4G, PARAM_SETS};
    use jedec_lpddr::lp5::timing as jt;
    use std::collections::HashMap;

    fn set(i: usize) -> Para {
        Para::from_sysconfig(&PARAM_SETS[i].para)
    }

    #[test]
    fn board_dfi_clocks_follow_the_4gb_sets() {
        for i in [CXMT_4G, CXMT_6G, HYNIX_4G] {
            let p = set(i);
            for ps in 0..4u32 {
                assert_eq!(Point::of(&p, ps), Point::board(ps), "set {i} ps {ps}");
            }
            assert_eq!((p.tpr3() >> 8 & 0xff) * 10, BOARD_VDDQ_MV);
        }
        let p0 = Point::board(0);
        assert_eq!(
            (p0.wck_mhz(), p0.data_rate(), p0.ui_ps(), p0.tck_ps()),
            (2400, 4800, 208, 1666)
        );
        let p1 = Point::board(1);
        assert_eq!(
            (p1.wck_mhz(), p1.data_rate(), p1.ui_ps(), p1.tck_ps()),
            (1200, 2400, 417, 3333)
        );
        // The default build is pstate 1 and keeps the pstate-1 literals.
        if !ACTIVE_FROM_ENV {
            assert_eq!(ACTIVE, 1);
            assert_eq!(ACTIVE_LOW_SPEED, low_speed(300));
            assert_eq!(ACTIVE_DLL_GAIN, 0x7342);
        }
    }

    #[test]
    fn acsm_unit_reproduces_phase_a() {
        // T21: count 64, outer 255 (256 executions): 222 us at pstate 1,
        // 666 us at pstate 3.
        let t1 = 256 * acsm_wait_ps(64, 300) / 1_000_000;
        let t3 = 256 * acsm_wait_ps(64, 100) / 1_000_000;
        assert_eq!((t1, t3), (221, 665));
        // T38: the override 200: 687 / 2056 us.
        assert_eq!(256 * acsm_wait_ps(200, 300) / 1_000_000, 686);
        assert_eq!(256 * acsm_wait_ps(200, 100) / 1_000_000, 2058);
        // Count for a time, and the split into pairs.
        assert_eq!(acsm_count_for(2_000_000, 300), 149);
        assert_eq!(acsm_count_for(2_000_000, 600), 299);
        assert_eq!(acsm_split::<4>(2_000_000, 600), Some(([255, 43, 0, 0], 2)));
        assert_eq!(acsm_split::<4>(2_000_000, 300), Some(([149, 0, 0, 0], 1)));
        assert_eq!(acsm_split::<1>(2_000_000, 600), None);
        for mhz in [100, 200, 300, 600] {
            for t in [1u64, 20_000, 250_000, 2_000_000] {
                let n = acsm_count_for(t, mhz);
                assert!(acsm_wait_ps(n, mhz) >= t, "{t} at {mhz}");
                if n > 0 {
                    assert!(acsm_wait_ps(n - 1, mhz) < t, "minimal: {t} at {mhz}");
                }
            }
        }
    }

    #[test]
    fn time_preserving_scale_matches_the_trace_at_pstate_0() {
        // The reference's WCK2CK measurement program: the head and the
        // second time wait go 0x5f00/0x0c00 (29) -> 0x3f00/0x1c00 (59) and
        // 0x5e00/0x0000 (5) -> 0x3e00/0x0400 (11) from pstate 1 to 0.
        assert_eq!(acsm_scale(29, 300, 600), 59);
        assert_eq!(acsm_scale(5, 300, 600), 11);
        assert_eq!(acsm_wait_ps(29, 300), acsm_wait_ps(59, 600));
        // Same clock: unchanged (the pstate-1 path is bit for bit).
        for n in 0..=255 {
            assert_eq!(acsm_scale(n, 300, 300), n);
            assert!(acsm_wait_ps(acsm_scale(n, 300, 600), 600) >= acsm_wait_ps(n, 300));
        }
    }

    #[test]
    fn low_speed_clock_follows_the_trace_and_keeps_tckb() {
        let t = |mhz| {
            let l = low_speed(mhz);
            (l.enable, l.enable_full, l.delay)
        };
        assert_eq!(t(600), (0x106, 0x186, 4));
        assert_eq!(t(300), (0x103, 0x183, 1));
        assert_eq!(t(200), (0x102, 0x182, 0));
        for mhz in [100, 200, 300, 450, 600, 800] {
            let ps = slow_ck_ps(low_speed(mhz), mhz);
            assert!(ps >= jt::INIT.t_ckb_min_ns * 1000, "{mhz}: {ps}");
        }
        assert_eq!(slow_ck_ps(low_speed(600), 600), 40_000);
        assert_eq!(slow_ck_ps(low_speed(300), 300), 40_000);
        // The pstate-1 literals at 600 MHz would give 20 ns: above tCKb
        // still, but not the reference's 40 ns.
        assert_eq!(slow_ck_ps(low_speed(300), 600), 20_000);
    }

    #[test]
    fn dll_gain_per_clock() {
        assert_eq!(dll_gain(300), Some(0x7342));
        assert_eq!(dll_gain(600), Some(0x7353));
        assert_eq!(dll_gain(100), None);
    }

    #[test]
    fn mode_registers_and_latencies_at_pstates_0_and_1() {
        for i in [CXMT_4G, CXMT_6G, HYNIX_4G] {
            let mut p = set(i);
            let m0 = Params::of(&mut p, 0).unwrap();
            let m1 = Params::of(&mut p, 1).unwrap();
            // The message block gives the same MRs at both pstates.
            assert_eq!(
                (m0.mr1, m0.mr2, m0.mr3, m0.mr10, m0.mr11, m0.mr12, m0.mr13, m0.mr18),
                (m1.mr1, m1.mr2, m1.mr3, m1.mr10, m1.mr11, m1.mr12, m1.mr13, m1.mr18)
            );
            assert_eq!((m0.mr1, m0.mr2, m0.mr3), (0x80, 0x88, 0xc6));
            assert_eq!(
                (m0.mr10, m0.mr13, m0.mr18, m0.mr19, m0.mr20),
                (0x54, 0x60, 0x0c, 0x10, 0x02)
            );
            assert_eq!(
                (m0.mr22, m0.mr24, m0.mr25, m0.mr28, m0.mr30, m0.mr41),
                (0, 0, 0, 0x04, 0, 0x60)
            );
            assert_eq!(m0.mr12, 0x44);
            assert_eq!(m0.mr14, PARAM_SETS[i].para[18] as u8);
            // Code 8 is exactly JEDEC's code at 4800 Mb/s, and legal (above
            // the required code 4) at 2400 Mb/s.
            assert_eq!((m0.code_required, m1.code_required), (8, 4));
            assert!(m0.code_ok() && m1.code_ok());
            assert_eq!(m0.lat.code, 8);
            // RL 13 (set 0) with the operating MR3 0x06, 14 with read DBI
            // (our end of init's 0xc6); WL 7 (set A); nWR 21.
            assert_eq!(
                (m0.lat.rl_set0, m0.lat.rl_set1, m0.lat.wl_set_a, m0.lat.nwr),
                (13, 14, 7, 21)
            );
            assert_eq!(m0.rl(), 14);
            assert_eq!(m0.wl(), 7);
            assert_eq!(
                (
                    m0.lat.wckenl_rd,
                    m0.lat.wckpre_static,
                    m0.lat.wckpre_toggle_rd
                ),
                (5, 3, 6)
            );
            // MR18 0x0c: WCK ODT RZQ/4 and high-frequency mode at both
            // pstates (the PMU's operating MR18 read 0x00 at pstate 1).
            assert!(m0.wck_high_frequency() && m1.wck_high_frequency());
            // DQ Vref: no cap at 1200 MHz WCK inside the sweep, 0x3c at 2400.
            assert_eq!(m1.dq_vref_code_max(BOARD_VDDQ_MV), 0x69);
            assert_eq!(m0.dq_vref_code_max(BOARD_VDDQ_MV), 0x3c);
            let level = |c: u32| BOARD_VDDQ_MV * (100 + 5 * c) / 1000;
            assert!(level(0x3c) <= 225 && level(0x3d) > 225);
        }
    }

    /// Register model for the controller set computation.
    struct Ctl(HashMap<usize, u32>);
    impl crate::hw::RegIo for Ctl {
        fn read32(&mut self, a: usize) -> u32 {
            *self.0.get(&a).unwrap_or(&0)
        }
        fn write32(&mut self, a: usize, v: u32) {
            self.0.insert(a, v);
        }
        fn read16(&mut self, _: usize) -> u16 {
            0
        }
        fn write16(&mut self, _: usize, _: u16) {}
        fn udelay(&mut self, _: u32) {}
        fn now_us(&mut self) -> u64 {
            0
        }
        fn log(&mut self, _: core::fmt::Arguments) {}
    }
    impl crate::hw::Hw for Ctl {
        fn set_voltage(&mut self, _: crate::hw::Rail, _: u32) -> bool {
            true
        }
    }

    #[test]
    fn controller_sets_0_and_1_as_measured() {
        use crate::ctrl::freq;
        let mut p = set(CXMT_4G);
        let mut hw = Ctl(HashMap::new());
        assert_eq!(
            crate::timing::set_channel_timing(&mut hw, &mut p, 0),
            Ok(13)
        );
        // DRAMSET1TMG0..7, +0x24, +0x38, +0x60, +0x64 read back on the
        // board after our timing step (before the
        // trained path's CDD fold, which moves set 0 +0x08/+0x24/+0x60):
        // MEASURED.
        const SET0: [(usize, u32); 12] = [
            (0x000, 0x1f0c141a),
            (0x004, 0x00050527),
            (0x008, 0x070d1013),
            (0x00c, 0x000a1d25),
            (0x010, 0x0b04030d),
            (0x014, 0x02030907),
            (0x018, 0x00000007),
            (0x01c, 0x00000002),
            (0x024, 0x0002040e),
            (0x038, 0x001800e9),
            (0x060, 0x000e120c),
            (0x064, 0x00001f05),
        ];
        const SET1: [(usize, u32); 12] = [
            (0x000, 0x1f060a0d),
            (0x004, 0x00030314),
            (0x008, 0x070d100f),
            (0x00c, 0x000a1a24),
            (0x010, 0x06040207),
            (0x014, 0x02030504),
            (0x018, 0x00000005),
            (0x01c, 0x00000001),
            (0x024, 0x0002030d),
            (0x038, 0x000c0075),
            (0x060, 0x000e120c),
            (0x064, 0x00001f03),
        ];
        for (ps, want) in [(0, &SET0), (1, &SET1)] {
            for &(off, v) in want.iter() {
                assert_eq!(hw.read32_at(freq(0, ps, off)), v, "set {ps} +{off:#x}");
            }
        }
        // Read and write latency fields of DRAMSET1TMG2: RL 13, WL 7 in
        // both sets (MR2 0x88 / MR1 0x80 at both pstates).
        for ps in [0, 1] {
            let t2 = hw.read32_at(freq(0, ps, 0x008));
            assert_eq!((t2 >> 16 & 0x3f, t2 >> 24 & 0x3f), (13, 7));
        }
    }

    impl Ctl {
        fn read32_at(&mut self, a: usize) -> u32 {
            crate::hw::RegIo::read32(self, a)
        }
    }
}
