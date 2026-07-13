// SPDX-License-Identifier: GPL-2.0-only
//! LPDDR4/4X timing for controller set-up.
//!
//! Sources, so each number can be checked:
//! - [`FREQ_RANGES`]: the JEDEC LPDDR4/4X RL, WL, nWR and nRTP frequency
//!   ranges, as transcribed in LiteDRAM
//!   74522f715d7b10163ef6f6caa09887a90874be83 `litedram/init.py`
//!   (`get_lpddr4_phy_init_sequence`, `get_nwr`). The row index is the MR1
//!   nWR code and the MR2 RL/WL code ([`super::mr`]).
//! - [`MT53E256M16D1_1866`]: the LPDDR4 part timings in LiteDRAM
//!   `litedram/modules.py` (class `MT53E256M16D1`, speed grade 1866; tFAW
//!   per JEDEC LPDDR4/4X max(32 nCK, 40 ns)); tREFI = 32 ms / 8192.
//!
//! - [`refresh`]: tRFCab/tRFCpb per density of one physical channel,
//!   JEDEC LPDDR4/4X as transcribed in coreboot
//!   `util/spd_tools/src/spd_gen/lp4x.go` fec3d608 lines 280-309 (orchestrator
//!   `references/datasheets/open-source/coreboot_spd_tools_lp4x.go`, sha256
//!   de2d68d9..., GPL-2.0-or-later).
//! - [`T_RPAB`], [`T_RPPB`], [`t_xsr`]: Longsys FORESEE FLXC2002G-26
//!   LPDDR4X datasheet D-00226 v1.2, Table 13 "Core Timing Parameters"
//!   (PDF page 18) (`references/datasheets/vendor/longsys_FLXC2002G-26_
//!   lpddr4x_D-00226_v1.2.pdf`, sha256 0828469e...), the R2S class of part.
//! - [`INIT`]: the same datasheet, Table 12 "Initialization Timing
//!   Parameters" (PDF page 17).
//!
//! P1-40 added tXP, tCKE, tMRD, tMRW, tZQCAL, tZQLAT, tVREF and the other
//! timings at the end of this module, each a JEDEC LPDDR4/4X
//! value; [`crate::pending::LP4`] is now empty.

/// One row of the JEDEC LPDDR4/4X RL/WL/nWR frequency ranges: CK frequency (MHz) in (`f_min`, `f_max`].
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct FreqRange {
    /// Read latency, DBI-RD disabled and enabled (nCK).
    pub rl: u8,
    pub rl_dbi: u8,
    /// Write latency, set A and set B (nCK).
    pub wl_a: u8,
    pub wl_b: u8,
    /// Write recovery and read-to-precharge (nCK).
    pub nwr: u8,
    pub nrtp: u8,
    pub f_min: u32,
    pub f_max: u32,
}

/// One row in the source's layout: (RL, RL with DBI), (WL set A, set B),
/// nWR, nRTP, (f_min, f_max].
const fn r(rl: (u8, u8), wl: (u8, u8), nwr: u8, nrtp: u8, f: (u32, u32)) -> FreqRange {
    FreqRange {
        rl: rl.0,
        rl_dbi: rl.1,
        wl_a: wl.0,
        wl_b: wl.1,
        nwr,
        nrtp,
        f_min: f.0,
        f_max: f.1,
    }
}

/// The JEDEC LPDDR4/4X frequency ranges; index = MR code (MR1 OP[6:4], MR2 OP[2:0]/OP[5:3]).
pub const FREQ_RANGES: [FreqRange; 8] = [
    r((6, 6), (4, 4), 6, 8, (10, 266)),
    r((10, 12), (6, 8), 10, 8, (266, 533)),
    r((14, 16), (8, 12), 16, 8, (533, 800)),
    r((20, 22), (10, 18), 20, 8, (800, 1066)),
    r((24, 28), (12, 22), 24, 10, (1066, 1333)),
    r((28, 32), (14, 26), 30, 12, (1333, 1600)),
    r((32, 36), (16, 30), 34, 14, (1600, 1866)),
    r((36, 40), (18, 34), 40, 16, (1866, 2133)),
];

/// The frequency-range row (and MR code) for a CK frequency in MHz.
pub const fn freq_range(ck_mhz: u32) -> Option<(u8, FreqRange)> {
    let mut i = 0;
    while i < FREQ_RANGES.len() {
        let f = FREQ_RANGES[i];
        if ck_mhz > f.f_min && ck_mhz <= f.f_max {
            return Some((i as u8, f));
        }
        i += 1;
    }
    None
}

/// A JEDEC minimum: at least `nck` clocks and at least `ps` picoseconds.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Min {
    pub nck: u32,
    pub ps: u32,
}

impl Min {
    pub const fn ns(nck: u32, ns: u32) -> Min {
        Min { nck, ps: ns * 1000 }
    }
    /// Clocks of a `ck_mhz` clock: max(nCK, time rounded up).
    pub const fn cycles(self, ck_mhz: u32) -> u32 {
        let t = ps_to_ck(self.ps, ck_mhz);
        if t > self.nck {
            t
        } else {
            self.nck
        }
    }
}

/// A JEDEC maximum (tREFI, the average refresh interval): at most `ps`.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Max {
    pub ps: u32,
}

impl Max {
    /// Clocks of a `ck_mhz` clock, rounded down (never longer than allowed).
    pub const fn cycles(self, ck_mhz: u32) -> u32 {
        (self.ps as u64 * ck_mhz as u64 / 1_000_000) as u32
    }
}

/// Picoseconds to whole cycles of a `mhz` clock, rounded up.
pub const fn ps_to_ck(ps: u32, mhz: u32) -> u32 {
    ((ps as u64 * mhz as u64).div_ceil(1_000_000)) as u32
}

/// Core timings of one LPDDR4 part and speed grade.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Timings {
    pub t_rp: Min,
    pub t_rcd: Min,
    pub t_wr: Min,
    pub t_rfc: Min,
    pub t_faw: Min,
    pub t_ras: Min,
    pub t_wtr: Min,
    pub t_rrd: Min,
    /// Write-to-write (masked writes: 32 nCK).
    pub t_ccd: Min,
    /// Average refresh interval, a maximum.
    pub t_refi: Max,
}

/// Micron MT53E256M16D1, speed grade 1866 (LiteDRAM `modules.py`).
pub const MT53E256M16D1_1866: Timings = Timings {
    t_rp: Min::ns(3, 21),
    t_rcd: Min::ns(4, 18),
    t_wr: Min::ns(4, 18),
    t_rfc: Min::ns(0, 180),
    t_faw: Min::ns(32, 40),
    t_ras: Min::ns(3, 42),
    t_wtr: Min::ns(8, 10),
    t_rrd: Min::ns(4, 10),
    t_ccd: Min { nck: 32, ps: 0 },
    // 32 ms / 8192 refreshes = 3906.25 ns.
    t_refi: Max { ps: 3_906_250 },
};

/// tRFCab and tRFCpb of one refresh-timing class.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Refresh {
    /// REFRESH cycle time, all banks.
    pub t_rfc_ab: Min,
    /// REFRESH cycle time, per bank.
    pub t_rfc_pb: Min,
}

/// tRFCab/tRFCpb for a density of `gb` Gb per physical channel (3, 4, 6,
/// 8, 12 or 16; the same for x8 and x16 devices): JEDEC LPDDR4/4X via
/// coreboot `lp4x.go` fec3d608 lines 280-309 (`LP4xDensityGbPhysicalChannel
/// ToRefreshEncoding`). Longsys FLXC2002G-26 Table 13 (8 Gb/channel) gives
/// the same 280/140 ns.
pub const fn refresh(gb: u8) -> Option<Refresh> {
    let (ab, pb) = match gb {
        3 | 4 => (180, 90),
        6 | 8 => (280, 140),
        12 | 16 => (380, 190),
        _ => return None,
    };
    Some(Refresh {
        t_rfc_ab: Min::ns(0, ab),
        t_rfc_pb: Min::ns(0, pb),
    })
}

/// Row precharge time, all banks: MAX(21 ns, 4 nCK) (Longsys FLXC2002G-26
/// Table 13, p.18).
pub const T_RPAB: Min = Min::ns(4, 21);
/// Row precharge time, single bank: MAX(18 ns, 4 nCK) (Longsys
/// FLXC2002G-26 Table 13, p.18).
pub const T_RPPB: Min = Min::ns(4, 18);

/// Self refresh exit to next valid command: MAX(tRFCab + 7.5 ns, 2 nCK)
/// (Longsys FLXC2002G-26 Table 13, p.18).
pub const fn t_xsr(t_rfc_ab: Min) -> Min {
    Min {
        nck: 2,
        ps: t_rfc_ab.ps + 7_500,
    }
}

/// Power-up and initialization timing (Longsys FLXC2002G-26 Table 12,
/// p.17). All values are minimums except `t_init0_max_ms`.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Init {
    /// tINIT0: maximum voltage ramp time, ms.
    pub t_init0_max_ms: u32,
    /// tINIT1: RESET_n LOW after the voltage ramp, us.
    pub t_init1_us: u32,
    /// tINIT2: CKE LOW before RESET_n goes HIGH, ns.
    pub t_init2_ns: u32,
    /// tINIT3: CKE LOW after RESET_n goes HIGH, ms.
    pub t_init3_ms: u32,
    /// tINIT4: stable clock before the first CKE HIGH, nCK.
    pub t_init4_nck: u32,
    /// tINIT5: idle before the first MRW/MRR, us.
    pub t_init5_us: u32,
}

/// [`Init`] values of Longsys FLXC2002G-26 Table 12 (p.17).
pub const INIT: Init = Init {
    t_init0_max_ms: 20,
    t_init1_us: 200,
    t_init2_ns: 10,
    t_init3_ms: 2,
    t_init4_nck: 5,
    t_init5_us: 2,
};

// ---------------------------------------------------------------------------
// P1-40: further timings. Each: JEDEC LPDDR4/4X value.

impl Min {
    /// A minimum with a picosecond part (for values such as 7.5 ns).
    pub const fn ps(nck: u32, ps: u32) -> Min {
        Min { nck, ps }
    }
}

/// tXP, exit power-down to a valid command: max(7.5 ns, 5 nCK).
/// JEDEC LPDDR4/4X value.
pub const T_XP: Min = Min::ps(5, 7500);
/// tCKE, minimum CKE high/low pulse: max(7.5 ns, 4 nCK).
/// JEDEC LPDDR4/4X value.
pub const T_CKE: Min = Min::ps(4, 7500);
/// tCKELPD, CKE low pulse in self refresh: max(7.5 ns, 3 nCK).
/// JEDEC LPDDR4/4X value.
pub const T_CKELPD: Min = Min::ps(3, 7500);
/// tMRW, MRW to any other command: max(10 ns, 10 nCK).
/// JEDEC LPDDR4/4X value.
pub const T_MRW: Min = Min::ps(10, 10_000);
/// tMRD, MRW to a command using the new value: max(14 ns, 10 nCK).
/// JEDEC LPDDR4/4X value.
pub const T_MRD: Min = Min::ps(10, 14_000);
/// tZQCAL, ZQ calibration time: 1 us (LPDDR4 has no tZQCAL4).
/// JEDEC LPDDR4/4X value.
pub const T_ZQCAL: Min = Min::ps(0, 1_000_000);
/// tZQLAT, ZQ latch to next command: max(30 ns, 8 nCK).
/// JEDEC LPDDR4/4X value.
pub const T_ZQLAT: Min = Min::ps(8, 30_000);
/// tZQRESET: max(50 ns, 3 nCK). JEDEC LPDDR4/4X value.
pub const T_ZQRESET: Min = Min::ps(3, 50_000);
/// tCKFSPE / tCKFSPX, clock hold around a frequency set point switch:
/// max(7.5 ns, 4 nCK). JEDEC LPDDR4/4X value.
pub const T_CKFSPE: Min = Min::ps(4, 7500);
pub const T_CKFSPX: Min = Min::ps(4, 7500);
/// tSDO, delay to output on DQ after an MPC/MRR in CBT: max(12 nCK, 20 ns)
/// (recorded as the maximum form). JEDEC LPDDR4/4X value.
pub const T_SDO: Min = Min::ps(12, 20_000);

/// Step size class of a Vref (MR12/MR14) or frequency-point change.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum VrefStep {
    Short,
    Middle,
    Long,
    /// VRCG off (MR13 OP[3] = 0): the weak, slow settling.
    Weak,
}

/// tVREF, settling after an MR12 (CA) or MR14 (DQ) write: SHORT 100 ns,
/// MIDDLE 200 ns, LONG 250 ns, weak (VRCG off) 1 ms; the same for CA and DQ.
/// JEDEC LPDDR4/4X value.
pub const fn t_vref(step: VrefStep) -> Min {
    Min::ps(
        0,
        match step {
            VrefStep::Short => 100_000,
            VrefStep::Middle => 200_000,
            VrefStep::Long => 250_000,
            VrefStep::Weak => 1_000_000_000,
        },
    )
}

/// tVREFca during command bus training, x16 mode: LONG 250 ns, SHORT 80 ns
/// (no MIDDLE or weak value). JEDEC LPDDR4/4X value.
pub const fn t_vref_ca_cbt_x16(step: VrefStep) -> Option<Min> {
    match step {
        VrefStep::Long => Some(Min::ps(0, 250_000)),
        VrefStep::Short => Some(Min::ps(0, 80_000)),
        _ => None,
    }
}

/// tVREFca during command bus training, x8 (byte) mode 2: LONG 250 ns,
/// MIDDLE 200 ns, SHORT 100 ns. JEDEC LPDDR4/4X value.
pub const fn t_vref_ca_cbt_x8_mode2(step: VrefStep) -> Option<Min> {
    match step {
        VrefStep::Long => Some(Min::ps(0, 250_000)),
        VrefStep::Middle => Some(Min::ps(0, 200_000)),
        VrefStep::Short => Some(Min::ps(0, 100_000)),
        VrefStep::Weak => None,
    }
}

/// tVRCG_ENABLE 200 ns, tVRCG_DISABLE 100 ns (VREF current generator).
/// JEDEC LPDDR4/4X value.
pub const T_VRCG_ENABLE: Min = Min::ps(0, 200_000);
pub const T_VRCG_DISABLE: Min = Min::ps(0, 100_000);

/// tFC, frequency set point switch: short 200 ns, middle 200 ns, long
/// 250 ns. JEDEC LPDDR4/4X value.
pub const fn t_fc(step: VrefStep) -> Option<Min> {
    match step {
        VrefStep::Short | VrefStep::Middle => Some(Min::ps(0, 200_000)),
        VrefStep::Long => Some(Min::ps(0, 250_000)),
        VrefStep::Weak => None,
    }
}

/// tXCBT, command bus training exit: short 200 ns, middle 200 ns, long
/// 250 ns, each at least 5 nCK. JEDEC LPDDR4/4X value.
pub const fn t_xcbt(step: VrefStep) -> Option<Min> {
    match step {
        VrefStep::Short | VrefStep::Middle => Some(Min::ps(5, 200_000)),
        VrefStep::Long => Some(Min::ps(5, 250_000)),
        VrefStep::Weak => None,
    }
}
