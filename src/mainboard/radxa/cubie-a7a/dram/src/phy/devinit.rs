//! DevInit before the DRAM reset is released (boot pstate), without the PMU:
//! our own code for what the provisional [`prefix`](super::prefix) table
//! replayed. Register names are from the EIC7700X TRM register list (same
//! PHY family). The step structure and the selection rule of the RxReplica
//! calibration were established from the PMU emulator (reference oracle,
//! `tools/pmu-trace`, varied synthetic DBYTE inputs); under the oracle's
//! inputs this code writes exactly the oracle's values
//! (`tests/fixtures/devinit-prefix-pstate1.txt`). Read-modify-write steps
//! use the live register values, as the firmware does.
//!
//! Groups, in order: PMU-side set-up with DRAM held in reset; impedance
//! calibration (ZCal); power-on-reset latch and PLL lock; DLL lock;
//! RxReplica calibration (per DBYTE: path phase selection against the 1-UI
//! LCDL measurement, ratio baseline); LCDL observation; DLL gains and lock
//! parameters; ACSM read pulse timing adjustment.
//!
//! Still opaque: the two PIE words at 0x9002e/0x9002f, 0x2f0a5/0x2f077 and
//! 0x70011 (oracle constants), and the constant DLL gains / UI calibration
//! wait (fixed in the oracle for every input tried).

use super::{csr, csr_access};
use crate::hw::Hw;

const POR_CONTROL: u32 = 0x20090;
const PIE_WORDS: [(u32, u16); 2] = [(0x9002e, 0x715d), (0x9002f, 0x00ce)];
const CLOCKING_CTRL: u32 = 0x200a2;
const TX_RD_PTR_INIT: u32 = 0x200f8;
const MEM_RESET_L: u32 = 0x20060;
const ACSM_CKE_CONTROL: u32 = 0x20028;
const UNNAMED_SETUP: [(u32, u16); 2] = [(0x2f0a5, 1), (0x2f077, 0)];
const PUBMODE: u32 = 0x2006e;
const PRBS_GEN_CTL: u32 = 0x70060;
const PRBS_RESERVED: u32 = 0x70011;
const PSTATE: u32 = 0x2008b;
const HWT_LP_CS_EN_A: u32 = 0x20072;
const HWT_LP_CS_EN_B: u32 = 0x20073;
const PPT_TRAIN_SETUP2: u32 = 0x20011;
const fn ppt_ctl_static(n: u32) -> u32 {
    0x100a3 + 0x1000 * n
}
const ZCAL_RATE: u32 = 0x20303;
const ZCAL_CLR_FIRST_RUN_DONE: u32 = 0x2030d;
const ZCAL_RESET: u32 = 0x20310;
const ZCAL_RUN: u32 = 0x20311;
const ZCAL_BUSY: u32 = 0x20312;
const ZQ_UPDATE: u32 = 0x20320;
const CLR_POR_MEM_RESET: u32 = 0x20061;
const PLL_CTRL3: u32 = 0x20049;
const PLL_BYPASS_MODE: u32 = 0x200e1;
const PLL_STANDBY: u32 = 0x200e0;
const PLL_PWR_DN: u32 = 0x200c3;
const PLL_START_LOCK: u32 = 0x200e2;
const DLL_CONTROL: u32 = 0x20078;
const PCLK_GATE_CONTROL: u32 = 0x20200;
const LCDL_DBG_CNTL: u32 = 0x200e3;
const LCDL_DBG_CNTL2: u32 = 0x200e5;
const AC_LCDL_STATUS: u32 = 0x300bf;
const DX_LCDL_STATUS: u32 = 0x100e0;
const AC_DLL_LOCK_PARAM: u32 = 0x2000a;
const DX_DLL_LOCK_PARAM: u32 = 0x20017;
const DLL_GAIN_CTL: u32 = 0x20012;
const RX_REPLICA_DLL_GAIN_CTL: u32 = 0x20186;
const RX_REPLICA_DLL_LOCK_PARAM: u32 = 0x20187;
/// ACSM read pulse timing registers adjusted by +4 (pstate values from
/// PhyInit): RxEn, RxVal, Rdcs, WckReadStaticLo; StaticHi takes StaticLo's
/// new value; then WckReadToggle, WckReadFastToggle.
const ACSM_RX_EN_PULSE: u32 = 0x2002c;
const ACSM_RX_VAL_PULSE: u32 = 0x2002d;
const ACSM_RDCS_PULSE: u32 = 0x20030;
const ACSM_WCK_READ_STATIC_LO: u32 = 0x20039;
const ACSM_WCK_READ_STATIC_HI: u32 = 0x2003a;
const ACSM_WCK_READ_TOGGLE: u32 = 0x2003b;
const ACSM_WCK_READ_FAST_TOGGLE: u32 = 0x2003c;
const ACSM_WCK_FREE_RUN: u32 = 0x20041;
const ACSM_WCK_EN_CONTROL: u32 = 0x2002a;
const PULSE_ADJUST: u16 = 4;

/// DBYTE `n` RxReplica registers (n = 0xf: broadcast to all DBYTEs).
const fn dx(n: u32, off: u32) -> u32 {
    0x10000 + 0x1000 * n + off
}
const RX_REPLICA_CTL04: u32 = 0x00f;
const RX_CLK_CNTL1: u32 = 0x057;
/// RxReplicaCtl: bit 0 rising latches the status registers below.
const RX_REPLICA_CTL: u32 = 0x0ac;
const RX_REPLICA_CTL01: u32 = 0x0ad;
const RX_REPLICA_CTL03: u32 = 0x0af;
const RX_REPLICA_PATH_PHASE0: u32 = 0x0d0;
const RX_REPLICA_LCDL_PH1UI: u32 = 0x0d5;
const RX_REPLICA_RATIO_NOW: u32 = 0x0d6;
const RX_REPLICA_UI_CAL_WAIT: u32 = 0x0d9;
const BROADCAST: u32 = 0xf;
const DBYTES: u32 = 4;

/// RxReplicaUICalWait before and after the calibration, and the DLL gains
/// and lock parameters: constant in the oracle for every input tried.
const UI_CAL_WAIT_RUN: u16 = 0x1000;
const UI_CAL_WAIT: u16 = 0x9c;
const DLL_GAIN: u16 = 0x7342;
const RX_REPLICA_DLL_GAIN: u16 = 0x73;
/// PllCtrl3 written between the POR latch pulses.
const PLL_CTRL3_VALUE: u16 = 0x1f0;
/// PclkGateControl override during DLL lock.
const PCLK_GATE_DLL_LOCK: u16 = 0x33;

/// Waits where the firmware waited (emulator cycles in parentheses); the
/// fixed values are far above what those gaps represent.
pub const ZCAL_WAIT_US: u32 = 1_000; // (19553)
pub const DLL_LOCK_WAIT_US: u32 = 1_000; // (60047)
pub const REPLICA_RUN_WAIT_US: u32 = 1_000; // (6172, 8104)
/// ZCalBusy poll budget after the fixed wait.
pub const ZCAL_POLLS: u32 = 10_000;

/// One DBYTE's RxReplica calibration.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Replica {
    /// RxReplicaPathPhase0..4.
    pub phases: [u16; 5],
    /// RxReplicaLcdlPh1UI.
    pub ph1ui: u16,
    /// Selected path phase (RxReplicaCtl01).
    pub select: u8,
    /// RxReplicaRatioNow, written as the baseline (RxReplicaCtl03).
    pub ratio: u16,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DevinitError {
    /// Every path phase satisfies 2 * phase <= Ph1UI: no phase to select
    /// (the oracle fails training here).
    ReplicaPhase(u8),
}

/// What [`pre_release`] measured.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct DevinitReport {
    pub zcal_rate: u16,
    /// ZCalBusy right after ZCalRun, after the fixed wait, and the polls
    /// until it read 0 (ZCAL_POLLS: never).
    pub zcal_busy_start: u16,
    pub zcal_busy_after_wait: u16,
    pub zcal_polls: u32,
    /// RxReplicaCtl04 of DBYTE 0 before the POR step, and of DBYTEs 0..3
    /// after the broadcast write (does a broadcast reach the per-DBYTE
    /// registers?).
    pub ctl04_before: u16,
    pub ctl04_after: [u16; 4],
    pub replica: [Replica; 4],
    /// RxReplicaLcdlPh1UI of DBYTE 0 at the end (RxReplicaDllLockParam).
    pub dll_lock_param: u16,
    /// AcLcdlStatus, DxLcdlStatus during the LCDL observation.
    pub lcdl_status: [u16; 2],
    pub failure: Option<DevinitError>,
}

impl DevinitReport {
    pub const EMPTY: DevinitReport = DevinitReport {
        zcal_rate: 0,
        zcal_busy_start: 0,
        zcal_busy_after_wait: 0,
        zcal_polls: 0,
        ctl04_before: 0,
        ctl04_after: [0; 4],
        replica: [Replica::EMPTY; 4],
        dll_lock_param: 0,
        lcdl_status: [0; 2],
        failure: None,
    };
}

impl Replica {
    pub const EMPTY: Replica = Replica {
        phases: [0; 5],
        ph1ui: 0,
        select: 0,
        ratio: 0,
    };
}

/// The path phase the oracle selects: the number of leading phases with
/// 2 * phase <= Ph1UI (scan from phase 0, stop at the first that is
/// larger). None when all five qualify.
pub fn select_phase(phases: &[u16; 5], ph1ui: u16) -> Option<u8> {
    let mut i = 0;
    while i < 5 && 2 * phases[i] as u32 <= ph1ui as u32 {
        i += 1;
    }
    (i < 5).then_some(i as u8)
}

struct Phy<'a, H: Hw>(&'a mut H);

impl<H: Hw> Phy<'_, H> {
    fn r(&mut self, a: u32) -> u16 {
        self.0.read16(csr(a))
    }
    fn w(&mut self, a: u32, v: u16) {
        self.0.write16(csr(a), v)
    }
    fn pulse_latch(&mut self, n: u32) {
        let c = self.r(dx(n, RX_REPLICA_CTL));
        self.w(dx(n, RX_REPLICA_CTL), c | 1);
        self.w(dx(n, RX_REPLICA_CTL), c & !1);
    }
    /// RxClk timing mode run on `n` (or broadcast): RxClkCntl1[0] and
    /// RxReplicaCtl04[1:0] = 01 for the wait, then restored.
    fn rxclk_run(&mut self, n: u32, src: u32) {
        let c57 = self.r(dx(src, RX_CLK_CNTL1));
        self.w(dx(n, RX_CLK_CNTL1), c57 | 1);
        let c04 = self.r(dx(src, RX_REPLICA_CTL04));
        self.w(dx(n, RX_REPLICA_CTL04), (c04 & !3) | 1);
        self.0.udelay(REPLICA_RUN_WAIT_US);
        self.w(dx(n, RX_CLK_CNTL1), c57 & !1);
        self.w(dx(n, RX_REPLICA_CTL04), c04);
    }
}

/// DevInit before reset release. Call after `blobless::pre_training` (PMU
/// stalled); leaves CPU CSR access selected and the DRAM in reset.
pub fn pre_release<H: Hw>(hw: &mut H, rep: &mut DevinitReport) {
    *rep = DevinitReport::EMPTY;
    csr_access(hw, true);
    let mut p = Phy(hw);

    // PMU-side set-up, DRAM held in reset.
    p.w(POR_CONTROL, 1);
    for (a, v) in PIE_WORDS {
        p.w(a, v);
    }
    let cc = p.r(CLOCKING_CTRL);
    p.w(CLOCKING_CTRL, cc | 1);
    p.w(TX_RD_PTR_INIT, 1);
    p.w(MEM_RESET_L, 2);
    p.w(ACSM_CKE_CONTROL, 1);
    for (a, v) in UNNAMED_SETUP {
        p.w(a, v);
    }
    p.w(PUBMODE, 1);
    p.w(PRBS_GEN_CTL, 0);
    p.w(PRBS_RESERVED, 0);
    p.w(PSTATE, 0);
    p.w(HWT_LP_CS_EN_A, 3);
    p.w(HWT_LP_CS_EN_B, 3);
    let pts = p.r(PPT_TRAIN_SETUP2);
    p.w(PPT_TRAIN_SETUP2, pts | 0xf);
    for n in 0..DBYTES {
        let v = p.r(ppt_ctl_static(n));
        p.w(ppt_ctl_static(n), v & 0xc);
    }

    // Impedance calibration.
    let rate = p.r(ZCAL_RATE);
    rep.zcal_rate = rate;
    p.w(ZCAL_RESET, 1);
    p.w(ZCAL_RATE, rate | 0x10);
    p.w(ZCAL_RESET, 0);
    p.w(ZCAL_CLR_FIRST_RUN_DONE, 1);
    p.w(ZCAL_CLR_FIRST_RUN_DONE, 0);
    p.w(ZCAL_RUN, 1);
    rep.zcal_busy_start = p.r(ZCAL_BUSY);
    p.0.udelay(ZCAL_WAIT_US);
    rep.zcal_busy_after_wait = p.r(ZCAL_BUSY);
    rep.zcal_polls = ZCAL_POLLS;
    for i in 0..ZCAL_POLLS {
        if p.r(ZCAL_BUSY) & 1 == 0 {
            rep.zcal_polls = i;
            break;
        }
    }
    p.w(ZCAL_RUN, 0);
    p.w(ZQ_UPDATE, 0);
    p.w(ZQ_UPDATE, 1);
    p.w(ZQ_UPDATE, 0);
    p.w(ZCAL_RATE, rate);

    // Power-on-reset latch, PLL lock.
    let c04 = p.r(dx(0, RX_REPLICA_CTL04));
    rep.ctl04_before = c04;
    p.w(dx(BROADCAST, RX_REPLICA_CTL04), c04 & !0x80);
    for n in 0..DBYTES {
        rep.ctl04_after[n as usize] = p.r(dx(n, RX_REPLICA_CTL04));
    }
    p.w(CLR_POR_MEM_RESET, 1);
    p.w(PLL_CTRL3, PLL_CTRL3_VALUE);
    p.w(CLR_POR_MEM_RESET, 0);
    p.w(PLL_BYPASS_MODE, 0);
    p.w(ZCAL_RESET, 0);
    p.w(PLL_STANDBY, 0);
    p.w(PLL_PWR_DN, 0);
    for v in [3, 2, 0] {
        p.w(PLL_START_LOCK, v);
    }

    // DLL lock.
    p.w(DLL_CONTROL, 1);
    p.w(PCLK_GATE_CONTROL, PCLK_GATE_DLL_LOCK);
    p.w(CLOCKING_CTRL, 0);
    p.w(DLL_CONTROL, 2);
    p.w(CLOCKING_CTRL, 2);
    p.0.udelay(DLL_LOCK_WAIT_US);
    p.w(DLL_CONTROL, 0);
    p.w(CLOCKING_CTRL, 0);
    p.w(PCLK_GATE_CONTROL, 0);
    p.w(TX_RD_PTR_INIT, 0);
    p.w(CLOCKING_CTRL, 2);

    // RxReplica calibration.
    p.w(dx(BROADCAST, RX_REPLICA_UI_CAL_WAIT), UI_CAL_WAIT_RUN);
    let c04 = p.r(dx(0, RX_REPLICA_CTL04));
    p.w(dx(BROADCAST, RX_REPLICA_CTL04), c04 & !1);
    let c57 = p.r(dx(0, RX_CLK_CNTL1));
    p.w(dx(BROADCAST, RX_CLK_CNTL1), c57 & !1);
    let cac = p.r(dx(0, RX_REPLICA_CTL));
    p.w(dx(BROADCAST, RX_REPLICA_CTL), cac & !1);
    let c04 = p.r(dx(0, RX_REPLICA_CTL04));
    p.w(dx(BROADCAST, RX_REPLICA_CTL04), c04 | 1);
    p.0.udelay(REPLICA_RUN_WAIT_US);
    p.w(dx(BROADCAST, RX_REPLICA_CTL04), c04 & !1);
    for n in 0..DBYTES {
        let mut r = Replica::default();
        for (k, ph) in r.phases.iter_mut().enumerate() {
            *ph = p.r(dx(n, RX_REPLICA_PATH_PHASE0 + k as u32));
        }
        p.pulse_latch(n);
        r.ph1ui = p.r(dx(n, RX_REPLICA_LCDL_PH1UI));
        let Some(sel) = select_phase(&r.phases, r.ph1ui) else {
            rep.replica[n as usize] = r;
            rep.failure = Some(DevinitError::ReplicaPhase(n as u8));
            return;
        };
        r.select = sel;
        p.w(dx(n, RX_REPLICA_CTL01), sel as u16);
        p.rxclk_run(n, n);
        p.pulse_latch(n);
        r.ratio = p.r(dx(n, RX_REPLICA_RATIO_NOW));
        p.w(dx(n, RX_REPLICA_CTL03), r.ratio);
        rep.replica[n as usize] = r;
    }
    p.rxclk_run(BROADCAST, 0);

    // LCDL observation (status kept for the report).
    p.w(LCDL_DBG_CNTL2, 0);
    for v in [0x800, 0xc00, 0x800] {
        p.w(LCDL_DBG_CNTL, v);
    }
    rep.lcdl_status = [p.r(AC_LCDL_STATUS), p.r(DX_LCDL_STATUS)];
    p.w(LCDL_DBG_CNTL, 0);
    p.w(LCDL_DBG_CNTL2, 0);

    // DLL gains and lock parameters.
    p.w(AC_DLL_LOCK_PARAM, 0);
    p.w(DX_DLL_LOCK_PARAM, 0);
    p.w(DLL_GAIN_CTL, DLL_GAIN);
    p.w(RX_REPLICA_DLL_GAIN_CTL, RX_REPLICA_DLL_GAIN);
    rep.dll_lock_param = p.r(dx(0, RX_REPLICA_LCDL_PH1UI));
    p.w(RX_REPLICA_DLL_LOCK_PARAM, rep.dll_lock_param);
    for n in 0..DBYTES {
        p.w(dx(n, RX_REPLICA_UI_CAL_WAIT), UI_CAL_WAIT);
    }
    p.w(TX_RD_PTR_INIT, 0);
    let cc = p.r(CLOCKING_CTRL);
    p.w(CLOCKING_CTRL, cc | 2);

    // ACSM read pulse timing.
    for a in [ACSM_RX_EN_PULSE, ACSM_RX_VAL_PULSE, ACSM_RDCS_PULSE] {
        let v = p.r(a);
        p.w(a, v.wrapping_add(PULSE_ADJUST));
    }
    let lo = p.r(ACSM_WCK_READ_STATIC_LO).wrapping_add(PULSE_ADJUST);
    p.w(ACSM_WCK_READ_STATIC_LO, lo);
    p.w(ACSM_WCK_READ_STATIC_HI, lo);
    for a in [ACSM_WCK_READ_TOGGLE, ACSM_WCK_READ_FAST_TOGGLE] {
        let v = p.r(a);
        p.w(a, v.wrapping_add(PULSE_ADJUST));
    }
    p.w(ACSM_WCK_FREE_RUN, 0);
    p.w(ACSM_WCK_EN_CONTROL, 1);
}
