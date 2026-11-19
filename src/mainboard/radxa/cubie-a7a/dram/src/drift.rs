// SPDX-License-Identifier: GPL-2.0-only
//! Run-time drift logging (feature `drift-log`: lab builds only).
//!
//! After training nothing adjusts the interface: no DFI update completes
//! and no PHY retraining runs. This module measures what moves with
//! temperature and voltage, so that the drift can be compared with the
//! trained windows before any compensation is designed. It compensates
//! nothing. One [`sample`] of a channel returns:
//!
//! - per rank, the DRAM's WCK2DQI (write path) and WCK2DQO (read path)
//!   interval oscillator counts (MR35/MR36 and MR38/MR39);
//! - per rank, MR4: OP[4:0] is the refresh multiplier code the DRAM's
//!   temperature sensor asks for, OP[7] its temperature update flag;
//! - for the channel's two DBYTEs, the PHY's own indicators: the DLL code
//!   for one UI (`DxPhase1UI`), the LCDL lock status, the RX replica
//!   snapshot (path delay over one UI, the replica's 1-UI code, the
//!   correction term with its valid and range bits, path phase 0), the
//!   RxClk tracking error flags and the current-delay viewer;
//! - the PHY's DFI update counters of the channel and the PLL lock.
//!
//! Running the oscillators. The controller has no software MPC. Its
//! DQS/WCK oscillator sequencer (`DQSOSCCTL0.dqsosc_enable`, a dynamic
//! field) sends the start command to each active rank, waits the
//! controller's run time and reads the result registers; `DQSOSCSTAT0`
//! holds its state and one start bit per rank. [`sample`] enables it for
//! one pass, disables it, waits for idle and then reads the counts by
//! software MRR: they stay in the DRAM until the next start.
//!
//! - The sequencer cannot stop an oscillator by MPC, so the DRAM's run
//!   time (MR37, MR40) must not be 0: where a rank holds another value
//!   than the caller's code, [`sample`] writes the code.
//! - The controller's own run time (`DQSOSCRUNTIME`) must not be shorter
//!   than the DRAM's, or its reads would land in a running oscillator.
//!   [`sample`] checks that and changes nothing there. It also refuses
//!   when the sequencer is enabled or busy already. Every refusal comes
//!   before the first write.
//! - The PHY is told a run time as well (`TrackingModeCntrl`
//!   `DqsOscRunTimeSel`: 2048 clocks in this tree's tables, code
//!   [`RUN_TIME_2048`]). The PHY can snoop the sequencer's reads for its
//!   own drift compensation, which this tree does not program: the first
//!   time this runs on a state, compare the current-delay viewer and the
//!   trained delays before and after.
//!
//! Every write: MRW of MR37/MR40 where the run time differs; the sequencer
//! enable, set and cleared; the MRR commands (reading MR4 clears its
//! update flag); the CSR access select and the RX replica's snapshot
//! strobe, both put back as found.
//!
//! Count to delay: delay = run time / (2 x count), the relation Linux
//! `drivers/memory/tegra/tegra210-emc-cc-r21021.c` uses for the LPDDR4 DQS
//! oscillator; the run time is in CK cycles. On this board the PHY
//! training firmware's state read counts 7245 and 2461 at CK 300 MHz with
//! run time code 0x40: 471 ps and 1387 ps, inside the WCK2DQI and WCK2DQO
//! ranges of [`jedec_lpddr::lp5::timing`].
//!
//! Public register basis (EIC7700X TRM Part 2, the same DDRCTL and
//! LPDDR5/4 PHY family): DQSOSCCTL0 (frequency set +0xa80), DQSOSCRUNTIME
//! (+0x300), DQSOSCSTAT0 (+0x304), DERATECTL6 (MR4[7] is the update flag)
//! and DERATECTL0 (MR4[4:0], LPDDR5); TrackingModeCntrl 0x20014,
//! RxReplicaCtl 0x1n0ac, RxReplicaPathPhase0 0x1n0d0, RxReplicaLcdlPh1UI,
//! RxReplicaRatioNow and RxReplicaRxClkDlyCorrection 0x1n0d5-0x1n0d7,
//! DxPhase1UI 0x1n0df, DxLcdlStatus 0x1n0e0, RxClkTrkErr 0x1n099,
//! DbCurrentDlyTimingInfoTg0 0x1n08f, the update counters 0x200b5-0x200bc
//! and PllLockStatus 0x200c9.

use crate::ctrl::{ddrc, freq, MSTR0};
use crate::hw::{wait, Error, Hw};
use crate::phy::{csr, csr_access, MICRO_CONT_MUX_SEL};
use dram_train::osc::Osc;
use jedec_lpddr::lp5::mr::{OscRunTime, Wck2DqOsc};
use jedec_lpddr::lp5::timing::{t_wck2dqi, t_wck2dqo};

/// MR37/MR40 code for a run time of 2048 CK cycles, the one the PHY's
/// tables in this tree name.
pub const RUN_TIME_2048: u32 = 0x40;

/// [`sample`] flag: strobe the RX replica's status snapshot (a write to
/// `RxReplicaCtl`, put back as found). Without it the replica registers
/// are read as they stand.
pub const FLAG_REPLICA_SNAP: u32 = 1;

/// [`Sample::status`] bits. 0: everything ran.
pub mod status {
    /// Channel, frequency set or run-time code outside what exists.
    pub const ARG: u32 = 1 << 0;
    /// A mode-register command failed while the run time was set up; the
    /// oscillators were not run.
    pub const RUN_TIME: u32 = 1 << 1;
    /// The sequencer was enabled or not idle already; it was left alone.
    pub const BUSY: u32 = 1 << 2;
    /// Not every active rank's start bit came up in time.
    pub const START: u32 = 1 << 3;
    /// The sequencer did not return to idle after the disable.
    pub const IDLE: u32 = 1 << 4;
    /// The controller's run time is shorter than the DRAM's; the
    /// oscillators were not run.
    pub const CTL_SHORT: u32 = 1 << 5;
    /// The oscillators were not asked for (run-time code 0).
    pub const NOT_RUN: u32 = 1 << 6;
}

/// In place of a mode-register value: the command timed out, was refused,
/// or the rank is not active.
pub const MR_TIMEOUT: u32 = 0xdead_0001;
pub const MR_CONFIG: u32 = 0xdead_0002;
pub const MR_OTHER: u32 = 0xdead_0003;
pub const MR_NO_RANK: u32 = 0xdead_0004;

const MR4: u32 = 4;
const MSTR2: usize = 0x008;
const DQSOSCRUNTIME: usize = 0x300;
const DQSOSCSTAT0: usize = 0x304;
const DQSOSCCTL0: usize = 0xa80;
const DQSOSC_ENABLE: u32 = 1;
const DQSOSC_STATE: u32 = 7;
const DQSOSC_RANK_SHIFT: u32 = 4;
/// Frequency sets of the A733's controllers.
const SETS: u32 = 4;
/// One interval of the sequencer is about 1 M DFI clocks in this tree's
/// timing; both ranks start within a few of them.
const START_TIMEOUT_US: u64 = 200_000;
const IDLE_TIMEOUT_US: u64 = 50_000;
const STOP_US: u32 = 200;

const RANKS: usize = 2;
const DBYTES_PER_CHANNEL: usize = 2;

const fn db(n: usize, off: u32) -> u32 {
    0x10000 + 0x1000 * n as u32 + off
}
const RX_REPLICA_CTL: u32 = 0x0ac;
const RX_REPLICA_SNAP_NOW: u16 = 1;
const DX_PHASE_1UI: u32 = 0x0df;
const DX_LCDL_STATUS: u32 = 0x0e0;
const RX_REPLICA_PATH_PHASE0: u32 = 0x0d0;
const RX_REPLICA_LCDL_PH1UI: u32 = 0x0d5;
const RX_REPLICA_RATIO_NOW: u32 = 0x0d6;
const RX_REPLICA_CORRECTION: u32 = 0x0d7;
const RX_CLK_TRK_ERR: u32 = 0x099;
const DB_CURRENT_DLY_TG0: u32 = 0x08f;
const MAS_UPD_GOOD_CTR: u32 = 0x200b5;
const PHY_UPD0_GOOD_CTR: u32 = 0x200b6;
const CTL_UPD0_GOOD_CTR: u32 = 0x200b8;
const PHY_UPD0_FAIL_CTR: u32 = 0x200bb;
const PLL_LOCK_STATUS: u32 = 0x200c9;

/// One rank's mode registers; each value may be an `MR_*` code instead.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct RankSample {
    pub mr4: u32,
    /// WCK2DQI count, MR36 << 8 | MR35.
    pub wck2dqi: u32,
    /// WCK2DQO count, MR39 << 8 | MR38.
    pub wck2dqo: u32,
    /// The run times as found: MR37 [7:0], MR40 [15:8]; bit 16 if either
    /// was written.
    pub run_time: u32,
}

/// One DBYTE's indicators, raw.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct DbyteSample {
    pub phase_1ui: u16,
    pub lcdl_status: u16,
    pub replica_ratio: u16,
    pub replica_1ui: u16,
    pub replica_correction: u16,
    pub replica_path0: u16,
    pub rxclk_trk_err: u16,
    pub current_dly_tg0: u16,
}

/// One channel's sample.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Sample {
    pub status: u32,
    /// Active frequency set [4:0], active ranks [9:8], the run-time code
    /// used [23:16].
    pub info: u32,
    /// DQSOSCSTAT0 after the pass.
    pub osc_stat: u32,
    pub rank: [RankSample; RANKS],
    pub dbyte: [DbyteSample; DBYTES_PER_CHANNEL],
    /// MasUpdGoodCtr, PhyUpd`ch`GoodCtr, CtlUpd`ch`GoodCtr,
    /// PhyUpd`ch`FailCtr, PllLockStatus, the CSR access select as found.
    pub master: [u16; 6],
}

/// Output words of one sample, the lab op's layout.
pub const WORDS: usize = 22;

const fn pair(lo: u16, hi: u16) -> u32 {
    lo as u32 | (hi as u32) << 16
}
const fn lo(w: u32) -> u16 {
    w as u16
}
const fn hi(w: u32) -> u16 {
    (w >> 16) as u16
}

impl Sample {
    /// Words 0-2 status, info, osc_stat; 3-10 the ranks (mr4, wck2dqi,
    /// wck2dqo, run_time); 11-18 the DBYTEs, two 16-bit values per word in
    /// field order, the first in the low half; 19-21 `master`, likewise.
    pub fn words(&self) -> [u32; WORDS] {
        let mut w = [0; WORDS];
        w[0] = self.status;
        w[1] = self.info;
        w[2] = self.osc_stat;
        for (r, s) in self.rank.iter().enumerate() {
            w[3 + 4 * r..7 + 4 * r].copy_from_slice(&[s.mr4, s.wck2dqi, s.wck2dqo, s.run_time]);
        }
        for (d, s) in self.dbyte.iter().enumerate() {
            w[11 + 4 * d..15 + 4 * d].copy_from_slice(&[
                pair(s.phase_1ui, s.lcdl_status),
                pair(s.replica_ratio, s.replica_1ui),
                pair(s.replica_correction, s.replica_path0),
                pair(s.rxclk_trk_err, s.current_dly_tg0),
            ]);
        }
        for k in 0..3 {
            w[19 + k] = pair(self.master[2 * k], self.master[2 * k + 1]);
        }
        w
    }

    pub fn from_words(w: &[u32; WORDS]) -> Self {
        let mut s = Sample {
            status: w[0],
            info: w[1],
            osc_stat: w[2],
            ..Default::default()
        };
        for (r, k) in s.rank.iter_mut().zip([3, 7]) {
            *r = RankSample {
                mr4: w[k],
                wck2dqi: w[k + 1],
                wck2dqo: w[k + 2],
                run_time: w[k + 3],
            };
        }
        for (d, k) in s.dbyte.iter_mut().zip([11, 15]) {
            *d = DbyteSample {
                phase_1ui: lo(w[k]),
                lcdl_status: hi(w[k]),
                replica_ratio: lo(w[k + 1]),
                replica_1ui: hi(w[k + 1]),
                replica_correction: lo(w[k + 2]),
                replica_path0: hi(w[k + 2]),
                rxclk_trk_err: lo(w[k + 3]),
                current_dly_tg0: hi(w[k + 3]),
            };
        }
        for k in 0..3 {
            s.master[2 * k] = lo(w[19 + k]);
            s.master[2 * k + 1] = hi(w[19 + k]);
        }
        s
    }
}

/// MR4 OP[4:0]: the refresh multiplier code (it rises with temperature).
pub const fn mr4_refresh_code(op: u32) -> u32 {
    op & 0x1f
}
/// MR4 OP[7]: the code changed since MR4 was last read.
pub const fn mr4_updated(op: u32) -> bool {
    op & 0x80 != 0
}

/// CK cycles of a run-time code; None for 0 (stop by MPC) and the codes
/// the encoding leaves out.
pub const fn run_clocks(code: u8) -> Option<u32> {
    match OscRunTime::decode(code) {
        Some(OscRunTime::Clocks(n)) => Some(n),
        _ => None,
    }
}

/// The oscillator arithmetic for `osc` at a CK period of `tck_ps` and
/// run-time code `code`, with the delay range of the WCK frequency mode
/// the DRAM is in (`high_frequency`) at `data_rate` Mb/s.
pub fn oscillator(
    osc: Wck2DqOsc,
    tck_ps: u32,
    code: u8,
    high_frequency: bool,
    data_rate: u32,
) -> Option<Osc> {
    let range = match osc {
        Wck2DqOsc::Input => t_wck2dqi(high_frequency, data_rate),
        Wck2DqOsc::Output => t_wck2dqo(high_frequency),
    };
    Some(Osc {
        tck_ps,
        runtime_tck: run_clocks(code)?,
        divisor: 2,
        min_ps: range.min,
        max_ps: range.max,
    })
}

/// A delay change of `delta_ps` in the PHY's delay steps (1/64 UI) at
/// `data_rate` Mb/s, rounded to the nearest step.
pub const fn delay_steps(delta_ps: i32, data_rate: u32) -> i32 {
    let n = delta_ps as i64 * 64 * data_rate as i64;
    let half = if n < 0 { -500_000 } else { 500_000 };
    ((n + half) / 1_000_000) as i32
}

fn mr_code(e: Error) -> u32 {
    match e {
        Error::Timeout(_) => MR_TIMEOUT,
        Error::Config(_) => MR_CONFIG,
        _ => MR_OTHER,
    }
}

fn mrr<H: Hw>(hw: &mut H, ch: usize, rank: u32, mr: u32) -> u32 {
    crate::mr::read(hw, ch, rank, mr).unwrap_or_else(mr_code)
}

fn count<H: Hw>(hw: &mut H, ch: usize, rank: u32, osc: Wck2DqOsc) -> u32 {
    let (lsb, msb, _) = osc.registers();
    let l = mrr(hw, ch, rank, lsb as u32);
    if l > 0xff {
        return l;
    }
    let m = mrr(hw, ch, rank, msb as u32);
    if m > 0xff {
        return m;
    }
    m << 8 | l
}

/// Each rank's run times as found: MR37 [7:0], MR40 [15:8], or an `MR_*`
/// code.
fn run_times<H: Hw>(hw: &mut H, ch: usize, ranks: u32) -> [u32; RANKS] {
    let (_, _, mr37) = Wck2DqOsc::Input.registers();
    let (_, _, mr40) = Wck2DqOsc::Output.registers();
    let mut found = [MR_NO_RANK; RANKS];
    for (r, f) in found.iter_mut().enumerate() {
        if ranks & (1 << r) != 0 {
            let a = mrr(hw, ch, r as u32, mr37 as u32);
            let b = mrr(hw, ch, r as u32, mr40 as u32);
            *f = if a > 0xff || b > 0xff {
                a.max(b)
            } else {
                a | b << 8
            };
        }
    }
    found
}

/// The run-time registers of the active ranks to `code` where they differ
/// (bit 16 of `found` marks a rank that was written), then one pass of the
/// controller's sequencer. Returns the status bits.
fn run_oscillators<H: Hw>(
    hw: &mut H,
    ch: usize,
    set: usize,
    ranks: u32,
    code: u8,
    found: &mut [u32; RANKS],
) -> u32 {
    let (_, _, mr37) = Wck2DqOsc::Input.registers();
    let (_, _, mr40) = Wck2DqOsc::Output.registers();
    let Some(clocks) = run_clocks(code) else {
        return status::ARG;
    };
    // Every refusal comes before the first write.
    if (0..RANKS).any(|r| ranks & (1 << r) != 0 && found[r] > 0xffff) {
        return status::RUN_TIME;
    }
    let t = hw.read32(ddrc(ch, DQSOSCRUNTIME));
    for c in [t as u8, (t >> 16) as u8] {
        if run_clocks(c).is_none_or(|n| n < clocks) {
            return status::CTL_SHORT;
        }
    }
    let ctl = freq(ch, set, DQSOSCCTL0);
    let stat = ddrc(ch, DQSOSCSTAT0);
    if hw.read32(ctl) & DQSOSC_ENABLE != 0 || hw.read32(stat) & DQSOSC_STATE != 0 {
        return status::BUSY;
    }
    for (r, f) in found.iter_mut().enumerate() {
        if ranks & (1 << r) == 0 {
            continue;
        }
        for (mr, v) in [(mr37, *f & 0xff), (mr40, *f >> 8)] {
            if v != code as u32 {
                *f |= 1 << 16;
                if crate::mr::write_rank(hw, ch, r as u32, mr as u32, code as u32).is_err() {
                    return status::RUN_TIME;
                }
            }
        }
    }
    let mut st = 0;
    hw.set32(ctl, DQSOSC_ENABLE);
    if wait(
        hw,
        "DQSOSCSTAT0.dqsosc_per_rank_stat",
        START_TIMEOUT_US,
        |hw| (hw.read32(stat) >> DQSOSC_RANK_SHIFT) & ranks == ranks,
    )
    .is_err()
    {
        st |= status::START;
    }
    hw.clr32(ctl, DQSOSC_ENABLE);
    if wait(hw, "DQSOSCSTAT0.dqsosc_state", IDLE_TIMEOUT_US, |hw| {
        hw.read32(stat) & DQSOSC_STATE == 0
    })
    .is_err()
    {
        st |= status::IDLE;
    }
    // The last oscillator started may still run: the longest run time is
    // 8192 clocks.
    hw.udelay(STOP_US);
    st
}

/// The PHY's indicators for channel `ch`: CSR access on for the reads,
/// then the select back as found.
fn phy_indicators<H: Hw>(hw: &mut H, ch: usize, flags: u32, s: &mut Sample) {
    let mux = hw.read16(MICRO_CONT_MUX_SEL);
    csr_access(hw, true);
    for (k, d) in s.dbyte.iter_mut().enumerate() {
        let n = 2 * ch + k;
        let r = |hw: &mut H, off: u32| hw.read16(csr(db(n, off)));
        let ctl = r(hw, RX_REPLICA_CTL);
        if flags & FLAG_REPLICA_SNAP != 0 {
            // The snapshot is taken on the leading edge.
            hw.write16(csr(db(n, RX_REPLICA_CTL)), ctl & !RX_REPLICA_SNAP_NOW);
            hw.write16(csr(db(n, RX_REPLICA_CTL)), ctl | RX_REPLICA_SNAP_NOW);
        }
        *d = DbyteSample {
            phase_1ui: r(hw, DX_PHASE_1UI),
            lcdl_status: r(hw, DX_LCDL_STATUS),
            replica_ratio: r(hw, RX_REPLICA_RATIO_NOW),
            replica_1ui: r(hw, RX_REPLICA_LCDL_PH1UI),
            replica_correction: r(hw, RX_REPLICA_CORRECTION),
            replica_path0: r(hw, RX_REPLICA_PATH_PHASE0),
            rxclk_trk_err: r(hw, RX_CLK_TRK_ERR),
            current_dly_tg0: r(hw, DB_CURRENT_DLY_TG0),
        };
        if flags & FLAG_REPLICA_SNAP != 0 {
            hw.write16(csr(db(n, RX_REPLICA_CTL)), ctl);
        }
    }
    let c = ch as u32;
    s.master = [
        hw.read16(csr(MAS_UPD_GOOD_CTR)),
        hw.read16(csr(PHY_UPD0_GOOD_CTR + c)),
        hw.read16(csr(CTL_UPD0_GOOD_CTR + c)),
        hw.read16(csr(PHY_UPD0_FAIL_CTR + c)),
        hw.read16(csr(PLL_LOCK_STATUS)),
        mux,
    ];
    hw.write16(MICRO_CONT_MUX_SEL, mux);
}

/// One drift sample of channel `ch` in mission mode. `run_time` is the
/// MR37/MR40 code to run both oscillators with ([`RUN_TIME_2048`]); 0
/// leaves the oscillators alone and reads the counts as they stand.
/// `flags`: [`FLAG_REPLICA_SNAP`].
pub fn sample<H: Hw>(hw: &mut H, ch: usize, run_time: u32, flags: u32) -> Sample {
    let mut s = Sample::default();
    if ch >= crate::mr::A733.channels || run_time > 0xff {
        s.status = status::ARG;
        return s;
    }
    let set = hw.read32(ddrc(ch, MSTR2)) & 0x1f;
    let ranks = (hw.read32(ddrc(ch, MSTR0)) >> 24) & 3;
    s.info = set | ranks << 8 | run_time << 16;
    let mut found = run_times(hw, ch, ranks);
    s.status = if set >= SETS {
        status::ARG
    } else if run_time == 0 {
        status::NOT_RUN
    } else {
        run_oscillators(hw, ch, set as usize, ranks, run_time as u8, &mut found)
    };
    hw.kick_watchdog();
    s.osc_stat = hw.read32(ddrc(ch, DQSOSCSTAT0));
    for (r, out) in s.rank.iter_mut().enumerate() {
        out.run_time = found[r];
        if ranks & (1 << r) == 0 {
            (out.mr4, out.wck2dqi, out.wck2dqo) = (MR_NO_RANK, MR_NO_RANK, MR_NO_RANK);
            continue;
        }
        let rank = r as u32;
        out.mr4 = mrr(hw, ch, rank, MR4);
        out.wck2dqi = count(hw, ch, rank, Wck2DqOsc::Input);
        out.wck2dqo = count(hw, ch, rank, Wck2DqOsc::Output);
        hw.kick_watchdog();
    }
    phy_indicators(hw, ch, flags, &mut s);
    s
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use crate::ctrl::{MRCTRL0, MRCTRL1};
    use crate::hw::{Rail, RegIo};
    use core::fmt;
    use std::collections::HashMap;
    use std::vec::Vec;

    const MRR_DATA: [usize; 2] = [0x0a02_002c, 0x0a02_0040];

    /// The controller's mode-register commands on a DRAM model (per
    /// channel and rank), its oscillator sequencer, and the PHY's CSRs as
    /// plain storage. Every write is recorded.
    struct Model {
        w32: Vec<(usize, u32)>,
        w16: Vec<(usize, u16)>,
        reg: HashMap<usize, u32>,
        csr: HashMap<usize, u16>,
        /// Mode registers by (channel, rank, MR).
        mr: HashMap<(usize, u32, u32), u32>,
        /// The oscillator passes run per channel.
        passes: [u32; 2],
        /// The sequencer never reports a rank as started.
        stuck: bool,
        clock: u64,
    }

    impl Model {
        fn new() -> Self {
            let mut m = Model {
                w32: Vec::new(),
                w16: Vec::new(),
                reg: HashMap::new(),
                csr: HashMap::new(),
                mr: HashMap::new(),
                passes: [0; 2],
                stuck: false,
                clock: 0,
            };
            for ch in 0..2 {
                // Two ranks, frequency set 1, the controller at 8192 clocks.
                m.reg.insert(ddrc(ch, MSTR0), 0x0308_0008);
                m.reg.insert(ddrc(ch, MSTR2), 1);
                m.reg.insert(ddrc(ch, DQSOSCRUNTIME), 0x00c0_00c0);
                m.reg.insert(freq(ch, 1, DQSOSCCTL0), 0x1f74);
                for rank in 0..2 {
                    m.mr.insert((ch, rank, 4), 0x86);
                    m.mr.insert((ch, rank, 37), 0x40);
                    m.mr.insert((ch, rank, 40), 0x40);
                }
            }
            m.csr.insert(MICRO_CONT_MUX_SEL, 1);
            m
        }
        fn channel(addr: usize) -> usize {
            (addr - 0x0a10_0000) / 0x40_0000
        }
        /// A pass: each rank's counts follow the pass number.
        fn pass(&mut self, ch: usize) {
            self.passes[ch] += 1;
            let p = self.passes[ch];
            for rank in 0..2u32 {
                let dqi = 3600 + 10 * ch as u32 + rank + p;
                let dqo = 1230 + 10 * ch as u32 + rank + p;
                self.mr.insert((ch, rank, 35), dqi & 0xff);
                self.mr.insert((ch, rank, 36), dqi >> 8);
                self.mr.insert((ch, rank, 38), dqo & 0xff);
                self.mr.insert((ch, rank, 39), dqo >> 8);
            }
        }
    }

    impl RegIo for Model {
        fn read32(&mut self, addr: usize) -> u32 {
            if let Some(ch) = MRR_DATA.iter().position(|&a| a == addr) {
                let c0 = self.reg[&ddrc(ch, MRCTRL0)];
                let ma = self.reg[&ddrc(ch, MRCTRL1)] >> 8;
                let rank = if c0 & 0x20 != 0 { 1 } else { 0 };
                return self.mr.get(&(ch, rank, ma)).copied().unwrap_or(0) << 8;
            }
            *self.reg.get(&addr).unwrap_or(&0)
        }
        fn write32(&mut self, addr: usize, val: u32) {
            self.w32.push((addr, val));
            let ch = Self::channel(addr);
            if addr == ddrc(ch, MRCTRL0) && val & (1 << 31) != 0 {
                // The command runs at once; a write stores the operand.
                if val & 1 == 0 {
                    let c1 = self.reg[&ddrc(ch, MRCTRL1)];
                    let rank = if val & 0x20 != 0 { 1 } else { 0 };
                    self.mr.insert((ch, rank, c1 >> 8), c1 & 0xff);
                }
                self.reg.insert(addr, val & !(1 << 31));
                return;
            }
            if addr == freq(ch, 1, DQSOSCCTL0) {
                let was = self.reg[&addr];
                if val & 1 != 0 && was & 1 == 0 && !self.stuck {
                    self.pass(ch);
                    self.reg.insert(ddrc(ch, DQSOSCSTAT0), 0x30);
                }
            }
            self.reg.insert(addr, val);
        }
        fn read16(&mut self, addr: usize) -> u16 {
            *self.csr.get(&addr).unwrap_or(&0)
        }
        fn write16(&mut self, addr: usize, val: u16) {
            self.w16.push((addr, val));
            self.csr.insert(addr, val);
        }
        fn udelay(&mut self, us: u32) {
            self.clock += us as u64;
        }
        fn now_us(&mut self) -> u64 {
            self.clock += 1000;
            self.clock
        }
        fn log(&mut self, _: fmt::Arguments) {}
    }
    impl Hw for Model {
        fn set_voltage(&mut self, _: Rail, _: u32) -> bool {
            false
        }
    }

    #[test]
    fn counts_to_delays() {
        // The board's reference state: CK 300 MHz, run time 2048 clocks.
        let tck = 3333;
        let i = oscillator(Wck2DqOsc::Input, tck, 0x40, false, 2400).unwrap();
        let o = oscillator(Wck2DqOsc::Output, tck, 0x40, false, 2400).unwrap();
        assert_eq!(i.delay_ps(7245), Ok(471));
        assert_eq!(o.delay_ps(2461), Ok(1387));
        // The same delays at CK 600 MHz count half as far.
        let i = oscillator(Wck2DqOsc::Input, 1667, 0x40, true, 4800).unwrap();
        assert_eq!(i.delay_ps(3623), Ok(471));
        // One count is about 0.13 ps there; 8192 clocks count four times as far.
        assert_eq!(i.delay_ps(3624), Ok(471));
        let long = oscillator(Wck2DqOsc::Input, 1667, 0xc0, true, 4800).unwrap();
        assert_eq!(long.delay_ps(4 * 3623), Ok(471));
        // Not run, or outside the range of the frequency mode.
        assert!(i.delay_ps(0).is_err());
        assert!(i.delay_ps(7245).is_err());
        assert!(oscillator(Wck2DqOsc::Input, 1667, 0, true, 4800).is_none());
        assert!(oscillator(Wck2DqOsc::Input, 1667, 0x41, true, 4800).is_none());
    }

    #[test]
    fn run_time_codes() {
        assert_eq!(run_clocks(0), None);
        assert_eq!(run_clocks(1), Some(16));
        assert_eq!(run_clocks(0x3f), Some(1008));
        assert_eq!(run_clocks(RUN_TIME_2048 as u8), Some(2048));
        assert_eq!(run_clocks(0x80), Some(4096));
        assert_eq!(run_clocks(0xc0), Some(8192));
    }

    #[test]
    fn drift_in_delay_steps() {
        // 4800 Mb/s: one UI is 208.3 ps, one step 3.26 ps.
        assert_eq!(delay_steps(36, 4800), 11);
        assert_eq!(delay_steps(-36, 4800), -11);
        assert_eq!(delay_steps(3, 4800), 1);
        assert_eq!(delay_steps(1, 4800), 0);
        assert_eq!(delay_steps(36, 2400), 6);
        assert_eq!(delay_steps(0, 4800), 0);
    }

    #[test]
    fn mr4_fields() {
        assert_eq!(mr4_refresh_code(0xc6), 6);
        assert!(mr4_updated(0x86));
        assert!(!mr4_updated(0x09));
        assert_eq!(mr4_refresh_code(0x1f), 0x1f);
    }

    /// One sample and its words; a host tool can check its unpacking
    /// against the same vector.
    fn golden() -> (Sample, [u32; WORDS]) {
        let s = Sample {
            status: 0,
            info: 1 | 3 << 8 | 0x40 << 16,
            osc_stat: 0x30,
            rank: [
                RankSample {
                    mr4: 0x86,
                    wck2dqi: 3623,
                    wck2dqo: 1231,
                    run_time: 0x4040,
                },
                RankSample {
                    mr4: 0xc7,
                    wck2dqi: 3734,
                    wck2dqo: MR_TIMEOUT,
                    run_time: 0x1_0000,
                },
            ],
            dbyte: [
                DbyteSample {
                    phase_1ui: 0x101,
                    lcdl_status: 0x400,
                    replica_ratio: 0x5a,
                    replica_1ui: 0x102,
                    replica_correction: 0x183,
                    replica_path0: 0x16b,
                    rxclk_trk_err: 0,
                    current_dly_tg0: 0xdd,
                },
                DbyteSample {
                    phase_1ui: 0x0ff,
                    lcdl_status: 0xc00,
                    replica_ratio: 0x5c,
                    replica_1ui: 0x100,
                    replica_correction: 0x101,
                    replica_path0: 0x170,
                    rxclk_trk_err: 0x8,
                    current_dly_tg0: 0xb2,
                },
            ],
            master: [0, 0, 0, 0xec60, 1, 1],
        };
        let w = [
            0,
            0x0040_0301,
            0x30,
            0x86,
            3623,
            1231,
            0x4040,
            0xc7,
            3734,
            0xdead_0001,
            0x1_0000,
            0x0400_0101,
            0x0102_005a,
            0x016b_0183,
            0x00dd_0000,
            0x0c00_00ff,
            0x0100_005c,
            0x0170_0101,
            0x00b2_0008,
            0,
            0xec60_0000,
            0x0001_0001,
        ];
        (s, w)
    }

    #[test]
    fn sample_words() {
        let (s, w) = golden();
        assert_eq!(s.words(), w);
        assert_eq!(Sample::from_words(&w), s);
    }

    #[test]
    fn a_sample_reads_both_ranks_and_restores_what_it_touched() {
        let mut m = Model::new();
        for n in 0..2 {
            m.csr.insert(csr(db(n, DX_PHASE_1UI)), 0x100 + n as u16);
            m.csr
                .insert(csr(db(n, RX_REPLICA_RATIO_NOW)), 0x50 + n as u16);
            m.csr
                .insert(csr(db(n, DB_CURRENT_DLY_TG0)), 0xd0 + n as u16);
        }
        m.csr.insert(csr(PHY_UPD0_FAIL_CTR), 0xec60);
        m.csr.insert(csr(PLL_LOCK_STATUS), 1);
        let s = sample(&mut m, 0, RUN_TIME_2048, FLAG_REPLICA_SNAP);
        assert_eq!(s.status, 0);
        assert_eq!(s.info, 1 | 3 << 8 | 0x40 << 16);
        assert_eq!(s.osc_stat, 0x30);
        assert_eq!(m.passes, [1, 0]);
        for (r, k) in s.rank.iter().zip(0..) {
            assert_eq!(
                *r,
                RankSample {
                    mr4: 0x86,
                    wck2dqi: 3601 + k,
                    wck2dqo: 1231 + k,
                    run_time: 0x4040,
                }
            );
        }
        assert_eq!(s.dbyte[0].phase_1ui, 0x100);
        assert_eq!(s.dbyte[1].phase_1ui, 0x101);
        assert_eq!(s.dbyte[1].replica_ratio, 0x51);
        assert_eq!(s.dbyte[0].current_dly_tg0, 0xd0);
        assert_eq!(s.master, [0, 0, 0, 0xec60, 1, 1]);

        // Controller writes: mode-register commands, and the sequencer
        // enable set and cleared with the other fields as they were.
        let ctl = freq(0, 1, DQSOSCCTL0);
        let other: Vec<_> = m
            .w32
            .iter()
            .filter(|(a, _)| *a != ddrc(0, MRCTRL0) && *a != ddrc(0, MRCTRL1))
            .copied()
            .collect();
        assert_eq!(other, [(ctl, 0x1f75), (ctl, 0x1f74)]);
        // Reads only: no MRW went out.
        assert!(m
            .w32
            .iter()
            .all(|(a, v)| *a != ddrc(0, MRCTRL0) || v & (1 << 31) == 0 || v & 1 == 1));
        // PHY writes: the access select and each DBYTE's snapshot strobe,
        // all back as found.
        let strobe = |n| csr(db(n, RX_REPLICA_CTL));
        assert_eq!(
            m.w16,
            [
                (MICRO_CONT_MUX_SEL, 0),
                (strobe(0), 0),
                (strobe(0), 1),
                (strobe(0), 0),
                (strobe(1), 0),
                (strobe(1), 1),
                (strobe(1), 0),
                (MICRO_CONT_MUX_SEL, 1),
            ]
        );
    }

    #[test]
    fn channel_1_uses_its_own_controller_and_dbytes() {
        let mut m = Model::new();
        m.csr.insert(csr(db(2, DX_PHASE_1UI)), 0x111);
        m.csr.insert(csr(db(3, DX_PHASE_1UI)), 0x112);
        m.csr.insert(csr(PHY_UPD0_FAIL_CTR + 1), 0x4ecd);
        let s = sample(&mut m, 1, RUN_TIME_2048, 0);
        assert_eq!(s.status, 0);
        assert_eq!(m.passes, [0, 1]);
        assert_eq!(s.rank[1].wck2dqi, 3612);
        assert_eq!(s.dbyte.map(|d| d.phase_1ui), [0x111, 0x112]);
        assert_eq!(s.master[3], 0x4ecd);
        // Without the flag the replica is not strobed.
        assert_eq!(m.w16, [(MICRO_CONT_MUX_SEL, 0), (MICRO_CONT_MUX_SEL, 1)]);
        assert!(m.w32.iter().all(|(a, _)| Model::channel(*a) == 1));
    }

    #[test]
    fn a_run_time_of_zero_in_the_dram_is_written_first() {
        let mut m = Model::new();
        m.mr.insert((0, 1, 37), 0);
        m.mr.insert((0, 1, 40), 0x80);
        let s = sample(&mut m, 0, RUN_TIME_2048, 0);
        assert_eq!(s.status, 0);
        assert_eq!(s.rank[0].run_time, 0x4040);
        assert_eq!(s.rank[1].run_time, 0x1_8000);
        assert_eq!(m.mr[&(0, 1, 37)], 0x40);
        assert_eq!(m.mr[&(0, 1, 40)], 0x40);
        assert_eq!(m.mr[&(0, 0, 37)], 0x40);
        assert_eq!(m.passes, [1, 0]);
    }

    #[test]
    fn nothing_runs_when_it_should_not() {
        // Run-time code 0: the counts are read as they stand.
        let mut m = Model::new();
        m.pass(0);
        m.w32.clear();
        let s = sample(&mut m, 0, 0, 0);
        assert_eq!(s.status, status::NOT_RUN);
        assert_eq!(s.rank[0].wck2dqi, 3601);
        assert_eq!(s.rank[0].run_time, 0x4040);
        assert_eq!(m.passes, [1, 0]);
        assert!(m.w32.iter().all(|(a, _)| *a != freq(0, 1, DQSOSCCTL0)));

        // The sequencer already enabled: left alone, and no run time is
        // written either.
        let mut m = Model::new();
        m.reg.insert(freq(0, 1, DQSOSCCTL0), 0x1f75);
        m.mr.insert((0, 0, 37), 0);
        let s = sample(&mut m, 0, RUN_TIME_2048, 0);
        assert_eq!(s.status, status::BUSY);
        assert_eq!(m.reg[&freq(0, 1, DQSOSCCTL0)], 0x1f75);
        assert_eq!(m.passes, [0, 0]);
        assert_eq!(m.mr[&(0, 0, 37)], 0);
        assert_eq!(s.rank[0].run_time, 0x4000);

        // The controller would read before the DRAM's oscillator stops.
        let mut m = Model::new();
        m.reg.insert(ddrc(0, DQSOSCRUNTIME), 0x00c0_0020);
        m.mr.insert((0, 1, 40), 0);
        let s = sample(&mut m, 0, RUN_TIME_2048, 0);
        assert_eq!(s.status, status::CTL_SHORT);
        assert_eq!(m.passes, [0, 0]);
        assert_eq!(m.mr[&(0, 1, 40)], 0);

        // No such channel, set or code.
        assert_eq!(
            sample(&mut Model::new(), 2, RUN_TIME_2048, 0).status,
            status::ARG
        );
        let mut m = Model::new();
        m.reg.insert(ddrc(0, MSTR2), 4);
        assert_eq!(sample(&mut m, 0, RUN_TIME_2048, 0).status, status::ARG);
        assert_eq!(sample(&mut Model::new(), 0, 0x41, 0).status, status::ARG);
        assert_eq!(sample(&mut Model::new(), 0, 0x140, 0).status, status::ARG);
    }

    #[test]
    fn a_sequencer_that_does_not_start_is_disabled_again() {
        let mut m = Model::new();
        m.stuck = true;
        let s = sample(&mut m, 0, RUN_TIME_2048, 0);
        assert_eq!(s.status, status::START);
        assert_eq!(m.reg[&freq(0, 1, DQSOSCCTL0)], 0x1f74);
        // The mode registers are still read.
        assert_eq!(s.rank[0].mr4, 0x86);
    }

    #[test]
    fn one_rank() {
        let mut m = Model::new();
        m.reg.insert(ddrc(0, MSTR0), 0x0108_0008);
        let s = sample(&mut m, 0, RUN_TIME_2048, 0);
        assert_eq!(s.status, 0);
        assert_eq!(s.rank[0].wck2dqi, 3601);
        assert_eq!(
            s.rank[1],
            RankSample {
                mr4: MR_NO_RANK,
                wck2dqi: MR_NO_RANK,
                wck2dqo: MR_NO_RANK,
                run_time: MR_NO_RANK,
            }
        );
    }
}
