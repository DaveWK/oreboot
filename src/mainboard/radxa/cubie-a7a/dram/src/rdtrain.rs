// SPDX-License-Identifier: GPL-2.0-only
//! Read training: read enable (RxEnDly) per byte, the receive clock
//! delays (RxClkT/RxClkC) per lane and the read latency (MRL), trained on
//! the data-mode read-centring compare that the `rd` searches validated on
//! hardware. No PMU, no vendor code.
//!
//! What the searches established and this training relies on:
//! - In data mode RxEnDly is double-buffered and takes effect only on
//!   PptRxEnEvnt (`0x1f0a2` bit 5); the receive clock delays only on
//!   ForceInternalUpdate (`0x200a3`); RxFifoInit (`0x2f0a0` bit 0) before
//!   every launch removes the FIFO read-pointer phase.
//! - The receive clock delay is 64 x coarse (bits 9:7) + fine (6:0); fine
//!   values 0x40 and up overlap the next coarse step. This module
//!   works in that effective delay and writes fine 0..0x3f only
//!   ([`clk_code`]).
//! - Per byte, the passing RxEnDly rows form a flat window (the gate inside
//!   the read preamble: the receive clock eye does not move) followed by
//!   rows where the eye slides down by 128 effective taps per 0x80 of
//!   RxEnDly (the gate one strobe period late). Training takes the earliest
//!   flat window.
//! - No PState flip anywhere (a diagnostic flip moved the gate between
//!   blocks).
//!
//! Per block (channel, rank), after the reference's set-up, entry, warm-up,
//! exit and exit mode launch (as `rd::measure_block` opens), with the
//! read-centring entry (MR33/MR34 0x36/0xa5) and the PHY's pattern compare:
//! 1. **Gate search:** RxEnDly over [`TrainConfig::gate_n`] rows of
//!    [`TrainConfig::gate_step`] x the receive clock delay (the rank's own
//!    T and C registers together) over [`TrainConfig::clk_n`] points of
//!    [`TrainConfig::clk_step`]. Per byte: the FIFO write boundary (the
//!    lowest row where lane 0's write pointer moved or a lane passed) and
//!    per row the span of receive clock delays where all nine lanes pass.
//! 2. **Flat window:** per byte, the longest region of consecutive passing
//!    rows at or above the boundary whose spans hold the region's first-row
//!    eye centre `clk0` (ties: the earliest). On hardware, DBYTE 2 had a short
//!    region one strobe period early (eye 128 taps higher) precedes the
//!    home region.
//! 3. **RxEnDly:** per byte at `clk0`, RxEnDly from the window's first row
//!    less [`TrainConfig::en_margin`] to its last row plus it, step
//!    [`TrainConfig::en_step`]; the longest all-lane passing run, centred.
//! 4. **Receive clock per lane:** T and C together around `clk0`
//!    +- [`TrainConfig::lane_span`] step [`TrainConfig::lane_step`], each
//!    lane's own pass; the longest run, centred, for both T and C. Then,
//!    as logged diagnostics only, T alone and C alone over
//!    +- [`TrainConfig::diag_span`] (on hardware the separate windows split
//!    at the coarse 3/4 boundary, not understood yet).
//! 5. **Data check** at the search latency: [`TrainConfig::check_n`]
//!    launches per RDC pattern of [`CHECK_PATTERNS`], then the first
//!    pattern with the inverted expected word as a negative control (every
//!    lane must fail).
//! 6. **MRR** of [`MRR_MAS`] through the synchronous path read from the RX
//!    FIFO with ObsSel 1 as the reference reads it (RxFifoVisibility 0x300,
//!    then 0x3fc per lane), decoded through Dq0..7LnSel when every lane
//!    reads 0x00 or 0xff. Reported, not judged.
//! 7. **MRL:** the diagnostic sweep from [`TrainConfig::mrl_search`] up to
//!    `mrl_max`, then down to `mrl_min`, each value followed by a probe at
//!    the search latency (on hardware, after a sweep from 1 nothing
//!    compared any more); going down stops after a value the probe does not survive.
//!    The applied value is **not** taken from the pass set (on hardware
//!    1-6 all pass; the PHY compare cannot see the controller's timing): it is the
//!    formula [`derive_mrl`] from the block's largest trained RxEnDly,
//!    cross-checked once against the vendor's trained point
//!    ([`choose_mrl`]), or the build-time override. Then the data
//!    check again at that latency.
//!
//! With [`TrainConfig::apply`] the trained delays and the read latency stay
//! (the default read training; nothing is kept from a stopped or failed
//! block); without it everything goes back (a measurement image).
//!
//! Then everything is put back as `rd::measure_block` does (a measurement
//! image: nothing trained stays applied, bt0 halts after the stage).
//!
//! Other pstates: the grid is in UI-relative units (RxEnDly 0x40 = 1 UI,
//! receive clock 64 effective taps = 1 UI); a pstate's [`TrainConfig`]
//! carries its mode launches, read latency start and UI for the table.
use crate::hw::Hw;
use crate::phy::coldinit::Row;
use crate::phy::csr_access;
use crate::phy::rxen::{self, EntryMrs, ModeMrs};
use crate::phy::wcklevel::{Channel, Rank, Step};
use crate::rd::{
    dbytes, fifo_info, fifo_vis, fire, lane_csr, lane_f, latency_group, measure_prog, param,
    pattern_off, pattern_on, read_data_saved, rx_en, DataSaved, ReadStart, ALL_ZERO, EMPTY_POINT,
    EV_FIFO, EV_RXEN, EV_UPDATE, LANE_CTL, MRL_CSRS, RX_CLK, SEL_CSRS, VIS_DFE0, VREF_DAC,
};
use crate::wl::{WlExec, WlFailure, LANES};

/// Rows of the gate search, receive clock points per row, RxEnDly points of
/// the refinement and receive clock points per lane sweep, at most.
pub const GATE_MAX: usize = 32;
pub const GATE_CLK_MAX: usize = 32;
pub const EN_MAX: usize = 64;
pub const LANE_MAX: usize = 128;
/// The largest effective receive clock delay (coarse 7, fine 0x3f).
pub const CLK_EFF_MAX: u16 = 7 * 64 + 0x3f;
/// RxEnDly field.
pub const EN_MAX_VALUE: u16 = 0x7ff;
/// No value.
pub const NONE: u16 = 0xffff;
/// All nine lanes.
pub const FULL: u16 = (1 << LANES) - 1;

/// Data check: RDC patterns (MR33, MR34); the PHY's expected word is
/// MR34 << 8 | MR33 (the reference's 0x36/0xa5 = `rd::PATTERN`).
pub const CHECK_PATTERNS: [(u8, u8); 4] = [(0x36, 0xa5), (0x55, 0xaa), (0x0f, 0xf0), (0xff, 0x00)];
/// The patterns plus the negative control.
pub const CHECKS: usize = CHECK_PATTERNS.len() + 1;
/// MRR: MR5 and MR8 (read-only identification), MR16 and MR2 (written by
/// the initialisation).
pub const MRR_MAS: [u8; 4] = [5, 8, 16, 2];
/// RxFifoVisibility for the MRR read-out: ObsSel 1, then ObsSel 1 with
/// RdEnDfe0/1 and pointer 0x3c (the reference's value).
pub const VIS_MRR: [u16; 2] = [0x300, 0x3fc];
/// DxRxStandbyEn broadcast (0 forces the receivers on for the MRR).
pub const RX_STANDBY: u32 = 0x1f00a;

/// The register value of effective receive clock delay `eff`: coarse
/// `eff / 64` in bits 9:7, fine `eff % 64` in bits 5:0.
pub const fn clk_code(eff: u16) -> u16 {
    let e = if eff > CLK_EFF_MAX { CLK_EFF_MAX } else { eff };
    ((e >> 6) & 7) << 7 | (e & 0x3f)
}

/// The effective delay of a receive clock register value.
pub const fn clk_eff(code: u16) -> u16 {
    ((code >> 7) & 7) * 64 + (code & 0x7f)
}

const _: () = assert!(clk_code(252) == 0x1bc && clk_eff(0x1bc) == 252);
const _: () = assert!(clk_eff(0x201) == 257 && clk_code(257) == 0x201);

/// Per-pstate inputs of the training.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TrainConfig {
    /// The mode launches before a later block and after the exit.
    pub next: ModeMrs,
    pub exit: ModeMrs,
    /// Gate search: RxEnDly `gate_step * i`, i < `gate_n`; effective
    /// receive clock delay `clk_step * j`, j < `clk_n`.
    pub gate_step: u16,
    pub gate_n: u16,
    pub clk_step: u16,
    pub clk_n: u16,
    /// RxEnDly refinement step and margin around the flat window.
    pub en_step: u16,
    pub en_margin: u16,
    /// Per-lane receive clock sweep step and half-span (effective taps).
    pub lane_step: u16,
    pub lane_span: u16,
    /// Half-span of the T-alone and C-alone diagnostic sweeps.
    pub diag_span: u16,
    /// Read latency during the searches; the range of the diagnostic sweep.
    pub mrl_search: u16,
    pub mrl_min: u16,
    pub mrl_max: u16,
    /// The read latency formula ([`derive_mrl`]): DfiClk in UI, the PHY's
    /// receive FIFO delay in UI and ps, the margin added.
    pub dficlk_ui: u16,
    pub fifo_ui: u16,
    pub fifo_ps: u16,
    pub mrl_margin: u16,
    /// A build-time read latency instead of the formula (measurement
    /// images; None: the formula, or [`MRL_FALLBACK`] when its vendor
    /// cross-check fails).
    pub mrl_override: Option<u16>,
    /// Leave the trained delays and the read latency applied (the default
    /// read training); false puts everything back (a measurement image).
    pub apply: bool,
    /// Launches per data-check pattern.
    pub check_n: u16,
    /// The data UI in ps (only for the printed table).
    pub ui_ps: u16,
}

impl TrainConfig {
    /// pstate 1 (WCK 1200 MHz, 2400 MT/s, UI 417 ps): the reference's mode
    /// launches, read latency 4 (as the `rd` searches).
    pub const PSTATE1: TrainConfig = TrainConfig {
        next: ModeMrs::NEXT_PSTATE1,
        exit: ModeMrs::EXIT_PSTATE1,
        gate_step: 0x40,
        gate_n: 32,
        clk_step: 16,
        clk_n: 32,
        en_step: 8,
        en_margin: 0x40,
        lane_step: 2,
        lane_span: 64,
        diag_span: 96,
        mrl_search: 4,
        mrl_min: 1,
        mrl_max: 7,
        dficlk_ui: 16,
        fifo_ui: 4,
        fifo_ps: 200,
        mrl_margin: 1,
        mrl_override: None,
        apply: false,
        check_n: 16,
        ui_ps: 417,
    };

    /// Launches of the gate search.
    pub const fn gate_launches(&self) -> usize {
        self.gate_n as usize * self.clk_n as usize
    }
}

/// The read latency when the formula fails its vendor cross-check and no
/// override is given: the vendor firmware's trained value at 1200 MHz CK
/// (HwtMRL 4, DFIMRL 4).
pub const MRL_FALLBACK: u16 = 4;
/// The vendor's trained RxEnDly the formula is cross-checked against
/// (1200 MHz CK) and the value it must give there.
pub const MRL_CHECK_RX_EN: u16 = 0x369;
pub const MRL_CHECK_WANT: u16 = 4;

/// The skip-training read latency formula (the STM32MP2 phyinit
/// progCsrSkipTrain, BSD-3, as the reference; LPDDR5 field widths from the
/// SiMa map): DFIMRL = ceil((RxEnDly + PHY_Rx_Fifo_dly) / DfiClk) + margin,
/// with RxEnDly in UI (0x40 = 1 UI; it already holds ARdPtrInitVal, the tx
/// and rx insertion delays, tDQSCK and tstaoff), PHY_Rx_Fifo_dly =
/// `fifo_ui` UI + `fifo_ps` ps, DfiClk = `dficlk_ui` UI. Computed in 1/64 UI.
pub const fn derive_mrl(rx_en: u16, cfg: &TrainConfig) -> u16 {
    let fifo_ps_64 = (cfg.fifo_ps as u32 * 64).div_ceil(cfg.ui_ps as u32);
    let num = rx_en as u32 + cfg.fifo_ui as u32 * 64 + fifo_ps_64;
    let den = cfg.dficlk_ui as u32 * 64;
    (num.div_ceil(den) + cfg.mrl_margin as u32) as u16
}

/// The read latency to apply: the override, else the formula when its
/// vendor cross-check holds, else [`MRL_FALLBACK`]; with the source.
pub const fn choose_mrl(rx_en_max: u16, cfg: &TrainConfig) -> (u16, MrlSource) {
    if let Some(m) = cfg.mrl_override {
        return (m, MrlSource::Override);
    }
    if derive_mrl(MRL_CHECK_RX_EN, cfg) == MRL_CHECK_WANT {
        (derive_mrl(rx_en_max, cfg), MrlSource::Formula)
    } else {
        (MRL_FALLBACK, MrlSource::FallbackFormulaWrong)
    }
}

/// Where the applied read latency came from.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MrlSource {
    Override,
    Formula,
    /// The formula did not give [`MRL_CHECK_WANT`] at [`MRL_CHECK_RX_EN`].
    FallbackFormulaWrong,
}

/// One lane's receive clock windows (effective taps, first and last
/// passing point; [`NONE`] when none) and the chosen delays.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LaneTrain {
    pub both: [u16; 2],
    pub t: [u16; 2],
    pub c: [u16; 2],
    pub t_eff: u16,
    pub c_eff: u16,
}

impl LaneTrain {
    pub const EMPTY: LaneTrain = LaneTrain {
        both: [NONE; 2],
        t: [NONE; 2],
        c: [NONE; 2],
        t_eff: NONE,
        c_eff: NONE,
    };
}

/// One byte's training.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ByteTrain {
    /// The FIFO write boundary (RxEnDly; [`NONE`] when never written).
    pub boundary: u16,
    /// Per gate-search row: the span (first, last effective receive clock
    /// delay) where all lanes passed, ([`NONE`], 0) for none.
    pub rows: [(u16, u16); GATE_MAX],
    /// The flat window's first and last row (RxEnDly) and its eye centre.
    pub flat: [u16; 2],
    pub clk0: u16,
    /// The refined RxEnDly window (first, last passing) and the choice.
    pub en_win: [u16; 2],
    pub rx_en: u16,
    pub lanes: [LaneTrain; LANES],
}

impl ByteTrain {
    pub const EMPTY: ByteTrain = ByteTrain {
        boundary: NONE,
        rows: [(NONE, 0); GATE_MAX],
        flat: [NONE; 2],
        clk0: NONE,
        en_win: [NONE; 2],
        rx_en: NONE,
        lanes: [LaneTrain::EMPTY; LANES],
    };
}

/// Why a block's training stopped early.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TrainStop {
    /// No flat window on this byte (0/1 of the block).
    NoGate(u8),
    /// No RxEnDly pass at the refinement on this byte.
    NoRxEn(u8),
    /// No receive clock eye on this byte and lane.
    NoLane(u8, u8),
    /// The applied read latency does not pass the PHY compare.
    NoMrl,
}

/// The data check: per pattern ([`CHECK_PATTERNS`], then the negative
/// control) and byte, the compares and errors summed over lanes and
/// launches, and the lanes with no error.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DataCheck {
    pub compares: [[u32; 2]; CHECKS],
    pub errors: [[u32; 2]; CHECKS],
    pub clean_lanes: [[u8; 2]; CHECKS],
}

impl DataCheck {
    pub const EMPTY: DataCheck = DataCheck {
        compares: [[0; 2]; CHECKS],
        errors: [[0; 2]; CHECKS],
        clean_lanes: [[0; 2]; CHECKS],
    };

    /// Every pattern compared with no error on any lane, and the negative
    /// control failed on every lane.
    pub fn passed(&self) -> bool {
        let n = CHECK_PATTERNS.len();
        (0..2).all(|i| {
            (0..n).all(|p| self.compares[p][i] > 0 && self.errors[p][i] == 0)
                && self.compares[n][i] > 0
                && self.clean_lanes[n][i] == 0
        })
    }
}

/// Read latency values the sweep can record (0..7).
pub const MRL_OBS: usize = 8;

/// One read latency value of the sweep: tried, the compares and errors
/// summed over both bytes, all lanes passed, and the probe at the search
/// latency right after it passed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MrlObs {
    pub tried: bool,
    pub compares: u32,
    pub errors: u32,
    pub pass: bool,
    pub back: bool,
}

impl MrlObs {
    pub const EMPTY: MrlObs = MrlObs {
        tried: false,
        compares: 0,
        errors: 0,
        pass: false,
        back: false,
    };
}

/// One MRR: per byte the FIFO contents byte of lanes 0..7 and the value
/// decoded through Dq0..7LnSel (None when a lane is not 0x00/0xff).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MrrRead {
    pub raw: [[u8; 8]; 2],
    pub value: [Option<u8>; 2],
}

impl MrrRead {
    pub const EMPTY: MrrRead = MrrRead {
        raw: [[0; 8]; 2],
        value: [None; 2],
    };
}

/// One channel/rank block of the training.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RdTrain {
    pub channel: Channel,
    pub rank: Rank,
    pub bytes: [ByteTrain; 2],
    /// Bit m: read latency m passed (the diagnostic sweep).
    pub mrl_pass: u16,
    /// The read latency applied for check 1 (and left applied with
    /// `apply`), its source, the formula's value for this block's largest
    /// RxEnDly and for the vendor cross-check point.
    pub mrl: u16,
    pub mrl_source: MrlSource,
    pub mrl_formula: u16,
    pub mrl_check: u16,
    /// Per read latency value: what the sweep saw.
    pub mrl_obs: [MrlObs; MRL_OBS],
    /// The data check at the search latency (before the MRL sweep) and at
    /// the chosen one.
    pub check: [DataCheck; 2],
    pub mrr: [MrrRead; MRR_MAS.len()],
    /// Rank 0: the live readback (RxClkT/C fine per lane, RxEnDly) equals
    /// the trained values. None on rank 1 (no Tg1 readback).
    pub live_ok: Option<bool>,
    /// Restored: RxEnDly, receive clock delays, references, latency and
    /// TrainingParam read back as found (with `apply`: the training set-up
    /// undone and TrainingParam back; the delays and latency stay).
    pub restored: bool,
    /// The trained delays and read latency were left applied.
    pub applied: bool,
    pub saved_rx_en: [u16; 2],
    pub launches: u16,
    pub writes: u16,
    pub max_launch_us: u32,
    pub start_us: u64,
    pub elapsed_us: u32,
    pub stop: Option<TrainStop>,
    pub failure: Option<WlFailure>,
}

impl RdTrain {
    pub const fn empty(channel: Channel, rank: Rank) -> RdTrain {
        RdTrain {
            channel,
            rank,
            bytes: [ByteTrain::EMPTY; 2],
            mrl_pass: 0,
            mrl: NONE,
            mrl_source: MrlSource::FallbackFormulaWrong,
            mrl_formula: NONE,
            mrl_check: NONE,
            mrl_obs: [MrlObs::EMPTY; MRL_OBS],
            check: [DataCheck::EMPTY; 2],
            mrr: [MrrRead::EMPTY; MRR_MAS.len()],
            live_ok: None,
            restored: false,
            applied: false,
            saved_rx_en: [0; 2],
            launches: 0,
            writes: 0,
            max_launch_us: 0,
            start_us: 0,
            elapsed_us: 0,
            stop: None,
            failure: None,
        }
    }

    /// The block trained, checked and put back.
    pub fn ok(&self) -> bool {
        self.stop.is_none()
            && self.failure.is_none()
            && self.check.iter().all(DataCheck::passed)
            && self.restored
    }
}

/// The rank's own receive clock registers: T, then C ([`RX_CLK`] order).
pub const fn own_clk(rank: Rank) -> [u32; 2] {
    match rank {
        Rank::R0 => [RX_CLK[0], RX_CLK[2]],
        Rank::R1 => [RX_CLK[1], RX_CLK[3]],
    }
}

/// The longest run of set bits among the first `n` of `mask`: (first, last).
pub fn longest_run(mask: u128, n: usize) -> Option<(usize, usize)> {
    let mut best: Option<(usize, usize)> = None;
    let mut start = None;
    for k in 0..=n {
        let on = k < n && k < 128 && mask >> k & 1 != 0;
        match (on, start) {
            (true, None) => start = Some(k),
            (false, Some(s)) => {
                let longer = match best {
                    None => true,
                    Some((a, b)) => k - 1 - s > b - a,
                };
                if longer {
                    best = Some((s, k - 1));
                }
                start = None;
            }
            _ => {}
        }
    }
    best
}

/// The flat window of a byte (module documentation, step 2): the longest
/// region of consecutive passing rows at or above `boundary` whose spans
/// hold the region's first-row eye centre (ties: the earliest), as (first
/// row, last row, eye centre) in RxEnDly and effective delay.
pub fn flat_window(rows: &[(u16, u16)], step: u16, boundary: u16) -> Option<(u16, u16, u16)> {
    let mut best: Option<(usize, usize, u16)> = None;
    let mut i = 0;
    while i < rows.len() {
        if rows[i].0 == NONE || (i as u16 * step) < boundary {
            i += 1;
            continue;
        }
        let c0 = (rows[i].0 + rows[i].1) / 2;
        let mut last = i;
        while last + 1 < rows.len()
            && rows[last + 1].0 != NONE
            && (rows[last + 1].0..=rows[last + 1].1).contains(&c0)
        {
            last += 1;
        }
        let longer = match best {
            None => true,
            Some((a, b, _)) => last - i > b - a,
        };
        if longer {
            best = Some((i, last, c0));
        }
        i = last + 1;
    }
    best.map(|(a, b, c0)| (a as u16 * step, b as u16 * step, c0))
}

/// One point: RxFifoInit, the centring launch; per byte the lanes with
/// compares and no error (bit per lane), and whether lane 0's FIFO write
/// pointer moved.
fn point<H: Hw>(ex: &mut WlExec<'_, H>, dbs: [u32; 2], prog: &[Row]) -> ([u16; 2], [bool; 2]) {
    let (m, w, _) = point_counts(ex, dbs, prog);
    (m, w)
}

/// [`point`] plus the compares and errors summed over both bytes' lanes.
#[inline(never)]
pub(crate) fn point_counts<H: Hw>(
    ex: &mut WlExec<'_, H>,
    dbs: [u32; 2],
    prog: &[Row],
) -> ([u16; 2], [bool; 2], [u32; 2]) {
    fire(ex, dbs, EV_FIFO);
    let f0 = [ex.read(fifo_info(dbs[0], 0)), ex.read(fifo_info(dbs[1], 0))];
    let mut o = EMPTY_POINT;
    measure_prog(ex, dbs, prog, rxen::CENTRING_OUTER, &mut o);
    let f1 = [ex.read(fifo_info(dbs[0], 0)), ex.read(fifo_info(dbs[1], 0))];
    let mut m = [0u16; 2];
    let mut n = [0u32; 2];
    for (i, oi) in o.iter().enumerate() {
        for lane in 0..LANES {
            if oi.cmp[lane] != 0 && oi.err[lane] == 0 {
                m[i] |= 1 << lane;
            }
            n[0] += oi.cmp[lane] as u32;
            n[1] += oi.err[lane] as u32;
        }
    }
    (
        m,
        [
            f0[0] & 0x3ff != f1[0] & 0x3ff,
            f0[1] & 0x3ff != f1[1] & 0x3ff,
        ],
        n,
    )
}

/// RxEnDly `en[i]` on byte i, latched.
#[inline(never)]
pub(crate) fn set_en<H: Hw>(ex: &mut WlExec<'_, H>, dbs: [u32; 2], rank: Rank, en: [u16; 2]) {
    for (i, &db) in dbs.iter().enumerate() {
        ex.write(rx_en(db, rank), en[i]);
    }
    fire(ex, dbs, EV_RXEN);
}

/// Effective receive clock delay `eff[i]` in registers `regs` of every lane
/// of byte i (lane F), latched.
fn set_clk<H: Hw>(ex: &mut WlExec<'_, H>, dbs: [u32; 2], regs: &[u32], eff: [u16; 2]) {
    for (i, &db) in dbs.iter().enumerate() {
        for &off in regs {
            ex.write(lane_f(db, off), clk_code(eff[i]));
        }
    }
    fire(ex, dbs, EV_UPDATE);
}

/// Per lane effective delay in register `off` of byte i (not latched).
#[inline(never)]
pub(crate) fn set_lanes<H: Hw>(
    ex: &mut WlExec<'_, H>,
    dbs: [u32; 2],
    off: u32,
    eff: &[[u16; LANES]; 2],
) {
    for (i, &db) in dbs.iter().enumerate() {
        for (lane, &e) in eff[i].iter().enumerate() {
            ex.write(lane_csr(db, lane as u32, off), clk_code(e));
        }
    }
}

/// Sweep registers `regs` (lane F) from `base[i]` in `n` steps of `step`;
/// per byte and lane the passing points.
fn sweep_lanes<H: Hw>(
    ex: &mut WlExec<'_, H>,
    dbs: [u32; 2],
    prog: &[Row],
    regs: &[u32],
    base: [u16; 2],
    step: u16,
    n: usize,
) -> [[u128; LANES]; 2] {
    let mut out = [[0u128; LANES]; 2];
    for k in 0..n.min(LANE_MAX) {
        if ex.failure.is_some() {
            break;
        }
        let eff = [base[0] + k as u16 * step, base[1] + k as u16 * step];
        set_clk(ex, dbs, regs, eff);
        let (m, _) = point(ex, dbs, prog);
        for i in 0..2 {
            for (lane, o) in out[i].iter_mut().enumerate() {
                if m[i] >> lane & 1 != 0 {
                    *o |= 1 << k;
                }
            }
        }
    }
    out
}

/// The longest passing run of a lane sweep as effective delays, and its
/// centre.
fn window(mask: u128, n: usize, base: u16, step: u16) -> Option<([u16; 2], u16)> {
    let (a, b) = longest_run(mask, n)?;
    let lo = (base + a as u16 * step).min(CLK_EFF_MAX.max(EN_MAX_VALUE));
    let hi = (base + b as u16 * step).min(CLK_EFF_MAX.max(EN_MAX_VALUE));
    Some(([lo, hi], (lo + hi) / 2))
}

/// The live readback of `lane` of `db`: RxClkT fine, RxClkC fine, receive
/// enable (Tg0); the selector put back.
fn live_lane<H: Hw>(ex: &mut WlExec<'_, H>, db: u32, lane: u16) -> [u16; 3] {
    let sel = 0x10000 + 0x1000 * db + crate::rd::SEL_DLY;
    let found = ex.read(sel);
    let mut v = [0; 3];
    for (k, x) in v.iter_mut().enumerate() {
        ex.write(sel, 0x0a + 3 * lane + k as u16);
        *x = ex.read(sel + 1);
    }
    ex.write(sel, found);
    v
}

/// The entry for RDC pattern `p`.
const fn rdc_entry(p: (u8, u8)) -> EntryMrs {
    EntryMrs {
        mr20: EntryMrs::READ_CENTRING_PSTATE1.mr20,
        mr33: p.0,
        mr34: p.1,
        mr31: 0,
        mr32: 0,
    }
}

pub(crate) const fn expected(p: (u8, u8)) -> u16 {
    (p.1 as u16) << 8 | p.0 as u16
}

/// The read-centring entry for `p`, the PHY's compare of `want`, DtsmLaneCtrl0
/// and the DFE0 visibility on both bytes.
#[inline(never)]
pub(crate) fn data_setup<H: Hw>(
    ex: &mut WlExec<'_, H>,
    dbs: [u32; 2],
    rank: Rank,
    p: (u8, u8),
    want: u16,
) {
    ex.step(Step::Launch {
        rows: &rxen::entry(rank, rdc_entry(p)),
        outer: 0,
    });
    pattern_on(dbs, want, &mut |s| ex.step(s));
    for &db in &dbs {
        ex.write(lane_f(db, 0xb5), LANE_CTL[0]);
        ex.write(fifo_vis(db), VIS_DFE0);
    }
}

/// Train the read path of `out.channel`/`out.rank` (module documentation).
/// Call after WL, CA training and the lane-map restore; `prev` as for
/// `rd::measure_block`.
#[inline(never)]
pub fn train_block<H: Hw>(
    hw: &mut H,
    out: &mut RdTrain,
    prev: Option<Rank>,
    start: Option<&ReadStart>,
    cfg: &TrainConfig,
) {
    let (ch, rank) = (out.channel, out.rank);
    *out = RdTrain::empty(ch, rank);
    let dbs = dbytes(ch);
    let t0 = hw.now_us();
    out.start_us = t0;
    let mut ex = WlExec::new(hw);

    // Found state.
    let mut saved_tg0 = [0u16; 2];
    for (i, &db) in dbs.iter().enumerate() {
        out.saved_rx_en[i] = ex.read(rx_en(db, rank));
        saved_tg0[i] = ex.read(rx_en(db, Rank::R0));
    }
    let saved_param: [u16; 4] = core::array::from_fn(|db| ex.read(param(db as u32)));
    let data_saved: DataSaved = read_data_saved(&mut ex);

    // Reference set-up, entry and warm-up.
    if prev.is_some_and(|r| r != rank) {
        rxen::rank_switch(&mut |s| ex.step(s));
    }
    rxen::block_setup(ch, &mut |s| ex.step(s));
    if prev.is_some() {
        ex.step(Step::Launch {
            rows: &rxen::mode_rows(rank, cfg.next),
            outer: 0,
        });
    }
    ex.step(Step::Launch {
        rows: &rxen::entry(rank, EntryMrs::READ_ENABLE_PSTATE1),
        outer: 0,
    });
    rxen::bars(&mut |s| ex.step(s));
    ex.step(Step::Launch {
        rows: &rxen::read_program_32(rank),
        outer: 96,
    });
    rxen::counter(ch, 0x20, &mut |s| ex.step(s));
    ex.step(Step::Launch {
        rows: &rxen::read_program_32(rank),
        outer: 0,
    });
    rxen::gates_on(&mut |s| ex.step(s));

    // Receive clock delays as found; the references from the start.
    let mut clk_before = [[[0u16; LANES]; 4]; 2];
    let mut vref_before = [[[0u16; LANES]; 2]; 2];
    for (i, &db) in dbs.iter().enumerate() {
        for (k, &off) in RX_CLK.iter().enumerate() {
            for (lane, v) in clk_before[i][k].iter_mut().enumerate() {
                *v = ex.read(lane_csr(db, lane as u32, off));
            }
        }
        for (k, &off) in VREF_DAC.iter().enumerate() {
            for (lane, v) in vref_before[i][k].iter_mut().enumerate() {
                let a = lane_csr(db, lane as u32, off);
                *v = ex.read(a);
                if let Some(st) = start {
                    ex.write(a, st.vref_dac[db as usize][k][lane]);
                }
            }
        }
    }

    // The reference exit with the found RxEnDly and its mode launch, then
    // the data set-up.
    rxen::apply(ch, rank, out.saved_rx_en, &mut |s| ex.step(s));
    ex.step(Step::Launch {
        rows: &rxen::mode_rows(rank, cfg.exit),
        outer: 0,
    });
    let mrl_before: [u16; MRL_CSRS.len()] = core::array::from_fn(|k| ex.read(MRL_CSRS[k]));
    let sel_before: [u16; SEL_CSRS.len()] = core::array::from_fn(|k| ex.read(SEL_CSRS[k]));
    let vis_before = [ex.read(fifo_vis(dbs[0])), ex.read(fifo_vis(dbs[1]))];
    data_setup(
        &mut ex,
        dbs,
        rank,
        CHECK_PATTERNS[0],
        expected(CHECK_PATTERNS[0]),
    );
    latency_group(&mut ex, None, ch, cfg.mrl_search);
    let prog = rxen::centring_program(rank);
    let regs = own_clk(rank);
    let name = block_name(ch, rank);

    train(&mut ex, out, dbs, &prog, regs, cfg, name);
    if out.stop.is_none() && ex.failure.is_none() {
        check(&mut ex, out, dbs, &prog, cfg, 0);
        mrr(&mut ex, out, dbs);
        mrl_sweep(&mut ex, out, dbs, &prog, cfg);
        if out.stop.is_none() {
            check(&mut ex, out, dbs, &prog, cfg, 1);
        }
    }
    // Apply: the trained delays and the read latency stay (a stopped or
    // failed block keeps nothing); else everything goes back.
    let keep = cfg.apply && out.stop.is_none() && ex.failure.is_none();
    out.applied = keep;
    if !keep {
        latency_group(&mut ex, None, ch, cfg.mrl_search);
    }

    // Back: receive clock delays (latched), latency, selects, visibility,
    // the all-zero entry, the pattern set-up, RxEnDly (latched), the
    // references, the exit mode launch, TrainingParam.
    let mut ok = true;
    if !keep {
        for (i, &db) in dbs.iter().enumerate() {
            for (k, &off) in RX_CLK.iter().enumerate() {
                for (lane, &v) in clk_before[i][k].iter().enumerate() {
                    ex.write(lane_csr(db, lane as u32, off), v);
                    ok &= ex.read(lane_csr(db, lane as u32, off)) == v;
                }
            }
        }
        fire(&mut ex, dbs, EV_UPDATE);
        for (k, &c) in MRL_CSRS.iter().enumerate() {
            ex.write(c, mrl_before[k]);
            ok &= ex.read(c) == mrl_before[k];
        }
        for (k, &c) in SEL_CSRS.iter().enumerate() {
            ex.write(c, sel_before[k]);
        }
    }
    for (i, &db) in dbs.iter().enumerate() {
        ex.write(fifo_vis(db), vis_before[i]);
    }
    ex.step(Step::Launch {
        rows: &rxen::entry(rank, ALL_ZERO),
        outer: 0,
    });
    pattern_off(dbs, &data_saved, &mut |s| ex.step(s));
    if !keep {
        for (i, &db) in dbs.iter().enumerate() {
            ex.write(rx_en(db, Rank::R0), saved_tg0[i]);
            ex.write(rx_en(db, rank), out.saved_rx_en[i]);
            ok &= ex.read(rx_en(db, rank)) == out.saved_rx_en[i];
        }
    }
    fire(&mut ex, dbs, EV_RXEN);
    fire(&mut ex, dbs, EV_FIFO);
    if !keep {
        for (i, &db) in dbs.iter().enumerate() {
            for (k, &off) in VREF_DAC.iter().enumerate() {
                for (lane, &v) in vref_before[i][k].iter().enumerate() {
                    let a = lane_csr(db, lane as u32, off);
                    ex.write(a, v);
                    ok &= ex.read(a) == v;
                }
            }
        }
    }
    ex.step(Step::Launch {
        rows: &rxen::mode_rows(rank, cfg.exit),
        outer: 0,
    });
    for db in 0..4u32 {
        ex.write(param(db), saved_param[db as usize]);
        ok &= ex.read(param(db)) == saved_param[db as usize];
    }
    out.restored = ok && ex.failure.is_none();
    out.launches = ex.launches;
    out.writes = ex.writes;
    out.max_launch_us = ex.max_launch_us;
    out.failure = ex.failure;
    csr_access(hw, false);
    out.elapsed_us = hw.now_us().wrapping_sub(t0) as u32;
}

/// "A0" etc.
pub const fn block_name(ch: Channel, rank: Rank) -> &'static str {
    match (ch, rank) {
        (Channel::A, Rank::R0) => "A0",
        (Channel::A, Rank::R1) => "A1",
        (Channel::B, Rank::R0) => "B0",
        (Channel::B, Rank::R1) => "B1",
    }
}

/// Steps 1-4 (module documentation).
fn train<H: Hw>(
    ex: &mut WlExec<'_, H>,
    out: &mut RdTrain,
    dbs: [u32; 2],
    prog: &[Row],
    regs: [u32; 2],
    cfg: &TrainConfig,
    name: &str,
) {
    let rank = out.rank;
    let gate_n = (cfg.gate_n as usize).min(GATE_MAX);
    let clk_n = (cfg.clk_n as usize).min(GATE_CLK_MAX);

    // 1. Gate search; rows logged as they complete ('.' none, 'w' none but
    // lane 0's FIFO written, else the passing-lane count).
    for gi in 0..gate_n {
        if ex.failure.is_some() {
            return;
        }
        let en = gi as u16 * cfg.gate_step;
        set_en(ex, dbs, rank, [en; 2]);
        let mut map = [[b'.'; GATE_CLK_MAX]; 2];
        let [map0, map1] = &mut map;
        for (ci, (c0, c1)) in map0.iter_mut().zip(map1.iter_mut()).take(clk_n).enumerate() {
            let eff = (ci as u16 * cfg.clk_step).min(CLK_EFF_MAX);
            set_clk(ex, dbs, &regs, [eff; 2]);
            let (m, w) = point(ex, dbs, prog);
            for (i, (b, cell)) in out.bytes.iter_mut().zip([c0, c1]).enumerate() {
                if (m[i] != 0 || w[i]) && b.boundary == NONE {
                    b.boundary = en;
                }
                if m[i] == FULL {
                    let r = &mut b.rows[gi];
                    r.0 = r.0.min(eff);
                    r.1 = r.1.max(eff);
                }
                let n = m[i].count_ones() as usize;
                *cell = match n {
                    0 if w[i] => b'w',
                    0 => b'.',
                    n => b"0123456789"[n],
                };
            }
        }
        let m0 = core::str::from_utf8(&map[0][..clk_n]).unwrap_or("?");
        let m1 = core::str::from_utf8(&map[1][..clk_n]).unwrap_or("?");
        ex.hw
            .log(format_args!("[dram] RDT {name} G {en:03x}: {m0} | {m1}\n"));
    }

    // 2. Flat window per byte.
    let mut clk0 = [0u16; 2];
    let mut win = [(0u16, 0u16); 2];
    for i in 0..2 {
        let b = &mut out.bytes[i];
        match flat_window(&b.rows[..gate_n], cfg.gate_step, b.boundary) {
            Some((first, last, c0)) => {
                b.flat = [first, last];
                b.clk0 = c0;
                clk0[i] = c0;
                win[i] = (first, last);
            }
            None => {
                out.stop = Some(TrainStop::NoGate(i as u8));
                return;
            }
        }
    }

    // 3. RxEnDly per byte at clk0.
    let lo = win[0].0.min(win[1].0).saturating_sub(cfg.en_margin);
    let hi = (win[0].1.max(win[1].1) + cfg.en_margin).min(EN_MAX_VALUE);
    let n = (((hi - lo) / cfg.en_step) as usize + 1).min(EN_MAX);
    set_clk(ex, dbs, &regs, clk0);
    let mut pass = [0u128; 2];
    for k in 0..n {
        if ex.failure.is_some() {
            return;
        }
        let en = lo + k as u16 * cfg.en_step;
        set_en(ex, dbs, rank, [en; 2]);
        let (m, _) = point(ex, dbs, prog);
        for i in 0..2 {
            if m[i] == FULL {
                pass[i] |= 1 << k;
            }
        }
    }
    let mut en = [0u16; 2];
    for i in 0..2 {
        match window(pass[i], n, lo, cfg.en_step) {
            Some((w, c)) => {
                out.bytes[i].en_win = w;
                out.bytes[i].rx_en = c;
                en[i] = c;
            }
            None => {
                out.stop = Some(TrainStop::NoRxEn(i as u8));
                return;
            }
        }
    }
    set_en(ex, dbs, rank, en);

    // 4. Receive clock per lane: both, then T, then C.
    let base = [
        clk0[0].saturating_sub(cfg.lane_span),
        clk0[1].saturating_sub(cfg.lane_span),
    ];
    let n = ((2 * cfg.lane_span / cfg.lane_step) as usize + 1).min(LANE_MAX);
    let both = sweep_lanes(ex, dbs, prog, &regs, base, cfg.lane_step, n);
    let mut t_eff = [[0u16; LANES]; 2];
    let mut c_eff = [[0u16; LANES]; 2];
    for i in 0..2 {
        for lane in 0..LANES {
            match window(both[i][lane], n, base[i], cfg.lane_step) {
                Some((w, c)) => {
                    out.bytes[i].lanes[lane].both = w;
                    t_eff[i][lane] = c;
                    c_eff[i][lane] = c;
                }
                None => {
                    out.stop = Some(TrainStop::NoLane(i as u8, lane as u8));
                    return;
                }
            }
        }
    }
    // Diagnostics (T alone and C alone split at the coarse 3/4
    // boundary): T alone with C per lane at the joint centre, then C alone
    // with T there, over +-`diag_span`, logged per lane ('#' pass). The
    // chosen delays stay at the joint centre.
    let dbase = [
        clk0[0].saturating_sub(cfg.diag_span),
        clk0[1].saturating_sub(cfg.diag_span),
    ];
    let dn = ((2 * cfg.diag_span / cfg.lane_step) as usize + 1).min(LANE_MAX);
    for (k, what) in [(0usize, "T"), (1, "C")] {
        set_lanes(ex, dbs, regs[1 - k], if k == 0 { &c_eff } else { &t_eff });
        let d = sweep_lanes(ex, dbs, prog, &regs[k..k + 1], dbase, cfg.lane_step, dn);
        for i in 0..2 {
            for (lane, &mask) in d[i].iter().enumerate() {
                let l = &mut out.bytes[i].lanes[lane];
                let w = window(mask, dn, dbase[i], cfg.lane_step).map(|x| x.0);
                if k == 0 {
                    l.t = w.unwrap_or([NONE; 2]);
                } else {
                    l.c = w.unwrap_or([NONE; 2]);
                }
                let mut line = [b'.'; LANE_MAX];
                for (j, x) in line.iter_mut().take(dn).enumerate() {
                    if mask >> j & 1 != 0 {
                        *x = b'#';
                    }
                }
                let txt = core::str::from_utf8(&line[..dn]).unwrap_or("?");
                ex.hw.log(format_args!(
                    "[dram] RDT {name} {what} b{i} l{lane} from {} step {}: {txt}\n",
                    dbase[i], cfg.lane_step
                ));
            }
        }
    }
    set_lanes(ex, dbs, regs[0], &t_eff);
    set_lanes(ex, dbs, regs[1], &c_eff);
    fire(ex, dbs, EV_UPDATE);
    for i in 0..2 {
        for lane in 0..LANES {
            out.bytes[i].lanes[lane].t_eff = t_eff[i][lane];
            out.bytes[i].lanes[lane].c_eff = c_eff[i][lane];
        }
    }
    if rank == Rank::R0 {
        let mut ok = true;
        for (i, &db) in dbs.iter().enumerate() {
            for lane in 0..LANES {
                let v = live_lane(ex, db, lane as u16);
                ok &= v[0] == clk_code(t_eff[i][lane]) & 0x7f;
                ok &= v[1] == clk_code(c_eff[i][lane]) & 0x7f;
                ok &= v[2] == en[i];
            }
        }
        out.live_ok = Some(ok);
    }
}

/// Step 5: read latency from `mrl_search` up to `mrl_max`, then down to
/// `mrl_min`; after each value a probe at `mrl_search` (on hardware,
/// after the sweep from 1 nothing compared). Going down stops at the first value
/// after which the probe fails.
fn mrl_sweep<H: Hw>(
    ex: &mut WlExec<'_, H>,
    out: &mut RdTrain,
    dbs: [u32; 2],
    prog: &[Row],
    cfg: &TrainConfig,
) {
    let up = cfg.mrl_search..=cfg.mrl_max.min(MRL_OBS as u16 - 1);
    let down = (cfg.mrl_min..cfg.mrl_search).rev();
    for m in up.chain(down) {
        if ex.failure.is_some() {
            return;
        }
        latency_group(ex, None, out.channel, m);
        let (p, _, n) = point_counts(ex, dbs, prog);
        latency_group(ex, None, out.channel, cfg.mrl_search);
        let (b, _) = point(ex, dbs, prog);
        let o = &mut out.mrl_obs[m as usize];
        *o = MrlObs {
            tried: true,
            compares: n[0],
            errors: n[1],
            pass: p == [FULL; 2],
            back: b == [FULL; 2],
        };
        if o.pass {
            out.mrl_pass |= 1 << m;
        }
        if m < cfg.mrl_search && !o.back {
            break;
        }
    }
    // The applied read latency is not taken from the pass set (1-6 all
    // pass the PHY compare, which cannot see the controller's timing):
    // the formula, the override or the fallback.
    let rx_en_max = out.bytes[0].rx_en.max(out.bytes[1].rx_en);
    out.mrl_formula = derive_mrl(rx_en_max, cfg);
    out.mrl_check = derive_mrl(MRL_CHECK_RX_EN, cfg);
    let (m, src) = choose_mrl(rx_en_max, cfg);
    out.mrl = m;
    out.mrl_source = src;
    if out.mrl_pass >> m & 1 == 0 {
        out.stop = Some(TrainStop::NoMrl);
    }
    latency_group(ex, None, out.channel, m);
}

/// Step 6: the data check at the trained point, into `out.check[k]`.
fn check<H: Hw>(
    ex: &mut WlExec<'_, H>,
    out: &mut RdTrain,
    dbs: [u32; 2],
    prog: &[Row],
    cfg: &TrainConfig,
    k: usize,
) {
    data_check(ex, dbs, out.rank, prog, cfg.check_n, &mut out.check[k]);
}

/// The data check at the current delays: `n` launches per pattern of
/// [`CHECK_PATTERNS`], then the negative control; the first pattern's
/// set-up is left in place.
#[inline(never)]
pub(crate) fn data_check<H: Hw>(
    ex: &mut WlExec<'_, H>,
    dbs: [u32; 2],
    rank: Rank,
    prog: &[Row],
    n: u16,
    out: &mut DataCheck,
) {
    let negative = (CHECK_PATTERNS[0], !expected(CHECK_PATTERNS[0]));
    let runs = CHECK_PATTERNS
        .iter()
        .map(|&p| (p, expected(p)))
        .chain(core::iter::once(negative));
    for (p, (pat, want)) in runs.enumerate() {
        data_setup(ex, dbs, rank, pat, want);
        let mut dirty = [0u16; 2];
        for _ in 0..n {
            if ex.failure.is_some() {
                return;
            }
            fire(ex, dbs, EV_FIFO);
            let mut o = EMPTY_POINT;
            measure_prog(ex, dbs, prog, rxen::CENTRING_OUTER, &mut o);
            for (i, oi) in o.iter().enumerate() {
                for lane in 0..LANES {
                    out.compares[p][i] += oi.cmp[lane] as u32;
                    out.errors[p][i] += oi.err[lane] as u32;
                    if oi.err[lane] != 0 || oi.cmp[lane] == 0 {
                        dirty[i] |= 1 << lane;
                    }
                }
            }
        }
        for (c, d) in out.clean_lanes[p].iter_mut().zip(dirty) {
            *c = (LANES as u32 - d.count_ones()) as u8;
        }
    }
    data_setup(
        ex,
        dbs,
        rank,
        CHECK_PATTERNS[0],
        expected(CHECK_PATTERNS[0]),
    );
}

/// Step 7: MRR known answers read from the FIFO with ObsSel 1.
fn mrr<H: Hw>(ex: &mut WlExec<'_, H>, out: &mut RdTrain, dbs: [u32; 2]) {
    let rank = out.rank;
    mrr_read(ex, dbs, rank, &mut out.mrr);
}

/// [`MRR_MAS`] through the synchronous path, read from the FIFO with
/// ObsSel 1 (module documentation, step 6).
#[inline(never)]
pub(crate) fn mrr_read<H: Hw>(
    ex: &mut WlExec<'_, H>,
    dbs: [u32; 2],
    rank: Rank,
    out: &mut [MrrRead; MRR_MAS.len()],
) {
    let standby = ex.read(0x1000a + 0x1000 * dbs[0]);
    let mut lane_sel = [[0u16; 8]; 2];
    for (i, &db) in dbs.iter().enumerate() {
        for (dq, v) in lane_sel[i].iter_mut().enumerate() {
            *v = ex.read(0x10080 + 0x1000 * db + dq as u32) & 0xf;
        }
    }
    for (k, &ma) in MRR_MAS.iter().enumerate() {
        if ex.failure.is_some() {
            return;
        }
        ex.write(RX_STANDBY, 0);
        fire(ex, dbs, EV_FIFO);
        ex.step(Step::Launch {
            rows: &rxen::mrr_program(rank, ma),
            outer: 0,
        });
        ex.write(RX_STANDBY, standby);
        let r = &mut out[k];
        for (i, &db) in dbs.iter().enumerate() {
            ex.write(fifo_vis(db), VIS_MRR[0]);
            for lane in 0..8u32 {
                ex.read(fifo_info(db, lane));
            }
            for lane in 0..8u32 {
                ex.write(fifo_vis(db), VIS_MRR[1]);
                r.raw[i][lane as usize] = ex.read(0x10073 + 0x1000 * db + 0x100 * lane) as u8;
            }
            ex.write(fifo_vis(db), VIS_DFE0);
            let mut v = 0u8;
            let mut ok = true;
            for (dq, &lane) in lane_sel[i].iter().enumerate() {
                match r.raw[i].get(lane as usize) {
                    Some(0xff) => v |= 1 << dq,
                    Some(0) => {}
                    _ => ok = false,
                }
            }
            r.value[i] = ok.then_some(v);
        }
    }
}

/// The live read-path delays of lane `lane` of every DBYTE (Tg0: RxClkT
/// fine, RxClkC fine, receive enable), for the stage after the training.
pub fn live_summary<H: Hw>(hw: &mut H, lane: u16) -> [[u16; 3]; 4] {
    let mut ex = WlExec::new(hw);
    let v = core::array::from_fn(|db| live_lane(&mut ex, db as u32, lane));
    csr_access(hw, false);
    v
}
