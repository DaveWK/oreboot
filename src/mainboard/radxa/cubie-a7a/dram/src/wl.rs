// SPDX-License-Identifier: GPL-2.0-only
//! WCK2CK leveling by the host on hardware, after our CA training
//! (`blobless::cold_init_ca_train`), without the PMU.
//!
//! Per channel and rank the DRAM is put in WCK2CK leveling mode (MR18 OP[6])
//! with the reference entry ([`wcklevel::calibration_prefix`], which also
//! lets the PHY's own hardware counter search run), then **our** search:
//! the rank's TxWckDly CSR of each of the channel's two DBYTEs is written
//! directly at its documented base address (TxWckDlyTg0/1, `0x1n02a`/`0x1n02b`,
//! EIC7700X TRM Part 2) over the whole supported range, and each point is
//! judged by a real measurement: one finite leveling launch
//! ([`wcklevel::measurement_once`]) and the DBYTE's DTSM error counters
//! (DtsmErrCount_iX, `0x1n0b7 + 0x100 * lane`), which count the DRAM's
//! leveling feedback on DQ against the expected level. No lane-broadcast
//! (`0x1nf2a`) delay address is used, and no emulator value is replayed as a
//! result. The PHY's TrainingCntr (`0x1n0aa`) after its calibration is read
//! as an independent cross-check only.
//!
//! The decision ([`decide`]) is provisional until a later stage (read/write
//! data checks) confirms it: it takes the first stable transition of the
//! measured pass/fail curve, and prefers the hardware counter's delay when
//! that lies on a measured transition.

use crate::hw::Hw;
use crate::phy::coldinit::Row;
use crate::phy::wcklevel::{self, Channel, Mr18Base, Rank, SavedControls, Step, MAX_WCK_DELAY};
use crate::phy::{csr, csr_access, MICRO_RESET};
use dram_train::launch::LaunchError;

const ACSM_MEM: u32 = 0x41000;
const ACSM_START: u32 = 0x20022;
const ACSM_STOP: u32 = 0x20023;
const ACSM_LAST: u32 = 0x20024;
const ACSM_OUTER_LOOP: u32 = 0x20027;
const RUN_CTRL: u32 = crate::phy::sequence::RUN_CTRL;
const DONE: u32 = crate::phy::sequence::DONE;
const MEM_RESET_L: u32 = 0x20060;
/// Second row base, so consecutive launches end on different stop rows and
/// ACSMLastAddr can witness a launch that finished before the first poll.
/// The longest program has 152 rows; rows up to 315 are in use elsewhere.
const ROW_ALT: u16 = 152;
/// Time budget of one launch (the 255-loop calibration launches are the
/// longest). Exceeding it stops the stage.
pub const LAUNCH_TIMEOUT_US: u64 = 200_000;

/// Sweep: TxWckDly linear value (coarse * 64 + fine) 0, 16, .., 1328.
pub const WL_STEP: u16 = 16;
pub const WL_POINTS: usize = (MAX_WCK_DELAY as usize + 1) / WL_STEP as usize;

/// TxWckDly encoding: coarse [10:6] UI, fine [5:0] UI/64, i.e. the linear
/// value in UI/64 for coarse <= 20.
const fn tx_wck_dly(db: u32, rank: Rank) -> u32 {
    0x1002a + 0x1000 * db + matches!(rank, Rank::R1) as u32
}
const fn training_cntr(db: u32, lane: u32) -> u32 {
    0x100aa + 0x1000 * db + 0x100 * lane
}
const fn dtsm_err_count(db: u32, lane: u32) -> u32 {
    0x100b7 + 0x1000 * db + 0x100 * lane
}
/// Dq0LnSel..Dq7LnSel of DBYTE `db`.
const fn dq_ln_sel(db: u32, dq: u32) -> u32 {
    0x10080 + 0x1000 * db + dq
}

const fn dbytes(ch: Channel) -> [u32; 2] {
    match ch {
        Channel::A => [0, 1],
        Channel::B => [2, 3],
    }
}

/// Per-DBYTE controls the stage changes, restored exactly per DBYTE.
const DBYTE_SAVED: [u32; 5] = [0x097, 0x093, 0x005, 0x08b, 0x095];
/// MASTER/other controls in [`wcklevel::SAVED_CSRS`] order after the DBYTE
/// ones (index 5..13 there), restored at their base addresses as well.
const OTHER_SAVED: [u32; 9] = [
    0x20074, 0x20057, 0x2002a, 0x2003f, 0x20041, 0x20037, 0x2003d, 0x2003e, 0x70011,
];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WlFailure {
    /// The PMU was not stalled (MicroReset 1) or the DRAM not out of reset.
    State { micro_reset: u16, mem_reset: u16 },
    /// Run was already set before launch `n`.
    AlreadyRunning(u16),
    /// Launch `n` did not report Done within [`LAUNCH_TIMEOUT_US`].
    Timeout(u16),
    /// Done was set at the first poll of launch `n` without a LastAddr witness.
    NoBusyObserved(u16),
    /// A saved TxWckDly value outside the supported range.
    UnsupportedDelay(u16),
}

/// Runs [`wcklevel::Step`]s on the PHY with bounded, time-limited launches.
pub struct WlExec<'a, H: Hw> {
    pub hw: &'a mut H,
    pub launches: u16,
    pub writes: u16,
    pub max_launch_us: u32,
    pub max_polls: u32,
    pub failure: Option<WlFailure>,
}

impl<'a, H: Hw> WlExec<'a, H> {
    pub fn new(hw: &'a mut H) -> Self {
        Self {
            hw,
            launches: 0,
            writes: 0,
            max_launch_us: 0,
            max_polls: 0,
            failure: None,
        }
    }

    fn read(&mut self, c: u32) -> u16 {
        csr_access(self.hw, true);
        self.hw.read16(csr(c))
    }

    fn write(&mut self, c: u32, v: u16) {
        if self.failure.is_some() {
            return;
        }
        csr_access(self.hw, true);
        self.hw.write16(csr(c), v);
        self.writes += 1;
    }

    pub fn step(&mut self, s: Step<'_>) {
        match s {
            Step::Write(c, v) => self.write(c, v),
            Step::Launch { rows, outer } => self.launch(rows, outer),
        }
    }

    fn launch(&mut self, rows: &[Row], outer: u16) {
        if self.failure.is_some() || rows.is_empty() {
            return;
        }
        self.hw.kick_watchdog();
        self.launches += 1;
        let n = self.launches;
        let micro_reset = self.hw.read16(MICRO_RESET);
        let mem_reset = self.read(MEM_RESET_L);
        if micro_reset != 1 || mem_reset != 3 {
            self.failure = Some(WlFailure::State {
                micro_reset,
                mem_reset,
            });
            return;
        }
        let base = if n & 1 == 0 { ROW_ALT } else { 0 };
        let stop = base + rows.len() as u16 - 1;
        self.hw.write16(csr(ACSM_START), base);
        self.hw.write16(csr(ACSM_STOP), base);
        for (i, row) in rows.iter().enumerate() {
            for (k, &v) in row.0.iter().enumerate() {
                let a = (base as u32 + i as u32) * 4 + k as u32;
                self.hw.write16(csr(ACSM_MEM + a), v);
            }
        }
        self.hw.write16(csr(ACSM_OUTER_LOOP), outer);
        self.hw.write16(csr(ACSM_STOP), stop);
        let prior_last = self.hw.read16(csr(ACSM_LAST));
        let control = self.hw.read16(csr(RUN_CTRL));
        if control & 1 != 0 {
            self.failure = Some(WlFailure::AlreadyRunning(n));
            return;
        }
        // Done reads 1 while idle on this PHY: a busy poll (Done clear) or the
        // LastAddr witness shows that this launch ran.
        let l = dram_train::launch::bounded_launch(
            self.hw,
            LAUNCH_TIMEOUT_US,
            |hw| hw.write16(csr(RUN_CTRL), control | 1),
            |hw| hw.read16(csr(DONE)) & 1 != 0,
            |hw| {
                let last = hw.read16(csr(ACSM_LAST));
                last == stop && prior_last != stop
            },
            |hw| hw.write16(csr(RUN_CTRL), control & !1),
        );
        let result = l.result.map_err(|e| match e {
            LaunchError::Timeout => WlFailure::Timeout(n),
            LaunchError::NoBusyObserved => WlFailure::NoBusyObserved(n),
        });
        self.max_launch_us = self.max_launch_us.max(l.us);
        self.max_polls = self.max_polls.max(l.polls);
        if let Err(f) = result {
            self.failure = Some(f);
        }
    }
}

/// Our decision on one DBYTE's measured curve: [`decide`] takes the first
/// fail-to-pass transition with two stable points on each side and prefers
/// the hardware counter's delay when it lies on such a transition (shared
/// `dram_train::edge`).
pub use dram_train::edge::{decide_legacy as decide, Decision, MAX_EDGES};

/// One channel/rank block.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct WlBlock {
    pub channel: Channel,
    pub rank: Rank,
    pub mr18_base: u8,
    /// Per channel DBYTE (index 0/1 = the channel's lower/upper DBYTE).
    pub saved_delay: [u16; 2],
    pub after_prefix_delay: [u16; 2],
    pub training_cntr: [[u16; 9]; 2],
    /// TrainingCntr lane 0 converted to a delay (`TxWckDelay`), if in range.
    pub counter_delay: [Option<u16>; 2],
    pub prefix_err: [u16; 2],
    /// DtsmErrCount lane 0 per sweep point, and the lanes with a nonzero
    /// count per point (bit per lane 0..8).
    pub err: [[u16; WL_POINTS]; 2],
    pub lanes_nz: [[u16; WL_POINTS]; 2],
    pub points: u8,
    pub decision: [Decision; 2],
    /// TxWckDly read back after writing the result (or the saved value when
    /// there is no decision).
    pub final_delay: [u16; 2],
    pub launches: u16,
    pub writes: u16,
    pub max_launch_us: u32,
    pub max_polls: u32,
    pub elapsed_us: u32,
    pub failure: Option<WlFailure>,
}

impl WlBlock {
    pub const fn empty(channel: Channel, rank: Rank) -> WlBlock {
        WlBlock {
            channel,
            rank,
            mr18_base: 0,
            saved_delay: [0; 2],
            after_prefix_delay: [0; 2],
            training_cntr: [[0; 9]; 2],
            counter_delay: [None; 2],
            prefix_err: [0; 2],
            err: [[0; WL_POINTS]; 2],
            lanes_nz: [[0; WL_POINTS]; 2],
            points: 0,
            decision: [Decision::EMPTY; 2],
            final_delay: [0; 2],
            launches: 0,
            writes: 0,
            max_launch_us: 0,
            max_polls: 0,
            elapsed_us: 0,
            failure: None,
        }
    }
}

/// The reference block order: rank 0 channel A, B, then rank 1 A, B.
pub const BLOCKS: [(Channel, Rank); 4] = [
    (Channel::A, Rank::R0),
    (Channel::B, Rank::R0),
    (Channel::A, Rank::R1),
    (Channel::B, Rank::R1),
];

/// Before the first block: save every DBYTE's DQ lane map and set it to the
/// identity (leveling feedback is per DQ pad), as the reference does.
pub fn begin<H: Hw>(hw: &mut H) -> [[u16; 8]; 4] {
    let mut saved = [[0; 8]; 4];
    csr_access(hw, true);
    for (db, s) in saved.iter_mut().enumerate() {
        for (dq, v) in s.iter_mut().enumerate() {
            *v = hw.read16(csr(dq_ln_sel(db as u32, dq as u32)));
        }
    }
    for db in 0..4 {
        for dq in 0..8 {
            hw.write16(csr(dq_ln_sel(db, dq)), dq as u16);
        }
    }
    csr_access(hw, false);
    saved
}

/// After the last block: restore the lane maps. Returns the mismatch mask
/// (bit per DBYTE) of the readback.
pub fn end<H: Hw>(hw: &mut H, saved: &[[u16; 8]; 4]) -> u8 {
    csr_access(hw, true);
    let mut bad = 0;
    for (db, s) in saved.iter().enumerate() {
        for (dq, &v) in s.iter().enumerate() {
            hw.write16(csr(dq_ln_sel(db as u32, dq as u32)), v);
        }
    }
    for (db, s) in saved.iter().enumerate() {
        for (dq, &v) in s.iter().enumerate() {
            if hw.read16(csr(dq_ln_sel(db as u32, dq as u32))) != v {
                bad |= 1 << db;
            }
        }
    }
    csr_access(hw, false);
    bad
}

/// Level `out.channel`/`out.rank` (see the module documentation). Call
/// after CA training succeeded, between [`begin`] and [`end`]. Stops at the
/// first launch failure, leaving the PHY for the boot watchdog/recovery.
pub fn level_block<H: Hw>(hw: &mut H, mr18: Mr18Base, out: &mut WlBlock) {
    let (ch, rank) = (out.channel, out.rank);
    *out = WlBlock::empty(ch, rank);
    out.mr18_base = mr18.value();
    let dbs = dbytes(ch);
    let t0 = hw.now_us();
    let mut ex = WlExec::new(hw);

    // Save what the stage changes, from the hardware.
    let mut dbyte_saved = [[0u16; 5]; 4];
    for (db, s) in dbyte_saved.iter_mut().enumerate() {
        for (v, &off) in s.iter_mut().zip(DBYTE_SAVED.iter()) {
            *v = ex.read(0x10000 + 0x1000 * db as u32 + off);
        }
    }
    let mut other = [0u16; 9];
    for (v, &c) in other.iter_mut().zip(OTHER_SAVED.iter()) {
        *v = ex.read(c);
    }
    let mut reference = [0u16; 14];
    reference[..5].copy_from_slice(&dbyte_saved[0]);
    reference[5..].copy_from_slice(&other);
    for (i, &db) in dbs.iter().enumerate() {
        out.saved_delay[i] = ex.read(tx_wck_dly(db, rank));
        if out.saved_delay[i] > MAX_WCK_DELAY {
            out.failure = Some(WlFailure::UnsupportedDelay(out.saved_delay[i]));
            csr_access(ex.hw, false);
            return;
        }
    }

    // Leveling entry and the PHY's own calibration (reference sequence).
    wcklevel::calibration_prefix(ch, rank, mr18, &mut |s| ex.step(s));
    for (i, &db) in dbs.iter().enumerate() {
        for lane in 0..9 {
            out.training_cntr[i][lane] = ex.read(training_cntr(db, lane as u32));
        }
        out.counter_delay[i] = wcklevel::TxWckDelay::from_training_counter(out.training_cntr[i][0])
            .ok()
            .map(|d| d.encoded());
        out.prefix_err[i] = ex.read(dtsm_err_count(db, 0));
        out.after_prefix_delay[i] = ex.read(tx_wck_dly(db, rank));
    }

    // Our sweep with real DTSM feedback, base CSRs only.
    wcklevel::measurement_clocks_start(ch, &mut |s| ex.step(s));
    for p in 0..WL_POINTS {
        if ex.failure.is_some() {
            break;
        }
        let d = p as u16 * WL_STEP;
        for &db in &dbs {
            ex.write(tx_wck_dly(db, rank), d);
        }
        if p == 0 {
            wcklevel::first_measurement(ch, rank, &mut |s| ex.step(s));
        } else {
            wcklevel::measurement_once(rank, &mut |s| ex.step(s));
        }
        if ex.failure.is_some() {
            break;
        }
        for (i, &db) in dbs.iter().enumerate() {
            let mut nz = 0;
            for lane in 0..9 {
                let c = ex.read(dtsm_err_count(db, lane));
                if lane == 0 {
                    out.err[i][p] = c;
                }
                if c != 0 {
                    nz |= 1 << lane;
                }
            }
            out.lanes_nz[i][p] = nz;
        }
        out.points += 1;
    }
    wcklevel::measurement_clocks_stop(ch, &mut |s| ex.step(s));

    // Decide and apply (the saved value when there is no decision).
    let n = out.points as usize;
    for (i, &db) in dbs.iter().enumerate() {
        if n == WL_POINTS {
            out.decision[i] = decide(&out.err[i][..n], WL_STEP, out.counter_delay[i]);
        }
        let v = out.decision[i].delay.unwrap_or(out.saved_delay[i]);
        ex.write(tx_wck_dly(db, rank), v);
        out.final_delay[i] = ex.read(tx_wck_dly(db, rank));
    }

    // Reference cleanup and its restores, then the exact per-DBYTE and base
    // restores of every saved control.
    wcklevel::cleanup(rank, mr18, &SavedControls(reference), &mut |s| ex.step(s));
    for (db, s) in dbyte_saved.iter().enumerate() {
        for (&v, &off) in s.iter().zip(DBYTE_SAVED.iter()) {
            ex.write(0x10000 + 0x1000 * db as u32 + off, v);
        }
    }
    for (&v, &c) in other.iter().zip(OTHER_SAVED.iter()) {
        ex.write(c, v);
    }
    out.launches = ex.launches;
    out.writes = ex.writes;
    out.max_launch_us = ex.max_launch_us;
    out.max_polls = ex.max_polls;
    out.failure = out.failure.or(ex.failure);
    csr_access(hw, false);
    out.elapsed_us = hw.now_us().wrapping_sub(t0) as u32;
}
