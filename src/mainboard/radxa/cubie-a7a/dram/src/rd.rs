// SPDX-License-Identifier: GPL-2.0-only
//! m13: read enable (RxEn) measured by the host on hardware, after WCK2CK
//! leveling (`wl`), without the PMU. A measurement milestone: our curves,
//! the PHY's own search as the cross-check, every saved delay restored. No
//! read enable value is applied yet; the decision rule is set from the
//! first hardware curves (orchestrator `a14/M12-DESIGN.md`).
//!
//! Per channel and rank (the reference block order) the reference read
//! enable set-up runs as the reference does it ([`rxen`]: channel and DTSM
//! set-up, the entry MRWs MR20/MR33/MR34/MR31/MR32 so that RDC returns the
//! all-zero DQ calibration pattern, the warm-up loop), then:
//!
//! 1. **Sweep T (the reference's knob):** TrainingCntr lane 0 of the
//!    channel's DBYTEs (`0x1n0aa`, base addresses) over 0..=1328 UI/64 in
//!    1/4 UI steps with TrainingParam `0x200` (the reference's value, where
//!    the counter sets the delay in use), one 34-row RDC launch per point,
//!    DtsmCmpCount/DtsmErrCount of all nine lanes read.
//! 2. **Sweep R (the documented register):** RxEnDly of the rank (`0x1n020`
//!    rank 0, `0x1n021` rank 1) at the base addresses over the same range,
//!    with TrainingParam 0 (m12: a nonzero TrainingParam leaves the counter
//!    in charge), same measurement.
//! 3. **The PHY's fine search** (the reference's 64-loop launch) from the
//!    first pass point of sweep T and from one UI below and above it (or
//!    from the reference's `0x1400` without one); TrainingCntr read after
//!    each: the cross-check, as for WL.
//! 4. Knob checks (`dram_train::sanity`): TrainingParam mode for each sweep,
//!    saturated curves.
//! 5. The reference exit with the **saved** RxEnDly ([`rxen::apply`]) and
//!    its mode launch (MR18/MR10/MR46); TrainingParam restored per DBYTE.
//!    The reference's second mode launch, for the next block's rank, opens
//!    the next block (after its channel set-up), as in the reference.
//!
//! Each curve is reported with its longest error-free run
//! (`dram_train::window`) and both edge decisions (`dram_train::edge`, v1.0.0
//! default policy). The decision curve is lane 0 (the reference's feedback
//! lane) plus lane 4.
use crate::hw::Hw;
use crate::phy::csr_access;
use crate::phy::rxen::{self, EntryMrs, ModeMrs};
use crate::phy::wcklevel::{self, Channel, Rank, Step, TxWckDelay};
use crate::wl::{WlExec, WlFailure, FEEDBACK_LANES, LANES};
use dram_train::edge::{self, Decision, Polarity, Verdict};
use dram_train::sanity::{self, SanityError};
use dram_train::window;

/// Sweep grid: linear UI/64 0, 16, .., 1328 (as WL).
pub const STEP: u16 = 16;
pub const POINTS: usize = 84;
/// TrainingParam of the reference read enable set-up (`rxen::block_setup`).
pub const PARAM_REFERENCE: u16 = 0x200;
/// The reference's fine-search start (no-feedback path) when sweep T finds
/// no pass point.
pub const FINE_DEFAULT: u16 = 0x1400;
/// Fine-search starts relative to sweep T's first pass point (UI/64).
pub const FINE_OFFSETS: [i16; 3] = [0, -64, 64];

const fn dtsm_err(db: u32, lane: u32) -> u32 {
    0x100b7 + 0x1000 * db + 0x100 * lane
}
const fn dtsm_cmp(db: u32, lane: u32) -> u32 {
    0x100b6 + 0x1000 * db + 0x100 * lane
}
const fn cntr(db: u32) -> u32 {
    0x100aa + 0x1000 * db
}
const fn param(db: u32) -> u32 {
    0x100a7 + 0x1000 * db
}
const fn rx_en(db: u32, rank: Rank) -> u32 {
    0x10020 + 0x1000 * db + matches!(rank, Rank::R1) as u32
}
const fn dbytes(ch: Channel) -> [u32; 2] {
    match ch {
        Channel::A => [0, 1],
        Channel::B => [2, 3],
    }
}

/// One sweep of one block: all-lane counts per point and DBYTE, the
/// decision curve, knob results and decisions per DBYTE.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Sweep {
    pub cmp: [[[u16; LANES]; 2]; POINTS],
    pub err: [[[u16; LANES]; 2]; POINTS],
    pub points: u8,
    /// TrainingParam read back per DBYTE before the sweep.
    pub param: [u16; 2],
    pub mode: [Result<(), SanityError>; 2],
    pub curve: [Result<(), SanityError>; 2],
    /// Longest error-free run on the decision curve (start point, length).
    pub window: [(u32, u32); 2],
    pub fail_to_pass: [Option<(Decision, Verdict)>; 2],
    pub pass_to_fail: [Option<(Decision, Verdict)>; 2],
}

impl Sweep {
    pub const EMPTY: Sweep = Sweep {
        cmp: [[[0; LANES]; 2]; POINTS],
        err: [[[0; LANES]; 2]; POINTS],
        points: 0,
        param: [0; 2],
        mode: [Ok(()); 2],
        curve: [Ok(()); 2],
        window: [(0, 0); 2],
        fail_to_pass: [None; 2],
        pass_to_fail: [None; 2],
    };

    /// The decision curve of DBYTE `i` (lanes 0 + 4, saturating).
    pub fn curve_of(&self, i: usize) -> [u16; POINTS] {
        let mut c = [0u16; POINTS];
        for (p, v) in c.iter_mut().enumerate() {
            *v = FEEDBACK_LANES
                .iter()
                .fold(0u16, |a, &l| a.saturating_add(self.err[p][i][l]));
        }
        c
    }
}

/// One PHY fine search: start (raw TrainingCntr) and the counter after.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Fine {
    pub start: u16,
    pub after: [u16; 2],
    /// `after` decoded to UI/64, if in range.
    pub delay: [Option<u16>; 2],
}

impl Fine {
    pub const EMPTY: Fine = Fine {
        start: 0,
        after: [0; 2],
        delay: [None; 2],
    };
}

/// One channel/rank block.
pub struct RdBlock {
    pub channel: Channel,
    pub rank: Rank,
    pub saved_rx_en: [u16; 2],
    pub final_rx_en: [u16; 2],
    pub saved_param: [u16; 4],
    pub final_param: [u16; 4],
    pub t: Sweep,
    pub r: Sweep,
    pub fine: [Fine; 3],
    /// The fine searches judged together (`edge::converge`, tolerance 2).
    pub fine_agree: [Option<edge::Convergence>; 2],
    pub launches: u16,
    pub writes: u16,
    pub max_launch_us: u32,
    pub elapsed_us: u32,
    pub failure: Option<WlFailure>,
}

impl RdBlock {
    pub const fn empty(channel: Channel, rank: Rank) -> RdBlock {
        RdBlock {
            channel,
            rank,
            saved_rx_en: [0; 2],
            final_rx_en: [0; 2],
            saved_param: [0; 4],
            final_param: [0; 4],
            t: Sweep::EMPTY,
            r: Sweep::EMPTY,
            fine: [Fine::EMPTY; 3],
            fine_agree: [None; 2],
            launches: 0,
            writes: 0,
            max_launch_us: 0,
            elapsed_us: 0,
            failure: None,
        }
    }
}

/// Launches per block: the next-rank mode launch (not on the first block),
/// entry 1, warm-up 2, two sweeps, three fine searches, the exit mode launch.
pub const BLOCK_LAUNCHES: usize = 1 + 1 + 2 + 2 * POINTS + FINE_OFFSETS.len() + 1;

fn measure<H: Hw>(ex: &mut WlExec<'_, H>, dbs: [u32; 2], rank: Rank, s: &mut Sweep, p: usize) {
    for &db in &dbs {
        wcklevel::clear_counts_dbyte(db, &mut |st| ex.step(st));
    }
    wcklevel::clear_counts(&mut |st| ex.step(st));
    ex.step(Step::Launch {
        rows: &rxen::read_program_34(rank),
        outer: 0,
    });
    if ex.failure.is_some() {
        return;
    }
    for (i, &db) in dbs.iter().enumerate() {
        for lane in 0..LANES {
            s.cmp[p][i][lane] = ex.read(dtsm_cmp(db, lane as u32));
            s.err[p][i][lane] = ex.read(dtsm_err(db, lane as u32));
        }
    }
    s.points += 1;
}

fn judge(s: &mut Sweep) {
    if s.points as usize != POINTS {
        return;
    }
    for i in 0..2 {
        let c = s.curve_of(i);
        s.curve[i] = sanity::curve(&c);
        let pass = c
            .iter()
            .enumerate()
            .fold(0u128, |m, (p, &e)| m | ((e == 0) as u128) << p);
        s.window[i] = window::longest_run_linear(pass, POINTS as u32).as_tuple();
        let policy = crate::wl::SWEEP_POLICY;
        s.fail_to_pass[i] = Some(edge::decide_explain(
            &c,
            STEP,
            None,
            Polarity::FailToPass,
            &policy,
        ));
        s.pass_to_fail[i] = Some(edge::decide_explain(
            &c,
            STEP,
            None,
            Polarity::PassToFail,
            &policy,
        ));
    }
}

/// Measure read enable for `out.channel`/`out.rank` (module documentation).
/// Call after WL, CA training and the lane-map restore. `prev` is the
/// previous block's rank (None for the first block): a different rank adds
/// the reference's [`rxen::rank_switch`], and every block but the first
/// opens with the reference's next-rank mode launch.
pub fn measure_block<H: Hw>(hw: &mut H, out: &mut RdBlock, prev: Option<Rank>) {
    let (ch, rank) = (out.channel, out.rank);
    *out = RdBlock::empty(ch, rank);
    let dbs = dbytes(ch);
    let t0 = hw.now_us();
    let mut ex = WlExec::new(hw);

    for (i, &db) in dbs.iter().enumerate() {
        out.saved_rx_en[i] = ex.read(rx_en(db, rank));
    }
    for db in 0..4u32 {
        out.saved_param[db as usize] = ex.read(param(db));
    }

    // Reference set-up, entry and warm-up.
    if prev.is_some_and(|r| r != rank) {
        rxen::rank_switch(&mut |s| ex.step(s));
    }
    rxen::block_setup(ch, &mut |s| ex.step(s));
    if prev.is_some() {
        ex.step(Step::Launch {
            rows: &rxen::mode_rows(rank, ModeMrs::NEXT_PSTATE1),
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

    // Sweep T: TrainingCntr with the reference TrainingParam.
    for (i, &db) in dbs.iter().enumerate() {
        out.t.param[i] = ex.read(param(db));
        out.t.mode[i] = sanity::mode(
            "TrainingParam (counter sweep)",
            out.t.param[i] as u32,
            0xffff,
            PARAM_REFERENCE as u32,
        );
    }
    for p in 0..POINTS {
        if ex.failure.is_some() {
            break;
        }
        let raw = TxWckDelay::new(p as u16 * STEP)
            .map(|d| d.training_counter())
            .unwrap_or(0);
        rxen::counter(ch, raw, &mut |s| ex.step(s));
        measure(&mut ex, dbs, rank, &mut out.t, p);
    }
    judge(&mut out.t);

    // Sweep R: RxEnDly at the base addresses, TrainingParam 0.
    ex.step(Step::Write(0x1f0a7, 0));
    for (i, &db) in dbs.iter().enumerate() {
        out.r.param[i] = ex.read(param(db));
        out.r.mode[i] = sanity::mode(
            "TrainingParam (RxEnDly sweep)",
            out.r.param[i] as u32,
            0xffff,
            0,
        );
    }
    for p in 0..POINTS {
        if ex.failure.is_some() {
            break;
        }
        for &db in &dbs {
            ex.write(rx_en(db, rank), p as u16 * STEP);
        }
        measure(&mut ex, dbs, rank, &mut out.r, p);
    }
    judge(&mut out.r);
    for (i, &db) in dbs.iter().enumerate() {
        ex.write(rx_en(db, rank), out.saved_rx_en[i]);
    }

    // The PHY's fine search (reference TrainingParam), from sweep T's first
    // pass point of either DBYTE, and one UI below and above.
    ex.step(Step::Write(0x1f0a7, PARAM_REFERENCE));
    let first_pass = (0..2)
        .filter_map(|i| {
            let c = out.t.curve_of(i);
            c.iter()
                .position(|&e| e == 0)
                .filter(|_| out.t.points as usize == POINTS)
        })
        .min()
        .map(|p| p as u16 * STEP);
    for (k, &off) in FINE_OFFSETS.iter().enumerate() {
        if ex.failure.is_some() {
            break;
        }
        let start = match first_pass {
            Some(d) => {
                let d = (d as i32 + off as i32).clamp(0, wcklevel::MAX_WCK_DELAY as i32) as u16;
                TxWckDelay::new(d)
                    .map(|d| d.training_counter())
                    .unwrap_or(FINE_DEFAULT)
            }
            None => FINE_DEFAULT,
        };
        let f = &mut out.fine[k];
        f.start = start;
        rxen::fine_setup(ch, start, start, &mut |s| ex.step(s));
        ex.step(Step::Launch {
            rows: &rxen::read_program_34(rank),
            outer: 64,
        });
        for (i, &db) in dbs.iter().enumerate() {
            f.after[i] = ex.read(cntr(db));
            f.delay[i] = TxWckDelay::from_training_counter(f.after[i])
                .ok()
                .map(|d| d.encoded());
        }
    }
    if ex.failure.is_none() {
        for i in 0..2 {
            let runs = [
                out.fine[0].delay[i],
                out.fine[1].delay[i],
                out.fine[2].delay[i],
            ];
            out.fine_agree[i] = edge::converge(&runs, 2);
        }
    }

    // Reference exit with the saved RxEnDly, its mode launches, then the
    // saved TrainingParam per DBYTE.
    rxen::apply(ch, rank, out.saved_rx_en, &mut |s| ex.step(s));
    ex.step(Step::Launch {
        rows: &rxen::mode_rows(rank, ModeMrs::EXIT_PSTATE1),
        outer: 0,
    });
    for db in 0..4u32 {
        ex.write(param(db), out.saved_param[db as usize]);
    }
    for (i, &db) in dbs.iter().enumerate() {
        out.final_rx_en[i] = ex.read(rx_en(db, rank));
    }
    for db in 0..4u32 {
        out.final_param[db as usize] = ex.read(param(db));
    }

    out.launches = ex.launches;
    out.writes = ex.writes;
    out.max_launch_us = ex.max_launch_us;
    out.failure = ex.failure;
    csr_access(hw, false);
    out.elapsed_us = hw.now_us().wrapping_sub(t0) as u32;
}
