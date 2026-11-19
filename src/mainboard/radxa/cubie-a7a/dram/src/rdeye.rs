// SPDX-License-Identifier: GPL-2.0-only
//! The read eye in two dimensions, receiver reference (VrefDAC) against
//! receive clock delay, per DBYTE lane, as a bounded search: the read
//! Vref stage for the full clock. Lab stage (feature `full-speed`): it
//! runs only with the read
//! training's `A7A_RD_VREF=eye` (measure) or `eye-apply`
//! ([`crate::rdvref::VrefPlan::parse_knob`]).
//!
//! The PHY's training firmware measures this eye at the full clock only,
//! on the whole grid (128 references x 128 delays, four passes: 65540
//! launches; REFERENCE-DERIVED count). Here the same compare (the read
//! training's own: the RDC pattern through the PHY's pattern compare, one
//! launch per point, every lane of the block judged in that launch) is
//! spent on a coarse grid and one refinement:
//!
//! 1. **Coarse.** Every lane at the same point: the plan's reference rows
//!    ([`crate::rdvref::VrefPlan::EYE`]: 0x18..0x58 step 8, nine rows)
//!    against [`COARSE_COLS`] delays, [`COARSE_STEP`] taps apart, around
//!    the byte's trained eye centre. Per lane the best cell ([`best`])
//!    and the run of passing rows through it.
//! 2. **Fine.** Every lane in its own window around its best coarse
//!    cell: [`FINE_ROWS`] references spread over that run and one coarse
//!    row beyond each end (so the window brackets the eye whatever its
//!    height; the lane's spacing follows, [`LaneEye::vstep`]) against
//!    [`FINE_COLS`] delays [`FINE_STEP`] taps apart, and one more row at
//!    the reference the lane had (the row the fixed init value gives), so
//!    each table states what the unmoved reference costs. The PHY has a
//!    reference and a delay per lane, so one launch still serves all
//!    eighteen lanes.
//!
//! **Budget.** [`BUDGET`] launches per block, a hard stop
//! ([`Stop::Budget`]): the default plan takes 9 x 9 + 10 x 41 = 491. At
//! the 0.58 ms a read-training launch takes at the full clock (measured:
//! 1522 launches in 877 ms) that is 0.3 s per block, 1.2 s for the four
//! blocks, a third on top of the read training; the whole grid would take
//! 38 s. The sizes follow the eye the read training reports at this
//! clock (46 to 56 taps where both receive clocks pass): the coarse
//! columns, 16 taps apart, hit it three times, and the fine window (80
//! taps) holds it with the coarse cell up to 8 taps off its middle.
//!
//! **The choice** ([`best`]) is the passing cell with the largest
//! smaller margin of its two axes, the margins counted in cells along the
//! cell's own row and column and weighted so that the fine window's
//! width and height count the same; ties go to the cell nearest the
//! lane's own reference and trained delay, so an eye that passes
//! everywhere in the window moves nothing. This is the margin rule of
//! the public trainers that keep the worst-direction margin (coreboot
//! MT8183/MT8192 `dramc` picks the reference whose smallest per-bit
//! window is largest, BSD-3), applied per cell; it is not a largest
//! rectangle and not a weighted area.
//!
//! **Measure** puts every reference and delay back as found and only
//! reports. **Apply** (rank-0 blocks; both ranks of a channel share the
//! references, so the rank-1 blocks then train their own delays at the
//! applied references and only measure) leaves the lane's chosen
//! reference and the centre of the passing run around the chosen delay,
//! when the lane's verdict is [`Verdict::Eye`]: edges inside the window
//! and at least [`MIN_SIDE`] columns and one row of margin on every side.
//! Another lane keeps what it had.
//!
//! Which of VrefDAC0 and VrefDAC1 the receiver uses is not known (the
//! plan's `dacs` moves one alone); the firmware-trained values differ per
//! lane and between the two.

use crate::rdvref::VrefPlan;
use crate::wl::LANES;

/// Launches one block may spend, at most.
pub const BUDGET: u32 = 512;
/// Coarse reference rows at most, delay columns and their spacing in
/// effective taps (64 taps = 1 UI): 9 columns span 2 UI.
pub const COARSE_ROWS_MAX: usize = 12;
pub const COARSE_COLS: usize = 9;
pub const COARSE_STEP: u16 = 16;
/// Fine window: rows around the best coarse cell, columns and their
/// spacing in taps: 41 columns span 80 taps.
pub const FINE_ROWS: usize = 9;
pub const FINE_COLS: usize = 41;
pub const FINE_STEP: u16 = 2;
/// Margin in fine columns a chosen cell needs on each side of the delay
/// axis (4 taps) to be applied; on the reference axis one row.
pub const MIN_SIDE: u8 = 2;
/// The fine window's last row index: the lane's own reference.
pub const FOUND_ROW: usize = FINE_ROWS;
/// The VrefDAC field and the largest effective receive clock delay.
pub const VREF_MAX: u16 = crate::rdvref::DAC_MAX;
pub const CLK_MAX: u16 = crate::rdtrain::CLK_EFF_MAX;

/// A lane's fine reference rows from the coarse rows `lo`..=`hi` that pass
/// through its best cell (None: nothing passed, the window goes around
/// `had`): the first code and the spacing, one coarse row beyond each end.
pub const fn fine_rows(plan: &VrefPlan, run: Option<(usize, usize)>, had: u16) -> (u16, u16) {
    let (lo, hi) = match run {
        Some((lo, hi)) => (
            plan.code(lo).saturating_sub(plan.step),
            plan.code(hi) + plan.step,
        ),
        None => (had.saturating_sub(plan.step), had + plan.step),
    };
    let hi = if hi > VREF_MAX { VREF_MAX } else { hi };
    let lo = if lo > hi { hi } else { lo };
    let step = (hi - lo).div_ceil(FINE_ROWS as u16 - 1);
    let step = if step == 0 { 1 } else { step };
    let span = step * (FINE_ROWS as u16 - 1);
    (
        if lo + span > VREF_MAX {
            VREF_MAX - span
        } else {
            lo
        },
        step,
    )
}

/// Launches the plan takes (coarse, fine and the found row).
pub const fn launches(plan: &VrefPlan) -> u32 {
    (plan.n as usize * COARSE_COLS + (FINE_ROWS + 1) * FINE_COLS) as u32
}

/// Whether `plan` can be this search's coarse rows within the budget.
pub const fn rows_valid(plan: &VrefPlan) -> bool {
    plan.n as usize <= COARSE_ROWS_MAX && launches(plan) <= BUDGET
}

const _: () = assert!(rows_valid(&VrefPlan::EYE) && launches(&VrefPlan::EYE) == 491);

/// One lane's point: reference code and effective receive clock delay.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Cell {
    pub vref: u16,
    pub clk: u16,
}

/// The compare the search is spent on: every lane of the block's two
/// bytes at its own [`Cell`], one launch; per byte the lanes that passed
/// (bit per lane). None: the launch failed.
pub trait Probe {
    fn launch(&mut self, at: &[[Cell; LANES]; 2]) -> Option<[u16; 2]>;
}

/// Why a block's search ended early.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Stop {
    /// The next launch would exceed [`BUDGET`].
    Budget,
    /// A launch failed.
    Launch,
}

/// What one lane's tables say.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Verdict {
    /// An eye whose chosen row ends inside the fine window on both sides,
    /// with the margins to apply it ([`MIN_SIDE`]).
    Eye,
    /// An eye with less margin than that at its best cell: reported,
    /// never applied.
    Small,
    /// Every cell of the fine window passes: nothing to choose by, the
    /// lane keeps its reference and delay.
    Flat,
    /// The chosen cell's row passes to the window's edge, or its column
    /// to the first and the last row with an end row at least half as
    /// wide as the chosen one: the margin on that axis is at least what
    /// is shown.
    Open,
    /// No passing cell in the coarse grid.
    NoCoarse,
    /// A coarse cell passed and no cell of the fine window does.
    NoFine,
}

/// One lane's eye.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LaneEye {
    /// Coarse grid: per reference row, bit c = delay column c passed.
    pub coarse: [u32; COARSE_ROWS_MAX],
    /// The best coarse cell (row, column).
    pub coarse_best: Option<(u8, u8)>,
    /// The fine window's first reference code and delay, the codes
    /// between its rows, and its rows: bit c = column c passed; the last
    /// is the lane's own reference.
    pub origin: Cell,
    pub vstep: u16,
    pub fine: [u64; FINE_ROWS + 1],
    /// What the lane had before the search.
    pub found: Cell,
    pub verdict: Verdict,
    /// The chosen cell, and its margins in fine cells: delay columns to
    /// the left and right, reference rows below and above.
    pub choice: Option<Cell>,
    pub margin: [u8; 4],
    /// Passing columns of the row at the chosen reference, of the row at
    /// the lane's own reference, and the centre delay of the chosen row's
    /// run (None when it reaches the window's edge).
    pub width_choice: u8,
    pub width_found: u8,
    pub recentre: Option<u16>,
}

impl LaneEye {
    pub const EMPTY: LaneEye = LaneEye {
        coarse: [0; COARSE_ROWS_MAX],
        coarse_best: None,
        origin: Cell { vref: 0, clk: 0 },
        vstep: 1,
        fine: [0; FINE_ROWS + 1],
        found: Cell { vref: 0, clk: 0 },
        verdict: Verdict::NoCoarse,
        choice: None,
        margin: [0; 4],
        width_choice: 0,
        width_found: 0,
        recentre: None,
    };
}

/// One block's search.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Eye {
    pub lanes: [[LaneEye; LANES]; 2],
    /// The coarse grid's first delay per byte.
    pub coarse_clk0: [u16; 2],
    pub launches: u32,
    pub stop: Option<Stop>,
    pub applied: bool,
}

impl Eye {
    pub const EMPTY: Eye = Eye {
        lanes: [[LaneEye::EMPTY; LANES]; 2],
        coarse_clk0: [0; 2],
        launches: 0,
        stop: None,
        applied: false,
    };
}

/// The passing run of row mask `m` (`n` columns) that holds column `c`:
/// columns to the left and to the right of `c` inside it, and whether it
/// touches the first and the last column.
fn run(m: u64, n: usize, c: usize) -> (u8, u8, bool, bool) {
    let (mut l, mut r) = (0, 0);
    while c > l && m >> (c - l - 1) & 1 != 0 {
        l += 1;
    }
    while c + r + 1 < n && m >> (c + r + 1) & 1 != 0 {
        r += 1;
    }
    (l as u8, r as u8, c == l, c + r + 1 == n)
}

/// The best cell of a `rows` x `cols` pass grid (`grid[r]` bit c): the
/// passing cell with the largest smaller margin of its two axes
/// (min(left, right) x (rows - 1) against min(below, above) x (cols -
/// 1), the margins in cells inside the cell's row and column), then the
/// largest sum of the four margins, then the cell nearest (`near_r`,
/// `near_c`). Returns (row, column) and the margins left, right, below,
/// above; None for a grid without a passing cell.
pub fn best(
    grid: &[u64],
    rows: usize,
    cols: usize,
    near_r: usize,
    near_c: usize,
) -> Option<((usize, usize), [u8; 4])> {
    type ScoredCell = ((usize, usize), [u8; 4], (u32, u32, u32));
    let mut found: Option<ScoredCell> = None;
    for r in 0..rows.min(grid.len()) {
        for c in 0..cols.min(64) {
            if grid[r] >> c & 1 == 0 {
                continue;
            }
            let (l, rt, _, _) = run(grid[r], cols, c);
            let (mut dn, mut up) = (0u8, 0u8);
            while r > dn as usize && grid[r - dn as usize - 1] >> c & 1 != 0 {
                dn += 1;
            }
            while r + (up as usize) + 1 < rows && grid[r + up as usize + 1] >> c & 1 != 0 {
                up += 1;
            }
            let h = l.min(rt) as u32 * (rows.max(2) - 1) as u32;
            let v = dn.min(up) as u32 * (cols.max(2) - 1) as u32;
            let sum = l as u32 + rt as u32 + dn as u32 + up as u32;
            let dist = (r.abs_diff(near_r) * cols + c.abs_diff(near_c)) as u32;
            let key = (h.min(v), sum, u32::MAX - dist);
            if found.is_none_or(|(_, _, k)| key > k) {
                found = Some(((r, c), [l, rt, dn, up], key));
            }
        }
    }
    found.map(|(at, m, _)| (at, m))
}

/// A window's first value so that `n` points `step` apart centred on
/// `centre` stay inside 0..=`max`.
fn origin(centre: u16, n: usize, step: u16, max: u16) -> u16 {
    let span = (n as u16 - 1) * step;
    centre
        .saturating_sub(span / 2)
        .min(max.saturating_sub(span))
}

/// The search (module documentation). `found`: what every lane has now
/// (its reference and trained delay); `clk0`: the byte's eye centre, the
/// middle of the coarse columns.
pub fn search<P: Probe>(
    p: &mut P,
    plan: &VrefPlan,
    found: &[[Cell; LANES]; 2],
    clk0: [u16; 2],
    out: &mut Eye,
) {
    *out = Eye::EMPTY;
    let rows = (plan.n as usize).min(COARSE_ROWS_MAX);
    let launch = |p: &mut P, out: &mut Eye, at: &[[Cell; LANES]; 2]| -> Option<[u16; 2]> {
        if out.stop.is_some() {
            return None;
        }
        if out.launches >= BUDGET {
            out.stop = Some(Stop::Budget);
            return None;
        }
        out.launches += 1;
        let m = p.launch(at);
        if m.is_none() {
            out.stop = Some(Stop::Launch);
        }
        m
    };
    for i in 0..2 {
        out.coarse_clk0[i] = origin(clk0[i], COARSE_COLS, COARSE_STEP, CLK_MAX);
        for (lane, &cell) in found[i].iter().enumerate() {
            out.lanes[i][lane].found = cell;
        }
    }
    // 1. Coarse: every lane at the same point.
    for r in 0..rows {
        for c in 0..COARSE_COLS {
            let at: [[Cell; LANES]; 2] = core::array::from_fn(|i| {
                [Cell {
                    vref: plan.code(r),
                    clk: out.coarse_clk0[i] + c as u16 * COARSE_STEP,
                }; LANES]
            });
            let Some(m) = launch(p, out, &at) else {
                return;
            };
            for (i, &mask) in m.iter().enumerate() {
                for lane in 0..LANES {
                    if mask >> lane & 1 != 0 {
                        out.lanes[i][lane].coarse[r] |= 1 << c;
                    }
                }
            }
        }
    }
    // The fine window of every lane: around its best coarse cell, or
    // around what it had when nothing passed.
    for i in 0..2 {
        for lane in 0..LANES {
            let l = &mut out.lanes[i][lane];
            let f = l.found;
            let near_r = (f.vref.saturating_sub(plan.start) + plan.step / 2) / plan.step;
            let near_c = (f.clk.saturating_sub(out.coarse_clk0[i]) + COARSE_STEP / 2) / COARSE_STEP;
            let g: [u64; COARSE_ROWS_MAX] = core::array::from_fn(|r| l.coarse[r] as u64);
            let hit = best(&g, rows, COARSE_COLS, near_r as usize, near_c as usize);
            let (centre, run) = match hit {
                Some(((r, c), m)) => {
                    l.coarse_best = Some((r as u8, c as u8));
                    (
                        out.coarse_clk0[i] + c as u16 * COARSE_STEP,
                        Some((r - m[2] as usize, r + m[3] as usize)),
                    )
                }
                None => (f.clk, None),
            };
            let (v0, vstep) = fine_rows(plan, run, f.vref);
            l.vstep = vstep;
            l.origin = Cell {
                vref: v0,
                clk: origin(centre, FINE_COLS, FINE_STEP, CLK_MAX),
            };
        }
    }
    // 2. Fine: every lane in its own window; the last row at its own
    // reference.
    for r in 0..=FINE_ROWS {
        for c in 0..FINE_COLS {
            let at: [[Cell; LANES]; 2] = core::array::from_fn(|i| {
                core::array::from_fn(|lane| {
                    let l = &out.lanes[i][lane];
                    Cell {
                        vref: if r == FOUND_ROW {
                            l.found.vref
                        } else {
                            l.origin.vref + r as u16 * l.vstep
                        },
                        clk: l.origin.clk + c as u16 * FINE_STEP,
                    }
                })
            });
            let Some(m) = launch(p, out, &at) else {
                return;
            };
            for (i, &mask) in m.iter().enumerate() {
                for lane in 0..LANES {
                    if mask >> lane & 1 != 0 {
                        out.lanes[i][lane].fine[r] |= 1 << c;
                    }
                }
            }
        }
    }
    for l in out.lanes.iter_mut().flatten() {
        judge(l);
    }
}

/// The verdict and the choice of one lane from its tables.
fn judge(l: &mut LaneEye) {
    let vstep = l.vstep;
    let full = (1u64 << FINE_COLS) - 1;
    l.width_found = l.fine[FOUND_ROW].count_ones() as u8;
    let near_r = (l.found.vref.saturating_sub(l.origin.vref) + vstep / 2) / vstep;
    let near_c = (l.found.clk.saturating_sub(l.origin.clk) + FINE_STEP / 2) / FINE_STEP;
    let grid = &l.fine[..FINE_ROWS];
    let Some(((r, c), m)) = best(grid, FINE_ROWS, FINE_COLS, near_r as usize, near_c as usize)
    else {
        l.verdict = if l.coarse_best.is_some() {
            Verdict::NoFine
        } else {
            Verdict::NoCoarse
        };
        return;
    };
    l.margin = m;
    l.width_choice = m[0] + m[1] + 1;
    if grid.iter().all(|&x| x == full) {
        l.verdict = Verdict::Flat;
        return;
    }
    let (lft, rt, at_first, at_last) = run(grid[r], FINE_COLS, c);
    // The column reaches the first and the last row, and an end row is
    // still at least half as wide as the chosen one: the eye goes on
    // beyond the window. (Narrow end rows are the eye's tips, which the
    // sparse coarse columns can miss one coarse row out.)
    let ends = grid[0].count_ones().max(grid[FINE_ROWS - 1].count_ones());
    let col_open = r == m[2] as usize
        && r + m[3] as usize + 1 == FINE_ROWS
        && 2 * ends >= l.width_choice as u32;
    l.verdict = if at_first || at_last || col_open {
        Verdict::Open
    } else if lft.min(rt) < MIN_SIDE || m[2].min(m[3]) < 1 {
        Verdict::Small
    } else {
        Verdict::Eye
    };
    l.choice = Some(Cell {
        vref: l.origin.vref + r as u16 * vstep,
        clk: l.origin.clk + c as u16 * FINE_STEP,
    });
    if !at_first && !at_last {
        // The middle of the chosen row's run.
        let first = c - lft as usize;
        let last = c + rt as usize;
        l.recentre = Some(l.origin.clk + ((first + last) as u16 * FINE_STEP) / 2);
    }
}

/// What apply leaves on one lane: the chosen reference and the centre of
/// its row's run, for [`Verdict::Eye`];
/// None: the lane keeps what it had.
pub const fn applied(l: &LaneEye) -> Option<Cell> {
    match (l.verdict, l.choice, l.recentre) {
        (Verdict::Eye, Some(c), Some(clk)) => Some(Cell { vref: c.vref, clk }),
        _ => None,
    }
}

// ---------------------------------------------------------------------------
// The stage on hardware.

use crate::hw::Hw;
use crate::phy::coldinit::Row;
use crate::phy::wcklevel::Rank;
use crate::rd::{fire, lane_csr, EV_UPDATE, VREF_DAC};
use crate::rdtrain::{clk_code, point_counts, RdTrain, TrainConfig};
use crate::rdvref::VrefResult;
use crate::wl::WlExec;

/// The block's eye (about 2 KiB; bt0 is single-threaded, and only a build
/// with the stage selected refers to it).
static mut EYE: Eye = Eye::EMPTY;

struct PhyProbe<'a, 'b, H: Hw> {
    ex: &'a mut WlExec<'b, H>,
    dbs: [u32; 2],
    prog: &'a [Row],
    regs: [u32; 2],
    dacs: u8,
}

impl<H: Hw> PhyProbe<'_, '_, H> {
    /// Every lane's reference (the selected DACs) and receive clock delay
    /// (the rank's T and C registers together), latched.
    fn set(&mut self, at: &[[Cell; LANES]; 2]) {
        for (i, &db) in self.dbs.iter().enumerate() {
            for (lane, cell) in at[i].iter().enumerate() {
                for (k, &off) in VREF_DAC.iter().enumerate() {
                    if self.dacs >> k & 1 != 0 {
                        self.ex.write(lane_csr(db, lane as u32, off), cell.vref);
                    }
                }
                for &off in &self.regs {
                    self.ex
                        .write(lane_csr(db, lane as u32, off), clk_code(cell.clk));
                }
            }
        }
        fire(self.ex, self.dbs, EV_UPDATE);
    }
}

impl<H: Hw> Probe for PhyProbe<'_, '_, H> {
    fn launch(&mut self, at: &[[Cell; LANES]; 2]) -> Option<[u16; 2]> {
        self.set(at);
        let (m, _, _) = point_counts(self.ex, self.dbs, self.prog);
        self.ex.failure.is_none().then_some(m)
    }
}

/// Whether this build's `A7A_RD_VREF` asks for the search: without it the
/// read training's reference stage is the sweep, and nothing of this
/// module is compiled in.
pub const SELECTED: bool = match VrefPlan::parse_knob(crate::knob!("A7A_RD_VREF")) {
    Ok(Some(p)) => p.eye(),
    _ => false,
};

/// The read training's reference stage: the sweep of `crate::rdvref`
/// (`rdtrain::vref_sweep`), or in a build that selects it ([`SELECTED`])
/// this search.
#[inline(always)]
pub(crate) fn stage<H: Hw>(
    ex: &mut WlExec<'_, H>,
    out: &mut RdTrain,
    vr: &mut VrefResult,
    dbs: [u32; 2],
    prog: &[Row],
    regs: [u32; 2],
    cfg: &TrainConfig,
) {
    if SELECTED {
        eye_stage(ex, out, vr, dbs, prog, regs)
    } else {
        crate::rdtrain::sweep_1d(ex, out, vr, dbs, prog, regs, cfg)
    }
}

/// The search on the block the read training has just trained (`out`:
/// its eye centres and per-lane delays). Fills `vr` with the coarse rows as the sweep's
/// profile (passing coarse columns per row) and the choices, logs the
/// tables (`RDE` lines) and, with `apply` on a rank-0 block, leaves the
/// choices ([`applied`]) with `out`'s per-lane delays following them.
#[inline(never)]
fn eye_stage<H: Hw>(
    ex: &mut WlExec<'_, H>,
    out: &mut RdTrain,
    vr: &mut VrefResult,
    dbs: [u32; 2],
    prog: &[Row],
    regs: [u32; 2],
) {
    let plan = vr.plan;
    let dacs = plan.dac_mask();
    // SAFETY: bt0 is single-threaded and nothing else refers to it.
    let eye = unsafe { &mut *core::ptr::addr_of_mut!(EYE) };
    let mut found = [[Cell { vref: 0, clk: 0 }; LANES]; 2];
    for (i, &db) in dbs.iter().enumerate() {
        for (lane, cell) in found[i].iter_mut().enumerate() {
            for (k, &off) in VREF_DAC.iter().enumerate() {
                vr.found[i][k][lane] = ex.read(lane_csr(db, lane as u32, off));
            }
            // The lane's reference: the first selected DAC's.
            let k = (dacs & 1 == 0) as usize;
            *cell = Cell {
                vref: vr.found[i][k][lane] & VREF_MAX,
                clk: out.bytes[i].lanes[lane].t_eff,
            };
        }
    }
    let clk0 = [out.bytes[0].clk0, out.bytes[1].clk0];
    let mut p = PhyProbe {
        ex: &mut *ex,
        dbs,
        prog,
        regs,
        dacs,
    };
    search(&mut p, &plan, &found, clk0, eye);
    // Back: every reference (both DACs) as found, the trained delays.
    let apply = plan.apply && out.rank == Rank::R0 && eye.stop.is_none();
    let mut last = found;
    for (i, cells) in last.iter_mut().enumerate() {
        for (lane, cell) in cells.iter_mut().enumerate() {
            cell.clk = out.bytes[i].lanes[lane].t_eff;
            let l = &eye.lanes[i][lane];
            vr.chosen[i][lane] = l.choice.map(|c| c.vref);
            if let (true, Some(c)) = (apply, applied(l)) {
                *cell = c;
                vr.recentred[i][lane] = Some(c.clk);
            }
        }
    }
    p.set(&last);
    for (i, &db) in dbs.iter().enumerate() {
        for lane in 0..LANES {
            for (k, &off) in VREF_DAC.iter().enumerate() {
                if !apply || dacs >> k & 1 == 0 || applied(&eye.lanes[i][lane]).is_none() {
                    ex.write(lane_csr(db, lane as u32, off), vr.found[i][k][lane]);
                }
            }
            let l = &mut out.bytes[i].lanes[lane];
            match applied(&eye.lanes[i][lane]) {
                Some(c) if apply => (l.t_eff, l.c_eff) = (c.clk, c.clk),
                // The C delay was trained on its own: put it back.
                _ => ex.write(lane_csr(db, lane as u32, regs[1]), clk_code(l.c_eff)),
            }
        }
    }
    fire(ex, dbs, EV_UPDATE);
    eye.applied = apply;
    vr.applied = apply;
    vr.ran = eye.stop.is_none();
    for i in 0..2 {
        for lane in 0..LANES {
            for r in 0..(plan.n as usize).min(COARSE_ROWS_MAX) {
                vr.profile[i][lane][r] = Some(eye.lanes[i][lane].coarse[r].count_ones() as u8);
            }
        }
    }
    log(
        ex.hw,
        crate::rdtrain::block_name(out.channel, out.rank),
        &plan,
        eye,
    );
}

/// The tables of one block (`RDE <block> ...`): per lane the coarse rows
/// and the fine rows as hex masks (bit c = column c passed; the last
/// fine row is the lane's own reference), then the choice and margins.
pub fn log<H: Hw>(hw: &mut H, nm: &str, plan: &VrefPlan, e: &Eye) {
    let rows = (plan.n as usize).min(COARSE_ROWS_MAX);
    hw.kick_watchdog();
    hw.log(format_args!(
        "[dram] RDE {nm}: {} launches of {BUDGET} (the whole grid: 65540), stop {:?}, applied {}; coarse rows {:02x}+{}x{} cols {COARSE_COLS} step {COARSE_STEP} from {:?}; fine {FINE_ROWS}+1 rows, {FINE_COLS} cols step {FINE_STEP}; dacs {}\n",
        e.launches, e.stop, e.applied, plan.start, plan.step, rows, e.coarse_clk0, plan.dac_mask()
    ));
    for i in 0..2 {
        for (lane, l) in e.lanes[i].iter().enumerate() {
            hw.kick_watchdog();
            hw.log(format_args!(
                "[dram] RDE {nm} b{i} l{lane} coarse {:05x?} best {:?}\n",
                &l.coarse[..rows],
                l.coarse_best
            ));
            hw.log(format_args!(
                "[dram] RDE {nm} b{i} l{lane} fine v {:02x}+{} c {} {:011x?}\n",
                l.origin.vref, l.vstep, l.origin.clk, l.fine
            ));
            hw.log(format_args!(
                "[dram] RDE {nm} b{i} l{lane} {:?}: had v {:02x} c {} width {}; choice {:x?} margins l/r/dn/up {:?} width {} recentre {:?}; leaves {:x?}\n",
                l.verdict, l.found.vref, l.found.clk, l.width_found, l.choice, l.margin, l.width_choice, l.recentre,
                if e.applied { applied(l) } else { None }
            ));
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use std::vec::Vec;

    /// A lane's eye: passes where |dv| / hv + |dc - shear dv| / hc < 1
    /// around (`v`, `c`) (a diamond, sheared by `shear` taps per code).
    #[derive(Clone, Copy)]
    struct Lane {
        v: f32,
        c: f32,
        hv: f32,
        hc: f32,
        shear: f32,
    }

    impl Lane {
        fn pass(&self, at: Cell) -> bool {
            if self.hv == f32::INFINITY {
                return true;
            }
            let dv = at.vref as f32 - self.v;
            let dc = at.clk as f32 - self.c - self.shear * dv;
            dv.abs() / self.hv + dc.abs() / self.hc < 1.0
        }
    }

    struct Model {
        lanes: [[Lane; LANES]; 2],
        launches: u32,
        fail_at: Option<u32>,
        seen: Vec<[[Cell; LANES]; 2]>,
    }

    impl Model {
        fn new(lane: Lane) -> Model {
            Model {
                lanes: [[lane; LANES]; 2],
                launches: 0,
                fail_at: None,
                seen: Vec::new(),
            }
        }
    }

    impl Probe for Model {
        fn launch(&mut self, at: &[[Cell; LANES]; 2]) -> Option<[u16; 2]> {
            self.launches += 1;
            if self.fail_at == Some(self.launches) {
                return None;
            }
            self.seen.push(*at);
            let mut m = [0u16; 2];
            for i in 0..2 {
                for (lane, &cell) in at[i].iter().enumerate() {
                    assert!(cell.vref <= VREF_MAX && cell.clk <= CLK_MAX);
                    if self.lanes[i][lane].pass(cell) {
                        m[i] |= 1 << lane;
                    }
                }
            }
            Some(m)
        }
    }

    const HAD: Cell = Cell {
        vref: 0x35,
        clk: 252,
    };
    const NOMINAL: Lane = Lane {
        v: 53.0,
        c: 252.0,
        hv: 14.0,
        hc: 22.0,
        shear: 0.0,
    };

    fn run_model(m: &mut Model) -> Eye {
        let mut e = Eye::EMPTY;
        search(m, &VrefPlan::EYE, &[[HAD; LANES]; 2], [252, 252], &mut e);
        e
    }

    #[test]
    fn every_lane_finds_its_own_centre_within_the_budget() {
        let mut m = Model::new(NOMINAL);
        // Lanes spread like the firmware-trained references (0x29..0x45)
        // and a few taps of skew.
        for i in 0..2 {
            for lane in 0..LANES {
                let k = (i * LANES + lane) as f32;
                m.lanes[i][lane] = Lane {
                    v: 41.0 + 1.6 * k,
                    c: 240.0 + 1.5 * k,
                    ..NOMINAL
                };
            }
        }
        let e = run_model(&mut m);
        assert_eq!(e.stop, None);
        assert_eq!((e.launches, m.launches), (491, 491));
        assert!(e.launches <= BUDGET);
        for i in 0..2 {
            for lane in 0..LANES {
                let (l, t) = (&e.lanes[i][lane], &m.lanes[i][lane]);
                assert_eq!(l.verdict, Verdict::Eye, "b{i} l{lane}: {l:?}");
                let c = l.choice.unwrap();
                // Within one fine row and two fine columns of the centre.
                assert!(
                    (c.vref as f32 - t.v).abs() <= l.vstep as f32,
                    "b{i} l{lane}: {c:?} step {}",
                    l.vstep
                );
                assert!((c.clk as f32 - t.c).abs() <= 4.0, "b{i} l{lane}: {c:?}");
                let a = applied(l).unwrap();
                assert!((a.clk as f32 - t.c).abs() <= 3.0, "b{i} l{lane}: {a:?}");
                // The chosen row is as wide as the row it had, to the
                // fine rows' spacing.
                assert!(l.width_choice + 3 >= l.width_found, "b{i} l{lane}");
                // The fine rows reach the eye's tips: its first and last
                // row are less than half as wide as the chosen one.
                let ends = l.fine[0]
                    .count_ones()
                    .max(l.fine[FINE_ROWS - 1].count_ones());
                assert!(2 * ends < l.width_choice as u32, "b{i} l{lane}: {l:?}");
            }
        }
        // A lane whose eye sits at 0x45 loses most of its width at 0x35.
        let far = &e.lanes[1][8];
        assert!(far.width_found * 2 < far.width_choice, "{far:?}");
    }

    #[test]
    fn a_sheared_eye_takes_the_centre_not_the_widest_row_end() {
        let mut m = Model::new(Lane {
            shear: 1.0,
            ..NOMINAL
        });
        let e = run_model(&mut m);
        let l = &e.lanes[0][0];
        assert_eq!(l.verdict, Verdict::Eye);
        let c = l.choice.unwrap();
        assert!(
            (c.vref as i32 - 53).abs() <= l.vstep as i32 && (c.clk as i32 - 252).abs() <= 6,
            "{c:?}"
        );
        // Margin on every side.
        assert!(l.margin.iter().all(|&x| x >= 1), "{:?}", l.margin);
        assert!(l.margin[0].min(l.margin[1]) >= MIN_SIDE);
    }

    #[test]
    fn a_degenerate_eye_has_no_choice_and_keeps_everything() {
        // No clean cell anywhere.
        let mut m = Model::new(Lane {
            hv: 0.0,
            hc: 0.0,
            ..NOMINAL
        });
        let e = run_model(&mut m);
        assert_eq!(e.stop, None);
        for l in e.lanes.iter().flatten() {
            assert_eq!(l.verdict, Verdict::NoCoarse);
            assert_eq!((l.choice, applied(l), l.coarse_best), (None, None, None));
            // The fine window was still measured, around what the lane had.
            assert!(l.origin.vref <= 0x35 && 0x35 <= l.origin.vref + 8 * l.vstep);
            assert_eq!(l.width_found, 0);
        }
        // An eye too small for the coarse grid on one lane only: the fine
        // window around what the lane had shows it, it is not applied,
        // and the other lanes are not disturbed.
        let mut m = Model::new(NOMINAL);
        m.lanes[0][3] = Lane {
            v: 53.0,
            c: 249.0,
            hv: 1.5,
            hc: 2.5,
            shear: 0.0,
        };
        let e = run_model(&mut m);
        let l = &e.lanes[0][3];
        assert_eq!((l.verdict, l.coarse_best), (Verdict::Small, None), "{l:?}");
        assert!(l.choice.is_some());
        assert_eq!(applied(l), None);
        assert_eq!(e.lanes[0][2].verdict, Verdict::Eye);
        // A coarse hit that the fine window does not confirm.
        let mut l = LaneEye::EMPTY;
        l.coarse_best = Some((4, 4));
        judge(&mut l);
        assert_eq!((l.verdict, applied(&l)), (Verdict::NoFine, None));
    }

    #[test]
    fn a_flat_eye_moves_nothing() {
        let mut m = Model::new(Lane {
            hv: f32::INFINITY,
            ..NOMINAL
        });
        let e = run_model(&mut m);
        for l in e.lanes.iter().flatten() {
            assert_eq!(l.verdict, Verdict::Flat);
            assert_eq!((l.choice, applied(l)), (None, None));
            assert_eq!(l.width_found as usize, FINE_COLS);
        }
        // An eye wider than the fine window on the delay axis only: the
        // reference is chosen, the delay is not moved.
        let mut m = Model::new(Lane {
            hc: 400.0,
            ..NOMINAL
        });
        let e = run_model(&mut m);
        let l = &e.lanes[1][4];
        assert_eq!(l.verdict, Verdict::Open);
        assert!(l.choice.is_some() && l.recentre.is_none() && applied(l).is_none());
    }

    #[test]
    fn the_budget_and_a_failed_launch_stop_the_search() {
        let mut m = Model::new(NOMINAL);
        m.fail_at = Some(100);
        let e = run_model(&mut m);
        assert_eq!((e.stop, e.launches), (Some(Stop::Launch), 100));
        assert!(e.lanes.iter().flatten().all(|l| l.choice.is_none()));
        // A plan that would not fit is refused before it runs; the search
        // itself stops at the budget whatever it is given.
        let big = VrefPlan {
            n: 12,
            ..VrefPlan::EYE
        };
        assert!(!rows_valid(&big));
        let mut m = Model::new(NOMINAL);
        let mut e = Eye::EMPTY;
        search(&mut m, &big, &[[HAD; LANES]; 2], [252, 252], &mut e);
        assert_eq!((e.stop, m.launches), (Some(Stop::Budget), BUDGET));
    }

    #[test]
    fn windows_stay_inside_the_registers() {
        // Trained delays and references at the ends of their ranges.
        for had in [
            Cell { vref: 0, clk: 0 },
            Cell {
                vref: VREF_MAX,
                clk: CLK_MAX,
            },
        ] {
            let mut m = Model::new(Lane {
                v: had.vref as f32,
                c: had.clk as f32,
                ..NOMINAL
            });
            let mut e = Eye::EMPTY;
            search(
                &mut m,
                &VrefPlan::EYE,
                &[[had; LANES]; 2],
                [had.clk; 2],
                &mut e,
            );
            assert_eq!(e.stop, None);
        }
        assert_eq!(origin(252, COARSE_COLS, COARSE_STEP, CLK_MAX), 188);
        assert_eq!(origin(10, FINE_COLS, FINE_STEP, CLK_MAX), 0);
        assert_eq!(origin(CLK_MAX, FINE_COLS, FINE_STEP, CLK_MAX), CLK_MAX - 80);
        // The fine rows: one coarse row beyond each end of the run.
        let p = VrefPlan::EYE;
        assert_eq!(fine_rows(&p, Some((3, 5)), 0x35), (0x28, 4));
        assert_eq!(fine_rows(&p, Some((4, 4)), 0x35), (0x30, 2));
        assert_eq!(fine_rows(&p, Some((0, 8)), 0x35), (0x10, 10));
        assert_eq!(fine_rows(&p, None, 0x35), (0x2d, 2));
        assert_eq!(
            fine_rows(&p, None, 0x7f).0 + 8 * fine_rows(&p, None, 0x7f).1,
            0x7f
        );
    }

    #[test]
    fn best_keeps_the_worst_direction_margin() {
        // A 5 x 9 grid: a wide row 0 and a 3 x 5 block under it. A cell
        // of the wide row outside the block has no margin above; the
        // block's middle has margin on both axes.
        let grid = [
            0b1_1111_1111u64,
            0b0_0111_1100,
            0b0_0111_1100,
            0b0_0111_1100,
            0,
        ];
        let ((r, c), m) = best(&grid, 5, 9, 2, 4).unwrap();
        assert_eq!((r, c), (2, 4));
        assert_eq!(m, [2, 2, 2, 1]);
        assert_eq!(best(&[0, 0], 2, 9, 0, 0), None);
        // All pass: the cell nearest the given one among the best.
        let all = [0x1ffu64; 5];
        assert_eq!(best(&all, 5, 9, 2, 4).unwrap().0, (2, 4));
        // One passing cell.
        assert_eq!(best(&[0, 0b100, 0], 3, 9, 0, 0), Some(((1, 2), [0; 4])));
    }

    #[test]
    fn plan_text() {
        let p = VrefPlan::parse_knob(Some("eye")).unwrap().unwrap();
        assert_eq!(p, VrefPlan::EYE);
        let p = VrefPlan::parse_knob(Some("eye-apply,20,4,9,1"))
            .unwrap()
            .unwrap();
        assert!(p.eye() && p.apply && (p.start, p.step, p.n, p.dac_mask()) == (0x20, 4, 9, 1));
        // Too many coarse rows for the budget.
        assert!(VrefPlan::parse_knob(Some("eye,10,4,c")).is_err());
        assert!(VrefPlan::parse_knob(Some("eyes")).is_err());
        // The sweep's forms are what they were.
        let m = VrefPlan::parse_knob(Some("measure")).unwrap().unwrap();
        assert!(!m.eye() && m == VrefPlan::MEASURE);
        assert_eq!(
            VrefPlan::parse_knob(Some("apply,20,2,19,2")),
            VrefPlan::parse(Some("apply,20,2,19,2"))
        );
        assert!(VrefPlan::parse(Some("eye")).is_err(), "the sweep's parser");
        assert_eq!(VrefPlan::parse_knob(None), Ok(None));
        assert!(VrefPlan::parse_knob(Some("eye,18,8,9,4")).is_err());
        const { assert!(!SELECTED, "the search is off without its knob") };
    }

    #[test]
    fn the_log_shows_tables_and_choices() {
        use crate::hw::{Rail, RegIo};
        struct Log(std::string::String);
        impl RegIo for Log {
            fn read32(&mut self, _: usize) -> u32 {
                0
            }
            fn write32(&mut self, _: usize, _: u32) {}
            fn read16(&mut self, _: usize) -> u16 {
                0
            }
            fn write16(&mut self, _: usize, _: u16) {}
            fn udelay(&mut self, _: u32) {}
            fn now_us(&mut self) -> u64 {
                0
            }
            fn log(&mut self, a: core::fmt::Arguments) {
                use core::fmt::Write;
                self.0.write_fmt(a).unwrap();
            }
        }
        impl Hw for Log {
            fn set_voltage(&mut self, _: Rail, _: u32) -> bool {
                true
            }
        }
        let mut e = run_model(&mut Model::new(NOMINAL));
        e.applied = true;
        let mut hw = Log(std::string::String::new());
        log(&mut hw, "A0", &VrefPlan::EYE, &e);
        let t = &hw.0;
        assert!(t.contains("RDE A0: 491 launches of 512"), "{t}");
        assert!(t.contains("RDE A0 b1 l8 coarse ["));
        assert!(t.contains("RDE A0 b0 l0 Eye: had v 35 c 252"));
        assert_eq!(t.lines().count(), 1 + 3 * 2 * LANES);
        assert!(t.lines().all(|l| l.len() < 400));
    }
}
