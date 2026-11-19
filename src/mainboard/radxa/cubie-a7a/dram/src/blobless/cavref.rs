// SPDX-License-Identifier: GPL-2.0-only
//! Command-bus reference (VrefCA, MR12) as a step of command bus
//! training. Lab stage (feature `full-speed`): it runs only with build
//! knob `A7A_CA_VREF`
//! (`measure` or `apply`, optionally `,<half span>,<step>` in hex codes).
//!
//! What command bus training does here without it. The DRAM is trained in
//! CBT mode 2 (MR13 OP[6]) on the set-point that will operate (MR16 CBT
//! field 2 with FSP-WR 1): after the entry the host raises DQ7, puts a
//! reference code on DQ[6:0] and strobes it in with DMI, then releases
//! DQ[6:0] for the DRAM's echo of the CA pins. The entry's two data words
//! (`coldinit::CbtChannel::tx_data`) are that code: through the board's
//! lane maps both channels' words read DQ7 high and DQ[6:0] = 0x44, the
//! parameter set's MR12 ([`decode`]; checked by a test and again on the
//! board before anything is written). So every block is trained at one
//! reference, the parameter set's; the reference's own command bus
//! training does the same at the full clock (its launches and writes of
//! this part are those of the lower clock: no reference sweep there).
//! Matching the firmware therefore needs no VrefCA stage; this one
//! measures the margin the fixed code leaves and can move it.
//!
//! **What is swept:** the code on DQ[6:0], per block (channel, rank, CBT
//! phase), [`Plan::n`] codes around the parameter set's, inside the
//! absolute CA reference limits (75 to 350 mV of VDDQ). The PHY has no
//! CA reference of its own to train: it only drives CA, and the echo
//! comes back as static levels through the asynchronous DQ read.
//!
//! **How a code is judged:** with the feedback CBT already uses. All
//! seven CA lanes are swept together over one CA period ([`POINTS`]
//! delays), four patterns per delay, and the code's figure is the longest
//! run of delays at which every echo bit is right, in delay points. Per
//! channel and rank the two phases are combined point by point (the
//! smaller width), and the choice is the middle of the plateau of codes
//! within [`TOLERANCE`] of the widest ([`choose`]). A plateau that
//! reaches the sweep's end, or a widest window under [`MIN_WIDTH`], gives
//! no choice.
//!
//! **Budget:** [`BUDGET`] launches per block, enforced when the plan is
//! parsed and again while it runs: the default 13 codes x 64 delays x 4
//! patterns = 3328, about 0.12 s per block at the 35 us a CBT launch
//! takes (measured: 18526 launches in 646 ms), 0.93 s for the eight.
//!
//! **Measure** restores the parameter set's code with the entry's own
//! strobe before the block's CA delays are trained, so the training that
//! follows is the one a build without the stage runs.
//!
//! **Apply** trains the CA delays a second time with each rank's chosen
//! code strobed in after every entry ([`ca_train`]), and the end of
//! initialization then writes that code to the rank's MR12
//! ([`end_of_init`]); a rank without a choice keeps the parameter set's.
//! If the second pass does not train all eight blocks with every echo
//! bit right, a third pass at the parameter set's code replaces it and
//! nothing is applied.
//!
//! **Why a bad code cannot leave the DRAM deaf.** Inferred from the
//! public LPDDR5 command descriptions, not shown on the board:
//! - inside CBT the code is set by the DQ strobe, a path that does not
//!   use the CA receivers, so the next strobe replaces any code;
//! - CBT is left by DQ7 going low, again not a CA command;
//! - the code under training belongs to the set-point being trained
//!   (FSP 1). Until the end of initialization switches FSP-OP, commands
//!   are received with FSP 0's reference, which nothing here touches;
//! - the end of initialization writes MR12 of FSP 1 in every case (the
//!   parameter set's value, or the applied choice), whatever the strobes
//!   left there, before it switches;
//! - a DRAM reset (every bring-up starts with one) returns every mode
//!   register to its reset value: nothing here outlives the run.

use super::{csr, csr_access, CaTrainReport, Executor};
use crate::hw::Hw;
use crate::para::Para;
use crate::phy::coldinit::{self, CbtBlock, CbtChannel};
use crate::phy::{devinit, endinit};

/// Launches one block's sweep may take, at most.
pub const BUDGET: u32 = 4096;
/// Common CA delays per code (one CA period in steps of four units) and
/// patterns per delay.
pub const POINTS: u32 = 64;
pub const POINT_STEP: u16 = 4;
pub const PATTERNS: u32 = 4;
/// Codes of one sweep, at most.
pub const CODES_MAX: usize = (BUDGET / (POINTS * PATTERNS)) as usize;
/// A plateau: codes whose window is within this many delay points of the
/// widest; the widest window a choice needs, in delay points.
pub const TOLERANCE: u8 = 1;
pub const MIN_WIDTH: u8 = 4;
/// The codes inside the absolute CA reference limits at the board's VDDQ
/// (the reference is 10 % + 0.5 % per code of VDDQ).
pub const CODE_MIN: u8 = code_at_least(jedec_lpddr::lp5::mr::CA_VREF_MV.0);
pub const CODE_MAX: u8 = code_at_most(jedec_lpddr::lp5::mr::CA_VREF_MV.1);

const fn code_mv_x1000(code: u32) -> u32 {
    crate::pstate::BOARD_VDDQ_MV * (100 + 5 * code)
}
const fn code_at_least(mv: u32) -> u8 {
    let mut c = 0;
    while code_mv_x1000(c) < mv * 1000 {
        c += 1;
    }
    c as u8
}
const fn code_at_most(mv: u32) -> u8 {
    let mut c = 127;
    while code_mv_x1000(c) > mv * 1000 {
        c -= 1;
    }
    c as u8
}
const _: () = assert!(CODE_MIN == 7 && CODE_MAX == 105 && CODES_MAX == 16);

/// The stage's plan (`A7A_CA_VREF`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Plan {
    pub apply: bool,
    /// Codes swept each side of the parameter set's, and their spacing.
    pub half: u8,
    pub step: u8,
}

/// Malformed CA Vref plan or sweep over the budget.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct InvalidPlan;

impl Plan {
    /// 0x18 codes each side in steps of 4 (13 codes: 12 % of VDDQ each
    /// way), measure only.
    pub const MEASURE: Plan = Plan {
        apply: false,
        half: 0x18,
        step: 4,
    };

    /// Codes of the sweep around `default`, before the limits.
    pub const fn n(&self) -> usize {
        2 * (self.half / self.step) as usize + 1
    }

    /// The `k`-th code around `default`; None outside [`CODE_MIN`]..=
    /// [`CODE_MAX`] (not swept).
    pub const fn code(&self, default: u8, k: usize) -> Option<u8> {
        let c = default as i32 + (k as i32 - (self.n() / 2) as i32) * self.step as i32;
        if c < CODE_MIN as i32 || c > CODE_MAX as i32 {
            None
        } else {
            Some(c as u8)
        }
    }

    pub const fn valid(&self) -> bool {
        self.step >= 1 && self.half >= self.step && self.n() <= CODES_MAX
    }

    /// `measure` or `apply`, optionally `,<half>,<step>` (hex). None:
    /// unset; Err: malformed or over the budget.
    pub const fn parse(s: Option<&str>) -> Result<Option<Plan>, InvalidPlan> {
        let Some(s) = s else {
            return Ok(None);
        };
        let b = s.as_bytes();
        let mut p = Plan::MEASURE;
        let mut i = if starts(b, b"measure") {
            7
        } else if starts(b, b"apply") {
            p.apply = true;
            5
        } else {
            return Err(InvalidPlan);
        };
        let mut field = 0;
        while i < b.len() {
            i += 1;
            let mut v: u32 = 0;
            let mut digits = 0;
            while i < b.len() && b[i] != b',' {
                let Some(d) = super::hex_nib(b[i]) else {
                    return Err(InvalidPlan);
                };
                v = v * 16 + d as u32;
                digits += 1;
                if v > 0x7f {
                    return Err(InvalidPlan);
                }
                i += 1;
            }
            if digits == 0 {
                return Err(InvalidPlan);
            }
            match field {
                0 => p.half = v as u8,
                1 => p.step = v as u8,
                _ => return Err(InvalidPlan),
            }
            field += 1;
        }
        if field == 1 || !p.valid() {
            return Err(InvalidPlan);
        }
        Ok(Some(p))
    }
}

const fn starts(b: &[u8], w: &[u8]) -> bool {
    if b.len() < w.len() {
        return false;
    }
    let mut i = 0;
    while i < w.len() {
        if b[i] != w[i] {
            return false;
        }
        i += 1;
    }
    b.len() == w.len() || b[w.len()] == b','
}

/// The plan of this build; None: the stage does not exist in it.
pub const STAGE: Option<Plan> = match Plan::parse(crate::knob!("A7A_CA_VREF")) {
    Ok(p) => p,
    Err(InvalidPlan) => {
        panic!("A7A_CA_VREF: measure|apply[,half,step] (hex codes, at most 16 codes)")
    }
};

// ---------------------------------------------------------------------------
// The code on the DQ lanes.

/// The asynchronous DQ data word (bit per PHY lane, bit 8 DMI) that puts
/// `code` on DQ[6:0] with DQ7 high and DMI low, through the DBYTE's lane
/// map (`lane_sel[dq]`: the lane of DRAM DQ `dq`).
pub const fn encode(lane_sel: &[u16; 8], code: u8) -> u16 {
    let mut w = 1 << (lane_sel[7] & 7);
    let mut dq = 0;
    while dq < 7 {
        if code >> dq & 1 != 0 {
            w |= 1 << (lane_sel[dq] & 7);
        }
        dq += 1;
    }
    w
}

/// The code a data word carries on DQ[6:0]; None unless DQ7 is high, DMI
/// low and the lane map a permutation.
pub const fn decode(lane_sel: &[u16; 8], word: u16) -> Option<u8> {
    let mut seen = 0u16;
    let mut code = 0u8;
    let mut dq = 0;
    while dq < 8 {
        let lane = lane_sel[dq];
        if lane > 7 || seen >> lane & 1 != 0 {
            return None;
        }
        seen |= 1 << lane;
        if dq < 7 && word >> lane & 1 != 0 {
            code |= 1 << dq;
        }
        dq += 1;
    }
    if word >> 8 != 0 || word >> lane_sel[7] & 1 == 0 {
        return None;
    }
    Some(code)
}

/// The code the entry of `c` strobes in, if its data words are a code
/// through `lane_sel` in the form this stage writes: the first word the
/// code with DMI low, the second the same with DMI high, the host's lanes
/// afterwards DQ7 and DMI.
pub const fn entry_code(c: &CbtChannel, lane_sel: &[u16; 8]) -> Option<u8> {
    let Some(code) = decode(lane_sel, c.tx_data[0]) else {
        return None;
    };
    if c.tx_data[1] != c.tx_data[0] | 0x100
        || c.tx_entry != 1 << lane_sel[7]
        || c.tx_lanes != c.tx_entry | 0x100
        || encode(lane_sel, code) != c.tx_data[0]
    {
        return None;
    }
    Some(code)
}

/// The CSR writes that replace the reference code inside CBT: DMI low
/// (the DRAM stops driving DQ[6:0]), the code on the host's lanes, all
/// lanes driven, DMI high (the strobe), DQ[6:0] released. The last four
/// are the entry's own (`coldinit` entry: data, data with DMI, mode,
/// enable). Each is followed by [`STROBE_HOLD_US`].
pub const fn strobe_writes(c: &CbtChannel, word: u16) -> [(u32, u16); 7] {
    let n = c.dbyte;
    [
        (coldinit::async_tx_data(n), c.tx_entry),
        (coldinit::async_tx_data(n), word),
        (coldinit::async_tx_mode(n), 0x1ff),
        (coldinit::async_tx_en(n), 0x1ff),
        (coldinit::async_tx_data(n), word | 0x100),
        (coldinit::async_tx_mode(n), c.tx_lanes),
        (coldinit::async_tx_en(n), c.tx_lanes),
    ]
}
/// Held after every write of [`strobe_writes`]: above each of the public
/// minimums between them (20 ns from DMI low to DQ input, 5 ns of set-up
/// and hold around the strobe, 250 ns for the new reference to settle).
pub const STROBE_HOLD_US: u32 = 2;

// ---------------------------------------------------------------------------
// The sweep and the choice (pure).

/// What the sweep needs from a block that is in CBT.
pub trait Cbt {
    /// Strobe in `code`; false: a launch or the port failed.
    fn set_code(&mut self, code: u8) -> bool;
    /// The common CA delay sweep at the current code: bit i = every echo
    /// bit right at delay point i. None: a launch failed.
    fn common(&mut self) -> Option<u64>;
}

/// One block's sweep: per code index the longest circular run of
/// passing delays, in points (None: not swept).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BlockSweep {
    pub width: [Option<u8>; CODES_MAX],
    pub launches: u32,
    /// The parameter set's code is back in the DRAM.
    pub restored: bool,
    pub failed: bool,
}

impl BlockSweep {
    pub const EMPTY: BlockSweep = BlockSweep {
        width: [None; CODES_MAX],
        launches: 0,
        restored: false,
        failed: false,
    };
}

/// Sweep one block and put `default` back.
pub fn sweep_block<C: Cbt>(c: &mut C, plan: &Plan, default: u8) -> BlockSweep {
    let mut out = BlockSweep::EMPTY;
    for k in 0..plan.n().min(CODES_MAX) {
        let Some(code) = plan.code(default, k) else {
            continue;
        };
        if out.launches + POINTS * PATTERNS > BUDGET {
            break;
        }
        if !c.set_code(code) {
            out.failed = true;
            break;
        }
        out.launches += POINTS * PATTERNS;
        let Some(map) = c.common() else {
            out.failed = true;
            break;
        };
        let (_, len) = coldinit::longest_run(map as u128, POINTS);
        out.width[k] = Some(len.min(255) as u8);
    }
    out.restored = c.set_code(default);
    out
}

/// The smaller width per code of two sweeps (the two CBT phases of a
/// rank); None where either has none.
pub fn combine(a: &BlockSweep, b: &BlockSweep) -> [Option<u8>; CODES_MAX] {
    core::array::from_fn(|k| match (a.width[k], b.width[k]) {
        (Some(x), Some(y)) => Some(x.min(y)),
        _ => None,
    })
}

/// What a rank's widths say.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Verdict {
    /// A plateau inside the sweep: its middle is the choice.
    Plateau,
    /// The plateau holds the parameter set's code as its middle: nothing
    /// to move.
    Default,
    /// The plateau reaches the first or last swept code: the best
    /// reference may lie outside the sweep. No choice.
    AtEnd,
    /// The widest window is under [`MIN_WIDTH`], or nothing was swept.
    Closed,
}

/// One rank's decision.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Choice {
    pub verdict: Verdict,
    /// The plateau's middle (a swept code), where there is a choice.
    pub code: Option<u8>,
    /// Width at the parameter set's code and the widest, in delay points.
    pub at_default: Option<u8>,
    pub widest: u8,
    /// The plateau as code indices.
    pub plateau: (u8, u8),
}

/// The choice from one rank's widths (module documentation).
pub fn choose(width: &[Option<u8>; CODES_MAX], plan: &Plan, default: u8) -> Choice {
    let n = plan.n().min(CODES_MAX);
    let mid = n / 2;
    let widest = width[..n].iter().flatten().copied().max().unwrap_or(0);
    let mut out = Choice {
        verdict: Verdict::Closed,
        code: None,
        at_default: width[mid],
        widest,
        plateau: (0, 0),
    };
    if widest < MIN_WIDTH {
        return out;
    }
    let on = |k: usize| width[k].is_some_and(|w| w + TOLERANCE >= widest);
    // The plateau that holds the widest code nearest the middle.
    let Some(top) = (0..n)
        .filter(|&k| width[k] == Some(widest))
        .min_by_key(|&k| k.abs_diff(mid))
    else {
        return out;
    };
    let (mut lo, mut hi) = (top, top);
    while lo > 0 && on(lo - 1) {
        lo -= 1;
    }
    while hi + 1 < n && on(hi + 1) {
        hi += 1;
    }
    out.plateau = (lo as u8, hi as u8);
    let swept = |k: usize| width[k].is_some();
    let first = (0..n).find(|&k| swept(k)).unwrap_or(0);
    let last = (0..n).rev().find(|&k| swept(k)).unwrap_or(0);
    if lo == first || hi == last {
        out.verdict = Verdict::AtEnd;
        return out;
    }
    let k = (lo + hi) / 2;
    out.code = plan.code(default, k);
    out.verdict = if k == mid {
        Verdict::Default
    } else {
        Verdict::Plateau
    };
    out
}

/// The stage's result.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Report {
    pub plan: Plan,
    /// Per channel: the code its entry strobes in (None: the entry's data
    /// words are not a code through the lane map read from the PHY; the
    /// stage then leaves the channel alone), and the lane map.
    pub default: [Option<u8>; 2],
    pub lane_sel: [[u16; 8]; 2],
    /// Per `coldinit::CBT_BLOCKS` entry.
    pub blocks: [BlockSweep; 8],
    /// Per channel and rank.
    pub choice: [[Option<Choice>; 2]; 2],
    /// The codes the second pass trained at and the end of initialization
    /// writes (None: the parameter set's).
    pub applied: [[Option<u8>; 2]; 2],
    /// The second pass ran; it was replaced by a pass at the parameter
    /// set's codes.
    pub second_pass: bool,
    pub reverted: bool,
}

impl Report {
    pub const fn empty(plan: Plan) -> Report {
        Report {
            plan,
            default: [None; 2],
            lane_sel: [[0; 8]; 2],
            blocks: [BlockSweep::EMPTY; 8],
            choice: [[None; 2]; 2],
            applied: [[None; 2]; 2],
            second_pass: false,
            reverted: false,
        }
    }
}

/// The codes apply trains at: per channel and rank the choice of a
/// [`Verdict::Plateau`]; None everywhere else.
pub fn to_apply(r: &Report) -> [[Option<u8>; 2]; 2] {
    core::array::from_fn(|ch| {
        core::array::from_fn(|rank| match r.choice[ch][rank] {
            Some(Choice {
                verdict: Verdict::Plateau,
                code,
                ..
            }) if r.plan.apply => code,
            _ => None,
        })
    })
}

/// A trained report is whole: eight blocks, no failure, every echo bit
/// right at the trained delays.
pub fn trained(t: &CaTrainReport) -> bool {
    t.done == 8
        && t.exec.failure.is_none()
        && coldinit::CBT_BLOCKS
            .iter()
            .zip(t.blocks.iter())
            .all(|(b, r)| r.final_pass == b.chan.echo_bits())
}

// ---------------------------------------------------------------------------
// The stage on hardware.

/// The codes the end of initialization writes (set by an applied
/// [`ca_train`]; bt0 is single-threaded).
static mut APPLIED: [[Option<u8>; 2]; 2] = [[None; 2]; 2];
/// The last run's report, for the log.
static mut REPORT: Report = Report::empty(Plan::MEASURE);

struct BlockPort<'a, 'b, H: Hw> {
    ex: &'a mut Executor<'b, H>,
    b: &'a CbtBlock,
    lane_sel: [u16; 8],
}

impl<H: Hw> BlockPort<'_, '_, H> {
    fn strobe(&mut self, code: u8) -> bool {
        let word = encode(&self.lane_sel, code);
        for (c, v) in strobe_writes(&self.b.chan, word) {
            self.ex.step(coldinit::Step::Write(c, v));
            self.ex.hw.udelay(STROBE_HOLD_US);
        }
        self.ex.r.failure.is_none()
    }
}

impl<H: Hw> Cbt for BlockPort<'_, '_, H> {
    fn set_code(&mut self, code: u8) -> bool {
        self.strobe(code)
    }
    fn common(&mut self) -> Option<u64> {
        let bits = self.b.chan.echo_bits();
        let mut map = 0u64;
        for i in 0..POINTS {
            self.ex.hw.kick_watchdog();
            let echo = self.ex.cbt_measure(self.b, &[i as u16 * POINT_STEP; 7])?;
            if coldinit::cbt_pass_bits(echo, bits) == bits {
                map |= 1 << i;
            }
        }
        Some(map)
    }
}

/// One pass over the eight blocks, as `cold_init_ca_train` makes it:
/// `first` brings the DRAM up to the first block's CBT, a later pass
/// enters it again (`coldinit::part2`). `sweep`: every block's reference
/// sweep before its CA training; `codes`: the code strobed in before a
/// block's CA training.
fn pass<H: Hw>(
    hw: &mut H,
    (m1, m2): (&coldinit::Mrs, &coldinit::Mrs2),
    out: &mut CaTrainReport,
    first: bool,
    mut sweep: Option<&mut Report>,
    codes: &[[Option<u8>; 2]; 2],
    lane_sel: &[[u16; 8]; 2],
) {
    let t0 = hw.now_us();
    let dev = out.devinit;
    *out = CaTrainReport::EMPTY;
    out.devinit = dev;
    let mut ex = Executor {
        hw: &mut *hw,
        r: out.exec,
        loop_run: 0,
        mem_reset: 3,
    };
    if first {
        super::bring_up_to_cbt(&mut ex, m1, m2, &mut out.devinit, &mut |_| {});
    } else {
        coldinit::part2(m2, &mut |s| ex.step(s));
    }
    for (i, b) in coldinit::CBT_BLOCKS.iter().enumerate() {
        if ex.r.failure.is_some() {
            break;
        }
        if i > 0 {
            coldinit::block_entry(b, m2, &mut |s| ex.step(s));
        }
        let ch = b.chan.ac as usize & 1;
        let mut port = BlockPort {
            ex: &mut ex,
            b,
            lane_sel: lane_sel[ch],
        };
        if let Some(r) = sweep.as_deref_mut() {
            if let Some(default) = r.default[ch] {
                r.blocks[i] = sweep_block(&mut port, &r.plan, default);
            }
        }
        if let Some(code) = codes[ch][b.rank as usize & 1] {
            port.strobe(code);
        }
        ex.train_block(b, &mut out.blocks[i]);
        coldinit::block_exit(b, m2, &mut |s| ex.step(s));
        if ex.r.failure.is_none() {
            out.done += 1;
        }
    }
    coldinit::cbt_end(&mut |s| ex.step(s));
    out.channels = [
        super::combine_channel(&out.blocks, 0),
        super::combine_channel(&out.blocks, 1),
    ];
    if out.done == 8 {
        for (c, ca) in [coldinit::CBT_A, coldinit::CBT_B]
            .iter()
            .zip(out.channels.iter())
        {
            coldinit::set_block_delays(c.ac, &super::ca_applied(&ca.delay), &mut |s| ex.step(s));
        }
    }
    out.exec = ex.r;
    csr_access(hw, false);
    out.exec.elapsed_us = hw.now_us().wrapping_sub(t0);
}

/// Command bus training with the reference step: `cold_init_ca_train`
/// itself in a build without `A7A_CA_VREF`; with it, the pass with every
/// block's sweep, the choices, and with `apply` the second pass (module
/// documentation). `out` is the report of the pass whose CA delays stay.
#[inline(always)]
pub fn ca_train<H: Hw>(
    hw: &mut H,
    m1: &coldinit::Mrs,
    m2: &coldinit::Mrs2,
    out: &mut CaTrainReport,
) {
    let Some(plan) = STAGE else {
        return super::cold_init_ca_train(hw, m1, m2, out);
    };
    // SAFETY: bt0 is single-threaded and nothing else refers to these.
    let (r, applied) = unsafe {
        (
            &mut *core::ptr::addr_of_mut!(REPORT),
            &mut *core::ptr::addr_of_mut!(APPLIED),
        )
    };
    *r = Report::empty(plan);
    *applied = [[None; 2]; 2];
    *out = CaTrainReport::EMPTY;
    out.devinit = devinit::DevinitReport::EMPTY;
    // The lane maps as the PHY holds them, and the code each channel's
    // entry strobes in through them.
    csr_access(hw, true);
    for (ch, c) in [coldinit::CBT_A, coldinit::CBT_B].iter().enumerate() {
        for dq in 0..8 {
            r.lane_sel[ch][dq] = hw.read16(csr(0x10080 + 0x1000 * c.dbyte + dq as u32)) & 0xf;
        }
        r.default[ch] = entry_code(c, &r.lane_sel[ch]);
    }
    let maps = r.lane_sel;
    pass(hw, (m1, m2), out, true, Some(r), &[[None; 2]; 2], &maps);
    for ch in 0..2 {
        let Some(default) = r.default[ch] else {
            continue;
        };
        for rank in 0..2 {
            let k = 4 * ch + 2 * rank;
            let w = combine(&r.blocks[k], &r.blocks[k + 1]);
            r.choice[ch][rank] = Some(choose(&w, &plan, default));
        }
    }
    let codes = to_apply(r);
    if trained(out) && codes.iter().flatten().any(|c| c.is_some()) {
        r.second_pass = true;
        pass(hw, (m1, m2), out, false, None, &codes, &maps);
        if trained(out) {
            r.applied = codes;
            *applied = codes;
        } else {
            // Back to the parameter set's code on every rank.
            r.reverted = true;
            let back = [[r.default[0]; 2], [r.default[1]; 2]];
            pass(hw, (m1, m2), out, false, None, &back, &maps);
        }
    }
    log(hw, r);
}

/// The end of initialization with an applied reference: as
/// `super::end_of_init`, MR12 of a rank with an applied code replaced by
/// it (an `A7A_BL_MRS` value for MR12 still wins). Without an applied
/// code this is `super::end_of_init` itself.
#[inline(always)]
pub fn end_of_init<H: Hw>(hw: &mut H, p: &mut Para, pstate: u32) -> super::EndInitReport {
    if STAGE.is_none() {
        return super::end_of_init(hw, p, pstate);
    }
    // SAFETY: bt0 is single-threaded.
    let codes = unsafe { *core::ptr::addr_of!(APPLIED) };
    if codes.iter().flatten().all(|c| c.is_none()) {
        return super::end_of_init(hw, p, pstate);
    }
    use crate::phy::wcklevel::{Channel, Rank};
    use endinit::FspMrs;
    let variant = crate::knobs::get().ei;
    let m = super::message_block(p, pstate);
    let mrs_of = |ch: Channel, rank: Rank| {
        let mut f = FspMrs::from_message_block(&m, ch, rank);
        let code = codes[matches!(ch, Channel::B) as usize][matches!(rank, Rank::R1) as usize];
        set_mr12(&mut f, code);
        super::mission_mr_overrides(&mut f);
        f
    };
    let mut mrs = [FspMrs { mr: [(0, 0); 20] }; 4];
    for (o, &(ch, rank)) in mrs.iter_mut().zip(endinit::BLOCKS.iter()) {
        *o = mrs_of(ch, rank);
    }
    let t0 = hw.now_us();
    let mut ex = crate::wl::WlExec::new(hw);
    endinit::run_with(Channel::B, mrs_of, &variant, &mut |s| ex.step(s));
    let r = super::EndInitReport {
        variant,
        mrs,
        launches: ex.launches,
        writes: ex.writes,
        max_launch_us: ex.max_launch_us,
        elapsed_us: 0,
        failure: ex.failure,
    };
    csr_access(hw, false);
    super::EndInitReport {
        elapsed_us: hw.now_us().wrapping_sub(t0) as u32,
        ..r
    }
}

/// MR12 of `f` with the reference code `code` (OP[7] kept).
pub fn set_mr12(f: &mut endinit::FspMrs, code: Option<u8>) {
    let Some(code) = code else {
        return;
    };
    for slot in f.mr.iter_mut().filter(|s| s.0 == 12) {
        slot.1 = (slot.1 & 0x80) | (code & 0x7f);
    }
}

/// The stage's lines (`CAV ...`): the plan, per channel the code its
/// entry carries, per block the window per code, per channel and rank
/// the combined windows and the choice, and what was applied.
pub fn log<H: Hw>(hw: &mut H, r: &Report) {
    let p = r.plan;
    hw.kick_watchdog();
    hw.log(format_args!(
        "[dram] CAV: command-bus reference in CBT: apply {}, {} codes step {} around the entry's, limits {:02x}..{:02x}; per code {} delays x {} patterns, at most {BUDGET} launches per block; entry code per channel {:02x?} (lane maps {:x?})\n",
        p.apply, p.n(), p.step, CODE_MIN, CODE_MAX, POINTS, PATTERNS, r.default, r.lane_sel
    ));
    for (i, (b, s)) in coldinit::CBT_BLOCKS.iter().zip(r.blocks.iter()).enumerate() {
        hw.kick_watchdog();
        hw.log(format_args!(
            "[dram] CAV {i} ch {} rank {} phase {}: window per code (delay points of {POINT_STEP}) {:?}; {} launches, restored {}, failed {}\n",
            ["A", "B"][b.chan.ac as usize & 1], b.rank, b.phase, &s.width[..p.n().min(CODES_MAX)], s.launches, s.restored, s.failed
        ));
    }
    for ch in 0..2 {
        for rank in 0..2 {
            let Some(c) = r.choice[ch][rank] else {
                continue;
            };
            let d = r.default[ch].unwrap_or(0);
            hw.kick_watchdog();
            hw.log(format_args!(
                "[dram] CAV ch {} rank {rank}: {:?}: entry code {d:02x} window {:?}, widest {} on codes {:02x?}..{:02x?}, choice {:02x?}; applied {:02x?}\n",
                ["A", "B"][ch], c.verdict, c.at_default, c.widest,
                p.code(d, c.plateau.0 as usize), p.code(d, c.plateau.1 as usize), c.code, r.applied[ch][rank]
            ));
        }
    }
    hw.log(format_args!(
        "[dram] CAV: second pass {}, reverted {}; MR12 at the end of initialization {:02x?} (None: the parameter set's)\n",
        r.second_pass, r.reverted, r.applied
    ));
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use coldinit::{CBT_A, CBT_B};
    use std::vec::Vec;

    /// The board's lane maps of DBYTE 1 and DBYTE 3 (a board log).
    const SEL_A: [u16; 8] = [3, 2, 1, 0, 5, 7, 6, 4];
    const SEL_B: [u16; 8] = [6, 7, 3, 0, 4, 5, 1, 2];

    #[test]
    fn the_entry_words_are_the_parameter_sets_mr12() {
        // Both channels' entry data words carry 0x44 on DQ[6:0] with DQ7
        // high: the reference code of the parameter set.
        assert_eq!(decode(&SEL_A, CBT_A.tx_data[0]), Some(0x44));
        assert_eq!(decode(&SEL_B, CBT_B.tx_data[0]), Some(0x44));
        assert_eq!(entry_code(&CBT_A, &SEL_A), Some(0x44));
        assert_eq!(entry_code(&CBT_B, &SEL_B), Some(0x44));
        assert_eq!(encode(&SEL_A, 0x44), 0x52);
        assert_eq!(encode(&SEL_B, 0x44), 0x0e);
        // Every code survives the lane maps.
        for code in 0..=0x7f {
            for sel in [&SEL_A, &SEL_B] {
                assert_eq!(decode(sel, encode(sel, code)), Some(code));
            }
        }
        // Another map is not accepted: nothing would be written.
        assert_eq!(entry_code(&CBT_A, &SEL_B), None);
        assert_eq!(entry_code(&CBT_A, &[0, 1, 2, 3, 4, 5, 6, 7]), None);
        assert_eq!(decode(&[0, 0, 2, 3, 4, 5, 6, 7], 0x80), None);
        assert_eq!(decode(&SEL_A, 0x152), None, "DMI high is not a code word");
        assert_eq!(decode(&SEL_A, 0x42), None, "DQ7 low is not a code word");
    }

    #[test]
    fn the_strobe_ends_as_the_entry_does() {
        for (c, sel) in [(CBT_A, SEL_A), (CBT_B, SEL_B)] {
            let w = strobe_writes(&c, encode(&sel, 0x44));
            let n = c.dbyte;
            // DMI low first, only DQ7 driven high.
            assert_eq!(w[0], (coldinit::async_tx_data(n), c.tx_entry));
            // The entry's own four writes close it.
            assert_eq!(w[1], (coldinit::async_tx_data(n), c.tx_data[0]));
            assert_eq!(w[4], (coldinit::async_tx_data(n), c.tx_data[1]));
            assert_eq!(w[5], (coldinit::async_tx_mode(n), c.tx_lanes));
            assert_eq!(w[6], (coldinit::async_tx_en(n), c.tx_lanes));
            // The code is on the lanes before they are driven, and DMI
            // rises only while they are.
            assert_eq!((w[2].1, w[3].1), (0x1ff, 0x1ff));
            assert!(w[1].1 & 0x100 == 0 && w[4].1 & 0x100 != 0);
            // DQ7 stays high throughout (leaving CBT is DQ7 low).
            assert!(w
                .iter()
                .filter(|x| x.0 == coldinit::async_tx_data(n))
                .all(|x| x.1 & c.tx_entry != 0));
        }
    }

    /// A block in CBT: the CA window in delay points as a function of the
    /// code (widest at `best`, closing by `slope` points per code), and
    /// codes at which the launches fail.
    struct Model {
        best: f32,
        peak: f32,
        slope: f32,
        code: u8,
        codes: Vec<u8>,
        fail_code: Option<u8>,
        commons: u32,
    }

    impl Model {
        fn new(best: f32, peak: f32, slope: f32) -> Model {
            Model {
                best,
                peak,
                slope,
                code: 0,
                codes: Vec::new(),
                fail_code: None,
                commons: 0,
            }
        }
        fn width(&self) -> u32 {
            (self.peak - self.slope * (self.code as f32 - self.best).abs()).max(0.0) as u32
        }
    }

    impl Cbt for Model {
        fn set_code(&mut self, code: u8) -> bool {
            assert!((CODE_MIN..=CODE_MAX).contains(&code));
            self.code = code;
            self.codes.push(code);
            true
        }
        fn common(&mut self) -> Option<u64> {
            self.commons += 1;
            if self.fail_code == Some(self.code) {
                return None;
            }
            // A window that wraps around the period's end.
            let w = self.width();
            let mut map = 0u64;
            for i in 0..w {
                map |= 1 << ((60 + i) % POINTS);
            }
            Some(map)
        }
    }

    const P: Plan = Plan::MEASURE;

    #[test]
    fn the_sweep_finds_the_plateau_and_puts_the_code_back() {
        // The best reference 10 codes above the parameter set's.
        let mut m = Model::new(78.0, 14.0, 0.5);
        let s = sweep_block(&mut m, &P, 0x44);
        assert_eq!(s.launches, 13 * 256);
        assert!(s.launches <= BUDGET && s.restored && !s.failed);
        assert_eq!(m.codes.len(), 14);
        assert_eq!(m.codes.first(), Some(&0x2c));
        assert_eq!(m.codes[12], 0x5c);
        assert_eq!(
            m.codes.last(),
            Some(&0x44),
            "the parameter set's code is back"
        );
        let c = choose(&combine(&s, &s), &P, 0x44);
        assert_eq!(c.verdict, Verdict::Plateau);
        assert_eq!(c.code, Some(0x4c));
        assert_eq!((c.at_default, c.widest), (Some(9), 13));
        // The window wrapped around the period and was still counted.
        assert_eq!(s.width[6], Some(9));
    }

    #[test]
    fn a_centred_window_keeps_the_parameter_sets_code() {
        let mut m = Model::new(68.0, 14.0, 0.3);
        let s = sweep_block(&mut m, &P, 0x44);
        let c = choose(&s.width, &P, 0x44);
        assert_eq!((c.verdict, c.code), (Verdict::Default, Some(0x44)));
        let mut r = Report::empty(Plan { apply: true, ..P });
        r.choice[0][0] = Some(c);
        assert_eq!(to_apply(&r), [[None; 2]; 2], "nothing to move");
    }

    #[test]
    fn no_choice_at_the_sweeps_end_or_without_a_window() {
        // Best beyond the sweep: the plateau reaches the last code.
        let mut m = Model::new(110.0, 30.0, 0.5);
        let s = sweep_block(&mut m, &P, 0x44);
        let c = choose(&s.width, &P, 0x44);
        assert_eq!((c.verdict, c.code), (Verdict::AtEnd, None));
        // A flat response: every code the same window.
        let mut m = Model::new(68.0, 12.0, 0.0);
        let s = sweep_block(&mut m, &P, 0x44);
        assert_eq!(choose(&s.width, &P, 0x44).verdict, Verdict::AtEnd);
        // No window at any code.
        let mut m = Model::new(68.0, 2.0, 1.0);
        let s = sweep_block(&mut m, &P, 0x44);
        let c = choose(&s.width, &P, 0x44);
        assert_eq!((c.verdict, c.code, c.widest), (Verdict::Closed, None, 2));
        assert_eq!(
            choose(&[None; CODES_MAX], &P, 0x44).verdict,
            Verdict::Closed
        );
    }

    #[test]
    fn a_failed_launch_stops_the_sweep_and_still_restores() {
        let mut m = Model::new(68.0, 14.0, 0.3);
        m.fail_code = Some(0x38);
        let s = sweep_block(&mut m, &P, 0x44);
        assert!(s.failed && s.restored);
        assert_eq!(m.codes.last(), Some(&0x44));
        assert_eq!(s.width[3], None);
        assert_eq!(m.commons, 4);
    }

    #[test]
    fn the_two_phases_combine_to_the_smaller_window() {
        let mut a = Model::new(64.0, 14.0, 0.5);
        let mut b = Model::new(84.0, 14.0, 0.5);
        let (sa, sb) = (sweep_block(&mut a, &P, 0x44), sweep_block(&mut b, &P, 0x44));
        let w = combine(&sa, &sb);
        for (k, &width) in w[..P.n()].iter().enumerate() {
            assert_eq!(width, Some(sa.width[k].unwrap().min(sb.width[k].unwrap())));
        }
        // Between the two phases' best codes.
        let c = choose(&w, &P, 0x44);
        assert_eq!(c.verdict, Verdict::Plateau);
        assert!((0x48..=0x4c).contains(&c.code.unwrap()), "{c:?}");
    }

    #[test]
    fn codes_stay_inside_the_limits_and_the_budget() {
        // A parameter-set code near the lower limit: the codes below it
        // are not swept.
        let mut m = Model::new(20.0, 14.0, 0.5);
        let s = sweep_block(&mut m, &P, 0x10);
        assert!(m.codes.iter().all(|&c| (CODE_MIN..=CODE_MAX).contains(&c)));
        assert_eq!(s.width[0], None);
        assert_eq!(s.width[4], Some(8), "code 0x08");
        assert_eq!(P.code(0x44, 6), Some(0x44));
        assert_eq!(P.code(0x44, 0), Some(0x2c));
        assert_eq!(P.code(0x68, 12), None);
        // The widest plan: 16 codes is the budget.
        let wide = Plan::parse(Some("measure,1e,4")).unwrap().unwrap();
        assert_eq!(wide.n(), 15);
        assert!(Plan::parse(Some("measure,20,4")).is_err(), "17 codes");
        assert!(Plan::parse(Some("apply,8,2")).unwrap().unwrap().apply);
        assert_eq!(Plan::parse(Some("measure")), Ok(Some(Plan::MEASURE)));
        assert_eq!(Plan::parse(None), Ok(None));
        for bad in [
            "sweep",
            "measure,",
            "measure,8",
            "apply,8,0",
            "apply,2,4",
            "measure,8,2,1",
        ] {
            assert!(Plan::parse(Some(bad)).is_err(), "{bad}");
        }
        assert_eq!(STAGE, None, "the stage is off without its knob");
    }

    #[test]
    fn apply_moves_only_a_measured_plateau_and_mr12_follows() {
        let mut r = Report::empty(Plan { apply: true, ..P });
        let pick = |verdict, code| {
            Some(Choice {
                verdict,
                code,
                at_default: Some(9),
                widest: 13,
                plateau: (5, 9),
            })
        };
        r.choice = [
            [
                pick(Verdict::Plateau, Some(0x4c)),
                pick(Verdict::Default, Some(0x44)),
            ],
            [pick(Verdict::AtEnd, None), pick(Verdict::Closed, None)],
        ];
        assert_eq!(to_apply(&r), [[Some(0x4c), None], [None, None]]);
        r.plan.apply = false;
        assert_eq!(to_apply(&r), [[None; 2]; 2], "measure applies nothing");
        // MR12 in the end of initialization's list.
        let mut f = endinit::FspMrs { mr: [(0, 0); 20] };
        for (slot, &ma) in f.mr.iter_mut().zip(endinit::ORDER.iter()) {
            *slot = (ma, if ma == 12 { 0x44 } else { 0x11 });
        }
        let before = f;
        set_mr12(&mut f, None);
        assert_eq!(f, before);
        set_mr12(&mut f, Some(0x4c));
        for (a, b) in f.mr.iter().zip(before.mr.iter()) {
            assert_eq!(*a, if b.0 == 12 { (12, 0x4c) } else { *b });
        }
    }

    #[test]
    fn a_report_is_trained_only_when_whole() {
        let mut t = CaTrainReport::EMPTY;
        assert!(!trained(&t));
        t.done = 8;
        for (b, r) in coldinit::CBT_BLOCKS.iter().zip(t.blocks.iter_mut()) {
            r.final_pass = b.chan.echo_bits();
        }
        assert!(trained(&t));
        t.blocks[5].final_pass &= !1;
        assert!(!trained(&t));
    }
}
