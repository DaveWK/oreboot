// SPDX-License-Identifier: GPL-2.0-only
//! ACSM row-field measurements (FEL lab). Each case changes ONE field of a known row and records what
//! the PHY shows. Nothing here runs unless a bt0 knob asks for it.
//!
//! [`reset_probe`] (bt0 `A7A_ACSM_PROBE=reset`, blobless pre-training with
//! the DRAM held in reset, MemResetL 2): no command can reach the DRAM.
//! - T cases: launch time of one row pair, repeated by the outer loop
//!   (ACSMOuterLoopRepeatCnt 255: 256 executions), three times each: the
//!   REPEATCNT position, width and unit, the even-row flags, the loop
//!   construct, ACSMRptCntOverride/Dbl (RCNE0) and the low-speed clock.
//! - S cases: ACSMCkeControl 1 (ACSM drives CKE; LPDDR5 has no CKE pin),
//!   then ACSMCkeStatus after a pair with one candidate bit: the CKE field.
//! - W cases: ACSMWckEnControl 1, ACSMWckEnStatus before and after a pair
//!   with one candidate bit: the WCKTG field (each WCKTG toggles).
//! - X cases: the RX FIFO DFE0 pointers of DBYTE 0/1 (RxFifoVisibility
//!   0x200, receivers out of standby, FIFO reset) before and after a pair
//!   with one hw1 strobe bit: which bits make the PHY capture.
//!
//! Every CSR a case changes is put back after it.
//!
//! [`field_probe`] (bt0 `A7A_ACSM_FIELD=<variant>`, feature `hybrid`, after
//! the PMU trained): one MRR through the DCA stage's read-out with one field
//! changed (the first read of our stage), then the reference MRRs of MR5 and
//! MR8. MRR only: no MRW, no DRAM state change.

use crate::dca::{self, Mrr};
use crate::hw::Hw;
use crate::phy::acsm_row::{hw1, hw2, hw3, pulses, wait_pair, wck};
use crate::phy::coldinit::Row;
use crate::phy::csr_access;
use crate::phy::rxen;
use crate::phy::wcklevel::{Channel, Rank, Step};
use crate::rd::{dbytes, fifo_info, fire, EV_FIFO};
use crate::rdtrain::RX_STANDBY;
use crate::wl::{WlExec, WlFailure};

/// What a case records besides the launch time.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Obs {
    Time,
    Cke,
    Wck,
    Fifo,
}

/// One case: up to four rows (two pairs), the outer loop count, up to
/// four CSR writes before (put back after) and the observation.
#[derive(Clone, Copy, Debug)]
pub struct Case {
    pub id: &'static str,
    pub rows: [Row; 4],
    pub len: u8,
    pub outer: u16,
    pub pre: [(u32, u16); 4],
    pub npre: u8,
    pub obs: Obs,
}

const E: Row = Row::EMPTY;
const NO_PRE: [(u32, u16); 4] = [(0, 0); 4];

const fn pair(even: u16, odd: u16) -> [Row; 4] {
    [Row([0, 0, 0, even]), Row([0, 0, 0, odd]), E, E]
}

/// Timing case: pair (`even`, `odd` hw3), outer 255.
const fn t(id: &'static str, even: u16, odd: u16) -> Case {
    Case {
        id,
        rows: pair(even, odd),
        len: 2,
        outer: 255,
        pre: NO_PRE,
        npre: 0,
        obs: Obs::Time,
    }
}

const fn with_pre(mut c: Case, pre: &[(u32, u16)]) -> Case {
    let mut i = 0;
    while i < pre.len() {
        c.pre[i] = pre[i];
        i += 1;
    }
    c.npre = pre.len() as u8;
    c
}

const fn outer(mut c: Case, n: u16) -> Case {
    c.outer = n;
    c
}

const fn rows4(mut c: Case, rows: [Row; 4]) -> Case {
    c.rows = rows;
    c.len = 4;
    c
}

/// Observation case with `rows` (two rows), outer 0.
const fn o(id: &'static str, rows: [Row; 2], obs: Obs, pre: &[(u32, u16)]) -> Case {
    with_pre(
        Case {
            id,
            rows: [rows[0], rows[1], E, E],
            len: 2,
            outer: 0,
            pre: NO_PRE,
            npre: 0,
            obs,
        },
        pre,
    )
}

pub const RUN_CTRL: u32 = 0x20020;
pub const CKE_CONTROL: u32 = 0x20028;
pub const CKE_STATUS: u32 = 0x20029;
pub const WCK_EN_CONTROL: u32 = 0x2002a;
pub const WCK_EN_STATUS: u32 = 0x2002b;
pub const LOW_SPEED_EN: u32 = 0x20042;
pub const LOW_SPEED_DELAY: u32 = 0x20044;
pub const RPT_OVERRIDE: u32 = 0x20045;
pub const RPT_DBL: u32 = 0x20046;

/// REPEATCNT 64 with the reference's wait flags: the reference of the flag
/// and register cases.
const N64: (u16, u16) = (0x0b00, 0x2000);
const OVR: [(u32, u16); 2] = [(RUN_CTRL, 0x100), (RPT_OVERRIDE, 200)];
const CKE: [(u32, u16); 1] = [(CKE_CONTROL, 1)];
const WCKC: [(u32, u16); 1] = [(WCK_EN_CONTROL, 1)];
const FIFO_PRE: [(u32, u16); 3] = [(RX_STANDBY, 0), (0x10072, 0x200), (0x11072, 0x200)];

const fn h3(even: u16) -> [Row; 2] {
    [Row([0, 0, 0, even]), E]
}
const fn h1(v: u16) -> [Row; 2] {
    [Row([0, v, 0, 0]), E]
}

/// The DRAM-free cases in run order.
pub const CASES: [Case; 66] = [
    // REPEATCNT position and width.
    t("T00-empty", 0, 0),
    t("T01-n0", 0x0b00, 0),
    t("T02-n1", 0x1b00, 0),
    t("T03-n2", 0x2b00, 0),
    t("T04-n4", 0x4b00, 0),
    t("T05-n7", 0x7b00, 0),
    t("T06-n8", 0x0b00, 0x0400),
    t("T07-n16", 0x0b00, 0x0800),
    t("T08-n32", 0x0b00, 0x1000),
    t("T09-n64", N64.0, N64.1),
    t("T10-n128", 0x0b00, 0x4000),
    t("T11-n248", 0x0b00, 0x7c00),
    t("T12-n255", 0x7b00, 0x7c00),
    t("T13-even-b15", 0x8b00, 0),
    t("T14-odd-b15", 0x0b00, 0x8000),
    t("T15-odd-b9", 0x0b00, 0x0200),
    t("T16-odd-b8", 0x0b00, 0x0100),
    // Even-row flags, one at a time from the reference N64.
    t("T17-n64-noflags", 0, N64.1),
    t("T18-n64-noF8", 0x0a00, N64.1),
    t("T19-n64-noF9", 0x0900, N64.1),
    t("T20-n64-noF11", 0x0300, N64.1),
    t("T21-n64-F10", 0x0f00, N64.1),
    t("T22-n64-F7", 0x0b80, N64.1),
    t("T23-n64-TG", 0x0b10, N64.1),
    t("T24-n7-noflags", 0x7000, 0),
    // Loop construct: start (F11|F9, count) ... end (F8), two pairs.
    rows4(
        t("T25-loop-a00-100", 0x0a00, 0x2000),
        [
            Row([0, 0, 0, 0x0a00]),
            Row([0, 0, 0, 0x2000]),
            Row([0, 0, 0, 0x0100]),
            E,
        ],
    ),
    rows4(
        t("T26-loop-b00-100", 0x0b00, 0x2000),
        [
            Row([0, 0, 0, 0x0b00]),
            Row([0, 0, 0, 0x2000]),
            Row([0, 0, 0, 0x0100]),
            E,
        ],
    ),
    rows4(
        t("T27-loop-a00-noend", 0x0a00, 0x2000),
        [Row([0, 0, 0, 0x0a00]), Row([0, 0, 0, 0x2000]), E, E],
    ),
    rows4(
        t("T28-cbt-1a00-100", 0x1a00, 0),
        [Row([0, 0, 0, 0x1a00]), E, Row([0, 0, 0, 0x0100]), E],
    ),
    rows4(
        t("T29-two-n64", N64.0, N64.1),
        [
            Row([0, 0, 0, N64.0]),
            Row([0, 0, 0, N64.1]),
            Row([0, 0, 0, N64.0]),
            Row([0, 0, 0, N64.1]),
        ],
    ),
    // Clock modes (ACSMLowSpeedClockEnable).
    with_pre(
        t("T30-n64-ls103", N64.0, N64.1),
        &[(LOW_SPEED_EN, 0x103), (LOW_SPEED_DELAY, 1)],
    ),
    with_pre(
        t("T31-n64-ls183", N64.0, N64.1),
        &[(LOW_SPEED_EN, 0x183), (LOW_SPEED_DELAY, 1)],
    ),
    with_pre(
        t("T32-empty-ls103", 0, 0),
        &[(LOW_SPEED_EN, 0x103), (LOW_SPEED_DELAY, 1)],
    ),
    // ACSMRptCntDbl, ACSMRptCntOverride (RCNE0 among the flags?).
    with_pre(t("T33-n64-dbl", N64.0, N64.1), &[(RPT_DBL, 1)]),
    with_pre(t("T34-n64-ovr200", N64.0, N64.1), &OVR),
    with_pre(t("T35-n64-ovr-noF8", 0x0a00, N64.1), &OVR),
    with_pre(t("T36-n64-ovr-noF9", 0x0900, N64.1), &OVR),
    with_pre(t("T37-n64-ovr-noF11", 0x0300, N64.1), &OVR),
    with_pre(t("T38-n64-ovr-F10", 0x0f00, N64.1), &OVR),
    with_pre(t("T39-n64-ovr-F7", 0x0b80, N64.1), &OVR),
    // One execution (outer loop 0): the per-launch overhead.
    outer(t("T40-n64-once", N64.0, N64.1), 0),
    outer(t("T41-n255-once", 0x7b00, 0x7c00), 0),
    // Outer loop 0 with a timed (F10) pair, against outer 1 and 3: the
    // public TRM says 0 executes the program twice. With the override of
    // 200 one execution is 805 DfiClk (T38), 8 us at pstate 3.
    outer(with_pre(t("T42-ovr-F10-outer0", 0x0f00, N64.1), &OVR), 0),
    outer(with_pre(t("T43-ovr-F10-outer1", 0x0f00, N64.1), &OVR), 1),
    outer(with_pre(t("T44-ovr-F10-outer3", 0x0f00, N64.1), &OVR), 3),
    // CKE field: ACSMCkeStatus after a pair with one candidate bit.
    o("S00-cke-none", h3(0), Obs::Cke, &CKE),
    o("S01-cke-F7", h3(hw3::F7), Obs::Cke, &CKE),
    o("S02-cke-F8", h3(hw3::F8), Obs::Cke, &CKE),
    o("S03-cke-F9", h3(hw3::F9), Obs::Cke, &CKE),
    o("S04-cke-F10", h3(hw3::F10), Obs::Cke, &CKE),
    o("S05-cke-F11", h3(hw3::F11), Obs::Cke, &CKE),
    o("S06-cke-b3-0", h3(0x000f), Obs::Cke, &CKE),
    o(
        "S07-cke-odd-b9-8",
        [E, Row([0, 0, 0, 0x0300])],
        Obs::Cke,
        &CKE,
    ),
    // WCKTG: ACSMWckEnStatus before/after (each toggle flips it).
    o("W00-wck-none", [E, E], Obs::Wck, &WCKC),
    o(
        "W01-wck-hw2b0-even",
        [Row([0, 0, hw2::WCKTG, 0]), E],
        Obs::Wck,
        &WCKC,
    ),
    o(
        "W02-wck-hw2b0-odd",
        [E, Row([0, 0, hw2::WCKTG, 0])],
        Obs::Wck,
        &WCKC,
    ),
    o(
        "W03-wck-hw2b0-both",
        [Row([0, 0, 1, 0]), Row([0, 0, 1, 0])],
        Obs::Wck,
        &WCKC,
    ),
    o("W04-wck-hw2b1", [Row([0, 0, 2, 0]), E], Obs::Wck, &WCKC),
    o(
        "W05-wck-hw1b3",
        [Row([0, hw1::UNUSED, 0, 0]), E],
        Obs::Wck,
        &WCKC,
    ),
    // RX FIFO pointers: which hw1 bits make the PHY capture.
    o("X00-fifo-none", h1(0), Obs::Fifo, &FIFO_PRE),
    o("X01-fifo-b0", h1(hw1::RX_A), Obs::Fifo, &FIFO_PRE),
    o("X02-fifo-b1", h1(hw1::RX_B), Obs::Fifo, &FIFO_PRE),
    o("X03-fifo-b1b0", h1(hw1::RX), Obs::Fifo, &FIFO_PRE),
    o("X04-fifo-b2", h1(hw1::TX_EN), Obs::Fifo, &FIFO_PRE),
    o(
        "X05-fifo-rdFT",
        h1(pulses(0, wck::FAST_TOGGLE, 0, 0)),
        Obs::Fifo,
        &FIFO_PRE,
    ),
    o(
        "X06-fifo-rdFT-b1b0",
        h1(pulses(0, wck::FAST_TOGGLE, 0, hw1::RX)),
        Obs::Fifo,
        &FIFO_PRE,
    ),
];

/// Launches per timing case.
pub const REPS: usize = 3;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Sample {
    pub us: [u32; REPS],
    pub polls: u32,
    pub before: [u16; 2],
    pub after: [u16; 2],
    pub failure: Option<WlFailure>,
}

impl Sample {
    pub const EMPTY: Sample = Sample {
        us: [0; REPS],
        polls: 0,
        before: [0; 2],
        after: [0; 2],
        failure: None,
    };
}

fn observe<H: Hw>(ex: &mut WlExec<'_, H>, obs: Obs) -> [u16; 2] {
    match obs {
        Obs::Time => [0, 0],
        Obs::Cke => [ex.read(CKE_STATUS), ex.read(CKE_CONTROL)],
        Obs::Wck => [ex.read(WCK_EN_STATUS), ex.read(WCK_EN_CONTROL)],
        Obs::Fifo => [ex.read(fifo_info(0, 0)), ex.read(fifo_info(1, 0))],
    }
}

/// Run one case. `mem_reset`: the MemResetL value launches require (2: DRAM
/// held in reset).
pub fn run_case<H: Hw>(hw: &mut H, c: &Case, mem_reset: u16) -> Sample {
    let mut s = Sample::EMPTY;
    let mut ex = WlExec::new(hw);
    ex.mem_reset_want = mem_reset;
    let n = c.npre as usize;
    let mut saved = [0u16; 4];
    for (k, &(a, v)) in c.pre[..n].iter().enumerate() {
        saved[k] = ex.read(a);
        ex.write(a, v);
    }
    if c.obs == Obs::Fifo {
        fire(&mut ex, [0, 1], EV_FIFO);
    }
    s.before = observe(&mut ex, c.obs);
    let reps = if c.obs == Obs::Time { REPS } else { 1 };
    for r in 0..reps {
        ex.max_launch_us = 0;
        ex.step(Step::Launch {
            rows: &c.rows[..c.len as usize],
            outer: c.outer,
        });
        s.us[r] = ex.max_launch_us;
        s.polls = s.polls.max(ex.max_polls);
    }
    s.after = observe(&mut ex, c.obs);
    s.failure = ex.failure;
    for k in (0..n).rev() {
        ex.write_force(c.pre[k].0, saved[k]);
    }
    csr_access(hw, false);
    s
}

/// All [`CASES`] in order; `emit` gets each result. Run is cleared after
/// every launch, also a failed one, so the series goes on after a failure
/// (the loop cases may not end); three failed cases in a row stop it. The
/// WCKTG cases end with ACSMWckEnStatus put back by one more toggle if it
/// changed.
pub fn reset_probe<H: Hw>(hw: &mut H, mem_reset: u16, emit: &mut impl FnMut(&Case, &Sample)) {
    let mut wck0 = None;
    let mut failed = 0;
    for c in CASES.iter() {
        if c.obs == Obs::Wck && wck0.is_none() {
            csr_access(hw, true);
            wck0 = Some(hw.read16(crate::phy::csr(WCK_EN_STATUS)));
        }
        let s = run_case(hw, c, mem_reset);
        emit(c, &s);
        failed = if s.failure.is_some() { failed + 1 } else { 0 };
        if failed == 3 {
            return;
        }
    }
    if let Some(w0) = wck0 {
        csr_access(hw, true);
        if hw.read16(crate::phy::csr(WCK_EN_STATUS)) != w0 {
            let fix = o("W99-wck-restore", [Row([0, 0, 1, 0]), E], Obs::Wck, &WCKC);
            let s = run_case(hw, &fix, mem_reset);
            emit(&fix, &s);
        }
        csr_access(hw, false);
    }
}

// ---------------------------------------------------------------------------
// MRR field variants (hybrid, DRAM live).

/// One field change of the reference MRR (`rxen::mrr_program`, the DCA
/// stage's read: CAS WS_RD with the read preamble, MRR with both strobes,
/// wait REPEATCNT 14).
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Variant {
    /// Unchanged, MR5 (control).
    Ref,
    /// MR8 instead of MR5: the MA field (hw0[13:7]).
    Ma8,
    /// Rank 1 (CS 2, TG): CS/TG fields.
    Rank1,
    /// Rank 1 rows with TG cleared.
    Rank1NoTg,
    /// Rank 0 rows with TG set.
    Rank0Tg,
    /// MRR row strobes 0x1 / 0x2 / 0.
    RxA,
    RxB,
    RxNone,
    /// CAS row read pulses without StaticLo / Toggle / FastToggle / any.
    PreNoStaticLo,
    PreNoToggle,
    PreNoFastToggle,
    PreNone,
    /// Wait after the MRR: REPEATCNT 0 / 8 instead of 14.
    Wait0,
    Wait8,
}

impl Variant {
    pub const fn parse(s: &str) -> Option<Variant> {
        let b = s.as_bytes();
        const NAMES: [(&str, Variant); 14] = [
            ("ref", Variant::Ref),
            ("ma8", Variant::Ma8),
            ("r1", Variant::Rank1),
            ("r1-notg", Variant::Rank1NoTg),
            ("r0-tg", Variant::Rank0Tg),
            ("rx-a", Variant::RxA),
            ("rx-b", Variant::RxB),
            ("rx-none", Variant::RxNone),
            ("pre-nolo", Variant::PreNoStaticLo),
            ("pre-notg", Variant::PreNoToggle),
            ("pre-noft", Variant::PreNoFastToggle),
            ("pre-none", Variant::PreNone),
            ("wait0", Variant::Wait0),
            ("wait8", Variant::Wait8),
        ];
        let mut i = 0;
        while i < NAMES.len() {
            let n = NAMES[i].0.as_bytes();
            if n.len() == b.len() {
                let mut k = 0;
                while k < n.len() && n[k] == b[k] {
                    k += 1;
                }
                if k == n.len() {
                    return Some(NAMES[i].1);
                }
            }
            i += 1;
        }
        None
    }

    /// The rank the variant's MRR addresses and its MA.
    pub const fn target(self) -> (Rank, u8) {
        match self {
            Variant::Ma8 => (Rank::R0, 8),
            Variant::Rank1 | Variant::Rank1NoTg => (Rank::R1, 5),
            _ => (Rank::R0, 5),
        }
    }

    /// The variant's rows: the reference program with one field changed.
    pub const fn rows(self) -> [Row; 6] {
        let (rank, ma) = self.target();
        let mut p = rxen::mrr_program(rank, ma);
        match self {
            Variant::Rank1NoTg => {
                p[0].0[3] &= !hw3::TG;
                p[1].0[3] &= !hw3::TG;
            }
            Variant::Rank0Tg => {
                p[0].0[3] |= hw3::TG;
                p[1].0[3] |= hw3::TG;
            }
            Variant::RxA => p[1].0[1] = (p[1].0[1] & !hw1::RX) | hw1::RX_A,
            Variant::RxB => p[1].0[1] = (p[1].0[1] & !hw1::RX) | hw1::RX_B,
            Variant::RxNone => p[1].0[1] &= !hw1::RX,
            Variant::PreNoStaticLo => p[0].0[1] = pulses(0, wck::PREAMBLE & !wck::STATIC_LO, 0, 0),
            Variant::PreNoToggle => p[0].0[1] = pulses(0, wck::PREAMBLE & !wck::TOGGLE, 0, 0),
            Variant::PreNoFastToggle => {
                p[0].0[1] = pulses(0, wck::PREAMBLE & !wck::FAST_TOGGLE, 0, 0)
            }
            Variant::PreNone => p[0].0[1] = 0,
            Variant::Wait0 => {
                let w = wait_pair(0, hw3::WAIT);
                p[2] = w[0];
                p[3] = w[1];
            }
            Variant::Wait8 => {
                let w = wait_pair(8, hw3::WAIT);
                p[2] = w[0];
                p[3] = w[1];
            }
            _ => {}
        }
        p
    }
}

// The reference is the DCA read exactly; each variant differs from it in
// one field (the rank variants: from the rank-1 reference).
const _: () = {
    let r = Variant::Ref.rows();
    let p = rxen::mrr_program(Rank::R0, 5);
    let mut i = 0;
    while i < 6 {
        let mut k = 0;
        while k < 4 {
            assert!(r[i].0[k] == p[i].0[k]);
            k += 1;
        }
        i += 1;
    }
    assert!(Variant::RxA.rows()[1].0[1] == 0x0001);
    assert!(Variant::PreNoToggle.rows()[0].0[1] == 0x0900);
    assert!(Variant::Wait8.rows()[3].0[3] == 0x0400);
};

#[derive(Clone, Copy, Debug)]
pub struct FieldReport {
    pub variant: Variant,
    /// The variant's MRR, then MR5 and MR8 of rank 0 with the reference.
    pub mrr: [Mrr; 3],
    pub restored: bool,
    pub launches: u16,
    pub failure: Option<WlFailure>,
}

/// The field probe on channel A in the PHY's training mode (the hybrid's
/// state after the PMU). RxFifoVisibility `vis` as the DCA stage (0xfffc).
pub fn field_probe<H: Hw>(hw: &mut H, v: Variant, pstate: u32, vis: u16) -> FieldReport {
    let mut ex = WlExec::new(hw);
    let ch = Channel::A;
    let dbs = dbytes(ch);
    let cfg = dca::Config {
        pstate,
        ..dca::Config::DEFAULT
    };
    let saved: [u16; dca::SAVED.len()] = core::array::from_fn(|k| ex.read(dca::SAVED[k]));
    dca::select(&mut ex, ch, &cfg);
    let (_, ma) = v.target();
    let first = dca::mrr_rows(&mut ex, dbs, &v.rows(), ma, vis);
    let mr5 = dca::mrr(&mut ex, dbs, Rank::R0, 5, vis);
    let mr8 = dca::mrr(&mut ex, dbs, Rank::R0, 8, vis);
    let mut ok = true;
    for (k, &c) in dca::SAVED.iter().enumerate() {
        ex.write_force(c, saved[k]);
        ok &= ex.read(c) == saved[k];
    }
    ex.write_force(0x2f0a0, 1);
    ex.write_force(0x2f0a0, 0);
    let r = FieldReport {
        variant: v,
        mrr: [first, mr5, mr8],
        restored: ok,
        launches: ex.launches,
        failure: ex.failure,
    };
    csr_access(hw, false);
    r
}

#[cfg(test)]
mod tests {
    use super::*;

    fn diff_bits(a: &[Row], b: &[Row]) -> u32 {
        a.iter()
            .zip(b.iter())
            .map(|(x, y)| {
                x.0.iter()
                    .zip(y.0.iter())
                    .map(|(p, q)| (p ^ q).count_ones())
                    .sum::<u32>()
            })
            .sum()
    }

    /// Each flag case differs from the N64 reference in one bit (T17: the
    /// whole flag set, T24 likewise from T05).
    #[test]
    fn flag_cases_change_one_bit() {
        let r = pair(N64.0, N64.1);
        for c in CASES.iter().filter(|c| {
            ["T18", "T19", "T20", "T21", "T22", "T23"]
                .iter()
                .any(|p| c.id.starts_with(p))
        }) {
            assert_eq!(diff_bits(&c.rows, &r), 1, "{}", c.id);
        }
    }

    /// Each MRR variant differs from its reference in one field.
    #[test]
    fn variants_change_one_field() {
        let r0 = rxen::mrr_program(Rank::R0, 5);
        let r1 = rxen::mrr_program(Rank::R1, 5);
        for (v, reference, max_bits) in [
            (Variant::Ma8, r0, 3),
            (Variant::Rank1NoTg, r1, 2),
            (Variant::Rank0Tg, r0, 2),
            (Variant::RxA, r0, 1),
            (Variant::RxB, r0, 1),
            (Variant::RxNone, r0, 2),
            (Variant::PreNoStaticLo, r0, 1),
            (Variant::PreNoToggle, r0, 1),
            (Variant::PreNoFastToggle, r0, 1),
            (Variant::PreNone, r0, 3),
            (Variant::Wait0, r0, 3),
            (Variant::Wait8, r0, 2),
        ] {
            let d = diff_bits(&v.rows(), &reference);
            assert!(d >= 1 && d <= max_bits, "{v:?}: {d} bits");
        }
        assert_eq!(Variant::parse("pre-notg"), Some(Variant::PreNoToggle));
        assert_eq!(Variant::parse("nope"), None);
    }
}
