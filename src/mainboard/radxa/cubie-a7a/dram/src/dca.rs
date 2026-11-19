// SPDX-License-Identifier: GPL-2.0-only
//! WCK duty-cycle adjuster (DCA) training on the DRAM's duty-cycle monitor
//! (DCM), host-sequenced through the PHY's ACSM, and the PHY-side
//! duty-cycle/receiver CSR snapshot (FEL lab). Measurement stage: nothing
//! here runs unless
//! a build knob asks for it (bt0 `hybrid`, env `A7A_DCA`).
//!
//! The DRAM side (JEDEC LPDDR5; layouts in `jedec_lpddr::lp5::mr`):
//! - MR30 OP[7:4] / OP[3:0]: the DCA code of the upper / lower byte, -7..+7
//!   steps ([`jedec_lpddr::lp5::mr::dca_code`]);
//! - MR26 OP[0] DCM start (1) / stop (0), OP[1] flip, results OP[2] DCML0,
//!   OP[3] DCML1, OP[4] DCMU0, OP[5] DCMU1 (lower/upper byte, flip 0/1).
//!
//! The method (public): sweep the DCA code, run the monitor with flip 0
//! and flip 1 at each code, find where each flip's result changes, and set
//! the code at the mean of the two transitions, which cancels the
//! monitor's own offset. Renesas DBSC5 U-Boot
//! `drivers/ram/renesas/dbsc5/r8a779g0-dram.c` (GPL-2.0+)
//! `dbsc5_dramdca_training`: flip-0 and flip-1 results averaged on a
//! linear code scale (`mr30_conv`); the SiMa.ai U-Boot debug string table
//! `arch/arm/mach-sima/ddr_phy_debug_modalix.c` (GPL-2.0+ OR MIT): "DCA
//! result ... as read by sweeping MR30", "Flip0/Flip1 transition observed
//! @ dca", "averaging", "DCAUpper/DCALower calculated", "Setting MR30";
//! coreboot MT8195 `DramcDutyCycleMonitor` (MR26/MR30 layouts, BSD-3).
//! The decision code ([`shape`], [`decide`]) is ours.
//!
//! REFERENCE-DERIVED (the reference oracle's stage 0x06 at pstates 0-2;
//! flagged like the other stages' rows): the DCM program's rows and its wait rows per
//! pstate ([`dcm_program`]), the per-block CSR set-up (`0x200a3` pulse,
//! `0x2002a` = 1, `0x20041` = 1, `0x20007` = 0x100, DxRxStandbyEn around
//! the MRR), the MRR read-out bracket (RxFifoVisibility 0x300, then 0xfffc
//! per lane) and the stage's read latency per pstate ([`TRACE_MRL`], used
//! only with the `mrl` knob). Not used: any value the firmware chose (the
//! reference has no DQ path).

use crate::hw::Hw;
use crate::phy::coldinit::{mrw_ddr, Row};
use crate::phy::csr_access;
use crate::phy::rxen;
use crate::phy::wcklevel::{Channel, Rank, Step};
use crate::rd::{dbytes, fifo_info, fifo_vis, fire, EV_FIFO, EV_UPDATE};
use crate::rdtrain::RX_STANDBY;
use crate::wl::{WlExec, WlFailure};
use jedec_lpddr::lp5::mr::{dca_code, Mr16};

/// DCA steps swept, -7..=+7 (code 1000b is RFU and never used).
pub const STEPS: usize = 15;

/// The DCA step at sweep index `i`.
pub const fn step_of(i: usize) -> i8 {
    i as i8 - 7
}

/// MR30 with both bytes at `step`.
pub const fn mr30_both(step: i8) -> u8 {
    match dca_code(step) {
        Some(c) => c << 4 | c,
        None => panic!("DCA step out of range"),
    }
}

/// MR30 from an upper and a lower step.
pub const fn mr30(upper: i8, lower: i8) -> Option<u8> {
    match (dca_code(upper), dca_code(lower)) {
        (Some(u), Some(l)) => Some(u << 4 | l),
        _ => None,
    }
}

// The reference's sweep order is -7..+7 with both nibbles together: 0x77 ...
// 0x11, 0x00, 0x99 ... 0xff.
const _: () = assert!(mr30_both(-7) == 0x77 && mr30_both(-1) == 0x11);
const _: () = assert!(mr30_both(0) == 0 && mr30_both(1) == 0x99 && mr30_both(7) == 0xff);

/// MR26 values of the DCM program: start with flip, start, stop.
pub const MR26_START_FLIP: u8 = 0x03;
pub const MR26_START: u8 = 0x01;
pub const MR26_STOP: u8 = 0x00;

/// The DCM result of one byte (`upper`: DCMU, else DCML) for `flip`.
pub const fn dcm_bit(mr26: u8, upper: bool, flip: usize) -> bool {
    let b = if upper { 4 } else { 2 } + flip as u32;
    (mr26 >> b) & 1 != 0
}

/// MR16 with FSP-WR moved to the operating set (FSP-OP), all else kept:
/// the value an MRW stage that must reach the running WCK (MR30) writes
/// first. None for a value in CBT mode or with a reserved FSP code.
pub const fn mr16_write_operating(found: u8) -> Option<u8> {
    let m = Mr16::decode(found);
    if m.cbt != 0 || m.cbt_phase || m.fsp_op > 2 || m.fsp_wr > 2 {
        return None;
    }
    Some(
        Mr16 {
            fsp_wr: m.fsp_op,
            ..m
        }
        .encode(),
    )
}

const _: () = assert!(matches!(mr16_write_operating(0x04), Some(0x05)));
const _: () = assert!(matches!(mr16_write_operating(0x44), Some(0x45)));

// ---------------------------------------------------------------------------
// The decision (pure).

/// One DRAM byte's sweep: per flip and DCA step, the runs that read 1 and
/// the runs that read at all (several iterations add up).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ByteSweep {
    pub ones: [[u8; STEPS]; 2],
    pub reads: [[u8; STEPS]; 2],
}

impl ByteSweep {
    pub const EMPTY: ByteSweep = ByteSweep {
        ones: [[0; STEPS]; 2],
        reads: [[0; STEPS]; 2],
    };

    /// Record one result (None: the read failed).
    pub fn record(&mut self, flip: usize, i: usize, bit: Option<bool>) {
        if let Some(b) = bit {
            self.reads[flip][i] = self.reads[flip][i].saturating_add(1);
            self.ones[flip][i] = self.ones[flip][i].saturating_add(b as u8);
        }
    }

    /// The majority result per step (None: no read, or a tie).
    pub fn bits(&self, flip: usize) -> [Option<bool>; STEPS] {
        let mut out = [None; STEPS];
        for (i, o) in out.iter_mut().enumerate() {
            let (n, k) = (self.reads[flip][i], self.ones[flip][i]);
            if n != 0 && 2 * k != n {
                *o = Some(2 * k > n);
            }
        }
        out
    }
}

/// The result curve of one flip over the sweep.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FlipShape {
    /// One change of level. `x2`: the transition in half steps (2 x DCA
    /// step; between two codes, so odd), `rising`: 0 below, 1 above.
    Step { rising: bool, x2: i8 },
    /// More than one change (`changes`); `x2` from the counts (the
    /// position of the one-change curve with the same number of 0s / 1s),
    /// the direction from the halves.
    Noisy { rising: bool, x2: i8, changes: u8 },
    /// No change: the result stays at `level`; the transition lies outside
    /// the DCA range.
    Flat { level: bool },
    /// More than one change and the halves agree: no direction.
    Ambiguous { changes: u8 },
    /// A step without a valid read.
    Missing,
}

/// The shape of one flip's curve (`bits` per step, -7..+7).
pub fn shape(bits: &[Option<bool>; STEPS]) -> FlipShape {
    let mut b = [false; STEPS];
    for (x, y) in b.iter_mut().zip(bits.iter()) {
        match y {
            Some(v) => *x = *v,
            None => return FlipShape::Missing,
        }
    }
    let changes = b.windows(2).filter(|w| w[0] != w[1]).count() as u8;
    let ones = b.iter().filter(|&&x| x).count() as i8;
    if changes == 0 {
        return FlipShape::Flat { level: b[0] };
    }
    let lo = b[..7].iter().filter(|&&x| x).count();
    let hi = b[8..].iter().filter(|&&x| x).count();
    let rising = match (changes, lo.cmp(&hi)) {
        (1, _) => b[STEPS - 1],
        (_, core::cmp::Ordering::Less) => true,
        (_, core::cmp::Ordering::Greater) => false,
        (_, core::cmp::Ordering::Equal) => return FlipShape::Ambiguous { changes },
    };
    // Rising with z zeros: 0 at indices 0..z, so the change is between
    // z-1 and z: step z - 7.5. Falling: the same with the ones.
    let z = if rising { STEPS as i8 - ones } else { ones };
    let x2 = 2 * z - 15;
    if changes == 1 {
        FlipShape::Step { rising, x2 }
    } else {
        FlipShape::Noisy {
            rising,
            x2,
            changes,
        }
    }
}

/// How [`decide`] treats imperfect curves.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DcaPolicy {
    /// Most level changes a flip's curve may have and still count
    /// (1: only clean steps).
    pub max_changes: u8,
    /// With one flip flat (its transition outside the range), decide from
    /// the other flip alone (the monitor's offset is then not cancelled).
    pub single_flip: bool,
}

impl DcaPolicy {
    /// Clean or slightly noisy curves on both flips (an isolated wrong
    /// read gives 3 changes).
    pub const DEFAULT: DcaPolicy = DcaPolicy {
        max_changes: 3,
        single_flip: false,
    };
}

/// Why no step was chosen for a byte.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NoDecision {
    Missing,
    /// A flip's curve changes more than the policy allows, or has no
    /// direction.
    NonMonotonic {
        flip: u8,
    },
    /// Both flips flat: the duty error is outside the DCA range or the
    /// monitor does not respond.
    Flat,
    /// One flip flat and the policy wants both.
    OneFlat {
        flip: u8,
    },
}

/// One byte's result.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ByteDecision {
    pub shapes: [FlipShape; 2],
    /// The chosen step, or why none.
    pub step: Result<i8, NoDecision>,
    /// The two flips change in the same direction (information: with the
    /// monitor's flip inverting the comparison they are expected to be
    /// opposite; the mean is taken either way).
    pub same_direction: Option<bool>,
    /// Sum of the two transitions in half steps (the mean is this / 4).
    pub x4: Option<i16>,
}

/// `x4 / 4` rounded to the nearest step, ties toward 0, clamped to -7..7.
pub const fn round_quarter(x4: i16) -> i8 {
    let q = x4 / 4;
    let r = x4 % 4;
    let s = if r > 2 {
        q + 1
    } else if r < -2 {
        q - 1
    } else {
        q
    };
    if s < -7 {
        -7
    } else if s > 7 {
        7
    } else {
        s as i8
    }
}

/// The DCA step of one byte from its sweep (module documentation).
pub fn decide(sweep: &ByteSweep, policy: &DcaPolicy) -> ByteDecision {
    let shapes = [shape(&sweep.bits(0)), shape(&sweep.bits(1))];
    let mut x = [None; 2];
    let mut rising = [None; 2];
    let mut flat = [false; 2];
    for (f, s) in shapes.iter().enumerate() {
        match *s {
            FlipShape::Missing => {
                return ByteDecision {
                    shapes,
                    step: Err(NoDecision::Missing),
                    same_direction: None,
                    x4: None,
                }
            }
            FlipShape::Step { rising: r, x2 } => {
                x[f] = Some(x2);
                rising[f] = Some(r);
            }
            FlipShape::Noisy {
                rising: r,
                x2,
                changes,
            } if changes <= policy.max_changes => {
                x[f] = Some(x2);
                rising[f] = Some(r);
            }
            FlipShape::Noisy { .. } | FlipShape::Ambiguous { .. } => {
                return ByteDecision {
                    shapes,
                    step: Err(NoDecision::NonMonotonic { flip: f as u8 }),
                    same_direction: None,
                    x4: None,
                }
            }
            FlipShape::Flat { .. } => flat[f] = true,
        }
    }
    let same_direction = match rising {
        [Some(a), Some(b)] => Some(a == b),
        _ => None,
    };
    let (step, x4) = match (x[0], x[1]) {
        (Some(a), Some(b)) => {
            let x4 = a as i16 + b as i16;
            (Ok(round_quarter(x4)), Some(x4))
        }
        (Some(a), None) | (None, Some(a)) => {
            let f = if x[0].is_some() { 1 } else { 0 };
            if policy.single_flip {
                (Ok(round_quarter(2 * a as i16)), Some(2 * a as i16))
            } else {
                (Err(NoDecision::OneFlat { flip: f }), None)
            }
        }
        (None, None) => {
            debug_assert!(flat[0] && flat[1]);
            (Err(NoDecision::Flat), None)
        }
    };
    ByteDecision {
        shapes,
        step,
        same_direction,
        x4,
    }
}

/// The MR30 to leave: decided bytes take their step, the others keep the
/// found nibble.
pub const fn final_mr30(
    found: u8,
    upper: Result<i8, NoDecision>,
    lower: Result<i8, NoDecision>,
) -> u8 {
    let u = match upper {
        Ok(s) => match dca_code(s) {
            Some(c) => c,
            None => found >> 4,
        },
        Err(_) => found >> 4,
    };
    let l = match lower {
        Ok(s) => match dca_code(s) {
            Some(c) => c,
            None => found & 0xf,
        },
        Err(_) => found & 0xf,
    };
    u << 4 | l
}

// ---------------------------------------------------------------------------
// ACSM programs (REFERENCE-DERIVED rows, see the module documentation).

const fn cs_bits(rank: Rank) -> u16 {
    (rank.cs() as u16) << 14
}

/// First-edge CA of CAS with WS_FS (`0x4c`) and with all three WS bits
/// (`0x7c`), as `phy::rxen` derives them from `jedec_lpddr::lp5::cmd`.
const CA_CAS_FS: u16 = 0x4c;
const CA_CAS_ALL: u16 = 0x7c;

/// One MRW as the reference's four-row launch: the two command rows with the
/// rank's control word, two empty rows.
pub const fn mrw_launch(rank: Rank, ma: u8, op: u8) -> [Row; 4] {
    let e = mrw_ddr(rank.cs(), ma, op);
    [
        Row::raw(e[0].0[0], 0, 0, rank.control()),
        Row::raw(e[1].0[0], 0, 0, rank.control()),
        Row::EMPTY,
        Row::EMPTY,
    ]
}

const _: () = assert!(mrw_launch(Rank::R0, 30, 0x77)[0].0[0] == 0x4f58);
const _: () = assert!(mrw_launch(Rank::R0, 30, 0x77)[1].0[0] == 0x7b88);
const _: () = assert!(mrw_launch(Rank::R1, 30, 0x77)[0].0[0] == 0x8f58);
const _: () = assert!(mrw_launch(Rank::R1, 30, 0x77)[1].0[3] == 0x10);

/// The wait rows (control words) after the MR26 start writes, per pstate.
pub const fn dcm_waits(pstate: u32) -> &'static [u16] {
    match pstate {
        0 => &[0x7b00, 0x7c00, 0x7b00, 0x7c00, 0x2b00, 0x2c00],
        1 => &[0x7b00, 0x7c00, 0x5b00, 0x1400],
        _ => &[0x0b00, 0x6400],
    }
}

/// Rows of [`dcm_program`] at most.
pub const DCM_ROWS_MAX: usize = 32;

/// The DCM program: CAS (WS_FS, starts WCK), MR26 start with flip, wait,
/// MR26 start, wait, MR26 stop, CAS (WS_WR/RD/FS), closing rows; 28 rows
/// at pstate 1, 32 at pstate 0, 24 at pstates 2 and 3. Returns the rows
/// and how many are used.
pub const fn dcm_program(rank: Rank, pstate: u32) -> ([Row; DCM_ROWS_MAX], usize) {
    let mut rows = [Row::EMPTY; DCM_ROWS_MAX];
    let c = rank.control();
    rows[0] = Row::raw(cs_bits(rank) | CA_CAS_FS, 0xd000, 0x0001, c);
    rows[2] = Row::nop(0x4b00);
    let mut n = 4;
    let waits = dcm_waits(pstate);
    let ops = [MR26_START_FLIP, MR26_START];
    let mut k = 0;
    while k < 2 {
        let m = mrw_launch(rank, 26, ops[k]);
        rows[n] = m[0];
        rows[n + 1] = m[1];
        n += 2;
        let mut w = 0;
        while w < waits.len() {
            rows[n] = Row::nop(waits[w]);
            n += 1;
            w += 1;
        }
        k += 1;
    }
    let m = mrw_launch(rank, 26, MR26_STOP);
    rows[n] = m[0];
    rows[n + 1] = m[1];
    rows[n + 2] = Row::nop(0x0b00);
    rows[n + 3] = Row::nop(0x0800);
    rows[n + 4] = Row::raw(cs_bits(rank) | CA_CAS_ALL, 0, 0, c);
    rows[n + 8] = Row::raw(0, 0, 0x0001, 0);
    (rows, n + 12)
}

const _: () = assert!(dcm_program(Rank::R0, 1).1 == 28);
const _: () = assert!(dcm_program(Rank::R0, 0).1 == 32);
const _: () = assert!(dcm_program(Rank::R0, 2).1 == 24);
const _: () = assert!(dcm_program(Rank::R0, 1).0[0].0[0] == 0x404c);
const _: () = assert!(dcm_program(Rank::R0, 1).0[5].0[0] == 0x4188);
const _: () = assert!(dcm_program(Rank::R0, 1).0[20].0[0] == 0x407c);

/// The read latency (HwtMRL `0x2000d`, DBYTE DFIMRL `0x1n000`) the reference
/// has in place during the stage, per pstate 0..3. Written only with the
/// `mrl` knob; by default the stage runs at the live latency.
pub const TRACE_MRL: [u16; 4] = [0x82, 3, 5, 8];

/// RxFifoVisibility values of the MRR read-out: ObsSel 1 (FIFO pointers),
/// then the per-lane contents value. The reference writes 0xfffc;
/// `rdtrain::VIS_MRR` uses 0x3fc (its blobless MRRs read 0 on the board).
pub const VIS_INFO: u16 = 0x300;
pub const VIS_TRACE: u16 = 0xfffc;

// ---------------------------------------------------------------------------
// The stage on hardware.

/// What the stage does (env `A7A_DCA`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    /// MRRs only (MR5, MR8, MR16, MR24, MR26, MR30 at the found FSP-WR,
    /// then MR30/MR26/MR24/MR18/MR2 with FSP-WR at the operating set); the
    /// only writes to the DRAM are the two MR16 switches.
    Mrr,
    /// [`Mode::Mrr`] plus one DCM run at the found MR30 (MR26 writes only).
    Dcm,
    /// The sweep; MR30 put back as found.
    Measure,
    /// The sweep; the decided bytes' steps written to MR30.
    Apply,
}

impl Mode {
    /// `mrr`, `dcm`, `measure`, `apply`; None for anything else.
    pub const fn parse(s: &str) -> Option<Mode> {
        match s.as_bytes() {
            b"mrr" => Some(Mode::Mrr),
            b"dcm" => Some(Mode::Dcm),
            b"measure" => Some(Mode::Measure),
            b"apply" => Some(Mode::Apply),
            _ => None,
        }
    }
}

/// The stage's inputs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Config {
    pub mode: Mode,
    /// The trained pstate (wait rows; [`TRACE_MRL`] index).
    pub pstate: u32,
    /// MR16 as the DRAM has it now (the PMU leaves 0x04; our end of
    /// initialization 0x45, or 0x04/0x44 with `A7A_EI_FSPWR0`). MRR of
    /// MR16 is printed but never trusted for this.
    pub mr16_found: u8,
    /// RxFifoVisibility for the per-lane contents ([`VIS_TRACE`]).
    pub vis: u16,
    /// Write [`TRACE_MRL`] for the pstate during the stage.
    pub trace_mrl: bool,
    /// Sweeps per block (results add up per step).
    pub iterations: u8,
    pub policy: DcaPolicy,
    /// MR5 the MRR path must return before any MR30/MR26 write (CXMT 0x13).
    pub mr5_want: u8,
}

impl Config {
    pub const DEFAULT: Config = Config {
        mode: Mode::Mrr,
        pstate: 1,
        mr16_found: 0x04,
        vis: VIS_TRACE,
        trace_mrl: false,
        iterations: 1,
        policy: DcaPolicy::DEFAULT,
        mr5_want: 0x13,
    };
}

/// One MRR through the ACSM and the RX FIFO, both DBYTEs of the channel:
/// per DBYTE the FIFO pointers (lane 0..7), the per-lane contents and the
/// value decoded through Dq0..7LnSel (None unless every lane reads clearly
/// 0 or 1: low byte with at most 2 or at least 6 bits set).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Mrr {
    pub ma: u8,
    pub info: [[u16; 8]; 2],
    pub raw: [[u16; 8]; 2],
    pub value: [Option<u8>; 2],
}

impl Mrr {
    pub const EMPTY: Mrr = Mrr {
        ma: 0,
        info: [[0; 8]; 2],
        raw: [[0; 8]; 2],
        value: [None; 2],
    };
    /// The value, from DBYTE `mrr_db` of the block (0/1).
    pub const fn get(&self, mrr_db: usize) -> Option<u8> {
        self.value[mrr_db]
    }
}

/// Bit of a FIFO lane's contents: None when unclear.
pub const fn lane_bit(raw: u16) -> Option<bool> {
    let n = (raw & 0xff).count_ones();
    if n >= 6 {
        Some(true)
    } else if n <= 2 {
        Some(false)
    } else {
        None
    }
}

/// Decode a DBYTE's lanes through its lane selects (`lane_sel[dq]` = the
/// lane that carries DRAM DQ `dq`).
pub fn decode(raw: &[u16; 8], lane_sel: &[u16; 8]) -> Option<u8> {
    let mut v = 0u8;
    for (dq, &lane) in lane_sel.iter().enumerate() {
        let r = *raw.get(lane as usize & 0xf)?;
        if lane_bit(r)? {
            v |= 1 << dq;
        }
    }
    Some(v)
}

/// Why a block stopped early.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DcaStop {
    /// MR5 did not read as wanted on either DBYTE: the MRR path does not
    /// work here, so nothing was written.
    MrrPath { got: [Option<u8>; 2] },
    /// The found MR16 cannot be moved to the operating set.
    Mr16 { found: u8 },
    /// MR30 of the operating set did not read back: not restorable, so
    /// no sweep.
    Mr30Unread,
}

/// MRRs of [`Mode::Mrr`] at the found FSP-WR, and with FSP-WR at the
/// operating set.
pub const MRR_FOUND: [u8; 6] = [5, 8, 16, 24, 26, 30];
pub const MRR_OPERATING: [u8; 5] = [30, 26, 24, 18, 2];

/// One block's (channel, rank) run.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DcaBlock {
    pub channel: Channel,
    pub rank: Rank,
    /// The DBYTE of the block (0/1) whose MRR read MR5 right (the reference
    /// reads the second: DBYTE 1 / 3).
    pub mrr_db: Option<usize>,
    pub found: [Mrr; MRR_FOUND.len()],
    pub operating: [Mrr; MRR_OPERATING.len()],
    /// MR30 of the operating set before the sweep.
    pub mr30_found: Option<u8>,
    /// [`Mode::Dcm`]: MR26 after one DCM run at the found MR30.
    pub dcm_once: Option<u8>,
    /// Per DRAM byte: 0 upper (MR30 OP[7:4], DCMU), 1 lower (DCML).
    pub sweep: [ByteSweep; 2],
    pub decision: [Option<ByteDecision>; 2],
    /// Per sweep step, the last MR26 read (None: unread).
    pub mr26: [Option<u8>; STEPS],
    pub mr30_final: Option<u8>,
    pub stop: Option<DcaStop>,
    pub restored: bool,
    pub launches: u16,
    pub writes: u16,
    pub failure: Option<WlFailure>,
    pub elapsed_us: u32,
}

impl DcaBlock {
    pub const fn empty(channel: Channel, rank: Rank) -> DcaBlock {
        DcaBlock {
            channel,
            rank,
            mrr_db: None,
            found: [Mrr::EMPTY; MRR_FOUND.len()],
            operating: [Mrr::EMPTY; MRR_OPERATING.len()],
            mr30_found: None,
            dcm_once: None,
            sweep: [ByteSweep::EMPTY; 2],
            decision: [None; 2],
            mr26: [None; STEPS],
            mr30_final: None,
            stop: None,
            restored: false,
            launches: 0,
            writes: 0,
            failure: None,
            elapsed_us: 0,
        }
    }
}

/// Blocks in the reference's order.
pub const BLOCKS: [(Channel, Rank); 4] = [
    (Channel::A, Rank::R0),
    (Channel::B, Rank::R0),
    (Channel::A, Rank::R1),
    (Channel::B, Rank::R1),
];

/// CSRs the stage changes and puts back: channel selects, the read
/// latency group, ACSMWckEnControl, the free-running WCK, `0x20007`.
pub(crate) const SAVED: [u32; 13] = [
    0x20072, 0x20073, 0x2007e, 0x2007a, 0x2000d, 0x10000, 0x11000, 0x12000, 0x13000, 0x14000,
    0x2002a, 0x20041, 0x20007,
];

fn launch<H: Hw>(ex: &mut WlExec<'_, H>, rows: &[Row]) {
    ex.step(Step::Launch { rows, outer: 0 });
}

/// MRR `ma` (module documentation of [`Mrr`]).
pub(crate) fn mrr<H: Hw>(
    ex: &mut WlExec<'_, H>,
    dbs: [u32; 2],
    rank: Rank,
    ma: u8,
    vis: u16,
) -> Mrr {
    mrr_rows(ex, dbs, &rxen::mrr_program(rank, ma), ma, vis)
}

/// [`mrr`] with the launch's rows given (ACSM field probes,
/// `acsm_probe`): the same bracket and RX FIFO read-out around `rows`.
pub(crate) fn mrr_rows<H: Hw>(
    ex: &mut WlExec<'_, H>,
    dbs: [u32; 2],
    rows: &[Row],
    ma: u8,
    vis: u16,
) -> Mrr {
    let mut m = Mrr { ma, ..Mrr::EMPTY };
    let standby = ex.read(0x1000a + 0x1000 * dbs[0]);
    ex.write(RX_STANDBY, 0);
    fire(ex, dbs, EV_FIFO);
    launch(ex, rows);
    ex.write(RX_STANDBY, standby);
    for (i, &db) in dbs.iter().enumerate() {
        let vis_found = ex.read(fifo_vis(db));
        let mut sel = [0u16; 8];
        for (dq, s) in sel.iter_mut().enumerate() {
            *s = ex.read(0x10080 + 0x1000 * db + dq as u32) & 0xf;
        }
        ex.write(fifo_vis(db), VIS_INFO);
        for lane in 0..8u32 {
            m.info[i][lane as usize] = ex.read(fifo_info(db, lane));
        }
        for lane in 0..8u32 {
            ex.write(fifo_vis(db), vis);
            m.raw[i][lane as usize] = ex.read(0x10073 + 0x1000 * db + 0x100 * lane);
        }
        ex.write(fifo_vis(db), vis_found);
        m.value[i] = decode(&m.raw[i], &sel);
    }
    m
}

pub(crate) fn select<H: Hw>(ex: &mut WlExec<'_, H>, ch: Channel, cfg: &Config) {
    let (a, b) = ch.enables();
    let mask = match ch {
        Channel::A => 0xf0,
        Channel::B => 0x0f,
    };
    for (c, v) in [(0x20072, a), (0x20073, b), (0x2007e, 0), (0x2007a, mask)] {
        ex.write(c, v);
    }
    if cfg.trace_mrl {
        let mrl = TRACE_MRL[(cfg.pstate & 3) as usize];
        for c in [0x2000d, 0x10000, 0x11000, 0x12000, 0x13000, 0x14000] {
            ex.write(c, mrl);
        }
    }
    ex.write(0x2f0a0, 1);
    ex.write(0x2f0a0, 0);
    let dbs = dbytes(ch);
    fire(ex, dbs, EV_UPDATE);
    for (c, v) in [(0x2002a, 1), (0x20041, 1), (0x20007, 0x100)] {
        ex.write(c, v);
    }
}

/// Run the stage on one block (`out.channel`, `out.rank`). Call in the
/// PHY's training mode (PUBMODE 1, the hybrid's state after the PMU, or
/// the blobless flow after WCK2CK leveling), never in mission mode.
#[inline(never)]
pub fn run_block<H: Hw>(hw: &mut H, out: &mut DcaBlock, cfg: &Config) {
    let (ch, rank) = (out.channel, out.rank);
    *out = DcaBlock::empty(ch, rank);
    let t0 = hw.now_us();
    let mut ex = WlExec::new(hw);
    let dbs = dbytes(ch);
    let saved: [u16; SAVED.len()] = core::array::from_fn(|k| ex.read(SAVED[k]));
    select(&mut ex, ch, cfg);
    body(&mut ex, out, dbs, cfg);
    // Back: the CSRs as found, the FIFO pointers reset.
    let mut ok = true;
    for (k, &c) in SAVED.iter().enumerate() {
        ex.write_force(c, saved[k]);
        ok &= ex.read(c) == saved[k];
    }
    ex.write_force(0x2f0a0, 1);
    ex.write_force(0x2f0a0, 0);
    out.restored = ok;
    out.launches = ex.launches;
    out.writes = ex.writes;
    out.failure = ex.failure;
    csr_access(hw, false);
    out.elapsed_us = hw.now_us().wrapping_sub(t0) as u32;
}

fn body<H: Hw>(ex: &mut WlExec<'_, H>, out: &mut DcaBlock, dbs: [u32; 2], cfg: &Config) {
    let rank = out.rank;
    for (k, &ma) in MRR_FOUND.iter().enumerate() {
        out.found[k] = mrr(ex, dbs, rank, ma, cfg.vis);
    }
    let mr5 = out.found[0].value;
    // The reference reads the second DBYTE; take the first one that reads MR5.
    out.mrr_db = [1usize, 0]
        .into_iter()
        .find(|&i| mr5[i] == Some(cfg.mr5_want));
    let Some(mdb) = out.mrr_db else {
        out.stop = Some(DcaStop::MrrPath { got: mr5 });
        return;
    };
    let Some(mr16_op) = mr16_write_operating(cfg.mr16_found) else {
        out.stop = Some(DcaStop::Mr16 {
            found: cfg.mr16_found,
        });
        return;
    };
    launch(ex, &mrw_launch(rank, 16, mr16_op));
    for (k, &ma) in MRR_OPERATING.iter().enumerate() {
        out.operating[k] = mrr(ex, dbs, rank, ma, cfg.vis);
    }
    out.mr30_found = out.operating[0].get(mdb);
    match cfg.mode {
        Mode::Mrr => {}
        Mode::Dcm => {
            let (rows, n) = dcm_program(rank, cfg.pstate);
            launch(ex, &rows[..n]);
            out.dcm_once = mrr(ex, dbs, rank, 26, cfg.vis).get(mdb);
        }
        Mode::Measure | Mode::Apply => match out.mr30_found {
            None => out.stop = Some(DcaStop::Mr30Unread),
            Some(found) => {
                sweep(ex, out, dbs, cfg, mdb, found);
                let d = [
                    decide(&out.sweep[0], &cfg.policy),
                    decide(&out.sweep[1], &cfg.policy),
                ];
                out.decision = [Some(d[0]), Some(d[1])];
                let last = if cfg.mode == Mode::Apply {
                    final_mr30(found, d[0].step, d[1].step)
                } else {
                    found
                };
                launch(ex, &mrw_launch(rank, 30, last));
                out.mr30_final = Some(last);
            }
        },
    }
    // MR16 back: FSP-WR to the non-operating set as found.
    launch(ex, &mrw_launch(rank, 16, cfg.mr16_found));
}

fn sweep<H: Hw>(
    ex: &mut WlExec<'_, H>,
    out: &mut DcaBlock,
    dbs: [u32; 2],
    cfg: &Config,
    mdb: usize,
    found: u8,
) {
    let rank = out.rank;
    let (rows, n) = dcm_program(rank, cfg.pstate);
    for _ in 0..cfg.iterations.max(1) {
        for i in 0..STEPS {
            if ex.failure.is_some() {
                return;
            }
            launch(ex, &mrw_launch(rank, 30, mr30_both(step_of(i))));
            launch(ex, &rows[..n]);
            // The reference puts MR30 back before the read-out (its found value
            // there is 0): the MRR then runs at the trained duty.
            launch(ex, &mrw_launch(rank, 30, found));
            let v = mrr(ex, dbs, rank, 26, cfg.vis).get(mdb);
            out.mr26[i] = v;
            for flip in 0..2 {
                out.sweep[0].record(flip, i, v.map(|x| dcm_bit(x, true, flip)));
                out.sweep[1].record(flip, i, v.map(|x| dcm_bit(x, false, flip)));
            }
        }
    }
}

// ---------------------------------------------------------------------------
// PHY-side duty-cycle, receiver and reference CSRs (names: SiMa.ai U-Boot
// `init_ddr_modalix.c` CSR names, GPL-2.0+ OR MIT; the unnamed ones are the
// ones the PMU leaves changed): read only, for the before/after snapshot
// of a PMU training (open-path `A7A_OPEN_OPTS` bit 12).

/// Per DBYTE (DBYTE-level offsets): TxDcaMode, RxDfeModeCfg, 0x00c, 0x00d
/// (wck2dqi per rank?), 0x015, 0x016 (wck2dqo per rank?), RxVrefCtl,
/// RxClkCntl, RxClkCntl1, TxWCKDcaCtrlT/C Tg0/Tg1, RxWCKDcaCtrlT/C
/// Tg0/Tg1, 0x066, 0x067, 0x0d2, RxDiffSeDIFF0, RxDiffSeVrefDACDIFF0.
pub const SNAP_DBYTE: [u32; 22] = [
    0x007, 0x009, 0x00c, 0x00d, 0x015, 0x016, 0x037, 0x056, 0x057, 0x05e, 0x05f, 0x060, 0x061,
    0x062, 0x063, 0x064, 0x065, 0x066, 0x067, 0x0d2, 0x0e8, 0x0e9,
];
/// Per lane 0..8: VrefDAC0..3.
pub const SNAP_LANE: [u32; 4] = [0x04e, 0x04f, 0x050, 0x051];
/// Master: VrefDacRefCtl, the free-running WCK, ACSMWckEnControl, 0x20007.
pub const SNAP_MASTER: [u32; 4] = [0x20328, 0x20041, 0x2002a, 0x20007];

/// Read the snapshot CSRs and log them (`what`: "before"/"after").
pub fn log_snapshot<H: Hw>(hw: &mut H, pstate: u32, what: &str) {
    csr_access(hw, true);
    let m: [u16; SNAP_MASTER.len()] =
        core::array::from_fn(|k| hw.read16(crate::phy::csr(SNAP_MASTER[k])));
    hw.log(format_args!(
        "[dram] SNAP ps{pstate} {what} master {:05x?} = {m:04x?}\n",
        SNAP_MASTER
    ));
    for db in 0..4u32 {
        let d: [u16; SNAP_DBYTE.len()] = core::array::from_fn(|k| {
            hw.read16(crate::phy::csr(0x10000 + 0x1000 * db + SNAP_DBYTE[k]))
        });
        hw.log(format_args!(
            "[dram] SNAP ps{pstate} {what} db{db} {:03x?} = {d:04x?}\n",
            SNAP_DBYTE
        ));
        for &off in SNAP_LANE.iter() {
            let l: [u16; 9] = core::array::from_fn(|lane| {
                hw.read16(crate::phy::csr(
                    0x10000 + 0x1000 * db + 0x100 * lane as u32 + off,
                ))
            });
            hw.log(format_args!(
                "[dram] SNAP ps{pstate} {what} db{db} lanes {off:03x} = {l:02x?}\n"
            ));
        }
    }
    csr_access(hw, false);
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;

    /// The DCM model: the WCK duty error is `err` (in DCA steps; the
    /// adjuster at step s gives err + s), the monitor's offset `off`. Flip
    /// 0 reads 1 above -off, flip 1 inverts the comparison (reads 1 below
    /// +off) unless `same` (both rising).
    fn model(err: f32, off: f32, same: bool) -> ByteSweep {
        let mut s = ByteSweep::EMPTY;
        for i in 0..STEPS {
            let d = err + step_of(i) as f32;
            s.record(0, i, Some(d + off > 0.0));
            let f1 = if same { d - off > 0.0 } else { d - off < 0.0 };
            s.record(1, i, Some(f1));
        }
        s
    }

    #[test]
    fn centred_offsets_cancel() {
        for (err, off) in [(0.0, 0.0), (2.3, 0.0), (-3.1, 1.4), (1.2, -2.6), (4.4, 2.0)] {
            for same in [false, true] {
                let d = decide(&model(err, off, same), &DcaPolicy::DEFAULT);
                let want = (-err).round() as i8;
                let got = d.step.unwrap();
                assert!(
                    (got - want).abs() <= 1,
                    "err {err} off {off}: {got} vs {want}"
                );
                assert_eq!(d.same_direction, Some(same));
            }
        }
    }

    #[test]
    fn exact_steps() {
        // Transition of flip 0 between -3 and -2 (x2 = -5), flip 1 between
        // -2 and -1 (x2 = -3): mean -2.
        let mut s = ByteSweep::EMPTY;
        for i in 0..STEPS {
            let st = step_of(i);
            s.record(0, i, Some(st >= -2));
            s.record(1, i, Some(st < -1));
        }
        let d = decide(&s, &DcaPolicy::DEFAULT);
        assert_eq!(
            d.shapes[0],
            FlipShape::Step {
                rising: true,
                x2: -5
            }
        );
        assert_eq!(
            d.shapes[1],
            FlipShape::Step {
                rising: false,
                x2: -3
            }
        );
        assert_eq!(d.x4, Some(-8));
        assert_eq!(d.step, Ok(-2));
    }

    #[test]
    fn rounding_ties_toward_zero() {
        assert_eq!(round_quarter(2), 0);
        assert_eq!(round_quarter(-2), 0);
        assert_eq!(round_quarter(3), 1);
        assert_eq!(round_quarter(-6), -1);
        assert_eq!(round_quarter(-7), -2);
        assert_eq!(round_quarter(40), 7);
        assert_eq!(round_quarter(-40), -7);
    }

    #[test]
    fn no_convergence_flat() {
        // Duty error far outside the range: both flips read the same level
        // everywhere.
        let d = decide(&model(20.0, 0.0, false), &DcaPolicy::DEFAULT);
        assert_eq!(d.step, Err(NoDecision::Flat));
        let mut s = ByteSweep::EMPTY;
        for i in 0..STEPS {
            s.record(0, i, Some(true));
            s.record(1, i, Some(true));
        }
        let d = decide(&s, &DcaPolicy::DEFAULT);
        assert_eq!(d.step, Err(NoDecision::Flat));
        assert_eq!(d.shapes[0], FlipShape::Flat { level: true });
    }

    #[test]
    fn sweep_edge_one_flip_flat() {
        // err 6.5, offset 1: flip 0 changes at -7.5 (outside), flip 1 at
        // -5.5. From flip 1 alone the offset stays in: -5.5 rounds to -5
        // where both flips would give -6.5.
        let s = model(6.5, 1.0, false);
        let strict = decide(&s, &DcaPolicy::DEFAULT);
        assert!(matches!(strict.step, Err(NoDecision::OneFlat { flip: 0 })));
        let loose = decide(
            &s,
            &DcaPolicy {
                single_flip: true,
                ..DcaPolicy::DEFAULT
            },
        );
        assert_eq!(loose.step, Ok(-5));
    }

    #[test]
    fn monotonicity_breaks() {
        // One wrong read on flip 0: three changes, still decided from the
        // counts; four wrong reads alternating: refused.
        let mut s = model(0.6, 0.0, false);
        let flip_one = |s: &mut ByteSweep, f: usize, i: usize| {
            let v = s.ones[f][i] == 0;
            s.ones[f][i] = v as u8;
        };
        flip_one(&mut s, 0, 12);
        let d = decide(&s, &DcaPolicy::DEFAULT);
        assert!(matches!(d.shapes[0], FlipShape::Noisy { changes: 3, .. }));
        assert!(d.step.is_ok());
        for i in [1, 3, 11, 13] {
            flip_one(&mut s, 0, i);
        }
        let d = decide(&s, &DcaPolicy::DEFAULT);
        assert!(matches!(d.step, Err(NoDecision::NonMonotonic { flip: 0 })));
        let strict = decide(
            &model(0.6, 0.0, false),
            &DcaPolicy {
                max_changes: 1,
                ..DcaPolicy::DEFAULT
            },
        );
        assert!(strict.step.is_ok());
    }

    #[test]
    fn missing_reads_and_majority() {
        let mut s = model(1.0, 0.0, false);
        s.reads[1][4] = 0;
        s.ones[1][4] = 0;
        assert_eq!(
            decide(&s, &DcaPolicy::DEFAULT).step,
            Err(NoDecision::Missing)
        );
        // Three iterations, one wrong: the majority holds.
        let mut s = ByteSweep::EMPTY;
        for it in 0..3 {
            let m = model(-2.0, 0.5, false);
            for f in 0..2 {
                for i in 0..STEPS {
                    let mut b = m.ones[f][i] != 0;
                    if it == 0 && i == 7 {
                        b = !b;
                    }
                    s.record(f, i, Some(b));
                }
            }
        }
        assert_eq!(decide(&s, &DcaPolicy::DEFAULT).step, Ok(2));
        // A tie is a missing step.
        let mut t = ByteSweep::EMPTY;
        for f in 0..2 {
            for i in 0..STEPS {
                t.record(f, i, Some(true));
                t.record(f, i, Some(i % 2 == 0));
            }
        }
        assert_eq!(
            decide(&t, &DcaPolicy::DEFAULT).step,
            Err(NoDecision::Missing)
        );
    }

    #[test]
    fn final_value_keeps_undecided_nibble() {
        assert_eq!(final_mr30(0x00, Ok(-2), Ok(3)), 0x2b);
        assert_eq!(final_mr30(0x9a, Err(NoDecision::Flat), Ok(0)), 0x90);
        assert_eq!(final_mr30(0x9a, Ok(1), Err(NoDecision::Missing)), 0x9a);
        assert_eq!(mr30(-7, 7), Some(0x7f));
        assert_eq!(mr30(8, 0), None);
    }

    #[test]
    fn fsp_targeting() {
        // FSP-WR follows FSP-OP; VRCG kept; CBT and reserved codes refused.
        assert_eq!(mr16_write_operating(0x04), Some(0x05));
        assert_eq!(mr16_write_operating(0x44), Some(0x45));
        assert_eq!(mr16_write_operating(0x45), Some(0x45));
        assert_eq!(mr16_write_operating(0x40), Some(0x40));
        assert_eq!(mr16_write_operating(0x08), Some(0x0a));
        assert_eq!(mr16_write_operating(0x0c), None);
        assert_eq!(mr16_write_operating(0x14), None);
        assert_eq!(mr16_write_operating(0x84), None);
        for v in 0..=255u8 {
            if let Some(w) = mr16_write_operating(v) {
                let (a, b) = (Mr16::decode(v), Mr16::decode(w));
                assert_eq!(b.fsp_wr, a.fsp_op);
                assert_eq!(b.fsp_op, a.fsp_op);
                assert_eq!(b.vrcg, a.vrcg);
            }
        }
    }

    #[test]
    fn dcm_rows_match_the_trace_structure() {
        // Pstate 1, rank 0: rows 200..227 of the reference.
        let (r, n) = dcm_program(Rank::R0, 1);
        let want: [[u16; 4]; 28] = [
            [0x404c, 0xd000, 0x0001, 0],
            [0; 4],
            [0, 0, 0, 0x4b00],
            [0; 4],
            [0x4d58, 0, 0, 0],
            [0x4188, 0, 0, 0],
            [0, 0, 0, 0x7b00],
            [0, 0, 0, 0x7c00],
            [0, 0, 0, 0x5b00],
            [0, 0, 0, 0x1400],
            [0x4d58, 0, 0, 0],
            [0x4088, 0, 0, 0],
            [0, 0, 0, 0x7b00],
            [0, 0, 0, 0x7c00],
            [0, 0, 0, 0x5b00],
            [0, 0, 0, 0x1400],
            [0x4d58, 0, 0, 0],
            [0x4008, 0, 0, 0],
            [0, 0, 0, 0x0b00],
            [0, 0, 0, 0x0800],
            [0x407c, 0, 0, 0],
            [0; 4],
            [0; 4],
            [0; 4],
            [0, 0, 0x0001, 0],
            [0; 4],
            [0; 4],
            [0; 4],
        ];
        assert_eq!(n, 28);
        for (k, w) in want.iter().enumerate() {
            assert_eq!(r[k].0, *w, "row {k}");
        }
        // Rank 1 (reference): CS 2 and control 0x10 on command rows.
        let (r1, _) = dcm_program(Rank::R1, 1);
        assert_eq!(r1[0].0, [0x804c, 0xd000, 0x0001, 0x10]);
        assert_eq!(r1[4].0, [0x8d58, 0, 0, 0x10]);
        assert_eq!(r1[5].0, [0x8188, 0, 0, 0x10]);
        // Pstate 2: waits 0b00/6400 (reference of pstate 2).
        let (r2, n2) = dcm_program(Rank::R0, 2);
        assert_eq!(n2, 24);
        assert_eq!(r2[6].0[3], 0x0b00);
        assert_eq!(r2[7].0[3], 0x6400);
        assert_eq!(r2[16].0[0], 0x407c);
        // The MRR of MR26 the reference uses.
        let m = rxen::mrr_program(Rank::R0, 26);
        assert_eq!(m[0].0, [0x402c, 0x0d00, 0, 0]);
        assert_eq!(m[1].0, [0x4d18, 0x0003, 0, 0]);
    }

    #[test]
    fn dcm_bits_and_decode() {
        // MR26 0x7c (read on the board with FSP-WR 0): all four results 1.
        for f in 0..2 {
            assert!(dcm_bit(0x7c, true, f) && dcm_bit(0x7c, false, f));
        }
        assert!(dcm_bit(0x04, false, 0) && !dcm_bit(0x04, false, 1));
        assert!(dcm_bit(0x20, true, 1) && !dcm_bit(0x20, true, 0));
        // Decode through the board's DBYTE 1 lane map (a board dump).
        let sel = [3, 2, 1, 0, 5, 7, 6, 4];
        let mut raw = [0u16; 8];
        for dq in 0..8 {
            if 0x13u8 >> dq & 1 != 0 {
                raw[sel[dq] as usize] = 0xff;
            }
        }
        assert_eq!(decode(&raw, &sel), Some(0x13));
        raw[0] = 0x0f;
        assert_eq!(decode(&raw, &sel), None);
        assert_eq!(lane_bit(0xfffe), Some(true));
        assert_eq!(lane_bit(0x0100), Some(false));
    }
}

#[cfg(test)]
mod sim {
    //! The stage against a simulated PHY + DRAM: ACSM launches decoded row
    //! by row (MRW/MRR, CS -> rank, the channel from the HwtLpCsEn
    //! selects), per-FSP MR30 in the DRAM (MRW to FSP-WR, the monitor
    //! sees FSP-OP, MRR reads FSP-WR), a duty-cycle monitor with an offset,
    //! and the RX FIFO contents through the board's lane maps.
    extern crate std;
    use super::*;
    use crate::phy::{MICRO_RESET, PHY_BASE};
    use jedec_lpddr::lp5::mr::dca_step;
    use std::collections::HashMap;
    use std::vec::Vec;

    struct Dram {
        mr16: u8,
        /// MR30 per FSP 0..2.
        mr30: [u8; 3],
        /// WCK duty error per byte (upper, lower) in DCA steps.
        err: [f32; 2],
        mr26_res: u8,
    }

    struct Sim {
        regs: HashMap<u32, u16>,
        t: u64,
        /// [channel][rank]
        dram: [[Dram; 2]; 2],
        off: f32,
        mr5: u8,
        mrws: Vec<(usize, usize, u8, u8)>,
    }

    const LANE_SEL: [[u16; 8]; 4] = [
        [4, 3, 5, 7, 0, 1, 6, 2],
        [3, 2, 1, 0, 5, 7, 6, 4],
        [1, 4, 5, 2, 7, 6, 0, 3],
        [6, 7, 3, 0, 4, 5, 1, 2],
    ];

    impl Sim {
        fn new(err: [[[f32; 2]; 2]; 2], mr30_op: u8) -> Sim {
            let d = |e: [f32; 2]| Dram {
                mr16: 0x04,
                mr30: [0, mr30_op, 0],
                err: e,
                mr26_res: 0x40,
            };
            let mut regs = HashMap::new();
            for db in 0..4u32 {
                for dq in 0..8u32 {
                    regs.insert(
                        0x10080 + 0x1000 * db + dq,
                        LANE_SEL[db as usize][dq as usize],
                    );
                }
                regs.insert(0x1000a + 0x1000 * db, 1);
            }
            regs.insert(0x2000d, 4);
            regs.insert(0x20060, 3);
            Sim {
                regs,
                t: 0,
                dram: [[d(err[0][0]), d(err[0][1])], [d(err[1][0]), d(err[1][1])]],
                off: 1.3,
                mr5: 0x13,
                mrws: Vec::new(),
            }
        }
        fn get(&self, a: u32) -> u16 {
            *self.regs.get(&a).unwrap_or(&0)
        }
        fn channel(&self) -> usize {
            if self.get(0x20072) == 3 {
                0
            } else {
                1
            }
        }
        fn run(&mut self) {
            let (start, stop) = (self.get(0x20022), self.get(0x20023));
            let ch = self.channel();
            let mut pend: Option<(usize, u8)> = None;
            for r in start..=stop {
                let w = self.get(0x41000 + 4 * r as u32);
                if w == 0 {
                    continue;
                }
                let cs = (w >> 14) as usize;
                let (e1, e2) = ((w & 0x7f) as u8, ((w >> 7) & 0x7f) as u8);
                let rank = if cs == 2 { 1 } else { 0 };
                if e1 == 0x58 {
                    pend = Some((rank, e2));
                } else if e1 & 0x3f == 0x08 && pend.is_some() {
                    let (rk, ma) = pend.take().unwrap();
                    self.mrw(ch, rk, ma, (e1 >> 6) << 7 | e2);
                } else if e1 == 0x18 {
                    self.mrr(ch, rank, e2);
                }
            }
            self.regs.insert(0x20024, stop);
        }
        fn mrw(&mut self, ch: usize, rk: usize, ma: u8, op: u8) {
            self.mrws.push((ch, rk, ma, op));
            let off = self.off;
            let d = &mut self.dram[ch][rk];
            let m = Mr16::decode(d.mr16);
            match ma {
                16 => d.mr16 = op,
                30 => d.mr30[m.fsp_wr as usize] = op,
                26 if op & 1 != 0 => {
                    let flip = (op >> 1 & 1) as usize;
                    let code = d.mr30[m.fsp_op as usize];
                    for (k, upper) in [(0, true), (1, false)] {
                        let nib = if upper { code >> 4 } else { code & 0xf };
                        let x = d.err[k] + dca_step(nib).unwrap() as f32;
                        let b = if flip == 0 {
                            x + off > 0.0
                        } else {
                            x - off < 0.0
                        };
                        let bit = (if upper { 4 } else { 2 }) + flip;
                        d.mr26_res = (d.mr26_res & !(1 << bit)) | (b as u8) << bit;
                    }
                }
                _ => {}
            }
        }
        fn mrr(&mut self, ch: usize, rk: usize, ma: u8) {
            let d = &self.dram[ch][rk];
            let v = match ma {
                5 => self.mr5,
                16 => d.mr16,
                26 => d.mr26_res,
                30 => d.mr30[Mr16::decode(d.mr16).fsp_wr as usize],
                _ => 0,
            };
            // The lower byte's DQ on the channel's second DBYTE.
            for (i, db) in [2 * ch as u32, 2 * ch as u32 + 1].into_iter().enumerate() {
                for dq in 0..8 {
                    let lane = LANE_SEL[db as usize][dq] as u32;
                    let bit = i == 1 && v >> dq & 1 != 0;
                    self.regs.insert(
                        0x10073 + 0x1000 * db + 0x100 * lane,
                        if bit { 0xff } else { 0 },
                    );
                }
            }
        }
    }

    impl crate::hw::RegIo for Sim {
        fn read32(&mut self, _: usize) -> u32 {
            0
        }
        fn write32(&mut self, _: usize, _: u32) {}
        fn read16(&mut self, addr: usize) -> u16 {
            self.t += 1;
            if addr == MICRO_RESET {
                return 1;
            }
            let a = ((addr - PHY_BASE) / 2) as u32;
            if a == 0x20021 {
                return 1;
            }
            self.get(a)
        }
        fn write16(&mut self, addr: usize, v: u16) {
            let a = ((addr - PHY_BASE) / 2) as u32;
            if a == 0x20020 && v & 1 != 0 && self.get(0x20020) & 1 == 0 {
                self.regs.insert(a, v);
                self.run();
                return;
            }
            self.regs.insert(a, v);
        }
        fn udelay(&mut self, us: u32) {
            self.t += us as u64;
        }
        fn now_us(&mut self) -> u64 {
            self.t += 1;
            self.t
        }
        fn log(&mut self, _: core::fmt::Arguments) {}
    }
    impl Hw for Sim {
        fn set_voltage(&mut self, _: crate::hw::Rail, _: u32) -> bool {
            true
        }
    }

    const ERR: [[[f32; 2]; 2]; 2] = [[[2.2, -1.1], [0.4, 3.6]], [[-2.7, 0.0], [5.2, -4.4]]];

    fn run_all(sim: &mut Sim, cfg: &Config) -> [DcaBlock; 4] {
        let mut out = [DcaBlock::empty(Channel::A, Rank::R0); 4];
        for (o, &(ch, rk)) in out.iter_mut().zip(BLOCKS.iter()) {
            o.channel = ch;
            o.rank = rk;
            run_block(sim, o, cfg);
        }
        out
    }

    #[test]
    fn apply_trains_the_operating_fsp_and_restores_mr16() {
        let mut sim = Sim::new(ERR, 0x00);
        let saved: Vec<u16> = SAVED.iter().map(|&c| sim.get(c)).collect();
        let cfg = Config {
            mode: Mode::Apply,
            ..Config::DEFAULT
        };
        let out = run_all(&mut sim, &cfg);
        for (o, &(ch, rk)) in out.iter().zip(BLOCKS.iter()) {
            let r = (rk == Rank::R1) as usize;
            assert_eq!(o.stop, None);
            assert_eq!(o.failure, None);
            assert!(o.restored);
            assert_eq!(o.mrr_db, Some(1));
            assert_eq!(o.mr30_found, Some(0));
            let ch_i = (ch == Channel::B) as usize;
            let d = &sim.dram[ch_i][r];
            let want = mr30((-d.err[0]).round() as i8, (-d.err[1]).round() as i8).unwrap();
            let got = o.mr30_final.unwrap();
            for (g, w) in [(got >> 4, want >> 4), (got & 0xf, want & 0xf)] {
                let (g, w) = (dca_step(g).unwrap(), dca_step(w).unwrap());
                assert!((g - w).abs() <= 1, "{ch:?} {rk:?}: {got:02x} vs {want:02x}");
            }
            // In the operating set (FSP 1); FSP 0 untouched; MR16 back.
            assert_eq!(d.mr30[1], got);
            assert_eq!(d.mr30[0], 0);
            assert_eq!(d.mr16, 0x04);
        }
        let now: Vec<u16> = SAVED.iter().map(|&c| sim.get(c)).collect();
        assert_eq!(saved, now);
        // Every MR30 write of a block came after its MR16 switch to 0x05
        // and before the switch back.
        let mut wr_op = [[false; 2]; 2];
        for &(ch, rk, ma, op) in &sim.mrws {
            match ma {
                16 => wr_op[ch][rk] = op == 0x05,
                30 => assert!(wr_op[ch][rk], "MR30 outside the operating-set window"),
                _ => {}
            }
        }
    }

    #[test]
    fn measure_puts_the_trained_value_back() {
        let mut sim = Sim::new(ERR, 0x9a);
        let cfg = Config {
            mode: Mode::Measure,
            ..Config::DEFAULT
        };
        let out = run_all(&mut sim, &cfg);
        for o in &out {
            assert_eq!(o.mr30_found, Some(0x9a));
            assert_eq!(o.mr30_final, Some(0x9a));
            assert!(o.decision.iter().all(|d| d.is_some()));
        }
        for ch in &sim.dram {
            for d in ch {
                assert_eq!(d.mr30, [0, 0x9a, 0]);
                assert_eq!(d.mr16, 0x04);
            }
        }
    }

    #[test]
    fn broken_mrr_path_writes_nothing() {
        let mut sim = Sim::new(ERR, 0);
        sim.mr5 = 0x00;
        let cfg = Config {
            mode: Mode::Apply,
            ..Config::DEFAULT
        };
        let out = run_all(&mut sim, &cfg);
        for o in &out {
            assert!(matches!(o.stop, Some(DcaStop::MrrPath { .. })));
            assert!(o.restored);
        }
        assert!(sim.mrws.is_empty());
    }

    #[test]
    fn cbt_mr16_refused_before_any_write() {
        let mut sim = Sim::new(ERR, 0);
        let cfg = Config {
            mode: Mode::Apply,
            mr16_found: 0x14,
            ..Config::DEFAULT
        };
        let out = run_all(&mut sim, &cfg);
        assert!(out
            .iter()
            .all(|o| o.stop == Some(DcaStop::Mr16 { found: 0x14 })));
        assert!(sim.mrws.is_empty());
    }

    #[test]
    fn mrr_mode_only_switches_mr16() {
        let mut sim = Sim::new(ERR, 0x9a);
        let out = run_all(&mut sim, &Config::DEFAULT);
        assert!(sim.mrws.iter().all(|m| m.2 == 16));
        assert_eq!(sim.mrws.len(), 8);
        for o in &out {
            assert_eq!(o.found[0].get(1), Some(0x13));
            assert_eq!(o.found[5].get(1), Some(0x00), "MR30 of FSP 0");
            assert_eq!(o.operating[0].get(1), Some(0x9a), "MR30 of FSP 1");
        }
    }

    #[test]
    fn duty_beyond_range_keeps_found() {
        let mut err = ERR;
        err[0][0] = [12.0, -0.4];
        let mut sim = Sim::new(err, 0x00);
        let cfg = Config {
            mode: Mode::Apply,
            ..Config::DEFAULT
        };
        let out = run_all(&mut sim, &cfg);
        let a0 = &out[0];
        assert_eq!(a0.decision[0].unwrap().step, Err(NoDecision::Flat));
        assert_eq!(a0.decision[1].unwrap().step, Ok(0));
        assert_eq!(a0.mr30_final, Some(0x00));
    }
}
