//! LPDDR5 cold initialization through the PHY's ACSM, without the PMU.
//!
//! Original encoder and step generator. The row format and the launch
//! structure were established from the PMU firmware's DevInit as recorded
//! by the PMU emulator (a reference oracle, `tools/pmu-trace --native`, Cubie
//! A7A PhyConfig 0x3a2) and are checked against that decoded record in
//! `tools/native-dram-controls/tests/coldinit_oracle.rs`. No firmware code or
//! bytes are used here.
//!
//! ACSM memory row (four halfwords at CSR 0x41000 + 4 * row):
//! - hw0: `CS[15:14] | CA[13:7] | CA[6:0]`. For these commands both 7-bit
//!   fields carry the same CA value; an LPDDR5 command's two CA edges take two
//!   rows, with CS asserted on the first only.
//! - hw3: control. 0x0010 on rows that assert CS bit 1 (rank 1); delay rows
//!   carry `0x?b00`, `0x0400`, `0x0800` or `0x4800` (field meaning not
//!   established: they are reproduced as the oracle's timing rows).
//!
//! This covers part 1 of DevInit at the boot pstate: after the DRAM reset is
//! released, CS initialization, a two-row command with rising code 0x70 and
//! operand 6 (multi-purpose command class; purpose not verified) and MRW of
//! MR16/MR1/MR28, then MR16/MR41, per channel and rank. WCK2CK sync, MR18 and
//! later steps follow in DevInit and are not generated yet. The mode register
//! values are inputs ([`Mrs`]).

/// One ACSM memory row.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct Row(pub [u16; 4]);

impl Row {
    pub const EMPTY: Row = Row([0; 4]);

    /// Delay/timing row: no command, CS idle.
    pub const fn nop(ctl: u16) -> Row {
        Row([0, 0, 0, ctl])
    }

    /// Command edge row: `ca` (7 bits) in both CA fields, chip selects `cs`
    /// (bit 0 rank 0, bit 1 rank 1).
    pub const fn cmd(cs: u8, ca: u8) -> Row {
        let ca = (ca & 0x7f) as u16;
        let ctl = if cs & 2 != 0 { 0x0010 } else { 0 };
        Row([((cs as u16 & 3) << 14) | (ca << 7) | ca, 0, 0, ctl])
    }
}

/// LPDDR5 MRW as four rows: MRW-1 code, MA, MRW-2 code with OP[7], OP[6:0].
pub const fn mrw(cs: u8, ma: u8, op: u8) -> [Row; 4] {
    [
        Row::cmd(cs, 0x58),
        Row::cmd(0, ma),
        Row::cmd(cs, 0x08 | ((op >> 7) << 6)),
        Row::cmd(0, op & 0x7f),
    ]
}

/// Mode register values written by part 1, in order.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Mrs {
    pub mr16_first: u8,
    pub mr1: u8,
    pub mr28: u8,
    pub mr16_second: u8,
    pub mr41: u8,
}

impl Mrs {
    /// The values the reference oracle wrote at pstate 1 for the CXMT 4 GB
    /// set. Their derivation from the parameter set is not implemented yet.
    pub const ORACLE_PSTATE1_CXMT_4G: Mrs = Mrs {
        mr16_first: 0x40,
        mr1: 0x80,
        mr28: 0x04,
        mr16_second: 0x41,
        mr41: 0x00,
    };
}

/// One step of the program: a PHY CSR write, or an ACSM launch of `rows`
/// loaded from row `start` (stop = start + len - 1, outer loop 0).
pub enum Step<'a> {
    Write(u32, u16),
    Launch { start: u16, rows: &'a [Row] },
}

const HW_CS_EN_A: u32 = 0x20072;
const HW_CS_EN_B: u32 = 0x20073;
const HWT_CONTROL_OVR: u32 = 0x2007a;
const HWT_CONTROL_VAL: u32 = 0x2007e;
const HWT_MRL: u32 = 0x2000d;
const LOW_SPEED_CLOCK_EN: u32 = 0x20042;
const LOW_SPEED_CLOCK_DELAY: u32 = 0x20044;
/// Written with 1 then 0 after the per-group setup (purpose not established).
const GROUP_LATCH: u32 = 0x2f0a0;
/// Per-DBYTE (0x1n000, n = 0..3) and 0x14000, written 8 with HwtMRL.
const GROUP_SETUP: [u32; 5] = [0x10000, 0x11000, 0x12000, 0x13000, 0x14000];
/// DBYTE broadcast writes between the two MRW rounds, as in the oracle.
const FSP_PREPARE: [(u32, u16); 7] = [
    (0x1f08a, 0x7ff),
    (0x1f089, 0x7ff),
    (0x1f08b, 0),
    (0x1f00a, 0),
    (0x1f005, 1),
    (0x1f094, 0),
    (0x1f00b, 2),
];

/// Which channels a group addresses: HwtLpCsEnA/B (both ranks) and the
/// HwtControlOvr mask that goes with them.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Channels {
    A,
    B,
    Both,
}

impl Channels {
    const fn mask(self) -> u16 {
        match self {
            Channels::A => 0xf0,
            Channels::B => 0x0f,
            Channels::Both => 0x00,
        }
    }
    const fn enables(self) -> (u16, u16) {
        match self {
            Channels::A => (3, 0),
            Channels::B => (0, 3),
            Channels::Both => (3, 3),
        }
    }
}

const HEAD: [Row; 2] = [Row::nop(0x4b00), Row::EMPTY];
const TAIL: [Row; 6] = [
    Row::nop(0x7b00),
    Row::nop(0x0400),
    Row::EMPTY,
    Row::EMPTY,
    Row::nop(0x1b00),
    Row::nop(0x0400),
];
const AFTER_FIRST_MRW: [Row; 4] = [
    Row::nop(0x7b00),
    Row::nop(0x0800),
    Row::nop(0x5b00),
    Row::EMPTY,
];
const AFTER_MRW: [Row; 2] = [Row::nop(0x5b00), Row::EMPTY];
const AFTER_CMD: [Row; 2] = [Row::nop(0x7b00), Row::nop(0x0400)];
const SETTLE: [Row; 10] = [
    Row::nop(0x7b00),
    Row::EMPTY,
    Row::EMPTY,
    Row::EMPTY,
    Row::nop(0x1b00),
    Row::nop(0x0400),
    Row::EMPTY,
    Row::EMPTY,
    Row::nop(0x5b00),
    Row::nop(0x4800),
];

/// Fixed-capacity row buffer for one launch.
struct Rows {
    buf: [Row; 32],
    len: usize,
}

impl Rows {
    const fn new() -> Self {
        Rows {
            buf: [Row::EMPTY; 32],
            len: 0,
        }
    }
    fn push(&mut self, rows: &[Row]) {
        for &r in rows {
            self.buf[self.len] = r;
            self.len += 1;
        }
    }
    fn rows(&self) -> &[Row] {
        &self.buf[..self.len]
    }
}

fn ls_on(mask: u16, emit: &mut impl FnMut(Step)) {
    for (csr, v) in [
        (HWT_CONTROL_VAL, 0),
        (HWT_CONTROL_OVR, 0xff),
        (LOW_SPEED_CLOCK_EN, 0x103),
        (LOW_SPEED_CLOCK_DELAY, 1),
        (HWT_CONTROL_OVR, mask),
        (HWT_CONTROL_VAL, 0),
    ] {
        emit(Step::Write(csr, v));
    }
}

fn ls_off(mask: u16, emit: &mut impl FnMut(Step)) {
    for (csr, v) in [
        (HWT_CONTROL_VAL, 0),
        (HWT_CONTROL_OVR, mask | 0xcc),
        (LOW_SPEED_CLOCK_EN, 0),
        (LOW_SPEED_CLOCK_DELAY, 0),
        (HWT_CONTROL_OVR, mask),
        (HWT_CONTROL_VAL, 0),
    ] {
        emit(Step::Write(csr, v));
    }
}

/// Select `ch`; `both_extra` is the HwtControlOvr value written before the
/// final 0 when selecting both channels (the oracle differs per occurrence).
fn select(ch: Channels, both_extra: u16, emit: &mut impl FnMut(Step)) {
    let (a, b) = ch.enables();
    emit(Step::Write(HW_CS_EN_A, a));
    emit(Step::Write(HW_CS_EN_B, b));
    emit(Step::Write(HWT_CONTROL_VAL, 0));
    if ch == Channels::Both {
        emit(Step::Write(HWT_CONTROL_OVR, both_extra));
    }
    emit(Step::Write(HWT_CONTROL_OVR, ch.mask()));
    emit(Step::Write(HWT_MRL, 8));
    for csr in GROUP_SETUP {
        emit(Step::Write(csr, 8));
    }
    emit(Step::Write(GROUP_LATCH, 1));
    emit(Step::Write(GROUP_LATCH, 0));
}

fn launch_cmd70(cs: u8, emit: &mut impl FnMut(Step)) {
    let mut r = Rows::new();
    r.push(&HEAD);
    r.push(&[Row::cmd(cs, 0x70), Row::cmd(0, 6)]);
    r.push(&AFTER_CMD);
    r.push(&TAIL);
    emit(Step::Launch {
        start: 0,
        rows: r.rows(),
    });
}

fn launch_mrws(cs: u8, mrs: &[(u8, u8)], emit: &mut impl FnMut(Step)) {
    let mut r = Rows::new();
    r.push(&HEAD);
    for (i, &(ma, op)) in mrs.iter().enumerate() {
        r.push(&mrw(cs, ma, op));
        r.push(if i == 0 { &AFTER_FIRST_MRW } else { &AFTER_MRW });
    }
    r.push(&TAIL);
    emit(Step::Launch {
        start: 0,
        rows: r.rows(),
    });
}

/// Part 1 of the cold init, from just after MemResetL is released: the
/// steps in order. `emit` performs (or records) each one.
pub fn part1(m: &Mrs, emit: &mut impl FnMut(Step)) {
    // CS initialization on both ranks.
    let mut r = Rows::new();
    r.push(&[Row::cmd(3, 0), Row::cmd(3, 0), Row::cmd(3, 0), Row::EMPTY]);
    emit(Step::Launch {
        start: 0,
        rows: r.rows(),
    });
    // Settling delay with the reduced-speed AC clock.
    ls_on(0, emit);
    let mut r = Rows::new();
    r.push(&SETTLE);
    r.push(&TAIL);
    emit(Step::Launch {
        start: 0,
        rows: r.rows(),
    });
    // Command 0x70/6 on both channels and ranks.
    ls_off(0, emit);
    select(Channels::Both, 0, emit);
    ls_on(0, emit);
    launch_cmd70(3, emit);
    // First round: per rank, per channel: MR16, MR1, MR28, then 0x70/6.
    let round1 = [(16, m.mr16_first), (1, m.mr1), (28, m.mr28)];
    let mut prev = None;
    for cs in [1u8, 2] {
        for ch in [Channels::A, Channels::B] {
            if let Some(p) = prev {
                ls_off(Channels::mask(p), emit);
            }
            select(ch, 0, emit);
            ls_on(ch.mask(), emit);
            launch_mrws(cs, &round1, emit);
            launch_cmd70(cs, emit);
            prev = Some(ch);
        }
    }
    // Second round: MR16 again, MR41.
    let round2 = [(16, m.mr16_second), (41, m.mr41)];
    ls_off(Channels::B.mask(), emit);
    select(Channels::Both, 0x0e, emit);
    for (csr, v) in FSP_PREPARE {
        emit(Step::Write(csr, v));
    }
    let mut first = true;
    for cs in [1u8, 2] {
        for ch in [Channels::A, Channels::B] {
            if !first {
                ls_off(Channels::mask(prev.unwrap()), emit);
            }
            first = false;
            select(ch, 0, emit);
            ls_on(ch.mask(), emit);
            launch_mrws(cs, &round2, emit);
            prev = Some(ch);
        }
    }
}
