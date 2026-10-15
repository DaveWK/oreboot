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
//! This covers DevInit at the boot pstate: part 1 after the DRAM reset is
//! released (CS initialization, a two-row command with rising code 0x70 and
//! operand 6, multi-purpose command class, purpose not verified, and MRW of
//! MR16/MR1/MR28, then MR16/MR41, per channel and rank), and the command bus
//! training blocks ([`CBT_BLOCKS`]: per channel, rank and CBT phase, the
//! entry with WCK2CK sync and I/O mode registers, CBT pattern points and the
//! exit). The second and later blocks are checked against the oracle's
//! decoded record in `tests/fixtures/devinit-cbt-blocks-pstate1.txt`. Mode
//! register values are inputs ([`Mrs`], [`Mrs2`]); the CA search on the
//! echo is ours (`blobless::cold_init_ca_train`).

use jedec_lpddr::lp5::cmd::{self, CaEdges};
use jedec_lpddr::lp5::mr::Mr16;

/// One ACSM memory row.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct Row(pub [u16; 4]);

impl Row {
    pub const EMPTY: Row = Row([0; 4]);

    /// Row with explicit halfwords (CA pattern rows with different phase
    /// values, and the hw1/hw2 control rows of the WCK and CBT sequences).
    pub const fn raw(hw0: u16, hw1: u16, hw2: u16, hw3: u16) -> Row {
        Row([hw0, hw1, hw2, hw3])
    }

    /// Row driving CA `lo` on the first and `hi` on the second phase with
    /// chip selects `cs` (CBT patterns, and commands sent while a rank is in
    /// command bus training). The oracle's rows of this form carry no control
    /// word, also for rank 1.
    pub const fn ca2(cs: u8, lo: u8, hi: u8) -> Row {
        Row([
            ((cs as u16 & 3) << 14) | (((hi & 0x7f) as u16) << 7) | (lo & 0x7f) as u16,
            0,
            0,
            0,
        ])
    }

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

/// The two CA edge pairs of an MRW (`jedec_lpddr::lp5::cmd::mrw`; MA
/// masked to its 7 bits as the rows carry it).
const fn mrw_edges(ma: u8, op: u8) -> [CaEdges; 2] {
    match cmd::mrw(ma & 0x7f, op) {
        Ok(e) => e,
        Err(_) => panic!("MA is 7 bits"),
    }
}

/// LPDDR5 MRW as four rows: MRW-1 code, MA, MRW-2 code with OP[7], OP[6:0].
pub const fn mrw(cs: u8, ma: u8, op: u8) -> [Row; 4] {
    let [w1, w2] = mrw_edges(ma, op);
    [
        Row::cmd(cs, w1.rise),
        Row::cmd(0, w1.fall),
        Row::cmd(cs, w2.rise),
        Row::cmd(0, w2.fall),
    ]
}

/// LPDDR5 MRW as two rows with both CA edges each (MRW-1 + MA, MRW-2 +
/// OP[6:0]), CS on both: the form the oracle uses for rank 0 while rank 1 is
/// in command bus training.
pub const fn mrw_ddr(cs: u8, ma: u8, op: u8) -> [Row; 2] {
    let [w1, w2] = mrw_edges(ma, op);
    [
        Row::ca2(cs, w1.rise, w1.fall),
        Row::ca2(cs, w2.rise, w2.fall),
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

/// LPDDR5 MR16 (`jedec_lpddr::lp5::mr::Mr16`): OP[1:0] FSP-WR, OP[3:2]
/// FSP-OP, OP[5:4] CBT mode, OP[6] VRCG; OP[7] CBT phase is set per block in
/// [`block_entry`].
pub const fn mr16(fsp_wr: u8, fsp_op: u8, cbt: u8, vrcg: bool) -> u8 {
    Mr16 {
        fsp_wr,
        fsp_op,
        cbt,
        vrcg,
        cbt_phase: false,
    }
    .encode()
}

/// Message block words (`phy::msgblock::build`) holding the mode register
/// inputs: low byte rank 0, high byte rank 1; the word after each is the
/// second channel's copy.
pub mod mb {
    pub const MR1: usize = 0x2d;
    pub const MR3: usize = 0x31;
    pub const MR11: usize = 0x35;
    pub const MR13: usize = 0x39;
    pub const MR17: usize = 0x41;
    pub const MR18: usize = 0x43;
    pub const MR22: usize = 0x4b;
    pub const MR28: usize = 0x55;
    pub const MR41: usize = 0x65;
}

const fn mb_byte(m: &[u16], word: usize, rank: usize) -> u8 {
    (m[word] >> (8 * rank)) as u8
}

/// DevInit's MRW values from the message block inputs, rank `rank`. The
/// per-bit rules were established by flipping each bit of each message
/// block word in the PMU emulator (oracle) and decoding the MRWs:
/// MR1 bit 3 is cleared; MR3 bit 7 is set and bit 6 is set when the message
/// block's MR3 and MR22 bit 6 agree; MR11 keeps OP[7:4]; MR13 bit 6 is set,
/// bit 4 cleared and bit 5 inverted; MR17 as given; MR18 OP[7:5] as given,
/// OP[4:0] = 8; MR28 bit 5 cleared; MR41 keeps OP[4:0]. MR16 follows the
/// FSP/CBT sequence ([`mr16`]).
pub struct MrInputs<'a>(pub &'a [u16]);

impl MrInputs<'_> {
    pub const fn mr1(&self, rank: usize) -> u8 {
        mb_byte(self.0, mb::MR1, rank) & !0x08
    }
    pub const fn mr3(&self, rank: usize) -> u8 {
        let v = mb_byte(self.0, mb::MR3, rank);
        let agree = (mb_byte(self.0, mb::MR3, 0) ^ mb_byte(self.0, mb::MR22, 0)) & 0x40 == 0;
        (v & 0x3f) | 0x80 | if agree { 0x40 } else { 0 }
    }
    pub const fn mr11(&self, rank: usize) -> u8 {
        mb_byte(self.0, mb::MR11, rank) & 0xf0
    }
    pub const fn mr13(&self, rank: usize) -> u8 {
        ((mb_byte(self.0, mb::MR13, rank) ^ 0x20) & !0x50) | 0x40
    }
    pub const fn mr17(&self, rank: usize) -> u8 {
        mb_byte(self.0, mb::MR17, rank)
    }
    pub const fn mr18(&self, rank: usize) -> u8 {
        (mb_byte(self.0, mb::MR18, rank) & 0xe0) | 0x08
    }
    pub const fn mr28(&self, rank: usize) -> u8 {
        mb_byte(self.0, mb::MR28, rank) & !0x20
    }
    pub const fn mr41(&self, rank: usize) -> u8 {
        mb_byte(self.0, mb::MR41, rank) & 0x1f
    }
}

impl Mrs {
    /// Part 1 values from the message block (rank 0).
    pub const fn from_msgblock(m: &[u16]) -> Mrs {
        let i = MrInputs(m);
        Mrs {
            mr16_first: mr16(0, 0, 0, true),
            mr1: i.mr1(0),
            mr28: i.mr28(0),
            mr16_second: mr16(1, 0, 0, true),
            mr41: i.mr41(0),
        }
    }
}

impl Mrs2 {
    /// Part 2 and CBT values from the message block (rank 0; MR17 also of
    /// rank 1).
    pub const fn from_msgblock(m: &[u16]) -> Mrs2 {
        let i = MrInputs(m);
        Mrs2 {
            mr18: i.mr18(0),
            mr3: i.mr3(0),
            mr17: i.mr17(0),
            mr11: i.mr11(0),
            mr13: i.mr13(0),
            mr16_cbt: mr16(1, 0, 2, true),
            mr16_exit: mr16(1, 0, 0, true),
            mr16_park: mr16(1, 1, 0, true),
            mr16_unpark: mr16(0, 0, 0, true),
            mr17_rank1: i.mr17(1),
        }
    }
}

/// One step of the program. `Launch` loads `rows` from row `start` (stop =
/// start + len - 1, outer loop 0), runs it and waits for Done. `LaunchLoop`
/// does the same without waiting: the caller has set ACSMInfiniteOLRC, so the
/// sequence repeats until the program clears it and issues `AwaitEnd` (wait
/// for Done, clear Run). `Read` samples a CSR (AsyncDbyteRxData).
pub enum Step<'a> {
    Write(u32, u16),
    Launch { start: u16, rows: &'a [Row] },
    LaunchLoop { start: u16, rows: &'a [Row] },
    AwaitEnd,
    Read(u32),
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
    buf: [Row; 48],
    len: usize,
}

impl Rows {
    const fn new() -> Self {
        Rows {
            buf: [Row::EMPTY; 48],
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

/// Part 2 and command bus training mode registers.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Mrs2 {
    pub mr18: u8,
    pub mr3: u8,
    pub mr17: u8,
    pub mr11: u8,
    pub mr13: u8,
    /// MR16 value that enters command bus training (phase 0; OP[7] selects
    /// the CBT phase and is set for phase 1).
    pub mr16_cbt: u8,
    /// MR16 written to the trained rank when it leaves CBT.
    pub mr16_exit: u8,
    /// MR16 of rank 0 while rank 1 is trained (OP[7] follows the phase), and
    /// when it is restored.
    pub mr16_park: u8,
    pub mr16_unpark: u8,
    /// MR17 written to rank 1 before its set-up (rank 1 gets MR18, MR3 and
    /// MR13 only; rank 0 gets MR17 and MR11 with them).
    pub mr17_rank1: u8,
}

impl Mrs2 {
    /// The reference oracle's pstate-1 values for the CXMT 4 GB set.
    pub const ORACLE_PSTATE1_CXMT_4G: Mrs2 = Mrs2 {
        mr18: 0x08,
        mr3: 0xc6,
        mr17: 0x06,
        mr11: 0x10,
        mr13: 0x60,
        mr16_cbt: 0x61,
        mr16_exit: 0x41,
        mr16_park: 0x45,
        mr16_unpark: 0x40,
        mr17_rank1: 0x2e,
    };
}

const ACSM_INFINITE_OLRC: u32 = 0x20031;
/// DBYTE `n` asynchronous DQ access (TRM: AsyncDbyteTxMode/TxEn/TxData/RxData).
pub const fn async_tx_mode(n: u32) -> u32 {
    0x10089 + 0x1000 * n
}
pub const fn async_tx_en(n: u32) -> u32 {
    0x1008b + 0x1000 * n
}
pub const fn async_tx_data(n: u32) -> u32 {
    0x1008c + 0x1000 * n
}
pub const fn async_rx_data(n: u32) -> u32 {
    0x1008d + 0x1000 * n
}
/// The DBYTE that returns channel A's CBT feedback.
pub const CBT_DBYTE_A: u32 = 1;
/// AC block `blk` CA lane `lane` delay (Address/Command Delay, per pstate).
/// Block 0xf addresses every AC block; lane 0xf every lane of a block.
pub const fn ca_delay(blk: u32, lane: u32) -> u32 {
    0x30001 + 0x1000 * blk + 0x100 * lane
}
/// AC block `blk` register 2 (written with the CA delays; meaning not
/// established).
pub const fn ac_reg2(blk: u32) -> u32 {
    0x30002 + 0x1000 * blk
}
/// AC broadcast block.
pub const AC_ALL: u32 = 0xf;
/// Applies new delay CSR values (TRM: ForceInternalUpdate), pulsed 1 then 0.
const FORCE_INTERNAL_UPDATE: u32 = 0x200a3;
/// TRM register list: ACSMWckFreeRunMode, ACSMWckEnControl.
const WCK_FREE_RUN: u32 = 0x20041;
const WCK_EN_CONTROL: u32 = 0x2002a;
/// Written 0x100 before CBT entry and 0 after the exit (not established).
const CBT_CONTROL: u32 = 0x20007;
/// WCK loop body: rows with hw1/hw2 control (meaning not established).
const WCK_LOOP: [Row; 16] = [
    Row::raw(0, 0x4000, 1, 0),
    Row::EMPTY,
    Row::EMPTY,
    Row::EMPTY,
    Row::nop(0x0400),
    Row::EMPTY,
    Row::nop(0x0400),
    Row::EMPTY,
    Row::nop(0x0400),
    Row::EMPTY,
    Row::EMPTY,
    Row::EMPTY,
    Row::raw(0, 0, 1, 0),
    Row::EMPTY,
    Row::EMPTY,
    Row::EMPTY,
];
const WCK_LOOP_START: u16 = 300;
const PAIR_GAP: [Row; 2] = [Row::nop(0x5b00), Row::EMPTY];

/// One channel's command bus training resources: the AC block driving its CA
/// lanes, the DBYTE returning the DRAM's echo, and the asynchronous DQ set-up
/// of that DBYTE (values as in the oracle).
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CbtChannel {
    pub ch: Channels,
    pub ac: u32,
    pub dbyte: u32,
    /// AsyncDbyteTxMode/TxEn while measuring: the lanes the host drives (one
    /// DQ lane plus lane 8, DMI); they carry no echo.
    pub tx_lanes: u16,
    /// AsyncDbyteTxData written after the CBT entry MRW.
    pub tx_entry: u16,
    /// AsyncDbyteTxData written before measuring (two writes).
    pub tx_data: [u16; 2],
}

impl CbtChannel {
    /// AsyncDbyteRxData bits that carry the echo of CA[6:0].
    pub const fn echo_bits(&self) -> u16 {
        0xff & !self.tx_lanes
    }
}

pub const CBT_A: CbtChannel = CbtChannel {
    ch: Channels::A,
    ac: 0,
    dbyte: CBT_DBYTE_A,
    tx_lanes: 0x110,
    tx_entry: 0x10,
    tx_data: [0x52, 0x152],
};
pub const CBT_B: CbtChannel = CbtChannel {
    ch: Channels::B,
    ac: 1,
    dbyte: 3,
    tx_lanes: 0x104,
    tx_entry: 0x04,
    tx_data: [0x0e, 0x10e],
};

/// One command bus training block: a channel, a rank and a CBT phase (MR16
/// OP[7]).
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CbtBlock {
    pub chan: CbtChannel,
    pub rank: u8,
    pub phase: u8,
}

impl CbtBlock {
    /// Chip select of the trained rank.
    pub const fn cs(&self) -> u8 {
        1 << self.rank
    }
}

const fn block(chan: CbtChannel, rank: u8, phase: u8) -> CbtBlock {
    CbtBlock { chan, rank, phase }
}

/// The oracle's block order: per channel, per rank, phase 0 then 1.
pub const CBT_BLOCKS: [CbtBlock; 8] = [
    block(CBT_A, 0, 0),
    block(CBT_A, 0, 1),
    block(CBT_A, 1, 0),
    block(CBT_A, 1, 1),
    block(CBT_B, 0, 0),
    block(CBT_B, 0, 1),
    block(CBT_B, 1, 0),
    block(CBT_B, 1, 1),
];

/// WCK loop with ACSMInfiniteOLRC set, then cleared and awaited.
fn wck_loop(emit: &mut impl FnMut(Step)) {
    emit(Step::Write(ACSM_INFINITE_OLRC, 1));
    emit(Step::LaunchLoop {
        start: WCK_LOOP_START,
        rows: &WCK_LOOP,
    });
    emit(Step::Write(ACSM_INFINITE_OLRC, 0));
    emit(Step::AwaitEnd);
}

fn async_open(n: u32, emit: &mut impl FnMut(Step)) {
    emit(Step::Write(async_tx_mode(n), 0x1ff));
    emit(Step::Write(async_tx_en(n), 0x1ff));
    emit(Step::Write(async_tx_data(n), 0));
}

/// The MRW launch that enters CBT on `b`'s rank and phase, with the WCK loop
/// body, repeating; rank 0 is parked while rank 1 enters.
fn entry_launch(b: &CbtBlock, m: &Mrs2, emit: &mut impl FnMut(Step)) {
    let mut r = Rows::new();
    r.push(&HEAD);
    r.push(&mrw(b.cs(), 16, m.mr16_cbt | (b.phase << 7)));
    if b.rank == 0 {
        r.push(&[Row::nop(0x1a00), Row::EMPTY, Row::nop(0x0100), Row::EMPTY]);
    } else {
        r.push(&[Row::nop(0x3b00), Row::EMPTY]);
        r.push(&mrw(1, 16, m.mr16_park | (b.phase << 7)));
        r.push(&[Row::nop(0x7b00), Row::nop(0x0c00)]);
    }
    r.push(&WCK_LOOP);
    r.push(&TAIL);
    emit(Step::LaunchLoop {
        start: 0,
        rows: r.rows(),
    });
}

/// After the entry launch: stop it, then the WCK loop again while the
/// asynchronous DQ path is set up for measuring.
fn entry_finish(c: &CbtChannel, emit: &mut impl FnMut(Step)) {
    let mask = c.ch.mask();
    emit(Step::Write(async_tx_data(c.dbyte), c.tx_entry));
    ls_off(mask, emit);
    emit(Step::Write(ACSM_INFINITE_OLRC, 0));
    emit(Step::AwaitEnd);
    emit(Step::Write(ACSM_INFINITE_OLRC, 1));
    emit(Step::LaunchLoop {
        start: WCK_LOOP_START,
        rows: &WCK_LOOP,
    });
    emit(Step::Write(async_tx_data(c.dbyte), c.tx_data[0]));
    emit(Step::Write(async_tx_data(c.dbyte), c.tx_data[1]));
    emit(Step::Write(async_tx_mode(c.dbyte), c.tx_lanes));
    emit(Step::Write(async_tx_en(c.dbyte), c.tx_lanes));
    emit(Step::Write(ACSM_INFINITE_OLRC, 0));
    emit(Step::AwaitEnd);
}

/// Enter command bus training for block `b`. For phase 0 this selects the
/// channel and writes the rank's WCK and I/O mode registers first (CAS-WCK
/// sync included); phase 1 follows the phase-0 exit of the same rank and
/// only re-enters with MR16 OP[7] set.
pub fn block_entry(b: &CbtBlock, m: &Mrs2, emit: &mut impl FnMut(Step)) {
    let c = &b.chan;
    let mask = c.ch.mask();
    let cs = b.cs();
    if b.phase == 0 {
        select(c.ch, 0, emit);
        emit(Step::Write(ca_delay(c.ac, 0xf), 0));
        emit(Step::Write(ac_reg2(c.ac), 0));
    }
    emit(Step::Write(WCK_FREE_RUN, 1));
    emit(Step::Write(WCK_EN_CONTROL, 1));
    emit(Step::Write(CBT_CONTROL, 0x100));
    async_open(c.dbyte, emit);
    if b.phase == 0 {
        wck_loop(emit);
        ls_on(mask, emit);
        if b.rank != 0 {
            let mut r = Rows::new();
            r.push(&HEAD);
            r.push(&mrw(cs, 17, m.mr17_rank1));
            r.push(&AFTER_MRW);
            r.push(&TAIL);
            emit(Step::Launch {
                start: 0,
                rows: r.rows(),
            });
            ls_off(mask, emit);
            ls_on(mask, emit);
        }
        // MR18, WCK2CK sync (CAS 0x38/0x40), then the I/O mode registers.
        let mut r = Rows::new();
        r.push(&HEAD);
        r.push(&mrw(cs, 18, m.mr18));
        r.push(&PAIR_GAP);
        r.push(&PAIR_GAP);
        r.push(&[Row::cmd(cs, 0x38), Row::cmd(cs, 0x40)]);
        r.push(&PAIR_GAP);
        let rank0 = [(3, m.mr3), (17, m.mr17), (11, m.mr11), (13, m.mr13)];
        let rank1 = [(3, m.mr3), (13, m.mr13)];
        let regs: &[(u8, u8)] = if b.rank == 0 { &rank0 } else { &rank1 };
        for &(ma, op) in regs {
            r.push(&mrw(cs, ma, op));
            r.push(&AFTER_MRW);
        }
        r.push(&TAIL);
        emit(Step::Launch {
            start: 0,
            rows: r.rows(),
        });
        ls_off(mask, emit);
    }
    emit(Step::Write(ACSM_INFINITE_OLRC, 1));
    ls_on(mask, emit);
    entry_launch(b, m, emit);
    entry_finish(c, emit);
}

/// Leave command bus training of block `b`: CA delays of every AC block back
/// to 0 (applied), rank 0 restored (rank-1 blocks, in the two-edge form while rank 1 is
/// still in CBT), the WCK loop, then MR16 on the trained rank.
pub fn block_exit(b: &CbtBlock, m: &Mrs2, emit: &mut impl FnMut(Step)) {
    let c = &b.chan;
    let mask = c.ch.mask();
    for lane in 0..7 {
        emit(Step::Write(ca_delay(AC_ALL, lane), 0));
    }
    emit(Step::Write(FORCE_INTERNAL_UPDATE, 1));
    emit(Step::Write(FORCE_INTERNAL_UPDATE, 0));
    if b.rank != 0 {
        let mut r = Rows::new();
        r.push(&mrw_ddr(1, 16, m.mr16_unpark));
        r.push(&[Row::nop(0x7b00), Row::nop(0x0400)]);
        emit(Step::Launch {
            start: 0,
            rows: r.rows(),
        });
    }
    ls_on(mask, emit);
    emit(Step::Write(ACSM_INFINITE_OLRC, 1));
    emit(Step::LaunchLoop {
        start: WCK_LOOP_START,
        rows: &WCK_LOOP,
    });
    emit(Step::Write(async_tx_data(c.dbyte), 0x100));
    emit(Step::Write(ACSM_INFINITE_OLRC, 0));
    emit(Step::AwaitEnd);
    emit(Step::Write(WCK_FREE_RUN, 0));
    ls_on(mask, emit);
    let mut r = Rows::new();
    r.push(&HEAD);
    r.push(&[Row::nop(0x5b00), Row::nop(0x0c00)]);
    r.push(&mrw(b.cs(), 16, m.mr16_exit));
    r.push(&AFTER_FIRST_MRW);
    r.push(&TAIL);
    emit(Step::Launch {
        start: 0,
        rows: r.rows(),
    });
    ls_off(mask, emit);
    emit(Step::Write(async_tx_data(c.dbyte), 0x1000));
    emit(Step::Write(CBT_CONTROL, 0));
}

/// After the last block: every AC block's CA delays and register 2 to 0,
/// applied.
pub fn cbt_end(emit: &mut impl FnMut(Step)) {
    for lane in 0..7 {
        emit(Step::Write(ca_delay(AC_ALL, lane), 0));
    }
    emit(Step::Write(ac_reg2(AC_ALL), 0));
    emit(Step::Write(FORCE_INTERNAL_UPDATE, 1));
    emit(Step::Write(FORCE_INTERNAL_UPDATE, 0));
}

/// Part 2 at the boot pstate: both channels selected once, then CBT entry on
/// channel A, rank 0, phase 0 ([`CBT_BLOCKS`]\[0\]) with the asynchronous DQ
/// path of DBYTE 1 set up and the WCK loop stopped. Continues where [`part1`]
/// ended.
pub fn part2(m: &Mrs2, emit: &mut impl FnMut(Step)) {
    ls_off(Channels::B.mask(), emit);
    select(Channels::Both, 0x0e, emit);
    block_entry(&CBT_BLOCKS[0], m, emit);
}

/// The four CBT patterns for phase 0: (first row, CS row) as (cs, CA first
/// phase, CA second phase), CS row on rank 0. The DRAM in CBT mode returns the
/// CA it latched on DQ.
pub const CBT_PATTERNS: [[(u8, u8, u8); 2]; 4] = [
    [(0, 0x00, 0x7f), (1, 0x00, 0x00)],
    [(0, 0x00, 0x00), (1, 0x7f, 0x00)],
    [(0, 0x7f, 0x00), (1, 0x7f, 0x7f)],
    [(0, 0x7f, 0x7f), (1, 0x00, 0x7f)],
];
/// Phase 1 (the DRAM latches the CS row's second edge): that edge carries the
/// level of [`CBT_EXPECT`], so the aligned echo is the same as in phase 0.
pub const CBT_PATTERNS_PHASE1: [[(u8, u8, u8); 2]; 4] = [
    [(0, 0x00, 0x00), (1, 0x7f, 0x00)],
    [(0, 0x00, 0x00), (1, 0x00, 0x7f)],
    [(0, 0x7f, 0x7f), (1, 0x00, 0x7f)],
    [(0, 0x7f, 0x7f), (1, 0x7f, 0x00)],
];
const CBT_STARTS: [u16; 4] = [100, 118, 136, 154];

/// One CBT point on channel A, rank 0, phase 0: set all seven CA lane delays
/// of AC block 0 to `delay`, apply, then for each pattern run its launch and
/// read DBYTE 1's AsyncDbyteRxData.
pub fn cbt_point(delay: u16, emit: &mut impl FnMut(Step)) {
    cbt_point_bytes(delay, &[CBT_DBYTE_A], emit)
}

/// [`cbt_point`] reading AsyncDbyteRxData of each DBYTE in `bytes` after
/// every pattern (diagnostics: where the CBT feedback appears).
pub fn cbt_point_bytes(delay: u16, bytes: &[u32], emit: &mut impl FnMut(Step)) {
    set_ca_delays(&[delay; 7], emit);
    cbt_patterns(bytes, emit);
}

/// One CBT point of block `b`: its channel's seven CA lane delays, then the
/// four patterns of its phase on its rank, reading its DBYTE after each.
pub fn cbt_point_block(b: &CbtBlock, delays: &[u16; 7], emit: &mut impl FnMut(Step)) {
    set_block_delays(b.chan.ac, delays, emit);
    block_patterns(b, &[b.chan.dbyte], emit);
}

/// Set the seven CA lane delays of AC block 0 and apply them
/// (ForceInternalUpdate pulse).
pub fn set_ca_delays(delays: &[u16; 7], emit: &mut impl FnMut(Step)) {
    set_block_delays(0, delays, emit);
}

/// Set the seven CA lane delays of AC block `ac` and apply them.
pub fn set_block_delays(ac: u32, delays: &[u16; 7], emit: &mut impl FnMut(Step)) {
    for (lane, &d) in delays.iter().enumerate() {
        emit(Step::Write(ca_delay(ac, lane as u32), d));
    }
    emit(Step::Write(FORCE_INTERNAL_UPDATE, 1));
    emit(Step::Write(FORCE_INTERNAL_UPDATE, 0));
}

/// Run the four phase-0 rank-0 CBT pattern launches, reading
/// AsyncDbyteRxData of each DBYTE in `bytes` after every one.
pub fn cbt_patterns(bytes: &[u32], emit: &mut impl FnMut(Step)) {
    block_patterns(&CBT_BLOCKS[0], bytes, emit);
}

/// The four CBT pattern launches of block `b` (its phase's patterns, CS row
/// on its rank), reading AsyncDbyteRxData of each DBYTE in `bytes` after
/// every one.
pub fn block_patterns(b: &CbtBlock, bytes: &[u32], emit: &mut impl FnMut(Step)) {
    let pats = if b.phase == 0 {
        &CBT_PATTERNS
    } else {
        &CBT_PATTERNS_PHASE1
    };
    for (pat, &start) in pats.iter().zip(CBT_STARTS.iter()) {
        let [(c0, l0, h0), (_, l1, h1)] = *pat;
        let rows = [
            Row::raw(0, 0x4000, 1, 0),
            Row::EMPTY,
            Row::nop(0x2b00),
            Row::EMPTY,
            Row::nop(0x3b00),
            Row::EMPTY,
            Row::ca2(c0, l0, h0),
            Row::ca2(b.cs(), l1, h1),
            Row::nop(0x3b00),
            Row::nop(0x0800),
            Row::raw(0, 0x0003, 0, 0),
            Row::EMPTY,
            Row::EMPTY,
            Row::EMPTY,
            Row::raw(0, 0, 1, 0),
            Row::EMPTY,
            Row::nop(0x2b00),
            Row::EMPTY,
        ];
        emit(Step::Launch { start, rows: &rows });
        for &n in bytes {
            emit(Step::Read(async_rx_data(n)));
        }
    }
}

/// AsyncDbyteRxData bits (physical lanes) that carry the CBT echo of CA[6:0]
/// on channel A (measured, M6): lanes 4 (host-driven, memory DQ7 by the
/// DBYTE 1 lane map) and 8 (DMI) do not. See [`CbtChannel::echo_bits`].
pub const CBT_ECHO_BITS: u16 = CBT_A.echo_bits();
/// Expected echo per pattern when the DRAM latches the CS row's edge of its
/// phase (the aligned case), as a per-bit level.
pub const CBT_EXPECT: [bool; 4] = [false, true, true, false];

/// Bits of `echo` (four AsyncDbyteRxData samples, one per pattern) that pass
/// on channel A: every pattern shows the expected level on that bit.
pub const fn cbt_pass(echo: [u16; 4]) -> u16 {
    cbt_pass_bits(echo, CBT_ECHO_BITS)
}

/// [`cbt_pass`] over the echo bits `bits` (a channel's
/// [`CbtChannel::echo_bits`]).
pub const fn cbt_pass_bits(echo: [u16; 4], bits: u16) -> u16 {
    dram_train::window::pass_bits(&echo, &CBT_EXPECT, bits)
}

/// The run of set bits in a circular `n`-bit map that contains point `pref`,
/// or else the run whose center is circularly nearest to it (ties: the
/// longer, then the first found): (start, len); (0, 0) for an empty map.
/// Used to pick one of several equivalent windows (the CA delay passes once
/// per 256 units) consistently around a reference point.
pub fn run_near(map: u128, n: u32, pref: u32) -> (u32, u32) {
    dram_train::window::run_near(map, n, pref).as_tuple()
}

/// Longest run of set bits in a circular `n`-bit window map: (start, len).
pub fn longest_run(map: u128, n: u32) -> (u32, u32) {
    dram_train::window::longest_run(map, n).as_tuple()
}
