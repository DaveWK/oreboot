// SPDX-License-Identifier: GPL-2.0-only
//! ACSM row generators for the A733 boot-pstate read enable stage (the
//! reference's PMU message 0x02), reproduced from derived register data.
//!
//! Reference: the a06 native PMU emulator trace, pstate 1, launches
//! 4286..4321, extracted in orchestrator evidence
//! `boards/radxa-cubie-a7a/20261008-a7a-board-01/a14` and checked row for
//! row by `tools/native-dram-controls/tests/rxen_oracle.rs`. No firmware code
//! or bytes are used. This module only encodes rows; it launches nothing
//! and claims no training result (the emulator has no DQ data path).
//!
//! One reference block (per channel and rank) is nine launches:
//! [`entry`] (MR20, MR33, MR34, MR31, MR32), [`read_program_32`] with outer
//! loop 96 and then once, [`read_program_34`] three times and then with
//! outer loop 64, and two [`mode_rows`] launches (MR18, MR10, MR46).
//! Between the launches the reference writes the controls that
//! [`block_setup`], [`rank_switch`], [`bars`], [`gates_on`], [`counter`],
//! [`fine_setup`] and [`apply`] emit as [`Step`]s; the TrainingCntr values
//! and the applied RxEnDly are search state, passed in by the caller.
//!
//! Command codes: the first-edge CA values of CAS (with WS_FS, and with
//! WS_WR/WS_RD/WS_FS) and RDC from `jedec_lpddr::lp5::cmd` (JEDEC LPDDR5/5X
//! truth table); the second edges are 0, as the reference's rows carry them.

use super::coldinit::{mrw_ddr, Row};
use super::wcklevel::{clear_counts, Channel, Rank, Step};
use jedec_lpddr::lp5::cmd;

/// Mode register inputs of the read enable entry ([`entry`]), in order.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct EntryMrs {
    pub mr20: u8,
    pub mr33: u8,
    pub mr34: u8,
    pub mr31: u8,
    pub mr32: u8,
}

impl EntryMrs {
    /// The values the reference wrote for read enable at pstate 1 (MR31-34
    /// zero: the DQ calibration pattern RDC returns).
    pub const READ_ENABLE_PSTATE1: EntryMrs = EntryMrs {
        mr20: 0x02,
        mr33: 0x00,
        mr34: 0x00,
        mr31: 0x00,
        mr32: 0x00,
    };
    /// The same launch before read DQ centring (launch 4321): pattern
    /// MR33/MR34 = 0x36/0xa5.
    pub const READ_CENTRING_PSTATE1: EntryMrs = EntryMrs {
        mr20: 0x02,
        mr33: 0x36,
        mr34: 0xa5,
        mr31: 0x00,
        mr32: 0x00,
    };
}

/// Mode register inputs of one [`mode_rows`] launch.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ModeMrs {
    pub mr18: u8,
    pub mr10: u8,
    pub mr46: u8,
}

impl ModeMrs {
    /// First launch after a block's search (reference, pstate 1).
    pub const EXIT_PSTATE1: ModeMrs = ModeMrs {
        mr18: 0x0c,
        mr10: 0x54,
        mr46: 0x00,
    };
    /// Second launch, for the rank the next block uses.
    pub const NEXT_PSTATE1: ModeMrs = ModeMrs {
        mr18: 0x1c,
        mr10: 0x59,
        mr46: 0x00,
    };
}

/// First-edge CA of a CAS with the given WCK sync bits.
const fn cas(ws_wr: bool, ws_rd: bool, ws_fs: bool) -> u16 {
    match (cmd::Cas {
        ws_wr,
        ws_rd,
        ws_fs,
        dc: 0,
        wrx: false,
        wxsa: false,
        wxsb: false,
    })
    .encode()
    {
        Some(e) => e.rise as u16,
        None => panic!("DC is 4 bits"),
    }
}
/// CAS with fast WCK sync (WS_FS): `0x4c`.
const CA_CAS_FS: u16 = cas(false, false, true);
/// RDC (read DQ calibration): `0x50`.
const CA_RDC: u16 = cmd::RDC.rise as u16;
/// CAS with WS_WR, WS_RD and WS_FS: `0x7c`.
const CA_CAS_ALL: u16 = cas(true, true, true);
const _: () = assert!(CA_CAS_FS == 0x4c && CA_RDC == 0x50 && CA_CAS_ALL == 0x7c);

const fn cs_bits(rank: Rank) -> u16 {
    (rank.cs() as u16) << 14
}

/// MRW as two rows with the rank's control word on both.
const fn mrw_rows(rank: Rank, ma: u8, op: u8) -> [Row; 2] {
    let e = mrw_ddr(rank.cs(), ma, op);
    [
        Row::raw(e[0].0[0], 0, 0, rank.control()),
        Row::raw(e[1].0[0], 0, 0, rank.control()),
    ]
}

/// Command row: first-edge CA `ca`, second edge 0, CS of `rank`.
const fn cmd(rank: Rank, ca: u16, hw1: u16, hw2: u16, ctl: u16) -> Row {
    Row::raw(cs_bits(rank) | ca, hw1, hw2, ctl | rank.control())
}

/// The 20-row entry: five MRWs, each followed by the observed timing rows.
pub const fn entry(rank: Rank, m: EntryMrs) -> [Row; 20] {
    let mut rows = [Row::EMPTY; 20];
    let mrs = [
        (20, m.mr20),
        (33, m.mr33),
        (34, m.mr34),
        (31, m.mr31),
        (32, m.mr32),
    ];
    let mut i = 0;
    while i < 5 {
        let r = mrw_rows(rank, mrs[i].0, mrs[i].1);
        rows[4 * i] = r[0];
        rows[4 * i + 1] = r[1];
        rows[4 * i + 2] = Row::nop(0x7b00);
        rows[4 * i + 3] = Row::nop(0x0400);
        i += 1;
    }
    rows
}

/// Timing rows both read programs share: (offset from the first, control).
const TAIL: [(usize, u16); 8] = [
    (0, 0x2e00),
    (1, 0x0400),
    (2, 0x0500),
    (4, 0x0480),
    (6, 0x0400),
    (8, 0x0400),
    (10, 0x2f00),
    (12, 0x0400),
];

/// The 32-row read program (warm-up loop, outer 96, then once): CAS with
/// fast WCK sync, four RDCs, the timing rows, CAS (all sync bits), NOP.
pub const fn read_program_32(rank: Rank) -> [Row; 32] {
    let mut rows = [Row::EMPTY; 32];
    rows[1] = cmd(rank, CA_CAS_FS, 0xd000, 0x0001, 0);
    rows[2] = cmd(rank, CA_RDC, 0, 0, 0x0400);
    rows[4] = cmd(rank, CA_RDC, 0, 0, 0x0400);
    rows[6] = cmd(rank, CA_RDC, 0x0003, 0, 0x0400);
    rows[8] = cmd(rank, CA_RDC, 0, 0, 0x0400);
    let mut i = 0;
    while i < TAIL.len() {
        rows[10 + TAIL[i].0] = Row::nop(TAIL[i].1);
        i += 1;
    }
    rows[26] = cmd(rank, CA_CAS_ALL, 0, 0, 0);
    rows[30] = cmd(rank, 0, 0, 0x0001, 0);
    rows
}

/// The 34-row read program (the measured launches; the last with outer 64):
/// as [`read_program_32`] with the third and fourth RDC one row later.
pub const fn read_program_34(rank: Rank) -> [Row; 34] {
    let mut rows = [Row::EMPTY; 34];
    rows[1] = cmd(rank, CA_CAS_FS, 0xd000, 0x0001, 0);
    rows[2] = cmd(rank, CA_RDC, 0, 0, 0x0400);
    rows[4] = cmd(rank, CA_RDC, 0, 0, 0x0400);
    rows[6] = Row::nop(0x0400);
    rows[7] = cmd(rank, CA_RDC, 0x0003, 0, 0);
    rows[8] = Row::nop(0x0400);
    rows[9] = cmd(rank, CA_RDC, 0, 0, 0);
    rows[10] = Row::nop(0x0400);
    let mut i = 0;
    while i < TAIL.len() {
        rows[12 + TAIL[i].0] = Row::nop(TAIL[i].1);
        i += 1;
    }
    rows[28] = cmd(rank, CA_CAS_ALL, 0, 0, 0);
    rows[32] = cmd(rank, 0, 0, 0x0001, 0);
    rows
}

/// The 14-row mode launch after a block: MR18, MR10, MR46 with the observed
/// timing rows.
pub const fn mode_rows(rank: Rank, m: ModeMrs) -> [Row; 14] {
    let mut rows = [Row::EMPTY; 14];
    let mrs = [(18, m.mr18), (10, m.mr10), (46, m.mr46)];
    let mut i = 0;
    while i < 3 {
        let r = mrw_rows(rank, mrs[i].0, mrs[i].1);
        rows[4 * i] = r[0];
        rows[4 * i + 1] = r[1];
        rows[4 * i + 2] = Row::nop(0x0b00);
        rows[4 * i + 3] = Row::nop(0x0800);
        i += 1;
    }
    rows[12] = Row::nop(0x0b00);
    rows[13] = Row::nop(0x1000);
    rows
}

fn writes(values: &[(u32, u16)], out: &mut impl FnMut(Step<'_>)) {
    for &(csr, value) in values {
        out(Step::Write(csr, value));
    }
}

/// The channel's two DBYTEs.
const fn active(channel: Channel) -> [u32; 2] {
    let first = 2 - channel.inactive_first_byte();
    [first, first + 1]
}

/// Before a block on `channel`: channel selection, DBYTE `0x1n000`, the
/// reference's DTSM/training set-up (TrainingCntr broadcast `0x800`,
/// TrainingIncDecDtsmEn lane 0, TrainingParam `0x200`), the inactive
/// DBYTEs' lane-F DtsmLaneCtrl0 cleared, a count clear and a PHY pulse.
pub fn block_setup(channel: Channel, out: &mut impl FnMut(Step<'_>)) {
    let (a, b) = channel.enables();
    writes(
        &[
            (0x20072, a),
            (0x20073, b),
            (0x2007e, 0),
            (0x2007a, channel.override_mask()),
            (0x2000d, 8),
        ],
        out,
    );
    for db in 0..5 {
        out(Step::Write(0x10000 + db * 0x1000, 8));
    }
    writes(
        &[
            (0x2f0a0, 1),
            (0x2f0a0, 0),
            (0x200a3, 1),
            (0x200a3, 0),
            (0x3f084, 0),
            (0x1f00a, 0),
            (0x2002a, 1),
            (0x20041, 1),
            (0x70061, 0),
            (0x70062, 0),
            (0x70063, 0),
            (0x70064, 0),
            (0x7ff24, 0),
            (0x7ff25, 0),
            (0x7ff26, 0),
            (0x7ff27, 0),
            (0x7ff28, 0),
            (0x7ff29, 0),
            (0x7ff2a, 0),
            (0x7ff2b, 0),
            (0x70065, 0),
            (0x2f059, 3),
            (0x1ffb5, 0),
            (0x1f0b5, 1),
            (0x1f0b3, 0),
            (0x1f0b4, 0),
            (0x1f0b0, 0x30),
            (0x7f060, 1),
            (0x7f065, 0x1ff),
            (0x7ff26, 0),
            (0x7ff27, 0),
            (0x1f0b1, 0x182),
            (0x1ffaa, 0x800),
            (0x1ffbe, 0),
            (0x1f0be, 1),
            (0x1f0a7, 0x200),
            (0x1f0a2, 2),
        ],
        out,
    );
    let inactive = channel.inactive_first_byte();
    for db in inactive..inactive + 2 {
        out(Step::Write(0x10fb5 + db * 0x1000, 0));
    }
    clear_counts(out);
    writes(&[(0x20079, 1), (0x20079, 0)], out);
}

/// Between the rank-0 and rank-1 blocks, before [`block_setup`]: both
/// channels enabled and DBYTE `0x1n000 = 3` with a PHY pulse.
pub fn rank_switch(out: &mut impl FnMut(Step<'_>)) {
    writes(
        &[
            (0x20072, 3),
            (0x20073, 3),
            (0x2007e, 0),
            (0x2007a, 0xe),
            (0x2007a, 0),
            (0x2000d, 3),
        ],
        out,
    );
    for db in 0..5 {
        out(Step::Write(0x10000 + db * 0x1000, 3));
    }
    writes(&[(0x2f0a0, 1), (0x2f0a0, 0)], out);
}

/// Before the warm-up loop: DtsmGoodBar = DtsmErrBar = 1.
pub fn bars(out: &mut impl FnMut(Step<'_>)) {
    writes(&[(0x1f0b9, 1), (0x1f0ba, 1)], out);
}

/// TrainingCntr lane 0 of the channel's DBYTEs at the base addresses.
pub fn counter(channel: Channel, raw: u16, out: &mut impl FnMut(Step<'_>)) {
    for db in active(channel) {
        out(Step::Write(0x100aa + db * 0x1000, raw));
    }
}

/// Before the first measured launch: count clear, DtsmGateInc/Dec all
/// lanes, DtsmByteCtrl1/0.
pub fn gates_on(out: &mut impl FnMut(Step<'_>)) {
    clear_counts(out);
    writes(
        &[
            (0x1f0b3, 0x1ff),
            (0x1f0b4, 0x1ff),
            (0x1f0b1, 0x81),
            (0x1f0b0, 0x30),
        ],
        out,
    );
}

/// Before the PHY's 64-loop search: count clear, TrainingCntr `first`,
/// gates off, TrainingCntr `second`, DtsmLaneCtrl0 lane 0 of the channel's
/// DBYTEs at the base addresses, DtsmByteCtrl1.
pub fn fine_setup(channel: Channel, first: u16, second: u16, out: &mut impl FnMut(Step<'_>)) {
    clear_counts(out);
    counter(channel, first, out);
    writes(&[(0x1f0b3, 0), (0x1f0b4, 0), (0x1f0b0, 0)], out);
    counter(channel, second, out);
    out(Step::Write(0x1f0b5, 0));
    for db in active(channel) {
        out(Step::Write(0x100b5 + db * 0x1000, 1));
    }
    out(Step::Write(0x1f0b1, 0x81));
}

/// After the search: RxEnDly of the rank (`0x1n020` rank 0, `0x1n021`
/// rank 1) of the channel's DBYTEs at the base addresses, then the
/// reference's exit controls.
pub fn apply(channel: Channel, rank: Rank, rx_en: [u16; 2], out: &mut impl FnMut(Step<'_>)) {
    let tg = matches!(rank, Rank::R1) as u32;
    for (i, db) in active(channel).into_iter().enumerate() {
        out(Step::Write(0x10020 + tg + db * 0x1000, rx_en[i]));
    }
    writes(
        &[
            (0x1f0a7, 0),
            (0x1f0a2, 0x20),
            (0x1f0a2, 0),
            (0x2f059, 0),
            (0x1f0b1, 0),
            (0x3f084, 1),
            (0x1f00a, 1),
            (0x20041, 0),
            (0x2f0a0, 1),
            (0x2f0a0, 0),
        ],
        out,
    );
}
