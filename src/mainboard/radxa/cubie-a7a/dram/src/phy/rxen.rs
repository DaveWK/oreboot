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
//!
//! Command codes: first-edge CA values of the LPDDR5 two-edge truth table
//! (JEDEC LPDDR5/5X; as printed in LiteDRAM 74522f7 `phy/lpddr5/commands.py`):
//! CAS with WS_FS `0x4c`, RDC `0x50`, CAS with WS_WR/WS_RD/WS_FS `0x7c`, NOP
//! `0x00`, second edge 0. They are the reference's row data, kept here as
//! observed values. A shared LPDDR5 command encoder for `jedec_lpddr` is
//! requested (orchestrator PROGRESS, REQ-A7A-01); these constants move to it
//! when it exists.
use super::coldinit::{mrw_ddr, Row};
use super::wcklevel::Rank;

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

const CA_CAS_FS: u16 = 0x4c;
const CA_RDC: u16 = 0x50;
const CA_CAS_ALL: u16 = 0x7c;

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
