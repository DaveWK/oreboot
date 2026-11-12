// SPDX-License-Identifier: GPL-2.0-only
//! End of initialization at the boot pstate, after command bus training and
//! before WCK2CK leveling (the reference's PMU message 0x00 stage): the
//! target frequency-set-point's mode registers written to every channel and
//! rank, then the DRAM switched to that FSP (MR16 FSP-OP = 1).
//!
//! Without this stage (CBT straight to WCK2CK leveling) the DRAM keeps
//! operating on FSP 0 with its reset mode registers (read latency, RDQS,
//! ODT); WL does not depend on them, reads do (on hardware: no read data at
//! any RxEn delay).
//!
//! Checked row for row and write for write against the reference (pstate
//! 1). The mode register values come from the message
//! block inputs ([`FspMrs::from_message_block`]); the row layout and the
//! timing rows are the reference's. No firmware code or bytes are used.
//!
//! Per channel and rank one 142-row launch: MR16 = 0x41 (FSP-WR 1, VRCG),
//! MR18, MR1, MR2, a two-edge command the reference issues here (CA 0x38,
//! 0x40, both with CS; not decoded), MR3, MR20, MR10, MR28, MR41, MR46,
//! MR17, MR11, MR13, MR22, MR25, MR12, MR14, MR15, MR24, MR30, MR19. Then,
//! on both channels and ranks at once, MR16 = 0x45 (FSP-OP 1) and two
//! timing launches. The CBT exit controls the reference writes before the
//! first launch are part of [`run`]; the reference's CA delay writes there
//! are not (ours come from our own CA training).
use super::coldinit::{mrw, Row};
use super::wcklevel::{Channel, Rank, Step};

/// The mode registers of one rank in the reference order (MA, OP); the
/// two-edge command sits after MR2.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct FspMrs {
    pub mr: [(u8, u8); 20],
}

/// The MAs in the reference order, after MR16.
pub const ORDER: [u8; 20] = [
    18, 1, 2, 3, 20, 10, 28, 41, 46, 17, 11, 13, 22, 25, 12, 14, 15, 24, 30, 19,
];

/// MR16 while writing the target FSP: FSP-WR 1, FSP-OP 0, VRCG.
pub const MR16_WRITE: u8 = 0x41;
/// MR16 switching the DRAM to the target FSP: FSP-WR 1, FSP-OP 1, VRCG.
pub const MR16_SWITCH: u8 = 0x45;

/// Message block words of the mode registers (rank 0 in the low byte, rank
/// 1 in the high byte; the word after each is channel B's copy). MR24, MR25
/// and MR30 are zero in every parameter set of this board; their words
/// follow the block's MR order and are not checked by a non-zero value.
mod mb {
    pub const MR1: usize = 0x2d;
    pub const MR2: usize = 0x2f;
    pub const MR10: usize = 0x33;
    pub const MR11: usize = 0x35;
    pub const MR12: usize = 0x37;
    pub const MR14: usize = 0x3b;
    pub const MR15: usize = 0x3d;
    pub const MR17: usize = 0x41;
    pub const MR18: usize = 0x43;
    pub const MR19: usize = 0x45;
    pub const MR20: usize = 0x47;
    pub const MR22: usize = 0x4b;
    pub const MR24: usize = 0x4d;
    pub const MR25: usize = 0x4f;
    pub const MR28: usize = 0x55;
    pub const MR30: usize = 0x57;
    pub const MR41: usize = 0x65;
}

impl FspMrs {
    /// The values for `channel`/`rank` from the message block words.
    /// MR3 and MR13 take the same rules as DevInit (`coldinit::MrInputs`),
    /// MR18 clears bit 4 (as WL's MR18 base), MR46 is 0; the others are the
    /// words as given (checked against the reference's MRWs).
    pub fn from_message_block(m: &[u16], channel: Channel, rank: Rank) -> FspMrs {
        let ch = matches!(channel, Channel::B) as usize;
        let shift = 8 * matches!(rank, Rank::R1) as u32;
        let b = |word: usize| (m[word + ch] >> shift) as u8;
        let i = super::coldinit::MrInputs(m);
        let r = matches!(rank, Rank::R1) as usize;
        let val = |ma: u8| -> u8 {
            match ma {
                1 => b(mb::MR1),
                2 => b(mb::MR2),
                3 => i.mr3(r),
                10 => b(mb::MR10),
                11 => b(mb::MR11),
                12 => b(mb::MR12),
                13 => i.mr13(r),
                14 => b(mb::MR14),
                15 => b(mb::MR15),
                17 => b(mb::MR17),
                18 => b(mb::MR18) & !0x10,
                19 => b(mb::MR19),
                20 => b(mb::MR20),
                22 => b(mb::MR22),
                24 => b(mb::MR24),
                25 => b(mb::MR25),
                28 => b(mb::MR28),
                30 => b(mb::MR30),
                41 => b(mb::MR41),
                _ => 0, // MR46
            }
        };
        let mut mr = [(0u8, 0u8); 20];
        for (slot, &ma) in mr.iter_mut().zip(ORDER.iter()) {
            *slot = (ma, val(ma));
        }
        FspMrs { mr }
    }
}

const fn cs(rank: Rank) -> u8 {
    match rank {
        Rank::R0 => 1,
        Rank::R1 => 2,
    }
}

const fn put(rows: &mut [Row; 142], at: usize, r: [Row; 4]) {
    rows[at] = r[0];
    rows[at + 1] = r[1];
    rows[at + 2] = r[2];
    rows[at + 3] = r[3];
}

/// The 142-row launch writing `m` to `rank` (CS `rank`), MR16 first.
pub fn program(rank: Rank, m: &FspMrs) -> [Row; 142] {
    let c = cs(rank);
    let mut rows = [Row::EMPTY; 142];
    rows[0] = Row::nop(0x4b00);
    put(&mut rows, 2, mrw(c, 16, MR16_WRITE));
    rows[6] = Row::nop(0x7b00);
    rows[7] = Row::nop(0x0800);
    rows[8] = Row::nop(0x5b00);
    let mut at = 10;
    for (k, &(ma, op)) in m.mr.iter().enumerate() {
        put(&mut rows, at, mrw(c, ma, op));
        rows[at + 4] = Row::nop(0x5b00);
        at += 6;
        if k == 2 {
            // After MR2: a timing row, then the reference's two-edge
            // command (CA 0x38, 0x40, CS on both rows).
            rows[at] = Row::nop(0x5b00);
            rows[at + 2] = Row::cmd(c, 0x38);
            rows[at + 3] = Row::cmd(c, 0x40);
            rows[at + 4] = Row::nop(0x5b00);
            at += 6;
        }
    }
    rows[136] = Row::nop(0x7b00);
    rows[137] = Row::nop(0x0400);
    rows[140] = Row::nop(0x1b00);
    rows[141] = Row::nop(0x0400);
    rows
}

/// MR16 = [`MR16_SWITCH`] to both ranks (CS 3) at once.
pub fn switch_rows() -> [Row; 12] {
    let mut rows = [Row::EMPTY; 12];
    rows[0] = Row::nop(0x4b00);
    let r = mrw(3, 16, MR16_SWITCH);
    rows[2..6].copy_from_slice(&r);
    rows[6] = Row::nop(0x7b00);
    rows[7] = Row::nop(0x0400);
    rows[10] = Row::nop(0x1b00);
    rows[11] = Row::nop(0x0400);
    rows
}

/// The reference's settling launch after the switch.
pub fn settle_rows() -> [Row; 12] {
    let mut rows = [Row::EMPTY; 12];
    rows[0] = Row::nop(0x4b00);
    rows[2] = Row::nop(0x7b00);
    rows[3] = Row::nop(0x0800);
    rows[4] = Row::nop(0x5b00);
    rows[6] = Row::nop(0x7b00);
    rows[7] = Row::nop(0x0400);
    rows[10] = Row::nop(0x1b00);
    rows[11] = Row::nop(0x0400);
    rows
}

/// The closing four-row delay launch.
pub fn close_rows() -> [Row; 4] {
    let mut rows = [Row::EMPTY; 4];
    rows[2] = Row::nop(0x2b00);
    rows
}

fn writes(values: &[(u32, u16)], out: &mut impl FnMut(Step<'_>)) {
    for &(c, v) in values {
        out(Step::Write(c, v));
    }
}

/// Stop the clocks of the channel the previous launch used.
fn clocks_stop(prev: Channel, out: &mut impl FnMut(Step<'_>)) {
    let (all_but, keep) = match prev {
        Channel::A => (0xfc, 0xf0),
        Channel::B => (0xcf, 0x0f),
    };
    writes(
        &[
            (0x2007e, 0),
            (0x2007a, all_but),
            (0x20042, 0),
            (0x20044, 0),
            (0x2007a, keep),
            (0x2007e, 0),
        ],
        out,
    );
}

/// Channel selection (`mask` for 0x2007a), DBYTE `0x1n000 = 8`, a PIE pulse.
fn select(a: u16, b: u16, masks: &[u16], out: &mut impl FnMut(Step<'_>)) {
    writes(&[(0x20072, a), (0x20073, b), (0x2007e, 0)], out);
    for &m in masks {
        out(Step::Write(0x2007a, m));
    }
    out(Step::Write(0x2000d, 8));
    for db in 0..5 {
        out(Step::Write(0x10000 + db * 0x1000, 8));
    }
    writes(&[(0x2f0a0, 1), (0x2f0a0, 0)], out);
}

/// Clocks on with `mask` afterwards.
fn clocks_start(mask: u16, out: &mut impl FnMut(Step<'_>)) {
    writes(
        &[
            (0x2007e, 0),
            (0x2007a, 0xff),
            (0x20042, 0x103),
            (0x20044, 1),
            (0x2007a, mask),
            (0x2007e, 0),
        ],
        out,
    );
}

/// The reference block order of this stage.
pub const BLOCKS: [(Channel, Rank); 4] = [
    (Channel::A, Rank::R0),
    (Channel::B, Rank::R0),
    (Channel::A, Rank::R1),
    (Channel::B, Rank::R1),
];

/// The CBT exit controls the reference writes before the first launch
/// (after its CA delays): PIE pulse, DBYTE CBT controls back.
pub fn cbt_exit(out: &mut impl FnMut(Step<'_>)) {
    writes(
        &[
            (0x2f0a0, 1),
            (0x2f0a0, 0),
            (0x1f08b, 0),
            (0x1f00b, 0),
            (0x1f005, 0),
            (0x1f094, 0),
            (0x1f00a, 1),
            (0x1f08a, 0),
            (0x1f089, 0),
            (0x1f08c, 0),
        ],
        out,
    );
}

/// The whole stage: per [`BLOCKS`] entry the clocks of the previous channel
/// stopped (`prev`: the channel CBT ended on; before the first block also
/// [`cbt_exit`]), the channel selected, clocks on and its program launched; then both channels
/// selected and the FSP switch, the settling launch and the closing launch.
/// `mrs(channel, rank)` gives each rank's values.
pub fn run(prev: Channel, mrs: impl Fn(Channel, Rank) -> FspMrs, out: &mut impl FnMut(Step<'_>)) {
    let mut last = prev;
    for (k, &(ch, rank)) in BLOCKS.iter().enumerate() {
        clocks_stop(last, out);
        if k == 0 {
            cbt_exit(out);
        }
        let (a, b) = ch.enables();
        select(a, b, &[ch.override_mask()], out);
        clocks_start(ch.override_mask(), out);
        let rows = program(rank, &mrs(ch, rank));
        out(Step::Launch {
            rows: &rows,
            outer: 0,
        });
        last = ch;
    }
    clocks_stop(last, out);
    select(3, 3, &[0x0e, 0x00], out);
    select(3, 3, &[0x00, 0x00], out);
    clocks_start(0, out);
    out(Step::Launch {
        rows: &switch_rows(),
        outer: 0,
    });
    writes(&[(0x20015, 0), (0x1f00b, 0), (0x1f005, 0)], out);
    out(Step::Launch {
        rows: &settle_rows(),
        outer: 0,
    });
    writes(
        &[
            (0x2007e, 0),
            (0x2007a, 0xcc),
            (0x20042, 0),
            (0x20044, 0),
            (0x2007a, 0),
            (0x2007e, 0),
        ],
        out,
    );
    out(Step::Launch {
        rows: &close_rows(),
        outer: 0,
    });
}
