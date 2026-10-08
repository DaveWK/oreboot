//! Original ACSM row generators for the A733 boot-pstate WCK2CK stage.
//!
//! The reference is derived register data from a06 pstate 1, parameter set 0,
//! PhyConfig 0x3a2/PUBREV 0x230. Oracle controls compare both ranks and channels.
//! Control words retain their observed meaning only; they are not qualified
//! for other pstates or parts. No vendor instructions or firmware bytes are
//! included. This module does not execute the programs or claim training.
//!
//! The hardware binding still needs the surrounding MR/control sequence,
//! bounded completion, and real DTSM feedback. The flat emulator's repeated
//! zero error counts and final delay zero are not a hardware search result.

use super::coldinit::Row;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Rank {
    R0,
    R1,
}

impl Rank {
    const fn cs(self) -> u8 {
        match self {
            Self::R0 => 1,
            Self::R1 => 2,
        }
    }

    const fn control(self) -> u16 {
        match self {
            Self::R0 => 0,
            Self::R1 => 0x10,
        }
    }
}

/// Observed short calibration body. The caller would set outer-loop count 8
/// initially and 255 on subsequent runs, matching the reference. No launch is
/// performed here, and repetition/timing are not independently qualified.
pub const fn calibration_program(rank: Rank) -> [Row; 26] {
    let mut rows = [Row::EMPTY; 26];
    let r = rank.control();
    rows[4] = Row::raw(0, 0x4000, 0, 0x0400 | r);
    rows[6] = Row::nop(0x0400 | r);
    rows[8] = Row::nop(0x4f00);
    rows[9] = Row::nop(0x1000);
    rows[10] = Row::raw(1, 3, 0, 0x0400);
    rows[12] = Row::nop(0x4f00);
    rows[13] = Row::nop(0x0800);
    rows[14] = Row::nop(0x1e80 | r);
    rows[16] = Row::nop(0x0500);
    rows[18] = Row::nop(0x0400 | r);
    rows[20] = Row::nop(0x6b00);
    rows[22] = Row::nop(r);
    rows
}

/// Hold a command edge for three observed pairs of ACSM rows. Every pair
/// repeats CA/CS and alternates control bit 10. Chip selects and rank control
/// use the existing original cold-init encoder.
const fn held_edge(rows: &mut [Row; 152], start: usize, rank: Rank, ca: u8, cs: bool) {
    let edge = Row::cmd(if cs { rank.cs() } else { 0 }, ca);
    let mut i = 0;
    while i < 6 {
        rows[start + i] = Row::raw(
            edge.0[0],
            0,
            0,
            edge.0[3] | if i % 2 == 0 { 0x0400 } else { 0 },
        );
        i += 1;
    }
}

/// Reference verification body: stretched MR18 edges, eight WCK pattern
/// periods and the DTSM control tail. This is a row encoder only. Its result
/// must never be treated as a measured WCK2CK phase or a trained delay.
pub const fn measurement_program(rank: Rank) -> [Row; 152] {
    let mut rows = [Row::EMPTY; 152];
    let r = rank.control();
    rows[0] = Row::nop(0x5f00);
    rows[1] = Row::nop(0x0c00);
    let mut i = 2;
    while i <= 10 {
        rows[i] = Row::nop(0x0400);
        i += 2;
    }
    held_edge(&mut rows, 12, rank, 0x58, true);
    held_edge(&mut rows, 18, rank, 18, true);
    held_edge(&mut rows, 24, rank, 0x48, true);
    held_edge(&mut rows, 30, rank, 0x48, false);
    rows[36] = Row::nop(0x0400);
    rows[38] = Row::nop(0x5f00);
    rows[39] = Row::nop(0x0c00);
    rows[40] = Row::nop(0x5e00);
    rows[42] = Row::nop(0x0500);
    // Eight periods, six rows each: three with hw1=0x2000, three=0x1000.
    i = 0;
    while i < 48 {
        let data = if i % 6 < 3 { 0x2000 } else { 0x1000 };
        rows[44 + i] = Row::raw(0, data, 0, r | if i % 2 == 0 { 0x0400 } else { 0 });
        i += 1;
    }
    rows[92] = Row::nop(0x0400);
    rows[94] = Row::nop(0x4f00);
    rows[95] = Row::nop(0x1400);
    rows[96] = Row::raw(0x81, 3, 0, 0x0400);
    rows[98] = Row::nop(0x4f00);
    rows[99] = Row::nop(0x0800);
    rows[100] = Row::raw(0, 3, 0, 0x0400);
    rows[102] = Row::nop(0x4f00);
    rows[103] = Row::nop(0x0800);
    rows[104] = Row::nop(0x1e80);
    rows[106] = Row::nop(0x0500);
    rows[108] = Row::nop(0x5f00);
    rows[109] = Row::nop(0x0c00);
    i = 110;
    while i <= 122 {
        rows[i] = Row::nop(0x0400);
        i += 2;
    }
    held_edge(&mut rows, 124, rank, 0x58, true);
    held_edge(&mut rows, 130, rank, 18, true);
    held_edge(&mut rows, 136, rank, 0x48, true);
    i = 142;
    while i <= 148 {
        rows[i] = Row::nop(0x0400);
        i += 2;
    }
    rows[150] = Row::nop(0x5f00);
    rows[151] = Row::nop(0x0c00);
    rows
}
