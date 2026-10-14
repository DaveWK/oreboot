//! Original ACSM row generators for the A733 boot-pstate WCK2CK stage.
//!
//! The reference is derived register data from a06 pstate 1, parameter set 0,
//! PhyConfig 0x3a2/PUBREV 0x230. Oracle controls compare both ranks and channels.
//! Control words retain their observed meaning only; they are not qualified
//! for other pstates or parts. No vendor instructions or firmware bytes are
//! included. This module does not execute the programs or claim training.
//!
//! Calibration entry and normal cleanup are described as control steps. The
//! hardware binding still needs bounded
//! completion, qualified register aliases, and real DTSM feedback. The flat emulator's repeated
//! zero error counts and final delay zero are not a hardware search result.

use super::coldinit::Row;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Rank {
    R0,
    R1,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Channel {
    A,
    B,
}

impl Channel {
    const fn enables(self) -> (u16, u16) {
        match self {
            Self::A => (3, 0),
            Self::B => (0, 3),
        }
    }

    const fn override_mask(self) -> u16 {
        match self {
            Self::A => 0xf0,
            Self::B => 0x0f,
        }
    }

    const fn inactive_first_byte(self) -> u32 {
        match self {
            Self::A => 2,
            Self::B => 0,
        }
    }
}

/// Describes a reference control action, without executing it. `Launch` uses
/// an explicit finite outer-loop count; it is not the CBT infinite-loop step.
/// Hardware binding must independently establish ownership and bounded
/// completion, and must not change an unknown register alias into a guess.
pub enum Step<'a> {
    Write(u32, u16),
    Launch { rows: &'a [Row], outer: u16 },
}

/// WL MR18 base, with bit 4 cleared and mode bits 7/6 supplied clear.
/// Derived from the selected part/pstate's original message-block input.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Mr18Base(u8);

impl Mr18Base {
    pub const fn value(self) -> u8 {
        self.0
    }

    pub const fn new(value: u8) -> Option<Self> {
        if value & 0xd0 != 0 {
            None
        } else {
            Some(Self(value))
        }
    }

    /// WL uses word 0x43/0x44 for channel A/B and its low/high byte for
    /// rank 0/1. Synthetic bit and isolated-byte probes establish clearing
    /// bit 4, preserving all other base bits. Reject preasserted mode bits.
    pub const fn from_message_block(
        words: &[u16; super::msgblock::WORDS],
        channel: Channel,
        rank: Rank,
    ) -> Option<Self> {
        let word = match channel {
            Channel::A => 0x43,
            Channel::B => 0x44,
        };
        let shift = match rank {
            Rank::R0 => 0,
            Rank::R1 => 8,
        };
        Self::new(((words[word] >> shift) as u8) & !0x10)
    }
}

fn writes(values: &[(u32, u16)], out: &mut impl FnMut(Step<'_>)) {
    for &(csr, value) in values {
        out(Step::Write(csr, value));
    }
}

/// Count-clear pulse observed for all nine DTSM lanes, including the extra
/// clear bit. Its semantics beyond this matching register-family observation
/// remain a prerequisite of the eventual hardware adapter.
pub fn clear_counts(out: &mut impl FnMut(Step<'_>)) {
    for value in [0, 0x1ff, 0x3ff, 0x1ff, 0] {
        out(Step::Write(0x1f0bb, value));
    }
}

/// MR18 in the observed two-edge encoding, with explicit rank control on
/// both rows. `base` is an input, not a guessed derivation from a parameter
/// set; the independent reference currently covers base=0x0c only.
fn mr18_rows(rank: Rank, value: u8, long_tail: bool, first_clock: bool) -> [Row; 6] {
    let mut rows = [Row::EMPTY; 6];
    let edges = super::coldinit::mrw_ddr(rank.cs(), 18, value);
    rows[0] = Row::raw(
        edges[0].0[0],
        0,
        0,
        rank.control() | if first_clock { 0x400 } else { 0 },
    );
    rows[1] = Row::raw(edges[1].0[0], 0, 0, rank.control());
    if long_tail {
        rows[2] = Row::nop(0x0b00);
        rows[3] = Row::nop(0x0800);
        rows[4] = Row::nop(0x0b00);
        rows[5] = Row::nop(0x1000);
    } else {
        rows[2] = Row::nop(0x7b00);
        rows[3] = Row::nop(0x0400);
    }
    rows
}

/// The first fifteen launches of one observed WL block, stopping immediately
/// after the calibration-mode MRW is restored. No synthetic TrainingCntr or
/// TxWckDly value is replayed. The caller must preserve the pre-block controls
/// first, then acquire the actual per-DBYTE TrainingCntr results after this
/// boundary. MR18 base bits 7/6 are supplied clear by the caller.
///
/// Exactly [`calibration_entry`] followed by [`calibration_loops`].
pub fn calibration_prefix(
    channel: Channel,
    rank: Rank,
    mr18_base: Mr18Base,
    out: &mut impl FnMut(Step<'_>),
) {
    calibration_entry(channel, rank, mr18_base, out);
    calibration_loops(channel, rank, mr18_base, out);
}

/// First part of [`calibration_prefix`]: channel selection, the first two
/// launches (MR18 base | 0x80) and the DTSM/training set-up, ending with the
/// TrainingCntr start value (`0x1ffaa = 0x2000`, i.e. 512) and TrainingParam
/// `0x300`. A caller may overwrite the start value per DBYTE/lane at the
/// base TrainingCntr addresses here, before [`calibration_loops`].
pub fn calibration_entry(
    channel: Channel,
    rank: Rank,
    mr18_base: Mr18Base,
    out: &mut impl FnMut(Step<'_>),
) {
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
            (0x70011, 0),
            (0x1f097, 0),
            (0x1f0b0, 8),
            (0x1f0b1, 0x40),
            (0x2f057, 0xaa),
        ],
        out,
    );
    let mut rows = [Row::EMPTY; 12];
    rows[0] = Row::raw((rank.cs() as u16) << 14, 0x4000, 1, rank.control());
    rows[6] = Row::nop(0x2b00);
    rows[8] = Row::raw((rank.cs() as u16) << 14, 0, 1, rank.control());
    out(Step::Launch {
        rows: &rows,
        outer: 0,
    });
    writes(
        &[
            (0x1f08a, 0xff),
            (0x1f089, 0xff),
            (0x1f08b, 0),
            (0x1f00a, 0),
            (0x1f005, 1),
            (0x1f094, 0),
            (0x1f00b, 2),
            (0x20041, 1),
            (0x2002a, 1),
            (0x2003f, 0x1014),
            (0x1f093, 0x200),
            (0x1f005, 1),
        ],
        out,
    );
    out(Step::Launch {
        rows: &mr18_rows(rank, mr18_base.0 | 0x80, true, false),
        outer: 0,
    });
    writes(
        &[
            (0x1ffb5, 0),
            (0x1f0b5, 1),
            (0x1f4b5, 1),
            (0x1f0b3, 0),
            (0x1f0b4, 0),
            (0x1f0b9, 1),
            (0x1f0ba, 1),
            (0x1f0b1, 2),
            (0x7f060, 1),
            (0x7f065, 0x1ff),
            (0x7ff28, 0xffff),
            (0x7ff29, 0xffff),
            (0x7ff2a, 0xffff),
            (0x7ff2b, 0xffff),
            (0x1ffbe, 0),
            (0x1f0be, 1),
            (0x1f4be, 0x10),
            (0x1ffaa, 0x2000),
            (0x1f0a7, 0x300),
        ],
        out,
    );
}

/// Second part of [`calibration_prefix`]: count clear, inactive-DBYTE DTSM
/// lane control, leveling entry (MR18 base | 0xc8), the PHY's own counter
/// search (outer loops 8, then 10 x 255) and the MR18 restore (base | 0x80).
pub fn calibration_loops(
    channel: Channel,
    rank: Rank,
    mr18_base: Mr18Base,
    out: &mut impl FnMut(Step<'_>),
) {
    clear_counts(out);
    for db in channel.inactive_first_byte()..channel.inactive_first_byte() + 2 {
        out(Step::Write(0x10fb5 + db * 0x1000, 0));
    }
    out(Step::Launch {
        rows: &mr18_rows(rank, mr18_base.0 | 0xc8, false, false),
        outer: 0,
    });
    out(Step::Write(0x1f0a7, 0x6300));
    let program = calibration_program(rank);
    out(Step::Launch {
        rows: &program,
        outer: 8,
    });
    out(Step::Write(0x1f0a7, 0x300));
    for _ in 0..10 {
        out(Step::Launch {
            rows: &program,
            outer: 255,
        });
    }
    out(Step::Launch {
        rows: &mr18_rows(rank, mr18_base.0 | 0x80, false, false),
        outer: 0,
    });
}

/// Start the observed verification clocks after the caller has supplied valid
/// per-DBYTE delays. The finite eight-row transition is followed by full-speed
/// ACSM with the observed WCK static pulse timing. No feedback is synthesized.
pub fn measurement_clocks_start(channel: Channel, out: &mut impl FnMut(Step<'_>)) {
    writes(
        &[
            (0x20079, 1),
            (0x20079, 0),
            (0x2007e, 0),
            (0x2007a, 0xff),
            (0x20042, 0x103),
            (0x20044, 1),
            (0x2007a, channel.override_mask()),
            (0x2007e, 0),
        ],
        out,
    );
    let mut rows = [Row::EMPTY; 8];
    for (i, control) in [
        (0, 0x4b00),
        (2, 0x7b00),
        (3, 0x0400),
        (6, 0x1b00),
        (7, 0x0400),
    ] {
        rows[i] = Row::nop(control);
    }
    out(Step::Launch {
        rows: &rows,
        outer: 0,
    });
    writes(
        &[
            (0x20044, 2),
            (0x2003e, 0x402),
            (0x2003d, 0x402),
            (0x20042, 0x183),
        ],
        out,
    );
}

/// Prepare DTSM for the first finite measurement. The caller must set and
/// read back its chosen per-rank delays before this boundary. Broadcast
/// writes here retain oracle addresses, still unqualified for hardware.
pub fn first_measurement(channel: Channel, rank: Rank, out: &mut impl FnMut(Step<'_>)) {
    writes(
        &[
            (0x20079, 1),
            (0x20079, 0),
            (0x1f0b9, 0xffff),
            (0x1f0ba, 0xffff),
            (0x1ffb5, 0),
        ],
        out,
    );
    let first = 2 - channel.inactive_first_byte();
    for db in first..first + 2 {
        out(Step::Write(0x10fb5 + db * 0x1000, 1));
    }
    writes(&[(0x7f060, 1), (0x1ffbe, 0)], out);
    measurement_once(rank, out);
}

/// One finite measurement after the caller has handled feedback and selected
/// delays. Repeated calls are not a training search: completion and actual
/// count/delay reads must be bound independently, with a caller-owned limit.
pub fn measurement_once(rank: Rank, out: &mut impl FnMut(Step<'_>)) {
    clear_counts(out);
    writes(&[(0x20079, 1), (0x20079, 0)], out);
    out(Step::Launch {
        rows: &measurement_program(rank),
        outer: 0,
    });
}

/// [`measurement_once`] without the count clear: two consecutive launches
/// then show whether DTSM counts accumulate (a counting/clear control).
pub fn measurement_once_no_clear(rank: Rank, out: &mut impl FnMut(Step<'_>)) {
    writes(&[(0x20079, 1), (0x20079, 0)], out);
    out(Step::Launch {
        rows: &measurement_program(rank),
        outer: 0,
    });
}

/// DtsmLaneCtrl0_iX of DBYTE `db`, lane `lane` (EIC7700X TRM Part 2,
/// `0x100b5 + 0x1000 * db + 0x100 * lane`).
pub const fn dtsm_lane_ctrl(db: u32, lane: u32) -> u32 {
    0x100b5 + 0x1000 * db + 0x100 * lane
}

/// DtsmCountClears of DBYTE `db` (`0x100bb + 0x1000 * db`).
pub const fn dtsm_count_clears(db: u32) -> u32 {
    0x100bb + 0x1000 * db
}

/// The reference count-clear pulse ([`clear_counts`]) at one DBYTE's base
/// DtsmCountClears address instead of the DBYTE-broadcast `0x1f0bb`.
pub fn clear_counts_dbyte(db: u32, out: &mut impl FnMut(Step<'_>)) {
    for value in [0, 0x1ff, 0x3ff, 0x1ff, 0] {
        out(Step::Write(dtsm_count_clears(db), value));
    }
}

/// Per-DBYTE base-address DTSM set-up for the channel's two DBYTEs: every
/// lane 0..8 gets DtsmLaneCtrl0 = `lane_ctrl` (the reference writes 1 to the
/// enabled lanes) and the count clear pulse, then one measurement launch.
/// Only documented base addresses are used.
pub fn measurement_dbyte_bases(
    dbytes: [u32; 2],
    rank: Rank,
    lane_ctrl: u16,
    out: &mut impl FnMut(Step<'_>),
) {
    for db in dbytes {
        for lane in 0..9 {
            out(Step::Write(dtsm_lane_ctrl(db, lane), lane_ctrl));
        }
    }
    for db in dbytes {
        clear_counts_dbyte(db, out);
    }
    measurement_once_no_clear(rank, out);
}

/// Stop the observed verification clocks before restoring delays and normal
/// cleanup. This is the normal path; it cannot recover an incomplete launch.
pub fn measurement_clocks_stop(channel: Channel, out: &mut impl FnMut(Step<'_>)) {
    let all_but_active = match channel {
        Channel::A => 0xfc,
        Channel::B => 0xcf,
    };
    writes(
        &[
            (0x20044, 1),
            (0x20042, 0x103),
            (0x2007e, 0),
            (0x2007a, all_but_active),
            (0x20042, 0),
            (0x20044, 0),
            (0x2007a, channel.override_mask()),
            (0x2007e, 0),
        ],
        out,
    );
}

/// Source CSRs saved by the reference before each block. Keep their actual
/// observed values: channel/rank boundaries do not all start from reset.
pub const SAVED_CSRS: [u32; 14] = [
    0x10097, 0x10093, 0x10005, 0x1008b, 0x10095, 0x20074, 0x20057, 0x2002a, 0x2003f, 0x20041,
    0x20037, 0x2003d, 0x2003e, 0x70011,
];

/// State captured before the block in `SAVED_CSRS` order. No fabricated default
/// is provided. The reference restores these via broadcast addresses; those
/// aliases are recorded here and still need qualification for hardware use.
pub struct SavedControls(pub [u16; 14]);

/// Supported TxWckDly range in the matching EIC7700X register schema:
/// coarse [10:6] is at most 20 UI; fine [5:0] is UI/64. This range check is
/// not proof of a valid analog result or of the A733 broadcast alias.
pub const MAX_WCK_DELAY: u16 = 20 * 64 + 63;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct TxWckDelay(u16);

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct UnsupportedCounter {
    pub raw: u16,
    pub decoded: u16,
}

impl TxWckDelay {
    /// Original bit-field conversion established by synthetic PMU oracle
    /// inputs: low ten bits stay, upper six shift down four, combined by OR
    /// (not addition). The independent fixture retains all sixteen input
    /// bits and mixed fields. No physical meaning is assigned to the raw
    /// TrainingCntr fields beyond that observed conversion.
    pub const fn from_training_counter(raw: u16) -> Result<Self, UnsupportedCounter> {
        let decoded = (raw & 0x03ff) | ((raw >> 4) & 0x0fc0);
        if decoded > MAX_WCK_DELAY {
            Err(UnsupportedCounter { raw, decoded })
        } else {
            Ok(Self(decoded))
        }
    }

    pub const fn encoded(self) -> u16 {
        self.0
    }

    /// A linear delay (UI/64) in the supported range.
    pub const fn new(delay: u16) -> Option<Self> {
        if delay > MAX_WCK_DELAY {
            None
        } else {
            Some(Self(delay))
        }
    }

    /// The raw TrainingCntr value that [`Self::from_training_counter`]
    /// converts back to this delay: coarse UI `[10:6]` moved to raw
    /// `[15:10]`, fine `[5:0]` kept (512 -> `0x2000`, 506 -> `0x1c3a`,
    /// 518 -> `0x2006`, as read on hardware Oct 14).
    pub const fn training_counter(self) -> u16 {
        ((self.0 & 0x0fc0) << 4) | (self.0 & 0x003f)
    }

    /// The observed first verification point is half a UI above the initial
    /// value. Reject a point outside the documented range, without wrapping
    /// or silently saturating it into a different measurement.
    pub const fn half_ui_later(self) -> Option<Self> {
        let delay = self.0 + 32;
        if delay > MAX_WCK_DELAY {
            None
        } else {
            Some(Self(delay))
        }
    }
}

/// Cleanup after the caller has stopped measurement clocks and decided what
/// to do with measured delay results. Does not accept or invent a trained
/// result, nor restore a guessed zero delay. The caller must retain recovery
/// on any incomplete launch; this generator is not an abort primitive.
pub fn cleanup(
    rank: Rank,
    mr18_base: Mr18Base,
    saved: &SavedControls,
    out: &mut impl FnMut(Step<'_>),
) {
    cleanup_sequence(rank, mr18_base, out);
    restore_saved(saved, out);
}

/// [`cleanup`] up to, not including, the restoration of the saved controls:
/// mode exits, the MR18 restore and the closing launches.
pub fn cleanup_sequence(rank: Rank, mr18_base: Mr18Base, out: &mut impl FnMut(Step<'_>)) {
    writes(
        &[
            (0x1f00b, 0),
            (0x1f005, 0),
            (0x1f094, 0),
            (0x1f00a, 1),
            (0x1f08a, 0),
            (0x1f089, 0),
            (0x1f08c, 0),
            (0x1f0b0, 0),
            (0x1f0b1, 0),
        ],
        out,
    );
    out(Step::Launch {
        rows: &[Row::nop(0x4b00), Row::EMPTY],
        outer: 0,
    });
    out(Step::Launch {
        rows: &mr18_rows(rank, mr18_base.0, true, true),
        outer: 0,
    });
    let mut rows = [Row::EMPTY; 12];
    rows[0] = Row::nop(0x0400);
    rows[4] = Row::raw(6, 4, 0, rank.control());
    rows[6] = Row::nop(0x4b00);
    rows[7] = Row::nop(0x1000);
    out(Step::Launch {
        rows: &rows,
        outer: 0,
    });
    writes(
        &[(0x20079, 1), (0x20079, 0), (0x2f0a0, 1), (0x2f0a0, 0)],
        out,
    );
}

/// The reference's restoration of [`SavedControls`] (its broadcast targets).
pub fn restore_saved(saved: &SavedControls, out: &mut impl FnMut(Step<'_>)) {
    for (target, index) in [
        (0x1f097, 0),
        (0x1f093, 1),
        (0x1f005, 2),
        (0x1f08b, 3),
        (0x1f095, 4),
        (0x2f074, 5),
        (0x2f057, 6),
        (0x2f02a, 7),
        (0x2f03f, 8),
        (0x2f041, 9),
        (0x2f037, 10),
        (0x2f03f, 8),
        (0x2f03d, 11),
        (0x2f03e, 12),
        (0x7f011, 13),
    ] {
        out(Step::Write(target, saved.0[index]));
    }
}

impl Rank {
    /// Chip-select mask of the rank in ACSM rows (bit 0 rank 0, bit 1 rank 1).
    pub(crate) const fn cs(self) -> u8 {
        match self {
            Self::R0 => 1,
            Self::R1 => 2,
        }
    }

    /// ACSM row control bit of the rank (0x10 on rank-1 command rows).
    pub(crate) const fn control(self) -> u16 {
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
