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
//! 0x40, both with CS: REFRESH all banks by the public LPDDR5 command table
//! as LiteDRAM 74522f7 transcribes it, `L L L H H H L | BA0-2 RFM SB0 V
//! AB`), MR3, MR20, MR10, MR28, MR41, MR46,
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
/// Fix candidate (FEL lab; not the default yet): the switch with
/// FSP-WR back to 0, FSP-OP 1, VRCG off: the MR16 the training firmware
/// leaves (read back on the board after its pstate-1 training). Later MRWs
/// (WCK2CK leveling's MR18) then go to the non-operating set. With the
/// reference's 0x45 the leveling's MR18 MRWs land in the operating set and
/// break the read path (measured).
pub const MR16_SWITCH_FSPWR0: u8 = 0x04;
/// [`MR16_SWITCH_FSPWR0`] with VRCG left on (also passed on the board).
pub const MR16_SWITCH_FSPWR0_VRCG: u8 = 0x44;

/// FEL lab bisect, measurement only: the parts of [`run_with`],
/// one bit each. A set bit leaves that part out wherever it occurs; the
/// rest keeps the reference order. A part that only sends rows to the DRAM
/// keeps its launch, with rows without a command in place of its own.
pub mod skip {
    /// [`super::cbt_exit`]: PIE pulse 0x2f0a0, DBYTE broadcast 0x1f08b,
    /// 0x1f00b, 0x1f005, 0x1f094 = 0, 0x1f00a = 1, 0x1f08a, 0x1f089, 0x1f08c
    /// = 0 (before the first block).
    pub const CBT_EXIT: u32 = 1 << 0;
    /// `clocks_stop`: 0x2007e/0x2007a/0x20042 = 0/0x20044 = 0 (before each
    /// block and after the last).
    pub const CLOCKS_STOP: u32 = 1 << 1;
    /// `select`, channel part: 0x20072/0x20073 enables, 0x2007e, 0x2007a
    /// masks (every select: per block and the two before the switch).
    pub const SELECT_CHANNEL: u32 = 1 << 2;
    /// `select`, DBYTE part: 0x2000d = 8, DBYTE 0-4 0x1n000 = 8.
    pub const SELECT_DBYTE: u32 = 1 << 3;
    /// `select`, the PIE pulse 0x2f0a0 = 1, 0.
    pub const SELECT_PULSE: u32 = 1 << 4;
    /// `clocks_start`: 0x2007a = 0xff, 0x20042 = 0x103, 0x20044 = 1, mask
    /// (per block and before the switch).
    pub const CLOCKS_START: u32 = 1 << 5;
    /// The MR16 = `mr16_write` MRW at the start of each per-rank launch.
    pub const MR16_WRITE: u32 = 1 << 6;
    /// The two-edge command after MR2 (CA 0x38/0x40, REFRESH all banks).
    pub const REFAB: u32 = 1 << 7;
    /// The four per-rank launches as a whole (MR16, the MRWs, the REFRESH).
    pub const RANK_LAUNCH: u32 = 1 << 8;
    /// The switch launch (MR16 = `mr16_switch` on both ranks).
    pub const SWITCH: u32 = 1 << 9;
    /// The writes after the switch: 0x20015, 0x1f00b, 0x1f005 = 0.
    pub const POST_SWITCH: u32 = 1 << 10;
    /// The settling launch (timing rows only).
    pub const SETTLE: u32 = 1 << 11;
    /// The closing clock writes (0x2007a = 0xcc, 0x20042/0x20044 = 0, ...).
    pub const CLOSE_CLOCKS: u32 = 1 << 12;
    /// The closing four-row delay launch.
    pub const CLOSE: u32 = 1 << 13;
    /// Every part.
    pub const ALL: u32 = (1 << 14) - 1;
    /// Every ACSM launch: what reaches the DRAM.
    pub const LAUNCHES: u32 = RANK_LAUNCH | SWITCH | SETTLE | CLOSE;
    /// Every PHY CSR write outside the launches.
    pub const PHY_CSRS: u32 = CBT_EXIT
        | CLOCKS_STOP
        | SELECT_CHANNEL
        | SELECT_DBYTE
        | SELECT_PULSE
        | CLOCKS_START
        | POST_SWITCH
        | CLOSE_CLOCKS;
}

/// A bisect variant of this stage (FEL lab, measurement only): parts left
/// out, MRWs left out of the per-rank launches, and the two MR16 values.
/// [`Variant::REFERENCE`] is the reference stage ([`run`]).
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Variant {
    /// [`skip`] bits.
    pub skip: u32,
    /// MAs whose MRW the per-rank launches leave out (bit n: MA n).
    pub no_mrw: u64,
    /// MR16 at the start of each per-rank launch ([`MR16_WRITE`]).
    pub mr16_write: u8,
    /// MR16 of the switch launch ([`MR16_SWITCH`]).
    pub mr16_switch: u8,
}

const fn hex_digit(c: u8) -> Option<u32> {
    match c {
        b'0'..=b'9' => Some((c - b'0') as u32),
        b'a'..=b'f' => Some((c - b'a' + 10) as u32),
        b'A'..=b'F' => Some((c - b'A' + 10) as u32),
        _ => None,
    }
}

/// `s[from..to]` as a number in `radix` (10 or 16), at most `max`; an
/// optional `0x` prefix in radix 16.
const fn number(s: &[u8], mut from: usize, to: usize, radix: u32, max: u64) -> Option<u64> {
    if radix == 16 && to >= from + 2 && s[from] == b'0' && (s[from + 1] | 0x20) == b'x' {
        from += 2;
    }
    if from >= to {
        return None;
    }
    let mut v: u64 = 0;
    let mut i = from;
    while i < to {
        let d = match hex_digit(s[i]) {
            Some(d) if d < radix => d as u64,
            _ => return None,
        };
        v = v * radix as u64 + d;
        if v > max {
            return None;
        }
        i += 1;
    }
    Some(v)
}

impl Variant {
    pub const REFERENCE: Variant = Variant {
        skip: 0,
        no_mrw: 0,
        mr16_write: MR16_WRITE,
        mr16_switch: MR16_SWITCH,
    };

    pub const fn skips(&self, part: u32) -> bool {
        self.skip & part != 0
    }

    pub const fn no_mrw_of(&self, ma: u8) -> bool {
        ma < 64 && self.no_mrw & (1 << ma) != 0
    }

    /// `self` with the switch's MR16 from the fix knob `fspwr0` (env
    /// `A7A_EI_FSPWR0`: `04` = [`MR16_SWITCH_FSPWR0`], `44` =
    /// [`MR16_SWITCH_FSPWR0_VRCG`]); `None` for any other text, or when the
    /// switch value was already changed (`A7A_HYBRID_EI_MR16`).
    pub const fn with_fspwr0(self, fspwr0: Option<&str>) -> Option<Variant> {
        let Some(s) = fspwr0 else {
            return Some(self);
        };
        let b = s.as_bytes();
        let v = if b.len() == 2 && b[0] == b'0' && b[1] == b'4' {
            MR16_SWITCH_FSPWR0
        } else if b.len() == 2 && b[0] == b'4' && b[1] == b'4' {
            MR16_SWITCH_FSPWR0_VRCG
        } else {
            return None;
        };
        if self.mr16_switch != MR16_SWITCH {
            return None;
        }
        Some(Variant {
            mr16_switch: v,
            ..self
        })
    }

    pub const fn is_reference(&self) -> bool {
        self.skip == 0
            && self.no_mrw == 0
            && self.mr16_write == MR16_WRITE
            && self.mr16_switch == MR16_SWITCH
    }

    /// The variant the build-time knobs name (`None`: malformed):
    /// - `skip`: [`skip`] bits, hex (`A7A_HYBRID_EI_SKIP`);
    /// - `no_mrw`: MAs whose MRW to leave out, decimal, comma-separated,
    ///   or `all` for every MA of [`ORDER`] (`A7A_HYBRID_EI_NO_MRW`);
    /// - `mr16`: `write,switch` in hex; either may be empty to keep the
    ///   reference's (`A7A_HYBRID_EI_MR16`, e.g. `45,` or `,05`).
    ///
    /// Unset knobs keep the reference.
    pub const fn parse(
        skip: Option<&str>,
        no_mrw: Option<&str>,
        mr16: Option<&str>,
    ) -> Option<Variant> {
        let mut v = Variant::REFERENCE;
        if let Some(s) = skip {
            let b = s.as_bytes();
            match number(b, 0, b.len(), 16, skip::ALL as u64) {
                Some(n) => v.skip = n as u32,
                None => return None,
            }
        }
        if let Some(s) = no_mrw {
            let b = s.as_bytes();
            if b.len() == 3 && b[0] == b'a' && b[1] == b'l' && b[2] == b'l' {
                let mut k = 0;
                while k < ORDER.len() {
                    v.no_mrw |= 1 << ORDER[k];
                    k += 1;
                }
            } else {
                let mut from = 0;
                while from < b.len() {
                    let mut to = from;
                    while to < b.len() && b[to] != b',' {
                        to += 1;
                    }
                    match number(b, from, to, 10, 63) {
                        Some(ma) => v.no_mrw |= 1 << ma,
                        None => return None,
                    }
                    from = to + 1;
                }
            }
        }
        if let Some(s) = mr16 {
            let b = s.as_bytes();
            let mut comma = 0;
            while comma < b.len() && b[comma] != b',' {
                comma += 1;
            }
            if comma == b.len() {
                return None;
            }
            if comma > 0 {
                match number(b, 0, comma, 16, 0xff) {
                    Some(n) => v.mr16_write = n as u8,
                    None => return None,
                }
            }
            if comma + 1 < b.len() {
                match number(b, comma + 1, b.len(), 16, 0xff) {
                    Some(n) => v.mr16_switch = n as u8,
                    None => return None,
                }
            }
        }
        Some(v)
    }
}

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
    program_with(rank, m, &Variant::REFERENCE)
}

/// [`program`] as `v` changes it: MR16 = `v.mr16_write`; the MR16 MRW
/// ([`skip::MR16_WRITE`]), the two-edge command ([`skip::REFAB`]) and the
/// MRWs of the MAs in `v.no_mrw` left as rows without a command (CS low,
/// timing rows unchanged).
pub fn program_with(rank: Rank, m: &FspMrs, v: &Variant) -> [Row; 142] {
    let c = cs(rank);
    let mut rows = [Row::EMPTY; 142];
    rows[0] = Row::nop(0x4b00);
    if !v.skips(skip::MR16_WRITE) {
        put(&mut rows, 2, mrw(c, 16, v.mr16_write));
    }
    rows[6] = Row::nop(0x7b00);
    rows[7] = Row::nop(0x0800);
    rows[8] = Row::nop(0x5b00);
    let mut at = 10;
    for (k, &(ma, op)) in m.mr.iter().enumerate() {
        if !v.no_mrw_of(ma) {
            put(&mut rows, at, mrw(c, ma, op));
        }
        rows[at + 4] = Row::nop(0x5b00);
        at += 6;
        if k == 2 {
            // After MR2: a timing row, then the reference's two-edge
            // command (CA 0x38, 0x40, CS on both rows).
            rows[at] = Row::nop(0x5b00);
            if !v.skips(skip::REFAB) {
                rows[at + 2] = Row::cmd(c, 0x38);
                rows[at + 3] = Row::cmd(c, 0x40);
            }
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
    switch_rows_with(MR16_SWITCH)
}

/// [`switch_rows`] with MR16 = `mr16`.
pub fn switch_rows_with(mr16: u8) -> [Row; 12] {
    let mut rows = [Row::EMPTY; 12];
    rows[0] = Row::nop(0x4b00);
    let r = mrw(3, 16, mr16);
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
/// Its delay row is `0x3b00` at pstate 0 (REFERENCE-DERIVED; a row without F10, which runs once on this PHY whatever
/// its count, measured).
pub fn close_rows() -> [Row; 4] {
    let mut rows = [Row::EMPTY; 4];
    rows[2] = Row::nop(if crate::pstate::ACTIVE == 0 {
        0x3b00
    } else {
        0x2b00
    });
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

/// Channel selection (`mask` for 0x2007a), DBYTE `0x1n000 = 8`, a PIE pulse;
/// `v` leaves out the parts named by [`skip::SELECT_CHANNEL`],
/// [`skip::SELECT_DBYTE`] and [`skip::SELECT_PULSE`].
fn select(a: u16, b: u16, masks: &[u16], v: &Variant, out: &mut impl FnMut(Step<'_>)) {
    if !v.skips(skip::SELECT_CHANNEL) {
        writes(&[(0x20072, a), (0x20073, b), (0x2007e, 0)], out);
        for &m in masks {
            out(Step::Write(0x2007a, m));
        }
    }
    if !v.skips(skip::SELECT_DBYTE) {
        out(Step::Write(0x2000d, 8));
        for db in 0..5 {
            out(Step::Write(0x10000 + db * 0x1000, 8));
        }
    }
    if !v.skips(skip::SELECT_PULSE) {
        writes(&[(0x2f0a0, 1), (0x2f0a0, 0)], out);
    }
}

/// Clocks on with `mask` afterwards.
fn clocks_start(mask: u16, out: &mut impl FnMut(Step<'_>)) {
    writes(
        &[
            (0x2007e, 0),
            (0x2007a, 0xff),
            (0x20042, crate::pstate::ACTIVE_LOW_SPEED.enable),
            (0x20044, crate::pstate::ACTIVE_LOW_SPEED.delay),
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
    run_with(prev, mrs, &Variant::REFERENCE, out)
}

/// [`run`] as `v` changes it (FEL lab bisect, measurement only): every part
/// whose [`skip`] bit is set is left out wherever it occurs, the rest stays
/// in the reference order. `Variant::REFERENCE` gives [`run`].
pub fn run_with(
    prev: Channel,
    mrs: impl Fn(Channel, Rank) -> FspMrs,
    v: &Variant,
    out: &mut impl FnMut(Step<'_>),
) {
    let mut last = prev;
    for (k, &(ch, rank)) in BLOCKS.iter().enumerate() {
        if !v.skips(skip::CLOCKS_STOP) {
            clocks_stop(last, out);
        }
        if k == 0 && !v.skips(skip::CBT_EXIT) {
            cbt_exit(out);
        }
        let (a, b) = ch.enables();
        select(a, b, &[ch.override_mask()], v, out);
        if !v.skips(skip::CLOCKS_START) {
            clocks_start(ch.override_mask(), out);
        }
        if !v.skips(skip::RANK_LAUNCH) {
            let rows = program_with(rank, &mrs(ch, rank), v);
            out(Step::Launch {
                rows: &rows,
                outer: 0,
            });
        }
        last = ch;
    }
    if !v.skips(skip::CLOCKS_STOP) {
        clocks_stop(last, out);
    }
    select(3, 3, &[0x0e, 0x00], v, out);
    select(3, 3, &[0x00, 0x00], v, out);
    if !v.skips(skip::CLOCKS_START) {
        clocks_start(0, out);
    }
    if !v.skips(skip::SWITCH) {
        out(Step::Launch {
            rows: &switch_rows_with(v.mr16_switch),
            outer: 0,
        });
    }
    if !v.skips(skip::POST_SWITCH) {
        writes(&[(0x20015, 0), (0x1f00b, 0), (0x1f005, 0)], out);
    }
    if !v.skips(skip::SETTLE) {
        out(Step::Launch {
            rows: &settle_rows(),
            outer: 0,
        });
    }
    if !v.skips(skip::CLOSE_CLOCKS) {
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
    }
    if !v.skips(skip::CLOSE) {
        out(Step::Launch {
            rows: &close_rows(),
            outer: 0,
        });
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use std::vec::Vec;

    /// One step of the stage, owned: a write, or a launch's rows.
    #[derive(Clone, Debug, PartialEq, Eq)]
    enum S {
        W(u32, u16),
        L(Vec<[u16; 4]>),
    }

    fn mrs(ch: Channel, rank: Rank) -> FspMrs {
        let mut mr = [(0, 0); 20];
        for (k, (slot, &ma)) in mr.iter_mut().zip(ORDER.iter()).enumerate() {
            *slot = (
                ma,
                0x10 + k as u8 + 0x40 * (ch == Channel::B) as u8 + 0x20 * (rank == Rank::R1) as u8,
            );
        }
        FspMrs { mr }
    }

    fn steps(v: Option<&Variant>) -> Vec<S> {
        let mut out = Vec::new();
        let mut f = |s: Step<'_>| match s {
            Step::Write(c, x) => out.push(S::W(c, x)),
            Step::Launch { rows, outer } => {
                assert_eq!(outer, 0);
                out.push(S::L(rows.iter().map(|r| r.0).collect()))
            }
        };
        match v {
            None => run(Channel::B, mrs, &mut f),
            Some(v) => run_with(Channel::B, mrs, v, &mut f),
        }
        out
    }

    fn launches(s: &[S]) -> Vec<Vec<[u16; 4]>> {
        s.iter()
            .filter_map(|x| match x {
                S::L(r) => Some(r.clone()),
                _ => None,
            })
            .collect()
    }

    fn writes(s: &[S]) -> Vec<(u32, u16)> {
        s.iter()
            .filter_map(|x| match *x {
                S::W(c, v) => Some((c, v)),
                _ => None,
            })
            .collect()
    }

    fn with_skip(skip: u32) -> Variant {
        Variant {
            skip,
            ..Variant::REFERENCE
        }
    }

    #[test]
    fn reference_variant_is_the_reference_stage() {
        assert_eq!(steps(Some(&Variant::REFERENCE)), steps(None));
        assert_eq!(Variant::parse(None, None, None), Some(Variant::REFERENCE));
        assert!(Variant::REFERENCE.is_reference());
        let r = steps(None);
        assert_eq!(launches(&r).len(), 7);
        assert_eq!(writes(&r).len(), 153);
    }

    #[test]
    fn launch_bits_drop_only_launches() {
        let r = steps(None);
        let l = launches(&r);
        let all = steps(Some(&with_skip(skip::LAUNCHES)));
        assert!(launches(&all).is_empty());
        assert_eq!(writes(&all), writes(&r));
        for (bit, kept) in [
            (skip::RANK_LAUNCH, &l[4..]),
            (skip::SWITCH, &[]),
            (skip::SETTLE, &[]),
            (skip::CLOSE, &[]),
        ] {
            let s = steps(Some(&with_skip(bit)));
            assert_eq!(writes(&s), writes(&r), "bit {bit:x}");
            assert_eq!(
                launches(&s).len(),
                if bit == skip::RANK_LAUNCH { 3 } else { 6 }
            );
            if bit == skip::RANK_LAUNCH {
                assert_eq!(launches(&s), kept);
            }
        }
        // The switch launch is the fifth: without it the settle and close follow.
        let s = steps(Some(&with_skip(skip::SWITCH)));
        let mut want = l.clone();
        want.remove(4);
        assert_eq!(launches(&s), want);
    }

    #[test]
    fn each_phy_csr_bit_drops_its_own_writes_and_together_all() {
        let r = steps(None);
        let total = writes(&r).len();
        let mut dropped = 0;
        let mut bit = 1;
        while bit <= skip::ALL {
            let s = steps(Some(&with_skip(bit)));
            let n = total - writes(&s).len();
            assert_eq!(n > 0, skip::PHY_CSRS & bit != 0, "bit {bit:x} drops {n}");
            if skip::PHY_CSRS & bit == 0 {
                assert_eq!(writes(&s), writes(&r), "bit {bit:x}");
            } else {
                assert_eq!(launches(&s), launches(&r), "bit {bit:x}");
            }
            dropped += n;
            bit <<= 1;
        }
        assert_eq!(dropped, total);
        let none = steps(Some(&with_skip(skip::PHY_CSRS)));
        assert!(writes(&none).is_empty());
        assert_eq!(launches(&none), launches(&r));
        // Counts per part, in the order of the skip bits.
        let count = |b| total - writes(&steps(Some(&with_skip(b)))).len();
        assert_eq!(count(skip::CBT_EXIT), 10);
        assert_eq!(count(skip::CLOCKS_STOP), 5 * 6);
        assert_eq!(count(skip::SELECT_CHANNEL), 4 * 4 + 2 * 5);
        assert_eq!(count(skip::SELECT_DBYTE), 6 * 6);
        assert_eq!(count(skip::SELECT_PULSE), 6 * 2);
        assert_eq!(count(skip::CLOCKS_START), 5 * 6);
        assert_eq!(count(skip::POST_SWITCH), 3);
        assert_eq!(count(skip::CLOSE_CLOCKS), 6);
    }

    #[test]
    fn row_bits_blank_their_rows_and_keep_the_timing() {
        let m = mrs(Channel::A, Rank::R1);
        let reference = program(Rank::R1, &m);
        let diff = |v: &Variant| -> Vec<usize> {
            let p = program_with(Rank::R1, &m, v);
            for (r, q) in reference.iter().zip(p.iter()) {
                assert!(r == q || *q == Row::EMPTY);
            }
            (0..142).filter(|&i| reference[i] != p[i]).collect()
        };
        assert_eq!(diff(&with_skip(skip::MR16_WRITE)), [2, 3, 4, 5]);
        // MR2 is the third MRW (rows 22-25); the REFRESH follows at 30/31.
        assert_eq!(diff(&with_skip(skip::REFAB)), [30, 31]);
        assert_eq!(reference[30], Row::cmd(2, 0x38));
        assert_eq!(reference[31], Row::cmd(2, 0x40));
        let v = Variant::parse(None, Some("2"), None).unwrap();
        assert_eq!(diff(&v), [22, 23, 24, 25]);
        let v = Variant::parse(None, Some("18,19"), None).unwrap();
        assert_eq!(diff(&v), [10, 11, 12, 13, 130, 131, 132, 133]);
        let all = Variant::parse(None, Some("all"), None).unwrap();
        assert_eq!(diff(&all).len(), 20 * 4);
        let p = program_with(Rank::R1, &m, &all);
        assert_eq!(&p[2..6], &mrw(2, 16, MR16_WRITE));
        assert_eq!(p[30], reference[30]);
        // MR16 values.
        let v = Variant::parse(None, None, Some("45,05")).unwrap();
        assert_eq!(&program_with(Rank::R1, &m, &v)[2..6], &mrw(2, 16, 0x45));
        let s = steps(Some(&v));
        assert_eq!(launches(&s)[4][2..6], mrw(3, 16, 0x05).map(|r| r.0));
        assert_eq!(
            launches(&steps(None))[4][2..6],
            mrw(3, 16, 0x45).map(|r| r.0)
        );
    }

    #[test]
    fn fspwr0_fix_knob() {
        let r = Variant::REFERENCE;
        assert_eq!(r.with_fspwr0(None), Some(r));
        assert_eq!(r.with_fspwr0(Some("04")).unwrap().mr16_switch, 0x04);
        assert_eq!(r.with_fspwr0(Some("44")).unwrap().mr16_switch, 0x44);
        assert_eq!(r.with_fspwr0(Some("45")), None);
        assert_eq!(r.with_fspwr0(Some("4")), None);
        let m = Variant::parse(None, None, Some(",05")).unwrap();
        assert_eq!(m.with_fspwr0(Some("04")), None);
        let w = Variant::parse(None, None, Some("45,")).unwrap();
        assert_eq!(w.with_fspwr0(Some("04")).unwrap().mr16_write, 0x45);
        // FSP-WR 0, FSP-OP 1 (jedec-lpddr's MR16 layout).
        let d = jedec_lpddr::lp5::mr::Mr16::decode(MR16_SWITCH_FSPWR0);
        assert_eq!((d.fsp_wr, d.fsp_op, d.vrcg), (0, 1, false));
        let d = jedec_lpddr::lp5::mr::Mr16::decode(MR16_SWITCH_FSPWR0_VRCG);
        assert_eq!((d.fsp_wr, d.fsp_op, d.vrcg), (0, 1, true));
    }

    #[test]
    fn knob_text() {
        let p = Variant::parse;
        assert_eq!(p(Some("2b00"), None, None).unwrap().skip, skip::LAUNCHES);
        assert_eq!(p(Some("0x3fff"), None, None).unwrap().skip, skip::ALL);
        assert_eq!(p(Some("4000"), None, None), None);
        assert_eq!(p(Some(""), None, None), None);
        assert_eq!(p(Some("g"), None, None), None);
        assert_eq!(
            p(None, Some("24,25,30,46"), None).unwrap().no_mrw,
            1 << 24 | 1 << 25 | 1 << 30 | 1 << 46
        );
        assert_eq!(p(None, Some("64"), None), None);
        assert_eq!(p(None, Some("1,,2"), None), None);
        assert_eq!(p(None, Some("x"), None), None);
        let v = p(None, None, Some("45,")).unwrap();
        assert_eq!((v.mr16_write, v.mr16_switch), (0x45, MR16_SWITCH));
        let v = p(None, None, Some(",44")).unwrap();
        assert_eq!((v.mr16_write, v.mr16_switch), (MR16_WRITE, 0x44));
        let v = p(None, None, Some(",")).unwrap();
        assert!(v.is_reference());
        assert_eq!(p(None, None, Some("45")), None);
        assert_eq!(p(None, None, Some("145,")), None);
        assert_eq!(
            skip::LAUNCHES | skip::PHY_CSRS | skip::MR16_WRITE | skip::REFAB,
            skip::ALL
        );
        assert_eq!(skip::LAUNCHES & skip::PHY_CSRS, 0);
    }
}
