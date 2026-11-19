// SPDX-License-Identifier: GPL-2.0-only
//! Write DQ centring and write Vref (FEL lab; measurement
//! stage, not in any default flow).
//!
//! **Where it runs.** On a working read path in mission mode: first the
//! hybrid (the PMU trained the pstate, our lean hand-over), later the
//! blobless flow once its reads work. The compare path is **the
//! controller**: the CPU writes a pattern to small probe blocks through the
//! controller and reads it back ([`WritePort::check`]). It needs no ACSM
//! write/read rows (none of ours exist, and the PHY's DTSM compare that the
//! read training uses only sees reads), and the controller keeps refreshing
//! the DRAM the whole time (normal mission operation), also during the Vref
//! sweep. A PHY-side route (write FIFO WFF / read FIFO RFF commands with the
//! DTSM compare, for use before the hand-over) is a later option; it would
//! have to issue its own REFRESH rows during long sweeps.
//!
//! **Registers** (EIC7700X SoC TRM Part 2, public, same DWC LPDDR5 PHY
//! family; sha256 955c99f0... of the v1.0.0-20250103 release PDF):
//! TxDqDlyTg0_rY_pX at 0x10026, TxDqDlyTg1_rY_pX at 0x10027, 9 bits,
//! [8:6] coarse (UI) and [5:0] fine (UI/64), i.e. the raw value is linear in
//! UI/64; reset 0x20; lane Y (0-7 DQ, 8 DMI) at +0x100 * Y, DBYTE n at
//! +0x1000 * n; Timing Group 0/1 = rank 0/1; replicated per pstate. (The
//! TRM's field text also names block offsets 0x0C0/0x0C1; the register
//! list's 0x10026/0x10027 is the one that matches our dumps: reset 0x20 in
//! the untrained blobless state, 0x68-0xa8 after the PMU's training.)
//!
//! **Method** (public, cited, not copied):
//! - the bit-to-lane and block-to-rank/channel maps are found by measurement
//!   ([`discover`]): one lane at a time is delayed by one UI, which shifts
//!   its data by a beat, and the failing data bits and blocks name it;
//! - every DQ lane has its own delay, so moving all lanes together over the
//!   range gives each lane its own pass window (per-bit windows; the strobe
//!   side, WCK, is fixed by WCK2CK leveling, so `dram_train::deskew::
//!   per_bit`'s strobe pushing is not needed); the setting is the centre of
//!   the lane's longest pass run (`dram_train::window`), coarse then fine
//!   (AMD PG313 p342-344 per-bit write deskew, PG353 p265-266 centring by
//!   margins);
//! - write Vref: MR14 (lower byte) and MR15 (upper byte) per channel and
//!   rank are swept, the write window is re-centred at every point, and the
//!   code is chosen on the per-byte worst-lane width profile: the middle of
//!   the eye at half height (`dram_train::eye::center`), with the largest
//!   worst-lane width (the max-min metric of coreboot MT8183/MT8195
//!   `TxChooseVref`) reported beside it;
//! - mode registers go to the set-point MR16 FSP-WR selects: a Vref MRW
//!   first points FSP-WR at the operating set (FSP-OP), then restores MR16
//!   ([`write_operating`]). The end-of-init bisect showed what happens otherwise.
//! - The DMI lane (8) carries no data in this compare (write DBI off, MR3
//!   OP[7] = 0): it gets the median of its DBYTE's DQ lanes; a data-mask
//!   check is a separate step.

use dram_train::window::{longest_run_linear, Window};

pub const DBYTES: usize = 4;
pub const LANES: usize = 9;
pub const DMI_LANE: usize = 8;
/// Probe blocks per check.
pub const BLOCKS: usize = 16;
/// TxDqDly range (9 bits) and one UI in its units.
pub const TX_DQ_MAX: u16 = 0x1ff;
pub const UI: u16 = 64;
/// Points per sweep (pass maps are `u128`).
pub const MAX_POINTS: u32 = 128;

/// TxDqDly CSR of DBYTE `db`, lane `lane`, timing group (rank) `tg`.
pub const fn tx_dq_csr(db: usize, lane: usize, tg: usize) -> u32 {
    0x10026 + 0x1000 * db as u32 + 0x100 * lane as u32 + tg as u32
}

/// Per probe block the OR of `got ^ want` over its 32-bit words.
pub type Folds = [u32; BLOCKS];
/// TxDqDly per DBYTE and lane.
pub type Delays = [[u16; LANES]; DBYTES];

/// The hardware a write stage needs.
pub trait WritePort {
    type Error;
    fn set_tx_dq(&mut self, db: usize, lane: usize, tg: usize, v: u16) -> Result<(), Self::Error>;
    fn tx_dq(&mut self, db: usize, lane: usize, tg: usize) -> Result<u16, Self::Error>;
    /// Write a pattern from `seed` to every probe block through the
    /// controller, read it back, return the [`Folds`]. A new seed every
    /// call keeps a write that never landed from passing on old data.
    fn check(&mut self, seed: u32) -> Result<Folds, Self::Error>;
}

/// Mode register access through the controller (mission mode).
pub trait MrPort {
    type Error;
    fn mrw(&mut self, ch: usize, rank: usize, ma: u8, op: u8) -> Result<(), Self::Error>;
    fn mrr(&mut self, ch: usize, rank: usize, ma: u8) -> Result<u8, Self::Error>;
    /// Wait after a Vref change before the next data access.
    fn settle(&mut self);
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WrError<E> {
    Port(E),
    /// The starting state does not write and read back cleanly (blocks).
    Baseline(u32),
    /// One block reacts to both timing groups.
    BlockTg(usize),
    /// A DQ lane (db, lane) moved no data bit.
    LaneUnseen(usize, usize),
    /// Two lanes of one channel share data bits (db, lane, other db).
    LaneOverlap(usize, usize, usize),
    /// The applied result does not read back cleanly (blocks).
    Verify(u32),
    /// A sweep with more than [`MAX_POINTS`] points or a zero step.
    Config,
}

/// What [`discover`] found.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Map {
    /// Timing group (rank) of each block; `None`: no reaction seen.
    pub block_tg: [Option<u8>; BLOCKS],
    /// DBYTEs whose lanes change each block's data (bit n: DBYTE n).
    pub block_dbs: [u8; BLOCKS],
    /// The fold bits each lane drives.
    pub lane_bits: [[u32; LANES]; DBYTES],
}

impl Map {
    /// Blocks usable for (`db`, `tg`): of that timing group, and reached by
    /// `db` and only by DBYTEs whose lanes use different bits.
    pub fn blocks(&self, db: usize, tg: usize) -> u32 {
        let mut m = 0;
        for b in 0..BLOCKS {
            if self.block_tg[b] == Some(tg as u8) && self.block_dbs[b] & (1 << db) != 0 {
                m |= 1 << b;
            }
        }
        m
    }
    /// Whether (`db`, `tg`) can be measured at all.
    pub fn reachable(&self, db: usize, tg: usize) -> bool {
        self.blocks(db, tg) != 0
    }
    /// DQ lanes of `db` this compare sees.
    pub fn seen(&self, db: usize, lane: usize) -> bool {
        self.lane_bits[db][lane] != 0
    }
}

/// A clean write-back of every block, two seeds.
fn clean<P: WritePort>(p: &mut P, seed: &mut u32) -> Result<u32, P::Error> {
    let mut bad = 0;
    for _ in 0..2 {
        *seed = seed.wrapping_add(0x9e37_79b9);
        let f = p.check(*seed)?;
        for (b, v) in f.iter().enumerate() {
            if *v != 0 {
                bad |= 1 << b;
            }
        }
    }
    Ok(bad)
}

/// One UI later, or earlier near the top of the range.
const fn disturbed(v: u16) -> u16 {
    if v + UI <= TX_DQ_MAX {
        v + UI
    } else {
        v - UI
    }
}

/// The block and lane maps, from a state that writes cleanly (the
/// PMU-trained one): each lane of each DBYTE and timing group in turn is
/// moved by one UI, the blocks and data bits that fail belong to it, the
/// lane goes back.
pub fn discover<P: WritePort>(p: &mut P, seed: &mut u32) -> Result<Map, WrError<P::Error>> {
    let bad = clean(p, seed).map_err(WrError::Port)?;
    if bad != 0 {
        return Err(WrError::Baseline(bad));
    }
    let mut m = Map {
        block_tg: [None; BLOCKS],
        block_dbs: [0; BLOCKS],
        lane_bits: [[0; LANES]; DBYTES],
    };
    for tg in 0..2 {
        for db in 0..DBYTES {
            for lane in 0..LANES {
                let v = p.tx_dq(db, lane, tg).map_err(WrError::Port)?;
                p.set_tx_dq(db, lane, tg, disturbed(v))
                    .map_err(WrError::Port)?;
                *seed = seed.wrapping_add(0x9e37_79b9);
                let f = p.check(*seed);
                p.set_tx_dq(db, lane, tg, v).map_err(WrError::Port)?;
                let f = f.map_err(WrError::Port)?;
                for (b, x) in f.iter().enumerate() {
                    if *x == 0 {
                        continue;
                    }
                    match m.block_tg[b] {
                        Some(t) if t != tg as u8 => return Err(WrError::BlockTg(b)),
                        _ => m.block_tg[b] = Some(tg as u8),
                    }
                    m.block_dbs[b] |= 1 << db;
                    m.lane_bits[db][lane] |= x;
                }
            }
        }
    }
    for db in 0..DBYTES {
        for lane in 0..DMI_LANE {
            if m.lane_bits[db][lane] == 0 {
                return Err(WrError::LaneUnseen(db, lane));
            }
            for other in 0..DBYTES {
                for ol in 0..LANES {
                    if (other, ol) == (db, lane) {
                        continue;
                    }
                    let shared_block = (0..BLOCKS).any(|b| {
                        m.block_dbs[b] & (1 << db) != 0 && m.block_dbs[b] & (1 << other) != 0
                    });
                    if shared_block && m.lane_bits[db][lane] & m.lane_bits[other][ol] != 0 {
                        return Err(WrError::LaneOverlap(db, lane, other));
                    }
                }
            }
        }
    }
    Ok(m)
}

/// Failing data bits over every block, two seeds.
fn error_bits<P: WritePort>(p: &mut P, seed: &mut u32) -> Result<u32, P::Error> {
    let mut n = 0;
    for _ in 0..2 {
        *seed = seed.wrapping_add(0x9e37_79b9);
        n += p.check(*seed)?.iter().map(|f| f.count_ones()).sum::<u32>();
    }
    Ok(n)
}

/// Set every lane of DBYTE `db`, both timing groups, to `v`.
fn set_dbyte<P: WritePort>(p: &mut P, db: usize, v: u16) -> Result<(), P::Error> {
    for tg in 0..2 {
        for lane in 0..LANES {
            p.set_tx_dq(db, lane, tg, v)?;
        }
    }
    Ok(())
}

/// A clean-writing start for [`discover`] without any map or reference
/// (the blobless state: every TxDqDly at 0x20): per DBYTE in turn, all its
/// lanes and both timing groups at one value, the value with the fewest
/// failing data bits over all blocks. A DBYTE's errors only depend on its
/// own delay, so the DBYTEs can be searched one at a time: a coarse pass
/// (step 8 over the whole range), then step 1 within +/-8 of the coarse
/// best, the middle of the longest run of minimal error counts; two
/// rounds. Returns the value per DBYTE and the remaining error bits (0
/// when the start is clean).
pub fn blind_baseline<P: WritePort>(
    p: &mut P,
    seed: &mut u32,
) -> Result<([u16; DBYTES], u32), WrError<P::Error>> {
    let mut v = [0x20u16; DBYTES];
    for db in 0..DBYTES {
        set_dbyte(p, db, v[db]).map_err(WrError::Port)?;
    }
    for _round in 0..2 {
        for db in 0..DBYTES {
            let mut best = (u32::MAX, v[db]);
            let mut x = 0u16;
            while x <= TX_DQ_MAX {
                set_dbyte(p, db, x).map_err(WrError::Port)?;
                let e = error_bits(p, seed).map_err(WrError::Port)?;
                if e < best.0 {
                    best = (e, x);
                }
                x += 8;
            }
            let lo = best.1.saturating_sub(8);
            let hi = (best.1 + 8).min(TX_DQ_MAX);
            let mut counts = [u32::MAX; 17];
            for (i, x) in (lo..=hi).enumerate() {
                set_dbyte(p, db, x).map_err(WrError::Port)?;
                counts[i] = error_bits(p, seed).map_err(WrError::Port)?;
            }
            let min = *counts.iter().min().unwrap_or(&u32::MAX);
            let (mut run, mut best_run) = ((0usize, 0usize), (0usize, 0usize));
            for (i, &c) in counts.iter().enumerate() {
                if c == min {
                    if run.1 == 0 {
                        run.0 = i;
                    }
                    run.1 += 1;
                    if run.1 > best_run.1 {
                        best_run = run;
                    }
                } else {
                    run = (0, 0);
                }
            }
            v[db] = lo + (best_run.0 + (best_run.1.max(1) - 1) / 2) as u16;
            set_dbyte(p, db, v[db]).map_err(WrError::Port)?;
        }
    }
    let left = error_bits(p, seed).map_err(WrError::Port)?;
    Ok((v, left))
}

/// A sweep: every lane at `base[db][lane] + lo + step * i`, i < `n`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Sweep {
    pub lo: i16,
    pub step: u16,
    pub n: u32,
}

/// The coarse sweep over the whole range: 0, 4, .., 0x1fc.
pub const COARSE: Sweep = Sweep {
    lo: 0,
    step: 4,
    n: 128,
};
/// The fine sweep around a coarse centre: -40..=+40 by 1.
pub const FINE: Sweep = Sweep {
    lo: -40,
    step: 1,
    n: 81,
};

/// Pass maps per DBYTE and lane (bit i: point i passed).
pub type PassMaps = [[u128; LANES]; DBYTES];

fn point(base: u16, s: &Sweep, i: u32) -> Option<u16> {
    let v = base as i32 + s.lo as i32 + (s.step as u32 * i) as i32;
    if (0..=TX_DQ_MAX as i32).contains(&v) {
        Some(v as u16)
    } else {
        None
    }
}

/// Run sweep `s` on timing group `tg` around `base` (all zeros for
/// [`COARSE`]); lanes of unreachable DBYTEs stay untouched. The delays are
/// left at the last point: callers apply a result afterwards.
pub fn sweep<P: WritePort>(
    p: &mut P,
    m: &Map,
    tg: usize,
    base: &Delays,
    s: &Sweep,
    seed: &mut u32,
) -> Result<PassMaps, WrError<P::Error>> {
    if s.n == 0 || s.n > MAX_POINTS || s.step == 0 {
        return Err(WrError::Config);
    }
    let mut maps = [[0u128; LANES]; DBYTES];
    for i in 0..s.n {
        let mut valid = [[false; LANES]; DBYTES];
        for db in 0..DBYTES {
            if !m.reachable(db, tg) {
                continue;
            }
            for lane in 0..LANES {
                if let Some(v) = point(base[db][lane], s, i) {
                    p.set_tx_dq(db, lane, tg, v).map_err(WrError::Port)?;
                    valid[db][lane] = true;
                }
            }
        }
        let mut fails = [0u32; BLOCKS];
        for _ in 0..2 {
            *seed = seed.wrapping_add(0x9e37_79b9);
            let f = p.check(*seed).map_err(WrError::Port)?;
            for b in 0..BLOCKS {
                fails[b] |= f[b];
            }
        }
        for db in 0..DBYTES {
            let blocks = m.blocks(db, tg);
            for lane in 0..DMI_LANE {
                if !valid[db][lane] || blocks == 0 {
                    continue;
                }
                let bits = m.lane_bits[db][lane];
                let ok = (0..BLOCKS).all(|b| blocks & (1 << b) == 0 || fails[b] & bits == 0);
                if ok {
                    maps[db][lane] |= 1 << i;
                }
            }
        }
    }
    Ok(maps)
}

/// One lane's window and the setting chosen in it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LaneFit {
    /// The window lies inside the sweep: its centre and width (delay units).
    Centre { value: u16, width: u16 },
    /// The window reaches an end of the sweep: the edge on that side is not
    /// known, so the centre is not either. `value` is the middle of the
    /// part seen; not applied without a wider sweep.
    AtEdge { value: u16, width: u16 },
    /// Narrower than `min_points` points (noise) or empty.
    NoWindow,
    /// Not measured (unreachable DBYTE or timing group, or the DMI lane).
    Unmeasured,
}

impl LaneFit {
    pub const fn value(self) -> Option<u16> {
        match self {
            LaneFit::Centre { value, .. } => Some(value),
            _ => None,
        }
    }
    pub const fn width(self) -> Option<u16> {
        match self {
            LaneFit::Centre { width, .. } | LaneFit::AtEdge { width, .. } => Some(width),
            _ => None,
        }
    }
}

/// The fit of one pass map: the longest run of passing points (not
/// circular: a shift by whole beats is not a pass).
pub fn fit(map: u128, base: u16, s: &Sweep, min_points: u32) -> LaneFit {
    let w: Window = longest_run_linear(map, s.n);
    if w.len == 0 || w.len < min_points {
        return LaneFit::NoWindow;
    }
    let first = point(base, s, w.start);
    let last = point(base, s, w.start + w.len - 1);
    let (Some(first), Some(last)) = (first, last) else {
        return LaneFit::NoWindow;
    };
    let value = first + (last - first) / 2;
    let width = last - first + s.step;
    let lo_open = w.start == 0 || point(base, s, w.start - 1).is_none();
    let hi_open = w.start + w.len == s.n || point(base, s, w.start + w.len).is_none();
    if lo_open || hi_open {
        LaneFit::AtEdge { value, width }
    } else {
        LaneFit::Centre { value, width }
    }
}

/// Fits of every lane; the DMI lane gets the median centre of its
/// DBYTE's DQ lanes when all eight are centred.
pub fn fits(
    m: &Map,
    tg: usize,
    maps: &PassMaps,
    base: &Delays,
    s: &Sweep,
) -> [[LaneFit; LANES]; DBYTES] {
    let mut out = [[LaneFit::Unmeasured; LANES]; DBYTES];
    for db in 0..DBYTES {
        if !m.reachable(db, tg) {
            continue;
        }
        let mut c = [0u16; DMI_LANE];
        let mut all = true;
        for lane in 0..DMI_LANE {
            out[db][lane] = fit(maps[db][lane], base[db][lane], s, 3);
            match out[db][lane].value() {
                Some(v) => c[lane] = v,
                None => all = false,
            }
        }
        if all {
            c.sort_unstable();
            let med = (c[3] + c[4]) / 2;
            out[db][DMI_LANE] = LaneFit::Centre {
                value: med,
                width: 0,
            };
        }
    }
    out
}

/// The result of [`centre`] for one timing group.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Centring {
    pub tg: usize,
    /// The delays before the stage (the PMU's in the hybrid).
    pub before: Delays,
    pub coarse: [[LaneFit; LANES]; DBYTES],
    pub fine: [[LaneFit; LANES]; DBYTES],
    /// What was applied: the fine centre where there is one, else `before`.
    pub applied: Delays,
    /// Lanes applied from a fine centre (bit 9 * db + lane).
    pub centred: u64,
}

/// Write DQ centring of timing group `tg`: coarse sweep over the whole
/// range, fine sweep around each lane's coarse centre, apply the fine
/// centres (other lanes keep their value), check the result. With
/// `apply` false the `before` delays are put back (measurement only).
pub fn centre<P: WritePort>(
    p: &mut P,
    m: &Map,
    tg: usize,
    apply: bool,
    seed: &mut u32,
) -> Result<Centring, WrError<P::Error>> {
    let mut before = [[0u16; LANES]; DBYTES];
    for db in 0..DBYTES {
        for lane in 0..LANES {
            before[db][lane] = p.tx_dq(db, lane, tg).map_err(WrError::Port)?;
        }
    }
    let zero = [[0u16; LANES]; DBYTES];
    let cmaps = sweep(p, m, tg, &zero, &COARSE, seed)?;
    let coarse = fits(m, tg, &cmaps, &zero, &COARSE);
    let mut fbase = before;
    for db in 0..DBYTES {
        for lane in 0..LANES {
            if let Some(v) = coarse[db][lane].value() {
                fbase[db][lane] = v;
            }
        }
    }
    let fmaps = sweep(p, m, tg, &fbase, &FINE, seed)?;
    let fine = fits(m, tg, &fmaps, &fbase, &FINE);
    let mut applied = before;
    let mut centred = 0u64;
    if apply {
        for db in 0..DBYTES {
            for lane in 0..LANES {
                if let Some(v) = fine[db][lane].value() {
                    applied[db][lane] = v;
                    centred |= 1 << (9 * db + lane);
                }
            }
        }
    }
    for db in 0..DBYTES {
        for lane in 0..LANES {
            p.set_tx_dq(db, lane, tg, applied[db][lane])
                .map_err(WrError::Port)?;
        }
    }
    let bad = clean(p, seed).map_err(WrError::Port)?;
    let tg_blocks = (0..BLOCKS)
        .filter(|&b| m.block_tg[b] == Some(tg as u8))
        .fold(0u32, |a, b| a | 1 << b);
    if bad & tg_blocks != 0 {
        return Err(WrError::Verify(bad & tg_blocks));
    }
    Ok(Centring {
        tg,
        before,
        coarse,
        fine,
        applied,
        centred,
    })
}

/// MRW `op` to MA `ma` of the **operating** set-point of (`ch`, `rank`):
/// MR16 read, FSP-WR pointed at FSP-OP (VRCG as `vrcg` asks), the MRW,
/// MR16 restored. Returns the MR16 found.
pub fn write_operating<M: MrPort>(
    mp: &mut M,
    ch: usize,
    rank: usize,
    ma: u8,
    op: u8,
    vrcg: bool,
) -> Result<u8, M::Error> {
    let mr16 = mp.mrr(ch, rank, 16)?;
    let fsp_op = (mr16 >> 2) & 3;
    let target = (mr16 & 0xbc) | fsp_op | if vrcg { 0x40 } else { 0 };
    if target != mr16 {
        mp.mrw(ch, rank, 16, target)?;
    }
    mp.mrw(ch, rank, ma, op)?;
    if target != mr16 {
        mp.mrw(ch, rank, 16, mr16)?;
    }
    Ok(mr16)
}

/// MRR of MA `ma` from the **operating** set-point (FSP-dependent MRs read
/// from the set FSP-WR selects): FSP-WR pointed at FSP-OP for the read,
/// MR16 restored.
pub fn read_operating<M: MrPort>(
    mp: &mut M,
    ch: usize,
    rank: usize,
    ma: u8,
) -> Result<u8, M::Error> {
    let mr16 = mp.mrr(ch, rank, 16)?;
    let target = (mr16 & 0xfc) | ((mr16 >> 2) & 3);
    if target != mr16 {
        mp.mrw(ch, rank, 16, target)?;
    }
    let v = mp.mrr(ch, rank, ma);
    if target != mr16 {
        mp.mrw(ch, rank, 16, mr16)?;
    }
    v
}

/// Which DBYTE each Vref MR drives, per channel: `[mr14_db, mr15_db]`.
pub type VrefBytes = [[usize; 2]; 2];

/// Find which DBYTE MR14 and MR15 of each channel drive: each MR in turn
/// set to `bad_code` (a code far from the trained one) on rank `tg`, the
/// DBYTE whose lanes fail is its byte; the MR goes back to `good_code`.
pub fn identify_vref_bytes<P: WritePort, M: MrPort>(
    p: &mut P,
    mp: &mut M,
    m: &Map,
    tg: usize,
    good_code: [[u8; 2]; 2],
    bad_code: u8,
    seed: &mut u32,
) -> Result<Option<VrefBytes>, WrError<P::Error>>
where
    M::Error: Into<P::Error>,
{
    let mut out = [[usize::MAX; 2]; 2];
    for ch in 0..2 {
        for (k, ma) in [14u8, 15].into_iter().enumerate() {
            write_operating(mp, ch, tg, ma, bad_code, false)
                .map_err(|e| WrError::Port(e.into()))?;
            mp.settle();
            *seed = seed.wrapping_add(0x9e37_79b9);
            let f = p.check(*seed);
            write_operating(mp, ch, tg, ma, good_code[ch][k], false)
                .map_err(|e| WrError::Port(e.into()))?;
            mp.settle();
            let f = f.map_err(WrError::Port)?;
            let mut hit = usize::MAX;
            for db in 0..DBYTES {
                let blocks = m.blocks(db, tg);
                let bits = (0..DMI_LANE).fold(0, |a, l| a | m.lane_bits[db][l]);
                if (0..BLOCKS).any(|b| blocks & (1 << b) != 0 && f[b] & bits != 0) {
                    if hit != usize::MAX {
                        return Ok(None);
                    }
                    hit = db;
                }
            }
            if hit == usize::MAX {
                return Ok(None);
            }
            out[ch][k] = hit;
        }
    }
    Ok(Some(out))
}

/// Points of a Vref sweep.
pub const VREF_POINTS: usize = 32;

/// The write Vref sweep of one timing group (rank).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct VrefSweep {
    pub tg: usize,
    pub codes: [u8; VREF_POINTS],
    pub n: usize,
    /// Per DBYTE and point the narrowest DQ lane window (fine sweep
    /// units); `None` where a lane had no centred window.
    pub widths: [[Option<u8>; VREF_POINTS]; DBYTES],
    /// Per DBYTE the code at the middle of the eye at half height.
    pub chosen: [Option<u8>; DBYTES],
    /// Per DBYTE the code with the widest worst lane (max-min).
    pub max_min: [Option<u8>; DBYTES],
}

/// The codes of a sweep from `first` by `step`, `n` of them.
pub fn vref_codes(first: u8, step: u8, n: usize) -> ([u8; VREF_POINTS], usize) {
    let mut c = [0u8; VREF_POINTS];
    let n = n.min(VREF_POINTS);
    for (i, slot) in c.iter_mut().enumerate().take(n) {
        *slot = first.saturating_add(step.saturating_mul(i as u8)).min(127);
    }
    (c, n)
}

/// The per-DBYTE choice from a width profile.
pub fn choose(widths: &[Option<u8>], codes: &[u8]) -> (Option<u8>, Option<u8>) {
    let chosen = dram_train::eye::center(widths)
        .ok()
        .map(|e| codes[e.center]);
    let mut best: Option<(u8, usize)> = None;
    for (i, w) in widths.iter().enumerate() {
        if let Some(w) = *w {
            if w > 0 && best.is_none_or(|(bw, _)| w > bw) {
                best = Some((w, i));
            }
        }
    }
    (chosen, best.map(|(_, i)| codes[i]))
}

/// Write Vref sweep of timing group `tg`: the same code to MR14 and MR15
/// of both channels at every point (operating set-point, [`write_operating`],
/// VRCG on during the sweep), the fine write window re-centred around
/// `centres` at each point, the narrowest lane per DBYTE as its width.
/// Afterwards every Vref MR gets its byte's chosen code (or `restore` where
/// none was chosen) and the delays are re-centred at the chosen codes when
/// `apply`; otherwise the `restore` codes and the `centres` go back.
#[allow(clippy::too_many_arguments)]
pub fn vref_sweep<P: WritePort, M: MrPort>(
    p: &mut P,
    mp: &mut M,
    m: &Map,
    tg: usize,
    bytes: &VrefBytes,
    codes: &[u8],
    restore: [[u8; 2]; 2],
    centres: &Delays,
    apply: bool,
    seed: &mut u32,
) -> Result<VrefSweep, WrError<P::Error>>
where
    M::Error: Into<P::Error>,
{
    let n = codes.len().min(VREF_POINTS);
    let mut out = VrefSweep {
        tg,
        codes: [0; VREF_POINTS],
        n,
        widths: [[None; VREF_POINTS]; DBYTES],
        chosen: [None; DBYTES],
        max_min: [None; DBYTES],
    };
    out.codes[..n].copy_from_slice(&codes[..n]);
    let mrw_all = |mp: &mut M, code: &dyn Fn(usize, usize) -> u8, vrcg: bool| {
        for ch in 0..2 {
            for (k, ma) in [14u8, 15].into_iter().enumerate() {
                write_operating(mp, ch, tg, ma, code(ch, k), vrcg)?;
            }
        }
        mp.settle();
        Ok::<(), M::Error>(())
    };
    for (i, &code) in codes.iter().enumerate().take(n) {
        mrw_all(mp, &|_, _| code, true).map_err(|e| WrError::Port(e.into()))?;
        let maps = sweep(p, m, tg, centres, &FINE, seed)?;
        let f = fits(m, tg, &maps, centres, &FINE);
        for db in 0..DBYTES {
            if !m.reachable(db, tg) {
                continue;
            }
            let mut worst: Option<u8> = Some(u8::MAX);
            for lane in f[db].iter().take(DMI_LANE) {
                worst = match (worst, lane) {
                    (Some(w), LaneFit::Centre { width, .. }) => {
                        Some(w.min((*width).min(255) as u8))
                    }
                    _ => None,
                };
            }
            out.widths[db][i] = worst;
        }
    }
    for db in 0..DBYTES {
        let (c, mm) = choose(&out.widths[db][..n], &codes[..n]);
        out.chosen[db] = c;
        out.max_min[db] = mm;
    }
    let pick = |ch: usize, k: usize| -> u8 {
        if !apply {
            return restore[ch][k];
        }
        out.chosen[bytes[ch][k]].unwrap_or(restore[ch][k])
    };
    mrw_all(mp, &pick, false).map_err(|e| WrError::Port(e.into()))?;
    for db in 0..DBYTES {
        for lane in 0..LANES {
            p.set_tx_dq(db, lane, tg, centres[db][lane])
                .map_err(WrError::Port)?;
        }
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use core::cell::RefCell;
    use std::vec::Vec;

    /// A simulated channel pair: per (db, lane, tg) a pass window of the
    /// TxDqDly value, narrowed by the Vref distance from the lane's best
    /// code; MRWs land in the FSP that MR16 FSP-WR selects and only the
    /// FSP-OP set acts.
    struct Sim {
        dq: [[[u16; 2]; LANES]; DBYTES],
        /// Window centre and half width per (db, lane, tg).
        centre: [[[u16; 2]; LANES]; DBYTES],
        half: u16,
        /// The best Vref code per DBYTE and the width lost per code away.
        vref_best: [u8; DBYTES],
        /// MR file per (ch, rank, fsp): MA 14, 15; MR16 per (ch, rank).
        mr: [[[[u8; 2]; 2]; 2]; 2],
        mr16: [[u8; 2]; 2],
        /// Blocks: (tg, ch) each; ch -1 for an unreached block.
        blocks: [(usize, i8); BLOCKS],
        /// Which DBYTE MR14 / MR15 of a channel drive.
        bytes: VrefBytes,
        checks: u32,
        mrws: Vec<(usize, usize, u8, u8)>,
    }

    /// Fold bits of DBYTE `db` lane `lane`: byte (db % 2) of both beats.
    fn bits(db: usize, lane: usize) -> u32 {
        let b = (db % 2) * 8 + lane;
        1 << b | 1 << (b + 16)
    }

    impl Sim {
        fn new() -> Sim {
            let mut centre = [[[0u16; 2]; LANES]; DBYTES];
            for db in 0..DBYTES {
                for lane in 0..LANES {
                    for tg in 0..2 {
                        centre[db][lane][tg] =
                            [0xa2, 0x72, 0x6d, 0x9c][db] + (lane as u16 * 3) % 7 + tg as u16;
                    }
                }
            }
            let mut blocks = [(0usize, 0i8); BLOCKS];
            for (b, slot) in blocks.iter_mut().enumerate() {
                *slot = (b / 8, (b % 2) as i8);
            }
            Sim {
                dq: [[[0x20; 2]; LANES]; DBYTES],
                centre,
                half: 20,
                vref_best: [0x2e, 0x30, 0x2c, 0x2e],
                mr: [[[[0x2e; 2]; 2]; 2]; 2],
                mr16: [[0x04; 2]; 2],
                blocks,
                bytes: [[0, 1], [2, 3]],
                checks: 0,
                mrws: Vec::new(),
            }
        }
        fn trained(mut self) -> Sim {
            for db in 0..DBYTES {
                for lane in 0..LANES {
                    for tg in 0..2 {
                        self.dq[db][lane][tg] = self.centre[db][lane][tg];
                    }
                }
            }
            self
        }
        fn operating_vref(&self, db: usize, tg: usize) -> u8 {
            for ch in 0..2 {
                for k in 0..2 {
                    if self.bytes[ch][k] == db {
                        let fsp = ((self.mr16[ch][tg] >> 2) & 1) as usize;
                        return self.mr[ch][tg][fsp][k];
                    }
                }
            }
            unreachable!()
        }
        fn lane_ok(&self, db: usize, lane: usize, tg: usize) -> bool {
            if lane == DMI_LANE {
                return true;
            }
            let d = self.operating_vref(db, tg).abs_diff(self.vref_best[db]) as u16;
            let half = self.half.saturating_sub(d * 2);
            let c = self.centre[db][lane][tg];
            half > 0 && self.dq[db][lane][tg].abs_diff(c) < half
        }
    }

    impl WritePort for Sim {
        type Error = ();
        fn set_tx_dq(&mut self, db: usize, lane: usize, tg: usize, v: u16) -> Result<(), ()> {
            assert!(v <= TX_DQ_MAX);
            self.dq[db][lane][tg] = v;
            Ok(())
        }
        fn tx_dq(&mut self, db: usize, lane: usize, tg: usize) -> Result<u16, ()> {
            Ok(self.dq[db][lane][tg])
        }
        fn check(&mut self, _seed: u32) -> Result<Folds, ()> {
            self.checks += 1;
            let mut f = [0u32; BLOCKS];
            for (b, &(tg, ch)) in self.blocks.iter().enumerate() {
                if ch < 0 {
                    continue;
                }
                for db in [2 * ch as usize, 2 * ch as usize + 1] {
                    for lane in 0..LANES {
                        if !self.lane_ok(db, lane, tg) && lane != DMI_LANE {
                            f[b] |= bits(db, lane);
                        }
                    }
                }
            }
            Ok(f)
        }
    }

    impl MrPort for Sim {
        type Error = ();
        fn mrw(&mut self, ch: usize, rank: usize, ma: u8, op: u8) -> Result<(), ()> {
            self.mrws.push((ch, rank, ma, op));
            match ma {
                16 => self.mr16[ch][rank] = op,
                14 | 15 => {
                    let fsp = (self.mr16[ch][rank] & 3) as usize;
                    self.mr[ch][rank][fsp][(ma - 14) as usize] = op;
                }
                _ => panic!("MRW to MA {ma}"),
            }
            Ok(())
        }
        fn mrr(&mut self, ch: usize, rank: usize, ma: u8) -> Result<u8, ()> {
            let fsp = (self.mr16[ch][rank] & 3) as usize;
            Ok(match ma {
                16 => self.mr16[ch][rank],
                14 | 15 => self.mr[ch][rank][fsp][(ma - 14) as usize],
                _ => panic!("MRR of MA {ma}"),
            })
        }
        fn settle(&mut self) {}
    }

    #[test]
    fn read_operating_reads_the_operating_set() {
        let mut s = Sim::new();
        s.mr[0][1][1][0] = 0x31;
        s.mr[0][1][0][0] = 0x50;
        s.mr16[0][1] = 0x04;
        assert_eq!(read_operating(&mut s, 0, 1, 14).unwrap(), 0x31);
        assert_eq!(s.mr16[0][1], 0x04);
        assert_eq!(s.mrws, [(0, 1, 16, 0x05), (0, 1, 16, 0x04)]);
    }

    #[test]
    fn csr_addresses() {
        assert_eq!(tx_dq_csr(0, 0, 0), 0x10026);
        assert_eq!(tx_dq_csr(0, 0, 1), 0x10027);
        assert_eq!(tx_dq_csr(1, 1, 0), 0x11126);
        assert_eq!(tx_dq_csr(3, 8, 1), 0x13827);
    }

    #[test]
    fn discovery_finds_blocks_and_lanes() {
        let mut s = Sim::new().trained();
        let mut seed = 1;
        let m = discover(&mut s, &mut seed).unwrap();
        for b in 0..BLOCKS {
            assert_eq!(m.block_tg[b], Some((b / 8) as u8));
            assert_eq!(m.block_dbs[b], 0b11 << (2 * (b % 2)));
        }
        for db in 0..DBYTES {
            for lane in 0..DMI_LANE {
                assert_eq!(m.lane_bits[db][lane], bits(db, lane));
            }
            assert!(!m.seen(db, DMI_LANE));
        }
        // Every lane went back.
        assert_eq!(s.dq, Sim::new().trained().dq);
    }

    #[test]
    fn discovery_needs_a_clean_start_and_reports_unreachable() {
        let mut s = Sim::new();
        let mut seed = 1;
        assert!(matches!(
            discover(&mut s, &mut seed),
            Err(WrError::Baseline(_))
        ));
        // Rank 1 out of reach (aarch32 window): its blocks never react.
        let mut s = Sim::new().trained();
        for b in 8..BLOCKS {
            s.blocks[b].1 = -1;
        }
        let m = discover(&mut s, &mut seed).unwrap();
        assert!(m.reachable(0, 0) && !m.reachable(0, 1));
        let r = centre(&mut s, &m, 1, true, &mut seed).unwrap();
        assert_eq!(r.centred, 0);
        assert!(r.fine.iter().flatten().all(|f| *f == LaneFit::Unmeasured));
    }

    #[test]
    fn centring_from_untrained_finds_the_trained_values() {
        let mut s = Sim::new().trained();
        let mut seed = 7;
        let m = discover(&mut s, &mut seed).unwrap();
        // Back to the reset value, as in the blobless state.
        for db in 0..DBYTES {
            for lane in 0..LANES {
                s.dq[db][lane][0] = 0x20;
            }
        }
        let r = centre(&mut s, &m, 0, true, &mut seed).unwrap();
        for db in 0..DBYTES {
            for lane in 0..DMI_LANE {
                let want = s.centre[db][lane][0];
                let got = r.applied[db][lane];
                assert!(
                    got.abs_diff(want) <= 1,
                    "db {db} lane {lane}: {got:x} vs {want:x}"
                );
                // Window |v - c| < half: 2 * half - 1 points, half narrowed by the
                // Vref distance (2 per code: DBYTEs 1 and 2 are off their best).
                let half = 20 - 2 * [0u16, 2, 2, 0][db];
                assert_eq!(r.fine[db][lane].width(), Some(2 * half - 1));
            }
            assert!(r.applied[db][DMI_LANE].abs_diff(s.centre[db][3][0]) <= 4);
        }
        assert_eq!(r.centred.count_ones(), 36);
    }

    #[test]
    fn blind_baseline_then_discovery_from_the_untrained_state() {
        let mut s = Sim::new();
        let mut seed = 21;
        assert!(matches!(
            discover(&mut s, &mut seed),
            Err(WrError::Baseline(_))
        ));
        let (v, left) = blind_baseline(&mut s, &mut seed).unwrap();
        assert_eq!(left, 0, "start {v:x?}");
        let m = discover(&mut s, &mut seed).unwrap();
        let r = centre(&mut s, &m, 0, true, &mut seed).unwrap();
        for db in 0..DBYTES {
            for lane in 0..DMI_LANE {
                assert!(r.applied[db][lane].abs_diff(s.centre[db][lane][0]) <= 1);
            }
        }
    }

    #[test]
    fn measure_only_restores() {
        let mut s = Sim::new().trained();
        let before = s.dq;
        let mut seed = 3;
        let m = discover(&mut s, &mut seed).unwrap();
        let r = centre(&mut s, &m, 0, false, &mut seed).unwrap();
        assert_eq!(s.dq, before);
        assert_eq!(r.applied, r.before);
    }

    #[test]
    fn fit_edge_cases() {
        let s = COARSE;
        // No window.
        assert_eq!(fit(0, 0, &s, 3), LaneFit::NoWindow);
        // A two-point blip is noise.
        assert_eq!(fit(0b110 << 40, 0, &s, 3), LaneFit::NoWindow);
        // Inside: points 40..=49, values 160..=196.
        let w = ((1u128 << 10) - 1) << 40;
        assert_eq!(
            fit(w, 0, &s, 3),
            LaneFit::Centre {
                value: 178,
                width: 40
            }
        );
        // At the low end of the sweep.
        assert!(matches!(
            fit((1 << 10) - 1, 0, &s, 3),
            LaneFit::AtEdge { .. }
        ));
        // At the high end.
        assert!(matches!(
            fit(((1u128 << 10) - 1) << 118, 0, &s, 3),
            LaneFit::AtEdge { .. }
        ));
        // The fine sweep clipped by the range: base 10, lo -40.
        let f = FINE;
        assert!(matches!(
            fit((1 << 45) - 1, 10, &f, 3),
            LaneFit::NoWindow | LaneFit::AtEdge { .. }
        ));
        // Two windows: the longer one.
        let two = (0b111u128 << 5) | (0b11111u128 << 20);
        assert_eq!(fit(two, 0, &s, 3).value(), Some(88));
    }

    #[test]
    fn mrw_goes_to_the_operating_fsp_and_mr16_is_restored() {
        let mut s = Sim::new();
        // The PMU's state: FSP-OP 1, FSP-WR 0.
        s.mr16 = [[0x04; 2]; 2];
        let old = write_operating(&mut s, 1, 0, 14, 0x33, true).unwrap();
        assert_eq!(old, 0x04);
        assert_eq!(s.mr[1][0][1][0], 0x33, "operating set written");
        assert_eq!(s.mr[1][0][0][0], 0x2e, "other set untouched");
        assert_eq!(s.mr16[1][0], 0x04);
        assert_eq!(
            s.mrws,
            [(1, 0, 16, 0x45), (1, 0, 14, 0x33), (1, 0, 16, 0x04)]
        );
        // Already pointing at the operating set: no MR16 traffic.
        s.mrws.clear();
        s.mr16[0][1] = 0x05;
        write_operating(&mut s, 0, 1, 15, 0x30, false).unwrap();
        assert_eq!(s.mrws, [(0, 1, 15, 0x30)]);
        assert_eq!(s.mr[0][1][1][1], 0x30);
        // FSP-OP 0, FSP-WR 1 (blobless before the switch): back to set 0.
        s.mrws.clear();
        s.mr16[0][0] = 0x41;
        write_operating(&mut s, 0, 0, 14, 0x31, false).unwrap();
        assert_eq!(s.mr[0][0][0][0], 0x31);
        assert_eq!(s.mr16[0][0], 0x41);
    }

    #[test]
    fn vref_bytes_and_sweep_choose_each_bytes_best_code() {
        let mut s = Sim::new().trained();
        // Upper and lower byte swapped on channel B.
        s.bytes = [[0, 1], [3, 2]];
        let mut seed = 11;
        let m = discover(&mut s, &mut seed).unwrap();
        let centres = s.dq.map(|l| l.map(|t| t[0]));
        let cell = RefCell::new(s);
        let good = [[0x2e; 2]; 2];
        let bytes =
            identify_vref_bytes(&mut Dq(&cell), &mut Mr(&cell), &m, 0, good, 0x0a, &mut seed)
                .unwrap();
        assert_eq!(bytes, Some([[0, 1], [3, 2]]));
        assert_eq!(cell.borrow().mr[1][0][1], [0x2e, 0x2e], "restored");
        let (codes, n) = vref_codes(0x22, 1, 24);
        let r = vref_sweep(
            &mut Dq(&cell),
            &mut Mr(&cell),
            &m,
            0,
            &bytes.unwrap(),
            &codes[..n],
            good,
            &centres,
            true,
            &mut seed,
        )
        .unwrap();
        let s = cell.into_inner();
        for db in 0..DBYTES {
            let best = [0x2e, 0x30, 0x2c, 0x2e][db];
            assert_eq!(r.chosen[db], Some(best), "db {db}");
            assert_eq!(r.max_min[db], Some(best), "db {db}");
        }
        // Applied per byte in the operating set; MR16 back to the PMU's.
        assert_eq!(s.mr[1][0][1], [0x2e, 0x2c], "MR14 drives DBYTE 3 here");
        assert_eq!(s.mr[0][0][1], [0x2e, 0x30]);
        assert_eq!(s.mr16, [[0x04; 2]; 2]);
        // The other FSP and the other rank never changed.
        assert_eq!(s.mr[0][0][0], [0x2e, 0x2e]);
        assert_eq!(s.mr[0][1][1], [0x2e, 0x2e]);
        // The delays went back to the centres.
        assert_eq!(s.dq.map(|l| l.map(|t| t[0])), centres);
    }

    #[test]
    fn vref_measure_only_restores_the_codes() {
        let mut s = Sim::new().trained();
        let mut seed = 5;
        let m = discover(&mut s, &mut seed).unwrap();
        let centres = s.dq.map(|l| l.map(|t| t[0]));
        let cell = RefCell::new(s);
        let (codes, n) = vref_codes(0x26, 2, 8);
        let restore = [[0x2e, 0x2f], [0x2d, 0x2e]];
        vref_sweep(
            &mut Dq(&cell),
            &mut Mr(&cell),
            &m,
            0,
            &[[0, 1], [2, 3]],
            &codes[..n],
            restore,
            &centres,
            false,
            &mut seed,
        )
        .unwrap();
        let s = cell.into_inner();
        assert_eq!(s.mr[0][0][1], [0x2e, 0x2f]);
        assert_eq!(s.mr[1][0][1], [0x2d, 0x2e]);
    }

    #[test]
    fn vref_choice_edge_cases() {
        let codes = [10u8, 11, 12, 13, 14];
        assert_eq!(choose(&[None; 5], &codes), (None, None));
        assert_eq!(choose(&[Some(0); 5], &codes), (None, None));
        // A peak at the sweep's edge still picks it (max-min); the centre
        // rule picks the plateau middle.
        let w = [Some(30), Some(28), Some(20), Some(10), Some(0)];
        assert_eq!(choose(&w, &codes).1, Some(10));
        assert!(choose(&w, &codes).0.is_some());
        // A flat top: centre in the middle of the plateau.
        let w = [Some(10), Some(30), Some(30), Some(30), Some(10)];
        assert_eq!(choose(&w, &codes), (Some(12), Some(11)));
    }

    #[test]
    fn sweep_config_is_checked() {
        let mut s = Sim::new().trained();
        let mut seed = 1;
        let m = discover(&mut s, &mut seed).unwrap();
        let zero = [[0u16; LANES]; DBYTES];
        let bad = Sweep {
            lo: 0,
            step: 0,
            n: 10,
        };
        assert_eq!(
            sweep(&mut s, &m, 0, &zero, &bad, &mut seed),
            Err(WrError::Config)
        );
        let bad = Sweep {
            lo: 0,
            step: 1,
            n: 129,
        };
        assert_eq!(
            sweep(&mut s, &m, 0, &zero, &bad, &mut seed),
            Err(WrError::Config)
        );
    }

    // The sim is both ports: two wrappers share it.
    struct Dq<'a>(&'a RefCell<Sim>);
    struct Mr<'a>(&'a RefCell<Sim>);
    impl WritePort for Dq<'_> {
        type Error = ();
        fn set_tx_dq(&mut self, db: usize, lane: usize, tg: usize, v: u16) -> Result<(), ()> {
            self.0.borrow_mut().set_tx_dq(db, lane, tg, v)
        }
        fn tx_dq(&mut self, db: usize, lane: usize, tg: usize) -> Result<u16, ()> {
            self.0.borrow_mut().tx_dq(db, lane, tg)
        }
        fn check(&mut self, seed: u32) -> Result<Folds, ()> {
            self.0.borrow_mut().check(seed)
        }
    }
    impl MrPort for Mr<'_> {
        type Error = ();
        fn mrw(&mut self, ch: usize, rank: usize, ma: u8, op: u8) -> Result<(), ()> {
            self.0.borrow_mut().mrw(ch, rank, ma, op)
        }
        fn mrr(&mut self, ch: usize, rank: usize, ma: u8) -> Result<u8, ()> {
            self.0.borrow_mut().mrr(ch, rank, ma)
        }
        fn settle(&mut self) {}
    }
}
