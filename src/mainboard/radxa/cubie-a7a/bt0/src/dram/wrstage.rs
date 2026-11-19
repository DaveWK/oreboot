// SPDX-License-Identifier: GPL-2.0-only
//! FEL lab, measurement only: our write DQ centring and write Vref
//! (`cubie_a7a_dram::wrtrain`) on the mission state, after the lean
//! hand-over. Env `A7A_WR_STAGE` (hex) at build time; unset: nothing runs.
//!
//! | bit | effect |
//! |---|---|
//! | 0x1 | discovery, then write DQ centring of every reachable timing group |
//! | 0x2 | after discovery, every TxDqDly to the reset value 0x20 (the untrained blobless state) before the centring |
//! | 0x4 | write Vref sweep (MR14/MR15, operating set-point) after the centring |
//! | 0x8 | measurement only: the delays and Vref codes found before go back at the end |
//! | 0x10 | before discovery, a blind clean start ([`wrtrain::blind_baseline`]: per DBYTE the delay with the fewest errors; for the blobless state, where nothing writes cleanly yet) |
//! | 0x20 | with 0x10: after a clean blind start and a discovery that holds, go on to the centring (every lane its own delay). Without it a clean blind start is kept as it is: the discovery only prints its map |
//!
//! The discovery prints the map it measured before it judges it
//! (`wrtrain::Map::check`), so a run whose map the rules refuse still
//! shows which lanes fail which bits. A centring that does not write
//! cleanly puts back the delays it found (`wrtrain::centre`).
//!
//! Env `A7A_WR_APPLY=1`: every compare first applies the TxDqDly writes
//! made since the last one (`cubie_a7a_dram::blobless::apply_delays`:
//! controller self-refresh so the DFI is idle, ForceInternalUpdate held,
//! self-refresh exit). On the board, delay CSR writes in mission
//! mode alone do not take effect, so without it the sweep measures
//! nothing. Unset: the writes are made as before (defaults unchanged).
//!
//! The compare is through the controller: the CPU writes and reads back
//! [`PROBE`] blocks. The verdict is the host's check afterwards (MRR,
//! then 3 x 16 MiB memtest), plus the printed delays against the ones found
//! before (the PMU's in the hybrid).

use cubie_a7a_dram::phy::{csr, csr_access};
use cubie_a7a_dram::wrtrain::{
    self, pattern, Delays, Folds, LaneFit, MrPort, WritePort, BLOCKS, DBYTES, LANES,
};
use cubie_a7a_dram::{Error, RegIo};

use super::bt0hw::Bt0Hw;

#[allow(non_snake_case)]
fn BITS() -> u32 {
    match cubie_a7a_dram::knobs::env("A7A_WR_STAGE", crate::knob!("A7A_WR_STAGE")) {
        Some(s) => match u32::from_str_radix(s, 16) {
            Ok(v) if v < 0x40 => v,
            _ => panic!("A7A_WR_STAGE is a hex mask below 0x40"),
        },
        None => 0,
    }
}

/// Probe blocks: 128 bytes each, at power-of-two offsets (so that any
/// channel interleave from 128 B to 8 KiB puts blocks on both channels),
/// at the bottom and the top of the 32-bit DRAM window (rank 1, if it is
/// mapped inside the window at all).
const PROBE: [usize; BLOCKS] = {
    let off = [0usize, 0x80, 0x100, 0x200, 0x400, 0x800, 0x1000, 0x2000];
    let mut p = [0usize; BLOCKS];
    let mut i = 0;
    while i < 8 {
        p[i] = 0x4010_0000 + off[i];
        p[i + 8] = 0xbf00_0000 + off[i];
        i += 1;
    }
    p
};
const BLOCK_BYTES: usize = 128;
/// Vref codes of the sweep (10 % + 0.5 % of VDDQ per code: 26 %..41 %;
/// at VDDQ 560 mV 146..230 mV, inside the 75..350 mV DQ Vref range at a
/// WCK up to 1600 MHz) and the code that identifies each MR's byte (15 %,
/// 84 mV).
const VREF_FIRST: u8 = 0x20;
const VREF_N: usize = 31;
const VREF_BAD: u8 = 0x0a;
/// The sweep's highest code at the stage's pstate: JEDEC's DQ Vref maximum
/// is 225 mV above a WCK of 1600 MHz (pstate 0: 2400 MHz), so at VDDQ
/// 560 mV the sweep stops at 0x3c there; at pstate 1 it is unchanged.
const VREF_MAX: u8 = cubie_a7a_dram::pstate::dq_vref_code_max(
    cubie_a7a_dram::pstate::Point::board(super::blobless::stage_pstate_build()).wck_mhz(),
    cubie_a7a_dram::pstate::BOARD_VDDQ_MV,
);
const VREF_N_CAPPED: usize = wrtrain::vref_n_capped(VREF_FIRST, 1, VREF_N, VREF_MAX);

struct Port(Bt0Hw);

/// `A7A_WR_APPLY` (module docs).
#[allow(non_snake_case)]
fn APPLY() -> bool {
    cubie_a7a_dram::knobs::env("A7A_WR_APPLY", crate::knob!("A7A_WR_APPLY")).is_some()
}
/// TxDqDly written since the last compare (applied by the next one).
static DIRTY: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);

impl WritePort for Port {
    type Error = Error;
    fn set_tx_dq(&mut self, db: usize, lane: usize, tg: usize, v: u16) -> Result<(), Error> {
        csr_access(&mut self.0, true);
        self.0.write16(csr(wrtrain::tx_dq_csr(db, lane, tg)), v);
        csr_access(&mut self.0, false);
        DIRTY.store(true, core::sync::atomic::Ordering::Relaxed);
        Ok(())
    }
    fn tx_dq(&mut self, db: usize, lane: usize, tg: usize) -> Result<u16, Error> {
        csr_access(&mut self.0, true);
        let v = self.0.read16(csr(wrtrain::tx_dq_csr(db, lane, tg)));
        csr_access(&mut self.0, false);
        Ok(v)
    }
    fn check(&mut self, seed: u32) -> Result<Folds, Error> {
        // Plain load and store: bt0 runs with the MMU off, where the
        // exclusive accesses of an atomic read-modify-write (swap) abort.
        if APPLY() && DIRTY.load(core::sync::atomic::Ordering::Relaxed) {
            DIRTY.store(false, core::sync::atomic::Ordering::Relaxed);
            cubie_a7a_dram::blobless::apply_delays(&mut self.0, DBYTES / 2)?;
        }
        for &b in self::probe!().iter() {
            for a in (b..b + BLOCK_BYTES).step_by(4) {
                // SAFETY: DRAM in mission mode; bt0 runs from SRAM and
                // nothing else uses these blocks.
                unsafe { core::ptr::write_volatile(a as *mut u32, pattern(a, seed)) };
            }
        }
        let mut f = [0u32; BLOCKS];
        for (k, &b) in self::probe!().iter().enumerate() {
            for a in (b..b + BLOCK_BYTES).step_by(4) {
                // SAFETY: as above.
                let got = unsafe { core::ptr::read_volatile(a as *const u32) };
                f[k] |= got ^ pattern(a, seed);
            }
        }
        crate::recovery::kick();
        Ok(f)
    }
}

impl MrPort for Port {
    type Error = Error;
    fn mrw(&mut self, ch: usize, rank: usize, ma: u8, op: u8) -> Result<(), Error> {
        cubie_a7a_dram::mr::write_rank(&mut self.0, ch, rank as u32, ma as u32, op as u32)
    }
    fn mrr(&mut self, ch: usize, rank: usize, ma: u8) -> Result<u8, Error> {
        cubie_a7a_dram::mr::read(&mut self.0, ch, rank as u32, ma as u32).map(|v| v as u8)
    }
    fn settle(&mut self) {
        // tVREF is not in our public sources yet:
        // a conservative wait.
        self.0.udelay(10);
    }
}

/// Two ports on one `Bt0Hw` (a unit struct): the write side and the MR side.
fn ports() -> (Port, Port) {
    (Port(Bt0Hw), Port(Bt0Hw))
}

fn print_delays(tag: &str, tg: usize, d: &Delays) {
    for (db, lanes) in d.iter().enumerate() {
        println!("[dram] WR {tag} tg {tg} DB{db}: {lanes:03x?}");
    }
}

/// The stage (see the module docs); nothing when `A7A_WR_STAGE` is unset.
/// Returns whether the probe blocks write cleanly at the end (true when
/// the stage is off).
pub fn run() -> bool {
    if BITS() == 0 {
        return true;
    }
    let (mut p, mut mp) = ports();
    let mut seed = 0x5eed_0001u32;
    println!(
        "[dram] WR: stage bits {:x}, probe blocks {PROBE:08x?} x {BLOCK_BYTES} B",
        BITS()
    );
    let mut blind_clean = false;
    if BITS() & 0x10 != 0 {
        match wrtrain::blind_baseline(&mut p, &mut seed) {
            Ok((v, left)) => {
                println!(
                    "[dram] WR: blind start: TxDqDly per DBYTE {v:03x?}, error bits left {left}"
                );
                blind_clean = left == 0;
                if blind_clean
                    && cubie_a7a_dram::knobs::env(
                        "A7A_WR_BLIND_CENTER",
                        crate::knob!("A7A_WR_BLIND_CENTER"),
                    )
                    .is_some()
                {
                    match wrtrain::blind_center(&mut p, v, &mut seed) {
                        Ok((c, run, left)) => {
                            println!(
                                "[dram] WR: blind centre: TxDqDly per DBYTE {c:03x?}, clean runs {run:03x?}, error bits left {left}"
                            );
                            blind_clean = left == 0;
                        }
                        Err(e) => {
                            println!("[dram] WR: blind centre failed: {e:?}; stopping");
                            return false;
                        }
                    }
                }
            }
            Err(e) => {
                println!("[dram] WR: blind start failed: {e:?}; stopping");
                return false;
            }
        }
    }
    let m = match wrtrain::measure(&mut p, &mut seed) {
        Ok(m) => m,
        Err(e) if blind_clean => {
            println!("[dram] WR: discovery failed: {e:?}; the blind start is clean, keeping it");
            return final_check(&mut p, &mut seed);
        }
        Err(e) => {
            println!("[dram] WR: discovery failed: {e:?}; stopping");
            return false;
        }
    };
    println!(
        "[dram] WR: blocks tg {:?}, DBYTEs {:x?}, on both tg {:04x}",
        m.block_tg, m.block_dbs, m.block_both
    );
    for db in 0..DBYTES {
        println!(
            "[dram] WR: DB{db} lane bits {:08x?}, reachable tg0 {} tg1 {}",
            m.lane_bits[db],
            m.reachable(db, 0),
            m.reachable(db, 1)
        );
    }
    match m.check::<Error>() {
        Ok(()) => {}
        Err(e) if blind_clean => {
            println!("[dram] WR: discovery failed: {e:?}; the blind start is clean, keeping it");
            return final_check(&mut p, &mut seed);
        }
        Err(e) => {
            println!("[dram] WR: discovery failed: {e:?}; stopping");
            return false;
        }
    }
    if blind_clean && BITS() & 0x20 == 0 {
        println!(
            "[dram] WR: discovery holds; no centring asked for (stage bit 20): the blind start is clean, keeping it"
        );
        return final_check(&mut p, &mut seed);
    }
    let mut pmu = [[[0u16; LANES]; DBYTES]; 2];
    for (tg, d) in pmu.iter_mut().enumerate() {
        for (db, lanes) in d.iter_mut().enumerate() {
            for (lane, v) in lanes.iter_mut().enumerate() {
                *v = p.tx_dq(db, lane, tg).unwrap_or(0);
            }
        }
        print_delays("before", tg, d);
    }
    if BITS() & 2 != 0 {
        for tg in 0..2 {
            for db in 0..DBYTES {
                for lane in 0..LANES {
                    let _ = p.set_tx_dq(db, lane, tg, 0x20);
                }
            }
        }
        println!("[dram] WR: every TxDqDly at 0x20 (untrained)");
    }
    let measure_only = BITS() & 8 != 0;
    let mut centred = [None; 2];
    let reached = [0, 1].map(|tg| (0..DBYTES).any(|db| m.reachable(db, tg)));
    for tg in 0..2 {
        if !reached[tg] {
            println!("[dram] WR: tg {tg} not reachable from the probe blocks; left as it is");
            continue;
        }
        match wrtrain::centre(&mut p, &m, tg, true, &mut seed) {
            Ok(c) => {
                print_delays("ours", tg, &c.applied);
                for (db, pmu_db) in pmu[tg].iter().enumerate() {
                    let mut delta = [0i16; LANES];
                    let mut width = [0u16; LANES];
                    for lane in 0..LANES {
                        delta[lane] = c.applied[db][lane] as i16 - pmu_db[lane] as i16;
                        width[lane] = c.fine[db][lane].width().unwrap_or(0);
                    }
                    let kinds = c.fine[db].map(|f| match f {
                        LaneFit::Centre { .. } => 'c',
                        LaneFit::AtEdge { .. } => 'e',
                        LaneFit::NoWindow => 'n',
                        LaneFit::Unmeasured => '-',
                    });
                    println!(
                        "[dram] WR tg {tg} DB{db}: ours - before {delta:?}, fine widths {width:?}, fits {kinds:?}"
                    );
                }
                centred[tg] = Some(c.applied);
            }
            Err(e) => {
                println!(
                    "[dram] WR: centring tg {tg} failed: {e:?}; the delays found before are back"
                );
            }
        }
    }
    if BITS() & 0x10 != 0 && !measure_only {
        // Blind (blobless) mode: a timing group the probe blocks cannot
        // reach (rank 1 above the 32-bit window) would keep the untrained
        // 0x20 and corrupt every later write to that rank. Take the other
        // group's result for it (the PMU's two groups agree within +/-3
        // units in a board dump); flagged, unmeasured.
        for tg in 0..2 {
            // Only a group out of reach: one whose centring failed has its
            // own delays back and keeps them.
            if reached[tg] {
                continue;
            }
            if let Some(other) = centred[1 - tg] {
                for (db, lanes) in other.iter().enumerate() {
                    for (lane, &v) in lanes.iter().enumerate() {
                        let _ = p.set_tx_dq(db, lane, tg, v);
                    }
                }
                println!(
                    "[dram] WR: tg {tg} unmeasured: set from tg {} (unverified)",
                    1 - tg
                );
            }
        }
    }
    if BITS() & 4 != 0 {
        let snapshot = centred;
        for (tg, slot) in snapshot.iter().enumerate() {
            let Some(centres) = *slot else {
                continue;
            };
            let mut cur = [[0u8; 2]; 2];
            for (ch, c) in cur.iter_mut().enumerate() {
                for (k, ma) in [14u8, 15].into_iter().enumerate() {
                    c[k] = wrtrain::read_operating(&mut mp, ch, tg, ma).unwrap_or(0x2e) & 0x7f;
                }
            }
            println!("[dram] WR: tg {tg} operating MR14/MR15 per channel {cur:02x?}");
            let bytes = match wrtrain::identify_vref_bytes(
                &mut p, &mut mp, &m, tg, cur, VREF_BAD, &mut seed,
            ) {
                Ok(Some(b)) => b,
                r => {
                    println!("[dram] WR: tg {tg} Vref byte identification: {r:?}; no sweep");
                    continue;
                }
            };
            println!("[dram] WR: tg {tg} MR14/MR15 drive DBYTEs {bytes:?}");
            let (codes, n) = wrtrain::vref_codes(VREF_FIRST, 1, VREF_N_CAPPED);
            match wrtrain::vref_sweep(
                &mut p,
                &mut mp,
                &m,
                tg,
                &bytes,
                &codes[..n],
                cur,
                &centres,
                !measure_only,
                &mut seed,
            ) {
                Ok(r) => {
                    for db in 0..DBYTES {
                        let w = r.widths[db].map(|x| x.unwrap_or(0));
                        println!(
                            "[dram] WR Vref tg {tg} DB{db}: codes {:02x}.. widths {:?}, chosen {:02x?} (max-min {:02x?})",
                            codes[0],
                            &w[..n],
                            r.chosen[db],
                            r.max_min[db]
                        );
                    }
                    if !measure_only {
                        match wrtrain::centre(&mut p, &m, tg, true, &mut seed) {
                            Ok(c) => {
                                print_delays("ours@vref", tg, &c.applied);
                                centred[tg] = Some(c.applied);
                            }
                            Err(e) => {
                                println!("[dram] WR: re-centring tg {tg} failed: {e:?}");
                            }
                        }
                    }
                }
                Err(e) => {
                    println!("[dram] WR: Vref sweep tg {tg} failed: {e:?}");
                }
            }
        }
    }
    if measure_only {
        for (tg, d) in pmu.iter().enumerate() {
            for (db, lanes) in d.iter().enumerate() {
                for (lane, &v) in lanes.iter().enumerate() {
                    let _ = p.set_tx_dq(db, lane, tg, v);
                }
            }
        }
        println!("[dram] WR: measurement only; the delays found before are back");
    }
    final_check(&mut p, &mut seed)
}

/// Four compares of every probe block with fresh seeds; true when all are
/// clean.
fn final_check(p: &mut Port, seed: &mut u32) -> bool {
    let mut bad = 0u32;
    for _ in 0..4 {
        *seed = seed.wrapping_add(0x9e37_79b9);
        if let Ok(f) = p.check(*seed) {
            for (b, v) in f.iter().enumerate() {
                if *v != 0 {
                    bad |= 1 << b;
                }
            }
        }
    }
    println!("[dram] WR: final probe check: failing blocks {bad:04x}");
    bad == 0
}

/// The probe blocks of a compare: [`PROBE`] itself (the same tokens as
/// before the feature existed, so images without it do not change), or
/// with feature `wr-r1` the list of [`r1::blocks`]. Used by path
/// (`self::probe!()`): a `use`d macro is in scope before its definition.
#[cfg(not(feature = "wr-r1"))]
macro_rules! probe_blocks {
    () => {
        PROBE
    };
}
#[cfg(feature = "wr-r1")]
macro_rules! probe_blocks {
    () => {
        r1::blocks()
    };
}
use probe_blocks as probe;

/// Feature `wr-r1` (the 6 GB board's preparation): the upper eight
/// probe blocks go to 1 MiB above the start of rank 1's lowest block
/// instead of [`PROBE`]'s 0xbf000000, so that the stage's compares (blind
/// start, discovery, centring, the final check) cover both ranks. Where
/// rank 1 lies comes from the address map the controller holds (ADDRMAP1
/// of channel A read back; `cubie_a7a_dram::ctrl::RankMap`):
///
/// | part | rank bit | rank 1 from | upper blocks |
/// |---|---|---|---|
/// | CXMT 4 GB | DRAM offset bit 31 | 0xc0000000 | 0xc0100000 |
/// | CXMT 6 GB (non-binary) | DRAM offset bit 30 | 0x80000000 | 0x80100000 |
///
/// Without the feature 0xbf000000 is rank 0 on the 4 GB part (timing
/// group 1 is never written or read by this stage) and rank 1 on the 6 GB
/// part. One rank, or a rank 1 that starts above 4 GiB: [`PROBE`] is kept
/// and a line says so.
#[cfg(feature = "wr-r1")]
mod r1 {
    use super::{Bt0Hw, BLOCKS, BLOCK_BYTES, PROBE};
    use cubie_a7a_dram::RegIo;

    /// [`PROBE`] with the upper eight blocks moved, once set.
    static mut R1_PROBE: [usize; BLOCKS] = PROBE;
    static mut R1_READY: bool = false;

    /// Start of the upper eight probe blocks for a rank bit at DRAM offset
    /// bit `rank_bit`: 1 MiB into rank 1's lowest block, if the blocks end
    /// below 4 GiB.
    const fn r1_base(rank_bit: u32) -> Option<usize> {
        if rank_bit >= 32 {
            return None;
        }
        let a = crate::mem::DRAM_BASE as u64 + (1u64 << rank_bit) + 0x10_0000;
        if a + (PROBE[BLOCKS - 1] - PROBE[8]) as u64 + BLOCK_BYTES as u64 <= 1 << 32 {
            Some(a as usize)
        } else {
            None
        }
    }
    const _: () = assert!(matches!(r1_base(31), Some(0xc010_0000)));
    const _: () = assert!(matches!(r1_base(30), Some(0x8010_0000)));
    const _: () = assert!(r1_base(32).is_none() && r1_base(40).is_none());

    /// The probe blocks of this run: the upper eight in rank 1.
    pub fn blocks() -> &'static [usize; BLOCKS] {
        // SAFETY: bt0 is single-threaded; the statics are only used here.
        unsafe {
            let b = &mut *core::ptr::addr_of_mut!(R1_PROBE);
            if !core::ptr::read_volatile(core::ptr::addr_of!(R1_READY)) {
                core::ptr::write_volatile(core::ptr::addr_of_mut!(R1_READY), true);
                let am1 = Bt0Hw.read32(cubie_a7a_dram::ctrl::addrmap(0, 1));
                let bit = cubie_a7a_dram::ctrl::RankMap::bit_from_addrmap1(am1, 2);
                match bit.and_then(r1_base) {
                    Some(base) => {
                        for i in 8..BLOCKS {
                            b[i] = base + (PROBE[i] - PROBE[8]);
                        }
                        println!(
                            "[dram] WR: wr-r1: ADDRMAP1 {am1:#x}, rank bit {bit:?}: probe blocks 8..15 at {base:08x} (rank 1), not {:08x}",
                            PROBE[8]
                        );
                    }
                    None => {
                        println!(
                            "[dram] WR: wr-r1: ADDRMAP1 {am1:#x}, rank bit {bit:?}: no rank 1 below 4 GiB; probe blocks unchanged"
                        );
                    }
                }
            }
            b
        }
    }
}
