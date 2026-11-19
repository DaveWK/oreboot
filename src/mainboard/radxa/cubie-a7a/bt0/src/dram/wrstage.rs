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
//! the live probe blocks. The verdict is the host's check afterwards (MRR,
//! then 3 x 16 MiB memtest), plus the printed delays against the ones found
//! before (the PMU's in the hybrid).

use cubie_a7a_dram::phy::{csr, csr_access};
use cubie_a7a_dram::wrtrain::{
    self, pattern, BlindRun, BlindStart, Delays, Folds, LaneFit, Ledger, MrPort, Snapshot,
    WritePort, BLOCKS, DBYTES, LANES,
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

use wrtrain::probe::{Plan, BLOCK_BYTES};
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

/// The write port and its staged-write ledger; the MR port never writes a
/// delay, so its ledger stays clean.
struct Port<'a>(Bt0Hw, Ledger, &'a Plan);

/// `A7A_WR_APPLY` (module docs).
#[allow(non_snake_case)]
fn APPLY() -> bool {
    cubie_a7a_dram::knobs::env("A7A_WR_APPLY", crate::knob!("A7A_WR_APPLY")).is_some()
}

/// Undo the delays a failed stage left behind. The caller halts whether or
/// not this succeeds, so a restore error is only logged.
fn undo(p: &mut Port<'_>, before: &Snapshot) -> bool {
    wrtrain::restore(p, before);
    p.apply_restore();
    false
}

/// The final probe compare, propagated: a compare error fails the stage.
fn finish(p: &mut Port<'_>, before: &Snapshot, seed: &mut u32, m: &wrtrain::Map) -> bool {
    match wrtrain::final_check(m, seed, |seed| p.compare(seed)) {
        Ok(report) => {
            let bad = report.bad;
            log_coverage("final", p.2.required, report.covered);
            println!("[dram] WR: final probe check: failing blocks {bad:04x}");
            report.passes(p.2.required) || undo(p, before)
        }
        Err(e) => {
            println!("[dram] WR: final probe check failed: {e:?}; stopping");
            undo(p, before)
        }
    }
}

impl WritePort for Port<'_> {
    type Error = Error;
    fn set_tx_dq(&mut self, db: usize, lane: usize, tg: usize, v: u16) -> Result<(), Error> {
        csr_access(&mut self.0, true);
        self.0.write16(csr(wrtrain::tx_dq_csr(db, lane, tg)), v);
        csr_access(&mut self.0, false);
        self.1.wrote();
        Ok(())
    }
    fn tx_dq(&mut self, db: usize, lane: usize, tg: usize) -> Result<u16, Error> {
        csr_access(&mut self.0, true);
        let v = self.0.read16(csr(wrtrain::tx_dq_csr(db, lane, tg)));
        csr_access(&mut self.0, false);
        Ok(v)
    }
    fn check(&mut self, seed: u32) -> Result<Folds, Error> {
        self.compare(seed).map(|(f, _)| f)
    }
}

impl Port<'_> {
    fn apply_restore(&mut self) {
        if APPLY() {
            let hw = &mut self.0;
            if let Err(e) = self
                .1
                .apply(&mut || cubie_a7a_dram::blobless::apply_delays(hw, DBYTES / 2))
            {
                println!("[dram] WR: restoring delays failed: {e:?}");
                return;
            }
        }
        println!("[dram] WR: stage failed; restored initial delay settings");
    }
    fn compare(&mut self, seed: u32) -> Result<(Folds, u32), Error> {
        // Plain load and store: bt0 runs with the MMU off, where the
        // exclusive accesses of an atomic read-modify-write (swap) abort.
        // Fail closed: the pending flag clears only after a successful apply,
        // so an apply error propagates and leaves the stage failed.
        if APPLY() {
            let hw = &mut self.0;
            self.1
                .apply(&mut || cubie_a7a_dram::blobless::apply_delays(hw, DBYTES / 2))?;
        }
        for &b in self.2.blocks.iter() {
            for a in (b..b + BLOCK_BYTES).step_by(4) {
                // SAFETY: DRAM in mission mode; bt0 runs from SRAM and
                // nothing else uses these blocks.
                unsafe { core::ptr::write_volatile(a as *mut u32, pattern(a, seed)) };
            }
        }
        let mut f = [0u32; BLOCKS];
        let mut compared = 0;
        for (k, &b) in self.2.blocks.iter().enumerate() {
            for a in (b..b + BLOCK_BYTES).step_by(4) {
                // SAFETY: as above.
                let got = unsafe { core::ptr::read_volatile(a as *const u32) };
                f[k] |= got ^ pattern(a, seed);
            }
            compared |= 1 << k;
        }
        crate::recovery::kick();
        Ok((f, compared))
    }
}

impl MrPort for Port<'_> {
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

/// One nibble per timing group; one bit per physical DBYTE.
fn log_coverage(stage: &str, required: u8, observed: u8) {
    for db in 0..DBYTES {
        let groups = |mask: u8| ((mask >> db) & 1) | (((mask >> (DBYTES + db)) & 1) << 1);
        println!(
            "[dram] WR: {stage} ch{} DB{db} TG mask {:x}, required {:x}",
            db / 2,
            groups(observed),
            groups(required)
        );
    }
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
    use cubie_a7a_dram::ctrl::{addrmap, ddrc, MSTR0};
    let mstr = [0, 1].map(|ch| Bt0Hw.read32(ddrc(ch, MSTR0)));
    let am1 = [0, 1].map(|ch| Bt0Hw.read32(addrmap(ch, 1)));
    let Some(plan) = Plan::from_live(crate::mem::DRAM_BASE as u32, mstr, am1) else {
        println!("[dram] WR: unsupported geometry MSTR0 {mstr:08x?} ADDRMAP1 {am1:08x?}; stopping");
        return false;
    };
    let mut p = Port(Bt0Hw, Ledger::new(), &plan);
    let mut mp = Port(Bt0Hw, Ledger::new(), &plan);
    let mut seed = 0x5eed_0001u32;
    println!(
        "[dram] WR: stage bits {:x}, probe blocks {:08x?} x {BLOCK_BYTES} B",
        BITS(),
        plan.blocks
    );
    let before = match wrtrain::snapshot(&mut p) {
        Ok(b) => b,
        Err(e) => {
            println!("[dram] WR: reading the delays found before failed: {e:?}; stopping");
            return false;
        }
    };
    // A step that fails and is recovered from still fails the stage: the
    // hand-over gate must not be set after any write-stage failure.
    let mut degraded = false;
    let mut blind_start: Option<BlindStart> = None;
    if BITS() & 0x10 != 0 {
        match wrtrain::blind_baseline(&mut p, &mut seed) {
            Ok((v, left)) => {
                println!(
                    "[dram] WR: blind start: TxDqDly per DBYTE {v:03x?}, error bits left {left}"
                );
                let mut clean = left == 0;
                let mut delays = v;
                if clean
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
                            delays = c;
                            clean = left == 0;
                        }
                        Err(e) => {
                            println!("[dram] WR: blind centre failed: {e:?}; stopping");
                            return undo(&mut p, &before);
                        }
                    }
                }
                blind_start = Some(BlindStart {
                    delays,
                    error_bits: if clean { 0 } else { left },
                });
            }
            Err(e) => {
                println!("[dram] WR: blind start failed: {e:?}; stopping");
                return undo(&mut p, &before);
            }
        }
    }
    let blind_clean = blind_start.as_ref().is_some_and(BlindStart::clean);
    let m = match blind_start.as_ref() {
        Some(start) => {
            match wrtrain::discover_or_restore(&mut p, &mut seed, start, plan.required, &before) {
                BlindRun::Map(m) => m,
                BlindRun::KeepBlind(m, e) => {
                    println!(
                        "[dram] WR: discovery failed: {e:?}; the blind start is clean, keeping it"
                    );
                    log_coverage("discovery", plan.required, m.coverage(u32::MAX));
                    return finish(&mut p, &before, &mut seed, &m);
                }
                BlindRun::Failed(e) => {
                    if let wrtrain::WrError::Coverage(missing) = &e {
                        log_coverage("discovery", plan.required, plan.required & !missing);
                    }
                    p.apply_restore();
                    println!("[dram] WR: discovery failed: {e:?}; stopping");
                    return false;
                }
            }
        }
        None => match wrtrain::discover(&mut p, &mut seed) {
            Ok(m) => m,
            Err(e) => {
                println!("[dram] WR: discovery failed: {e:?}; stopping");
                return undo(&mut p, &before);
            }
        },
    };
    let observed = m.coverage(u32::MAX);
    log_coverage("discovery", plan.required, observed);
    if observed & plan.required != plan.required {
        return undo(&mut p, &before);
    }
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
    if blind_clean && BITS() & 0x20 == 0 {
        println!(
            "[dram] WR: discovery holds; no centring asked for (stage bit 20): the blind start is clean, keeping it"
        );
        return finish(&mut p, &before, &mut seed, &m);
    }
    let mut pmu = [[[0u16; LANES]; DBYTES]; 2];
    for (tg, d) in pmu.iter_mut().enumerate() {
        for (db, lanes) in d.iter_mut().enumerate() {
            for (lane, v) in lanes.iter_mut().enumerate() {
                *v = match p.tx_dq(db, lane, tg) {
                    Ok(v) => v,
                    Err(e) => {
                        println!("[dram] WR: reading a delay failed: {e:?}; stopping");
                        return undo(&mut p, &before);
                    }
                };
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
                degraded = true;
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
                    c[k] = match wrtrain::read_operating(&mut mp, ch, tg, ma) {
                        Ok(v) => v & 0x7f,
                        Err(e) => {
                            println!(
                                "[dram] WR: tg {tg} ch {ch} MR{ma} read failed: {e:?}; stopping"
                            );
                            return undo(&mut p, &before);
                        }
                    };
                }
            }
            println!("[dram] WR: tg {tg} operating MR14/MR15 per channel {cur:02x?}");
            let bytes = match wrtrain::identify_vref_bytes(
                &mut p, &mut mp, &m, tg, cur, VREF_BAD, &mut seed,
            ) {
                Ok(Some(b)) => b,
                r => {
                    println!("[dram] WR: tg {tg} Vref byte identification: {r:?}; no sweep");
                    degraded = true;
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
                                degraded = true;
                            }
                        }
                    }
                }
                Err(e) => {
                    println!("[dram] WR: Vref sweep tg {tg} failed: {e:?}");
                    degraded = true;
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
    if degraded {
        println!("[dram] WR: a step failed; the stage does not pass");
        return undo(&mut p, &before);
    }
    finish(&mut p, &before, &mut seed, &m)
}
