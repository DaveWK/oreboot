//! Blobless DRAM backend (feature `blobless`): no PHY firmware is built in,
//! loaded or run. Development stage: it runs the firmware-free PHY probe
//! (crate `cubie-a7a-dram`, module `blobless`), prints what it found and
//! halts. DRAM is not brought up yet, so the boot watchdog resets the board
//! and the next boot takes the recovery path (README, Recovery).

#[cfg(not(feature = "wck-alias-probe"))]
use cubie_a7a_dram::blobless::BlockTrain;
use cubie_a7a_dram::blobless::{self, Readback};
#[cfg(not(feature = "wck-alias-probe"))]
use cubie_a7a_dram::phy::coldinit::CbtBlock;
use cubie_a7a_dram::Para;

use super::bt0hw::Bt0Hw;
use crate::soc;

/// The pstate the vendor order programs and trains first.
const PROBE_PSTATE: u32 = 1;

fn report(name: &str, r: &Readback) {
    println!(
        "[dram] blobless: {name} readback {}/{} match, {} mismatches",
        r.matched, r.checked, r.mismatches
    );
    for &(csr, want, got) in r.first.iter().take(r.mismatches.min(8) as usize) {
        println!("[dram] blobless:   csr {csr:05x} wrote {want:04x} read {got:04x}");
    }
}

/// Print one CBT block's training result.
#[cfg(not(feature = "wck-alias-probe"))]
fn report_block(i: usize, b: &CbtBlock, t: &BlockTrain) {
    let ch = ["A", "B"][b.chan.ac as usize];
    let cs = blobless::CA_SWEEP_STEP as u32;
    let (s0, l0) = t.common_run;
    println!(
        "[dram] blobless: CBT {i} ch {ch} rank {} phase {}: common {}..+{} center {}, raw {:04x?} {:04x?}",
        b.rank,
        b.phase,
        s0 * cs,
        l0 * cs,
        t.common_center,
        t.raw_first,
        t.raw_mid
    );
    for bit in 0..8 {
        if b.chan.echo_bits() >> bit & 1 != 0 {
            let (s, l) = t.bit_runs[bit];
            println!(
                "[dram] blobless: CBT {i} bit {bit}: map {:032x} run {}..+{}",
                t.bit_maps[bit],
                s * cs,
                l * cs
            );
        }
    }
    let ls = blobless::LANE_SWEEP_STEP as u32;
    for lane in 0..7 {
        let (s, l) = t.lane_runs[lane];
        println!(
            "[dram] blobless: CBT {i} lane {lane}: bit {} window {}..+{} delay {}",
            t.lane_bit[lane],
            s * ls,
            l * ls,
            t.lane_delay[lane]
        );
    }
    println!(
        "[dram] blobless: CBT {i} trained: pass {:04x} (want {:04x}), echo {:04x?}",
        t.final_pass,
        b.chan.echo_bits(),
        t.final_echo
    );
}

/// The held-reset TxWckDly alias probe (feature `wck-alias-probe`): the
/// write trials first, printed, then the alias reads, printed last.
#[cfg(feature = "wck-alias-probe")]
fn wck_alias_probe(config: u16, revision: u16) {
    use cubie_a7a_dram::phy::wckalias::{self, Trial, ALIASES, DELAYS};
    let trial = |name: &str, t: &Trial| {
        println!(
            "[dram] WCK alias: {name} write {:05x}={:04x}: changed {:02x} after {:04x?} restored {:02x} mismatch",
            t.address, t.written, t.changed_mask, t.after, t.restore_mismatch_mask
        );
    };
    println!("[dram] WCK alias: base {DELAYS:05x?}");
    let r = wckalias::observe(&mut Bt0Hw, config, revision);
    println!(
        "[dram] WCK alias: refusal {:?}, before {:04x?}, stopped {:?}, mux {:?}",
        r.refusal, r.before, r.stopped, r.mux_restore
    );
    if let Some(c) = &r.control {
        trial("control", c);
    }
    for (k, t) in r.aliases.iter().enumerate() {
        if let Some(t) = t {
            trial("alias", t);
            println!(
                "[dram] WCK alias: {:05x} lands only on {:05x}: {}",
                ALIASES[k],
                DELAYS[k],
                t.lands_only_on(k)
            );
        }
    }
    println!("[dram] WCK alias: reading the alias addresses {ALIASES:05x?}");
    let a = wckalias::read_aliases(&mut Bt0Hw, config, revision);
    println!(
        "[dram] WCK alias: alias reads {:04x?}, base after {:04x?}, refusal {:?}, mux {:?}",
        a.values, a.base, a.refusal, a.mux_restore
    );
    println!("[dram] WCK alias: done; CSR observation only, no training or analog claim");
}

/// WL result storage, outside the 16 KiB stack (the per-point all-lane
/// counts are about 6 KiB). bt0 is single-threaded.
#[cfg(not(feature = "wck-alias-probe"))]
static mut WL_BLOCK: cubie_a7a_dram::wl::WlBlock = cubie_a7a_dram::wl::WlBlock::empty(
    cubie_a7a_dram::phy::wcklevel::Channel::A,
    cubie_a7a_dram::phy::wcklevel::Rank::R0,
);
#[cfg(not(feature = "wck-alias-probe"))]
static mut WL_SWEEP: cubie_a7a_dram::wl::WlSweep = cubie_a7a_dram::wl::WlSweep::EMPTY;

/// WCK2CK leveling on every channel and rank (crate module `wl`): our sweep
/// of the base TxWckDly CSRs with real DTSM feedback, DTSM controls and
/// TrainingCntr convergence runs, printed per block. Printing kicks the boot
/// watchdog per line (each line is bounded; the launches kick it as well).
#[cfg(not(feature = "wck-alias-probe"))]
fn wck_level(p: &mut Para) {
    use cubie_a7a_dram::phy::wcklevel::{Channel, Rank};
    use cubie_a7a_dram::wl::{self, SNAP_NAMES, WL_POINTS, WL_STEP};
    let bases = blobless::wl_mr18_bases(p, PROBE_PSTATE);
    let saved = wl::begin(&mut Bt0Hw);
    println!("[dram] WL: lane maps {saved:x?} saved, identity set; sweep {WL_POINTS} points step {WL_STEP}, feedback lanes {:?}, convergence starts {:?}", wl::FEEDBACK_LANES, wl::CONVERGENCE_STARTS);
    // SAFETY: bt0 is single-threaded and nothing else refers to these.
    let (b, sw) = unsafe {
        (
            &mut *core::ptr::addr_of_mut!(WL_BLOCK),
            &mut *core::ptr::addr_of_mut!(WL_SWEEP),
        )
    };
    for (k, &(ch, rank)) in wl::BLOCKS.iter().enumerate() {
        let name = (
            if ch == Channel::A { "A" } else { "B" },
            if rank == Rank::R0 { 0 } else { 1 },
        );
        let Some(mr18) = bases[k] else {
            println!(
                "[dram] WL ch {} rank {}: no MR18 base; stopping",
                name.0, name.1
            );
            break;
        };
        b.channel = ch;
        b.rank = rank;
        wl::level_block(&mut Bt0Hw, mr18, b, sw);
        let tag = |i: usize| (name.0, name.1, i);
        println!(
            "[dram] WL ch {} rank {}: MR18 base {:02x}, {} launches, {} writes, max {} us / {} polls, {} us, points {}, failure {:?}",
            name.0, name.1, b.mr18_base, b.launches, b.writes, b.max_launch_us, b.max_polls, b.elapsed_us, b.points, b.failure
        );
        for (n, pair) in b.snaps.iter().enumerate() {
            for (i, s) in pair.iter().enumerate() {
                crate::recovery::kick();
                let (c, r, i) = tag(i);
                println!(
                    "[dram] WL ch {c} rank {r} byte {i} snap {n} ({}): lanectrl {:x?} incdec {:x?} cntr {:04x?} cmp {:x?} err {:x?} good {:x?} nz {:x} bars {:x?} xing {:x?} bytectrl {:x?} param {:x}",
                    SNAP_NAMES[n], s.lane_ctrl, s.inc_dec_en, s.cntr, s.cmp, s.err, s.good, s.err_nz, s.bars, s.xing, s.byte_ctrl, s.training_param
                );
            }
        }
        for i in 0..2 {
            let (c, r, i) = tag(i);
            println!(
                "[dram] WL ch {c} rank {r} byte {i}: saved {:04x}, after prefix {:04x}, TrainingCntr {:04x?} -> {:?}, prefix err {}",
                b.saved_delay[i], b.after_prefix_delay[i], b.training_cntr[i], b.counter_delay[i], b.prefix_err[i]
            );
        }
        for (pt, d) in sw.points.iter().zip(0u16..).take(b.points as usize) {
            crate::recovery::kick();
            println!(
                "[dram] WL ch {} rank {} p {} d {}: b0 cmp {:x?} err {:x?} b1 cmp {:x?} err {:x?}",
                name.0,
                name.1,
                d,
                d * WL_STEP,
                pt.cmp[0],
                pt.err[0],
                pt.cmp[1],
                pt.err[1]
            );
        }
        for i in 0..2 {
            crate::recovery::kick();
            let (c, r, i) = tag(i);
            println!("[dram] WL ch {c} rank {r} byte {i}: err04 {:?}", b.err[i]);
            println!(
                "[dram] WL ch {c} rank {r} byte {i}: lanes {:x?}",
                b.lanes_nz[i]
            );
            let d = &b.decision[i];
            println!(
                "[dram] WL ch {c} rank {r} byte {i}: sweep cross-check {:?}, threshold {}, edges {:?} ({}), delay {:?}, counter on edge {}",
                b.verdict[i], d.threshold, &d.edges[..d.n_edges.min(8) as usize], d.n_edges, d.delay, d.counter_agrees
            );
        }
        for cv in b.convergence.iter() {
            crate::recovery::kick();
            println!(
                "[dram] WL ch {} rank {} converge from {} ({:04x}): {} launches, failure {:?}, result {:?}",
                name.0, name.1, cv.start, cv.start_raw, cv.launches, cv.failure, cv.result
            );
            for i in 0..2 {
                let (c, r, i) = tag(i);
                println!(
                    "[dram] WL ch {c} rank {r} byte {i} converge from {}: start read {:04x?} after {:04x?}",
                    cv.start, cv.start_read[i], cv.after[i]
                );
            }
        }
        for i in 0..2 {
            crate::recovery::kick();
            let (c, r, i) = tag(i);
            println!(
                "[dram] WL ch {c} rank {r} byte {i}: result (PHY counter fixed point) {:?}, final {:04x}",
                b.counter[i], b.final_delay[i]
            );
        }
        if b.failure.is_some() {
            break;
        }
    }
    let bad = wl::end(&mut Bt0Hw, &saved);
    println!("[dram] WL: lane maps restored, mismatch mask {bad:x}");
}

/// Probe the PHY with `para`, print the result and stop.
pub fn init_one(para: &[u32; 32]) -> Option<u32> {
    let mut p = Para::from_sysconfig(para);
    let mut rb = [Readback::default(); 2];
    let t0 = soc::ticks();
    let r = blobless::pre_training(&mut Bt0Hw, &mut p, PROBE_PSTATE, &mut rb);
    let ms = (soc::ticks() - t0) / (soc::TIMER_HZ / 1000);
    match r {
        Ok(id) => {
            println!("[dram] blobless: pstate {PROBE_PSTATE} pre-training done in {ms} ms");
            println!(
                "[dram] blobless: reset state MicroContMuxSel {:04x} MicroReset {:04x}",
                id.mux_before, id.micro_reset_before
            );
            println!(
                "[dram] blobless: PhyConfig (0x{:05x}) = {:04x}, PUBREV (0x{:05x}) = {:04x}",
                blobless::PHY_CONFIG,
                id.phy_config,
                blobless::PUB_REV,
                id.pub_rev
            );
            report("C_ONCE", &rb[0]);
            report("C_PSTATE", &rb[1]);
            #[cfg(feature = "wck-alias-probe")]
            wck_alias_probe(id.phy_config, id.pub_rev);
            #[cfg(not(feature = "wck-alias-probe"))]
            {
                let mut samples = [None; 8];
                blobless::acsm_delay_probe(&mut Bt0Hw, &mut samples);
                for a in samples.iter().flatten() {
                    println!(
                    "[dram] blobless: ACSM delay row ctl {:04x} stop {}: {:?}; LastAddr {:04x}, {} us",
                    a.ctl, a.stop, a.run, a.last_addr, a.elapsed_us
                );
                }
                // Cold init to CBT, then our own command bus training on the
                // DRAM's CBT echo for both channels, ranks and phases.
                let (m1, m2) = blobless::mode_registers(&mut p, PROBE_PSTATE);
                println!(
                "[dram] blobless: MRs from our parameters: {m1:x?} {m2:x?}; oracle constants equal: {}",
                m1 == cubie_a7a_dram::phy::coldinit::Mrs::ORACLE_PSTATE1_CXMT_4G
                    && m2 == cubie_a7a_dram::phy::coldinit::Mrs2::ORACLE_PSTATE1_CXMT_4G
            );
                let mut t = blobless::CaTrainReport::EMPTY;
                blobless::cold_init_ca_train(&mut Bt0Hw, &m1, &m2, &mut t);
                let e = &t.exec;
                println!(
                "[dram] blobless: CA train: {} blocks, {} launches ({} witnessed, max {} busy polls), {} us, failure {:?}",
                t.done, e.launches, e.witnessed, e.busy_max, e.elapsed_us, e.failure
            );
                let d = &t.devinit;
                println!(
                "[dram] blobless: devinit: failure {:?}, ZCalRate {:04x}, ZCalBusy {:04x} -> {:04x} after wait, {} polls",
                d.failure, d.zcal_rate, d.zcal_busy_start, d.zcal_busy_after_wait, d.zcal_polls
            );
                println!(
                "[dram] blobless: devinit: RxReplicaCtl04 {:04x}, after broadcast {:04x?}; DllLockParam {}; LCDL status AC {:04x} DX {:04x}",
                d.ctl04_before, d.ctl04_after, d.dll_lock_param, d.lcdl_status[0], d.lcdl_status[1]
            );
                for (n, r) in d.replica.iter().enumerate() {
                    println!(
                    "[dram] blobless: devinit: DBYTE {n} replica phases {:?} Ph1UI {} select {} ratio {}",
                    r.phases, r.ph1ui, r.select, r.ratio
                );
                }
                for (i, (b, r)) in cubie_a7a_dram::phy::coldinit::CBT_BLOCKS
                    .iter()
                    .zip(t.blocks.iter())
                    .enumerate()
                {
                    report_block(i, b, r);
                }
                for (ch, c) in t.channels.iter().enumerate() {
                    let ls = blobless::LANE_SWEEP_STEP as u32;
                    println!(
                    "[dram] blobless: CA ch {}: lane map agrees {}, delays {:?}, windows {}..+{} {}..+{} {}..+{} {}..+{} {}..+{} {}..+{} {}..+{}",
                    ["A", "B"][ch],
                    c.lane_map_agrees,
                    c.delay,
                    c.runs[0].0 * ls, c.runs[0].1 * ls,
                    c.runs[1].0 * ls, c.runs[1].1 * ls,
                    c.runs[2].0 * ls, c.runs[2].1 * ls,
                    c.runs[3].0 * ls, c.runs[3].1 * ls,
                    c.runs[4].0 * ls, c.runs[4].1 * ls,
                    c.runs[5].0 * ls, c.runs[5].1 * ls,
                    c.runs[6].0 * ls, c.runs[6].1 * ls,
                );
                }
                if t.done == 8 && t.exec.failure.is_none() {
                    wck_level(&mut p);
                } else {
                    println!("[dram] WL: skipped, CA training incomplete");
                }
            }
        }
        Err(e) => {
            println!("[dram] blobless: pre-training failed after {ms} ms: {e:?}");
        }
    }
    println!("[dram] blobless: no training backend yet; stopping");
    crate::halt()
}
