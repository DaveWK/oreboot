// SPDX-License-Identifier: GPL-2.0-only
#![cfg_attr(feature = "hybrid", allow(dead_code))]
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
/// The probe's pstate: 1, or 3 (the lowest, 400 MT/s by the parameter set's
/// dividers) with feature `probe-ps3` (a timing-independence test), or
/// the build's `A7A_PSTATE` (`cubie_a7a_dram::pstate::ACTIVE`; 0 is the
/// mission rate, 2400 MHz WCK). The stage generators of the DRAM crate
/// follow `A7A_PSTATE`, so the two can never disagree.
const PROBE_PSTATE: u32 = if cubie_a7a_dram::pstate::ACTIVE_FROM_ENV {
    cubie_a7a_dram::pstate::ACTIVE
} else if cfg!(feature = "probe-ps3") {
    3
} else {
    1
};
const _: () = assert!(
    !(cfg!(feature = "probe-ps3") && cubie_a7a_dram::pstate::ACTIVE_FROM_ENV),
    "feature probe-ps3 and A7A_PSTATE select the pstate twice"
);

/// The pstate our stages run at: the hybrid's trained pstate (A7A_OPEN_OPTS
/// bits 3-5), else [`PROBE_PSTATE`].
/// [`stage_pstate`] from the build's knobs (const contexts).
pub(super) const fn stage_pstate_build() -> u32 {
    #[cfg(feature = "hybrid")]
    {
        hybrid_pstate_build()
    }
    #[cfg(not(feature = "hybrid"))]
    {
        PROBE_PSTATE
    }
}

pub(super) fn stage_pstate() -> u32 {
    #[cfg(feature = "hybrid")]
    {
        hybrid_pstate()
    }
    #[cfg(not(feature = "hybrid"))]
    {
        PROBE_PSTATE
    }
}

/// Our WCK2CK leveling and read training (their ACSM programs and clock
/// settings) are built for `cubie_a7a_dram::pstate::ACTIVE`: a hybrid that
/// runs them must have trained that pstate.
#[cfg(feature = "hybrid")]
const _: () = assert!(
    (option_env!("A7A_HYBRID_NO_WL").is_some() && option_env!("A7A_HYBRID_ENDINIT").is_none())
        || hybrid_pstate_build() == cubie_a7a_dram::pstate::ACTIVE,
    "hybrid WL/RD at a pstate other than A7A_PSTATE (set A7A_PSTATE to the trained pstate)"
);

/// The parameter set's DfiClk at the stage pstate must be the one the
/// generators were built for (`pstate::BOARD_DFI_MHZ`); prints and returns
/// false otherwise.
fn clock_matches(p: &Para) -> bool {
    use cubie_a7a_dram::pstate::{Point, ACTIVE};
    let ps = stage_pstate();
    let have = Point::of(p, ps);
    let want = Point::board(ps);
    if have != want || (cubie_a7a_dram::pstate::ACTIVE_FROM_ENV && ps != ACTIVE) {
        println!(
            "[dram] PSTATE: stages at pstate {ps} need DfiClk {} MHz (generators built for pstate {ACTIVE}); the set gives {} MHz: not running them",
            want.dfi_mhz, have.dfi_mhz
        );
        return false;
    }
    println!(
        "[dram] PSTATE: our stages at pstate {ps}: DfiClk {} MHz, WCK {} MHz, {} MT/s, UI {} ps; low-speed clock {:x?}, DLL gain {:04x}",
        have.dfi_mhz,
        have.wck_mhz(),
        have.data_rate(),
        have.ui_ps(),
        cubie_a7a_dram::pstate::ACTIVE_LOW_SPEED,
        cubie_a7a_dram::pstate::ACTIVE_DLL_GAIN
    );
    true
}

/// FEL lab `A7A_WL_MR16` (hex): MRW MR16 to all ranks right before WCK2CK
/// leveling, so that its MR18 writes go to a set-point other than the
/// operating one (measured: WL must not run with FSP-WR = FSP-OP). The PMU
/// leaves 0x04 at pstate 1; what it leaves at pstate 0 is read by the
/// pstate-0 control run. Unset: nothing.
const WL_MR16: Option<u8> = match option_env!("A7A_WL_MR16") {
    Some(s) => match u8::from_str_radix(s, 16) {
        Ok(v) => Some(v),
        Err(_) => panic!("A7A_WL_MR16 is not a hex byte"),
    },
    None => None,
};

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
/// Print WL in full (snapshots, sweep points, curves, convergence detail).
/// Off by default: WL is qualified and repeats; its summary lines
/// (block, knob, cross-check, convergence, result) are always printed.
#[cfg(not(feature = "wck-alias-probe"))]
const WL_VERBOSE: bool = false;

#[cfg(not(feature = "wck-alias-probe"))]
static mut WL_SWEEP: cubie_a7a_dram::wl::WlSweep = cubie_a7a_dram::wl::WlSweep::EMPTY;

/// WCK2CK leveling on every channel and rank (crate module `wl`): the PHY
/// counter fixed point as the result, our sweep of the base TxWckDly CSRs
/// with real DTSM feedback as the cross-check (TrainingParam 0, knob
/// checks), DTSM controls and the convergence and framing runs, printed per
/// block. Printing kicks the boot
/// watchdog per line (each line is bounded; the launches kick it as well).
#[cfg(not(feature = "wck-alias-probe"))]
#[inline(never)]
fn wck_level(p: &mut Para) {
    use cubie_a7a_dram::phy::wcklevel::{Channel, Rank};
    use cubie_a7a_dram::wl::{self, SNAP_NAMES, WL_POINTS, WL_STEP};
    let bases = blobless::wl_mr18_bases(p, stage_pstate());
    if let Some(v) = WL_MR16 {
        let f = blobless::mr16_all(&mut Bt0Hw, v);
        println!("[dram] WL: MR16 {v:02x} to all ranks before leveling: failure {f:?}");
    }
    {
        use cubie_a7a_dram::phy::wcklevel::Mr18Op;
        let ops = cubie_a7a_dram::knobs::get().wl_mr18_ops;
        if ops != [Mr18Op::Reference; 4] {
            println!("[dram] WL: MR18 ops (entry, level, restore, cleanup) {ops:x?}");
        }
    }
    let saved = wl::begin(&mut Bt0Hw);
    println!("[dram] WL: lane maps {saved:x?} saved, identity set; sweep {WL_POINTS} points step {WL_STEP}, feedback lanes {:?}, convergence starts {:?}, framing starts {:?}, knob {} launches at counter -/+ {}", wl::FEEDBACK_LANES, wl::CONVERGENCE_STARTS, wl::FRAME_STARTS, wl::KNOB_LAUNCHES, wl::KNOB_OFFSET);
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
        for (n, pair) in b.snaps.iter().enumerate().filter(|_| WL_VERBOSE) {
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
        for (pt, d) in
            sw.points
                .iter()
                .zip(0u16..)
                .take(if WL_VERBOSE { b.points as usize } else { 0 })
        {
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
            if WL_VERBOSE {
                println!("[dram] WL ch {c} rank {r} byte {i}: err04 {:?}", b.err[i]);
                println!(
                    "[dram] WL ch {c} rank {r} byte {i}: lanes {:x?}",
                    b.lanes_nz[i]
                );
            }
            let k = &b.knob[i];
            println!(
                "[dram] WL ch {c} rank {r} byte {i}: knob param {:x} mode {:?} curve {:?} two-point at {:?} err {:?} -> {:?}",
                k.training_param, k.mode, k.curve, k.settings, k.err, k.two_point
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
            for i in (0..2).filter(|_| WL_VERBOSE) {
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
                "[dram] WL ch {c} rank {r} byte {i}: result (PHY counter fixed point) {:?}, limits {:?}, final {:04x}; framing (all starts) {:?}",
                b.counter[i], b.knob[i].limits, b.final_delay[i], b.frame[i]
            );
        }
        if b.failure.is_some() {
            break;
        }
    }
    let bad = wl::end(&mut Bt0Hw, &saved);
    println!("[dram] WL: lane maps restored, mismatch mask {bad:x}");
    if blobless::WL_TXWCK_ADD() != 0 {
        let after = blobless::wl_txwck_add(&mut Bt0Hw);
        println!(
            "[dram] WL: TxWckDly +{:x} (held update): Tg0 per DBYTE {after:03x?}",
            blobless::WL_TXWCK_ADD()
        );
    }
    #[cfg(feature = "hybrid")]
    if cubie_a7a_dram::knobs::get().hybrid_skip_rd {
        let r = blobless::mission_handover(&mut Bt0Hw, p, stage_pstate());
        println!("[dram] MISSION (hybrid, no read training): {r:?}");
        if r.is_ok() {
            let _ = super::wrstage::run();
        }
        return;
    }
    if b.failure.is_none() {
        #[cfg(feature = "rd-measure")]
        read_enable(bases);
        #[cfg(not(feature = "rd-measure"))]
        {
            if let Some((before, _)) = blobless::RD_FSPWR() {
                let f = blobless::mr16_all(&mut Bt0Hw, before);
                println!("[dram] RD: MR16 {before:02x} to all ranks before the read training: failure {f:?}");
            }
            if blobless::RD_PULSE_RESTORE() {
                let v = blobless::acsm_pulse_restore(&mut Bt0Hw);
                println!(
                    "[dram] RD: ACSM read pulses -{} before the read training: {v:03x?}",
                    blobless::ACSM_PULSE_ADJUST
                );
            }
            let applied = read_training(bases);
            if let Some((_, after)) = blobless::RD_FSPWR() {
                let f = blobless::mr16_all(&mut Bt0Hw, after);
                println!("[dram] RD: MR16 {after:02x} to all ranks after the read training: failure {f:?}");
            }
            #[cfg(any(feature = "bl-mission", feature = "hybrid"))]
            if applied {
                let relock = cubie_a7a_dram::knobs::get().bl_relock;
                let r = blobless::mission_handover_relock(&mut Bt0Hw, p, stage_pstate(), relock);
                println!("[dram] MISSION: PLL/DLL relock before the DFI init: {relock}");
                println!(
                    "[dram] MISSION: lean hand-over at pstate {}: {r:?}",
                    stage_pstate()
                );
                if r.is_ok() {
                    let clean = super::wrstage::run();
                    #[cfg(feature = "bl-mission")]
                    // SAFETY: bt0 is single-threaded.
                    unsafe {
                        MISSION_OK = clean;
                    }
                    let _ = clean;
                }
            } else {
                println!("[dram] MISSION: skipped, read training not applied on every block");
            }
            #[cfg(feature = "handover")]
            if applied {
                handover_stage(p);
            } else {
                println!("[dram] HO: skipped, read training not applied on every block");
            }
            let _ = applied;
        }
    } else {
        println!("[dram] RD: skipped, WL incomplete");
    }
}

/// The vendor library's trained receive clock delays at 1200 MHz (feature
/// `av3c`: one pstate at 1200 MHz, dumped after training; two UART-damaged
/// characters restored from the same lines). A **measurement starting point** for the read enable
/// stage, never a default; per DBYTE, RxClkT2UIDlyTg0/Tg1, RxClkC2UIDlyTg0/Tg1,
/// lanes 0..8, and the same dump's VrefDAC0/VrefDAC1 per lane
/// (the receiver references; ours are a constant 0x35 from the init
/// tables).
#[cfg(not(feature = "wck-alias-probe"))]
const AV3C_START: cubie_a7a_dram::rd::ReadStart = cubie_a7a_dram::rd::ReadStart {
    rx_clk: [
        [
            [
                0x01bf, 0x01b9, 0x01bc, 0x0201, 0x01bc, 0x01be, 0x01be, 0x0202, 0x01b7,
            ],
            [
                0x01bf, 0x01b9, 0x01bd, 0x0202, 0x01bc, 0x01be, 0x01bf, 0x0203, 0x01b8,
            ],
            [
                0x01bd, 0x01b8, 0x01bc, 0x0200, 0x01bc, 0x01bd, 0x01be, 0x0201, 0x01b7,
            ],
            [
                0x01bd, 0x01b7, 0x01bb, 0x0201, 0x01bc, 0x01bd, 0x01bd, 0x0201, 0x01b6,
            ],
        ],
        [
            [
                0x01b9, 0x01b8, 0x01b7, 0x01b9, 0x0201, 0x01b8, 0x01c0, 0x01b9, 0x01ba,
            ],
            [
                0x01ba, 0x01b8, 0x01b9, 0x01b9, 0x0202, 0x01b8, 0x01bf, 0x01bb, 0x01ba,
            ],
            [
                0x01b7, 0x01b8, 0x01b9, 0x01ba, 0x0202, 0x01b8, 0x01bf, 0x01ba, 0x01bb,
            ],
            [
                0x01b9, 0x01b7, 0x01b9, 0x01b9, 0x0201, 0x01b7, 0x01bf, 0x01bb, 0x01bb,
            ],
        ],
        [
            [
                0x01bb, 0x01b9, 0x01b8, 0x01bd, 0x01b8, 0x01bb, 0x01bc, 0x01c0, 0x01be,
            ],
            [
                0x01bc, 0x01b9, 0x01b7, 0x01bd, 0x01b8, 0x01bb, 0x01bc, 0x01c0, 0x01bd,
            ],
            [
                0x01bb, 0x01b8, 0x01b6, 0x01be, 0x01b7, 0x01b9, 0x01bc, 0x01bf, 0x01be,
            ],
            [
                0x01bb, 0x01b7, 0x01b6, 0x01bd, 0x01b7, 0x01b9, 0x01bc, 0x01bf, 0x01bd,
            ],
        ],
        [
            [
                0x01bd, 0x01b9, 0x01b8, 0x0200, 0x01b7, 0x01b7, 0x0200, 0x01b8, 0x01b9,
            ],
            [
                0x01bf, 0x01bb, 0x01b9, 0x0201, 0x01b8, 0x01b8, 0x0202, 0x01b9, 0x01ba,
            ],
            [
                0x01bf, 0x01ba, 0x01b9, 0x0200, 0x01b7, 0x01b8, 0x0200, 0x01b7, 0x01ba,
            ],
            [
                0x01be, 0x01b9, 0x01b8, 0x0200, 0x01b6, 0x01b6, 0x0200, 0x01b7, 0x01b9,
            ],
        ],
    ],
    vref_dac: [
        [
            [
                0x002e, 0x0037, 0x0032, 0x0040, 0x0033, 0x0033, 0x0039, 0x0040, 0x002e,
            ],
            [
                0x003b, 0x0033, 0x003a, 0x0032, 0x0034, 0x0036, 0x0035, 0x0035, 0x0034,
            ],
        ],
        [
            [
                0x0035, 0x0035, 0x0033, 0x003a, 0x003d, 0x0035, 0x0039, 0x0032, 0x0036,
            ],
            [
                0x0031, 0x0036, 0x0039, 0x0031, 0x0035, 0x0038, 0x0030, 0x0035, 0x002e,
            ],
        ],
        [
            [
                0x0035, 0x002f, 0x0037, 0x0037, 0x0037, 0x0041, 0x002f, 0x0031, 0x0032,
            ],
            [
                0x002c, 0x0030, 0x002f, 0x0030, 0x0033, 0x002c, 0x0039, 0x0034, 0x002f,
            ],
        ],
        [
            [
                0x0039, 0x0033, 0x0038, 0x0037, 0x0034, 0x002e, 0x0037, 0x0034, 0x0033,
            ],
            [
                0x0036, 0x0034, 0x0033, 0x0034, 0x0033, 0x0032, 0x0036, 0x0036, 0x003a,
            ],
        ],
    ],
};

#[cfg(all(not(feature = "wck-alias-probe"), feature = "rd-measure"))]
static mut RD_BLOCK: cubie_a7a_dram::rd::RdBlock = cubie_a7a_dram::rd::RdBlock::empty(
    cubie_a7a_dram::phy::wcklevel::Channel::A,
    cubie_a7a_dram::phy::wcklevel::Rank::R0,
);

/// The latched RxEnDly x (with the DLL 1-UI codes after every step)
/// receive clock search (crate module `rd`;
/// rows are printed by the stage as they complete) on ch A rank 0, ch B
/// rank 0, each rank-1 block with Tg1 and with Tg0 swept, then ch A rank 0
/// again (the order test); per-lane eye totals after each run; everything
/// put back.
#[cfg(all(not(feature = "wck-alias-probe"), feature = "rd-measure"))]
#[inline(never)]
fn read_enable(bases: [Option<cubie_a7a_dram::phy::wcklevel::Mr18Base>; 4]) {
    use cubie_a7a_dram::phy::rxen::ModeMrs;
    use cubie_a7a_dram::phy::wcklevel::{Channel, Rank};
    use cubie_a7a_dram::rd;
    use cubie_a7a_dram::rd::TgSel;
    use cubie_a7a_dram::wl;
    // The order test: ch A rank 0 twice, ch B
    // rank 0, the first rank-1 block, then rank 0 again on both channels;
    // a PHY CSR snapshot at every block start.
    const RUNS: [(Channel, Rank, TgSel); 6] = [
        (Channel::A, Rank::R0, TgSel::Own),
        (Channel::A, Rank::R0, TgSel::Own),
        (Channel::B, Rank::R0, TgSel::Own),
        (Channel::A, Rank::R1, TgSel::Tg1),
        (Channel::A, Rank::R0, TgSel::Own),
        (Channel::B, Rank::R0, TgSel::Own),
    ];
    let search = rd::RdConfig::PSTATE1.search;
    println!(
        "[dram] RD: read path (pstate {}): no PState flip; DLL 1-UI codes after every step of each block; exit mode launch right after the exit; PHY CSR snapshot at each block start; order {:?}; data mode after the exit, QC set-up (latency pulse A {:x} B {:x}, DtsmLaneCtrl0 {:x?}, centring outer {}), receive clock delays as found, VrefDAC from the vendor's trained values; search {:x?} (RxEnDly latched by PptRxEnEvnt, RxClk lane F latched by ForceInternalUpdate, RxFifoInit, one launch); latch events (CSR, read at, bits) {:05x?} held {} us; rows: passing lanes per RxClk point and DBYTE ('.' none, 'w' none but the FIFO written); no refresh is issued; vendor trained point RxEnDly {:x} RxClk {:x}",
        PROBE_PSTATE,
        RUNS,
        rd::latency_mask(Channel::A),
        rd::latency_mask(Channel::B),
        rd::LANE_CTL,
        cubie_a7a_dram::phy::rxen::CENTRING_OUTER,
        search,
        rd::EVENTS,
        rd::HOLD_US,
        rd::VENDOR_RX_EN,
        rd::VENDOR_CLK
    );
    // SAFETY: bt0 is single-threaded and nothing else refers to it.
    let b = unsafe { &mut *core::ptr::addr_of_mut!(RD_BLOCK) };
    let mut prev = None;
    // The mode registers the flow believes it last wrote per channel and
    // rank (MR18, MR10, MR46, MR20, MR33, MR34); None before our first
    // block there (WL / end of initialization values).
    let mut mrs: [[Option<[u8; 6]>; 2]; 2] = [[None; 2]; 2];
    for (n, &(ch, rank, tg)) in RUNS.iter().enumerate() {
        let c = if ch == Channel::A { "A" } else { "B" };
        let r = if rank == Rank::R0 { 0 } else { 1 };
        let k = wl::BLOCKS
            .iter()
            .position(|&x| x == (ch, rank))
            .unwrap_or(0);
        b.channel = ch;
        b.rank = rank;
        // pstate 1: the reference's mode MRs and the vendor's trained
        // references; another pstate: MR18 from the block's WL base, no
        // vendor starts
        // (they are pstate 1 values).
        let (mut cfg, start) = if PROBE_PSTATE == 1 {
            (rd::RdConfig::PSTATE1, Some(&AV3C_START))
        } else {
            let base = bases[k].map(|m| m.value()).unwrap_or(0x0c);
            (
                rd::RdConfig {
                    next: ModeMrs {
                        mr18: base | 0x10,
                        mr10: 0x59,
                        mr46: 0,
                    },
                    exit: ModeMrs {
                        mr18: base,
                        mr10: 0x54,
                        mr46: 0,
                    },
                    qc_rx_en: 0x400,
                    search,
                },
                None,
            )
        };
        cfg.search.tg = tg;
        println!(
            "[dram] RD run {n} before: MRs believed written (MR18, MR10, MR46, MR20, MR33, MR34) [ch A, ch B][rank 0, rank 1] {mrs:x?}"
        );
        {
            let mut snap = [0u16; cubie_a7a_dram::phy::csrlist::TRM_CSRS.len()];
            rd::snapshot(&mut Bt0Hw, &mut snap);
            let at = |c: u32| {
                cubie_a7a_dram::phy::csrlist::TRM_CSRS
                    .iter()
                    .position(|&x| x == c)
                    .map(|k| snap[k])
                    .unwrap_or(0xffff)
            };
            println!(
                "[dram] RD run {n} DLL: AcPhase1UI {:x} DxPhase1UI {:x} AcDllLockParam {:x} DxDllLockParam {:x}",
                at(0x300be),
                at(0x100df),
                at(0x2000a),
                at(0x20017)
            );
            for (k, chunk) in snap.chunks(16).enumerate() {
                crate::recovery::kick();
                println!(
                    "[dram] RD SNAP r{n} {:05x}: {:04x?}",
                    cubie_a7a_dram::phy::csrlist::TRM_CSRS[16 * k],
                    chunk
                );
            }
        }
        println!("[dram] RD ch {c} rank {r} run {n} ({tg:?}): config {cfg:x?}");
        rd::measure_block(&mut Bt0Hw, b, prev, start, &cfg);
        prev = Some(rank);
        // Every block ends with the exit mode launch and the all-zero entry
        // on its own channel and rank.
        mrs[(ch == Channel::B) as usize][(rank == Rank::R1) as usize] = Some([
            cfg.exit.mr18,
            cfg.exit.mr10,
            cfg.exit.mr46,
            rd::ALL_ZERO.mr20,
            rd::ALL_ZERO.mr33,
            rd::ALL_ZERO.mr34,
        ]);
        for (k, d) in b.dll.iter().enumerate() {
            println!(
                "[dram] RD run {n} DLL after {}: AcPhase1UI {:x} DxPhase1UI {:x} PState {:x}",
                rd::DLL_STEPS[k],
                d[0],
                d[1],
                d[2]
            );
        }
        println!(
            "[dram] RD ch {c} rank {r} run {n}: start {} us, end {} us",
            b.start_us,
            b.start_us + b.elapsed_us as u64
        );
        let u = &b.u;
        println!(
            "[dram] RD ch {c} rank {r}: {} launches, {} writes, max {} us, {} us, failure {:?}; RxEnDly saved {:04x?} final {:04x?}; TrainingParam saved {:x?} final {:x?}; data controls saved {:x?} restored {:x?}",
            b.launches, b.writes, b.max_launch_us, b.elapsed_us, b.failure, b.saved_rx_en, b.final_rx_en, b.saved_param, b.final_param, b.data_saved, b.data_restored
        );
        println!("[dram] RD ch {c} rank {r}: read path {:x?}", b.read_path);
        for i in 0..2 {
            println!(
                "[dram] RD ch {c} rank {r} byte {i}: RxClk T0/T1/C0/C1 found {:x?} restored {}",
                b.rx_clk_before[i],
                b.rx_clk_after[i] == b.rx_clk_before[i]
            );
            println!(
                "[dram] RD ch {c} rank {r} byte {i}: VrefDAC0/1 found {:x?} used {:x?} restored {}",
                b.vref_before[i],
                b.vref_used[i],
                b.vref_after[i] == b.vref_before[i]
            );
        }
        for (j, rb) in b.rb.iter().enumerate() {
            println!(
                "[dram] RD {c}{r} R {} master {:04x?} dbyte {:04x?}",
                ["start", "start PState^1"][j],
                rb.master,
                rb.dbyte
            );
        }
        println!(
            "[dram] RD {c}{r} DbyteRxEnTrain before/after the exit {:x}/{:x}; live lane 0 (RxClkT, RxClkC, RxEn) after the exit {:x?}; clocks {:04x?}",
            b.rxen_train[0], b.rxen_train[1], u.live_exit, u.clocks
        );
        for (mi, &mrl) in cfg.search.mrl.iter().take(rd::MRL_MAX).enumerate() {
            for i in 0..2 {
                println!(
                    "[dram] RD {c}{r} U m{mrl} EYE b{i}: {} points with all lanes; per lane (points, RxEnDly min, max, RxClk min, max) {:x?}",
                    u.full[mi][i], u.lanes[mi][i]
                );
            }
        }
        println!(
            "[dram] RD {c}{r} U live readback rows not as written: {}",
            u.live_bad
        );
        println!(
            "[dram] RD {c}{r} QC MRL found {:x?} restored {:x?}; selects found {:x?} restored {:x?}; visibility found/restored {:x?}",
            b.qc.mrl_before, b.qc.mrl_after, b.qc.sel_before, b.qc.sel_after, b.qc.vis
        );
        println!(
            "[dram] RD {c}{r} QV PUB CSRs at the first data launch {:04x?}; {} writes read back, mismatches:",
            b.qc.pub_state, b.qc.n_raw
        );
        for &(csr, want, got) in b.qc.raw_mismatches() {
            println!("[dram] RD {c}{r} QV RAW {csr:05x} want {want:04x} got {got:04x}");
        }
        if b.failure.is_some() {
            break;
        }
    }
    println!("[dram] RD: done");
}

#[cfg(all(not(feature = "wck-alias-probe"), not(feature = "rd-measure")))]
/// `A7A_RD_VREF` (FEL lab): the read Vref sweep inside the read
/// training (crate module `rdvref`): `measure` or `apply`, optionally
/// `,<start>,<step>,<n>[,<dacs>]` in hex. Unset: no sweep (the default).
#[cfg(all(not(feature = "wck-alias-probe"), not(feature = "rd-measure")))]
const RD_VREF: Option<cubie_a7a_dram::rdvref::VrefPlan> =
    match cubie_a7a_dram::rdvref::VrefPlan::parse(option_env!("A7A_RD_VREF")) {
        Ok(p) => p,
        Err(()) => panic!("A7A_RD_VREF: measure|apply[,start,step,n[,dacs]] (hex)"),
    };

#[cfg(all(not(feature = "wck-alias-probe"), not(feature = "rd-measure")))]
static mut RD_VREF_RESULT: cubie_a7a_dram::rdvref::VrefResult =
    cubie_a7a_dram::rdvref::VrefResult::EMPTY;

/// The read Vref sweep's lines for one block.
#[cfg(all(not(feature = "wck-alias-probe"), not(feature = "rd-measure")))]
#[inline(never)]
fn report_vref(nm: &str, v: &cubie_a7a_dram::rdvref::VrefResult) {
    let p = v.plan;
    println!(
        "[dram] RDV {nm}: ran {} plan start {:02x} step {} n {} dacs {} apply {}; common choice per byte {:02x?}; applied {}",
        v.ran, p.start, p.step, p.n, p.dacs, p.apply, v.common, v.applied
    );
    for i in 0..2 {
        for lane in 0..cubie_a7a_dram::wl::LANES {
            crate::recovery::kick();
            let mut buf = [b'.'; 2 * cubie_a7a_dram::rdvref::POINTS_MAX];
            for (k, w) in v.profile[i][lane].iter().take(p.n as usize).enumerate() {
                let x = w.unwrap_or(0xff);
                buf[2 * k] = b"0123456789abcdef"[(x >> 4) as usize];
                buf[2 * k + 1] = b"0123456789abcdef"[(x & 0xf) as usize];
            }
            let txt = core::str::from_utf8(&buf[..2 * p.n as usize]).unwrap_or("?");
            println!(
                "[dram] RDV {nm} b{i} l{lane}: found {:02x}/{:02x} chosen {:02x?} recentred {:?} widths {txt}",
                v.found[i][0][lane], v.found[i][1][lane], v.chosen[i][lane], v.recentred[i][lane]
            );
        }
    }
}

#[cfg(all(not(feature = "wck-alias-probe"), not(feature = "rd-measure")))]
static mut RD_TRAIN: cubie_a7a_dram::rdtrain::RdTrain = cubie_a7a_dram::rdtrain::RdTrain::empty(
    cubie_a7a_dram::phy::wcklevel::Channel::A,
    cubie_a7a_dram::phy::wcklevel::Rank::R0,
);

/// The read training (crate module `rdtrain`) on every channel and rank,
/// **applied**: the trained RxEnDly, receive clock delays and the read
/// latency stay (the latency from the formula `rdtrain::derive_mrl`, or `RD_MRL`). Per block the compact
/// result table (`RDT ... TABLE` lines: per byte RxEnDly and window, per
/// lane RxClkT/C with eye width in taps and ps, the MRL and its inputs, the
/// data checks, the MRR read-out). Feature `rd-train-only` runs it as the
/// measurement image (the vendor's trained values as references, the
/// initial state repeated, everything put back); feature `rd-measure` runs
/// the read path searches instead.
#[cfg(all(not(feature = "wck-alias-probe"), not(feature = "rd-measure")))]
#[inline(never)]
fn read_training(bases: [Option<cubie_a7a_dram::phy::wcklevel::Mr18Base>; 4]) -> bool {
    use cubie_a7a_dram::phy::rxen::ModeMrs;
    use cubie_a7a_dram::phy::wcklevel::{Channel, Rank};
    use cubie_a7a_dram::rd;
    use cubie_a7a_dram::rdtrain::{self, clk_code, clk_eff, TrainConfig, CHECK_PATTERNS, MRR_MAS};
    use cubie_a7a_dram::wl;
    const TRAIN_ONLY: bool = cfg!(feature = "rd-train-only");
    const ALL: [(Channel, Rank); 5] = [
        (Channel::A, Rank::R0),
        (Channel::B, Rank::R0),
        (Channel::A, Rank::R1),
        (Channel::B, Rank::R1),
        (Channel::A, Rank::R0),
    ];
    // Applied: the four blocks once; the measurement image repeats A0.
    let runs: &[(Channel, Rank)] = if TRAIN_ONLY { &ALL } else { &ALL[..4] };
    let ps = stage_pstate();
    let base_cfg = TrainConfig {
        apply: !TRAIN_ONLY,
        mrl_override: cubie_a7a_dram::knobs::get().rd_mrl,
        exit: cubie_a7a_dram::rdtrain::EXIT_MRS(),
        ..match TrainConfig::for_pstate(ps) {
            Some(c) => c,
            None => TrainConfig::PSTATE1,
        }
    };
    println!(
        "[dram] RDT: read training ({}, pstate {}): order {:?}; config {:x?}; latch events (CSR, read at, bits) {:05x?} held {} us; receive clock effective delay 64 x coarse + fine (fine < 0x40 written); VrefDAC {}; check patterns (MR33, MR34) {:x?} then the first inverted (negative control); MRR {:?} with RxFifoVisibility {:x?}; no PState flip, no refresh; ps = taps x UI / 64; MRL = ceil((RxEnDly/64 UI + {} UI + {} ps) / {} UI) + {}, cross-checked at the vendor's RxEnDly {:x} (must give {}), fallback {}; vendor (av3c) RxEnDly {:x}",
        if TRAIN_ONLY { "measurement image" } else { "applied" },
        ps,
        runs,
        base_cfg,
        rd::EVENTS,
        rd::HOLD_US,
        if TRAIN_ONLY { "from the vendor (av3c)" } else { "as initialised (ours)" },
        CHECK_PATTERNS,
        MRR_MAS,
        rdtrain::VIS_MRR,
        base_cfg.fifo_ui,
        base_cfg.fifo_ps,
        base_cfg.dficlk_ui,
        base_cfg.mrl_margin,
        base_cfg.mrl_check.0,
        base_cfg.mrl_check.1,
        base_cfg.mrl_fallback,
        rd::VENDOR_RX_EN,
    );
    if rdtrain::PASS_LANES() != rdtrain::FULL {
        println!(
            "[dram] RDT: A7A_RD_LANES {:03x}: lanes outside the mask (DMI = lane 8) left out of the pass tests",
            rdtrain::PASS_LANES()
        );
    }
    // SAFETY: bt0 is single-threaded and nothing else refers to it.
    let t = unsafe { &mut *core::ptr::addr_of_mut!(RD_TRAIN) };
    let mut prev = None;
    let mut ok = 0;
    for (n, &(ch, rank)) in runs.iter().enumerate() {
        crate::recovery::kick();
        let k = wl::BLOCKS
            .iter()
            .position(|&x| x == (ch, rank))
            .unwrap_or(0);
        // pstate 1: the reference's mode MRs, and the vendor's trained
        // references only in the measurement image (never a default);
        // pstate 0: the same mode MRs (the reference's pstate-0 read enable
        // writes the same values), no vendor starts; another pstate: MR18
        // from the block's WL base, no vendor starts.
        let (cfg, start) = if ps == 1 {
            (base_cfg, if TRAIN_ONLY { Some(&AV3C_START) } else { None })
        } else if ps == 0 {
            (base_cfg, None)
        } else {
            let base = bases[k].map(|m| m.value()).unwrap_or(0x0c);
            (
                TrainConfig {
                    next: ModeMrs {
                        mr18: base | 0x10,
                        mr10: 0x59,
                        mr46: 0,
                    },
                    exit: ModeMrs {
                        mr18: base,
                        mr10: 0x54,
                        mr46: 0,
                    },
                    ..base_cfg
                },
                None,
            )
        };
        t.channel = ch;
        t.rank = rank;
        // SAFETY: bt0 is single-threaded and nothing else refers to it.
        let vr = unsafe { &mut *core::ptr::addr_of_mut!(RD_VREF_RESULT) };
        rdtrain::train_block_with(
            &mut Bt0Hw,
            t,
            prev,
            start,
            &cfg,
            RD_VREF.as_ref().map(|p| (p, &mut *vr)),
        );
        prev = Some(rank);
        ok += t.ok() as u32;
        let nm = rdtrain::block_name(ch, rank);
        let r = (rank == Rank::R1) as usize;
        let dbs: [usize; 2] = if ch == Channel::A { [0, 1] } else { [2, 3] };
        let ps = |taps: u16| taps as u32 * cfg.ui_ps as u32 / 64;
        let span = |w: [u16; 2], step: u16| {
            if w[0] == rdtrain::NONE {
                0
            } else {
                w[1] - w[0] + step
            }
        };
        println!(
            "[dram] RDT {nm} run {n} TABLE block: ok {} applied {} stop {:?} failure {:?} restored {} live {:?}; MRL applied {} ({:?}; formula {} from RxEnDly max {:03x}, vendor check {} want {}; sweep pass set {:#x}); {} launches, {} writes, max {} us, {} us",
            t.ok(), t.applied, t.stop, t.failure, t.restored, t.live_ok, t.mrl, t.mrl_source, t.mrl_formula, t.bytes[0].rx_en.max(t.bytes[1].rx_en), t.mrl_check, cfg.mrl_check.1, t.mrl_pass, t.launches, t.writes, t.max_launch_us, t.elapsed_us
        );
        for (i, b) in t.bytes.iter().enumerate() {
            let w = span(b.en_win, cfg.en_step);
            println!(
                "[dram] RDT {nm} run {n} TABLE b{i} (DBYTE {}): RxEnDly {:03x} window {:03x}-{:03x} {} = {} ps; FIFO boundary {:03x}; flat rows {:03x}-{:03x} at RxClk {}",
                dbs[i], b.rx_en, b.en_win[0], b.en_win[1], w, ps(w), b.boundary, b.flat[0], b.flat[1], b.clk0
            );
            for (lane, l) in b.lanes.iter().enumerate() {
                let (tw, cw) = (span(l.t, cfg.lane_step), span(l.c, cfg.lane_step));
                let vt = clk_eff(AV3C_START.rx_clk[dbs[i]][r][lane]);
                let vc = clk_eff(AV3C_START.rx_clk[dbs[i]][2 + r][lane]);
                println!(
                    "[dram] RDT {nm} run {n} TABLE b{i} l{lane}: T {:03x} {} eye {}-{} {} taps {} ps | C {:03x} {} eye {}-{} {} taps {} ps | both {}-{} | vendor T {} C {}",
                    clk_code(l.t_eff), l.t_eff, l.t[0], l.t[1], tw, ps(tw),
                    clk_code(l.c_eff), l.c_eff, l.c[0], l.c[1], cw, ps(cw),
                    l.both[0], l.both[1], vt, vc
                );
            }
        }
        for (ci, chk) in t.check.iter().enumerate() {
            let at = if ci == 0 { "search MRL" } else { "applied MRL" };
            for (p, (c, e)) in chk.compares.iter().zip(chk.errors.iter()).enumerate() {
                let (pat, what) = if p < CHECK_PATTERNS.len() {
                    (CHECK_PATTERNS[p], "pattern")
                } else {
                    (CHECK_PATTERNS[0], "negative")
                };
                println!(
                    "[dram] RDT {nm} run {n} TABLE check {ci} ({at}) {what} {:02x}/{:02x}: errors/compares b0 {}/{} b1 {}/{}; clean lanes {:?}",
                    pat.0, pat.1, e[0], c[0], e[1], c[1], chk.clean_lanes[p]
                );
            }
            println!(
                "[dram] RDT {nm} run {n} TABLE check {ci} ({at}) passed {}",
                chk.passed()
            );
        }
        for (m, o) in t.mrl_obs.iter().enumerate().filter(|(_, o)| o.tried) {
            println!(
                "[dram] RDT {nm} run {n} TABLE MRL {m}: compares {} errors {} all lanes {}; probe at the search MRL after it {}",
                o.compares, o.errors, o.pass, o.back
            );
        }
        if RD_VREF.is_some() {
            report_vref(nm, vr);
        }
        for (k, m) in t.mrr.iter().enumerate() {
            println!(
                "[dram] RDT {nm} run {n} TABLE MRR MR{}: b0 {:x?} raw {:02x?} b1 {:x?} raw {:02x?}",
                MRR_MAS[k], m.value[0], m.raw[0], m.value[1], m.raw[1]
            );
        }
        if t.failure.is_some() {
            break;
        }
    }
    println!(
        "[dram] RDT: done, {ok}/{} blocks trained and checked{}",
        runs.len(),
        if TRAIN_ONLY {
            ""
        } else {
            "; trained delays and read latency applied"
        }
    );
    !TRAIN_ONLY && ok as usize == runs.len()
}

/// Feature `handover`: the hand-over to the controller without a
/// PIE (crate module `handover`), after the read training is applied.
#[cfg(all(
    not(feature = "wck-alias-probe"),
    not(feature = "rd-measure"),
    feature = "handover"
))]
#[inline(never)]
fn handover_stage(p: &cubie_a7a_dram::Para) {
    use cubie_a7a_dram::handover::{
        self, Handover, Probe, CTRL_REGS, DBYTE_CANDIDATES, MATRIX_MRL, MATRIX_RDEN, MISSION, MRS,
        MRS_KNOWN, PHY_CSRS, PIE_VECTORS, SRE,
    };
    use cubie_a7a_dram::rd::CLOCK_CSRS;
    static mut PROBE: Probe = Probe::EMPTY;
    static mut HO: Handover = Handover::EMPTY;
    // SAFETY: bt0 is single-threaded and nothing else refers to these.
    let (pr, ho) = unsafe {
        (
            &mut *core::ptr::addr_of_mut!(PROBE),
            &mut *core::ptr::addr_of_mut!(HO),
        )
    };
    let channels = p.channels();
    println!(
        "[dram] HO: hand-over probe (pstate {PROBE_PSTATE}): stage 0 reads clocks {CLOCK_CSRS:05x?}, PHY {PHY_CSRS:05x?}, PIE vectors {PIE_VECTORS:05x?}, controller [DFIMISC, DFISTAT, STAT, PWRCTL, SWSTAT, DFIUPD0, DFIPHYMSTR, set1 DFITMG0, set1 DRAMTMG2] x {channels}; stage 1 writes 0x200f9=1, 0xd00fa=1, PUBMODE=0 each put back; stage 2 only if DFISTAT[0] followed: SRE {:04x?}, mission CSRs {MISSION:x?}, rddata_en formula, DFI init with our answer, ports, MRR {MRS:?}, quick memtest 0x40000000 {} MiB x {} words",
        SRE.map(|r| r.0),
        handover::QUICK_MIB,
        handover::QUICK_WORDS
    );
    handover::probe(&mut Bt0Hw, channels, pr);
    println!("[dram] HO probe clocks {:04x?}", pr.clocks);
    println!("[dram] HO probe PHY {:04x?}", pr.phy);
    println!("[dram] HO probe PIE vectors {:04x?}", pr.pie);
    for ch in 0..channels.min(2) {
        println!("[dram] HO probe controller ch {ch} {:08x?}", pr.ctrl[ch]);
    }
    println!(
        "[dram] HO probe PState SRAM records with the enable bit: {}",
        pr.sram_nonzero
    );
    for st in pr.steps.iter() {
        println!(
            "[dram] HO probe write {:05x}={:x}: 0x200f9/0xd00fa/Dfi0Status/Dfi1Status {:04x?}, DFISTAT {:08x?}, restored {:x}",
            st.csr, st.value, st.phy, st.dfistat, st.restored
        );
    }
    println!("[dram] HO probe: cpu_drives {:?}", pr.cpu_drives);
    let _ = CTRL_REGS;
    let Some(answer) = pr.cpu_drives else {
        println!("[dram] HO: the probe is negative; no hand-over attempted");
        return;
    };
    // No SRE row (its ACSM launch never completed, Timeout(1));
    // the vendor firmware also leaves the DRAM out of self-refresh (in
    // power-down) when the controller leaves selfref_sw.
    const SRE_FIRST: bool = false;
    static mut SNAP: [u16; cubie_a7a_dram::phy::csrlist::TRM_CSRS.len()] =
        [0; cubie_a7a_dram::phy::csrlist::TRM_CSRS.len()];
    // SAFETY: bt0 is single-threaded and nothing else refers to it.
    let snap = unsafe { &mut *core::ptr::addr_of_mut!(SNAP) };
    handover::handover(&mut Bt0Hw, p, answer, SRE_FIRST, PROBE_PSTATE, ho, snap);
    println!(
        "[dram] HO TABLE: stop {:?}; SRE {}; mission CSRs read back {}/{}; dfi_twck_delay (+0x588 [21:16], sets 0 and 1) {} from PclkPtrInitVal/TxWckDly {:x?}; MSTR2 before/after {:08x?} (active set [4:0]); +0xb84/+0xb88 {:08x?}; active set +0x580 found {:08x} (dfi_t_rddata_en [22:16] = {:#x}); controller after [DFIMISC, DFISTAT, STAT, PWRCTL] {:08x?}; PHY after [DfiInitComplete, PMIEnable, MemResetL, Dfi0Status, Dfi1Status, 0x200bb] {:04x?}; {} us",
        ho.stop, ho.sre_done, ho.mission_ok, MISSION.len(), ho.rden, ho.rden_in, ho.mstr2, ho.opctrl, ho.dfitmg0_found, (ho.dfitmg0_found >> 16) & 0x7f, ho.ctrl, ho.phy_after, ho.elapsed_us
    );
    println!(
        "[dram] HO TABLE mission CSR not read back: {:05x} want {:04x} got {:04x}; DBYTE candidates {:03x?} per DBYTE after the mission writes {:04x?} after the DFI init {:04x?}",
        ho.mission_bad.0, ho.mission_bad.1, ho.mission_bad.2, DBYTE_CANDIDATES, ho.dbyte_before, ho.dbyte_after
    );
    for (i, &v) in handover::MATRIX_WCK.iter().enumerate() {
        println!(
            "[dram] HO TABLE twck +0x588 [21:16] = {v} (active set, HwtMRL 4, rddata_en as found): [MR16, MR2] {:x?}; words {:x?} -> {:08x?}",
            ho.matrix_wck[i],
            handover::WCK_WORDS,
            ho.matrix_wck_mem[i]
        );
    }
    println!(
        "[dram] HO TABLE twck put back: active set +0x588 {:08x} (TWCK_LIVE {}, formula {})",
        ho.twck_after,
        handover::TWCK_LIVE,
        ho.rden
    );
    println!(
        "[dram] HO TABLE fold (the vendor's CDD deltas (+offset, shift, add) {:x?} on channel 0's active set; vendor live set 0 {:x?}); before/after per register {:08x?}",
        handover::FOLD_DELTA,
        handover::FOLD_LIVE,
        ho.fold
    );
    for ch in 0..channels.min(2) {
        for rank in 0..2 {
            println!(
                "[dram] HO TABLE MRR ch {ch} rank {rank} MR {MRS:?}: {:x?}; known {:x?}",
                ho.mrr[ch][rank], MRS_KNOWN
            );
        }
    }
    for (i, &mrl) in MATRIX_MRL.iter().enumerate() {
        println!(
            "[dram] HO TABLE matrix HwtMRL=DFIMRL {mrl}: dfi_t_rddata_en (+0x580 [22:16], active set) {MATRIX_RDEN:x?} -> [MR16, MR2] {:x?}; ch 0 [DFISTAT, STAT, +0x308, MRSTAT] after {:x?}",
            ho.matrix[i], ho.matrix_status[i]
        );
    }
    println!(
        "[dram] HO TABLE pair put back: HwtMRL {:x}, active set +0x580 {:08x}",
        ho.restored_pair.0, ho.restored_pair.1
    );
    println!(
        "[dram] HO TABLE DFI read-data configuration ch 0 active set [DFIMISC, +0x580 DFITMG0, +0x584 DFITMG1, +0x588 (dfi_twck_delay), DBICTL, MSTR0] {:08x?} (DFIMISC: [0] init_complete_en, [2] data_cs_polarity, [4] phy_dbi_mode, [5] init_start, [12:8] dfi_frequency; +0x580: [22:16] dfi_t_rddata_en); PHY [DfiFreqRatio, PclkPtrInitVal, DMIPinPresent, DfiMode, DfiCfgRdDataValidTicks, DfiInitComplete] {:04x?}",
        ho.dfi_cfg, ho.dfi_phy
    );
    println!(
        "[dram] HO TABLE quick memtest ran {}: {:x?}; liveness compare A right after the DFI init {:x?}, B after the memtest {:x?} (PUBMODE 1, ACSM, ch A rank 0: passing lanes per byte, compares, errors); controller +0xd00..0xd14 before {:08x?} after {:08x?}; +0xd00 [31:30] {} (already set: {}); ACSM MRR [MR3, MR20, MR22, MR10] byte 0 lanes 0-7 at A {:02x?} at B {:02x?}",
        ho.quick_ran, ho.quick, ho.live_compare_a, ho.live_compare, ho.init_before, ho.init_after, ho.init_before[0] >> 30, ho.init_skip_set, ho.live_mrr_a, ho.live_mrr_b
    );
    // The full TRM snapshot right after the hand-over (host-side comparison
    // with the vendor's mission state, the av3-dump SNAP lines).
    for (k, chunk) in snap.chunks(16).enumerate() {
        crate::recovery::kick();
        println!(
            "[dram] HO SNAP {:05x}: {:04x?}",
            cubie_a7a_dram::phy::csrlist::TRM_CSRS[16 * k],
            chunk
        );
    }
}

/// The end-of-initialization stage, printed (kept out of `init_one`'s
/// frame).
#[cfg(not(feature = "wck-alias-probe"))]
#[inline(never)]
fn end_of_init(p: &mut Para) -> blobless::EndInitReport {
    let e = blobless::end_of_init(&mut Bt0Hw, p, stage_pstate());
    println!(
        "[dram] end of init: {} launches, {} writes, max {} us, {} us, failure {:?}",
        e.launches, e.writes, e.max_launch_us, e.elapsed_us, e.failure
    );
    let v = e.variant;
    if !v.is_reference() {
        println!(
            "[dram] end of init BISECT: skip {:04x}, no MRW to MAs {:x} (bit n = MA n), MR16 {:02x}/{:02x}",
            v.skip, v.no_mrw, v.mr16_write, v.mr16_switch
        );
    }
    for (k, m) in e.mrs.iter().enumerate() {
        println!(
            "[dram] end of init block {k}: MR16 {:02x}, {:02x?}, then MR16 {:02x}",
            v.mr16_write, m.mr, v.mr16_switch
        );
    }
    e
}

/// FEL lab: print the hand-over knobs (`blobless::HO_RDDATA_EN()`,
/// `blobless::HO_MR2()`) when set.
fn print_ho_knobs() {
    if blobless::HO_RDDATA_EN().is_some()
        || blobless::HO_MR2().is_some()
        || blobless::HO_TXWCK_ADD() != 0
        || blobless::HO_RXEN_ADD() != 0
    {
        println!(
            "[dram] HO knobs: dfi_t_rddata_en {:x?}, MR2 after the hand-over {:x?}, TxWckDly +{:x}, RxEnDly +{:x}",
            blobless::HO_RDDATA_EN(),
            blobless::HO_MR2(),
            blobless::HO_TXWCK_ADD(),
            blobless::HO_RXEN_ADD()
        );
    }
    if cubie_a7a_dram::obs::OBS_ARM() != 0 || cubie_a7a_dram::obs::HO_RDCSLAT().is_some() {
        println!(
            "[dram] OBS knobs: A7A_OBS_ARM {:x} (1 RX FIFO pointer check armed before the DFI init, 2 snapshots), dfi_tphy_rdcslat {:x?}",
            cubie_a7a_dram::obs::OBS_ARM(),
            cubie_a7a_dram::obs::HO_RDCSLAT()
        );
    }
    if let Some(s) = cubie_a7a_dram::knobs::env("A7A_HO_CSRS", option_env!("A7A_HO_CSRS")) {
        println!(
            "[dram] HO knobs: {} PHY CSR writes before the hand-over",
            s.split(',').count()
        );
    }
    if let Some(s) = cubie_a7a_dram::knobs::env("A7A_HO_MRS", option_env!("A7A_HO_MRS")) {
        println!("[dram] HO knobs: operating-set MRWs after the hand-over {s}");
    }
    if blobless::HO_EPILOGUE().is_some() || blobless::HO_FIFOINIT() || blobless::HO_PULSE_RESTORE()
    {
        println!(
            "[dram] HO knobs: epilogue MR16 {:x?} MRs {:x?} + ZQ latch + SR cycle, RX FIFO init {}, ACSM read pulses -{} {} {:05x?}",
            blobless::HO_EPILOGUE(),
            blobless::EPILOGUE_MRS,
            blobless::HO_FIFOINIT(),
            blobless::ACSM_PULSE_ADJUST,
            blobless::HO_PULSE_RESTORE(),
            blobless::ACSM_READ_PULSES
        );
    }
    if blobless::HO_WCKON().is_some() || blobless::HO_DBICTL().is_some() || blobless::HO_SRCYCLE() {
        println!(
            "[dram] HO knobs: MSTR4.wck_on {:x?}, DBICTL {:x?} before the DFI init; self-refresh cycle at the end {}",
            blobless::HO_WCKON(),
            blobless::HO_DBICTL(),
            blobless::HO_SRCYCLE()
        );
    }
}

/// Feature `bl-mission`, FEL lab knob `A7A_BL_BOOT`: when the lean
/// hand-over succeeded and the write stage (`A7A_WR_STAGE`) left the probe
/// blocks writing cleanly, [`init_one`] returns the size instead of
/// halting, and bt0 goes on (its DRAM tests, then main): the blob-free
/// boot attempt. Unset: the halt, as before.
#[cfg(feature = "bl-mission")]
#[allow(non_snake_case)]
fn BL_BOOT() -> bool {
    cubie_a7a_dram::knobs::env("A7A_BL_BOOT", option_env!("A7A_BL_BOOT")).is_some()
}
#[cfg(feature = "bl-mission")]
static mut MISSION_OK: bool = false;

/// Probe the PHY with `para`, print the result and stop (or, with
/// `A7A_BL_BOOT`, return the size after a clean blobless bring-up).
pub fn init_one(para: &[u32; 32]) -> Option<u32> {
    print_ho_knobs();
    let mut p = Para::from_sysconfig(para);
    if !clock_matches(&p) {
        crate::halt()
    }
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
            if ACSM_PROBE {
                acsm_reset_probe();
            }
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
                    "[dram] blobless: CA ch {}: lane map agrees {}, delays {:?} (ACTxDly applied {:02x?}), windows {}..+{} {}..+{} {}..+{} {}..+{} {}..+{} {}..+{} {}..+{}",
                    ["A", "B"][ch],
                    c.lane_map_agrees,
                    c.delay,
                    blobless::ca_applied(&c.delay),
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
                    // The reference's end of initialization (target
                    // FSP mode registers, FSP-OP switch) before WL.
                    if let Some(spec) =
                        cubie_a7a_dram::knobs::env("A7A_FSP0_MRS", option_env!("A7A_FSP0_MRS"))
                    {
                        let f = blobless::fsp0_mrs(&mut Bt0Hw, &mut p, PROBE_PSTATE);
                        println!("[dram] FSP0 MRs {spec} written (MR16 40 .. 00): failure {f:?}");
                    }
                    let e = end_of_init(&mut p);
                    if e.failure.is_none() {
                        wck_level(&mut p);
                    } else {
                        println!("[dram] WL: skipped, end of init failed");
                    }
                } else {
                    println!("[dram] WL: skipped, CA training incomplete");
                }
            }
        }
        Err(e) => {
            println!("[dram] blobless: pre-training failed after {ms} ms: {e:?}");
        }
    }
    #[cfg(feature = "bl-mission")]
    // SAFETY: bt0 is single-threaded.
    if BL_BOOT() && unsafe { MISSION_OK } {
        match cubie_a7a_dram::size::boot_size_mib(&p) {
            Ok(mib) => {
                println!("[dram] BLOBLESS BOOT: hand-over and write stage clean; {mib} MiB, continuing the boot");
                return Some(mib);
            }
            Err(e) => {
                println!("[dram] BLOBLESS BOOT: no size: {e:?}");
            }
        }
    }
    #[cfg(not(any(feature = "wck-alias-probe", feature = "rd-measure")))]
    {
        // The stage after the read training: the live read-path delays of
        // lane 0 of every DBYTE (Tg0 readback), as the next stage finds them.
        let live = cubie_a7a_dram::rdtrain::live_summary(&mut Bt0Hw, 0);
        println!(
            "[dram] after RD: live lane 0 per DBYTE (RxClkT fine, RxClkC fine, RxEnDly) {live:03x?}"
        );
    }
    println!("[dram] blobless: read training is the last stage so far; stopping");
    crate::halt()
}

/// `A7A_ACSM_PROBE=reset` (FEL lab, phase A): right after pre-training,
/// with the DRAM
/// still held in reset (MemResetL 2), the ACSM row-field cases of crate
/// module `acsm_probe` (`ACSMP` lines), then the halt. Nothing reaches the
/// DRAM. Unset: the normal blobless flow.
#[cfg(not(feature = "wck-alias-probe"))]
const ACSM_PROBE: bool = match option_env!("A7A_ACSM_PROBE") {
    None => false,
    Some(s) if s.len() == 5 && s.as_bytes()[0] == b'r' => true,
    Some(_) => panic!("A7A_ACSM_PROBE must be reset"),
};

/// The phase-A cases, printed, then the halt.
#[cfg(not(feature = "wck-alias-probe"))]
#[inline(never)]
fn acsm_reset_probe() -> ! {
    use cubie_a7a_dram::acsm_probe;
    println!(
        "[dram] ACSMP: pstate {PROBE_PSTATE}, DRAM in reset, {} cases, outer 255 x {} launches each (T)",
        acsm_probe::CASES.len(),
        acsm_probe::REPS
    );
    acsm_probe::reset_probe(&mut Bt0Hw, 2, &mut |c, s| {
        crate::recovery::kick();
        println!(
            "[dram] ACSMP {} rows {:04x?} outer {} pre {:05x?}: us {:?} polls {} obs {:04x?} -> {:04x?} failure {:?}",
            c.id,
            &c.rows[..c.len as usize],
            c.outer,
            &c.pre[..c.npre as usize],
            s.us,
            s.polls,
            s.before,
            s.after,
            s.failure
        );
    });
    println!("[dram] ACSMP: done; stopping");
    crate::halt()
}

/// `A7A_DCA` (FEL lab; feature `hybrid`): the WCK duty-cycle
/// stage (crate module `dca`) right after the PMU's training, before
/// anything else of ours: `mrr`, `dcm`, `measure` or `apply`. Unset: no
/// stage (the default). Companions: `A7A_DCA_MR16` (hex, MR16 as the DRAM
/// has it; default 04, the PMU's end state, or the end of
/// initialization's switch value when `A7A_HYBRID_ENDINIT` ran),
/// `A7A_DCA_VIS` (hex RxFifoVisibility for the MRR contents, default
/// fffc), `A7A_DCA_MRL=trace` (the reference's read latency during the stage),
/// `A7A_DCA_ITER` (sweeps, 1-9), `A7A_DCA_SINGLE_FLIP=1`.
#[cfg(feature = "hybrid")]
const DCA_MODE: Option<cubie_a7a_dram::dca::Mode> = match option_env!("A7A_DCA") {
    None => None,
    Some(s) => match cubie_a7a_dram::dca::Mode::parse(s) {
        Some(m) => Some(m),
        None => panic!("A7A_DCA must be mrr, dcm, measure or apply"),
    },
};

#[cfg(feature = "hybrid")]
const fn hex_knob(s: Option<&str>, default: u32, max: u32, what: &str) -> u32 {
    let Some(s) = s else {
        return default;
    };
    match u32::from_str_radix(s, 16) {
        Ok(v) if v <= max => v,
        _ => panic!("{}", what),
    }
}

#[cfg(feature = "hybrid")]
const DCA_CONFIG: cubie_a7a_dram::dca::Config = {
    use cubie_a7a_dram::dca::{Config, DcaPolicy, Mode, VIS_TRACE};
    let mr16_default = if option_env!("A7A_HYBRID_ENDINIT").is_some() {
        blobless::END_OF_INIT_VARIANT.mr16_switch as u32
    } else {
        0x04
    };
    Config {
        mode: match DCA_MODE {
            Some(m) => m,
            None => Mode::Mrr,
        },
        // The build's A7A_OPEN_OPTS (a const; the DCA stage is not a lab knob).
        pstate: match super::open::opts(cubie_a7a_dram::knobs::BUILD.open_opts).only_pstate {
            Some(ps) => ps,
            None => PROBE_PSTATE,
        },
        mr16_found: hex_knob(
            option_env!("A7A_DCA_MR16"),
            mr16_default,
            0xff,
            "A7A_DCA_MR16: hex byte",
        ) as u8,
        vis: hex_knob(
            option_env!("A7A_DCA_VIS"),
            VIS_TRACE as u32,
            0xffff,
            "A7A_DCA_VIS: hex halfword",
        ) as u16,
        trace_mrl: match option_env!("A7A_DCA_MRL") {
            None => false,
            Some(s) if s.len() == 5 && s.as_bytes()[0] == b't' => true,
            Some(_) => panic!("A7A_DCA_MRL must be trace"),
        },
        iterations: hex_knob(option_env!("A7A_DCA_ITER"), 1, 9, "A7A_DCA_ITER: 1-9") as u8,
        policy: DcaPolicy {
            single_flip: option_env!("A7A_DCA_SINGLE_FLIP").is_some(),
            ..DcaPolicy::DEFAULT
        },
        timed_waits: option_env!("A7A_DCA_TIMED").is_some(),
        ..Config::DEFAULT
    }
};

#[cfg(feature = "hybrid")]
static mut DCA_BLOCK: cubie_a7a_dram::dca::DcaBlock = cubie_a7a_dram::dca::DcaBlock::empty(
    cubie_a7a_dram::phy::wcklevel::Channel::A,
    cubie_a7a_dram::phy::wcklevel::Rank::R0,
);

/// The DCA stage on the four blocks, printed (`DCA ...` lines).
#[cfg(feature = "hybrid")]
#[inline(never)]
fn dca_stage() {
    use cubie_a7a_dram::dca::{self, MRR_FOUND, MRR_OPERATING};
    let cfg = DCA_CONFIG;
    println!("[dram] DCA: {cfg:x?}");
    // SAFETY: bt0 is single-threaded and nothing else refers to it.
    let b = unsafe { &mut *core::ptr::addr_of_mut!(DCA_BLOCK) };
    for &(ch, rank) in dca::BLOCKS.iter() {
        crate::recovery::kick();
        b.channel = ch;
        b.rank = rank;
        dca::run_block(&mut Bt0Hw, b, &cfg);
        let nm = cubie_a7a_dram::rdtrain::block_name(ch, rank);
        println!(
            "[dram] DCA {nm}: stop {:?} failure {:?} restored {} MRR DBYTE {:?}; MR30 found {:02x?} final {:02x?}; DCM once {:02x?}; {} launches, {} writes, {} us",
            b.stop, b.failure, b.restored, b.mrr_db, b.mr30_found, b.mr30_final, b.dcm_once, b.launches, b.writes, b.elapsed_us
        );
        for (what, list) in [
            ("FSP-WR found", &b.found[..]),
            ("FSP-WR operating", &b.operating[..]),
        ] {
            for m in list {
                println!(
                    "[dram] DCA {nm} MRR {what} MR{}: {:02x?} raw {:04x?} | {:04x?} info {:04x?} | {:04x?}",
                    m.ma, m.value, m.raw[0], m.raw[1], m.info[0], m.info[1]
                );
            }
        }
        let _ = (MRR_FOUND, MRR_OPERATING);
        if b.mr26.iter().any(|x| x.is_some()) {
            println!("[dram] DCA {nm} MR26 per step -7..+7: {:02x?}", b.mr26);
            for (k, byte) in ["upper", "lower"].iter().enumerate() {
                let s = &b.sweep[k];
                println!(
                    "[dram] DCA {nm} {byte}: flip0 ones {:?} flip1 ones {:?} reads {:?}; decision {:?}",
                    s.ones[0], s.ones[1], s.reads[0], b.decision[k]
                );
            }
        }
    }
}

/// `A7A_ACSM_FIELD=<variant>` (FEL lab, phase B; feature `hybrid`): right
/// after the PMU,
/// before anything else of ours, one MRR of channel A with one ACSM row
/// field changed (crate `acsm_probe::Variant`: ref, ma8, r1, r1-notg,
/// r0-tg, rx-a, rx-b, rx-none, pre-nolo, pre-notg, pre-noft, pre-none,
/// wait0, wait8), then the reference MRRs of MR5 and MR8. MRR only.
#[cfg(feature = "hybrid")]
const ACSM_FIELD: Option<cubie_a7a_dram::acsm_probe::Variant> = match option_env!("A7A_ACSM_FIELD")
{
    None => None,
    Some(s) => match cubie_a7a_dram::acsm_probe::Variant::parse(s) {
        Some(v) => Some(v),
        None => panic!("A7A_ACSM_FIELD: unknown variant"),
    },
};

#[cfg(feature = "hybrid")]
#[inline(never)]
fn acsm_field_probe(v: cubie_a7a_dram::acsm_probe::Variant) {
    use cubie_a7a_dram::acsm_probe;
    let r = acsm_probe::field_probe(
        &mut Bt0Hw,
        v,
        hybrid_pstate(),
        cubie_a7a_dram::dca::VIS_TRACE,
    );
    println!(
        "[dram] ACSMF {:?}: rows {:04x?}; failure {:?} restored {} launches {}",
        v,
        v.rows(),
        r.failure,
        r.restored,
        r.launches
    );
    for (what, m) in ["variant", "ref MR5", "ref MR8"].iter().zip(r.mrr.iter()) {
        println!(
            "[dram] ACSMF {:?} {what} MR{}: {:02x?} raw {:04x?} | {:04x?} info {:04x?} | {:04x?}",
            v, m.ma, m.value, m.raw[0], m.raw[1], m.info[0], m.info[1]
        );
    }
}

/// The pstate the hybrid trained and hands over (A7A_OPEN_OPTS bits 3-5);
/// our WL and read training run there too (they need `A7A_PSTATE` equal to
/// it: build-time check above).
#[cfg(feature = "hybrid")]
/// [`hybrid_pstate`] from the build's `A7A_OPEN_OPTS` (const contexts).
#[cfg(feature = "hybrid")]
const fn hybrid_pstate_build() -> u32 {
    match super::open::opts(cubie_a7a_dram::knobs::BUILD.open_opts).only_pstate {
        Some(ps) => ps,
        None => PROBE_PSTATE,
    }
}

#[cfg(feature = "hybrid")]
fn hybrid_pstate() -> u32 {
    match super::open::opts(cubie_a7a_dram::knobs::get().open_opts).only_pstate {
        Some(ps) => ps,
        None => PROBE_PSTATE,
    }
}

/// Feature `hybrid`: after the PHY training firmware
/// trained the pstate and our mission bring-up relocked the PHY (open
/// backend, variant bit 7), PUBMODE back to 1 and our own stages from
/// WCK2CK leveling on (env `A7A_HYBRID_SKIP_WL`: from the read training
/// on), then the lean hand-over (`bl-mission`), then the halt. Env
/// `A7A_HYBRID_ENDINIT` runs our end of initialization first (bisect knobs:
/// `blobless::END_OF_INIT_VARIANT`); `A7A_HYBRID_NO_WL` then goes straight
/// to the lean hand-over, with no leveling or read training (the firmware's
/// trained delays stay).
#[cfg(feature = "hybrid")]
pub fn hybrid_after_pmu(p: &mut Para) -> ! {
    use cubie_a7a_dram::phy::{csr, csr_access};
    use cubie_a7a_dram::RegIo;
    let mut hw = Bt0Hw;
    csr_access(&mut hw, true);
    hw.write16(csr(0x2006e), 1);
    csr_access(&mut hw, false);
    println!(
        "[dram] HYBRID: PMU trained pstate {}; PUBMODE 1; our stages follow",
        hybrid_pstate()
    );
    print_ho_knobs();
    if !clock_matches(p) {
        crate::halt()
    }
    if let Some(stage) =
        cubie_a7a_dram::knobs::env("A7A_HYBRID_COLD", option_env!("A7A_HYBRID_COLD"))
    {
        // FEL lab: our DevInit / cold init / CBT over the
        // PMU-trained state (blobless::hybrid_cold).
        let stage = (stage.as_bytes()[0] - b'0') as u32;
        let mut t = blobless::CaTrainReport::EMPTY;
        blobless::hybrid_cold(&mut Bt0Hw, p, PROBE_PSTATE, stage, &mut t);
        println!(
            "[dram] HYBRID: our cold stage {stage}: devinit failure {:?}, exec failure {:?}, CBT blocks done {}, CA ch A {:?} ch B {:?}",
            t.devinit.failure, t.exec.failure, t.done, t.channels[0].delay, t.channels[1].delay
        );
    }
    if let Some(spec) = cubie_a7a_dram::knobs::env("A7A_FSP0_MRS", option_env!("A7A_FSP0_MRS")) {
        let f = blobless::fsp0_mrs(&mut Bt0Hw, p, PROBE_PSTATE);
        println!("[dram] FSP0 MRs {spec} written (MR16 40 .. 00): failure {f:?}");
    }
    let k = cubie_a7a_dram::knobs::get();
    if k.hybrid_endinit {
        let e = end_of_init(p);
        println!(
            "[dram] HYBRID: our end of initialization: failure {:?}",
            e.failure
        );
    }
    if let Some(v) = ACSM_FIELD {
        acsm_field_probe(v);
    }
    if DCA_MODE.is_some() {
        dca_stage();
    }
    if k.hybrid_no_wl {
        let r = blobless::mission_handover(&mut Bt0Hw, p, hybrid_pstate());
        println!("[dram] MISSION (hybrid, no leveling, no read training): {r:?}");
        if r.is_ok() {
            let _ = super::wrstage::run();
        }
    } else if k.hybrid_skip_wl {
        let bases = blobless::wl_mr18_bases(p, stage_pstate());
        if let Some((before, _)) = blobless::RD_FSPWR() {
            let f = blobless::mr16_all(&mut Bt0Hw, before);
            println!(
                "[dram] RD: MR16 {before:02x} to all ranks before the read training: failure {f:?}"
            );
        }
        let applied = read_training(bases);
        if let Some((_, after)) = blobless::RD_FSPWR() {
            let f = blobless::mr16_all(&mut Bt0Hw, after);
            println!(
                "[dram] RD: MR16 {after:02x} to all ranks after the read training: failure {f:?}"
            );
        }
        if applied {
            let r = blobless::mission_handover(&mut Bt0Hw, p, stage_pstate());
            println!(
                "[dram] MISSION: lean hand-over at pstate {}: {r:?}",
                stage_pstate()
            );
        }
    } else {
        wck_level(p);
    }
    crate::halt()
}
