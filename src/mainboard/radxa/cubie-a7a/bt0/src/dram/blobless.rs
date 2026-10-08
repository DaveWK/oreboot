//! Blobless DRAM backend (feature `blobless`): no PHY firmware is built in,
//! loaded or run. Development stage: it runs the firmware-free PHY probe
//! (crate `cubie-a7a-dram`, module `blobless`), prints what it found and
//! halts. DRAM is not brought up yet, so the boot watchdog resets the board
//! and the next boot takes the recovery path (README, Recovery).

use cubie_a7a_dram::blobless::{self, BlockTrain, Readback};
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
        }
        Err(e) => {
            println!("[dram] blobless: pre-training failed after {ms} ms: {e:?}");
        }
    }
    println!("[dram] blobless: no training backend yet; stopping");
    crate::halt()
}
