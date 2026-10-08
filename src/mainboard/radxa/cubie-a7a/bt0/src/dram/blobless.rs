//! Blobless DRAM backend (feature `blobless`): no PHY firmware is built in,
//! loaded or run. Development stage: it runs the firmware-free PHY probe
//! (crate `cubie-a7a-dram`, module `blobless`), prints what it found and
//! halts. DRAM is not brought up yet, so the boot watchdog resets the board
//! and the next boot takes the recovery path (README, Recovery).

use cubie_a7a_dram::blobless::{self, Readback};
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
            // Cold init to CBT on channel A rank 0, then our own command bus
            // training on the DRAM's CBT echo.
            let m1 = cubie_a7a_dram::phy::coldinit::Mrs::ORACLE_PSTATE1_CXMT_4G;
            let m2 = cubie_a7a_dram::phy::coldinit::Mrs2::ORACLE_PSTATE1_CXMT_4G;
            let t = blobless::cold_init_ca_train(&mut Bt0Hw, &m1, &m2);
            let e = &t.exec;
            println!(
                "[dram] blobless: CA train: {} launches ({} witnessed, max {} busy polls), {} us, failure {:?}",
                e.launches, e.witnessed, e.busy_max, e.elapsed_us, e.failure
            );
            let cs = blobless::CA_SWEEP_STEP as u32;
            for b in 0..16 {
                if cubie_a7a_dram::phy::coldinit::CBT_ECHO_BITS >> b & 1 != 0 {
                    let (s0, l0) = t.bit_runs[b];
                    println!(
                        "[dram] blobless: CA common sweep bit {b}: pass map {:032x}, run {}..+{}",
                        t.bit_maps[b],
                        s0 * cs,
                        l0 * cs
                    );
                }
            }
            let (s0, l0) = t.common_run;
            println!(
                "[dram] blobless: CA common window {}..+{}, center {}",
                s0 * cs,
                l0 * cs,
                t.common_center
            );
            let ls = blobless::LANE_SWEEP_STEP as u32;
            for lane in 0..7 {
                let (s1, l1) = t.lane_runs[lane];
                println!(
                    "[dram] blobless: CA lane {lane}: echo bit {}, window {}..+{}, delay {}",
                    t.lane_bit[lane],
                    s1 * ls,
                    l1 * ls,
                    t.lane_delay[lane]
                );
            }
            println!(
                "[dram] blobless: CA trained: pass {:04x} (want {:04x}), echo {:04x} {:04x} {:04x} {:04x}",
                t.final_pass,
                cubie_a7a_dram::phy::coldinit::CBT_ECHO_BITS,
                t.final_echo[0], t.final_echo[1], t.final_echo[2], t.final_echo[3]
            );
        }
        Err(e) => {
            println!("[dram] blobless: pre-training failed after {ms} ms: {e:?}");
        }
    }
    println!("[dram] blobless: no training backend yet; stopping");
    crate::halt()
}
