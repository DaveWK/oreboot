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
            // DRAM reset released here: cold init part 1 (MRW only, no read).
            let mrs = cubie_a7a_dram::phy::coldinit::Mrs::ORACLE_PSTATE1_CXMT_4G;
            let c = blobless::cold_init_part1(&mut Bt0Hw, &mrs);
            println!(
                "[dram] blobless: cold init part 1: {} launches ({} witnessed, max {} busy polls), {} CSR writes, {} us, failure {:?}",
                c.launches, c.witnessed, c.busy_max, c.writes, c.elapsed_us, c.failure
            );
        }
        Err(e) => {
            println!("[dram] blobless: pre-training failed after {ms} ms: {e:?}");
        }
    }
    println!("[dram] blobless: no training backend yet; stopping");
    crate::halt()
}
