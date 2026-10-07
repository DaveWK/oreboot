//! Boot operating point for the CPU clusters and the DSU.
//!
//! Without cpufreq (mainline has none for the A733 yet) the OS keeps
//! whatever bt0 programs, with no thermal throttling, so this picks a point
//! well inside the vendor's characterised tables rather than the maximum.
//!
//! The vendor kernel keys its OPP voltages on the eFuse speed bin
//! (`dvfs_ori`, SID +0x24C bits [15:8], mapped by the DT's
//! `vf_mapping_table`). Voltages below are the vendor `opp-microvolt-26m-
//! vf0400` column (radxa 6.6.98 DT) plus 20 mV, because nothing here does
//! adaptive voltage scaling. vf0500/vf0401/vf0501 parts need the same or
//! less. Rails: CPU_L and DSU share DCDC5, CPU_B is on DCDC3.

use util::mmio::read32;

use crate::axp::Axp;
use crate::ccu;
use crate::soc::{udelay, SID_BASE};

struct OperatingPoint {
    cpu_l: u32,
    cpu_b: u32,
    dsu: u32,
    /// DCDC5 (CPU_L + DSU) and DCDC3 (CPU_B) in mV.
    vdd_little: u32,
    vdd_big: u32,
}

/// vf0400: CPU_L 1612 MHz @ 920 mV, DSU 1222 @ 920, CPU_B 1794 @ 920.
const PERFORMANCE: OperatingPoint = OperatingPoint {
    cpu_l: 1612,
    cpu_b: 1794,
    dsu: 1222,
    vdd_little: 940,
    vdd_big: 940,
};

/// Every 26 MHz column supports these at 800 mV; 850 mV covers any bin.
const SAFE: OperatingPoint = OperatingPoint {
    cpu_l: 1196,
    cpu_b: 1404,
    dsu: 910,
    vdd_little: 850,
    vdd_big: 850,
};

/// Speed bins whose tables are vf0400 or better (vf_mapping_table).
const KNOWN_BINS: [u32; 4] = [0x04, 0x05, 0x14, 0x15];

fn speed_bin() -> u32 {
    (read32(SID_BASE + 0x24c) >> 8) & 0xff
}

/// Raise the rails, then the clocks. On any PMIC error the clocks stay at
/// bt0's 0.8 V reset point (~1 GHz).
pub fn boost(pmic: &Axp) {
    let bin = speed_bin();
    let (op, which) = if KNOWN_BINS.contains(&bin) {
        (&PERFORMANCE, "performance")
    } else {
        (&SAFE, "safe (unknown bin)")
    };
    println!("[bt0] DVFS: speed bin {bin:#04x}, {which} operating point");

    for (rail, mv) in [("dcdc5", op.vdd_little), ("dcdc3", op.vdd_big)] {
        if let Err(e) = pmic.set_voltage(rail, mv, None) {
            println!("[bt0] DVFS: setting {rail} to {mv} mV failed ({e}); clocks unchanged");
            return;
        }
    }
    // DVM slews 10 mV per 7.8 us: 0.8 -> 0.94 V takes ~110 us.
    udelay(1000);
    ccu::set_cpu_clocks(op.cpu_l, op.cpu_b, op.dsu);
}
