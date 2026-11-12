// SPDX-License-Identifier: GPL-2.0-only
//! DRAM init through the open driver (crate `cubie-a7a-dram`).
//!
//! The PHY's own firmware (training firmware IMEM/DMEM and the PIE
//! sequencer images, Synopsys code that runs inside the PHY) is data: the
//! build extracts it from the pinned vendor library object by symbol name
//! and checks its SHA-256 (build.rs); no vendor code is linked.

use core::fmt;

use cubie_a7a_dram::{Error, InitError, Para, PhyFirmware};

use super::bt0hw::Bt0Hw;
use crate::soc;

static IMEM: &[u8] = include_bytes!(concat!(env!("OUT_DIR"), "/phy-imem.bin"));
static DMEM: &[u8] = include_bytes!(concat!(env!("OUT_DIR"), "/phy-dmem.bin"));
static PIE_A: &[u8] = include_bytes!(concat!(env!("OUT_DIR"), "/phy-pie-a.bin"));
static PIE_B: &[u8] = include_bytes!(concat!(env!("OUT_DIR"), "/phy-pie-b.bin"));
static PIE_A_1RANK: &[u8] = include_bytes!(concat!(env!("OUT_DIR"), "/phy-pie-a-1rank.bin"));
static PIE_B_1RANK: &[u8] = include_bytes!(concat!(env!("OUT_DIR"), "/phy-pie-b-1rank.bin"));

/// One init attempt with `para`; the size in MiB, or None.
pub fn init_one(para: &[u32; 32]) -> Option<u32> {
    if OPTS_BITS != 0 {
        println!("[bt0] DRAM: open init variant {OPTS_BITS:#x}: {OPTS:?}");
    }
    let fw = PhyFirmware {
        imem: IMEM,
        dmem: DMEM,
        pie: [PIE_A, PIE_B, PIE_A_1RANK, PIE_B_1RANK],
    };
    let mut p = Para::from_sysconfig(para);
    let t0 = soc::ticks();
    let r = cubie_a7a_dram::init_opts(&mut Bt0Hw, &mut p, &fw, &OPTS);
    #[cfg(feature = "hybrid")]
    if r.is_ok() && OPTS.stop_before_dfi {
        super::blobless::hybrid_after_pmu(&mut p);
    }
    let ms = (soc::ticks() - t0) / (soc::TIMER_HZ / 1000);
    match r {
        Ok(mib) => {
            println!(
                "[bt0] DRAM: open init done in {ms} ms; MR14 {:08x} MR15 {:08x}",
                p.w[50], p.w[51]
            );
            Some(mib)
        }
        Err(e) => {
            println!(
                "[bt0] DRAM: open init failed after {ms} ms: {}",
                ErrorText(e)
            );
            if matches!(e, InitError::ResultSave(failure) if failure.recovery_failed()) {
                // Do not automatically attempt another parameter set after
                // ambiguous restoration. The already armed boot watchdog and
                // existing halt/recovery path retain control.
                crate::halt();
            }
            None
        }
    }
}

struct ErrorText(InitError);

impl fmt::Display for ErrorText {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let error = match self.0 {
            InitError::Operation(error) => error,
            InitError::ResultSave(failure) => {
                return write!(f, "result save/recovery failed: {failure:?}")
            }
        };
        match error {
            Error::Power => write!(f, "PMIC refused a DRAM rail"),
            Error::Timeout(what) => write!(f, "timeout: {what}"),
            Error::Training { pstate, message } => {
                write!(
                    f,
                    "pstate {pstate} training ended with message {message:#x}"
                )
            }
            Error::Config(why) => write!(f, "{why}"),
            Error::Test {
                addr,
                expected,
                got,
            } => write!(
                f,
                "quick test at {addr:#x}: wrote {expected:#x}, read {got:#x}"
            ),
        }
    }
}

/// FEL lab variants (env `A7A_OPEN_OPTS` at build time, a bit mask; empty:
/// the vendor order): bit 0 no PIE program memory, bit 1 no step I, bit 2
/// the CPU answers dfi_init_start, bit 3 only one pstate trained, the one in
/// bits 4-5, bit 6 no DRAM tests in bt0, bit 7 stop before the DFI init
/// (feature `hybrid`: our stages follow), bits 8-11 the level of our mission
/// bring-up before the DFI init, bits 16-31 SequenceCtrl + 1 (0: from the
/// parameters).
const OPTS_BITS: u32 = match option_env!("A7A_OPEN_OPTS") {
    Some(s) => match u32::from_str_radix(s, 16) {
        Ok(v) => v,
        Err(_) => panic!("A7A_OPEN_OPTS is not a hex number"),
    },
    None => 0,
};
pub const OPTS: ::cubie_a7a_dram::phy::init::Opts = ::cubie_a7a_dram::phy::init::Opts {
    no_pie_mem: OPTS_BITS & 1 != 0,
    no_i_steps: OPTS_BITS & 2 != 0,
    cpu_dfi_answer: OPTS_BITS & 4 != 0,
    only_pstate: if OPTS_BITS & 8 != 0 {
        Some((OPTS_BITS >> 4) & 3)
    } else {
        None
    },
    sequence_ctrl: match OPTS_BITS >> 16 {
        0 => None,
        n => Some((n - 1) as u16),
    },
    no_quick_test: OPTS_BITS & 0x40 != 0,
    mission_level: (OPTS_BITS >> 8) & 0xf,
    stop_before_dfi: OPTS_BITS & 0x80 != 0,
};

/// Bit 6 of `A7A_OPEN_OPTS`: no DRAM test in bt0 (quick test, address
/// lines); entry 1 returns to FEL right after the init.
pub const SKIP_TESTS: bool = OPTS.no_quick_test;
