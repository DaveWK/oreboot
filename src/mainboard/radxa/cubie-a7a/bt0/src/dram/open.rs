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

#[cfg(not(feature = "dram-lab"))]
static IMEM: &[u8] = include_bytes!(concat!(env!("OUT_DIR"), "/phy-imem.bin"));
#[cfg(not(feature = "dram-lab"))]
static DMEM: &[u8] = include_bytes!(concat!(env!("OUT_DIR"), "/phy-dmem.bin"));
#[cfg(not(feature = "dram-lab"))]
static PIE_A: &[u8] = include_bytes!(concat!(env!("OUT_DIR"), "/phy-pie-a.bin"));
#[cfg(not(feature = "dram-lab"))]
static PIE_B: &[u8] = include_bytes!(concat!(env!("OUT_DIR"), "/phy-pie-b.bin"));
#[cfg(not(feature = "dram-lab"))]
static PIE_A_1RANK: &[u8] = include_bytes!(concat!(env!("OUT_DIR"), "/phy-pie-a-1rank.bin"));
#[cfg(not(feature = "dram-lab"))]
static PIE_B_1RANK: &[u8] = include_bytes!(concat!(env!("OUT_DIR"), "/phy-pie-b-1rank.bin"));

/// DRAM lab mode (feature `dram-lab`): the same six images, in NOLOAD
/// statics in the ROM's FEL SRAM (build.rs `.phyfw`), filled by
/// [`lab_fw::load`] from the host's staged copy before the run; the
/// training uses them exactly as the embedded ones.
#[cfg(feature = "dram-lab")]
pub mod lab_fw {
    include!(concat!(env!("OUT_DIR"), "/phy-fw-lab.rs"));
    macro_rules! buf {
        ($name:ident, $i:expr) => {
            #[unsafe(link_section = ".phyfw")]
            pub static mut $name: [u8; LEN[$i]] = [0; LEN[$i]];
        };
    }
    buf!(IMEM, 0);
    buf!(DMEM, 1);
    buf!(PIE_A, 2);
    buf!(PIE_B, 3);
    buf!(PIE_A_1RANK, 4);
    buf!(PIE_B_1RANK, 5);

    static mut LOADED: bool = false;

    pub fn loaded() -> bool {
        unsafe { core::ptr::read_volatile(core::ptr::addr_of!(LOADED)) }
    }

    /// Check the host's blob (`A7AP`, count, length and CRC per image,
    /// the images) against this build's lengths and CRCs, then copy it.
    pub fn load(blob: &[u8]) -> Result<(), &'static str> {
        let le = |o: usize| {
            blob.get(o..o + 4)
                .map(|b| u32::from_le_bytes([b[0], b[1], b[2], b[3]]))
        };
        if blob.get(..4) != Some(b"A7AP") || le(4) != Some(LEN.len() as u32) {
            return Err("not this build's PHY firmware blob");
        }
        let mut at = 8 + 8 * LEN.len();
        let dst = [
            core::ptr::addr_of_mut!(IMEM).cast::<u8>(),
            core::ptr::addr_of_mut!(DMEM).cast::<u8>(),
            core::ptr::addr_of_mut!(PIE_A).cast::<u8>(),
            core::ptr::addr_of_mut!(PIE_B).cast::<u8>(),
            core::ptr::addr_of_mut!(PIE_A_1RANK).cast::<u8>(),
            core::ptr::addr_of_mut!(PIE_B_1RANK).cast::<u8>(),
        ];
        for i in 0..LEN.len() {
            let (n, crc) = (le(8 + 8 * i), le(12 + 8 * i));
            let img = blob.get(at..at + LEN[i]).ok_or("blob too short")?;
            if n != Some(LEN[i] as u32)
                || crc != Some(CRC[i])
                || cubie_a7a_common::crc32(img) != CRC[i]
            {
                return Err("PHY firmware image does not match this build");
            }
            // SAFETY: the static's length is LEN[i]; nothing reads it yet.
            unsafe { core::ptr::copy_nonoverlapping(img.as_ptr(), dst[i], LEN[i]) };
            at += LEN[i];
        }
        unsafe { core::ptr::write_volatile(core::ptr::addr_of_mut!(LOADED), true) };
        Ok(())
    }
}
#[cfg(feature = "dram-lab")]
static IMEM: &[u8] = unsafe { &*core::ptr::addr_of!(lab_fw::IMEM) };
#[cfg(feature = "dram-lab")]
static DMEM: &[u8] = unsafe { &*core::ptr::addr_of!(lab_fw::DMEM) };
#[cfg(feature = "dram-lab")]
static PIE_A: &[u8] = unsafe { &*core::ptr::addr_of!(lab_fw::PIE_A) };
#[cfg(feature = "dram-lab")]
static PIE_B: &[u8] = unsafe { &*core::ptr::addr_of!(lab_fw::PIE_B) };
#[cfg(feature = "dram-lab")]
static PIE_A_1RANK: &[u8] = unsafe { &*core::ptr::addr_of!(lab_fw::PIE_A_1RANK) };
#[cfg(feature = "dram-lab")]
static PIE_B_1RANK: &[u8] = unsafe { &*core::ptr::addr_of!(lab_fw::PIE_B_1RANK) };

/// One init attempt with `para`; the size in MiB, or None.
pub fn init_one(para: &[u32; 32]) -> Option<u32> {
    let bits = ::cubie_a7a_dram::knobs::get().open_opts;
    let opts = opts(bits);
    if bits != 0 {
        println!("[bt0] DRAM: open init variant {bits:#x}: {opts:?}");
    }
    let fw = PhyFirmware {
        imem: IMEM,
        dmem: DMEM,
        pie: [PIE_A, PIE_B, PIE_A_1RANK, PIE_B_1RANK],
    };
    let mut p = Para::from_sysconfig(para);
    let t0 = soc::ticks();
    let r = cubie_a7a_dram::init_opts(&mut Bt0Hw, &mut p, &fw, &opts);
    #[cfg(feature = "hybrid")]
    if r.is_ok() && opts.stop_before_dfi {
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
/// bring-up before the DFI init, bit 12 the PHY duty-cycle/receiver CSR
/// snapshot before and after each pstate's training, bits 16-31
/// SequenceCtrl + 1 (0: from the
/// parameters).
/// (`::cubie_a7a_dram::knobs::get().open_opts`: the build's `A7A_OPEN_OPTS`,
/// or the bt0 DRAM lab mode's runtime value.)
pub const fn opts(bits: u32) -> ::cubie_a7a_dram::phy::init::Opts {
    ::cubie_a7a_dram::phy::init::Opts {
        no_pie_mem: bits & 1 != 0,
        no_i_steps: bits & 2 != 0,
        cpu_dfi_answer: bits & 4 != 0,
        only_pstate: if bits & 8 != 0 {
            Some((bits >> 4) & 3)
        } else {
            None
        },
        sequence_ctrl: match bits >> 16 {
            0 => None,
            n => Some((n - 1) as u16),
        },
        no_quick_test: bits & 0x40 != 0,
        mission_level: (bits >> 8) & 0xf,
        stop_before_dfi: bits & 0x80 != 0,
        snap: bits & 0x1000 != 0,
    }
}

/// Bit 6 of `A7A_OPEN_OPTS`: no DRAM test in bt0 (quick test, address
/// lines); entry 1 returns to FEL right after the init.
pub fn skip_tests() -> bool {
    opts(::cubie_a7a_dram::knobs::get().open_opts).no_quick_test
}
