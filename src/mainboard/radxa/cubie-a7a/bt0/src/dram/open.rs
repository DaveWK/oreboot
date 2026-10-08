//! DRAM init through the open driver (crate `cubie-a7a-dram`).
//!
//! The PHY's own firmware (training firmware IMEM/DMEM and the PIE
//! sequencer images, Synopsys code that runs inside the PHY) is data: the
//! build extracts it from the pinned vendor library object by symbol name
//! and checks its SHA-256 (build.rs); no vendor code is linked.

use core::fmt;

use cubie_a7a_dram::{Error, Hw, Para, PhyFirmware, Rail};

use super::PMIC;
use crate::soc;

static IMEM: &[u8] = include_bytes!(concat!(env!("OUT_DIR"), "/phy-imem.bin"));
static DMEM: &[u8] = include_bytes!(concat!(env!("OUT_DIR"), "/phy-dmem.bin"));
static PIE_A: &[u8] = include_bytes!(concat!(env!("OUT_DIR"), "/phy-pie-a.bin"));
static PIE_B: &[u8] = include_bytes!(concat!(env!("OUT_DIR"), "/phy-pie-b.bin"));
static PIE_A_1RANK: &[u8] = include_bytes!(concat!(env!("OUT_DIR"), "/phy-pie-a-1rank.bin"));
static PIE_B_1RANK: &[u8] = include_bytes!(concat!(env!("OUT_DIR"), "/phy-pie-b-1rank.bin"));

struct Bt0Hw;

impl Hw for Bt0Hw {
    fn read32(&mut self, addr: usize) -> u32 {
        util::mmio::read32(addr)
    }
    fn write32(&mut self, addr: usize, val: u32) {
        util::mmio::write32(addr, val)
    }
    fn read16(&mut self, addr: usize) -> u16 {
        util::mmio::read16(addr)
    }
    fn write16(&mut self, addr: usize, val: u16) {
        util::mmio::write16(addr, val)
    }
    fn udelay(&mut self, us: u32) {
        soc::udelay(us as u64)
    }
    fn now_us(&mut self) -> u64 {
        soc::ticks() / (soc::TIMER_HZ / 1_000_000)
    }
    fn set_voltage(&mut self, rail: Rail, mv: u32) -> bool {
        let name = rail.pmic_name();
        println!("[bt0] DRAM: set {name} to {mv} mV");
        match unsafe { (*core::ptr::addr_of!(PMIC)).as_ref() } {
            Some(pmic) => match pmic.set_voltage(name, mv, Some(true)) {
                Ok(()) => true,
                Err(e) => {
                    println!("[bt0] DRAM: {name}: {e}");
                    false
                }
            },
            None => {
                println!("[bt0] DRAM: no PMIC, voltage request ignored");
                true
            }
        }
    }
    fn log(&mut self, args: fmt::Arguments) {
        print!("{args}");
    }
    fn kick_watchdog(&mut self) {
        crate::recovery::kick();
    }
}

/// One init attempt with `para`; the size in MiB, or None.
pub fn init_one(para: &[u32; 32]) -> Option<u32> {
    let fw = PhyFirmware {
        imem: IMEM,
        dmem: DMEM,
        pie: [PIE_A, PIE_B, PIE_A_1RANK, PIE_B_1RANK],
    };
    let mut p = Para::from_sysconfig(para);
    let t0 = soc::ticks();
    let r = cubie_a7a_dram::init(&mut Bt0Hw, &mut p, &fw);
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
            None
        }
    }
}

struct ErrorText(Error);

impl fmt::Display for ErrorText {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.0 {
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
