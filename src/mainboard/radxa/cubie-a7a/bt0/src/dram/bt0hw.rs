//! [`Hw`] for the DRAM driver on bt0: MMIO, the timer, the PMIC rails, the
//! UART log and the boot watchdog. Shared by the open and blobless backends.

use core::fmt;

use cubie_a7a_dram::{Hw, Rail};

use super::PMIC;
use crate::soc;

pub(super) struct Bt0Hw;

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
        // This mandatory settling ABI cannot return an error. Never silently
        // shorten it or proceed with a stalled counter: stop through the
        // timer-free failure path and the already armed boot watchdog.
        crate::ccu::delay_us(us).unwrap_or_else(|error| crate::clock_failure(error));
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
                println!("[bt0] DRAM: no PMIC; voltage request failed");
                false
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
