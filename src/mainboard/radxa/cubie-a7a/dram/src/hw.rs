//! What the DRAM init needs from its environment. bt0 implements this on
//! the real hardware; the host harness implements it on a register model.

use core::fmt;

/// Power rails the DRAM init asks the PMIC for (AXP8191 names, as the
/// board's vendor parameters use them).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Rail {
    /// VDD2H, the LPDDR5 core/IO supply (DCDC7).
    Vdd2h,
    /// VDD2L (ELDO1), tied to VDD2H's voltage on the Cubie A7A.
    Vdd2l,
    /// VDDQ (DCDC6).
    Vddq,
}

impl Rail {
    pub const fn pmic_name(self) -> &'static str {
        match self {
            Rail::Vdd2h => "dcdc7",
            Rail::Vdd2l => "eldo1",
            Rail::Vddq => "dcdc6",
        }
    }
}

pub trait Hw {
    fn read32(&mut self, addr: usize) -> u32;
    fn write32(&mut self, addr: usize, val: u32);
    fn read16(&mut self, addr: usize) -> u16;
    fn write16(&mut self, addr: usize, val: u16);
    /// Busy-wait.
    fn udelay(&mut self, us: u32);
    /// Free-running microsecond clock, for timeouts.
    fn now_us(&mut self) -> u64;
    /// Switch `rail` on at `mv`; false if the PMIC refused.
    fn set_voltage(&mut self, rail: Rail, mv: u32) -> bool;
    fn log(&mut self, args: fmt::Arguments);
    /// Restart the platform's boot watchdog (long steps: PHY training).
    fn kick_watchdog(&mut self) {}

    fn modify32(&mut self, addr: usize, f: impl FnOnce(u32) -> u32) {
        let v = self.read32(addr);
        self.write32(addr, f(v));
    }
    fn set32(&mut self, addr: usize, bits: u32) {
        self.modify32(addr, |v| v | bits);
    }
    fn clr32(&mut self, addr: usize, bits: u32) {
        self.modify32(addr, |v| v & !bits);
    }
    /// Clear `clear`, then set `set`, in one read-modify-write.
    fn clrset32(&mut self, addr: usize, clear: u32, set: u32) {
        self.modify32(addr, |v| (v & !clear) | set);
    }
}

/// Why DRAM init failed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    /// The PMIC refused a DRAM rail.
    Power,
    /// A status bit did not reach its value in time.
    Timeout(&'static str),
    /// PHY training of a pstate ended with this mailbox message.
    Training { pstate: u8, message: u32 },
    /// The parameter set describes no usable configuration.
    Config(&'static str),
    /// The quick write/read check after init failed.
    Test {
        addr: usize,
        expected: u32,
        got: u32,
    },
}

/// Independent software cap for controller/MR/DFI polling. This is not a
/// characterized physical timing margin; individual callbacks must return.
pub const WAIT_MAX_POLLS: u32 = 10_000_000;

/// Poll `cond` before each timeout check, including the final permitted poll.
/// A frozen clock is bounded by WAIT_MAX_POLLS; wrapping elapsed time avoids
/// absolute deadline overflow. A zero deadline permits one immediate check.
/// Do not log on the failure path here: the UART can have its own dependencies.
pub fn wait<H: Hw>(
    hw: &mut H,
    what: &'static str,
    timeout_us: u64,
    mut cond: impl FnMut(&mut H) -> bool,
) -> Result<(), Error> {
    let t0 = hw.now_us();
    for polls in 1..=WAIT_MAX_POLLS {
        if cond(hw) {
            return Ok(());
        }
        if polls == WAIT_MAX_POLLS || hw.now_us().wrapping_sub(t0) >= timeout_us {
            return Err(Error::Timeout(what));
        }
    }
    unreachable!()
}
