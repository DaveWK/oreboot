//! What the DRAM init needs from its environment. bt0 implements this on
//! the real hardware; the host harness implements it on a register model.

pub use dram_train::io::{RegIo, WAIT_MAX_POLLS};

/// Power rails the DRAM init asks the PMIC for (AXP8191 names, as the
/// board's vendor parameters use them).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Rail {
    /// VDD2H, the LPDDR5 core/IO supply (DCDC7).
    Vdd2h,
    /// VDD2L (ELDO1), the separate 900 mV VCC09-VDD2L net.
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

/// The A733 DRAM init's environment: the shared register access
/// ([`RegIo`]: MMIO, timer, log, watchdog) plus the board's PMIC rails.
pub trait Hw: RegIo {
    /// Switch `rail` on at `mv`; false if the PMIC refused.
    fn set_voltage(&mut self, rail: Rail, mv: u32) -> bool;
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

/// Poll `cond` before each timeout check, including the final permitted poll.
/// A frozen clock is bounded by WAIT_MAX_POLLS; wrapping elapsed time avoids
/// absolute deadline overflow. A zero deadline permits one immediate check.
/// Do not log on the failure path here: the UART can have its own dependencies.
pub fn wait<H: Hw>(
    hw: &mut H,
    what: &'static str,
    timeout_us: u64,
    cond: impl FnMut(&mut H) -> bool,
) -> Result<(), Error> {
    dram_train::io::wait(hw, what, timeout_us, cond).map_err(|t| Error::Timeout(t.0))
}
