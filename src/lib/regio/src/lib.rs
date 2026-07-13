// SPDX-License-Identifier: GPL-2.0-only
//! The one hardware-access trait of ore-edk2-boot, and bounded polling.
//!
//! Split out of `dram-train` (which re-exports it as `dram_train::io`) so
//! that non-DRAM code (clocks, PCIe, power domains, UARTs) uses the same
//! trait and the same host register model.
//!
//! A board implements [`RegIo`] once on its MMIO, timer, log and watchdog
//! (bt0), and host tests implement it on a register model. Board-only needs
//! (PMIC rails, clocks) are a board trait with `RegIo` as its supertrait.

#![no_std]

use core::fmt;

pub trait RegIo {
    fn read32(&mut self, addr: usize) -> u32;
    fn write32(&mut self, addr: usize, val: u32);
    fn read16(&mut self, addr: usize) -> u16;
    fn write16(&mut self, addr: usize, val: u16);
    /// 8-bit read. The default reads the containing 32-bit word, which is
    /// not a byte access on the bus: implement it for hardware whose byte
    /// registers must be accessed alone (SDHCI host and power control,
    /// software reset).
    fn read8(&mut self, addr: usize) -> u8 {
        (self.read32(addr & !3) >> (8 * (addr & 3))) as u8
    }
    /// 8-bit write. The default is a read-modify-write of the containing
    /// 32-bit word, which also rewrites its other bytes: implement it for
    /// hardware with byte registers.
    fn write8(&mut self, addr: usize, val: u8) {
        let s = 8 * (addr & 3);
        self.modify32(addr & !3, |x| (x & !(0xff << s)) | ((val as u32) << s));
    }
    /// Busy-wait.
    fn udelay(&mut self, us: u32);
    /// Free-running microsecond clock, for timeouts.
    fn now_us(&mut self) -> u64;
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

/// A condition did not hold in time; names what was waited for.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Timeout(pub &'static str);

/// Independent software cap for controller/MR/DFI polling. This is not a
/// characterized physical timing margin; individual callbacks must return.
pub const WAIT_MAX_POLLS: u32 = 10_000_000;

/// Poll `cond` before each timeout check, including the final permitted poll.
/// A frozen clock is bounded by [`WAIT_MAX_POLLS`]; wrapping elapsed time
/// avoids absolute deadline overflow. A zero deadline permits one immediate
/// check. Nothing is logged on the failure path: the UART can have its own
/// dependencies.
pub fn wait<H: RegIo + ?Sized>(
    hw: &mut H,
    what: &'static str,
    timeout_us: u64,
    mut cond: impl FnMut(&mut H) -> bool,
) -> Result<(), Timeout> {
    let t0 = hw.now_us();
    for polls in 1..=WAIT_MAX_POLLS {
        if cond(hw) {
            return Ok(());
        }
        if polls == WAIT_MAX_POLLS || hw.now_us().wrapping_sub(t0) >= timeout_us {
            return Err(Timeout(what));
        }
    }
    unreachable!()
}

#[cfg(any(test, feature = "mock"))]
pub mod mock {
    //! A fixed-capacity register model for host tests (no allocation): reads
    //! of unwritten registers return 0 unless a responder answers, the clock
    //! advances `tick_us` per `now_us` call, and every access is counted.
    use super::RegIo;
    use core::fmt;

    /// Answers a read of `addr` from the stored registers, or None.
    pub type Responder = fn(&mut [(usize, u32)], usize) -> Option<u32>;

    pub struct Mock<const N: usize> {
        regs: [(usize, u32); N],
        used: usize,
        pub clock_us: u64,
        pub tick_us: u64,
        pub reads: u32,
        pub writes: u32,
        /// Called on every read before the stored value: Some(v) answers it.
        pub responder: Option<Responder>,
    }

    impl<const N: usize> Default for Mock<N> {
        fn default() -> Self {
            Self {
                regs: [(0, 0); N],
                used: 0,
                clock_us: 0,
                tick_us: 1,
                reads: 0,
                writes: 0,
                responder: None,
            }
        }
    }

    impl<const N: usize> Mock<N> {
        pub fn get(&self, addr: usize) -> u32 {
            self.regs[..self.used]
                .iter()
                .find(|r| r.0 == addr)
                .map_or(0, |r| r.1)
        }
        pub fn set(&mut self, addr: usize, val: u32) {
            if let Some(r) = self.regs[..self.used].iter_mut().find(|r| r.0 == addr) {
                r.1 = val;
            } else {
                assert!(self.used < N, "Mock register capacity {N} exceeded");
                self.regs[self.used] = (addr, val);
                self.used += 1;
            }
        }
    }

    impl<const N: usize> RegIo for Mock<N> {
        fn read32(&mut self, addr: usize) -> u32 {
            self.reads += 1;
            if let Some(f) = self.responder {
                if let Some(v) = f(&mut self.regs[..self.used], addr) {
                    return v;
                }
            }
            self.get(addr)
        }
        fn write32(&mut self, addr: usize, val: u32) {
            self.writes += 1;
            self.set(addr, val)
        }
        fn read16(&mut self, addr: usize) -> u16 {
            self.read32(addr) as u16
        }
        fn write16(&mut self, addr: usize, val: u16) {
            self.write32(addr, val as u32)
        }
        fn udelay(&mut self, us: u32) {
            self.clock_us += us as u64;
        }
        fn now_us(&mut self) -> u64 {
            self.clock_us += self.tick_us;
            self.clock_us
        }
        fn log(&mut self, _args: fmt::Arguments) {}
    }
}

/// Kani proofs (P3-A7; `cargo kani` in this crate, the `kani` workflow).
#[cfg(kani)]
mod proofs {
    use super::*;

    /// Registers read anything; the clock moves by 1-4 us per read from
    /// any start (wrap included).
    pub(crate) struct AnyIo {
        pub t: u64,
        pub step: u64,
    }
    impl RegIo for AnyIo {
        fn read32(&mut self, _: usize) -> u32 {
            kani::any()
        }
        fn write32(&mut self, _: usize, _: u32) {}
        fn read16(&mut self, _: usize) -> u16 {
            kani::any()
        }
        fn write16(&mut self, _: usize, _: u16) {}
        fn udelay(&mut self, _: u32) {}
        fn now_us(&mut self) -> u64 {
            self.t = self.t.wrapping_add(self.step);
            self.t
        }
        fn log(&mut self, _: fmt::Arguments) {}
    }

    /// `wait` never reaches its `unreachable!`, never panics, and gives up
    /// once the timeout has passed, whatever the condition answers.
    #[kani::proof]
    #[kani::unwind(10)]
    fn wait_ends_by_its_timeout() {
        let step: u64 = kani::any();
        kani::assume((1..=4).contains(&step));
        let timeout: u64 = kani::any();
        kani::assume(timeout <= 6);
        let mut io = AnyIo {
            t: kani::any(),
            step,
        };
        let mut polls = 0u32;
        let r = wait(&mut io, "x", timeout, |_| {
            polls += 1;
            kani::any()
        });
        assert!(polls as u64 <= timeout + 1);
        if let Err(Timeout(w)) = r {
            assert_eq!(w, "x");
        }
    }
}
