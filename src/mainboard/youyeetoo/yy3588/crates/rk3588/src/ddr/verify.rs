// SPDX-License-Identifier: GPL-2.0-only
//! Post-init checks through the CPU: the boot smoke test of crate memtest
//! ([`memtest::quick`]) per channel and over the whole space, and a size
//! check by address aliasing.
//!
//! The aliasing check follows the idea of the RK3568 U-Boot RFC (Pavel
//! Golikov, GPL-2.0+, sdram_rk3568.c dram_detect_cs1_row(): write 0 at the
//! base, a pattern at base + 2^n, and accept n only if the pattern reads
//! back and the base still reads 0), applied to every power of two below
//! the detected size: a geometry that claims more than is fitted folds a
//! high address onto a low one and is caught before the handoff.

use regio::{RegIo, Timeout};

/// The RK3568 RFC's PATTERN.
pub const PATTERN: u32 = 0x5aa5_f00f;

/// Forwards a `?Sized` RegIo to crates that take a sized one.
pub struct Fwd<'a, H: RegIo + ?Sized>(pub &'a mut H);

impl<H: RegIo + ?Sized> RegIo for Fwd<'_, H> {
    fn read32(&mut self, a: usize) -> u32 {
        self.0.read32(a)
    }
    fn write32(&mut self, a: usize, v: u32) {
        self.0.write32(a, v)
    }
    fn read16(&mut self, a: usize) -> u16 {
        self.0.read16(a)
    }
    fn write16(&mut self, a: usize, v: u16) {
        self.0.write16(a, v)
    }
    fn udelay(&mut self, us: u32) {
        self.0.udelay(us)
    }
    fn udelay_checked(&mut self, us: u32) -> Result<(), Timeout> {
        self.0.udelay_checked(us)
    }
    fn now_us(&mut self) -> u64 {
        self.0.now_us()
    }
    fn log(&mut self, args: core::fmt::Arguments) {
        self.0.log(args)
    }
    fn kick_watchdog(&mut self) {
        self.0.kick_watchdog()
    }
}

/// memtest::quick over `mib` MiB at `base` (start and middle, `words`
/// words each). Err(address) of the first mismatch.
pub fn quick<H: RegIo + ?Sized>(
    io: &mut H,
    base: usize,
    mib: u64,
    words: usize,
) -> Result<(), usize> {
    let mib = mib.clamp(2, 4096) as u32;
    match memtest::quick(&mut Fwd(io), base, mib, words) {
        Ok(()) => Ok(()),
        Err(memtest::QuickError::Mismatch { addr, .. }) => Err(addr),
        Err(memtest::QuickError::Config(_)) => Err(base),
    }
}

/// Aliasing check of `total_mib` MiB at `base`: Err(offset) of the first
/// power-of-two offset that folds onto the base.
pub fn alias<H: RegIo + ?Sized>(io: &mut H, base: usize, total_mib: u64) -> Result<(), usize> {
    let mut off: usize = 1 << 20;
    while (off as u64) < (total_mib << 20) {
        io.write32(base, 0);
        io.write32(base + off, PATTERN);
        if io.read32(base + off) != PATTERN || io.read32(base) != 0 {
            return Err(off);
        }
        off <<= 1;
    }
    Ok(())
}
