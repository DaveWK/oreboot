// SPDX-License-Identifier: GPL-2.0-only
//! Arm Power Policy Units (PPU, Arm DEN0051), one 4 KiB frame per power
//! domain as in an Arm PCK-600, over [`regio::RegIo`].
//!
//! Moved from ore-edk-boot-radxa-a7a ed42550 (oreboot a803f4f780d7)
//! `src/mainboard/radxa/cubie-a7a/main/src/ppu.rs`. The SoC's instance is a
//! [`Pck600`] value: frame base and spacing, domain count, the delay
//! registers written before a transition and the bus clock gate to open
//! before any frame access. The A733 domain numbering and values stay in the
//! board (the Linux sun60i-a733 binding and sun55i-pck600.c).
#![no_std]

use regio::RegIo;

/// PPU power policy register (bits [3:0]: the requested mode).
pub const PWPR: usize = 0x000;
/// PPU power status register (bits [3:0]: the current mode).
pub const PWSR: usize = 0x008;
pub const MODE_MASK: u32 = 0xf;
pub const MODE_ON: u32 = 0x8;

/// A clock gate bit that must be set before the frames are read.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Gate {
    pub addr: usize,
    pub bit: u32,
    /// Settle time after opening it.
    pub settle_us: u32,
}

/// One PCK-600 (or similar) instance.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Pck600<'a> {
    pub base: usize,
    /// log2 of the frame spacing (12: 4 KiB frames).
    pub frame_shift: u32,
    /// Domains `0..count` exist.
    pub count: u32,
    /// Register writes (frame offset, value) before an OFF-to-ON transition.
    pub setup: &'a [(usize, u32)],
    pub gate: Option<Gate>,
    /// Status polls, `poll_us` apart, before [`PpuError::Timeout`].
    pub polls: u32,
    pub poll_us: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PpuError {
    /// No such domain.
    BadDomain(u32),
    /// PWSR did not report ON in time; holds the last PWSR value.
    Timeout(u32),
}

impl Pck600<'_> {
    pub const fn frame(&self, domain: u32) -> usize {
        self.base + ((domain as usize) << self.frame_shift)
    }

    fn open_gate<R: RegIo + ?Sized>(&self, io: &mut R) {
        if let Some(g) = self.gate {
            let v = io.read32(g.addr);
            if v & (1 << g.bit) == 0 {
                io.write32(g.addr, v | (1 << g.bit));
                io.udelay(g.settle_us);
            }
        }
    }

    /// True if the domain's PPU reports ON (false for a nonexistent one).
    pub fn is_on<R: RegIo + ?Sized>(&self, io: &mut R, domain: u32) -> bool {
        if domain >= self.count {
            return false;
        }
        self.open_gate(io);
        io.read32(self.frame(domain) + PWSR) & MODE_MASK == MODE_ON
    }

    /// Request ON and wait for the status to follow. Nothing is written if
    /// the domain is already on.
    pub fn power_on<R: RegIo + ?Sized>(&self, io: &mut R, domain: u32) -> Result<(), PpuError> {
        if domain >= self.count {
            return Err(PpuError::BadDomain(domain));
        }
        if self.is_on(io, domain) {
            return Ok(());
        }
        let f = self.frame(domain);
        for &(off, v) in self.setup {
            io.write32(f + off, v);
        }
        let p = io.read32(f + PWPR);
        io.write32(f + PWPR, (p & !MODE_MASK) | MODE_ON);
        for _ in 0..self.polls {
            if io.read32(f + PWSR) & MODE_MASK == MODE_ON {
                return Ok(());
            }
            io.udelay(self.poll_us);
        }
        Err(PpuError::Timeout(io.read32(f + PWSR)))
    }
}
