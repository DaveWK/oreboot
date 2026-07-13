// Verbatim copy of ore-edk-boot-radxa-a7a ed42550 (oreboot tree a803f4f780d7)
// src/mainboard/radxa/cubie-a7a/main/src/ppu.rs, for the equivalence test; its
// //! header lines are // here and its inner #![allow] is an outer attribute
// on the including module (the file is include!d into a module).

// A733 PCK-600 peripheral power domains.
//
// The PCK-600 at 0x07060000 holds one Arm PPU per 4 KiB frame; domain n is
// frame n. Numbering follows the Linux binding
// include/dt-bindings/power/allwinner,sun60i-a733-pck-600.h, the same as
// EDK2's A733PowerDomainLib.
//
// - PPU_PWPR +0x0 (policy, bits [3:0]), PPU_PWSR +0x8 (status, bits [3:0]);
//   ON = 0x8 (Arm PPU architecture, DEN0051).
// - Delay registers +0x170, +0x174, +0xC00, +0xC04, +0xC10 are programmed
//   before a transition, with the values from Linux's sun55i-pck600.c
//   (sun60i_a733_pck600_desc) and the vendor BSP's pck600_domains.c. The
//   A733 instance has no reset line.
// - The PPU frames are clocked by R_CCU 0x07010000 + 0x1AC bit 0 (R_PPU
//   gate); reading a frame with that gate closed is not safe, so it is
//   opened first.

// The full domain list is kept so callers use names, not numbers.
// #![allow(dead_code)] (moved to the including module)

use cubie_a7a_common::time::udelay;
use util::mmio::{read32, write32};

pub const VI: u32 = 0;
pub const DE_SYS: u32 = 1;
pub const VE_DEC: u32 = 2;
pub const VE_ENC: u32 = 3;
pub const NPU: u32 = 4;
pub const GPU_TOP: u32 = 5;
pub const GPU_CORE: u32 = 6;
pub const PCIE: u32 = 7;
pub const USB2: u32 = 8;
pub const VO: u32 = 9;
pub const VO1: u32 = 10;
pub const COUNT: u32 = 11;

const PCK600_BASE: usize = 0x0706_0000;
const R_CCU_R_PPU_GATE: usize = 0x0701_0000 + 0x1ac;

const PWPR: usize = 0x000;
const PWSR: usize = 0x008;
const DCDR0: usize = 0x170;
const DCDR1: usize = 0x174;
const LPS0_DELAY: usize = 0xc00;
const LPS1_DELAY: usize = 0xc04;
const OFF2ON_DELAY: usize = 0xc10;

const MODE_MASK: u32 = 0xf;
const MODE_ON: u32 = 0x8;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PpuError {
    /// No such domain.
    BadDomain(u32),
    /// PWSR did not report ON within 10 ms; holds the last PWSR value.
    Timeout(u32),
}

const fn frame(domain: u32) -> usize {
    PCK600_BASE + ((domain as usize) << 12)
}

fn open_bus_clock() {
    let v = read32(R_CCU_R_PPU_GATE);
    if v & 1 == 0 {
        write32(R_CCU_R_PPU_GATE, v | 1);
        udelay(10);
    }
}

/// True if the domain's PPU reports ON.
pub fn is_on(domain: u32) -> bool {
    if domain >= COUNT {
        return false;
    }
    open_bus_clock();
    read32(frame(domain) + PWSR) & MODE_MASK == MODE_ON
}

/// Power a domain on and wait (at most 10 ms, as Linux allows) for the PPU
/// status to follow. Does nothing if it is already on.
pub fn power_on(domain: u32) -> Result<(), PpuError> {
    if domain >= COUNT {
        return Err(PpuError::BadDomain(domain));
    }
    if is_on(domain) {
        return Ok(());
    }
    let f = frame(domain);
    write32(f + DCDR0, 0x001f_1f1f);
    write32(f + DCDR1, 0x0000_1f1f);
    write32(f + LPS0_DELAY, 0x0808_0808);
    write32(f + LPS1_DELAY, 0x0000_0808);
    write32(f + OFF2ON_DELAY, 0x0000_0008);
    write32(f + PWPR, (read32(f + PWPR) & !MODE_MASK) | MODE_ON);
    for _ in 0..1000 {
        if read32(f + PWSR) & MODE_MASK == MODE_ON {
            return Ok(());
        }
        udelay(10);
    }
    Err(PpuError::Timeout(read32(f + PWSR)))
}
