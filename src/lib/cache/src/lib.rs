// SPDX-License-Identifier: GPL-2.0-only
//! Data-cache maintenance for DMA on SoCs whose peripherals are not cache
//! coherent.
//!
//! Moved from ore-edk-boot-opi 78479e21ba `oreboot/src/mainboard/spacemit/
//! k1x/bt0/src/cache.rs` (SpacemiT K1: X60 cores with Zicbom, 64-byte
//! blocks). The block walk is [`blocks`] (host-testable); [`zicbom`] issues
//! the instructions. An AArch64 `dc civac`/`ivac` back end can sit beside it
//! when a board needs one.
#![no_std]

/// The cache-block-aligned addresses covering `[addr, addr + len)`, in
/// order (`block` a power of two).
pub fn blocks(addr: usize, len: usize, block: usize) -> impl Iterator<Item = usize> {
    let start = addr & !(block - 1);
    let end = addr + len;
    (start..end).step_by(block)
}

/// RISC-V Zicbom: `cbo.flush` / `cbo.inval` per block with full fences.
pub mod zicbom {
    /// The K1 (X60) cache block.
    pub const BLOCK: usize = 64;

    /// Write back and invalidate: call before a device reads the range, or
    /// before it writes the range (so no dirty line is evicted over its data).
    pub fn flush(addr: usize, len: usize) {
        for a in super::blocks(addr, len, BLOCK) {
            cbo_flush(a);
        }
        fence();
    }

    /// Discard cached copies: call after a device wrote the range, before
    /// the CPU reads it.
    pub fn invalidate(addr: usize, len: usize) {
        fence();
        for a in super::blocks(addr, len, BLOCK) {
            cbo_inval(a);
        }
        fence();
    }

    #[cfg(any(target_arch = "riscv64", target_arch = "riscv32"))]
    #[inline(always)]
    fn cbo_flush(a: usize) {
        // cbo.flush 0(a)
        unsafe { core::arch::asm!(".insn i 0x0f, 2, x0, {0}, 2", in(reg) a) };
    }
    #[cfg(any(target_arch = "riscv64", target_arch = "riscv32"))]
    #[inline(always)]
    fn cbo_inval(a: usize) {
        // cbo.inval 0(a)
        unsafe { core::arch::asm!(".insn i 0x0f, 2, x0, {0}, 0", in(reg) a) };
    }
    #[cfg(any(target_arch = "riscv64", target_arch = "riscv32"))]
    #[inline(always)]
    fn fence() {
        unsafe { core::arch::asm!("fence rw, rw") };
    }

    // Other targets (host tests) have no Zicbom: the walk still runs.
    #[cfg(not(any(target_arch = "riscv64", target_arch = "riscv32")))]
    fn cbo_flush(_: usize) {}
    #[cfg(not(any(target_arch = "riscv64", target_arch = "riscv32")))]
    fn cbo_inval(_: usize) {}
    #[cfg(not(any(target_arch = "riscv64", target_arch = "riscv32")))]
    fn fence() {}
}
