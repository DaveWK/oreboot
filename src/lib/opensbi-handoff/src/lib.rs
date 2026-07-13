// SPDX-License-Identifier: GPL-2.0-only
//! Hand-off to OpenSBI's `fw_dynamic` firmware.
//!
//! From ore-edk-boot-opi `bt0/src/fit.rs` (K1). The layout of [`FwDynamicInfo`]
//! is OpenSBI's `include/sbi/fw_dynamic.h` (version 2). On RISC-V the jump
//! passes the boot hart in a0, the DT in a1 and the info structure in a2, as
//! `fw_dynamic` expects; the next stage's address and privilege mode are in
//! the structure. Other targets build the data types only.
#![no_std]

/// `FW_DYNAMIC_INFO_MAGIC_VALUE` ("OSBI").
pub const MAGIC: usize = 0x4942_534f;
/// The structure version this crate fills in.
pub const VERSION: usize = 2;

/// Privilege mode of the next stage (`FW_DYNAMIC_INFO_NEXT_MODE_*`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(usize)]
pub enum NextMode {
    User = 0,
    Supervisor = 1,
    Machine = 3,
}

/// OpenSBI `struct fw_dynamic_info`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(C)]
pub struct FwDynamicInfo {
    pub magic: usize,
    pub version: usize,
    pub next_addr: usize,
    pub next_mode: usize,
    pub options: usize,
    pub boot_hart: usize,
}

impl FwDynamicInfo {
    /// Info for `next_addr` in `mode`, started by `boot_hart`, no options.
    pub const fn new(next_addr: usize, mode: NextMode, boot_hart: usize) -> Self {
        Self {
            magic: MAGIC,
            version: VERSION,
            next_addr,
            next_mode: mode as usize,
            options: 0,
            boot_hart,
        }
    }
}

/// Jump to OpenSBI `fw_dynamic` at `firmware` on hart `hart` with the DT at
/// `fdt` and `info` (which must stay valid: keep it in a static).
///
/// # Safety
/// `firmware` must hold OpenSBI and `fdt` a valid DT; nothing returns.
#[cfg(target_arch = "riscv64")]
pub unsafe fn start(firmware: usize, hart: usize, fdt: usize, info: *const FwDynamicInfo) -> ! {
    core::arch::asm!(
        "fence.i",
        "jr {entry}",
        entry = in(reg) firmware,
        in("a0") hart,
        in("a1") fdt,
        in("a2") info as usize,
        options(noreturn)
    );
}

/// Copy `count` bytes to physical `destination`, which may be address zero
/// (OpenSBI loads there; Rust's copy intrinsics require non-null pointers, so
/// no Rust pointer to it is formed).
///
/// # Safety
/// Both ranges must be valid, distinct physical memory.
#[cfg(target_arch = "riscv64")]
pub unsafe fn copy_to_physical(source: usize, destination: usize, count: usize) {
    if count == 0 {
        return;
    }
    core::arch::asm!(
        "2:",
        "lbu t0, 0({source})",
        "sb t0, 0({destination})",
        "addi {source}, {source}, 1",
        "addi {destination}, {destination}, 1",
        "addi {count}, {count}, -1",
        "bnez {count}, 2b",
        source = inout(reg) source => _,
        destination = inout(reg) destination => _,
        count = inout(reg) count => _,
        out("t0") _,
        options(nostack)
    );
}
