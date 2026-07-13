// SPDX-License-Identifier: GPL-2.0-only
//! First-stage start-up helpers shared by the boards' bt0/main.
//!
//! Moved from ore-edk-boot-opi 78479e21ba `oreboot/src/mainboard/spacemit/
//! k1x/bt0/src/`: `startup.rs` ([`memory`]), the const build-environment
//! parsers of `main.rs` ([`env`]), the RISC-V id names of `main.rs` and the
//! cycle counter of `train_report.rs` ([`riscv`]).
#![no_std]

pub mod memory {
    //! Linker-delimited start-up operations, independently exercised on
    //! poisoned RAM.

    /// Initialize BSS and initialized data. The live stack is in a distinct
    /// linker section.
    ///
    /// # Safety
    /// All ranges must be valid, ordered, and BSS/data must exclude the live
    /// stack. Data is either loaded in place or has a nonoverlapping load
    /// address.
    pub unsafe fn initialize(
        sbss: *mut u8,
        ebss: *mut u8,
        sdata: *mut u8,
        edata: *mut u8,
        sidata: *const u8,
    ) {
        core::ptr::write_bytes(sbss, 0, ebss as usize - sbss as usize);
        // In-place .data is already initialized by the ROM image copy.
        // Copying it to itself with copy_nonoverlapping would be undefined
        // behaviour. Linker symbols may alias even though LLVM treats
        // distinct extern statics as distinct objects. Force comparison of
        // their resolved machine addresses; otherwise LTO can remove this
        // guard and emit a self-copy anyway.
        let source = core::hint::black_box(sidata as usize);
        let destination = core::hint::black_box(sdata as usize);
        if source != destination {
            core::ptr::copy_nonoverlapping(sidata, sdata, edata as usize - sdata as usize);
        }
    }
}

pub mod env {
    //! Build-time configuration from `option_env!` values, usable in
    //! `const` items.

    /// `v` is present and equals `want`.
    pub const fn eq(v: Option<&str>, want: &str) -> bool {
        match v {
            None => false,
            Some(v) => {
                let (a, b) = (v.as_bytes(), want.as_bytes());
                if a.len() != b.len() {
                    return false;
                }
                let mut i = 0;
                while i < a.len() {
                    if a[i] != b[i] {
                        return false;
                    }
                    i += 1;
                }
                true
            }
        }
    }

    /// Decimal value of `v`, 0 when absent. The digits are not checked (as
    /// the K1 bt0 used it, for values its build script writes).
    pub const fn u32(v: Option<&str>) -> u32 {
        match v {
            None => 0,
            Some(v) => {
                let b = v.as_bytes();
                let mut n = 0u32;
                let mut i = 0;
                while i < b.len() {
                    n = n * 10 + (b[i] - b'0') as u32;
                    i += 1;
                }
                n
            }
        }
    }
}

pub mod riscv {
    //! RISC-V machine-mode ids and counters.

    /// Name of a `mvendorid` value.
    pub fn vendor_name<'a>(vendorid: usize) -> &'a str {
        match vendorid {
            0x0489 => "SiFive",
            0x0710 => "SpacemiT",
            _ => "unknown",
        }
    }

    /// Name of a `mimpid` value (SiFive U74-MC manual 21G3; SpacemiT X60).
    /// `u64`: the X60's value does not fit a 32-bit `usize`.
    pub fn impl_name<'a>(impid: u64) -> &'a str {
        match impid {
            0x0421_0427 => "21G1.02.00 / llama.02.00-general",
            0x1000_0000_4977_2200 => "X60",
            _ => "unknown",
        }
    }

    /// `mvendorid`, `marchid`, `mimpid` and `mhartid` of this hart.
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    pub struct Ids {
        pub vendor: usize,
        pub arch: usize,
        pub implementation: usize,
        pub hart: usize,
    }

    #[cfg(any(target_arch = "riscv64", target_arch = "riscv32"))]
    macro_rules! csrr {
        ($csr:literal) => {{
            let v: usize;
            // SAFETY: reading a machine-mode information or counter CSR.
            unsafe { core::arch::asm!(concat!("csrr {0}, ", $csr), out(reg) v) };
            v
        }};
    }

    impl Ids {
        /// Read the four id CSRs (machine mode).
        #[cfg(any(target_arch = "riscv64", target_arch = "riscv32"))]
        pub fn read() -> Self {
            Self {
                vendor: csrr!("mvendorid"),
                arch: csrr!("marchid"),
                implementation: csrr!("mimpid"),
                hart: csrr!("mhartid"),
            }
        }
    }

    /// `mcycle` (machine mode); 0 on other targets.
    pub fn cycles() -> u64 {
        #[cfg(target_arch = "riscv64")]
        {
            csrr!("mcycle") as u64
        }
        #[cfg(not(target_arch = "riscv64"))]
        {
            0
        }
    }
}
