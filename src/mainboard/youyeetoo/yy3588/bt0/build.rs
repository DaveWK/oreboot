// SPDX-License-Identifier: GPL-2.0-only
//! Linker script: SYSTEM_SRAM 0xff001000 (BootROM init image, own DRAM
//! init) or DRAM 0x0 with feature `stepping-stone` (BootROM "boot" image
//! after the rkbin DDR init returned; mainline U-Boot links the RK3588 SPL
//! there, arch/arm/mach-rockchip/Kconfig SPL_TEXT_BASE). The DRAM build
//! must stay below BL31 at 0x40000: 192 KiB including stack.
use std::{env, fs, path::PathBuf};

fn main() {
    let dram = env::var_os("CARGO_FEATURE_STEPPING_STONE").is_some();
    let (origin, length) = if dram {
        ("0x00000000", "192K")
    } else {
        ("0xff001000", "252K")
    };
    let script = format!(
        "OUTPUT_ARCH(aarch64)
ENTRY(start)
MEMORY {{ RAM : ORIGIN = {origin}, LENGTH = {length} }}
SECTIONS {{
    .text : {{ KEEP(*(.text.entry)) *(.text .text.*) . = ALIGN(8); }} > RAM
    .rodata : {{ *(.rodata .rodata.*) . = ALIGN(8); }} > RAM
    .data : {{ *(.data .data.*) . = ALIGN(8); }} > RAM
    _image_end = .;
    .bss (NOLOAD) : {{ . = ALIGN(16); _sbss = .; *(.bss .bss.*) *(COMMON) . = ALIGN(16); _ebss = .; }} > RAM
    _stack_top = ORIGIN(RAM) + LENGTH(RAM);
    _ram_end = ORIGIN(RAM) + LENGTH(RAM);
    /DISCARD/ : {{ *(.eh_frame*) *(.comment*) }}
}}
"
    );
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    fs::write(out.join("yy3588-bt0.ld"), script).unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=build.rs");
}
