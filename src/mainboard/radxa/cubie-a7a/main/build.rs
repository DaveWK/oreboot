use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

const LINKERSCRIPT_FILENAME: &str = "link-cubie-a7a-main.ld";

// oreboot main stays resident at the bottom of DRAM (see
// cubie_a7a_common::mem): code, data, bss and per-CPU EL3 stacks must fit
// in MAIN_MAX_SIZE (512K).
const LINKERSCRIPT: &[u8] = b"
OUTPUT_ARCH(aarch64)
ENTRY(start)
MEMORY {
    DRAM : ORIGIN = 0x40000000, LENGTH = 512K
}
SECTIONS {
    .text : {
        KEEP(*(.text.entry))
        *(.text .text.*)
    } > DRAM
    .rodata : ALIGN(8) {
        *(.rodata .rodata.*)
    } > DRAM
    .data : ALIGN(8) {
        *(.data .data.*)
    } > DRAM
    _image_end = .;
    .bss (NOLOAD) : ALIGN(16) {
        _sbss = .;
        *(.bss .bss.*)
        . = ALIGN(16);
        _ebss = .;
    } > DRAM
    .stacks (NOLOAD) : ALIGN(16) {
        _stacks_bottom = .;
        . += 8 * 16K;
        _stacks_top = .;
    } > DRAM

    /DISCARD/ : {
        *(.comment*)
        *(.note*)
        *(.eh_frame*)
    }
}";

fn main() {
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join(LINKERSCRIPT_FILENAME))
        .unwrap()
        .write_all(LINKERSCRIPT)
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());
}
