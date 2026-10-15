use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

const LINKERSCRIPT_FILENAME: &str = "link-spacemit-k1x-bt0.ld";

const LINKERSCRIPT: &[u8] = br#"
OUTPUT_ARCH(riscv)
ENTRY(start)
MEMORY {
    # SRAM : ORIGIN = 0x200000, LENGTH = 256k
    # taken from U-Boot
    SRAM : ORIGIN = 0xc0801000, LENGTH = 252k
}
SECTIONS {
    .head : {
        *(.head.text)
    } > SRAM
    .text : {
        KEEP(*(.text.entry))
        *(.text .text.*)
        . = ALIGN(8);
    } > SRAM
    .bss (NOLOAD) : {
        . = ALIGN(16);
        _sbss = .;
        *(.bss .bss.* .sbss .sbss.*);
        *(COMMON);
        _ebss = .;
    } > SRAM

    .stack (NOLOAD) : {
        . = ALIGN(16);
        _sstack = .;
        KEEP(*(.stack))
        _estack = .;
    } > SRAM
    ASSERT(_ebss <= _sstack, "BSS overlaps live stack")

    # https://docs.rust-embedded.org/embedonomicon/main.html
    .rodata : {
        *(.rodata .rodata.*);
    } > SRAM #FLASH
    .data : {
        . = ALIGN(16);
        _sdata = .;
        PROVIDE(__global_pointer$ = . + 0x800);
        *(.data .data.* .sdata .sdata.*);
        _edata = .;
    } > SRAM
    _sidata = LOADADDR(.data);

    /DISCARD/ : {
        *(.eh_frame)
        *(.debug_*)
        *(.comment*)
    }
}"#;

fn main() {
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join(LINKERSCRIPT_FILENAME))
        .unwrap()
        .write_all(LINKERSCRIPT)
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());
}
