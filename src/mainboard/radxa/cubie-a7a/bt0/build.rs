use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

const LINKERSCRIPT_FILENAME: &str = "link-cubie-a7a-bt0.ld";

// The A733 mask ROM copies the eGON image from boot media to SRAM at 0x47000
// and branches to its first word. The image (code, data, DRAM PHY firmware
// inside the vendor DRAM library) must fit in 256K; bss and stack follow.
// See SyterKit boards/radxa-cubie-a7a/configs (SPL_BIN_TEXT_BASE/MAX_SIZE).
const LINKERSCRIPT: &[u8] = b"
OUTPUT_ARCH(arm)
ENTRY(start)
MEMORY {
    SRAM : ORIGIN = 0x00047000, LENGTH = 256K
}
SECTIONS {
    .head : {
        KEEP(*(.head.egon))
        . = ALIGN(64);
        KEEP(*(.text.entry))
    } > SRAM
    .text : {
        *(.text .text.*)
    } > SRAM
    .rodata : ALIGN(8) {
        *(.rodata .rodata.*)
    } > SRAM
    .data : ALIGN(8) {
        *(.data .data.*)
    } > SRAM
    .ARM.exidx : {
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    } > SRAM
    _image_end = .;
    .bss (NOLOAD) : ALIGN(8) {
        _sbss = .;
        *(.bss .bss.* COMMON)
        . = ALIGN(8);
        _ebss = .;
    } > SRAM
    .stack (NOLOAD) : ALIGN(16) {
        . += 16K;
        _stack_top = .;
    } > SRAM

    ASSERT(start == ORIGIN(SRAM) + 0x40, \"eGON entry must be 0x40 bytes into the image\")

    /DISCARD/ : {
        *(.ARM.attributes)
        *(.comment*)
        *(.note*)
    }
}";

fn main() {
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join(LINKERSCRIPT_FILENAME))
        .unwrap()
        .write_all(LINKERSCRIPT)
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());

    // The A733 LPDDR5 init is only available as Allwinner's closed library
    // (ARMv7-A, soft-float EABI), as shipped by SyterKit. It is not part of
    // oreboot; point A733_DRAM_LIB at libdram-lpddr5.a to link it.
    println!("cargo:rerun-if-env-changed=A733_DRAM_LIB");
    match env::var_os("A733_DRAM_LIB") {
        Some(lib) => {
            let lib = PathBuf::from(lib);
            let dir = lib.parent().expect("A733_DRAM_LIB has no parent dir");
            let name = lib
                .file_stem()
                .and_then(|s| s.to_str())
                .and_then(|s| s.strip_prefix("lib"))
                .expect("A733_DRAM_LIB must be named lib<name>.a");
            println!("cargo:rerun-if-changed={}", lib.display());
            println!("cargo:rustc-link-search=native={}", dir.display());
            println!("cargo:rustc-link-lib=static={name}");
        }
        None => panic!(
            "set A733_DRAM_LIB to the vendor DRAM library, e.g. \
             SyterKit/soc/sun60iw2/lib/libdram-lpddr5.a"
        ),
    }
}
