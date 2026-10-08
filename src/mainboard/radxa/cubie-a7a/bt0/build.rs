use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

const LINKERSCRIPT_FILENAME: &str = "link-cubie-a7a-bt0.ld";

// The A733 mask ROM copies the eGON image from boot media to SRAM at 0x47000
// and branches to its first word. The image (code, data, DRAM PHY firmware
// inside the vendor DRAM library), bss and stack must end below 0x90000,
// where recovery.rs stages the vendor SPL. The BROM loads eGON images of at
// least 0x4a000 bytes (the vendor boot0 on the Cubie A7A card).
// See SyterKit boards/radxa-cubie-a7a/configs (SPL_BIN_TEXT_BASE/MAX_SIZE).
const LINKERSCRIPT: &[u8] = b"
OUTPUT_ARCH(arm)
ENTRY(start)
MEMORY {
    SRAM : ORIGIN = 0x00047000, LENGTH = 0x49000
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

    if env::var_os("CARGO_FEATURE_DRAMLIB").is_some() {
        link_dram_library();
    } else {
        extract_phy_firmware(out);
    }
}

/// Feature `dramlib`: link Allwinner's closed A733 LPDDR5 library
/// (ARMv7-A, soft-float EABI). A733_DRAM_LIB names the archive.
fn link_dram_library() {
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
        None => panic!("feature dramlib: set A733_DRAM_LIB to the vendor DRAM library archive"),
    }
}

#[path = "../common/src/sha256.rs"]
#[allow(dead_code)]
mod sha256;

/// The DRAM PHY's firmware, which runs inside the PHY (Synopsys training
/// firmware and PIE sequencer images): data the open driver loads. It
/// comes from the pinned Tina SDK dramlib object (A733_PHY_FW_OBJ,
/// patches/dramlib/pin.env); only these data symbols are read, no code.
/// Each image must match its SHA-256.
const PHY_FW: [(&str, &str, &str); 6] = [
    (
        "imem_lpddr5",
        "phy-imem.bin",
        "65a0730d0c74cf34e2e3303d9aa54cbab742aff8a789b12499934a3932d864a7",
    ),
    (
        "dmem_lpddr5",
        "phy-dmem.bin",
        "705534c5a995c9ff4be52dfa9a31e0cdb14858c7769ea59dc84fd55db81b06b6",
    ),
    (
        "pie1_lpddr5",
        "phy-pie-a.bin",
        "91320fc6d2e2e2cc2d2acc16856e0c94ad860b9f3775beec8e9896494a4dc15f",
    ),
    (
        "pie3_lpddr5",
        "phy-pie-b.bin",
        "ef51c191dcf9176dc2df0dbb096df2aafa27519fda1f93f52bd08ab8cbd2139d",
    ),
    (
        "pie2_lpddr5",
        "phy-pie-a-1rank.bin",
        "315b502f03971180db072c448770cfa4a26b53aedfc251d1a1de864ecbe81020",
    ),
    (
        "pie4_lpddr5",
        "phy-pie-b-1rank.bin",
        "cd738f7047255581b710ce6a0b1345a5e277a542adc6b7e27de8d04b0c4b413f",
    ),
];

fn extract_phy_firmware(out: &std::path::Path) {
    println!("cargo:rerun-if-env-changed=A733_PHY_FW_OBJ");
    let Some(path) = env::var_os("A733_PHY_FW_OBJ") else {
        panic!("set A733_PHY_FW_OBJ to the Tina SDK dramlib object (DRAM PHY firmware)");
    };
    println!("cargo:rerun-if-changed={}", PathBuf::from(&path).display());
    let obj = std::fs::read(&path).expect("A733_PHY_FW_OBJ");
    for (sym, file, sha) in PHY_FW {
        let data = elf_symbol_data(&obj, sym).unwrap_or_else(|e| panic!("{sym}: {e}"));
        let hex: String = sha256::digest(&data)
            .iter()
            .map(|b| format!("{b:02x}"))
            .collect();
        assert_eq!(
            hex, sha,
            "{sym}: SHA-256 mismatch, not the pinned PHY firmware"
        );
        std::fs::write(out.join(file), &data).unwrap();
    }
}

/// The bytes of data symbol `name` in an ELF32 little-endian relocatable
/// object (the symbols read here carry no relocations).
fn elf_symbol_data(obj: &[u8], name: &str) -> Result<Vec<u8>, String> {
    let u16_at = |o: usize| u16::from_le_bytes([obj[o], obj[o + 1]]) as usize;
    let u32_at = |o: usize| u32::from_le_bytes(obj[o..o + 4].try_into().unwrap()) as usize;
    if obj.len() < 52 || &obj[..4] != b"\x7fELF" || obj[4] != 1 || obj[5] != 1 {
        return Err("not an ELF32 little-endian object".into());
    }
    let (shoff, shentsize, shnum) = (u32_at(32), u16_at(46), u16_at(48));
    let sec = |i: usize| shoff + i * shentsize;
    let symtab = (0..shnum)
        .find(|&i| u32_at(sec(i) + 4) == 2)
        .ok_or("no symbol table")?;
    let strtab = u32_at(sec(u32_at(sec(symtab) + 24)) + 16);
    let (off, size) = (u32_at(sec(symtab) + 16), u32_at(sec(symtab) + 20));
    for k in 0..size / 16 {
        let e = off + 16 * k;
        let n = strtab + u32_at(e);
        let end = n + obj[n..].iter().position(|&b| b == 0).unwrap_or(0);
        if &obj[n..end] != name.as_bytes() {
            continue;
        }
        let (value, len, shndx) = (u32_at(e + 4), u32_at(e + 8), u16_at(e + 14));
        let data_off = u32_at(sec(shndx) + 16) + value;
        return Ok(obj[data_off..data_off + len].to_vec());
    }
    Err("symbol not found".into())
}
