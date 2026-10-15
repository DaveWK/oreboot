#![no_std]
#![no_main]
// TODO: remove when done debugging crap
#![allow(unused)]

#[macro_use]
extern crate log;

use core::{
    arch::{asm, naked_asm},
    mem::transmute,
    panic::PanicInfo,
    ptr::{self, addr_of, addr_of_mut},
    slice::from_raw_parts as slice_from,
};

use embedded_hal_nb::serial::Write;
use riscv::register::{marchid, mhartid, mimpid, mip, mvendorid};

mod dram;
mod eeprom;
mod fastboot;
mod fit;
mod fit_checked;
mod mmc;
mod pll;
mod k1io;
mod uart;
mod usb;

use uart::K1XSerial;
use util::{
    mem::{copy, dump_block},
    mmio::{read32, write32, write8},
};

pub type EntryPoint = unsafe extern "C" fn();

const SRAM0_BASE: usize = 0x0020_0000;
const SRAM0_SIZE: usize = 0x0002_0000;

const DRAM_BASE: usize = 0x0000_0000;
const FLASH_BASE: usize = 0xb800_0000;

const FLASH_SIZE: usize = 16 * 1024 * 1024;

// vendor partitions, taken from OrangePi RV 2 U-Boot SPL; U-Boot ends at
// - 0x0020_3fb0 (OrangePi RV2)
// - 0x0029_ce80 (Jupiter)
// 64K@0(bootinfo)
// 64K@64K(private)
// 256K@128K(fsbl),
// 64K@384K(env)
// 192K@448K(opensbi)
// -@640K(uboot)
const PART_BOOTINFO: usize = 0x0;
const PART_RESERVED: usize = 0x0001_0000;
const PART_FSBL: usize = 0x0002_0000;
const PART_UBOOT_ENV: usize = 0x0006_0000;
const PART_OPENSBI: usize = 0x0007_0000;
const PART_UBOOT: usize = 0x000a_0000;

// ours
const ORE_MAIN_OFFSET: usize = 4 * 1024 * 1024;

const LOAD_ADDR: usize = DRAM_BASE + 0x1000;

const MEM_TEST: bool = true;
const MEM_TEST_FULL: bool = false;

const DUMP_FLASH: bool = false;

// K1_BOOT=emmc|sd: load a FIT (OpenSBI, next stage, DT) from the eMMC boot1
// partition (OrangePi R2S) or from raw SD card sectors starting at
// K1_NEXT_LBA (OrangePi RV2: 8192, its 4 MiB "uboot" GPT partition).
// Build-time values (startup::env, ore-edk2-boot).
use startup::env::{eq as env_eq, u32 as env_u32};

const BOOT_FIT_EMMC: bool = env_eq(option_env!("K1_BOOT"), "emmc");
const BOOT_FIT_SD: bool = env_eq(option_env!("K1_BOOT"), "sd");
// K1_BOOT=nor: the FIT is in the SPI NOR, read through its memory-mapped
// window; K1_NEXT_LBA counts 512-byte sectors from the start of the flash.
const BOOT_FIT_NOR: bool = env_eq(option_env!("K1_BOOT"), "nor");
const BOOT_EMMC_FIT: bool = BOOT_FIT_EMMC || BOOT_FIT_SD || BOOT_FIT_NOR;
const NEXT_LBA: u32 = env_u32(option_env!("K1_NEXT_LBA"));
// K1_NEXT_PART: on an SD card, look the FIT up in the GPT by this partition
// name (then by the reserved-firmware type), K1_NEXT_LBA being the fallback.
// K1_PCIE_PWR_GPIO: a GPIO that switches a PCIe slot's 3.3 V supply on (OrangePi
// RV2: 116, the M.2 slot's vpcie3v3 regulator). 0 means none.
const PCIE_PWR_GPIO: u32 = env_u32(option_env!("K1_PCIE_PWR_GPIO"));
// Where the FIT is read to before its images are copied out.
const FIT_STAGING_ADDR: usize = 0x1000_0000;
// K1_FASTBOOT=no disables the fastboot flasher bt0 otherwise serves when the
// BootROM started it from USB download mode. K1_PRODUCT is its product name.
const FASTBOOT: bool = !env_eq(option_env!("K1_FASTBOOT"), "no");
const PRODUCT: &str = match option_env!("K1_PRODUCT") {
    Some(p) => p,
    None => "spacemit-k1",
};

// Otherwise hand off to a next stage appended to this image (see main()).
const PAYLOAD_HANDOFF: bool = !BOOT_EMMC_FIT;
const PAYLOAD_OFFSET: usize = 0x0001_0000;
const PAYLOAD_ADDR: usize = 0x0400_0000;
// Up to the end of the 256K SRAM the BootROM loads the image into.
const PAYLOAD_MAX_SIZE: usize = 0xc084_0000 - (0xc080_1000 + PAYLOAD_OFFSET);
const BOOT_FLASH: bool = false;

const STORAGE_API_P_ADDR: usize = 0xC083_8498;
const USB_BOOT_ENTRY: usize = 0xc083_81a0;
// 0xffe0_3b36
const SDCARD_API_ENTRY: usize = 0xFFE0_A548;

const GPIO_BASE: usize = 0xd401_9000;

const PINCTRL_BASE: usize = 0xd401_e000;

const GPIO68: usize = GPIO_BASE + 68 * 4;
// const GPIO68: usize = GPIO_BASE + 0x0110;

const PULL_DOWN: u32 = (5 << 13);
const PAD_1V8_DS2: u32 = (2 << 11);
const EDGE_NONE: u32 = (1 << 6);
const MUX_MODE2: u32 = 2;

// octacore, 2 clusters of 4x X60
const BOOT_HART_ID: usize = 0;

const STACK_SIZE: usize = 8 * 1024;

#[link_section = ".stack"]
static mut BT0_STACK: [u8; STACK_SIZE] = [0; STACK_SIZE];

/// Set up stack and jump to executable code.
///
/// # Safety
///
/// Naked function.
#[unsafe(naked)]
#[export_name = "start"]
#[link_section = ".text.entry"]
#[allow(named_asm_labels)]
pub unsafe extern "C" fn start() -> ! {
    naked_asm!(
        "auipc  s4, 0",

        "csrw   mstatus, zero",
        "csrw   mie, zero",
        "la     t0, .trap",
        "csrw   mtvec, t0",
        // 1. suspend non-boot hart
        "li     t1, {boothart}",
        "csrr   t0, mhartid",
        "bne    t0, t1, .nonboothart",
        // 2. prepare stack
        // NOTE: non-boot harts need no stack here, they skip this
        "la     sp, {stack}",
        "li     t0, {stack_size}",
        "add    sp, sp, t0",
        "j      .boothart",
        // wait for multihart to get back into the game
        ".nonboothart:",
        // "csrw   mie, (1 << 3)",
        "wfi",
        "j      .nonboothart",
        ".boothart:",

        ".option push",
        ".option norelax",
        "la     gp, __global_pointer$",
        ".option pop",
        "call   {reset}",
        ".balign 4",
        ".trap:",
        "wfi",
        "j      .trap",
        boothart   = const BOOT_HART_ID,
        stack      = sym BT0_STACK,
        stack_size = const STACK_SIZE,
        reset      = sym reset,
    )
}

/// Initialize RAM: Clear BSS and set up data.
/// See https://docs.rust-embedded.org/embedonomicon/main.html
///
/// # Safety
/// :shrug:
#[no_mangle]
pub unsafe extern "C" fn reset() {
    extern "C" {
        static mut _sbss: u8;
        static mut _ebss: u8;

        static mut _sdata: u8;
        static mut _edata: u8;
        static _sidata: u8;
    }

    startup::memory::initialize(
        addr_of_mut!(_sbss),
        addr_of_mut!(_ebss),
        addr_of_mut!(_sdata),
        addr_of_mut!(_edata),
        addr_of!(_sidata),
    );
    // Call user entry point
    main();
}

/// Print RISC-V core information: vendor, arch, implementation, hart ID
/// (names and CSR reads: startup::riscv, ore-edk2-boot).
fn print_ids() {
    let ids = startup::riscv::Ids::read();
    let (vid, aid, iid) = (ids.vendor, ids.arch, ids.implementation);
    println!("RISC-V arch {aid:08x}");
    let vendor_name = startup::riscv::vendor_name(vid);
    println!("RISC-V core vendor: {vendor_name} (0x{vid:04x})");
    let imp_name = startup::riscv::impl_name(iid as u64);
    println!("RISC-V implementation: {imp_name} (0x{iid:08x})");
    println!("RISC-V hart ID {}", ids.hart);
}

static mut SERIAL: Option<uart::K1XSerial> = None;

fn init_logger(s: uart::K1XSerial) {
    unsafe {
        SERIAL.replace(s);
        if let Some(m) = SERIAL.as_mut() {
            log::init(m);
        }
    }
}

#[no_mangle]
fn main() {
    let mut ini_pc: usize = 0;
    unsafe { asm!("mv {}, s4", out(reg) ini_pc) };

    let s = uart::K1XSerial::noinit();
    init_logger(s);
    println!("oreboot 🦀 bt0");
    println!("initial program counter (PC) {ini_pc:016x}");

    print_ids();

    let boot_mode = read32(STORAGE_API_P_ADDR);
    println!("Boot mode register: 0x{boot_mode:08x}");
    let boot_entry = read32(boot_mode as usize);
    println!("Boot entry: 0x{boot_entry:08x}");

    // Put PLL1-3 into states the OS clock drivers accept (see pll.rs) before
    // any later stage relies on them; a mismatch is reported, not fatal.
    pll::init();

    // On the OrangePi boot path the board EEPROM says how many DRAM ranks
    // there are, as it does for SpacemiT's U-Boot SPL. The next stage (EDK2)
    // also reads it early and relies on this bus setup.
    let ddr = if BOOT_EMMC_FIT {
        eeprom_i2c_init();
        eeprom::ddr_info()
    } else {
        eeprom::DdrInfo::default()
    };
    let cs_num = match eeprom::resolve_profile(&ddr, dram::CS_NUM, dram::DATA_RATE) {
        Ok(n) => {
            println!(
                "[bt0] DRAM ranks: {n}; EEPROM rank {:?}, rate {:?} MT/s; profile {} MT/s",
                ddr.cs_num,
                ddr.data_rate,
                dram::DATA_RATE
            );
            n
        }
        Err(e) => {
            println!("[bt0] EEPROM/profile mismatch: {e:?}");
            halt();
        }
    };
    let dram_mb = match dram::init(cs_num) {
        Ok(mib) => mib,
        Err(e) => {
            println!("[bt0] DRAM initialization failed: {e:?}");
            halt();
        }
    };
    let expected_mb = env_u32(option_env!("K1_DRAM_MIB"));
    if dram::check_profile_size(dram_mb, expected_mb).is_err() {
        println!("[bt0] DRAM size {dram_mb} MiB differs from board profile {expected_mb} MiB");
        halt();
    }

    const DRAM_SIZE: usize = 0x8000_0000;
    if MEM_TEST {
        if MEM_TEST_FULL {
            // NOTE: The full test will take _very long_.
            // FIXME: We need a little offset from address 0 because Rust
            // currently errors otherwise.
            util::mem::test(DRAM_BASE + 0x10, DRAM_SIZE - 0x10);
        } else {
            // Test a small amount of DRAM only.
            util::mem::test(LOAD_ADDR, 2 * 1024 * 1024);
        }
    }

    if DUMP_FLASH {
        dump_block(FLASH_BASE, FLASH_SIZE, 32);
    }

    // Started from USB download mode (fastboot stage + continue): the
    // BootROM's storage API is its USB loader in SRAM. Serve fastboot so the
    // host can write the board's storage, then boot as usual.
    if FASTBOOT && boot_mode as usize == USB_BOOT_ENTRY {
        println!("[bt0] started from USB download mode: serving fastboot");
        if let Err(e) = fastboot::run(&fastboot::Board {
            name: PRODUCT,
            disk: if BOOT_FIT_EMMC {
                mmc::Kind::Emmc
            } else {
                mmc::Kind::Sd
            },
        }) {
            println!("[bt0] fastboot failed: {e:?}; refusing boot handoff");
            halt();
        }
        println!("[bt0] fastboot done, booting");
    }

    if BOOT_EMMC_FIT {
        if PCIE_PWR_GPIO != 0 {
            // Switch the slot supply on early so the next stage finds the
            // link; nothing else on this boot path drives the regulator.
            gpio_output_high(PCIE_PWR_GPIO);
            println!("[bt0] PCIe slot power on (GPIO {PCIE_PWR_GPIO})");
        }
        // Load OpenSBI, the next stage and its DT from a FIT in eMMC boot1,
        // then start OpenSBI (fw_dynamic) with that next stage.
        let src = fit::Source {
            medium: if BOOT_FIT_NOR {
                fit::Medium::Nor
            } else if BOOT_FIT_SD {
                fit::Medium::Mmc(mmc::Kind::Sd, mmc::Partition::User)
            } else {
                fit::Medium::Mmc(mmc::Kind::Emmc, mmc::Partition::Boot1)
            },
            lba: NEXT_LBA,
            gpt_name: option_env!("K1_NEXT_PART"),
        };
        match fit::load(&src, FIT_STAGING_ADDR) {
            Ok(b) => {
                println!(
                    "[bt0] OpenSBI @{:08x}, next @{:08x}, DT @{:08x}",
                    b.firmware, b.next, b.fdt
                );
                if let Err(e) = fit::fix_memory(b.fdt, b.fdt_size, (dram_mb as u64) << 20) {
                    println!("[bt0] DT memory fixup failed: {e:?}");
                    halt();
                }
                fit::start_opensbi(BOOT_HART_ID, &b);
            }
            Err(e) => {
                println!("[bt0] FIT boot failed: {e:?}");
                halt();
            }
        }
    }

    if PAYLOAD_HANDOFF {
        // The FSBL image carries the next stage (U-Boot SPL, linked to run
        // from DRAM) at a fixed offset behind bt0. Copy it out of SRAM and
        // run it with DRAM up; it loads OpenSBI and U-Boot proper.
        let src = ini_pc + PAYLOAD_OFFSET;
        println!(
            "[bt0] Copy next stage {src:08x} -> {PAYLOAD_ADDR:08x} ({PAYLOAD_MAX_SIZE} bytes)"
        );
        copy(src, PAYLOAD_ADDR, PAYLOAD_MAX_SIZE);
        println!("[bt0] Jump to next stage @{PAYLOAD_ADDR:08x}");
        unsafe {
            asm!(
                "fence.i",
                "jr {entry}",
                entry = in(reg) PAYLOAD_ADDR,
                in("a0") BOOT_HART_ID,
                in("a1") 0usize,
                options(noreturn)
            );
        }
    }

    if BOOT_FLASH {
        copy(FLASH_BASE + ORE_MAIN_OFFSET, LOAD_ADDR, 0x8000);

        // GO!
        println!("[bt0] Jump to main stage @{LOAD_ADDR:08x}");
        exec_payload(LOAD_ADDR);

        println!("[bt0] Exit from main stage, resetting...");
    } else {
        println!("DRAM init done, WFI...");
    }

    unsafe {
        // udelay(0x0100_0000);
        riscv::asm::wfi()
    };
}

// jump to main stage or payload
// I2C2 (TWSI2) carries the board TLV EEPROM; U-Boot SPL's i2c_early_init()
// and SpacemiT's FSBL set it up the same way.
const APBC_TWSI2_CLK_RST: usize = 0xd401_5038; // bit 0 bus clk, bit 1 func clk, bit 2 reset
const MFP_GPIO_84: usize = 0xd401_e154;
const MFP_GPIO_85: usize = 0xd401_e158;
// MUX_MODE4 | EDGE_NONE | PULL_UP | PAD_1V8_DS2
const I2C_PIN_CONFIG: u32 = 4 | (1 << 6) | (6 << 13) | (1 << 12);

// TWSI2 controller and its SCL timing registers.
const TWSI2_BASE: usize = 0xd401_2000;
const TWSI_ILCR: usize = 0x10;
const TWSI_IWCR: usize = 0x14;
// Functional clock mux (APBC_TWSIn_CLK_RST bits 6:4): only index 2 (pll1,
// 61.44 MHz) has a running parent on these boards; at the power-on index 0
// the registers respond but the state machine never moves.
const TWSI_CLK_MUX_61M44: u32 = 2 << 4;
// SCL timing for a 61.44 MHz functional clock, as FreeBSD's spacemit_i2c
// derives it: standard-mode SLV 307 (100 kHz), fast-mode FLV 76 (400 kHz),
// vendor high-speed fields kept; IWCR SDA hold of 312 ns (19 cycles).
const TWSI_ILCR_61M44: u32 = 0x082c_9933;
const TWSI_IWCR_61M44: u32 = 0x0000_1433;

fn eeprom_i2c_init() {
    let v = read32(APBC_TWSI2_CLK_RST) & !(0x7 << 4);
    write32(
        APBC_TWSI2_CLK_RST,
        (v | TWSI_CLK_MUX_61M44 | 0b11) & !(1 << 2),
    );
    write32(MFP_GPIO_84, I2C_PIN_CONFIG);
    write32(MFP_GPIO_85, I2C_PIN_CONFIG);
    // The next stage's I2C driver (SpacemiT's EDK2) resets the unit but never
    // programs SCL timing; without it every transfer times out.
    write32(TWSI2_BASE + TWSI_ILCR, TWSI_ILCR_61M44);
    write32(TWSI2_BASE + TWSI_IWCR, TWSI_IWCR_61M44);
    println!(
        "[bt0] EEPROM I2C: clk {:08x} ILCR {:08x} IWCR {:08x}",
        read32(APBC_TWSI2_CLK_RST),
        read32(TWSI2_BASE + TWSI_ILCR),
        read32(TWSI2_BASE + TWSI_IWCR)
    );
}

// K1 GPIO and pad mux (U-Boot drivers/gpio/spacemit_gpio.c and
// drivers/pinctrl/spacemit/pinctrl-k1.c).
const GPIO_BASE_ADDR: usize = 0xd401_9000;
const PAD_MUX_BASE: usize = 0xd401_e000;

fn gpio_bank_offset(pin: u32) -> usize {
    match pin / 32 {
        0 => 0x0,
        1 => 0x4,
        2 => 0x8,
        _ => 0x100,
    }
}

fn pad_mux_reg(pin: u32) -> usize {
    let p = pin as usize;
    let offset = 1 + match p {
        0..=85 => p,
        86..=92 => p + 36,
        93..=97 => p + 23,
        98 => 92,
        99 => 91,
        100 => 90,
        101 => 89,
        102 => 94,
        103 => 93,
        104..=110 => p + 5,
        _ => p + 19,
    };
    PAD_MUX_BASE + (offset << 2)
}

fn gpio_mux(pin: u32) -> u32 {
    match pin {
        70..=73 | 93..=103 => 1,
        104..=109 => 4,
        _ => 0,
    }
}

fn gpio_output_high(pin: u32) {
    let bank = GPIO_BASE_ADDR + gpio_bank_offset(pin);
    let bit = 1u32 << (pin % 32);
    write32(bank + 0x18, bit); // GPSR: drive high
    write32(bank + 0x54, bit); // GSDR: output
    let pad = pad_mux_reg(pin);
    write32(pad, (read32(pad) & !0x7) | gpio_mux(pin));
}

fn exec_payload(addr: usize) {
    unsafe {
        let f: EntryPoint = transmute(addr);
        asm!("fence.i");
        f();
    }
}

#[cfg_attr(not(test), panic_handler)]
fn panic(info: &PanicInfo) -> ! {
    if let Some(location) = info.location() {
        println!(
            "[bt0] panic in '{}' line {}",
            location.file(),
            location.line(),
        );
    } else {
        println!("[bt0] panic at unknown location");
    };
    let msg = info.message();
    println!("[bt0]   {msg}");
    loop {
        core::hint::spin_loop();
    }
}

// Every failure path parks before touching DRAM consumers or entering fastboot.
fn halt() -> ! {
    loop {
        unsafe { riscv::asm::wfi() };
    }
}

// Our own DRAM training (cargo feature own-training), the side-by-side
// training report (train-report) and the hardware probe (train-probe).
// Declared at the end so the default build is unchanged.
#[cfg(any(feature = "own-training", feature = "train-report"))]
mod train_engine;
#[cfg(any(feature = "own-training", feature = "train-report"))]
mod train_eye;
#[cfg(feature = "own-training")]
mod train_own;
#[cfg(feature = "train-probe")]
mod train_probe;
#[cfg(feature = "train-report")]
mod train_report;
