//! oreboot bt0 for the Radxa Cubie A7A (Allwinner A733, sun60iw2).
//!
//! Loaded by the mask ROM from the SD card (eGON image at 8 KiB) into SRAM
//! and run in AArch32. It brings up clocks, UART0, the PMIC and LPDDR5,
//! loads oreboot main and the payload from the SD card as described by the
//! boot layout table, then warm-resets CPU0 into AArch64 at oreboot main.
#![no_std]
#![no_main]
#![cfg_attr(feature = "dramlib", feature(c_variadic))]

#[macro_use]
extern crate log;

use core::arch::{asm, global_asm, naked_asm};
use core::panic::PanicInfo;
use core::ptr::addr_of_mut;
use core::sync::atomic::{AtomicBool, Ordering};

use cubie_a7a_common::{
    crc32, layout, mem,
    twi::Twi,
    uart::{init_logger, Uart, UART0_BASE},
    Handoff, HANDOFF_MAGIC, HANDOFF_VERSION, R_TWI0_BASE,
};
use util::mmio::write32;

mod axp;
mod ccu;
mod dram;
mod dvfs;
mod gpadc;
mod gpio;
mod recovery;
mod smhc;
mod soc;

use gpio::{Port, Pull};

const SRAM_BASE: u32 = 0x0004_7000;

// The BROM checks this header, copies `length` bytes to SRAM_BASE and
// branches to its first word. Fields follow the vendor A733 boot0 images
// (as shipped by Armbian for the Cubie A7Z / Orange Pi 4 Pro). Checksum and
// length are filled in by the image tool (SyterKit mksunxi algorithm).
global_asm!(
    r#"
    .section .head.egon, "ax"
    .arm
    .global egon_head
egon_head:
    b       start                 @ 0x00: jump over the header to +0x40
    .ascii  "eGON.BT0"            @ 0x04
    .word   0x5f0a6c39            @ 0x0c: checksum (stamp value)
    .word   0                     @ 0x10: length
    .word   48                    @ 0x14: header size
    .word   0                     @ 0x18: header version (0 in vendor boot0)
    .word   {base}                @ 0x1c: return address
    .word   {base}                @ 0x20: run address
    .word   0                     @ 0x24: boot CPU
    .byte   0, 0, 0, 0, '4', '.', '0', 0  @ 0x28: platform, as vendor boot0
    "#,
    base = const SRAM_BASE,
);

// Exception vectors: report what happened instead of hanging silently.
global_asm!(
    r#"
    .section .text.vectors, "ax"
    .arm
    .balign 32
    .global bt0_vectors
bt0_vectors:
    b       start
    b       1f
    b       2f
    b       3f
    b       4f
    b       .
    b       5f
    b       6f
1:  mov     r0, #1
    b       7f
2:  mov     r0, #2
    b       7f
3:  mov     r0, #3
    b       7f
4:  mov     r0, #4
    b       7f
5:  mov     r0, #5
    b       7f
6:  mov     r0, #6
7:  mov     r1, lr
    ldr     sp, =_stack_top
    bl      bt0_exception
    b       .
    "#
);

/// # Safety
/// Entry point from the mask ROM; sets up the C runtime environment.
#[unsafe(naked)]
#[unsafe(export_name = "start")]
#[unsafe(link_section = ".text.entry")]
pub unsafe extern "C" fn start() -> ! {
    naked_asm!(
        ".arm",
        // SVC mode, IRQ and FIQ masked
        "mrs    r0, cpsr",
        "bic    r0, r0, #0x1f",
        "orr    r0, r0, #0xd3",
        "msr    cpsr_c, r0",
        // Green power LED (PJ26, active low) on: the first sign bt0 runs,
        // before clocks or UART. Linux keeps it on later.
        "ldr    r0, ={led_cfg}",
        "ldr    r1, [r0]",
        "bic    r1, r1, #(0xf << {led_shift})",
        "orr    r1, r1, #(0x1 << {led_shift})",
        "str    r1, [r0]",
        "ldr    r0, ={led_dat}",
        "ldr    r1, [r0]",
        "bic    r1, r1, #(1 << {led_pin})",
        "str    r1, [r0]",
        // VBAR
        "ldr    r0, =bt0_vectors",
        "mcr    p15, 0, r0, c12, c0, 0",
        // SCTLR: MMU, alignment check and D-cache off, low vectors via VBAR,
        // I-cache and branch prediction on.
        "mrc    p15, 0, r0, c1, c0, 0",
        "bic    r0, r0, #0x7",
        "bic    r0, r0, #(1 << 13)",
        "orr    r0, r0, #(1 << 12)",
        "orr    r0, r0, #(1 << 11)",
        "mcr    p15, 0, r0, c1, c0, 0",
        "isb",
        "ldr    sp, =_stack_top",
        // clear .bss
        "ldr    r0, =_sbss",
        "ldr    r1, =_ebss",
        "mov    r2, #0",
        "1:",
        "cmp    r0, r1",
        "strlo  r2, [r0], #4",
        "blo    1b",
        "bl     {main}",
        "2:",
        "wfi",
        "b      2b",
        main = sym main,
        led_cfg = const gpio::POWER_LED_CFG,
        led_dat = const gpio::POWER_LED_DAT,
        led_shift = const gpio::POWER_LED_CFG_SHIFT,
        led_pin = const gpio::POWER_LED_PIN,
    )
}

#[unsafe(no_mangle)]
extern "C" fn bt0_exception(kind: u32, lr: u32) -> ! {
    let name = match kind {
        1 => "undefined instruction",
        2 => "SVC",
        3 => "prefetch abort",
        4 => "data abort",
        5 => "IRQ",
        _ => "FIQ",
    };
    let (dfsr, dfar, ifsr, ifar): (u32, u32, u32, u32);
    unsafe {
        asm!(
            "mrc p15, 0, {}, c5, c0, 0",
            "mrc p15, 0, {}, c6, c0, 0",
            "mrc p15, 0, {}, c5, c0, 1",
            "mrc p15, 0, {}, c6, c0, 2",
            out(reg) dfsr, out(reg) dfar, out(reg) ifsr, out(reg) ifar
        )
    };
    if !UART_UP.load(Ordering::Relaxed) {
        // Nothing to print on: blink the exception number on the power LED.
        gpio::blink_forever(kind);
    }
    println!("[bt0] EXCEPTION: {name}, lr {lr:08x}");
    println!("[bt0]   DFSR {dfsr:08x} DFAR {dfar:08x} IFSR {ifsr:08x} IFAR {ifar:08x}");
    halt()
}

fn halt() -> ! {
    println!("[bt0] halted; resetting in 10 s");
    soc::mdelay(10_000);
    soc::watchdog_reset()
}

/// Clock failure can invalidate UART/timer assumptions. Keep the already armed
/// DCXO watchdog running; record exact failure without any timer delay, UART,
/// storage access, new PLL gate-off, or attempt to execute vendor fallback.
/// RTC diagnostic0/1/2 then contain address/lastvalue/phase+readcount; stageE*
/// distinguishes this from the normal initial-clock snapshots.
fn clock_failure(error: ccu::Error) -> ! {
    soc::diag(0, error.addr as u32);
    soc::diag(1, error.last);
    soc::diag(2, ((error.phase as u32) << 24) | error.reads);
    soc::stage(0xe0 | error.phase as u32);
    loop {
        unsafe { asm!("wfi") };
    }
}

/// The UART runs from SYS_CLK24M, which the A733 derives from its REFPLL at a
/// fixed 24 MHz even on boards with a 26 MHz crystal such as the Cubie A7A
/// (A733 manual; same choice as Yixun Lan's U-Boot, COUNTER_FREQUENCY too).
const UART_CLOCK_HZ: u32 = 24_000_000;

fn init_uart() {
    // UART0: PB9 = TX, PB10 = RX (40-pin header pins 8 and 10)
    gpio::set_function(Port::B, 9, 2);
    gpio::set_function(Port::B, 10, 2);
    gpio::set_pull(Port::B, 10, Pull::Up);
    ccu::uart_clock_24m();
    ccu::bus_enable(ccu::UART0_BGR).unwrap_or_else(|error| crate::clock_failure(error));
    cubie_a7a_common::fwlog::start_early();
    init_logger(Uart::init(UART0_BASE, UART_CLOCK_HZ, 115_200));
    UART_UP.store(true, Ordering::Relaxed);
}

static UART_UP: AtomicBool = AtomicBool::new(false);

fn init_pmic() -> Option<axp::Axp<'static>> {
    static mut TWI: Option<Twi> = None;
    // R_TWI0: PL0 = SCK, PL1 = SDA
    gpio::set_function(Port::L, 0, 2);
    gpio::set_function(Port::L, 1, 2);
    gpio::set_pull(Port::L, 0, Pull::Up);
    gpio::set_pull(Port::L, 1, Pull::Up);
    ccu::bus_enable(ccu::R_TWI0_BGR).unwrap_or_else(|error| crate::clock_failure(error));
    let twi: &'static Twi = unsafe {
        let t = &mut *addr_of_mut!(TWI);
        t.replace(Twi::new(R_TWI0_BASE));
        t.as_ref().unwrap()
    };
    let pmic = axp::Axp::new(twi);
    match pmic.init() {
        Ok(()) => {
            pmic.enable_board_ldos();
            Some(pmic)
        }
        Err(e) => {
            println!("[bt0] PMIC init failed: {e:?}; continuing without it");
            None
        }
    }
}

fn read_layout(sd: &smhc::Smhc) -> layout::Table {
    let mut sector = [0u32; smhc::BLOCK_SIZE / 4];
    if let Err(e) = sd.read_blocks(layout::TABLE_OFFSET / smhc::BLOCK_SIZE as u32, &mut sector) {
        println!("[bt0] SD: reading layout table failed: {e:?}");
        halt();
    }
    // The u32 sector buffer need not satisfy Table's u64 alignment.
    let bytes = unsafe {
        core::slice::from_raw_parts(sector.as_ptr().cast::<u8>(), smhc::BLOCK_SIZE)
    };
    match layout::decode(bytes) {
        Ok(table) => table,
        Err(error) => {
            println!("[bt0] invalid boot layout frame: {error:?}");
            halt();
        }
    }
}

fn load_entry(sd: &smhc::Smhc, e: &layout::Entry, addr: usize, padded: usize) {
    const _: () = assert!(smhc::BLOCK_SIZE as u64 == layout::BLOCK_SIZE);
    // Called only by layout::load_entries after whole-table preflight.
    let t0 = soc::time_us();
    // SAFETY: whole-table preflight checked bounds, alignment and disjoint loads.
    let dst = unsafe { core::slice::from_raw_parts_mut(addr as *mut u32, padded / 4) };
    if let Err(err) = sd.read_blocks(e.offset / smhc::BLOCK_SIZE as u32, dst) {
        println!("[bt0] SD: read of kind {} failed: {err:?}", e.kind);
        halt();
    }
    let ms = (soc::time_us() - t0) / 1000;
    let data = unsafe { core::slice::from_raw_parts(addr as *const u8, e.size as usize) };
    if e.crc32 != 0 && crc32(data) != e.crc32 {
        println!("[bt0] kind {}: CRC mismatch", e.kind);
        halt();
    }
    println!(
        "[bt0] loaded kind {} ({} KiB) to 0x{addr:08x} in {ms} ms{}",
        e.kind,
        e.size / 1024,
        if e.crc32 != 0 { ", CRC ok" } else { "" }
    );
}

/// Warm-reset CPU0 into AArch64 EL3 at `entry`.
fn rmr_switch(entry: u32) -> ! {
    write32(soc::RVBAR_LO_CPU0, entry);
    write32(soc::RVBAR_HI_CPU0, 0);
    // Also select AArch64 in CPU0's reset configuration, as TF-A does for
    // each core it powers on; RMR.AA64 below requests the same.
    write32(soc::CPU0_CTRL, 1);
    unsafe {
        asm!(
            "dsb sy",
            "isb",
            // RMR: AA64 = 1, RR = 1 (request warm reset)
            "mrc p15, 0, r0, c12, c0, 2",
            "orr r0, r0, #3",
            "mcr p15, 0, r0, c12, c0, 2",
            "isb",
            "1:",
            "wfi",
            "b 1b",
            in("r0") 0u32,
            options(noreturn)
        )
    }
}

#[unsafe(no_mangle)]
extern "C" fn main() -> ! {
    soc::set_cntfrq();
    soc::stage(1);
    soc::diag(1, ccu::apb_uart_clk());
    soc::diag(2, soc::xo_ctrl0());
    recovery::arm_watchdog();
    match ccu::ref_pll_init() {
        Ok(found) => soc::diag(0, found),
        Err(error) => clock_failure(error),
    }
    soc::stage(2);
    let t_start = soc::ticks();
    let version = soc::soc_version();
    let hosc = soc::hosc_mhz();
    soc::board_common_init(version);
    init_uart();
    soc::stage(3);

    println!();
    println!("oreboot 🦀 bt0 | Radxa Cubie A7A | Allwinner A733");
    let sid = soc::chip_sid();
    println!(
        "[bt0] SoC rev {}, DCXO {} kHz (XO_CTRL0[15:14] = {}), SID {:08x}{:08x}{:08x}{:08x}",
        (b'A' + version as u8) as char,
        soc::hosc_khz(),
        soc::xo_freq_field(),
        sid[0],
        sid[1],
        sid[2],
        sid[3]
    );

    soc::stage(4);
    dram::report_previous_trace();
    if recovery::previous_boot_failed() {
        println!("[bt0] the previous boot never reached EDK2's boot manager");
        let mut sd = smhc::Smhc::new_smhc0(soc::SMHC0_BASE);
        match sd.init_card() {
            Ok(()) => recovery::boot_vendor_spl(&sd, &read_layout(&sd)),
            Err(e) => {
                println!("[bt0] recovery: SD init failed: {e:?}");
            }
        }
        println!("[bt0] recovery not possible; trying oreboot again");
    }
    recovery::begin_attempt();

    if let Err(error) = ccu::init() {
        clock_failure(error);
    }

    soc::stage(5);
    let pmic = init_pmic();
    ccu::dump();

    soc::stage(6);
    let set = dram::select();
    dram::dump_clocks("before init_DRAM");
    let Some((dram_mb, dram_mhz)) = dram::init(pmic, set) else {
        println!("[bt0] DRAM init failed");
        halt();
    };
    // Before anything else writes DRAM: keep the last boot's log, then move
    // the SRAM log into this boot's (cubie_a7a_common::fwlog).
    cubie_a7a_common::fwlog::start_dram();
    soc::stage(7);
    // 64-bit: 4 GiB does not fit bt0's 32-bit usize (it wrapped to 0).
    let dram_size = dram_mb as u64 * 1024 * 1024;
    println!("[bt0] DRAM: {dram_mb} MiB");
    // Only test what fits the 32-bit address space for now.
    let test_size = dram_size.min((0xc000_0000 - mem::DRAM_BASE) as u64) as usize;
    if let Err((a, want, got)) = dram::test(mem::DRAM_BASE, test_size) {
        println!("[bt0] DRAM test failed at {a:08x}: wrote {want:08x}, read {got:08x}");
        halt();
    }
    let dram_end = mem::DRAM_BASE + test_size;

    // Raise the CPU clocks only now: libdram trained at the boot clocks
    // (as SyterKit and the vendor SPL do).
    if let Some(p) = dram::pmic() {
        if let Err(error) = dvfs::boost(p) {
            clock_failure(error);
        }
        ccu::dump();
    }

    let mut sd = smhc::Smhc::new_smhc0(soc::SMHC0_BASE);
    if let Err(e) = sd.init_card() {
        println!("[bt0] SD init failed: {e:?}");
        halt();
    }
    soc::stage(8);
    let table = read_layout(&sd);

    let mut handoff = Handoff {
        magic: HANDOFF_MAGIC,
        version: HANDOFF_VERSION,
        dram_base: mem::DRAM_BASE as u64,
        dram_size,
        soc_version: version,
        hosc_mhz: hosc,
        bt0_done_ticks: 0,
        payload_addr: 0,
        payload_size: 0,
        dtb_addr: 0,
        dtb_size: 0,
        cpu_mhz: ccu::cpu_clocks(),
        dram_mhz,
    };
    let mut main_entry = None;
    if let Err(error) = layout::load_entries(&table, dram_end as u64, |e, addr, padded| {
        load_entry(&sd, e, addr, padded);
        match e.kind {
            layout::KIND_MAIN => main_entry = Some(addr as u32),
            layout::KIND_PAYLOAD => {
                handoff.payload_addr = e.load_addr;
                handoff.payload_size = e.size as u64;
            }
            layout::KIND_DTB => {
                handoff.dtb_addr = e.load_addr;
                handoff.dtb_size = e.size as u64;
            }
            _ => unreachable!(), // validated kinds; recovery entries skipped
        }
    }) {
        println!("[bt0] invalid boot layout: {error:?}");
        halt();
    }
    let Some(main_entry) = main_entry else {
        println!("[bt0] layout: no oreboot main image");
        halt();
    };

    handoff.bt0_done_ticks = soc::ticks();
    unsafe { core::ptr::write_volatile(mem::HANDOFF_ADDR as *mut Handoff, handoff) };
    println!(
        "[bt0] done in {} ms, warm reset into AArch64 at 0x{main_entry:08x}",
        (handoff.bt0_done_ticks - t_start) / (soc::TIMER_HZ / 1000)
    );
    Uart::attach(UART0_BASE).drain();
    // A full watchdog period for main and EDK2 to reach the boot manager.
    recovery::kick();
    soc::stage(9);
    rmr_switch(main_entry)
}

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    if let Some(l) = info.location() {
        println!("[bt0] panic at {}:{}", l.file(), l.line());
    }
    println!("[bt0]   {}", info.message());
    halt()
}
