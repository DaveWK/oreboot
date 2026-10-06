//! oreboot bt0 for the Radxa Cubie A7A (Allwinner A733, sun60iw2).
//!
//! Loaded by the mask ROM from the SD card (eGON image at 8 KiB) into SRAM
//! and run in AArch32. It brings up clocks, UART0, the PMIC and LPDDR5,
//! loads oreboot main and the payload from the SD card as described by the
//! boot layout table, then warm-resets CPU0 into AArch64 at oreboot main.
#![no_std]
#![no_main]
#![feature(c_variadic)]

#[macro_use]
extern crate log;

use core::arch::{asm, global_asm, naked_asm};
use core::panic::PanicInfo;
use core::ptr::addr_of_mut;

use cubie_a7a_common::{
    crc32, layout, mem,
    uart::{init_logger, Uart, UART0_BASE},
    Handoff, HANDOFF_MAGIC, HANDOFF_VERSION,
};
use util::mmio::write32;

mod axp;
mod ccu;
mod dram;
mod gpadc;
mod gpio;
mod smhc;
mod soc;
mod twi;

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
    println!("[bt0] EXCEPTION: {name}, lr {lr:08x}");
    println!("[bt0]   DFSR {dfsr:08x} DFAR {dfar:08x} IFSR {ifsr:08x} IFAR {ifar:08x}");
    halt()
}

fn halt() -> ! {
    println!("[bt0] halted; resetting in 10 s");
    soc::mdelay(10_000);
    soc::watchdog_reset()
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
    ccu::bus_enable(ccu::UART0_BGR);
    init_logger(Uart::init(UART0_BASE, UART_CLOCK_HZ, 115_200));
}

fn init_pmic() -> Option<axp::Axp<'static>> {
    static mut TWI: Option<twi::Twi> = None;
    // R_TWI0: PL0 = SCK, PL1 = SDA
    gpio::set_function(Port::L, 0, 2);
    gpio::set_function(Port::L, 1, 2);
    gpio::set_pull(Port::L, 0, Pull::Up);
    gpio::set_pull(Port::L, 1, Pull::Up);
    ccu::bus_enable(ccu::R_TWI0_BGR);
    let twi: &'static twi::Twi = unsafe {
        let t = &mut *addr_of_mut!(TWI);
        t.replace(twi::Twi::new(soc::R_TWI0_BASE));
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
    // SAFETY: Table is repr(C), plain integers, and fits in one sector.
    let table: layout::Table = unsafe { core::ptr::read(sector.as_ptr() as *const layout::Table) };
    let mut bytes = [0u8; layout::TABLE_SIZE];
    // SAFETY: the sector buffer is larger than the table.
    unsafe {
        core::ptr::copy_nonoverlapping(sector.as_ptr() as *const u8, bytes.as_mut_ptr(), bytes.len())
    };
    bytes[layout::CRC_FIELD_OFFSET..layout::CRC_FIELD_OFFSET + 4].fill(0);
    if table.magic != layout::MAGIC
        || table.version != layout::VERSION
        || table.count as usize > layout::MAX_ENTRIES
        || crc32(&bytes) != table.crc32
    {
        println!("[bt0] SD: no valid boot layout table at 0x{:x}", layout::TABLE_OFFSET);
        halt();
    }
    table
}

fn load_entry(sd: &smhc::Smhc, e: &layout::Entry, dram_end: usize) {
    let addr = e.load_addr as usize;
    let padded = (e.size as usize).next_multiple_of(smhc::BLOCK_SIZE);
    let ok = addr >= mem::DRAM_BASE
        && addr % 4 == 0
        && addr.checked_add(padded).is_some_and(|end| end <= dram_end)
        && e.offset as usize % smhc::BLOCK_SIZE == 0;
    if !ok {
        println!("[bt0] layout: bad entry {e:x?}");
        halt();
    }
    let t0 = soc::time_us();
    // SAFETY: range checked against DRAM above; DRAM is not otherwise used.
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
    let t_start = soc::ticks();
    let version = soc::soc_version();
    let hosc = soc::hosc_mhz();
    soc::board_common_init(version);
    init_uart();

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

    ccu::init();
    ccu::dump();

    let pmic = init_pmic();

    let dram_mb = dram::init(pmic, dram::select());
    if dram_mb == 0 {
        println!("[bt0] DRAM init failed");
        halt();
    }
    let dram_size = dram_mb as usize * 1024 * 1024;
    println!("[bt0] DRAM: {dram_mb} MiB");
    // Only test what fits the 32-bit address space for now.
    let test_size = dram_size.min(0xc000_0000 - mem::DRAM_BASE);
    if let Err((a, want, got)) = dram::test(mem::DRAM_BASE, test_size) {
        println!("[bt0] DRAM test failed at {a:08x}: wrote {want:08x}, read {got:08x}");
        halt();
    }
    let dram_end = mem::DRAM_BASE + test_size;

    let mut sd = smhc::Smhc::new_smhc0(soc::SMHC0_BASE);
    if let Err(e) = sd.init_card() {
        println!("[bt0] SD init failed: {e:?}");
        halt();
    }
    let table = read_layout(&sd);

    let mut handoff = Handoff {
        magic: HANDOFF_MAGIC,
        version: HANDOFF_VERSION,
        dram_base: mem::DRAM_BASE as u64,
        dram_size: dram_size as u64,
        soc_version: version,
        hosc_mhz: hosc,
        bt0_done_ticks: 0,
        payload_addr: 0,
        payload_size: 0,
        dtb_addr: 0,
        dtb_size: 0,
    };
    let mut main_entry = None;
    for e in &table.entries[..table.count as usize] {
        load_entry(&sd, e, dram_end);
        match e.kind {
            layout::KIND_MAIN => main_entry = Some(e.load_addr as u32),
            layout::KIND_PAYLOAD => {
                handoff.payload_addr = e.load_addr;
                handoff.payload_size = e.size as u64;
            }
            layout::KIND_DTB => {
                handoff.dtb_addr = e.load_addr;
                handoff.dtb_size = e.size as u64;
            }
            k => {
                println!("[bt0] layout: ignoring unknown kind {k}");
            }
        }
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
