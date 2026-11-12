// SPDX-License-Identifier: GPL-2.0-only
//! SMCCC TRNG firmware interface (Arm DEN0098 1.0) on the A733 crypto
//! engine's TRNG.
//!
//! The Cortex-A55/A76 cores have no FEAT_RNG (RNDR), so without this service
//! EDK2's RngDxe has no source, unloads itself, and takes the network stack
//! (DxeNetLib's depex on the RNG protocol) and the Linux EFI stub's KASLR
//! seed with it. With it, EDK2's ArmPkg ArmTrngLib publishes EFI_RNG_PROTOCOL
//! (RAW) and Linux finds the same service through `smccc_probe_trng()`
//! (early `arch_get_random_seed_longs()` and the `smccc_trng` hwrng).
//!
//! Hardware, from the vendor sun60iw2p1.dtsi and ccu-sun60iw2.c: the CE has
//! a non-secure task window at 0x0460_3000 and a secure one at 0x0460_3800
//! (0xa0 bytes each); its clocks are CE_CLK (CCU +0x0ac0: gate 31, source
//! [26:24] = 1 for PERI0_400M, divider M [4:0]), CE_BGR (+0x0ac4: bus gate 0,
//! sys gate 1, resets 16 and 17) and the CE MBUS gate (+0x05e4 bit 2). The
//! vendor kernel left them at CE_CLK 0x8100_0000, CE_BGR 0x0003_0003 and the
//! MBUS gate on. The TRNG conditions its output with SHA-256 in hardware and
//! produces whole 8-word (32-byte) blocks; health tests are in the block.
//!
//! The A733 CE is the generation the vendor's GPL BSP driver calls "v5"
//! (radxa/allwinner-bsp drivers/ce/v5, shared with sun55iw3/A523), not the
//! H6/H616 one Linux's sun8i-ce drives (which has no A523/A733 variant).
//! Hash and random-number tasks have their own descriptor ([`Task`], the
//! BSP's `ce_new_task_desc`) and their own queue:
//!
//! - `comm_ctl` = flow | LPKG (bit 12) | IE (bit 16); `main_cmd` = RNG
//!   method << 8 | hash method, TRNG 2 with SHA-256 3;
//! - 40-bit byte addresses, 5 bytes little-endian; each scatter entry pairs
//!   a source and a destination, lengths in bytes (one 32-byte block here);
//! - TSK0 (+0x00) takes the task address [31:0] (TSK1, +0x04, [39:32]) and
//!   TLR bit 10 starts the hash/RBG queue (bit 0 is the symmetric queue);
//! - ISR has two pending bits per flow (2n and 2n+1, written back to clear),
//!   ESR eight error bits per flow (0 unsupported, 1 length, 2 key SRAM,
//!   5 address), TSR [7:0] is non-zero while a flow is busy;
//! - bit 1 of +0xd0 enables the TRNG's second entropy source, set after
//!   every CE reset ("should be enabled when ce is enabled").
//!
//! The first version wrote the sun8i-ce H6 task (method 0x1c in the common
//! word, TLR 0x1c01: symmetric queue bit 0 plus bits 10-12) and polled ISR
//! bit 0 only. The board answered ISR 0x2 (flow 0's second pending bit)
//! with ESR 0x31 (word addresses: unsupported, bit 4 and address error,
//! the task fetched from a quarter of its address) or, with byte
//! addresses, either ISR 0x2 with ESR 0x1 (unsupported) or a task that
//! completed without writing a usable block; TSR was 0 (idle) every time.
//!
//! NOT VERIFIED ON THE A733 (feature `trng` is off by default). A
//! cold-boot self-test turns the service on only if two blocks complete
//! without a CE error, were written (the output is preset with a pattern),
//! are not constant and differ; a failure prints the CE registers, the task
//! descriptor and the output words. Until then every TRNG call answers
//! NOT_SUPPORTED, as before, without the "unknown SMC" line. Feature
//! `trng-byte-addr` no longer changes anything (this task takes byte
//! addresses only) and is kept so existing build lines still work.
//!
//! Try once per hang: bt0's watchdog stays armed until EDK2 confirms the
//! boot, and a card without bt0's fallback stage (VENDOR_DRAM=yes) has
//! nothing else to run, so a CE access that stalls the bus here would reset
//! the board into the same stall on every boot. Before the first CE clock
//! or register access, [`init`] puts an attempt marker with the stage it is
//! in into RTC GP_DATA_REG7, which keeps its value across the watchdog
//! reset; it takes the marker out again when the self-test has finished,
//! passed or failed. If [`init`] finds its marker still there, the previous
//! boot stopped in here: it leaves the CE alone, so the service answers
//! NOT_SUPPORTED, and takes the marker out, so the boot after that tries
//! again.
//!
//! GP_DATA_REG0-6 all belong to bt0 (0-3 its diagnostics and progress,
//! rewritten on every boot; 4-5 the DRAM library trace of dramlib builds,
//! which bt0 clears before main runs; 6 the boot attempt), so the marker
//! shares REG7 with EDK2's A733DisplayDxe attempt marker (magic 0xD15A in
//! [31:16], stage and state in [15:0], never 0 in [15:0] while set). main
//! runs before the display driver on every boot and carries the display
//! marker's low half through in its own:
//!
//!   [31:20] 0xA7A (TRNG attempt), [19:16] TRNG stage 1-4,
//!   [15:0]  the display marker's [15:0] as main found it (0 for none)
//!
//! and puts the display marker back exactly when it takes its own out, so
//! the display driver sees what it would have seen without this service. A
//! value that is neither (nor 0) is left alone and the TRNG stays off for
//! that boot; the display driver clears it on its next run.
//!
//! EL3 runs with the MMU and D-cache off, so the descriptor and output
//! block (in EL3's own reserved RAM) need no cache maintenance; all memory
//! is Device-nGnRnE, so the 5-byte address fields are written a byte at a
//! time. Calls are serialised by the PSCI bakery lock; a contended read or
//! a stuck or repeated block answers NO_ENTROPY and the caller retries. A
//! CE error, timeout or unwritten block puts the CE back in reset with its
//! clocks gated and turns the service off for the rest of the boot
//! (NOT_SUPPORTED).

use core::arch::asm;
use core::mem::{offset_of, size_of};
use core::ptr::{addr_of, addr_of_mut, read_volatile, write_volatile};

use cubie_a7a_common::time::{ticks, udelay, TIMER_HZ};
use util::mmio::{read32, write32};

use crate::psci_lock;

/// DEN0098 function IDs (Linux include/linux/arm-smccc.h).
pub mod fid {
    pub const TRNG_VERSION: u32 = 0x8400_0050;
    pub const TRNG_FEATURES: u32 = 0x8400_0051;
    pub const TRNG_GET_UUID: u32 = 0x8400_0052;
    pub const TRNG_RND32: u32 = 0x8400_0053;
    pub const TRNG_RND64: u32 = 0xc400_0053;
}
use fid::*;

/// DEN0098 return codes.
const SUCCESS: i64 = 0;
const NOT_SUPPORTED: i64 = -1;
const INVALID_PARAMETERS: i64 = -2;
const NO_ENTROPY: i64 = -3;

/// Interface version 1.0.
const VERSION_1_0: i64 = 1 << 16;

/// Backend UUID 7860846d-ca14-4056-a9d4-88f3ed0df068, as w0..w3 in the
/// layout ArmPkg's ArmTrngLib (and TF-A) use.
const UUID: [u32; 4] = [0x7860_846d, 0x4056_ca14, 0xf388_d4a9, 0x68f0_0ded];

const CCU: usize = 0x0200_2000;
const MBUS_GATE_EN: usize = CCU + 0x05e4;
/// Ordinary R/W bit (see gmac.rs: the BSP "key" must not be ORed in).
const MBUS_CE: u32 = 1 << 2;
const CE_CLK: usize = CCU + 0x0ac0;
const CE_CLK_GATE: u32 = 1 << 31;
/// Gate, PERI0_400M, M = 1: the vendor kernel's 400 MHz.
const CE_CLK_400M: u32 = CE_CLK_GATE | 1 << 24;
const CE_BGR: usize = CCU + 0x0ac4;
const CE_BGR_RESETS: u32 = 1 << 17 | 1 << 16;
const CE_BGR_GATES: u32 = 1 << 1 | 1 << 0;

/// The secure task window: the non-secure one stays the OS's.
const CE: usize = 0x0460_3800;
/// Task address [31:0], in bytes.
const CE_TSK0: usize = CE;
/// Task address [39:32] (written only for a task above 4 GiB, as the BSP).
const CE_TSK1: usize = CE + 0x04;
const CE_ICR: usize = CE + 0x08;
const CE_ISR: usize = CE + 0x0c;
const CE_TLR: usize = CE + 0x10;
const CE_TSR: usize = CE + 0x14;
const CE_ESR: usize = CE + 0x18;
/// Hash/RBG queue current source and destination addresses.
const CE_HCSA: usize = CE + 0x34;
const CE_HCDA: usize = CE + 0x38;
const CE_VER: usize = CE + 0x90;
/// Low-power/TRNG control, outside the 0xa0 bytes the DT gives either
/// window; written through the non-secure one, as the vendor kernel does.
const CE_LPC: usize = 0x0460_3000 + 0xd0;
/// The TRNG's second entropy source.
const LPC_DBL_ENT_SRC: u32 = 1 << 1;

const FLOW: u32 = 0;
/// TLR: start the hash/RBG queue.
const TLR_HASH_RBG: u32 = 1 << 10;
/// Both ISR pending bits of [`FLOW`].
const ISR_FLOW: u32 = 0x3 << (2 * FLOW);
/// `comm_ctl`: channel [`FLOW`], last package, interrupt enable.
const COMM_CTL: u32 = FLOW | 1 << 12 | 1 << 16;
/// `main_cmd`: RNG method TRNG (2) << 8, hash method SHA-256 (3).
const MAIN_CMD_TRNG: u32 = 2 << 8 | 3;
const BLOCK_WORDS: usize = 8;
const BLOCK_BYTES: u32 = (BLOCK_WORDS * 4) as u32;
const TASK_TIMEOUT_US: u64 = 10_000;
/// Preset in the output block before each task: still there afterwards
/// means the CE never wrote it.
const CANARY: u32 = 0xce5a_0000;

/// RTC GP_DATA_REG7, shared with EDK2's A733DisplayDxe (see the module
/// documentation for the layout).
const MARKER_REG: usize = 0x0709_011c;
const MARKER_MAGIC: u32 = 0xa7a0_0000;
const MARKER_MAGIC_MASK: u32 = 0xfff0_0000;
const DISPLAY_MAGIC: u32 = 0xd15a_0000;

/// Where [`init`] is, in the attempt marker.
#[derive(Clone, Copy)]
#[repr(u32)]
enum Stage {
    /// CE clocks, MBUS gate and resets (CCU writes only).
    Clocks = 1,
    /// Self-test block 0: the first CE register accesses (entropy source
    /// enable, then the task).
    Block0 = 2,
    /// Self-test block 1.
    Block1 = 3,
    /// Putting the CE back in reset after a failed self-test.
    PowerOff = 4,
}

/// What [`init`] found in [`MARKER_REG`].
enum Previous {
    /// No TRNG marker: the display marker's low half (0 for none).
    Clear(u32),
    /// The previous boot stopped in [`init`].
    Hung { stage: u32, display: u32 },
    /// Neither 0, a display marker nor a TRNG marker.
    Foreign(u32),
}

fn previous() -> Previous {
    let v = read32(MARKER_REG);
    if v == 0 {
        Previous::Clear(0)
    } else if v & 0xffff_0000 == DISPLAY_MAGIC {
        Previous::Clear(v & 0xffff)
    } else if v & MARKER_MAGIC_MASK == MARKER_MAGIC {
        Previous::Hung {
            stage: (v >> 16) & 0xf,
            display: v & 0xffff,
        }
    } else {
        Previous::Foreign(v)
    }
}

fn marker(display: u32, stage: Stage) -> u32 {
    MARKER_MAGIC | (stage as u32) << 16 | display
}

fn set_marker(display: u32, stage: Stage) {
    write32(MARKER_REG, marker(display, stage));
}

/// Take the TRNG marker out and put the display marker back as found.
fn clear_marker(display: u32) {
    write32(
        MARKER_REG,
        if display == 0 {
            0
        } else {
            DISPLAY_MAGIC | display
        },
    );
}

/// A scatter entry of the v5 descriptor: a source and a destination.
// Hardware layout: most fields are only ever zero.
#[allow(dead_code)]
#[repr(C)]
#[derive(Clone, Copy)]
struct Sg {
    src_addr: [u8; 5],
    dst_addr: [u8; 5],
    pad: [u8; 2],
    /// Bytes.
    src_len: u32,
    /// Bytes.
    dst_len: u32,
}

/// The v5 hash/RBG task descriptor (the BSP's `ce_new_task_desc` without
/// its software-only tail), 208 bytes.
#[allow(dead_code)]
#[repr(C)]
struct Task {
    comm_ctl: u32,
    main_cmd: u32,
    data_len: [u8; 5],
    key_addr: [u8; 5],
    iv_addr: [u8; 5],
    pad: u8,
    sg: [Sg; 8],
    next_sg_addr: [u8; 5],
    next_task_addr: [u8; 5],
    pad2: [u8; 2],
    reserved: [u32; 3],
}

const _: () = assert!(size_of::<Sg>() == 20);
const _: () = assert!(offset_of!(Task, sg) == 24);
const _: () = assert!(offset_of!(Task, next_sg_addr) == 184);
const _: () = assert!(offset_of!(Task, reserved) == 196);
const _: () = assert!(size_of::<Task>() == 208);

/// The task, followed by the output block.
#[repr(C, align(64))]
struct Dma {
    task: Task,
    out: [u32; BLOCK_WORDS],
}

const TASK_WORDS: usize = size_of::<Task>() / 4;
const DMA_WORDS: usize = size_of::<Dma>() / 4;

// SAFETY: all-zero is a valid `Dma` (integers and byte arrays only).
static mut DMA: Dma = unsafe { core::mem::zeroed() };

/// Set on the boot CPU before the payload runs once the self-test passed;
/// cleared (under the lock) if the CE fails later. Never set again.
static mut ENABLED: u32 = 0;
/// The last block handed out, for the repetition test (under the lock).
static mut LAST: [u32; BLOCK_WORDS] = [0; BLOCK_WORDS];
/// The CE MBUS gate bit as [`init`] found it, put back by [`ce_off`].
static mut MBUS_CE_FOUND: u32 = 0;

/// The CE registers when the last task finished or timed out, before ISR
/// was cleared (under the lock, or single-core cold boot).
#[derive(Clone, Copy)]
struct Regs {
    tsk0: u32,
    tsk1: u32,
    icr: u32,
    isr: u32,
    tlr: u32,
    tsr: u32,
    esr: u32,
    hcsa: u32,
    hcda: u32,
    ver: u32,
    lpc: u32,
}

static mut SNAP: Regs = Regs {
    tsk0: 0,
    tsk1: 0,
    icr: 0,
    isr: 0,
    tlr: 0,
    tsr: 0,
    esr: 0,
    hcsa: 0,
    hcda: 0,
    ver: 0,
    lpc: 0,
};

fn regs() -> Regs {
    Regs {
        tsk0: read32(CE_TSK0),
        tsk1: read32(CE_TSK1),
        icr: read32(CE_ICR),
        isr: read32(CE_ISR),
        tlr: read32(CE_TLR),
        tsr: read32(CE_TSR),
        esr: read32(CE_ESR),
        hcsa: read32(CE_HCSA),
        hcda: read32(CE_HCDA),
        ver: read32(CE_VER),
        lpc: read32(CE_LPC),
    }
}

// Read through Debug in the log lines only.
#[allow(dead_code)]
#[derive(Clone, Copy, Debug)]
enum Error {
    /// No ISR pending bit for the flow within [`TASK_TIMEOUT_US`].
    Timeout,
    /// The flow's ESR byte.
    Ce(u32),
    /// The CE reported completion but the output still holds [`CANARY`].
    Unwritten,
    Repeated,
    Stuck,
    /// A DMA buffer the CE cannot address.
    Address(usize),
}

impl Error {
    /// The CE itself failed (as opposed to a block the checks rejected):
    /// the service goes off.
    fn is_fatal(self) -> bool {
        matches!(
            self,
            Error::Timeout | Error::Ce(_) | Error::Unwritten | Error::Address(_)
        )
    }
}

fn dsb() {
    unsafe { asm!("dsb sy", options(nostack)) };
}

fn enabled() -> bool {
    unsafe { read_volatile(addr_of!(ENABLED)) != 0 }
}

/// A CE bus address: 40 bits, in bytes.
fn ce_addr(p: usize) -> Result<u64, Error> {
    if p >> 40 != 0 {
        return Err(Error::Address(p));
    }
    Ok(p as u64)
}

/// A 5-byte little-endian descriptor address, a byte at a time (Device
/// memory: no unaligned accesses).
///
/// # Safety
/// `f` points into [`DMA`], which the caller owns.
unsafe fn put_addr(f: *mut [u8; 5], a: u64) {
    let b = f as *mut u8;
    for i in 0..5 {
        write_volatile(b.add(i), (a >> (8 * i)) as u8);
    }
}

/// Clocks and resets on: gate the clock in, then the bus and MBUS gates
/// with the resets released.
fn ce_on() {
    // SAFETY: boot CPU, before the payload and the other cores run.
    unsafe { write_volatile(addr_of_mut!(MBUS_CE_FOUND), read32(MBUS_GATE_EN) & MBUS_CE) };
    write32(MBUS_GATE_EN, read32(MBUS_GATE_EN) | MBUS_CE);
    write32(CE_CLK, CE_CLK_400M);
    write32(CE_BGR, 0);
    udelay(10);
    write32(CE_BGR, CE_BGR_RESETS);
    write32(CE_BGR, CE_BGR_RESETS | CE_BGR_GATES);
    udelay(10);
}

/// The service off and the CE back as before [`init`]: bus gates off,
/// resets asserted (which also stops a task still in flight), module clock
/// gated, MBUS gate as found; the descriptor, output and last block wiped.
fn ce_off() {
    // SAFETY: on the boot CPU before the payload, or under the lock.
    unsafe { write_volatile(addr_of_mut!(ENABLED), 0) };
    write32(CE_BGR, read32(CE_BGR) & !CE_BGR_GATES);
    write32(CE_BGR, 0);
    write32(CE_CLK, read32(CE_CLK) & !CE_CLK_GATE);
    if unsafe { read_volatile(addr_of!(MBUS_CE_FOUND)) } == 0 {
        write32(MBUS_GATE_EN, read32(MBUS_GATE_EN) & !MBUS_CE);
    }
    dsb();
    // SAFETY: the CE is in reset, nothing else uses these.
    unsafe {
        let words = addr_of_mut!(DMA) as *mut u32;
        for i in 0..DMA_WORDS {
            write_volatile(words.add(i), 0);
        }
        write_volatile(addr_of_mut!(LAST), [0; BLOCK_WORDS]);
    }
}

/// Print the register snapshot of the last task, its descriptor and the
/// output words (only ever a rejected block: one that passed is wiped).
fn dump() {
    // SAFETY: under the lock or on the boot CPU; the CE has stopped or is
    // about to be reset, and nothing in here is a block handed out.
    let r = unsafe { read_volatile(addr_of!(SNAP)) };
    println!(
        "[main] TRNG CE: TSK0 {:#010x} TSK1 {:#x} ICR {:#x} ISR {:#x} TLR {:#x} TSR {:#x} \
         ESR {:#x} HCSA {:#010x} HCDA {:#010x} VER {:#x} LPC {:#x}",
        r.tsk0, r.tsk1, r.icr, r.isr, r.tlr, r.tsr, r.esr, r.hcsa, r.hcda, r.ver, r.lpc
    );
    let e = (r.esr >> (8 * FLOW)) & 0xff;
    if e != 0 {
        const NAMES: [&str; 8] = [
            "unsupported",
            "length",
            "key-sram",
            "bit3",
            "bit4",
            "address",
            "key-ladder",
            "bit7",
        ];
        print!("[main] TRNG CE: flow {FLOW} error {e:#x}:");
        for (i, n) in NAMES.iter().enumerate() {
            if e & 1 << i != 0 {
                print!(" {n}");
            }
        }
        println!();
    }
    let d = addr_of!(DMA) as *const u32;
    println!(
        "[main] TRNG task @ {:#x} ({} words), output @ {:#x}:",
        d as usize,
        TASK_WORDS,
        unsafe { addr_of!((*addr_of!(DMA)).out) } as usize
    );
    const WORDS: usize = TASK_WORDS + BLOCK_WORDS;
    for row in 0..WORDS.div_ceil(8) {
        print!("[main]   +{:03x}:", row * 32);
        for i in row * 8..(row * 8 + 8).min(WORDS) {
            // SAFETY: inside DMA, aligned word reads.
            print!(" {:08x}", unsafe { read_volatile(d.add(i)) });
        }
        println!();
    }
}

/// Run one TRNG task into `DMA.out`; returns the block, which stays in
/// `DMA.out` until the caller wipes it. Every wait is bounded by
/// [`TASK_TIMEOUT_US`]; [`SNAP`] holds the registers at the end.
fn read_block() -> Result<[u32; BLOCK_WORDS], Error> {
    let d = addr_of_mut!(DMA);
    let task = ce_addr(d as usize)?;
    let out = ce_addr(unsafe { addr_of!((*d).out) } as usize)?;
    // SAFETY: EL3-owned, serialised by the caller; uncached (MMU off).
    unsafe {
        let words = d as *mut u32;
        for i in 0..DMA_WORDS {
            write_volatile(words.add(i), 0);
        }
        for i in 0..BLOCK_WORDS {
            write_volatile(addr_of_mut!((*d).out[i]), CANARY | i as u32);
        }
        let t = addr_of_mut!((*d).task);
        write_volatile(addr_of_mut!((*t).comm_ctl), COMM_CTL);
        write_volatile(addr_of_mut!((*t).main_cmd), MAIN_CMD_TRNG);
        put_addr(addr_of_mut!((*t).sg[0].dst_addr), out);
        write_volatile(addr_of_mut!((*t).sg[0].dst_len), BLOCK_BYTES);
    }
    dsb();
    write32(CE_ISR, ISR_FLOW);
    write32(CE_ICR, read32(CE_ICR) | 1 << FLOW);
    write32(CE_TSK0, task as u32);
    if task >> 32 != 0 {
        write32(CE_TSK1, (task >> 32) as u32);
    }
    dsb();
    write32(CE_TLR, TLR_HASH_RBG);

    let t0 = ticks();
    let limit = TASK_TIMEOUT_US * (TIMER_HZ / 1_000_000);
    let done = loop {
        if read32(CE_ISR) & ISR_FLOW != 0 {
            break true;
        }
        if ticks().wrapping_sub(t0) > limit {
            break false;
        }
    };
    let r = regs();
    // SAFETY: under the caller's lock (or single-core cold boot).
    unsafe { write_volatile(addr_of_mut!(SNAP), r) };
    write32(CE_ISR, ISR_FLOW);
    if !done {
        return Err(Error::Timeout);
    }
    let esr = (r.esr >> (8 * FLOW)) & 0xff;
    if esr != 0 {
        return Err(Error::Ce(esr));
    }
    dsb();
    let mut b = [0u32; BLOCK_WORDS];
    // SAFETY: as above; the CE has finished with the block.
    unsafe {
        for (i, w) in b.iter_mut().enumerate() {
            *w = read_volatile(addr_of!((*d).out[i]));
        }
    }
    if b.iter().enumerate().all(|(i, &w)| w == CANARY | i as u32) {
        return Err(Error::Unwritten);
    }
    Ok(b)
}

/// Firmware-side checks on top of the CE's own health tests: an unwritten,
/// stuck (constant) block or a repeat of the previous block is never handed
/// out. A block that passes is wiped from `DMA.out`; a rejected one stays
/// there for [`dump`] until the next task or [`ce_off`].
fn checked_block() -> Result<[u32; BLOCK_WORDS], Error> {
    let b = read_block()?;
    if b.iter().all(|&w| w == b[0]) {
        return Err(Error::Stuck);
    }
    // SAFETY: under the caller's lock (or single-core cold boot).
    unsafe {
        let last = addr_of_mut!(LAST);
        if (*last) == b {
            return Err(Error::Repeated);
        }
        write_volatile(last, b);
        let out = addr_of_mut!(DMA.out) as *mut u32;
        for i in 0..BLOCK_WORDS {
            write_volatile(out.add(i), 0);
        }
    }
    Ok(b)
}

/// The clocks (marker at [`Stage::Clocks`] already), then two blocks, each
/// stage in the marker before its first access. On any failure the CE goes
/// back to reset ([`Stage::PowerOff`]).
fn self_test(display: u32) -> bool {
    ce_on();

    for (n, stage) in [Stage::Block0, Stage::Block1].into_iter().enumerate() {
        set_marker(display, stage);
        if n == 0 {
            // The BSP sets it after every CE reset.
            write32(CE_LPC, read32(CE_LPC) | LPC_DBL_ENT_SRC);
        }
        match checked_block() {
            // The self-test blocks are discarded, so their first words may
            // be shown for the board check.
            Ok(b) => {
                println!(
                    "[main] TRNG self-test {n} (CE v5 hash/RBG task): {:08x} {:08x} ...",
                    b[0], b[1]
                );
            }
            Err(e) => {
                println!(
                    "[main] TRNG self-test {n} (CE v5 hash/RBG task) failed: {e:?}; \
                     CE back in reset, SMCCC TRNG stays off"
                );
                dump();
                set_marker(display, Stage::PowerOff);
                ce_off();
                return false;
            }
        }
    }
    true
}

/// Clocks, reset and the self-test, on the boot CPU before the payload,
/// guarded by the attempt marker (module documentation).
pub fn init() {
    if !cfg!(feature = "trng") {
        return;
    }
    let display = match previous() {
        Previous::Clear(display) => display,
        Previous::Hung { stage, display } => {
            println!(
                "[main] TRNG: previous boot stopped in TRNG init at stage {stage}; disabled this boot"
            );
            // One retry per hang: the next boot tries again.
            clear_marker(display);
            return;
        }
        Previous::Foreign(v) => {
            println!(
                "[main] TRNG: RTC GP_DATA_REG7 holds {v:#010x}, not a display or TRNG \
                 attempt marker; left alone, TRNG disabled this boot"
            );
            return;
        }
    };
    // Without a marker that holds, a hang would repeat on every boot.
    set_marker(display, Stage::Clocks);
    if read32(MARKER_REG) != marker(display, Stage::Clocks) {
        println!(
            "[main] TRNG: attempt marker in RTC GP_DATA_REG7 does not hold ({:#010x}); \
             disabled this boot",
            read32(MARKER_REG)
        );
        clear_marker(display);
        return;
    }

    let ok = self_test(display);
    clear_marker(display);
    if ok {
        // SAFETY: boot CPU, before the payload and the other cores run.
        unsafe { write_volatile(addr_of_mut!(ENABLED), 1) };
        let ver = unsafe { read_volatile(addr_of!(SNAP.ver)) };
        println!("[main] SMCCC TRNG 1.0 on the CE TRNG (CE VER {ver:#x})");
    }
}

fn features(f: u32) -> i64 {
    match f {
        TRNG_VERSION | TRNG_FEATURES | TRNG_GET_UUID | TRNG_RND32 | TRNG_RND64 => SUCCESS,
        _ => NOT_SUPPORTED,
    }
}

/// TRNG_RND32/64: `n` bits, least significant in x3, unused bits zero.
fn rnd(x: &mut [u64], width: u32) -> i64 {
    let n = if width == 32 {
        x[1] & 0xffff_ffff
    } else {
        x[1]
    };
    x[1] = 0;
    x[2] = 0;
    x[3] = 0;
    if n == 0 || n > 3 * width as u64 {
        return INVALID_PARAMETERS;
    }
    let Ok(_guard) = psci_lock::acquire() else {
        return NO_ENTROPY;
    };
    // Another core may have turned the service off while this one waited.
    if !enabled() {
        return NOT_SUPPORTED;
    }
    let b = match checked_block() {
        Ok(b) => b,
        Err(e) if e.is_fatal() => {
            println!("[main] TRNG read failed: {e:?}; CE back in reset, SMCCC TRNG off");
            dump();
            ce_off();
            return NOT_SUPPORTED;
        }
        Err(e) => {
            println!("[main] TRNG read failed: {e:?}");
            return NO_ENTROPY;
        }
    };
    let mut v = [0u64; 3];
    for (i, r) in v.iter_mut().enumerate() {
        *r = if width == 32 {
            b[i] as u64
        } else {
            b[2 * i] as u64 | (b[2 * i + 1] as u64) << 32
        };
        let lo = i as u64 * width as u64;
        if n <= lo {
            *r = 0;
        } else if n - lo < width as u64 {
            *r &= (1u64 << (n - lo)) - 1;
        }
    }
    x[3] = v[0];
    x[2] = v[1];
    x[1] = v[2];
    SUCCESS
}

/// Handle a TRNG call; false for any other function ID. `x` is x0..x3.
pub fn handle_smc(x: &mut [u64]) -> bool {
    let fid = x[0] as u32;
    if !matches!(
        fid,
        TRNG_VERSION | TRNG_FEATURES | TRNG_GET_UUID | TRNG_RND32 | TRNG_RND64
    ) {
        return false;
    }
    let ret = if !enabled() {
        NOT_SUPPORTED
    } else {
        match fid {
            TRNG_VERSION => VERSION_1_0,
            TRNG_FEATURES => features(x[1] as u32),
            TRNG_GET_UUID => {
                x[1] = UUID[1] as u64;
                x[2] = UUID[2] as u64;
                x[3] = UUID[3] as u64;
                UUID[0] as i64
            }
            TRNG_RND32 => rnd(x, 32),
            _ => rnd(x, 64),
        }
    };
    x[0] = ret as u64;
    true
}
