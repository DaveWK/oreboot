// SPDX-License-Identifier: GPL-2.0-only
// Copy of ore-edk-boot-radxa-a7a oreboot 4904019
// src/mainboard/radxa/cubie-a7a/common/src/fwlog.rs, the oracle for the
// equivalence tests. Edits (mechanical only): header_valid, init_header and
// ring_putc are `pub` so the test can call them; the module docs are plain
// comments. `crate::mem` is a stub in tests/equivalence.rs.
#![allow(dead_code, static_mut_refs, clippy::all)]
// Firmware console log in RAM, so a boot can be read back from Linux when
// no UART is attached (e.g. a HAT covers the header).
//
// Everything bt0, main and EDK2 print to UART0 is also appended to a ring
// buffer in the part of DRAM main keeps for itself (never reported to the
// payload or the OS as memory). bt0 collects its output in SRAM until DRAM
// is trained, then moves it over. Before starting a new log, bt0 copies the
// previous boot's log, if it is still intact, to a second area: DRAM keeps
// its contents across a watchdog or warm reset often enough to be useful,
// but nothing guarantees it, so that copy is best effort.
//
// Layout of each area (little endian; EDK2's OrebootLogSerialPortLib and
// the Linux reader `a7a-fwlog` mirror it; docs/fwlog.md):
//
// | Offset | Size | Field |
// |---|---|---|
// | 0  | 8 | magic `"A7AFWLOG"` |
// | 8  | 4 | version (1) |
// | 12 | 4 | data size in bytes (area size − 64) |
// | 16 | 8 | bytes written so far; the ring position is `written % size` |
// | 24 | 4 | boot sequence number, +1 per boot while the log survives |
// | 28 | 4 | flags, 0 |
// | 32 | 32 | reserved, 0 |
// | 64 | size | ring data |

use core::ptr::{read_volatile, write_volatile};

pub const MAGIC: [u8; 8] = *b"A7AFWLOG";
/// SMC32 SiP call: returns [`CUR_BASE`] from oreboot main. A payload asks
/// before writing to the log, so it never writes there after a vendor
/// chainload (whose BL31 answers NOT_SUPPORTED) even if DRAM still holds a
/// valid-looking header from an earlier oreboot boot.
pub const SIP_FWLOG_BASE: u32 = 0x8200_0002;
pub const VERSION: u32 = 1;
pub const HEADER_SIZE: usize = 64;

/// This boot's log.
pub const CUR_BASE: usize = 0x4014_0000;
/// The previous boot's log, if it survived the reset.
pub const PREV_BASE: usize = 0x4018_0000;
pub const AREA_SIZE: usize = 0x0004_0000;
pub const DATA_SIZE: usize = AREA_SIZE - HEADER_SIZE;

/// bt0's buffer until DRAM is up: the top 16 KiB of SHARED_SRAM, clear of
/// bt0's image, bss and stack and of the recovery staging area.
pub const EARLY_BASE: usize = 0x000f_0000;
pub const EARLY_SIZE: usize = 0x4000;

const OFF_VERSION: usize = 8;
const OFF_SIZE: usize = 12;
const OFF_WRITTEN: usize = 16;
const OFF_SEQ: usize = 24;

const _: () = assert!(CUR_BASE + AREA_SIZE <= PREV_BASE);
const _: () = assert!(PREV_BASE + AREA_SIZE <= crate::mem::HANDOFF_ADDR);
const _: () = assert!(crate::mem::DTB_ADDR + crate::mem::DTB_MAX_SIZE <= CUR_BASE);

/// Where log bytes go: none, a linear early buffer, or the DRAM ring.
#[derive(Clone, Copy)]
enum Sink {
    None,
    /// Linear buffer at EARLY_BASE; the length is kept in the first word.
    Early,
    Ring,
}

static mut SINK: Sink = Sink::None;

// All accesses are byte or naturally aligned word accesses: main runs with
// the MMU off, where unaligned accesses fault.
fn r32(a: usize) -> u32 {
    unsafe { read_volatile(a as *const u32) }
}
fn w32(a: usize, v: u32) {
    unsafe { write_volatile(a as *mut u32, v) }
}
fn r64(a: usize) -> u64 {
    r32(a) as u64 | (r32(a + 4) as u64) << 32
}
fn w64(a: usize, v: u64) {
    w32(a, v as u32);
    w32(a + 4, (v >> 32) as u32);
}
fn w8(a: usize, v: u8) {
    unsafe { write_volatile(a as *mut u8, v) }
}
fn r8(a: usize) -> u8 {
    unsafe { read_volatile(a as *const u8) }
}

pub fn header_valid(base: usize) -> bool {
    (0..8).all(|i| r8(base + i) == MAGIC[i])
        && r32(base + OFF_VERSION) == VERSION
        && r32(base + OFF_SIZE) as usize == DATA_SIZE
}

pub fn init_header(base: usize, seq: u32) {
    for i in 0..HEADER_SIZE / 4 {
        w32(base + 4 * i, 0);
    }
    w32(base + OFF_VERSION, VERSION);
    w32(base + OFF_SIZE, DATA_SIZE as u32);
    w64(base + OFF_WRITTEN, 0);
    w32(base + OFF_SEQ, seq);
    // Magic last, so a reader never sees a valid header over stale fields.
    for (i, &b) in MAGIC.iter().enumerate() {
        w8(base + i, b);
    }
}

pub fn ring_putc(base: usize, c: u8) {
    let n = r64(base + OFF_WRITTEN);
    w8(base + HEADER_SIZE + (n % DATA_SIZE as u64) as usize, c);
    w64(base + OFF_WRITTEN, n + 1);
}

/// Append one byte to the current log. Called by the UART writer for every
/// byte it sends.
pub fn putc(c: u8) {
    // SAFETY: SINK is only changed by the boot CPU during early setup.
    match unsafe { SINK } {
        Sink::None => {}
        Sink::Early => {
            let n = r32(EARLY_BASE) as usize;
            if 4 + n < EARLY_SIZE {
                w8(EARLY_BASE + 4 + n, c);
                w32(EARLY_BASE, (n + 1) as u32);
            }
        }
        Sink::Ring => ring_putc(CUR_BASE, c),
    }
}

/// bt0, first thing: collect output in SRAM until DRAM works.
pub fn start_early() {
    w32(EARLY_BASE, 0);
    unsafe { SINK = Sink::Early };
}

/// bt0, once DRAM is trained: keep the previous boot's log if it survived,
/// start this boot's log and move the early output into it.
pub fn start_dram() {
    let mut seq = 0;
    if header_valid(CUR_BASE) {
        seq = r32(CUR_BASE + OFF_SEQ).wrapping_add(1);
        for i in (0..AREA_SIZE).step_by(4) {
            w32(PREV_BASE + i, r32(CUR_BASE + i));
        }
    } else {
        // Nothing usable: make sure no stale copy passes for the last boot.
        w32(PREV_BASE, 0);
    }
    init_header(CUR_BASE, seq);
    let early = match unsafe { SINK } {
        Sink::Early => (r32(EARLY_BASE) as usize).min(EARLY_SIZE - 4),
        _ => 0,
    };
    for i in 0..early {
        ring_putc(CUR_BASE, r8(EARLY_BASE + 4 + i));
    }
    unsafe { SINK = Sink::Ring };
}

/// main: append to the log bt0 started (or start one if bt0 did not).
pub fn attach() {
    if !header_valid(CUR_BASE) {
        init_header(CUR_BASE, 0);
    }
    unsafe { SINK = Sink::Ring };
}
