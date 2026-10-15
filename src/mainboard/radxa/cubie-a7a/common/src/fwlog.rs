//! Firmware console log in RAM, so a boot can be read back from Linux when
//! no UART is attached (e.g. a HAT covers the header).
//!
//! Everything bt0, main and EDK2 print to UART0 is also appended to a ring
//! buffer in the part of DRAM main keeps for itself (never reported to the
//! payload or the OS as memory). bt0 collects its output in SRAM until DRAM
//! is trained, then moves it over. Before starting a new log, bt0 copies the
//! previous boot's log, if it is still intact, to a second area: DRAM keeps
//! its contents across a watchdog or warm reset often enough to be useful,
//! but nothing guarantees it, so that copy is best effort.
//!
//! The area format and operations are the shared `bootlog::ring` (format
//! [`ring::A7A`]); this module keeps the A7A addresses and the sink
//! selection. Layout of each area (little endian; EDK2's
//! OrebootLogSerialPortLib and the Linux reader mirror it; bootlog README):
//!
//! | Offset | Size | Field |
//! |---|---|---|
//! | 0  | 8 | magic `"A7AFWLOG"` |
//! | 8  | 4 | version (1) |
//! | 12 | 4 | data size in bytes (area size − 64) |
//! | 16 | 8 | bytes written so far; the ring position is `written % size` |
//! | 24 | 4 | boot sequence number, +1 per boot while the log survives |
//! | 28 | 4 | flags, 0 |
//! | 32 | 32 | reserved, 0 |
//! | 64 | size | ring data |

use bootlog::{ring, Area};
use core::ptr::{read_volatile, write_volatile};

pub const MAGIC: [u8; 8] = *b"A7AFWLOG";
/// SMC32 SiP call: returns [`CUR_BASE`] from oreboot main. A payload asks
/// before writing to the log, so it never writes there after a vendor
/// chainload (whose BL31 answers NOT_SUPPORTED) even if DRAM still holds a
/// valid-looking header from an earlier oreboot boot.
pub const SIP_FWLOG_BASE: u32 = 0x8200_0002;
pub const VERSION: u32 = ring::VERSION;
pub const HEADER_SIZE: usize = ring::HEADER_SIZE;

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

const _: () = assert!(MAGIC[0] == ring::A7A.magic[0] && DATA_SIZE == ring::A7A.data_size);

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

/// One log area in memory. All accesses are byte or naturally aligned word
/// accesses: main runs with the MMU off, where unaligned accesses fault.
struct Mem(usize);

impl Area for Mem {
    fn read8(&self, off: usize) -> u8 {
        unsafe { read_volatile((self.0 + off) as *const u8) }
    }
    fn write8(&mut self, off: usize, v: u8) {
        unsafe { write_volatile((self.0 + off) as *mut u8, v) }
    }
    fn read32(&self, off: usize) -> u32 {
        unsafe { read_volatile((self.0 + off) as *const u32) }
    }
    fn write32(&mut self, off: usize, v: u32) {
        unsafe { write_volatile((self.0 + off) as *mut u32, v) }
    }
}

/// Append one byte to the current log. Called by the UART writer for every
/// byte it sends.
pub fn putc(c: u8) {
    // SAFETY: SINK is only changed by the boot CPU during early setup.
    match unsafe { SINK } {
        Sink::None => {}
        Sink::Early => ring::early_putc(&mut Mem(EARLY_BASE), EARLY_SIZE, c),
        Sink::Ring => ring::putc(&mut Mem(CUR_BASE), &ring::A7A, c),
    }
}

/// bt0, first thing: collect output in SRAM until DRAM works.
pub fn start_early() {
    Mem(EARLY_BASE).write32(0, 0);
    unsafe { SINK = Sink::Early };
}

/// bt0, once DRAM is trained: keep the previous boot's log if it survived,
/// start this boot's log and move the early output into it.
pub fn start_dram() {
    let mut cur = Mem(CUR_BASE);
    ring::rotate(&mut cur, &mut Mem(PREV_BASE), &ring::A7A);
    if let Sink::Early = unsafe { SINK } {
        ring::drain_early(&Mem(EARLY_BASE), EARLY_SIZE, &mut cur, &ring::A7A);
    }
    unsafe { SINK = Sink::Ring };
}

/// main: append to the log bt0 started (or start one if bt0 did not).
pub fn attach() {
    let mut cur = Mem(CUR_BASE);
    if !ring::header_valid(&cur, &ring::A7A) {
        ring::init_header(&mut cur, &ring::A7A, 0);
    }
    unsafe { SINK = Sink::Ring };
}
