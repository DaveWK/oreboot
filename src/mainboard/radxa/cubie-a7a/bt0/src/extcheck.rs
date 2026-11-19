//! The SRAM side of the fastboot DRAM extension (`ext.rs`): where it is,
//! the load check entry 4 runs, and the gate every use passes. This module
//! must stay in SRAM (build.rs links only `ext`, `ehci`, `smhc` and the
//! crate's dump modules into `.dram`): it runs before the extension is
//! known to be there.
use core::sync::atomic::{AtomicBool, Ordering};

/// Where the extension is linked and loaded (DRAM, above the fastboot
/// download buffer and the EHCI schedule area at 0x58000000).
pub const BASE: usize = 0x5900_0000;
pub const MAX_LEN: usize = 0x10_0000;
const MAGIC: u32 = u32::from_le_bytes(*b"A7AX");

/// The extension's first word (linked at the start of `.dram`).
#[used]
#[unsafe(link_section = ".dram.head")]
static HEAD: u32 = MAGIC;

unsafe extern "C" {
    static _dram_start: u8;
    static _dram_end: u8;
}

static READY: AtomicBool = AtomicBool::new(false);

/// The linked length of the extension.
pub fn len() -> usize {
    core::ptr::addr_of!(_dram_end) as usize - core::ptr::addr_of!(_dram_start) as usize
}

/// Check what the host loaded: the length and CRC-32 it wrote to the
/// mailbox against the linked length and the bytes at [`BASE`], and the
/// magic. Runs in SRAM (no extension code is called before it passes).
pub fn check(host_len: u32, host_crc: u32) -> Result<usize, &'static str> {
    READY.store(false, Ordering::Relaxed);
    let n = len();
    if n == 0 || n > MAX_LEN || host_len as usize != n {
        return Err("DRAM extension not loaded (or from another build)");
    }
    // SAFETY: DRAM is up and tested; the extension's range.
    let bytes = unsafe { core::slice::from_raw_parts(BASE as *const u8, n) };
    if u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]) != MAGIC {
        return Err("DRAM extension: bad magic");
    }
    if cubie_a7a_common::crc32(bytes) != host_crc {
        return Err("DRAM extension: CRC mismatch");
    }
    READY.store(true, Ordering::Relaxed);
    Ok(n)
}

pub fn ready() -> Result<(), &'static str> {
    if READY.load(Ordering::Relaxed) {
        Ok(())
    } else {
        Err("DRAM extension not loaded (fastboot-boot.sh load)")
    }
}
