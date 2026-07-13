// SPDX-License-Identifier: GPL-2.0-only
//! Device timing to controller clock cycles.

/// Nanoseconds to whole cycles of a `mhz` clock, rounded up (a JEDEC minimum
/// must never be shortened). `t_ns * mhz` must fit in u32 (it does for every
/// JEDEC time below 2 ms up to 2133 MHz; larger products wrap, as the board
/// code this replaces did).
pub const fn ns_to_ck(t_ns: u32, mhz: u32) -> u32 {
    (t_ns * mhz).div_ceil(1000)
}

/// [`ns_to_ck`] that reports overflow instead of wrapping.
pub const fn checked_ns_to_ck(t_ns: u32, mhz: u32) -> Option<u32> {
    match t_ns.checked_mul(mhz) {
        Some(p) => Some(p.div_ceil(1000)),
        None => None,
    }
}
