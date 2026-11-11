// SPDX-License-Identifier: GPL-2.0-only
//! Software sweeps for when the PHY's hardware training passes but the
//! channel still fails the CPU test, or the training itself fails: each
//! knob is swept with a caller-supplied pass/fail test (normally
//! crate::ddr::verify::quick on the channel in the linear layout), the
//! longest passing run is found with dram-train's window search
//! ([`dram_train::window::longest_run`]) and its centre applied.
//!
//! The knobs are the ones the public TRM leaves open:
//! - the DFI latencies (DFITMG0 dfi_t_rddata_en / dfi_tphy_wrlat: "Refer
//!   to PHY specification", P2 p.85), as offsets from RL / WL;
//! - the PHY receiver Vref (ZQ_CON9 zq_ds*_vref, P2 p.377, 6-bit code);
//! - the DRAM DQ Vref (MR14) along the monotonic jedec-lpddr axis.

use super::{ctl, phy, DfiLatency};
use dram_train::window::{longest_run, Window};
use jedec_lpddr::lp4;
use regio::RegIo;

/// Longest run of a linear `n`-point map (n < 128): one always-failing
/// guard point past the end stops the circular search from wrapping.
pub fn linear_window(map: u128, n: u32) -> Window {
    longest_run(map & ((1u128 << n) - 1), n + 1)
}

/// Sweep `n` points: `set(io, i)` then `test(io)`; apply the centre of the
/// longest passing run and return (centre, window), or None if none pass
/// (the knob is left at `restore`).
pub fn sweep<H: RegIo + ?Sized>(
    io: &mut H,
    n: u32,
    restore: u32,
    mut set: impl FnMut(&mut H, u32),
    mut test: impl FnMut(&mut H) -> bool,
) -> Option<(u32, Window)> {
    let mut map = 0u128;
    for i in 0..n {
        set(io, i);
        io.kick_watchdog();
        if test(io) {
            map |= 1 << i;
        }
    }
    let w = linear_window(map, n);
    if w.is_empty() {
        set(io, restore);
        return None;
    }
    let c = w.start + w.len / 2;
    set(io, c);
    Some((c, w))
}

/// PHY receiver Vref sweep over the 64 codes.
pub fn rx_vref<H: RegIo + ?Sized>(
    io: &mut H,
    ch: usize,
    test: impl FnMut(&mut H) -> bool,
) -> Option<u8> {
    sweep(
        io,
        64,
        0x14,
        |io, i| phy::set_rx_vref(io, ch, i as u8),
        test,
    )
    .map(|(c, _)| c as u8)
}

/// DRAM DQ Vref (MR14) sweep on `ranks` ranks along the jedec-lpddr axis
/// (range 0 settings 0..50, then range 1 21..50), every second point.
pub fn dq_vref<H: RegIo + ?Sized>(
    io: &mut H,
    ch: usize,
    ranks: u8,
    restore_op: u8,
    test: impl FnMut(&mut H) -> bool,
) -> Option<u8> {
    let n = (lp4::VREF_AXIS_LEN as u32).div_ceil(2);
    let restore = lp4::vref_axis_index(restore_op).unwrap_or(0) as u32 / 2;
    let set = |io: &mut H, i: u32| {
        let op = lp4::vref_axis_op((2 * i) as usize);
        for r in 0..ranks {
            let _ = super::mr::write(io, ch, r, lp4::MR_VREF_DQ, op);
        }
    };
    sweep(io, n, restore, set, test).map(|(c, _)| lp4::vref_axis_op((2 * c) as usize))
}

/// DFI read-enable offset sweep (RL - 0..=7) for each write-latency offset
/// (WL - 0..=3); the write offset with the widest read window wins.
pub fn dfi<H: RegIo + ?Sized>(
    io: &mut H,
    ch: usize,
    rl: u32,
    wl: u32,
    tphy_wrdata: u32,
    mut test: impl FnMut(&mut H) -> bool,
) -> Option<DfiLatency> {
    let word = |rd: u32, wr: u32| {
        (7 << 24)
            | ((rl.saturating_sub(rd) & 0x7f) << 16)
            | ((tphy_wrdata & 0x3f) << 8)
            | (wl.saturating_sub(wr) & 0x3f)
    };
    let mut best: Option<(u32, u32, u32)> = None; // (len, wr, centre)
    for wr in 0..4 {
        let mut map = 0u128;
        for rd in 0..8 {
            if ctl::set_dfitmg0(io, ch, word(rd, wr)).is_ok() && test(io) {
                map |= 1 << rd;
            }
        }
        let w = linear_window(map, 8);
        if !w.is_empty() && best.is_none_or(|b| w.len > b.0) {
            best = Some((w.len, wr, w.start + w.len / 2));
        }
    }
    let (_, wr, rd) = best?;
    ctl::set_dfitmg0(io, ch, word(rd, wr)).ok()?;
    Some(DfiLatency {
        wrlat_sub: wr,
        tphy_wrdata,
        rddata_en_sub: rd,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn linear_does_not_wrap() {
        // Points 0-1 and 6-7 pass: circular would join them; linear must not.
        let w = linear_window(0b1100_0011, 8);
        assert_eq!(w.len, 2);
        let w = linear_window(0b0011_1100, 8);
        assert_eq!((w.start, w.len), (2, 4));
        assert!(linear_window(0, 8).is_empty());
    }
}
