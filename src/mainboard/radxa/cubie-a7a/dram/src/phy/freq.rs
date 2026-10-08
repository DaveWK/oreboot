//! The operating-point-dependent PHY CSR values ([`super::prog::COMPUTED`]
//! entries of the PHY programs).
//!
//! Provenance: the PLL band tables and the timer formulas are those of the
//! vendor library's PhyInit (Synopsys LPDDR5 PHY), read from its code and
//! checked against its register trace by the harness for clocks from 100 to
//! 800 MHz per pstate; the drive/ODT fields copy the board's `dram_dx_odt`,
//! `dram_dx_dri` and `dram_ca_dri` bytes.

use super::prog::{Ctx, Op};

/// PHY PLL settings per data-clock band: (upper limit in MHz, inclusive
/// unless noted, PllCtrl 0x2004c for the configuration steps, 0x2004c for
/// the PIE step, 0x2004a, 0x2004b for configuration, 0x2004a/b for PIE).
/// Each group of four bands shares a VCO range; [14:13] of 0x2004c is the
/// output divider within it.
struct Band {
    max_mhz: u32,
    exclusive: bool,
    pll_c: u16,
    pll_i: u16,
    ctl_a: u16,
    ctl_b: u16,
    ctl_i: u16,
}

const fn band(max_mhz: u32, pll_c: u16, pll_i: u16, ctl_a: u16, ctl_b: u16, ctl_i: u16) -> Band {
    Band {
        max_mhz,
        exclusive: false,
        pll_c,
        pll_i,
        ctl_a,
        ctl_b,
        ctl_i,
    }
}

static BANDS: [Band; 14] = [
    band(358, 0x6823, 0x6823, 0x0502, 0x123e, 0x143e),
    band(424, 0x4823, 0x4823, 0x0502, 0x123e, 0x123e),
    band(521, 0x2823, 0x2823, 0x0402, 0x123e, 0x0e3e),
    band(625, 0x0827, 0x0827, 0x0302, 0x123e, 0x0e3e),
    Band {
        max_mhz: 716,
        exclusive: true,
        pll_c: 0x6957,
        pll_i: 0x6817,
        ctl_a: 0x0502,
        ctl_b: 0x123e,
        ctl_i: 0x143e,
    },
    band(848, 0x4957, 0x4817, 0x0502, 0x123e, 0x123e),
    band(1042, 0x2957, 0x2817, 0x0402, 0x123e, 0x0e3e),
    band(1250, 0x095b, 0x081b, 0x0302, 0x123e, 0x0e3e),
    band(1430, 0x6a8b, 0x680b, 0x0502, 0x123e, 0x143e),
    band(1696, 0x4a8b, 0x480b, 0x0502, 0x123e, 0x123e),
    band(2084, 0x2a8b, 0x280b, 0x0402, 0x123e, 0x0e3e),
    band(2500, 0x0a8f, 0x080f, 0x0302, 0x123e, 0x0e3e),
    band(2700, 0x4e8f, 0x4c0f, 0x0701, 0x1c3e, 0x1e3e),
    band(u32::MAX, 0x2e8f, 0x2c0f, 0x0601, 0x1c3e, 0x1a3e),
];

fn band_of(data_mhz: u32) -> &'static Band {
    BANDS
        .iter()
        .find(|b| {
            if b.exclusive {
                data_mhz < b.max_mhz
            } else {
                data_mhz <= b.max_mhz
            }
        })
        .unwrap_or(&BANDS[BANDS.len() - 1])
}

fn byte(word: u32, n: u32) -> u16 {
    ((word >> (8 * n)) & 0xff) as u16
}

/// Values of the configuration steps (C: once and per pstate).
pub fn config(op: &Op, c: &Ctx) -> u16 {
    let b = band_of(c.data_mhz);
    let blk = op.csr >> 12;
    let reg = op.csr & 0xfff;
    match op.csr {
        0x2004c => b.pll_c,
        0x2004a => b.ctl_a,
        0x2004b => b.ctl_b,
        // MASTER: the DFI clock in MHz.
        0x20004 => c.mhz as u16,
        // PIE: 390 ns in DFI clocks / 4.
        0x90801 => ((390 * c.mhz).div_ceil(1000) >> 2) as u16,
        _ => match (blk, reg) {
            // DBYTE n: [15:8] the DFI clock / 50 MHz.
            (0x10..=0x13, 0x00f) => ((c.mhz / 50) << 8) as u16 | (op.val & 0xff),
            // DBYTE n TX impedance (pull-up/pull-down): dram_dx_dri byte n.
            (0x10..=0x13, 0x040 | 0x042 | 0x044 | 0x045) => byte(c.dx_dri, blk - 0x10) * 0x101,
            // DBYTE n ODT: dram_dx_odt byte n.
            (0x10..=0x13, 0x048 | 0x04a | 0x04b) => byte(c.dx_odt, blk - 0x10) << 8,
            // ACX 0/1 CA drive: dram_ca_dri bytes 0 and 1; CA ODT: byte 3.
            (0x30 | 0x31, 0x040 | 0x041) => byte(c.ca_dri, 0) * 0x101,
            (0x30 | 0x31, 0x042 | 0x043) => byte(c.ca_dri, 1) * 0x101,
            (0x30 | 0x31, 0x048 | 0x049) => (byte(c.ca_dri, 3) & 0xf) << 8,
            _ => op.val,
        },
    }
}

/// Values of the per-pstate PIE step (I).
pub fn pie(op: &Op, c: &Ctx) -> u16 {
    let b = band_of(c.data_mhz);
    match op.csr {
        0x2004c => b.pll_i,
        0x2004a | 0x2004b => b.ctl_i,
        // PIE sequencer timers in DFI clocks.
        0x908f0 => ((c.mhz >> 3) + 1) as u16,
        0x908f1 => (c.mhz * 3 / 8) as u16,
        0x908f3 => {
            if c.mhz > 266 {
                88
            } else if c.mhz > 200 {
                66
            } else {
                64
            }
        }
        0x908f4 => (c.mhz / 40) as u16,
        0x90811 => (c.data_mhz <= 1600) as u16,
        0x90812 => 0xff00 | (c.data_mhz <= 1600) as u16,
        0x1000f | 0x1100f | 0x1200f | 0x1300f => ((c.mhz / 50) << 8) as u16 | (op.val & 0xff),
        _ => op.val,
    }
}
