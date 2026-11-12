// SPDX-License-Identifier: GPL-2.0-only
//! The PHY's mission state, dumped by address range for a host-side diff
//! between the vendor library's end state (feature av3h) and ours after
//! the hand-over. Every 16-bit CSR of [`RANGES`] is read once; only
//! non-zero words are printed, eight per line as `aaaaa=vvvv`, so the UART
//! volume stays bounded. Identical ranges and order in both images. Read
//! only: the CSR mux and the PMU clocks are the only writes, both put
//! back.
use crate::hw::Hw;
use crate::phy::{csr, csr_access, UCCLK_HCLK_ENABLES};

/// The CSR ranges `[start, end)`: the four DBYTEs, MASTER, the two AC
/// blocks, the ACSM, the message block (DMEM head), the PIE/INITENG, DRTUB
/// and APBONLY.
pub const RANGES: [(u32, u32); 12] = [
    (0x10000, 0x10400),
    (0x11000, 0x11400),
    (0x12000, 0x12400),
    (0x13000, 0x13400),
    (0x20000, 0x20400),
    (0x30000, 0x30400),
    (0x31000, 0x31400),
    (0x40000, 0x42000),
    (0x58000, 0x58100),
    (0x90000, 0x91000),
    (0xc0000, 0xc0200),
    (0xd0000, 0xd0100),
];

/// Print every non-zero CSR of [`RANGES`] with `tag` in the prefix.
pub fn dump_phy<H: Hw>(hw: &mut H, tag: &str) {
    csr_access(hw, true);
    let hclk = hw.read16(UCCLK_HCLK_ENABLES);
    hw.write16(UCCLK_HCLK_ENABLES, 3);
    hw.log(format_args!(
        "[dump {tag}] start: ranges {RANGES:05x?}; UcclkHclkEnables was {hclk:04x}\n"
    ));
    let mut total = 0u32;
    for &(a, b) in RANGES.iter() {
        let mut buf = [(0u32, 0u16); 8];
        let mut k = 0;
        let mut n = 0u32;
        for c in a..b {
            if c & 0xff == 0 {
                hw.kick_watchdog();
            }
            let v = hw.read16(csr(c));
            if v == 0 {
                continue;
            }
            buf[k] = (c, v);
            k += 1;
            n += 1;
            if k == buf.len() {
                line(hw, tag, &buf[..k]);
                k = 0;
            }
        }
        if k > 0 {
            line(hw, tag, &buf[..k]);
        }
        hw.log(format_args!(
            "[dump {tag}] range {a:05x}-{b:05x}: {n} non-zero\n"
        ));
        total += n;
    }
    hw.write16(UCCLK_HCLK_ENABLES, hclk);
    csr_access(hw, false);
    hw.log(format_args!("[dump {tag}] end: {total} non-zero words\n"));
}

fn line<H: Hw>(hw: &mut H, tag: &str, words: &[(u32, u16)]) {
    hw.log(format_args!("[dump {tag}]"));
    for &(c, v) in words {
        hw.log(format_args!(" {c:05x}={v:04x}"));
    }
    hw.log(format_args!("\n"));
}
