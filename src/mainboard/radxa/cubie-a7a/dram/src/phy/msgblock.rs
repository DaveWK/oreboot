//! The training firmware's message block: its inputs at the start of DMEM
//! (0x235 halfwords, before the firmware's own data image).
//!
//! Layout and values as the vendor library builds them for LPDDR5 (field
//! names after Synopsys' message block where the role is clear). Offsets
//! are in halfwords.

use crate::para::Para;

pub const WORDS: usize = 0x235;

/// Build the message block for `pstate` training at `data_mhz` (the DRAM
/// data clock: 2400 MHz at pstate 0 of the Cubie A7A). `rl` is the read
/// latency chosen for the controller timing (13 or 14 nCK).
pub fn build(p: &mut Para, pstate: u32, data_mhz: u32, rl: u32) -> [u16; WORDS] {
    let mut m = [0u16; WORDS];
    let dfs = p.tpr13() & 4 != 0;
    // [2] low byte: Pstate and its training steps; [3]: DRAMFreq (MT/s,
    // at least 1600).
    m[2] = if dfs {
        match pstate {
            0 => 0xa0,
            1 => 0x91,
            2 => 0x82,
            _ => 0x83,
        }
    } else {
        0x30
    };
    m[3] = (2 * data_mhz).max(1600) as u16;
    m[4] = 4;
    m[5] = 4;
    // [8]: training options from tpr10[28:16] (default 0x31f, written back).
    let mut opts = (p.tpr10() >> 16) & 0x1fff;
    if opts == 0 {
        opts = 0x31f;
        p.set_tpr10(p.tpr10() | 0x031f_0000);
    }
    m[8] = opts as u16;
    m[9] = 0xff;
    m[11] = 2;
    m[14] = if p.tpr10() & (1 << 31) != 0 { 15 } else { 0 };
    // [0x12]: enabled DQ bytes/ranks: 16-bit channel, one or two ranks.
    let half = p.para2() & 1 != 0;
    m[0x12] = match (half, p.dual_rank()) {
        (true, true) => 0x310,
        (true, false) => 0x110,
        (false, true) => 0x320,
        (false, false) => 0x120,
    };
    if p.channels() == 2 {
        m[0x1f] = 0x1000;
    }
    m[0x20] = if p.dual_rank() { 3 } else { 1 };
    // Mode registers for both channels (low byte channel A, high byte B).
    let mr2 = |v: u32| ((v & 0xff) | (v & 0xff) << 8) as u16;
    m[0x2d] = mr2(p.mr(1));
    m[0x2e] = m[0x2d];
    m[0x2f] = mr2(p.mr(2));
    m[0x30] = m[0x2f];
    m[0x31] = mr2(p.mr(3));
    m[0x32] = m[0x31];
    m[0x33] = 0x5454;
    m[0x34] = 0x5454;
    m[0x35] = mr2(p.w[15]); // MR11
    m[0x36] = m[0x35];
    m[0x37] = mr2(p.w[16]); // MR12
    m[0x38] = m[0x37];
    m[0x39] = mr2(p.w[17]); // MR13
    m[0x3a] = m[0x39];
    if p.tpr10() & (1 << 29) != 0 {
        // MR14 (VREF(DQ)) for both ranks.
        m[0x3b..=0x3e].fill(mr2(p.w[18]));
    }
    m[0x3f] = mr2(p.w[19]); // MR16
    m[0x40] = m[0x3f];
    m[0x41] = mr2(p.w[20]) | 0x2800; // MR17 (+ MR18 high bits)
    m[0x42] = m[0x41];
    m[0x43] = 0x1c1c;
    m[0x44] = 0x1c1c;
    m[0x45] = 0x1010;
    m[0x46] = 0x1010;
    m[0x47] = 0x0202;
    m[0x48] = 0x0202;
    m[0x4b] = mr2(p.w[21]); // MR22
    m[0x4c] = m[0x4b];
    m[0x55] = 0x0404;
    m[0x56] = 0x0404;
    m[0x65] = 0x6060;
    m[0x66] = 0x6060;
    m[0x67] = if p.tpr10() & (1 << 29) == 0 || (dfs && pstate != 0) {
        1
    } else {
        0
    };
    m[0x74] = 0x2e;
    m[0x75] = 0x50;
    if dfs {
        m[0x78] = 0x0058 + 0x400 * pstate as u16;
        m[0x79] = 0x0287 + 0x400 * pstate as u16;
    }
    m[0x80] = 0x0f01;
    // Read latency in WCK cycles, minus one and minus five.
    m[0x81] = ((rl * 4 - 1) << 8) as u16;
    m[0x82] = (rl * 4 - 5) as u16;
    m[0x204] = 0x0500;
    m[0x212] = 0x715d;
    m[0x213] = 0x00ce;
    m[0x21d] = 4;
    m[0x22d] = 4;
    m
}
