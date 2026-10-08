//! Controller timing registers of the four frequency sets (DRAMTMGn and the
//! DFI/ZQ/refresh timings), from LPDDR5 device timings.
//!
//! The controller runs at the LPDDR5 CK rate, a quarter of the WCK rate the
//! parameter set calls `dram_clk`: pstate n uses `dram_clk / (div_n + 1)`
//! MHz with `div_n` from tpr2 (600/300/200/100 MHz on the Cubie A7A). Every
//! time below is in nanoseconds and rounded up to whole controller cycles.

use crate::ctrl::{ddrc, freq, DDRC_D00, DFIMISC};
use crate::hw::Hw;
use crate::para::Para;

/// Nanoseconds to controller cycles at `mhz`, rounded up.
pub const fn ns(t_ns: u32, mhz: u32) -> u32 {
    (t_ns * mhz).div_ceil(1000)
}

/// LPDDR5 device timings (JEDEC LPDDR5/5X, 8 Gb/16 Gb per channel dies), in ns,
/// as the vendor parameter tables use them.
mod lp5 {
    pub const T_FAW: u32 = 20;
    pub const T_RRD: u32 = 5;
    pub const T_RCD: u32 = 18;
    pub const T_XP: u32 = 7;
    pub const T_WTR_L: u32 = 12;
    pub const T_RTP: u32 = 8; // 7.5 ns
    pub const T_RPAB: u32 = 21;
    pub const T_RAS: u32 = 42;
    pub const T_REFI: u32 = 3906;
    pub const T_RFCAB: u32 = 380;
    /// Power-up: 200 us of RESET/CKE timing for the DFI init counters.
    pub const T_INIT_200US: u32 = 200_000;
    pub const T_ZQCAL: u32 = 260;
    pub const T_XSR_EXTRA: u32 = 90;
    pub const T_CKE: u32 = 3;
    pub const T_ESCKE: u32 = 10;
    pub const T_CCDMW_NS: u32 = 15;
    pub const T_CMD_1NS: u32 = 1;
    pub const T_MRW_40: u32 = 40;
    pub const T_ZQRESET_6US: u32 = 6000;
    pub const T_ZQLAT_30: u32 = 30;
    pub const T_ZQCS_50: u32 = 50;
}

/// Write the timing registers of every frequency set of channel `ch`.
/// Also fixes up the mode-register words of the parameter set the PHY
/// firmware trains with (MR1 [7], MR2 [7]/[3]) and returns the read latency
/// it chose (13 nCK, 14 with tpr10[31]).
pub fn set_channel_timing<H: Hw>(hw: &mut H, p: &mut Para, ch: usize) -> u32 {
    let multi = p.tpr13() & 5 == 5;
    let mut rl = 13;
    for ps in (0..=if multi { 3 } else { 0 }).rev() {
        let mhz = p.clk() / (((p.tpr2() >> (8 * ps)) & 0x1f) + 1);
        rl = set_freq_timing(hw, p, ch, ps, mhz);
        hw.udelay(20);
    }
    rl
}

/// Mode register words of the parameter set as the timing step leaves them
/// for the message block: MR1 [7], MR2 [7] and [3] set.
pub fn mr_fixup(p: &mut Para) {
    p.w[9] |= 0x80; // MR1
    p.w[10] |= 0x88; // MR2
}

fn set_freq_timing<H: Hw>(hw: &mut H, p: &mut Para, ch: usize, ps: usize, mhz: u32) -> u32 {
    use lp5::*;
    let t = |n| ns(n, mhz);
    let long_rl = p.tpr10() & (1 << 31) != 0;

    let faw = t(T_FAW);
    let rrd = t(T_RRD).max(2);
    let rcd = t(T_RCD).max(2);
    let xp = t(T_XP).max(3);
    // tWTR_L: 12 ns, 14 ns for the tpr10[31] (slower) speed bin.
    let wtr = t(if long_rl { T_WTR_L + 2 } else { T_WTR_L }).max(4);
    let rtp = t(T_RTP).max(2);
    let rp = t(T_RPAB).max(2);
    let ras = t(T_RAS).max(3);
    let rc = ras + rp;
    let refi_x32 = t(T_REFI) >> 5;
    let rfc = t(T_RFCAB);
    let xsr = rfc + rtp;
    // 200 us in units of 1024 cycles. The vendor init takes the cycle count
    // modulo 2^16 first, which only matters at 600 MHz (54 instead of 118);
    // kept, since that is what has run on hardware.
    let init_1k = (t(T_INIT_200US) & 0xffff).div_ceil(1024);
    let zqcal = t(T_ZQCAL);
    let xsr_extra = t(T_XSR_EXTRA);
    let cke = t(T_CKE);
    let rrd1 = rrd + 1;
    // tCKE-ish minimum for powerdown exit: tRTP when tpr10[31], else tXP.
    let pd_min = (if long_rl { t(T_RTP) } else { t(T_XP) }).max(4);
    let esck = t(T_ESCKE) + 1;
    let rrd_min3 = t(T_RRD).max(3);
    let ccdmw = t(T_CCDMW_NS).max(2);
    let ras_max = (refi_x32 * 9) >> 5;
    let cmd_1ns = t(T_CMD_1NS);
    let mrw = t(T_MRW_40).max(8);
    let zqreset = t(T_ZQRESET_6US) + 10;
    let zqlat = t(T_ZQLAT_30);
    let zqlat4 = zqlat.max(4);
    let zqcs = t(T_ZQCS_50).max(3);

    // Read/write latencies in nCK (JEDEC LPDDR5/5X RL/WL sets for this speed bin).
    let rl = if long_rl { 14 } else { 13 };
    let wl_term = if long_rl { 22 } else { 21 };
    let wr2pre = wl_term + 10;
    let rd2wr = rl + 3;
    let wr2rd = wtr + 11;
    let mr_wl = rl * 4 - 5;
    mr_fixup(p);

    let rd2pre = rtp;
    let wr2mr = rtp.max(4) + rd2pre + rl + 6;
    let mrd = rtp.max(4) + 7 + wl_term + 4;
    let dfi_rddata = rl + 11 + faw;
    let dfi_wrdata = pd_min + 9;
    let cmd_q = (cmd_1ns >> 2) & 0x3fff;
    let cke_dfi = cke + 7;
    let rd_tphy = cmd_q + 11;
    let wr_tphy = cmd_q + 2 + rd2wr;

    let f = |off| freq(ch, ps, off);
    hw.write32(f(0x00), ras | (ras_max << 8) | (faw << 16) | (wr2pre << 24));
    hw.write32(f(0x04), rc | (rd2pre << 8) | (xp << 16));
    hw.write32(f(0x08), wr2rd | (rd2wr << 8) | (rl << 16) | (7 << 24));
    hw.write32(f(0x0c), mrd | (wr2mr << 8) | (10 << 16));
    hw.write32(f(0x10), rp | (rrd << 8) | (4 << 16) | (rcd << 24));
    hw.write32(f(0x14), esck | (ccdmw << 8) | (rrd_min3 << 16) | (2 << 24));
    hw.write32(f(0x18), xp + 2);
    hw.write32(f(0x1c), cke);
    hw.write32(f(0x24), dfi_wrdata | (rrd1 << 8) | 0x2_0000);
    hw.write32(f(0x30), 3 << 16);
    hw.write32(f(0x34), 2 | (16 << 16) | (10 << 24));
    hw.write32(f(0x38), xsr | (mrw << 16));
    hw.write32(f(0x5c), esck | (init_1k << 16));
    hw.write32(f(0x60), 12 | ((rl + 5) << 8) | ((rl + 1) << 16));
    hw.write32(f(0x64), rtp | (wr2pre << 8));
    hw.write32(f(0x78), 0x0055_6262);
    hw.write32(f(0x80), 0);
    // [0]: DFI 1:4 frequency ratio (cleared in the LPDDR4 low-speed mode).
    hw.set32(f(DDRC_D00), 1);
    if ps == 0 {
        // DDRC +0xd00: LPDDR5 mode [31:30] = 1, or 3 with the training
        // mode word set (the vendor default), and the DFI init timers in
        // units of 1024 cycles of the quarter-rate clock: 2 us and 2 ms.
        let q = mhz / 4;
        let mut v = hw.read32(ddrc(ch, DDRC_D00));
        v = (v & !0xc000_0000) | 0x4000_0000;
        v |= 0xc000_0000;
        v &= !(0x3fc_0000 | 0x3_1c00 | 0x3fc | 3);
        let t2us = ns(2000, q);
        let t2ms = ns(2_000_000, q);
        hw.write32(
            ddrc(ch, DDRC_D00),
            v | ((t2ms >> 10) + 1) | (((t2us >> 10) + 1) << 16),
        );
        hw.clr32(ddrc(ch, DFIMISC), 1);
    }
    hw.modify32(f(0xd04), |v| (v & 0xffff_0000) | rd_tphy | (cke_dfi << 8));
    hw.modify32(f(0xd08), |v| {
        (v & 0xffff_0000) | dfi_rddata | (wr_tphy << 8)
    });
    hw.write32(f(0xd0c), 0x0040_0005);
    hw.write32(f(0x5a0), 0x0002_0202);
    hw.write32(f(0x5a4), 0x201);
    hw.write32(f(0x5a8), 0x001c_0003);
    hw.write32(f(0x5ac), 0x0058_00a0);
    hw.write32(f(0x5b0), 0x97);
    hw.write32(f(0x5b8), 0x147);
    hw.write32(f(0x580), 0x0300_0217 | (mr_wl << 16));
    hw.write32(f(0x584), 0x0007_0303);
    hw.write32(f(0x588), 0x0018_0017 | (mr_wl << 8));
    hw.write32(f(0x590), (2 << 18) | (4 << 26) | 0x411);
    hw.write32(f(0x594), 0x0400_000f | (3 << 18));
    hw.modify32(f(0x600), |v| (v & 0x7fff_0000) | refi_x32);
    hw.modify32(f(0x604), |v| (v & 0xf000_f000) | rfc | (rfc << 16));
    hw.modify32(f(0x608), |v| {
        (v & 0xffff) | (xsr_extra << 16) | (t(T_RTP) << 24)
    });
    hw.modify32(f(0x650), |v| (v & !0xfff) | zqcal);
    hw.write32(f(0x800), zqreset | (zqlat4 << 16));
    hw.write32(f(0x804), (zqcs << 20) | 0x100);
    hw.write32(f(0x808), zqlat);
    hw.write32(f(0xa80), 0x1f74);
    hw.write32(f(0xb00), 0x07c0_c3e2);
    hw.write32(f(0xb04), 0x1024_100a);
    hw.write32(f(0xb08), 0x33);
    hw.write32(f(0xb80), 0x17_0000);
    rl
}
