// SPDX-License-Identifier: GPL-2.0-only
//! The DDRCTL/DDRPHY value set for one configuration, computed from JEDEC
//! LPDDR4/4X rules and the DDRCTL field formulas printed in the RK3588 TRM
//! Part 2 field descriptions (pp.69-108, the "LPDDR4:" lines). Units are
//! DRAM clock cycles (tCK) unless a field says otherwise (p.69); the
//! controller runs 1:2 (TMGCFG.frequency_ratio = 0, p.105).
//!
//! JEDEC values not in crate jedec-lpddr (LPDDR4 core AC timing, the
//! speed-bin-independent minima valid up to 4266 MT/s, so valid at any
//! lower clock) [I: to re-check against the fitted part's datasheet]:
//! tRCD 18 ns/4 nCK, tRAS 42 ns/3, tWTR 10 ns/8, tRTP 7.5 ns/8, tRRD 10
//! ns/4, tFAW 40 ns, tDQSCK(max) 3.5 ns, tCCD 8 (BL16), tCCDMW 32, tPPD 4,
//! tMRR 8 nCK, tMRWCKEL 14 ns/10, tESCKE/tCMDCKE 1.75 ns/3, tCKCKEH 1.75
//! ns/3, tCKELCK 5 ns/5, tSR 15 ns/3, tOSCO 40 ns/8, tREFI 3.904 us
//! (32 ms / 8192, 1x rate). From jedec-lpddr: the RL/WL/nWR/nRTP table, the
//! MR encoders, tRFCab by density, tRPab, tXSR, tXP, tCKE, tMRD, tMRW,
//! tZQCAL, tZQLAT, tZQRESET.
//!
//! Licence guard: nothing here comes from a Rockchip binary or a register
//! dump of one.

use super::Config;
use jedec_lpddr::lp4::{mr, timing as jt};

/// Burst length 16: BL/2 = 8 clocks.
const BL2: u32 = 8;

/// One channel's register values (FREQ0 set, static registers, PHY).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Plan {
    pub ck_mhz: u32,
    pub ranks: u8,
    pub rl: u32,
    pub wl: u32,
    pub nwr: u32,
    pub mstr0: u32,
    /// DRAMSET1TMG0..9.
    pub dramset1tmg: [u32; 10],
    pub tmg12: u32,
    pub tmg13: u32,
    pub tmg14: u32,
    pub tmg25: u32,
    pub initmr: [u32; 4],
    pub dfitmg0: u32,
    pub rfshset1tmg0: u32,
    pub rfshset1tmg1: u32,
    pub zqset1tmg0: u32,
    pub zqset1tmg1: u32,
    pub inittmg0: u32,
    pub inittmg1: u32,
    pub odtmap: u32,
    /// MR1, MR2, MR3, MR11, MR12, MR13, MR14, MR22 opcodes.
    pub mr1: u8,
    pub mr2: u8,
    pub mr3: u8,
    pub mr11: u8,
    pub mr12: u8,
    pub mr13: u8,
    pub mr14: u8,
    pub mr22: u8,
    /// PHY scheduler timings (SCHD_TIME_CON0/1, P2 pp.486-487).
    pub schd_time0: u32,
    pub schd_time1: u32,
    /// LP4_MR_OP0/1: the opcodes the PHY sends in its own MRWs (p.498).
    pub lp4_mr_op0: u32,
    pub lp4_mr_op1: u32,
    /// A computed value did not fit its field (saturated).
    pub overflow: bool,
}

const fn c(ps: u32, nck: u32, mhz: u32) -> u32 {
    jt::Min { nck, ps }.cycles(mhz)
}

/// tREFI at 1x refresh rate (JEDEC LPDDR4: 32 ms / 8192 rows).
pub const T_REFI_PS: u32 = 3_904_000;
/// tDQSCK(max), JEDEC LPDDR4.
pub const T_DQSCK_MAX_PS: u32 = 3_500;

/// tRFCab class for a per-die channel density (Gb); 16 Gb for unknown.
fn refresh(density_gb: u8) -> jt::Refresh {
    match jt::refresh(density_gb) {
        Some(r) => r,
        None => jt::refresh(16).expect("16 Gb refresh class"),
    }
}

/// The value set for `ranks` ranks of `density_gb` Gb per die and channel.
pub fn plan(cfg: &Config, ranks: u8, density_gb: u8) -> Plan {
    let f = cfg.ck_mhz;
    let (code, row) = jt::freq_range(f).unwrap_or((3, jt::FREQ_RANGES[3]));
    let (rl, wl, nwr, nrtp) = (
        row.rl as u32,
        row.wl_a as u32,
        row.nwr as u32,
        row.nrtp as u32,
    );
    let t_rcd = c(18_000, 4, f);
    let t_rp = jt::T_RPAB.cycles(f);
    let t_ras = c(42_000, 3, f);
    let t_rc = t_ras + t_rp;
    let t_wtr = c(10_000, 8, f);
    let t_rtp = c(7_500, 8, f).max(nrtp);
    let t_rrd = c(10_000, 4, f);
    let t_faw = c(40_000, 0, f);
    let t_xp = jt::T_XP.cycles(f).max(jt::T_CKELPD.cycles(f));
    let t_cke = jt::T_CKE.cycles(f);
    let t_mrd = jt::T_MRD.cycles(f);
    let t_mrw = jt::T_MRW.cycles(f);
    let t_dqsck = c(T_DQSCK_MAX_PS, 0, f);
    let pad = cfg.turnaround_pad as u32;
    let rfsh = refresh(density_gb);
    let t_rfc = rfsh.t_rfc_ab.cycles(f);
    let t_xsr = jt::t_xsr(rfsh.t_rfc_ab).cycles(f);
    let t_refi = jt::Max { ps: T_REFI_PS }.cycles(f);
    // tRAS(max) = 9 * tREFI, RoundDown(/1024) (p.69).
    let t_ras_max = (9 * t_refi) / 1024;
    let wr2pre = wl + BL2 + nwr + 1; // p.69, +1 for LPDDR4
    let rd2pre = BL2 + t_rtp.max(8) - 8; // p.70
    let wr2rd = wl + BL2 + t_wtr + 1 + pad; // p.73
                                            // p.72, DQ ODT disabled: RL + BL/2 + RU(tDQSCKmax) + WR_PRE(2) + RD_POST(0.5 -> 1) - WL.
    let rd2wr = rl + BL2 + t_dqsck + 2 + 1 - wl + pad;
    // p.74: t_mr = max(tMRR, tMRW, tMRWCKEL, tMRD).
    let t_mr = 8.max(t_mrw).max(c(14_000, 10, f)).max(t_mrd);
    // rd2mr: RL + BL/2 + RU(tDQSCKmax) + RD(tRPST = 0.5) + max(RU(7.5ns), 8) + nRTP - 8.
    let rd2mr = rl + BL2 + t_dqsck + c(7_500, 8, f) + nrtp - 8;
    // wr2mr: WL + 1 + BL/2 + max(RU(7.5ns), 8) + nWR.
    let wr2mr = wl + 1 + BL2 + c(7_500, 8, f) + nwr;
    let t_ckcskeh = c(1_750, 3, f); // tCKCKEH
    let t_ckelck = c(5_000, 5, f); // tCKELCK
    let t_ckesr = t_cke.max(c(15_000, 3, f)); // max(tCKE, tSR)
    let x32 = |v: u32| v.div_ceil(32);
    // Fields with their TRM widths (pp.69-82): a value that does not fit
    // saturates and sets `overflow` (never silently truncated).
    let mut overflow = false;
    let mut pk = |fields: &[(u32, u32, u32)]| {
        fields.iter().fold(0u32, |acc, &(v, shift, width)| {
            let max = (1u32 << width) - 1;
            if v > max {
                overflow = true;
            }
            acc | (v.min(max) << shift)
        })
    };
    let tmg = [
        pk(&[
            (wr2pre, 24, 8),
            (t_faw, 16, 8),
            (t_ras_max, 8, 8),
            (t_ras, 0, 8),
        ]),
        pk(&[(t_xp, 16, 6), (rd2pre, 8, 8), (t_rc, 0, 8)]),
        pk(&[(wl, 24, 7), (rl, 16, 7), (rd2wr, 8, 8), (wr2rd, 0, 8)]),
        pk(&[(t_mr, 16, 7), (rd2mr, 8, 8), (wr2mr, 0, 8)]),
        pk(&[(t_rcd, 24, 8), (8, 16, 6), (t_rrd, 8, 6), (t_rp, 0, 7)]),
        pk(&[
            (t_ckcskeh, 24, 6),
            (t_ckelck, 16, 7),
            (t_ckesr, 8, 7),
            (t_cke, 0, 6),
        ]),
        pk(&[(t_xp + 2, 0, 6)]), // TMG6 t_ckcsx = tXP + 2 (p.77)
        0,                       // TMG7 t_csh: LPDDR5 only (p.78)
        pk(&[(x32(t_xsr), 8, 7), (x32(t_xsr), 0, 7)]), // TMG8 t_xs_dll_x32 / t_xs_x32
        // TMG9: bank-group fields (DDR4/LPDDR5); reset-style values.
        pk(&[(4, 16, 5), (t_rrd, 8, 6), (wr2rd, 0, 8)]),
    ];
    let tmg12 = pk(&[(c(1_750, 3, f), 16, 4)]); // t_cmdcke = max(tESCKE, tCMDCKE)
                                                // TMG13: odtloff keeps its reset 0x1c (DQ ODT off), t_ccd_mw 32, t_ppd 4.
    let tmg13 = (0x1c << 24) | (32 << 16) | 4;
    let tmg14 = pk(&[(c(40_000, 8, f), 16, 9), (t_xsr, 0, 12)]);
    // TMG25: rwa2pre 2 (LPDDR4), wra2pre = WL + BL/2 + nWR + 1, rda2pre = nRTP.
    let tmg25 = pk(&[(2, 16, 3), (wl + BL2 + nwr + 1, 8, 8), (nrtp, 0, 8)]);
    // Mode registers.
    let mr1 = (1 << 2) // WR preamble 2 tCK (static), BL16 sequential
        | (mr::nwr_code(nwr as u8).unwrap_or(code) << 4); // nWR OP[6:4]
    let mr2 =
        mr::rl_code(rl as u8).unwrap_or(code) | (mr::wl_a_code(wl as u8).unwrap_or(code) << 3);
    // MR3: jedec-lpddr defaults (OP[0] PU-CAL = 1, WR postamble 0.5 tCK,
    // DBI off) with PDDS RZQ/6 (40 ohm).
    let mr3 = mr::Mr3::defaults(mr::Rzq::Div6).encode().unwrap_or(0x31);
    let mr13 = 0u8; // FSP-WR/OP 0, DM enabled (OP[5] = 0)
    let initmr = [
        ((mr1 as u32) << 16) | mr2 as u32,
        ((mr3 as u32) << 16) | mr13 as u32,
        ((cfg.mr11 as u32) << 16) | cfg.mr12 as u32, // INITMR2: MR11, MR12 (p.84)
        ((cfg.mr22 as u32) << 16) | cfg.mr14 as u32, // INITMR3: MR22, MR14
    ];
    let active = if ranks >= 2 { 3 } else { 1 };
    // MSTR0: lpddr4 [1], BL16 burst_rdwr = 0b01000 [20:16], active_ranks [25:24] (p.108).
    let mstr0 = (1 << 1) | (0x8 << 16) | (active << 24);
    let d = cfg.dfi;
    let dfitmg0 = (7 << 24) // dfi_t_ctrl_delay: reset value (p.85)
        | ((rl.saturating_sub(d.rddata_en_sub) & 0x7f) << 16)
        | ((d.tphy_wrdata & 0x3f) << 8)
        | (wl.saturating_sub(d.wrlat_sub) & 0x3f);
    // INITTMG at the 24 MHz init clock, counted as DRAM clocks (the longer
    // reading of the unit): tINIT3 2 ms -> pre_cke, tINIT5 2 us -> post_cke,
    // tINIT1 200 us -> dram_rstn; at least the reset values (p.168-169).
    let x1024 = |us: u32| (us * 24).div_ceil(1024).max(1);
    let inittmg0 = x1024(2_000).max(0x4e) | (x1024(2).max(2) << 16);
    let inittmg1 = x1024(200) << 16;
    // ZQ timings (p.99): long = tZQCAL, short = tZQLAT, reset = tZQRESET,
    // interval keeps the reset value (0x100 x 1024 clocks).
    let zqset1tmg0 = (jt::T_ZQLAT.cycles(f).min(0x3ff) << 16) | jt::T_ZQCAL.cycles(f).min(0x3fff);
    let zqset1tmg1 = (jt::T_ZQRESET.cycles(f).min(0x3ff) << 20) | 0x100;
    // ODTMAP: write ODT on the written rank only; read ODT none (DQ ODT is
    // not used by the default MR11) (p.166).
    let odtmap = if ranks >= 2 { 0x0201 } else { 0x0001 };
    // PHY scheduler (p.486-487): WL = DRAM WL; phy_t_rddata_en keeps its
    // reset 7 [U: the LP5 note on p.567 uses 6]; tMRW/tMRR in tCK.
    let schd_time0 = (t_mrw.min(0x3f) << 18) | (8 << 14) | (7 << 6) | (wl & 0x3f);
    let schd_time1 =
        (6 << 28) | (t_wtr.min(0x3f) << 22) | (t_rcd.min(0xff) << 14) | (t_rtp.min(0x3f) << 8) | 8;
    let lp4_mr_op0 =
        ((cfg.mr12 as u32) << 24) | ((cfg.mr11 as u32) << 16) | ((mr2 as u32) << 8) | mr1 as u32;
    Plan {
        ck_mhz: f,
        ranks,
        rl,
        wl,
        nwr,
        mstr0,
        dramset1tmg: tmg,
        tmg12,
        tmg13,
        tmg14,
        tmg25,
        initmr,
        dfitmg0,
        // t_refi_x1_x32 rounded down: 32 x value must not exceed tREFI (p.95).
        rfshset1tmg0: pk(&[(t_refi / 32, 0, 12), (0x10, 16, 6), (2, 24, 4)]),
        rfshset1tmg1: pk(&[(t_rfc, 0, 12)]),
        zqset1tmg0,
        zqset1tmg1,
        inittmg0,
        inittmg1,
        odtmap,
        mr1,
        mr2,
        mr3,
        mr11: cfg.mr11,
        mr12: cfg.mr12,
        mr13,
        mr14: cfg.mr14,
        mr22: cfg.mr22,
        schd_time0,
        schd_time1,
        lp4_mr_op0,
        lp4_mr_op1: mr13 as u32,
        overflow,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn plan_1056() {
        let p = plan(&Config::yy3588_default(), 2, 8);
        assert_eq!((p.rl, p.wl, p.nwr), (20, 10, 20));
        // tRCD 18 ns at 1056 MHz = 19.008 -> 20 clocks (rounded up).
        assert_eq!(p.dramset1tmg[4] >> 24, 20);
        assert_eq!(p.mstr0, 0x0308_0002);
        assert_eq!(p.rfshset1tmg1, 296); // tRFCab 280 ns (8 Gb)
        assert_eq!(p.initmr[0] & 0xffff, 3 | (3 << 3));
        // tREFI 3904 ns = 4122 clocks; /32 rounded down = 128.
        assert_eq!(p.rfshset1tmg0 & 0xfff, 128);
    }
}
