// SPDX-License-Identifier: GPL-2.0-only
//! RK3588 DDRPHY set-up outside training.
//!
//! The PHY (VERSION_INFO_STAT0 reset 0x09080005, P2 p.372) is not a
//! Synopsys PHY: it has no firmware and its own training scheduler. Its
//! register map is TRM Part 2 §2.4.3-2.4.4 (pp.314-555); the same map is
//! used by the RK3576 U-Boot RFC (Johan Axelsson, 2026-05, GPL-2.0+,
//! sdram_phy_rk3576.h), whose phy_set_mode / phy_mdll_lock /
//! phy_zq_calibrate / phy_post_training order this module follows where
//! the RK3588 TRM agrees (step numbers below are RK3588 §2.6.1).

use super::timing::Plan;
use super::DqsClean;
use crate::regs::{base, phy};
use regio::{wait, RegIo, Timeout};

const T: u64 = 100_000;

/// Step 11: LPDDR4 mode (GNR_CON0.ctrl_ddr_mode = 0), 1:2:2 clock mode
/// with the PHY 2x clock (CLKMODE_CON.ctrl_phy_mode = 1, ctrl_phy_clk_2x
/// = 1) (pp.314, 501).
pub fn set_mode<H: RegIo + ?Sized>(io: &mut H, ch: usize) {
    let pb = base::DDRPHY[ch];
    io.clrset32(pb + phy::GNR_CON0, 3 << 24, 0);
    io.clrset32(pb + phy::CLKMODE_CON, 0xf, (1 << 3) | 1);
}

/// Steps 21-22 and 26-27: latencies, burst, rank enables, the scheduler's
/// own DRAM timings and the MR opcodes the PHY uses in its MRWs. IO drive
/// and ODT keep their reset values (codes not specified in the TRM) except
/// the read ODT, which the DQS-clean choice needs.
pub fn configure<H: RegIo + ?Sized>(io: &mut H, ch: usize, p: &Plan, dqs: DqsClean, row_bits: u32) {
    let pb = base::DDRPHY[ch];
    // GNR_CON0: ctrl_rdlat_rank0 [5:0] = RL, ctrl_twpre [15] = 0 (2 tCK, the
    // only LPDDR4 write preamble), BL16 (ctrl_bstlen / otf keep reset).
    io.clrset32(pb + phy::GNR_CON0, (1 << 15) | 0x3f, p.rl & 0x3f);
    // RANK1_CON0.ctrl_rdlat_rank1 [5:0] (p.484).
    io.clrset32(pb + phy::RANK1_CON0, 0x3f, p.rl & 0x3f);
    // GATE_CON0.rank_en [25:24] (p.330).
    let ranks = if p.ranks >= 2 { 3 } else { 1 };
    io.clrset32(pb + phy::GATE_CON0, 3 << 24, ranks << 24);
    io.write32(pb + phy::SCHD_TIME_CON0, p.schd_time0);
    io.write32(pb + phy::SCHD_TIME_CON1, p.schd_time1);
    io.write32(pb + phy::LP4_MR_OP0, p.lp4_mr_op0);
    io.write32(pb + phy::LP4_MR_OP1, p.lp4_mr_op1);
    // CAL_CON4.LP4_High_density_mode: R17 (18th row bit) in ACT-2 (p.326).
    if row_bits >= 18 {
        io.set32(pb + phy::CAL_CON4, phy::CAL_CON4_LP4_HIGH_DENSITY);
    } else {
        io.clr32(pb + phy::CAL_CON4, phy::CAL_CON4_LP4_HIGH_DENSITY);
    }
    if let DqsClean::Auto = dqs {
        // §2.6.5 option 1 (needs VSSQ termination): dqs*_testircv = 3,
        // gate_cal_mode = 0, auto_dqs_clean = 1, ctrl_rodt_disable = 0.
        io.clrset32(pb + phy::TESTIRCV_CON0, 0xff, 0x33);
        io.clr32(pb + phy::CAL_CON0, phy::CAL_GATE_CAL_MODE);
        io.set32(pb + phy::CAL_CON3, phy::CAL3_AUTO_DQS_CLEAN);
        io.clr32(pb + phy::CAL_CON2, phy::CAL2_RODT_DISABLE);
    }
}

/// Step 28: ZQ calibration. Ok(true) when the PHY flags a failure
/// (ZQ_CON1.zq_mon, p.374): logged by the caller, not fatal at low speed.
pub fn zq<H: RegIo + ?Sized>(io: &mut H, ch: usize) -> Result<bool, Timeout> {
    let pb = base::DDRPHY[ch];
    io.set32(pb + phy::ZQ_CON0, phy::ZQ_CLK_DIV_EN);
    io.set32(pb + phy::ZQ_CON0, phy::ZQ_MANUAL_STR);
    wait(io, "ZQ_CON1.zq_done", T, |io| {
        io.read32(pb + phy::ZQ_CON1) & phy::ZQ_DONE != 0
    })?;
    let fail = io.read32(pb + phy::ZQ_CON1) & phy::ZQ_FAIL != 0;
    io.clr32(pb + phy::ZQ_CON0, phy::ZQ_MANUAL_STR);
    io.clr32(pb + phy::ZQ_CON0, phy::ZQ_CLK_DIV_EN);
    Ok(fail)
}

/// Steps 30-31: master DLL on, ctrl_start 0 -> 1, wait ctrl_locked (p.344).
pub fn dll_lock<H: RegIo + ?Sized>(io: &mut H, ch: usize) -> Result<(), Timeout> {
    let pb = base::DDRPHY[ch];
    io.set32(pb + phy::MDLL_CON0, phy::MDLL_DLL_ON);
    io.clr32(pb + phy::MDLL_CON0, phy::MDLL_START);
    io.set32(pb + phy::MDLL_CON0, phy::MDLL_START);
    wait(io, "MDLL_CON1.ctrl_locked", T, |io| {
        io.read32(pb + phy::MDLL_CON1) & phy::MDLL_LOCKED != 0
    })
}

/// Toggle OFFSETD_CON0.ctrl_resync: push DLL codes to the slave DLLs
/// (the TRM's UPD_CODE, §2.6.4).
pub fn resync<H: RegIo + ?Sized>(io: &mut H, ch: usize) {
    let pb = base::DDRPHY[ch];
    io.set32(pb + phy::OFFSETD_CON0, phy::OFFSETD_RESYNC);
    io.clr32(pb + phy::OFFSETD_CON0, phy::OFFSETD_RESYNC);
}

/// Step 34 (and the §2.6.9 "setting after training", the same engine):
/// DLL tracking back on, VT compensation on, cycle-mode write training off,
/// resync. dvfs_wr_train_en stays 0 (no DVFS or periodic training).
pub fn after_training<H: RegIo + ?Sized>(io: &mut H, ch: usize) {
    let pb = base::DDRPHY[ch];
    io.clr32(pb + phy::MDLL_CON0, phy::MDLL_CLKM_CG_EN_SW);
    io.set32(pb + phy::CAL_CON0, phy::CAL_VTC_EN);
    io.clr32(pb + phy::CAL_CON5, 1); // wrtrn_cyc_mode = 0
    resync(io, ch);
}

/// PHY receiver Vref of both data slices (ZQ_CON9 zq_ds*_vref, 6-bit
/// VSEL code, reset 0x14; the code-to-voltage table is not in the TRM).
pub fn set_rx_vref<H: RegIo + ?Sized>(io: &mut H, ch: usize, code: u8) {
    let c = (code & 0x3f) as u32;
    io.clrset32(base::DDRPHY[ch] + phy::ZQ_CON9, 0x3f3f, (c << 8) | c);
}
