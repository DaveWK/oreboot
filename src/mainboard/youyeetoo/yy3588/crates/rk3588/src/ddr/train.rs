// SPDX-License-Identifier: GPL-2.0-only
//! DDRPHY hardware training, RK3588 TRM Part 2 §2.6.3-2.6.7 (pp.562-569).
//!
//! - [`prepare`]: §2.6.3 settings before training (PHY-initiated update
//!   mode, DVFS_CON.freq_train, DLL lock value frozen), the §2.6.6 read
//!   pattern (CAL_RD_PATTERN_CON0 0xEEEE5555 with OP_CODE_RDC 0x5555EEEE,
//!   pp.343/498) and the §2.6.7 write-training mode for the clock.
//! - [`run`]: one run of the PHY training scheduler (SCHD_TRAIN_CON0,
//!   p.485) with write levelling, gate (unless auto DQS clean), read DQ and
//!   write DQ for the enabled ranks; phy_train_en / phy_train_done is how
//!   the RK3576 RFC drives the same engine (phy_train()).
//! - [`gate_probe`]: gate training on one rank only, the rank-presence test
//!   of the RK3568 RFC (Pavel Golikov, GPL-2.0+, sdram_rk3568.c
//!   dram_detect_cap(): "data_training(dram, 1, ..., READ_GATE_TRAINING)
//!   == 0" means CS1 exists), ported to this PHY's engine.
//! - [`cbt_hw`]: §2.6.4 hardware command bus training (optional, off by
//!   default at <= 1066 MHz).

use super::timing::Plan;
use super::{seq, Config, DqsClean};
use crate::regs::{base, phy};
use regio::{wait, RegIo};

// SCHD_TRAIN_CON0 fields (p.485).
pub const TRAIN_EN: u32 = 1 << 0;
pub const TRAIN_DONE: u32 = 1 << 1;
pub const CBT_EN: u32 = 1 << 2;
pub const WRLVL_EN: u32 = 1 << 3;
pub const GTTRN_EN: u32 = 1 << 4;
pub const RDTRN_EN: u32 = 1 << 5;
pub const WRTRN_EN: u32 = 1 << 6;

/// Ranks bit mask (bit 0 = rank 0) for `ranks` ranks.
pub const fn rank_mask(ranks: u8) -> u32 {
    if ranks >= 2 {
        3
    } else {
        1
    }
}

/// The SCHD_TRAIN_CON0 step and rank-enable bits (without phy_train_en).
pub const fn schd_train(mask: u32, gate: bool) -> u32 {
    let mut v = WRLVL_EN | RDTRN_EN | WRTRN_EN
        | (mask << 8)   // phy_wrlvl_rank_en [9:8]
        | (mask << 12)  // phy_rdtrn_rank_en [13:12]
        | (mask << 14); // phy_wrtrn_rank_en [15:14]
    if gate {
        v |= GTTRN_EN | (mask << 10); // phy_gttrn_rank_en [11:10]
    }
    v
}

/// CAL_CON5 for the write-training clock (§2.6.7): >= 1066 MHz high
/// frequency cycle mode (cyc_en 1, mode 0, threshold 0x1f); 533..1066 MHz
/// low frequency cycle mode (cyc_en 1, mode 1); below 533 MHz cycle mode
/// off (the DVFS-style low-frequency path).
pub const fn cal_con5(ck_mhz: u32) -> u32 {
    if ck_mhz >= 1066 {
        (0x1f << 2) | (1 << 1)
    } else if ck_mhz >= 533 {
        (1 << 1) | 1
    } else {
        0
    }
}

/// §2.6.3/§2.6.6/§2.6.7 settings before any training run.
pub fn prepare<H: RegIo + ?Sized>(io: &mut H, ch: usize, p: &Plan, cfg: &Config) {
    let pb = base::DDRPHY[ch];
    io.clr32(pb + phy::OFFSETD_CON0, phy::OFFSETD_UPD_MODE);
    io.clrset32(pb + phy::DVFS_CON, 0xfff, p.ck_mhz & 0xfff);
    io.set32(pb + phy::MDLL_CON0, phy::MDLL_CLKM_CG_EN_SW);
    io.write32(pb + phy::CAL_RD_PATTERN_CON0, 0xEEEE_5555);
    io.write32(pb + phy::OP_CODE_RDC, 0x5555_EEEE);
    // DM is enabled (DBICTL.dm_en = 1, MR13 OP[5] = 0): wrtrn_dbi_cal_en = 1.
    io.set32(pb + phy::CAL_CON0, phy::CAL_WRTRN_DBI_CAL_EN);
    io.clrset32(pb + phy::CAL_CON5, 0x3ff, cal_con5(p.ck_mhz));
    io.clrset32(
        pb + phy::CAL_CON4,
        0x1f << phy::CAL_CON4_NUM_REPEAT_SHIFT,
        ((cfg.wr_num_repeat & 0x1f) as u32) << phy::CAL_CON4_NUM_REPEAT_SHIFT,
    );
    if p.ck_mhz < 533 {
        io.set32(pb + phy::CAL_CON0, phy::CAL_CON0_DVFS_WR_TRAIN_EN);
        io.clr32(pb + phy::CAL_CON4, 0x1f); // dvfs_wr_start_adj = 0
    }
}

/// Training outcome of one run.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Result {
    /// CAL_FAIL_STAT0 dq_fail_status1/0 (p.357).
    pub dq_fail: u32,
    /// CAL_WRLVL_STAT.wrlvl_fail_status (p.356).
    pub wrlvl_fail: u32,
    /// WR_LVL_CON2.wlcal_fail (p.337).
    pub wlcal_fail: u32,
}

impl Result {
    pub const fn ok(&self) -> bool {
        self.dq_fail == 0 && self.wrlvl_fail == 0 && self.wlcal_fail == 0
    }
}

fn launch<H: RegIo + ?Sized>(
    io: &mut H,
    ch: usize,
    steps: u32,
    timeout_us: u64,
) -> core::result::Result<Result, &'static str> {
    let pb = base::DDRPHY[ch];
    io.write32(pb + phy::SCHD_TRAIN_CON0, steps);
    io.write32(pb + phy::SCHD_TRAIN_CON0, steps | TRAIN_EN);
    io.kick_watchdog();
    let r = wait(io, "SCHD_TRAIN_CON0.phy_train_done", timeout_us, |io| {
        io.read32(pb + phy::SCHD_TRAIN_CON0) & TRAIN_DONE != 0
    });
    io.write32(pb + phy::SCHD_TRAIN_CON0, steps);
    r.map_err(|_| "training: phy_train_done timeout")?;
    Ok(Result {
        dq_fail: io.read32(pb + phy::CAL_FAIL_STAT0) & 0xffff,
        wrlvl_fail: (io.read32(pb + phy::CAL_WRLVL_STAT) >> 16) & 3,
        wlcal_fail: (io.read32(pb + phy::WR_LVL_CON2) >> 4) & 3,
    })
}

/// One full training run of the ranks in `mask`.
pub fn run<H: RegIo + ?Sized>(
    io: &mut H,
    ch: usize,
    mask: u32,
    cfg: &Config,
) -> core::result::Result<Result, &'static str> {
    let gate = matches!(cfg.dqs_clean, DqsClean::Gate);
    launch(io, ch, schd_train(mask, gate), 1_000_000)
}

/// Gate training on `rank` alone: true when it completes without failure
/// (a missing rank never returns a gate edge).
pub fn gate_probe<H: RegIo + ?Sized>(io: &mut H, ch: usize, rank: u8) -> bool {
    let steps = GTTRN_EN | (1 << (10 + rank as u32));
    matches!(launch(io, ch, steps, 100_000), Ok(r) if r.dq_fail == 0)
}

/// 91 ps CK/CS offset in fine-delay steps (§2.6.4): the master DLL lock
/// value spans one clock (MDLL_CON0.ctrl_force text, p.344) [I].
pub fn ck_offset_steps(lock_value: u32, ck_mhz: u32) -> u32 {
    let tck_ps = 1_000_000 / ck_mhz.max(1);
    (91 * lock_value).div_ceil(tck_ps.max(1)).min(0x3ff)
}

/// §2.6.4 hardware command bus training on rank 0 (rank 1 gets rank 0's
/// CS code, as the TRM says). Leaves the channel at the target clock in
/// normal mode. Returns CBT_CAL_STAT0.cbt_fail_status.
pub fn cbt_hw<H: RegIo + ?Sized>(
    io: &mut H,
    ch: usize,
    p: &Plan,
    cfg: &Config,
) -> core::result::Result<u32, &'static str> {
    let pb = base::DDRPHY[ch];
    let lock = io.read32(pb + phy::MDLL_CON1) & 0x7ff;
    let off = ck_offset_steps(lock, p.ck_mhz);
    // Low frequency (the 24 MHz DxAPLL).
    seq::freq_change(io, ch, false, cfg.sr_freq_change)?;
    io.clr32(pb + phy::LP_CON0, phy::LP_DS_IO_PD);
    io.set32(pb + phy::CAL_CON0, phy::CAL_CA_SWAP_MODE_DIS);
    // DQS offset.
    io.set32(pb + phy::CAL_CON3, phy::CAL3_WRLVL_SW_MODE);
    io.write32(pb + phy::WR_LVL_CON0, (off << 16) | off);
    super::phy::resync(io, ch);
    io.clr32(pb + phy::CAL_CON3, phy::CAL3_WRLVL_SW_MODE);
    // DRAM into CBT mode: MR13 OP[0] = 1.
    super::mr::write(io, ch, 0, 13, p.mr13 | 1).map_err(|_| "CBT: MR13 entry")?;
    io.udelay(1); // tMRD
    io.set32(pb + phy::CAL_CON0, phy::CAL_CA_CAL_MODE);
    io.clrset32(pb + phy::CBT_CON0, 3 << 25, 1 << 25); // cbt_vref_dqs_en = 01
    io.udelay(1); // tDQSCKE
    io.clrset32(pb + phy::CBT_CON3, 3 << 22, 0); // cbt_cke = 0 (the TRM text says CBT_CON2; the field is in CBT_CON3, p.393)
    io.udelay(1); // tCKELCK
    seq::freq_change(io, ch, true, false)?;
    // CA Vref onto DQ.
    io.clrset32(pb + phy::CBT_CON0, 0xff, (p.mr12 & 0x7f) as u32);
    io.set32(pb + phy::CAL_CON3, phy::CAL3_WR_SW_MODE);
    io.udelay(1); // tCAENT
    io.clrset32(pb + phy::CBT_CON0, 3 << 21, 1 << 21); // cbt_vref_dq_en = 01
    io.udelay(1); // tDSTRAIN + tDHTRAIN + 6 DFI clocks
    io.clrset32(pb + phy::CBT_CON0, 3 << 21, 0);
    io.clr32(pb + phy::CAL_CON3, phy::CAL3_WR_SW_MODE);
    io.udelay(1); // tVREF_LONG
                  // CK/CS offset, then the hardware CS/CA calibration.
    io.set32(pb + phy::CBT_CON0, phy::CBT_SW_MODE);
    io.clrset32(pb + phy::CA_DESKEW_CON4, 0x3ff << 16, off << 16);
    io.clrset32(pb + phy::CA_DESKEW_CON5, 0x3ff, off);
    super::phy::resync(io, ch);
    io.clr32(pb + phy::CBT_CON0, phy::CBT_SW_MODE);
    io.set32(
        pb + phy::LP_CON0,
        phy::LP_SCHED_HW_CG_DIS | phy::LP_SCHEDULER_EN,
    );
    io.set32(pb + phy::CBT_CON0, phy::CBT_CS_COARSE_DIS);
    io.set32(pb + phy::CBT_CON3, phy::CBT_HW_CAL_START);
    let done = wait(io, "CBT_CAL_STAT0.cbt_hw_cal_done", 100_000, |io| {
        io.read32(pb + phy::CBT_CAL_STAT0) & phy::CBT_HW_CAL_DONE != 0
    });
    let stat = io.read32(pb + phy::CBT_CAL_STAT0);
    io.clr32(pb + phy::LP_CON0, phy::LP_SCHED_HW_CG_DIS);
    // Exit.
    io.clrset32(pb + phy::CBT_CON3, 3 << 22, 3 << 22);
    io.udelay(1); // tCKEHDQS
    io.clrset32(pb + phy::CBT_CON0, 3 << 25, 0);
    io.clr32(pb + phy::CAL_CON0, phy::CAL_CA_CAL_MODE);
    io.clr32(pb + phy::CBT_CON3, phy::CBT_HW_CAL_START);
    io.udelay(1); // tFC
    super::mr::write(io, ch, 0, 13, p.mr13).map_err(|_| "CBT: MR13 exit")?;
    done.map_err(|_| "CBT: cbt_hw_cal_done timeout")?;
    if p.ranks >= 2 {
        let cs0 = io.read32(pb + phy::CA_DESKEW_CON5) & 0x3ff;
        io.clrset32(pb + phy::CA_DESKEW_CON5, 0x3ff << 16, cs0 << 16);
        super::phy::resync(io, ch);
    }
    Ok((stat >> 9) & 0xff)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn train_word() {
        assert_eq!(schd_train(3, true), 0xff78);
        assert_eq!(schd_train(1, true), 0x5578);
        assert_eq!(schd_train(1, false), 0x5168);
    }
    #[test]
    fn write_training_modes() {
        assert_eq!(cal_con5(1056), 0b11);
        assert_eq!(cal_con5(1068), (0x1f << 2) | 2);
        assert_eq!(cal_con5(400), 0);
    }
}
