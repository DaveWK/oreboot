// SPDX-License-Identifier: GPL-2.0-only
//! RK3588 register map used by the YY3588 bt0.
//!
//! Sources, cited per item: "P1 p.N" / "P2 p.N" = RK3588 TRM Part 1 / Part 2,
//! V1.0 (2022-03-09), page N; "U-Boot <file>" = mainline U-Boot (GPL-2.0+);
//! "Linux <file>" = mainline Linux (GPL-2.0). Nothing here comes from a
//! Rockchip binary.

/// Rockchip "write-masked" register value: write enables in [31:16] for
/// the field `[lo + width - 1 : lo]`, value in [15:0].
pub const fn wm(lo: u32, width: u32, value: u32) -> u32 {
    let mask = ((1u32 << width) - 1) << lo;
    (mask << 16) | ((value << lo) & mask)
}

/// Base addresses, P1 Table 1-1 pp.16-18.
pub mod base {
    pub const DDRCTL: [usize; 4] = [0xF700_0000, 0xF800_0000, 0xF900_0000, 0xFA00_0000];
    pub const PMU1_GRF: usize = 0xFD58_A000;
    pub const DDR01_GRF: usize = 0xFD59_C000;
    pub const DDR23_GRF: usize = 0xFD59_D000;
    pub const CENTER_GRF: usize = 0xFD59_E000;
    pub const PMU1_IOC: usize = 0xFD5F_0000;
    pub const PMU2_IOC: usize = 0xFD5F_4000;
    pub const BUS_IOC: usize = 0xFD5F_8000;
    pub const CRU: usize = 0xFD7C_0000;
    pub const DDR_CRU: [usize; 4] = [0xFD80_0000, 0xFD80_4000, 0xFD80_8000, 0xFD80_C000];
    /// DDR schedulers, P2 Chapter 1 §1.3 (p.17): 0xFE000000 + 0x2000 * ch.
    pub const DDRSCH: [usize; 4] = [0xFE00_0000, 0xFE00_2000, 0xFE00_4000, 0xFE00_6000];
    pub const DDRPHY: [usize; 4] = [0xFE0C_0000, 0xFE0D_0000, 0xFE0E_0000, 0xFE0F_0000];
    pub const EMMC: usize = 0xFE2E_0000;
    pub const SPI2: usize = 0xFEB2_0000;
    pub const UART2: usize = 0xFEB5_0000;
    pub const SYSTEM_SRAM: usize = 0xFF00_0000;
    pub const PMU_SRAM: usize = 0xFF10_0000;
}

/// The GRF of a channel (DDR01_GRF for 0/1, DDR23_GRF for 2/3) and whether
/// the channel is the GRF's "CHB" (odd channel).
pub const fn ddr_grf(ch: usize) -> (usize, bool) {
    (
        if ch < 2 {
            base::DDR01_GRF
        } else {
            base::DDR23_GRF
        },
        ch & 1 == 1,
    )
}

/// DDR GRF, P1 Chapter 6 §6.9 (p.741-760).
pub mod ddr_grf {
    /// DDR_GRF_CHA_CON2 / CHB_CON2 (0x0008 / 0x0038), p.743.
    pub const CHA_CON2: usize = 0x0008;
    pub const CHB_CON2: usize = 0x0038;
    /// DDR_GRF_CHA_PHY_CON0 / CHB_PHY_CON0 (0x0130 / 0x0134), the TRM's
    /// "DDR_GRF_PHY_CON0/1" of §2.6.1. Fields: ddrphy2xclkgate_enable [3],
    /// dfi_init_complete_sel [4], dfi_init_complete [5] (p.~770).
    pub const CHA_PHY_CON0: usize = 0x0130;
    pub const CHB_PHY_CON0: usize = 0x0134;
    pub const PHY2XCLKGATE_EN_BIT: u32 = 3;
    pub const DFI_INIT_COMPLETE_SEL_BIT: u32 = 4;
    pub const DFI_INIT_COMPLETE_BIT: u32 = 5;
    /// DDR_GRF_CHA_PHY_STATUS0 / CHB (0x0138 / 0x013C).
    pub const CHA_PHY_STATUS0: usize = 0x0138;
    pub const CHB_PHY_STATUS0: usize = 0x013C;
}

/// CENTER_GRF_SOC_CON4 (0x0010), P1 p.761; P2 §1.3.5 p.18: [8:4] = 0 for
/// 4-channel interleave (the reset value 0x0000C000 has them 0).
pub const CENTER_GRF_SOC_CON4: usize = base::CENTER_GRF + 0x0010;

/// CENTER_GRF_CON4 [8:4] channel address-space codes, P2 §1.3.5 Table 1-5
/// (p.20): 0 = 4-channel interleave; otherwise channel `c` starts at
/// `c * size` for the per-channel size the code names (MiB, code).
pub const CHANNEL_LAYOUT: [(u64, u32); 8] = [
    // 8 GiB per channel shares code 0 with the interleave mode (the TRM
    // prints both), so it is not a linear choice; msch::test_window tests
    // 8 GiB channels through the 4 GiB code instead [I]. The 5'b1xxxx codes
    // (channels 2/3 only) are not listed.
    (6144, 0b00101),
    (4096, 0b00001),
    (3072, 0b00110),
    (2048, 0b00010),
    (1536, 0b01000),
    (1024, 0b00011),
    (512, 0b00100),
    (256, 0b00111),
];

/// DDR0..3 CRU, P1 Chapter 2 §2.5 (pp.262-265); same layout per channel.
pub mod ddr_cru {
    /// DxBPLL_CON0..6 (0x0020..0x0038). CON0: bp [15], m [9:0];
    /// CON1: resetb [13], s [8:6], p [5:0]; CON2: k [15:0]; CON6: lock [15].
    pub const BPLL_CON0: usize = 0x0020;
    pub const BPLL_CON1: usize = 0x0024;
    pub const BPLL_CON2: usize = 0x0028;
    pub const BPLL_CON6: usize = 0x0038;
    pub const PLL_LOCK_BIT: u32 = 15;
    /// CLKSEL_CON00 (0x0300): clk_ddrphy2x_chN_sel [0], 0 = DxAPLL (24 MHz
    /// default), 1 = DxBPLL. p.264.
    pub const CLKSEL_CON00: usize = 0x0300;
    /// SOFTRST_CON00 (0x0A00), 1 = in reset. presetn_ddrphy [4],
    /// presetn_ddr_cru [3], resetn_ddrphy2x [2] (reset value 1),
    /// resetn_ddrphy2xdiv = div_rst_n [1] (reset value 1). p.265.
    pub const SOFTRST_CON00: usize = 0x0A00;
    /// GATE_CON00 (0x0800), 1 = clock gated: clk_osc_ddrphy [5],
    /// pclk_ddrphy [4], pclk_ddr_cru [3]; all 0 at reset. P1 p.264.
    pub const GATE_CON00: usize = 0x0800;
    pub const RST_PRESETN_PHY: u32 = 4;
    pub const RST_PHY2X: u32 = 2;
    pub const RST_PHY2XDIV: u32 = 1;
}

/// Main CRU resets of the DDR controllers (DDRCTL = "ddr_upctl"), 1 = in
/// reset, all 0 after power-on. P1 Chapter 2 SOFTRST_CON20..24 (0x0A50..0x0A60).
/// `sbr` = resetn_sbr_chN (the TRM's "sbr_resetn", §2.6.1 steps 7/15);
/// `dfi` = resetn_dfi_chN, reset value 1 (held in reset at power-on), taken
/// as the TRM's "rst_n of DDRPHY" (step 8) [I]. P1 pp.~95-98 (field
/// tables of CRU_SOFTRST_CON20/21/23/24).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CtlResets {
    pub reg: usize,
    pub presetn: u32,
    pub core: u32,
    pub aresetn: u32,
    pub sbr: u32,
    pub dfi: u32,
}
pub const CTL_RESETS: [CtlResets; 4] = [
    CtlResets {
        reg: 0x0A50,
        presetn: 3,
        core: 8,
        aresetn: 12,
        sbr: 7,
        dfi: 6,
    },
    CtlResets {
        reg: 0x0A54,
        presetn: 0,
        core: 4,
        aresetn: 8,
        sbr: 3,
        dfi: 2,
    },
    CtlResets {
        reg: 0x0A5C,
        presetn: 3,
        core: 8,
        aresetn: 12,
        sbr: 7,
        dfi: 6,
    },
    CtlResets {
        reg: 0x0A60,
        presetn: 0,
        core: 4,
        aresetn: 8,
        sbr: 3,
        dfi: 2,
    },
];

/// PMU1 GRF OS_REG2..5: the DRAM geometry handoff. U-Boot
/// arch/arm/include/asm/arch-rockchip/grf_rk3588.h (os_reg[8] at +0x200);
/// edk2-rk3588 SdramLib.c reads 0xFD58A208 + 8 * bank.
pub const OS_REG2: usize = base::PMU1_GRF + 0x208;
pub const OS_REG3: usize = base::PMU1_GRF + 0x20C;
pub const OS_REG4: usize = base::PMU1_GRF + 0x210;
pub const OS_REG5: usize = base::PMU1_GRF + 0x214;

/// DDRCTL (Synopsys DWC DDRCTL register set). Offsets from the channel
/// base, P2 §2.4.1 (pp.57-108) and field pages as cited. The per-frequency
/// set 0 (FREQ0) registers sit at +0x0000, the main block at +0x10000,
/// ADDRMAP at +0x30000, the AXI port at +0x20000.
pub mod ctl {
    pub const MSTR0: usize = 0x1_0000; // p.108: lpddr4 [1], lpddr5 [3], data_bus_width [13:12], burst_rdwr [20:16], active_ranks [25:24]
    pub const MSTR2: usize = 0x1_0008;
    pub const STAT: usize = 0x1_0014; // operating_mode [2:0], 1 = Normal (p.~110)
    pub const MRCTRL0: usize = 0x1_0080; // mr_wr [31], mrr_done_clr [24], mr_rank [5:4], sw_init_int [3], mr_type [0] (p.112)
    pub const MRCTRL1: usize = 0x1_0084; // LPDDR4: [15:8] MR address, [7:0] data (p.114)
    pub const MRSTAT: usize = 0x1_0090; // mrr_done [16], mr_wr_busy [0] (p.114)
    pub const MRRDATA0: usize = 0x1_0094; // p.115
    pub const PWRCTL: usize = 0x1_0180; // all fields 0 at reset (p.~120)
    pub const RFSHCTL0: usize = 0x1_0208; // dis_auto_refresh [0] (p.123)
    pub const DFIMISC: usize = 0x1_0510; // dfi_init_complete_en [0], dfi_init_start [5], dfi_frequency [12:8] (p.137)
    pub const DFISTAT: usize = 0x1_0514; // dfi_init_complete [0] (p.138)
    pub const SWCTL: usize = 0x1_0C80; // sw_done [0] (p.163)
    pub const SWSTAT: usize = 0x1_0C84; // sw_done_ack [0] (p.164)
    pub const INITTMG0: usize = 0x1_0D00; // pre_cke_x1024 [12:0], post_cke_x1024 [25:16], skip_dram_init [31:30] (p.168)
    pub const INITTMG1: usize = 0x1_0D04; // dram_rstn_x1024 [25:16] (p.169)
    pub const PCTRL: usize = 0x2_0090; // port_en [0] (p.173)
    /// ADDRMAP1 (cs_bit0) .. ADDRMAP12 at 0x30000 + 4n; there is no
    /// ADDRMAP0 or ADDRMAP2 on RK3588 (P2 pp.181-190).
    pub const ADDRMAP_BASE: usize = 0x3_0000;
    pub const HWLPCTL: usize = 0x1_0184; // hw_lp_exit_idle_en [1], hw_lp_en [0], reset 0x3 (p.121)
    pub const RFSHMOD0: usize = 0x1_0200; // per_bank_refresh [8], refresh_burst [5:0] (p.123)
    pub const ZQCTL0: usize = 0x1_0280; // dis_auto_zq [31] (p.124)
    pub const DFIUPD0: usize = 0x1_0508; // dis_auto_ctrlupd [31], dfi_phyupd_en [15] (p.136)
    pub const DFIPHYMSTR: usize = 0x1_0518; // dfi_phymstr_en [0] (p.138)
    pub const RANKCTL: usize = 0x1_0C90; // max_rank_wr [15:12], max_rank_rd [3:0] (p.164)
    pub const DBICTL: usize = 0x1_0C94; // rd_dbi_en [2], wr_dbi_en [1], dm_en [0] reset 1 (p.165)
    pub const ODTMAP: usize = 0x1_0C9C; // rank1 rd/wr [13:12]/[9:8], rank0 [5:4]/[1:0] (p.166)
    pub const MRRDATA1: usize = 0x1_0098;
    pub const PWRCTL_SELFREF_SW: u32 = 1 << 11; // PWRCTL.selfref_sw (DWC field position, crate dwc-ddrctl)
    pub const STAT_MODE_SELFREF: u32 = 3;
    pub const DFIUPD0_DIS_AUTO_CTRLUPD: u32 = 1 << 31;
    pub const ZQCTL0_DIS_AUTO_ZQ: u32 = 1 << 31;
    pub const DRAMSET1TMG12: usize = 0x0030; // t_cmdcke [19:16] (p.79)
    pub const DRAMSET1TMG13: usize = 0x0034; // odtloff [30:24], t_ccd_mw [22:16], t_ppd [3:0] (p.80)
    pub const DRAMSET1TMG14: usize = 0x0038; // t_osco [24:16], t_xsr [11:0] (p.80)
    pub const DRAMSET1TMG25: usize = 0x0064; // lpddr4_diff_bank_rwa2pre [18:16], wra2pre [15:8], rda2pre [7:0] (p.82)
    pub const ZQSET1TMG0: usize = 0x0800; // t_zq_short_nop [25:16], t_zq_long_nop [13:0] (p.99)
    pub const ZQSET1TMG1: usize = 0x0804; // t_zq_reset_nop [29:20], t_zq_short_interval_x1024 [19:0] (p.99)
    pub const TMGCFG: usize = 0x0D00; // frequency_ratio [0]: 0 = 1:2 (p.105)
    /// Frequency set `fs` (0..3) register `off`: FREQn at n * 0x100000
    /// (P2 register summary, e.g. INITMR0_FREQ1 at 0x100500).
    pub const fn freq(fs: usize, off: usize) -> usize {
        fs * 0x10_0000 + off
    }

    /// FREQ0 frequency set (+0x0000), pp.69-97. Units: DRAM clock cycles
    /// unless noted (p.69 "Unit: DRAM clock cycles").
    pub const fn dramset1tmg(n: usize) -> usize {
        n * 4
    }
    pub const INITMR0: usize = 0x0500; // [31:16] MR1, [15:0] MR2 (LPDDR4), p.83
    pub const INITMR1: usize = 0x0504; // [31:16] MR3, [15:0] MR13, p.84
    pub const INITMR2: usize = 0x0508; // [31:16] MR11 (mr4 field), [15:0] MR12 (mr5 field) — see timing.rs
    pub const INITMR3: usize = 0x050C; // [31:16] MR22, [15:0] MR14 (mr6 field)
    pub const DFITMG0: usize = 0x0580; // tphy_wrlat [5:0], tphy_wrdata [13:8], t_rddata_en [22:16], t_ctrl_delay [28:24] (p.85)
    pub const DFITMG1: usize = 0x0584;
    pub const RFSHSET1TMG0: usize = 0x0600; // t_refi_x1_x32 [11:0], refresh_to_x1_x32 [21:16], refresh_margin [27:24] (p.95)
    pub const RFSHSET1TMG1: usize = 0x0604; // t_rfc_min [11:0] (p.97)

    pub const MRCTRL0_MR_WR: u32 = 1 << 31;
    pub const MRCTRL0_MRR_DONE_CLR: u32 = 1 << 24;
    pub const MRCTRL0_READ: u32 = 1 << 0;
    pub const MRSTAT_MRR_DONE: u32 = 1 << 16;
    pub const MRSTAT_WR_BUSY: u32 = 1 << 0;
    pub const DFIMISC_INIT_COMPLETE_EN: u32 = 1 << 0;
    pub const DFIMISC_INIT_START: u32 = 1 << 5;
    pub const STAT_MODE_MASK: u32 = 0x7;
    pub const STAT_MODE_NORMAL: u32 = 1;
}

/// DDRPHY, P2 §2.4.3-2.4.4 (pp.314-555). Field positions as cited.
pub mod phy {
    pub const GNR_CON0: usize = 0x0000; // ctrl_ddr_mode [25:24], ctrl_twpre [15], ctrl_bstlen [13:8], ctrl_cmosrcv [6], ctrl_rdlat_rank0 [5:0] (p.314)
    pub const CAL_CON0: usize = 0x0004; // dvfs_wr_train_en [26], cal_vtc_en [22], ca_swap_mode_dis [21], wrtrn_dbi_cal_en [17], wr_cal_mode [5], ca_cal_mode [2], gate_cal_mode [1] (p.317)
    pub const CAL_CON2: usize = 0x000C; // ctrl_rodt_disable [30] (p.321)
    pub const CAL_CON3: usize = 0x0010; // auto_dqs_clean [11], wr_sw_mode [9], wrlvl_sw_mode [7] (p.324)
    pub const CAL_CON4: usize = 0x0014;
    pub const LP_CON0: usize = 0x0018; // ds_io_pd [14], scheduler_HW_clock_gating_disable [13], ctrl_scheduler_en [6] (p.328)
    pub const OFFSETD_CON0: usize = 0x0050; // upd_mode [28], ctrl_resync [24] (p.334)
    pub const MDLL_CON0: usize = 0x00B0; // clkm_cg_en_sw [23], ctrl_force [18:8], ctrl_start [6], ctrl_dll_on [5] (p.344)
    pub const MDLL_CON1: usize = 0x00B4; // ctrl_locked [18], ctrl_lock_value [10:0] (p.345)
    pub const DVFS_CON: usize = 0x00B8; // freq_train [11:0] (p.345)
    pub const CAL_FAIL_STAT0: usize = 0x00E8; // dq_fail_status1/0 [15:8]/[7:0] (p.357)
    pub const CAL_CON5: usize = 0x0160; // wrtrn_cyc_th [9:2], wrtrn_cyc_en [1], wrtrn_cyc_mode [0] (p.359)
    pub const ZQ_CON0: usize = 0x03C8; // zq_clk_div_en [18], zq_manual_str [1] (p.372)
    pub const ZQ_CON1: usize = 0x03CC; // zq_done [0], zq_mon (fail) [2] (p.374)
    pub const TESTIRCV_CON0: usize = 0x0400; // dqs1_testircv [7:4], dqs0_testircv [3:0] (p.378)
    pub const CBT_CON0: usize = 0x0600; // cbt_sw_mode [31], cbt_vref_dqs_en [26:25], cbt_vref_dq_en [22:21], cbt_cs_coarse_dis [11], cbt_ca_vref [7:0] (p.391)
    pub const CBT_CON2: usize = 0x0608;
    pub const CBT_CON3: usize = 0x060C; // cbt_cke [23:22], cbt_hw_cal_start [1] (p.393)
    pub const RANK1_CON0: usize = 0x0A1C; // ctrl_rdlat_rank1 [5:0]
    pub const SCHD_TRAIN_CON0: usize = 0x0A24; // see train.rs (p.485)
    pub const CLKMODE_CON: usize = 0x0A94; // ctrl_phy_clk_2x [3], ctrl_phy_mode [2:0] (p.501)
    pub const CBT_CAL_STAT0: usize = 0x0BC4; // cbt_hw_cal_done [25], cbt_fail_status [16:9] (p.519)
    pub const CBT_CON5: usize = 0x05F4; // cbt_tDSTRAIN [20:16], cbt_tDHTRAIN [14:10] (p.391)
    pub const GATE_CON0: usize = 0x001C; // rank_en [25:24] reset 3 (p.330)
    pub const WR_LVL_CON0: usize = 0x006C; // ctrl_wrlvl1_code [25:16], ctrl_wrlvl0_code [9:0] (p.336)
    pub const WR_LVL_CON2: usize = 0x0074; // wlcal_fail [5:4] RO (p.337)
    pub const CA_DESKEW_CON4: usize = 0x008C; // CKDeSkewCode [25:16] (p.339)
    pub const CA_DESKEW_CON5: usize = 0x0090; // CS1DeSkewCode [25:16], CS0DeSkewCode [9:0] (p.339)
    pub const CAL_RD_PATTERN_CON0: usize = 0x00AC; // inv ds0 [31:24], inv ds1 [23:16], pattern0 [15:8], pattern1 [7:0] (p.343)
    pub const CAL_WRLVL_STAT: usize = 0x00E4; // wrlvl_fail_status [17:16] (p.356)
    pub const VERSION_INFO_STAT0: usize = 0x03AC; // reset 0x09080005 (p.372)
    pub const ZQ_CON9: usize = 0x03EC; // zq_ds1_vref [13:8], zq_ds0_vref [5:0], reset 0x14 each (p.377)
    pub const SCHD_TIME_CON0: usize = 0x0A2C; // tMRW [23:18], tMRR [17:14], phy_t_rddata_en [10:6], WL [5:0] (p.486)
    pub const SCHD_TIME_CON1: usize = 0x0A30; // tNOP [31:28], tWTR [27:22], tRCD [21:14], tRTP [13:8], tCCD [7:0] (p.487)
    pub const OP_CODE_RDC: usize = 0x0A7C; // PATB [31:24], PATA [23:16], INVB [15:8], INVA [7:0] (p.498)
    pub const LP4_MR_OP0: usize = 0x0A80; // MR12 [31:24], MR11 [23:16], MR2 [15:8], MR1 [7:0] (p.498)
    pub const LP4_MR_OP1: usize = 0x0A84; // MR13 [7:0] (p.499)
    pub const CAL_CON4_LP4_HIGH_DENSITY: u32 = 1 << 31; // R17/R18 in ACT2 (p.326)
    pub const CAL_CON4_NUM_REPEAT_SHIFT: u32 = 6; // num_repeat [10:6]
    pub const CAL_CON0_DVFS_WR_TRAIN_EN: u32 = 1 << 26;
    pub const CAL_CON0_GATE_CAL_MODE: u32 = 1 << 1;
    pub const CAL3_WR_SW_MODE: u32 = 1 << 9;
    pub const CAL3_WRLVL_SW_MODE: u32 = 1 << 7;
    pub const CBT_SW_MODE: u32 = 1 << 31;
    pub const CBT_CS_COARSE_DIS: u32 = 1 << 11;

    pub const MDLL_DLL_ON: u32 = 1 << 5;
    pub const MDLL_START: u32 = 1 << 6;
    pub const MDLL_CLKM_CG_EN_SW: u32 = 1 << 23;
    pub const MDLL_LOCKED: u32 = 1 << 18;
    pub const OFFSETD_RESYNC: u32 = 1 << 24;
    pub const OFFSETD_UPD_MODE: u32 = 1 << 28;
    pub const ZQ_CLK_DIV_EN: u32 = 1 << 18;
    pub const ZQ_MANUAL_STR: u32 = 1 << 1;
    pub const ZQ_DONE: u32 = 1 << 0;
    pub const ZQ_FAIL: u32 = 1 << 2;
    pub const LP_SCHEDULER_EN: u32 = 1 << 6;
    pub const LP_SCHED_HW_CG_DIS: u32 = 1 << 13;
    pub const LP_DS_IO_PD: u32 = 1 << 14;
    pub const CAL_VTC_EN: u32 = 1 << 22;
    pub const CAL_CA_SWAP_MODE_DIS: u32 = 1 << 21;
    pub const CAL_CA_CAL_MODE: u32 = 1 << 2;
    pub const CAL_GATE_CAL_MODE: u32 = 1 << 1;
    pub const CAL_WRTRN_DBI_CAL_EN: u32 = 1 << 17;
    pub const CAL3_AUTO_DQS_CLEAN: u32 = 1 << 11;
    pub const CAL2_RODT_DISABLE: u32 = 1 << 30;
    pub const CBT_HW_CAL_START: u32 = 1 << 1;
    pub const CBT_HW_CAL_DONE: u32 = 1 << 25;
}

/// DDRSCH (per-channel DDR scheduler), P2 §1.4 pp.21-24.
pub mod ddrsch {
    /// DeviceConf [5:0]: predefined ddrConf selector (p.21).
    pub const DEVICE_CONF: usize = 0x0008;
    /// DeviceSize: Rank1 [15:8], Rank0 [7:0], granule 64 MiB (p.22).
    pub const DEVICE_SIZE: usize = 0x000C;
}

/// IOC iomux (write-masked), P1 Chapter 6.
pub mod ioc {
    /// PMU1_IOC_GPIO0A_IOMUX_SEL_H (+0x4): gpio0a5 [7:4], gpio0a6 [11:8] = 1 → SPI2_CLK_M2 / MOSI_M2 (p.957).
    pub const PMU1_GPIO0A_SEL_H: usize = 0x0004;
    /// PMU1_IOC_GPIO0A_IOMUX_SEL_L (+0x0): gpio0a2 [11:8], gpio0a3 [15:12]
    /// (P1 p.957); function 1 is PMIC_SLEEP1/2 (vendor rk3588s-pinctrl.dtsi
    /// pmic_pins). Read only, for the RK806 PWRCTRL report.
    pub const PMU1_GPIO0A_SEL_L: usize = 0x0000;
    /// PMU1_IOC_GPIO0B_IOMUX_SEL_L (+0x8): gpio0b1 [7:4] = 1 → SPI2_CS0_M2, gpio0b3 [15:12] = 1 → SPI2_MISO_M2 (p.958;
    /// pin functions also Linux arch/arm64/boot/dts/rockchip/rk3588-base-pinctrl.dtsi spi2m2_*).
    pub const PMU1_GPIO0B_SEL_L: usize = 0x0008;
    /// PMU1_IOC_GPIO0A_DS_H / GPIO0B_DS_L (+0x14 / +0x18): 2-bit drive
    /// field per pin at [4n+1:4n] (a5 [5:4], a6 [9:8]; b1 [5:4], b3
    /// [13:12]), reset 0x1111 = 1 per pin (P1 pp.959-960). Linux writes
    /// `drive-strength` as is on RK3588 (vendor drivers/pinctrl/
    /// pinctrl-rockchip.c:4163 rockchip_set_drive_perpin, rk3588_ds_regs
    /// :3831), so the DTS pcfg_pull_up_drv_level_1 is the reset value.
    pub const PMU1_GPIO0A_DS_H: usize = 0x0014;
    pub const PMU1_GPIO0B_DS_L: usize = 0x0018;
    /// PMU1_IOC_GPIO0A_P / GPIO0B_P (+0x20 / +0x24): per pin pe [2n]
    /// (enable) and ps [2n+1] (1 = up, 0 = down), so a 2-bit field of 3 is
    /// pull-up, 1 pull-down, 0 none (P1 pp.961-963). Reset 0xC751 / 0x0030:
    /// a5 pull-down, a6/b1/b3 none. Linux agrees: rk3588_p_regs (vendor
    /// pinctrl-rockchip.c:3874) and rockchip_pull_list[PULL_TYPE_IO_1V8_ONLY]
    /// (:4341) = {none, down, none, up}.
    pub const PMU1_GPIO0A_P: usize = 0x0020;
    pub const PMU1_GPIO0B_P: usize = 0x0024;
    pub const PULL_UP: u32 = 3;
    pub const PULL_DOWN: u32 = 1;
    /// PMU1_IOC_GPIO0A_IE / GPIO0B_IE (+0x28 / +0x2C): input enable, bit n
    /// per pin, reset 0xFF / 0x0F (P1 pp.963-964).
    pub const PMU1_GPIO0A_IE: usize = 0x0028;
    pub const PMU1_GPIO0B_IE: usize = 0x002C;
    /// BUS_IOC_GPIO1A_IOMUX_SEL_H (+0x24): gpio1a4 [3:0] = 8 is SPI2_MISO_M0
    /// (P1 p.988); BUS_IOC_GPIO4A_IOMUX_SEL_H (+0x84): gpio4a4 [3:0] = 8 is
    /// SPI2_MISO_M1 (P1 p.1003). Read only: another pad routed to the same
    /// SPI2 input would compete with GPIO0_B3.
    pub const BUS_GPIO1A_SEL_H: usize = 0x0024;
    pub const BUS_GPIO4A_SEL_H: usize = 0x0084;
}

/// GPIO0 (GPIO v2) at 0xFD8A0000 (Linux rk3588-base.dtsi gpio@fd8a0000).
/// Vendor Linux drivers/gpio/gpio-rockchip.c:94-110 gpio_regs_v2:
/// SWPORT_DR_L +0x00 and SWPORT_DDR_L +0x08 (pins 0-15, write enables in
/// [31:16], rockchip_gpio_writel_bit :172), EXT_PORT +0x70 (pad levels),
/// VERSION_ID +0x78. Used by the diagnostics only.
pub const GPIO0: usize = 0xFD8A_0000;
pub const GPIO_DR_L: usize = 0x0000;
pub const GPIO_DDR_L: usize = 0x0008;
pub const GPIO_EXT_PORT: usize = 0x0070;
pub const GPIO_VERSION_ID: usize = 0x0078;

/// CRU_CLKSEL_CON59 (0x03EC): clk_spi2_sel [7:6], 2 = xin_osc0_func (P1 p.79;
/// Linux drivers/clk/rockchip/clk-rk3588.c CLK_SPI2, mux_200m_150m_24m_p).
pub const CRU_CLKSEL_CON59: usize = base::CRU + 0x03EC;
/// CRU_GATE_CON14 (0x0838), 1 = clock gated: clk_spi2_en [13], pclk_spi2_en [8];
/// reset value 0 (P1 p.123).
pub const CRU_GATE_CON14: usize = base::CRU + 0x0838;
/// CRU_SOFTRST_CON14 (0x0A38), 1 = in reset: resetn_spi2 [13], presetn_spi2 [8];
/// reset value 0 (P1 p.174).
pub const CRU_SOFTRST_CON14: usize = base::CRU + 0x0A38;
/// CRU_GATE_CON17 (0x0844), 1 = gated: pclk_gpio3_en [2]; CRU_GATE_CON72
/// (0x0920): pclk_usb2phy_u3_0_grf0_en [8] (USB2PHY0_GRF); reset 0 (P1
/// pp.126-127, Chapter 2 CRU). Read only, before the carrier-rail probe
/// touches GPIO3 / USB2PHY0_GRF.
pub const CRU_GATE_CON17: usize = base::CRU + 0x0844;
pub const CRU_GATE_CON72: usize = base::CRU + 0x0920;
pub const PCLK_GPIO3_BIT: u32 = 2;
pub const PCLK_USB2PHY0_GRF_BIT: u32 = 8;
/// GPIO3 (Linux rk3588-base.dtsi gpio@fec40000), same register layout as
/// [`GPIO0`].
pub const GPIO3: usize = 0xFEC4_0000;
/// USB2PHY0_GRF (P1 address map, 0xFD5D0000); +0xC0 status, utmi
/// session valid [11] (as the YY3588 EDK2 rail probe reads it).
pub const USB2PHY0_GRF: usize = 0xFD5D_0000;
pub const USB2PHY_GRF_STATUS0: usize = 0x00C0;
pub const SPI2_CLK_BIT: u32 = 13;
pub const SPI2_PCLK_BIT: u32 = 8;

/// Rockchip SPI controller, P1 Chapter 31 pp.1928-1947; register and
/// field names as Linux drivers/spi/spi-rockchip.c.
pub mod spi {
    pub const CTRLR0: usize = 0x0000; // fields below (pp.1932-1934)
    pub const CTRLR1: usize = 0x0004;
    pub const ENR: usize = 0x0008;
    pub const SER: usize = 0x000C;
    pub const BAUDR: usize = 0x0010;
    pub const TXFLR: usize = 0x001C;
    pub const RXFLR: usize = 0x0020;
    pub const SR: usize = 0x0024; // bsf [0], tfe [2], rfe [3] (p.1935)
    pub const IMR: usize = 0x002C;
    /// SPI_ISR / SPI_RISR: masked / raw interrupt status, tfe [0], tfo
    /// [1], rfu [2], rfo [3], rff [4] (P1 pp.1937-1938).
    pub const ISR: usize = 0x0030;
    pub const RISR: usize = 0x0034;
    /// SPI_VERSION (0x0048): 0x00110002 on RK3588 (Linux spi-rockchip.c
    /// ROCKCHIP_SPI_VER2_TYPE2) [I].
    pub const VERSION: usize = 0x0048;
    pub const TXDR: usize = 0x0400;
    pub const RXDR: usize = 0x0800;
    /// CTRLR0.dfs [1:0] = 1: 8-bit data frames (p.1934; reset value 2 = 16 bit).
    pub const CR0_DFS_8BIT: u32 = 1 << 0;
    /// CTRLR0.ssd [10] = 1: one sclk_out cycle between ss_n active and the
    /// first clock edge (p.1933).
    pub const CR0_SSD_ONE: u32 = 1 << 10;
    /// CTRLR0.em [11] = 1: big endian serial (p.1933).
    pub const CR0_EM_BIG: u32 = 1 << 11;
    /// CTRLR0.bht [13] = 1: "APB 8bit write/read, spi 8bit write/read";
    /// 0 (reset value) is "APB 16bit write/read": one TXDR/RXDR access is
    /// two 8-bit frames (p.1933).
    pub const CR0_BHT_8BIT: u32 = 1 << 13;
    /// CTRLR0.scph [6] / scpol [7] (SPI mode = scpol << 1 | scph), lbk
    /// [25] loopback, rxd = txd inside the controller (pp.1932-1933).
    pub const CR0_MODE_SHIFT: u32 = 6;
    pub const CR0_LBK: u32 = 1 << 25;
    /// CTRLR0.xfm [19:18]: 0 transmit+receive, 1 transmit only, 2 receive
    /// only (Linux spi-rockchip.c CR0_XFM_OFFSET 18, CR0_XFM_TR/TO/RO).
    /// In receive only the controller clocks CTRLR1 + 1 frames once ENR
    /// is set, without TX FIFO data (rockchip_spi_config() writes CTRLR1 =
    /// len - 1; rockchip_spi_prepare_irq() writes nothing to TXDR).
    pub const CR0_XFM_SHIFT: u32 = 18;
    pub const CR0_XFM_MASK: u32 = 3 << 18;
    pub const XFM_TR: u32 = 0;
    pub const XFM_RO: u32 = 2;
    pub const SR_BUSY: u32 = 1 << 0;
    pub const SR_TFE: u32 = 1 << 2;
    pub const SR_RFE: u32 = 1 << 3;
}

/// The main-CRU DDRCTL reset bits of channel `ch`.
pub const fn ctl_resets_of(ch: usize) -> CtlResets {
    CTL_RESETS[ch]
}
