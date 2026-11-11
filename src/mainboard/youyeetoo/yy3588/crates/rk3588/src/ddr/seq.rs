// SPDX-License-Identifier: GPL-2.0-only
//! Per-channel LPDDR4/4X initialization, RK3588 TRM Part 2 §2.6.1 steps
//! 3-32 (pp.558-561); each step is numbered as in the TRM. Steps 1-2
//! (rails, DxBPLL) are done before (crate::rk806, crate::pll), training
//! (step 33) is crate::ddr::train, step 34 is [`finish`].
//!
//! The TRM writes the sequence for all four channels in lockstep; the
//! channels share nothing but the DDR01/DDR23 GRF registers (one
//! write-masked bit per channel), so one channel is brought up at a time
//! and a failing channel does not stop the others.
//!
//! Reset mapping (P1 CRU tables; the mapping of TRM names to CRU bits is
//! [I] where marked): PRESETn of DDRPHY = DDR CRU presetn_ddrphy;
//! RESETn = resetn_ddrphy2x; div_rst_n = resetn_ddrphy2xdiv (named so,
//! P1 p.265); rst_n of DDRPHY = main CRU resetn_dfi_chN [I] (the only
//! remaining PHY-side reset, asserted at power-on like RESETn); DDRCTL
//! presetn / core_ddrc_rstn / sbr_resetn / aresetn_0 = presetn_ddr_upctl /
//! resetn_ddr_upctl / resetn_sbr / aresetn_ddr_upctl.
//!
//! Two additions to the TRM text, both switchable in [`Config`]:
//! - `sr_freq_change`: software self-refresh around the step-29 clock
//!   switch (DWC DDRCTL convention for software frequency change; the
//!   A733 flow in crate dwc-ddrctl leaves self-refresh after its DFI
//!   init the same way) [I];
//! - `grf_dfi_guard`: hold dfi_init_complete at the DDRCTL from the GRF
//!   (DDR_GRF_CHx_PHY_CON0 [5:4], P1 p.757) while the master DLL powers
//!   on, from the RK3576 RFC (step 31-34 of its TRM §7.6.2) [I for RK3588].

use super::detect::ChannelGeometry;
use super::timing::Plan;
use super::{ctl as dctl, phy as dphy, Config};
use crate::regs::{base, ctl, ctl_resets_of, ddr_cru, ddr_grf, ddr_grf as g, phy, wm};
use regio::{wait, RegIo};

const T: u64 = 100_000;

/// Errors name the step that failed.
pub type StepError = &'static str;

fn grf_phy_con(ch: usize) -> usize {
    let (b, chb) = ddr_grf(ch);
    b + if chb {
        g::CHB_PHY_CON0
    } else {
        g::CHA_PHY_CON0
    }
}

/// DDRPHY2XCLKGATE_ENABLE (DDR_GRF_CHx_PHY_CON0 [3], P1 p.757).
fn clkgate<H: RegIo + ?Sized>(io: &mut H, ch: usize, on: bool) {
    io.write32(grf_phy_con(ch), wm(g::PHY2XCLKGATE_EN_BIT, 1, on as u32));
}

fn div_rst<H: RegIo + ?Sized>(io: &mut H, ch: usize, assert: bool) {
    io.write32(
        base::DDR_CRU[ch] + ddr_cru::SOFTRST_CON00,
        wm(ddr_cru::RST_PHY2XDIV, 1, assert as u32),
    );
}

fn main_rst<H: RegIo + ?Sized>(io: &mut H, ch: usize, bits: &[u32], assert: bool) {
    let r = ctl_resets_of(ch);
    let v = bits.iter().fold(0, |a, b| a | wm(*b, 1, assert as u32));
    io.write32(base::CRU + r.reg, v);
}

fn short<H: RegIo + ?Sized>(io: &mut H) {
    // ">= 10 cycles of clk_phy2x" at 24 MHz: 1 us is 24 cycles.
    io.udelay(1);
}

fn dfi_complete<H: RegIo + ?Sized>(io: &mut H, ch: usize, level: bool) -> Result<(), StepError> {
    let b = base::DDRCTL[ch];
    wait(io, "DFISTAT.dfi_init_complete", T, |io| {
        (io.read32(b + ctl::DFISTAT) & 1 == 1) == level
    })
    .map_err(|_| {
        if level {
            "dfi_init_complete=1"
        } else {
            "dfi_init_complete=0"
        }
    })
}

fn q<H: RegIo + ?Sized>(io: &mut H, ch: usize, f: impl FnOnce(&mut H)) -> Result<(), StepError> {
    dctl::quasi(io, ch, f).map_err(|_| "SWSTAT.sw_done_ack")
}

/// Steps 3-13: resets and clocks, PHY mode at 24 MHz. Starts from all
/// resets asserted and the PHY clock on the DxAPLL (24 MHz), so a second
/// pass (probe, then real) is repeatable.
pub fn reset_and_clock<H: RegIo + ?Sized>(io: &mut H, ch: usize) {
    let cru = base::DDR_CRU[ch];
    let r = ctl_resets_of(ch);
    io.write32(cru + ddr_cru::CLKSEL_CON00, wm(0, 1, 0));
    io.write32(cru + ddr_cru::GATE_CON00, wm(3, 3, 0));
    io.write32(
        cru + ddr_cru::SOFTRST_CON00,
        wm(ddr_cru::RST_PRESETN_PHY, 1, 1)
            | wm(ddr_cru::RST_PHY2X, 1, 1)
            | wm(ddr_cru::RST_PHY2XDIV, 1, 1),
    );
    main_rst(io, ch, &[r.presetn, r.core, r.aresetn, r.sbr, r.dfi], true);
    clkgate(io, ch, false);
    short(io);
    // 3. De-assert PRESETn of DDRPHY and presetn of DDRCTL.
    io.write32(
        cru + ddr_cru::SOFTRST_CON00,
        wm(ddr_cru::RST_PRESETN_PHY, 1, 0),
    );
    main_rst(io, ch, &[r.presetn], false);
    // 4. De-assert RESETn of DDRPHY; wait >= 10 clk_phy2x.
    io.write32(cru + ddr_cru::SOFTRST_CON00, wm(ddr_cru::RST_PHY2X, 1, 0));
    short(io);
    // 5. DDRPHY2XCLKGATE_ENABLE = 1; wait.
    clkgate(io, ch, true);
    short(io);
    // 6. De-assert div_rst_n; wait.
    div_rst(io, ch, false);
    short(io);
    // 7. sbr_resetn, core_ddrc_rstn, aresetn_0 asserted (kept from above).
    main_rst(io, ch, &[r.sbr, r.core, r.aresetn], true);
    // 8. De-assert rst_n of DDRPHY at 24 MHz.
    main_rst(io, ch, &[r.dfi], false);
    short(io);
    // 9. DDRPHY2XCLKGATE_ENABLE = 0.
    clkgate(io, ch, false);
    // 10. Assert div_rst_n.
    div_rst(io, ch, true);
    // 11. Memory type and clock mode.
    dphy::set_mode(io, ch);
    // 12. De-assert div_rst_n.
    div_rst(io, ch, false);
    // 13. Step 5 again: clk_dfi to the DDRCTL.
    clkgate(io, ch, true);
    short(io);
}

/// Steps 14-22: controller registers, controller out of reset, refresh and
/// low power off, AXI port on, PHY latencies.
pub fn controller_up<H: RegIo + ?Sized>(
    io: &mut H,
    ch: usize,
    p: &Plan,
    geo: &ChannelGeometry,
    cfg: &Config,
) -> Result<(), StepError> {
    let b = base::DDRCTL[ch];
    let r = ctl_resets_of(ch);
    // 14.
    dctl::program(io, ch, p, geo)?;
    // 15.
    main_rst(io, ch, &[r.sbr, r.core, r.aresetn], false);
    short(io);
    // 16. dis_auto_refresh = 1; powerdown_en [4], selfref_en [0],
    // en_dfi_dram_clk_disable [9], selfref_sw [11] = 0 (p.123, PWRCTL).
    io.set32(b + ctl::RFSHCTL0, 1);
    io.clr32(
        b + ctl::PWRCTL,
        (1 << 0) | (1 << 4) | (1 << 9) | ctl::PWRCTL_SELFREF_SW,
    );
    // 17-19. dfi_init_complete_en = 0.
    q(io, ch, |io| {
        io.clr32(b + ctl::DFIMISC, ctl::DFIMISC_INIT_COMPLETE_EN)
    })?;
    // 20. AXI port on.
    io.set32(b + ctl::PCTRL, 1);
    // 21-22, 26-27.
    dphy::configure(io, ch, p, cfg.dqs_clean, geo.row);
    Ok(())
}

/// Steps 23-25: DFI initialization at 24 MHz; the controller then runs
/// the JEDEC power-up and MR1/2/3/13 writes (INITMR0/1) itself.
pub fn dfi_init<H: RegIo + ?Sized>(io: &mut H, ch: usize) -> Result<(), StepError> {
    let b = base::DDRCTL[ch];
    // 23.
    io.clr32(base::DDRPHY[ch] + phy::MDLL_CON0, phy::MDLL_DLL_ON);
    q(io, ch, |io| {
        io.set32(b + ctl::DFIMISC, ctl::DFIMISC_INIT_START)
    })?;
    dfi_complete(io, ch, true).map_err(|_| "step 23: dfi_init_complete")?;
    // 24.
    q(io, ch, |io| {
        io.clrset32(
            b + ctl::DFIMISC,
            ctl::DFIMISC_INIT_START,
            ctl::DFIMISC_INIT_COMPLETE_EN,
        )
    })?;
    // 25.
    wait(io, "STAT.operating_mode", T, |io| {
        io.read32(b + ctl::STAT) & ctl::STAT_MODE_MASK == ctl::STAT_MODE_NORMAL
    })
    .map_err(|_| "step 25: operating_mode normal")
}

/// Step 29: switch clk_phy2x from the DxAPLL (24 MHz) to the DxBPLL with
/// the MC-PHY handshake (`to_bpll`), or back (used by the CBT path).
pub fn freq_change<H: RegIo + ?Sized>(
    io: &mut H,
    ch: usize,
    to_bpll: bool,
    sr: bool,
) -> Result<(), StepError> {
    let b = base::DDRCTL[ch];
    if sr {
        dctl::self_refresh(io, ch, true).map_err(|_| "step 29: self-refresh entry")?;
    }
    q(io, ch, |io| {
        io.clrset32(
            b + ctl::DFIMISC,
            ctl::DFIMISC_INIT_COMPLETE_EN,
            ctl::DFIMISC_INIT_START,
        )
    })?;
    dfi_complete(io, ch, false).map_err(|_| "step 29: dfi_init_complete=0")?;
    clkgate(io, ch, false);
    div_rst(io, ch, true);
    io.write32(
        base::DDR_CRU[ch] + ddr_cru::CLKSEL_CON00,
        wm(0, 1, to_bpll as u32),
    );
    div_rst(io, ch, false);
    clkgate(io, ch, true);
    q(io, ch, |io| {
        io.clr32(b + ctl::DFIMISC, ctl::DFIMISC_INIT_START)
    })?;
    dfi_complete(io, ch, true).map_err(|_| "step 29: dfi_init_complete=1")?;
    q(io, ch, |io| {
        io.set32(b + ctl::DFIMISC, ctl::DFIMISC_INIT_COMPLETE_EN)
    })?;
    if sr {
        dctl::self_refresh(io, ch, false).map_err(|_| "step 29: self-refresh exit")?;
    }
    Ok(())
}

/// GRF hold of dfi_init_complete (RK3576 RFC step 31/34): bit 5 (value)
/// first, then bit 4 (select); release clears the select.
fn grf_guard<H: RegIo + ?Sized>(io: &mut H, ch: usize, hold: bool) {
    let r = grf_phy_con(ch);
    if hold {
        io.write32(r, wm(g::DFI_INIT_COMPLETE_BIT, 1, 1));
        io.write32(r, wm(g::DFI_INIT_COMPLETE_SEL_BIT, 1, 1));
    } else {
        io.write32(r, wm(g::DFI_INIT_COMPLETE_SEL_BIT, 1, 0));
    }
}

/// What the sequence observed.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Observed {
    pub zq_fail: bool,
}

/// Steps 3-32 for channel `ch`: the channel ends in normal mode at the
/// target clock, DLL locked, untrained, MR11/12/14/22 written.
pub fn init_channel<H: RegIo + ?Sized>(
    io: &mut H,
    ch: usize,
    p: &Plan,
    geo: &ChannelGeometry,
    cfg: &Config,
) -> Result<Observed, StepError> {
    reset_and_clock(io, ch);
    controller_up(io, ch, p, geo, cfg)?;
    dfi_init(io, ch)?;
    // 28.
    let zq_fail = dphy::zq(io, ch).map_err(|_| "step 28: zq_done")?;
    // 29.
    freq_change(io, ch, true, cfg.sr_freq_change)?;
    // 30-31.
    if cfg.grf_dfi_guard {
        grf_guard(io, ch, true);
    }
    let locked = dphy::dll_lock(io, ch).map_err(|_| "step 31: DLL lock");
    if cfg.grf_dfi_guard {
        grf_guard(io, ch, false);
    }
    locked?;
    // 32.
    io.set32(base::DDRPHY[ch] + phy::LP_CON0, phy::LP_SCHEDULER_EN);
    // MR11/12/14/22 by MRW (INITMR2/3 are "not applicable for
    // initialization", p.84).
    for rank in 0..p.ranks {
        for (ma, op) in [(11, p.mr11), (12, p.mr12), (14, p.mr14), (22, p.mr22)] {
            super::mr::write(io, ch, rank, ma, op).map_err(|_| "MRW MR11/12/14/22")?;
        }
    }
    Ok(Observed { zq_fail })
}

/// Step 34 is crate::ddr::phy::after_training; this re-enables what the
/// controller had off during init and training: auto-refresh, auto ZQ,
/// controller-initiated DFI updates (VT compensation needs them, p.317).
pub fn finish<H: RegIo + ?Sized>(io: &mut H, ch: usize) -> Result<(), StepError> {
    let b = base::DDRCTL[ch];
    dphy::after_training(io, ch);
    q(io, ch, |io| {
        io.clr32(b + ctl::DFIUPD0, ctl::DFIUPD0_DIS_AUTO_CTRLUPD);
        io.clr32(b + ctl::ZQCTL0, ctl::ZQCTL0_DIS_AUTO_ZQ);
    })?;
    io.clr32(b + ctl::RFSHCTL0, 1);
    Ok(())
}
