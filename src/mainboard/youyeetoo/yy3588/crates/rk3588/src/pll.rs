// SPDX-License-Identifier: GPL-2.0-only
//! DDR PLL (DxBPLL) setup and the main-CRU CPLL / GPLL / PPLL rates.
//!
//! Formula and limits from mainline U-Boot drivers/clk/rockchip/clk_pll.c
//! (GPL-2.0+): Fvco = Fref * m / p within 2250..4500 MHz, Fout = Fvco >> s,
//! Fout 37..4500 MHz; programming order from rk3588_pll_set_rate (power
//! down, m/p/s/k, power up, wait for lock). Register fields: TRM Part 1
//! pp.262-264 (DDR0CRU_D0BPLL_CON0/1/2/6). The DDR BPLL resets with its
//! bypass bit set (CON0 [15] = 1, p.262), so it is cleared after lock.

use crate::regs::{ddr_cru, wm};
use regio::{wait, RegIo, Timeout};

pub const FREF_MHZ: u32 = 24;
const VCO_MIN: u32 = 2250;
const VCO_MAX: u32 = 4500;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Pll {
    pub p: u32,
    pub m: u32,
    pub s: u32,
}

impl Pll {
    pub const fn fout_mhz(self) -> u32 {
        (FREF_MHZ * self.m / self.p) >> self.s
    }
}

/// Integer settings for `mhz` (no fractional K): the first p (1..=4) and s
/// (0..=6) with an exact m and an in-range VCO. 1056 MHz gives p=1, m=176,
/// s=2 (VCO 4224 MHz); U-Boot's table lists 1008 MHz as p=2, m=336, s=2.
pub fn settings(mhz: u32) -> Option<Pll> {
    if !(37..=4500).contains(&mhz) {
        return None;
    }
    for p in 1..=4u32 {
        for s in 0..=6u32 {
            let vco = mhz << s;
            if !(VCO_MIN..=VCO_MAX).contains(&vco) {
                continue;
            }
            if !(vco * p).is_multiple_of(FREF_MHZ) {
                continue;
            }
            let m = vco * p / FREF_MHZ;
            if (64..=1023).contains(&m) {
                return Some(Pll { p, m, s });
            }
        }
    }
    None
}

/// Program DDR channel `ch`'s BPLL to `pll` and wait for lock. The PHY
/// clock still comes from the APLL (24 MHz) until CLKSEL_CON00 switches it
/// (§2.6.1 step 29).
pub fn program<H: RegIo + ?Sized>(io: &mut H, cru: usize, pll: Pll) -> Result<(), Timeout> {
    // Power down (resetb = 1), program, power up.
    io.write32(cru + ddr_cru::BPLL_CON1, wm(13, 1, 1));
    io.write32(cru + ddr_cru::BPLL_CON0, wm(0, 10, pll.m));
    io.write32(cru + ddr_cru::BPLL_CON1, wm(0, 6, pll.p) | wm(6, 3, pll.s));
    io.write32(cru + ddr_cru::BPLL_CON2, wm(0, 16, 0));
    io.write32(cru + ddr_cru::BPLL_CON1, wm(13, 1, 0));
    wait(io, "DDR BPLL lock", 10_000, |io| {
        io.read32(cru + ddr_cru::BPLL_CON6) & (1 << ddr_cru::PLL_LOCK_BIT) != 0
    })?;
    // Leave bypass: PLL output instead of Fref.
    io.write32(cru + ddr_cru::BPLL_CON0, wm(15, 1, 0));
    Ok(())
}

/// A main-CRU PLL (CRU 0xFD7C0000; PPLL sits in the PHP CRU at +0x8000)
/// and integer settings for it. Offsets and mode fields: U-Boot
/// drivers/clk/rockchip/clk_rk3588.c rk3588_pll_clks (CPLL PLL_CON(104),
/// GPLL PLL_CON(112), PPLL PHP PLL_CON(128); MODE_CON0 shifts 8 / 2 / 10).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SocPll {
    pub name: &'static str,
    /// Offset of PLL_CON0 from the CRU base.
    pub con: usize,
    /// Field [shift + 1 : shift] of CRU_MODE_CON00: 0 slow (24 MHz), 1 normal.
    pub mode_shift: u32,
    pub pll: Pll,
}

/// The rates Rockchip's SPL sets and every later stage assumes (EDK2
/// Rk3588CruLib hard-codes CPLL 1500 / GPLL 1188 MHz; U-Boot
/// cru_rk3588.h CPLL_HZ / GPLL_HZ / PPLL_HZ); p/m/s from U-Boot's
/// rk3588_pll_rates. After the BootROM and the rkbin DDR init, CPLL runs
/// at 750 MHz, GPLL at 594 MHz and PPLL is left in slow mode.
pub const CPLL_1500: SocPll = SocPll {
    name: "CPLL",
    con: 0x1A0,
    mode_shift: 8,
    pll: Pll { p: 2, m: 250, s: 1 },
};
pub const GPLL_1188: SocPll = SocPll {
    name: "GPLL",
    con: 0x1C0,
    mode_shift: 2,
    pll: Pll { p: 2, m: 198, s: 1 },
};
pub const PPLL_1100: SocPll = SocPll {
    name: "PPLL",
    con: 0x8200,
    mode_shift: 10,
    pll: Pll { p: 3, m: 550, s: 2 },
};

const MODE_CON0: usize = 0x280;
/// CRU_CLKSEL_CON09 / CON38 (U-Boot cru_rk3588.h).
const CLKSEL_CON09: usize = 0x300 + 9 * 4;
const CLKSEL_CON38: usize = 0x300 + 38 * 4;

/// Output of a main-CRU PLL in MHz as programmed (0 in slow mode = 24 MHz
/// bypass is reported as 24).
pub fn soc_pll_mhz<H: RegIo + ?Sized>(io: &mut H, cru: usize, p: &SocPll) -> u32 {
    if (io.read32(cru + MODE_CON0) >> p.mode_shift) & 3 != 1 {
        return FREF_MHZ;
    }
    let con0 = io.read32(cru + p.con);
    let con1 = io.read32(cru + p.con + 4);
    let pll = Pll {
        m: con0 & 0x3FF,
        p: (con1 & 0x3F).max(1),
        s: (con1 >> 6) & 7,
    };
    pll.fout_mhz()
}

/// Set one main-CRU PLL. When it is already locked in normal mode with the
/// target VCO (Fref * m / p) and only the post-divider differs, change s
/// alone: the VCO stays locked and the output steps straight to the new
/// rate (after the rkbin DDR init CPLL runs VCO 3000 MHz / s=2, GPLL
/// 2376 MHz / s=2, while the CLKSEL dividers are set for 1500 / 1188 MHz;
/// the slow-mode detour below takes every live CPLL consumer down to
/// 24 MHz and hangs bt0). Otherwise the full U-Boot rk3588_pll_set_rate
/// sequence: slow mode, power down, m/p/s/k, power up, wait for lock,
/// normal mode. Returns true when only s was changed.
pub fn program_soc<H: RegIo + ?Sized>(io: &mut H, cru: usize, p: &SocPll) -> Result<bool, Timeout> {
    let con0 = io.read32(cru + p.con);
    let con1 = io.read32(cru + p.con + 4);
    let (m, pd) = (con0 & 0x3FF, (con1 & 0x3F).max(1));
    let normal = (io.read32(cru + MODE_CON0) >> p.mode_shift) & 3 == 1;
    let locked = io.read32(cru + p.con + 0x18) & (1 << 15) != 0;
    let k = io.read32(cru + p.con + 8) & 0xFFFF;
    if normal && locked && k == 0 && FREF_MHZ * m * p.pll.p == FREF_MHZ * p.pll.m * pd {
        io.write32(cru + p.con + 4, wm(6, 3, p.pll.s));
        return Ok(true);
    }
    io.write32(cru + MODE_CON0, wm(p.mode_shift, 2, 0));
    io.write32(cru + p.con + 4, wm(13, 1, 1));
    io.write32(cru + p.con, wm(0, 10, p.pll.m));
    io.write32(cru + p.con + 4, wm(0, 6, p.pll.p) | wm(6, 3, p.pll.s));
    io.write32(cru + p.con + 8, wm(0, 16, 0));
    io.write32(cru + p.con + 4, wm(13, 1, 0));
    wait(io, p.name, 10_000, |io| {
        io.read32(cru + p.con + 0x18) & (1 << 15) != 0
    })?;
    io.write32(cru + MODE_CON0, wm(p.mode_shift, 2, 1));
    Ok(false)
}

/// U-Boot rk3588_clk_init: aclk_bus_root divider for the new GPLL (GPLL,
/// div field = ceil(1188 / 300)), CPLL 1500, GPLL 1188, PPLL 1100, then
/// aclk_top_s400 / s200 to 400 / 200 MHz. `step` runs before each write
/// group with its name (progress output while bring-up is being proven).
/// Returns the PLL that failed to lock, if any; the rest are still
/// attempted.
pub fn soc_clocks_init<H, F>(io: &mut H, cru: usize, mut step: F) -> Result<(), &'static str>
where
    H: RegIo + ?Sized,
    F: FnMut(&mut H, &'static str),
{
    step(io, "aclk_bus_root");
    io.write32(cru + CLKSEL_CON38, wm(0, 5, 1188_u32.div_ceil(300)) | wm(5, 2, 0));
    let mut failed = Ok(());
    for p in [&CPLL_1500, &GPLL_1188, &PPLL_1100] {
        step(io, p.name);
        if program_soc(io, cru, p).is_err() && failed.is_ok() {
            failed = Err(p.name);
        }
    }
    step(io, "aclk_top");
    io.write32(cru + CLKSEL_CON09, wm(8, 2, 0) | wm(6, 2, 0));
    failed
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn known_rates() {
        assert_eq!(settings(1056), Some(Pll { p: 1, m: 176, s: 2 }));
        for mhz in [528, 800, 933, 1056, 1560, 2112] {
            let p = settings(mhz).unwrap();
            assert_eq!(p.fout_mhz(), mhz, "{mhz}");
            let vco = FREF_MHZ * p.m / p.p;
            assert!((VCO_MIN..=VCO_MAX).contains(&vco));
        }
        assert_eq!(settings(10), None);
    }

    #[test]
    fn soc_rates() {
        assert_eq!(CPLL_1500.pll.fout_mhz(), 1500);
        assert_eq!(GPLL_1188.pll.fout_mhz(), 1188);
        assert_eq!(PPLL_1100.pll.fout_mhz(), 1100);
        // The state found after the rkbin DDR init (CPLL con0/1 = 0x7D/0x81).
        assert_eq!(Pll { m: 0x7D, p: 1, s: 2 }.fout_mhz(), 750);
    }
}
