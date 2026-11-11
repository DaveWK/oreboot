// SPDX-License-Identifier: GPL-2.0-only
//! RK3588 eMMC (Synopsys DWC MSHC, SDHCI v3 compatible, TRM Part 2
//! Chapter 4) glue for crate sdhci.
//!
//! - Pins: BUS_IOC GPIO2A_IOMUX_SEL_L (+0x40) gpio2a0..3 and GPIO2D_IOMUX_SEL
//!   _L/_H (+0x58/+0x5C) gpio2d0..7 = 1 (EMMC_*; reset value 0 = GPIO,
//!   TRM Part 1 pp.992-995): set for a MaskROM USB start, where the BootROM
//!   may not have touched the eMMC.
//! - Clock: cclk_emmc from xin 24 MHz (CRU_CLKSEL_CON77 0x434: sel [15:14]
//!   = 2, div [13:8] = 0; reset is GPLL/6, P1 p.~120) so the source is known;
//!   the fastboot loader moves it to GPLL / 12 (99 MHz) for 52 MHz
//!   high-speed writes ([`source_from_gpll`]). The capabilities register's
//!   base clock field is masked to 0 in [`EmmcIo`] so crate sdhci asks the
//!   platform, which reads CLKSEL_CON77 back.
//! - DLL bypass at <= 52 MHz and the vendor area writes: mainline Linux
//!   drivers/mmc/host/sdhci-of-dwcmshc.c (GPL-2.0) dwcmshc_rk3568_set_clock /
//!   dwcmshc_rk35xx_init: EMMC_DLL_CTRL (0x800) = BYPASS (24) | START (0),
//!   DLL_RXCLK (0x804) = ORI_GATE (31), DLL_TXCLK (0x808) = 0, DLL_CMDOUT
//!   (0x810) = 0; vendor area 1 (P_VENDOR_AREA1 0xE8, low 12 bits) HOST_CTRL3
//!   (+0x8) bit 0 clear (cmd conflict check off), bit 4 set; EMMC_CONTROL
//!   (+0x2C) bit 0 (card is eMMC).

use regio::RegIo;
use rk3588::pll::{soc_pll_mhz, GPLL_1188};
use rk3588::regs::{base, wm};
use sdhci::{Error, Kind, SdhciPlatform};

const CAPABILITIES: usize = 0x40;

/// RegIo wrapper for the eMMC controller: masks the capabilities base
/// clock so the platform's known 24 MHz source is used.
pub struct EmmcIo<'a, H: RegIo>(pub &'a mut H);

impl<H: RegIo> RegIo for EmmcIo<'_, H> {
    fn read32(&mut self, addr: usize) -> u32 {
        let v = self.0.read32(addr);
        if addr == base::EMMC + CAPABILITIES {
            v & !(0xff << 8)
        } else {
            v
        }
    }
    fn write32(&mut self, addr: usize, val: u32) {
        self.0.write32(addr, val)
    }
    fn read16(&mut self, addr: usize) -> u16 {
        self.0.read16(addr)
    }
    fn write16(&mut self, addr: usize, val: u16) {
        self.0.write16(addr, val)
    }
    fn read8(&mut self, addr: usize) -> u8 {
        self.0.read8(addr)
    }
    fn write8(&mut self, addr: usize, val: u8) {
        self.0.write8(addr, val)
    }
    fn udelay(&mut self, us: u32) {
        self.0.udelay(us)
    }
    fn now_us(&mut self) -> u64 {
        self.0.now_us()
    }
    fn log(&mut self, args: core::fmt::Arguments) {
        self.0.log(args)
    }
}

pub struct Rk3588Emmc;

impl SdhciPlatform for Rk3588Emmc {
    fn enable<IO: RegIo>(&mut self, io: &mut IO, _base: usize, _kind: Kind) {
        io.write32(base::BUS_IOC + 0x40, wm(0, 16, 0x1111));
        io.write32(base::BUS_IOC + 0x58, wm(0, 16, 0x1111));
        io.write32(base::BUS_IOC + 0x5C, wm(0, 16, 0x1111));
        io.write32(base::CRU + 0x0434, wm(14, 2, 2) | wm(8, 6, 0));
    }
    fn init_quirks<IO: RegIo>(&mut self, io: &mut IO, b: usize, _kind: Kind) {
        io.write32(b + 0x800, (1 << 24) | 1);
        io.write32(b + 0x804, 1 << 31);
        io.write32(b + 0x808, 0);
        io.write32(b + 0x810, 0);
        let area1 = b + (io.read32(b + 0xE8) as usize & 0xfff);
        io.clrset32(area1 + 0x8, 1, 1 << 4);
        io.set32(area1 + 0x2C, 1);
    }
    /// What CLKSEL_CON77 selects: xin 24 MHz (after `enable`) or GPLL (after
    /// [`source_from_gpll`]), through its divider.
    fn source_khz<IO: RegIo>(&mut self, io: &mut IO, _b: usize, _kind: Kind) -> Result<u32, Error> {
        let v = io.read32(CLKSEL_CON77);
        let div = ((v >> 8) & 0x3f) + 1;
        match (v >> 14) & 3 {
            2 => Ok(24_000 / div),
            0 => match soc_pll_mhz(io, base::CRU, &GPLL_1188) {
                0..=24 => Err(Error::ClockSource),
                mhz => Ok(mhz * 1000 / div),
            },
            _ => Err(Error::ClockSource),
        }
    }
}

const CLKSEL_CON77: usize = base::CRU + 0x0434;
/// The highest cclk_emmc source taken from GPLL: the SDHCI divider halves
/// it at least once, so ~100 MHz gives the 52 MHz high-speed card clock.
const GPLL_SOURCE_MAX_MHZ: u32 = 100;

/// Move cclk_emmc to GPLL / n (at most 100 MHz; 1188 / 12 = 99 MHz once
/// bt0 set GPLL). The caller has divided the card clock down first: the
/// source grows by up to 4x. False (nothing changed) while GPLL is not
/// running.
pub fn source_from_gpll<H: RegIo>(io: &mut H) -> bool {
    let gpll = soc_pll_mhz(io, base::CRU, &GPLL_1188);
    if gpll < 2 * GPLL_SOURCE_MAX_MHZ {
        return false;
    }
    let n = gpll.div_ceil(GPLL_SOURCE_MAX_MHZ);
    io.write32(CLKSEL_CON77, wm(14, 2, 0) | wm(8, 6, n - 1));
    true
}
