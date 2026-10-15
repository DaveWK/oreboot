// SPDX-License-Identifier: GPL-2.0-only
//! SD/eMMC on the K1: the shared SDHCI driver (`sdhci`, ore-edk2-boot) with
//! the K1 platform side. eMMC is SDH2 at 0xd428_1000 (OrangePi R2S), SD is
//! SDH0 at 0xd428_0000 (OrangePi RV2). The K1 part is what the K1 driver did
//! around the standard controller: APMU clock and reset, PHY and pad mode,
//! the source clock tree when the capabilities report none, tuning and the
//! TX hold control before writes, and the data cache clean before SDMA. The
//! crate's equivalence test runs this platform against the previous K1
//! driver and requires the same register trace, results and card data.

use crate::k1io::K1Io;
use regio::RegIo;
pub use sdhci::{Error, Kind, Partition};
use sdhci::{Sdhci, SdhciPlatform};

pub const EMMC_BASE: usize = 0xd428_1000;
pub const SD_BASE: usize = 0xd428_0000;

// APMU clock/reset of SDH0 (also the shared AXI clock) and SDH2.
const APMU: usize = 0xd428_2800;
const APMU_SDH0_CLK_RES_CTRL: usize = APMU + 0x054;
const APMU_SDH2_CLK_RES_CTRL: usize = APMU + 0x0e0;

// SpacemiT SDHCI vendor registers.
const OP_EXT: usize = 0x108;
const LEGACY_CTRL: usize = 0x10c;
const MMC_CTRL: usize = 0x114;
const TX_CFG: usize = 0x11c;
const PHY_CTRL: usize = 0x160;
const PHY_PADCFG: usize = 0x178;

/// The K1 board's side of the SDHCI controller.
pub struct K1Sdhci;

impl SdhciPlatform for K1Sdhci {
    fn enable<IO: RegIo>(&mut self, io: &mut IO, _: usize, kind: Kind) {
        let v = io.read32(APMU_SDH0_CLK_RES_CTRL);
        io.write32(APMU_SDH0_CLK_RES_CTRL, v | (1 << 3) | (1 << 0));
        let r = match kind {
            Kind::Emmc => APMU_SDH2_CLK_RES_CTRL,
            Kind::Sd => APMU_SDH0_CLK_RES_CTRL,
        };
        let v = io.read32(r);
        io.write32(r, v | (1 << 4) | (1 << 1));
    }
    fn init_quirks<IO: RegIo>(&mut self, io: &mut IO, b: usize, kind: Kind) {
        if kind == Kind::Emmc {
            io.set32(b + MMC_CTRL, 1 << 12);
            io.set32(b + PHY_CTRL, 0x3);
            io.set32(b + PHY_PADCFG, 1 << 5);
        }
        io.set32(b + TX_CFG, 1 << 30);
        io.set32(b + OP_EXT, (1 << 11) | (1 << 12));
    }
    /// The APMU source clock: mux [7:5] over the parents, divider [10:8] + 1.
    fn source_khz<IO: RegIo>(&mut self, io: &mut IO, _: usize, kind: Kind) -> u32 {
        let (reg, third) = match kind {
            Kind::Emmc => (io.read32(APMU_SDH2_CLK_RES_CTRL), 819_200),
            Kind::Sd => (io.read32(APMU_SDH0_CLK_RES_CTRL), 600_000),
        };
        let parents = [409_600, 614_400, 375_000, third, 223_400, 189_000, 106_800];
        let mux = ((reg >> 5) & 7) as usize;
        let div = ((reg >> 8) & 7) + 1;
        parents.get(mux).copied().unwrap_or(409_600) / div
    }
    fn tune_for_writes<IO: RegIo>(&mut self, io: &mut IO, b: usize, kind: Kind) {
        let apmu = match kind {
            Kind::Emmc => APMU_SDH2_CLK_RES_CTRL,
            Kind::Sd => APMU_SDH0_CLK_RES_CTRL,
        };
        log::println!(
            "[mmc] before: APMU {:08x} PHY_CTRL {:08x} PAD {:08x} LEGACY {:08x} TX {:08x} OP_EXT {:08x}",
            io.read32(apmu),
            io.read32(b + PHY_CTRL),
            io.read32(b + PHY_PADCFG),
            io.read32(b + LEGACY_CTRL),
            io.read32(b + TX_CFG),
            io.read32(b + OP_EXT)
        );
        if kind == Kind::Emmc {
            io.set32(b + PHY_CTRL, 0x3);
            io.clrset32(b + PHY_PADCFG, 0x7, 4 | (1 << 5));
        }
        io.set32(b + LEGACY_CTRL, 1 << 6);
        self.set_tx_hold(io, b, true);
    }
    fn set_tx_hold<IO: RegIo>(&mut self, io: &mut IO, b: usize, on: bool) {
        let v = io.read32(b + TX_CFG) & !(1 << 30);
        io.write32(b + TX_CFG, v | if on { 1 << 30 } else { 0 });
    }
    fn flush_dcache(&mut self, addr: usize, len: usize) {
        cache::zicbom::flush(addr, len);
    }
}

pub type Mmc = Sdhci<K1Io, K1Sdhci>;

/// Bring up the controller of `kind` and its card.
pub fn init(kind: Kind) -> Result<Mmc, Error> {
    let base = match kind {
        Kind::Emmc => EMMC_BASE,
        Kind::Sd => SD_BASE,
    };
    Sdhci::init(K1Io, K1Sdhci, base, kind)
}
