//! GMAC0 and its Maxio MAE0621A-Q3C RGMII PHY.
//!
//! Brings the Ethernet hardware to the state the vendor kernel's dwmac-sunxi
//! probe leaves it in, so that any payload or OS only has to drive the MAC
//! and DMA: clocks and resets, pins, the PHY reset pulse, the RGMII glue
//! register with the board's delay chains, and the PHY's vendor register
//! sequence (without it the PHY drives no RGMII RX clock and the MAC's DMA
//! reset never completes). bt0 has already switched on the PHY's 3.3 V
//! (AXP DC1SW1) and the PH bank supply (DC1SW2).
//!
//! The MAC address comes from the SID chip ID
//! ([`cubie_a7a_common::ethernet`]); it is programmed into MAC address 0 and
//! published in the DTB so EDK2 and the OS DTB fixup use the same value.
//!
//! Sources: vendor DT (`captures/cubie-a7a-vendor-6.6.98/sys/live.dts`,
//! ethernet@4500000), the BSP's dwmac-sunxi.c / ccu-sun60iw2.c, the A733
//! user manual (CCU 4.1.6.84/210-212, GMAC 18.1, PIO 18.14), and the Maxio
//! driver Armbian carries (`drv-net-phy-add-maxio-mae0621-phy-driver.patch`,
//! `maxio_mae0621aq3ci_config_init`; register values only).
//!
//! Every wait is bounded. Failures are logged and the boot continues; the
//! payload then simply finds no working link.

use core::ptr::{addr_of, addr_of_mut};

use cubie_a7a_common::{
    ethernet::{mac_from_chip_id, SID_CHIP_ID},
    time::{ticks, udelay, TIMER_HZ},
};
use util::mmio::{read32, write32};

const CCU: usize = 0x0200_2000;
const GMAC_PTP_CLK: usize = CCU + 0x1400;
const GMAC0_PHY_CLK: usize = CCU + 0x1410;
const GMAC0_BGR: usize = CCU + 0x141c;
const MBUS_GATE_EN: usize = CCU + 0x05e4;
/// Write-enable key for MBUS_GATE_EN_REG (ccu-sun60iw2.c
/// `MBUS_GATE_KEY_VALUE`); the key bits read back as 0.
const MBUS_GATE_KEY: u32 = 0x0004_0302;
const MBUS_GMAC0: u32 = 1 << 11;

const PIO: usize = 0x0200_0000;
const PH_CFG0: usize = PIO + 0x400;
const PH_CFG1: usize = PIO + 0x404;
const PH_CFG2: usize = PIO + 0x408;
const PH_DAT: usize = PIO + 0x410;
const PH_DRV0: usize = PIO + 0x420;
const PH_DRV1: usize = PIO + 0x424;
const PH_PUL0: usize = PIO + 0x430;
/// PH0..PH15 function 5: RGMII0 data/clock/control, MDC/MDIO and the
/// 25 MHz PHY clock (PH15). Drive level 1, pull-up, as the vendor DT.
const PH_FUNC_RGMII0: u32 = 0x5555_5555;
const PH_DRV_LEVEL1: u32 = 0x1111_1111;
const PH_PULL_UP: u32 = 0x5555_5555;
/// PHY reset, active low (vendor DT reset-gpios = <&pio 7 16 ACTIVE_LOW>).
const PHY_RESET_PIN: u32 = 16;
const PHY_RESET_ASSERT_US: u64 = 10_000;
const PHY_RESET_DEASSERT_US: u64 = 150_000;

const GMAC: usize = 0x0450_0000;
const MAC_VERSION: usize = GMAC + 0x0110;
const MAC_MDIO_ADDRESS: usize = GMAC + 0x0200;
const MAC_MDIO_DATA: usize = GMAC + 0x0204;
const MAC_ADDRESS0_HIGH: usize = GMAC + 0x0300;
const MAC_ADDRESS0_LOW: usize = GMAC + 0x0304;
const MAC_ADDRESS0_HIGH_AE: u32 = 1 << 31;
const DMA_MODE: usize = GMAC + 0x1000;
const DMA_MODE_SWR: u32 = 1 << 0;

/// GMAC_CFG_REG (manual 18.1.7.1, BSP SUNXI_DWMAC210_CFG_*). Written whole:
/// that also clears PHY_SELECT (bit 15, internal PHY), set at reset.
const GLUE_CFG: usize = 0x0450_8000;
const GLUE_EPIT_RGMII: u32 = 1 << 2;
const GLUE_ETCS_INT_GMII: u32 = 2;
/// Vendor DT tx-delay = 12, rx-delay = 10 (delay-chain steps).
const TX_DELAY: u32 = 12;
const RX_DELAY: u32 = 10;
const GLUE_VALUE: u32 = ((TX_DELAY >> 3) << 16)
    | ((TX_DELAY & 7) << 10)
    | (RX_DELAY << 5)
    | GLUE_EPIT_RGMII
    | GLUE_ETCS_INT_GMII;
const _: () = assert!(GLUE_VALUE == 0x0001_1146);

/// MDIO: CSR clock range 150-250 MHz (AHB 200 MHz, MDC = 200/102 MHz), as
/// dwmac-sunxi sets `clk_csr = 4`. Clause 22 read = GOC 3, write = GOC 1.
const MDIO_CR: u32 = 4 << 8;
const MDIO_GOC_READ: u32 = 3 << 2;
const MDIO_GOC_WRITE: u32 = 1 << 2;
const MDIO_GB: u32 = 1 << 0;
/// A clause 22 frame is 64 MDC cycles, ~33 us at 1.96 MHz.
const MDIO_TIMEOUT_US: u64 = 10_000;

const PHY_ADDR: u32 = 1;
const PHY_ID_MAE0621A_Q3C: u32 = 0x7b74_4412;
const MII_BMCR: u32 = 0x00;
const MII_PHYID1: u32 = 0x02;
const MII_PHYID2: u32 = 0x03;
const MAXIO_PAGE_SELECT: u32 = 0x1f;

/// `maxio_mae0621aq3ci_config_init`: (page, register, value). The last two
/// entries set the 10/100 advertisement with pause, then BMCR = reset |
/// autoneg enable | full duplex | 1000 Mb/s, which restarts negotiation.
const MAE0621A_Q3C_INIT: [(u16, u8, u16); 27] = [
    (0xa43, 0x19, 0x0823),
    (0xdab, 0x17, 0x0c13),
    (0xd96, 0x15, 0xc08a),
    (0xda4, 0x12, 0x07bc),
    (0xd8f, 0x16, 0x2500),
    (0xd90, 0x16, 0x1555),
    (0xd92, 0x11, 0x2b15),
    (0xd96, 0x16, 0x4010),
    (0xda5, 0x11, 0x4a12),
    (0xda5, 0x12, 0x4a12),
    (0xd99, 0x16, 0x000a),
    (0xd95, 0x13, 0x5b00),
    (0xd8f, 0x10, 0x0300),
    (0xd98, 0x17, 0x8506),
    (0xd95, 0x12, 0x6870),
    (0xd93, 0x15, 0x0940),
    (0xdad, 0x12, 0x0303), // TX clock stop off
    (0xdad, 0x13, 0x050d), // I/O drive strength 1
    (0xdad, 0x14, 0x0d05),
    (0xdad, 0x15, 0x0505),
    (0xdad, 0x17, 0x0001),
    (0xda8, 0x11, 0x0177),
    (0xda8, 0x10, 0x09a9),
    (0xda8, 0x12, 0x0868),
    (0xa42, 0x12, 0x0028),
    (0x000, 0x04, 0x0de1),
    (0x000, 0x00, 0x9140),
];

/// After the BMCR reset the PHY restarts its RGMII clocks; the DMA reset
/// needs the RX clock. Linux's stmmac allows 10 s here; the PHY normally
/// takes a few milliseconds.
const DMA_RESET_TIMEOUT_US: u64 = 200_000;

static mut MAC: Option<[u8; 6]> = None;

/// The board MAC address, once [`init`] has run.
pub fn mac_address() -> Option<[u8; 6]> {
    // SAFETY: written once by the boot CPU before anything reads it.
    unsafe { *addr_of!(MAC) }
}

fn elapsed_us(t0: u64) -> u64 {
    (ticks() - t0) / (TIMER_HZ / 1_000_000)
}

/// Poll `done` every microsecond for up to `timeout_us`.
fn wait(timeout_us: u64, mut done: impl FnMut() -> bool) -> bool {
    let t0 = ticks();
    loop {
        if done() {
            return true;
        }
        if elapsed_us(t0) >= timeout_us {
            return done();
        }
        udelay(1);
    }
}

#[derive(Debug)]
struct MdioTimeout;

fn mdio_wait() -> Result<(), MdioTimeout> {
    if wait(MDIO_TIMEOUT_US, || read32(MAC_MDIO_ADDRESS) & MDIO_GB == 0) {
        Ok(())
    } else {
        Err(MdioTimeout)
    }
}

fn mdio_read(reg: u32) -> Result<u16, MdioTimeout> {
    mdio_wait()?;
    write32(
        MAC_MDIO_ADDRESS,
        PHY_ADDR << 21 | reg << 16 | MDIO_CR | MDIO_GOC_READ | MDIO_GB,
    );
    mdio_wait()?;
    Ok(read32(MAC_MDIO_DATA) as u16)
}

fn mdio_write(reg: u32, val: u16) -> Result<(), MdioTimeout> {
    mdio_wait()?;
    write32(MAC_MDIO_DATA, val as u32);
    write32(
        MAC_MDIO_ADDRESS,
        PHY_ADDR << 21 | reg << 16 | MDIO_CR | MDIO_GOC_WRITE | MDIO_GB,
    );
    mdio_wait()
}

/// Like the Maxio driver's `maxio_write_paged()`; it restores the page it
/// found, which is page 0 throughout this sequence.
fn phy_write_paged(page: u16, reg: u8, val: u16) -> Result<(), MdioTimeout> {
    mdio_write(MAXIO_PAGE_SELECT, page)?;
    mdio_write(reg as u32, val)?;
    mdio_write(MAXIO_PAGE_SELECT, 0)
}

fn clocks_and_resets() {
    // PTP reference: SYS_CLK24M / 1 (the vendor kernel runs it at 24 MHz).
    write32(GMAC_PTP_CLK, 1 << 31);
    // 25 MHz PHY clock out on PH15: PERI0_150M / 6.
    write32(GMAC0_PHY_CLK, 1 << 31 | (6 - 1));
    write32(MBUS_GATE_EN, read32(MBUS_GATE_EN) | MBUS_GATE_KEY | MBUS_GMAC0);
    // Pulse both resets, then release them and open the bus gate.
    write32(GMAC0_BGR, 0);
    udelay(10);
    write32(GMAC0_BGR, 1 << 17 | 1 << 16);
    write32(GMAC0_BGR, 1 << 17 | 1 << 16 | 1 << 0);
}

fn pins() {
    write32(PH_DRV0, PH_DRV_LEVEL1);
    write32(PH_DRV1, PH_DRV_LEVEL1);
    write32(PH_PUL0, PH_PULL_UP);
    write32(PH_CFG0, PH_FUNC_RGMII0);
    write32(PH_CFG1, PH_FUNC_RGMII0);
}

/// Hold the PHY in reset with its 25 MHz clock running, then release it.
fn phy_reset() {
    let bit = 1 << PHY_RESET_PIN;
    write32(PH_DAT, read32(PH_DAT) & !bit);
    // PH16 select: output (bits 3:0 of PH_CFG2).
    write32(PH_CFG2, (read32(PH_CFG2) & !0xf) | 0x1);
    udelay(PHY_RESET_ASSERT_US);
    write32(PH_DAT, read32(PH_DAT) | bit);
    udelay(PHY_RESET_DEASSERT_US);
}

fn phy_init() -> Result<(), MdioTimeout> {
    let id = (mdio_read(MII_PHYID1)? as u32) << 16 | mdio_read(MII_PHYID2)? as u32;
    if id != PHY_ID_MAE0621A_Q3C {
        println!("[main] GMAC0: PHY @{PHY_ADDR} has ID {id:#010x}, not a MAE0621A-Q3C; left as is");
        return Ok(());
    }
    for (page, reg, val) in MAE0621A_Q3C_INIT {
        phy_write_paged(page, reg, val)?;
    }
    let bmcr = mdio_read(MII_BMCR)?;
    println!("[main] GMAC0: Maxio MAE0621A-Q3C configured, BMCR {bmcr:#06x}");
    Ok(())
}

fn set_mac_address(mac: &[u8; 6]) {
    write32(
        MAC_ADDRESS0_HIGH,
        MAC_ADDRESS0_HIGH_AE | (mac[5] as u32) << 8 | mac[4] as u32,
    );
    write32(MAC_ADDRESS0_LOW, u32::from_le_bytes([mac[0], mac[1], mac[2], mac[3]]));
}

/// Bring up GMAC0 and its PHY (see the module documentation).
pub fn init() {
    let t0 = ticks();
    let id = [0, 4, 8, 12].map(|o| read32(SID_CHIP_ID + o));
    let mac = mac_from_chip_id(id);
    // SAFETY: boot CPU only, before any reader.
    unsafe { *addr_of_mut!(MAC) = Some(mac) };

    clocks_and_resets();
    pins();
    write32(GLUE_CFG, GLUE_VALUE);
    let version = read32(MAC_VERSION);
    phy_reset();

    if let Err(e) = phy_init() {
        println!("[main] GMAC0: MDIO {e:?}; PHY not configured");
    }
    // Software reset: completes only once the PHY supplies the RX clock.
    write32(DMA_MODE, DMA_MODE_SWR);
    let reset_ok = wait(DMA_RESET_TIMEOUT_US, || read32(DMA_MODE) & DMA_MODE_SWR == 0);
    // The reset clears the address registers; program them afterwards.
    set_mac_address(&mac);

    println!(
        "[main] GMAC0: Synopsys ID {:#04x}, MAC {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}, DMA reset {}, {} ms",
        version & 0xff,
        mac[0],
        mac[1],
        mac[2],
        mac[3],
        mac[4],
        mac[5],
        if reset_ok { "ok" } else { "timed out (no RX clock from the PHY?)" },
        elapsed_us(t0) / 1000
    );
}
