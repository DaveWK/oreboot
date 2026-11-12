// SPDX-License-Identifier: GPL-2.0-only
//! USB host bring-up for the Cubie A7A: power domain, clocks, resets, PHYs
//! and VBUS, so that EDK2 (and an OS) find powered, out-of-reset
//! controllers. Moved here from EDK2's SunxiUsbDxe, keeping its values and
//! ordering exactly; those orderings were found by trial on hardware and
//! the comments say why.
//!
//! Bases (Linux /proc/iomem on an A733 board):
//!
//! | Base         | Block                                       |
//! |--------------|---------------------------------------------|
//! | `0x02002000` | CCU                                         |
//! | `0x04101000` | EHCI0 (`0x04100000` is the OTG block)       |
//! | `0x04200000` | EHCI1                                       |
//! | `0x06A00000` | xHCI2, dwc3, combo0 USB3.1 + USB2          |
//! | `0x06B00000` | USB2 PHY, shared aw-phy                     |
//!
//! Register layout from the orange-pi-5.15-sun60iw2 BSP kernel
//! (`ccu-sun60iw2.c`, `sunxi-awphy-plat.c`), SyterKit's sun60iw2 CCU
//! header and live BSP register snapshots.
//!
//! Board facts from Radxa's Cubie A7A DTB (bullseye r6, 5.15.147-21-a733):
//! usb0-vbus PL2 and usb1-vbus PM5, both active high; u2phy
//! `aw,rext_mode = 2`, `aw,phy_tune_param = 0x143338d6`. The EHCI-internal
//! PHY tune at HCI+0x818 and HCI+0x81C are not in the DT; the values below
//! were captured on an Orange Pi 4 Pro and still need an A7A capture.
//!
//! Port wiring (RS501 V1.10 block diagram): USB0/EHCI0 -> USB-C (OTG);
//! USB1/EHCI1 -> the blue USB 3.1 Type-A's high-speed pair; the dwc3's own
//! USB2 PHY -> FE1.1S hub -> the three black USB 2.0 ports and Wi-Fi/BT.
//!
//! combo0 provides the blue port's SuperSpeed pair (usb_combo0.rs). The
//! USB2 hub path uses the real USB2 PHY UTMI clock. A bounded combo0 failure
//! parks only that PHY and retains the explicit CCU PIPE high-speed fallback;
//! the separate readiness/status fields never imply SuperSpeed acceptance.
//! A host reset must complete before EDK2 receives xHCI readiness.
//!
//! The dwc3 follows the order Linux 7.3 (dwc3 core + the board's
//! sun60i-a733 SERDES PHY provider) uses on this board, where the xHCI
//! reset completes in well under a millisecond: PHY interface setup with
//! SUSPHY clear, USB2 PHY power-up, then the USB2 PHY reset-n release,
//! combo0 with its PIPE already selected, a DCTL.CSFTRST core reset that
//! the controller clears once its clock domains are in step, global
//! control and the 24 MHz reference period, and only then HCRST.
//! GCTL.CORESOFTRESET and the GUSB2PHYCFG/GUSB3PIPECTL PHYSOFTRST pulses
//! are not used (Linux replaced them with DCTL.CSFTRST as debug-only); the
//! earlier sequence built on them saw HCRST clear but USBSTS.CNR stay set
//! on every boot, while Linux, starting from exactly that state, resets the
//! controller at once.

use core::ptr::{addr_of, addr_of_mut};

use cubie_a7a_common::time::{ticks, udelay, TIMER_HZ};
use util::mmio::{read32, write32};

use crate::ppu;

#[path = "usb_combo0.rs"]
mod combo0;

const CCU: usize = 0x0200_2000;

// Ordinary R/W bus-master gates. The BSP's
// "key" constants overlap other masters' live gates; preserve those fields.
// USB needs usb-sys/SERDES AHB bits9/8 and MSILITE2/SERDES MBUS bits31/28.
const AHB_GATE: usize = 0x05c0;
const AHB_GATE_USB: u32 = (1 << 9) | (1 << 8);
const MBUS_GATE: usize = 0x05e0;
const MBUS_GATE_USB: u32 = (1 << 31) | (1 << 28);
/// CLK_MSI_LITE2 gate (bit 0), MSI_LITE2_AHB reset (bit 16), MSI_LITE2_MBUS
/// reset (bit 17). The EHCI DT binding (sun60iw2p1.dtsi:2454) needs it;
/// without it USBSTS.PSS/ASS never echo USBCMD.PSE/ASE.
const MSI_LITE2: usize = 0x05a4;

// USB clock + reset registers (BSP ccu-sun60iw2.c):
//   +0x1300 usb_clk parent gate  bit 31 + USB_0_PHY_RSTN bit 30
//   +0x1304 usb0 device bit 8/rst 24, ehci bit 4/rst 20, ohci bit 0/rst 16
//   +0x1308 usb1_clk parent gate bit 31 + USB_1_PHY_RSTN bit 30
//   +0x130C usb1 ehci bit 4/rst 20, ohci bit 0/rst 16
//   +0x1340 usb_ref bit 31
// These take the BSP Linux runtime values directly: the known-good steady
// state.
const USB0_CLK: usize = 0x1300;
const USB0_GATE_RST: usize = 0x1304;
const USB1_CLK: usize = 0x1308;
const USB1_GATE_RST: usize = 0x130c;
const USB_REF: usize = 0x1340;
const USB_CLK_LIVE: u32 = 0xc000_0000; // gate + PHY_RSTN deasserted
const USB_PHY_RSTN: u32 = 1 << 30;
const USB_GATE_RST_LIVE: u32 = 0x0011_0011; // ehci+ohci gates + resets deasserted
const USB_REF_LIVE: u32 = 0x8000_0000;

// USB2 (xHCI2 + its PHY) bus clocks and reset. The PHY MMIO at 0x06B00000
// sits behind this controller's AXI/AHB clock: with these off, its reads
// return 0 and writes are dropped. BSP ccu-sun60iw2.c: usb2_u2_ref 0x1348,
// usb2_suspend 0x1350, usb2_mf (bus) 0x1354, RST_USB_2 0x135C bit 16.
const USB2_U2_REF: usize = 0x1348;
const USB2_SUSPEND: usize = 0x1350;
const USB2_MF: usize = 0x1354;
const USB2_RST: usize = 0x135c;
const GATE: u32 = 1 << 31;
const USB2_RST_BIT: u32 = 1 << 16;

/// CLK_RES_DCAP_24M, the USB2 PHY's mclk (BSP ccu-sun60iw2.c:1785; u2phy
/// `clocks = <&ccu CLK_RES_DCAP_24M>`). Without it all of 0x06B00000
/// reads 0 even though the controllers behind it look alive.
const RES_DCAP: usize = 0x1a00;
const RES_DCAP_USB_PHY: u32 = 1 << 3;

// CLK_SERDES_PHY_CFG and RST_BUS_SERDES drive the Cadence combo PHY
// subsystem; without them the dwc3 bus fabric at 0x06A00000 is gated and
// GCTL reads 0 (BSP sunxi-cadence-combophy.c sunxi_cadence_phy_serdes_init).
const SERDES_PHY_CFG: usize = 0x13c0;
const SERDES_RST: usize = 0x13c4;
const SERDES_RST_BIT: u32 = 1 << 16;

/// Cadence serdes subsystem top, SUBSYS_USB3P1_BGR at +0x8 (BSP
/// combo_usb_clk_set): bit 17 ACLK (AXI), bit 16 HCLK (AHB) gate the dwc3
/// core bus.
const SERDES_USB3P1_BGR: usize = 0x06c0_0000 + 0x0008;
const USB3P1_UTMI_SEL_CCU: u32 = 1 << 21; // 0: USB2 PHY UTMI, 1: USB3-only CCU clock
const USB3P1_PIPE_SEL_U2: u32 = 1 << 20; // PIPE clock: 0 combo PHY, 1 CCU USB2_U2_PIPE
const USB3P1_ACLK_EN: u32 = 1 << 17;
const USB3P1_HCLK_EN: u32 = 1 << 16;
const USB3P1_U2PHY_RSTN: u32 = 1 << 4; // USB2 PHY of the USB3 controller, 1 = released

// USB3 controller clocks without the combo PHY (vendor ccu-sun60iw2.c):
// M[4:0], source [26:24], gate bit 31. U3_UTMI sources: 0 sys24M,
// 1 PERI0 300M; U2_PIPE sources: 0 sys24M, 1 PERI0 480M.
const USB2_U3_UTMI: usize = 0x1360;
const USB2_U2_PIPE: usize = 0x1364;
const fn clk_src(n: u32) -> u32 {
    n << 24
}

/// USB2 PHY (aw-phy v2): per-port registers, stride 0x20.
const USB2_PHY: usize = 0x06b0_0000;
/// PHY+0x18 tune, the A7A DT's `aw,phy_tune_param`.
const U2PHY_TUNE: usize = 0x18;
const U2PHY_TUNE_VALUE: u32 = 0x1433_38d6;

// SYSCFG resistance calibration (BSP sunxi-hci.c usb_phyx_res_cal):
//   RESCAL_CTRL +0x160: CAL_EN bit 0, PHY_o_RES200_SEL(n) bit 4+n,
//                       PCIE_USB_RES200_TRIM_SEL bit 10
//   RES0_CTRL   +0x164: PHY_o_RES200_TRIM(n) = 0xFF << 8n
//   RES1_CTRL   +0x168: PCIE_USB_RES200_TRIM [15:8] (the dwc3's USB2 PHY,
//                       rext_mode 2: BSP phy-sunxi-plat.c phy_rescal_set_v2)
const SYSCFG_RESCAL_CTRL: usize = 0x0300_0000 + 0x160;
const SYSCFG_RES0_CTRL: usize = 0x0300_0000 + 0x164;
const SYSCFG_RES1_CTRL: usize = 0x0300_0000 + 0x168;
const RESCAL_CAL_EN: u32 = 1 << 0;
const RES200_TRIM_SEL: u32 = 1 << 10;
const RES1_PCIE_USB_RES200_TRIM: u32 = 0xff << 8;

/// OTG block; +0x420 bit 0 routes the shared PHY0 to OTG (1) or EHCI0 (0).
const OTG_PHY_CFG: usize = 0x0410_0000 + 0x420;

pub const EHCI0: usize = 0x0410_1000;
pub const EHCI1: usize = 0x0420_0000;
pub const XHCI: usize = 0x06a0_0000;

// EHCI-internal PHY control (BSP sunxi-hci.h, SUN60IW2): +0x800 USB_CTRL
// (PMU IRQ enable / passby), +0x808 CTRL_3 (bit 9 FORCE_SUSPEND), +0x810
// PHY_CTRL (bit 3 SIDDQ), +0x818 PHY tune, +0x81C undocumented, +0x824
// UTMI status.
const HCI_USB_CTRL: usize = 0x800;
const HCI_CTRL_3: usize = 0x808;
const HCI_PHY_CTRL: usize = 0x810;
const HCI_PHY_TUNE: usize = 0x818;
const HCI_81C: usize = 0x81c;
const HCI_UTMI_STAT: usize = 0x824;
const HCI_SIDDQ: u32 = 1 << 3;
const HCI_FORCE_SUSPEND: u32 = 1 << 9;
/// PHY tune, read from a working Linux on an Orange Pi 4 Pro. Without it
/// the EHCI UTMI PHY signal levels are wrong and the controller stays
/// halted.
const HCI_PHY_TUNE_VALUE: u32 = 0x0633_38c6;
/// SUN60IW2 EHCI "passby" / AHB master configuration (BSP sunxi-hci.c
/// __usb_passby): bit 15 bypass OHCI bulk-out changes, bit 11 INCR16,
/// bit 9 INCR4, bit 8 INCRX align, bit 0 on-chip UTMI PHY (HCI0) / ULPI
/// bypass (HCI1); both meanings select the on-chip USB2 PHY.
const HCI_PASSBY: u32 = (1 << 15) | (1 << 11) | (1 << 9) | (1 << 8) | (1 << 0);

// dwc3 globals and xHCI operational registers (Linux drivers/usb/dwc3/core.h).
const DWC3_GCTL: usize = 0xc110;
const GCTL_PWRDNSCALE_MASK: u32 = 0x1fff << 19;
const GCTL_CORESOFTRESET: u32 = 1 << 11;
const GCTL_PRTCAPDIR_MASK: u32 = 3 << 12;
const GCTL_PRTCAPDIR_HOST: u32 = 1 << 12;
const GCTL_SOFITPSYNC: u32 = 1 << 10;
const GCTL_SCALEDOWN_MASK: u32 = 3 << 4;
const GCTL_DISSCRAMBLE: u32 = 1 << 3;
const GCTL_GBLHIBERNATIONEN: u32 = 1 << 1;
const GCTL_DSBLCLKGTNG: u32 = 1 << 0;
const DWC3_GSTS: usize = 0xc118;
const DWC3_GSNPSID: usize = 0xc120;
const DWC3_GUCTL: usize = 0xc12c;
const GUCTL_REFCLKPER_SHIFT: u32 = 22;
const GUCTL_REFCLKPER_MASK: u32 = 0x3ff << GUCTL_REFCLKPER_SHIFT;
const DWC3_GHWPARAMS1: usize = 0xc144;
/// DWC_usb31/32 release number (GSNPSID only names the IP there).
const DWC3_VER_NUMBER: usize = 0xc1a0;
const DWC3_GUSB2PHYCFG: usize = 0xc200;
const GUSB2PHYCFG_PHYSOFTRST: u32 = 1 << 31;
const GUSB2PHYCFG_ENBLSLPM: u32 = 1 << 8;
const GUSB2PHYCFG_SUSPHY: u32 = 1 << 6;
const DWC3_GUSB3PIPECTL: usize = 0xc2c0;
const GUSB3PIPECTL_PHYSOFTRST: u32 = 1 << 31;
const GUSB3PIPECTL_UX_EXIT_PX: u32 = 1 << 27;
const GUSB3PIPECTL_SUSPHY: u32 = 1 << 17;
const DWC3_GFLADJ: usize = 0xc630;
const GFLADJ_240MHZDECR_PLS1: u32 = 1 << 31;
const GFLADJ_240MHZDECR_SHIFT: u32 = 24;
const GFLADJ_240MHZDECR_MASK: u32 = 0x7f << GFLADJ_240MHZDECR_SHIFT;
const GFLADJ_REFCLK_FLADJ_SHIFT: u32 = 8;
const GFLADJ_REFCLK_FLADJ_MASK: u32 = 0x3fff << GFLADJ_REFCLK_FLADJ_SHIFT;
const DWC3_DCTL: usize = 0xc704;
const DCTL_RUN_STOP: u32 = 1 << 31;
const DCTL_CSFTRST: u32 = 1 << 30;
const DWC3_IP: u32 = 0x5533;
const DWC31_IP: u32 = 0x3331;
const DWC32_IP: u32 = 0x3332;
const DWC3_REVISION_250A: u32 = 0x5533_250a;
const DWC31_REVISION_180A: u32 = 0x3138_302a;
const DWC31_REVISION_200A: u32 = 0x3230_302a;
/// The dwc3 ref_clk (USB2_U2_REF), suspend_clk (USB2_SUSPEND) and bus
/// clock all run from sys24M, as the board DT assigns them under Linux.
const DWC3_REF_HZ: u64 = 24_000_000;
// Bounds. Linux allows 10 s for the first xHCI reset and 10 x 20 ms for
// DCTL.CSFTRST; on this board under Linux the reset takes < 0.4 ms.
const CSFTRST_TIMEOUT_US: u32 = 250_000;
const HALT_TIMEOUT_US: u32 = 20_000;
const HCRST_TIMEOUT_US: u32 = 250_000;
const CNR_TIMEOUT_US: u32 = 250_000;
const XHCI_USBCMD: usize = 0x00;
const USBCMD_RS: u32 = 1 << 0;
const USBCMD_HCRST: u32 = 1 << 1;
const XHCI_USBSTS: usize = 0x04;
const USBSTS_HCH: u32 = 1 << 0;
const USBSTS_CNR: u32 = 1 << 11;

// R_PIO (S_GPIO): bank L at +0x00, M at +0x30; CFG0 +0x00 (4 bits per
// pin, 1 = output), DAT +0x10.
const R_PIO: usize = 0x0702_5000;
const BANK_L: usize = 0x00;
const BANK_M: usize = 0x30;
const PIN_USB0_VBUS: u32 = 2; // PL2
const PIN_USB1_VBUS: u32 = 5; // PM5

/// Which host controllers came up, for the payload (DTB `usb-ready`).
#[derive(Clone, Copy, Default)]
pub struct Ready {
    pub ehci0: bool,
    pub ehci1: bool,
    pub xhci: bool,
    pub superspeed: bool,
    /// 0: not attempted, 1: PHY ready, 2/3/4: reset/PLL/PMA failure,
    /// 5: off by build (feature usb-hs-only).
    pub combo0_status: u32,
}

static mut READY: Ready = Ready {
    ehci0: false,
    ehci1: false,
    xhci: false,
    superspeed: false,
    combo0_status: 0,
};

/// Result of the last [`init`].
pub fn ready() -> Ready {
    // SAFETY: written once by the boot CPU in init(), before any reader.
    unsafe { *addr_of!(READY) }
}

fn set(addr: usize, bits: u32) {
    write32(addr, read32(addr) | bits);
}

fn clr(addr: usize, bits: u32) {
    write32(addr, read32(addr) & !bits);
}

fn master_gate(reg: usize, gates: u32) {
    let old = read32(CCU + reg);
    if old & gates != gates {
        write32(CCU + reg, old | gates);
    }
}

fn r_pio_output_high(bank: usize, pin: u32) {
    let cfg = R_PIO + bank + (pin as usize / 8) * 4;
    let shift = (pin % 8) * 4;
    set(R_PIO + bank + 0x10, 1 << pin);
    write32(cfg, (read32(cfg) & !(0xf << shift)) | (1 << shift));
}

/// Microseconds since `start` (a `ticks()` value), saturated.
fn elapsed_us(start: u64) -> u32 {
    let us = ticks().wrapping_sub(start) / (TIMER_HZ / 1_000_000);
    u32::try_from(us).unwrap_or(u32::MAX)
}

/// Poll until `read32(addr) & mask == want`, for at most `timeout_us` of
/// arch-timer time. Returns how long it took, or `None` on timeout.
fn poll(addr: usize, mask: u32, want: u32, timeout_us: u32) -> Option<u32> {
    let start = ticks();
    loop {
        if read32(addr) & mask == want {
            return Some(elapsed_us(start));
        }
        if elapsed_us(start) >= timeout_us {
            // One last look: the deadline may have passed between reads.
            return (read32(addr) & mask == want).then(|| elapsed_us(start));
        }
        udelay(10);
    }
}

fn ccu_init() {
    // 1. AHB and MBUS master gates for the USB and SERDES (dwc3) blocks.
    master_gate(AHB_GATE, AHB_GATE_USB);
    master_gate(MBUS_GATE, MBUS_GATE_USB);
    // 1c. MSI_LITE2 gate and resets.
    set(CCU + MSI_LITE2, (1 << 0) | (1 << 16) | (1 << 17));

    // 2. PHY mclk, without which 0x06B00000 reads 0.
    set(CCU + RES_DCAP, RES_DCAP_USB_PHY);
    udelay(10);

    // 3. USB0 and USB1 parent clock + PHY reset deasserted, then their
    // EHCI/OHCI gates and resets. The PHY resets are cycled again per
    // controller in hci_phy_enable(), after SIDDQ is cleared, matching the
    // BSP's usb_hcd_open_clock order (USBC_Clean_SIDDP, then
    // reset_control_deassert(reset_phy)).
    write32(CCU + USB0_CLK, USB_CLK_LIVE);
    write32(CCU + USB0_GATE_RST, USB_GATE_RST_LIVE);
    write32(CCU + USB1_CLK, USB_CLK_LIVE);
    write32(CCU + USB1_GATE_RST, USB_GATE_RST_LIVE);

    // 5. USB 24 MHz reference.
    write32(CCU + USB_REF, USB_REF_LIVE);

    // 6. USB2 (xHCI2) bus clocks + reset; the USB2 PHY MMIO sits behind
    // this controller's bus.
    set(CCU + USB2_U2_REF, GATE);
    // M[4:0], MUX[24], GATE[31]; mux 1 = sys24M.
    write32(CCU + USB2_SUSPEND, 0x8100_0000);
    // As live Linux: gate only, mux 0 (sys24M).
    write32(CCU + USB2_MF, 0x8000_0000);
    set(CCU + USB2_RST, USB2_RST_BIT);
    udelay(50);

    // 7. Cadence serdes subsystem: bus clocks to the dwc3, exactly the
    // three writes of Linux's sunxi_cadence_phy_serdes_init(). Without
    // them the dwc3 slave at 0x06A00000 is bus-gated (GCTL reads 0, writes
    // dropped). Order: clock gate, reset deassert, subsystem AHB/AXI.
    set(CCU + SERDES_PHY_CFG, GATE);
    udelay(10);
    set(CCU + SERDES_RST, SERDES_RST_BIT);
    udelay(50);
    set(SERDES_USB3P1_BGR, USB3P1_ACLK_EN | USB3P1_HCLK_EN);
    udelay(10);

    // 8. The controller's "only" clocks. U3_UTMI at 24 MHz from sys24M, as
    // the board DT assigns it under Linux (the vendor kernel leaves it
    // gated; the earlier PERI0 300 MHz matched neither). It is not
    // selected while bit 21 is 0. U2_PIPE (PERI0 480M) stays the
    // explicit PIPE clock for the high-speed-only fallback.
    write32(CCU + USB2_U3_UTMI, GATE | clk_src(0));
    write32(CCU + USB2_U2_PIPE, GATE | clk_src(1));
    udelay(10);
    // USB2 needs the real PHY UTMI clock (bit 21 = 0); do not retain a
    // previous USB3-only UTMI selection across warm boot. PIPE stays on the
    // CCU until combo0 is about to start. The USB2 PHY's reset-n is held
    // here and released only after the PHY is powered (u2phy_start()), the
    // vendor kernel's order (combo_usb2_clk_set() runs in the USB3 PHY
    // init, after the USB2 PHY init has cleared SIDDQ).
    let clocks = read32(SERDES_USB3P1_BGR);
    write32(
        SERDES_USB3P1_BGR,
        (clocks & !(USB3P1_UTMI_SEL_CCU | USB3P1_U2PHY_RSTN)) | USB3P1_PIPE_SEL_U2,
    );
    udelay(10);

    // Settle.
    udelay(100);
}

/// BSP usb_phyx_res_cal(enable): clear CAL_EN, set this port's RES200_SEL
/// and clear its TRIM byte. SUN60IW2 keeps the trim in RES0_CTRL (the BSP
/// defines RES200_CTRL at the same address, 0x164).
fn res_cal(port: u32) {
    let v = read32(SYSCFG_RESCAL_CTRL);
    write32(SYSCFG_RESCAL_CTRL, (v & !RESCAL_CAL_EN) | (1 << (4 + port)));
    clr(SYSCFG_RES0_CTRL, 0xff << (8 * port));
}

fn hci_phy_enable(hci: usize, ccu_phy_clk: usize) {
    // BSP order: USBC_Clean_SIDDP() (HCI+0x810 bit 3), then
    // reset_control_deassert(reset_phy) (CCU bit 30). Deasserting the PHY
    // reset in the CCU init alone (before SIDDQ was cleared) left
    // UTMI_PHY_STATUS (+0x824) at 0: no PHY clock. So: assert phy_rst
    // here, clear SIDDQ, then deassert.
    write32(CCU + ccu_phy_clk, USB_CLK_LIVE & !USB_PHY_RSTN);
    udelay(200);

    // Don't keep the PHY parked.
    clr(hci + HCI_CTRL_3, HCI_FORCE_SUSPEND);
    // Power up the PHY.
    clr(hci + HCI_PHY_CTRL, HCI_SIDDQ);
    set(hci + HCI_USB_CTRL, HCI_PASSBY);
    // PHY tune (signal level / drive strength): Orange Pi 4 Pro value.
    write32(hci + HCI_PHY_TUNE, HCI_PHY_TUNE_VALUE);
    // Undocumented +0x81C: live Linux has 0x53 (bits 0, 1, 4, 6, likely
    // UTMI bus/clock enables); with the reset value 0, PSS never echoes.
    write32(hci + HCI_81C, 0x0000_0053);

    // Now deassert the PHY reset; the PHY clock domain should come alive.
    write32(CCU + ccu_phy_clk, USB_CLK_LIVE);
    udelay(1000);
}

fn phy_init() {
    res_cal(0);
    res_cal(1);
    udelay(10);

    // USBC_SelectPhyToHci (BSP sunxi-hci.c): route the OTG-shared PHY0 to
    // EHCI0 by clearing OTG+0x420 bit 0 (live Linux ends at 0x40000000; bit
    // 30 comes from the MUSB bring-up, not from this). It must happen
    // before EHCI0's PHY reset/SIDDQ sequence, or the OTG block still owns
    // PHY0 and EHCI0's UTMI clock never starts.
    clr(OTG_PHY_CFG, 1 << 0);

    hci_phy_enable(EHCI0, USB0_CLK);
    hci_phy_enable(EHCI1, USB1_CLK);

    // The dwc3's own USB2 PHY (0x06B00000); its reset-n is released at the
    // end, after the PHY is powered.
    if !u2phy_start(true) {
        println!(
            "[main] USB: USB2 PHY writes did not stick (+0x18 = {:#010x}); PHY block gated?",
            read32(USB2_PHY + U2PHY_TUNE)
        );
    }

    // VBUS: usb0-vbus (PL2) feeds the EHCI ports, usb1-vbus (PM5) the
    // xHCI side; both fixed regulators are active high, 1 ms ramp in the
    // vendor DT.
    r_pio_output_high(BANK_L, PIN_USB0_VBUS);
    r_pio_output_high(BANK_M, PIN_USB1_VBUS);
    udelay(5000); // VBUS rise + device debounce
}

/// USB2 PHY core at 0x06B00000, the dwc3's UTMI PHY (BSP phy-sunxi-plat.c
/// sunxi_phy_init() for the A7A's `aw,rext_mode = 2`, as Linux's
/// sun60i_a733_serdes a7a_usb2_start()): RESCAL_CTRL CAL_EN clear and
/// PCIE_USB_RES200_TRIM_SEL, the RES1_CTRL trim byte cleared, then per port
/// PHY+0x10 (SIDDQ clear, live 0x000E2430) and PHY+0x18 (the DT's
/// aw,phy_tune_param 0x143338D6). Live Linux has SYSCFG+0x160 =
/// 0x00C81532.
///
/// With `release`, the PHY's reset-n (SERDES USB3P1_BGR bit 4) is released
/// after the PHY is powered up, the vendor kernel's order; a repeat (the
/// xHCI retry) leaves it released, as Linux does. Returns whether the tune
/// register kept its value.
fn u2phy_start(release: bool) -> bool {
    let v = read32(SYSCFG_RESCAL_CTRL);
    write32(SYSCFG_RESCAL_CTRL, (v & !RESCAL_CAL_EN) | RES200_TRIM_SEL);
    clr(SYSCFG_RES1_CTRL, RES1_PCIE_USB_RES200_TRIM);
    let program = || {
        write32(USB2_PHY + 0x10, 0x000e_2430);
        write32(USB2_PHY + U2PHY_TUNE, U2PHY_TUNE_VALUE);
        write32(USB2_PHY + 0x30, 0x000e_2430);
        write32(USB2_PHY + 0x38, U2PHY_TUNE_VALUE);
        udelay(10);
        read32(USB2_PHY + U2PHY_TUNE) == U2PHY_TUNE_VALUE
    };
    let mut ok = program();
    if release {
        set(SERDES_USB3P1_BGR, USB3P1_U2PHY_RSTN);
        // PHY PLL lock and UTMI clock start, as for the EHCI PHYs.
        udelay(1000);
        if !ok {
            // Were the PHY registers held by reset-n after all? Program
            // them again now that it is released (the earlier order).
            ok = program();
        }
    }
    ok
}

/// An EHCI is usable if its capability registers read back sanely.
fn ehci_alive(base: usize) -> bool {
    let cap = read32(base);
    let caplength = cap & 0xff;
    (0x10..0x40).contains(&caplength) && cap >> 16 != 0 && cap != 0xffff_ffff
}

/// The xHCI operational register base, if CAPLENGTH is plausible. EL3 runs
/// with the MMU off, where every access is Device memory and must be
/// aligned, so an odd CAPLENGTH is rejected too.
fn xhci_op() -> Option<usize> {
    let caplength = (read32(XHCI) & 0xff) as usize;
    ((0x20..0xff).contains(&caplength) && caplength.is_multiple_of(4)).then_some(XHCI + caplength)
}

/// The dwc3 IP (GSNPSID[31:16]) and release, as Linux's
/// dwc3_core_is_valid() reads them.
#[derive(Clone, Copy)]
struct Dwc3Version {
    ip: u32,
    rev: u32,
}

fn dwc3_version() -> Option<Dwc3Version> {
    let id = read32(XHCI + DWC3_GSNPSID);
    match id >> 16 {
        DWC3_IP => Some(Dwc3Version {
            ip: DWC3_IP,
            rev: id,
        }),
        ip @ (DWC31_IP | DWC32_IP) => Some(Dwc3Version {
            ip,
            rev: read32(XHCI + DWC3_VER_NUMBER),
        }),
        _ => None,
    }
}

/// Halt a host controller an earlier stage left running. Operational
/// registers other than USBSTS may only be written once CNR is clear.
fn xhci_quiesce(op: usize) {
    if read32(op + XHCI_USBSTS) & USBSTS_CNR == 0 && read32(op + XHCI_USBCMD) & USBCMD_RS != 0 {
        clr(op + XHCI_USBCMD, USBCMD_RS);
        let _ = poll(op + XHCI_USBSTS, USBSTS_HCH, USBSTS_HCH, HALT_TIMEOUT_US);
    }
}

/// Linux dwc3_phy_setup() for the board DT (snps,dis_u2_susphy_quirk,
/// snps,dis_enblslpm_quirk, no phy_type): both PHY interfaces out of
/// suspend before the PHYs are initialised, UX_EXIT_PX clear. Also drops
/// reset bits an older firmware may have left set; nothing here pulses a
/// reset.
fn dwc3_phy_setup() {
    let b = XHCI;
    clr(b + DWC3_GCTL, GCTL_CORESOFTRESET);
    clr(
        b + DWC3_GUSB3PIPECTL,
        GUSB3PIPECTL_PHYSOFTRST | GUSB3PIPECTL_UX_EXIT_PX | GUSB3PIPECTL_SUSPHY,
    );
    clr(
        b + DWC3_GUSB2PHYCFG,
        GUSB2PHYCFG_PHYSOFTRST | GUSB2PHYCFG_SUSPHY | GUSB2PHYCFG_ENBLSLPM,
    );
}

/// Linux dwc3_core_soft_reset() as dwc3_core_init() runs it on first
/// probe: DCTL.CSFTRST with run/stop clear, which the core clears once all
/// its clock domains (bus, ref, suspend, UTMI, PIPE) have been reset in
/// step. Returns the time it took.
fn dwc3_core_soft_reset(v: Dwc3Version) -> Option<u32> {
    let b = XHCI;
    let dctl = read32(b + DWC3_DCTL);
    write32(b + DWC3_DCTL, (dctl & !DCTL_RUN_STOP) | DCTL_CSFTRST);
    let t = poll(b + DWC3_DCTL, DCTL_CSFTRST, 0, CSFTRST_TIMEOUT_US)?;
    // DWC_usb31 1.80a and prior: wait 50 ms before touching the PHY domain.
    if v.ip == DWC31_IP && v.rev <= DWC31_REVISION_180A {
        udelay(50_000);
    }
    Some(t)
}

// Linux dwc3_ref_clk_period() for a 24 MHz ref_clk.
const REF_PERIOD_NS: u64 = 1_000_000_000 / DWC3_REF_HZ;
const REF_FLADJ: u64 = 125_000 * 1_000_000_000 / (DWC3_REF_HZ * REF_PERIOD_NS) - 125_000;
const REF_240MHZ_DECR: u64 = 480_000_000 / DWC3_REF_HZ;
// Linux dwc3_set_power_down_clk_scale(): suspend_clk periods per 16 kHz.
const PWRDNSCALE: u32 = DWC3_REF_HZ.div_ceil(16_000) as u32;

/// Linux dwc3_core_setup_global_control(), dwc3_set_power_down_clk_scale(),
/// dwc3_ref_clk_period() and dwc3_set_prtcap(HOST), in that order.
fn dwc3_global_setup(v: Dwc3Version) {
    let b = XHCI;
    let power_opt = (read32(b + DWC3_GHWPARAMS1) >> 24) & 3;
    let mut gctl = read32(b + DWC3_GCTL) & !(GCTL_SCALEDOWN_MASK | GCTL_DISSCRAMBLE);
    match power_opt {
        // Clock gating. Host mode on DWC_usb3 2.10a..2.50a keeps it off.
        1 if v.ip == DWC3_IP && (0x5533_210a..=DWC3_REVISION_250A).contains(&v.rev) => {
            gctl |= GCTL_DSBLCLKGTNG | GCTL_SOFITPSYNC;
        }
        1 => gctl &= !GCTL_DSBLCLKGTNG,
        2 => gctl |= GCTL_GBLHIBERNATIONEN,
        _ => {}
    }
    if power_opt != 0 && v.ip == DWC31_IP && v.rev == DWC31_REVISION_200A {
        gctl |= GCTL_DSBLCLKGTNG;
    }
    write32(b + DWC3_GCTL, gctl);

    let scale = (read32(b + DWC3_GCTL) & GCTL_PWRDNSCALE_MASK) >> 19;
    if !(PWRDNSCALE..=PWRDNSCALE * 3).contains(&scale) {
        let gctl = read32(b + DWC3_GCTL) & !GCTL_PWRDNSCALE_MASK;
        write32(b + DWC3_GCTL, gctl | (PWRDNSCALE << 19));
    }

    let guctl = read32(b + DWC3_GUCTL) & !GUCTL_REFCLKPER_MASK;
    write32(
        b + DWC3_GUCTL,
        guctl | ((REF_PERIOD_NS as u32) << GUCTL_REFCLKPER_SHIFT),
    );
    if !(v.ip == DWC3_IP && v.rev < DWC3_REVISION_250A) {
        let fladj = read32(b + DWC3_GFLADJ)
            & !(GFLADJ_REFCLK_FLADJ_MASK | GFLADJ_240MHZDECR_MASK | GFLADJ_240MHZDECR_PLS1);
        write32(
            b + DWC3_GFLADJ,
            fladj
                | ((REF_FLADJ as u32) << GFLADJ_REFCLK_FLADJ_SHIFT)
                | (((REF_240MHZ_DECR >> 1) as u32) << GFLADJ_240MHZDECR_SHIFT)
                | if REF_240MHZ_DECR & 1 != 0 {
                    GFLADJ_240MHZDECR_PLS1
                } else {
                    0
                },
        );
    }

    let gctl = read32(b + DWC3_GCTL);
    write32(
        b + DWC3_GCTL,
        (gctl & !GCTL_PRTCAPDIR_MASK) | GCTL_PRTCAPDIR_HOST,
    );
}

/// What the xHCI bring-up measured, for the log.
struct XhciTimes {
    csftrst_us: u32,
    hcrst_us: u32,
    cnr_us: u32,
}

/// Core reset, global setup and host mode, then one xHCI reset (Linux
/// xhci_gen_setup(): halt, HCRST, wait for HCRST and CNR to clear) to prove
/// the controller and its PHY clocks are alive. The PHYs are already up.
fn dwc3_host_init(v: Dwc3Version, op: usize, superspeed: bool) -> Result<XhciTimes, &'static str> {
    let b = XHCI;
    let csftrst_us = dwc3_core_soft_reset(v).ok_or("DCTL.CSFTRST did not clear")?;
    dwc3_global_setup(v);

    clr(op + XHCI_USBCMD, USBCMD_RS);
    if poll(op + XHCI_USBSTS, USBSTS_HCH, USBSTS_HCH, HALT_TIMEOUT_US).is_none() {
        return Err("did not halt");
    }
    set(op + XHCI_USBCMD, USBCMD_HCRST);
    let hcrst_us = poll(op + XHCI_USBCMD, USBCMD_HCRST, 0, HCRST_TIMEOUT_US)
        .ok_or("HCRST did not clear within 250 ms")?;
    let cnr_us = poll(op + XHCI_USBSTS, USBSTS_CNR, 0, CNR_TIMEOUT_US)
        .ok_or("CNR did not clear within 250 ms")?;

    // A working combo0 remains awake for SuperSpeed. Park only the absent
    // interface on the explicit high-speed failure fallback.
    if !superspeed {
        set(b + DWC3_GUSB3PIPECTL, GUSB3PIPECTL_SUSPHY);
    }
    Ok(XhciTimes {
        csftrst_us,
        hcrst_us,
        cnr_us,
    })
}

/// combo0 for the blue port's SuperSpeed pair, with the dwc3 PIPE input
/// already switched to it (Linux a7a_usb_start(): bits 21/20 clear before
/// the PHY reset sequence). On failure the PIPE input goes back to the CCU
/// clock: the explicit high-speed-only fallback. Returns the DT status.
fn combo0_start() -> u32 {
    // Feature usb-hs-only: combo0 never starts (no SuperSpeed receiver
    // terminations on the blue port), the PIPE input is the CCU's as on the
    // failure fallback. DT status 5.
    if cfg!(feature = "usb-hs-only") {
        println!("[main] USB: combo0 off (usb-hs-only): blue port high speed on EHCI1");
        set(SERDES_USB3P1_BGR, USB3P1_PIPE_SEL_U2);
        udelay(10);
        return 5;
    }
    clr(SERDES_USB3P1_BGR, USB3P1_PIPE_SEL_U2 | USB3P1_UTMI_SEL_CCU);
    udelay(10);
    match combo0::init() {
        Ok(()) => 1,
        Err(e) => {
            println!("[main] USB: combo0 {e:?}; explicit high-speed-only fallback");
            // combo0::init parks only its own PHY; select the CCU PIPE.
            set(SERDES_USB3P1_BGR, USB3P1_PIPE_SEL_U2);
            udelay(10);
            match e {
                combo0::Error::Reset => 2,
                combo0::Error::PllEnable => 3,
                combo0::Error::PmaReady => 4,
            }
        }
    }
}

/// Enough state to tell a missing UTMI clock from a missing PIPE clock
/// without a later read from the OS.
fn xhci_log_failure(why: &str, attempt: u32, combo0_status: u32) {
    let usbsts = xhci_op().map_or(0xffff_ffff, |op| read32(op + XHCI_USBSTS));
    println!(
        "[main] USB: xHCI {why}, attempt {attempt} (CAP {:#010x}, GCTL {:#010x}, DCTL {:#010x})",
        read32(XHCI),
        read32(XHCI + DWC3_GCTL),
        read32(XHCI + DWC3_DCTL)
    );
    println!(
        "[main] USB: xHCI USBSTS {:#010x} GSTS {:#010x} GUSB2PHYCFG {:#010x} GUSB3PIPECTL {:#010x} USB3P1_BGR {:#010x} combo0 {}",
        usbsts,
        read32(XHCI + DWC3_GSTS),
        read32(XHCI + DWC3_GUSB2PHYCFG),
        read32(XHCI + DWC3_GUSB3PIPECTL),
        read32(SERDES_USB3P1_BGR),
        combo0_status
    );
}

/// Bring up all USB host controllers. Never fails the boot: what did not
/// come up is logged and left out of [`ready`].
pub fn init() {
    // Boot CPU only; a failed repeat must not retain an earlier readiness.
    unsafe { *addr_of_mut!(READY) = Ready::default() };
    // The USB2 PHY block is in PCK-600 domain 8, off at cold boot; until it
    // is on, the PHY MMIO at 0x06B00000 drops writes and reads 0.
    if let Err(e) = ppu::power_on(ppu::USB2) {
        println!("[main] USB: USB2 power domain: {e:?}; controllers withheld");
        // Do not access PHY/controller registers behind a domain that failed
        // to power on. No USB readiness has been published.
        return;
    }
    ccu_init();
    // The dwc3 bus is clocked now. Identify it and set its PHY interfaces
    // up before any PHY is initialised (Linux dwc3_core_init() order).
    let dwc3 = match (xhci_op(), dwc3_version()) {
        (Some(op), Some(v)) => {
            xhci_quiesce(op);
            dwc3_phy_setup();
            Some((op, v))
        }
        _ => None,
    };
    phy_init();
    // Do not change the shared SERDES reset: combo1 may own the PCIe link.
    let mut combo0_status = combo0_start();

    let mut r = Ready {
        ehci0: ehci_alive(EHCI0),
        ehci1: ehci_alive(EHCI1),
        xhci: false,
        superspeed: false,
        combo0_status,
    };
    match dwc3 {
        None => {
            println!(
                "[main] USB: xHCI not a dwc3 (CAP {:#010x}, GSNPSID {:#010x}); not offered",
                read32(XHCI),
                read32(XHCI + DWC3_GSNPSID)
            );
        }
        Some((op, v)) => {
            for attempt in 1..=2 {
                match dwc3_host_init(v, op, combo0_status == 1) {
                    Ok(t) => {
                        println!(
                            "[main] USB: dwc3 {:#06x} rev {:#010x}: CSFTRST {} us, HCRST {} us, CNR {} us (attempt {attempt})",
                            v.ip, v.rev, t.csftrst_us, t.hcrst_us, t.cnr_us
                        );
                        r.xhci = true;
                        r.superspeed = combo0_status == 1;
                        break;
                    }
                    Err(why) => {
                        xhci_log_failure(why, attempt, combo0_status);
                        if attempt == 1 {
                            // What Linux does over a controller the firmware
                            // left un-ready, and which then resets at once:
                            // USB2 PHY and combo0 again, then the core reset.
                            u2phy_start(false);
                            if combo0_status == 1 {
                                combo0_status = combo0_start();
                            }
                        }
                    }
                }
            }
            if !r.xhci {
                println!("[main] USB: xHCI not offered");
            }
        }
    }
    r.combo0_status = combo0_status;
    println!(
        "[main] USB: EHCI0 {} (UTMI {:#x}), EHCI1 {} (UTMI {:#x}), xHCI {}",
        if r.ehci0 { "up" } else { "dead" },
        read32(EHCI0 + HCI_UTMI_STAT),
        if r.ehci1 { "up" } else { "dead" },
        read32(EHCI1 + HCI_UTMI_STAT),
        if r.superspeed {
            "up (combo0 SuperSpeed)"
        } else if r.xhci {
            "up (high-speed fallback)"
        } else {
            "dead"
        },
    );
    // SAFETY: boot CPU only, before build_dtb reads it.
    unsafe { *addr_of_mut!(READY) = r };
}
