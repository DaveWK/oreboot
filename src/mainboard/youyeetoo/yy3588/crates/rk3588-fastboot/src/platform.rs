// SPDX-License-Identifier: GPL-2.0-only
//! The RK3588 side of the USB3OTG0 device-mode bring-up.
//!
//! What the BootROM leaves configured after a MaskROM USB download [I]:
//! it served the Rockusb download device on this very controller and USB
//! 2.0 PHY until it jumped to bt0 (the 471 DDR init and then the 472 bt0
//! image arrive over it), so PD_USB is powered, the controller's clocks
//! run and its resets are released, and the PHY is powered and clocked.
//! Nothing documents this (the TRM describes no BootROM state; the public
//! rkbin/rkdeveloptool material is binaries we do not read), so bt0 does
//! not rely on it where a write is idempotent: the steps below only ungate
//! clocks and select documented modes. It does not toggle resets (a PHY
//! reset needs its own sequence and is not needed for a running PHY) and
//! does not touch the USBDP combo PHY at all: the SuperSpeed port is
//! disabled in USB_GRF, which mainline U-Boot also does for a
//! High-Speed-only port instead of initialising the USBDP PHY
//! (drivers/phy/rockchip/phy-rockchip-usbdp.c rockchip_u3phy_init when
//! `udphy->hs`).
//!
//! Steps, with sources ("TRM p.N" = RK3588 TRM Part 1 V1.0):
//! 1. Clock gates off ("when high, disable clock"; write-masked):
//!    CRU_GATE_CON42 (0xFD7C0000 + 0x08A8) aclk/suspend/ref_clk_usb3otg0
//!    [4]/[5]/[6] (p.145); CRU_GATE_CON72 (+0x0920) pclk_usb2phy_u3_0_grf0
//!    [8] (p.165); PMU1CRU_GATE_CON04 (0xFD7F0000 + 0x0810)
//!    clk_otgphy_u3_0 [7] (p.351; the clock of the reset the DT names as
//!    the USB2 PHY's "phy" reset, SRST_OTGPHY_U3_0 [I]). CRU bases p.29.
//! 2. USB_GRF_USB3OTG0_CON1 (0xFD5AC000 + 0x1C) = 0x0188, all 16 bits
//!    write-enabled: no xHCI USB3 port, one USB2 port, PIPE clocks from the
//!    UTMI clock, PIPE PhyStatus forced 0, USB3 SS port disabled (p.811);
//!    the value is U-Boot/Linux phy-rockchip-usbdp.c rk3588 `usb3otg0_cfg`
//!    "enable" (= U3 port disabled). Must precede the core soft reset,
//!    which needs the PIPE clock otherwise.
//! 3. USB2PHY0_GRF (0xFD5D0000, p.685) CON2 (+0x08) SIDDQ [13] = 0, analog
//!    powered (p.945; Linux phy-rockchip-inno-usb2.c rk3588_usb2phy_tuning
//!    "Deassert SIDDQ"); CON3 (+0x0C) [11] = 0, SUSPENDM from the
//!    controller instead of the GRF (p.947; U-Boot
//!    phy-rockchip-inno-usb2.c rk3588 `phy_sus` power-on), then 2 ms for
//!    the UTMI clock (U-Boot usleep_range(1500, 2000)).
//! 4. VBUS indication ([`Otg0::force_vbus`], default on): the YY3588
//!    USB-C port is managed by an FUSB302 (mainline DTS
//!    rk3588-youyeetoo-yy3588.dts `usbc0`), and Linux then reports VBUS to
//!    the PHY and controller through the GRF rather than the PHY's own
//!    comparator: U-Boot/Linux phy-rockchip-usbdp.c rk3588 `bvalid_phy_con`
//!    = USB2PHY_GRF_CON2 [1:0] = 3 (VBUSVLDEXTSEL, VBUSVLDEXT: "VBUS valid,
//!    D+ pull-up enabled", pp.946-947) and `bvalid_grf_con` =
//!    USB2PHY_GRF_CON4 [3:2] = 3 (sft_vbus_sel, sft_vbus, pp.948-949).
//! 5. HS eye tuning ([`Otg0::tune_eye`], default off: the BootROM ran
//!    High-Speed without it [I]): USB2PHY_GRF_CON1 txvreftune [11:8] = 9
//!    (+5.89 %) and CON2 txpreempamptune [4:3] = 2 (2x), pp.944/946, the
//!    values of Linux rk3588_usb2phy_tuning.
//!
//! bt0 never drives the Type-C VBUS switch (vbus5v0_typec, GPIO4_B0): the
//! board is the USB device here, the host supplies VBUS.

use dwc3_udc::{Dwc3Platform, PhyTimeout};
use regio::RegIo;
use rk3588::regs::wm;

pub const CRU: usize = 0xfd7c_0000;
pub const CRU_GATE_CON42: usize = CRU + 0x08a8;
pub const CRU_GATE_CON72: usize = CRU + 0x0920;
pub const PMU1CRU: usize = 0xfd7f_0000;
pub const PMU1CRU_GATE_CON04: usize = PMU1CRU + 0x0810;
pub const USB_GRF: usize = 0xfd5a_c000;
pub const USB_GRF_USB3OTG0_CON1: usize = USB_GRF + 0x001c;
/// U3 port disabled, PIPE from UTMI (see step 2).
pub const USB3OTG0_CON1_U2_ONLY: u32 = 0xffff_0188;
pub const USB2PHY0_GRF: usize = 0xfd5d_0000;
pub const USB2PHY_CON1: usize = 0x0004;
pub const USB2PHY_CON2: usize = 0x0008;
pub const USB2PHY_CON3: usize = 0x000c;
pub const USB2PHY_CON4: usize = 0x0010;
pub const USB2PHY_STATUS0: usize = 0x00c0;

/// USB3OTG0 + USB2PHY0 for `dwc3_udc`.
#[derive(Clone, Copy, Debug)]
pub struct Otg0 {
    /// Report VBUS valid / B-session valid through the GRF (step 4).
    pub force_vbus: bool,
    /// Apply Linux's HS eye tuning (step 5).
    pub tune_eye: bool,
}

impl Default for Otg0 {
    fn default() -> Self {
        Self {
            force_vbus: true,
            tune_eye: false,
        }
    }
}

impl Otg0 {
    /// The GRF/CRU program of [`Dwc3Platform::bring_up`].
    pub fn program<IO: RegIo>(&self, io: &mut IO) {
        io.write32(CRU_GATE_CON42, wm(4, 3, 0));
        io.write32(CRU_GATE_CON72, wm(8, 1, 0));
        io.write32(PMU1CRU_GATE_CON04, wm(7, 1, 0));
        io.write32(USB_GRF_USB3OTG0_CON1, USB3OTG0_CON1_U2_ONLY);
        let phy = USB2PHY0_GRF;
        io.write32(phy + USB2PHY_CON2, wm(13, 1, 0));
        io.write32(phy + USB2PHY_CON3, wm(11, 1, 0));
        io.udelay(2_000);
        if self.force_vbus {
            io.write32(phy + USB2PHY_CON2, wm(0, 2, 3));
            io.write32(phy + USB2PHY_CON4, wm(2, 2, 3));
        }
        if self.tune_eye {
            io.write32(phy + USB2PHY_CON1, wm(8, 4, 9));
            io.write32(phy + USB2PHY_CON2, wm(3, 2, 2));
        }
        // USB2PHY_GRF_STATUS0 (p.950-951): OTG session valid [11], line
        // state [10:9], UTMI VBUS valid [8]: a report for the console.
        let s = io.read32(phy + USB2PHY_STATUS0);
        io.log(format_args!(
            "[usb] usb2phy0 status {s:#06x}: sessvld {} vbusvalid {} linestate {}\n",
            (s >> 11) & 1,
            (s >> 8) & 1,
            (s >> 9) & 3
        ));
    }
}

/// Order the CPU's writes (TRBs, buffers) before the controller's doorbell:
/// bt0 runs with the MMU and data cache off, so no cache maintenance is
/// needed, only the barrier.
fn barrier() {
    #[cfg(target_arch = "aarch64")]
    // SAFETY: a barrier, no memory or register effects.
    unsafe {
        core::arch::asm!("dsb sy", options(nostack, preserves_flags))
    };
    #[cfg(not(target_arch = "aarch64"))]
    core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);
}

impl Dwc3Platform for Otg0 {
    fn bring_up<IO: RegIo>(&mut self, io: &mut IO) -> Result<(), PhyTimeout> {
        self.program(io);
        Ok(())
    }
    fn flush_dcache(&mut self, _addr: usize, _len: usize) {
        barrier();
    }
    fn invalidate_dcache(&mut self, _addr: usize, _len: usize) {
        barrier();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use regio::mock::Mock;

    #[test]
    fn program_writes_the_documented_values() {
        let mut m: Mock<16> = Mock::default();
        Otg0::default().program(&mut m);
        assert_eq!(m.get(CRU_GATE_CON42), 0x0070_0000);
        assert_eq!(m.get(CRU_GATE_CON72), 0x0100_0000);
        assert_eq!(m.get(PMU1CRU_GATE_CON04), 0x0080_0000);
        assert_eq!(m.get(USB_GRF_USB3OTG0_CON1), 0xffff_0188);
        // The last CON2 write is the VBUS override; CON3 bit 11 cleared.
        assert_eq!(m.get(USB2PHY0_GRF + USB2PHY_CON2), 0x0003_0003);
        assert_eq!(m.get(USB2PHY0_GRF + USB2PHY_CON3), 0x0800_0000);
        assert_eq!(m.get(USB2PHY0_GRF + USB2PHY_CON4), 0x000c_000c);
        assert_eq!(m.get(USB2PHY0_GRF + USB2PHY_CON1), 0);
        let mut m: Mock<16> = Mock::default();
        Otg0 {
            force_vbus: false,
            tune_eye: true,
        }
        .program(&mut m);
        assert_eq!(m.get(USB2PHY0_GRF + USB2PHY_CON1), 0x0f00_0900);
        assert_eq!(m.get(USB2PHY0_GRF + USB2PHY_CON2), 0x0018_0010);
        assert_eq!(m.get(USB2PHY0_GRF + USB2PHY_CON4), 0);
    }
}
