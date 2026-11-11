// SPDX-License-Identifier: GPL-2.0-only
//! fastboot for the youyeetoo YY3588 (RK3588) bt0 when the BootROM started
//! it over MaskROM USB (`rkdeveloptool db`), on the USB-C OTG port
//! (USB3OTG0, a DWC3, driven by crate `dwc3-udc`).
//!
//! - [`platform`]: the SoC part of the bring-up ([`platform::Otg0`], a
//!   `dwc3_udc::Dwc3Platform`): clock gates, the SuperSpeed port routed
//!   away (High-Speed only), the USB 2.0 PHY out of software suspend, the
//!   VBUS indication the Type-C port does not give the PHY.
//! - [`device`]: the fastboot device ([`device::Rk3588Device`], a
//!   `fastboot::Device`): `getvar`, `flash:<target>` for the eMMC targets of
//!   the K1 bt0 plus two RK3588 regions, `boot` / `stage`+`continue` of a
//!   downloaded FIT, `reboot`; the eMMC write and the reset are the board's
//!   [`device::Board`] hook.
//! - [`stage`]: placing a FIT that sits in memory (the download buffer) and
//!   entering TF-A BL31, with crate `rk3588`'s `fit` and `handoff`, the way
//!   bt0's eMMC path does (boards/rk3588/yy3588/bt0/src/boot.rs).
//!
//! The protocol layer is crate `fastboot` (first written for the SpacemiT
//! K1 bt0, whose command set, target names, IDs and download buffer this
//! mirrors: oreboot src/mainboard/spacemit/k1x/bt0/src/fastboot.rs).
#![no_std]

pub mod device;
pub mod platform;
pub mod stage;

/// USB3OTG0 controller base, TRM Part 2 Table 13-1 (p.2187).
pub const USB3OTG0_BASE: usize = 0xfc00_0000;

/// The fastboot download buffer in DRAM, and its size: the K1 bt0's
/// (fastboot.rs BUF = 0x1000_0000, MAX_DOWNLOAD = 0x2000_0000). Outside
/// bt0 (0..0x30000), the dram-info record (0x1F0000), every FIT
/// destination of edk2-rk3588 (0x40000, 0x200000.., 0x7C0000, PMU SRAM),
/// bt0's FDT/bl_params (0x3000000/0x3100000) and its eMMC staging buffer
/// (0x4000000 + 8 MiB): boot.rs memory map.
pub const DOWNLOAD_BUF: usize = 0x1000_0000;
pub const DOWNLOAD_MAX: usize = 0x2000_0000;

/// The download buffer size for `dram_bytes` of DRAM: [`DOWNLOAD_MAX`],
/// or what is left above [`DOWNLOAD_BUF`] (whole MiB) on a smaller part,
/// or 0 when DRAM ends below it.
pub fn download_size(dram_bytes: u64) -> usize {
    let top = dram_bytes.min(rk3588::handoff::MMIO_HOLE);
    let room = top.saturating_sub(DOWNLOAD_BUF as u64) as usize;
    room.min(DOWNLOAD_MAX) & !0xf_ffff
}

/// Where the BootROM leaves the boot source id: SYSTEM_SRAM + 0x10
/// (mainline U-Boot arch/arm/include/asm/arch-rockchip/bootrom.h
/// BROM_BOOTSOURCE_ID_ADDR = CFG_IRAM_BASE + 0x10; include/configs/
/// rk3588_common.h CFG_IRAM_BASE = 0xff000000). Mainline U-Boot's SPL
/// reads it after the rkbin DDR init ran, so that init keeps it [I].
pub const BOOTSOURCE_ID: usize = 0xff00_0010;

/// The id means "started over USB" (MaskROM download): 10, or the raw
/// 0x81 that U-Boot arch/arm/mach-rockchip/spl.c read_brom_bootsource_id
/// maps to BROM_BOOTSOURCE_USB.
pub fn launched_over_usb(id: u32) -> bool {
    id == 10 || id == 0x81
}

/// The USB IDs and strings: the fastboot tool's and U-Boot's IDs, as the
/// K1 bt0 uses them (18d1:4ee0, interface ff/42/03).
pub fn descriptors<'a>(product: &'a str, serial: &'a str) -> dwc3_udc::Descriptors<'a> {
    dwc3_udc::Descriptors {
        vendor: 0x18d1,
        product: 0x4ee0,
        class: [0xff, 0x42, 0x03],
        strings: ["oreboot", product, serial, "fastboot"],
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn download_buffer_fits_the_dram() {
        assert_eq!(download_size(16 << 30), DOWNLOAD_MAX);
        assert_eq!(download_size(4 << 30), DOWNLOAD_MAX);
        assert_eq!(download_size(512 << 20), 256 << 20);
        assert_eq!(download_size(256 << 20), 0);
    }
    #[test]
    fn usb_boot_source() {
        assert!(launched_over_usb(0x81) && launched_over_usb(10));
        assert!(!launched_over_usb(2) && !launched_over_usb(5));
    }
}
