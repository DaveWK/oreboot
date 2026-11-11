# YY3588 bt0: fastboot over MaskROM USB

When the BootROM starts bt0 over USB (`rkdeveloptool db <loader>`, bt0 as
the loader's 472 image after Rockchip's DDR init as 471), bt0 serves
fastboot on the **USB-C OTG port** (USB3OTG0, a Synopsys DWC3, High-Speed
only) as the K1 bt0 does from its USB download mode: stage and boot a FIT
from RAM, or write the eMMC. Hardware (2026-11-09): enumeration
("configured"), `download` and `boot` up to the FIT check ran; the boot
stopped at `FAILFIT data past the download` (fixed below, not yet re-run).
Booting BL31/EDK2 from RAM and `flash` are host-tested only.

Core revision: the RK3588 USB3OTG0 reads GSNPSID 0x5533300a (DWC_usb3 3.00a,
first hardware run); `crates/dwc3-udc` documents the Linux revision checks
it reviewed (none changes its program between 3.00a and 3.30a).

Code: `src/usb.rs` (this hook), `crates/dwc3-udc` (the DWC3 device-mode
driver), `crates/rk3588-fastboot` (RK3588 PHY/GRF bring-up, the fastboot
device, FIT boot from memory), `crates/fastboot` (the protocol loop, shared
with the K1). Sources are cited in each crate.

## How bt0 gets there

- **Build-time selection (what is wired now):** feature `usb-loader`
  (implies `stepping-stone`). `main.rs` calls `usb::enter(&mut io)` after
  dram-info and the DRAM quick check, instead of the eMMC boot.
- **Runtime detection (available, not wired):** the BootROM leaves its
  boot source id at `0xFF000010` (SYSTEM_SRAM + 0x10). The values 10 or
  0x81 mean USB, as mainline U-Boot's SPL reads it after the rkbin DDR init
  (`bootrom.h` BROM_BOOTSOURCE_ID_ADDR, `spl.c` read_brom_bootsource_id).
  `rk3588_fastboot::{BOOTSOURCE_ID, launched_over_usb}` decode it.
  `usb::enter` prints it, so the first hardware run tells whether one
  stepping-stone bt0 can choose fastboot or eMMC by itself. That it
  survives the rkbin DDR init is inferred from U-Boot relying on it [I].

## The interface

```rust
// src/usb.rs, in short
let udc = dwc3_udc::Udc::init(Mmio { uart: Uart2 }, Otg0::default(),
    dwc3_udc::Config::new(USB3OTG0_BASE), &mut DMA,
    rk3588_fastboot::descriptors("yy3588", "rk3588"))?;
let mut dev = Rk3588Device::new(Bt0Board { .. }, "yy3588", "rk3588", &keep);
fastboot::serve(&mut udc, &mut dev, &fastboot::Config { unlocked: true }, buf)?;
match dev.next {
    Next::Boot { plan, addr, len } => stage::enter(&mut Bt0Machine(io), fit, &plan, total, &layout),
    Next::Continue => boot::load_and_enter(io, total),   // the normal eMMC boot
}
```

- `Otg0` ungates the controller and PHY clocks, disables the SuperSpeed
  port in USB_GRF (so the USBDP combo PHY is not needed), takes the USB2
  PHY out of GRF suspend and reports VBUS valid through the GRF (the port
  is managed by an FUSB302, as in Linux). It changes no resets.
  `crates/rk3588-fastboot/src/platform.rs` lists each write with its TRM
  page.
- `Bt0Board` is the eMMC hook (emmc.rs + crate sdhci, PIO writes) and the
  reset (CRU_GLB_SRST_FST = 0xfdb9).
- `stage::enter` copies the FIT images, writes the BL31 FDT and bl_params
  (crate rk3588 `handoff`) and jumps to BL31, the same steps as boot.rs
  after its eMMC read.
- The controller is quiesced (Run/Stop off, events masked, no transfer
  left) before `serve` returns, so nothing DMAs into BL31 or EDK2.

## Memory

| What | Where | Size |
|---|---|---|
| bt0 (stepping-stone link) | 0x0 | 192 KiB incl. stack; the DWC3 area (event buffer, TRBs, ep0 buffers) is 4 KiB of its .bss |
| dram-info record | 0x1F0000 | 64 KiB |
| BL31 FDT, bl_params | 0x3000000, 0x3100000 | 2 MiB window |
| **download buffer** | **0x10000000** | **512 MiB** (`DOWNLOAD_MAX`, the K1's), less when DRAM ends below 0x30000000; reported as `max-download-size` |

A booted FIT may not place anything over bt0, the record, the handoff
window or the download buffer (`stage::prepare` refuses with FAIL before
the OKAY).

Every image's data must lie inside the download, with one exception: the
UEFI variable store window, edk2-rk3588's `nvdata` image
(`data-position = <0x6c0000>`, `data-size = <0x30000>`, load 0x7c0000;
it has no hash). A FIT cut from a built image ends before that window
(the second hardware run: 0x52C200 bytes, `FAILFIT data past the
download`); the eMMC path reads 7 MiB and always has it. On the fastboot
path (`boot`, `stage` + `continue`) the missing part of the window is now
filled with 0xFF, the erased-flash state, and bt0 logs
`fastboot: nvdata past download, filled 0x30000 bytes with 0xff`.
edk2-rk3588's RkFvbDxe then finds no firmware volume header there, erases
the window (to 0xFF) and formats a fresh variable store
(`stage::NV_FILL` cites the source lines). Only `nvdata` without a hash
node is filled; a hashed image, or any other image past the download, is
still refused, and the eMMC path stays strict.

## Commands

| Host | Device |
|---|---|
| `fastboot getvar version` / `product` / `serialno` / `max-download-size` / `partition-size:<t>` / `partition-type:<t>` | answered |
| `fastboot boot <fit.itb>` | the FIT (sent wrapped in an Android boot image by the tool, unwrapped here) is checked (structure, SHA-256s, placement), OKAY, USB off, BL31 + EDK2 |
| `fastboot stage <fit.itb>` then `fastboot continue` | the same, raw |
| `fastboot flash <target> <file>` | raw or Android sparse to the eMMC; the tool splits images over 512 MiB into sparse pieces |
| `fastboot continue` (nothing staged) | USB off, the normal eMMC boot (FIT at 0x100000) |
| `fastboot reboot` | SoC reset (the tool may report a status read failure: the reset comes before the OKAY) |

Targets: `emmc` = `user` = `mmc0` = `disk` (user area from LBA 0),
`boot0`, `boot1` (the K1 bt0's set without `sd`), `idblock` (user area
0x8000, up to 0x100000) and `fit` (user area 0x100000, up to 0x800000,
the end of the UEFI variable window).

## When hardware is available

On the host (rkdeveloptool and fastboot installed; the board in MaskROM:
hold the MaskROM key, plug the USB-C OTG port, release):

```sh
make yy3588-loader     # DEV ONLY (rkbin DDR init): out/yy3588/YY3588_rkbinddr_draminfo_loader.bin
                       # (472 = bt0 with stepping-stone,usb-loader; name from the in-progress Makefile)
rkdeveloptool ld       # expect: Maskrom 2207:350b
rkdeveloptool db out/yy3588/YY3588_rkbinddr_draminfo_loader.bin
# UART2 (1500000 8N1): dram-info, "fastboot: BootROM boot source 0x..",
# "[usb] dwc3 GSNPSID 0x5533300a (DWC_usb3 3.00a)", "[usb] high speed", "[usb] configured"
lsusb -d 18d1:4ee0
fastboot devices
fastboot getvar product            # yy3588
fastboot getvar max-download-size  # 0x20000000
# The edk2-rk3588 FIT, e.g. cut from a built image (FIT at 1 MiB, 7 MiB).
# The built image file ends before the UEFI variable window (0x6c0000 +
# 0x30000 into the FIT), so the cut is shorter than 7 MiB; bt0 from this
# change on fills the window. With an older bt0, pad the cut first:
dd if=out/yy3588/YY3588_rkbinddr_DEBUG.img of=/tmp/yy3588-fit.itb bs=1M skip=1 count=7
truncate -s 7M /tmp/yy3588-fit.itb   # only needed for a bt0 before the nvdata fill
fastboot boot /tmp/yy3588-fit.itb                     # RAM boot: BL31 -> EDK2
# or: fastboot stage /tmp/yy3588-fit.itb && fastboot continue
fastboot flash idblock out/yy3588/idblock-rkbinddr.bin
fastboot flash fit /tmp/yy3588-fit.itb
fastboot flash emmc out/yy3588/YY3588_rkbinddr_DEBUG.img
fastboot continue                                     # boot from eMMC
```

Building only the 472 image by hand:

```sh
cd boards/rk3588/yy3588/bt0
env -u RUSTUP_TOOLCHAIN -u RUSTFLAGS cargo +1.99.0 build --release --locked --features usb-loader
aarch64-linux-gnu-objcopy -O binary target/aarch64-unknown-none-softfloat/release/yy3588-bt0 yy3588-bt0-usb.bin
```
