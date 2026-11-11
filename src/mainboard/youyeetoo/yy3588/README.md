# youyeetoo YY3588 (Rockchip RK3588)

First stage (`bt0`) for the youyeetoo YY3588: BootROM → bt0 → TF-A BL31
(built from source) → EDK2. No U-Boot, no SPL, no OP-TEE.

This directory is its own cargo workspace on a pinned stable toolchain
(`rust-toolchain.toml`, Rust 1.99.0), excluded from oreboot's top-level
workspace:

| Path | What |
|---|---|
| `bt0/` | the first stage: UART2 console, RK806 PMIC report and rail setup, DRAM (rkbin OS_REG handoff today, own LPDDR4X init behind the `own-ddr` feature, WIP), eMMC (SDHCI), FIT load, BL31 + EDK2 handoff; with `usb-loader`, a fastboot flasher on the USB-C OTG port (`bt0/FASTBOOT.md`) |
| `crates/` | the libraries bt0 uses (`rk3588`, `regio`, `sdhci`, `dwc3-udc`, `fastboot`, the DRAM crates, ...) |
| `tools/rk-idblock`, `tools/rk-loader` | host tools: BootROM idblock packer and MaskROM USB loader packer (`rkdeveloptool db`) |

Build it through the board workspace, which pins this branch and the
matching EDK2 (DaveWK/edk2-rk3588 `yy3588`):
<https://github.com/DaveWK/yy3588-workspace>.

Standalone:

    cd bt0
    cargo +1.99.0 build --release --locked                       # SRAM bt0
    cargo +1.99.0 build --release --locked --features stepping-stone  # DRAM-linked
    aarch64-linux-gnu-objcopy -O binary target/aarch64-unknown-none-softfloat/release/yy3588-bt0 bt0.bin

Licence: GPL-2.0-only.
