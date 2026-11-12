# Radxa Cubie A7A (Allwinner A733)

Work-in-progress oreboot port for the Radxa Cubie A7A.

| Crate | Target | What it does |
|---|---|---|
| `bt0` | `armv7a-none-eabi` | eGON image the BROM loads to SRAM at 0x47000: clocks, AXP318W PMIC rails, UART0, SD card, LPDDR5 init, loads `main` and the payload |
| `main` | `aarch64-unknown-none-softfloat` | EL3 runtime at the bottom of DRAM: PSCI/SMCCC, GICv3, devicetree for the payload; optional GMAC, USB (DWC3), PCIe and SMCCC TRNG bring-up |
| `common` | | code shared by `bt0` and `main` (UART, TWI, boot layout, firmware log) |
| `dram` | | LPDDR5 controller and PHY init without the vendor library |

The shared crates under `src/lib` (`regio`, `dram-train`, `jedec-lpddr`,
`dwc-ddrctl`, `dwc-pcie`, `psci-el3`, ...) are used as path dependencies.

## DRAM paths

No vendor binary is part of this tree. Depending on the `bt0` features, the
build reads one from outside:

- default (open driver): the DRAM PHY's own training firmware and PIE images
  are extracted as data from Allwinner's Tina SDK DRAM library object
  (`https://gitlab.com/tina5.0_aiot/lichee/dramlib.git`, commit
  `35e84c835b26f8d933e50299869657365e996363`,
  `sun60iw2p1/spl_libdram/libdram`), named by `A733_PHY_FW_OBJ`; `build.rs`
  checks each image's SHA-256.
- `dramlib`: DRAM init through that closed library, linked from an archive
  named by `A733_DRAM_LIB` (`lib<name>.a`). Wrap the object with
  `ar rcsD libdram-a733.a libdram` and weaken the functions bt0 provides
  itself: `dwc_ddrphy_phyinit_G_execFW`, `mctl_com_set_controller_after_phy`,
  `lp5_sw_mr_wr`, `lp5_sw_mr_rd`, `lpddr4_phyinit_D_loadIMEM`,
  `lpddr4_phyinit_F_loadDMEM`, `lpddr4_phyinit_I_loadPIEImage`
  (`objcopy --weaken-symbol=...`).
- `blobless`: no PHY firmware at all; a development stage that probes and
  trains parts of the PHY and halts.

The other `bt0` features are measurement builds; see `bt0/Cargo.toml`.

## Building

```sh
cd src/mainboard/radxa/cubie-a7a/bt0
A733_DRAM_LIB=/path/to/libdram-a733.a cargo build --release --features dramlib

cd ../main
cargo build --release --no-default-features --features "gmac usb trng trng-byte-addr"
```

Then `rust-objcopy -O binary` each ELF. Packing the eGON header, the boot
layout table and the payload (EDK2) into a card image, and the USB FEL boot
script, are in the workspace that builds the whole chain:
<https://github.com/DaveWK/radxa-a7a-workspace> (`make firmware`,
`make fel-boot`).

## USB FEL boot (feature `fel`)

`bt0` built with `--features fel` (plus a DRAM backend, e.g. `dramlib`) is
linked at 0x90000 and started over USB by the A733 mask ROM's FEL mode
instead of from the SD card (`bt0/src/fel.rs`):

- entry 1 (0x90040): clocks, PMIC, DRAM and CPU boost, then back to FEL;
  the result is in the mailbox at 0xdf000;
- the host writes the boot layout table frame (0xdf200), oreboot `main`
  and the EDK2 FD into DRAM, each write read back and compared;
- entry 2 (0x90044): checks the table and the CRCs, turns off the GIC
  state the mask ROM left (its USB runs on interrupts) and hands over to
  `main` as the SD boot does.

The host side is `scripts/fel-boot.sh` in the workspace; it needs `xfel`.

A fastboot device in bt0 (flashing the microSD over USB from FEL) is
planned, not implemented.
