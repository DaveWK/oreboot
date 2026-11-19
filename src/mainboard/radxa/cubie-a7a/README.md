# Radxa Cubie A7A (Allwinner A733)

Work-in-progress oreboot port for the Radxa Cubie A7A.

| Crate | Target | What it does |
|---|---|---|
| `bt0` | `armv7a-none-eabi` | eGON image the BROM loads to SRAM at 0x47000 from the SD card or the eMMC module: clocks, AXP318W PMIC rails, UART0, LPDDR5 init, then `main` and the payload from the medium it was loaded from |
| `main` | `aarch64-unknown-none-softfloat` | EL3 runtime at the bottom of DRAM: PSCI/SMCCC, GICv3, devicetree for the payload; optional GMAC, USB (DWC3), PCIe and SMCCC TRNG bring-up |
| `common` | | code shared by `bt0` and `main` (UART, TWI, boot layout, firmware log) |
| `dram` | | LPDDR5 controller and PHY init without the vendor library |
| `fastboot` | | the fastboot protocol, the SD, eMMC, USB disk and SPI NOR access behind it, and their guards (used by `bt0`) |

The shared crates under `src/lib` (`regio`, `dram-train`, `jedec-lpddr`,
`dwc-ddrctl`, `dwc-pcie`, `psci-el3`, ...) are used as path dependencies.

## DRAM paths

No vendor binary is part of this tree, and the default build reads none.

- **default (feature `blobfree`): the blob-free boot.** The `dram` crate
  brings LPDDR5 up by itself: no vendor library, and none of the PHY's
  training firmware or PIE images is built in, loaded or run. Command bus
  training, WCK2CK leveling, read training, the hand-over to the
  controllers and a write stage are this tree's code; when they end clean
  `bt0` goes on to its DRAM test and the boot, otherwise it halts (see Boot
  failures). The settings are built in, one profile per DRAM clock
  (`dram/src/knobs/profile.rs`):

  | `A7A_PROFILE` | DRAM clock | state on the 4 GB board, over USB FEL |
  |---|---|---|
  | `1200` | WCK 1200 MHz, 2400 Mb/s | boots Linux |
  | `2400` | WCK 2400 MHz, 4800 Mb/s | does not boot Linux yet: the bring-up, bt0's DRAM test and memory tests from the host pass and the firmware runs, then the kernel fails to decompress (rare data errors under load) |

  Unset, the profile is the line `pub const DEFAULT` in that file (1200).
  The profiles were made on the CXMT 4 GB board; the same image selects the
  6 GB part's parameters by the strap, and has not run there yet. The
  parameter sets with another DRAM clock (8 and 12 GB) are refused: use
  `dramlib` for those. Every board run so far was a USB FEL boot; the
  profile keeps that boot's DRAM rails on the SD card and the eMMC.

  The profiles are knob lines: each setting is a build-time `A7A_*`
  variable of the lab images (`bt0/Cargo.toml`, `dram/src/knobs.rs`). One
  set in the build's environment overrides the profile's value, `A7A_X=-`
  unsets one, and `A7A_PROFILE=none` builds the same code with no profile.

- `--no-default-features --features dramlib`: DRAM init through Allwinner's
  closed library instead, for comparison, linked from an archive named by
  `A733_DRAM_LIB` (`lib<name>.a`). The library is the Tina SDK DRAM library
  object (`https://gitlab.com/tina5.0_aiot/lichee/dramlib.git`, commit
  `35e84c835b26f8d933e50299869657365e996363`,
  `sun60iw2p1/spl_libdram/libdram`). Wrap the object with
  `ar rcsD libdram-a733.a libdram` and weaken the functions bt0 provides
  itself: `dwc_ddrphy_phyinit_G_execFW`, `mctl_com_set_controller_after_phy`,
  `lp5_sw_mr_wr`, `lp5_sw_mr_rd`, `lpddr4_phyinit_D_loadIMEM`,
  `lpddr4_phyinit_F_loadDMEM`, `lpddr4_phyinit_I_loadPIEImage`
  (`objcopy --weaken-symbol=...`).
- `--no-default-features` alone: the `dram` crate with the PHY's own
  training firmware and PIE images loaded as data. They are extracted from
  the same library object, named by `A733_PHY_FW_OBJ`; `build.rs` checks
  each image's SHA-256. No vendor code is linked.
- `--no-default-features --features blobless` or `bl-mission`: the lab
  images of the blob-free flow, with every setting from the build's
  environment and none built in (`blobless` halts after the read training,
  `bl-mission` after the write stage unless `A7A_BL_BOOT` is set).

Another backend or a lab feature together with the default features is a
compile error; with `--no-default-features` every feature set builds the
image it built before the default existed. The other `bt0` features are
measurement builds; see `bt0/Cargo.toml`.

## Building

```sh
cd src/mainboard/radxa/cubie-a7a/bt0
cargo build --release                   # SD card and eMMC, blob-free DRAM
cargo build --release --features fel    # the same, loaded over USB FEL
A7A_PROFILE=2400 cargo build --release  # at the higher DRAM clock (no Linux boot yet)
# with the vendor DRAM library instead:
A733_DRAM_LIB=/path/to/libdram-a733.a cargo build --release --no-default-features --features dramlib

cd ../main
cargo build --release --no-default-features --features "gmac usb trng trng-byte-addr"
```

Then `rust-objcopy -O binary` each ELF. Packing the eGON header, the boot
layout table and the payload (EDK2) into a card image, and the USB FEL boot
script, are in the workspace that builds the whole chain:
<https://github.com/DaveWK/radxa-a7a-workspace> (`make firmware`,
`make fel-boot`).

## Boot failures

`bt0` arms the watchdog and marks each boot attempt in an RTC register;
EDK2 confirms the boot from its boot manager. A boot that fails or hangs
before that (no boot medium, a read or CRC error, a hang in a later stage)
ends in a watchdog reset. The next `bt0` sees the marker before it sets up
the bus clocks, the PMIC or DRAM: loaded from the SD card, it prints one
line and enters the mask ROM's FEL mode, so the board answers on USB
(`xfel`) instead of resetting in a loop; the reset after that tries the
boot again. Loaded from the eMMC or the SPI NOR, it reports the failed boot
and tries again.

FEL is also reached with the board's UBOOT key, and from a running system
by writing 0x5AA5A55A to RTC GP_DATA_REG2 before a reset
(`bt0/src/felreboot.rs`).

## USB FEL boot (feature `fel`)

`bt0` built with `--features fel` (with any DRAM backend) is linked at
0x90000 and started over USB by the A733 mask ROM's FEL mode instead of
from the SD card or the eMMC (`bt0/src/fel.rs`):

- entry 1 (0x90040): clocks, PMIC, DRAM and CPU boost, then back to FEL;
  the result is in the mailbox at 0xdf000;
- the host writes the boot layout table frame (0xdf200), oreboot `main`
  and the EDK2 FD into DRAM, each write read back and compared;
- entry 2 (0x90044): checks the table and the CRCs, turns off the GIC
  state the mask ROM left (its USB runs on interrupts) and hands over to
  `main` as the SD boot does.

The host side is `scripts/fel-boot.sh` in the workspace; it needs `xfel`.

With feature `fastboot` as well, entry 4 (0x9004c) brings DRAM up and
serves the stock `fastboot` tool on the same USB port instead of going
back to FEL (`bt0/src/fastboot.rs`): `getvar`, `download`, `boot` of a
layout bundle, `reboot`, `reboot-bootloader` (back to FEL) and read-only
`fetch` of the SD card, the eMMC, a USB disk and the SPI NOR. Writing is
separate: `fastboot-emmc-write` and `fastboot-nor-write` add flash and
erase behind arming and content checks (`fastboot/src`).
