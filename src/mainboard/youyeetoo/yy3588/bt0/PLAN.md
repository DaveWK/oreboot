# YY3588 bt0: stages, sources and status

Rule: every register value bt0 writes comes from a documented source,
recorded per value, and is reviewed before it is used. No vendor-derived or GPL-derived init; rkbin binaries are
never disassembled (also forbidden by the rkbin LICENSE).

Sources used so far: RK3588 TRM Part 1 and Part 2, V1.0 (2022-03-09);
TF-A `plat/rockchip/rk3588` (BSD-3-Clause) for cross-checking base
addresses; the board's vendor and mainline DTS for pin assignments.

| # | Stage | Needs | Source | Status |
|---|---|---|---|---|
| 1 | BootROM handoff | idblock v2 at sector 64; init image in SYSTEM_SRAM | format: U-Boot `tools/rkcommon.c` (public reference), checked byte for byte by `tools/rk-idblock` tests; load address 0xff001000 inferred | built (`yy3588`); the stepping stone uses init = rkbin DDR init, boot = bt0 at DRAM 0x0 (U-Boot `SPL_TEXT_BASE` 0 on RK3588); load addresses to confirm on hardware |
| 2 | Console | UART2 M0 (GPIO0_B5/B6) at 1,500,000 baud | TRM Part 1: Table 1-1; PMU2_IOC/BUS_IOC GPIO0B_IOMUX_SEL_H (pp. 968, 985); CRU_CLKSEL_CON45 (p. 75); UART (p. 1439) | in the skeleton |
| 3 | Clocks | none for bt0 itself (24 MHz); DDR PLLs belong to stage 5 | TRM Part 2 §2.6.1 step 2; TRM Part 1 Chapter 2 (CRU, DDRPHY CRUs) | done (no-op) |
| 4 | PMIC | RK806 over SPI2 M2: DDR rails at operating voltage before DRAM init (vdd_ddr_s0, vdd2_ddr_s3, vddq_ddr_s0, vdd_ddr_pll_s0); RK8602/8603 (CPU big cores) not needed before DRAM | Linux `drivers/mfd/rk8xx-spi.c`, `rk808.h`, `rk808-regulator.c`; Linux `spi-rockchip.c`; TRM Part 1 Chapter 31 (SPI) | first hardware run read CHIP_NAME 0x00: CTRLR0.bht was 0 (two SPI frames per FIFO write); fixed (`crates/rk3588::rk806`), the read-only readback (id, POWER_EN1..3, BUCK5/6/9, NLDO2) awaits the next run; rail writes only in the own-DDR path |
| 5 | DRAM | 4 × DDRCTL (Synopsys DWC DDRCTL register set; crates/dwc-ddrctl, crates/dram-ctrl `dwc`), 4 × DDRPHY with hardware training engine (new backend), LPDDR4/4X MR/timing (crates/jedec-lpddr), sweeps (crates/dram-train) | TRM Part 2 Chapter 2: DDRCTL and DDRPHY register descriptions, §2.6.1 LPDDR4/4X init sequence, §2.6.3–2.6.8 training; DRAM part datasheet | host-tested, not run; parts identified on the first hardware run (LPDDR4X, 4 x 16-bit channels, 2 ranks of x8 byte-mode dies, MR5 0xff Micron, MR8 0x58, 8 GiB per channel, 32 GiB): `Config::yy3588_default` uses them (`ddr::YY3588_PARTS`) and writes the same OS_REG words the rkbin init left |
| 6 | Load | eMMC (DWC MSHC, crates/sdhci), FIT at 0x100000 (crates/fit), BL31 segments + EDK2 FV, `bl_params` for BL31 (no BL32) | TRM Part 2 Chapter 4; TF-A docs | implemented (`src/boot.rs`, `crates/rk3588::fit`; host-tested on a real edk2-rk3588 FIT); runs once DRAM is up (stepping stone) |
| 7 | Handoff | BL31 entry at EL3 with bl_params; BL33 = EDK2 at 0x200000 | TF-A rk3588 platform (`params_setup.c`, `bl31_params_parse_helper`) | implemented (`crates/rk3588::handoff`, host-tested); not run on hardware |

OP-TEE (BL32) is dropped: nothing in the EDK2 path needs it, and the
vendor kernel's `optee` node then fails to probe harmlessly. Building
OP-TEE OS from source is a later, separate option.

Stepping stone (dev only, contains the rkbin DDR init): with DRAM brought
up by rkbin's DDR init, stages 2, 4 (read-only), 6 and 7 can be tested on
hardware before stage 5 is ours (first runs 2026-11-09: stages 1, 2 and the
OS_REG/MRR part of dram-info verified; see the README of
DaveWK/yy3588-workspace). bt0's DRAM step is one interface
(`src/dram.rs`): an OS_REG handoff from rkbin's DDR init, or, with feature
`own-ddr`, `rk3588::ddr::init`. See the README of
DaveWK/yy3588-workspace for the targets, the
dram-info record and its licence guard.
