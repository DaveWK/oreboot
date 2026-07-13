// SPDX-License-Identifier: GPL-2.0-only
// Verbatim copy of the board code lp5::mr::Mr16 replaces, the oracle of the
// equivalence test. Source: ore-edk-boot-radxa-a7a ed42550, oreboot board
// series applied (tree a803f4f780d7),
// src/mainboard/radxa/cubie-a7a/dram/src/phy/coldinit.rs (mr16).
#![allow(dead_code, clippy::all)]

/// LPDDR5 MR16: OP[1:0] FSP-WR, OP[3:2] FSP-OP, OP[5:4] CBT mode, OP[6]
/// VRCG, OP[7] CBT phase (set per block in [`block_entry`]).
pub const fn mr16(fsp_wr: u8, fsp_op: u8, cbt: u8, vrcg: bool) -> u8 {
    (fsp_wr & 3) | (fsp_op & 3) << 2 | (cbt & 3) << 4 | (vrcg as u8) << 6
}
