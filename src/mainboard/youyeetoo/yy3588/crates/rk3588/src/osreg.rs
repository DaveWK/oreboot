// SPDX-License-Identifier: GPL-2.0-only
//! The Rockchip DRAM geometry handoff in PMU1 GRF OS_REG2..5.
//!
//! Layout and decode ported from mainline U-Boot (GPL-2.0+):
//! arch/arm/include/asm/arch-rockchip/sdram.h (SYS_REG_* fields, the
//! sys_reg3 comment block) and arch/arm/mach-rockchip/sdram.c
//! (rockchip_sdram_size, rockchip_sdram_type). On RK3588, OS_REG2/3 describe
//! channels 0/1 and OS_REG4/5 channels 2/3 (U-Boot
//! drivers/ram/rockchip/sdram_rk3588.c adds the two decodes); edk2-rk3588
//! SdramLib.c reads the same words. bt0 writes them after its own init so
//! TF-A, EDK2 and Linux see the size, and reads them to detect DRAM that is
//! already initialised (the stepping-stone image).
//!
//! Licence guard: OS_REG is a documented handoff (geometry only). It may
//! select configurations; it carries no timing or training values.

use core::fmt;

/// DRAM type codes (U-Boot sdram.h enum).
pub const DDR4: u32 = 0;
pub const DDR3: u32 = 3;
pub const LPDDR2: u32 = 5;
pub const LPDDR3: u32 = 6;
pub const LPDDR4: u32 = 7;
pub const LPDDR4X: u32 = 8;
pub const LPDDR5: u32 = 9;
pub const DDR5: u32 = 10;

/// Handoff version bt0 writes (sys_reg3 [31:28]); version 3 carries the
/// extended type bits needed for LPDDR4X (8) and LPDDR5 (9).
pub const VERSION: u32 = 3;

/// One decoded channel of a sys_reg pair.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct Channel {
    pub rank: u32,
    pub cs0_col: u32,
    pub cs1_col: u32,
    /// log2 of the bank count.
    pub bk: u32,
    pub cs0_row: u32,
    pub cs1_row: u32,
    /// log2 of the bus width in bytes (2 = 32 bit, 1 = 16 bit).
    pub bw: u32,
    /// The raw DBW field. Rockchip's encoder writes `2 >> dbw` with
    /// sdram_cap_info.dbw "die buswidth, 2:32bit, 1:16bit, 0:8bit" (U-Boot
    /// sdram_common.h SYS_REG_ENC_DBW), so the field reads 2 for x8
    /// (byte-mode) dies, 1 for x16 dies, 0 for x32. Kept raw, as U-Boot's
    /// sdram.c decode uses it (DDR4 bank groups: field 2 = x8).
    pub dbw: u32,
    pub row_3_4: bool,
    pub size_mib: u64,
}

/// One decoded OS_REG pair (two channels).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct Pair {
    pub version: u32,
    pub dram_type: u32,
    pub channels: u32,
    pub ch: [Channel; 2],
}

impl Pair {
    pub fn size_mib(&self) -> u64 {
        self.ch[..self.channels as usize]
            .iter()
            .map(|c| c.size_mib)
            .sum()
    }
}

/// Plausibility of a pair read from hardware: a known version and type and
/// a size in the range RK3588 boards ship (cold boot leaves zeros).
pub fn plausible(p: &Pair) -> bool {
    (2..=3).contains(&p.version)
        && matches!(p.dram_type, LPDDR4 | LPDDR4X | LPDDR5 | DDR4)
        && (256..=16384).contains(&p.size_mib())
}

pub fn type_name(t: u32) -> &'static str {
    match t {
        DDR4 => "DDR4",
        DDR3 => "DDR3",
        LPDDR2 => "LPDDR2",
        LPDDR3 => "LPDDR3",
        LPDDR4 => "LPDDR4",
        LPDDR4X => "LPDDR4X",
        LPDDR5 => "LPDDR5",
        DDR5 => "DDR5",
        _ => "unknown",
    }
}

/// Decode one pair (sys_reg2, sys_reg3), as rockchip_sdram_size does.
pub fn decode(reg2: u32, reg3: u32) -> Pair {
    let version = (reg3 >> 28) & 0xf;
    let mut dram_type = (reg2 >> 13) & 7;
    if version >= 3 {
        dram_type |= ((reg3 >> 12) & 3) << 3;
    }
    let channels = 1 + ((reg2 >> 12) & 1);
    let mut p = Pair {
        version,
        dram_type,
        channels,
        ch: [Channel::default(); 2],
    };
    for c in 0..channels as usize {
        let s = 16 * c as u32;
        let rank = 1 + ((reg2 >> (11 + s)) & 1);
        let cs0_col = 9 + ((reg2 >> (9 + s)) & 3);
        let mut cs1_col = cs0_col;
        let bk = if dram_type == LPDDR5 {
            3 + ((reg2 >> (8 + s)) & 1)
        } else {
            3 - ((reg2 >> (8 + s)) & 1)
        };
        let row = |lo: u32, ext_shift: u32| -> u32 {
            let code = (((reg3 >> ext_shift) & 1) << 2) + lo;
            if code == 7 {
                12
            } else {
                13 + code
            }
        };
        let (cs0_row, cs1_row) = if version >= 2 {
            cs1_col = 9 + ((reg3 >> (2 * c as u32)) & 3);
            (
                row((reg2 >> (6 + s)) & 3, 5 + 2 * c as u32),
                row((reg2 >> (4 + s)) & 3, 4 + 2 * c as u32),
            )
        } else {
            (13 + ((reg2 >> (6 + s)) & 3), 13 + ((reg2 >> (4 + s)) & 3))
        };
        let bw = 2 >> ((reg2 >> (2 + s)) & 3);
        let dbw = (reg2 >> s) & 3;
        let row_3_4 = (reg2 >> (30 + c as u32)) & 1 == 1;
        let bg = if dram_type == DDR4 {
            if dbw == 2 {
                2
            } else {
                1
            }
        } else {
            0
        };
        let mut size: u64 = 1u64 << (cs0_row + cs0_col + bk + bg + bw - 20);
        if rank > 1 {
            size += size >> ((cs0_row - cs1_row) + (cs0_col - cs1_col));
        }
        if row_3_4 {
            size = size * 3 / 4;
        }
        p.ch[c] = Channel {
            rank,
            cs0_col,
            cs1_col,
            bk,
            cs0_row,
            cs1_row,
            bw,
            dbw,
            row_3_4,
            size_mib: size,
        };
    }
    p
}

/// Geometry of one channel bt0 initialised, to be encoded.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Geometry {
    pub rank: u32,
    pub col: u32,
    /// log2 bank count (3 = 8 banks for LPDDR4).
    pub bk: u32,
    pub cs0_row: u32,
    pub cs1_row: u32,
    /// log2 bus width in bytes: 1 = x16 channel (RK3588 channels are 16 bit).
    pub bw: u32,
    /// The raw DBW field (see [`Channel::dbw`]): [`DBW_X8`] for byte-mode
    /// dies, [`DBW_X16`] for x16 dies.
    pub dbw: u32,
}

/// DBW field values (`2 >> dbw`, U-Boot sdram_common.h SYS_REG_ENC_DBW):
/// x8 dies, as the rkbin DDR init wrote for the YY3588's byte-mode parts
/// (OS_REG2 0x3a561a56, field 2 per channel; banner "Die BW=8").
pub const DBW_X8: u32 = 2;
pub const DBW_X16: u32 = 1;

/// Encode a pair (inverse of [`decode`] for row codes 13..=18, version 3),
/// field by field as U-Boot drivers/ram/rockchip/sdram_common.c
/// sdram_org_config_v3 does, including the chinfo bit of each described
/// channel (sys_reg2 [29:28], SYS_REG_ENC_CHINFO_V3), which the decode
/// does not need.
pub fn encode(dram_type: u32, ch: &[Geometry]) -> (u32, u32) {
    let mut r2 = ((dram_type & 7) << 13) | (((ch.len() as u32).saturating_sub(1) & 1) << 12);
    let mut r3 = (VERSION << 28) | (((dram_type >> 3) & 3) << 12);
    for (c, g) in ch.iter().enumerate().take(2) {
        let s = 16 * c as u32;
        let code = |row: u32| row.saturating_sub(13);
        let c0 = code(g.cs0_row);
        let c1 = code(g.cs1_row);
        let bwf = match g.bw {
            2 => 0,
            1 => 1,
            _ => 2,
        };
        r2 |= 1 << (28 + c as u32)
            | ((g.rank - 1) & 1) << (11 + s)
            | ((g.col - 9) & 3) << (9 + s)
            | ((3 - g.bk) & 1) << (8 + s)
            | (c0 & 3) << (6 + s)
            | (c1 & 3) << (4 + s)
            | bwf << (2 + s)
            | (g.dbw & 3) << s;
        r3 |= ((c0 >> 2) & 1) << (5 + 2 * c as u32)
            | ((c1 >> 2) & 1) << (4 + 2 * c as u32)
            | ((g.col - 9) & 3) << (2 * c as u32);
    }
    (r2, r3)
}

impl fmt::Display for Pair {
    /// One parseable line per channel: `key=value` pairs.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "osreg version={} type={} channels={} size_mib={}",
            self.version,
            type_name(self.dram_type),
            self.channels,
            self.size_mib()
        )?;
        for (i, c) in self.ch[..self.channels as usize].iter().enumerate() {
            write!(
                f,
                "\nosreg ch={} rank={} col={}/{} bank_bits={} row={}/{} bw_bytes_log2={} dbw={} row_3_4={} size_mib={}",
                i, c.rank, c.cs0_col, c.cs1_col, c.bk, c.cs0_row, c.cs1_row, c.bw, c.dbw, c.row_3_4 as u8, c.size_mib
            )?;
        }
        Ok(())
    }
}
