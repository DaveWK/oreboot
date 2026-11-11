// SPDX-License-Identifier: GPL-2.0-only
//! Runtime identification of the fitted LPDDR4/4X parts: MR5 (vendor),
//! MR6/MR7 (revision), MR8 (type, density, I/O width) per rank, decoded
//! with jedec-lpddr's MR8 tables, and the per-channel geometry the DDRCTL
//! address map, the DDR scheduler and the OS_REG handoff need.
//! Identification data only (licence guard).
//!
//! Cross-check: [`rk3568_rows`] is the MR8-to-row rule of the RK3568 U-Boot
//! RFC (Pavel Golikov, GPL-2.0+, drivers/ram/rockchip/sdram_rk3568.c,
//! dram_detect_cap(): `cs0_row = 14 + (mr8 + 1) / 2`, +1 for x8 dies,
//! `row_3_4 = mr8 odd`), ported; host tests prove it agrees with
//! [`geometry`] for every valid MR8.

use super::mr;
use jedec_lpddr::lp4::density;
use regio::RegIo;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct Ids {
    pub ranks: u8,
    pub mr5: u8,
    pub mr6: u8,
    pub mr7: u8,
    pub mr8: u8,
    /// Rank 1's MR8 as read (0xff when not probed or timed out).
    pub mr8_rank1: u8,
    /// MRRDATA0 of rank 0's MR8 (log: DQ-swizzle evidence).
    pub mr8_raw: u32,
}

/// One channel's geometry (16-bit channel).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct ChannelGeometry {
    /// Density per die and channel, Gb (sets the tRFC class).
    pub density_gb: u8,
    /// Byte-mode (x8) dies: two dies per 16-bit channel.
    pub byte_mode: bool,
    pub row: u32,
    /// 3/4-populated top row (3, 6, 12, 24 Gb).
    pub row_3_4: bool,
    pub col: u32,
    pub bank_bits: u32,
    pub rank_mib: u64,
}

impl ChannelGeometry {
    pub fn channel_mib(&self, ranks: u8) -> u64 {
        self.rank_mib * ranks.max(1) as u64
    }
    /// The largest LPDDR4 geometry (32 Gb channel, 18 rows): used for the
    /// probe pass, where only MRR runs.
    pub const PROBE: ChannelGeometry = ChannelGeometry {
        density_gb: 16,
        byte_mode: false,
        row: 17,
        row_3_4: false,
        col: 10,
        bank_bits: 3,
        rank_mib: 2048,
    };
}

/// A valid LPDDR4/4X MR8: type code 0 (S16) and a known density/width.
pub fn mr8_valid(mr8: u8) -> bool {
    density::channel_mib(mr8).is_ok()
}

/// MRR MR5/6/7/8 on rank 0 and, if asked, MR8 on rank 1.
pub fn read_ids<H: RegIo + ?Sized>(io: &mut H, ch: usize, max_ranks: u8) -> Ids {
    let rd = |io: &mut H, rank, ma| mr::read(io, ch, rank, ma).unwrap_or(0xff);
    let mr8_raw = mr::read_raw(io, ch, 0, 8).unwrap_or(u32::MAX);
    let mut ids = Ids {
        ranks: 1,
        mr5: rd(io, 0, 5),
        mr6: rd(io, 0, 6),
        mr7: rd(io, 0, 7),
        mr8: mr8_raw as u8,
        mr8_rank1: 0xff,
        mr8_raw,
    };
    if max_ranks >= 2 {
        ids.mr8_rank1 = rd(io, 1, 8);
    }
    ids
}

/// Whether rank 0 answered like an LPDDR4 part: a valid MR8 and a
/// non-reserved vendor ID. An undriven bus can read MR8 = 0x00, which is a
/// valid MR8 (2 Gb x16), so MR5 = 0x00 (JEDEC: reserved) rejects it; MR5 =
/// 0xff cannot be rejected (Micron).
pub fn present(ids: &Ids) -> bool {
    mr8_valid(ids.mr8) && ids.mr5 != 0x00
}

/// Rank count from the MRR evidence and, when run, the rank-1 gate probe
/// (RK3568 RFC method). Rank 1 is counted when its MR8 equals rank 0's
/// valid MR8 and the gate probe (if any) passed.
pub fn ranks(ids: &Ids, max_ranks: u8, gate_probe: Option<bool>) -> u8 {
    let mr8_match = max_ranks >= 2 && ids.mr8_rank1 == ids.mr8 && mr8_valid(ids.mr8);
    match (mr8_match, gate_probe) {
        (true, None | Some(true)) => 2,
        _ => 1,
    }
}

/// Geometry from MR8. `channel_mib` (jedec-lpddr) is the size of one
/// 16-bit channel, doubled for byte-mode (x8) dies, which pair up per
/// channel. Rows by channel density (JEDEC LPDDR4 addressing, 10 columns,
/// 8 banks): 2 Gb 14, 3 Gb 15 (3/4), 4 Gb 15, 6 Gb 16 (3/4), 8 Gb 16,
/// 12 Gb 17 (3/4), 16 Gb 17, 24 Gb 18 (3/4), 32 Gb 18. The refresh class
/// (tRFC) follows the per-die channel density, half of it in byte mode.
pub fn geometry(mr8: u8) -> Option<ChannelGeometry> {
    let cap = density::Mr8Capture::from_raw(mr8);
    let mib = density::channel_mib(mr8).ok()? as u64;
    let byte_mode = cap.width_code() == 1;
    let gb = (mib * 8 / 1024) as u32;
    let (row, row_3_4) = match gb {
        2 => (14, false),
        3 => (15, true),
        4 => (15, false),
        6 => (16, true),
        8 => (16, false),
        12 => (17, true),
        16 => (17, false),
        24 => (18, true),
        32 => (18, false),
        _ => return None,
    };
    let die_gb = if byte_mode { gb / 2 } else { gb };
    Some(ChannelGeometry {
        density_gb: die_gb.clamp(3, 16) as u8,
        byte_mode,
        row,
        row_3_4,
        col: 10,
        bank_bits: 3,
        rank_mib: mib,
    })
}

/// The RK3568 RFC rule: (row bits, 3/4 row) from MR8 (density OP[5:2],
/// width OP[7:6]: 0 = x16, 1 = x8).
pub const fn rk3568_rows(mr8: u8) -> (u32, bool) {
    let d = ((mr8 >> 2) & 0xf) as u32;
    let x8 = (mr8 >> 6) & 3 != 0;
    let row = 14 + d.div_ceil(2) + x8 as u32; // the RFC's (mr8 + 1) / 2
    (row, d % 2 == 1)
}

/// JEDEC manufacturer ID (MR5) names for the vendors seen on RK3588 boards.
pub fn vendor_name(mr5: u8) -> &'static str {
    match mr5 {
        0x01 => "Samsung",
        0x05 => "Nanya",
        0x06 => "SK hynix",
        0x08 => "Winbond",
        0x13 => "CXMT",
        0x1b => "XMC",
        0xf8 => "Fidelix",
        0xff => "Micron",
        _ => "unknown",
    }
}
