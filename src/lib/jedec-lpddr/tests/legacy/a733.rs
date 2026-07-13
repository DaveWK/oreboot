// SPDX-License-Identifier: GPL-2.0-only
// Verbatim copies of the board code this crate replaces, kept as the test
// oracle for the equivalence tests. Source: ore-edk-boot-radxa-a7a oreboot
// 2d79de7 (= ore-edk2-boot patches/oreboot/a733-cubie-a7a up to "a7a: one-boot
// TxWckDly alias probe ...", tree 6507a1ab86cf with the K1 series), path
// src/mainboard/radxa/cubie-a7a/dram/src/<file>. Only the stubs marked
// "stub:" are not from the source.
#![allow(dead_code, clippy::all)]

// ---- timing.rs ------------------------------------------------------------
pub const fn ns(t_ns: u32, mhz: u32) -> u32 {
    (t_ns * mhz).div_ceil(1000)
}

pub mod lp5 {
    pub const T_FAW: u32 = 20;
    pub const T_RRD: u32 = 5;
    pub const T_RCD: u32 = 18;
    pub const T_XP: u32 = 7;
    pub const T_WTR_L: u32 = 12;
    pub const T_RTP: u32 = 8; // 7.5 ns
    pub const T_RPAB: u32 = 21;
    pub const T_RAS: u32 = 42;
    pub const T_REFI: u32 = 3906;
    pub const T_RFCAB: u32 = 380;
    /// Power-up: 200 us of RESET/CKE timing for the DFI init counters.
    pub const T_INIT_200US: u32 = 200_000;
    pub const T_ZQCAL: u32 = 260;
    pub const T_XSR_EXTRA: u32 = 90;
    pub const T_CKE: u32 = 3;
    pub const T_ESCKE: u32 = 10;
    pub const T_CCDMW_NS: u32 = 15;
    pub const T_CMD_1NS: u32 = 1;
    pub const T_MRW_40: u32 = 40;
    pub const T_ZQRESET_6US: u32 = 6000;
    pub const T_ZQLAT_30: u32 = 30;
    pub const T_ZQCS_50: u32 = 50;
}

// ---- size.rs (Para accessors replaced by parameters) -----------------------
// stub: the Para and Error of the board crate.
pub struct Para {
    pub row: u32,
    pub bank: u32,
    pub col: u32,
    pub bg: u32,
    pub rank: u32,
    pub para2: u32,
    pub channels: u32,
    pub tpr13: u32,
}
impl Para {
    pub fn row_bits(&self) -> u32 {
        self.row
    }
    pub fn bank_bits(&self) -> u32 {
        self.bank
    }
    pub fn col_bits(&self) -> u32 {
        self.col
    }
    pub fn bg_bits(&self) -> u32 {
        self.bg
    }
    pub fn rank_bits(&self) -> u32 {
        self.rank
    }
    pub fn para2(&self) -> u32 {
        self.para2
    }
    pub fn channels(&self) -> u32 {
        self.channels
    }
    pub fn tpr13(&self) -> u32 {
        self.tpr13
    }
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    Config(&'static str),
    Timeout(&'static str),
}
pub fn checked_size_mib(p: &Para) -> Result<u32, Error> {
    let bits = p.row_bits() + p.bank_bits() + p.col_bits() + p.bg_bits() + p.rank_bits();
    let half_width = p.para2() & 0xf != 0;
    let overhead = if !half_width || p.channels() == 2 {
        18
    } else {
        19
    };
    let shift = bits
        .checked_sub(overhead)
        .ok_or(Error::Config("DRAM geometry below size representation"))?;
    let mut mib = 1u32
        .checked_shl(shift)
        .ok_or(Error::Config("DRAM geometry exceeds size arithmetic"))?;
    if p.tpr13() & (7 << 16) != 0 && p.para2() >> 30 != 2 {
        mib = mib
            .checked_mul(3)
            .ok_or(Error::Config("non-binary DRAM size arithmetic overflow"))?
            >> 2;
    }
    validate_mib(mib)
}

fn validate_mib(mib: u32) -> Result<u32, Error> {
    // para2's handoff size is 15 bits; the bounded smoke check needs at least
    // two MiB. These are software bounds, not detected physical capacity.
    if !(2..=0x7fff).contains(&mib) {
        return Err(Error::Config("DRAM size outside boot handoff field"));
    }
    Ok(mib)
}
