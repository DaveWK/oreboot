// SPDX-License-Identifier: GPL-2.0-only
//! Named fields of the ACSM instruction memory of the DWC LPDDR5/4 PHY
//! (CSR 0x41000 + 4 * row + halfword), as an encoder/decoder the stages can
//! switch to once each field is PUBLIC or MEASURED.
//!
//! Provenance per field (P = public, I = inferred from public data, M =
//! to measure):
//! - Addressing, P: a row is four halfwords at 0x41000 + 4 r, rows 0..1023
//!   (ACSMStartAddr/StopAddr/LastAddr are 10 bits; Start bit 0 ignored, the
//!   AcsmPtr translation stores address [9:1] with stop bit 0 forced 1:
//!   instructions are row PAIRS, an even row and the next odd row). ESWIN
//!   EIC7700X TRM Part 2 (sha256 955c99f0...) ACSMStartAddr/StopAddr,
//!   AcsmStart/StopAddrXlatValX, ACSMNopAddr ("ACSM nop instruction pair");
//!   NXP imx-oei 9aea335 (BSD-3) LPDDR5 timing data: its translation table
//!   brackets exactly its non-empty row ranges.
//! - hw0, P/I: CS[1:0] in [15:14] and CA[13:0] in [13:0] (ACSMDefaultCs 2
//!   bits, ACSMDefaultAddr and ACSMAddressMask "CA[13:0]" 14 bits). The
//!   split CA[6:0] = first (rising) edge, CA[13:7] = second (falling) edge
//!   is I: it decodes every NXP row with the JEDEC LPDDR5 command table
//!   (LiteDRAM transcription, `jedec_lpddr::lp5::cmd`) into MRW/MRR/CAS/
//!   RDC/WFF/RFF with sensible MA/OP.
//! - hw1, I/M: four 4-bit WCK pulse groups and the data-path strobes. The
//!   TRM lists per-pulse timing registers in this order: RxEn 0x2002c,
//!   RxVal 0x2002d, TxEn 0x2002e (Wrcs/Rdcs follow TxEn/RxEn), then
//!   WckWrite{StaticLo,StaticHi,Toggle,FastToggle} 0x20035-38,
//!   WckRead{...} 0x20039-3c, WckFreqSw{...} 0x2003d-40. hw1 bits 0, 1, 2
//!   and nibbles [7:4], [11:8], [15:12] follow that order; the nibble that
//!   is set matches the CAS WS bit of the row's command in every NXP and
//!   reference row (WS_WR -> [7:4], WS_RD -> [11:8], WS_FS -> [15:12]) and its
//!   value 0xd (StaticLo + Toggle + FastToggle) is the JEDEC WCK preamble.
//!   Which of bits 0/1 is RxEn and which RxVal: M.
//! - hw2 bit 0, I/M: WCKTG ("setting WCKTG causes a toggle" of hwt_wck_en,
//!   ACSMWckEnControl = 1): set exactly on the rows that start and stop WCK.
//! - hw3, I/M: TG (timing group, "as selected by the TG bits") in bit 4 on
//!   rank-1 rows; REPEATCNT (8 bits, ACSMRptCntOverride is 8 bits) split
//!   over the pair: [2:0] in the even row's [14:12], [7:3] in the odd row's
//!   [14:10]. The split is I from NXP's images at 3733-6400 Mb/s (the count
//!   grows with the data rate only when read this way) and our DCM waits
//!   (N x 2 DfiClk = tDCMM 2 us exactly at pstates 0-2). The unit (1 or 2
//!   DfiClk per count, N or N+1 executions) and the even-row flags [11:7]
//!   (RCNE0, loop start/end, CKE?) are M.
//!
//! Every row of the reference-derived fixtures (8666 rows) and of NXP's nine
//! LPDDR5 images (5490 rows) has no bit outside these fields.

use super::coldinit::Row;
use jedec_lpddr::lp5::cmd::CaEdges;

/// ACSM instruction memory base (CSR address of row 0, halfword 0).
pub const MEM: u32 = 0x41000;
/// Rows the 10-bit ACSM addresses reach.
pub const ROWS: u16 = 1024;

/// CSR of halfword `hw` (0..3) of row `row`.
pub const fn row_csr(row: u16, hw: u32) -> u32 {
    MEM + 4 * row as u32 + hw
}

/// hw0: chip selects and the two CA edges.
pub mod hw0 {
    pub const CS_SHIFT: u32 = 14;
    pub const CA2_SHIFT: u32 = 7;
    pub const CA_MASK: u16 = 0x7f;
}

/// hw1: WCK pulse groups (4 bits each) and data-path strobes.
pub mod hw1 {
    pub const RX_A: u16 = 1 << 0;
    pub const RX_B: u16 = 1 << 1;
    /// Both read strobes (RxEn and RxVal; order to measure).
    pub const RX: u16 = RX_A | RX_B;
    pub const TX_EN: u16 = 1 << 2;
    pub const WRITE_SHIFT: u32 = 4;
    pub const READ_SHIFT: u32 = 8;
    pub const FREQ_SW_SHIFT: u32 = 12;
    /// Bit 3: never set in any known row.
    pub const UNUSED: u16 = 1 << 3;
}

/// Bits of one WCK pulse group (TRM register order).
pub mod wck {
    pub const STATIC_LO: u8 = 1 << 0;
    pub const STATIC_HI: u8 = 1 << 1;
    pub const TOGGLE: u8 = 1 << 2;
    pub const FAST_TOGGLE: u8 = 1 << 3;
    /// The JEDEC WCK2CK sync preamble: static, toggle, fast toggle.
    pub const PREAMBLE: u8 = STATIC_LO | TOGGLE | FAST_TOGGLE;
}

/// hw2.
pub mod hw2 {
    pub const WCKTG: u16 = 1 << 0;
}

/// hw3 (even and odd rows of a pair).
pub mod hw3 {
    pub const TG: u16 = 1 << 4;
    /// Even row: REPEATCNT[2:0].
    pub const CNT_LO_SHIFT: u32 = 12;
    pub const CNT_LO_MASK: u16 = 0x7 << CNT_LO_SHIFT;
    /// Odd row: REPEATCNT[7:3].
    pub const CNT_HI_SHIFT: u32 = 10;
    pub const CNT_HI_MASK: u16 = 0x1f << CNT_HI_SHIFT;
    /// Even-row flags, meaning to measure (names: bit numbers).
    pub const F7: u16 = 1 << 7;
    pub const F8: u16 = 1 << 8;
    pub const F9: u16 = 1 << 9;
    pub const F10: u16 = 1 << 10;
    pub const F11: u16 = 1 << 11;
    pub const FLAGS: u16 = F7 | F8 | F9 | F10 | F11;
    /// Flags of every wait pair in the reference and in NXP's images.
    pub const WAIT: u16 = F11 | F9 | F8;
    /// Even-row bits in use.
    pub const EVEN_MASK: u16 = CNT_LO_MASK | FLAGS | TG;
    /// Odd-row bits in use.
    pub const ODD_MASK: u16 = CNT_HI_MASK | TG;
}

/// Bits of each halfword that no field covers, for an even (`odd` false)
/// or odd row. All zero for every known row.
pub const fn residual(r: Row, odd: bool) -> [u16; 4] {
    let h3 = if odd { hw3::ODD_MASK } else { hw3::EVEN_MASK };
    [0, r.0[1] & hw1::UNUSED, r.0[2] & !hw2::WCKTG, r.0[3] & !h3]
}

/// hw0 of chip selects `cs` with CA `first` on the rising and `second` on
/// the falling edge.
pub const fn ca(cs: u8, first: u8, second: u8) -> u16 {
    ((cs as u16 & 3) << hw0::CS_SHIFT)
        | (((second & 0x7f) as u16) << hw0::CA2_SHIFT)
        | (first & 0x7f) as u16
}

/// hw1 of WCK pulse groups and strobes.
pub const fn pulses(freq_sw: u8, read: u8, write: u8, strobes: u16) -> u16 {
    ((freq_sw as u16 & 0xf) << hw1::FREQ_SW_SHIFT)
        | ((read as u16 & 0xf) << hw1::READ_SHIFT)
        | ((write as u16 & 0xf) << hw1::WRITE_SHIFT)
        | (strobes & 7)
}

/// The REPEATCNT of a pair (even row, odd row).
pub const fn repeat_count(even: Row, odd: Row) -> u8 {
    (((odd.0[3] & hw3::CNT_HI_MASK) >> hw3::CNT_HI_SHIFT) << 3) as u8
        | ((even.0[3] & hw3::CNT_LO_MASK) >> hw3::CNT_LO_SHIFT) as u8
}

/// hw3 halves of REPEATCNT `n`: (even-row part, odd-row part).
pub const fn count_halves(n: u8) -> (u16, u16) {
    (
        ((n & 7) as u16) << hw3::CNT_LO_SHIFT,
        ((n >> 3) as u16) << hw3::CNT_HI_SHIFT,
    )
}

/// A wait pair: no command, REPEATCNT `n`, even-row `flags` ([`hw3::WAIT`]
/// in every known wait).
pub const fn wait_pair(n: u8, flags: u16) -> [Row; 2] {
    let (lo, hi) = count_halves(n);
    [Row([0, 0, 0, flags | lo]), Row([0, 0, 0, hi])]
}

/// Total of REPEATCNT values a wait of `units` count units needs, as
/// whole pairs of at most 255 (the reference's split: 600 = 255 + 255 + 90).
pub const fn wait_split(units: u32, out: &mut [u8; 8]) -> usize {
    let mut left = units;
    let mut n = 0;
    while left > 0 && n < out.len() {
        let k = if left > 255 { 255 } else { left };
        out[n] = k as u8;
        left -= k;
        n += 1;
    }
    n
}

/// A command edge row of the low-speed form (ACSMLowSpeedClockEnable
/// 0x103: ACSM clock DfiClk/6, AC bump clock DfiClk/12, so a row is half
/// a CK): the edge's CA in both 7-bit fields, CS when `cs` is non-zero, TG
/// with CS bit 1 (the reference's rank-1 rule).
pub const fn low_speed_edge(cs: u8, code: u8) -> Row {
    Row([
        ca(cs, code, code),
        0,
        0,
        if cs & 2 != 0 { hw3::TG } else { 0 },
    ])
}

/// A command in the low-speed form: rising edge (with CS), falling edge.
pub const fn low_speed_cmd(cs: u8, e: CaEdges) -> [Row; 2] {
    [low_speed_edge(cs, e.rise), low_speed_edge(0, e.fall)]
}

/// A command in the full-speed form (ACSMLowSpeedClockEnable 0: a row is
/// one CK, both edges in one row), CS `cs`, hw1 `pulses`, hw3 `ctl`.
pub const fn full_speed_cmd(cs: u8, e: CaEdges, pulses: u16, wcktg: bool, ctl: u16) -> Row {
    Row([
        ca(cs, e.rise, e.fall),
        pulses,
        if wcktg { hw2::WCKTG } else { 0 },
        ctl,
    ])
}

/// Field view of one row (decoder; inverse of [`encode`]).
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct Fields {
    pub cs: u8,
    pub ca1: u8,
    pub ca2: u8,
    pub freq_sw: u8,
    pub read: u8,
    pub write: u8,
    pub strobes: u16,
    pub wcktg: bool,
    pub tg: bool,
    /// This row's part of REPEATCNT (even: [2:0], odd: [7:3]).
    pub cnt_part: u8,
    /// Even-row flags (bits 11:7 of hw3 in place).
    pub flags: u16,
}

pub const fn decode(r: Row, odd: bool) -> Fields {
    let [h0, h1, h2, h3] = r.0;
    Fields {
        cs: (h0 >> hw0::CS_SHIFT) as u8,
        ca1: (h0 & hw0::CA_MASK) as u8,
        ca2: ((h0 >> hw0::CA2_SHIFT) & hw0::CA_MASK) as u8,
        freq_sw: ((h1 >> hw1::FREQ_SW_SHIFT) & 0xf) as u8,
        read: ((h1 >> hw1::READ_SHIFT) & 0xf) as u8,
        write: ((h1 >> hw1::WRITE_SHIFT) & 0xf) as u8,
        strobes: h1 & 7,
        wcktg: h2 & hw2::WCKTG != 0,
        tg: h3 & hw3::TG != 0,
        cnt_part: if odd {
            ((h3 & hw3::CNT_HI_MASK) >> hw3::CNT_HI_SHIFT) as u8
        } else {
            ((h3 & hw3::CNT_LO_MASK) >> hw3::CNT_LO_SHIFT) as u8
        },
        flags: if odd { 0 } else { h3 & hw3::FLAGS },
    }
}

pub const fn encode(f: Fields, odd: bool) -> Row {
    let cnt = if odd {
        ((f.cnt_part & 0x1f) as u16) << hw3::CNT_HI_SHIFT
    } else {
        ((f.cnt_part & 7) as u16) << hw3::CNT_LO_SHIFT
    };
    Row([
        ca(f.cs, f.ca1, f.ca2),
        pulses(f.freq_sw, f.read, f.write, f.strobes),
        if f.wcktg { hw2::WCKTG } else { 0 },
        cnt | (f.flags & hw3::FLAGS) | if f.tg { hw3::TG } else { 0 },
    ])
}

// Identity with the reference-derived constructors and rows (more in the tests).
const _: () = {
    // coldinit::Row::cmd is the low-speed edge.
    assert!(low_speed_edge(3, 0).0[0] == Row::cmd(3, 0).0[0]);
    assert!(low_speed_edge(2, 0x58).0[3] == Row::cmd(2, 0x58).0[3]);
    // Wait pairs of the reference: 0x7b00/0x0400 is REPEATCNT 15, the DCM's
    // 0x7b00/0x7c00 255 and 0x2b00/0x2c00 90.
    assert!(repeat_count(Row::nop(0x7b00), Row::nop(0x0400)) == 15);
    assert!(repeat_count(Row::nop(0x7b00), Row::nop(0x7c00)) == 255);
    assert!(repeat_count(Row::nop(0x2b00), Row::nop(0x2c00)) == 90);
    let w = wait_pair(90, hw3::WAIT);
    assert!(w[0].0[3] == 0x2b00 && w[1].0[3] == 0x2c00);
    // The DCA MRR row: CAS WS_RD with the read preamble; MRR with both
    // read strobes.
    assert!(pulses(0, wck::PREAMBLE, 0, 0) == 0x0d00);
    assert!(pulses(wck::PREAMBLE, 0, 0, 0) == 0xd000);
    assert!(pulses(0, 0, 0, hw1::RX) == 0x0003);
};

#[cfg(test)]
mod tests {
    use super::*;
    use crate::phy::coldinit::{mrw, mrw_ddr};
    use jedec_lpddr::lp5::cmd;

    /// The low-speed MRW of coldinit is two low-speed commands.
    #[test]
    fn low_speed_mrw_identity() {
        for cs in 1..=3u8 {
            for ma in 0..128u8 {
                for op in [0u8, 1, 0x40, 0x7f, 0x80, 0xc8, 0xff] {
                    let [w1, w2] = cmd::mrw(ma, op).unwrap();
                    let a = low_speed_cmd(cs, w1);
                    let b = low_speed_cmd(cs, w2);
                    assert_eq!(mrw(cs, ma, op), [a[0], a[1], b[0], b[1]]);
                }
            }
        }
    }

    /// mrw_ddr is two full-speed commands without pulses.
    #[test]
    fn full_speed_mrw_identity() {
        for cs in 1..=3u8 {
            for ma in 0..128u8 {
                let [w1, w2] = cmd::mrw(ma, 0xa5).unwrap();
                assert_eq!(
                    mrw_ddr(cs, ma, 0xa5),
                    [
                        full_speed_cmd(cs, w1, 0, false, 0),
                        full_speed_cmd(cs, w2, 0, false, 0)
                    ]
                );
            }
        }
    }

    /// The DCA's MRR program and DCM program from named fields.
    #[test]
    fn dca_rows_identity() {
        use crate::phy::rxen::mrr_program;
        use crate::phy::wcklevel::Rank;
        let cas_rd = cmd::Cas {
            ws_rd: true,
            ..cmd::Cas::default()
        }
        .encode()
        .unwrap();
        let p = mrr_program(Rank::R0, 26);
        assert_eq!(
            p[0],
            full_speed_cmd(1, cas_rd, pulses(0, wck::PREAMBLE, 0, 0), false, 0)
        );
        assert_eq!(
            p[1],
            full_speed_cmd(1, cmd::mrr(26).unwrap(), pulses(0, 0, 0, hw1::RX), false, 0)
        );
        assert_eq!([p[2], p[3]], wait_pair(14, hw3::WAIT));
        let (d, n) = crate::dca::dcm_program(Rank::R0, 1);
        let cas_fs = cmd::Cas {
            ws_fs: true,
            ..cmd::Cas::default()
        }
        .encode()
        .unwrap();
        assert_eq!(
            d[0],
            full_speed_cmd(1, cas_fs, pulses(wck::PREAMBLE, 0, 0, 0), true, 0)
        );
        assert_eq!(n, 28);
    }

    /// The DCM waits per pstate are tDCMM (2 us, JEDEC via jedec_lpddr)
    /// in units of 2 DfiClk, split into pairs of at most 255: equal to the
    /// reference's rows at pstates 0, 1 and 2 (DfiClk 600, 300, 200 MHz).
    #[test]
    fn dcm_waits_from_tdcmm() {
        use jedec_lpddr::lp5::timing::T_DCMM_PS;
        for (ps, mhz) in [(0u32, 600u64), (1, 300), (2, 200)] {
            let units = (T_DCMM_PS as u64 * mhz).div_ceil(2 * 1_000_000) as u32;
            let mut split = [0u8; 8];
            let k = wait_split(units, &mut split);
            let mut rows = [0u16; 16];
            for (i, &n) in split[..k].iter().enumerate() {
                let w = wait_pair(n, hw3::WAIT);
                rows[2 * i] = w[0].0[3];
                rows[2 * i + 1] = w[1].0[3];
            }
            assert_eq!(&rows[..2 * k], crate::dca::dcm_waits(ps), "pstate {ps}");
        }
    }

    #[test]
    fn decode_encode_round_trip() {
        for v in [0x7b00u16, 0x5a10, 0x1e90, 0x0410, 0x4f00, 0x0500] {
            let r = Row([0x8d18, 0x0d03, 1, v]);
            assert_eq!(residual(r, false), [0; 4]);
            assert_eq!(encode(decode(r, false), false), r);
        }
        for v in [0x7c00u16, 0x0410, 0x4800] {
            let r = Row([0x4d58, 0xd0f4, 1, v]);
            assert_eq!(residual(r, true), [0; 4]);
            assert_eq!(encode(decode(r, true), true), r);
        }
    }
}
