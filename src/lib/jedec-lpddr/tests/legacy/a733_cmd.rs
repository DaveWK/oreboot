// SPDX-License-Identifier: GPL-2.0-only
// Verbatim copies of the board code lp5::cmd replaces, the oracle of the
// equivalence tests. Source: ore-edk-boot-radxa-a7a ed42550, oreboot board
// series applied (tree a803f4f780d7), src/mainboard/radxa/cubie-a7a/dram/src/
// phy/acsm.rs (whole file) and phy/coldinit.rs (Row, mrw, mrw_ddr).
#![allow(dead_code, clippy::all)]

// ---- phy/acsm.rs ----------------------------------------------------------

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PayloadError {
    ModeAddress,
    ChipSelect,
}

/// The two CA/CS halfwords of a four-edge LPDDR5 MRW.
/// Other halfwords of the ACSM record require a separate reviewed contract.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct MrWritePayload {
    first: u16,
    second: u16,
}

impl MrWritePayload {
    /// Explicit CS mask 1..=3: no rank/channel mapping or broadcast policy is
    /// inferred. The caller must establish that the selected device(s), MR and
    /// operand are valid for the fitted part and current FSP.
    pub const fn new(address: u8, operand: u8, cs_mask: u8) -> Result<Self, PayloadError> {
        if address > 127 {
            return Err(PayloadError::ModeAddress);
        }
        if cs_mask == 0 || cs_mask > 3 {
            return Err(PayloadError::ChipSelect);
        }
        let cs = (cs_mask as u16) << 14;
        Ok(Self {
            first: cs | ((address as u16) << 7) | 0x58,
            second: cs | (((operand & 0x7f) as u16) << 7) | (((operand >> 7) as u16) << 6) | 0x08,
        })
    }

    /// Relative record halfword index plus CA/CS payload, not an APB address.
    /// Deliberately no conversion to a complete record or hardware write.
    pub const fn fields(self) -> [(usize, u16); 2] {
        [(0, self.first), (4, self.second)]
    }
}

// ---- phy/coldinit.rs (Row, mrw, mrw_ddr) ----------------------------------
/// One ACSM memory row.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct Row(pub [u16; 4]);

impl Row {
    pub const EMPTY: Row = Row([0; 4]);

    /// Row with explicit halfwords (CA pattern rows with different phase
    /// values, and the hw1/hw2 control rows of the WCK and CBT sequences).
    pub const fn raw(hw0: u16, hw1: u16, hw2: u16, hw3: u16) -> Row {
        Row([hw0, hw1, hw2, hw3])
    }

    /// Row driving CA `lo` on the first and `hi` on the second phase with
    /// chip selects `cs` (CBT patterns, and commands sent while a rank is in
    /// command bus training). The oracle's rows of this form carry no control
    /// word, also for rank 1.
    pub const fn ca2(cs: u8, lo: u8, hi: u8) -> Row {
        Row([
            ((cs as u16 & 3) << 14) | (((hi & 0x7f) as u16) << 7) | (lo & 0x7f) as u16,
            0,
            0,
            0,
        ])
    }

    /// Delay/timing row: no command, CS idle.
    pub const fn nop(ctl: u16) -> Row {
        Row([0, 0, 0, ctl])
    }

    /// Command edge row: `ca` (7 bits) in both CA fields, chip selects `cs`
    /// (bit 0 rank 0, bit 1 rank 1).
    pub const fn cmd(cs: u8, ca: u8) -> Row {
        let ca = (ca & 0x7f) as u16;
        let ctl = if cs & 2 != 0 { 0x0010 } else { 0 };
        Row([((cs as u16 & 3) << 14) | (ca << 7) | ca, 0, 0, ctl])
    }
}

/// LPDDR5 MRW as four rows: MRW-1 code, MA, MRW-2 code with OP[7], OP[6:0].
pub const fn mrw(cs: u8, ma: u8, op: u8) -> [Row; 4] {
    [
        Row::cmd(cs, 0x58),
        Row::cmd(0, ma),
        Row::cmd(cs, 0x08 | ((op >> 7) << 6)),
        Row::cmd(0, op & 0x7f),
    ]
}

/// LPDDR5 MRW as two rows with both CA edges each (MRW-1 + MA, MRW-2 +
/// OP[6:0]), CS on both: the form the oracle uses for rank 0 while rank 1 is
/// in command bus training.
pub const fn mrw_ddr(cs: u8, ma: u8, op: u8) -> [Row; 2] {
    [
        Row::ca2(cs, 0x58, ma),
        Row::ca2(cs, 0x08 | ((op >> 7) << 6), op & 0x7f),
    ]
}
