// SPDX-License-Identifier: GPL-2.0-only
//! LPDDR4/4X command bus (CA) encoding of the mode-register commands.
//!
//! An LPDDR4 command is two SDR clock cycles of CA[5:0], CS high in the
//! first and low in the second. MRW is MRW-1 then MRW-2; MRR is MRR-1 then
//! CAS-2. Truth table: JEDEC LPDDR4/4X as transcribed in LiteDRAM
//! 74522f715d7b10163ef6f6caa09887a90874be83 `litedram/phy/lpddr4/commands.py`
//! lines 164-166 and 173 (`TRUTH_TABLE`, CA0..CA5 left to right), BSD-2-Clause
//! (orchestrator `references/datasheets/open-source/litedram-litedram_phy_
//! lpddr4_commands.py`, sha256 cf2a2724...):
//!
//! - MRW-1 `L H H L L OP7 | MA0-5`
//! - MRW-2 `L H H L H OP6 | OP0-5`
//! - MRR-1 `L H H H L V | MA0-5`
//! - CAS-2 `L H L L H C8 | C2-7` (column 0 after MRR-1)
//!
//! Only the CA values: how a controller sequences or packs them is the
//! board's.

/// First-cycle CA of MRW-1 without OP7 (CA1, CA2 high); OP7 is CA5.
pub const MRW1: u8 = 0x06;
/// First-cycle CA of MRW-2 without OP6 (CA1, CA2, CA4 high); OP6 is CA5.
pub const MRW2: u8 = 0x16;
/// First-cycle CA of MRR-1 (CA1, CA2, CA3 high; V low).
pub const MRR1: u8 = 0x0e;
/// First-cycle CA of CAS-2 without C8 (CA1, CA4 high); C8 is CA5.
pub const CAS2: u8 = 0x12;

/// One command: CA[5:0] in the CS-high cycle and in the following cycle.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct CaCycles {
    pub first: u8,
    pub second: u8,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CmdError {
    /// Mode-register address above 63.
    Address,
}

/// MRW of operand `op` to mode register `ma`: MRW-1, MRW-2.
pub const fn mrw(ma: u8, op: u8) -> Result<[CaCycles; 2], CmdError> {
    if ma > 63 {
        return Err(CmdError::Address);
    }
    Ok([
        CaCycles {
            first: MRW1 | ((op >> 7) & 1) << 5,
            second: ma,
        },
        CaCycles {
            first: MRW2 | ((op >> 6) & 1) << 5,
            second: op & 0x3f,
        },
    ])
}

/// MRR of mode register `ma`: MRR-1, then CAS-2 with column 0.
pub const fn mrr(ma: u8) -> Result<[CaCycles; 2], CmdError> {
    if ma > 63 {
        return Err(CmdError::Address);
    }
    Ok([
        CaCycles {
            first: MRR1,
            second: ma,
        },
        CaCycles {
            first: CAS2,
            second: 0,
        },
    ])
}

/// MPC (multi-purpose command) opcodes. RD-FIFO, RD-DQ-CAL and WR-FIFO must
/// be followed by CAS-2. JEDEC LPDDR4/4X value.
pub mod mpc {
    pub const RD_FIFO: u8 = 0x41;
    pub const RD_DQ_CAL: u8 = 0x43;
    pub const WR_FIFO: u8 = 0x47;
    pub const DQS_OSC_START: u8 = 0x4b;
    pub const DQS_OSC_STOP: u8 = 0x4d;
    pub const ZQCAL_START: u8 = 0x4f;
    pub const ZQCAL_LATCH: u8 = 0x51;
    /// The opcodes above.
    pub const ALL: [u8; 7] = [
        RD_FIFO,
        RD_DQ_CAL,
        WR_FIFO,
        DQS_OSC_START,
        DQS_OSC_STOP,
        ZQCAL_START,
        ZQCAL_LATCH,
    ];
    /// Whether the MPC must be followed by CAS-2.
    pub const fn needs_cas2(op: u8) -> bool {
        matches!(op, RD_FIFO | RD_DQ_CAL | WR_FIFO)
    }
}
