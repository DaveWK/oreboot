// SPDX-License-Identifier: GPL-2.0-only
//! LPDDR5 command bus (CA) encoding of the mode-register commands: each
//! command is one CK cycle with a 7-bit CA value on the rising and one on the
//! falling edge.
//!
//! Truth table: JEDEC LPDDR5/5X, as transcribed in LiteDRAM
//! 74522f715d7b10163ef6f6caa09887a90874be83 `litedram/phy/lpddr5/commands.py`
//! (CA0..CA6 left to right): MRW-1 `L L L H H L H | MA0-6`, MRW-2
//! `L L L H L L OP7 | OP0-6`, MRR `L L L H H L L | MA0-6`. The MRW values are
//! the ones the Radxa Cubie A7A bt0 drives (ore-edk-boot-radxa-a7a
//! `phy/acsm.rs` `MrWritePayload`, `phy/coldinit.rs` `mrw`/`mrw_ddr`), which
//! this module replaces; equivalence tests compare them.
//!
//! Only the CA values: how a controller or PHY sequencer packs them (chip
//! selects, record halfwords, delays) is the board's.

/// Rising-edge CA of MRW-1 (CA3, CA4, CA6 high).
pub const MRW1: u8 = 0x58;
/// Rising-edge CA of MRW-2 without OP7 (CA3 high); OP7 is CA6.
pub const MRW2: u8 = 0x08;
/// Rising-edge CA of MRR (CA3, CA4 high).
pub const MRR: u8 = 0x18;

/// The two edges of one command cycle.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct CaEdges {
    /// CA[6:0] on the rising CK edge.
    pub rise: u8,
    /// CA[6:0] on the falling CK edge.
    pub fall: u8,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CmdError {
    /// Mode-register address above 127.
    Address,
}

/// MRW of operand `op` to mode register `ma`: MRW-1 then MRW-2.
pub const fn mrw(ma: u8, op: u8) -> Result<[CaEdges; 2], CmdError> {
    if ma > 127 {
        return Err(CmdError::Address);
    }
    Ok([
        CaEdges {
            rise: MRW1,
            fall: ma,
        },
        CaEdges {
            rise: MRW2 | ((op >> 7) << 6),
            fall: op & 0x7f,
        },
    ])
}

/// MRR of mode register `ma`.
pub const fn mrr(ma: u8) -> Result<CaEdges, CmdError> {
    if ma > 127 {
        return Err(CmdError::Address);
    }
    Ok(CaEdges {
        rise: MRR,
        fall: ma,
    })
}

// ---------------------------------------------------------------------------
// P1-40: MPC, FIFO, RDC and CAS. Each encoding: JEDEC LPDDR5 value.

/// Rising-edge CA of MPC without OP7 (CA4, CA5 high); OP7 is CA6, OP[6:0]
/// are on the falling edge.
pub const MPC: u8 = 0x30;
/// Write FIFO: rising 60h, falling 00h.
pub const WFF: CaEdges = CaEdges {
    rise: 0x60,
    fall: 0x00,
};
/// Read FIFO: rising 20h, falling 00h.
pub const RFF: CaEdges = CaEdges {
    rise: 0x20,
    fall: 0x00,
};
/// Read DQ calibration: rising 50h, falling 00h.
pub const RDC: CaEdges = CaEdges {
    rise: 0x50,
    fall: 0x00,
};

/// MPC opcodes; the others are reserved.
pub mod mpc {
    pub const WCK2DQI_START: u8 = 0x81;
    pub const WCK2DQI_STOP: u8 = 0x82;
    pub const WCK2DQO_START: u8 = 0x83;
    pub const WCK2DQO_STOP: u8 = 0x84;
    pub const ZQCAL_START: u8 = 0x85;
    pub const ZQCAL_LATCH: u8 = 0x86;
    pub const ALL: [u8; 6] = [
        WCK2DQI_START,
        WCK2DQI_STOP,
        WCK2DQO_START,
        WCK2DQO_STOP,
        ZQCAL_START,
        ZQCAL_LATCH,
    ];
}

/// MPC with opcode `op` (one cycle).
pub const fn mpc(op: u8) -> CaEdges {
    CaEdges {
        rise: MPC | ((op >> 7) << 6),
        fall: op & 0x7f,
    }
}

/// CAS: rising edge 0Ch | WS_WR (CA4) | WS_RD (CA5) | WS_FS (CA6); falling
/// edge DC0-DC3 (CA0-CA3), WRX (CA4), WXSA (CA5), WXSB (CA6).
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct Cas {
    pub ws_wr: bool,
    pub ws_rd: bool,
    pub ws_fs: bool,
    /// DC0-DC3.
    pub dc: u8,
    pub wrx: bool,
    pub wxsa: bool,
    pub wxsb: bool,
}

/// Rising-edge CA of CAS without the WS bits.
pub const CAS: u8 = 0x0c;

impl Cas {
    pub const fn encode(self) -> Option<CaEdges> {
        if self.dc > 15 {
            return None;
        }
        Some(CaEdges {
            rise: CAS | (self.ws_wr as u8) << 4 | (self.ws_rd as u8) << 5 | (self.ws_fs as u8) << 6,
            fall: self.dc | (self.wrx as u8) << 4 | (self.wxsa as u8) << 5 | (self.wxsb as u8) << 6,
        })
    }
    /// None unless the rising edge is a CAS.
    pub const fn decode(e: CaEdges) -> Option<Cas> {
        if e.rise & 0x0f != CAS || e.rise & 0x80 != 0 || e.fall & 0x80 != 0 {
            return None;
        }
        Some(Cas {
            ws_wr: e.rise & 0x10 != 0,
            ws_rd: e.rise & 0x20 != 0,
            ws_fs: e.rise & 0x40 != 0,
            dc: e.fall & 0x0f,
            wrx: e.fall & 0x10 != 0,
            wxsa: e.fall & 0x20 != 0,
            wxsb: e.fall & 0x40 != 0,
        })
    }
}
