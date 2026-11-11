// SPDX-License-Identifier: GPL-2.0-only
//! LPDDR5 command bus (CA) encoding of the mode-register commands: each
//! command is one CK cycle with a 7-bit CA value on the rising and one on the
//! falling edge.
//!
//! Truth table: JEDEC LPDDR5/5X, as transcribed in LiteDRAM
//! 74522f715d7b10163ef6f6caa09887a90874be83 `litedram/phy/lpddr5/commands.py`
//! (CA0..CA6 left to right): MRW-1 `L L L H H L H | MA0-6`, MRW-2
//! `L L L H L L OP7 | OP0-6`, MRR `L L L H H L L | MA0-6`. The MRW values are
//! the ones the Radxa Cubie A7A bt0 drives (`MrWritePayload`, `mrw`,
//! `mrw_ddr`), which this module replaces; equivalence tests compare them.
//!
//! Only the CA values: how a controller or PHY sequencer packs them (chip
//! selects, record halfwords, delays) is the board's.
//!
//! The whole command set is [`Command`] with `encode` and
//! [`decode`] / [`decode_trace`] (a captured CA trace back to commands),
//! on the same truth table.

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
// MPC, FIFO, RDC and CAS. Each encoding: JEDEC LPDDR5 value.

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

// ---------------------------------------------------------------------------
// The whole command set, encoders and decoders.
//
// Truth table (JEDEC LPDDR5/5X as transcribed in LiteDRAM 74522f715d
// `litedram/phy/lpddr5/commands.py` `TRUTH_TABLE`, BSD-2-Clause; CA0..CA6
// left to right, rising | falling edge):
//
//   NOP    L L L L L L L           | X             PDE    L L L L L L H | X
//   ACT-1  H H H R14-17            | BA0-3 R11-13  ACT-2  H H L R7-10   | R0-6
//   PRE    L L L H H H H           | BA0-3 V V AB
//   REF    L L L H H H L           | BA0-2 RFM SB0 V AB
//   MWR    L H L C0 C3-5           | BA0-3 C1-2 AP
//   WR16   L H H C0 C3-5           | BA0-3 C1-2 AP
//   WR32   L L H L C3-5            | BA0-3 C1-2 AP
//   RD16   H L L C0 C3-5           | BA0-3 C1-2 AP
//   RD32   H L H C0 C3-5           | BA0-3 C1-2 AP
//   CAS    L L H H WS_WR WS_RD WS_FS | DC0-3 WRX WXSA WXSB
//   MPC    L L L L H H OP7         | OP0-6
//   SRE    L L L H L H H           | V V V V V DSM PD
//   SRX    L L L H L H L           | V
//   MRW-1  L L L H H L H           | MA0-6         MRW-2  L L L H L L OP7 | OP0-6
//   MRR    L L L H H L L           | MA0-6
//   WFF    L L L L L H H           | L x 7         RFF    L L L L L H L | L x 7
//   RDC    L L L L H L H           | L x 7
//
// The 16-bank organisation (BA0-3) is the one the table gives; bank-group
// modes are not encoded here. DESELECT is CS low, not a CA pattern, so it
// is outside these edges.

/// One LPDDR5 command, decoded from or encoded to its CA edges.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Command {
    Nop,
    PowerDownEntry,
    /// ACT-1 + ACT-2: bank BA0-3, row R0-R17.
    Activate {
        bank: u8,
        row: u32,
    },
    /// PRE: one bank, or all (AB).
    Precharge {
        bank: u8,
        all: bool,
    },
    /// REF: bank BA0-2 (per-bank), RFM, SB0, all banks (AB).
    Refresh {
        bank: u8,
        rfm: bool,
        sb0: bool,
        all: bool,
    },
    /// MWR: column C0-C5, auto-precharge.
    MaskWrite {
        bank: u8,
        col: u8,
        ap: bool,
    },
    Write16 {
        bank: u8,
        col: u8,
        ap: bool,
    },
    /// WR32: C0 is not carried (even columns only).
    Write32 {
        bank: u8,
        col: u8,
        ap: bool,
    },
    Read16 {
        bank: u8,
        col: u8,
        ap: bool,
    },
    Read32 {
        bank: u8,
        col: u8,
        ap: bool,
    },
    Cas(Cas),
    Mpc(u8),
    SelfRefreshEntry {
        dsm: bool,
        pd: bool,
    },
    SelfRefreshExit,
    /// MRW-1 + MRW-2.
    ModeRegisterWrite {
        ma: u8,
        op: u8,
    },
    ModeRegisterRead {
        ma: u8,
    },
    WriteFifo,
    ReadFifo,
    ReadDqCal,
}

/// Why a command could not be encoded.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum EncodeError {
    /// Bank above 15 (REF: above 7).
    Bank,
    /// Row above R17.
    Row,
    /// Column above 63, or odd for WR32.
    Column,
    /// Mode-register address above 127.
    Address,
    /// CAS DC above 15.
    Cas,
}

/// Why CA edges did not decode.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DecodeError {
    /// No command has this rising edge (or the second cycle of a two-cycle
    /// command came first).
    Unknown,
    /// A two-cycle command (ACT, MRW) needs its second cycle.
    NeedSecond,
    /// The second cycle is not the one the first calls for.
    BadSecond,
}

/// Rising-edge CA constants of the one-cycle and first-cycle commands
/// (fixed bits only; the variable bits are 0).
pub mod rise {
    pub const NOP: u8 = 0x00;
    pub const PDE: u8 = 0x40;
    /// ACT-1 without R14-17 (CA0-2 high).
    pub const ACT1: u8 = 0x07;
    /// ACT-2 without R7-10 (CA0-1 high).
    pub const ACT2: u8 = 0x03;
    pub const PRE: u8 = 0x78;
    pub const REF: u8 = 0x38;
    /// MWR without C0/C3-5 (CA1 high).
    pub const MWR: u8 = 0x02;
    /// WR16 without C0/C3-5 (CA1-2 high).
    pub const WR16: u8 = 0x06;
    /// WR32 without C3-5 (CA2 high, CA3 low).
    pub const WR32: u8 = 0x04;
    /// RD16 without C0/C3-5 (CA0 high).
    pub const RD16: u8 = 0x01;
    /// RD32 without C0/C3-5 (CA0, CA2 high).
    pub const RD32: u8 = 0x05;
    pub const SRE: u8 = 0x68;
    pub const SRX: u8 = 0x28;
}

const fn one(e: CaEdges) -> Result<([CaEdges; 2], usize), EncodeError> {
    Ok(([e, CaEdges { rise: 0, fall: 0 }], 1))
}

const fn bit(v: u8, i: u8) -> bool {
    v >> i & 1 != 0
}

/// Column C0-C5 and bank onto the WR/RD/MWR edges: rising `base | C0<<3 |
/// C3-5<<4`, falling `BA0-3 | C1-2<<4 | AP<<6`.
const fn data_cmd(base: u8, bank: u8, col: u8, ap: bool, c0: bool) -> Result<CaEdges, EncodeError> {
    if bank > 15 {
        return Err(EncodeError::Bank);
    }
    if col > 63 || (!c0 && col & 1 != 0) {
        return Err(EncodeError::Column);
    }
    Ok(CaEdges {
        rise: base | ((col & 1) << 3) | (((col >> 3) & 7) << 4),
        fall: bank | (((col >> 1) & 3) << 4) | ((ap as u8) << 6),
    })
}

const fn data_fields(e: CaEdges) -> (u8, u8, bool) {
    let c0 = (e.rise >> 3) & 1;
    let c12 = (e.fall >> 4) & 3;
    let c35 = (e.rise >> 4) & 7;
    (e.fall & 0x0f, c0 | (c12 << 1) | (c35 << 3), bit(e.fall, 6))
}

impl Command {
    /// Cycles on the bus: 2 for ACT and MRW, 1 otherwise.
    pub const fn cycles(self) -> usize {
        match self {
            Command::Activate { .. } | Command::ModeRegisterWrite { .. } => 2,
            _ => 1,
        }
    }

    /// The CA edges (the second entry is used for two-cycle commands only)
    /// and how many cycles. Variable (`V`, `X`) bits are driven low.
    pub const fn encode(self) -> Result<([CaEdges; 2], usize), EncodeError> {
        match self {
            Command::Nop => one(CaEdges {
                rise: rise::NOP,
                fall: 0,
            }),
            Command::PowerDownEntry => one(CaEdges {
                rise: rise::PDE,
                fall: 0,
            }),
            Command::Activate { bank, row } => {
                if bank > 15 {
                    return Err(EncodeError::Bank);
                }
                if row >= 1 << 18 {
                    return Err(EncodeError::Row);
                }
                let r = row;
                Ok((
                    [
                        CaEdges {
                            rise: rise::ACT1 | ((((r >> 14) & 0xf) as u8) << 3),
                            fall: bank | ((((r >> 11) & 0x7) as u8) << 4),
                        },
                        CaEdges {
                            rise: rise::ACT2 | ((((r >> 7) & 0xf) as u8) << 3),
                            fall: (r & 0x7f) as u8,
                        },
                    ],
                    2,
                ))
            }
            Command::Precharge { bank, all } => {
                if bank > 15 {
                    return Err(EncodeError::Bank);
                }
                one(CaEdges {
                    rise: rise::PRE,
                    fall: bank | ((all as u8) << 6),
                })
            }
            Command::Refresh {
                bank,
                rfm,
                sb0,
                all,
            } => {
                if bank > 7 {
                    return Err(EncodeError::Bank);
                }
                one(CaEdges {
                    rise: rise::REF,
                    fall: bank | ((rfm as u8) << 3) | ((sb0 as u8) << 4) | ((all as u8) << 6),
                })
            }
            Command::MaskWrite { bank, col, ap } => {
                match data_cmd(rise::MWR, bank, col, ap, true) {
                    Ok(e) => one(e),
                    Err(x) => Err(x),
                }
            }
            Command::Write16 { bank, col, ap } => match data_cmd(rise::WR16, bank, col, ap, true) {
                Ok(e) => one(e),
                Err(x) => Err(x),
            },
            Command::Write32 { bank, col, ap } => {
                match data_cmd(rise::WR32, bank, col, ap, false) {
                    Ok(e) => one(e),
                    Err(x) => Err(x),
                }
            }
            Command::Read16 { bank, col, ap } => match data_cmd(rise::RD16, bank, col, ap, true) {
                Ok(e) => one(e),
                Err(x) => Err(x),
            },
            Command::Read32 { bank, col, ap } => match data_cmd(rise::RD32, bank, col, ap, true) {
                Ok(e) => one(e),
                Err(x) => Err(x),
            },
            Command::Cas(c) => match c.encode() {
                Some(e) => one(e),
                None => Err(EncodeError::Cas),
            },
            Command::Mpc(op) => one(mpc(op)),
            Command::SelfRefreshEntry { dsm, pd } => one(CaEdges {
                rise: rise::SRE,
                fall: ((dsm as u8) << 5) | ((pd as u8) << 6),
            }),
            Command::SelfRefreshExit => one(CaEdges {
                rise: rise::SRX,
                fall: 0,
            }),
            Command::ModeRegisterWrite { ma, op } => match mrw(ma, op) {
                Ok(e) => Ok((e, 2)),
                Err(_) => Err(EncodeError::Address),
            },
            Command::ModeRegisterRead { ma } => match mrr(ma) {
                Ok(e) => one(e),
                Err(_) => Err(EncodeError::Address),
            },
            Command::WriteFifo => one(WFF),
            Command::ReadFifo => one(RFF),
            Command::ReadDqCal => one(RDC),
        }
    }
}

/// Decode a captured command: `first` is the CS-asserted cycle, `second`
/// the next one when the first opens a two-cycle command (ACT-1, MRW-1).
/// Returns the command and the cycles consumed (1 or 2). Variable bits are
/// ignored; CA7+ (bit 7) must be clear on every edge.
pub const fn decode(
    first: CaEdges,
    second: Option<CaEdges>,
) -> Result<(Command, usize), DecodeError> {
    if first.rise & 0x80 != 0 || first.fall & 0x80 != 0 {
        return Err(DecodeError::Unknown);
    }
    let r = first.rise;
    let f = first.fall;
    // Rising edge, by its fixed bits.
    if r & 0x07 == rise::ACT1 {
        let Some(s) = second else {
            return Err(DecodeError::NeedSecond);
        };
        if s.rise & 0x07 != rise::ACT2 || s.rise & 0x80 != 0 || s.fall & 0x80 != 0 {
            return Err(DecodeError::BadSecond);
        }
        let row = (((r >> 3) & 0xf) as u32) << 14
            | (((f >> 4) & 0x7) as u32) << 11
            | (((s.rise >> 3) & 0xf) as u32) << 7
            | (s.fall & 0x7f) as u32;
        return Ok((
            Command::Activate {
                bank: f & 0x0f,
                row,
            },
            2,
        ));
    }
    if r & 0x07 == rise::ACT2 {
        // ACT-2 on its own: the first cycle is missing.
        return Err(DecodeError::Unknown);
    }
    if r & 0x07 == rise::RD16
        || r & 0x07 == rise::RD32
        || r & 0x07 == rise::MWR
        || r & 0x07 == rise::WR16
    {
        let (bank, col, ap) = data_fields(first);
        let c = match r & 0x07 {
            rise::RD16 => Command::Read16 { bank, col, ap },
            rise::RD32 => Command::Read32 { bank, col, ap },
            rise::MWR => Command::MaskWrite { bank, col, ap },
            _ => Command::Write16 { bank, col, ap },
        };
        return Ok((c, 1));
    }
    if r & 0x0f == rise::WR32 {
        let (bank, col, ap) = data_fields(first);
        return Ok((Command::Write32 { bank, col, ap }, 1));
    }
    if r & 0x0f == CAS {
        return match Cas::decode(first) {
            Some(c) => Ok((Command::Cas(c), 1)),
            None => Err(DecodeError::Unknown),
        };
    }
    // The remaining commands have CA0-2 low: CA3-6 select them.
    if r & 0x07 != 0 {
        return Err(DecodeError::Unknown);
    }
    match r & 0x78 {
        rise::NOP => Ok((Command::Nop, 1)),
        rise::PDE => Ok((Command::PowerDownEntry, 1)),
        rise::PRE => Ok((
            Command::Precharge {
                bank: f & 0x0f,
                all: bit(f, 6),
            },
            1,
        )),
        rise::REF => Ok((
            Command::Refresh {
                bank: f & 0x07,
                rfm: bit(f, 3),
                sb0: bit(f, 4),
                all: bit(f, 6),
            },
            1,
        )),
        rise::SRE => Ok((
            Command::SelfRefreshEntry {
                dsm: bit(f, 5),
                pd: bit(f, 6),
            },
            1,
        )),
        rise::SRX => Ok((Command::SelfRefreshExit, 1)),
        MRW1 => {
            let Some(s) = second else {
                return Err(DecodeError::NeedSecond);
            };
            if s.rise & 0x3f != MRW2 || s.rise & 0x80 != 0 || s.fall & 0x80 != 0 {
                return Err(DecodeError::BadSecond);
            }
            Ok((
                Command::ModeRegisterWrite {
                    ma: f & 0x7f,
                    op: ((s.rise >> 6) << 7) | (s.fall & 0x7f),
                },
                2,
            ))
        }
        MRW2 => Err(DecodeError::Unknown),
        MRR => Ok((Command::ModeRegisterRead { ma: f & 0x7f }, 1)),
        // MPC and MPC|OP7 (CA6).
        MPC | 0x70 => Ok((Command::Mpc(((r >> 6) << 7) | (f & 0x7f)), 1)),
        0x60 => Ok((Command::WriteFifo, 1)),
        0x20 => Ok((Command::ReadFifo, 1)),
        0x50 => Ok((Command::ReadDqCal, 1)),
        _ => Err(DecodeError::Unknown),
    }
}

/// Decode a captured CA trace into `out`, returning how many commands were
/// decoded; stops at the first undecodable cycle (its index is returned in
/// the error with the cause).
pub fn decode_trace(trace: &[CaEdges], out: &mut [Command]) -> Result<usize, (usize, DecodeError)> {
    let (mut i, mut n) = (0, 0);
    while i < trace.len() && n < out.len() {
        let second = trace.get(i + 1).copied();
        let (c, used) = decode(trace[i], second).map_err(|e| (i, e))?;
        out[n] = c;
        n += 1;
        i += used;
    }
    Ok(n)
}
