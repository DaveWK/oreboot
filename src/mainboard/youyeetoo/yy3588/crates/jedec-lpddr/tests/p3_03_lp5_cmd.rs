// SPDX-License-Identifier: GPL-2.0-only
//! The LPDDR5 command set, encode and decode, against the cited
//! truth table and each other.

use jedec_lpddr::lp5::cmd::{
    self, decode, decode_trace, mpc, rise, CaEdges, Cas, Command, DecodeError, EncodeError,
};

fn rt(c: Command) -> Command {
    let (edges, n) = c.encode().unwrap();
    assert_eq!(n, c.cycles());
    let second = (n == 2).then_some(edges[1]);
    let (d, used) = decode(edges[0], second).unwrap();
    assert_eq!(used, n, "{c:?}");
    d
}

#[test]
fn every_command_round_trips() {
    for bank in 0..16u8 {
        for row in [0u32, 1, 0x7f, 0x80, 0x7ff, 0x800, 0x3fff, 0x4000, 0x3ffff] {
            let c = Command::Activate { bank, row };
            assert_eq!(rt(c), c);
        }
        for all in [false, true] {
            let c = Command::Precharge { bank, all };
            assert_eq!(rt(c), c);
        }
        for col in 0..64u8 {
            for ap in [false, true] {
                for c in [
                    Command::MaskWrite { bank, col, ap },
                    Command::Write16 { bank, col, ap },
                    Command::Read16 { bank, col, ap },
                    Command::Read32 { bank, col, ap },
                ] {
                    assert_eq!(rt(c), c);
                }
                let w32 = Command::Write32 { bank, col, ap };
                if col % 2 == 0 {
                    assert_eq!(rt(w32), w32);
                } else {
                    assert_eq!(w32.encode(), Err(EncodeError::Column));
                }
            }
        }
    }
    for bank in 0..8u8 {
        for bits in 0..8u8 {
            let c = Command::Refresh {
                bank,
                rfm: bits & 1 != 0,
                sb0: bits & 2 != 0,
                all: bits & 4 != 0,
            };
            assert_eq!(rt(c), c);
        }
    }
    for ma in 0..128u8 {
        for op in [0u8, 1, 0x7f, 0x80, 0xa5, 0xff] {
            let c = Command::ModeRegisterWrite { ma, op };
            assert_eq!(rt(c), c);
        }
        let c = Command::ModeRegisterRead { ma };
        assert_eq!(rt(c), c);
    }
    for op in 0..=255u8 {
        assert_eq!(rt(Command::Mpc(op)), Command::Mpc(op));
    }
    for bits in 0..4u8 {
        let c = Command::SelfRefreshEntry {
            dsm: bits & 1 != 0,
            pd: bits & 2 != 0,
        };
        assert_eq!(rt(c), c);
    }
    for c in [
        Command::Nop,
        Command::PowerDownEntry,
        Command::SelfRefreshExit,
        Command::WriteFifo,
        Command::ReadFifo,
        Command::ReadDqCal,
        Command::Cas(Cas {
            ws_wr: true,
            dc: 9,
            wxsb: true,
            ..Cas::default()
        }),
    ] {
        assert_eq!(rt(c), c);
    }
}

#[test]
fn rows_of_the_truth_table() {
    // ACT-1 `H H H R14-17 | BA0-3 R11-13`, ACT-2 `H H L R7-10 | R0-6`.
    let (e, _) = Command::Activate {
        bank: 0b1010,
        row: 0b10_1101_0110_1001_0110,
    }
    .encode()
    .unwrap();
    // row bits: R17..R14 = 1011 (0b10_11), R13..R11 = 010, R10..R7 = 1101, R6..R0 = 0010110.
    assert_eq!(
        e[0],
        CaEdges {
            rise: rise::ACT1 | (0b1011 << 3),
            fall: 0b1010 | (0b010 << 4)
        }
    );
    assert_eq!(
        e[1],
        CaEdges {
            rise: rise::ACT2 | (0b1101 << 3),
            fall: 0b001_0110
        }
    );
    // PRE all banks: 78h | BA, AB on CA6.
    let (e, _) = Command::Precharge { bank: 3, all: true }.encode().unwrap();
    assert_eq!(
        e[0],
        CaEdges {
            rise: 0x78,
            fall: 0x43
        }
    );
    // REF: 38h; RFM CA3, SB0 CA4, AB CA6.
    let (e, _) = Command::Refresh {
        bank: 5,
        rfm: true,
        sb0: false,
        all: false,
    }
    .encode()
    .unwrap();
    assert_eq!(
        e[0],
        CaEdges {
            rise: 0x38,
            fall: 0x0d
        }
    );
    // RD16 `H L L C0 C3-5 | BA0-3 C1-2 AP`: column 0b101101 = C5..C0.
    let (e, _) = Command::Read16 {
        bank: 1,
        col: 0b101101,
        ap: true,
    }
    .encode()
    .unwrap();
    // C0 = 1 -> CA3; C3-5 = 101 -> CA4-6; C1-2 = 10 -> CA4-5 of the fall; AP CA6.
    assert_eq!(
        e[0],
        CaEdges {
            rise: 0x01 | (1 << 3) | (0b101 << 4),
            fall: 1 | (0b10 << 4) | (1 << 6)
        }
    );
    // WR32 `L L H L C3-5`: CA3 stays low.
    let (e, _) = Command::Write32 {
        bank: 0,
        col: 0b111110,
        ap: false,
    }
    .encode()
    .unwrap();
    assert_eq!(e[0].rise & 0x0f, 0x04);
    assert_eq!(e[0].rise >> 4, 0b111);
    // SRE 68h with DSM (CA5) and PD (CA6); SRX 28h; PDE 40h; NOP 00h.
    let (e, _) = Command::SelfRefreshEntry {
        dsm: true,
        pd: true,
    }
    .encode()
    .unwrap();
    assert_eq!(
        e[0],
        CaEdges {
            rise: 0x68,
            fall: 0x60
        }
    );
    assert_eq!(Command::SelfRefreshExit.encode().unwrap().0[0].rise, 0x28);
    assert_eq!(Command::PowerDownEntry.encode().unwrap().0[0].rise, 0x40);
    assert_eq!(Command::Nop.encode().unwrap().0[0], CaEdges::default());
    // The single-command encoders are what Command uses.
    assert_eq!(
        Command::ModeRegisterWrite { ma: 18, op: 0xc3 }
            .encode()
            .unwrap()
            .0,
        cmd::mrw(18, 0xc3).unwrap()
    );
    assert_eq!(
        Command::ModeRegisterRead { ma: 8 }.encode().unwrap().0[0],
        cmd::mrr(8).unwrap()
    );
    assert_eq!(
        Command::Mpc(cmd::mpc::ZQCAL_START).encode().unwrap().0[0],
        mpc(0x85)
    );
    assert_eq!(Command::WriteFifo.encode().unwrap().0[0], cmd::WFF);
}

#[test]
fn encode_errors() {
    assert_eq!(
        Command::Activate { bank: 16, row: 0 }.encode(),
        Err(EncodeError::Bank)
    );
    assert_eq!(
        Command::Activate {
            bank: 0,
            row: 1 << 18
        }
        .encode(),
        Err(EncodeError::Row)
    );
    assert_eq!(
        Command::Refresh {
            bank: 8,
            rfm: false,
            sb0: false,
            all: false
        }
        .encode(),
        Err(EncodeError::Bank)
    );
    assert_eq!(
        Command::Read16 {
            bank: 0,
            col: 64,
            ap: false
        }
        .encode(),
        Err(EncodeError::Column)
    );
    assert_eq!(
        Command::ModeRegisterWrite { ma: 128, op: 0 }.encode(),
        Err(EncodeError::Address)
    );
    assert_eq!(
        Command::ModeRegisterRead { ma: 128 }.encode(),
        Err(EncodeError::Address)
    );
    assert_eq!(
        Command::Cas(Cas {
            dc: 16,
            ..Cas::default()
        })
        .encode(),
        Err(EncodeError::Cas)
    );
}

#[test]
fn decode_errors_and_variable_bits() {
    // ACT-1 and MRW-1 need their second cycle, and the right one.
    let act = Command::Activate { bank: 1, row: 2 }.encode().unwrap().0;
    assert_eq!(decode(act[0], None), Err(DecodeError::NeedSecond));
    assert_eq!(
        decode(
            act[0],
            Some(CaEdges {
                rise: 0x78,
                fall: 0
            })
        ),
        Err(DecodeError::BadSecond)
    );
    assert_eq!(
        decode(act[1], None),
        Err(DecodeError::Unknown),
        "ACT-2 alone"
    );
    let mrw = cmd::mrw(3, 4).unwrap();
    assert_eq!(decode(mrw[0], None), Err(DecodeError::NeedSecond));
    assert_eq!(decode(mrw[0], Some(mrw[0])), Err(DecodeError::BadSecond));
    assert_eq!(
        decode(mrw[1], None),
        Err(DecodeError::Unknown),
        "MRW-2 alone"
    );
    // A bit above CA6 is no LPDDR5 CA.
    assert_eq!(
        decode(
            CaEdges {
                rise: 0x80,
                fall: 0
            },
            None
        ),
        Err(DecodeError::Unknown)
    );
    // Rising edges no command has.
    assert_eq!(
        decode(
            CaEdges {
                rise: 0x48,
                fall: 0
            },
            None
        ),
        Err(DecodeError::Unknown)
    );
    // V bits are ignored on decode: PRE with CA4/CA5 of the fall set.
    assert_eq!(
        decode(
            CaEdges {
                rise: 0x78,
                fall: 0x33
            },
            None
        ),
        Ok((
            Command::Precharge {
                bank: 3,
                all: false
            },
            1
        ))
    );
    // SRX with any falling edge.
    assert_eq!(
        decode(
            CaEdges {
                rise: 0x28,
                fall: 0x7f
            },
            None
        ),
        Ok((Command::SelfRefreshExit, 1))
    );
}

#[test]
fn a_trace_decodes_in_order() {
    let cmds = [
        Command::ModeRegisterWrite { ma: 1, op: 0x55 },
        Command::Activate { bank: 2, row: 1234 },
        Command::Write16 {
            bank: 2,
            col: 8,
            ap: false,
        },
        Command::Cas(Cas {
            ws_wr: true,
            ..Cas::default()
        }),
        Command::Read16 {
            bank: 2,
            col: 8,
            ap: true,
        },
        Command::Refresh {
            bank: 0,
            rfm: false,
            sb0: false,
            all: true,
        },
        Command::Mpc(cmd::mpc::ZQCAL_LATCH),
        Command::SelfRefreshEntry {
            dsm: false,
            pd: true,
        },
    ];
    let mut trace = Vec::new();
    for c in cmds {
        let (e, n) = c.encode().unwrap();
        trace.extend_from_slice(&e[..n]);
    }
    assert_eq!(trace.len(), 10);
    let mut out = [Command::Nop; 16];
    let n = decode_trace(&trace, &mut out).unwrap();
    assert_eq!(&out[..n], &cmds);
    // A damaged cycle stops the decode at its index.
    trace[4].rise = 0x48;
    assert_eq!(
        decode_trace(&trace, &mut out),
        Err((4, DecodeError::Unknown))
    );
    // A truncated two-cycle command at the end.
    let act = Command::Activate { bank: 0, row: 0 }.encode().unwrap().0;
    assert_eq!(
        decode_trace(&act[..1], &mut out),
        Err((0, DecodeError::NeedSecond))
    );
    // Output capacity bounds the decode: MRW (2 cycles) + ACT (2 cycles)
    // fill two slots; the WR16 after them is left in the trace.
    let mut two = [Command::Nop; 2];
    assert_eq!(decode_trace(&trace[..4], &mut two), Ok(2));
    assert_eq!(decode_trace(&trace, &mut two), Ok(2));
    assert_eq!(two, [cmds[0], cmds[1]]);
}
