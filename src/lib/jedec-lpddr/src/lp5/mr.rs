// SPDX-License-Identifier: GPL-2.0-only
//! LPDDR5 mode-register encoders and decoders.
//!
//! Sources, so each layout can be checked:
//! - MR1, MR2, MR3, MR10, MR11, MR17, MR18, MR20, MR22, MR28: LiteDRAM
//!   74522f715d7b10163ef6f6caa09887a90874be83 `litedram/init.py`
//!   (`get_lpddr5_phy_init_sequence`) and `litedram/phy/lpddr5/commands.py`
//!   (`BankOrganization`). Only the meanings that source states are named;
//!   other bits are carried raw.
//! - MR16: the Radxa Cubie A7A bt0 (ore-edk-boot-radxa-a7a
//!   `phy/coldinit.rs` `mr16`: OP[1:0] FSP-WR, OP[3:2] FSP-OP, OP[5:4] CBT
//!   mode, OP[6] VRCG, OP[7] CBT phase), which this replaces.
//! - MR12/MR14/MR15 Vref codes: [`super::VrefCode`].
//!
//! - P1-40: MR8, MR10, MR11, MR12/14/15, MR13, MR16, MR17, MR18, MR19,
//!   MR20, MR24, MR26, MR30/MR69, MR31-MR34, MR35-MR40, MR41, MR46, MR58,
//!   MR70-MR74 and the MR3 bank-organization codes: each a JEDEC LPDDR5 value
//!   (LPDDR5X where marked). MR8 OP[1:0] = 10b is a vendor extension.

pub use super::DramVref;

/// MR numbers.
pub const MR1: u8 = 1;
pub const MR2: u8 = 2;
pub const MR3: u8 = 3;
pub const MR10: u8 = 10;
pub const MR11: u8 = 11;
pub const MR12: u8 = 12;
pub const MR13: u8 = 13;
pub const MR14: u8 = 14;
pub const MR15: u8 = 15;
pub const MR16: u8 = 16;
pub const MR17: u8 = 17;
pub const MR18: u8 = 18;
pub const MR20: u8 = 20;
pub const MR22: u8 = 22;
pub const MR28: u8 = 28;

/// Termination and drive strength codes (MR3 PDDS, MR11, MR17, MR18 ODT).
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Rzq {
    Disabled = 0,
    Div1 = 1,
    Div2 = 2,
    Div3 = 3,
    Div4 = 4,
    Div5 = 5,
    Div6 = 6,
}

impl Rzq {
    pub const fn from_code(c: u8) -> Option<Rzq> {
        Some(match c {
            0 => Rzq::Disabled,
            1 => Rzq::Div1,
            2 => Rzq::Div2,
            3 => Rzq::Div3,
            4 => Rzq::Div4,
            5 => Rzq::Div5,
            6 => Rzq::Div6,
            _ => return None,
        })
    }
}

/// MR3 OP[4:3]: bank organization: 00b BG, 01b 8B, 10b 16B, 11b RFU.
/// JEDEC LPDDR5/5X value (also LiteDRAM's).
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BankOrg {
    /// 4 banks, 4 bank groups (above 3200 Mbps).
    Bg = 0,
    /// 8 banks, no bank groups (all data rates, BL32 only).
    B8 = 1,
    /// 16 banks, no bank groups (up to 3200 Mbps).
    B16 = 2,
}

impl BankOrg {
    /// None for 11b (RFU).
    pub const fn from_code(c: u8) -> Option<BankOrg> {
        match c {
            0 => Some(BankOrg::Bg),
            1 => Some(BankOrg::B8),
            2 => Some(BankOrg::B16),
            _ => None,
        }
    }
    /// The bank organization of an MR3 operand (OP[4:3]).
    pub const fn from_mr3(op: u8) -> Option<BankOrg> {
        BankOrg::from_code((op >> 3) & 3)
    }
}

/// MR1: OP[3] CK mode (0: differential), OP[7:4] WL code
/// ([`super::timing::FreqRange::mr`]); OP[2:0] raw.
pub const fn mr1(wl_code: u8, single_ended_ck: bool, raw_2_0: u8) -> Option<u8> {
    if wl_code > 15 || raw_2_0 > 7 {
        return None;
    }
    Some(raw_2_0 | (single_ended_ck as u8) << 3 | wl_code << 4)
}

/// MR2: OP[3:0] RL/nRBTP code, OP[7:4] nWR code.
pub const fn mr2(rl_code: u8, nwr_code: u8) -> Option<u8> {
    if rl_code > 15 || nwr_code > 15 {
        return None;
    }
    Some(rl_code | nwr_code << 4)
}

/// MR3: OP[2:0] pull-down drive strength, OP[4:3] bank organization,
/// OP[5] WL set (0: set A); OP[7:6] raw.
pub const fn mr3(pdds: Rzq, bank_org: BankOrg, wl_set_b: bool, raw_7_6: u8) -> Option<u8> {
    if raw_7_6 > 3 {
        return None;
    }
    Some(pdds as u8 | (bank_org as u8) << 3 | (wl_set_b as u8) << 5 | raw_7_6 << 6)
}

/// MR10: OP[3:2] WCK postamble (0: 2.5 tWCK), OP[5:4] RDQS preamble (0:
/// 4 tWCK static, 0 toggle), OP[7:6] RDQS postamble (0: 0.5 tWCK); OP[1:0]
/// raw. The codes other than 0 are as JEDEC LPDDR5/5X defines them.
pub const fn mr10(raw_1_0: u8, wck_pst: u8, rdqs_pre: u8, rdqs_pst: u8) -> Option<u8> {
    if raw_1_0 > 3 || wck_pst > 3 || rdqs_pre > 3 || rdqs_pst > 3 {
        return None;
    }
    Some(raw_1_0 | wck_pst << 2 | rdqs_pre << 4 | rdqs_pst << 6)
}

/// MR11: OP[2:0] DQ ODT, OP[3] non-target ODT (0: disabled), OP[6:4] CA ODT.
pub const fn mr11(dq_odt: Rzq, nt_odt: bool, ca_odt: Rzq) -> u8 {
    dq_odt as u8 | (nt_odt as u8) << 3 | (ca_odt as u8) << 4
}

/// MR16, from the A733 code: FSP-WR, FSP-OP, CBT mode, VRCG, CBT phase.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Mr16 {
    pub fsp_wr: u8,
    pub fsp_op: u8,
    pub cbt: u8,
    pub vrcg: bool,
    pub cbt_phase: bool,
}

impl Mr16 {
    pub const fn encode(self) -> u8 {
        (self.fsp_wr & 3)
            | (self.fsp_op & 3) << 2
            | (self.cbt & 3) << 4
            | (self.vrcg as u8) << 6
            | (self.cbt_phase as u8) << 7
    }
    pub const fn decode(op: u8) -> Mr16 {
        Mr16 {
            fsp_wr: op & 3,
            fsp_op: (op >> 2) & 3,
            cbt: (op >> 4) & 3,
            vrcg: op & 0x40 != 0,
            cbt_phase: op & 0x80 != 0,
        }
    }
}

/// MR17: OP[2:0] SoC ODT; OP[7:3] raw (the source's defaults: OP[3..=5]
/// set, OP[6], OP[7] clear, [`MR17_DEFAULT_7_3`]). OP[5:3] are ODTD-CA,
/// ODTD-CS and ODTD-CK with 1 = ODT disabled, default 1 (JEDEC LPDDR5
/// value), so 0b00111 is the JEDEC reset state: CK, CS and CA ODT all
/// disabled. Named fields: [`Mr17`].
pub const MR17_DEFAULT_7_3: u8 = 0b00111;
pub const fn mr17(soc_odt: Rzq, raw_7_3: u8) -> Option<u8> {
    if raw_7_3 > 31 {
        return None;
    }
    Some(soc_odt as u8 | raw_7_3 << 3)
}

/// MR18: OP[2:0] WCK ODT, OP[3] WCK frequency mode (0: low frequency),
/// OP[4] WCK always on (0: disabled), OP[5] raw, OP[6] WCK2CK leveling,
/// OP[7] CK ratio (1: WCK:CK 2:1, 0: 4:1; [`super::WckCkRatio`]).
///
/// OP[7] CKR: 0 = 4:1, 1 = 2:1 (the default; 2:1 only up to 3200 Mb/s),
/// JEDEC LPDDR5 value, matching LiteDRAM. Full layout: OP[2:0] WCK ODT
/// (111b RFU), OP[3] WCK FM (1 high frequency), OP[4] WCK always on, OP[5]
/// RFU, OP[6] WCK2CK leveling; reset value [`MR18_DEFAULT`]. JEDEC LPDDR5
/// value.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Mr18 {
    pub wck_odt: Rzq,
    pub wck_fm: bool,
    pub wck_always_on: bool,
    pub raw5: bool,
    pub leveling: bool,
    pub ratio_2_1: bool,
}

impl Mr18 {
    pub const fn encode(self) -> u8 {
        self.wck_odt as u8
            | (self.wck_fm as u8) << 3
            | (self.wck_always_on as u8) << 4
            | (self.raw5 as u8) << 5
            | (self.leveling as u8) << 6
            | (self.ratio_2_1 as u8) << 7
    }
    pub const fn decode(op: u8) -> Option<Mr18> {
        match Rzq::from_code(op & 7) {
            Some(wck_odt) => Some(Mr18 {
                wck_odt,
                wck_fm: op & 8 != 0,
                wck_always_on: op & 0x10 != 0,
                raw5: op & 0x20 != 0,
                leveling: op & 0x40 != 0,
                ratio_2_1: op & 0x80 != 0,
            }),
            None => None,
        }
    }
}

/// MR18 reset value: everything 0 except OP[7] CKR = 1 (2:1). JEDEC LPDDR5
/// value.
pub const MR18_DEFAULT: u8 = 0x80;

/// MR20: OP[1:0] RDQS mode (the source writes 0b01); OP[7:2] raw.
pub const fn mr20(rdqs: u8, raw_7_2: u8) -> Option<u8> {
    if rdqs > 3 || raw_7_2 > 63 {
        return None;
    }
    Some(rdqs | raw_7_2 << 2)
}

/// MR22 = 0: write and read link ECC disabled (the source's value).
pub const MR22_LINK_ECC_OFF: u8 = 0;

/// MR28: OP[0] ZQ reset, OP[1] ZQ stop, OP[3:2] ZQ background calibration
/// interval (0b01: 64 ms, the default), OP[5] ZQ mode; OP[4], OP[7:6] raw.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Mr28 {
    pub zq_reset: bool,
    pub zq_stop: bool,
    pub zq_interval: u8,
    pub zq_mode: bool,
    pub raw: u8,
}

impl Mr28 {
    pub const fn encode(self) -> Option<u8> {
        if self.zq_interval > 3 || self.raw & !0xd0 != 0 {
            return None;
        }
        Some(
            self.zq_reset as u8
                | (self.zq_stop as u8) << 1
                | self.zq_interval << 2
                | (self.zq_mode as u8) << 5
                | self.raw,
        )
    }
    pub const fn decode(op: u8) -> Mr28 {
        Mr28 {
            zq_reset: op & 1 != 0,
            zq_stop: op & 2 != 0,
            zq_interval: (op >> 2) & 3,
            zq_mode: op & 0x20 != 0,
            raw: op & 0xd0,
        }
    }
}

/// MR numbers added with the free sources (P1-39).
pub const MR8: u8 = 8;
pub const MR26: u8 = 26;
pub const MR30: u8 = 30;

/// MR8 OP[1:0]: device type. 00b LPDDR5 and 01b LPDDR5X are JEDEC LPDDR5
/// values; 10b ("LPDDR5X up to 9600 Mb/s", Micron addenda) is a vendor
/// extension, reserved in JEDEC; 11b is reserved.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DeviceType {
    Lpddr5,
    Lpddr5x,
    /// 10b: vendor extension (Micron: LPDDR5X up to 9600 Mb/s), not JEDEC.
    VendorLpddr5x9600,
}

/// MR8 I/O width, OP[7:6]: 00b x16, 01b x8, others reserved. JEDEC LPDDR5
/// value (also in the Micron addenda).
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum IoWidth {
    X16,
    X8,
}

/// MR8 (read-only): OP[7:6] I/O width, OP[5:2] density per die (0000b 2,
/// 0001b 3, 0010b 4, 0011b 6, 0100b 8, 0101b 12, 0110b 16, 0111b 24, 1000b
/// 32 Gb; others reserved), OP[1:0] device type ([`DeviceType`]). JEDEC
/// LPDDR5 value; the x16/x8, 8/16 Gb and type codes also match the Micron
/// LPDDR5/5X addenda Y52Q (`references/datasheets/vendor/micron_315b-441b-
/// 561b_y52q_sdp_ddp_lpddr5x.pdf`, sha256 477cc488..., p.29), Y52P (sha256
/// 44013607..., p.39) and Y42M (sha256 caeb8893..., p.34).
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Mr8 {
    pub io_width: Option<IoWidth>,
    pub density_gb: Option<u8>,
    /// OP[1:0] raw.
    pub device_type: u8,
}

/// MR8 OP[5:2] density codes 0..=8 in Gb per die. JEDEC LPDDR5 value.
pub const MR8_DENSITY_GB: [u8; 9] = [2, 3, 4, 6, 8, 12, 16, 24, 32];

impl Mr8 {
    pub const fn decode(op: u8) -> Mr8 {
        let d = ((op >> 2) & 0xf) as usize;
        Mr8 {
            io_width: match op >> 6 {
                0 => Some(IoWidth::X16),
                1 => Some(IoWidth::X8),
                _ => None,
            },
            density_gb: if d < MR8_DENSITY_GB.len() {
                Some(MR8_DENSITY_GB[d])
            } else {
                None
            },
            device_type: op & 3,
        }
    }
    /// OP[1:0] as a [`DeviceType`]; None for 11b (reserved).
    pub const fn device_type_kind(self) -> Option<DeviceType> {
        match self.device_type {
            0b00 => Some(DeviceType::Lpddr5),
            0b01 => Some(DeviceType::Lpddr5x),
            0b10 => Some(DeviceType::VendorLpddr5x9600),
            _ => None,
        }
    }
}

/// MR26 duty-cycle monitor (DCM): OP[5] DCMU1, OP[4] DCMU0, OP[3] DCML1,
/// OP[2] DCML0, OP[1] DCM flip, OP[0] DCM start/stop; OP[7:6] raw. Layout:
/// coreboot `src/vendorcode/mediatek/mt8195/include/dramc_pi_api.h`
/// fec3d608 lines 1602-1607 (`MR26_*`), BSD-3-Clause (sha256 4a8d1e86...).
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Mr26 {
    pub dcmu1: bool,
    pub dcmu0: bool,
    pub dcml1: bool,
    pub dcml0: bool,
    pub flip: bool,
    pub start: bool,
    /// OP[7:6], raw.
    pub raw_7_6: u8,
}

impl Mr26 {
    pub const fn encode(self) -> u8 {
        (self.raw_7_6 & 3) << 6
            | (self.dcmu1 as u8) << 5
            | (self.dcmu0 as u8) << 4
            | (self.dcml1 as u8) << 3
            | (self.dcml0 as u8) << 2
            | (self.flip as u8) << 1
            | self.start as u8
    }
    pub const fn decode(op: u8) -> Mr26 {
        Mr26 {
            dcmu1: op & 0x20 != 0,
            dcmu0: op & 0x10 != 0,
            dcml1: op & 0x08 != 0,
            dcml0: op & 0x04 != 0,
            flip: op & 0x02 != 0,
            start: op & 0x01 != 0,
            raw_7_6: op >> 6,
        }
    }
}

/// MR30 duty-cycle adjuster (DCA): OP[7:4] upper byte, OP[3:0] lower byte
/// code. Layout: coreboot mt8195 `dramc_pi_api.h` fec3d608 lines 1598-1599
/// (`MR30_DCAU`, `MR30_DCAL`), BSD-3-Clause.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Mr30 {
    pub dca_upper: u8,
    pub dca_lower: u8,
}

impl Mr30 {
    pub const fn encode(self) -> Option<u8> {
        if self.dca_upper > 15 || self.dca_lower > 15 {
            return None;
        }
        Some(self.dca_upper << 4 | self.dca_lower)
    }
    pub const fn decode(op: u8) -> Mr30 {
        Mr30 {
            dca_upper: op >> 4,
            dca_lower: op & 0xf,
        }
    }
}

// ---------------------------------------------------------------------------
// P1-40: full layouts. Each layout, code and default below is a JEDEC LPDDR5
// value; the ones marked LPDDR5X are JEDEC LPDDR5X values.

/// Register numbers of the layouts below (MR8, MR10-MR18, MR20, MR26, MR30
/// are above).
pub const MR0: u8 = 0;
pub const MR19: u8 = 19;
pub const MR24: u8 = 24;
pub const MR31: u8 = 31;
pub const MR32: u8 = 32;
pub const MR33: u8 = 33;
pub const MR34: u8 = 34;
pub const MR35: u8 = 35;
pub const MR36: u8 = 36;
pub const MR37: u8 = 37;
pub const MR38: u8 = 38;
pub const MR39: u8 = 39;
pub const MR40: u8 = 40;
pub const MR41: u8 = 41;
pub const MR46: u8 = 46;
pub const MR58: u8 = 58;
pub const MR69: u8 = 69;
pub const MR70: u8 = 70;
pub const MR71: u8 = 71;
pub const MR72: u8 = 72;
pub const MR73: u8 = 73;
pub const MR74: u8 = 74;

const fn rzq_code(r: Rzq) -> u8 {
    r as u8
}

/// MR10 OP[3:2] WCK postamble (read and write): 2.5 (default), 4.5 or 6.5
/// tWCK; 11b reserved. Must be longer than the RDQS postamble.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum WckPst {
    T2_5 = 0,
    T4_5 = 1,
    T6_5 = 2,
}

/// MR10 OP[7:6] RDQS postamble: 0.5 (default), 2.5 or 4.5 tWCK; 11b
/// reserved.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RdqsPst {
    T0_5 = 0,
    T2_5 = 1,
    T4_5 = 2,
}

/// MR10 (write-only, per FSP): OP[0] RPST mode (0 toggle, default; 1
/// static), OP[1] RDQS PRE-2 (LPDDR5X above 3200 Mb/s: 1 ignores OP[5:4],
/// static = tRDQS_PRE + 2 tWCK, toggle = 2 tWCK), OP[3:2] [`WckPst`],
/// OP[5:4] RDQS PRE-1 (00b static 4 / toggle 0 tWCK, the default; 01b 2/2;
/// 10b 0/4; 11b tRDQS_PRE/4, above 3200 Mb/s only), OP[7:6] [`RdqsPst`].
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Mr10 {
    pub rpst_static: bool,
    pub rdqs_pre2: bool,
    pub wck_pst: WckPst,
    pub rdqs_pre1: u8,
    pub rdqs_pst: RdqsPst,
}

impl Mr10 {
    pub const fn encode(self) -> Option<u8> {
        if self.rdqs_pre1 > 3 {
            return None;
        }
        Some(
            self.rpst_static as u8
                | (self.rdqs_pre2 as u8) << 1
                | (self.wck_pst as u8) << 2
                | self.rdqs_pre1 << 4
                | (self.rdqs_pst as u8) << 6,
        )
    }
    /// None for the reserved WCK PST or RDQS PST codes.
    pub const fn decode(op: u8) -> Option<Mr10> {
        let wck_pst = match (op >> 2) & 3 {
            0 => WckPst::T2_5,
            1 => WckPst::T4_5,
            2 => WckPst::T6_5,
            _ => return None,
        };
        let rdqs_pst = match op >> 6 {
            0 => RdqsPst::T0_5,
            1 => RdqsPst::T2_5,
            2 => RdqsPst::T4_5,
            _ => return None,
        };
        Some(Mr10 {
            rpst_static: op & 1 != 0,
            rdqs_pre2: op & 2 != 0,
            wck_pst,
            rdqs_pre1: (op >> 4) & 3,
            rdqs_pst,
        })
    }
}

/// MR11 (per FSP): OP[2:0] DQ ODT, OP[3] NT-ODT enable, OP[6:4] CA ODT
/// (000b disabled, RZQ/1..RZQ/6, 111b RFU), OP[7] CS ODT option (LPDDR5X;
/// must be 0 on LPDDR5).
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Mr11 {
    pub dq_odt: Rzq,
    pub nt_odt: bool,
    pub ca_odt: Rzq,
    pub cs_odt_option: bool,
}

impl Mr11 {
    pub const fn encode(self) -> u8 {
        rzq_code(self.dq_odt)
            | (self.nt_odt as u8) << 3
            | rzq_code(self.ca_odt) << 4
            | (self.cs_odt_option as u8) << 7
    }
    pub const fn decode(op: u8) -> Option<Mr11> {
        match (Rzq::from_code(op & 7), Rzq::from_code((op >> 4) & 7)) {
            (Some(dq_odt), Some(ca_odt)) => Some(Mr11 {
                dq_odt,
                nt_odt: op & 8 != 0,
                ca_odt,
                cs_odt_option: op & 0x80 != 0,
            }),
            _ => None,
        }
    }
}

/// MR12/MR14/MR15 Vref: OP[6:0] a [`super::VrefCode`] (10.0 % + 0.5 % per
/// code of VDDQ); OP[7] is MR12 VBS (byte select in byte mode, 1 upper; 0 on
/// x16), MR14 VDLC (x16: 1 = the upper byte copies MR14) or MR15 OCC (Rx
/// offset calibration, LPDDR5X). Reset code [`VREF_DEFAULT_CODE`] (50.0 %);
/// codes up to [`VREF_TEST_ONLY_MAX`] are for test only.
pub const VREF_DEFAULT_CODE: u8 = 0b101_0000;
pub const VREF_TEST_ONLY_MAX: u8 = 9;

/// One Vref register (MR12, MR14 or MR15): the code and OP[7].
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct VrefMr {
    pub code: u8,
    pub op7: bool,
}

impl VrefMr {
    pub const fn encode(self) -> Option<u8> {
        if self.code > 127 {
            return None;
        }
        Some(self.code | (self.op7 as u8) << 7)
    }
    pub const fn decode(op: u8) -> VrefMr {
        VrefMr {
            code: op & 0x7f,
            op7: op & 0x80 != 0,
        }
    }
}

/// Absolute CA Vref limits: 75 mV to 350 mV.
pub const CA_VREF_MV: (u32, u32) = (75, 350);

/// Absolute DQ Vref limits in mV (min, max) at a WCK frequency: max 350 mV
/// up to 1600 MHz, 225 mV up to 3200 MHz, 180 mV above; 300 mV in E-DVFSC;
/// min 75 mV.
pub const fn dq_vref_mv(wck_mhz: u32, edvfsc: bool) -> (u32, u32) {
    let max = if edvfsc {
        300
    } else if wck_mhz <= 1600 {
        350
    } else if wck_mhz <= 3200 {
        225
    } else {
        180
    };
    (75, max)
}

/// MR13 OP[1:0] thermal offset: none (0-5 C, default), 5 C, 10 C; 11b
/// reserved.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ThermalOffset {
    None = 0,
    C5 = 1,
    C10 = 2,
}

/// MR13: OP[1:0] [`ThermalOffset`], OP[2] VRO (Vref output on DQ), OP[3]
/// RFU, OP[4] DMI I/O control (1: DMI drives valid data for read FIFO and
/// RDC with DM or WR-DBI on), OP[5] DMD (1 data mask disabled), OP[6] CBT
/// mode 2, OP[7] VDD2 single rail ([`Vdd2Rail`]; set during init only; with
/// a single rail MR19 OP[1:0] is ignored).
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Mr13 {
    pub thermal_offset: ThermalOffset,
    pub vro: bool,
    pub dmi_io: bool,
    pub dm_disable: bool,
    pub cbt_mode2: bool,
    pub single_rail: bool,
}

impl Mr13 {
    pub const fn encode(self) -> u8 {
        self.thermal_offset as u8
            | (self.vro as u8) << 2
            | (self.dmi_io as u8) << 4
            | (self.dm_disable as u8) << 5
            | (self.cbt_mode2 as u8) << 6
            | (self.single_rail as u8) << 7
    }
    /// None for the reserved thermal offset or OP[3] set.
    pub const fn decode(op: u8) -> Option<Mr13> {
        let thermal_offset = match op & 3 {
            0 => ThermalOffset::None,
            1 => ThermalOffset::C5,
            2 => ThermalOffset::C10,
            _ => return None,
        };
        if op & 8 != 0 {
            return None;
        }
        Some(Mr13 {
            thermal_offset,
            vro: op & 4 != 0,
            dmi_io: op & 0x10 != 0,
            dm_disable: op & 0x20 != 0,
            cbt_mode2: op & 0x40 != 0,
            single_rail: op & 0x80 != 0,
        })
    }
}

/// MR13 OP[7]: VDD2 rail mode.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Vdd2Rail {
    /// 1: single rail, VDD2L = VDD2H (nominal 1.05 V).
    Single,
    /// 0 (default): dual rail, nominal 1.05 V and 0.9 V.
    Dual,
}

impl Vdd2Rail {
    pub const fn from_mr13(op: u8) -> Vdd2Rail {
        if op & 0x80 != 0 {
            Vdd2Rail::Single
        } else {
            Vdd2Rail::Dual
        }
    }
    /// The VDD2L operating range in mV (min, max): single rail 1010-1120 mV
    /// (VDD2L = VDD2H), dual rail 870-970 mV.
    pub const fn vdd2l_mv(self) -> (u32, u32) {
        match self {
            Vdd2Rail::Single => (1010, 1120),
            Vdd2Rail::Dual => (870, 970),
        }
    }
}

/// MR16 FSP codes (OP[1:0] FSP-WR, OP[3:2] FSP-OP): 0..=2, 3 reserved; CBT
/// (OP[5:4]): 0 normal, n = CBT for FSP n-1. [`Mr16`] carries them raw.
pub const fn mr16_valid(m: Mr16) -> bool {
    m.fsp_wr < 3 && m.fsp_op < 3
}

/// MR17 (per FSP) named: OP[2:0] SoC ODT, OP[3] ODTD-CK, OP[4] ODTD-CS,
/// OP[5] ODTD-CA (1 = disabled, default 1; CS ODT ignored when MR21 OP[3] =
/// 0), OP[6]/OP[7] x8ODTD lower/upper byte (byte mode; 1 = CA/CS/CK ODT
/// disabled).
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Mr17 {
    pub soc_odt: Rzq,
    pub ck_odt_disable: bool,
    pub cs_odt_disable: bool,
    pub ca_odt_disable: bool,
    pub x8_odt_disable_lower: bool,
    pub x8_odt_disable_upper: bool,
}

/// MR17 reset value: SoC ODT off, CK/CS/CA ODT disabled.
pub const MR17_DEFAULT: u8 = MR17_DEFAULT_7_3 << 3;

impl Mr17 {
    pub const fn encode(self) -> u8 {
        rzq_code(self.soc_odt)
            | (self.ck_odt_disable as u8) << 3
            | (self.cs_odt_disable as u8) << 4
            | (self.ca_odt_disable as u8) << 5
            | (self.x8_odt_disable_lower as u8) << 6
            | (self.x8_odt_disable_upper as u8) << 7
    }
    pub const fn decode(op: u8) -> Option<Mr17> {
        match Rzq::from_code(op & 7) {
            Some(soc_odt) => Some(Mr17 {
                soc_odt,
                ck_odt_disable: op & 0x08 != 0,
                cs_odt_disable: op & 0x10 != 0,
                ca_odt_disable: op & 0x20 != 0,
                x8_odt_disable_lower: op & 0x40 != 0,
                x8_odt_disable_upper: op & 0x80 != 0,
            }),
            None => None,
        }
    }
}

/// MR19 OP[1:0]: 00b high-speed mode, VDD2H only (default); 01b DVFSC
/// (low speed, VDD2L); 10b enhanced DVFSC; 11b RFU.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Dvfsc {
    HighSpeed = 0,
    Dvfsc = 1,
    Enhanced = 2,
}

/// MR41 OP[2:1] (read-only, LPDDR5X): 00b DVFSC only, 01b enhanced DVFSC
/// only, 10b both; 11b RFU. LPDDR5 supports DVFSC.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DvfscSupport {
    DvfscOnly,
    EnhancedOnly,
    Both,
}

impl DvfscSupport {
    pub const fn from_mr41(op: u8) -> Option<DvfscSupport> {
        match (op >> 1) & 3 {
            0 => Some(DvfscSupport::DvfscOnly),
            1 => Some(DvfscSupport::EnhancedOnly),
            2 => Some(DvfscSupport::Both),
            _ => None,
        }
    }
    /// Whether MR19 OP[1:0] may be `d` on a die with this support.
    pub const fn allows(self, d: Dvfsc) -> bool {
        matches!(
            (self, d),
            (_, Dvfsc::HighSpeed)
                | (DvfscSupport::DvfscOnly, Dvfsc::Dvfsc)
                | (DvfscSupport::EnhancedOnly, Dvfsc::Enhanced)
                | (DvfscSupport::Both, _)
        )
    }
}

/// MR19 (per FSP): OP[1:0] [`Dvfsc`], OP[3:2] DVFSQ (00b VDDQ 0.5 V,
/// default; 01b 0.3 V with all ODT forced off; others reserved), OP[4]
/// WCK2DQ oscillator for high-frequency mode, OP[5] WCK2DQ oscillator FM
/// supported (read-only), OP[7:6] CS ODT (LPDDR5X: 00b RZQ/3, default; 01b
/// RZQ/2; 10b RZQ/1; 11b RFU), carried as the code.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Mr19 {
    pub dvfsc: Dvfsc,
    pub dvfsq_0v3: bool,
    pub osc_hf: bool,
    pub osc_fm_supported: bool,
    pub cs_odt_code: u8,
}

impl Mr19 {
    pub const fn encode(self) -> Option<u8> {
        if self.cs_odt_code > 2 {
            return None;
        }
        Some(
            self.dvfsc as u8
                | (self.dvfsq_0v3 as u8) << 2
                | (self.osc_hf as u8) << 4
                | (self.osc_fm_supported as u8) << 5
                | self.cs_odt_code << 6,
        )
    }
    pub const fn decode(op: u8) -> Option<Mr19> {
        let dvfsc = match op & 3 {
            0 => Dvfsc::HighSpeed,
            1 => Dvfsc::Dvfsc,
            2 => Dvfsc::Enhanced,
            _ => return None,
        };
        let dvfsq_0v3 = match (op >> 2) & 3 {
            0 => false,
            1 => true,
            _ => return None,
        };
        if op >> 6 == 3 {
            return None;
        }
        Some(Mr19 {
            dvfsc,
            dvfsq_0v3,
            osc_hf: op & 0x10 != 0,
            osc_fm_supported: op & 0x20 != 0,
            cs_odt_code: op >> 6,
        })
    }
    /// The CS ODT strength (RZQ/3, RZQ/2, RZQ/1).
    pub const fn cs_odt(self) -> Rzq {
        match self.cs_odt_code {
            0 => Rzq::Div3,
            1 => Rzq::Div2,
            _ => Rzq::Div1,
        }
    }
}

/// MR20 OP[1:0] RDQS: 00b disabled, 01b RDQS_t only (default), 10b RDQS_t
/// and RDQS_c, 11b RDQS_c only.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RdqsMode {
    Disabled = 0,
    TrueOnly = 1,
    Differential = 2,
    ComplementOnly = 3,
}

/// MR20 OP[3:2] WCK mode: 00b differential (default), 01b single-ended
/// WCK_t, 10b single-ended WCK_c (low-frequency mode only); 11b reserved.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum WckMode {
    Differential = 0,
    SingleTrue = 1,
    SingleComplement = 2,
}

/// MR20 (per FSP): [`RdqsMode`], [`WckMode`], OP[4]/OP[5] MRW disable for
/// the lower/upper byte (byte mode), OP[6] RDC DMI low-fixed, OP[7] RDC DQ
/// mode (0: MR31/MR32 bits invert, 1: they low-fix).
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Mr20 {
    pub rdqs: RdqsMode,
    pub wck: WckMode,
    pub mrw_disable_lower: bool,
    pub mrw_disable_upper: bool,
    pub rdc_dmi_low_fix: bool,
    pub rdc_dq_low_fix: bool,
}

/// MR20 reset value: RDQS_t only, differential WCK.
pub const MR20_DEFAULT: u8 = 0x01;

impl Mr20 {
    pub const fn encode(self) -> u8 {
        self.rdqs as u8
            | (self.wck as u8) << 2
            | (self.mrw_disable_lower as u8) << 4
            | (self.mrw_disable_upper as u8) << 5
            | (self.rdc_dmi_low_fix as u8) << 6
            | (self.rdc_dq_low_fix as u8) << 7
    }
    pub const fn decode(op: u8) -> Option<Mr20> {
        let rdqs = match op & 3 {
            0 => RdqsMode::Disabled,
            1 => RdqsMode::TrueOnly,
            2 => RdqsMode::Differential,
            _ => RdqsMode::ComplementOnly,
        };
        let wck = match (op >> 2) & 3 {
            0 => WckMode::Differential,
            1 => WckMode::SingleTrue,
            2 => WckMode::SingleComplement,
            _ => return None,
        };
        Some(Mr20 {
            rdqs,
            wck,
            mrw_disable_lower: op & 0x10 != 0,
            mrw_disable_upper: op & 0x20 != 0,
            rdc_dmi_low_fix: op & 0x40 != 0,
            rdc_dq_low_fix: op & 0x80 != 0,
        })
    }
}

/// MR24: OP[2:0] DFEQL and OP[6:4] DFEQU (write-only DFE quantity per byte:
/// 0 off, default; 1 minimum .. 7 maximum negative feedback; LPDDR5 has the
/// 3-step DFE only (1-3), LPDDR5X 3-step or 7-step), OP[3] read DCA
/// supported and OP[7] DFE supported (read-only).
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Mr24 {
    pub dfe_lower: u8,
    pub read_dca_supported: bool,
    pub dfe_upper: u8,
    pub dfe_supported: bool,
}

impl Mr24 {
    pub const fn encode(self) -> Option<u8> {
        if self.dfe_lower > 7 || self.dfe_upper > 7 {
            return None;
        }
        Some(
            self.dfe_lower
                | (self.read_dca_supported as u8) << 3
                | self.dfe_upper << 4
                | (self.dfe_supported as u8) << 7,
        )
    }
    pub const fn decode(op: u8) -> Mr24 {
        Mr24 {
            dfe_lower: op & 7,
            read_dca_supported: op & 8 != 0,
            dfe_upper: (op >> 4) & 7,
            dfe_supported: op & 0x80 != 0,
        }
    }
    /// Highest DFE quantity code for a 3-step or 7-step DFE.
    pub const fn dfe_max_code(seven_step: bool) -> u8 {
        if seven_step {
            7
        } else {
            3
        }
    }
}

impl Mr26 {
    /// OP[6] RDQSTFS (read-only): RD/WR-based WCK-RDQS_t training supported.
    pub const fn rdqst_training_supported(self) -> bool {
        self.raw_7_6 & 1 != 0
    }
    /// OP[7] RDQSTFE (write): enable RD/WR-based WCK-RDQS_t training.
    pub const fn rdqst_training_enabled(self) -> bool {
        self.raw_7_6 & 2 != 0
    }
}

/// MR30 (WCK DCA) and MR69 (read DCA, LPDDR5X optional) nibble codes:
/// 0000b 0 steps (default); 0001b..0111b -1..-7 (lower duty cycle); 1000b
/// RFU; 1001b..1111b +1..+7. Upper byte OP[7:4], lower byte OP[3:0]
/// ([`Mr30`] carries the two codes). WCK DCA only in WCK high-frequency
/// mode; MR30 must be 0 in low-frequency mode.
pub const fn dca_code(step: i8) -> Option<u8> {
    match step {
        0 => Some(0),
        -7..=-1 => Some((-step) as u8),
        1..=7 => Some(8 + step as u8),
        _ => None,
    }
}

/// The step of a DCA nibble code; None for 1000b (RFU) or above 15.
pub const fn dca_step(code: u8) -> Option<i8> {
    match code {
        0 => Some(0),
        1..=7 => Some(-(code as i8)),
        9..=15 => Some((code - 8) as i8),
        _ => None,
    }
}

impl Mr30 {
    /// (upper, lower) DCA steps; None if either code is RFU.
    pub const fn steps(self) -> Option<(i8, i8)> {
        match (dca_step(self.dca_upper), dca_step(self.dca_lower)) {
            (Some(u), Some(l)) => Some((u, l)),
            _ => None,
        }
    }
    pub const fn from_steps(upper: i8, lower: i8) -> Option<Mr30> {
        match (dca_code(upper), dca_code(lower)) {
            (Some(u), Some(l)) => Some(Mr30 {
                dca_upper: u,
                dca_lower: l,
            }),
            _ => None,
        }
    }
}

/// Read DQ calibration (RDC): MR31 (lower byte, DQ0-7) and MR32 (upper
/// byte, DQ8-15) per-bit masks, default 55h (MR20 OP[7] = 0: 1 inverts the
/// pattern, = 1: 1 low-fixes it; DMI is never inverted); MR33 pattern A,
/// default 5Ah; MR34 pattern B, default 3Ch.
pub const RDC_DEFAULTS: [(u8, u8); 4] = [(MR31, 0x55), (MR32, 0x55), (MR33, 0x5a), (MR34, 0x3c)];

/// The 16-bit RDC burst on one DQ: MR33 then MR34, low-order bit first
/// (27h gives 1,1,1,0,0,1,0,0).
pub const fn rdc_burst(mr33: u8, mr34: u8) -> [bool; 16] {
    let mut out = [false; 16];
    let mut i = 0;
    while i < 8 {
        out[i] = (mr33 >> i) & 1 != 0;
        out[8 + i] = (mr34 >> i) & 1 != 0;
        i += 1;
    }
    out
}

/// The RDC burst a DQ pin returns: [`rdc_burst`], inverted (or driven low,
/// `low_fix`) where its MR31/MR32 mask bit is set.
pub const fn rdc_pin_burst(mr33: u8, mr34: u8, mask_bit: bool, low_fix: bool) -> [bool; 16] {
    let mut out = rdc_burst(mr33, mr34);
    if mask_bit {
        let mut i = 0;
        while i < 16 {
            out[i] = if low_fix { false } else { !out[i] };
            i += 1;
        }
    }
    out
}

/// The WCK2DQ interval oscillators: input (WCK2DQI, write path) and output
/// (WCK2DQO, read path). Never run both at once; the count is in CK cycles
/// and overflows at 2^16; power-down or deep sleep invalidates it.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Wck2DqOsc {
    Input,
    Output,
}

impl Wck2DqOsc {
    /// (count LSB MR, count MSB MR, run-time MR): MR35/36/37 or MR38/39/40.
    pub const fn registers(self) -> (u8, u8, u8) {
        match self {
            Wck2DqOsc::Input => (MR35, MR36, MR37),
            Wck2DqOsc::Output => (MR38, MR39, MR40),
        }
    }
    /// (start, stop) MPC opcodes ([`super::cmd::mpc`]).
    pub const fn mpc(self) -> (u8, u8) {
        match self {
            Wck2DqOsc::Input => (
                super::cmd::mpc::WCK2DQI_START,
                super::cmd::mpc::WCK2DQI_STOP,
            ),
            Wck2DqOsc::Output => (
                super::cmd::mpc::WCK2DQO_START,
                super::cmd::mpc::WCK2DQO_STOP,
            ),
        }
    }
}

/// The 16-bit oscillator count from its LSB and MSB registers.
pub const fn osc_count(lsb: u8, msb: u8) -> u16 {
    (msb as u16) << 8 | lsb as u16
}

/// MR37/MR40 run time: the same coding as the LPDDR4 MR23 run time (0 stop
/// by MPC; 00nnnnnnb n x 16 clocks; 01b/10b/11b in OP[7:6]: 2048/4096/8192
/// clocks). An MPC stop is illegal when the run time is not 0.
pub use crate::lp4::mr::OscRunTime;

/// MR41: OP[0] per-pin DFE enable (LPDDR5X with MR0 OP[7] = 1, else 0),
/// OP[2:1] [`DvfscSupport`] (read-only), OP[3] E-DVFSC ODT supported
/// (read-only; if 0, all ODT must be off in E-DVFSC), OP[4] PPR enable,
/// OP[7:5] NT DQ ODT (RZQ/n, default RZQ/3, 111b RFU).
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Mr41 {
    pub per_pin_dfe: bool,
    pub dvfsc_code: u8,
    pub edvfsc_odt: bool,
    pub ppr_enable: bool,
    pub nt_dq_odt: Rzq,
}

/// MR41 reset value: NT DQ ODT RZQ/3, the rest 0.
pub const MR41_DEFAULT: u8 = 0b011 << 5;

impl Mr41 {
    pub const fn encode(self) -> Option<u8> {
        if self.dvfsc_code > 3 {
            return None;
        }
        Some(
            self.per_pin_dfe as u8
                | self.dvfsc_code << 1
                | (self.edvfsc_odt as u8) << 3
                | (self.ppr_enable as u8) << 4
                | rzq_code(self.nt_dq_odt) << 5,
        )
    }
    pub const fn decode(op: u8) -> Option<Mr41> {
        match Rzq::from_code(op >> 5) {
            Some(nt_dq_odt) => Some(Mr41 {
                per_pin_dfe: op & 1 != 0,
                dvfsc_code: (op >> 1) & 3,
                edvfsc_odt: op & 8 != 0,
                ppr_enable: op & 0x10 != 0,
                nt_dq_odt,
            }),
            None => None,
        }
    }
    pub const fn dvfsc_support(self) -> Option<DvfscSupport> {
        DvfscSupport::from_mr41(self.dvfsc_code << 1)
    }
}

/// MR46 (write-only): OP[0] enhanced RDQS training mode, OP[1] RDQS toggle
/// mode, OP[2] FIFO RDQS (WCK-RDQS_t / parity) training; OP[7:3] RFU.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct Mr46 {
    pub enhanced_rdqs: bool,
    pub rdqs_toggle: bool,
    pub fifo_rdqs: bool,
}

impl Mr46 {
    pub const fn encode(self) -> u8 {
        self.enhanced_rdqs as u8 | (self.rdqs_toggle as u8) << 1 | (self.fifo_rdqs as u8) << 2
    }
    pub const fn decode(op: u8) -> Option<Mr46> {
        if op & 0xf8 != 0 {
            return None;
        }
        Some(Mr46 {
            enhanced_rdqs: op & 1 != 0,
            rdqs_toggle: op & 2 != 0,
            fifo_rdqs: op & 4 != 0,
        })
    }
}

/// MR58 pre-emphasis strength: off, weak, middle, strong.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Emphasis {
    Off = 0,
    Weak = 1,
    Middle = 2,
    Strong = 3,
}

const fn emphasis(c: u8) -> Emphasis {
    match c & 3 {
        0 => Emphasis::Off,
        1 => Emphasis::Weak,
        2 => Emphasis::Middle,
        _ => Emphasis::Strong,
    }
}

/// MR58 (per FSP): DQ pre-emphasis, OP[1:0] pull-up lower byte, OP[3:2]
/// pull-down lower byte, OP[5:4] pull-up upper byte, OP[7:6] pull-down
/// upper byte. Only above 6400 Mb/s and only through an FSP switch.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Mr58 {
    pub pu_lower: Emphasis,
    pub pd_lower: Emphasis,
    pub pu_upper: Emphasis,
    pub pd_upper: Emphasis,
}

impl Mr58 {
    pub const fn encode(self) -> u8 {
        self.pu_lower as u8
            | (self.pd_lower as u8) << 2
            | (self.pu_upper as u8) << 4
            | (self.pd_upper as u8) << 6
    }
    pub const fn decode(op: u8) -> Mr58 {
        Mr58 {
            pu_lower: emphasis(op),
            pd_lower: emphasis(op >> 2),
            pu_upper: emphasis(op >> 4),
            pd_upper: emphasis(op >> 6),
        }
    }
}

/// MR70-MR74 per-pin DFE offsets (LPDDR5X; valid with MR0 OP[7] = 1 and
/// MR41 OP[0] = 1): four 2-bit fields, OP[1:0] first, each 0 (none) or
/// +1/+2/+3 offset steps. MR70 DQ0-3, MR71 DQ4-7, MR72 DQ8-11, MR73
/// DQ12-15, MR74 DMI0, DMI1, RDQS0, RDQS1.
pub const fn dfe_offsets_encode(steps: [u8; 4]) -> Option<u8> {
    let mut op = 0;
    let mut i = 0;
    while i < 4 {
        if steps[i] > 3 {
            return None;
        }
        op |= steps[i] << (2 * i);
        i += 1;
    }
    Some(op)
}

pub const fn dfe_offsets_decode(op: u8) -> [u8; 4] {
    [op & 3, (op >> 2) & 3, (op >> 4) & 3, op >> 6]
}

/// The MR70-MR74 register of a DQ pin (0..=15), or MR74 for DMI/RDQS.
pub const fn dfe_offset_register(dq: u8) -> Option<(u8, u8)> {
    if dq > 15 {
        return None;
    }
    Some((MR70 + dq / 4, dq % 4))
}

/// MR0 OP[7]: per-pin DFE supported (read-only).
pub const MR0_PER_PIN_DFE: u8 = 1 << 7;

/// P1-40: single bits, kept for callers that test one bit of an operand.
pub mod bits {
    pub use super::{DvfscSupport, Vdd2Rail};
    pub use super::{MR0, MR13, MR19, MR24, MR41};

    /// MR0 OP[7]: per-pin DFE supported.
    pub const MR0_PER_PIN_DFE: u8 = super::MR0_PER_PIN_DFE;
    /// MR13 OP[7]: VDD2 single rail.
    pub const MR13_VDD2_SINGLE_RAIL: u8 = 1 << 7;
    /// MR13 OP[2]: VRO.
    pub const MR13_VRO: u8 = 1 << 2;
    /// MR19 OP[1:0]: DVFSC mode ([`super::Dvfsc`]).
    pub const MR19_DVFSC_MASK: u8 = 0b11;
    /// MR19 OP[5]: WCK2DQ oscillator FM supported.
    pub const MR19_WCK2DQ_OSC_FM: u8 = 1 << 5;
    /// MR24 OP[7]: DFE supported.
    pub const MR24_DFE_SUPPORTED: u8 = 1 << 7;
    /// MR24 OP[3]: read DCA supported.
    pub const MR24_READ_DCA_SUPPORTED: u8 = 1 << 3;
    /// MR26 OP[6]: RD/WR-based RDQS_t training supported.
    pub const MR26_RDQS_T_TRAINING: u8 = 1 << 6;
    /// MR41 OP[2:1]: DVFSC support.
    pub const MR41_DVFSC_MASK: u8 = 0b11 << 1;
    pub const MR41_DVFSC_SHIFT: u8 = 1;
    /// MR41 OP[3]: E-DVFSC ODT supported.
    pub const MR41_E_DVFSC_ODT: u8 = 1 << 3;

    /// Whether `bit` is set in the operand `op`.
    pub const fn has(op: u8, bit: u8) -> bool {
        op & bit == bit
    }
    /// The MR41 OP[2:1] field, raw.
    pub const fn mr41_dvfsc(op: u8) -> u8 {
        (op & MR41_DVFSC_MASK) >> MR41_DVFSC_SHIFT
    }
    /// The MR19 OP[1:0] field, raw.
    pub const fn mr19_dvfsc(op: u8) -> u8 {
        op & MR19_DVFSC_MASK
    }
}
