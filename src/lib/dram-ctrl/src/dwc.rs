// SPDX-License-Identifier: GPL-2.0-only
//! The DesignWare DDRCTL as a packing table: DRAMSET1TMG0/1/2/4 of a
//! frequency block (`dwc_ddrctl::timing`), the registers whose fields are
//! JEDEC timings and latencies. A SoC's other frequency-block registers
//! (DFI timers, ZQ, refresh controls) are its own table entries.

use crate::pack::{Entry, Src};
use crate::values::Name;
use dwc_ddrctl::timing::{dramset1tmg, tmg};

const fn e(reg: usize, field: dwc_ddrctl::timing::Field, name: Name) -> Entry {
    Entry {
        reg,
        field,
        src: Src::Value(name),
    }
}

/// DRAMSET1TMG0, 1, 2 and 4 from named values (offsets within a frequency
/// block).
pub const DRAMSET1TMG: [Entry; 15] = [
    e(dramset1tmg(0), tmg::RAS_MIN, Name::Ras),
    e(dramset1tmg(0), tmg::RAS_MAX, Name::RasMax),
    e(dramset1tmg(0), tmg::FAW, Name::Faw),
    e(dramset1tmg(0), tmg::WR2PRE, Name::Wr2Pre),
    e(dramset1tmg(1), tmg::RC, Name::Rc),
    e(dramset1tmg(1), tmg::RD2PRE, Name::Rd2Pre),
    e(dramset1tmg(1), tmg::XP, Name::Xp),
    e(dramset1tmg(2), tmg::WR2RD, Name::Wr2Rd),
    e(dramset1tmg(2), tmg::RD2WR, Name::Rd2Wr),
    e(dramset1tmg(2), tmg::READ_LATENCY, Name::ReadLatency),
    e(dramset1tmg(2), tmg::WRITE_LATENCY, Name::WriteLatency),
    e(dramset1tmg(4), tmg::RP, Name::Rp),
    e(dramset1tmg(4), tmg::RRD, Name::Rrd),
    e(dramset1tmg(4), tmg::CCD, Name::Ccd),
    e(dramset1tmg(4), tmg::RCD, Name::Rcd),
];

// ---------------------------------------------------------------------------
// Programming modes. Every DesignWare DDRCTL field has one: static (only in
// reset), dynamic (any time), dynamic and refresh related (followed by a
// RFSHCTL3.refresh_update_level toggle), or quasi-dynamic in one or more
// groups, writable outside reset only between SWCTL.sw_done = 0 and
// sw_done = 1 / SWSTAT.sw_done_ack and only while one of its groups' states
// holds:
//
// - group 1: no DFI read/write traffic (DBG1.dis_dq, DBGCAM pipelines empty);
// - group 2: the DRAM in self-refresh, deep power-down or MPSM;
// - group 3: the controller empty (ports disabled, PSTAT 0, queues empty);
// - group 4: the field's shadow copy of the inactive frequency set.
//
// Classes: the uMCTL2 2.40a field pages of the Zynq UltraScale+ register
// reference (AMD UG1087 v1.11, `ddrc___*`) and UG1085 v2.5 p491-494; facts
// only. The A733's DDRCTL is a later release of the same controller (field
// names DRAMSET1TMGn for DRAMTMGn); its classes are taken as the same and
// are unverified against the vendor tables.

/// A set of quasi-dynamic groups (bit n-1 for group n).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Groups(pub u8);

impl Groups {
    pub const G1: Groups = Groups(1);
    pub const G2: Groups = Groups(2);
    pub const G3: Groups = Groups(4);
    pub const G4: Groups = Groups(8);
    pub const fn or(self, o: Groups) -> Groups {
        Groups(self.0 | o.0)
    }
    pub const fn has(self, n: u8) -> bool {
        n >= 1 && n <= 4 && self.0 & (1 << (n - 1)) != 0
    }
}

/// A field's programming mode.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ProgMode {
    Static,
    Dynamic,
    /// Dynamic, refresh related: toggle RFSHCTL3.refresh_update_level after.
    DynamicRefresh,
    QuasiDynamic(Groups),
}

/// One field and its mode (uMCTL2 register and field names).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FieldMode {
    pub reg: &'static str,
    pub field: &'static str,
    pub mode: ProgMode,
}

const fn f(reg: &'static str, field: &'static str, mode: ProgMode) -> FieldMode {
    FieldMode { reg, field, mode }
}

use ProgMode::{Dynamic as D, DynamicRefresh as DR, Static as S};
const G24: ProgMode = ProgMode::QuasiDynamic(Groups(0b1010));
const G124: ProgMode = ProgMode::QuasiDynamic(Groups(0b1011));
const G14: ProgMode = ProgMode::QuasiDynamic(Groups(0b1001));
const Q2: ProgMode = ProgMode::QuasiDynamic(Groups::G2);
const Q3: ProgMode = ProgMode::QuasiDynamic(Groups::G3);
const Q4: ProgMode = ProgMode::QuasiDynamic(Groups::G4);

/// The fields the shared controller code and the boards program.
pub const FIELD_MODES: &[FieldMode] = &[
    f("MSTR", "device_config", S),
    f("MSTR", "frequency_mode", S),
    f("MSTR", "active_ranks", S),
    f("MSTR", "burst_rdwr", S),
    f("MSTR", "dll_off_mode", Q2),
    f("MSTR", "data_bus_width", S),
    f("MSTR", "lpddr4", S),
    f("MRCTRL0", "mr_wr", D),
    f("MRCTRL0", "mr_addr", D),
    f("MRCTRL0", "mr_rank", D),
    f("MRCTRL0", "sw_init_int", D),
    f("MRCTRL1", "mr_data", D),
    f("PWRCTL", "selfref_sw", D),
    f("PWRCTL", "selfref_en", D),
    f("PWRCTL", "powerdown_en", D),
    f("PWRCTL", "en_dfi_dram_clk_disable", D),
    f("RFSHCTL3", "refresh_mode", Q2),
    f("RFSHCTL3", "refresh_update_level", DR),
    f("RFSHCTL3", "dis_auto_refresh", DR),
    f("RFSHTMG", "t_rfc_nom_x32", DR),
    f("RFSHTMG", "t_rfc_min", DR),
    f("INIT0", "skip_dram_init", Q2),
    f("INIT0", "post_cke_x1024", S),
    f("INIT0", "pre_cke_x1024", S),
    f("DRAMTMG0", "wr2pre", G124),
    f("DRAMTMG0", "t_faw", G24),
    f("DRAMTMG0", "t_ras_max", G24),
    f("DRAMTMG0", "t_ras_min", G24),
    f("DRAMTMG1", "t_xp", G24),
    f("DRAMTMG1", "rd2pre", G124),
    f("DRAMTMG1", "t_rc", G24),
    f("DRAMTMG2", "write_latency", G124),
    f("DRAMTMG2", "read_latency", G124),
    f("DRAMTMG2", "rd2wr", G124),
    f("DRAMTMG2", "wr2rd", G124),
    f("DRAMTMG4", "t_rcd", G124),
    f("DRAMTMG4", "t_ccd", G24),
    f("DRAMTMG4", "t_rrd", G24),
    f("DRAMTMG4", "t_rp", G24),
    f("ZQCTL0", "dis_auto_zq", D),
    f("ZQCTL0", "dis_srx_zqcl", G24),
    f("ZQCTL0", "t_zq_long_nop", S),
    f("ZQCTL0", "t_zq_short_nop", S),
    f("DFITMG0", "dfi_t_ctrl_delay", Q4),
    f("DFITMG0", "dfi_t_rddata_en", G14),
    f("DFITMG0", "dfi_tphy_wrdata", Q4),
    f("DFITMG0", "dfi_tphy_wrlat", G14),
    f("DFITMG1", "dfi_t_cmd_lat", G24),
    f("DFITMG1", "dfi_t_wrdata_delay", Q4),
    f("DFITMG1", "dfi_t_dram_clk_disable", Q4),
    f("DFITMG1", "dfi_t_dram_clk_enable", Q4),
    f("DFIUPD0", "dis_auto_ctrlupd", Q2),
    f("DFIUPD0", "dfi_t_ctrlup_max", S),
    f("DFIUPD0", "dfi_t_ctrlup_min", S),
    f("DFIMISC", "dfi_init_complete_en", Q3),
    f("DFIMISC", "phy_dbi_mode", S),
    f("DFILPCFG0", "dfi_lp_en_sr", S),
    f("DFILPCFG0", "dfi_lp_en_pd", S),
    f("ODTMAP", "rank0_wr_odt", S),
    f("ODTMAP", "rank0_rd_odt", S),
    f("DBG1", "dis_dq", D),
    f("DBG1", "dis_hif", D),
    f("PCTRL_0", "port_en", D),
    f("SWCTL", "sw_done", D),
];

/// The mode of a uMCTL2 field, by register and field name.
pub fn mode_of(reg: &str, field: &str) -> Option<ProgMode> {
    FIELD_MODES
        .iter()
        .find(|m| m.reg == reg && m.field == field)
        .map(|m| m.mode)
}

/// The modes of [`DRAMSET1TMG`]'s entries, in order (DRAMSET1TMGn =
/// uMCTL2 DRAMTMGn).
pub const DRAMSET1TMG_MODES: [ProgMode; 15] = [
    G24, G24, G24, G124, // RAS_MIN, RAS_MAX, FAW, WR2PRE
    G24, G124, G24, // RC, RD2PRE, XP
    G124, G124, G124, G124, // WR2RD, RD2WR, READ_LATENCY, WRITE_LATENCY
    G24, G24, G24, G124, // RP, RRD, CCD, RCD
];

/// uMCTL2 status and debug bits used to establish the group states
/// (UG1087 v1.11 `ddrc___dbg1`, `ddrc___dbgcam`, `ddrc___pstat`,
/// `ddrc___rfshctl3`).
pub mod bits {
    pub const DBG1_DIS_DQ: u32 = 1 << 0;
    pub const DBGCAM_RD_DATA_PIPELINE_EMPTY: u32 = 1 << 28;
    pub const DBGCAM_WR_DATA_PIPELINE_EMPTY: u32 = 1 << 29;
    pub const DBGCAM_RD_Q_EMPTY: u32 = 1 << 25;
    pub const DBGCAM_WR_Q_EMPTY: u32 = 1 << 26;
    pub const PCTRL_PORT_EN: u32 = 1 << 0;
    pub const RFSHCTL3_REFRESH_UPDATE_LEVEL: u32 = 1 << 1;
    pub const RFSHCTL3_DIS_AUTO_REFRESH: u32 = 1 << 0;
    pub const DBG1_DIS_HIF: u32 = 1 << 1;
    pub const INIT0_SKIP_DRAM_INIT: u32 = 3 << 30;
    /// skip_dram_init = 01: skip, start in normal mode.
    pub const INIT0_SKIP_NORMAL: u32 = 1 << 30;
    /// skip_dram_init = 11: skip, start in self-refresh.
    pub const INIT0_SKIP_SELFREF: u32 = 3 << 30;
    pub const DFIUPD0_DIS_AUTO_CTRLUPD: u32 = 1 << 31;
    pub const DFIMISC_DFI_INIT_COMPLETE_EN: u32 = 1 << 0;
    pub const ZQCTL0_DIS_AUTO_ZQ: u32 = 1 << 31;
    pub const PWRCTL_SELFREF_SW: u32 = 1 << 5;
    pub const STAT_OPERATING_MODE: u32 = 7;
    pub const STAT_SELFREF_TYPE: u32 = 3 << 4;
    /// operating_mode values.
    pub const MODE_NORMAL: u32 = 1;
    pub const MODE_SELF_REFRESH: u32 = 3;
    /// selfref_type 10: self-refresh not (only) from automatic entry.
    pub const SELFREF_TYPE_SW: u32 = 2 << 4;
}

use crate::steps::{Conditions, Step};

/// Steps that establish group 1 (no DFI read/write traffic): set
/// DBG1.dis_dq, wait for both data pipelines to drain, record the state.
pub const fn enter_group1(dbg1: usize, dbgcam: usize, timeout_us: u64) -> [Step; 3] {
    let empty = bits::DBGCAM_RD_DATA_PIPELINE_EMPTY | bits::DBGCAM_WR_DATA_PIPELINE_EMPTY;
    [
        Step::Modify {
            addr: dbg1,
            clear: 0,
            set: bits::DBG1_DIS_DQ,
        },
        Step::Poll {
            addr: dbgcam,
            mask: empty,
            want: empty,
            timeout_us,
            what: "DBGCAM data pipelines empty",
        },
        Step::Assert(Conditions::GROUP1),
    ]
}

/// Steps that leave group 1: clear DBG1.dis_dq.
pub const fn leave_group1(dbg1: usize) -> [Step; 2] {
    [
        Step::Clear(Conditions::GROUP1),
        Step::Modify {
            addr: dbg1,
            clear: bits::DBG1_DIS_DQ,
            set: 0,
        },
    ]
}

/// Steps that establish group 3 (controller empty) for one port: disable
/// it, wait for PSTAT = 0 and for the queues to drain.
pub const fn enter_group3(pctrl: usize, pstat: usize, dbgcam: usize, timeout_us: u64) -> [Step; 4] {
    let q = bits::DBGCAM_RD_Q_EMPTY | bits::DBGCAM_WR_Q_EMPTY;
    [
        Step::Modify {
            addr: pctrl,
            clear: bits::PCTRL_PORT_EN,
            set: 0,
        },
        Step::Poll {
            addr: pstat,
            mask: u32::MAX,
            want: 0,
            timeout_us,
            what: "PSTAT idle",
        },
        Step::Poll {
            addr: dbgcam,
            mask: q,
            want: q,
            timeout_us,
            what: "DBGCAM queues empty",
        },
        Step::Assert(Conditions::GROUP3),
    ]
}

/// Controller register addresses the hand-over and quiesce templates use
/// (absolute; the board computes them).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Regs {
    pub stat: usize,
    pub init0: usize,
    pub pwrctl: usize,
    pub rfshctl3: usize,
    pub zqctl0: usize,
    pub dfiupd0: usize,
    pub dfimisc: usize,
    pub dbg1: usize,
    pub dbgcam: usize,
    pub swctl: usize,
    pub swstat: usize,
    pub pctrl: usize,
    pub pstat: usize,
}

const fn field(addr: usize, clear: u32, set: u32, reg_field: &'static str) -> Step {
    let mode = match reg_field.as_bytes() {
        b"DFIMISC.dfi_init_complete_en" => Q3,
        b"DFIUPD0.dis_auto_ctrlupd" | b"INIT0.skip_dram_init" => Q2,
        b"RFSHCTL3.dis_auto_refresh" => DR,
        _ => D,
    };
    Step::Field {
        addr,
        clear,
        set,
        mode,
        what: reg_field,
    }
}

/// Hand the DFI to the controller after PHY init and training (the FSBL
/// and Ultra96 psu_init order, UG1085 p491-494): empty the controller
/// (port off, group 3), set DFIMISC.dfi_init_complete_en inside a
/// sw_done handshake, wait for STAT.operating_mode = normal, re-enable
/// automatic ZQ calibration and refresh (with the refresh-update toggle),
/// and re-enable the port.
pub const fn handover(r: &Regs, timeout_us: u64) -> [Step; 15] {
    let g3 = enter_group3(r.pctrl, r.pstat, r.dbgcam, timeout_us);
    [
        Step::Note("hand-over: DFI to the controller"),
        g3[0],
        g3[1],
        g3[2],
        g3[3],
        Step::SwDoneClear { swctl: r.swctl },
        field(
            r.dfimisc,
            0,
            bits::DFIMISC_DFI_INIT_COMPLETE_EN,
            "DFIMISC.dfi_init_complete_en",
        ),
        Step::SwDoneSet {
            swctl: r.swctl,
            swstat: r.swstat,
            timeout_us,
        },
        Step::Poll {
            addr: r.stat,
            mask: bits::STAT_OPERATING_MODE,
            want: bits::MODE_NORMAL,
            timeout_us,
            what: "STAT.operating_mode normal",
        },
        field(r.zqctl0, bits::ZQCTL0_DIS_AUTO_ZQ, 0, "ZQCTL0.dis_auto_zq"),
        field(
            r.rfshctl3,
            bits::RFSHCTL3_DIS_AUTO_REFRESH,
            0,
            "RFSHCTL3.dis_auto_refresh",
        ),
        Step::RefreshUpdate {
            rfshctl3: r.rfshctl3,
        },
        Step::Clear(Conditions::GROUP3),
        Step::Modify {
            addr: r.pctrl,
            clear: 0,
            set: bits::PCTRL_PORT_EN,
        },
        Step::Note("hand-over done"),
    ]
}

/// Stop background updates before the board writes trained delays into
/// the PHY: software self-refresh entry (PWRCTL.selfref_sw, wait for
/// operating_mode = self-refresh with selfref_type = software), which
/// establishes group 2; then, inside a sw_done handshake,
/// DFIUPD0.dis_auto_ctrlupd = 1 (no dfi_ctrlupd_req while the delays
/// change), and automatic ZQ calibration off. [`resume`] undoes it.
pub const fn quiesce(r: &Regs, timeout_us: u64) -> [Step; 8] {
    [
        Step::Note("quiesce: self-refresh, no controller updates"),
        field(r.pwrctl, 0, bits::PWRCTL_SELFREF_SW, "PWRCTL.selfref_sw"),
        Step::Poll {
            addr: r.stat,
            mask: bits::STAT_OPERATING_MODE | bits::STAT_SELFREF_TYPE,
            want: bits::MODE_SELF_REFRESH | bits::SELFREF_TYPE_SW,
            timeout_us,
            what: "STAT self-refresh (software)",
        },
        Step::Assert(Conditions::GROUP2),
        Step::SwDoneClear { swctl: r.swctl },
        field(
            r.dfiupd0,
            0,
            bits::DFIUPD0_DIS_AUTO_CTRLUPD,
            "DFIUPD0.dis_auto_ctrlupd",
        ),
        field(r.zqctl0, 0, bits::ZQCTL0_DIS_AUTO_ZQ, "ZQCTL0.dis_auto_zq"),
        Step::SwDoneSet {
            swctl: r.swctl,
            swstat: r.swstat,
            timeout_us,
        },
    ]
}

/// Undo [`quiesce`]: controller updates back on (sw_done handshake), leave
/// self-refresh, wait for normal mode, automatic ZQ calibration back on.
pub const fn resume(r: &Regs, timeout_us: u64) -> [Step; 8] {
    [
        Step::SwDoneClear { swctl: r.swctl },
        field(
            r.dfiupd0,
            bits::DFIUPD0_DIS_AUTO_CTRLUPD,
            0,
            "DFIUPD0.dis_auto_ctrlupd",
        ),
        Step::SwDoneSet {
            swctl: r.swctl,
            swstat: r.swstat,
            timeout_us,
        },
        Step::Clear(Conditions::GROUP2),
        field(r.pwrctl, bits::PWRCTL_SELFREF_SW, 0, "PWRCTL.selfref_sw"),
        Step::Poll {
            addr: r.stat,
            mask: bits::STAT_OPERATING_MODE,
            want: bits::MODE_NORMAL,
            timeout_us,
            what: "STAT.operating_mode normal",
        },
        field(r.zqctl0, bits::ZQCTL0_DIS_AUTO_ZQ, 0, "ZQCTL0.dis_auto_zq"),
        Step::Note("resumed"),
    ]
}

/// Before a PHY re-init while the controller waits (DRAM in self-refresh
/// or deep power-down, group 2 established; UG1085 p498 steps 1-3): the
/// controller must not drive CKE (INIT0.skip_dram_init = 11), send
/// updates (DFIUPD0.dis_auto_ctrlupd, DBG1.dis_hif) or leave init before
/// the PHY is ready (DFIMISC.dfi_init_complete_en = 0). The controller is
/// emptied first (group 3) for the DFIMISC write. The board then exits
/// the low-power state, runs its PHY init and calls [`phy_reinit_release`].
/// Run [`quiesce`] first (it establishes group 2).
pub const fn phy_reinit_hold(r: &Regs, timeout_us: u64) -> [Step; 11] {
    let g3 = enter_group3(r.pctrl, r.pstat, r.dbgcam, timeout_us);
    [
        Step::Note("PHY re-init: hold the controller"),
        g3[0],
        g3[1],
        g3[2],
        g3[3],
        Step::SwDoneClear { swctl: r.swctl },
        field(
            r.init0,
            bits::INIT0_SKIP_DRAM_INIT,
            bits::INIT0_SKIP_SELFREF,
            "INIT0.skip_dram_init",
        ),
        field(
            r.dfiupd0,
            0,
            bits::DFIUPD0_DIS_AUTO_CTRLUPD,
            "DFIUPD0.dis_auto_ctrlupd",
        ),
        field(r.dbg1, 0, bits::DBG1_DIS_HIF, "DBG1.dis_hif"),
        field(
            r.dfimisc,
            bits::DFIMISC_DFI_INIT_COMPLETE_EN,
            0,
            "DFIMISC.dfi_init_complete_en",
        ),
        Step::SwDoneSet {
            swctl: r.swctl,
            swstat: r.swstat,
            timeout_us,
        },
    ]
}

/// After the PHY re-init (UG1085 p498 steps 5, 7, 8): skip_dram_init back
/// to `skip_after` (01: skip, normal mode, for a controller whose DRAM
/// init the PHY did), updates and the host interface back on,
/// dfi_init_complete_en = 1 so the controller leaves init; software
/// self-refresh off ([`quiesce`] set it), wait for normal mode; groups 2
/// and 3 cleared, the port back on. Check the controller version accepts
/// a skipped DRAM init for the memory type (the ZynqMP DDRC supports only
/// 00 for LPDDR4).
pub const fn phy_reinit_release(r: &Regs, skip_after: u32, timeout_us: u64) -> [Step; 12] {
    [
        Step::SwDoneClear { swctl: r.swctl },
        field(
            r.init0,
            bits::INIT0_SKIP_DRAM_INIT,
            skip_after & bits::INIT0_SKIP_DRAM_INIT,
            "INIT0.skip_dram_init",
        ),
        field(
            r.dfiupd0,
            bits::DFIUPD0_DIS_AUTO_CTRLUPD,
            0,
            "DFIUPD0.dis_auto_ctrlupd",
        ),
        field(r.dbg1, bits::DBG1_DIS_HIF, 0, "DBG1.dis_hif"),
        field(
            r.dfimisc,
            0,
            bits::DFIMISC_DFI_INIT_COMPLETE_EN,
            "DFIMISC.dfi_init_complete_en",
        ),
        Step::SwDoneSet {
            swctl: r.swctl,
            swstat: r.swstat,
            timeout_us,
        },
        Step::Clear(Conditions::GROUP2),
        field(r.pwrctl, bits::PWRCTL_SELFREF_SW, 0, "PWRCTL.selfref_sw"),
        Step::Poll {
            addr: r.stat,
            mask: bits::STAT_OPERATING_MODE,
            want: bits::MODE_NORMAL,
            timeout_us,
            what: "STAT.operating_mode normal",
        },
        Step::Clear(Conditions::GROUP3),
        Step::Modify {
            addr: r.pctrl,
            clear: 0,
            set: bits::PCTRL_PORT_EN,
        },
        Step::Note("PHY re-init: released"),
    ]
}
