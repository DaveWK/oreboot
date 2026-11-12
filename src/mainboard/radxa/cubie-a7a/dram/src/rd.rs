// SPDX-License-Identifier: GPL-2.0-only
//! Read enable and the read data path measured by the host on hardware,
//! after WCK2CK leveling (`wl`), without the PMU. A measurement stage:
//! every changed control is read first and put back, no read enable value
//! is applied, bt0 halts afterwards and the watchdog resets the board into
//! the vendor recovery.
//!
//! What the measurements on hardware established (stages that answered
//! their question were removed):
//! - In the reference's read-enable **training** mode (DbyteRxEnTrain 3):
//!   every RxEnDly over the whole 11-bit field, the receive clock delays
//!   and references from the vendor's trained values, VrefDAC sweeps, the
//!   PUB-generated and free-running strobes, the asynchronous receive data
//!   (the DRAM drives DQ on RDC) and the RX FIFO pointers left every count
//!   constant. Only the ACSM RxEn/RxVal pulse delay moved anything (a hump
//!   in lane 0, the gate sample, at delay 44..54 around the programmed 51).
//! - In data mode after the reference's exit (DbyteRxEnTrain 0) with the
//!   reference's post-read-enable latency, DtsmLaneCtrl0 0x2801 and its
//!   18-row centring program ([`rxen::centring_program`]): about 50 %
//!   errors everywhere; no group of the vendor's CSR values changed that.
//! - Every data-stage write is read back and takes. ch B rank 0 DBYTE 2
//!   captured read data (lanes 0-6 and DMI at 0 errors at some points);
//!   the vendor's CSR values destroyed it.
//! - The live-delay readback SelDbCurDlyTmngInfo (`0x1n08e`, value at
//!   `0x1n08f`: per lane k, selector 0x0a+3k RxClkT fine, 0x0b+3k RxClkC
//!   fine, 0x0c+3k the receive-enable delay, Tg0) showed that **in data
//!   mode RxEnDly is double-buffered and latched by PptRxEnEvnt**
//!   (`0x1f0a2` bit 5) and **RxClk by ForceInternalUpdate** (`0x200a3`); in
//!   read-enable training mode the gate follows the training counter, not
//!   RxEnDly. Searches that fired neither after their writes ran every
//!   data-mode point at the exit's RxEnDly 0x100 and the reset RxClk 0x110,
//!   and their point-to-point changes followed the FIFO read pointer.
//! - The RxEnDly x receive clock search with both latches and RxFifoInit
//!   per point found the first full-lane passes at
//!   read latency 4 (ch A rank 0 DBYTE 1 at RxEnDly 0x4c0-0x600 x RxClk
//!   0x1c0, the vendor's RxClk; ch B rank 0 at isolated points). RxFifoInit
//!   per point removes the read-pointer phase; rank 1 passed nowhere.
//! - The fine latched search found the per-lane eye on
//!   ch A rank 0 (RxEnDly 0x480-0x560, RxClk 56-64 taps around 0x1bc, the
//!   vendor's RxClk; read latency 4 = 5); ch B rank 0 and rank 1 (with Tg0
//!   swept) none.
//!
//! **The search stage**: per block, after the reference set-up, entry and
//! warm-up (VrefDAC from the [`ReadStart`], the receive clock delays as
//! found), the reference exit and the QC set-up (latency group with the
//! channel's HwtControlOvr pulse, DtsmLaneCtrl0 0x2801, the read-centring
//! entry and expected pattern): per read latency, RxEnDly and receive clock
//! delay of the [`Search`] in the [`RdConfig`] (RxEnDly in the register
//! [`TgSel`] names, latched by PptRxEnEvnt; the receive clock delays on
//! lane F, all four registers, latched by ForceInternalUpdate), RxFifoInit
//! and one centring launch. Rows are logged as they complete
//! ([`crate::hw::Hw`] `log`): the passing-lane count per point and DBYTE,
//! and for rows with a pass the first and last passing receive clock delay
//! per lane; per lane the totals ([`Eye`]) are kept. At the first point of
//! every row the live readback of lane 0 is compared with what was written
//! (the receive-enable delay only when Tg0 was written: Tg1 has no
//! readback). bt0 runs the coarse search in the order A0, B0, A1 (Tg1),
//! A1 (Tg0), B1 (Tg1), B1 (Tg0), A0 again, with the start and end time of
//! each block. The reference's exit mode launch (MR18/MR10/MR46) is issued
//! right after the exit instead of only at the block's end.
//!
//! Then everything is put back, each delay with its latch event: the
//! receive clock delays as found, the latency and selects, the all-zero
//! entry (MR33/MR34 = 0), the PHY's pattern set-up undone, RxEnDly (Tg0
//! and the block's own), the references, TrainingParam; the reference's
//! exit mode launch.
use crate::hw::Hw;
use crate::phy::coldinit::Row;
use crate::phy::csr_access;
use crate::phy::rxen::{self, EntryMrs, ModeMrs};
use crate::phy::wcklevel::{self, Channel, Rank, Step};
use crate::wl::{WlExec, WlFailure, LANES};

/// The read-centring pattern as the PHY's expected-data registers hold it
/// (MR34 << 8 | MR33).
pub const PATTERN: u16 = 0xa536;

/// RxClkT2UIDlyTg0/Tg1, RxClkC2UIDlyTg0/Tg1 (`0x1n010..0x1n013 + 0x100
/// lane`).
pub const RX_CLK: [u32; 4] = [0x010, 0x011, 0x012, 0x013];

/// VrefDAC0/VrefDAC1 (`0x1n04e`, `0x1n04f + 0x100 lane`): the DQ
/// receiver's reference, a 7-bit code (EIC7700X TRM Part 2, same PHY
/// family; reset 0x40).
pub const VREF_DAC: [u32; 2] = [0x04e, 0x04f];

/// R: master read-backs: ACSMWckEnControl/Status `0x2002a`..
/// WckFreqSwFastToggle `0x20040`, the free-running WCK extension `0x20041`,
/// then PState `0x2008b`.
pub const RB_MASTER: [u32; 25] = [
    0x2002a, 0x2002b, 0x2002c, 0x2002d, 0x2002e, 0x2002f, 0x20030, 0x20031, 0x20032, 0x20033,
    0x20034, 0x20035, 0x20036, 0x20037, 0x20038, 0x20039, 0x2003a, 0x2003b, 0x2003c, 0x2003d,
    0x2003e, 0x2003f, 0x20040, 0x20041, 0x2008b,
];
/// R: per DBYTE: DxOdtEn, OdtImpedanceSE0, DIFF0T, DIFF0C, DIFF1T, DIFF1C,
/// DxRxPowerDown.
pub const RB_DBYTE: [u32; 7] = [0x097, 0x048, 0x04a, 0x04b, 0x04c, 0x04d, 0x093];
/// PState select.
pub const PSTATE_SEL: u32 = 0x2008b;

/// R: one read-back of [`RB_MASTER`] and [`RB_DBYTE`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Readback {
    pub master: [u16; RB_MASTER.len()],
    pub dbyte: [[u16; RB_DBYTE.len()]; 2],
}

impl Readback {
    pub const EMPTY: Readback = Readback {
        master: [0; RB_MASTER.len()],
        dbyte: [[0; RB_DBYTE.len()]; 2],
    };
}

/// The all-ones RDC entry (MR33/MR34 0xff).
pub const ALL_ONES: EntryMrs = EntryMrs {
    mr20: 0x02,
    mr33: 0xff,
    mr34: 0xff,
    mr31: 0x00,
    mr32: 0x00,
};

/// RxDfe0FifoInfo of lanes 0-8 per DBYTE, before and after one launch.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Fifo {
    pub before: [[u16; LANES]; 2],
    pub after: [[u16; LANES]; 2],
}

impl Fifo {
    pub const EMPTY: Fifo = Fifo {
        before: [[0; LANES]; 2],
        after: [[0; LANES]; 2],
    };
}

/// QC: read latencies (HwtMRL and DFIMRL), DtsmLaneCtrl0 values, RxEnDly
/// (the vendor's 1200 MHz value, a measurement start) and receive clock
/// delays.
pub const MRL_VALUES: [u16; 1] = [4];
pub const LANE_CTL: [u16; 1] = [0x2801];
pub const QC_RX_EN: u16 = 0x369;
/// The vendor's ACSMRxEn/RxVal/Rdcs pulses and WCK read pulses (measurement
/// values, never defaults; on hardware they break our capture).
pub const VENDOR_PULSES: [(u32, u16); 7] = [
    (0x2002c, 0x082f),
    (0x2002d, 0x082f),
    (0x20030, 0x082f),
    (0x20039, 0x0c10),
    (0x2003a, 0x0c10),
    (0x2003b, 0x041c),
    (0x2003c, 0x2920),
];
/// PUB mode and the CSRs that could select the DBYTE RxEn/RxVal source:
/// PUBMODE, HwtControlVal, HwtControlOvr, HwtCtrl, HwtLpCsEnA/B/Bypass,
/// DbyteRxEnTrain, DfiMode, PMIEnable, DfiCtrlRxFifoRst, EnRxDqsTracking,
/// InhibitTxRdPtrInit, AllowInhibitTxRdPtrInit, DllControl, HwtMRL.
pub const PUB_CSRS: [u32; 16] = [
    0x2006e, 0x2007e, 0x2007a, 0x20077, 0x20072, 0x20073, 0x20074, 0x20059, 0x20051, 0x20054,
    0x20018, 0x20019, 0x2001b, 0x2001c, 0x20078, 0x2000d,
];
/// The reverse check: clock, gating and handover CSRs,
/// read only. UcclkHclkEnables, NeverGateDmaClk, MicroContMuxSel,
/// ContextToMicro, TDRDisable, NeverGateCsrClock, NeverGateAcCsrClock,
/// DfiCfgRdDataValidTicks, DisPieTrigCntr, SequencerOverride,
/// DfiInitCompleteShadow, MicroReset; PMIEnable, MemResetL, PUBMODE,
/// DfiInitComplete, ACSMNopAddr, AcsmStart/StopAddrXlatVal0, ForceClkDisable,
/// ForceClkGaterEnables, PclkGateControl, Dfi0Status, Dfi1Status,
/// PorControl, then NeverGateTrainCntrClk and the DBYTE DfiClk/PClk
/// disables of DBYTE 0-3.
pub const CLOCK_CSRS: [u32; 36] = [
    0xc0080, 0xc0008, 0xd0000, 0xd0001, 0xd0003, 0xd0035, 0xd0036, 0xd0037, 0xd0038, 0xd00e7,
    0xd00fa, 0xd0099, 0x20054, 0x20060, 0x2006e, 0x200f9, 0x2018a, 0x20124, 0x2018b, 0x200a1,
    0x200a6, 0x20200, 0x2005a, 0x2005b, 0x20090, 0x10008, 0x11008, 0x12008, 0x13008, 0x10003,
    0x11003, 0x12003, 0x13003, 0x10004, 0x11004, 0x12004,
];
/// Read-after-write records kept per block.
pub const RAW_MAX: usize = 64;

/// Per-pstate inputs of the RD stage.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RdConfig {
    /// The mode launches before a later block and after a block.
    pub next: ModeMrs,
    pub exit: ModeMrs,
    /// RxEnDly of the data-mode compares.
    pub qc_rx_en: u16,
    /// The latched search.
    pub search: Search,
}

/// Which RxEnDly register the search writes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TgSel {
    /// The block's rank's own (Tg0 on rank 0, Tg1 on rank 1).
    Own,
    Tg0,
    Tg1,
}

/// The latched search grid: RxEnDly `en_step * i` for i < `en_n`, receive
/// clock delay `clk_step * j` for j < `clk_n` (at most [`CLK_MAX`]), per
/// read latency of `mrl` (at most [`MRL_MAX`]).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Search {
    pub en_step: u16,
    pub en_n: u16,
    pub clk_step: u16,
    pub clk_n: u16,
    pub mrl: &'static [u16],
    pub tg: TgSel,
}

/// Receive clock points and read latencies per search, at most.
pub const CLK_MAX: usize = 128;
pub const MRL_MAX: usize = 2;

impl Search {
    /// Fine: RxEnDly 0..0x7e0 step 0x20 x RxClk 0..0x3f8 step 8, MRL 4/5.
    pub const FINE: Search = Search {
        en_step: 0x20,
        en_n: 64,
        clk_step: 8,
        clk_n: 128,
        mrl: &[4, 5],
        tg: TgSel::Own,
    };
    /// Coarse: RxEnDly 0..0x7c0 step 0x40 x RxClk 0..0x3f0 step 0x10, MRL 4.
    pub const COARSE: Search = Search {
        en_step: 0x40,
        en_n: 32,
        clk_step: 0x10,
        clk_n: 64,
        mrl: &[4],
        tg: TgSel::Own,
    };

    /// Launches of the search.
    pub const fn launches(&self) -> usize {
        self.mrl.len() * self.en_n as usize * self.clk_n as usize
    }
}

impl RdConfig {
    /// pstate 1 (the reference's values; RxEnDly the vendor's trained value).
    pub const PSTATE1: RdConfig = RdConfig {
        next: ModeMrs::NEXT_PSTATE1,
        exit: ModeMrs::EXIT_PSTATE1,
        qc_rx_en: QC_RX_EN,
        search: Search::COARSE,
    };
}

/// HwtMRL, then DFIMRL of DBYTE 0-3 and of AC 0 and 1 (`0x30000`,
/// `0x31000`).
pub const MRL_CSRS: [u32; 7] = [
    0x2000d, 0x10000, 0x11000, 0x12000, 0x13000, 0x30000, 0x31000,
];
/// The read latency the training sets: HwtMRL and the DBYTE DFIMRLs. The
/// AC DFIMRLs stay as initialised (6 in the vendor's trained state and in
/// ours).
pub const MRL_WRITE_CSRS: [u32; 5] = [0x2000d, 0x10000, 0x11000, 0x12000, 0x13000];
/// The other post-read-enable controls (channel selects and the two the
/// reference pulses).
pub const SEL_CSRS: [u32; 4] = [0x20072, 0x20073, 0x2007e, 0x2007a];

/// The HwtControlOvr pulse of the post-read-enable latency group for a
/// block on `ch`: the channel's override nibble with bit 0/4 clear (A
/// 0xe0, B 0x0e). The reference writes 0xe0 after a channel A select
/// (a14 launch 4321) and 0xe after a channel B block.
pub const fn latency_mask(ch: Channel) -> u16 {
    ch.override_mask() & 0xee
}

/// QC results.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Centring {
    pub mrl_before: [u16; MRL_CSRS.len()],
    pub mrl_after: [u16; MRL_CSRS.len()],
    pub sel_before: [u16; 4],
    pub sel_after: [u16; 4],
    /// RxFifoVisibility per DBYTE as found and after the restore.
    pub vis: [[u16; 2]; 2],
    pub fifo: Fifo,
    /// Read-after-write: (CSR, wanted, read back).
    pub raw: [(u32, u16, u16); RAW_MAX],
    pub n_raw: u8,
    /// [`PUB_CSRS`] at the first data-mode launch.
    pub pub_state: [u16; PUB_CSRS.len()],
}

impl Centring {
    pub const EMPTY: Centring = Centring {
        mrl_before: [0; MRL_CSRS.len()],
        mrl_after: [0; MRL_CSRS.len()],
        sel_before: [0; 4],
        sel_after: [0; 4],
        vis: [[0; 2]; 2],
        fifo: Fifo::EMPTY,
        raw: [(0, 0, 0); RAW_MAX],
        n_raw: 0,
        pub_state: [0; PUB_CSRS.len()],
    };

    fn push_raw(&mut self, csr: u32, want: u16, got: u16) {
        let n = self.n_raw as usize;
        if n < RAW_MAX {
            self.raw[n] = (csr, want, got);
            self.n_raw += 1;
        }
    }

    /// The read-after-write records whose read differs from the write.
    pub fn raw_mismatches(&self) -> impl Iterator<Item = &(u32, u16, u16)> {
        self.raw[..self.n_raw as usize]
            .iter()
            .filter(|r| r.1 != r.2)
    }

    fn clear(&mut self) {
        self.mrl_before = [0; MRL_CSRS.len()];
        self.mrl_after = [0; MRL_CSRS.len()];
        self.sel_before = [0; 4];
        self.sel_after = [0; 4];
        self.vis = [[0; 2]; 2];
        self.fifo = Fifo::EMPTY;
        self.raw = [(0, 0, 0); RAW_MAX];
        self.n_raw = 0;
        self.pub_state = [0; PUB_CSRS.len()];
    }
}

/// DbyteRxEnTrain (master; `0x2f059` broadcast).
pub const RXEN_TRAIN: u32 = 0x20059;

/// Read-path mode CSRs per DBYTE, read only (offsets in the DBYTE block):
/// DFIMRL, DbyteMiscMode, RxDigStrbEn, RxDfeModeCfg, DxRxStandbyEn,
/// DxDigStrobeMode, RxReplicaRangeVal, RxClkCntl, DxRxPowerDown,
/// DxDigStrobeGenSel.
pub const MODE_DBYTE: [u32; 10] = [
    0x000, 0x002, 0x005, 0x009, 0x00a, 0x00b, 0x014, 0x056, 0x093, 0x094,
];
/// Read-path mode CSRs of the master and AC blocks, read only:
/// DfiFreqRatio, PclkPtrInitVal, DqsPreambleControl, DMIPinPresent,
/// HwtMRL, DbyteRxDqsModeCntrl, TrackingModeCntrl, SingleEndedMode,
/// RxStandbyExtnd, then DFIMRL of AC 0 and AC 1.
pub const MODE_MASTER: [u32; 11] = [
    0x20000, 0x20002, 0x20007, 0x20008, 0x2000d, 0x20013, 0x20014, 0x20015, 0x20016, 0x30000,
    0x31000,
];

/// Starting values for the measurement, per DBYTE 0..3: receive clock
/// delays ([`RX_CLK`] order) and receiver references ([`VREF_DAC`] order),
/// lane 0..8.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ReadStart {
    pub rx_clk: [[[u16; LANES]; 4]; 4],
    pub vref_dac: [[[u16; LANES]; 2]; 4],
}

// ---------------------------------------------------------------------------
// The latched search (module documentation).

/// SelDbCurDlyTmngInfo (DBYTE offset); DbCurrentDlyTimingInfoTg0 is the
/// next CSR. Per lane k: selector 0x0a+3k RxClkT fine, 0x0b+3k RxClkC
/// fine, 0x0c+3k the live receive-enable delay.
pub const SEL_DLY: u32 = 0x08e;
/// The live readback selectors of lane 0: RxClkT, RxClkC, receive enable.
pub const LIVE_SEL: [u16; 3] = [0x0a, 0x0b, 0x0c];
/// Latch events: (written CSR, CSR read for the found value, bits set for
/// the pulse). PptRxEnEvnt (DBYTE broadcast, read through the block's first
/// DBYTE) latches RxEnDly; RxFifoInit resets the FIFO pointers;
/// ForceInternalUpdate latches the receive clock delays.
pub const EVENTS: [(u32, u32, u16); 3] = [
    (0x1f0a2, 0x100a2, 0x20),
    (0x2f0a0, 0x200a0, 0x01),
    (0x200a3, 0x200a3, 0x01),
];
pub(crate) const EV_RXEN: usize = 0;
pub(crate) const EV_FIFO: usize = 1;
pub(crate) const EV_UPDATE: usize = 2;
/// Hold time of every pulse.
pub const HOLD_US: u32 = 2;
/// RxFifoVisibility: RxFifoObsSelOvrd with ObsSel 0 (the DFE0 pointers).
pub const VIS_DFE0: u16 = 0x200;
/// The all-zero RDC entry (MR33/MR34 0).
pub const ALL_ZERO: EntryMrs = EntryMrs::READ_ENABLE_PSTATE1;
/// The vendor's trained point at 1200 MHz (av3c/av3g: RxEnDly
/// 0x357-0x36b, RxClk about 0x1b7-0x1c0), for reference in the output.
pub const VENDOR_RX_EN: u16 = 0x369;
pub const VENDOR_CLK: u16 = 0x1bd;
/// Per lane: passing points, RxEnDly min and max, receive clock delay min
/// and max.
pub type LaneEye = [u16; 5];
pub const LANE_EYE_EMPTY: LaneEye = [0, 0xffff, 0, 0xffff, 0];

/// Search results of one block (module documentation).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Eye {
    /// [`CLOCK_CSRS`] in the data stage.
    pub clocks: [u16; CLOCK_CSRS.len()],
    /// The live readback of lane 0 per DBYTE right after the exit.
    pub live_exit: [[u16; 3]; 2],
    /// Per read latency, DBYTE and lane.
    pub lanes: [[[LaneEye; LANES]; 2]; MRL_MAX],
    /// Per read latency and DBYTE: points where all nine lanes passed.
    pub full: [[u16; 2]; MRL_MAX],
    /// Rows whose live readback differs from what was written.
    pub live_bad: u16,
}

impl Eye {
    pub const EMPTY: Eye = Eye {
        clocks: [0; CLOCK_CSRS.len()],
        live_exit: [[0; 3]; 2],
        lanes: [[[LANE_EYE_EMPTY; LANES]; 2]; MRL_MAX],
        full: [[0; 2]; MRL_MAX],
        live_bad: 0,
    };
}

pub(crate) const fn lane_csr(db: u32, lane: u32, off: u32) -> u32 {
    0x10000 + 0x1000 * db + 0x100 * lane + off
}
pub(crate) const fn lane_f(db: u32, off: u32) -> u32 {
    0x10000 + 0x1000 * db + 0xf00 + off
}
const fn cntr(db: u32) -> u32 {
    0x100aa + 0x1000 * db
}
pub(crate) const fn param(db: u32) -> u32 {
    0x100a7 + 0x1000 * db
}
pub(crate) const fn rx_en(db: u32, rank: Rank) -> u32 {
    0x10020 + 0x1000 * db + matches!(rank, Rank::R1) as u32
}
pub(crate) const fn dbytes(ch: Channel) -> [u32; 2] {
    match ch {
        Channel::A => [0, 1],
        Channel::B => [2, 3],
    }
}

/// DTSM state of one DBYTE at one point.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Obs {
    pub cmp: [u16; LANES],
    pub err: [u16; LANES],
    pub good: [u16; LANES],
    /// TrainingCntr lane 0.
    pub cntr: u16,
    /// DtsmErrCountNZ (`0x1n9b7`).
    pub err_nz: u16,
    /// DtsmGoodThldXingInd, DtsmErrThldXingInd (`0x1n0bc`, `0x1n0bd`).
    pub xing: [u16; 2],
}

impl Obs {
    pub const EMPTY: Obs = Obs {
        cmp: [0; LANES],
        err: [0; LANES],
        good: [0; LANES],
        cntr: 0,
        err_nz: 0,
        xing: [0; 2],
    };
}

/// One point's observation pair, empty.
pub const EMPTY_POINT: [Obs; 2] = [Obs::EMPTY; 2];

fn observe<H: Hw>(ex: &mut WlExec<'_, H>, db: u32) -> Obs {
    let mut o = Obs::EMPTY;
    for lane in 0..LANES {
        let l = lane as u32;
        o.cmp[lane] = ex.read(lane_csr(db, l, 0x0b6));
        o.err[lane] = ex.read(lane_csr(db, l, 0x0b7));
        o.good[lane] = ex.read(lane_csr(db, l, 0x0b8));
    }
    o.cntr = ex.read(cntr(db));
    o.err_nz = ex.read(0x109b7 + 0x1000 * db);
    o.xing = [
        ex.read(0x100bc + 0x1000 * db),
        ex.read(0x100bd + 0x1000 * db),
    ];
    o
}

/// Controls the data control changes that no later reference step rewrites,
/// read before it and written back after it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DataSaved {
    /// `0x70011`, `0x700d6`, `0x700d7` (bases of the `0x7f0..` broadcasts).
    pub misc: [u16; 3],
    /// `0x1n001` per DBYTE (the `0x1f001` broadcast).
    pub dbyte_001: [u16; 4],
}

impl DataSaved {
    pub const EMPTY: DataSaved = DataSaved {
        misc: [0; 3],
        dbyte_001: [0; 4],
    };
}

/// One channel/rank block.
pub struct RdBlock {
    pub channel: Channel,
    pub rank: Rank,
    pub saved_rx_en: [u16; 2],
    /// RxEnDly Tg0 as found (the search writes Tg0 on every block).
    pub saved_tg0: [u16; 2],
    pub final_rx_en: [u16; 2],
    pub saved_param: [u16; 4],
    pub final_param: [u16; 4],
    pub read_path: ReadPath,
    /// R at RD start and with PState flipped.
    pub rb: [Readback; 2],
    /// RxClk delays of the channel's DBYTEs as found ([`RX_CLK`] order),
    /// and as set for the measurement (equal without a [`ReadStart`]).
    pub rx_clk_before: [[[u16; LANES]; 4]; 2],
    pub rx_clk_used: [[[u16; LANES]; 4]; 2],
    /// After the restore.
    pub rx_clk_after: [[[u16; LANES]; 4]; 2],
    /// VrefDAC0/1 of the channel's DBYTEs as found, as set, after the
    /// restore.
    pub vref_before: [[[u16; LANES]; 2]; 2],
    pub vref_used: [[[u16; LANES]; 2]; 2],
    pub vref_after: [[[u16; LANES]; 2]; 2],
    /// QC set-up records.
    pub qc: Centring,
    /// The search's eye totals.
    pub u: Eye,
    /// DbyteRxEnTrain before and after the exit.
    pub rxen_train: [u16; 2],
    pub data_saved: DataSaved,
    /// The same controls read back after the data control's restore.
    pub data_restored: DataSaved,
    pub launches: u16,
    pub writes: u16,
    pub max_launch_us: u32,
    /// AcPhase1UI, DxPhase1UI (DBYTE 0), PState select after each
    /// step of the block ([`DLL_STEPS`]).
    pub dll: [[u16; 3]; DLL_STEPS.len()],
    /// The block's start (`Hw::now_us`) and its duration.
    pub start_us: u64,
    pub elapsed_us: u32,
    pub failure: Option<WlFailure>,
}

impl RdBlock {
    pub const fn empty(channel: Channel, rank: Rank) -> RdBlock {
        RdBlock {
            channel,
            rank,
            saved_rx_en: [0; 2],
            saved_tg0: [0; 2],
            final_rx_en: [0; 2],
            saved_param: [0; 4],
            final_param: [0; 4],
            read_path: ReadPath::EMPTY,
            rb: [Readback::EMPTY; 2],
            rx_clk_before: [[[0; LANES]; 4]; 2],
            rx_clk_used: [[[0; LANES]; 4]; 2],
            rx_clk_after: [[[0; LANES]; 4]; 2],
            vref_before: [[[0; LANES]; 2]; 2],
            vref_used: [[[0; LANES]; 2]; 2],
            vref_after: [[[0; LANES]; 2]; 2],
            qc: Centring::EMPTY,
            u: Eye::EMPTY,
            rxen_train: [0; 2],
            data_saved: DataSaved::EMPTY,
            data_restored: DataSaved::EMPTY,
            launches: 0,
            writes: 0,
            max_launch_us: 0,
            dll: [[0; 3]; DLL_STEPS.len()],
            start_us: 0,
            elapsed_us: 0,
            failure: None,
        }
    }

    /// Back to [`RdBlock::empty`] for `channel`/`rank`, in place: the block
    /// is larger than bt0 can hold as a stack temporary.
    pub fn reset(&mut self, channel: Channel, rank: Rank) {
        self.channel = channel;
        self.rank = rank;
        self.saved_rx_en = [0; 2];
        self.saved_tg0 = [0; 2];
        self.final_rx_en = [0; 2];
        self.saved_param = [0; 4];
        self.final_param = [0; 4];
        self.read_path = ReadPath::EMPTY;
        self.rb = [Readback::EMPTY; 2];
        self.rx_clk_before = [[[0; LANES]; 4]; 2];
        self.rx_clk_used = [[[0; LANES]; 4]; 2];
        self.rx_clk_after = [[[0; LANES]; 4]; 2];
        self.vref_before = [[[0; LANES]; 2]; 2];
        self.vref_used = [[[0; LANES]; 2]; 2];
        self.vref_after = [[[0; LANES]; 2]; 2];
        self.qc.clear();
        self.u = Eye::EMPTY;
        self.rxen_train = [0; 2];
        self.data_saved = DataSaved::EMPTY;
        self.data_restored = DataSaved::EMPTY;
        self.launches = 0;
        self.writes = 0;
        self.max_launch_us = 0;
        self.dll = [[0; 3]; DLL_STEPS.len()];
        self.start_us = 0;
        self.elapsed_us = 0;
        self.failure = None;
    }
}

/// The steps after which the DLL 1-UI codes are read.
pub const DLL_STEPS: [&str; 10] = [
    "start",
    "set-up",
    "next mode",
    "warm-up",
    "R readback",
    "exit",
    "exit mode",
    "data set-up",
    "search",
    "restore",
];

fn dll_codes<H: Hw>(ex: &mut WlExec<'_, H>) -> [u16; 3] {
    [ex.read(0x300be), ex.read(0x100df), ex.read(PSTATE_SEL)]
}

/// Launches per block with `search`: the next-rank mode launch (not on the
/// first block), entry and warm-up 2, the exit mode launch after the exit,
/// the read-centring entry, the search,
/// the all-zero entry and the exit mode launch.
pub const fn block_launches(search: &Search) -> usize {
    1 + 1 + 2 + 1 + 1 + search.launches() + 1 + 1
}

/// Read-path settings read once per block before the measurements (no
/// change): the PHY CSRs the vendor's read-data-enable formula uses
/// (`phy::read_latency_csrs`: `0x20002`, DBYTE 0's TxWckDly), the
/// channel's DTSM byte controls, and the controller's DRAMSET1TMG2 (read
/// latency field) and DFI read-data enable register (`+0x588`) of
/// frequency sets 0 and 1 (the probe trains at pstate 1); the mode CSRs
/// [`MODE_DBYTE`] of the channel's DBYTEs and [`MODE_MASTER`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ReadPath {
    pub ptr_init: u16,
    pub tx_wck_db0: u16,
    pub byte_ctrl: [[u16; 2]; 2],
    pub tmg2: [u32; 2],
    pub dfi_588: [u32; 2],
    pub dbyte_mode: [[u16; MODE_DBYTE.len()]; 2],
    pub master_mode: [u16; MODE_MASTER.len()],
}

impl ReadPath {
    pub const EMPTY: ReadPath = ReadPath {
        ptr_init: 0,
        tx_wck_db0: 0,
        byte_ctrl: [[0; 2]; 2],
        tmg2: [0; 2],
        dfi_588: [0; 2],
        dbyte_mode: [[0; MODE_DBYTE.len()]; 2],
        master_mode: [0; MODE_MASTER.len()],
    };
}

/// The read-path settings ([`ReadPath`]) of channel `ch`, read only.
pub(crate) fn read_path<H: Hw>(ex: &mut WlExec<'_, H>, ch: Channel) -> ReadPath {
    let dbs = dbytes(ch);
    let mut rp = ReadPath::EMPTY;
    let (ptr_init, tx_wck) = crate::phy::read_latency_csrs(ex.hw);
    rp.ptr_init = ptr_init as u16;
    rp.tx_wck_db0 = tx_wck as u16;
    for (i, &db) in dbs.iter().enumerate() {
        rp.byte_ctrl[i] = [
            ex.read(0x100b0 + 0x1000 * db),
            ex.read(0x100b1 + 0x1000 * db),
        ];
    }
    let c = match ch {
        Channel::A => 0,
        Channel::B => 1,
    };
    for ps in 0..2 {
        rp.tmg2[ps] = ex.hw.read32(crate::ctrl::freq(c, ps, 0x08));
        rp.dfi_588[ps] = ex.hw.read32(crate::ctrl::freq(c, ps, 0x588));
    }
    for (i, &db) in dbs.iter().enumerate() {
        for (v, &off) in rp.dbyte_mode[i].iter_mut().zip(MODE_DBYTE.iter()) {
            *v = ex.read(0x10000 + 0x1000 * db + off);
        }
    }
    for (v, &c) in rp.master_mode.iter_mut().zip(MODE_MASTER.iter()) {
        *v = ex.read(c);
    }
    rp
}

/// The expected-data and comparison set-up the reference writes before read
/// centring (a14, launch 4322), without its delay writes: expected-data
/// words (`0x70f24..0x70f2b` = `expected`; the reference: [`PATTERN`]),
/// masks, the pattern mode, the DTSM gates and byte controls for data
/// compares, and DtsmLaneCtrl0 of every lane of the channel's DBYTEs
/// (lane-F addresses, value 1).
pub(crate) fn pattern_on(dbs: [u32; 2], expected: u16, out: &mut impl FnMut(Step<'_>)) {
    out(Step::Write(0x7f065, 0));
    for c in 0x70061..=0x70064 {
        out(Step::Write(c, 0xffff));
    }
    for c in 0x70f24..=0x70f2b {
        out(Step::Write(c, expected));
    }
    for (c, v) in [
        (0x70065, 0x1ff),
        (0x7f011, 0x10),
        (0x7f0d7, 0),
        (0x7f0d6, 0),
        (0x7f060, 0x85),
        (0x1f0b3, 1),
        (0x1f0b4, 1),
        (0x1f0b0, 0x34),
        (0x1f0b1, 1),
        (0x1f001, 0),
    ] {
        out(Step::Write(c, v));
    }
    out(Step::Write(0x2f0a0, 1));
    out(Step::Write(0x2f0a0, 0));
    for db in dbs {
        out(Step::Write(0x10fb5 + db * 0x1000, 1));
    }
}

/// Undo [`pattern_on`]: the values `rxen::block_setup` and `gates_on` write
/// for read enable (pattern words and masks 0, pattern mode 1, `0x7f065`
/// `0x1ff`, gates and byte controls), the saved `0x7f011`/`0x7f0d6`/
/// `0x7f0d7`/`0x1n001` values, lane enables back to lane 0 only.
pub(crate) fn pattern_off(dbs: [u32; 2], saved: &DataSaved, out: &mut impl FnMut(Step<'_>)) {
    for c in 0x70061..=0x70064 {
        out(Step::Write(c, 0));
    }
    for c in 0x70f24..=0x70f2b {
        out(Step::Write(c, 0));
    }
    for (c, v) in [
        (0x70065, 0),
        (0x7f060, 1),
        (0x7f065, 0x1ff),
        (0x7f011, saved.misc[0]),
        (0x7f0d6, saved.misc[1]),
        (0x7f0d7, saved.misc[2]),
    ] {
        out(Step::Write(c, v));
    }
    for (db, &v) in saved.dbyte_001.iter().enumerate() {
        out(Step::Write(0x10001 + 0x1000 * db as u32, v));
    }
    for db in dbs {
        out(Step::Write(0x10fb5 + db * 0x1000, 0));
    }
    out(Step::Write(0x1f0b5, 1));
    rxen::gates_on(out);
    out(Step::Write(0x2f0a0, 1));
    out(Step::Write(0x2f0a0, 0));
}

pub(crate) fn read_data_saved<H: Hw>(ex: &mut WlExec<'_, H>) -> DataSaved {
    let mut s = DataSaved::EMPTY;
    for (v, c) in s.misc.iter_mut().zip([0x70011u32, 0x700d6, 0x700d7]) {
        *v = ex.read(c);
    }
    for (db, v) in s.dbyte_001.iter_mut().enumerate() {
        *v = ex.read(0x10001 + 0x1000 * db as u32);
    }
    s
}

/// R: [`RB_MASTER`] and [`RB_DBYTE`] of `dbs`, read only.
fn readback<H: Hw>(ex: &mut WlExec<'_, H>, dbs: [u32; 2]) -> Readback {
    let mut r = Readback::EMPTY;
    for (v, &c) in r.master.iter_mut().zip(RB_MASTER.iter()) {
        *v = ex.read(c);
    }
    for (i, &db) in dbs.iter().enumerate() {
        for (v, &off) in r.dbyte[i].iter_mut().zip(RB_DBYTE.iter()) {
            *v = ex.read(0x10000 + 0x1000 * db + off);
        }
    }
    r
}

pub(crate) const fn fifo_info(db: u32, lane: u32) -> u32 {
    0x10070 + 0x1000 * db + 0x100 * lane
}
pub(crate) const fn fifo_vis(db: u32) -> u32 {
    0x10072 + 0x1000 * db
}

/// Write `v` to `c` and read it back into `qc`'s read-after-write records.
fn raw_write<H: Hw>(ex: &mut WlExec<'_, H>, qc: &mut Centring, c: u32, v: u16) {
    ex.write(c, v);
    let got = ex.read(c);
    qc.push_raw(c, v, got);
}

/// One launch of `rows` with outer `outer` and the DTSM state of both
/// DBYTEs after it (counts cleared first).
pub(crate) fn measure_prog<H: Hw>(
    ex: &mut WlExec<'_, H>,
    dbs: [u32; 2],
    rows: &[Row],
    outer: u16,
    o: &mut [Obs; 2],
) {
    for &db in &dbs {
        wcklevel::clear_counts_dbyte(db, &mut |st| ex.step(st));
    }
    wcklevel::clear_counts(&mut |st| ex.step(st));
    ex.step(Step::Launch { rows, outer });
    if ex.failure.is_some() {
        return;
    }
    for (i, &db) in dbs.iter().enumerate() {
        o[i] = observe(ex, db);
    }
}

/// Fire update event `k` of [`EVENTS`]: its bits set on the value found,
/// held [`HOLD_US`], the found value again, held. Returns the CSR as found
/// and while set.
pub(crate) fn fire<H: Hw>(ex: &mut WlExec<'_, H>, dbs: [u32; 2], k: usize) -> [u16; 2] {
    let (w, r, bits) = EVENTS[k];
    let r = if r >> 12 == 0x10 {
        r + 0x1000 * dbs[0]
    } else {
        r
    };
    let found = ex.read(r);
    ex.write(w, found | bits);
    let during = ex.read(r);
    ex.hw.udelay(HOLD_US);
    ex.write(w, found);
    ex.hw.udelay(HOLD_US);
    [found, during]
}

/// RxDfe0FifoInfo lane 0 of both DBYTEs (visibility [`VIS_DFE0`] set).
fn fifo_lane0<H: Hw>(ex: &mut WlExec<'_, H>, dbs: [u32; 2]) -> [u16; 2] {
    [ex.read(fifo_info(dbs[0], 0)), ex.read(fifo_info(dbs[1], 0))]
}

/// The receive clock delays (lane F, all four registers) of both DBYTEs.
fn set_clk<H: Hw>(ex: &mut WlExec<'_, H>, dbs: [u32; 2], clk: u16) {
    for &db in &dbs {
        for off in RX_CLK {
            ex.write(lane_f(db, off), clk);
        }
    }
}

/// The post-read-enable latency group as QC writes it, with read
/// latency `mrl` and the channel's HwtControlOvr pulse; every write read
/// back into `qc` when given.
pub(crate) fn latency_group<H: Hw>(
    ex: &mut WlExec<'_, H>,
    mut qc: Option<&mut Centring>,
    ch: Channel,
    mrl: u16,
) {
    let mut w = |ex: &mut WlExec<'_, H>, c: u32, v: u16| match qc.as_deref_mut() {
        Some(q) => raw_write(ex, q, c, v),
        None => ex.write(c, v),
    };
    for (c, v) in [
        (0x20072u32, 3u16),
        (0x20073, 3),
        (0x2007e, 0),
        (0x2007a, latency_mask(ch)),
        (0x2007a, 0),
    ] {
        w(ex, c, v);
    }
    for &c in MRL_WRITE_CSRS.iter() {
        w(ex, c, mrl);
    }
    ex.write(0x2f0a0, 1);
    ex.write(0x2f0a0, 0);
}

/// The live readback of lane 0 of `db`: RxClkT, RxClkC, receive enable;
/// the selector put back.
pub(crate) fn live_lane0<H: Hw>(ex: &mut WlExec<'_, H>, db: u32) -> [u16; 3] {
    let sel = 0x10000 + 0x1000 * db + SEL_DLY;
    let found = ex.read(sel);
    let mut v = [0; 3];
    for (x, &s) in v.iter_mut().zip(LIVE_SEL.iter()) {
        ex.write(sel, s);
        *x = ex.read(sel + 1);
    }
    ex.write(sel, found);
    v
}

/// RxEnDly `en` on both DBYTEs, latched (PptRxEnEvnt).
fn set_rx_en<H: Hw>(ex: &mut WlExec<'_, H>, dbs: [u32; 2], rank: Rank, en: u16) {
    for &db in &dbs {
        ex.write(rx_en(db, rank), en);
    }
    fire(ex, dbs, EV_RXEN);
}

/// The receive clock delays (lane F, all four) `clk` on both DBYTEs,
/// latched (ForceInternalUpdate).
fn set_clk_latched<H: Hw>(ex: &mut WlExec<'_, H>, dbs: [u32; 2], clk: u16) {
    set_clk(ex, dbs, clk);
    fire(ex, dbs, EV_UPDATE);
}

/// Measure the read data path for `out.channel`/`out.rank` (module
/// documentation). Call after WL, CA training and the lane-map restore.
/// `prev` is the previous block's rank (None for the first block): a
/// different rank adds the reference's [`rxen::rank_switch`], and every
/// block but the first opens with the reference's next-rank mode launch.
#[inline(never)]
pub fn measure_block<H: Hw>(
    hw: &mut H,
    out: &mut RdBlock,
    prev: Option<Rank>,
    start: Option<&ReadStart>,
    cfg: &RdConfig,
) {
    let (ch, rank) = (out.channel, out.rank);
    out.reset(ch, rank);
    let dbs = dbytes(ch);
    let t0 = hw.now_us();
    out.start_us = t0;
    let mut ex = WlExec::new(hw);
    out.dll[0] = dll_codes(&mut ex);

    for (i, &db) in dbs.iter().enumerate() {
        out.saved_rx_en[i] = ex.read(rx_en(db, rank));
        out.saved_tg0[i] = ex.read(rx_en(db, Rank::R0));
    }
    for db in 0..4u32 {
        out.saved_param[db as usize] = ex.read(param(db));
    }
    out.data_saved = read_data_saved(&mut ex);

    // Reference set-up, entry and warm-up.
    if prev.is_some_and(|r| r != rank) {
        rxen::rank_switch(&mut |s| ex.step(s));
    }
    rxen::block_setup(ch, &mut |s| ex.step(s));
    out.dll[1] = dll_codes(&mut ex);
    if prev.is_some() {
        ex.step(Step::Launch {
            rows: &rxen::mode_rows(rank, cfg.next),
            outer: 0,
        });
    }
    out.dll[2] = dll_codes(&mut ex);
    ex.step(Step::Launch {
        rows: &rxen::entry(rank, EntryMrs::READ_ENABLE_PSTATE1),
        outer: 0,
    });
    rxen::bars(&mut |s| ex.step(s));
    ex.step(Step::Launch {
        rows: &rxen::read_program_32(rank),
        outer: 96,
    });
    rxen::counter(ch, 0x20, &mut |s| ex.step(s));
    ex.step(Step::Launch {
        rows: &rxen::read_program_32(rank),
        outer: 0,
    });
    rxen::gates_on(&mut |s| ex.step(s));
    out.dll[3] = dll_codes(&mut ex);

    // The read-path settings, read only.
    out.read_path = read_path(&mut ex, ch);

    // R at RD start. No PState flip (with it the gate jumped between
    // blocks).
    out.rb[0] = readback(&mut ex, dbs);
    out.rb[1] = out.rb[0];
    out.dll[4] = dll_codes(&mut ex);

    // The receive clock delays as found (not set from the start), the
    // references from the start.
    let rx_clk = |db: u32, k: usize, lane: usize| lane_csr(db, lane as u32, RX_CLK[k]);
    for (i, &db) in dbs.iter().enumerate() {
        for k in 0..4 {
            for lane in 0..LANES {
                out.rx_clk_before[i][k][lane] = ex.read(rx_clk(db, k, lane));
                out.rx_clk_used[i][k][lane] = out.rx_clk_before[i][k][lane];
            }
        }
        for (k, &off) in VREF_DAC.iter().enumerate() {
            for lane in 0..LANES {
                let a = lane_csr(db, lane as u32, off);
                out.vref_before[i][k][lane] = ex.read(a);
                if let Some(st) = start {
                    ex.write(a, st.vref_dac[db as usize][k][lane]);
                }
                out.vref_used[i][k][lane] = ex.read(a);
            }
        }
    }

    // The reference exit with the saved RxEnDly, then data mode.
    out.rxen_train[0] = ex.read(RXEN_TRAIN);
    rxen::apply(ch, rank, out.saved_rx_en, &mut |s| ex.step(s));
    out.dll[5] = dll_codes(&mut ex);
    // The exit mode launch right after the exit, as the reference orders
    // it (when it came only at the block's end, every block but the first
    // ran its search with the next-rank MR18/MR10).
    ex.step(Step::Launch {
        rows: &rxen::mode_rows(rank, cfg.exit),
        outer: 0,
    });
    out.rxen_train[1] = ex.read(RXEN_TRAIN);
    out.dll[6] = dll_codes(&mut ex);
    for (i, &db) in dbs.iter().enumerate() {
        out.u.live_exit[i] = live_lane0(&mut ex, db);
    }
    // QC set-up (the latency pulse per channel), every write
    // read back.
    for (k, &c) in MRL_CSRS.iter().enumerate() {
        out.qc.mrl_before[k] = ex.read(c);
    }
    for (k, &c) in SEL_CSRS.iter().enumerate() {
        out.qc.sel_before[k] = ex.read(c);
    }
    for (i, &db) in dbs.iter().enumerate() {
        out.qc.vis[i][0] = ex.read(fifo_vis(db));
    }
    ex.step(Step::Launch {
        rows: &rxen::entry(rank, EntryMrs::READ_CENTRING_PSTATE1),
        outer: 0,
    });
    pattern_on(dbs, PATTERN, &mut |s| ex.step(s));
    let centring = rxen::centring_program(rank);
    latency_group(&mut ex, Some(&mut out.qc), ch, MRL_VALUES[0]);
    for &db in &dbs {
        ex.write(lane_f(db, 0xb5), LANE_CTL[0]);
        // Lane-F broadcasts read back through lane 0.
        let got = ex.read(0x10000 + 0x1000 * db + 0xb5);
        out.qc
            .push_raw(0x10000 + 0x1000 * db + 0xb5, LANE_CTL[0], got);
        raw_write(&mut ex, &mut out.qc, fifo_vis(db), VIS_DFE0);
    }
    for (v, &c) in out.qc.pub_state.iter_mut().zip(PUB_CSRS.iter()) {
        *v = ex.read(c);
    }
    out.dll[7] = dll_codes(&mut ex);
    for (v, &c) in out.u.clocks.iter_mut().zip(CLOCK_CSRS.iter()) {
        *v = ex.read(c);
    }

    // U: the fine search, every delay latched, RxFifoInit before every
    // launch; RxEnDly on Tg0 (module documentation). Rows are logged as
    // they complete.
    let label = [
        match ch {
            Channel::A => b'A',
            Channel::B => b'B',
        },
        match rank {
            Rank::R0 => b'0',
            Rank::R1 => b'1',
        },
        b't',
        match cfg.search.tg {
            TgSel::Tg1 => b'1',
            _ => b'0',
        },
    ];
    let n = if cfg.search.tg == TgSel::Own { 2 } else { 4 };
    let name = core::str::from_utf8(&label[..n]).unwrap_or("??");
    let sr = cfg.search;
    let tg = match (sr.tg, rank) {
        (TgSel::Tg0, _) | (TgSel::Own, Rank::R0) => Rank::R0,
        _ => Rank::R1,
    };
    let clk_n = (sr.clk_n as usize).min(CLK_MAX);
    for (mi, &mrl) in sr.mrl.iter().take(MRL_MAX).enumerate() {
        latency_group(&mut ex, None, ch, mrl);
        for en in (0..sr.en_n).map(|i| i * sr.en_step) {
            if ex.failure.is_some() {
                break;
            }
            set_rx_en(&mut ex, dbs, tg, en);
            // Per DBYTE: the passing-lane count per point ('.' none, 'w'
            // none but lane 0's FIFO write pointer moved), and per lane the
            // first and last passing receive clock delay and the count.
            let mut map = [[b'.'; CLK_MAX]; 2];
            let mut edges = [[(0xffffu16, 0u16, 0u16); LANES]; 2];
            for (ci, clk) in (0..clk_n).map(|c| (c, c as u16 * sr.clk_step)) {
                if ex.failure.is_some() {
                    break;
                }
                set_clk_latched(&mut ex, dbs, clk);
                if ci == 0 {
                    let mut bad = false;
                    for &db in &dbs {
                        let v = live_lane0(&mut ex, db);
                        bad |= v[0] != clk & 0x7f || v[1] != clk & 0x7f;
                        bad |= tg == Rank::R0 && v[2] != en;
                    }
                    out.u.live_bad += bad as u16;
                }
                fire(&mut ex, dbs, EV_FIFO);
                let f0 = fifo_lane0(&mut ex, dbs);
                let mut o = EMPTY_POINT;
                measure_prog(&mut ex, dbs, &centring, rxen::CENTRING_OUTER, &mut o);
                let f1 = fifo_lane0(&mut ex, dbs);
                for (i, oi) in o.iter().enumerate() {
                    let mut n = 0u8;
                    for (lane, &e) in oi.err.iter().enumerate() {
                        if oi.cmp[lane] == 0 || e != 0 {
                            continue;
                        }
                        n += 1;
                        let w = &mut edges[i][lane];
                        w.0 = w.0.min(clk);
                        w.1 = w.1.max(clk);
                        w.2 += 1;
                        let t = &mut out.u.lanes[mi][i][lane];
                        t[0] += 1;
                        t[1] = t[1].min(en);
                        t[2] = t[2].max(en);
                        t[3] = t[3].min(clk);
                        t[4] = t[4].max(clk);
                    }
                    if n as usize == LANES {
                        out.u.full[mi][i] += 1;
                    }
                    map[i][ci] = match n {
                        0 if f0[i] & 0x3ff != f1[i] & 0x3ff => b'w',
                        0 => b'.',
                        n => b"0123456789"[n as usize],
                    };
                }
            }
            let m0 = core::str::from_utf8(&map[0][..clk_n]).unwrap_or("?");
            let m1 = core::str::from_utf8(&map[1][..clk_n]).unwrap_or("?");
            ex.hw.log(format_args!(
                "[dram] RD {name} U m{mrl} d {en:03x}: {m0} | {m1}\n"
            ));
            for (i, e) in edges.iter().enumerate() {
                if e.iter().any(|x| x.2 > 0) {
                    ex.hw.log(format_args!(
                        "[dram] RD {name} U m{mrl} d {en:03x} b{i} lanes (first, last, points): {e:x?}\n"
                    ));
                }
            }
        }
    }

    out.dll[8] = dll_codes(&mut ex);
    // Back: the receive clock delays as found (latched), latency, selects,
    // visibility, the all-zero entry, the pattern set-up, RxEnDly
    // (latched), the references.
    for (i, &db) in dbs.iter().enumerate() {
        for k in 0..4 {
            for lane in 0..LANES {
                ex.write(rx_clk(db, k, lane), out.rx_clk_before[i][k][lane]);
                out.rx_clk_after[i][k][lane] = ex.read(rx_clk(db, k, lane));
            }
        }
    }
    fire(&mut ex, dbs, EV_UPDATE);
    for (k, &c) in MRL_CSRS.iter().enumerate() {
        ex.write(c, out.qc.mrl_before[k]);
        out.qc.mrl_after[k] = ex.read(c);
    }
    for (k, &c) in SEL_CSRS.iter().enumerate() {
        ex.write(c, out.qc.sel_before[k]);
        out.qc.sel_after[k] = ex.read(c);
    }
    for (i, &db) in dbs.iter().enumerate() {
        ex.write(fifo_vis(db), out.qc.vis[i][0]);
        out.qc.vis[i][1] = ex.read(fifo_vis(db));
    }
    ex.step(Step::Launch {
        rows: &rxen::entry(rank, ALL_ZERO),
        outer: 0,
    });
    pattern_off(dbs, &out.data_saved, &mut |s| ex.step(s));
    out.data_restored = read_data_saved(&mut ex);
    for (i, &db) in dbs.iter().enumerate() {
        ex.write(rx_en(db, Rank::R0), out.saved_tg0[i]);
        ex.write(rx_en(db, rank), out.saved_rx_en[i]);
    }
    fire(&mut ex, dbs, EV_RXEN);
    fire(&mut ex, dbs, EV_FIFO);
    for (i, &db) in dbs.iter().enumerate() {
        for (k, &off) in VREF_DAC.iter().enumerate() {
            for lane in 0..LANES {
                let a = lane_csr(db, lane as u32, off);
                ex.write(a, out.vref_before[i][k][lane]);
                out.vref_after[i][k][lane] = ex.read(a);
            }
        }
    }

    ex.step(Step::Launch {
        rows: &rxen::mode_rows(rank, cfg.exit),
        outer: 0,
    });
    for db in 0..4u32 {
        ex.write(param(db), out.saved_param[db as usize]);
    }
    for (i, &db) in dbs.iter().enumerate() {
        out.final_rx_en[i] = ex.read(rx_en(db, rank));
    }
    for db in 0..4u32 {
        out.final_param[db as usize] = ex.read(param(db));
    }

    out.dll[9] = dll_codes(&mut ex);
    out.launches = ex.launches;
    out.writes = ex.writes;
    out.max_launch_us = ex.max_launch_us;
    out.failure = ex.failure;
    csr_access(hw, false);
    out.elapsed_us = hw.now_us().wrapping_sub(t0) as u32;
}

/// AcDllLockParam (`0x2000a`) and DxDllLockParam (`0x20017`) of the
/// selected PState set to the AC and DBYTE DLL 1-UI codes measured now
/// (AcPhase1UI `0x300be`, DxPhase1UI of DBYTE 0 `0x100df`), as the vendor's
/// trained state holds them (with 0 the codes walk). Returns the
/// PState select, the parameters as found, the codes and the parameters
/// read back.
pub fn lock_dlls<H: Hw>(hw: &mut H) -> [u16; 7] {
    csr_access(hw, true);
    let r = |hw: &mut H, c: u32| hw.read16(crate::phy::csr(c));
    let ps = r(hw, PSTATE_SEL);
    let found = [r(hw, 0x2000a), r(hw, 0x20017)];
    let codes = [r(hw, 0x300be), r(hw, 0x100df)];
    hw.write16(crate::phy::csr(0x2000a), codes[0]);
    hw.write16(crate::phy::csr(0x20017), codes[1]);
    let after = [r(hw, 0x2000a), r(hw, 0x20017)];
    csr_access(hw, false);
    [
        ps, found[0], found[1], codes[0], codes[1], after[0], after[1],
    ]
}

/// Every PHY CSR the TRM lists ([`crate::phy::csrlist::TRM_CSRS`]), read
/// only (the state at each block start).
pub fn snapshot<H: Hw>(hw: &mut H, out: &mut [u16; crate::phy::csrlist::TRM_CSRS.len()]) {
    csr_access(hw, true);
    for (v, &c) in out.iter_mut().zip(crate::phy::csrlist::TRM_CSRS.iter()) {
        *v = hw.read16(crate::phy::csr(c));
    }
    csr_access(hw, false);
}
