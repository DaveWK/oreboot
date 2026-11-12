// SPDX-License-Identifier: GPL-2.0-only
//! The hand-over of the trained PHY to the DDR controller without a PIE
//! (a measurement image). Runs after the read
//! training is applied at pstate 1, with the DRAM initialised and idle, the
//! PHY owning the DFI (PUBMODE 1), the CSR mux on the APB and the
//! controller in software self-refresh with its DFI initialisation not
//! started (`ctrl::com_init`).
//!
//! The flow with a PIE: after training the PIE is loaded and the mission
//! CSRs are written, then
//! the controller's DFI initialisation completes with the PHY answering by
//! itself; the PIE (reference only, never loaded here) or hardware produces
//! dfi_init_complete. So this image is adaptive, in one boot:
//!
//! - **Stage 0** reads the state (no writes).
//! - **Stage 1, the probe** (no DRAM traffic, no dfi_init_start): writes
//!   DfiInitComplete `0x200f9` = 1, then its shadow `0xd00fa` = 1, then
//!   PUBMODE 0, each put back, and reads what the controllers' DFISTAT bit
//!   0 does. `cpu_drives` names the write DFISTAT followed.
//! - **Stage 2, the hand-over**, only with a positive probe: self-refresh
//!   entry through our own ACSM row (both ranks; the firmware's SRE row
//!   shape), the mission CSRs ([`MISSION`]), PUBMODE 0, the controller's read-data
//!   enable for pstate 1 computed from our PHY CSRs, DFIMISC[12:8] = 0x10 |
//!   pstate, then the DFI initialisation with our answer inserted after
//!   dfi_init_start (every wait bounded), the ports after the PHY, the CSR
//!   mux to the PHY as the vendor leaves it, mode-register reads through
//!   the controller (MR5, MR8, MR4 per rank and channel: the first
//!   controller-path reads, no AXI traffic), then the shared quick memtest
//!   at the DRAM base and its 1 GiB middle.
//!
//! Everything is logged; the caller halts afterwards and the boot
//! watchdog resets into the vendor recovery.
use crate::ctrl::{self, ddrc, freq, sw_done};
use crate::hw::{wait, Error, Hw};
use crate::para::Para;
use crate::phy::coldinit::Row;
use crate::phy::csr_access;
use crate::phy::wcklevel::Step;
use crate::rd::CLOCK_CSRS;
use crate::wl::{WlExec, WlFailure};
use dwc_ddrctl::regs::{DFIMISC, DFIPHYMSTR, DFISTAT, DFIUPD0, PWRCTL, STAT, SWSTAT};

/// PHY CSRs read in stage 0: DfiInitComplete, its shadow, Dfi0/1Status,
/// PMIEnable, MemResetL, PUBMODE, HwtControlVal, HwtControlOvr,
/// HwtLpCsEnA/B, SequencerOverride, xlat 0x20124/0x2018b, ACSMWckFreeRunMode,
/// 0x20045, PllCtrl5, 0x200ef, 0x20310/11, MicroContMuxSel, UcclkHclkEnables,
/// DfiInitCompleteShadow's neighbour 0xd00e7 is SequencerOverride.
pub const PHY_CSRS: [u32; 20] = [
    0x200f9, 0xd00fa, 0x2005a, 0x2005b, 0x20054, 0x20060, 0x2006e, 0x2007e, 0x2007a, 0x20072,
    0x20073, 0xd00e7, 0x20124, 0x2018b, 0x20041, 0x20045, 0x2004c, 0x200ef, 0x20310, 0x20311,
];
/// The PIE sequencer's vectors as the library writes them (unwritten in our
/// flow): 0x9001c-0x9002f, then 0x90700-0x90708.
pub const PIE_VECTORS: [u32; 29] = [
    0x9001c, 0x9001d, 0x9001e, 0x9001f, 0x90020, 0x90021, 0x90022, 0x90023, 0x90024, 0x90025,
    0x90026, 0x90027, 0x90028, 0x90029, 0x9002a, 0x9002b, 0x9002c, 0x9002d, 0x9002e, 0x9002f,
    0x90700, 0x90701, 0x90702, 0x90703, 0x90704, 0x90705, 0x90706, 0x90707, 0x90708,
];
/// Controller registers read per channel: DFIMISC, DFISTAT, STAT, PWRCTL,
/// SWSTAT, DFIUPD0, DFIPHYMSTR, then set 1's DFITMG0 (+0x588) and DRAMTMG2
/// (+0x08).
pub const CTRL_REGS: usize = 9;
/// PState SRAM records (`0xa1000` + 4 x slot, 4 halfwords each).
pub const SRAM_SLOTS: u32 = 1024;

/// The mission CSRs written by APB after the PIE load (DBYTE ones through the `0x1f0..` broadcast): per DBYTE
/// 0x0b9 = 1, 0x0b1 = 0x180, 0x0ba = 1, 0x0a2 = 0, 0x0b5 = 1, 0x057 = 0;
/// 0x0b0 = 4 and RxClkCntl 0x056 = 4; HwtControlVal 0x33, 0x200ef
/// 0xffff (bit 15 reads 0), 0x20310/11 1; per pstate
/// ACSMWckFreeRunMode 1, 0x20045 2, PllCtrl5 0x080f, 0x20015 0, AC
/// 0x3n04a/b 0, HwtLpCsEnA/B 3/3; HwtControlOvr 0. SequencerOverride, the
/// PIE vectors, MemResetL (protect bit kept) and the xlat table are not
/// written.
pub const MISSION: [(u32, u16); 26] = [
    // ACSMWckEnControl 0 (our WL/read-enable set-up writes 1 and
    // never clears it; the vendor's final state is 0: the controller drives
    // WCK through the DFI in mission mode).
    (0x2002a, 0),
    // The vendor mission state's PMIEnable (0x200bb/bc are status).
    (0x20054, 1),
    // The two DBYTE controls the vendor's mission state holds at 4 and
    // ours at 0x30 / 0 (without them the controller's reads did not depend
    // on the read timing at all): 0x1n0b0 (the DTSM byte control that routes
    // the receive data) and RxClkCntl 0x1n056.
    (0x1f0b0, 4),
    (0x1f056, 4),
    (0x1f0b9, 1),
    (0x1f0b1, 0x180),
    (0x1f0ba, 1),
    (0x1f0a2, 0),
    (0x1f0b5, 1),
    (0x1f057, 0),
    (0x2007e, 0x33),
    (0x200ef, 0x7fe0),
    (0x20310, 1),
    (0x20311, 1),
    (0x20041, 1),
    (0x20045, 2),
    (0x2004c, 0x080f),
    (0x20015, 0),
    (0x3004a, 0),
    (0x3004b, 0),
    (0x3104a, 0),
    (0x3104b, 0),
    (0x20072, 3),
    (0x20073, 3),
    (0x2007a, 0),
    (0x2006e, 0),
];
/// The firmware's own end-of-training restores after its PUBMODE 0 (its
/// trace): PptCtlStatic per DBYTE 0x0b3f/0x0b33, 0x20011 0xf, the DTSM
/// gates off, 0x200f8 1, 0x20009 0. With these in the mission list the CPU hung on the first APB read after
/// PUBMODE 0. They are now written last, after everything else, one by one
/// with a log line each, to name the one that hangs.
pub const MISSION_LATE: [(u32, u16); 8] = [
    (0x100a3, 0x0b3f),
    (0x110a3, 0x0b33),
    (0x120a3, 0x0b3f),
    (0x130a3, 0x0b33),
    (0x20011, 0xf),
    (0x1f0b3, 0),
    (0x1f0b4, 0),
    (0x20009, 0),
];
/// Self-refresh entry to both ranks: the firmware's SRE row shape (CA
/// first edge 0x28, both chip selects, four empty rows, the timing row
/// 0x7a00, the end row 0x0400).
pub const SRE: [Row; 6] = [
    Row::raw(0xc028, 0, 0, 0),
    Row::EMPTY,
    Row::EMPTY,
    Row::EMPTY,
    Row::nop(0x7a00),
    Row::nop(0x0400),
];
/// Mode registers read through the controller after the hand-over: the
/// device's own (MR5 manufacturer, MR8 density/width, MR4 refresh rate,
/// MR14/MR15 Vref), then the ones our initialisation wrote (known answers
/// at pstate 1, CXMT 4 GB set: MR16 0x45, MR2 0x88, MR1 0x80, MR3 0xc6,
/// MR10 0x54, MR13 0x60, MR18 0x0c, MR20 0x02, MR28 0x04, MR11 0x10, MR17
/// 0x06).
pub const MRS: [u32; 16] = [5, 8, 4, 14, 15, 16, 2, 1, 3, 10, 13, 18, 20, 28, 11, 17];
/// The values our initialisation wrote (index as [`MRS`], None: read-only
/// or not written by us).
pub const MRS_KNOWN: [Option<u32>; 16] = [
    None,
    None,
    None,
    None,
    None,
    Some(0x45),
    Some(0x88),
    Some(0x80),
    Some(0xc6),
    Some(0x54),
    Some(0x60),
    Some(0x0c),
    Some(0x02),
    Some(0x04),
    Some(0x10),
    Some(0x06),
];
/// DBYTE CSRs the vendor's mission state holds differently from ours
/// (offsets in the DBYTE block): RxClkCntl 0x056, TrainingCntr 0x0aa,
/// 0x0b0, 0x074, 0x00c, 0x00d, FIFO info 0x070/0x071, read before and
/// after the DFI initialisation on every DBYTE.
pub const DBYTE_CANDIDATES: [u32; 8] = [0x056, 0x0aa, 0x0b0, 0x074, 0x00c, 0x00d, 0x070, 0x071];
/// The read-timing matrix after the hand-over: the PHY's HwtMRL/DFIMRL
/// over [`MATRIX_MRL`] and the controller's dfi_t_rddata_en (+0x580
/// [22:16] of the active frequency set; 0x2f in the vendor's sets and in
/// ours) over [`MATRIX_RDEN`], MR16 and MR2 of channel 0 rank 0 read at
/// each point; the pair is put back afterwards. Sweeping +0x588 [21:16],
/// which is dfi_twck_delay, in set 1 while the controller ran set 0 (MSTR2
/// [4:0] never written) changed nothing; with MSTR2 set to the pstate and
/// the real field of set 1 swept, the PHY's own compare failed on every
/// lane. This leaves MSTR2 as found (set 0, the vendor's) and sweeps the
/// real field of that active set.
pub const MATRIX_MRL: [u16; 6] = [2, 3, 4, 5, 6, 7];
pub const MATRIX_RDEN: [u32; 10] = [0x29, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f, 0x30, 0x31, 0x32, 0x34];
/// The frequency-set register holding dfi_t_rddata_en and its field.
pub const DFITMG0: usize = 0x580;
pub const RDDATA_EN_SHIFT: u32 = 16;
pub const RDDATA_EN_MASK: u32 = 0x7f << RDDATA_EN_SHIFT;
/// The frequency-set register holding dfi_twck_delay [21:16] (computed
/// from TxWckDly; post.rs calls it rddata_en).
pub const DFITMG_WCK: usize = 0x588;
/// MSTR2: [4:0] the controller's active frequency set.
pub const MSTR2: usize = 0x008;
/// +0x588 [21:16] of the active set as the vendor firmware left it on this
/// board (Debian, Oct 15; its TxWckDly 0x1f0 gives 21 + 8 = 29). Our
/// timing code leaves 24 there (the PHY's compare passed after the
/// hand-over, the controller read 0x000060ff); the formula on our TxWckDly
/// 0x1f8 gives 33 (the compare failed on every lane, the controller read
/// 0x9fff0000). The hand-over writes 29 before the DFI init and then sweeps
/// the field on the active set ([`MATRIX_WCK`]).
pub const TWCK_LIVE: u32 = 29;
/// The +0x588 [21:16] sweep on the active set at HwtMRL 4 and the found
/// dfi_t_rddata_en: MR16 and MR2 of channel 0 rank 0 and two DRAM words
/// (written then read at each point).
pub const MATRIX_WCK: [u32; 16] = [
    21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36,
];
/// The two DRAM words of the sweep (address, pattern).
pub const WCK_WORDS: [(usize, u32); 2] = [(0x8000_0000, 0xfedc_ba98), (0x8000_0004, 0x0123_4567)];
/// The post-training fold of the active set 0: after the PHY training
/// these six set-0 registers are adjusted from the message block's CDD
/// bytes (rd2wr/wr2rd in DRAMTMG2 +0x008, the write-data delay +0x024
/// [7:0], +0x060 [23:16], DFITMG1 +0x584 [23:16], +0xd04 and +0xd08
/// [7:0]/[15:8]). The open driver's post-training path applies the fold
/// with `dwc_ddrctl::dfi::fold_cdd` (`post.rs`). This hand-over image has
/// no message block, so it writes instead the values the vendor firmware
/// left in set 0 of both channels on this board (measured under Debian:
/// rd2wr +2, wr2rd +0, +0x024 +1, +0x060 +2, DFITMG1 +1, +0xd04 +1/+1,
/// +0xd08 +1/+2 against the pre-fold values) as they are.
pub const FOLD_LIVE: [(usize, u32); 6] = [
    (0x008, 0x070d_1213),
    (0x024, 0x0002_040f),
    (0x060, 0x0010_120c),
    (0x584, 0x0008_0303),
    (0xd04, 0x0000_0a0c),
    (0xd08, 0x0000_1425),
];
/// The same fold as deltas on the active set's registers (register, field
/// shift, CDD maximum added): the vendor's live set 0 is its pre-fold set 0
/// plus these, and the hand-over runs the controller on set 1 (our pstate's 1200
/// MT/s timings), where the absolute set-0 values do not belong.
pub const FOLD_DELTA: [(usize, u32, u32); 8] = [
    (0x008, 8, 2),
    (0x024, 0, 1),
    (0x060, 16, 2),
    (0x584, 16, 1),
    (0xd04, 0, 1),
    (0xd04, 8, 1),
    (0xd08, 0, 1),
    (0xd08, 8, 2),
];
/// Controller status read at every matrix point: DFISTAT, STAT, +0x308,
/// MRSTAT of channel 0.
pub const MATRIX_STATUS: usize = 4;
/// Timeouts of the DFI waits.
pub const T_US: u64 = 100_000;
/// DFI frequency field written for pstate `ps` at initialisation: the
/// vendor sets bit 4 with the pstate.
pub const fn dfi_frequency(ps: u32) -> u32 {
    0x10 | ps
}
/// The quick memtest: 2048 MiB (the 32-bit reach of bt0: the DRAM base and
/// its 1 GiB middle), 1024 words at each.
pub const QUICK_MIB: u32 = 2048;
pub const QUICK_WORDS: usize = 1024;

/// One probe step's readback: `0x200f9`, `0xd00fa`, Dfi0Status,
/// Dfi1Status, and DFISTAT of both channels.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ProbeStep {
    pub csr: u32,
    pub value: u16,
    pub phy: [u16; 4],
    pub dfistat: [u32; 2],
    pub restored: u16,
}

/// Stage 0 and 1.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Probe {
    pub clocks: [u16; CLOCK_CSRS.len()],
    pub phy: [u16; PHY_CSRS.len()],
    pub pie: [u16; PIE_VECTORS.len()],
    pub ctrl: [[u32; CTRL_REGS]; 2],
    pub sram_nonzero: u16,
    pub steps: [ProbeStep; 3],
    /// The step whose write DFISTAT[0] followed on channel 0 (0: `0x200f9`,
    /// 1: `0xd00fa`, 2: PUBMODE 0), or None.
    pub cpu_drives: Option<u8>,
}

impl Probe {
    pub const EMPTY: Probe = Probe {
        clocks: [0; CLOCK_CSRS.len()],
        phy: [0; PHY_CSRS.len()],
        pie: [0; PIE_VECTORS.len()],
        ctrl: [[0; CTRL_REGS]; 2],
        sram_nonzero: 0,
        steps: [ProbeStep {
            csr: 0,
            value: 0,
            phy: [0; 4],
            dfistat: [0; 2],
            restored: 0,
        }; 3],
        cpu_drives: None,
    };
}

/// Where stage 2 stopped.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Stop {
    Sre(WlFailure),
    Dfi(&'static str),
    Mrr(&'static str),
}

/// Stage 2.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Handover {
    pub sre_done: bool,
    /// Mission CSRs written and read back equal.
    pub mission_ok: u8,
    /// dfi_twck_delay (+0x588 [21:16]) written to sets 0 and 1 of every
    /// channel computed from TxWckDly, and the PHY CSRs it came
    /// from (PclkPtrInitVal, TxWckDly DBYTE 0).
    pub rden: u32,
    pub rden_in: [u32; 2],
    /// MSTR2 of channel 0 before and after the active-set write.
    pub mstr2: [u32; 2],
    /// +0xb84 / +0xb88 of channel 0 after the hand-over.
    pub opctrl: [u32; 2],
    /// The active set's +0x580 as found (dfi_t_rddata_en in [22:16]).
    pub dfitmg0_found: u32,
    /// [`FOLD_LIVE`] registers of channel 0's active set before and after
    /// the fold.
    pub fold: [[u32; 2]; FOLD_LIVE.len()],
    /// MR16, MR2 of channel 0 rank 0 per [`MATRIX_WCK`] point (0xffff: the
    /// read failed), and the two [`WCK_WORDS`] read back at each point.
    pub matrix_wck: [[u16; 2]; MATRIX_WCK.len()],
    pub matrix_wck_mem: [[u32; 2]; MATRIX_WCK.len()],
    /// +0x588 of channel 0's active set after the sweep.
    pub twck_after: u32,
    /// DFIMISC, DFISTAT, STAT, PWRCTL per channel after the hand-over (or
    /// at the stop).
    pub ctrl: [[u32; 4]; 2],
    /// PHY after the hand-over: DfiInitComplete, PMIEnable, MemResetL,
    /// Dfi0/1Status, 0x200bb.
    pub phy_after: [u16; 6],
    /// The first mission CSR that did not read back: (CSR, want, got).
    pub mission_bad: (u32, u16, u16),
    /// [`DBYTE_CANDIDATES`] per DBYTE after the mission writes and after
    /// the DFI initialisation.
    pub dbyte_before: [[u16; DBYTE_CANDIDATES.len()]; 4],
    pub dbyte_after: [[u16; DBYTE_CANDIDATES.len()]; 4],
    /// [`MRS`] per channel and rank (None: the read failed).
    pub mrr: [[[Option<u32>; MRS.len()]; 2]; 2],
    /// MR16, MR2 of channel 0 rank 0 per [`MATRIX_MRL`] x [`MATRIX_RDEN`]
    /// (0xffff: the read failed).
    pub matrix: [[[u16; 2]; MATRIX_RDEN.len()]; MATRIX_MRL.len()],
    /// [`MATRIX_STATUS`] after the two reads at each point.
    pub matrix_status: [[[u32; MATRIX_STATUS]; MATRIX_RDEN.len()]; MATRIX_MRL.len()],
    /// The timing pair read back after the matrix (HwtMRL, DFITMG0 set 1
    /// of channel 0).
    pub restored_pair: (u16, u32),
    /// The quick memtest: Ok, or the first mismatch (addr, expected, got).
    pub quick: Result<(), (usize, u32, u32)>,
    pub quick_ran: bool,
    /// The liveness check after everything: PUBMODE back to 1 and one RDC
    /// compare through the ACSM at the trained point on channel A rank 0
    /// (passing-lane bits per byte, compares, errors; None: not run or the
    /// launch failed).
    pub live_compare: Option<([u16; 2], u32, u32)>,
    /// The same compare right after the DFI initialisation, before any
    /// controller-driven command (PUBMODE 1 briefly, then 0 again).
    pub live_compare_a: Option<([u16; 2], u32, u32)>,
    /// The controller's INIT0, INIT1, INIT3, INIT4, INIT6, INIT7 of channel
    /// 0 before the DFI initialisation and after the hand-over.
    pub init_before: [u32; 6],
    pub init_after: [u32; 6],
    /// +0xd00 [31:30] (the skip / LPDDR5 mode) was already set.
    pub init_skip_set: bool,
    /// At each liveness compare, MR3, MR20, MR22, MR10 read through the
    /// ACSM into the RX FIFO (byte 0's eight lane bytes each; the decode
    /// as in `rdtrain::mrr_read`).
    pub live_mrr_a: [[u8; 8]; 4],
    pub live_mrr_b: [[u8; 8]; 4],
    /// The DFI read-data configuration after the hand-over (channel 0):
    /// DFIMISC, set-1 DFITMG0 (+0x588), DFITMG1 (+0x58c), +0x590, DBICTL
    /// (0x1c0), MSTR0; then the PHY's DfiFreqRatio 0x20000, PclkPtrInitVal
    /// 0x20002, DMIPinPresent 0x20008, DfiMode 0x20018,
    /// DfiCfgRdDataValidTicks 0xd0037, DfiInitComplete 0x200f9.
    pub dfi_cfg: [u32; 6],
    pub dfi_phy: [u16; 6],
    pub stop: Option<Stop>,
    pub elapsed_us: u32,
}

impl Handover {
    pub const EMPTY: Handover = Handover {
        sre_done: false,
        mission_ok: 0,
        rden: 0,
        rden_in: [0; 2],
        mstr2: [0; 2],
        opctrl: [0; 2],
        dfitmg0_found: 0,
        fold: [[0; 2]; FOLD_LIVE.len()],
        matrix_wck: [[0xffff; 2]; MATRIX_WCK.len()],
        matrix_wck_mem: [[0; 2]; MATRIX_WCK.len()],
        twck_after: 0,
        ctrl: [[0; 4]; 2],
        phy_after: [0; 6],
        mission_bad: (0, 0, 0),
        dbyte_before: [[0; DBYTE_CANDIDATES.len()]; 4],
        dbyte_after: [[0; DBYTE_CANDIDATES.len()]; 4],
        mrr: [[[None; MRS.len()]; 2]; 2],
        matrix: [[[0xffff; 2]; MATRIX_RDEN.len()]; MATRIX_MRL.len()],
        matrix_status: [[[0; MATRIX_STATUS]; MATRIX_RDEN.len()]; MATRIX_MRL.len()],
        restored_pair: (0, 0),
        quick: Ok(()),
        quick_ran: false,
        live_compare: None,
        live_compare_a: None,
        init_before: [0; 6],
        init_after: [0; 6],
        init_skip_set: false,
        live_mrr_a: [[0; 8]; 4],
        live_mrr_b: [[0; 8]; 4],
        dfi_cfg: [0; 6],
        dfi_phy: [0; 6],
        stop: None,
        elapsed_us: 0,
    };
}

fn r16<H: Hw>(hw: &mut H, c: u32) -> u16 {
    hw.read16(crate::phy::csr(c))
}
fn w16<H: Hw>(hw: &mut H, c: u32, v: u16) {
    hw.write16(crate::phy::csr(c), v);
}

fn ctrl_regs<H: Hw>(hw: &mut H, ch: usize) -> [u32; CTRL_REGS] {
    [
        hw.read32(ddrc(ch, DFIMISC)),
        hw.read32(ddrc(ch, DFISTAT)),
        hw.read32(ddrc(ch, STAT)),
        hw.read32(ddrc(ch, PWRCTL)),
        hw.read32(ddrc(ch, SWSTAT)),
        hw.read32(ddrc(ch, DFIUPD0)),
        hw.read32(ddrc(ch, DFIPHYMSTR)),
        hw.read32(freq(ch, 1, 0x588)),
        hw.read32(freq(ch, 1, 0x08)),
    ]
}

fn probe_read<H: Hw>(hw: &mut H, channels: usize) -> ([u16; 4], [u32; 2]) {
    let phy = [
        r16(hw, 0x200f9),
        r16(hw, 0xd00fa),
        r16(hw, 0x2005a),
        r16(hw, 0x2005b),
    ];
    let mut d = [0u32; 2];
    for (ch, v) in d.iter_mut().enumerate().take(channels) {
        *v = hw.read32(ddrc(ch, DFISTAT));
    }
    (phy, d)
}

/// Stage 0 and 1 (module documentation).
pub fn probe<H: Hw>(hw: &mut H, channels: usize, out: &mut Probe) {
    *out = Probe::EMPTY;
    csr_access(hw, true);
    for (v, &c) in out.clocks.iter_mut().zip(CLOCK_CSRS.iter()) {
        *v = r16(hw, c);
    }
    for (v, &c) in out.phy.iter_mut().zip(PHY_CSRS.iter()) {
        *v = r16(hw, c);
    }
    for (v, &c) in out.pie.iter_mut().zip(PIE_VECTORS.iter()) {
        *v = r16(hw, c);
    }
    for ch in 0..channels.min(2) {
        out.ctrl[ch] = ctrl_regs(hw, ch);
    }
    let mut n = 0u16;
    for slot in 0..SRAM_SLOTS {
        if r16(hw, 0xa1000 + 4 * slot + 3) & 8 != 0 {
            n += 1;
        }
        if slot % 128 == 0 {
            hw.kick_watchdog();
        }
    }
    out.sram_nonzero = n;
    // The probe: each write, its effect, the value back.
    for (k, (csr, value)) in [(0x200f9u32, 1u16), (0xd00fa, 1), (0x2006e, 0)]
        .into_iter()
        .enumerate()
    {
        let found = r16(hw, csr);
        w16(hw, csr, value);
        hw.udelay(10);
        let (phy, dfistat) = probe_read(hw, channels);
        w16(hw, csr, found);
        hw.udelay(10);
        let restored = r16(hw, csr);
        out.steps[k] = ProbeStep {
            csr,
            value,
            phy,
            dfistat,
            restored,
        };
        if out.cpu_drives.is_none() && dfistat[0] & 1 != 0 && k < 2 {
            out.cpu_drives = Some(k as u8);
        }
    }
    csr_access(hw, false);
}

/// The read-data enable formula on the PHY's PclkPtrInitVal and
/// DBYTE 0 TxWckDly (as `post::timing_after_phy`).
pub const fn rddata_en(ptr_init: u32, tx_wck: u32) -> u32 {
    ((tx_wck >> 1) & 0xf) + (((ptr_init & 0xf) + 1) >> 3) * 4 + 21
}

/// The ACSM RDC compare at the trained point on channel A rank 0 with the
/// PHY owning the DFI (PUBMODE 1), then PUBMODE back to `pubmode_after`.
fn live_compare<H: Hw>(
    hw: &mut H,
    pubmode_after: u16,
    mrr: &mut [[u8; 8]; 4],
) -> Option<([u16; 2], u32, u32)> {
    use crate::phy::wcklevel::{Channel, Rank};
    use crate::rdtrain::{data_setup, expected, point_counts, CHECK_PATTERNS};
    let dbs = crate::rd::dbytes(Channel::A);
    let mut ex = WlExec::new(hw);
    ex.write(0x2006e, 1);
    // MR3, MR20, MR22, MR10 through the ACSM into the RX FIFO (the
    // reference's read-out; byte 0's lanes 0-7).
    {
        use crate::rd::{fifo_info, fifo_vis, VIS_DFE0};
        let standby = ex.read(0x1000a);
        for (k, ma) in [3u8, 20, 22, 10].into_iter().enumerate() {
            ex.write(crate::rdtrain::RX_STANDBY, 0);
            crate::rd::fire(&mut ex, dbs, crate::rd::EV_FIFO);
            ex.step(Step::Launch {
                rows: &crate::phy::rxen::mrr_program(Rank::R0, ma),
                outer: 0,
            });
            ex.write(crate::rdtrain::RX_STANDBY, standby);
            ex.write(fifo_vis(0), crate::rdtrain::VIS_MRR[0]);
            for lane in 0..8u32 {
                ex.read(fifo_info(0, lane));
            }
            for lane in 0..8u32 {
                ex.write(fifo_vis(0), crate::rdtrain::VIS_MRR[1]);
                mrr[k][lane as usize] = ex.read(0x10073 + 0x100 * lane) as u8;
            }
            ex.write(fifo_vis(0), VIS_DFE0);
        }
    }
    data_setup(
        &mut ex,
        dbs,
        Rank::R0,
        CHECK_PATTERNS[0],
        expected(CHECK_PATTERNS[0]),
    );
    let prog = crate::phy::rxen::centring_program(Rank::R0);
    let (m, _, n) = point_counts(&mut ex, dbs, &prog);
    let r = ex.failure.is_none().then_some((m, n[0], n[1]));
    ex.write_force(0x2006e, pubmode_after);
    csr_access(hw, false);
    r
}

/// The controller's initialisation block on this part: +0xd00 ([31:30]
/// the LPDDR5 mode / skip, set by our timing code) and the five words after
/// it.
const INIT_REGS: [usize; 6] = [0xd00, 0xd04, 0xd08, 0xd0c, 0xd10, 0xd14];

/// Stage 2 (module documentation). `answer` is the probe's `cpu_drives`
/// step: the CSR written to 1 after dfi_init_start.
#[inline(never)]
pub fn handover<H: Hw>(
    hw: &mut H,
    p: &Para,
    answer: u8,
    sre: bool,
    pstate: u32,
    out: &mut Handover,
    snap: &mut [u16; crate::phy::csrlist::TRM_CSRS.len()],
) {
    *out = Handover::EMPTY;
    let t0 = hw.now_us();
    let channels = p.channels().min(2);
    let ranks = if p.dual_rank() { 2 } else { 1 };

    // 1. Self-refresh entry, both ranks, through the ACSM (PUBMODE still 1).
    if sre {
        let mut ex = WlExec::new(hw);
        ex.step(Step::Launch {
            rows: &SRE,
            outer: 0,
        });
        if let Some(f) = ex.failure {
            out.stop = Some(Stop::Sre(f));
            csr_access(hw, false);
            out.elapsed_us = hw.now_us().wrapping_sub(t0) as u32;
            return;
        }
        out.sre_done = true;
    }

    // 2. The mission CSRs, PUBMODE last. Each write is logged first (an
    // earlier run hung the CPU somewhere in this list with no print after
    // the probe).
    csr_access(hw, true);
    for &(c, v) in MISSION.iter() {
        hw.log(format_args!("[dram] HO mission {c:05x} <- {v:04x}\n"));
        w16(hw, c, v);
        // DBYTE broadcasts read back through DBYTE 0.
        let rb = if c >> 12 == 0x1f { c - 0xf000 } else { c };
        let got = r16(hw, rb);
        hw.log(format_args!("[dram] HO mission {c:05x} read {got:04x}\n"));
        if got == v {
            out.mission_ok += 1;
        } else if out.mission_bad.0 == 0 {
            out.mission_bad = (c, v, got);
        }
    }
    for db in 0..4u32 {
        for (k, &off) in DBYTE_CANDIDATES.iter().enumerate() {
            out.dbyte_before[db as usize][k] = r16(hw, 0x10000 + 0x1000 * db + off);
        }
    }
    out.init_before = INIT_REGS.map(|o| hw.read32(ddrc(0, o)));
    out.init_skip_set = out.init_before[0] >> 30 != 0;
    // 3. The controller's read-data enable (set 1) and DFI frequency.
    let ptr_init = r16(hw, 0x20002) as u32;
    let tx_wck = r16(hw, 0x1002a) as u32;
    out.rden_in = [ptr_init, tx_wck];
    out.rden = rddata_en(ptr_init, tx_wck);
    csr_access(hw, false);
    let rden = out.rden;
    out.mstr2[0] = hw.read32(ddrc(0, MSTR2));
    // The controller's active frequency set follows the PHY's pstate (set
    // 1 holds the 1200 MT/s timings our pstate 1 runs at; the vendor's
    // mission is pstate 0 on set 0, DFIMISC 0x15014 live). An earlier run
    // did this together with +0x588 [21:16] = 33, which alone broke the
    // PHY's compare; set 1 itself is untested.
    let active = pstate as usize;
    out.dfitmg0_found = hw.read32(freq(0, active, DFITMG0));
    for (k, &(off, _)) in FOLD_LIVE.iter().enumerate() {
        out.fold[k][0] = hw.read32(freq(0, active, off));
    }
    let mut dfi = || -> Result<(), Error> {
        for ch in 0..channels {
            hw.write32(ddrc(ch, ctrl::SWCTL), 0);
            hw.clrset32(ddrc(ch, MSTR2), 0x1f, pstate);
            sw_done(hw, ch)?;
            // The fold of the active set, with dis_dq held during the
            // quasi-dynamic writes.
            hw.set32(ddrc(ch, ctrl::DDRC_B84), 1);
            hw.write32(ddrc(ch, ctrl::SWCTL), 0);
            for &(off, shift, add) in FOLD_DELTA.iter() {
                hw.modify32(freq(ch, active, off), |v| v + (add << shift));
            }
            // The vendor's live value, not our formula's (`rden`,
            // logged).
            let _ = rden;
            for ps in 0..2 {
                hw.modify32(freq(ch, ps, DFITMG_WCK), |v| {
                    (v & !0x3f_0000) | (TWCK_LIVE << 16)
                });
            }
            hw.clrset32(ddrc(ch, DFIMISC), 0x1f00, dfi_frequency(pstate) << 8);
            sw_done(hw, ch)?;
            hw.clr32(ddrc(ch, ctrl::DDRC_B84), 1);
        }
        // 4. DFI initialisation with our answer after dfi_init_start.
        for ch in 0..channels {
            hw.write32(ddrc(ch, ctrl::SWCTL), 0);
        }
        for ch in 0..channels {
            hw.set32(ddrc(ch, DFIMISC), 0x1);
        }
        for ch in 0..channels {
            hw.set32(ddrc(ch, DFIMISC), 0x20);
        }
        for ch in 0..channels {
            hw.write32(ddrc(ch, ctrl::SWCTL), 1);
        }
        for ch in 0..channels {
            wait(hw, "SWSTAT.sw_done_ack (init_start)", T_US, |hw| {
                hw.read32(ddrc(ch, SWSTAT)) & 1 != 0
            })?;
        }
        hw.udelay(10);
        csr_access(hw, true);
        w16(hw, if answer == 1 { 0xd00fa } else { 0x200f9 }, 1);
        csr_access(hw, false);
        for ch in 0..channels {
            wait(hw, "DFISTAT.dfi_init_complete", T_US, |hw| {
                hw.read32(ddrc(ch, DFISTAT)) & 1 != 0
            })?;
        }
        hw.udelay(50);
        hw.set32(0x0701_0250, 1);
        hw.udelay(2000);
        for ch in 0..channels {
            hw.write32(ddrc(ch, ctrl::SWCTL), 0);
        }
        for ch in 0..channels {
            hw.clr32(ddrc(ch, DFIMISC), 0x20);
        }
        for ch in 0..channels {
            hw.clr32(ddrc(ch, PWRCTL), 0x800);
        }
        for ch in 0..channels {
            wait(hw, "STAT.selfref_type", T_US, |hw| {
                hw.read32(ddrc(ch, STAT)) & (3 << 4) == 0
            })?;
            wait(hw, "STAT.operating_mode", T_US, |hw| {
                hw.read32(ddrc(ch, STAT)) & 7 == 1
            })?;
        }
        hw.udelay(50);
        for ch in 0..channels {
            hw.clr32(ddrc(ch, DFIMISC), 0x1);
        }
        for ch in 0..channels {
            hw.write32(ddrc(ch, ctrl::SWCTL), 1);
        }
        for ch in 0..channels {
            wait(hw, "SWSTAT.sw_done_ack (init done)", T_US, |hw| {
                hw.read32(ddrc(ch, SWSTAT)) & 1 != 0
            })?;
        }
        // 5. The ports after the PHY.
        crate::post::controller_after_phy(hw, p)
    };
    let r = dfi();
    out.mstr2[1] = hw.read32(ddrc(0, MSTR2));
    for (k, &(off, _)) in FOLD_LIVE.iter().enumerate() {
        out.fold[k][1] = hw.read32(freq(0, active, off));
    }
    out.opctrl = [hw.read32(ddrc(0, 0xb84)), hw.read32(ddrc(0, 0xb88))];
    for ch in 0..channels {
        out.ctrl[ch] = [
            hw.read32(ddrc(ch, DFIMISC)),
            hw.read32(ddrc(ch, DFISTAT)),
            hw.read32(ddrc(ch, STAT)),
            hw.read32(ddrc(ch, PWRCTL)),
        ];
    }
    csr_access(hw, true);
    out.phy_after = [
        r16(hw, 0x200f9),
        r16(hw, 0x20054),
        r16(hw, 0x20060),
        r16(hw, 0x2005a),
        r16(hw, 0x2005b),
        r16(hw, 0x200bb),
    ];
    for db in 0..4u32 {
        for (k, &off) in DBYTE_CANDIDATES.iter().enumerate() {
            out.dbyte_after[db as usize][k] = r16(hw, 0x10000 + 0x1000 * db + off);
        }
    }
    // The CSR mux to the PHY, as the vendor leaves it.
    csr_access(hw, false);
    if let Err(e) = r {
        out.stop = Some(Stop::Dfi(match e {
            Error::Timeout(m) | Error::Config(m) => m,
            _ => "controller",
        }));
        out.elapsed_us = hw.now_us().wrapping_sub(t0) as u32;
        return;
    }
    out.init_after = INIT_REGS.map(|o| hw.read32(ddrc(0, o)));
    // The firmware's end-of-training CSRs right after the DFI init, before
    // any read. Written last instead (the earlier hang was with them before
    // the DFI init), every controller read ran with PptCtlStatic 0x000c/0,
    // 0x1n0b3/0x1n0b4 = 1 and 0x20011, 0x20009 as training left them.
    late_csrs(hw);
    // The full snapshot right after the hand-over, before any compare.
    crate::rd::snapshot(hw, snap);
    // Liveness A: right after the DFI initialisation, before any
    // controller-driven command.
    out.live_compare_a = live_compare(hw, 0, &mut out.live_mrr_a);
    // The PHY's mission state for the host-side diff against the vendor
    // library's (feature av3h), read-only.
    crate::dump::dump_phy(hw, "ho");
    out.dfi_cfg = [
        hw.read32(ddrc(0, DFIMISC)),
        hw.read32(freq(0, active, DFITMG0)),
        hw.read32(freq(0, active, 0x584)),
        hw.read32(freq(0, active, DFITMG_WCK)),
        hw.read32(ddrc(0, 0x1c0)),
        hw.read32(ddrc(0, dwc_ddrctl::regs::MSTR0)),
    ];
    csr_access(hw, true);
    out.dfi_phy = [
        r16(hw, 0x20000),
        r16(hw, 0x20002),
        r16(hw, 0x20008),
        r16(hw, 0x20018),
        r16(hw, 0xd0037),
        r16(hw, 0x200f9),
    ];
    csr_access(hw, false);
    // 6. Mode registers through the controller.
    for ch in 0..channels {
        for rank in 0..ranks {
            for (k, &mr) in MRS.iter().enumerate() {
                hw.kick_watchdog();
                out.mrr[ch][rank as usize][k] = crate::mr::read(hw, ch, rank, mr).ok();
            }
        }
    }
    if out.mrr[0][0].iter().all(Option::is_none) {
        out.stop = Some(Stop::Mrr("every MRR of rank 0 channel 0 failed"));
        out.elapsed_us = hw.now_us().wrapping_sub(t0) as u32;
        return;
    }
    // 6a. The +0x588 [21:16] sweep on the active set (HwtMRL 4 as applied,
    // dfi_t_rddata_en as found), MRRs and two DRAM words at each point.
    let set_twck = |hw: &mut H, v: u32| -> Result<(), Error> {
        for ch in 0..channels {
            hw.write32(ddrc(ch, ctrl::SWCTL), 0);
            hw.modify32(freq(ch, active, DFITMG_WCK), |r| {
                (r & !0x3f_0000) | (v << 16)
            });
            sw_done(hw, ch)?;
        }
        Ok(())
    };
    for (i, &v) in MATRIX_WCK.iter().enumerate() {
        hw.kick_watchdog();
        if set_twck(hw, v).is_err() {
            continue;
        }
        for (k, mr) in [16u32, 2].into_iter().enumerate() {
            out.matrix_wck[i][k] = crate::mr::read(hw, 0, 0, mr)
                .map(|x| x as u16)
                .unwrap_or(0xffff);
        }
        for &(a, pat) in WCK_WORDS.iter() {
            hw.write32(a, pat);
        }
        for (k, &(a, _)) in WCK_WORDS.iter().enumerate() {
            out.matrix_wck_mem[i][k] = hw.read32(a);
        }
    }
    let _ = set_twck(hw, TWCK_LIVE);
    out.twck_after = hw.read32(freq(0, active, DFITMG_WCK));
    // 6b. The read-timing matrix on channel 0 rank 0 (the active set's
    // dfi_t_rddata_en), the pair put back.
    let found = out.dfitmg0_found;
    let set_pair = |hw: &mut H, mrl: u16, rden: u32| -> Result<(), Error> {
        csr_access(hw, true);
        for &c in crate::rd::MRL_WRITE_CSRS.iter() {
            w16(hw, c, mrl);
        }
        csr_access(hw, false);
        for ch in 0..channels {
            hw.write32(ddrc(ch, ctrl::SWCTL), 0);
            hw.modify32(freq(ch, active, DFITMG0), |v| {
                (v & !RDDATA_EN_MASK) | (rden << RDDATA_EN_SHIFT)
            });
            sw_done(hw, ch)?;
        }
        Ok(())
    };
    for (i, &mrl) in MATRIX_MRL.iter().enumerate() {
        for (j, &rden) in MATRIX_RDEN.iter().enumerate() {
            hw.kick_watchdog();
            if set_pair(hw, mrl, rden).is_err() {
                continue;
            }
            for (k, mr) in [16u32, 2].into_iter().enumerate() {
                out.matrix[i][j][k] = crate::mr::read(hw, 0, 0, mr)
                    .map(|v| v as u16)
                    .unwrap_or(0xffff);
            }
            out.matrix_status[i][j] = [
                hw.read32(ddrc(0, DFISTAT)),
                hw.read32(ddrc(0, STAT)),
                hw.read32(ddrc(0, 0x308)),
                hw.read32(ddrc(0, dwc_ddrctl::regs::MRSTAT)),
            ];
        }
    }
    let _ = set_pair(hw, 4, (found & RDDATA_EN_MASK) >> RDDATA_EN_SHIFT);
    csr_access(hw, true);
    out.restored_pair = (r16(hw, 0x2000d), 0);
    csr_access(hw, false);
    out.restored_pair.1 = hw.read32(freq(0, active, DFITMG0));
    // 7. The quick memtest.
    hw.kick_watchdog();
    out.quick_ran = true;
    out.quick = match memtest::quick(hw, 0x4000_0000, QUICK_MIB, QUICK_WORDS) {
        Ok(()) => Ok(()),
        Err(memtest::QuickError::Mismatch {
            addr,
            expected,
            got,
        }) => Err((addr, expected, got)),
        Err(memtest::QuickError::Config(_)) => Err((0, 0, 0)),
    };
    // 8. Liveness B: after the controller's reads and the memtest.
    out.live_compare = live_compare(hw, 1, &mut out.live_mrr_b);
    out.elapsed_us = hw.now_us().wrapping_sub(t0) as u32;
    // 9. Last: PUBMODE 0 again (liveness B leaves the PHY on the DFI).
    csr_access(hw, true);
    w16(hw, 0x2006e, 0);
    csr_access(hw, false);
}

/// The firmware's end-of-training CSRs ([`MISSION_LATE`]) one by one, each
/// logged with an APB read after it (a hang bisect form), with
/// PUBMODE 0 confirmed first.
fn late_csrs<H: Hw>(hw: &mut H) {
    csr_access(hw, true);
    w16(hw, 0x2006e, 0);
    let pm = r16(hw, 0x2006e);
    hw.log(format_args!("[dram] HO late: PUBMODE 0, read {pm:x}\n"));
    for &(c, v) in MISSION_LATE.iter() {
        hw.log(format_args!("[dram] HO late {c:05x} <- {v:04x}\n"));
        w16(hw, c, v);
        let rb = if c >> 12 == 0x1f { c - 0xf000 } else { c };
        let got = r16(hw, rb);
        let d0 = r16(hw, 0x10056);
        hw.log(format_args!(
            "[dram] HO late {c:05x} read {got:04x}; DBYTE 0 0x056 {d0:04x}\n"
        ));
    }
    hw.log(format_args!("[dram] HO late: done\n"));
    csr_access(hw, false);
}
