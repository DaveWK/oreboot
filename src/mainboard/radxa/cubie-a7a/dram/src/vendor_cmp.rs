// SPDX-License-Identifier: GPL-2.0-only
//! Our data-mode read compare in the vendor library's trained state (a
//! positive control; bt0 feature `av3g`). A first version never ran the
//! compare, did not run the restores after the failure, and the vendor
//! recovery then stopped at "Missing DTB"; hence these rules:
//!
//! - **UcclkHclkEnables (`0xc0080`) is read first** and both clocks are
//!   turned on for the compare (the training hardware, ACSM and DTSM, runs
//!   on Hclk); the found value goes back.
//! - **Every set-up write is read back; one mismatch stops the compare
//!   before any launch** ([`VendorCmp::aborted`]).
//! - **The restores run after a failure too** (`WlExec::write_force`):
//!   the trained delays with their latch pulses, the data set-up, the
//!   lane controls, TrainingParam, PUBMODE, MicroReset, the clocks, and
//!   **DFIUPD0 exactly as found** on both controllers.
//! - The caller keeps bt0 under 0x40000.
//!
//! Measured, for the side-by-side with our read training (`rdtrain`):
//! 1. the read-path settings as the vendor left them ([`crate::rd::ReadPath`]:
//!    HwtMRL, DFIMRL, the controller's read latency and DFI read-data
//!    enable), read only;
//! 2. the gate image (`EYE` false): the data check at the vendor's trained point (the
//!    RDC patterns and the negative control,
//!    [`crate::rdtrain::data_check`]) and **the gate window:** RxEnDly at
//!    trained + [`EN_OFFS`] (step [`EN_STEP`]), per byte the all-lane pass
//!    bits;
//! 3. the eye image (`EYE` true): one compare at the trained point and **the per-lane
//!    receive clock eye:** every lane's four receive clock registers at
//!    trained + [`CLK_OFFS`] effective taps (step [`CLK_STEP`], the
//!    training's joint sweep), per lane the pass bits.
//!
//! The MRR read-out is left out (bt0 size; it reads 0 in our state).
//! Two images, one per `EYE`, keep bt0 under 0x40000.
//!
//! The ACSM program memory is overwritten (the vendor's mission sequences
//! with it): the caller halts afterwards and the vendor recovery
//! re-initialises the DRAM.
use crate::hw::Hw;
use crate::phy::csr_access;
use crate::phy::rxen::{self, EntryMrs};
use crate::phy::wcklevel::{Channel, Rank, Step};
use crate::rd::{
    dbytes, fifo_vis, lane_csr, lane_f, param, pattern_off, pattern_on, read_data_saved, read_path,
    rx_en, ReadPath, ALL_ZERO, EVENTS, EV_FIFO, EV_RXEN, EV_UPDATE, HOLD_US, LANE_CTL, RX_CLK,
    VIS_DFE0,
};
use crate::rdtrain::{
    clk_eff, data_check, expected, point_counts, set_en, set_lanes, DataCheck, CHECK_PATTERNS,
    CLK_EFF_MAX,
};
use crate::wl::{WlExec, WlFailure, LANES};
use dwc_ddrctl::regs::{DFIUPD0, STAT, SWCTL, SWSTAT};

/// RxEnDly offsets around the trained value: -256..=256 step 8.
pub const EN_STEP: i16 = 8;
pub const EN_N: usize = 65;
/// Receive clock offsets (effective taps) around the trained values:
/// -64..=64 step 2 (the training's per-lane sweep).
pub const CLK_STEP: i16 = 2;
pub const CLK_N: usize = 65;
pub const fn en_off(i: usize) -> i16 {
    (i as i16 - (EN_N as i16 - 1) / 2) * EN_STEP
}
pub const fn clk_off(i: usize) -> i16 {
    (i as i16 - (CLK_N as i16 - 1) / 2) * CLK_STEP
}
/// For the report: the offsets.
pub const EN_OFFS: [i16; 2] = [en_off(0), en_off(EN_N - 1)];
pub const CLK_OFFS: [i16; 2] = [clk_off(0), clk_off(CLK_N - 1)];
const _: () = assert!(EN_OFFS[0] == -256 && EN_OFFS[1] == 256);
const _: () = assert!(CLK_OFFS[0] == -64 && CLK_OFFS[1] == 64);

/// PUBMODE, MemResetL, UcclkHclkEnables.
pub const PUBMODE: u32 = 0x2006e;
pub const MEM_RESET_L: u32 = 0x20060;
pub const UCCLK_HCLK: u32 = 0xc0080;
/// DFIUPD0.dis_auto_ctrlupd.
pub const DIS_AUTO_CTRLUPD: u32 = 1 << 31;
/// The eight DQ lanes (lane 8 is DMI).
pub const DQ_LANES: u16 = 0xff;
/// Launches per pattern of the data check.
pub const CHECK_N: u16 = 16;
/// Read-after-write records.
pub const RAW_MAX: usize = 24;

/// The compare's results.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct VendorCmp {
    /// PUBMODE, MemResetL, MicroReset, MicroContMuxSel, controller STAT
    /// ch 0/1 (low 16 bits), DFIUPD0 ch 0/1 (high 16 bits), as found.
    pub state_before: [u16; 8],
    /// The same after the quiesce and the PHY taking the DFI.
    pub state_at_launch: [u16; 8],
    /// UcclkHclkEnables as found and as set for the compare.
    pub clocks: [u16; 2],
    /// DFIUPD0 of both controllers as found.
    pub dfiupd0: [u32; 2],
    /// Read-after-write: (CSR, wanted, read back).
    pub raw: [(u32, u16, u16); RAW_MAX],
    pub n_raw: u8,
    /// A set-up write did not take: no launch ran.
    pub aborted: bool,
    /// The vendor's trained RxEnDly (both bytes, this rank) and receive
    /// clock registers ([`RX_CLK`] order) of every lane.
    pub trained_rx_en: [u16; 2],
    pub trained_clk: [[[u16; LANES]; 4]; 2],
    pub read_path: ReadPath,
    /// The gate image: the four-pattern data check.
    pub check: DataCheck,
    /// The eye image: one compare at the trained point: compares, errors
    /// (both bytes) and the passing-lane bits per byte.
    pub trained: (u32, u32, [u16; 2]),
    /// `0x70011`, `0x700d6`, `0x700d7` as found (the pattern block's
    /// controls; the firmware clears bit 4 of `0x70011` before it loads the
    /// expected-data words and sets it after, so the compare does the same).
    pub misc: [u16; 3],
    /// Per byte: bit i set when the eight DQ lanes passed at RxEnDly
    /// trained + [`en_off`]`(i)` (the DMI lane is left out: the vendor runs
    /// read DBI, so the pattern compare does not apply to it there;
    /// hardware-gate-01 saw no all-lane pass for that reason).
    pub en_pass: [u128; 2],
    /// Per byte and lane: bit i set when the lane passed at the receive
    /// clock trained + [`clk_off`]`(i)`.
    pub clk_pass: [[u128; LANES]; 2],
    /// Everything read back as found after the restore.
    pub restored: bool,
    pub launches: u16,
    pub failure: Option<WlFailure>,
}

impl VendorCmp {
    pub const EMPTY: VendorCmp = VendorCmp {
        state_before: [0; 8],
        state_at_launch: [0; 8],
        clocks: [0; 2],
        dfiupd0: [0; 2],
        raw: [(0, 0, 0); RAW_MAX],
        n_raw: 0,
        aborted: false,
        trained_rx_en: [0; 2],
        trained_clk: [[[0; LANES]; 4]; 2],
        read_path: ReadPath::EMPTY,
        check: DataCheck::EMPTY,
        trained: (0, 0, [0; 2]),
        misc: [0; 3],
        en_pass: [0; 2],
        clk_pass: [[0; LANES]; 2],
        restored: false,
        launches: 0,
        failure: None,
    };

    fn raw(&mut self, csr: u32, want: u16, got: u16) {
        let n = self.n_raw as usize;
        if n < RAW_MAX {
            self.raw[n] = (csr, want, got);
            self.n_raw += 1;
        }
        if want != got {
            self.aborted = true;
        }
    }

    /// The read-after-write records whose read differs from the write.
    pub fn raw_mismatches(&self) -> impl Iterator<Item = &(u32, u16, u16)> {
        self.raw[..self.n_raw as usize]
            .iter()
            .filter(|r| r.1 != r.2)
    }
}

fn state<H: Hw>(ex: &mut WlExec<'_, H>) -> [u16; 8] {
    [
        ex.read(PUBMODE),
        ex.read(MEM_RESET_L),
        ex.hw.read16(crate::phy::MICRO_RESET),
        ex.hw.read16(crate::phy::MICRO_CONT_MUX_SEL),
        ex.hw.read32(crate::ctrl::ddrc(0, STAT)) as u16,
        ex.hw.read32(crate::ctrl::ddrc(1, STAT)) as u16,
        (ex.hw.read32(crate::ctrl::ddrc(0, DFIUPD0)) >> 16) as u16,
        (ex.hw.read32(crate::ctrl::ddrc(1, DFIUPD0)) >> 16) as u16,
    ]
}

/// DFIUPD0 of controller `ch` to `v` inside the SWCTL/SWSTAT handshake.
fn set_dfiupd0<H: Hw>(hw: &mut H, ch: usize, v: u32) {
    hw.write32(crate::ctrl::ddrc(ch, SWCTL), 0);
    hw.write32(crate::ctrl::ddrc(ch, DFIUPD0), v);
    hw.write32(crate::ctrl::ddrc(ch, SWCTL), 1);
    for _ in 0..1000 {
        if hw.read32(crate::ctrl::ddrc(ch, SWSTAT)) & 1 != 0 {
            break;
        }
        hw.udelay(1);
    }
}

/// A write that is read back into `out` (a mismatch aborts the compare).
fn w<H: Hw>(ex: &mut WlExec<'_, H>, out: &mut VendorCmp, c: u32, v: u16) {
    ex.write(c, v);
    let got = ex.read(c);
    out.raw(c, v, got);
}

/// Latch pulse `k` of [`EVENTS`] with forced writes (the restore path).
fn pulse_force<H: Hw>(ex: &mut WlExec<'_, H>, dbs: [u32; 2], k: usize) {
    let (wr, rd, bits) = EVENTS[k];
    let rd = if rd >> 12 == 0x10 {
        rd + 0x1000 * dbs[0]
    } else {
        rd
    };
    let found = ex.read(rd);
    ex.write_force(wr, found | bits);
    ex.hw.udelay(HOLD_US);
    ex.write_force(wr, found);
    ex.hw.udelay(HOLD_US);
}

/// The trained receive clock registers of every lane, forced writes.
fn put_clk<H: Hw>(ex: &mut WlExec<'_, H>, dbs: [u32; 2], clk: &[[[u16; LANES]; 4]; 2]) {
    for (i, &db) in dbs.iter().enumerate() {
        for (k, &off) in RX_CLK.iter().enumerate() {
            for (lane, &v) in clk[i][k].iter().enumerate() {
                ex.write_force(lane_csr(db, lane as u32, off), v);
            }
        }
    }
}

/// Run the compare on `channel`/`rank` in the vendor's trained state
/// (module documentation).
#[inline(never)]
pub fn compare<H: Hw, const EYE: bool>(
    hw: &mut H,
    channel: Channel,
    rank: Rank,
    out: &mut VendorCmp,
) {
    *out = VendorCmp::EMPTY;
    let dbs = dbytes(channel);
    let mut ex = WlExec::new(hw);

    // Found state, the clocks first.
    out.clocks[0] = ex.read(UCCLK_HCLK);
    out.state_before = state(&mut ex);
    for ch in 0..2 {
        out.dfiupd0[ch] = ex.hw.read32(crate::ctrl::ddrc(ch, DFIUPD0));
    }
    out.read_path = read_path(&mut ex, channel);
    let micro = out.state_before[2];
    let pubmode = out.state_before[0];
    let saved_param: [u16; 4] = core::array::from_fn(|db| ex.read(param(db as u32)));
    let data_saved = read_data_saved(&mut ex);
    let lane_ctl0 = [
        ex.read(0x100b5 + 0x1000 * dbs[0]),
        ex.read(0x100b5 + 0x1000 * dbs[1]),
    ];
    let vis = [ex.read(fifo_vis(dbs[0])), ex.read(fifo_vis(dbs[1]))];
    for (i, &db) in dbs.iter().enumerate() {
        out.trained_rx_en[i] = ex.read(rx_en(db, rank));
        for (k, &off) in RX_CLK.iter().enumerate() {
            for (lane, v) in out.trained_clk[i][k].iter_mut().enumerate() {
                *v = ex.read(lane_csr(db, lane as u32, off));
            }
        }
    }

    // Quiesce and take over: no controller DFI updates, the training
    // clocks on, the PHY takes the DFI, the PMU stalled; the launch guard
    // accepts the MemResetL found.
    for ch in 0..2 {
        set_dfiupd0(ex.hw, ch, out.dfiupd0[ch] | DIS_AUTO_CTRLUPD);
    }
    w(&mut ex, out, UCCLK_HCLK, out.clocks[0] | 3);
    out.clocks[1] = ex.read(UCCLK_HCLK);
    w(&mut ex, out, PUBMODE, 1);
    if micro != 1 {
        ex.hw.write16(crate::phy::MICRO_RESET, 1);
    }
    ex.mem_reset_want = ex.read(MEM_RESET_L);
    out.state_at_launch = state(&mut ex);

    // The data set-up, every write that matters read back, before any
    // launch: TrainingParam 0, the PHY's expected data and compare mode,
    // DtsmLaneCtrl0, the FIFO visibility.
    let want = expected(CHECK_PATTERNS[0]);
    ex.write(0x1f0a7, 0);
    let got = ex.read(param(dbs[0]));
    out.raw(param(dbs[0]), 0, got);
    // Bit 4 of 0x70011 is clear while the firmware loads the words and set
    // after (the vendor state holds 0x10); the compare does the same.
    out.misc = data_saved.misc;
    ex.write(0x7f011, data_saved.misc[0] & !0x10);
    let got = ex.read(0x70011);
    out.raw(0x70011, data_saved.misc[0] & !0x10, got);
    pattern_on(dbs, want, &mut |s| ex.step(s));
    // The words are per-lane registers (0x70n24..2b, lanes 0-8); pattern_on
    // writes them through the block's lane-F broadcast 0x70f2x, which reads
    // back 0 (reading the broadcast address back stopped the compare). Read lane 0 and lane 8.
    for c in [0x70024u32, 0x7002b, 0x70824, 0x7082b] {
        let got = ex.read(c);
        out.raw(c, want, got);
    }
    let got = ex.read(0x70060);
    out.raw(0x70060, 0x85, got);
    for &db in &dbs {
        ex.write(lane_f(db, 0xb5), LANE_CTL[0]);
        let got = ex.read(0x100b5 + 0x1000 * db);
        out.raw(0x100b5 + 0x1000 * db, LANE_CTL[0], got);
        w(&mut ex, out, fifo_vis(db), VIS_DFE0);
    }

    if !out.aborted {
        let prog = rxen::centring_program(rank);
        // The entry (MR33/MR34) and the data check at the trained point,
        // then the MRR read-out.
        ex.step(Step::Launch {
            rows: &rxen::entry(rank, EntryMrs::READ_CENTRING_PSTATE1),
            outer: 0,
        });
        if EYE {
            let (m, _, n) = point_counts(&mut ex, dbs, &prog);
            out.trained = (n[0], n[1], m);
        } else {
            data_check(&mut ex, dbs, rank, &prog, CHECK_N, &mut out.check);
        }
        // The gate window around the trained RxEnDly (latched).
        for i in 0..if EYE { 0 } else { EN_N } {
            if ex.failure.is_some() {
                break;
            }
            let en = core::array::from_fn(|b| {
                (out.trained_rx_en[b] as i32 + en_off(i) as i32).clamp(0, 0x7ff) as u16
            });
            set_en(&mut ex, dbs, rank, en);
            let (m, _, _) = point_counts(&mut ex, dbs, &prog);
            for (p, &mb) in out.en_pass.iter_mut().zip(m.iter()) {
                if mb & DQ_LANES == DQ_LANES {
                    *p |= 1 << i;
                }
            }
        }
        set_en(&mut ex, dbs, rank, out.trained_rx_en);
        // The per-lane eye around the trained receive clocks (all four
        // registers per lane, latched).
        for i in 0..if EYE { CLK_N } else { 0 } {
            if ex.failure.is_some() {
                break;
            }
            for (k, &off) in RX_CLK.iter().enumerate() {
                let mut eff = [[0u16; LANES]; 2];
                for (b, lanes) in eff.iter_mut().enumerate() {
                    for (lane, e) in lanes.iter_mut().enumerate() {
                        let t = clk_eff(out.trained_clk[b][k][lane]) as i32 + clk_off(i) as i32;
                        *e = t.clamp(0, CLK_EFF_MAX as i32) as u16;
                    }
                }
                set_lanes(&mut ex, dbs, off, &eff);
            }
            pulse_force(&mut ex, dbs, EV_UPDATE);
            let (m, _, _) = point_counts(&mut ex, dbs, &prog);
            for (lanes, &mb) in out.clk_pass.iter_mut().zip(m.iter()) {
                for (lane, p) in lanes.iter_mut().enumerate() {
                    if mb >> lane & 1 != 0 {
                        *p |= 1 << i;
                    }
                }
            }
        }
        // MR33/MR34 back to 0 (a launch: only without a failure).
        ex.step(Step::Launch {
            rows: &rxen::entry(rank, ALL_ZERO),
            outer: 0,
        });
    }
    out.launches = ex.launches;
    out.failure = ex.failure;

    // Back, with forced writes: the trained delays and their latches, the
    // pattern set-up, lane controls, visibility, TrainingParam, PUBMODE,
    // the PMU, the clocks, DFIUPD0 as found.
    put_clk(&mut ex, dbs, &out.trained_clk);
    pulse_force(&mut ex, dbs, EV_UPDATE);
    for (i, &db) in dbs.iter().enumerate() {
        ex.write_force(rx_en(db, rank), out.trained_rx_en[i]);
    }
    pulse_force(&mut ex, dbs, EV_RXEN);
    pulse_force(&mut ex, dbs, EV_FIFO);
    pattern_off(dbs, &data_saved, &mut |s| match s {
        Step::Write(c, v) => ex.write_force(c, v),
        Step::Launch { .. } => {}
    });
    for (i, &db) in dbs.iter().enumerate() {
        ex.write_force(lane_f(db, 0xb5), lane_ctl0[i]);
        ex.write_force(fifo_vis(db), vis[i]);
    }
    for db in 0..4u32 {
        ex.write_force(param(db), saved_param[db as usize]);
    }
    let mut ok = true;
    for (i, &db) in dbs.iter().enumerate() {
        ok &= ex.read(rx_en(db, rank)) == out.trained_rx_en[i];
        ok &= ex.read(fifo_vis(db)) == vis[i];
        for (k, &off) in RX_CLK.iter().enumerate() {
            for (lane, &t) in out.trained_clk[i][k].iter().enumerate() {
                ok &= ex.read(lane_csr(db, lane as u32, off)) == t;
            }
        }
    }
    for db in 0..4u32 {
        ok &= ex.read(param(db)) == saved_param[db as usize];
    }
    ex.write_force(PUBMODE, pubmode);
    ok &= ex.read(PUBMODE) == pubmode;
    if micro != 1 {
        ex.hw.write16(crate::phy::MICRO_RESET, micro);
    }
    ex.write_force(UCCLK_HCLK, out.clocks[0]);
    ok &= ex.read(UCCLK_HCLK) == out.clocks[0];
    for ch in 0..2 {
        set_dfiupd0(ex.hw, ch, out.dfiupd0[ch]);
        ok &= ex.hw.read32(crate::ctrl::ddrc(ch, DFIUPD0)) == out.dfiupd0[ch];
    }
    out.restored = ok;
    csr_access(hw, false);
}

/// The widths of the gate window and of every lane's eye, in their steps:
/// (first, last) offsets of the longest passing run, None when none.
pub fn window(mask: u128, n: usize, off: fn(usize) -> i16) -> Option<(i16, i16)> {
    crate::rdtrain::longest_run(mask, n).map(|(a, b)| (off(a), off(b)))
}
