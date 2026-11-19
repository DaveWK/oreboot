// SPDX-License-Identifier: GPL-2.0-only
//! Firmware-free bring-up of the DRAM PHY: the open pre-training sequence
//! with no PHY firmware loaded or run, plus read-only probes of the PHY.
//!
//! This is the base of the blobless path (no PMU training firmware, no PIE
//! images). It currently stops before training: [`pre_training`] leaves the
//! controllers configured, the PHY programmed for one pstate (PhyInit steps
//! C and E and the CA/DQ swizzle, without D/F/G/I) and the PMU held in
//! reset/stall. DRAM is not usable afterwards.

use crate::hw::{Error, Hw};
use crate::para::Para;
use crate::phy::prog::{self, Kind, Op};
use crate::phy::{
    coldinit, csr, csr_access, devinit, freq, init, sequence, tables, MICRO_CONT_MUX_SEL,
    MICRO_RESET,
};
use crate::{ccu, ctrl, power};

/// MASTER PhyConfig (synthesis configuration word; field layout per the
/// matching EIC7700X register family, see `phy::sequence`).
pub const PHY_CONFIG: u32 = 0x200a4;
/// PUB revision (DRTUB block).
pub const PUB_REV: u32 = 0xc00ee;

/// Identity read of the PHY, before any PMU release.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Identity {
    /// MicroContMuxSel and MicroReset as found (before this probe wrote them).
    pub mux_before: u16,
    pub micro_reset_before: u16,
    pub phy_config: u16,
    pub pub_rev: u16,
}

/// Write-back check of one PHY program: how many plain CSR writes read back
/// the written value. Mismatches are data (self-clearing, write-only or
/// pstate-banked CSRs), not failures.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Readback {
    pub checked: u16,
    pub matched: u16,
    /// First mismatches: (CSR, written, read).
    pub first: [(u32, u16, u16); 8],
    pub mismatches: u16,
}

/// Same validation as the trained path, then rails, clocks, controllers and
/// PHY steps C (once and for `pstate`) and E plus the CA/DQ swizzle. No PHY
/// firmware is written; the PMU is left in reset and stalled (MicroReset 9
/// then 1) with the CSR mux returned to the microcontroller side, as the
/// vendor sequence leaves it before loading DMEM.
pub fn pre_training<H: Hw>(
    hw: &mut H,
    p: &mut Para,
    pstate: u32,
    readback: &mut [Readback; 2],
) -> Result<Identity, Error> {
    if !p.is_lpddr5() {
        return Err(Error::Config("only LPDDR5 (dram_type 9) is supported"));
    }
    if pstate >= 4 {
        return Err(Error::Config("pstate outside 0..4"));
    }
    ccu::validate_pll_ddr_rate(p.clk())?;
    init::validate(p)?;
    ctrl::validate_geometry(p)?;
    ccu::pad_calibration(hw);
    let vddq = power::set_rails(hw, p)?;
    hw.log(format_args!("[dram] blobless: VDDQ {vddq} mV\n"));
    crate::controller_pre_phy(hw, p)?;
    // PHY clocks and resets are now up (the vendor sequence writes CSRs from
    // here); nothing has written the APB-only block yet.
    let mux_before = hw.read16(MICRO_CONT_MUX_SEL);
    let micro_reset_before = hw.read16(MICRO_RESET);

    let none: [&[u8]; 4] = [&[]; 4];
    let c0 = init::ctx_with(p, none, 0);
    run_checked(hw, tables::C_ONCE, &c0, &mut readback[0]);
    let c = init::ctx_with(p, none, pstate);
    run_checked(hw, tables::C_PSTATE, &c, &mut readback[1]);
    init::set_dfi_clk(hw, p, pstate)?;
    csr_access(hw, true);
    prog::run(hw, tables::CA_REMAP, &c, freq::config);
    prog::run(hw, tables::DQ_REMAP, &c, freq::config);

    // Hold the PMU: reset + stall, then stall only (the state the vendor
    // sequence establishes before release). Never write 0 here.
    hw.write16(MICRO_RESET, 9);
    hw.write16(MICRO_RESET, 1);
    let phy_config = hw.read16(csr(PHY_CONFIG));
    let pub_rev = hw.read16(csr(PUB_REV));
    csr_access(hw, false);
    Ok(Identity {
        mux_before,
        micro_reset_before,
        phy_config,
        pub_rev,
    })
}

/// Run `prog` (which must contain no firmware blob), then read back each of
/// its plain, non-computed CSR writes once with CPU CSR access.
fn run_checked<H: Hw>(hw: &mut H, prog: &[Op], c: &prog::Ctx, out: &mut Readback) {
    prog::run(hw, prog, c, freq::config);
    // Leave the mux as the program found it: the vendor order runs the next
    // program without an intervening mux write.
    let mux = hw.read16(MICRO_CONT_MUX_SEL);
    csr_access(hw, true);
    *out = Readback::default();
    for op in prog {
        if op.kind != Kind::Write
            || op.flags & (prog::COMPUTED | prog::PSTATE_INDEXED | prog::DUAL_RANK) != 0
        {
            continue;
        }
        let mut want = op.val as u32;
        if !c.dual_rank {
            want ^= op.xr;
        }
        if c.slow_bin {
            want ^= op.xs;
        }
        let want = want as u16;
        let got = hw.read16(csr(op.csr));
        out.checked += 1;
        if got == want {
            out.matched += 1;
        } else {
            if (out.mismatches as usize) < out.first.len() {
                out.first[out.mismatches as usize] = (op.csr, want, got);
            }
            out.mismatches += 1;
        }
    }
    hw.write16(MICRO_CONT_MUX_SEL, mux);
}

/// DevInit's mode register values for `pstate` from our own parameters:
/// the message block (`phy::msgblock::build`, as for the trained path)
/// through the firmware's per-bit rules ([`coldinit::MrInputs`]). Call
/// after [`pre_training`] (its timing step fixes up MR1/MR2).
pub fn mode_registers(p: &mut Para, pstate: u32) -> (coldinit::Mrs, coldinit::Mrs2) {
    let m = message_block(p, pstate);
    (
        coldinit::Mrs::from_msgblock(&m),
        coldinit::Mrs2::from_msgblock(&m),
    )
}

/// The message block for `pstate` from our parameters, as the trained path
/// builds it (`phy::msgblock::build`): the source of the mode register
/// inputs. Nothing is loaded into the PHY.
pub fn message_block(p: &mut Para, pstate: u32) -> [u16; crate::phy::msgblock::WORDS] {
    let c = init::ctx_with(p, [&[]; 4], pstate);
    let rl = if p.tpr10() & (1 << 31) != 0 { 14 } else { 13 };
    crate::phy::msgblock::build(p, pstate, c.data_mhz, rl)
}

/// FEL lab, measurement only: the MR18 base of WCK2CK leveling and
/// read training on every block instead of the message block's, from env
/// `A7A_HYBRID_WL_MR18` (hex) at build time. The leveling's last MRW leaves
/// MR18 = base in the DRAM's FSP-WR set; after our end of initialization
/// FSP-WR is the operating set (MR16 0x45), in the PMU-trained state
/// without it possibly not.
/// (`crate::knobs`: runtime in the bt0 DRAM lab mode, else the build's.)
fn wl_mr18() -> Option<u8> {
    crate::knobs::get().wl_mr18
}

/// WCK2CK leveling MR18 bases per [`crate::wl::BLOCKS`] entry, from the
/// message block (`wcklevel::Mr18Base::from_message_block`), or the FEL
/// lab's `A7A_HYBRID_WL_MR18`.
pub fn wl_mr18_bases(p: &mut Para, pstate: u32) -> [Option<crate::phy::wcklevel::Mr18Base>; 4] {
    let m = message_block(p, pstate);
    let mut out = [None; 4];
    for (o, &(ch, rank)) in out.iter_mut().zip(crate::wl::BLOCKS.iter()) {
        *o = match wl_mr18() {
            Some(v) => crate::phy::wcklevel::Mr18Base::new(v),
            None => crate::phy::wcklevel::Mr18Base::from_message_block(&m, ch, rank),
        };
    }
    out
}

/// Measurement over FEL: mode-register values for the end of initialization
/// instead of the message block's, from env `A7A_BL_MRS` at build time:
/// comma-separated `mr=hex` pairs. The message block carries MR2 = 0x88
/// (`timing::mr_fixup`, the 2400 MT/s point) for every pstate; the training
/// firmware programs 0x00 at pstate 1 (read back on hardware, and MR2 0x88
/// at pstate 1 breaks the controller's reads in the PMU-trained state).
fn mission_mr_overrides(f: &mut crate::phy::endinit::FspMrs) {
    let Some(list) = crate::knobs::get().bl_mrs else {
        return;
    };
    for &(a, v) in list.iter() {
        for slot in f.mr.iter_mut() {
            if slot.0 == a {
                slot.1 = v;
            }
        }
    }
}

/// FEL lab bisect of the end of initialization, measurement only:
/// the [`crate::phy::endinit::Variant`] from env at build time,
/// `A7A_HYBRID_EI_SKIP` (hex `endinit::skip` bits), `A7A_HYBRID_EI_NO_MRW`
/// (decimal MAs, comma-separated, or `all`) and `A7A_HYBRID_EI_MR16`
/// (`write,switch` hex; either empty keeps the reference's). Unset: the
/// reference stage. Applies wherever [`end_of_init`] runs (feature
/// `hybrid` with `A7A_HYBRID_ENDINIT`, and the blobless flow).
/// The fix candidate on top: env `A7A_EI_FSPWR0` = `04` or `44` ends the
/// stage with FSP-WR 0 (`endinit::MR16_SWITCH_FSPWR0[_VRCG]`), as the
/// training firmware leaves it; it cannot be combined with a switch value
/// from `A7A_HYBRID_EI_MR16`.
/// The build's variant (`crate::knobs::BUILD`); the stage runs
/// `crate::knobs::get().ei` (the bt0 DRAM lab mode can override it).
pub const END_OF_INIT_VARIANT: crate::phy::endinit::Variant = crate::knobs::BUILD.ei;

/// MRW MR16 = `mr16` to both channels and ranks at once, through the end
/// of initialization's own switch launch (`endinit::run_with` with the
/// per-rank launches and the CBT exit controls left out; the remaining
/// channel-select and clock CSR writes ran harmlessly on the PMU-trained
/// state, as a bisect run showed). FEL lab: opens or closes an FSP-WR =
/// FSP-OP window for a stage whose MRWs must reach the operating set.
pub fn mr16_all<H: Hw>(hw: &mut H, mr16: u8) -> Option<crate::wl::WlFailure> {
    use crate::phy::endinit::{self, skip, FspMrs, Variant};
    use crate::phy::wcklevel::Channel;
    let v = Variant {
        skip: skip::RANK_LAUNCH | skip::CBT_EXIT,
        mr16_switch: mr16,
        ..Variant::REFERENCE
    };
    let mut ex = crate::wl::WlExec::new(hw);
    endinit::run_with(
        Channel::B,
        |_, _| FspMrs { mr: [(0, 0); 20] },
        &v,
        &mut |s| ex.step(s),
    );
    let f = ex.failure;
    csr_access(hw, false);
    f
}

/// The results a mission hand-over needs, collected in one place so that a
/// failure of any single step leaves the gate unset. Fail-closed: the gate is
/// the conjunction of every field, and the write stage's own verdict is one
/// of them.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct StageResults {
    /// The read training applied on every block.
    pub read_training: bool,
    /// The MR16 window write before the read training reached every rank.
    pub mr16_before: bool,
    /// The MR16 window write after the read training reached every rank.
    pub mr16_after: bool,
    /// The lean hand-over succeeded.
    pub handover: bool,
    /// The write stage's final compare was clean.
    pub write_stage: bool,
}

impl StageResults {
    /// No mode-register step was requested: only the read training and the
    /// hand-over are required.
    pub const fn new() -> Self {
        Self {
            read_training: false,
            mr16_before: true,
            mr16_after: true,
            handover: false,
            write_stage: false,
        }
    }

    /// Record the MR16 window write before the read training: `None` is a
    /// failure, and a failure here blocks the hand-over.
    pub const fn mr16_before_result(mut self, failure: Option<crate::wl::WlFailure>) -> Self {
        self.mr16_before = failure.is_none();
        self
    }

    /// Record the MR16 window write after the read training.
    pub const fn mr16_after_result(mut self, failure: Option<crate::wl::WlFailure>) -> Self {
        self.mr16_after = failure.is_none();
        self
    }

    /// Record whether the read training applied on every block.
    pub const fn read_training_result(mut self, applied: bool) -> Self {
        self.read_training = applied;
        self
    }

    /// Record whether the lean hand-over succeeded.
    pub const fn handover_result(mut self, ok: bool) -> Self {
        self.handover = ok;
        self
    }

    /// Record the write stage's verdict.
    pub const fn write_stage_result(mut self, clean: bool) -> Self {
        self.write_stage = clean;
        self
    }

    /// Every required step completed; only then may the mission gate be set.
    pub const fn mission_ok(&self) -> bool {
        self.read_training
            && self.mr16_before
            && self.mr16_after
            && self.handover
            && self.write_stage
    }
}

const fn hex_nib(c: u8) -> Option<u8> {
    match c {
        b'0'..=b'9' => Some(c - b'0'),
        b'a'..=b'f' => Some(c - b'a' + 10),
        b'A'..=b'F' => Some(c - b'A' + 10),
        _ => None,
    }
}

/// Two hex bytes `a,b` (FEL lab knob text); `None` for anything else.
pub const fn hex_pair(s: &str) -> Option<(u8, u8)> {
    let b = s.as_bytes();
    if b.len() != 5 || b[2] != b',' {
        return None;
    }
    match (hex_nib(b[0]), hex_nib(b[1]), hex_nib(b[3]), hex_nib(b[4])) {
        (Some(a), Some(b2), Some(c), Some(e)) => Some((a * 16 + b2, c * 16 + e)),
        _ => None,
    }
}

/// FEL lab `A7A_RD_FSPWR=<before>,<after>` (hex MR16 values): an MR16 write
/// to all ranks before the read training and another after it, so that its
/// mode MRWs (MR20, MR18, MR10, MR46) reach the operating set-point; e.g.
/// `05,04` after the FSP-WR 0 end of initialization. Unset: none.
#[allow(non_snake_case)]
pub fn RD_FSPWR() -> Option<(u8, u8)> {
    match crate::knobs::env("A7A_RD_FSPWR", crate::knob!("A7A_RD_FSPWR")) {
        Some(s) => match hex_pair(s) {
            Some(p) => Some(p),
            None => panic!("A7A_RD_FSPWR is two hex bytes: before,after"),
        },
        None => None,
    }
}

/// The end-of-initialization stage's run ([`end_of_init`]).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EndInitReport {
    /// The variant that ran ([`END_OF_INIT_VARIANT`]).
    pub variant: crate::phy::endinit::Variant,
    /// MR values per [`crate::phy::endinit::BLOCKS`] entry.
    pub mrs: [crate::phy::endinit::FspMrs; 4],
    pub launches: u16,
    pub writes: u16,
    pub max_launch_us: u32,
    pub elapsed_us: u32,
    pub failure: Option<crate::wl::WlFailure>,
}

/// The reference's end of initialization after CBT and before WL
/// ([`crate::phy::endinit`]): the target FSP's mode registers from our
/// message block to every channel and rank, then the switch to that FSP.
/// `prev` is the channel CBT ended on (channel B: the last CBT block).
#[inline(never)]
pub fn end_of_init<H: Hw>(hw: &mut H, p: &mut Para, pstate: u32) -> EndInitReport {
    use crate::phy::endinit::{self, FspMrs};
    use crate::phy::wcklevel::Channel;
    let variant = crate::knobs::get().ei;
    let m = message_block(p, pstate);
    let mut mrs = [FspMrs { mr: [(0, 0); 20] }; 4];
    for (o, &(ch, rank)) in mrs.iter_mut().zip(endinit::BLOCKS.iter()) {
        *o = FspMrs::from_message_block(&m, ch, rank);
    }
    for o in mrs.iter_mut() {
        mission_mr_overrides(o);
    }
    let t0 = hw.now_us();
    let mut ex = crate::wl::WlExec::new(hw);
    endinit::run_with(
        Channel::B,
        |ch, rank| {
            let mut f = FspMrs::from_message_block(&m, ch, rank);
            mission_mr_overrides(&mut f);
            f
        },
        &variant,
        &mut |s| ex.step(s),
    );
    let r = EndInitReport {
        variant,
        mrs,
        launches: ex.launches,
        writes: ex.writes,
        max_launch_us: ex.max_launch_us,
        elapsed_us: 0,
        failure: ex.failure,
    };
    csr_access(hw, false);
    EndInitReport {
        elapsed_us: hw.now_us().wrapping_sub(t0) as u32,
        ..r
    }
}

/// ACSM sequence memory: row `r` is four halfwords at CSR 0x41000 + 4 r.
pub const ACSM_MEM: u32 = 0x41000;
pub const ACSM_START: u32 = 0x20022;
pub const ACSM_STOP: u32 = 0x20023;
pub const ACSM_LAST: u32 = 0x20024;
pub const ACSM_OUTER_LOOP: u32 = 0x20027;
/// MemResetL: [1] ProtectMemReset, [0] BP_MemReset_L. 2 keeps DRAM in reset.
pub const MEM_RESET_L: u32 = 0x20060;

/// [`sequence::Port`] on the PHY CSRs for the firmware-free path. Preflight
/// requires the PMU stalled (MicroReset 1) and MemResetL at `mem_reset`
/// (2: DRAM held in reset, as [`pre_training`] leaves it; 3: released); it
/// then selects host CSR access and records ACSMLastAddr. The completion
/// witness is ACSMLastAddr reaching `stop` from a different value.
pub struct PhyPort<'a, H: Hw> {
    pub hw: &'a mut H,
    pub stop: u16,
    pub prior_last: u16,
    pub mem_reset: u16,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PortError {
    PmuNotHeld(u16),
    DramNotInReset(u16),
}

impl<H: Hw> sequence::Port for PhyPort<'_, H> {
    type Error = PortError;
    fn preflight(&mut self) -> Result<(), PortError> {
        let micro = self.hw.read16(MICRO_RESET);
        if micro != 1 {
            return Err(PortError::PmuNotHeld(micro));
        }
        csr_access(self.hw, true);
        let reset = self.hw.read16(csr(MEM_RESET_L));
        if reset != self.mem_reset {
            csr_access(self.hw, false);
            return Err(PortError::DramNotInReset(reset));
        }
        self.prior_last = self.hw.read16(csr(ACSM_LAST));
        Ok(())
    }
    fn read_control(&mut self) -> Result<u16, PortError> {
        Ok(self.hw.read16(csr(sequence::RUN_CTRL)))
    }
    fn write_control(&mut self, value: u16) -> Result<(), PortError> {
        self.hw.write16(csr(sequence::RUN_CTRL), value);
        Ok(())
    }
    fn read_done(&mut self) -> Result<u16, PortError> {
        Ok(self.hw.read16(csr(sequence::DONE)))
    }
    fn completion_witness(&mut self) -> Result<bool, PortError> {
        let last = self.hw.read16(csr(ACSM_LAST));
        Ok(last == self.stop && self.prior_last != self.stop)
    }
}

/// One sample of [`acsm_delay_probe`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AcsmSample {
    /// Control halfword of the delay row.
    pub ctl: u16,
    /// Stop row: alternates 1 and 2 so ACSMLastAddr can witness each run.
    pub stop: u16,
    pub run: Result<sequence::Completion, sequence::Failure<PortError>>,
    pub last_addr: u16,
    pub elapsed_us: u64,
}

/// Control words of the delay-only rows of the reference's DevInit
/// launches, shortest-looking first.
pub const DELAY_CONTROLS: [u16; 8] = [
    0x0400, 0x0800, 0x1b00, 0x2b00, 0x4800, 0x4b00, 0x5b00, 0x7b00,
];

/// Run ACSM sequences of one delay-only row (no command, CS idle) followed by
/// empty rows, as in the firmware's first DevInit launch (Start 0, Stop 1),
/// for each of [`DELAY_CONTROLS`] while DRAM stays in reset, timing each.
/// The stop row alternates 1 and 2 so that ACSMLastAddr changes on every
/// run. A failed sample stops the series. Call after [`pre_training`].
pub fn acsm_delay_probe<H: Hw>(hw: &mut H, out: &mut [Option<AcsmSample>; 8]) {
    *out = [None; 8];
    for (i, (slot, &ctl)) in out.iter_mut().zip(DELAY_CONTROLS.iter()).enumerate() {
        let stop = 1 + (i as u16 & 1);
        csr_access(hw, true);
        hw.write16(csr(ACSM_START), 0);
        hw.write16(csr(ACSM_STOP), 0);
        let rows: [[u16; 4]; 3] = [[0, 0, 0, ctl], [0; 4], [0; 4]];
        for (r, row) in rows.iter().enumerate() {
            for (k, &v) in row.iter().enumerate() {
                hw.write16(csr(ACSM_MEM + 4 * r as u32 + k as u32), v);
            }
        }
        hw.write16(csr(ACSM_OUTER_LOOP), 0);
        hw.write16(csr(ACSM_STOP), stop);
        let t0 = hw.now_us();
        let mut run = sequence::DirectRun::new(PhyPort {
            hw: &mut *hw,
            stop,
            prior_last: 0,
            mem_reset: 2,
        });
        let result = run.run(sequence::MAX_POLLS);
        let elapsed_us = hw.now_us().wrapping_sub(t0);
        csr_access(hw, true);
        let last_addr = hw.read16(csr(ACSM_LAST));
        csr_access(hw, false);
        let ok = result.is_ok();
        *slot = Some(AcsmSample {
            ctl,
            stop,
            run: result,
            last_addr,
            elapsed_us,
        });
        if !ok {
            break;
        }
    }
}

/// ClrPORMemReset: switches the power-on-reset latch (written 1 then 0).
pub const CLR_POR_MEM_RESET: u32 = 0x20061;
/// DRAM reset low time before release (JEDEC tINIT1 is 200 us minimum; the
/// reset has been low since the rails came up much earlier).
pub const RESET_LOW_US: u32 = 200;
/// Wait after releasing reset before the first command (tINIT3 >= 2 ms).
pub const RESET_HIGH_US: u32 = 5_000;
/// Row bases used alternately so that ACSMLastAddr changes on every launch.
const ROW_BASES: [u16; 2] = [0, 64];

/// Why an execution stopped.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ExecFailure {
    /// A blocking launch failed (1-based launch number).
    Launch(u16, sequence::Failure<PortError>),
    /// A loop launch found Run already set.
    LoopBusy(u16),
    /// A loop did not report Done within the poll budget after OLRC cleared.
    LoopEnd(u16),
    /// DevInit before reset release failed.
    Devinit(devinit::DevinitError),
}

/// Outcome of a cold-init execution.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ColdInitReport {
    pub launches: u16,
    pub writes: u16,
    pub witnessed: u16,
    pub busy_max: u16,
    pub failure: Option<ExecFailure>,
    pub elapsed_us: u64,
    /// AsyncDbyteRxData samples in program order.
    pub reads: [u16; 192],
    pub n_reads: u16,
}

impl ColdInitReport {
    pub const EMPTY: ColdInitReport = ColdInitReport {
        launches: 0,
        writes: 0,
        witnessed: 0,
        busy_max: 0,
        failure: None,
        elapsed_us: 0,
        reads: [0; 192],
        n_reads: 0,
    };
}

/// Runs [`coldinit::Step`]s on the PHY: blocking launches through
/// [`sequence::DirectRun`] (row base alternating 0/64 for launches at row 0,
/// so ACSMLastAddr witnesses each), loop launches without waiting, loop ends
/// with a bounded Done poll, and reads. Stops at the first failure.
struct Executor<'a, H: Hw> {
    hw: &'a mut H,
    r: ColdInitReport,
    loop_run: u16,
    /// MemResetL state blocking launches expect (2 in reset, 3 released).
    mem_reset: u16,
}

impl<H: Hw> Executor<'_, H> {
    fn load(&mut self, start: u16, rows: &[coldinit::Row]) -> (u16, u16) {
        let base = if start == 0 {
            ROW_BASES[self.r.launches as usize & 1]
        } else {
            start
        };
        let stop = base + rows.len() as u16 - 1;
        csr_access(self.hw, true);
        self.hw.write16(csr(ACSM_START), base);
        self.hw.write16(csr(ACSM_STOP), base);
        for (i, row) in rows.iter().enumerate() {
            for (k, &v) in row.0.iter().enumerate() {
                let a = (base as u32 + i as u32) * 4 + k as u32;
                self.hw.write16(csr(ACSM_MEM + a), v);
            }
        }
        self.hw.write16(csr(ACSM_OUTER_LOOP), 0);
        self.hw.write16(csr(ACSM_STOP), stop);
        self.r.launches += 1;
        (base, stop)
    }

    fn step(&mut self, step: coldinit::Step) {
        if self.r.failure.is_some() {
            return;
        }
        match step {
            coldinit::Step::Write(c, v) => {
                csr_access(self.hw, true);
                self.hw.write16(csr(c), v);
                self.r.writes += 1;
            }
            coldinit::Step::Read(c) => {
                csr_access(self.hw, true);
                let v = self.hw.read16(csr(c));
                if (self.r.n_reads as usize) < self.r.reads.len() {
                    self.r.reads[self.r.n_reads as usize] = v;
                }
                self.r.n_reads += 1;
            }
            coldinit::Step::Launch { start, rows } => {
                let (_, stop) = self.load(start, rows);
                let mut run = sequence::DirectRun::new(PhyPort {
                    hw: &mut *self.hw,
                    stop,
                    prior_last: 0,
                    mem_reset: self.mem_reset,
                });
                match run.run(sequence::MAX_POLLS) {
                    Ok(c) => {
                        self.r.witnessed += c.witnessed as u16;
                        self.r.busy_max = self.r.busy_max.max(c.busy_polls);
                    }
                    Err(f) => self.r.failure = Some(ExecFailure::Launch(self.r.launches, f)),
                }
            }
            coldinit::Step::LaunchLoop { start, rows } => {
                self.load(start, rows);
                let control = self.hw.read16(csr(sequence::RUN_CTRL));
                if control & 1 != 0 {
                    self.r.failure = Some(ExecFailure::LoopBusy(self.r.launches));
                    return;
                }
                self.hw.write16(csr(sequence::RUN_CTRL), control | 1);
                self.loop_run = self.r.launches;
            }
            coldinit::Step::AwaitEnd => {
                csr_access(self.hw, true);
                let mut done = false;
                for _ in 0..sequence::MAX_POLLS {
                    if self.hw.read16(csr(sequence::DONE)) & 1 != 0 {
                        done = true;
                        break;
                    }
                }
                let control = self.hw.read16(csr(sequence::RUN_CTRL));
                self.hw.write16(csr(sequence::RUN_CTRL), control & !1);
                if !done {
                    self.r.failure = Some(ExecFailure::LoopEnd(self.loop_run));
                }
            }
        }
    }
}

/// Release the DRAM reset (ClrPORMemReset pulse, reset-low time, MemResetL
/// 3, tINIT3) and run `program` through an [`Executor`].
fn run_program<H: Hw>(hw: &mut H, program: impl FnOnce(&mut Executor<H>)) -> ColdInitReport {
    let t0 = hw.now_us();
    csr_access(hw, true);
    hw.write16(csr(CLR_POR_MEM_RESET), 1);
    hw.write16(csr(CLR_POR_MEM_RESET), 0);
    hw.udelay(RESET_LOW_US);
    hw.write16(csr(MEM_RESET_L), 3);
    hw.udelay(RESET_HIGH_US);
    let mut ex = Executor {
        hw: &mut *hw,
        r: ColdInitReport::EMPTY,
        loop_run: 0,
        mem_reset: 3,
    };
    program(&mut ex);
    let mut r = ex.r;
    csr_access(hw, false);
    r.elapsed_us = hw.now_us().wrapping_sub(t0);
    r
}

/// Release the DRAM reset, run cold-init part 1, part 2 up to CBT entry on
/// channel A, rank 0, and the CBT points `delays` (each: four CA patterns,
/// reading DBYTE 1's AsyncDbyteRxData after each). Call after
/// [`pre_training`]. Stops at the first failure.
pub fn cold_init_cbt<H: Hw>(
    hw: &mut H,
    m1: &coldinit::Mrs,
    m2: &coldinit::Mrs2,
    delays: &[u16],
) -> ColdInitReport {
    run_program(hw, |ex| {
        coldinit::part1(m1, &mut |s| ex.step(s));
        coldinit::part2(m2, &mut |s| ex.step(s));
        for &d in delays {
            coldinit::cbt_point(d, &mut |s| ex.step(s));
        }
    })
}

/// Diagnostic run: AsyncDbyteRxData of all four DBYTEs is sampled before the
/// DevInit prefix, after it (DRAM still in reset), after reset release and
/// part 1, after part 2 (CBT entered), and after every CBT pattern at each of
/// `delays`. [`devinit::pre_release`] (impedance and DLL set-up, RxReplica
/// calibration, CKE control) runs first, then the firmware's pre-release
/// delay launch. Call after [`pre_training`].
pub fn cold_init_diag<H: Hw>(
    hw: &mut H,
    m1: &coldinit::Mrs,
    m2: &coldinit::Mrs2,
    delays: &[u16],
) -> ColdInitReport {
    const ALL: [u32; 4] = [0, 1, 2, 3];
    let sample = |ex: &mut Executor<H>| {
        for n in ALL {
            ex.step(coldinit::Step::Read(coldinit::async_rx_data(n)));
        }
    };
    let t0 = hw.now_us();
    let mut ex = Executor {
        hw: &mut *hw,
        r: ColdInitReport::EMPTY,
        loop_run: 0,
        mem_reset: 3,
    };
    let mut dev = devinit::DevinitReport::default();
    bring_up_to_cbt(&mut ex, m1, m2, &mut dev, &mut |ex| sample(ex));
    for &d in delays {
        coldinit::cbt_point_bytes(d, &ALL, &mut |s| ex.step(s));
    }
    let mut r = ex.r;
    csr_access(hw, false);
    r.elapsed_us = hw.now_us().wrapping_sub(t0);
    r
}

/// Release the DRAM reset and run part 1 only.
pub fn cold_init_part1<H: Hw>(hw: &mut H, mrs: &coldinit::Mrs) -> ColdInitReport {
    run_program(hw, |ex| coldinit::part1(mrs, &mut |s| ex.step(s)))
}

/// Bring the DRAM from reset to command bus training on channel A, rank 0:
/// DevInit before reset release ([`devinit::pre_release`]), pre-release
/// delay launch, reset release,
/// parts 1 and 2. `checkpoint` runs before the prefix, after it, after part 1
/// and after part 2.
fn bring_up_to_cbt<H: Hw>(
    ex: &mut Executor<H>,
    m1: &coldinit::Mrs,
    m2: &coldinit::Mrs2,
    dev: &mut devinit::DevinitReport,
    checkpoint: &mut impl FnMut(&mut Executor<H>),
) {
    ex.mem_reset = 2;
    checkpoint(ex);
    devinit::pre_release(ex.hw, dev);
    if let Some(e) = dev.failure {
        ex.r.failure = Some(ExecFailure::Devinit(e));
        return;
    }
    let delay_row = [coldinit::Row::nop(0x1b00), coldinit::Row::EMPTY];
    ex.step(coldinit::Step::Launch {
        start: 0,
        rows: &delay_row,
    });
    checkpoint(ex);
    csr_access(ex.hw, true);
    ex.hw.write16(csr(CLR_POR_MEM_RESET), 1);
    ex.hw.write16(csr(CLR_POR_MEM_RESET), 0);
    ex.hw.udelay(RESET_LOW_US);
    ex.hw.write16(csr(MEM_RESET_L), 3);
    ex.hw.udelay(RESET_HIGH_US);
    ex.mem_reset = 3;
    coldinit::part1(m1, &mut |s| ex.step(s));
    checkpoint(ex);
    coldinit::part2(m2, &mut |s| ex.step(s));
    checkpoint(ex);
}

impl<H: Hw> Executor<'_, H> {
    /// Block `b`'s CA lane delays `delays`, then its four CBT patterns: its
    /// DBYTE's AsyncDbyteRxData after each, or None after a failure.
    fn cbt_measure(&mut self, b: &coldinit::CbtBlock, delays: &[u16; 7]) -> Option<[u16; 4]> {
        coldinit::set_block_delays(b.chan.ac, delays, &mut |s| self.step(s));
        let mut echo = [0u16; 4];
        let mut k = 0;
        coldinit::block_patterns(b, &[b.chan.dbyte], &mut |s| match s {
            coldinit::Step::Read(c) => {
                csr_access(self.hw, true);
                if k < 4 {
                    echo[k] = self.hw.read16(csr(c));
                }
                k += 1;
            }
            s => self.step(s),
        });
        if self.r.failure.is_some() {
            None
        } else {
            Some(echo)
        }
    }

    /// Our CA search on one block (the DRAM is in CBT for it): a common CA
    /// delay sweep gives each echo bit's window; each CA lane is then swept
    /// alone around the common center to find the echo bit it drives and its
    /// window; the per-lane centers are applied and verified.
    fn train_block(&mut self, b: &coldinit::CbtBlock, out: &mut BlockTrain) {
        let bits = b.chan.echo_bits();
        *out = BlockTrain::EMPTY;
        for i in 0..CA_SWEEP_POINTS {
            let d = i as u16 * CA_SWEEP_STEP;
            let Some(echo) = self.cbt_measure(b, &[d; 7]) else {
                return;
            };
            if i == 0 {
                out.raw_first = echo;
            }
            if i == CA_RAW_POINT {
                out.raw_mid = echo;
            }
            let pass = coldinit::cbt_pass_bits(echo, bits);
            for (bit, map) in out.bit_maps.iter_mut().enumerate() {
                if pass >> bit & 1 != 0 {
                    *map |= 1u128 << i;
                }
            }
        }
        let mut all = u128::MAX >> (128 - CA_SWEEP_POINTS);
        for bit in 0..8 {
            if bits >> bit & 1 != 0 {
                out.bit_runs[bit] = coldinit::longest_run(out.bit_maps[bit], CA_SWEEP_POINTS);
                all &= out.bit_maps[bit];
            }
        }
        out.common_run = coldinit::longest_run(all, CA_SWEEP_POINTS);
        let (s0, l0) = out.common_run;
        out.common_center = (((s0 + l0 / 2) % CA_SWEEP_POINTS) as u16) * CA_SWEEP_STEP;
        if l0 == 0 {
            return;
        }
        let full = u64::MAX >> (64 - LANE_SWEEP_POINTS);
        for lane in 0..7 {
            let mut maps = [0u64; 8];
            for i in 0..LANE_SWEEP_POINTS {
                let mut delays = [out.common_center; 7];
                delays[lane] = i as u16 * LANE_SWEEP_STEP;
                let Some(echo) = self.cbt_measure(b, &delays) else {
                    return;
                };
                let pass = coldinit::cbt_pass_bits(echo, bits);
                for (bit, map) in maps.iter_mut().enumerate() {
                    if pass >> bit & 1 != 0 {
                        *map |= 1u64 << i;
                    }
                }
            }
            let moved: u16 = (0..8)
                .filter(|&bit| bits >> bit & 1 != 0 && maps[bit] != full)
                .fold(0, |m, bit| m | 1 << bit);
            if moved.count_ones() == 1 {
                let bit = moved.trailing_zeros() as usize;
                out.lane_bit[lane] = bit as u8;
                out.lane_maps[lane] = maps[bit];
                // The window at the same period as the common center (the CA
                // delay passes once per 256 units).
                let pref = out.common_center as u32 / LANE_SWEEP_STEP as u32;
                out.lane_runs[lane] =
                    coldinit::run_near(maps[bit] as u128, LANE_SWEEP_POINTS, pref);
                let (s, l) = out.lane_runs[lane];
                out.lane_delay[lane] = (((s + l / 2) % LANE_SWEEP_POINTS) as u16) * LANE_SWEEP_STEP;
            } else {
                out.lane_delay[lane] = out.common_center;
            }
        }
        if let Some(echo) = self.cbt_measure(b, &out.lane_delay) {
            out.final_echo = echo;
            out.final_pass = coldinit::cbt_pass_bits(echo, bits);
        }
    }
}

/// FEL lab `A7A_CBT_CA_ADD` (hex): added to every CA lane's trained delay.
#[allow(non_snake_case)]
pub fn CBT_CA_ADD() -> u16 {
    hex_knob(crate::knobs::env(
        "A7A_CBT_CA_ADD",
        crate::knob!("A7A_CBT_CA_ADD"),
    ))
}

/// The ACTxDly values CBT applies: its centres plus [`CBT_CA_ADD()`], wrapped
/// into the 8-bit register (ACTxDly [7:6] coarse UI 0-3, [5:0] fine UI/64,
/// EIC7700X TRM Part 2). The sweep runs 0..511 and the delay passes once per
/// 256 units, so a centre found in the second period (288 on this board)
/// was written past the register's width and truncated by the hardware.
pub fn ca_applied(delay: &[u16; 7]) -> [u16; 7] {
    delay.map(|d| d.wrapping_add(CBT_CA_ADD()) & 0xff)
}

/// FEL lab `A7A_WL_TXWCK_ADD` (hex): added to every DBYTE's TxWckDlyTg0/1
/// right after WCK2CK leveling, with a held ForceInternalUpdate (the DFI is
/// still offline). Blobless leveling lands exactly one WCK cycle (0x80)
/// before the PMU's TxWckDly; with this the read training runs on the PMU's
/// WCK cycle.
#[allow(non_snake_case)]
pub fn WL_TXWCK_ADD() -> u16 {
    hex_knob(crate::knobs::env(
        "A7A_WL_TXWCK_ADD",
        crate::knob!("A7A_WL_TXWCK_ADD"),
    ))
}

/// Apply [`WL_TXWCK_ADD()`]; returns the TxWckDlyTg0 of each DBYTE after.
pub fn wl_txwck_add<H: Hw>(hw: &mut H) -> [u16; 4] {
    let mut out = [0u16; 4];
    csr_access(hw, true);
    for (db, o) in out.iter_mut().enumerate() {
        for tg in 0..2u32 {
            let c = csr(0x1002a + 0x1000 * db as u32 + tg);
            let v = hw.read16(c);
            hw.write16(c, v.wrapping_add(WL_TXWCK_ADD()));
        }
        *o = hw.read16(csr(0x1002a + 0x1000 * db as u32));
    }
    hw.write16(csr(0x200a3), 1);
    hw.udelay(5);
    hw.write16(csr(0x200a3), 0);
    hw.udelay(5);
    csr_access(hw, false);
    out
}

/// Points of the common CA delay sweep (0..511 in steps of 4).
pub const CA_SWEEP_POINTS: u32 = 128;
pub const CA_SWEEP_STEP: u16 = 4;
/// Common-sweep point whose raw echo is kept (delay 288, inside the window
/// measured on channel A rank 0 in M7).
pub const CA_RAW_POINT: u32 = 72;
/// Points of each per-lane sweep (0..511 in steps of 8).
pub const LANE_SWEEP_POINTS: u32 = 64;
pub const LANE_SWEEP_STEP: u16 = 8;

/// Our CA training result for one CBT block.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BlockTrain {
    /// Common sweep: per AsyncDbyteRxData bit 0..7, pass map over the sweep
    /// points and its longest circular pass run (start point, length).
    pub bit_maps: [u128; 8],
    pub bit_runs: [(u32, u32); 8],
    /// Longest run where every echo bit passes, and its center delay.
    pub common_run: (u32, u32),
    pub common_center: u16,
    /// Per CA lane (delay register index): the echo bit it moves (0xff if not
    /// exactly one), its pass map and run in lane-sweep points, and the chosen
    /// delay.
    pub lane_bit: [u8; 7],
    pub lane_maps: [u64; 7],
    pub lane_runs: [(u32, u32); 7],
    pub lane_delay: [u16; 7],
    /// Pass bits with the final per-lane delays (the channel's echo bits when
    /// all pass).
    pub final_pass: u16,
    pub final_echo: [u16; 4],
    /// Raw echo at common-sweep points 0 and [`CA_RAW_POINT`].
    pub raw_first: [u16; 4],
    pub raw_mid: [u16; 4],
}

impl BlockTrain {
    pub const EMPTY: BlockTrain = BlockTrain {
        bit_maps: [0; 8],
        bit_runs: [(0, 0); 8],
        common_run: (0, 0),
        common_center: 0,
        lane_bit: [0xff; 7],
        lane_maps: [0; 7],
        lane_runs: [(0, 0); 7],
        lane_delay: [0; 7],
        final_pass: 0,
        final_echo: [0; 4],
        raw_first: [0; 4],
        raw_mid: [0; 4],
    };
}

/// One channel's CA delays from the windows of its four blocks (both ranks,
/// both phases): per lane, the center of the intersection of the windows.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ChannelCa {
    /// Lane-sweep run of the intersection per lane (length 0: no overlap or
    /// a block without a window for that lane).
    pub runs: [(u32, u32); 7],
    pub delay: [u16; 7],
    /// Every lane drives the same echo bit in all four blocks.
    pub lane_map_agrees: bool,
}

impl ChannelCa {
    pub const EMPTY: ChannelCa = ChannelCa {
        runs: [(0, 0); 7],
        delay: [0; 7],
        lane_map_agrees: false,
    };
}

/// Result of [`cold_init_ca_train`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CaTrainReport {
    pub exec: ColdInitReport,
    /// Per [`coldinit::CBT_BLOCKS`] entry.
    pub blocks: [BlockTrain; 8],
    /// Blocks trained (entered, swept and left).
    pub done: u8,
    /// Per channel (A, B); applied to the AC blocks after the last block.
    pub channels: [ChannelCa; 2],
    /// DevInit before reset release.
    pub devinit: devinit::DevinitReport,
}

impl CaTrainReport {
    pub const EMPTY: CaTrainReport = CaTrainReport {
        exec: ColdInitReport::EMPTY,
        blocks: [BlockTrain::EMPTY; 8],
        done: 0,
        channels: [ChannelCa::EMPTY; 2],
        devinit: devinit::DevinitReport::EMPTY,
    };
}

/// Combine the four blocks of channel `ch` (indices 4 ch .. 4 ch + 3).
pub fn combine_channel(blocks: &[BlockTrain; 8], ch: usize) -> ChannelCa {
    let set = &blocks[4 * ch..4 * ch + 4];
    let mut out = ChannelCa {
        lane_map_agrees: true,
        ..ChannelCa::EMPTY
    };
    for lane in 0..7 {
        let bit = set[0].lane_bit[lane];
        if set.iter().any(|b| b.lane_bit[lane] != bit) || bit == 0xff {
            out.lane_map_agrees = false;
        }
        let pref = set[0].lane_delay[lane] as u32 / LANE_SWEEP_STEP as u32;
        let w = dram_train::window::intersect_near(
            set.iter().map(|b| b.lane_maps[lane] as u128),
            LANE_SWEEP_POINTS,
            pref,
        );
        out.runs[lane] = w.as_tuple();
        out.delay[lane] = if w.is_empty() {
            set[0].lane_delay[lane]
        } else {
            w.center(LANE_SWEEP_POINTS) as u16 * LANE_SWEEP_STEP
        };
    }
    out
}

/// Command bus training by the host on every channel, rank and CBT phase, in
/// the oracle's block order ([`coldinit::CBT_BLOCKS`]): cold init to CBT on
/// the first block, then per block our own CA search
/// ([`BlockTrain`]) between the block's entry and exit. Afterwards each
/// channel's CA delays are the centers of its four blocks' window
/// intersections, applied to its AC block. Call after [`pre_training`]. The
/// DRAM has left CBT at the end. Stops at the first ACSM failure.
pub fn cold_init_ca_train<H: Hw>(
    hw: &mut H,
    m1: &coldinit::Mrs,
    m2: &coldinit::Mrs2,
    out: &mut CaTrainReport,
) {
    let t0 = hw.now_us();
    *out = CaTrainReport::EMPTY;
    let mut ex = Executor {
        hw: &mut *hw,
        r: out.exec,
        loop_run: 0,
        mem_reset: 3,
    };
    bring_up_to_cbt(&mut ex, m1, m2, &mut out.devinit, &mut |_| {});
    for (i, b) in coldinit::CBT_BLOCKS.iter().enumerate() {
        if ex.r.failure.is_some() {
            break;
        }
        if i > 0 {
            coldinit::block_entry(b, m2, &mut |s| ex.step(s));
        }
        ex.train_block(b, &mut out.blocks[i]);
        coldinit::block_exit(b, m2, &mut |s| ex.step(s));
        if ex.r.failure.is_none() {
            out.done += 1;
        }
    }
    coldinit::cbt_end(&mut |s| ex.step(s));
    out.channels = [
        combine_channel(&out.blocks, 0),
        combine_channel(&out.blocks, 1),
    ];
    if out.done == 8 {
        for (c, ca) in [coldinit::CBT_A, coldinit::CBT_B]
            .iter()
            .zip(out.channels.iter())
        {
            coldinit::set_block_delays(c.ac, &ca_applied(&ca.delay), &mut |s| ex.step(s));
        }
    }
    out.exec = ex.r;
    csr_access(hw, false);
    out.exec.elapsed_us = hw.now_us().wrapping_sub(t0);
}

/// FEL lab `A7A_FSP0_MRS=<MAs|all>`: after our cold init,
/// before our end of initialization, the listed mode registers (the end of
/// initialization's values, `A7A_BL_MRS` applied) written into FSP 0 while
/// FSP 0 operates: MR16 0x40 (FSP-WR 0, FSP-OP 0, VRCG), the MRWs, MR16
/// 0x00. After a DRAM reset FSP 0 holds the reset values (MR18 CKR 1, a
/// 2:1 WCK ratio), and the end of initialization makes FSP 0 operate during
/// its per-rank writes (MR16 0x41). Returns the launch failure, if any.
pub fn fsp0_mrs<H: Hw>(hw: &mut H, p: &mut Para, pstate: u32) -> Option<crate::wl::WlFailure> {
    use crate::phy::endinit::{self, skip, FspMrs, Variant, ORDER};
    use crate::phy::wcklevel::Channel;
    let spec = crate::knobs::env("A7A_FSP0_MRS", crate::knob!("A7A_FSP0_MRS"))?;
    let all = ORDER.iter().fold(0u64, |a, &ma| a | 1 << ma);
    let only = if spec == "all" {
        all
    } else {
        Variant::parse(None, Some(spec), None)
            .map(|v| v.no_mrw)
            .unwrap_or(0)
    };
    let v = Variant {
        skip: skip::REFAB | skip::CBT_EXIT,
        no_mrw: all & !only,
        mr16_write: 0x40,
        mr16_switch: 0x00,
    };
    let m = message_block(p, pstate);
    let mut ex = crate::wl::WlExec::new(hw);
    endinit::run_with(
        Channel::B,
        |ch, rank| {
            let mut f = FspMrs::from_message_block(&m, ch, rank);
            mission_mr_overrides(&mut f);
            f
        },
        &v,
        &mut |s| ex.step(s),
    );
    let f = ex.failure;
    csr_access(hw, false);
    f
}

/// FEL lab hybrid: our DevInit / cold init / CBT over the
/// PMU-trained state, before our end of initialization, WL and the lean
/// hand-over. `stage` 0: DevInit's PHY steps only (the DRAM kept out of
/// reset); 1: DevInit, DRAM reset release and the cold-init MRWs (part 1 and
/// 2), then the CBT exit of block 0 and the CBT end (the PMU's CA delays
/// stay); 2: the whole cold init and CA training ([`cold_init_ca_train`]);
/// 3: DevInit and the DRAM reset release only; 4: 3 plus cold-init part 1.
pub fn hybrid_cold<H: Hw>(
    hw: &mut H,
    p: &mut Para,
    pstate: u32,
    stage: u32,
    out: &mut CaTrainReport,
) {
    let (m1, m2) = mode_registers(p, pstate);
    *out = CaTrainReport::EMPTY;
    match stage {
        0 => {
            crate::phy::devinit::pre_release_opts(hw, &mut out.devinit, true);
            csr_access(hw, false);
        }
        1 => {
            let t0 = hw.now_us();
            let mut ex = Executor {
                hw: &mut *hw,
                r: out.exec,
                loop_run: 0,
                mem_reset: 3,
            };
            bring_up_to_cbt(&mut ex, &m1, &m2, &mut out.devinit, &mut |_| {});
            coldinit::block_exit(&coldinit::CBT_BLOCKS[0], &m2, &mut |s| ex.step(s));
            coldinit::cbt_end(&mut |s| ex.step(s));
            out.exec = ex.r;
            csr_access(hw, false);
            out.exec.elapsed_us = hw.now_us().wrapping_sub(t0);
        }
        3 | 4 => {
            // 3: DevInit and the DRAM reset release only; 4: plus cold-init
            // part 1 (no CBT entry, no CBT exit).
            let t0 = hw.now_us();
            let mut ex = Executor {
                hw: &mut *hw,
                r: out.exec,
                loop_run: 0,
                mem_reset: 2,
            };
            devinit::pre_release(ex.hw, &mut out.devinit);
            let delay_row = [coldinit::Row::nop(0x1b00), coldinit::Row::EMPTY];
            ex.step(coldinit::Step::Launch {
                start: 0,
                rows: &delay_row,
            });
            csr_access(ex.hw, true);
            ex.hw.write16(csr(CLR_POR_MEM_RESET), 1);
            ex.hw.write16(csr(CLR_POR_MEM_RESET), 0);
            ex.hw.udelay(RESET_LOW_US);
            ex.hw.write16(csr(MEM_RESET_L), 3);
            ex.hw.udelay(RESET_HIGH_US);
            ex.mem_reset = 3;
            if stage == 4 {
                coldinit::part1(&m1, &mut |s| ex.step(s));
            }
            out.exec = ex.r;
            csr_access(hw, false);
            out.exec.elapsed_us = hw.now_us().wrapping_sub(t0);
        }
        _ => cold_init_ca_train(hw, &m1, &m2, out),
    }
}

/// The lean hand-over after our own training, the same
/// steps the open driver takes without the PIE (`phy::devinit::
/// mission_bringup` level 3 minus the PLL/DLL relock, which our flow never
/// powered down; `post::dfi_init_cpu`): PUBMODE 0 and the mission CSRs, the
/// controller's read-data enable computed from our PHY CSRs
/// into set 0 and the active set `pstate` (MSTR2), the CPU-answered DFI
/// init and the ports. No CDD fold (no message block without the PMU).
pub fn mission_handover<H: Hw>(hw: &mut H, p: &Para, pstate: u32) -> Result<u32, Error> {
    mission_handover_relock(hw, p, pstate, false)
}

/// FEL lab: INIT0.skip_dram_init for the hand-over (env `A7A_BL_INIT0` at
/// build time, 0..3; unset: as `ctrl::com_init` left it).
fn init0_skip() -> Option<u32> {
    crate::knobs::get().init0
}

/// Compute the mission `DFIMRL` (`trained + mrl_add`) for handover.
/// Fails closed if read training did not record an applied MRL (`0`) or
/// if the sum exceeds the 5-bit `DFIMRL` CSR width.
pub(crate) const fn handover_mrl(trained: u16, mrl_add: u16) -> Result<u16, Error> {
    if trained == 0 {
        return Err(Error::Config("no trained MRL for mission handover"));
    }
    match trained.checked_add(mrl_add) {
        Some(v) if v <= crate::rdtrain::MRL_CSR_MAX => Ok(v),
        _ => Err(Error::Config("mission MRL out of CSR range")),
    }
}

/// Check whether an `A7A_HO_CSRS` lab override explicitly writes valid
/// (1..=`MRL_CSR_MAX`) `DFIMRL` values to all four DBYTEs (`0x10000..=0x13000`).
fn ho_csrs_overrides_all_dfimrl(spec: &str) -> bool {
    let mut seen = 0u8;
    for pair in spec.split(',') {
        let mut it = pair.split('=');
        if let (Some(a), Some(v)) = (it.next(), it.next()) {
            if let (Ok(a), Ok(v)) = (u32::from_str_radix(a, 16), u16::from_str_radix(v, 16)) {
                if (1..=crate::rdtrain::MRL_CSR_MAX).contains(&v) {
                    match a {
                        0x10000 => seen |= 1 << 0,
                        0x11000 => seen |= 1 << 1,
                        0x12000 => seen |= 1 << 2,
                        0x13000 => seen |= 1 << 3,
                        _ => {}
                    }
                }
            }
        }
    }
    seen == 0x0f
}

/// [`mission_handover`]; with `relock`, first the training firmware's end
/// state (PLL powered down, ClockingCtrl 1, TxRdPtrInit 1) and then our
/// mission bring-up (`phy::devinit::mission_bringup` level 3) before the
/// DFI init, as the open driver does after the PMU firmware.
pub fn mission_handover_relock<H: Hw>(
    hw: &mut H,
    p: &Para,
    pstate: u32,
    relock: bool,
) -> Result<u32, Error> {
    mission_handover_relock_with(
        hw,
        p,
        pstate,
        relock,
        HO_MRL_ADD(),
        crate::knobs::env("A7A_HO_CSRS", crate::knob!("A7A_HO_CSRS")),
    )
}

#[inline(always)]
fn mission_handover_relock_with<H: Hw>(
    hw: &mut H,
    p: &Para,
    pstate: u32,
    relock: bool,
    mrl_add: u16,
    ho_csrs: Option<&str>,
) -> Result<u32, Error> {
    use crate::ctrl::{ddrc, freq, sw_done, DDRC_B84, DDRC_B88, SWCTL, SWSTAT};
    use crate::hw::wait;
    const T: u64 = 100_000;
    let channels = p.channels();
    csr_access(hw, true);
    // FEL lab: whole-cycle corrections of the trained delays, before the
    // mission CSRs and the DFI init (delay CSR writes after the DFI init do
    // not take effect: measured). TxWckDlyTg0/1 0x1n02a/0x1n02b and
    // RxEnDlyTg0/1 0x1n020/0x1n021 of every DBYTE.
    for (add, base) in [(HO_TXWCK_ADD(), 0x1002a), (HO_RXEN_ADD(), 0x10020)] {
        if add == 0 {
            continue;
        }
        for db in 0..4u32 {
            for tg in 0..2u32 {
                let c = csr(base + 0x1000 * db + tg);
                let v = hw.read16(c);
                hw.write16(c, v.wrapping_add(add));
            }
        }
    }
    // Always validate APPLIED_MRL and write the mission read latency
    // (`trained + mrl_add`, including `mrl_add = 0` or unset) to every DBYTE
    // so the post-training select's 8 cannot remain in mission mode. An
    // explicit full-DBYTE `A7A_HO_CSRS` lab override stays separate and runs
    // after this write so it is never clobbered.
    if !ho_csrs.is_some_and(ho_csrs_overrides_all_dfimrl) {
        let trained = crate::rdtrain::APPLIED_MRL.load(core::sync::atomic::Ordering::Relaxed);
        let mission = match handover_mrl(trained, mrl_add) {
            Ok(v) => v,
            Err(e) => {
                csr_access(hw, false);
                return Err(e);
            }
        };
        for db in 0..4u32 {
            hw.write16(csr(0x10000 + 0x1000 * db), mission);
        }
    }
    if let Some(spec) = ho_csrs {
        // FEL lab A7A_HO_CSRS="csr=hex,...": explicit PHY CSR writes before
        // the latch pulse and DFI init (includes absolute DFIMRL overrides
        // separate from `A7A_HO_MRL_ADD`).
        for pair in spec.split(',') {
            let mut it = pair.split('=');
            if let (Some(a), Some(v)) = (it.next(), it.next()) {
                if let (Ok(a), Ok(v)) = (u32::from_str_radix(a, 16), u16::from_str_radix(v, 16)) {
                    hw.write16(csr(a), v);
                }
            }
        }
    }
    // Always pulse ForceInternalUpdate with the DFI still offline (held well
    // over 32 DFICLKs, TRM), so every DBYTE's mission DFIMRL and any delay CSR
    // write is latched regardless of `A7A_HO_MRL_ADD` or `A7A_HO_FIU`.
    hw.write16(csr(0x200a3), 1);
    hw.udelay(5);
    hw.write16(csr(0x200a3), 0);
    hw.udelay(5);
    if crate::obs::OBS_ARM() & crate::obs::ARM_CHECK != 0 {
        // FEL lab A7A_OBS_ARM bit 0: the PHY's RX FIFO pointer check,
        // armed (masked off the interrupt pin) before the DFI init, so the
        // dfi_init_start check and every later read are covered.
        crate::obs::arm(hw);
    }
    if HO_PULSE_RESTORE() {
        for c in ACSM_READ_PULSES {
            let v = hw.read16(csr(c));
            hw.write16(csr(c), v.wrapping_sub(ACSM_PULSE_ADJUST));
        }
    }
    for (c, v) in [(0x2006e, 0u16), (0x20060, 1), (0x20054, 1), (0x2002a, 0)] {
        hw.write16(csr(c), v);
    }
    csr_access(hw, false);
    let (ptr_init, rcv) = crate::phy::read_latency_csrs(hw);
    let rden = ((rcv >> 1) & 0xf) + (((ptr_init & 0xf) + 1) >> 3) * 4 + 21;
    for ch in 0..channels {
        hw.set32(ddrc(ch, DDRC_B84), 1);
        wait(hw, "DDRC+0xb88 ack [26:25]", T, |hw| {
            (hw.read32(ddrc(0, DDRC_B88)) >> 25) & 3 == 3
        })?;
        hw.write32(ddrc(ch, SWCTL), 0);
        wait(hw, "SWSTAT.sw_done_ack clear", T, |hw| {
            hw.read32(ddrc(ch, SWSTAT)) & 1 == 0
        })?;
        for set in [0, pstate as usize] {
            hw.modify32(freq(ch, set, 0x588), |v| (v & !0x3f_0000) | (rden << 16));
            if let Some(en) = HO_RDDATA_EN() {
                // FEL lab: dfi_t_rddata_en (+0x580 [22:16]) of the set.
                hw.modify32(freq(ch, set, crate::handover::DFITMG0), |v| {
                    (v & !crate::handover::RDDATA_EN_MASK)
                        | (en << crate::handover::RDDATA_EN_SHIFT)
                });
            }
            if let Some(cs) = crate::obs::HO_RDCSLAT() {
                // FEL lab: dfi_tphy_rdcslat (+0x588 [14:8]) of the set.
                hw.modify32(freq(ch, set, 0x588), |v| {
                    (v & !crate::obs::RDCSLAT_MASK) | (cs << crate::obs::RDCSLAT_SHIFT)
                });
            }
        }
        hw.clrset32(ddrc(ch, 0x008), 0x1f, pstate);
        if let Some(n) = HO_CDD() {
            crate::post::fold_cdd_set(
                hw,
                ch,
                pstate as usize,
                &dwc_ddrctl::dfi::Cdd {
                    rr: n,
                    rr_alt: n,
                    ww: n,
                    ww_alt: n,
                    rw: n,
                    wr: n,
                },
            );
        }
        if let Some(on) = HO_WCKON() {
            // FEL lab: MSTR4.wck_on [0] (quasi-dynamic, written with
            // sw_done clear while the controller still holds the DRAM in
            // its start-up self-refresh state).
            hw.clrset32(ddrc(ch, crate::ctrl::MSTR4), 1, on);
        }
        if let Some(v) = HO_DBICTL() {
            // FEL lab: DBICTL rd_dbi_en [2], wr_dbi_en [1], dm_en [0].
            hw.write32(ddrc(ch, DBICTL), v);
        }
        sw_done(hw, ch)?;
        hw.clr32(ddrc(ch, DDRC_B84), 1);
    }
    if let Some(skip) = init0_skip() {
        // INIT0.skip_dram_init [31:30]: 3 (the vendor's) starts the
        // controller in self-refresh, so leaving selfref_sw issues SRX; our
        // training leaves the DRAM idle, not in self-refresh.
        for ch in 0..channels {
            hw.write32(ddrc(ch, SWCTL), 0);
            hw.modify32(ddrc(ch, 0xd00), |v| (v & !(3 << 30)) | (skip << 30));
            sw_done(hw, ch)?;
        }
    }
    if relock {
        csr_access(hw, true);
        for (c, v) in [(0x200f8, 1u16), (0x200a2, 1), (0x200c3, 1)] {
            hw.write16(csr(c), v);
        }
        hw.udelay(100);
        csr_access(hw, false);
        crate::phy::devinit::mission_bringup(hw, 3);
    }
    if crate::obs::OBS_ARM() & crate::obs::ARM_SNAP != 0 {
        let s = crate::obs::snap(hw);
        crate::obs::print(hw, "before the DFI init", &s);
    }
    crate::post::dfi_init_cpu(hw, channels)?;
    crate::post::controller_after_phy(hw, p)?;
    if crate::obs::OBS_ARM() & crate::obs::ARM_SNAP != 0 {
        let s = crate::obs::snap(hw);
        crate::obs::print(hw, "after the DFI init", &s);
    }
    if let Some(spec) = crate::knobs::env("A7A_HO_MRS", crate::knob!("A7A_HO_MRS")) {
        // FEL lab A7A_HO_MRS="ma=hex,...": operating-set MRWs through the
        // controller after the hand-over, before any read (FSP window
        // MR16 0x05 .. 0x04), e.g. "3=06,18=00": the PMU's operating MR3
        // (DBI off) and MR18; our training needs MR18 0x0c.
        for ch in 0..channels {
            for rank in 0..2 {
                crate::mr::write_rank(hw, ch, rank, 16, 0x05)?;
                for pair in spec.split(',') {
                    let mut it = pair.split('=');
                    if let (Some(a), Some(v)) = (it.next(), it.next()) {
                        if let (Ok(a), Ok(v)) = (a.parse::<u32>(), u32::from_str_radix(v, 16)) {
                            crate::mr::write_rank(hw, ch, rank, a, v & 0xff)?;
                        }
                    }
                }
                crate::mr::write_rank(hw, ch, rank, 16, 0x04)?;
            }
        }
    }
    if let Some(mr2) = HO_MR2() {
        // FEL lab: MR2 of the operating set-point through the controller,
        // inside an FSP-WR = FSP-OP window (MR16 0x05 .. 0x04: the state
        // after the PMU and after the FSP-WR 0 end of initialization).
        for ch in 0..channels {
            for rank in 0..2 {
                crate::mr::write_rank(hw, ch, rank, 16, 0x05)?;
                crate::mr::write_rank(hw, ch, rank, 2, mr2 as u32)?;
                crate::mr::write_rank(hw, ch, rank, 16, 0x04)?;
            }
        }
    }
    if let Some(mr16_open) = HO_EPILOGUE() {
        epilogue(hw, channels, mr16_open)?;
    } else if HO_SRCYCLE() {
        sr_cycle(hw, channels)?;
    }
    if HO_FIFOINIT() {
        fifo_init(hw, channels)?;
    }
    Ok(rden)
}

/// FEL lab `A7A_HO_CDD=<hex>`: a flat rank-to-rank margin folded into the
/// active set's turnaround timings by the lean hand-over
/// ([`crate::post::fold_cdd_set`], every CDD field = the value, in the
/// fields' own units: controller clocks). The vendor folds its trained
/// CDDs into set 0 only, so set 1 runs with none; our own trained rank 0
/// against rank 1 differences are below one clock (RxEnDly Tg0/Tg1 within
/// 0x10 of 1/64 UI, TxDqDly equal), so 1 covers them. Unset: no fold.
#[allow(non_snake_case)]
pub fn HO_CDD() -> Option<u32> {
    match crate::knobs::env("A7A_HO_CDD", crate::knob!("A7A_HO_CDD")) {
        Some(s) => match u32::from_str_radix(s, 16) {
            Ok(v) if v <= 0x1f => Some(v),
            _ => panic!("A7A_HO_CDD is a hex value up to 0x1f"),
        },
        None => None,
    }
}

/// `A7A_HO_MRL_ADD=<hex>`: offset (0..=3, unset: 0) added to the read
/// training's applied latency (`rdtrain::APPLIED_MRL`) when the lean
/// hand-over writes every DBYTE's DFIMRL (`0x1n000`) and pulses
/// `ForceInternalUpdate` (`0x200a3`). Validation, the per-DBYTE write, and
/// the latch pulse run unconditionally even when `A7A_HO_MRL_ADD` is 0 or
/// unset. Absolute lab overrides stay explicit and separate: `A7A_RD_MRL`
/// (training-time override, `knobs.rs`) and `A7A_HO_CSRS` (hand-over CSR
/// override, e.g. `10000=m,11000=m,12000=m,13000=m`).
#[allow(non_snake_case)]
pub fn HO_MRL_ADD() -> u16 {
    match crate::knobs::env("A7A_HO_MRL_ADD", crate::knob!("A7A_HO_MRL_ADD")) {
        Some(s) => match u16::from_str_radix(s, 16) {
            Ok(v) if v <= 3 => v,
            _ => panic!("A7A_HO_MRL_ADD is 0-3"),
        },
        None => 0,
    }
}

/// DDRCTL DBICTL (DM/DBI control), main block +0xc94.
pub const DBICTL: usize = 0xc94;

/// FEL lab `A7A_HO_WCKON=0|1`: the controller's MSTR4.wck_on (WCK always on)
/// written by the lean hand-over before the DFI init. `com_init` sets 1
/// (the vendor's value). wck_on 1 makes the controller sync WCK once
/// (CAS-WS_FS to all ranks) and expects the DRAM's MR18 OP[4] (WCK always
/// on) = 1 (public: DDRCTL MSTR4); our end of initialization and read
/// training leave MR18 OP[4] = 0 (0x0c). Unset: unchanged.
#[allow(non_snake_case)]
pub fn HO_WCKON() -> Option<u32> {
    match crate::knobs::env("A7A_HO_WCKON", crate::knob!("A7A_HO_WCKON")) {
        Some(s) => match s.as_bytes() {
            [b'0'] => Some(0),
            [b'1'] => Some(1),
            _ => panic!("A7A_HO_WCKON is 0 or 1"),
        },
        None => None,
    }
}

/// FEL lab `A7A_HO_DBICTL=<hex>`: DBICTL of both controllers, written with
/// [`HO_WCKON()`] before the DFI init (rd_dbi_en must equal MR3 OP[6],
/// wr_dbi_en MR3 OP[7], dm_en the inverse of MR13 OP[5]; public: DDRCTL
/// DBICTL). Our code never writes it (reset 0x1). Unset: unchanged.
#[allow(non_snake_case)]
pub fn HO_DBICTL() -> Option<u32> {
    match crate::knobs::env("A7A_HO_DBICTL", crate::knob!("A7A_HO_DBICTL")) {
        Some(s) => match u32::from_str_radix(s, 16) {
            Ok(v) if v <= 7 => Some(v),
            _ => panic!("A7A_HO_DBICTL is a hex value up to 7"),
        },
        None => None,
    }
}

/// FEL lab `A7A_HO_SRCYCLE=1`: at the end of the lean hand-over (after
/// `A7A_HO_MRS`/`A7A_HO_MR2`), one software self-refresh entry and exit per
/// controller ([`sr_cycle`]): the controller's own SRE, then SRX followed by
/// its WCK re-sync (with wck_on: CAS-WS_FS to all ranks), so MR18 changes
/// made through `A7A_HO_MRS` start from a fresh WCK2CK sync. Unset: none.
#[allow(non_snake_case)]
pub fn HO_SRCYCLE() -> bool {
    crate::knobs::env("A7A_HO_SRCYCLE", crate::knob!("A7A_HO_SRCYCLE")).is_some()
}

/// One software self-refresh cycle per channel (public DDRCTL procedure,
/// "Clock Removal and Recovery"): PWRCTL.selfref_sw = 1, STAT.selfref_type
/// = 2'b10 (software self-refresh), then selfref_sw = 0, selfref_type = 0
/// and operating_mode normal.
pub fn sr_cycle<H: Hw>(hw: &mut H, channels: usize) -> Result<(), Error> {
    sr_enter(hw, channels)?;
    hw.udelay(10);
    sr_exit(hw, channels)
}

/// Software self-refresh entry on every controller (PWRCTL.selfref_sw = 1,
/// STAT.selfref_type = 2'b10). While it lasts the controller sends no
/// commands to the DRAM (DES only on the DFI), the state the TRM requires
/// for ForceInternalUpdate and for an RX FIFO pointer reset.
pub fn sr_enter<H: Hw>(hw: &mut H, channels: usize) -> Result<(), Error> {
    use crate::ctrl::ddrc;
    use crate::hw::wait;
    use dwc_ddrctl::regs::{PWRCTL, PWRCTL_SELFREF_SW, STAT, STAT_SELFREF_TYPE};
    for ch in 0..channels {
        hw.set32(ddrc(ch, PWRCTL), PWRCTL_SELFREF_SW);
        wait(hw, "STAT.selfref_type software (SR entry)", 100_000, |hw| {
            hw.read32(ddrc(ch, STAT)) & STAT_SELFREF_TYPE == 0x20
        })?;
    }
    Ok(())
}

/// Software self-refresh exit on every controller: SRX, then (with
/// MSTR4.wck_on) the controller's CAS-WS_FS WCK re-sync, back to normal.
pub fn sr_exit<H: Hw>(hw: &mut H, channels: usize) -> Result<(), Error> {
    use crate::ctrl::ddrc;
    use crate::hw::wait;
    use dwc_ddrctl::regs::{
        OPERATING_MODE_NORMAL, PWRCTL, PWRCTL_SELFREF_SW, STAT, STAT_OPERATING_MODE,
        STAT_SELFREF_TYPE,
    };
    const T: u64 = 100_000;
    for ch in 0..channels {
        hw.clr32(ddrc(ch, PWRCTL), PWRCTL_SELFREF_SW);
        wait(hw, "STAT.selfref_type clear (SR exit)", T, |hw| {
            hw.read32(ddrc(ch, STAT)) & STAT_SELFREF_TYPE == 0
        })?;
        wait(hw, "STAT.operating_mode (SR exit)", T, |hw| {
            hw.read32(ddrc(ch, STAT)) & STAT_OPERATING_MODE == OPERATING_MODE_NORMAL
        })?;
    }
    hw.udelay(2);
    Ok(())
}

/// FEL lab `A7A_HO_EPILOGUE=1` (MR16 0x45, the PMU's) or `=05` (MR16 0x05,
/// the window `A7A_HO_MRS` used on the board): the training
/// firmware's final-stage DRAM state, made by the controller after the
/// lean hand-over ([`epilogue`]). Unset: nothing (defaults unchanged).
#[allow(non_snake_case)]
pub fn HO_EPILOGUE() -> Option<u32> {
    match crate::knobs::env("A7A_HO_EPILOGUE", crate::knob!("A7A_HO_EPILOGUE")) {
        Some(s) => match s.as_bytes() {
            [b'1'] => Some(0x45),
            [b'0', b'5'] => Some(0x05),
            _ => panic!("A7A_HO_EPILOGUE is 1 (MR16 0x45) or 05 (MR16 0x05)"),
        },
        None => None,
    }
}

/// The operating-set MRs where the PMU's final stage differs from our end
/// of initialization (MA, OP). Measured: MR18 OP[4] (WCK always on) is
/// what the controller's MSTR4.wck_on = 1 needs (MR18 0x0c fails with
/// the blobless signature, 0x1c passes); MR3 0x06 (read/write DBI
/// off, DBICTL rd/wr_dbi_en 0) and MR13 0x00 (data mask on, DBICTL dm_en 1)
/// are the PMU's values read back on the board (MR3/MR13 MRR). Every other
/// MR of the PMU's final program equals our end of initialization's
/// (MR1 0x80, MR2 0x88, MR10 0x54, MR11/MR17 per rank, MR12, MR14/15
/// trained, MR19 0x10, MR20 0x02, MR22, MR24, MR25, MR28 0x04, MR30, MR41,
/// MR46) and is not rewritten.
pub const EPILOGUE_MRS: [(u32, u32); 3] = [(18, 0x1c), (3, 0x06), (13, 0x00)];

/// DDRCTL OPCTRLCMD (+0xb8c) and OPCTRLSTAT (+0xb90): bit 16
/// zq_calib_short / zq_calib_short_busy (public DDRCTL; for LPDDR5 the MPC
/// ZQ calibration, allowed with ZQCTL0.dis_auto_zq = 1, which `com_init`
/// sets).
pub const OPCTRLCMD: usize = 0xb8c;
pub const OPCTRLSTAT: usize = 0xb90;
pub const ZQ_CALIB_SHORT: u32 = 1 << 16;

/// The training firmware's final stage, re-made through the controller
/// after the lean hand-over (`A7A_HO_EPILOGUE`):
/// 1. per channel and rank, MR16 = `mr16_open` (FSP-WR = FSP-OP = 1), the
///    [`EPILOGUE_MRS`], MR16 0x04;
/// 2. per channel, ZQ calibration latch (OPCTRLCMD.zq_calib_short);
/// 3. one software self-refresh cycle ([`sr_cycle`]): the controller's
///    SRE and SRX, then its CAS-WS_FS (WCK always on) to every rank.
///
/// Why the controller and not an ACSM program before the DFI init (the
/// PMU's own way: the MRWs, ZQ latch, SRE, MR28 ZQ stop, PDE, then the
/// controller's SRX at start with INITTMG0.skip_dram_init = 3): every step
/// here is a public DDRCTL operation, and the result (MR18 0x1c + SR cycle)
/// read correctly on the board. The ACSM route
/// needs new row programs whose command spacing would come from the reference
/// (SRE/PDE timing), and its one earlier attempt (with the SRX
/// row mis-labelled SRE) timed out. The controller's start-up SRX to an
/// idle DRAM, which this leaves in place, read correctly in every board
/// run. REFab: the controller's own refresh runs from the hand-over on.
pub fn epilogue<H: Hw>(hw: &mut H, channels: usize, mr16_open: u32) -> Result<(), Error> {
    use crate::ctrl::ddrc;
    use crate::hw::wait;
    for ch in 0..channels {
        for rank in 0..2 {
            crate::mr::write_rank(hw, ch, rank, 16, mr16_open)?;
            for (ma, op) in EPILOGUE_MRS {
                crate::mr::write_rank(hw, ch, rank, ma, op)?;
            }
            crate::mr::write_rank(hw, ch, rank, 16, 0x04)?;
        }
    }
    for ch in 0..channels {
        hw.set32(ddrc(ch, OPCTRLCMD), ZQ_CALIB_SHORT);
        wait(
            hw,
            "OPCTRLSTAT.zq_calib_short_busy (epilogue)",
            100_000,
            |hw| hw.read32(ddrc(ch, OPCTRLSTAT)) & ZQ_CALIB_SHORT == 0,
        )?;
    }
    sr_cycle(hw, channels)
}

/// FEL lab `A7A_HO_FIFOINIT=1`: at the very end of the hand-over, before
/// any read, the RX FIFO pointers of every DBYTE reset (RxFifoInit
/// RxFifoInitPtr, broadcast 0x2f0a0, pulse held 2 µs) inside a controller
/// self-refresh (DFI idle). Measured: without it DBYTE 2's write pointers
/// start misaligned (the PHY's own FIFO check fires
/// on DBYTE 2). Unset: nothing.
#[allow(non_snake_case)]
pub fn HO_FIFOINIT() -> bool {
    crate::knobs::env("A7A_HO_FIFOINIT", crate::knob!("A7A_HO_FIFOINIT")).is_some()
}

/// RX FIFO pointer reset inside a controller self-refresh ([`HO_FIFOINIT()`]).
pub fn fifo_init<H: Hw>(hw: &mut H, channels: usize) -> Result<(), Error> {
    sr_enter(hw, channels)?;
    csr_access(hw, true);
    hw.write16(csr(0x2f0a0), 1);
    hw.udelay(2);
    hw.write16(csr(0x2f0a0), 0);
    hw.udelay(2);
    csr_access(hw, false);
    sr_exit(hw, channels)
}

/// FEL lab `A7A_HO_PULSE_RESTORE=1`: before the DFI init, the ACSM read
/// pulse timings our DevInit raised by 4 (`phy::devinit` PULSE_ADJUST:
/// AcsmRxEnPulse, RxValPulse, RdcsPulse, WckReadStaticLo/Hi, WckReadToggle,
/// WckReadFastToggle) put back by 4, as the training firmware does at the
/// end of its read latency stage (PMU state 0x82f/0xc10/0x41c/0x2920 against
/// our 0x833/0xc14/0x420/0x2924). Unset: unchanged.
#[allow(non_snake_case)]
pub fn HO_PULSE_RESTORE() -> bool {
    crate::knobs::env("A7A_HO_PULSE_RESTORE", crate::knob!("A7A_HO_PULSE_RESTORE")).is_some()
}
pub const ACSM_READ_PULSES: [u32; 7] = [
    0x2002c, 0x2002d, 0x20030, 0x20039, 0x2003a, 0x2003b, 0x2003c,
];
pub const ACSM_PULSE_ADJUST: u16 = 4;

/// `A7A_RD_PULSE_RESTORE=1`: the ACSM read pulse timings back by
/// [`ACSM_PULSE_ADJUST`] right before the read training instead of at the
/// hand-over ([`acsm_pulse_restore`]). Measured: with the pulses at our
/// DevInit's +4 our read training finds the gate exactly 0x200 earlier
/// (0x170-0x174) than with the training firmware's values
/// (0x36c-0x374, the PMU's own 0x36d), and the mission reads
/// then fail; the mission path does not use the ACSM pulses.
#[allow(non_snake_case)]
pub fn RD_PULSE_RESTORE() -> bool {
    crate::knobs::env("A7A_RD_PULSE_RESTORE", crate::knob!("A7A_RD_PULSE_RESTORE")).is_some()
}

/// The ACSM read pulse timings ([`ACSM_READ_PULSES`]) lowered by
/// [`ACSM_PULSE_ADJUST`]; returns the values written.
pub fn acsm_pulse_restore<H: Hw>(hw: &mut H) -> [u16; 7] {
    let mut out = [0u16; 7];
    csr_access(hw, true);
    for (k, c) in ACSM_READ_PULSES.into_iter().enumerate() {
        let v = hw.read16(csr(c)).wrapping_sub(ACSM_PULSE_ADJUST);
        hw.write16(csr(c), v);
        out[k] = v;
    }
    csr_access(hw, false);
    out
}

/// Delay CSRs written in mission mode applied: controller self-refresh (DFI
/// idle, DES only), ForceInternalUpdate held 5 µs (TRM: DFI offline, at
/// least 32 DFICLKs), self-refresh exit with the controller's WCK re-sync.
/// Measured: delay writes in mission mode alone do not take effect. Used
/// by the write stage per sweep point (`A7A_WR_APPLY`).
pub fn apply_delays<H: Hw>(hw: &mut H, channels: usize) -> Result<(), Error> {
    sr_enter(hw, channels)?;
    csr_access(hw, true);
    hw.write16(csr(0x200a3), 1);
    hw.udelay(5);
    hw.write16(csr(0x200a3), 0);
    hw.udelay(2);
    csr_access(hw, false);
    sr_exit(hw, channels)
}

/// FEL lab `A7A_HO_RDDATA_EN=<hex>`: the controller's dfi_t_rddata_en
/// (+0x580 [22:16]) in set 0 and the active set, written by the lean
/// hand-over before the DFI init. The timing code leaves 0x2f, which reads
/// correctly on the PMU-trained state (MR2 0x00, RL 3); MR2 0x88 (RL 13,
/// what our read training needs) is expected to need about 0x2f + 10.
/// Unset: unchanged.
#[allow(non_snake_case)]
pub fn HO_RDDATA_EN() -> Option<u32> {
    match crate::knobs::env("A7A_HO_RDDATA_EN", crate::knob!("A7A_HO_RDDATA_EN")) {
        Some(s) => match u32::from_str_radix(s, 16) {
            Ok(v) if v <= 0x7f => Some(v),
            _ => panic!("A7A_HO_RDDATA_EN is a hex value up to 0x7f"),
        },
        None => None,
    }
}

/// FEL lab `A7A_HO_TXWCK_ADD=<hex>` and `A7A_HO_RXEN_ADD=<hex>`: added to
/// every DBYTE's TxWckDlyTg0/1 and RxEnDlyTg0/1 by the lean hand-over before
/// anything else. Blobless WL lands exactly one WCK cycle (0x80) before the
/// PMU's TxWckDly; the read training (WCK always on) follows with RxEnDly
/// about 0x75 earlier. Unset: 0.
#[allow(non_snake_case)]
pub fn HO_TXWCK_ADD() -> u16 {
    hex_knob(crate::knobs::env(
        "A7A_HO_TXWCK_ADD",
        crate::knob!("A7A_HO_TXWCK_ADD"),
    ))
}
#[allow(non_snake_case)]
pub fn HO_RXEN_ADD() -> u16 {
    hex_knob(crate::knobs::env(
        "A7A_HO_RXEN_ADD",
        crate::knob!("A7A_HO_RXEN_ADD"),
    ))
}

const fn hex_knob(s: Option<&str>) -> u16 {
    let Some(s) = s else {
        return 0;
    };
    let b = s.as_bytes();
    let mut v: u32 = 0;
    let mut i = 0;
    if b.is_empty() || b.len() > 3 {
        panic!("A7A_HO_*_ADD is up to three hex digits");
    }
    while i < b.len() {
        v = v * 16
            + match hex_nib(b[i]) {
                Some(d) => d as u32,
                None => panic!("A7A_HO_*_ADD is up to three hex digits"),
            };
        i += 1;
    }
    v as u16
}

/// FEL lab `A7A_HO_MR2=<hex>`: after the lean hand-over, MR2 of every
/// channel and rank's operating set through the controller (MR16 0x05,
/// MR2, MR16 0x04), before any read: the hybrid test of the read latency
/// (the PMU leaves MR2 0x00). Unset: nothing.
#[allow(non_snake_case)]
pub fn HO_MR2() -> Option<u8> {
    match crate::knobs::env("A7A_HO_MR2", crate::knob!("A7A_HO_MR2")) {
        Some(s) => match u8::from_str_radix(s, 16) {
            Ok(v) => Some(v),
            Err(_) => panic!("A7A_HO_MR2 is a hex byte"),
        },
        None => None,
    }
}

/// The command-bus reference step of command bus training (feature
/// `full-speed`; declared here, below the code, so that no line of it
/// moves).
#[cfg(any(test, feature = "full-speed"))]
pub mod cavref;

#[cfg(test)]
mod tests {
    #[test]
    fn hex_pair_text() {
        use super::hex_pair;
        assert_eq!(hex_pair("05,04"), Some((5, 4)));
        assert_eq!(hex_pair("0c,5A"), Some((0x0c, 0x5a)));
        assert_eq!(hex_pair("5,4"), None);
        assert_eq!(hex_pair("05;04"), None);
        assert_eq!(hex_pair("0g,04"), None);
        assert_eq!(super::RD_FSPWR(), None);
        assert_eq!(
            crate::rdtrain::EXIT_MRS(),
            crate::phy::rxen::ModeMrs::EXIT_PSTATE1
        );
    }

    #[test]
    fn mission_switch_knobs_default_off() {
        assert_eq!(super::HO_WCKON(), None);
        assert_eq!(super::HO_DBICTL(), None);
        assert!(!super::HO_SRCYCLE());
        assert_eq!(super::DBICTL, 0xc94);
    }

    #[test]
    fn global_mr16_is_one_mrw_to_both_ranks() {
        use crate::phy::coldinit::{mrw, Row};
        use crate::phy::endinit::{run_with, skip, FspMrs, Variant};
        use crate::phy::wcklevel::{Channel, Step};
        let v = Variant {
            skip: skip::RANK_LAUNCH | skip::CBT_EXIT,
            mr16_switch: 0x05,
            ..Variant::REFERENCE
        };
        let mut launches: [[Row; 12]; 4] = [[Row::EMPTY; 12]; 4];
        let mut n = 0;
        run_with(
            Channel::B,
            |_, _| FspMrs { mr: [(0, 0); 20] },
            &v,
            &mut |s| {
                if let Step::Launch { rows, .. } = s {
                    launches[n][..rows.len()].copy_from_slice(rows);
                    n += 1;
                }
            },
        );
        // The switch, the settle and the close launches.
        assert_eq!(n, 3);
        assert_eq!(&launches[0][2..6], &mrw(3, 16, 0x05));
        // No other command row (CS) in any launch.
        for (k, l) in launches[..n].iter().enumerate() {
            for (i, r) in l.iter().enumerate() {
                let cs = r.0[0] >> 14;
                assert!(
                    cs == 0 || (k == 0 && (2..6).contains(&i)),
                    "launch {k} row {i}"
                );
            }
        }
    }

    /// A register model that answers every read with zero: the first launch
    /// of [`super::mr16_all`] then sees MicroResetL/MemResetL not in the
    /// required state and reports a failure.
    struct ZeroHw;

    impl crate::hw::RegIo for ZeroHw {
        fn read32(&mut self, _: usize) -> u32 {
            0
        }
        fn write32(&mut self, _: usize, _: u32) {}
        fn read16(&mut self, _: usize) -> u16 {
            0
        }
        fn write16(&mut self, _: usize, _: u16) {}
        fn udelay(&mut self, _: u32) {}
        fn now_us(&mut self) -> u64 {
            0
        }
        fn log(&mut self, _: core::fmt::Arguments) {}
    }

    impl crate::hw::Hw for ZeroHw {
        fn set_voltage(&mut self, _: crate::hw::Rail, _: u32) -> bool {
            true
        }
    }

    #[test]
    fn mr16_failure_blocks_the_mission_gate() {
        use super::StageResults;
        let failure = super::mr16_all(&mut ZeroHw, 0x04);
        assert!(
            failure.is_some(),
            "the fault injection must make the MR16 launch fail"
        );
        let stages = StageResults::new()
            .read_training_result(true)
            .mr16_before_result(failure)
            .mr16_after_result(None)
            .handover_result(true)
            .write_stage_result(true);
        assert!(!stages.mission_ok());
        // Every field is load-bearing.
        let clean = StageResults::new()
            .read_training_result(true)
            .mr16_before_result(None)
            .mr16_after_result(None)
            .handover_result(true)
            .write_stage_result(true);
        assert!(clean.mission_ok());
        assert!(!clean.write_stage_result(false).mission_ok());
        assert!(!clean.handover_result(false).mission_ok());
        assert!(!clean.read_training_result(false).mission_ok());
        assert!(!clean.mr16_after_result(Some(failure.unwrap())).mission_ok());
    }

    struct HandoverHw {
        swctl: [u32; 2],
        writes16: [(usize, u16); 256],
        writes16_len: usize,
    }

    impl HandoverHw {
        fn new() -> Self {
            Self {
                swctl: [0; 2],
                writes16: [(0, 0); 256],
                writes16_len: 0,
            }
        }

        fn recorded(&self) -> &[(usize, u16)] {
            &self.writes16[..self.writes16_len]
        }

        fn dfimrl_writes(&self) -> [(usize, Option<u16>); 4] {
            core::array::from_fn(|db| {
                let addr = crate::phy::csr(0x10000 + 0x1000 * db as u32);
                let last = self
                    .recorded()
                    .iter()
                    .rev()
                    .find(|(a, _)| *a == addr)
                    .map(|(_, v)| *v);
                (db, last)
            })
        }

        fn fiu_pulsed_after_last_dfimrl(&self) -> bool {
            let fiu = crate::phy::csr(0x200a3);
            let recs = self.recorded();
            let last_dfimrl = recs.iter().rposition(|(a, _)| {
                (0..4u32).any(|db| *a == crate::phy::csr(0x10000 + 0x1000 * db))
            });
            let Some(dfimrl_idx) = last_dfimrl else {
                return false;
            };
            let after = &recs[dfimrl_idx + 1..];
            let hi = after.iter().position(|&(a, v)| a == fiu && v == 1);
            let Some(hi_idx) = hi else {
                return false;
            };
            after[hi_idx + 1..].iter().any(|&(a, v)| a == fiu && v == 0)
        }
    }

    impl crate::hw::RegIo for HandoverHw {
        fn read32(&mut self, addr: usize) -> u32 {
            use crate::ctrl::{ddrc, DDRC_B88, SWSTAT};
            use dwc_ddrctl::regs::{DFISTAT, OPERATING_MODE_NORMAL, STAT};
            if addr == ddrc(0, SWSTAT) {
                self.swctl[0] & 1
            } else if addr == ddrc(1, SWSTAT) {
                self.swctl[1] & 1
            } else if addr == ddrc(0, DDRC_B88) {
                3 << 25
            } else if addr == ddrc(0, DFISTAT) || addr == ddrc(1, DFISTAT) {
                1
            } else if addr == ddrc(0, STAT) || addr == ddrc(1, STAT) {
                OPERATING_MODE_NORMAL
            } else {
                0
            }
        }
        fn write32(&mut self, addr: usize, val: u32) {
            use crate::ctrl::{ddrc, SWCTL};
            if addr == ddrc(0, SWCTL) {
                self.swctl[0] = val;
            } else if addr == ddrc(1, SWCTL) {
                self.swctl[1] = val;
            }
        }
        fn read16(&mut self, _: usize) -> u16 {
            0
        }
        fn write16(&mut self, addr: usize, val: u16) {
            assert!(self.writes16_len < self.writes16.len(), "write16 log full");
            self.writes16[self.writes16_len] = (addr, val);
            self.writes16_len += 1;
        }
        fn udelay(&mut self, _: u32) {}
        fn now_us(&mut self) -> u64 {
            0
        }
        fn log(&mut self, _: core::fmt::Arguments) {}
    }

    impl crate::hw::Hw for HandoverHw {
        fn set_voltage(&mut self, _: crate::hw::Rail, _: u32) -> bool {
            true
        }
    }

    static HANDOVER_LOCK: core::sync::atomic::AtomicBool =
        core::sync::atomic::AtomicBool::new(false);

    struct HandoverGuard;
    impl HandoverGuard {
        fn acquire() -> Self {
            use core::sync::atomic::Ordering;
            while HANDOVER_LOCK
                .compare_exchange_weak(false, true, Ordering::Acquire, Ordering::Relaxed)
                .is_err()
            {
                core::hint::spin_loop();
            }
            Self
        }
    }
    impl Drop for HandoverGuard {
        fn drop(&mut self) {
            #[cfg(feature = "lab-knobs")]
            crate::knobs::set(crate::knobs::BUILD);
            crate::rdtrain::APPLIED_MRL.store(0, core::sync::atomic::Ordering::Relaxed);
            HANDOVER_LOCK.store(false, core::sync::atomic::Ordering::Release);
        }
    }

    fn cxmt_4g() -> crate::para::Para {
        crate::para::Para::from_sysconfig(&crate::sets::PARAM_SETS[crate::sets::CXMT_4G].para)
    }

    #[test]
    fn handover_mrl_unset_writes_dfimrl_and_pulses_latch() {
        let _g = HandoverGuard::acquire();
        crate::rdtrain::APPLIED_MRL.store(4, core::sync::atomic::Ordering::Relaxed);
        let p = cxmt_4g();
        let mut hw = HandoverHw::new();
        let r = super::mission_handover_relock(&mut hw, &p, 0, false);
        assert!(r.is_ok(), "handover failed: {r:?}");
        assert_eq!(
            hw.dfimrl_writes(),
            [(0, Some(4)), (1, Some(4)), (2, Some(4)), (3, Some(4))]
        );
        assert!(
            hw.fiu_pulsed_after_last_dfimrl(),
            "ForceInternalUpdate not pulsed after DFIMRL writes"
        );
    }

    #[test]
    fn handover_mrl_unset_fails_closed_on_missing_applied_mrl() {
        let _g = HandoverGuard::acquire();
        let p = cxmt_4g();
        crate::rdtrain::APPLIED_MRL.store(0, core::sync::atomic::Ordering::Relaxed);
        let mut hw = HandoverHw::new();
        let r = super::mission_handover_relock(&mut hw, &p, 0, false);
        assert!(
            r.is_err(),
            "expected Err for missing APPLIED_MRL, got {r:?}"
        );
        assert_eq!(
            hw.dfimrl_writes(),
            [(0, None), (1, None), (2, None), (3, None)]
        );
        assert!(!hw.fiu_pulsed_after_last_dfimrl());
    }

    #[test]
    fn handover_mrl_unset_fails_closed_on_invalid_applied_mrl() {
        let _g = HandoverGuard::acquire();
        let p = cxmt_4g();
        for bad in [crate::rdtrain::MRL_CSR_MAX + 1, crate::rdtrain::NONE] {
            crate::rdtrain::APPLIED_MRL.store(bad, core::sync::atomic::Ordering::Relaxed);
            let mut hw = HandoverHw::new();
            let r = super::mission_handover_relock(&mut hw, &p, 0, false);
            assert!(
                r.is_err(),
                "expected Err for invalid APPLIED_MRL={bad:#x}, got {r:?}"
            );
            assert_eq!(
                hw.dfimrl_writes(),
                [(0, None), (1, None), (2, None), (3, None)]
            );
            assert!(!hw.fiu_pulsed_after_last_dfimrl());
        }
    }

    #[test]
    fn handover_mrl_add_zero_and_one_and_ho_csrs_override() {
        let _g = HandoverGuard::acquire();
        let p = cxmt_4g();

        // add = 0: validates, writes trained + 0, pulses latch.
        crate::rdtrain::APPLIED_MRL.store(4, core::sync::atomic::Ordering::Relaxed);
        let mut hw = HandoverHw::new();
        assert!(super::mission_handover_relock_with(&mut hw, &p, 0, false, 0, None).is_ok());
        assert_eq!(
            hw.dfimrl_writes(),
            [(0, Some(4)), (1, Some(4)), (2, Some(4)), (3, Some(4))]
        );
        assert!(hw.fiu_pulsed_after_last_dfimrl());

        // add = 1: validates, writes trained + 1, pulses latch.
        let mut hw = HandoverHw::new();
        assert!(super::mission_handover_relock_with(&mut hw, &p, 0, false, 1, None).is_ok());
        assert_eq!(
            hw.dfimrl_writes(),
            [(0, Some(5)), (1, Some(5)), (2, Some(5)), (3, Some(5))]
        );
        assert!(hw.fiu_pulsed_after_last_dfimrl());

        // add = 1 with overflow (0x1f + 1 > 5-bit CSR): fails closed.
        crate::rdtrain::APPLIED_MRL.store(
            crate::rdtrain::MRL_CSR_MAX,
            core::sync::atomic::Ordering::Relaxed,
        );
        let mut hw = HandoverHw::new();
        assert!(super::mission_handover_relock_with(&mut hw, &p, 0, false, 1, None).is_err());
        assert_eq!(
            hw.dfimrl_writes(),
            [(0, None), (1, None), (2, None), (3, None)]
        );

        // Explicit full-DBYTE A7A_HO_CSRS override stays separate from add
        // and works even when APPLIED_MRL is 0; partial override still fails
        // closed without APPLIED_MRL.
        crate::rdtrain::APPLIED_MRL.store(0, core::sync::atomic::Ordering::Relaxed);
        let mut hw = HandoverHw::new();
        assert!(super::mission_handover_relock_with(
            &mut hw,
            &p,
            0,
            false,
            0,
            Some("10000=5,11000=5,12000=5,13000=5"),
        )
        .is_ok());
        assert_eq!(
            hw.dfimrl_writes(),
            [(0, Some(5)), (1, Some(5)), (2, Some(5)), (3, Some(5))]
        );
        assert!(hw.fiu_pulsed_after_last_dfimrl());

        let mut hw = HandoverHw::new();
        assert!(super::mission_handover_relock_with(
            &mut hw,
            &p,
            0,
            false,
            0,
            Some("10000=5,11000=5"),
        )
        .is_err());
        assert_eq!(
            hw.dfimrl_writes(),
            [(0, None), (1, None), (2, None), (3, None)]
        );
    }

    #[cfg(feature = "lab-knobs")]
    #[test]
    fn handover_mrl_add_zero_writes_and_latches() {
        let _g = HandoverGuard::acquire();
        let p = cxmt_4g();
        let mut k = crate::knobs::BUILD;
        k.apply("A7A_HO_MRL_ADD=0").unwrap();
        crate::knobs::set(k);
        crate::rdtrain::APPLIED_MRL.store(4, core::sync::atomic::Ordering::Relaxed);
        let mut hw = HandoverHw::new();
        let r = super::mission_handover_relock(&mut hw, &p, 0, false);
        assert!(r.is_ok(), "add=0 failed: {r:?}");
        assert_eq!(
            hw.dfimrl_writes(),
            [(0, Some(4)), (1, Some(4)), (2, Some(4)), (3, Some(4))]
        );
        assert!(hw.fiu_pulsed_after_last_dfimrl());
    }

    #[cfg(feature = "lab-knobs")]
    #[test]
    fn handover_mrl_add_zero_fails_closed_on_missing_applied_mrl() {
        let _g = HandoverGuard::acquire();
        let p = cxmt_4g();
        let mut k = crate::knobs::BUILD;
        k.apply("A7A_HO_MRL_ADD=0").unwrap();
        crate::knobs::set(k);
        crate::rdtrain::APPLIED_MRL.store(0, core::sync::atomic::Ordering::Relaxed);
        let mut hw = HandoverHw::new();
        let r = super::mission_handover_relock(&mut hw, &p, 0, false);
        assert!(
            r.is_err(),
            "add=0 with missing APPLIED_MRL must fail closed"
        );
    }

    #[cfg(feature = "lab-knobs")]
    #[test]
    fn handover_mrl_add_one_writes_and_latches() {
        let _g = HandoverGuard::acquire();
        let p = cxmt_4g();
        let mut k = crate::knobs::BUILD;
        k.apply("A7A_HO_MRL_ADD=1").unwrap();
        crate::knobs::set(k);
        crate::rdtrain::APPLIED_MRL.store(4, core::sync::atomic::Ordering::Relaxed);
        let mut hw = HandoverHw::new();
        let r = super::mission_handover_relock(&mut hw, &p, 0, false);
        assert!(r.is_ok(), "add=1 failed: {r:?}");
        assert_eq!(
            hw.dfimrl_writes(),
            [(0, Some(5)), (1, Some(5)), (2, Some(5)), (3, Some(5))]
        );
        assert!(hw.fiu_pulsed_after_last_dfimrl());
    }

    #[cfg(feature = "lab-knobs")]
    #[test]
    fn handover_mrl_ho_csrs_override_preserved_with_add() {
        let _g = HandoverGuard::acquire();
        let p = cxmt_4g();
        let mut k = crate::knobs::BUILD;
        k.apply("A7A_HO_MRL_ADD=1").unwrap();
        k.apply("A7A_HO_CSRS=10000=6,11000=6,12000=6,13000=6")
            .unwrap();
        crate::knobs::set(k);
        crate::rdtrain::APPLIED_MRL.store(4, core::sync::atomic::Ordering::Relaxed);
        let mut hw = HandoverHw::new();
        let r = super::mission_handover_relock(&mut hw, &p, 0, false);
        assert!(r.is_ok(), "ho_csrs override failed: {r:?}");
        assert_eq!(
            hw.dfimrl_writes(),
            [(0, Some(6)), (1, Some(6)), (2, Some(6)), (3, Some(6))]
        );
        assert!(hw.fiu_pulsed_after_last_dfimrl());
    }
}
