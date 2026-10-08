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
    let c = init::ctx_with(p, [&[]; 4], pstate);
    let rl = if p.tpr10() & (1 << 31) != 0 { 14 } else { 13 };
    let m = crate::phy::msgblock::build(p, pstate, c.data_mhz, rl);
    (
        coldinit::Mrs::from_msgblock(&m),
        coldinit::Mrs2::from_msgblock(&m),
    )
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

/// Control words of the delay-only rows seen in the PMU firmware's DevInit
/// launches (emulator trace), shortest-looking first.
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
    let full = u64::MAX >> (64 - LANE_SWEEP_POINTS);
    let set = &blocks[4 * ch..4 * ch + 4];
    let mut out = ChannelCa {
        lane_map_agrees: true,
        ..ChannelCa::EMPTY
    };
    for lane in 0..7 {
        let bit = set[0].lane_bit[lane];
        let mut map = full;
        for b in set {
            if b.lane_bit[lane] != bit || bit == 0xff {
                out.lane_map_agrees = false;
            }
            map &= b.lane_maps[lane];
        }
        let pref = set[0].lane_delay[lane] as u32 / LANE_SWEEP_STEP as u32;
        out.runs[lane] = coldinit::run_near(map as u128, LANE_SWEEP_POINTS, pref);
        let (s, l) = out.runs[lane];
        out.delay[lane] = if l == 0 {
            set[0].lane_delay[lane]
        } else {
            (((s + l / 2) % LANE_SWEEP_POINTS) as u16) * LANE_SWEEP_STEP
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
            coldinit::set_block_delays(c.ac, &ca.delay, &mut |s| ex.step(s));
        }
    }
    out.exec = ex.r;
    csr_access(hw, false);
    out.exec.elapsed_us = hw.now_us().wrapping_sub(t0);
}
