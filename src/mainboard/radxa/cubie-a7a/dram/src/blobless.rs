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
    coldinit, csr, csr_access, freq, init, sequence, tables, MICRO_CONT_MUX_SEL, MICRO_RESET,
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

/// Outcome of [`cold_init_part1`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ColdInitReport {
    pub launches: u16,
    pub writes: u16,
    pub witnessed: u16,
    pub busy_max: u16,
    /// First failing launch (1-based) and its failure.
    pub failure: Option<(u16, sequence::Failure<PortError>)>,
    pub elapsed_us: u64,
}

/// Release the DRAM reset and run part 1 of the cold init
/// ([`phy::coldinit::part1`](crate::phy::coldinit::part1)) through the host
/// ACSM. Call after [`pre_training`]. Stops at the first failed launch.
pub fn cold_init_part1<H: Hw>(hw: &mut H, mrs: &coldinit::Mrs) -> ColdInitReport {
    let t0 = hw.now_us();
    csr_access(hw, true);
    hw.write16(csr(CLR_POR_MEM_RESET), 1);
    hw.write16(csr(CLR_POR_MEM_RESET), 0);
    hw.udelay(RESET_LOW_US);
    hw.write16(csr(MEM_RESET_L), 3);
    hw.udelay(RESET_HIGH_US);
    let mut report = ColdInitReport {
        launches: 0,
        writes: 0,
        witnessed: 0,
        busy_max: 0,
        failure: None,
        elapsed_us: 0,
    };
    coldinit::part1(mrs, &mut |step| {
        if report.failure.is_some() {
            return;
        }
        match step {
            coldinit::Step::Write(c, v) => {
                csr_access(hw, true);
                hw.write16(csr(c), v);
                report.writes += 1;
            }
            coldinit::Step::Launch { start, rows } => {
                let base = ROW_BASES[report.launches as usize & 1] + start;
                let stop = base + rows.len() as u16 - 1;
                csr_access(hw, true);
                hw.write16(csr(ACSM_START), base);
                hw.write16(csr(ACSM_STOP), base);
                for (i, row) in rows.iter().enumerate() {
                    for (k, &v) in row.0.iter().enumerate() {
                        let r = (base as u32 + i as u32) * 4 + k as u32;
                        hw.write16(csr(ACSM_MEM + r), v);
                    }
                }
                hw.write16(csr(ACSM_OUTER_LOOP), 0);
                hw.write16(csr(ACSM_STOP), stop);
                report.launches += 1;
                let mut run = sequence::DirectRun::new(PhyPort {
                    hw: &mut *hw,
                    stop,
                    prior_last: 0,
                    mem_reset: 3,
                });
                match run.run(sequence::MAX_POLLS) {
                    Ok(c) => {
                        report.witnessed += c.witnessed as u16;
                        report.busy_max = report.busy_max.max(c.busy_polls);
                    }
                    Err(f) => report.failure = Some((report.launches, f)),
                }
            }
        }
    });
    csr_access(hw, false);
    report.elapsed_us = hw.now_us().wrapping_sub(t0);
    report
}
