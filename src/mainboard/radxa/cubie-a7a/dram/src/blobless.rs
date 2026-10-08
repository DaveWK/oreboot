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
use crate::phy::{csr, csr_access, freq, init, tables, MICRO_CONT_MUX_SEL, MICRO_RESET};
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
