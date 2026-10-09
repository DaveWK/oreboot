// SPDX-License-Identifier: GPL-2.0-only
//! Bounded, opt-in F-to-base readback observation, with no training launch.
//! Only DBYTE0/rank0's observed F address is perturbed. All eight documented
//! rank delay CSRs are captured and restored. Undocumented lanes and analog
//! timing remain unqualified even when the known readbacks change as expected.
use super::{csr, MICRO_CONT_MUX_SEL, MICRO_RESET};
use crate::hw::Hw;

pub const DELAYS: [u32; 8] = [
    0x1002a, 0x1002b, 0x1102a, 0x1102b, 0x1202a, 0x1202b, 0x1302a, 0x1302b,
];
pub const TARGET_ALIAS: u32 = 0x10f2a;
const MAX_DELAY: u16 = super::wcklevel::MAX_WCK_DELAY;
const MEM_RESET_L: u32 = 0x20060;
const RUN_CTRL: u32 = 0x20020;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Refusal {
    Identity { config: u16, revision: u16 },
    PmuState(u16),
    MuxState(u16),
    PmuResetReadback(u16),
    HostMuxReadback(u16),
    DramReset(u16),
    AcsmActive(u16),
    UnsupportedDelay { index: usize, value: u16 },
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Report {
    pub refusal: Option<Refusal>,
    pub before: Option<[u16; 8]>,
    pub after: Option<[u16; 8]>,
    pub restored: Option<[u16; 8]>,
    pub perturbation: Option<u16>,
    pub changed_mask: u8,
    pub restore_mismatch_mask: u8,
    pub mux_restore: Option<(u16, u16)>,
}

impl Report {
    const EMPTY: Self = Self {
        refusal: None,
        before: None,
        after: None,
        restored: None,
        perturbation: None,
        changed_mask: 0,
        restore_mismatch_mask: 0,
        mux_restore: None,
    };
}

fn read_delays(hw: &mut impl Hw) -> [u16; 8] {
    let mut values = [0; 8];
    for (value, &address) in values.iter_mut().zip(DELAYS.iter()) {
        *value = hw.read16(csr(address));
    }
    values
}

fn with_host_mux(hw: &mut impl Hw, report: &mut Report) {
    let reset = hw.read16(csr(MEM_RESET_L));
    if reset != 2 {
        report.refusal = Some(Refusal::DramReset(reset));
        return;
    }
    let run = hw.read16(csr(RUN_CTRL));
    if run != 0 {
        report.refusal = Some(Refusal::AcsmActive(run));
        return;
    }
    let before = read_delays(hw);
    report.before = Some(before);
    for (index, &value) in before.iter().enumerate() {
        // This rejects all reserved high bits as well as unsupported coarse
        // values. Never mask unknown state into an apparently valid delay.
        if value > MAX_DELAY {
            report.refusal = Some(Refusal::UnsupportedDelay { index, value });
            return;
        }
    }
    // Minimal fine-field change, preserving coarse bits even at the limit.
    let perturbation = before[0] ^ 1;
    hw.write16(csr(TARGET_ALIAS), perturbation);
    report.perturbation = Some(perturbation);
    let after = read_delays(hw);
    report.after = Some(after);
    for i in 0..8 {
        if after[i] != before[i] {
            report.changed_mask |= 1 << i;
        }
    }
    // Unconditional restoration of every known delay, including unexpected
    // cross-rank/DBYTE changes. No early return after the F write.
    for (&address, &value) in DELAYS.iter().zip(before.iter()) {
        hw.write16(csr(address), value);
    }
    let restored = read_delays(hw);
    report.restored = Some(restored);
    for i in 0..8 {
        if restored[i] != before[i] {
            report.restore_mismatch_mask |= 1 << i;
        }
    }
}

/// Call only after the firmware-free pre-training prefix, before any launch.
/// Refuse unknown identity/state. Assert reset+stall (MicroReset=9), never
/// release the PMU, preserve/verify the CSR mux, and keep DRAM in reset.
/// Exactly one alias write, eight restoration writes and finite readbacks;
/// no polling, ACSM memory/control write, MRW, DTSM or trained result.
/// Each MMIO callback still depends on a responsive bus; the platform boot
/// watchdog/recovery must remain active. Restoration covers known CSRs only.
pub fn observe(hw: &mut impl Hw, config: u16, revision: u16) -> Report {
    let mut report = Report::EMPTY;
    if config != 0x3a2 || revision != 0x230 {
        report.refusal = Some(Refusal::Identity { config, revision });
        return report;
    }
    let micro = hw.read16(MICRO_RESET);
    if micro != 1 && micro != 9 {
        report.refusal = Some(Refusal::PmuState(micro));
        return report;
    }
    let mux = hw.read16(MICRO_CONT_MUX_SEL);
    if mux > 1 {
        report.refusal = Some(Refusal::MuxState(mux));
        return report;
    }
    hw.write16(MICRO_RESET, 9);
    let micro = hw.read16(MICRO_RESET);
    if micro != 9 {
        report.refusal = Some(Refusal::PmuResetReadback(micro));
        return report;
    }
    hw.write16(MICRO_CONT_MUX_SEL, 0);
    let host = hw.read16(MICRO_CONT_MUX_SEL);
    if host != 0 {
        report.refusal = Some(Refusal::HostMuxReadback(host));
    } else {
        with_host_mux(hw, &mut report);
    }
    hw.write16(MICRO_CONT_MUX_SEL, mux);
    report.mux_restore = Some((mux, hw.read16(MICRO_CONT_MUX_SEL)));
    report
}
