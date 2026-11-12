// SPDX-License-Identifier: GPL-2.0-only
//! Bounded, opt-in held-reset observation of the TxWckDly CSRs and their
//! lane-field-0xf addresses, with no training launch. One boot answers:
//!
//! 1. Positive control: a direct write to TxWckDlyTg0 of DBYTE 0 (`0x1002a`)
//!    reads back the written value and leaves the other seven unchanged.
//!    Without it a "no change" result after an alias write means nothing.
//! 2. For each of the eight `0x1nf2a`/`0x1nf2b` addresses: one write of the
//!    matching base value with fine bit 0 flipped, then all eight base CSRs
//!    are read, restored and verified.
//! 3. Separately ([`read_aliases`]): plain reads of the eight alias
//!    addresses, last, so an aborting read cannot hide the write results.
//!
//! The base CSRs are named in ESWIN's EIC7700X TRM Part 2 (same DWC LPDDR5/4
//! PHY family): TxWckDlyTg0_pX at 0x1n02a, TxWckDlyTg1_pX at 0x1n02b, 11 bits,
//! coarse [10:6] UI (at most 20), fine [5:0] UI/64, reset value 0x200. The
//! TRM does not describe the 0xf lane field for these per-DBYTE registers;
//! the PMU firmware reference writes them, which is why this probe exists.
//! The blobless trainer writes the base CSRs directly and does not depend on
//! the answer.
use super::{csr, MICRO_CONT_MUX_SEL, MICRO_RESET};
use crate::hw::Hw;

/// TxWckDlyTg0/Tg1 of DBYTEs 0..3, in this order.
pub const DELAYS: [u32; 8] = [
    0x1002a, 0x1002b, 0x1102a, 0x1102b, 0x1202a, 0x1202b, 0x1302a, 0x1302b,
];
/// The same CSRs with lane field [11:8] = 0xf (`0x1nf2a`/`0x1nf2b`).
pub const ALIASES: [u32; 8] = {
    let mut a = [0; 8];
    let mut i = 0;
    while i < 8 {
        a[i] = DELAYS[i] | 0xf00;
        i += 1;
    }
    a
};
/// The address the PMU reference writes first (DBYTE 0, rank 0).
pub const TARGET_ALIAS: u32 = ALIASES[0];
const MAX_DELAY: u16 = super::wcklevel::MAX_WCK_DELAY;
const MEM_RESET_L: u32 = 0x20060;
const RUN_CTRL: u32 = 0x20020;

/// Upper bounds of one [`observe`] call (checked by the host controls).
pub const MAX_OBSERVE_READS: usize = 6 + 8 + 9 * 16 + 1;
pub const MAX_OBSERVE_WRITES: usize = 2 + 9 * 9 + 1;
/// Upper bounds of one [`read_aliases`] call.
pub const MAX_ALIAS_READS: usize = 6 + 16 + 1;
pub const MAX_ALIAS_WRITES: usize = 2 + 1;

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

/// One write and its effect on the eight base CSRs.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Trial {
    /// CSR written (a base CSR for the control, an alias otherwise).
    pub address: u32,
    pub written: u16,
    /// Base CSRs after the write, and which differ from the snapshot.
    pub after: [u16; 8],
    pub changed_mask: u8,
    /// Base CSRs after restoring the snapshot, and which still differ.
    pub restored: [u16; 8],
    pub restore_mismatch_mask: u8,
}

impl Trial {
    /// Only base CSR `index` changed, to the written value, and the snapshot
    /// came back.
    pub const fn lands_only_on(&self, index: usize) -> bool {
        self.changed_mask == 1 << index
            && self.after[index] == self.written
            && self.restore_mismatch_mask == 0
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Report {
    pub refusal: Option<Refusal>,
    pub before: Option<[u16; 8]>,
    /// Direct write to `DELAYS[0]`.
    pub control: Option<Trial>,
    /// Per alias address, in [`ALIASES`] order; None after a stop.
    pub aliases: [Option<Trial>; 8],
    /// Why the alias trials stopped early, if they did.
    pub stopped: Option<Stop>,
    /// (mux found, mux read back after restoring it).
    pub mux_restore: Option<(u16, u16)>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Stop {
    /// The positive control did not land exactly on its CSR.
    ControlFailed,
    /// Trial `n` (0 = control, 1..8 = alias n-1) left a CSR unrestored.
    RestoreMismatch(usize),
}

impl Report {
    const EMPTY: Self = Self {
        refusal: None,
        before: None,
        control: None,
        aliases: [None; 8],
        stopped: None,
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

fn diff(a: &[u16; 8], b: &[u16; 8]) -> u8 {
    (0..8).filter(|&i| a[i] != b[i]).fold(0, |m, i| m | 1 << i)
}

/// One write, then read, restore and verify all eight base CSRs. The
/// restore is unconditional, including unexpected cross-DBYTE changes.
fn trial(hw: &mut impl Hw, before: &[u16; 8], address: u32, written: u16) -> Trial {
    hw.write16(csr(address), written);
    let after = read_delays(hw);
    for (&a, &v) in DELAYS.iter().zip(before.iter()) {
        hw.write16(csr(a), v);
    }
    let restored = read_delays(hw);
    Trial {
        address,
        written,
        after,
        changed_mask: diff(&after, before),
        restored,
        restore_mismatch_mask: diff(&restored, before),
    }
}

/// Identity, PMU and mux checks, then PMU reset+stall (MicroReset 9) and host
/// CSR access, DRAM-in-reset and ACSM-idle checks. On success returns the
/// mux value to restore; on refusal the caller still restores the mux when
/// it was changed (`Err((refusal, Some(mux)))`).
fn enter(hw: &mut impl Hw, config: u16, revision: u16) -> Result<u16, (Refusal, Option<u16>)> {
    if config != 0x3a2 || revision != 0x230 {
        return Err((Refusal::Identity { config, revision }, None));
    }
    let micro = hw.read16(MICRO_RESET);
    if micro != 1 && micro != 9 {
        return Err((Refusal::PmuState(micro), None));
    }
    let mux = hw.read16(MICRO_CONT_MUX_SEL);
    if mux > 1 {
        return Err((Refusal::MuxState(mux), None));
    }
    hw.write16(MICRO_RESET, 9);
    let micro = hw.read16(MICRO_RESET);
    if micro != 9 {
        return Err((Refusal::PmuResetReadback(micro), None));
    }
    hw.write16(MICRO_CONT_MUX_SEL, 0);
    let host = hw.read16(MICRO_CONT_MUX_SEL);
    if host != 0 {
        return Err((Refusal::HostMuxReadback(host), Some(mux)));
    }
    let reset = hw.read16(csr(MEM_RESET_L));
    if reset != 2 {
        return Err((Refusal::DramReset(reset), Some(mux)));
    }
    let run = hw.read16(csr(RUN_CTRL));
    if run != 0 {
        return Err((Refusal::AcsmActive(run), Some(mux)));
    }
    Ok(mux)
}

fn leave(hw: &mut impl Hw, mux: u16) -> (u16, u16) {
    hw.write16(MICRO_CONT_MUX_SEL, mux);
    (mux, hw.read16(MICRO_CONT_MUX_SEL))
}

fn observe_held(hw: &mut impl Hw, report: &mut Report) {
    let before = read_delays(hw);
    report.before = Some(before);
    for (index, &value) in before.iter().enumerate() {
        // Rejects reserved high bits as well as unsupported coarse values.
        // Never mask unknown state into an apparently valid delay.
        if value > MAX_DELAY {
            report.refusal = Some(Refusal::UnsupportedDelay { index, value });
            return;
        }
    }
    // Fine bit 0 flipped: representable at both range ends, coarse kept.
    let control = trial(hw, &before, DELAYS[0], before[0] ^ 1);
    report.control = Some(control);
    if control.restore_mismatch_mask != 0 {
        report.stopped = Some(Stop::RestoreMismatch(0));
        return;
    }
    if !control.lands_only_on(0) {
        report.stopped = Some(Stop::ControlFailed);
        return;
    }
    for (k, slot) in report.aliases.iter_mut().enumerate() {
        let t = trial(hw, &before, ALIASES[k], before[k] ^ 1);
        *slot = Some(t);
        if t.restore_mismatch_mask != 0 {
            report.stopped = Some(Stop::RestoreMismatch(k + 1));
            return;
        }
    }
}

/// Call only after the firmware-free pre-training prefix, before any launch.
/// Refuses unknown identity/state. Asserts reset+stall (MicroReset=9), never
/// releases the PMU, keeps DRAM in reset, restores and verifies the CSR mux.
/// At most [`MAX_OBSERVE_WRITES`] writes and [`MAX_OBSERVE_READS`] reads:
/// one control write, up to eight alias writes, eight restoration writes
/// after each; no polling, ACSM memory/control write, MRW or DTSM.
/// Each MMIO access still depends on a responsive bus; the platform boot
/// watchdog/recovery must remain active. Restoration covers known CSRs only.
pub fn observe(hw: &mut impl Hw, config: u16, revision: u16) -> Report {
    let mut report = Report::EMPTY;
    match enter(hw, config, revision) {
        Ok(mux) => {
            observe_held(hw, &mut report);
            report.mux_restore = Some(leave(hw, mux));
        }
        Err((refusal, mux)) => {
            report.refusal = Some(refusal);
            report.mux_restore = mux.map(|m| leave(hw, m));
        }
    }
    report
}

/// Plain reads of the alias addresses, with the base CSRs read after them.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct AliasReads {
    pub refusal: Option<Refusal>,
    /// In [`ALIASES`] order.
    pub values: Option<[u16; 8]>,
    /// Base CSRs read after the alias reads (a read must not change them).
    pub base: Option<[u16; 8]>,
    pub mux_restore: Option<(u16, u16)>,
}

/// Read the eight alias addresses once each (no writes besides PMU hold and
/// mux), then the base CSRs. Run after [`observe`] and after its report is
/// printed: a read of an undecoded address could abort.
pub fn read_aliases(hw: &mut impl Hw, config: u16, revision: u16) -> AliasReads {
    let mut out = AliasReads {
        refusal: None,
        values: None,
        base: None,
        mux_restore: None,
    };
    match enter(hw, config, revision) {
        Ok(mux) => {
            let mut values = [0; 8];
            for (v, &a) in values.iter_mut().zip(ALIASES.iter()) {
                *v = hw.read16(csr(a));
            }
            out.values = Some(values);
            out.base = Some(read_delays(hw));
            out.mux_restore = Some(leave(hw, mux));
        }
        Err((refusal, mux)) => {
            out.refusal = Some(refusal);
            out.mux_restore = mux.map(|m| leave(hw, m));
        }
    }
    out
}
