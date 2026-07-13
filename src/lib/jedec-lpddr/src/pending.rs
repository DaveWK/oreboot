// SPDX-License-Identifier: GPL-2.0-only
//! JEDEC values this crate does not have yet: **pending spec**.
//!
//! Every value here is left out on purpose rather than guessed: neither our
//! board code nor the cited LiteDRAM transcription (74522f715d) of JEDEC LPDDR4/4X
//! and JEDEC LPDDR5/5X gives it. Dave chose free DRAM-part datasheets first (the
//! JEDEC PDFs are paid): they are being collected in the orchestrator's
//! `references/datasheets/` with a README mapping each document to these
//! items. When a value is filled in, its source document and table go in a
//! comment next to it, and its entry leaves this list.
//!
//! P1-39 (Oct 14, 2026) filled, from the free sources of the orchestrator's
//! `references/datasheets/README.md` section 3: LPDDR4 tRFCab/pb per density,
//! tRPab/pb, tXSR, the init timings, MR13, the MRW/MRR CA encoding; LPDDR5
//! tRFCab/pb and tpbR2pbR per density, tRRD/tFAW per bank mode, tCCD (BG,
//! 4:1, BL16), MR26, MR30 and a partial MR8 decode.
//!
//! P1-40 (Oct 14, 2026) filled every remaining LPDDR4/4X item (tXP, tCKE,
//! tCKELPD, tMRW, tMRD, tZQCAL/LAT/RESET, tVREF and the CBT tVREFca, tVRCG,
//! tFC, tCKFSPE/X, tXCBT, tSDO; MR4, MR18/19, MR22, MR23, the MPC opcodes
//! and the RD-DQ-CAL defaults) and the LPDDR5/5X timings, mode-register
//! layouts, code tables and MPC/FIFO/RDC/CAS encodings. Those are cited
//! only as "JEDEC LPDDR4/4X value" or "JEDEC LPDDR5 value" (LPDDR5X).

/// One missing value or field layout.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Pending {
    /// Where it belongs in this crate.
    pub module: &'static str,
    /// What is missing.
    pub item: &'static str,
}

const fn p(module: &'static str, item: &'static str) -> Pending {
    Pending { module, item }
}

/// LPDDR4/4X values pending spec (JEDEC LPDDR4/4X or the part datasheets).
pub const LP4: &[Pending] = &[];

/// LPDDR5 values pending spec (JEDEC LPDDR5/5X or the part datasheets).
pub const LP5: &[Pending] = &[
    p(
        "lp5::timing",
        "tWCK2CK and tWCK2CK_Leveling picosecond caps (TBD in JEDEC)",
    ),
    p(
        "lp5",
        "CA and DQ Vref training procedures (TBD in JEDEC)",
    ),
    p(
        "lp5::timing",
        "enhanced-DVFSC refresh for densities other than 16 Gb; tPBR2ACT for densities other than 12 Gb",
    ),
];

#[cfg(test)]
mod tests {
    #[test]
    fn every_entry_names_a_module_and_an_item() {
        for e in super::LP4.iter().chain(super::LP5) {
            assert!(!e.module.is_empty() && !e.item.is_empty());
        }
    }
}
