// SPDX-License-Identifier: GPL-2.0-only
//! The receiver reference the PHY is initialized with.
//!
//! Step C's per-pstate program (`tables::C_PSTATE`) writes one VrefDAC code
//! to every receiver reference of the PHY, 156 CSRs
//! ([`is_vref_dac_csr`]), and stores it in the PState SRAM records that
//! replay them. No stage of a default build moves it afterwards (the read
//! reference stage runs only in a build that sets `A7A_RD_VREF`), so it is
//! the receiver reference every default boot reads with, at every DRAM
//! clock.
//!
//! The code in the table is [`VENDOR_VREF_DAC`]: not ours. Build
//! variable `A7A_VREF_INIT=<hex>` (one or two hex digits without `0x`,
//! `00..7f`, FEL lab) replaces it at all 156 CSRs, to measure whether a
//! code of our own does as well; unset, the table and the image are what
//! they were. In a build with feature `boot-profile`, `A7A_VREF_INIT=-`
//! ([`crate::knobs::profile::UNSET`]) also means unset through
//! [`crate::knob!`]; without `boot-profile`, `"-"` reaches [`parse`] raw
//! and is rejected as not hex.
//!
//! Register names and the 7-bit field are those of the EIC7700X TRM Part 2
//! register list (public; another SoC with the same DWC LPDDR5/4 PHY
//! family, see `sequence.rs` for what that correspondence does and does not
//! qualify): VrefDAC0..3_rY_pX `0x1004e..0x10051` and AcVrefDAC0..3_pX
//! `0x30050..0x30053`, "DAC Control for Internal VREF", and
//! RxDiffSeVrefDACDIFF0_pX `0x100e9`, the same for the differential
//! (DQS) receivers in single-ended mode. That list gives 0x40 as the value
//! after reset of each; on the A733 these 156 CSRs have not been read
//! before step C writes them, so the reset value here is not measured.

// Nothing of this module is compiled into an image: its functions are
// `#[inline]` and used at compile time only (a function of this crate
// that is not would change how the crate's code is split and so the
// other images, though never called), and the derives are the tests'. A
// default image is therefore the one from before the module existed (as
// for `crate::knobs::profile`).

/// The VrefDAC code the vendor library programs in step C, at all 156
/// CSRs. REFERENCE-DERIVED like the rest of `tables::C_PSTATE` (its bus
/// events): taken over, not derived and not measured by us. It is the
/// receiver reference of every build that does not set `A7A_VREF_INIT`.
pub const VENDOR_VREF_DAC: u16 = 0x0035;

/// The largest VrefDAC code: the field is 7 bits.
pub const VREF_DAC_MAX: u16 = crate::rdvref::DAC_MAX;

/// The receiver reference CSRs of the PHY, all written by
/// `tables::C_PSTATE`.
pub const VREF_DAC_CSRS: usize = 2 * 4 + 4 * 9 * 4 + 4;

/// Whether `csr` is one of the [`VREF_DAC_CSRS`]:
///
/// - AcVrefDAC0..3 of the two AC blocks, `0x3m050..0x3m053`;
/// - VrefDAC0..3 of the nine lanes (eight DQ and DMI) of the four DBYTEs,
///   `0x1n04e..0x1n051 + 0x100 * lane`;
/// - RxDiffSeVrefDACDIFF0 of the four DBYTEs, `0x1n0e9`.
#[inline]
pub const fn is_vref_dac_csr(csr: u32) -> bool {
    let reg = csr & 0xff;
    let lane = (csr >> 8) & 0xf;
    let unit = (csr >> 12) & 0xf;
    match csr >> 16 {
        0x3 => unit < 2 && lane == 0 && reg >= 0x50 && reg <= 0x53,
        0x1 => unit < 4 && ((lane < 9 && reg >= 0x4e && reg <= 0x51) || (lane == 0 && reg == 0xe9)),
        _ => false,
    }
}

/// Why the text of `A7A_VREF_INIT` is refused.
#[cfg_attr(test, derive(Clone, Copy, Debug, PartialEq, Eq))]
pub enum VrefInitError {
    /// Not one or two hex digits.
    NotHex,
    /// Above [`VREF_DAC_MAX`].
    OutOfRange,
}

impl VrefInitError {
    /// The build's diagnostic.
    #[inline]
    pub const fn message(self) -> &'static str {
        match self {
            VrefInitError::NotHex => {
                "A7A_VREF_INIT is not one or two hex digits (a VrefDAC code without 0x, 00..7f)"
            }
            VrefInitError::OutOfRange => {
                "A7A_VREF_INIT is above 7f: the VrefDAC field is 7 bits (00..7f)"
            }
        }
    }
}

/// The VrefDAC code step C writes in a build whose resolved
/// `A7A_VREF_INIT` knob is `text`: `None` gives [`VENDOR_VREF_DAC`]; else
/// the code `text` names, one or two hex digits without `0x`, at most
/// [`VREF_DAC_MAX`] (`00..7f`). Anything else is refused (the build fails),
/// never ignored or clamped.
///
/// With feature `boot-profile`, [`crate::knob!`] runs
/// [`crate::knobs::profile::resolve`] before this parser, mapping
/// [`crate::knobs::profile::UNSET`] (`"-"`) in the environment to `None`
/// (so `A7A_VREF_INIT=-` yields [`VENDOR_VREF_DAC`]). Without
/// `boot-profile`, [`crate::knob!`] passes `option_env!` raw, so `"-"` is
/// refused here with [`VrefInitError::NotHex`].
#[inline]
pub const fn parse(text: Option<&str>) -> Result<u16, VrefInitError> {
    let Some(text) = text else {
        return Ok(VENDOR_VREF_DAC);
    };
    let b = text.as_bytes();
    if b.is_empty() || b.len() > 2 {
        return Err(VrefInitError::NotHex);
    }
    let mut v: u16 = 0;
    let mut i = 0;
    while i < b.len() {
        let d = match b[i] {
            c @ b'0'..=b'9' => c - b'0',
            c @ b'a'..=b'f' => c - b'a' + 10,
            c @ b'A'..=b'F' => c - b'A' + 10,
            _ => return Err(VrefInitError::NotHex),
        };
        v = v * 16 + d as u16;
        i += 1;
    }
    if v > VREF_DAC_MAX {
        return Err(VrefInitError::OutOfRange);
    }
    Ok(v)
}

/// The VrefDAC code for a build whose resolved `A7A_VREF_INIT` knob (as
/// returned by [`crate::knob!`]) is `resolved`. Panics at compile time
/// with [`VrefInitError::message`] if [`parse`] rejects `resolved`.
#[inline]
pub const fn value(resolved: Option<&str>) -> u16 {
    match parse(resolved) {
        Ok(v) => v,
        Err(why) => panic!("{}", why.message()),
    }
}

/// The VrefDAC code of this build's `tables::C_PSTATE` ([`value`] of
/// `crate::knob!("A7A_VREF_INIT")`). The table is a constant of the image,
/// so the knob is a build variable only: the lab mode's run-time knobs
/// (`crate::knobs::EXTRA`) do not have it.
pub const VREF_DAC_INIT: u16 = value(crate::knob!("A7A_VREF_INIT"));

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use crate::knobs::profile::{BOOT_1200, BOOT_2400, BOOT_400, BOOT_800};
    use crate::para::Para;
    use crate::phy::prog::{self, Kind, Op};
    use crate::phy::{csr, freq, init, tables};
    use crate::sets::{CXMT_4G, CXMT_6G, HYNIX_4G, PARAM_SETS};
    use std::collections::{BTreeMap, BTreeSet};
    use std::vec::Vec;

    /// The 156 CSRs, listed from the block layout instead of by
    /// [`is_vref_dac_csr`].
    fn listed() -> BTreeSet<u32> {
        let mut s = BTreeSet::new();
        for ac in 0..2 {
            for reg in 0x50..=0x53 {
                s.insert(0x30000 + 0x1000 * ac + reg);
            }
        }
        for db in 0..4 {
            for lane in 0..9 {
                for reg in 0x4e..=0x51 {
                    s.insert(0x10000 + 0x1000 * db + 0x100 * lane + reg);
                }
            }
            s.insert(0x10000 + 0x1000 * db + 0xe9);
        }
        s
    }

    /// Every field of an op but its value.
    fn shape(o: &Op) -> (Kind, u8, u32, u16, u16, u16, u32, u32, u8) {
        (o.kind, o.flags, o.csr, o.hi, o.a, o.b, o.xr, o.xs, o.xen)
    }

    /// Records every 16-bit write in order; nothing else happens in a
    /// program without firmware blobs.
    #[derive(Default)]
    struct Recorder {
        writes: Vec<(usize, u16)>,
        delays: Vec<u32>,
    }

    impl crate::hw::RegIo for Recorder {
        fn read32(&mut self, addr: usize) -> u32 {
            panic!("read32 {addr:#x}")
        }
        fn write32(&mut self, addr: usize, _: u32) {
            panic!("write32 {addr:#x}")
        }
        fn read16(&mut self, addr: usize) -> u16 {
            panic!("read16 {addr:#x}")
        }
        fn write16(&mut self, addr: usize, val: u16) {
            self.writes.push((addr, val));
        }
        fn udelay(&mut self, us: u32) {
            self.delays.push(us);
        }
        fn now_us(&mut self) -> u64 {
            0
        }
        fn log(&mut self, _: core::fmt::Arguments) {}
    }

    impl crate::hw::Hw for Recorder {
        fn set_voltage(&mut self, _: crate::hw::Rail, _: u32) -> bool {
            panic!("set_voltage")
        }
    }

    fn run(table: &[Op], c: &prog::Ctx) -> Recorder {
        let mut hw = Recorder::default();
        prog::run(&mut hw, table, c, freq::config);
        hw
    }

    #[test]
    fn knob_text() {
        assert_eq!(parse(None), Ok(VENDOR_VREF_DAC));
        assert_eq!(VENDOR_VREF_DAC, 0x35);
        for (text, want) in [
            ("0", 0x00),
            ("00", 0x00),
            ("7", 0x07),
            ("33", 0x33),
            ("34", 0x34),
            ("35", 0x35),
            ("40", 0x40),
            ("7f", 0x7f),
            ("7F", 0x7f),
        ] {
            assert_eq!(parse(Some(text)), Ok(want), "{text}");
        }
        // Every code of the field, and nothing above it.
        for v in 0..=0xffu16 {
            let text = std::format!("{v:02x}");
            let want = if v <= VREF_DAC_MAX {
                Ok(v)
            } else {
                Err(VrefInitError::OutOfRange)
            };
            assert_eq!(parse(Some(&text)), want, "{text}");
        }
        assert_eq!(VREF_DAC_MAX, 0x7f);
        for text in ["80", "ff", "FF", "a0"] {
            assert_eq!(parse(Some(text)), Err(VrefInitError::OutOfRange), "{text}");
        }
        for text in [
            "", " ", "-", "0x40", "040", "100", "4g", "g", "4 ", " 4", "+4", "-1", "64d", "0x",
        ] {
            assert_eq!(parse(Some(text)), Err(VrefInitError::NotHex), "{text:?}");
        }
        for e in [VrefInitError::NotHex, VrefInitError::OutOfRange] {
            assert!(e.message().starts_with("A7A_VREF_INIT "));
        }
    }

    /// This build's table is [`tables::c_pstate`] of this build's knob, and
    /// without the knob that is the vendor library's code. No boot profile
    /// sets the knob, so a default image has it only from the environment.
    #[test]
    fn build_value() {
        assert_eq!(parse(option_env!("A7A_VREF_INIT")), Ok(VREF_DAC_INIT));
        if option_env!("A7A_VREF_INIT").is_none() {
            assert_eq!(VREF_DAC_INIT, VENDOR_VREF_DAC);
        }
        for p in [&BOOT_1200, &BOOT_2400, &BOOT_800, &BOOT_400] {
            assert_eq!(p.get("A7A_VREF_INIT"), None, "profile {}", p.mhz);
        }
        assert!(!crate::knobs::EXTRA.contains(&"A7A_VREF_INIT"));
        let want = tables::c_pstate(VREF_DAC_INIT);
        assert_eq!(tables::C_PSTATE.len(), tables::C_PSTATE_OPS);
        for (i, (got, want)) in tables::C_PSTATE.iter().zip(&want).enumerate() {
            assert_eq!((shape(got), got.val), (shape(want), want.val), "op {i}");
        }
    }

    /// The table has one plain write per receiver reference CSR, 156 of
    /// them, and the code reaches exactly those ops: for every code of the
    /// field the table differs from the vendor library's in the value of
    /// those 156 ops and in nothing else.
    #[test]
    fn code_reaches_the_156_writes_only() {
        let listed = listed();
        assert_eq!(listed.len(), VREF_DAC_CSRS);
        assert_eq!(VREF_DAC_CSRS, 156);
        for csr in 0..0x10_0000u32 {
            assert_eq!(is_vref_dac_csr(csr), listed.contains(&csr), "{csr:05x}");
        }
        let base = tables::c_pstate(VENDOR_VREF_DAC);
        // The sites: every write to one of the CSRs, once each, plain.
        let sites: Vec<usize> = (0..base.len())
            .filter(|&i| base[i].kind == Kind::Write && is_vref_dac_csr(base[i].csr))
            .collect();
        let written: BTreeSet<u32> = sites.iter().map(|&i| base[i].csr).collect();
        assert_eq!(sites.len(), VREF_DAC_CSRS);
        assert_eq!(written, listed);
        for &i in &sites {
            let o = &base[i];
            assert_eq!(o.val, VENDOR_VREF_DAC, "op {i}");
            assert_eq!((o.flags, o.xr, o.xs, o.xen), (0, 0, 0, 0), "op {i}");
        }
        for v in 0..=VREF_DAC_MAX {
            let t = tables::c_pstate(v);
            for i in 0..base.len() {
                assert_eq!(shape(&t[i]), shape(&base[i]), "code {v:02x} op {i}");
                let want = if sites.contains(&i) { v } else { base[i].val };
                assert_eq!(t[i].val, want, "code {v:02x} op {i}");
            }
        }
    }

    /// What the PHY is sent, per board parameter set and pstate: with
    /// another code the program makes the same writes to the same
    /// addresses in the same order, and the values differ only where the
    /// vendor library's code was written to a receiver reference CSR (all
    /// 156, once each) or stored as that CSR's half of its PState SRAM
    /// record (all 156, once each).
    #[test]
    fn writes_differ_at_the_references_and_their_records_only() {
        let listed = listed();
        let base_table = tables::c_pstate(VENDOR_VREF_DAC);
        let mut ranks = BTreeSet::new();
        let mut clocks = BTreeSet::new();
        for set in [CXMT_4G, CXMT_6G, HYNIX_4G] {
            let p = Para::from_sysconfig(&PARAM_SETS[set].para);
            for ps in 0..4u32 {
                let c = init::ctx_with(&p, [&[]; 4], ps);
                ranks.insert(c.dual_rank);
                clocks.insert(c.data_mhz);
                let base = run(&base_table, &c);
                let sram = csr(0xa0000 + 0x1000 * ps)..csr(0xa0000 + 0x1000 * (ps + 1));
                // Each record's address word and enables, by its first
                // CSR's address (records are written whole, once).
                let mut cells = BTreeMap::new();
                for &(addr, val) in &base.writes {
                    if sram.contains(&addr) {
                        assert!(cells.insert(addr, val).is_none(), "{addr:#x} twice");
                    }
                }
                for v in [0x00, 0x33, 0x34, 0x40, VREF_DAC_MAX] {
                    let what = std::format!("set {set} ps {ps} code {v:02x}");
                    let got = run(&tables::c_pstate(v), &c);
                    assert_eq!(got.delays, base.delays, "{what}");
                    assert_eq!(got.writes.len(), base.writes.len(), "{what}");
                    let mut direct = BTreeSet::new();
                    let mut recorded = BTreeSet::new();
                    for (k, (g, b)) in got.writes.iter().zip(&base.writes).enumerate() {
                        assert_eq!(g.0, b.0, "{what} write {k}: address");
                        if g.1 == b.1 {
                            continue;
                        }
                        assert_eq!((b.1, g.1), (VENDOR_VREF_DAC, v), "{what} write {k}");
                        let addr = g.0;
                        if sram.contains(&addr) {
                            // Record: low half, high half, address word,
                            // enables with the word's high bits.
                            let slot = sram.start + (addr - sram.start) / 8 * 8;
                            let half = (addr - slot) / 2;
                            assert!(half < 2, "{what} write {k}: not a data half");
                            let word =
                                cells[&(slot + 4)] as u32 | (cells[&(slot + 6)] as u32 & 7) << 16;
                            let enables = cells[&(slot + 6)] >> 3;
                            assert!(enables & (1 << half) != 0, "{what} write {k}");
                            let of = 2 * word + half as u32;
                            assert!(listed.contains(&of), "{what} write {k}: {of:05x}");
                            assert!(recorded.insert(of), "{what}: {of:05x} twice");
                        } else {
                            let of = *listed
                                .iter()
                                .find(|&&c| csr(c) == addr)
                                .unwrap_or_else(|| panic!("{what} write {k}: {addr:#x}"));
                            assert!(direct.insert(of), "{what}: {of:05x} twice");
                        }
                    }
                    assert_eq!(direct, listed, "{what}");
                    assert_eq!(recorded, listed, "{what}");
                }
                // The same code: the same writes.
                let same = run(&tables::c_pstate(VENDOR_VREF_DAC), &c);
                assert_eq!(same.writes, base.writes);
            }
        }
        // Both rank counts and every DRAM clock of the profiles were run.
        assert_eq!(ranks.len(), 2);
        for mhz in [2400, 1200, 800, 400] {
            assert!(clocks.contains(&mhz), "{mhz}");
        }
    }
}
