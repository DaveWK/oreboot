// SPDX-License-Identifier: GPL-2.0-only
//! `psci_el3::el3` against the board's main.rs excerpt
//! (tests/legacy/a7a_el3.rs): per-core EL3 set-up with the CPU errata over
//! every part/variant/revision combination of interest (same system
//! register writes in the same order), trap classification over random
//! ESRs, the unhandled-vector names, the EL3/EL2 constants and the
//! [`Context`] layout the vector code depends on.
#![allow(dead_code, clippy::all)]

use std::cell::RefCell;
use std::panic::{catch_unwind, AssertUnwindSafe};

use psci_el3::el3::{self, El3Regs, PatchSlot, Trap};

#[derive(Clone, Debug, PartialEq, Eq)]
enum Ev {
    Patch(u64, u64, u64, u64),
    Isb,
    Scr(u64),
    Cptr(u64),
    Mdcr(u64),
    Cntfrq(u64),
}

thread_local! {
    static TRACE: RefCell<Vec<Ev>> = const { RefCell::new(Vec::new()) };
    static MIDR: RefCell<u64> = const { RefCell::new(0) };
    static ESR: RefCell<u64> = const { RefCell::new(0) };
    static SMC: RefCell<u32> = const { RefCell::new(0) };
}

fn rec(e: Ev) {
    TRACE.with(|t| t.borrow_mut().push(e));
}

pub mod soc {
    pub const TIMER_HZ: u64 = 24_000_000;
}

// The board excerpt's calls out of its file.
fn model_midr() -> u64 {
    MIDR.with(|m| *m.borrow())
}
fn model_esr() -> u64 {
    ESR.with(|m| *m.borrow())
}
fn psci_handle_smc(_ctx: &mut legacy::Context) {
    SMC.with(|s| *s.borrow_mut() += 1)
}
fn halt_sync(ec: u64) -> ! {
    std::panic::panic_any(("sync", ec))
}
fn halt_unhandled(name: &'static str, from: &'static str) -> ! {
    std::panic::panic_any((name, from))
}
fn patch_slot(slot: u64, a: u64, b: u64, c: u64) {
    rec(Ev::Patch(slot, a, b, c))
}
fn isb() {
    rec(Ev::Isb)
}
fn write_scr(v: u64) {
    rec(Ev::Scr(v))
}
fn write_cptr(v: u64) {
    rec(Ev::Cptr(v))
}
fn write_mdcr(v: u64) {
    rec(Ev::Mdcr(v))
}
fn write_cntfrq(v: u64) {
    rec(Ev::Cntfrq(v))
}

#[path = "legacy/a7a_el3.rs"]
mod legacy;

struct Regs;
impl El3Regs for Regs {
    fn midr(&mut self) -> u64 {
        model_midr()
    }
    fn patch_slot(&mut self, p: &PatchSlot) {
        patch_slot(p.slot, p.opcode, p.mask, p.control)
    }
    fn isb(&mut self) {
        isb()
    }
    fn write_scr(&mut self, v: u64) {
        write_scr(v)
    }
    fn write_cptr(&mut self, v: u64) {
        write_cptr(v)
    }
    fn write_mdcr(&mut self, v: u64) {
        write_mdcr(v)
    }
    fn write_cntfrq(&mut self, v: u64) {
        write_cntfrq(v)
    }
}

fn take() -> Vec<Ev> {
    TRACE.with(|t| std::mem::take(&mut *t.borrow_mut()))
}

#[test]
fn el3_cpu_init_and_errata_match_the_board() {
    let mut applied = 0;
    for part in [0xd0bu64, 0xd05, 0xd41, 0xd0a, 0x000, 0xfff] {
        for variant in 0..16u64 {
            for rev in 0..16u64 {
                for implementer in [0x41u64, 0x00] {
                    let midr = implementer << 24 | variant << 20 | 0xf << 16 | part << 4 | rev;
                    MIDR.with(|m| *m.borrow_mut() = midr);
                    legacy::el3_cpu_init_for_test();
                    let a = take();
                    el3::el3_cpu_init(&mut Regs, soc::TIMER_HZ);
                    let b = take();
                    assert_eq!(a, b, "MIDR {midr:#x}");
                    assert_eq!(legacy::is_cortex_a76(), el3::is_cortex_a76(midr));
                    if a.iter().any(|e| matches!(e, Ev::Patch(..))) {
                        applied += 1;
                    }
                }
            }
        }
    }
    // A76 r3p0..r4p1 (0x30..=0x41): 16 + 2 revisions, both implementer values.
    assert_eq!(applied, 2 * (16 + 2));
}

#[test]
fn trap_classification_matches_the_board() {
    let mut x = 0x2545_f491_4f6c_dd1du64;
    let mut smc = 0;
    for i in 0..20_000u64 {
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        // Every exception class, random ISS.
        let esr = ((i % 64) << 26) | (x & 0x3ff_ffff);
        ESR.with(|e| *e.borrow_mut() = esr);
        let before = SMC.with(|s| *s.borrow());
        let mut ctx = legacy::new_context();
        let r = catch_unwind(AssertUnwindSafe(|| {
            legacy::el3_sync_lower_for_test(&mut ctx)
        }));
        let board = match r {
            Ok(()) => {
                assert_eq!(SMC.with(|s| *s.borrow()), before + 1);
                Trap::Smc
            }
            Err(p) => {
                let (tag, ec) = *p.downcast::<(&str, u64)>().unwrap();
                assert_eq!(tag, "sync");
                Trap::Other(ec)
            }
        };
        assert_eq!(board, el3::classify(esr), "ESR {esr:#x}");
        if board == Trap::Smc {
            smc += 1;
        }
    }
    assert!(smc > 0);
}

#[test]
fn unhandled_vector_names_match_the_board() {
    for kind in 0..16u64 {
        let p = catch_unwind(|| legacy::el3_unhandled_for_test(kind, 0, 0, 0)).unwrap_err();
        let (name, from) = *p.downcast::<(&str, &str)>().unwrap();
        assert_eq!(el3::unhandled_kind(kind), (name, from), "kind {kind}");
    }
}

#[test]
fn constants_and_context_layout_match_the_board() {
    assert_eq!(el3::SCR_EL3_VAL, legacy::consts::SCR_EL3_VAL);
    assert_eq!(el3::SCTLR_EL2_VAL, legacy::consts::SCTLR_EL2_VAL);
    assert_eq!(el3::HCR_EL2_VAL, legacy::consts::HCR_EL2_VAL);
    assert_eq!(el3::CPTR_EL2_VAL, legacy::consts::CPTR_EL2_VAL);
    assert_eq!(el3::CNTHCTL_EL2_VAL, legacy::consts::CNTHCTL_EL2_VAL);
    assert_eq!(el3::SPSR_EL2H, legacy::consts::SPSR_EL2H);
    // The reset entry's movz #0x1830 / movk #0x30c5, lsl #16.
    assert_eq!(el3::SCTLR_EL3_VAL, 0x1830 | 0x30c5 << 16);
    assert_eq!(std::mem::size_of::<el3::Context>(), el3::CONTEXT_SIZE);
    assert_eq!(
        std::mem::size_of::<legacy::Context>(),
        std::mem::size_of::<el3::Context>()
    );
    assert_eq!(std::mem::offset_of!(el3::Context, elr), 248);
    assert_eq!(std::mem::offset_of!(el3::Context, spsr), 256);
}
