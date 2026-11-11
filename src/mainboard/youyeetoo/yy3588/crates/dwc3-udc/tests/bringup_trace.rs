// SPDX-License-Identifier: GPL-2.0-only
//! The register program of `Udc::init` against a reviewed reference trace
//! (tests/data/bringup.trace, annotated with the U-Boot step and TRM page
//! of every access), on the behavioural model with the BootROM's session
//! still running. The event buffer address is a host address and masked.
//!
//! `DWC3_BLESS=1 cargo test --test bringup_trace` rewrites the reference
//! without its annotations (review it, then re-annotate).

mod model;

use dwc3_udc::{regs::*, Config, Descriptors, Udc};
use model::{dma, plat, Model, BASE, SNPSID_300A, SNPSID_330A};
use regtrace::{compare, parse, render, CompareOpts, Ev};

const DESC: Descriptors = Descriptors {
    vendor: 0x18d1,
    product: 0x4ee0,
    class: [0xff, 0x42, 3],
    strings: ["oreboot", "yy3588", "0123", "fastboot"],
};

fn bringup() -> Vec<Ev> {
    bringup_rev(SNPSID_300A)
}

fn bringup_rev(id: u32) -> Vec<Ev> {
    let mut m = Model::with_snpsid(id);
    // The BootROM's MaskROM device: running, addressed, one pending event.
    m.regs.insert(DCTL, 0x00f0_0000 | DCTL_RUN_STOP);
    m.regs.insert(DCFG, 0x0008_0000 | (3 << 3));
    m.stale_events = 1;
    let mut u = Udc::init(m, plat(), Config::new(BASE), dma(), DESC).expect("init");
    std::mem::take(&mut u.io().evs)
}

fn opts(mask_id: bool) -> CompareOpts {
    let mut o = CompareOpts {
        field_masks: vec![
            ((BASE + GEVNTADRLO) as u64, 0),
            ((BASE + GEVNTADRHI) as u64, 0),
            // STARTTRANSFER parameters: the SETUP TRB's host address.
            ((BASE + depcmdpar0(0)) as u64, 0),
            ((BASE + depcmdpar1(0)) as u64, 0),
        ],
        ..CompareOpts::default()
    };
    if mask_id {
        o.field_masks.push(((BASE + GSNPSID) as u64, 0));
    }
    o
}

#[test]
fn bringup_matches_the_reference() {
    let got = bringup();
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/bringup.trace");
    if std::env::var_os("DWC3_BLESS").is_some() {
        let text: String = got.iter().map(|e| format!("{e}\n")).collect();
        std::fs::write(path, text).unwrap();
    }
    let want = parse(&std::fs::read_to_string(path).unwrap()).unwrap();
    if let Some(diff) = compare(&want, &got, &opts(false)) {
        panic!("{diff}\n--- candidate, rendered\n{}", render(&got));
    }
}

/// 3.00a (the RK3588, read on hardware) and 3.30a (the earlier model)
/// get the same register program: only the GSNPSID value differs.
#[test]
fn the_3_00a_and_3_30a_programs_are_the_same() {
    let a = bringup_rev(SNPSID_300A);
    let b = bringup_rev(SNPSID_330A);
    assert!(a.contains(&Ev::R(32, (BASE + GSNPSID) as u64, SNPSID_300A)));
    assert!(b.contains(&Ev::R(32, (BASE + GSNPSID) as u64, SNPSID_330A)));
    if let Some(diff) = compare(&a, &b, &opts(true)) {
        panic!("{diff}");
    }
    let log = |evs: &[Ev]| {
        evs.iter()
            .find_map(|e| match e {
                Ev::Print(s) if s.contains("GSNPSID") => Some(s.clone()),
                _ => None,
            })
            .unwrap()
    };
    assert_eq!(log(&a), "[usb] dwc3 GSNPSID 0x5533300a (DWC_usb3 3.00a)\n");
    assert_eq!(log(&b), "[usb] dwc3 GSNPSID 0x5533330a (DWC_usb3 3.30a)\n");
}

/// The one revision-gated write: GUSB3PIPECTL.SUSPHY only after 1.94a.
#[test]
fn susphy_is_gated_on_the_revision() {
    let pipectl = |evs: &[Ev]| {
        evs.iter()
            .find_map(|e| match e {
                Ev::W(_, a, v) if *a == (BASE + GUSB3PIPECTL) as u64 => Some(*v),
                _ => None,
            })
            .unwrap()
    };
    for id in [SNPSID_300A, SNPSID_330A, 0x5533_250a] {
        assert_ne!(
            pipectl(&bringup_rev(id)) & GUSB3PIPECTL_SUSPHY,
            0,
            "{id:#x}"
        );
    }
    assert_eq!(pipectl(&bringup_rev(0x5533_194a)) & GUSB3PIPECTL_SUSPHY, 0);
}

#[test]
fn the_soft_reset_comes_after_the_halt_and_before_the_phy_set_up() {
    let evs = bringup();
    let pos = |pred: &dyn Fn(&Ev) -> bool| evs.iter().position(pred).unwrap();
    let w = |o: usize| move |e: &Ev| matches!(e, Ev::W(_, a, _) if *a == (BASE + o) as u64);
    let halt = pos(
        &|e| matches!(e, Ev::R(_, a, v) if *a == (BASE + DSTS) as u64 && v & DSTS_DEVCTRLHLT != 0),
    );
    let reset =
        pos(&|e| matches!(e, Ev::W(_, a, v) if *a == (BASE + DCTL) as u64 && *v == DCTL_CSFTRST));
    let phy = pos(&w(GUSB2PHYCFG));
    let evb = pos(&w(GEVNTSIZ));
    let run = evs
        .iter()
        .rposition(
            |e| matches!(e, Ev::W(_, a, v) if *a == (BASE + DCTL) as u64 && v & DCTL_RUN_STOP != 0),
        )
        .unwrap();
    assert!(halt < reset && reset < phy && phy < evb && evb < run);
}
