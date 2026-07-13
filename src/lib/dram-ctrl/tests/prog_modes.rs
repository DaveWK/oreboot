// SPDX-License-Identifier: GPL-2.0-only
//! dram_ctrl::dwc programming modes and steps::run_checked.

use dram_ctrl::dwc::{
    self, mode_of, Groups, ProgMode, DRAMSET1TMG, DRAMSET1TMG_MODES, FIELD_MODES,
};
use dram_ctrl::steps::{run, run_checked, Conditions, Guard, NoMr, Step};
use regio::mock::Mock;

const SWCTL: usize = 0x320;
const SWSTAT: usize = 0x324;
const DBG1: usize = 0x304;
const DBGCAM: usize = 0x308;
const PCTRL: usize = 0x490;
const PSTAT: usize = 0x3fc;
const RFSHCTL3: usize = 0x060;
const DRAMTMG0: usize = 0x100;

fn mock() -> Mock<32> {
    let mut m = Mock::<32>::default();
    m.responder = Some(|regs, a| {
        let get = |a: usize| regs.iter().find(|r| r.0 == a).map_or(0, |r| r.1);
        match a {
            SWSTAT => Some(get(SWCTL) & 1),
            DBGCAM => Some(0x3600_0000),
            PSTAT => Some(0),
            _ => None,
        }
    });
    m
}

const T0: Step = Step::Field {
    addr: DRAMTMG0,
    clear: 0x3f,
    set: 0x21,
    mode: DRAMSET1TMG_MODES[0],
    what: "t_ras_min",
};

#[test]
fn table_is_consistent() {
    assert_eq!(DRAMSET1TMG.len(), DRAMSET1TMG_MODES.len());
    assert_eq!(
        mode_of("DFIMISC", "dfi_init_complete_en"),
        Some(ProgMode::QuasiDynamic(Groups::G3))
    );
    assert_eq!(mode_of("MSTR", "active_ranks"), Some(ProgMode::Static));
    assert_eq!(mode_of("SWCTL", "sw_done"), Some(ProgMode::Dynamic));
    assert_eq!(
        mode_of("RFSHTMG", "t_rfc_min"),
        Some(ProgMode::DynamicRefresh)
    );
    assert_eq!(
        mode_of("DRAMTMG2", "read_latency"),
        Some(ProgMode::QuasiDynamic(
            Groups::G1.or(Groups::G2).or(Groups::G4)
        ))
    );
    assert_eq!(mode_of("MSTR", "nonexistent"), None);
    for (i, a) in FIELD_MODES.iter().enumerate() {
        for b in &FIELD_MODES[i + 1..] {
            assert!(
                !(a.reg == b.reg && a.field == b.field),
                "{} {} twice",
                a.reg,
                a.field
            );
        }
    }
}

#[test]
fn quasi_dynamic_write_needs_sw_done_and_a_group() {
    // No sw_done: refused.
    let mut io = mock();
    let mut g = Guard::default();
    let e = run_checked(&mut io, &[T0], &mut NoMr, false, &mut g).unwrap_err();
    assert_eq!(e.index, 0);
    assert_eq!(io.get(DRAMTMG0), 0);
    // sw_done open but no group state: refused.
    let steps = [Step::SwDoneClear { swctl: SWCTL }, T0];
    let e = run_checked(&mut io, &steps, &mut NoMr, false, &mut Guard::default()).unwrap_err();
    assert_eq!(e.index, 1);
    // Group 1 established (t_ras_min is groups 2 and 4): still refused.
    let g1 = dwc::enter_group1(DBG1, DBGCAM, 100);
    let mut steps = vec![Step::SwDoneClear { swctl: SWCTL }];
    steps.extend_from_slice(&g1);
    steps.push(T0);
    assert!(run_checked(&mut io, &steps, &mut NoMr, false, &mut Guard::default()).is_err());
    // Group 2 (self-refresh) established by the board: accepted, then sw_done.
    let steps = [
        Step::Assert(Conditions::GROUP2),
        Step::SwDoneClear { swctl: SWCTL },
        T0,
        Step::SwDoneSet {
            swctl: SWCTL,
            swstat: SWSTAT,
            timeout_us: 100,
        },
    ];
    let mut g = Guard::default();
    assert_eq!(
        run_checked(&mut io, &steps, &mut NoMr, false, &mut g),
        Ok(4)
    );
    assert_eq!(io.get(DRAMTMG0) & 0x3f, 0x21);
    assert!(!g.sw_open && g.conds.has(Conditions::GROUP2));
    assert_eq!(io.get(SWCTL), 1);
}

#[test]
fn static_only_in_reset_and_refresh_needs_the_update_toggle() {
    let st = Step::Field {
        addr: 0,
        clear: 0,
        set: 1 << 24,
        mode: ProgMode::Static,
        what: "MSTR.active_ranks",
    };
    let mut io = mock();
    assert!(run_checked(&mut io, &[st], &mut NoMr, false, &mut Guard::default()).is_err());
    let steps = [Step::Assert(Conditions::RESET), st, T0];
    assert_eq!(
        run_checked(&mut io, &steps, &mut NoMr, false, &mut Guard::default()),
        Ok(3)
    );
    let rf = Step::Field {
        addr: 0x64,
        clear: 0x3ff,
        set: 0x8c,
        mode: ProgMode::DynamicRefresh,
        what: "RFSHTMG.t_rfc_min",
    };
    let set = Step::SwDoneSet {
        swctl: SWCTL,
        swstat: SWSTAT,
        timeout_us: 100,
    };
    let mut g = Guard::default();
    let e = run_checked(&mut io, &[rf, set], &mut NoMr, false, &mut g).unwrap_err();
    assert_eq!(e.index, 1);
    let before = io.get(RFSHCTL3);
    let mut g = Guard::default();
    let steps = [rf, Step::RefreshUpdate { rfshctl3: RFSHCTL3 }, set];
    assert_eq!(
        run_checked(&mut io, &steps, &mut NoMr, false, &mut g),
        Ok(3)
    );
    assert_eq!(io.get(RFSHCTL3) ^ before, 2);
}

#[test]
fn group3_helper_and_unchecked_run() {
    let mut io = mock();
    io.set(PCTRL, 1);
    let g3 = dwc::enter_group3(PCTRL, PSTAT, DBGCAM, 100);
    let dfimisc = Step::Field {
        addr: 0x1b0,
        clear: 1,
        set: 1,
        mode: mode_of("DFIMISC", "dfi_init_complete_en").unwrap(),
        what: "DFIMISC.dfi_init_complete_en",
    };
    let mut steps = vec![Step::SwDoneClear { swctl: SWCTL }];
    steps.extend_from_slice(&g3);
    steps.push(dfimisc);
    let mut g = Guard::default();
    assert_eq!(
        run_checked(&mut io, &steps, &mut NoMr, false, &mut g),
        Ok(steps.len())
    );
    assert_eq!(io.get(PCTRL), 0);
    assert_eq!(io.get(0x1b0), 1);
    // The unchecked runner executes the same steps without checking.
    let mut io = mock();
    assert_eq!(run(&mut io, &[T0], &mut NoMr, false), Ok(1));
    assert_eq!(io.get(DRAMTMG0), 0x21);
}
