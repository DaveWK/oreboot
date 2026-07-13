// SPDX-License-Identifier: GPL-2.0-only
//! P3-M12: the DWC hand-over, quiesce/resume and PHY re-init templates run
//! through run_checked on a controller model.

use dram_ctrl::dwc::{
    bits, handover, mode_of, phy_reinit_hold, phy_reinit_release, quiesce, resume, Groups,
    ProgMode, Regs,
};
use dram_ctrl::steps::{run_checked, Guard, NoMr, Step};
use regio::mock::Mock;
use regio::RegIo;

const R: Regs = Regs {
    stat: 0x004,
    init0: 0x0d0,
    pwrctl: 0x030,
    rfshctl3: 0x060,
    zqctl0: 0x180,
    dfiupd0: 0x1a0,
    dfimisc: 0x1b0,
    dbg1: 0x304,
    dbgcam: 0x308,
    swctl: 0x320,
    swstat: 0x324,
    pctrl: 0x490,
    pstat: 0x3fc,
};

/// STAT follows the model: software self-refresh, else normal once
/// dfi_init_complete_en is set, else init.
fn controller() -> Mock<32> {
    let mut m = Mock::<32>::default();
    m.responder = Some(|regs, a| {
        let get = |a: usize| regs.iter().find(|r| r.0 == a).map_or(0, |r| r.1);
        match a {
            0x324 => Some(get(0x320) & 1),
            0x308 => Some(0x3600_0000),
            0x3fc => Some(0),
            0x004 => Some(if get(0x030) & bits::PWRCTL_SELFREF_SW != 0 {
                bits::MODE_SELF_REFRESH | bits::SELFREF_TYPE_SW
            } else if get(0x1b0) & 1 != 0 {
                bits::MODE_NORMAL
            } else {
                0
            }),
            _ => None,
        }
    });
    m
}

fn run(io: &mut Mock<32>, steps: &[Step], g: &mut Guard) -> Result<usize, &'static str> {
    run_checked(io, steps, &mut NoMr, false, g).map_err(|e| e.what)
}

#[test]
fn handover_after_training_passes_the_checker() {
    let mut io = controller();
    io.set(R.zqctl0, bits::ZQCTL0_DIS_AUTO_ZQ);
    io.set(R.rfshctl3, bits::RFSHCTL3_DIS_AUTO_REFRESH);
    io.set(R.swctl, 1);
    let mut g = Guard::default();
    run(&mut io, &handover(&R, 1000), &mut g).unwrap();
    assert_eq!(io.get(R.dfimisc) & 1, 1);
    assert_eq!(io.get(R.zqctl0) & bits::ZQCTL0_DIS_AUTO_ZQ, 0);
    assert_eq!(
        io.get(R.rfshctl3),
        bits::RFSHCTL3_REFRESH_UPDATE_LEVEL,
        "auto refresh on, update level toggled"
    );
    assert_eq!(io.get(R.pctrl) & bits::PCTRL_PORT_EN, 1);
    assert_eq!(g, Guard::default(), "no state left behind");
}

#[test]
fn the_checker_refuses_the_handover_without_its_group_steps() {
    let mut io = controller();
    let mut g = Guard::default();
    // Skip the port-off/drain/assert steps: DFIMISC is group 3.
    let h = handover(&R, 1000);
    assert_eq!(
        run(&mut io, &h[5..], &mut g),
        Err("quasi-dynamic group state not established")
    );
}

#[test]
fn quiesce_and_resume_around_delay_writes() {
    let mut io = controller();
    io.set(R.dfimisc, 1); // in normal operation
    let mut g = Guard::default();
    run(&mut io, &quiesce(&R, 1000), &mut g).unwrap();
    assert_eq!(io.read32(R.stat) & 7, bits::MODE_SELF_REFRESH);
    assert_ne!(io.get(R.dfiupd0) & bits::DFIUPD0_DIS_AUTO_CTRLUPD, 0);
    assert_ne!(io.get(R.zqctl0) & bits::ZQCTL0_DIS_AUTO_ZQ, 0);
    // ... the board writes trained delays into the PHY here ...
    run(&mut io, &resume(&R, 1000), &mut g).unwrap();
    assert_eq!(io.read32(R.stat) & 7, bits::MODE_NORMAL);
    assert_eq!(io.get(R.dfiupd0), 0);
    assert_eq!(io.get(R.zqctl0), 0);
    assert_eq!(g, Guard::default());
}

#[test]
fn phy_reinit_holds_and_releases_the_controller() {
    let mut io = controller();
    io.set(R.dfimisc, 1);
    let mut g = Guard::default();
    // The hold needs group 2 (self-refresh): refused on its own.
    assert_eq!(
        run(&mut io, &phy_reinit_hold(&R, 1000), &mut Guard::default()),
        Err("quasi-dynamic group state not established")
    );
    let mut io = controller();
    io.set(R.dfimisc, 1);
    run(&mut io, &quiesce(&R, 1000), &mut g).unwrap();
    run(&mut io, &phy_reinit_hold(&R, 1000), &mut g).unwrap();
    assert_eq!(io.get(R.init0) >> 30, 0b11);
    assert_ne!(io.get(R.dbg1) & bits::DBG1_DIS_HIF, 0);
    assert_eq!(io.get(R.dfimisc) & 1, 0);
    // ... the board exits the low-power state and re-inits the PHY ...
    run(
        &mut io,
        &phy_reinit_release(&R, bits::INIT0_SKIP_NORMAL, 1000),
        &mut g,
    )
    .unwrap();
    assert_eq!(io.get(R.init0) >> 30, 0b01);
    assert_eq!(io.get(R.dbg1), 0);
    assert_eq!(io.get(R.dfiupd0), 0);
    assert_eq!(io.get(R.dfimisc) & 1, 1);
    assert_eq!(io.read32(R.stat) & 7, bits::MODE_NORMAL);
    assert_eq!(io.get(R.pctrl) & 1, 1);
    assert_eq!(g, Guard::default());
}

#[test]
fn dis_auto_ctrlupd_is_quasi_dynamic_group_2() {
    // UG1087 ddrc___dfiupd0: "Quasi-dynamic Group 2" (was listed static).
    assert_eq!(
        mode_of("DFIUPD0", "dis_auto_ctrlupd"),
        Some(ProgMode::QuasiDynamic(Groups::G2))
    );
}
