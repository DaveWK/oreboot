// SPDX-License-Identifier: GPL-2.0-only
//! Parameters and helpers the A7A bring-up does not exercise.
use dwc_pcie::{find_express_cap, link_mode, wait, Config, Window};
use regio::mock::Mock;
use regio::RegIo;

const W: Window = Window {
    cpu: 0,
    pci: 0,
    size: 0x1000,
};
fn cfg() -> Config {
    Config {
        dbi: 0x1000,
        dbi2: 0x2000,
        atu: 0x3000,
        max_gen: 2,
        lanes: 4,
        perst_hold_us: 100_000,
        config: W,
        mem: W,
        io: W,
    }
}

#[test]
fn lane_modes() {
    assert_eq!(
        [1, 2, 4, 8].map(link_mode),
        [0x1, 0x3, 0x7, 0xf],
        "PORT_LINK_CTRL link mode per lane count"
    );
}

#[test]
fn capability_walk_is_bounded_on_a_circular_list() {
    let c = cfg();
    let mut io: Mock<8> = Mock::default();
    io.write16(c.dbi + 0x34, 0x40);
    io.write16(c.dbi + 0x40, 0x5001); // id 1 (PM), next 0x50
    io.write16(c.dbi + 0x50, 0x4005); // id 5 (MSI), next 0x40: a loop
    assert_eq!(find_express_cap(&mut io, &c), None);
    io.write16(c.dbi + 0x50, 0x0010); // id 0x10 (express)
    assert_eq!(find_express_cap(&mut io, &c), Some(0x50));
    io.write16(c.dbi + 0x34, 0xfe); // out of the config header
    assert_eq!(find_express_cap(&mut io, &c), None);
}

#[test]
fn wait_checks_once_more_after_the_time_ran_out() {
    let mut io: Mock<2> = Mock::default();
    io.tick_us = 0;
    let mut polls = 0;
    // Polls at 0, 250, 500, 750 and 1000 us fail; at 1000 us the time is up
    // and the condition is checked once more, which succeeds.
    let ok = wait(&mut io, 1000, 250, |_| {
        polls += 1;
        polls == 6
    });
    assert!(ok);
    assert_eq!(polls, 6);
    let mut polls = 0;
    assert!(!wait(&mut io, 1000, 250, |_| {
        polls += 1;
        false
    }));
    assert_eq!(polls, 6);
}
