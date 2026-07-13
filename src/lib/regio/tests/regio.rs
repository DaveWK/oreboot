// SPDX-License-Identifier: GPL-2.0-only
//! Host tests of the register-access trait, bounded polling and the mock.

use regio::mock::Mock;
use regio::{wait, RegIo, Timeout, WAIT_MAX_POLLS};

#[test]
fn wait_succeeds_on_the_first_true_poll() {
    let mut io: Mock<2> = Mock::default();
    let mut n = 0;
    assert_eq!(
        wait(&mut io, "ready", 1000, |_| {
            n += 1;
            n == 3
        }),
        Ok(())
    );
    assert_eq!(n, 3);
}

#[test]
fn wait_times_out_on_the_clock() {
    let mut io: Mock<2> = Mock::default();
    io.tick_us = 10;
    assert_eq!(wait(&mut io, "slow", 50, |_| false), Err(Timeout("slow")));
}

#[test]
fn wait_is_bounded_with_a_frozen_clock() {
    let mut io: Mock<2> = Mock::default();
    io.tick_us = 0;
    let mut n = 0u32;
    assert_eq!(
        wait(&mut io, "never", 10, |_| {
            n += 1;
            false
        }),
        Err(Timeout("never"))
    );
    assert_eq!(n, WAIT_MAX_POLLS);
}

#[test]
fn read_modify_write_helpers_and_counts() {
    let mut io: Mock<4> = Mock::default();
    io.write32(0x10, 0xf0);
    io.set32(0x10, 0x1);
    io.clr32(0x10, 0x10);
    io.clrset32(0x10, 0xe0, 0x200);
    assert_eq!(io.read32(0x10), 0x201);
    io.write16(0x14, 0xbeef);
    assert_eq!(io.read16(0x14), 0xbeef);
    assert_eq!(io.read32(0x18), 0, "unwritten registers read 0");
    assert!(io.writes >= 5 && io.reads >= 5);
}

#[test]
fn responder_answers_before_stored_values() {
    fn status(_: &mut [(usize, u32)], addr: usize) -> Option<u32> {
        (addr == 0x40).then_some(0xa5)
    }
    let mut io: Mock<2> = Mock::default();
    io.write32(0x40, 1);
    io.responder = Some(status);
    assert_eq!(io.read32(0x40), 0xa5);
    assert_eq!(io.read32(0x44), 0);
}
