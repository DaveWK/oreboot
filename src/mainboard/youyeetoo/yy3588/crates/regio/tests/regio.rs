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

#[test]
fn delay_zero_does_not_read_the_timer() {
    assert_eq!(
        regio::delay_ticks(|| panic!("zero delay read"), 0, 0),
        Ok(())
    );
}

#[test]
fn delay_requires_full_elapsed_duration_and_accepts_wrap() {
    let mut times = [u64::MAX - 3, u64::MAX - 2, u64::MAX - 1, 0, 1].into_iter();
    assert_eq!(regio::delay_ticks(|| times.next().unwrap(), 5, 4), Ok(()));
    assert!(times.next().is_none());
}

#[test]
fn delay_frozen_timer_returns_with_exact_read_budget() {
    let mut reads = 0;
    let error = regio::delay_ticks(
        || {
            reads += 1;
            7
        },
        1,
        23,
    );
    assert_eq!(
        error,
        Err(Timeout("delay timer did not advance within read budget"))
    );
    assert_eq!(reads, 24, "initial sample plus exactly23 permitted polls");
}

#[test]
fn delay_progress_below_duration_does_not_claim_success() {
    let mut ticks = 0;
    assert!(regio::delay_ticks(
        || {
            let now = ticks;
            ticks += 1;
            now
        },
        4,
        3
    )
    .is_err());
    assert!(regio::delay_ticks(|| 7, 1, 0).is_err());
}

#[test]
fn checked_delay_preserves_existing_legacy_implementation() {
    let mut io: Mock<2> = Mock::default();
    assert_eq!(io.udelay_checked(11), Ok(()));
    assert_eq!(io.clock_us, 11);
}
