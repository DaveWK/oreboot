// SPDX-License-Identifier: GPL-2.0-only
use arm_ppu::{Pck600, PpuError, MODE_ON, PWPR, PWSR};
use regio::mock::Mock;
use regio::RegIo;

const P: Pck600 = Pck600 {
    base: 0x1000_0000,
    frame_shift: 12,
    count: 2,
    setup: &[],
    gate: None,
    polls: 3,
    poll_us: 1,
};

#[test]
fn bad_domain_and_already_on() {
    let mut io: Mock<8> = Mock::default();
    assert_eq!(P.power_on(&mut io, 2), Err(PpuError::BadDomain(2)));
    assert!(!P.is_on(&mut io, 2));
    io.write32(P.frame(1) + PWSR, MODE_ON);
    let w = io.writes;
    assert_eq!(P.power_on(&mut io, 1), Ok(()));
    assert_eq!(io.writes, w, "nothing written when already on");
}

#[test]
fn request_and_timeout() {
    let mut io: Mock<8> = Mock::default();
    io.write32(P.frame(0) + PWPR, 0x31);
    assert_eq!(P.power_on(&mut io, 0), Err(PpuError::Timeout(0)));
    assert_eq!(
        io.get(P.frame(0) + PWPR),
        0x38,
        "mode ON requested, other bits kept"
    );
}
