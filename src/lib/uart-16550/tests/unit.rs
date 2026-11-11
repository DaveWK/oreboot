// SPDX-License-Identifier: GPL-2.0-only
use regio::mock::Mock;
use regio::RegIo;
use uart_16550::{reg, Config, Rounding, Uart16550, LCR_8N1};

#[test]
fn init_programs_divisor_and_8n1_at_the_stride() {
    let mut io: Mock<16> = Mock::default();
    let cfg = Config {
        base: 0x1000,
        stride: 1,
        clock_hz: 24_000_000,
        baud: 115_200,
        rounding: Rounding::Nearest,
        dw_busy_check: false,
    };
    let u = Uart16550::init(&mut io, &cfg);
    assert_eq!(io.get(0x1000 + reg::RBR_THR_DLL), 13);
    assert_eq!(io.get(0x1000 + reg::IER_DLH), 0);
    assert_eq!(io.get(0x1000 + reg::LCR), LCR_8N1);
    assert_eq!(u.base(), 0x1000);
}

#[test]
fn try_write_respects_thre() {
    let mut io: Mock<4> = Mock::default();
    let u = Uart16550::attach(0x2000, 4);
    assert!(!u.try_write(&mut io, b'a'), "LSR 0: holding register full");
    io.write32(0x2000 + 4 * reg::LSR, 0x20);
    assert!(u.try_write(&mut io, b'a'));
    assert_eq!(io.get(0x2000), b'a' as u32);
}
