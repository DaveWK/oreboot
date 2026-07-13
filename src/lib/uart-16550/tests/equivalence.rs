// SPDX-License-Identifier: GPL-2.0-only
//! The crate against verbatim copies of the K1 and A733 board UARTs: same
//! register accesses, in the same order, with the same values.

use regio::RegIo;
use std::cell::RefCell;
use uart_16550::{divisor, Config, Rounding, Uart16550};

#[derive(Clone, Debug, PartialEq, Eq)]
enum Op {
    R(usize),
    W(usize, u32),
}

/// A UART model: LSR reports THRE/TEMT after `lsr_busy` polls, the DW USR
/// is busy for `usr_busy` polls.
#[derive(Default)]
struct Model {
    ops: Vec<Op>,
    lsr_busy: u32,
    usr_busy: u32,
}

impl Model {
    fn read(&mut self, addr: usize) -> u32 {
        self.ops.push(Op::R(addr));
        let (counter, idle, busy) = match addr & 0xff {
            0x14 => (&mut self.lsr_busy, 0x60, 0),
            0x7c => (&mut self.usr_busy, 0, 1),
            _ => return 0,
        };
        if *counter > 0 {
            *counter -= 1;
            busy
        } else {
            idle
        }
    }
    fn write(&mut self, addr: usize, val: u32) {
        self.ops.push(Op::W(addr, val));
    }
}

thread_local! {
    static LEGACY: RefCell<Model> = RefCell::new(Model::default());
}

struct Io(Model);
impl RegIo for Io {
    fn read32(&mut self, addr: usize) -> u32 {
        self.0.read(addr)
    }
    fn write32(&mut self, addr: usize, val: u32) {
        self.0.write(addr, val)
    }
    fn read16(&mut self, addr: usize) -> u16 {
        self.0.read(addr) as u16
    }
    fn write16(&mut self, addr: usize, val: u16) {
        self.0.write(addr, val as u32)
    }
    fn udelay(&mut self, _: u32) {}
    fn now_us(&mut self) -> u64 {
        0
    }
    fn log(&mut self, _: core::fmt::Arguments) {}
}

// Host stand-ins for what the board code imports.
mod util {
    pub mod mmio {
        pub fn read32(addr: usize) -> u32 {
            super::super::LEGACY.with(|m| m.borrow_mut().read(addr))
        }
        pub fn write32(addr: usize, val: u32) {
            super::super::LEGACY.with(|m| m.borrow_mut().write(addr, val))
        }
    }
}
mod log {
    #[derive(Debug)]
    pub struct Error;
    pub trait Serial {}
    pub fn init<T>(_: &mut T) {}
}
mod nb {
    #[derive(Debug, PartialEq)]
    pub enum Error<E> {
        WouldBlock,
        #[allow(dead_code)]
        Other(E),
    }
    pub type Result<T, E> = core::result::Result<T, Error<E>>;
}
mod embedded_hal_nb {
    pub mod serial {
        pub trait ErrorType {
            type Error;
        }
        pub trait Write<W>: ErrorType {
            fn write(&mut self, word: W) -> super::super::nb::Result<(), Self::Error>;
            fn flush(&mut self) -> super::super::nb::Result<(), Self::Error>;
        }
    }
}
mod fwlog {
    pub fn putc(_: u8) {}
}

#[allow(dead_code, unused_imports, clippy::all)]
mod k1 {
    use super::{embedded_hal_nb, log, nb, util};
    include!("legacy/k1_uart.rs");
}
#[allow(dead_code, unused_imports, clippy::all)]
mod a7a {
    use super::{embedded_hal_nb, log, nb, util};
    include!("legacy/a7a_uart.rs");
}

use embedded_hal_nb::serial::Write as _;

fn legacy_ops() -> Vec<Op> {
    LEGACY.with(|m| std::mem::take(&mut m.borrow_mut().ops))
}
fn legacy_model(lsr_busy: u32, usr_busy: u32) {
    LEGACY.with(|m| {
        *m.borrow_mut() = Model {
            ops: Vec::new(),
            lsr_busy,
            usr_busy,
        }
    });
}
fn io(lsr_busy: u32, usr_busy: u32) -> Io {
    Io(Model {
        ops: Vec::new(),
        lsr_busy,
        usr_busy,
    })
}

/// xorshift64*, seeded: reproducible inputs without a dependency.
struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        self.0.wrapping_mul(0x2545_f491_4f6c_dd1d)
    }
}

#[test]
fn a733_init_putc_drain_write_match() {
    let mut rng = Rng(0x5eed_a733);
    for _ in 0..2000 {
        let base = 0x0250_0000 + ((rng.next() as usize & 7) << 10);
        let clock = 1_000_000 + (rng.next() % 199_000_000) as u32;
        let baud = [9600, 19200, 57600, 115_200, 230_400, 1_500_000][rng.next() as usize % 6];
        let (lsr_busy, usr_busy) = ((rng.next() % 4) as u32, (rng.next() % 4) as u32);
        let c = rng.next() as u8;

        legacy_model(lsr_busy, usr_busy);
        let mut old = a7a::Uart::init(base, clock, baud);
        old.putc(c);
        old.drain();
        let r_old = old.write(c.wrapping_add(1));
        let want = legacy_ops();

        let mut io = io(lsr_busy, usr_busy);
        let cfg = Config {
            base,
            stride: 4,
            clock_hz: clock,
            baud,
            rounding: Rounding::Nearest,
            dw_busy_check: true,
        };
        let new = Uart16550::init(&mut io, &cfg);
        new.putc(&mut io, c);
        new.drain(&mut io);
        let r_new = new.try_write(&mut io, c.wrapping_add(1));
        assert_eq!(io.0.ops, want, "clock {clock} baud {baud}");
        assert_eq!(r_old.is_ok(), r_new);
    }
}

#[test]
fn a733_attach_and_busy_write_match() {
    for lsr_busy in 0..3 {
        legacy_model(lsr_busy, 0);
        let mut old = a7a::Uart::attach(0x0250_0000);
        let r_old = old.write(b'x');
        let want = legacy_ops();
        let mut io = io(lsr_busy, 0);
        let r_new = Uart16550::attach(0x0250_0000, 4).try_write(&mut io, b'x');
        assert_eq!(io.0.ops, want);
        assert_eq!(r_old.is_ok(), r_new);
    }
}

#[test]
fn k1_write_matches() {
    let mut rng = Rng(0x5eed_00c1);
    for _ in 0..1000 {
        let lsr_busy = (rng.next() % 2) as u32;
        let c = rng.next() as u8;
        legacy_model(lsr_busy, 0);
        let mut old = k1::K1XSerial::noinit();
        let r_old = old.write(c);
        let flush_old = old.flush();
        let want = legacy_ops();
        let mut io = io(lsr_busy, 0);
        let r_new = Uart16550::attach(0xd401_7000, 4).try_write(&mut io, c);
        assert_eq!(io.0.ops, want);
        assert_eq!(r_old.is_ok(), r_new);
        assert!(flush_old.is_ok());
    }
}

#[test]
fn k1_divisor_constant() {
    // The K1 bt0 never programs the divisor (the BootROM did); its constant
    // is the floor rounding of 100 MHz / 115200.
    let k1 = (100_000_000u32 / 115_200) >> 4;
    assert_eq!(divisor(100_000_000, 115_200, Rounding::Floor), k1);
}

#[test]
fn divisor_roundings() {
    let mut rng = Rng(0x5eed_d170);
    for _ in 0..100_000 {
        let clock = 1 + (rng.next() % 400_000_000) as u32;
        let baud = 1 + (rng.next() % 4_000_000) as u32;
        assert_eq!(divisor(clock, baud, Rounding::Floor), (clock / baud) >> 4);
        if (clock as u64 + 8 * baud as u64) < u32::MAX as u64 {
            assert_eq!(
                divisor(clock, baud, Rounding::Nearest),
                (clock + 8 * baud) / (16 * baud)
            );
        }
    }
}
