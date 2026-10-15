//! Allwinner UART (DesignWare 8250/16550 compatible, 32-bit register stride),
//! on the shared `uart-16550` crate, with every transmitted byte also
//! appended to the RAM log ([`crate::fwlog`]).
//!
//! Clock gating, reset and pin muxing are done by bt0; main only reuses the
//! already configured port.

use crate::mmio::Mmio;
use log::{Error, Serial};
use uart_16550::{Config, Rounding, Uart16550};

pub const UART0_BASE: usize = 0x0250_0000;
const STRIDE: usize = 4;

pub struct Uart {
    port: Uart16550,
}

impl Uart {
    /// Program 8N1 at `baud` from the UART's `clock_hz` (APB, = HOSC).
    pub fn init(base: usize, clock_hz: u32, baud: u32) -> Self {
        let cfg = Config {
            base,
            stride: STRIDE,
            clock_hz,
            baud,
            rounding: Rounding::Nearest,
            // LCR writes are ignored while the DW UART is busy.
            dw_busy_check: true,
        };
        Self {
            port: Uart16550::init(&mut Mmio, &cfg),
        }
    }

    /// Use a port that an earlier stage already configured.
    pub fn attach(base: usize) -> Self {
        Self {
            port: Uart16550::attach(base, STRIDE),
        }
    }

    pub fn putc(&mut self, c: u8) {
        self.port.putc(&mut Mmio, c);
        crate::fwlog::putc(c);
    }

    /// Wait until the transmitter is completely idle, e.g. before a reset.
    pub fn drain(&self) {
        self.port.drain(&mut Mmio);
    }
}

impl Serial for Uart {}

impl embedded_hal_nb::serial::ErrorType for Uart {
    type Error = Error;
}

impl embedded_hal_nb::serial::Write<u8> for Uart {
    fn write(&mut self, c: u8) -> nb::Result<(), Error> {
        if !self.port.try_write(&mut Mmio, c) {
            return Err(nb::Error::WouldBlock);
        }
        crate::fwlog::putc(c);
        Ok(())
    }

    fn flush(&mut self) -> nb::Result<(), Error> {
        Ok(())
    }
}

/// Install `uart` as the global logger for `log::print!`/`println!`.
pub fn init_logger(uart: Uart) {
    static mut SERIAL: Option<Uart> = None;
    // SAFETY: called once, single-threaded, before any logging.
    unsafe {
        let s = &mut *core::ptr::addr_of_mut!(SERIAL);
        s.replace(uart);
        log::init(s.as_mut().unwrap());
    }
}
