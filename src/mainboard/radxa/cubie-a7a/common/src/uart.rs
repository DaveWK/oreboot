//! Allwinner UART (DesignWare 8250/16550 compatible, 32-bit register stride).
//!
//! Clock gating, reset and pin muxing are done by bt0; main only reuses the
//! already configured port.

use log::{Error, Serial};
use util::mmio::{read32, write32};

pub const UART0_BASE: usize = 0x0250_0000;

const RBR_THR_DLL: usize = 0x00;
const IER_DLH: usize = 0x04;
const FCR: usize = 0x08;
const LCR: usize = 0x0c;
const MCR: usize = 0x10;
const LSR: usize = 0x14;
const USR: usize = 0x7c;

const LCR_DLAB: u32 = 1 << 7;
const LCR_8N1: u32 = 0x3;
const LSR_THRE: u32 = 1 << 5;
const LSR_TEMT: u32 = 1 << 6;
const USR_BUSY: u32 = 1 << 0;

pub struct Uart {
    base: usize,
}

impl Uart {
    /// Program 8N1 at `baud` from the UART's `clock_hz` (APB, = HOSC).
    pub fn init(base: usize, clock_hz: u32, baud: u32) -> Self {
        let divisor = (clock_hz + 8 * baud) / (16 * baud);
        // LCR writes are ignored while the DW UART is busy.
        while read32(base + USR) & USR_BUSY != 0 {}
        write32(base + IER_DLH, 0);
        write32(base + LCR, LCR_DLAB | LCR_8N1);
        write32(base + RBR_THR_DLL, divisor & 0xff);
        write32(base + IER_DLH, (divisor >> 8) & 0xff);
        write32(base + LCR, LCR_8N1);
        // Enable and reset FIFOs.
        write32(base + FCR, 0x7);
        // DTR | RTS
        write32(base + MCR, 0x3);
        Self { base }
    }

    /// Use a port that an earlier stage already configured.
    pub fn attach(base: usize) -> Self {
        Self { base }
    }

    pub fn putc(&mut self, c: u8) {
        while read32(self.base + LSR) & LSR_THRE == 0 {}
        write32(self.base + RBR_THR_DLL, c as u32);
        crate::fwlog::putc(c);
    }

    /// Wait until the transmitter is completely idle, e.g. before a reset.
    pub fn drain(&self) {
        while read32(self.base + LSR) & LSR_TEMT == 0 {}
    }
}

impl Serial for Uart {}

impl embedded_hal_nb::serial::ErrorType for Uart {
    type Error = Error;
}

impl embedded_hal_nb::serial::Write<u8> for Uart {
    fn write(&mut self, c: u8) -> nb::Result<(), Error> {
        if read32(self.base + LSR) & LSR_THRE == 0 {
            return Err(nb::Error::WouldBlock);
        }
        write32(self.base + RBR_THR_DLL, c as u32);
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
