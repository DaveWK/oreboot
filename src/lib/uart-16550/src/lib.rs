// SPDX-License-Identifier: GPL-2.0-only
//! 16550-compatible UART (including the DesignWare 8250 variant) over
//! [`regio::RegIo`].
//!
//! Moved from the board repositories: ore-edk-boot-opi K1 bt0 `uart.rs`
//! (SpacemiT K1, 4-byte stride, divisor rounded down) and the A7A oreboot
//! `common/src/uart.rs` (Allwinner A733, DesignWare 8250, 4-byte stride,
//! divisor rounded to nearest, LCR writes held off while USR.BUSY). The
//! register stride, input clock, baud rate, divisor rounding and the busy
//! check are parameters; base addresses, pin muxing and the logger glue
//! (`log::Serial`, `embedded_hal_nb`) stay in the boards.
#![no_std]

use regio::RegIo;

/// Register indices (multiplied by [`Config::stride`] for the offset).
pub mod reg {
    /// Receive buffer / transmit holding / divisor latch low (DLAB=1).
    pub const RBR_THR_DLL: usize = 0;
    /// Interrupt enable / divisor latch high (DLAB=1).
    pub const IER_DLH: usize = 1;
    /// FIFO control (write) / interrupt identification (read).
    pub const FCR_IIR: usize = 2;
    pub const LCR: usize = 3;
    pub const MCR: usize = 4;
    pub const LSR: usize = 5;
    /// DesignWare UART status register (index 31: offset 0x7c at stride 4).
    pub const DW_USR: usize = 31;
}

pub const LCR_DLAB: u32 = 1 << 7;
/// 8 data bits, no parity, 1 stop bit.
pub const LCR_8N1: u32 = 0x3;
pub const LSR_THRE: u32 = 1 << 5;
pub const LSR_TEMT: u32 = 1 << 6;
/// FIFO enable, clear RX, clear TX.
pub const FCR_ENABLE_CLEAR: u32 = 0x7;
/// DTR | RTS.
pub const MCR_DTR_RTS: u32 = 0x3;
pub const DW_USR_BUSY: u32 = 1 << 0;

/// How the 16x divisor is rounded.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Rounding {
    /// `(clock / baud) >> 4` (the K1 bt0's constant).
    Floor,
    /// `(clock + 8 baud) / (16 baud)` (the A733 init).
    Nearest,
}

/// Divisor latch value for `clock_hz` and `baud`.
pub const fn divisor(clock_hz: u32, baud: u32, rounding: Rounding) -> u32 {
    match rounding {
        Rounding::Floor => (clock_hz / baud) >> 4,
        Rounding::Nearest => (clock_hz + 8 * baud) / (16 * baud),
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Config {
    pub base: usize,
    /// Bytes between registers (4 on the K1 and the A733).
    pub stride: usize,
    pub clock_hz: u32,
    pub baud: u32,
    pub rounding: Rounding,
    /// DesignWare 8250: wait for USR.BUSY to clear before writing LCR.
    pub dw_busy_check: bool,
}

/// A port: base address and register stride.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Uart16550 {
    base: usize,
    stride: usize,
}

impl Uart16550 {
    /// Use a port that an earlier stage already configured.
    pub const fn attach(base: usize, stride: usize) -> Self {
        Self { base, stride }
    }

    pub const fn base(&self) -> usize {
        self.base
    }

    const fn reg(&self, index: usize) -> usize {
        self.base + index * self.stride
    }

    /// Program 8N1 at `cfg.baud`, enable and reset the FIFOs, raise DTR/RTS.
    pub fn init<R: RegIo + ?Sized>(io: &mut R, cfg: &Config) -> Self {
        let u = Self::attach(cfg.base, cfg.stride);
        let d = divisor(cfg.clock_hz, cfg.baud, cfg.rounding);
        if cfg.dw_busy_check {
            // LCR writes are ignored while the DW UART is busy.
            while io.read32(u.reg(reg::DW_USR)) & DW_USR_BUSY != 0 {}
        }
        io.write32(u.reg(reg::IER_DLH), 0);
        io.write32(u.reg(reg::LCR), LCR_DLAB | LCR_8N1);
        io.write32(u.reg(reg::RBR_THR_DLL), d & 0xff);
        io.write32(u.reg(reg::IER_DLH), (d >> 8) & 0xff);
        io.write32(u.reg(reg::LCR), LCR_8N1);
        io.write32(u.reg(reg::FCR_IIR), FCR_ENABLE_CLEAR);
        io.write32(u.reg(reg::MCR), MCR_DTR_RTS);
        u
    }

    /// Transmit `c` if the holding register is empty; false if it is not
    /// (the non-blocking write of `embedded_hal_nb::serial::Write`).
    pub fn try_write<R: RegIo + ?Sized>(&self, io: &mut R, c: u8) -> bool {
        if io.read32(self.reg(reg::LSR)) & LSR_THRE == 0 {
            return false;
        }
        io.write32(self.reg(reg::RBR_THR_DLL), c as u32);
        true
    }

    /// Wait for the holding register, then transmit `c`.
    pub fn putc<R: RegIo + ?Sized>(&self, io: &mut R, c: u8) {
        while io.read32(self.reg(reg::LSR)) & LSR_THRE == 0 {}
        io.write32(self.reg(reg::RBR_THR_DLL), c as u32);
    }

    /// Wait until the transmitter is completely idle (before a reset).
    pub fn drain<R: RegIo + ?Sized>(&self, io: &mut R) {
        while io.read32(self.reg(reg::LSR)) & LSR_TEMT == 0 {}
    }
}
