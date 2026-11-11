// SPDX-License-Identifier: GPL-2.0-only
//! UART2 M0 console at 1,500,000 baud from documented registers (RK3588 TRM
//! Part 1): PMU2_IOC_GPIO0B_IOMUX_SEL_H (+0x0) gpio0b5/b6 = 4'h8 "refer to
//! BUS_IOC" (p.968), BUS_IOC_GPIO0B_IOMUX_SEL_H (+0xC) = 4'ha UART2_TX/RX_M0
//! (p.985), CRU_CLKSEL_CON45 (+0x3B4) sclk_uart2_sel [1:0] = 2 xin 24 MHz
//! (p.75), DesignWare UART registers (p.1439): 24 MHz / 16 / 1 = 1.5 Mbaud.
//! In the stepping-stone image the rkbin DDR init already set the console
//! up; programming it again is harmless.

use core::fmt;
use rk3588::regs::{base, wm};

const PMU2_IOC_GPIO0B_SEL_H: usize = base::PMU2_IOC;
const BUS_IOC_GPIO0B_SEL_H: usize = base::BUS_IOC + 0x000C;
const CRU_CLKSEL_CON45: usize = base::CRU + 0x03B4;
const THR: usize = base::UART2;
const DLH: usize = base::UART2 + 0x04;
const FCR: usize = base::UART2 + 0x08;
const LCR: usize = base::UART2 + 0x0C;
const MCR: usize = base::UART2 + 0x10;
const LSR: usize = base::UART2 + 0x14;

fn w(a: usize, v: u32) {
    unsafe { core::ptr::write_volatile(a as *mut u32, v) }
}
fn r(a: usize) -> u32 {
    unsafe { core::ptr::read_volatile(a as *const u32) }
}

pub struct Uart2;

impl Uart2 {
    pub fn init() -> Uart2 {
        w(CRU_CLKSEL_CON45, wm(0, 2, 2));
        w(PMU2_IOC_GPIO0B_SEL_H, wm(4, 4, 8) | wm(8, 4, 8));
        w(BUS_IOC_GPIO0B_SEL_H, wm(4, 4, 0xa) | wm(8, 4, 0xa));
        // Wait for the transmitter to drain before touching the divisor.
        for _ in 0..100_000 {
            if r(LSR) & (1 << 6) != 0 {
                break;
            }
        }
        w(LCR, 0x83);
        w(THR, 1);
        w(DLH, 0);
        w(LCR, 0x03);
        // Vendor U-Boot values: FIFOs on and both reset (0x07, so a
        // previous stage's RX bytes are dropped), DTR/RTS set (0x03; no
        // auto flow control, the pins are not muxed).
        w(FCR, 0x07);
        w(MCR, 0x03);
        Uart2
    }

    pub fn putc(&mut self, c: u8) {
        for _ in 0..1_000_000 {
            if r(LSR) & (1 << 5) != 0 {
                break;
            }
        }
        w(THR, c as u32);
    }
}

impl fmt::Write for Uart2 {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        for b in s.bytes() {
            if b == b'\n' {
                self.putc(b'\r');
            }
            self.putc(b);
        }
        Ok(())
    }
}
