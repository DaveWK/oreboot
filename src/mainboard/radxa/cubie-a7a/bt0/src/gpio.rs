//! sun60iw2 pin controller ("GPIO v3" register layout in SyterKit).
//!
//! Main PIO (ports A..K): bank N registers start at PIO_BASE + N * 0x80 + 0x80
//! (CFG0..3 at +0x00, DAT at +0x10, DRV0..3 at +0x20, PULL0..1 at +0x30).
//! R_PIO (ports L..N): bank N at R_PIO_BASE + N * 0x30 (CFG at +0x00, DRV at
//! +0x14, PULL at +0x24).

use util::mmio::read32;

use crate::soc::{clr_set_bits, PIO_BASE, R_PIO_BASE};

#[allow(dead_code)] // full port list of the SoC
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Port {
    A = 0,
    B,
    C,
    D,
    E,
    F,
    G,
    H,
    I,
    J,
    K,
    L,
    M,
    N,
}

#[allow(dead_code)]
#[derive(Clone, Copy)]
pub enum Pull {
    None = 0,
    Up = 1,
    Down = 2,
}

struct Bank {
    cfg: usize,
    dat: usize,
    drv: usize,
    pull: usize,
}

fn bank(port: Port) -> Bank {
    let p = port as usize;
    if p >= Port::L as usize {
        let base = R_PIO_BASE + (p - Port::L as usize) * 0x30;
        Bank {
            cfg: base,
            dat: base + 0x10,
            drv: base + 0x14,
            pull: base + 0x24,
        }
    } else {
        let base = PIO_BASE + p * 0x80 + 0x80;
        Bank {
            cfg: base,
            dat: base + 0x10,
            drv: base + 0x20,
            pull: base + 0x30,
        }
    }
}

/// Select pin function `func` (0 = input, 1 = output, 2.. = peripherals).
pub fn set_function(port: Port, pin: u32, func: u32) {
    let b = bank(port);
    let reg = b.cfg + ((pin as usize >> 3) << 2);
    let shift = (pin & 7) * 4;
    clr_set_bits(reg, 0xf << shift, (func & 0xf) << shift);
}

pub fn set_pull(port: Port, pin: u32, pull: Pull) {
    let b = bank(port);
    let reg = b.pull + ((pin as usize >> 4) << 2);
    let shift = (pin & 0xf) * 2;
    clr_set_bits(reg, 0x3 << shift, (pull as u32) << shift);
}

pub fn set_drive(port: Port, pin: u32, level: u32) {
    let b = bank(port);
    let reg = b.drv + ((pin as usize >> 4) << 2);
    let shift = (pin & 0xf) * 2;
    clr_set_bits(reg, 0x3 << shift, (level & 0x3) << shift);
}

/// Level of an input pin (main PIO banks only; R_PIO data sits elsewhere).
pub fn read(port: Port, pin: u32) -> bool {
    assert!((port as usize) < Port::L as usize);
    read32(bank(port).dat) & (1 << pin) != 0
}
