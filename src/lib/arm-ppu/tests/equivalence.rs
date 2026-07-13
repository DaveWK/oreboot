// SPDX-License-Identifier: GPL-2.0-only
//! arm-ppu against the verbatim A733 ppu.rs: identical register and delay
//! traces and results for every domain, gate state and status timing.

use arm_ppu::{Gate, Pck600, PpuError};
use regio::RegIo;
use std::cell::RefCell;

#[derive(Clone, Debug, PartialEq, Eq)]
enum Op {
    R(usize),
    W(usize, u32),
    D(u32),
}

/// Gate register value, and PWSR reads ON after `on_after` reads (never if
/// None); other registers read back what was written (PWPR starts 0x3).
#[derive(Default)]
struct Model {
    ops: Vec<Op>,
    gate: u32,
    on_after: Option<u32>,
    pwpr: u32,
}
impl Model {
    fn read(&mut self, a: usize) -> u32 {
        self.ops.push(Op::R(a));
        if a == 0x0701_01ac {
            return self.gate;
        }
        if a & 0xfff == 0x008 {
            return match self.on_after {
                Some(0) => 0x8,
                Some(ref mut n) => {
                    *n -= 1;
                    0x1
                }
                None => 0x1,
            };
        }
        if a & 0xfff == 0x000 {
            return self.pwpr;
        }
        0
    }
    fn write(&mut self, a: usize, v: u32) {
        self.ops.push(Op::W(a, v));
        if a == 0x0701_01ac {
            self.gate = v;
        }
    }
}

thread_local! {
    static LEGACY: RefCell<Model> = RefCell::new(Model::default());
}

mod util {
    pub mod mmio {
        pub fn read32(a: usize) -> u32 {
            super::super::LEGACY.with(|m| m.borrow_mut().read(a))
        }
        pub fn write32(a: usize, v: u32) {
            super::super::LEGACY.with(|m| m.borrow_mut().write(a, v))
        }
    }
}
mod cubie_a7a_common {
    pub mod time {
        pub fn udelay(us: u64) {
            super::super::LEGACY.with(|m| m.borrow_mut().ops.push(super::super::Op::D(us as u32)))
        }
    }
}

#[allow(dead_code, unused_imports, clippy::all)]
mod a7a {
    use super::{cubie_a7a_common, util};
    include!("legacy/a7a_ppu.rs");
}

struct Io(Model);
impl RegIo for Io {
    fn read32(&mut self, a: usize) -> u32 {
        self.0.read(a)
    }
    fn write32(&mut self, a: usize, v: u32) {
        self.0.write(a, v)
    }
    fn read16(&mut self, a: usize) -> u16 {
        self.0.read(a) as u16
    }
    fn write16(&mut self, a: usize, v: u16) {
        self.0.write(a, v as u32)
    }
    fn udelay(&mut self, us: u32) {
        self.0.ops.push(Op::D(us))
    }
    fn now_us(&mut self) -> u64 {
        0
    }
    fn log(&mut self, _: core::fmt::Arguments) {}
}

/// The A733 instance, as the board will declare it.
const A733: Pck600 = Pck600 {
    base: 0x0706_0000,
    frame_shift: 12,
    count: 11,
    setup: &[
        (0x170, 0x001f_1f1f),
        (0x174, 0x0000_1f1f),
        (0xc00, 0x0808_0808),
        (0xc04, 0x0000_0808),
        (0xc10, 0x0000_0008),
    ],
    gate: Some(Gate {
        addr: 0x0701_01ac,
        bit: 0,
        settle_us: 10,
    }),
    polls: 1000,
    poll_us: 10,
};

fn model(gate: u32, on_after: Option<u32>, pwpr: u32) -> Model {
    Model {
        ops: Vec::new(),
        gate,
        on_after,
        pwpr,
    }
}

#[test]
fn power_on_and_is_on_traces_match() {
    let mut x: u64 = 0x5eed_0bb0;
    for _ in 0..3000 {
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        let domain = (x % 14) as u32;
        let gate = ((x >> 8) & 1) as u32 | 0x10;
        let on_after = match (x >> 9) % 4 {
            0 => None,
            1 => Some(0),
            _ => Some(((x >> 12) % 1200) as u32),
        };
        let pwpr = (x >> 24) as u32;

        LEGACY.with(|m| *m.borrow_mut() = model(gate, on_after, pwpr));
        let r_old = a7a::power_on(domain);
        let on_old = a7a::is_on(domain);
        let want = LEGACY.with(|m| std::mem::take(&mut m.borrow_mut().ops));

        let mut io = Io(model(gate, on_after, pwpr));
        let r_new = A733.power_on(&mut io, domain);
        let on_new = A733.is_on(&mut io, domain);
        assert_eq!(io.0.ops, want, "domain {domain} on_after {on_after:?}");
        let r_old = r_old.map_err(|e| match e {
            a7a::PpuError::BadDomain(d) => PpuError::BadDomain(d),
            a7a::PpuError::Timeout(s) => PpuError::Timeout(s),
        });
        assert_eq!(r_new, r_old);
        assert_eq!(on_new, on_old);
    }
}
