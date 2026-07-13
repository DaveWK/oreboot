// SPDX-License-Identifier: GPL-2.0-only
//! The crate, driven by an A733 Platform made of the board's own functions,
//! against the verbatim A7A board bring-up (`legacy/a7a_pcie.rs`): both run
//! on one simulated controller (PHY, DBI, glue, iATU, endpoint, a 24 MHz
//! clock) in many scenarios; the register-access traces (every 16/32-bit
//! read and write with address and value) and the results must be equal.
#![allow(static_mut_refs, clippy::all)]

use std::cell::RefCell;
use std::collections::HashMap;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Scenario {
    #[default]
    Good,
    Empty,
    NeverUp,
    PmaTimeout,
    NoCap,
    NoEndpoint,
    RetryThenAnswer,
    DropAfterSpeedChange,
    DropForGood,
    StuckSpeedBit,
    AtuStuck,
    PpuFails,
    Gen1Only,
}

#[derive(Default)]
pub struct Model {
    r32: HashMap<usize, u32>,
    r16: HashMap<usize, u16>,
    pub clock_us: u64,
    pub trace: Vec<(char, u8, usize, u32)>,
    pub sc: Scenario,
    ltssm_on: Option<u64>,
    speed_req: Option<u64>,
    retrained: bool,
    cfg_reads: u32,
}

const DBI: usize = 0x0600_0000;
const APP: usize = DBI + 0x0040_0000;
const APP_LTSSM_CTRL: usize = APP + 0xc00;
const APP_LINK_STAT: usize = APP + 0xe0c;
const C1_TOP: usize = 0x06c0_2000;
const CAP: usize = 0x40;
const CFG: usize = 0x2000_0000;

impl Model {
    fn up(&self) -> bool {
        let Some(on) = self.ltssm_on else {
            return false;
        };
        let sc = self.sc;
        if matches!(sc, Scenario::Empty | Scenario::NeverUp) || self.clock_us < on + 3000 {
            return false;
        }
        if let Some(t) = self.speed_req {
            match sc {
                Scenario::DropAfterSpeedChange => return self.retrained,
                Scenario::DropForGood => return false,
                _ => return self.clock_us >= t + 300,
            }
        }
        true
    }
    fn read32(&mut self, a: usize) -> u32 {
        let v = match a {
            _ if a == C1_TOP + 0x900 => (self.sc != Scenario::PmaTimeout) as u32,
            APP_LINK_STAT => {
                if self.up() {
                    3
                } else {
                    0
                }
            }
            _ if a == DBI + 0x728 => match (self.ltssm_on, self.sc) {
                (None, _) | (_, Scenario::Empty) => 0,
                _ if self.up() => 0x11,
                _ => 5,
            },
            _ if a == DBI + 0x80c => {
                let v = *self.r32.get(&a).unwrap_or(&0);
                if self.sc == Scenario::StuckSpeedBit {
                    v
                } else {
                    v & !(1 << 17)
                }
            }
            _ if (0x0630_0004..0x0630_0004 + 3 * 0x200).contains(&a)
                && (a - 0x0630_0004) % 0x200 == 0 =>
            {
                if self.sc == Scenario::AtuStuck {
                    0
                } else {
                    *self.r32.get(&a).unwrap_or(&0)
                }
            }
            CFG => {
                self.cfg_reads += 1;
                match self.sc {
                    Scenario::NoEndpoint => 0xffff_ffff,
                    Scenario::RetryThenAnswer if self.cfg_reads < 5 => 0x0000_0001,
                    _ => 0x1234_8086,
                }
            }
            _ if a == CFG + 8 => 0x0108_0200,
            _ => *self.r32.get(&a).unwrap_or(&0),
        };
        self.trace.push(('r', 32, a, v));
        v
    }
    fn write32(&mut self, a: usize, v: u32) {
        if a == APP_LTSSM_CTRL {
            if v & 1 != 0 && self.ltssm_on.is_none() {
                self.ltssm_on = Some(self.clock_us);
            }
            if v & 1 == 0 {
                self.ltssm_on = None;
            }
        }
        if a == DBI + 0x80c && v & (1 << 17) != 0 {
            self.speed_req = Some(self.clock_us);
        }
        if a == DBI + CAP + 0x10 && v & (1 << 5) != 0 {
            self.retrained = true;
        }
        self.trace.push(('w', 32, a, v));
        self.r32.insert(a, v);
    }
    fn read16(&mut self, a: usize) -> u16 {
        let v = match a {
            _ if a == DBI + 0x34 => {
                if self.sc == Scenario::NoCap {
                    0
                } else {
                    CAP as u16
                }
            }
            _ if a == DBI + CAP => 0x0010,
            _ if a == DBI + CAP + 0x12 => {
                if self.sc == Scenario::Gen1Only {
                    0x0011
                } else {
                    0x0013
                }
            }
            _ => *self.r16.get(&a).unwrap_or(&0),
        };
        self.trace.push(('r', 16, a, v as u32));
        v
    }
    fn write16(&mut self, a: usize, v: u16) {
        self.trace.push(('w', 16, a, v as u32));
        self.r16.insert(a, v);
    }
}

thread_local! {
    pub static M: RefCell<Model> = RefCell::new(Model::default());
}
pub fn read32(a: usize) -> u32 {
    M.with(|m| m.borrow_mut().read32(a))
}
pub fn write32(a: usize, v: u32) {
    M.with(|m| m.borrow_mut().write32(a, v))
}
pub fn read16(a: usize) -> u16 {
    M.with(|m| m.borrow_mut().read16(a))
}
pub fn write16(a: usize, v: u16) {
    M.with(|m| m.borrow_mut().write16(a, v))
}
pub fn clock() -> u64 {
    M.with(|m| m.borrow().clock_us)
}
pub fn advance(us: u64) {
    M.with(|m| m.borrow_mut().clock_us += us)
}

/// The crate's register access on the same model.
pub struct Io;
impl regio::RegIo for Io {
    fn read32(&mut self, a: usize) -> u32 {
        read32(a)
    }
    fn write32(&mut self, a: usize, v: u32) {
        write32(a, v)
    }
    fn read16(&mut self, a: usize) -> u16 {
        read16(a)
    }
    fn write16(&mut self, a: usize, v: u16) {
        write16(a, v)
    }
    fn udelay(&mut self, us: u32) {
        advance(us as u64)
    }
    fn now_us(&mut self) -> u64 {
        clock()
    }
    fn log(&mut self, _: std::fmt::Arguments) {}
}

pub mod ppu {
    #[derive(Debug, Clone, Copy)]
    pub enum PpuError {
        Timeout,
    }
    pub const PCIE: u32 = 7;
    pub fn power_on(_domain: u32) -> Result<(), PpuError> {
        if crate::M.with(|m| m.borrow().sc) == crate::Scenario::PpuFails {
            Err(PpuError::Timeout)
        } else {
            Ok(())
        }
    }
}
pub mod fdt {
    pub struct Fdt;
    impl Fdt {
        pub fn begin_node(&mut self, _: &str) {}
        pub fn end_node(&mut self) {}
        pub fn prop_str(&mut self, _: &str, _: &str) {}
        pub fn prop_u32(&mut self, _: &str, _: u32) {}
        pub fn prop_u32_cells(&mut self, _: &str, _: &[u32]) {}
        pub fn prop_u64_cells(&mut self, _: &str, _: &[u64]) {}
    }
}

#[allow(dead_code, unused_imports, unused_macros)]
mod legacy {
    mod util {
        pub mod mmio {
            pub use crate::{read16, read32, write16, write32};
        }
    }
    mod cubie_a7a_common {
        pub mod time {
            pub const TIMER_HZ: u64 = 24_000_000;
            pub fn ticks() -> u64 {
                crate::clock() * 24
            }
            pub fn udelay(us: u64) {
                crate::advance(us)
            }
        }
    }
    macro_rules! println {
        ($($t:tt)*) => {{
            use std::fmt::Write as _;
            let mut s = String::new();
            let _ = write!(s, $($t)*);
        }};
    }
    include!("legacy/a7a_pcie.rs");

    /// The board run exactly as its init() does, without the prints and the
    /// static: Ok(link fields) or the error's Debug text.
    pub fn run_board() -> Result<(u32, u32, u32, u32), String> {
        let mut clocked = false;
        match bring_up(&mut clocked) {
            Ok(l) => Ok((l.gen, l.width, l.id, l.class)),
            Err(e) => {
                shut_down(clocked);
                Err(format!("{e:?}"))
            }
        }
    }

    /// The board's (private) error, shown as the board shows it.
    #[derive(Clone, Copy)]
    pub struct BoardError(Error);
    impl std::fmt::Debug for BoardError {
        fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
            self.0.fmt(f)
        }
    }

    /// The A733 around the DesignWare core, from the board's functions.
    pub struct A733;
    impl dwc_pcie::Platform<crate::Io> for A733 {
        type Error = BoardError;
        fn perst(&mut self, _: &mut crate::Io, asserted: bool) {
            pe_out(PIN_PERST, !asserted);
        }
        fn slot_power(&mut self, _: &mut crate::Io, on: bool) {
            pe_out(PIN_POWER, on);
            if on {
                pe_out(PIN_WAKE, true);
            }
        }
        fn power_domain_on(&mut self, _: &mut crate::Io) -> Result<(), BoardError> {
            ppu::power_on(ppu::PCIE).map_err(|e| BoardError(Error::PowerDomain(e)))
        }
        fn clocks_on(&mut self, _: &mut crate::Io) {
            clocks_on();
        }
        fn phy_init(&mut self, _: &mut crate::Io) -> Result<(), BoardError> {
            phy_init().map_err(BoardError)
        }
        fn after_phy(&mut self, _: &mut crate::Io) {
            let _ = (read32(APP_LTSSM_CTRL), read32(APP_PHY_CFG));
        }
        fn ltssm_enable(&mut self, _: &mut crate::Io, on: bool) {
            if on {
                set32(APP_LTSSM_CTRL, LTSSM_EN);
            } else {
                clr32(APP_LTSSM_CTRL, LTSSM_EN);
            }
        }
        fn link_up(&mut self, _: &mut crate::Io) -> bool {
            link_up()
        }
        fn link_status(&mut self, _: &mut crate::Io) -> u32 {
            read32(APP_LINK_STAT)
        }
        fn clocks_off(&mut self, _: &mut crate::Io) {
            clr32(ITS_BGR, (1 << 16) | (1 << 1));
            clr32(PCIE_AXI_SLV_CLK, GATE);
            clr32(PCIE_AUX_CLK, GATE);
            clr32(PCIE_BGR, (1 << 17) | (1 << 16));
        }
    }

    pub const CONFIG: dwc_pcie::Config = dwc_pcie::Config {
        dbi: DBI,
        dbi2: DBI2,
        atu: ATU,
        max_gen: MAX_GEN,
        lanes: 1,
        perst_hold_us: PERST_HOLD_US,
        config: dwc_pcie::Window {
            cpu: CFG_BASE,
            pci: 0,
            size: CFG_WINDOW,
        },
        mem: dwc_pcie::Window {
            cpu: MEM_BASE,
            pci: MEM_BASE,
            size: MEM_SIZE,
        },
        io: dwc_pcie::Window {
            cpu: IO_CPU,
            pci: 0,
            size: IO_SIZE,
        },
    };
}

fn reset(sc: Scenario) {
    M.with(|m| {
        *m.borrow_mut() = Model {
            sc,
            clock_us: 1_000,
            ..Model::default()
        }
    });
}
fn trace() -> Vec<(char, u8, usize, u32)> {
    M.with(|m| std::mem::take(&mut m.borrow_mut().trace))
}

#[test]
fn every_scenario_matches_the_board_code() {
    use Scenario::*;
    for sc in [
        Good,
        Empty,
        NeverUp,
        PmaTimeout,
        NoCap,
        NoEndpoint,
        RetryThenAnswer,
        DropAfterSpeedChange,
        DropForGood,
        StuckSpeedBit,
        AtuStuck,
        PpuFails,
        Gen1Only,
    ] {
        reset(sc);
        let board = legacy::run_board();
        let board_trace = trace();
        let board_clock = clock();
        reset(sc);
        let r = dwc_pcie::init(&mut Io, &mut legacy::A733, &legacy::CONFIG);
        let crate_trace = trace();
        let ours = match r {
            Ok(l) => Ok((l.gen, l.width, l.id, l.class)),
            Err(dwc_pcie::Error::Platform(e)) => Err(format!("{e:?}")),
            Err(e) => Err(format!("{e:?}")),
        };
        assert_eq!(board, ours, "{sc:?}: result");
        assert_eq!(board_trace.len(), crate_trace.len(), "{sc:?}: trace length");
        for (i, (a, b)) in board_trace.iter().zip(crate_trace.iter()).enumerate() {
            assert_eq!(a, b, "{sc:?}: access {i}");
        }
        assert_eq!(board_clock, clock(), "{sc:?}: elapsed time");
        let least = if sc == PpuFails { 4 } else { 20 };
        assert!(board_trace.len() > least, "{sc:?}: the run did something");
    }
}

#[test]
fn the_good_link_is_reported() {
    reset(Scenario::Good);
    let l = dwc_pcie::init(&mut Io, &mut legacy::A733, &legacy::CONFIG).unwrap();
    assert_eq!(
        (l.gen, l.width, l.id, l.class),
        (3, 1, 0x1234_8086, 0x01_0802)
    );
}
