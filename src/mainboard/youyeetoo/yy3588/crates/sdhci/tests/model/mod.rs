// SPDX-License-Identifier: GPL-2.0-only
//! Host model of the SpacemiT K1 SDHCI controllers (SDH0 SD, SDH2 eMMC), the
//! APMU clock/reset registers and one card, for the equivalence tests.
//!
//! Every access inside the controller and APMU windows is recorded in a
//! trace; any other address is host memory (the tests' buffers) and is read
//! or written for real. Both the verbatim K1 driver (through its global
//! `util::mmio` functions) and the crate (through `RegIo`) drive the same
//! model, so their traces, memory effects and results can be compared.
#![allow(dead_code)]

use std::cell::RefCell;
use std::collections::HashMap;

pub const SD_BASE: usize = 0xd428_0000;
pub const EMMC_BASE: usize = 0xd428_1000;
pub const APMU_SDH0: usize = 0xd428_2854;
pub const APMU_SDH2: usize = 0xd428_28e0;
const WINDOW: std::ops::Range<usize> = 0xd428_0000..0xd428_3000;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Access {
    R(u8, usize, u32),
    W(u8, usize, u32),
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Card {
    Emmc,
    SdHc,
    SdV1,
}

pub struct Model {
    pub trace: Vec<Access>,
    regs: HashMap<usize, u8>,
    pub card: Card,
    pub caps_mhz: u32,
    /// eMMC CMD1 answers busy this many times first.
    pub ocr_busy: u32,
    pub rca: u32,
    pub ext_csd: [u8; 512],
    /// Sector data per (partition, lba).
    pub data: HashMap<(u8, u32), [u8; 512]>,
    pub sd_c_size: u32,
    /// Command index that fails (INT_ERROR) `fail_times` times.
    pub fail_cmd: Option<u8>,
    pub fail_times: u32,
    /// Full host address the next SDMA transfer reads from (the register
    /// holds only 32 bits of it).
    pub dma_source: usize,
    pub dma_boundary_once: bool,
    pub trims: Vec<(u32, u32)>,
    app: bool,
    trim_start: u32,
    // data phase
    read_buf: Vec<u32>,
    write_words: Vec<u32>,
    write_lba: u32,
    blocks_left: u32,
    dma_pending: Option<(u32, u32)>,
}

impl Default for Model {
    fn default() -> Self {
        let mut ext = [0u8; 512];
        ext[212..216].copy_from_slice(&(0x0074_0000u32).to_le_bytes()); // ~3.6 GiB
        ext[226] = 32; // 4 MiB boot partitions
        ext[231] = 1 << 4; // TRIM
        ext[181] = 0; // erased memory reads 0
        Self {
            trace: Vec::new(),
            regs: HashMap::new(),
            card: Card::Emmc,
            caps_mhz: 200,
            ocr_busy: 2,
            rca: 0,
            ext_csd: ext,
            data: HashMap::new(),
            sd_c_size: 15159, // 7.4 GiB
            fail_cmd: None,
            fail_times: 0,
            dma_source: 0,
            dma_boundary_once: false,
            trims: Vec::new(),
            app: false,
            trim_start: 0,
            read_buf: Vec::new(),
            write_words: Vec::new(),
            write_lba: 0,
            blocks_left: 0,
            dma_pending: None,
        }
    }
}

thread_local! { pub static MODEL: RefCell<Model> = RefCell::new(Model::default()); }

pub fn reset(card: Card) {
    MODEL.with(|m| {
        *m.borrow_mut() = Model {
            card,
            ..Model::default()
        }
    })
}
pub fn with<T>(f: impl FnOnce(&mut Model) -> T) -> T {
    MODEL.with(|m| f(&mut m.borrow_mut()))
}

fn base_of(a: usize) -> Option<usize> {
    if (SD_BASE..SD_BASE + 0x200).contains(&a) {
        Some(SD_BASE)
    } else if (EMMC_BASE..EMMC_BASE + 0x200).contains(&a) {
        Some(EMMC_BASE)
    } else {
        None
    }
}

const INT_CMD: u32 = 1;
const INT_XFER: u32 = 2;
const INT_DMA: u32 = 8;
const INT_WR: u32 = 16;
const INT_RD: u32 = 32;
const INT_ERR: u32 = 1 << 15;

impl Model {
    /// SDMA addresses are host buffer addresses, different in every run:
    /// trace them relative to the buffer.
    fn traced(&self, a: usize, v: u32) -> u32 {
        match base_of(a) {
            Some(b) if a - b == 0 => v.wrapping_sub(self.dma_source as u32),
            _ => v,
        }
    }
    fn get(&self, a: usize, n: usize) -> u32 {
        (0..n).fold(0, |v, i| {
            v | (*self.regs.get(&(a + i)).unwrap_or(&0) as u32) << (8 * i)
        })
    }
    fn put(&mut self, a: usize, n: usize, v: u32) {
        for i in 0..n {
            self.regs.insert(a + i, (v >> (8 * i)) as u8);
        }
    }
    fn status(&self, b: usize) -> u32 {
        self.get(b + 0x30, 4)
    }
    fn raise(&mut self, b: usize, bits: u32) {
        let s = self.status(b) | bits;
        self.put(b + 0x30, 4, s);
    }

    pub fn read(&mut self, a: usize, n: usize) -> u32 {
        let v = self.read_inner(a, n);
        self.trace.push(Access::R(n as u8, a, self.traced(a, v)));
        v
    }
    fn read_inner(&mut self, a: usize, n: usize) -> u32 {
        let Some(b) = base_of(a) else {
            return self.get(a, n);
        };
        match a - b {
            0x24 => 0, // present state: never inhibited
            0x2c if n == 2 => {
                let v = self.get(a, 2);
                if v & 1 != 0 {
                    v | 2
                } else {
                    v
                }
            }
            0x2f => 0, // reset completes at once
            0x40 => self.caps_mhz << 8,
            0x20 => {
                let v = if self.read_buf.is_empty() {
                    0
                } else {
                    self.read_buf.remove(0)
                };
                if self.read_buf.is_empty() {
                    self.raise(b, INT_XFER);
                }
                v
            }
            _ => self.get(a, n),
        }
    }

    pub fn write(&mut self, a: usize, n: usize, v: u32) {
        let t = self.traced(a, v);
        self.trace.push(Access::W(n as u8, a, t));
        let Some(b) = base_of(a) else {
            self.put(a, n, v);
            return;
        };
        match a - b {
            0x30 => {
                let s = self.status(b) & !v;
                self.put(a, 4, s);
            }
            0x2f => {}
            0x00 => {
                self.put(a, 4, v);
                if let Some((lba, blocks)) = self.dma_pending.take() {
                    self.dma_write(b, lba, blocks);
                }
            }
            0x0e => {
                self.put(a, 2, v);
                self.command(b, (v >> 8) as u8, v & 0xff);
            }
            0x20 => {
                self.write_words.push(v);
                if self.write_words.len() == 128 {
                    let mut s = [0u8; 512];
                    for (i, w) in self.write_words.drain(..).enumerate() {
                        s[4 * i..4 * i + 4].copy_from_slice(&w.to_le_bytes());
                    }
                    let p = self.part();
                    self.data.insert((p, self.write_lba), s);
                    self.write_lba += 1;
                    self.blocks_left -= 1;
                    if self.blocks_left == 0 {
                        self.raise(b, INT_XFER);
                    } else {
                        self.raise(b, INT_WR);
                    }
                }
            }
            _ => self.put(a, n, v),
        }
    }

    fn part(&self) -> u8 {
        if self.card == Card::Emmc {
            self.ext_csd[179] & 7
        } else {
            0
        }
    }
    fn lba(&self, arg: u32) -> u32 {
        if self.card == Card::SdV1 {
            arg / 512
        } else {
            arg
        }
    }
    fn respond(&mut self, b: usize, r: [u32; 4]) {
        for (i, w) in r.iter().enumerate() {
            self.put(b + 0x10 + 4 * i, 4, *w);
        }
    }
    fn tran(&self) -> u32 {
        (4 << 9) | (1 << 8)
    }

    fn command(&mut self, b: usize, index: u8, flags: u32) {
        let arg = self.get(b + 0x08, 4);
        let mode = self.get(b + 0x0c, 2);
        let blocks = self.get(b + 0x06, 2);
        let app = std::mem::take(&mut self.app);
        if self.fail_cmd == Some(index) && self.fail_times > 0 {
            self.fail_times -= 1;
            self.raise(b, INT_ERR | (1 << 16));
            return;
        }
        let emmc = self.card == Card::Emmc;
        let mut r = [self.tran(), 0, 0, 0];
        match (index, app) {
            (0, _) | (7, _) | (12, _) | (16, _) => {}
            (1, _) if emmc => {
                r[0] = if self.ocr_busy > 0 {
                    self.ocr_busy -= 1;
                    0x00ff_8080
                } else {
                    0xc0ff_8080
                }
            }
            (2, _) => r = [0x1111_1111, 0x2222_2222, 0x3333_3333, 0x0044_4444],
            (3, _) if emmc => self.rca = arg >> 16,
            (3, _) => {
                self.rca = 0x1234;
                r[0] = 0x1234 << 16;
            }
            (8, _) if !emmc => {
                if self.card == Card::SdV1 {
                    self.raise(b, INT_ERR | (1 << 16));
                    return;
                }
                r[0] = arg & 0xfff;
            }
            (55, _) => self.app = true,
            (41, true) => {
                let ccs = if self.card == Card::SdHc && arg & (1 << 30) != 0 {
                    1 << 30
                } else {
                    0
                };
                r[0] = (1 << 31) | ccs | 0x00ff_8000;
            }
            (9, _) => {
                let csd: u128 = if self.card == Card::SdHc {
                    (1u128 << 126) | ((self.sd_c_size as u128) << 48)
                } else {
                    (9u128 << 80) | (4095u128 << 62) | (7u128 << 47)
                };
                r = [
                    (csd >> 8) as u32,
                    (csd >> 40) as u32,
                    (csd >> 72) as u32,
                    (csd >> 104) as u32,
                ];
            }
            (6, true) => {}
            (6, false) => {
                let (access, idx, val) = (
                    (arg >> 24) & 3,
                    ((arg >> 16) & 0xff) as usize,
                    (arg >> 8) & 0xff,
                );
                let e = &mut self.ext_csd[idx];
                match access {
                    1 => *e |= val as u8,
                    2 => *e &= !(val as u8),
                    _ => *e = val as u8,
                }
            }
            (13, _) => {}
            (35, _) => self.trim_start = arg,
            (36, _) => self.trims.push((self.trim_start, arg)),
            (38, _) => {
                let (s, e) = *self.trims.last().unwrap();
                let p = self.part();
                for l in s..=e {
                    self.data.remove(&(p, l));
                }
            }
            (8, _) | (17, _) => {
                let sector = if index == 8 {
                    self.ext_csd
                } else {
                    let l = self.lba(arg);
                    *self.data.get(&(self.part(), l)).unwrap_or(&[0; 512])
                };
                self.read_buf = sector
                    .chunks(4)
                    .map(|c| u32::from_le_bytes(c.try_into().unwrap()))
                    .collect();
                self.respond(b, r);
                self.raise(b, INT_CMD | INT_RD);
                return;
            }
            (24, _) | (25, _) => {
                self.respond(b, r);
                let lba = self.lba(arg);
                let n = if index == 24 { 1 } else { blocks };
                if mode & 1 != 0 {
                    self.raise(b, INT_CMD);
                    if self.dma_boundary_once && n > 1 {
                        self.dma_boundary_once = false;
                        self.dma_pending = Some((lba, n));
                        self.raise(b, INT_DMA);
                    } else {
                        self.dma_write(b, lba, n);
                    }
                } else {
                    self.write_lba = lba;
                    self.blocks_left = n;
                    self.raise(b, INT_CMD | INT_WR);
                }
                return;
            }
            _ => r[0] = 0,
        }
        self.respond(b, r);
        self.raise(b, INT_CMD);
        if flags & 3 == 3 {
            self.raise(b, INT_XFER);
        }
    }

    fn dma_write(&mut self, b: usize, lba: u32, blocks: u32) {
        let p = self.part();
        for i in 0..blocks as usize {
            let mut s = [0u8; 512];
            for (k, byte) in s.iter_mut().enumerate() {
                *byte = unsafe { ((self.dma_source + i * 512 + k) as *const u8).read_volatile() };
            }
            self.data.insert((p, lba + i as u32), s);
        }
        self.raise(b, INT_XFER);
    }
}

/// Host-memory access for addresses outside the register windows.
pub fn access(a: usize, n: usize, write: Option<u32>) -> u32 {
    if WINDOW.contains(&a) {
        with(|m| match write {
            Some(v) => {
                m.write(a, n, v);
                0
            }
            None => m.read(a, n),
        })
    } else {
        unsafe {
            match (n, write) {
                (1, Some(v)) => (a as *mut u8).write_volatile(v as u8),
                (2, Some(v)) => (a as *mut u16).write_volatile(v as u16),
                (4, Some(v)) => (a as *mut u32).write_volatile(v),
                (1, None) => return (a as *const u8).read_volatile() as u32,
                (2, None) => return (a as *const u16).read_volatile() as u32,
                (4, None) => return (a as *const u32).read_volatile(),
                _ => unreachable!(),
            }
        }
        0
    }
}
