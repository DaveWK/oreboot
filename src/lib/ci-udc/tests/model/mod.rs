// SPDX-License-Identifier: GPL-2.0-only
//! The K1 bt0 USB host model (ore-edk-boot-opi 78479e21ba tests/usb-model.rs)
//! as a shared model for the equivalence tests, extended with SETUP packet
//! injection, ep0 completion and a record of what ep0 sends. Register
//! accesses are hashed into a running trace (the timeout scenarios make
//! hundreds of millions of them); EPLISTADDR values are host addresses and
//! are traced as 0.
#![allow(dead_code)]

use std::cell::RefCell;

pub const CMD: usize = 0xc0900140;
pub const STS: usize = 0xc0900144;
pub const LIST: usize = 0xc0900158;
pub const PORTSC: usize = 0xc0900184;
pub const SETUPSTAT: usize = 0xc09001ac;
pub const PRIME: usize = 0xc09001b0;
pub const FLUSH: usize = 0xc09001b4;
pub const STAT: usize = 0xc09001b8;
pub const COMPLETE: usize = 0xc09001bc;
pub const PHY: usize = 0xc0940004;
pub const APMU_USB: usize = 0xd428_285c;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Transfer {
    Never,
    Complete,
    After(usize),
    Short,
    Error,
}

pub struct Model {
    pub phy_at: usize,
    pub phy_reads: usize,
    pub reset_at: usize,
    pub reset_reads: usize,
    pub cmd: u32,
    pub flush_stuck: bool,
    pub stat_stuck: bool,
    pub flush_mask: u32,
    pub flush_reads: usize,
    pub flush_writes: usize,
    pub list_writes: usize,
    pub run_writes: usize,
    pub prime_writes: usize,
    pub cache_writes: usize,
    pub tds: Vec<usize>,
    pub invalidations: usize,
    pub transfer: Transfer,
    pub status: u32,
    pub setup: u32,
    // extensions
    pub portsc: u32,
    pub complete: u32,
    pub setup_bytes: Option<[u8; 8]>,
    pub ep0_log: Vec<Vec<u8>>,
    pub other: std::collections::HashMap<usize, u32>,
    pub hash: u64,
    pub accesses: u64,
    /// Low 32 bits of the queue head list (EPLISTADDR).
    pub list: u32,
    ep0_pending: bool,
}

impl Default for Model {
    fn default() -> Self {
        Self {
            phy_at: 1,
            phy_reads: 0,
            reset_at: 1,
            reset_reads: 0,
            cmd: 1,
            flush_stuck: false,
            stat_stuck: false,
            flush_mask: 0,
            flush_reads: 0,
            flush_writes: 0,
            list_writes: 0,
            run_writes: 0,
            prime_writes: 0,
            cache_writes: 0,
            tds: vec![],
            invalidations: 0,
            transfer: Transfer::Complete,
            status: 0,
            setup: 0,
            portsc: 0,
            complete: 0,
            setup_bytes: None,
            ep0_log: vec![],
            other: Default::default(),
            hash: 0xcbf2_9ce4_8422_2325,
            accesses: 0,
            list: 0,
            ep0_pending: false,
        }
    }
}

thread_local! { pub static MODEL: RefCell<Model> = RefCell::new(Model::default()); }

pub fn change(f: impl FnOnce(&mut Model)) {
    MODEL.with(|m| f(&mut m.borrow_mut()))
}
pub fn state<T>(f: impl FnOnce(&Model) -> T) -> T {
    MODEL.with(|m| f(&m.borrow()))
}
pub fn reset() {
    change(|m| *m = Model::default())
}

impl Model {
    fn trace(&mut self, write: bool, a: usize, v: u32) {
        let v = if write && a == LIST { 0 } else { v };
        for x in [write as u64, a as u64, v as u64] {
            self.hash = (self.hash ^ x).wrapping_mul(0x0100_0000_01b3);
        }
        self.accesses += 1;
    }
}

pub fn read32(a: usize) -> u32 {
    MODEL.with(|s| {
        let mut m = s.borrow_mut();
        let v = match a {
            PHY => {
                m.phy_reads += 1;
                u32::from(m.phy_reads >= m.phy_at)
            }
            CMD => {
                if m.cmd & 2 != 0 {
                    m.reset_reads += 1;
                    if m.reset_reads >= m.reset_at {
                        m.cmd &= !2
                    }
                }
                m.cmd
            }
            FLUSH => {
                m.flush_reads += 1;
                if m.flush_stuck {
                    m.flush_mask
                } else {
                    0
                }
            }
            STAT => {
                if m.stat_stuck {
                    m.flush_mask
                } else {
                    0
                }
            }
            STS => m.status,
            SETUPSTAT => m.setup,
            PORTSC => m.portsc,
            COMPLETE => m.complete,
            _ => *m.other.get(&a).unwrap_or(&0),
        };
        m.trace(false, a, v);
        v
    })
}

pub fn write32(a: usize, v: u32) {
    change(|m| {
        m.trace(true, a, v);
        match a {
            CMD => {
                m.cmd = v;
                if v & 1 != 0 {
                    m.run_writes += 1
                }
            }
            FLUSH => {
                m.flush_mask = v;
                m.flush_writes += 1
            }
            LIST => {
                m.list_writes += 1;
                m.list = v;
            }
            PRIME => {
                m.prime_writes += 1;
                // ep0 transfers complete at once.
                m.complete |= v & 0x0001_0001;
            }
            STS => m.status &= !v,
            SETUPSTAT => m.setup &= !v,
            COMPLETE => m.complete &= !v,
            _ => {
                m.other.insert(a, v);
            }
        }
    })
}

pub fn flush(a: usize, n: usize) {
    change(|m| {
        m.cache_writes += 1;
        if n == 32 {
            m.tds.push(a);
            if std::mem::take(&mut m.ep0_pending) {
                // The ep0 TD follows its buffer: keep the bytes it sends.
                let token = unsafe { ((a + 4) as *const u32).read_volatile() };
                let len = (token >> 16 & 0x7fff) as usize;
                m.ep0_log.last_mut().unwrap().truncate(len);
            }
        }
        // (The queue head array is 256 bytes too, at the page-aligned start
        // of the DMA area.)
        if n == 256 && a as u32 != m.list && !a.is_multiple_of(4096) {
            // The ep0 buffer just before an IN data stage.
            let bytes = (0..n)
                .map(|i| unsafe { ((a + i) as *const u8).read_volatile() })
                .collect();
            m.ep0_log.push(bytes);
            m.ep0_pending = true;
        }
    })
}

pub fn invalidate(a: usize, n: usize) {
    if n == 64 {
        // The SETUP packet lands in queue head 0 (offset 40).
        change(|m| {
            if let Some(s) = m.setup_bytes.take() {
                for (i, b) in s.iter().enumerate() {
                    unsafe { ((a + 40 + i) as *mut u8).write_volatile(*b) }
                }
            }
        });
        return;
    }
    if n != 32 {
        return;
    }
    change(|m| {
        m.invalidations += 1;
        let token = match m.transfer {
            Transfer::Complete => Some(0),
            Transfer::After(n) if m.invalidations >= n => Some(0),
            Transfer::Short if m.tds.first() == Some(&a) => Some(1 << 16),
            Transfer::Error if m.tds.first() == Some(&a) => Some(1 << 6),
            _ => None,
        };
        // Only actual host TD addresses collected by flush are touched.
        if let Some(t) = token {
            assert!(m.tds.contains(&a));
            unsafe { ((a + 4) as *mut u32).write_volatile(t) }
        }
    })
}
