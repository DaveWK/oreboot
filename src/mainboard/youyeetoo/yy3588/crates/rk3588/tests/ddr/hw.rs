// SPDX-License-Identifier: GPL-2.0-only
//! A behavioural model of the RK3588 DDR blocks for host tests: write-masked
//! CRU/GRF registers, the DDRCTL status bits the init polls (SWSTAT,
//! DFISTAT, STAT, MRSTAT), MRR answers per channel and rank, the PHY's
//! ZQ/DLL/training/CBT done bits and fail status, and DRAM as a sparse
//! word store that aliases above the fitted size and can corrupt reads
//! while the DFI read latency is outside a window. Every access is logged
//! as a regtrace event.

#![allow(dead_code)]

use regio::RegIo;
use regtrace::Ev;
use rk3588::regs::{base, ctl, ddr_cru, phy};
use std::collections::BTreeMap;
use std::fmt;

#[derive(Clone, Copy, Debug)]
pub struct Part {
    /// Ranks actually fitted on the channel (0 = channel dead).
    pub ranks: u8,
    pub mr5: u8,
    pub mr8: u8,
}

pub struct Hw {
    pub regs: BTreeMap<usize, u32>,
    pub evs: Vec<Ev>,
    pub text: String,
    pub clock: u64,
    pub parts: [Part; 4],
    /// Normal mode reached since the last MSTR0 write (per channel).
    inited: [bool; 4],
    /// MRW log: (channel, rank, MR, opcode).
    pub mrw: Vec<(usize, u8, u8, u8)>,
    /// CAL_FAIL_STAT0 the next full training returns, per channel.
    pub train_fail: [u32; 4],
    /// Rank-1 gate probe result override (None: follows `parts`).
    pub gate_rank1: Option<bool>,
    pub zq_fail: bool,
    /// Bytes of DRAM actually fitted (addresses alias modulo this).
    pub dram_bytes: u64,
    pub dram: BTreeMap<u64, u32>,
    /// DRAM reads come back corrupted unless DFITMG0's RL offset
    /// (RL - dfi_t_rddata_en) of the channel holding the address is in
    /// this window (None: always good). Linear layout assumed.
    pub good_rd_sub: Option<(u32, u32)>,
    pub log_reads: bool,
}

fn masked(addr: usize) -> bool {
    // CRU, DDR CRUs, DDR GRFs, CENTER GRF: Rockchip write-masked registers
    // (status registers are not written by the init).
    (base::CRU..base::CRU + 0x1000).contains(&addr)
        || (base::DDR_CRU[0]..base::DDR_CRU[3] + 0x4000).contains(&addr)
        || (base::DDR01_GRF..base::CENTER_GRF + 0x1000).contains(&addr)
}

fn ctl_ch(addr: usize) -> Option<(usize, usize)> {
    base::DDRCTL
        .iter()
        .position(|b| (*b..*b + 0x100_0000).contains(&addr))
        .map(|c| (c, addr - base::DDRCTL[c]))
}

fn phy_ch(addr: usize) -> Option<(usize, usize)> {
    base::DDRPHY
        .iter()
        .position(|b| (*b..*b + 0x1_0000).contains(&addr))
        .map(|c| (c, addr - base::DDRPHY[c]))
}

/// DRAM is everything outside the 0xF000_0000..0x1_0000_0000 register
/// window (P1 Table 1-1).
pub fn is_dram(addr: usize) -> bool {
    !(0xF000_0000..0x1_0000_0000).contains(&addr)
}

impl Hw {
    pub fn new(parts: [Part; 4]) -> Hw {
        let mut hw = Hw {
            regs: BTreeMap::new(),
            evs: Vec::new(),
            text: String::new(),
            clock: 0,
            parts,
            inited: [false; 4],
            mrw: Vec::new(),
            train_fail: [0; 4],
            gate_rank1: None,
            zq_fail: false,
            dram_bytes: 0,
            dram: BTreeMap::new(),
            good_rd_sub: None,
            log_reads: true,
        };
        // Reset values the code read-modify-writes.
        for c in 0..4 {
            let p = base::DDRPHY[c];
            hw.regs.insert(p + phy::GNR_CON0, 0x4400_5011);
            hw.regs.insert(p + phy::MDLL_CON0, 0x2000_016f | (1 << 5));
            hw.regs.insert(p + phy::CAL_CON0, 0x7896_06C0);
            hw.regs.insert(p + phy::CAL_CON2, 0x4200_1300);
            hw.regs.insert(p + phy::CAL_CON4, 0x0083_8041);
            hw.regs.insert(p + phy::LP_CON0, 0x000C_4403);
            hw.regs.insert(p + phy::GATE_CON0, 0x0300_F00F);
            hw.regs.insert(p + phy::ZQ_CON9, 0x1414);
            hw.regs.insert(base::DDRCTL[c] + ctl::DFIMISC, 1);
            hw.regs.insert(base::DDRCTL[c] + ctl::DFIUPD0, 0x8000);
            hw.regs.insert(base::DDRCTL[c] + ctl::HWLPCTL, 3);
        }
        hw.dram_bytes = parts
            .iter()
            .map(|p| {
                rk3588::ddr::detect::geometry(p.mr8).map_or(0, |g| g.channel_mib(p.ranks)) << 20
            })
            .sum::<u64>()
            .max(1 << 20);
        hw
    }

    pub fn get(&self, addr: usize) -> u32 {
        *self.regs.get(&addr).unwrap_or(&0)
    }

    /// All writes, in order.
    pub fn writes(&self) -> Vec<(usize, u32)> {
        self.evs
            .iter()
            .filter_map(|e| match e {
                Ev::W(_, a, v) => Some((*a as usize, *v)),
                _ => None,
            })
            .collect()
    }

    /// Index of the first write matching `f` at or after `from`.
    pub fn find_write(&self, from: usize, f: impl Fn(usize, u32) -> bool) -> Option<usize> {
        self.evs
            .iter()
            .enumerate()
            .skip(from)
            .find_map(|(i, e)| match e {
                Ev::W(_, a, v) if f(*a as usize, *v) => Some(i),
                _ => None,
            })
    }

    fn mrr(&self, ch: usize, rank: u8, ma: u8) -> u32 {
        let p = self.parts[ch];
        if rank >= p.ranks {
            return 0; // nobody drives the bus
        }
        let b = match ma {
            5 => p.mr5,
            6 => 0x00,
            7 => 0x00,
            8 => p.mr8,
            _ => 0,
        } as u32;
        // The byte on both lanes.
        b | (b << 8)
    }

    fn channel_mib(&self) -> u64 {
        let p = self.parts[0];
        rk3588::ddr::detect::geometry(p.mr8).map_or(0, |g| g.channel_mib(p.ranks))
    }

    fn read_ok(&self, addr: u64) -> bool {
        let Some((lo, hi)) = self.good_rd_sub else {
            return true;
        };
        let mib = self.channel_mib().max(1);
        let ch = ((addr >> 20) / mib).min(3) as usize;
        let d = self.get(base::DDRCTL[ch] + ctl::DFITMG0);
        let rl = (self.get(base::DDRCTL[ch] + ctl::dramset1tmg(2)) >> 16) & 0x7f;
        let sub = rl.wrapping_sub((d >> 16) & 0x7f);
        (lo..=hi).contains(&sub)
    }

    fn status(&mut self, addr: usize) -> Option<u32> {
        if let Some((c, off)) = ctl_ch(addr) {
            let b = base::DDRCTL[c];
            let misc = self.get(b + ctl::DFIMISC);
            let start = misc & ctl::DFIMISC_INIT_START != 0;
            return match off {
                o if o == ctl::SWSTAT => Some(1),
                o if o == ctl::DFISTAT => Some(if start { !self.inited[c] as u32 } else { 1 }),
                o if o == ctl::STAT => {
                    if self.get(b + ctl::PWRCTL) & ctl::PWRCTL_SELFREF_SW != 0 {
                        Some(ctl::STAT_MODE_SELFREF)
                    } else if self.inited[c] || (misc & 1 != 0 && !start) {
                        self.inited[c] = true;
                        Some(1)
                    } else {
                        Some(0)
                    }
                }
                o if o == ctl::MRSTAT => Some(ctl::MRSTAT_MRR_DONE),
                _ => None,
            };
        }
        if let Some((c, off)) = phy_ch(addr) {
            let v = self.get(addr);
            return match off {
                o if o == phy::ZQ_CON1 => Some(1 | ((self.zq_fail as u32) << 2)),
                o if o == phy::MDLL_CON1 => Some(phy::MDLL_LOCKED | 0x100),
                o if o == phy::SCHD_TRAIN_CON0 => Some(if v & 1 != 0 { v | 2 } else { v }),
                o if o == phy::CBT_CAL_STAT0 => Some(phy::CBT_HW_CAL_DONE),
                o if o == phy::CAL_FAIL_STAT0 => {
                    let t = self.get(base::DDRPHY[c] + phy::SCHD_TRAIN_CON0);
                    let gate_only = t & 0x7c == 0x10;
                    let rank = if (t >> 10) & 3 == 2 { 1 } else { 0 };
                    Some(if gate_only {
                        let ok = if rank == 1 {
                            self.gate_rank1.unwrap_or(self.parts[c].ranks >= 2)
                        } else {
                            self.parts[c].ranks >= 1
                        };
                        if ok {
                            0
                        } else {
                            0xffff
                        }
                    } else {
                        self.train_fail[c]
                    })
                }
                _ => None,
            };
        }
        for (i, cru) in base::DDR_CRU.iter().enumerate() {
            if addr == cru + ddr_cru::BPLL_CON6 {
                let _ = i;
                return Some(1 << 15);
            }
        }
        None
    }

    fn side_effects(&mut self, addr: usize, val: u32) {
        if let Some((c, off)) = ctl_ch(addr) {
            if off == ctl::MSTR0 {
                self.inited[c] = false;
            }
            if off == ctl::MRCTRL0 && val & ctl::MRCTRL0_MR_WR != 0 {
                let rank = if val & (1 << 5) != 0 { 1 } else { 0 };
                let m1 = self.get(base::DDRCTL[c] + ctl::MRCTRL1);
                let ma = (m1 >> 8) as u8;
                if val & ctl::MRCTRL0_READ != 0 {
                    let d = self.mrr(c, rank, ma);
                    self.regs.insert(base::DDRCTL[c] + ctl::MRRDATA0, d);
                } else {
                    self.mrw.push((c, rank, ma, m1 as u8));
                }
            }
        }
    }
}

impl RegIo for Hw {
    fn read32(&mut self, addr: usize) -> u32 {
        let v = if is_dram(addr) {
            let a = (addr as u64) % self.dram_bytes;
            let v = *self.dram.get(&a).unwrap_or(&0);
            if self.read_ok(addr as u64) {
                v
            } else {
                v ^ 0x0100_0010
            }
        } else {
            match self.status(addr) {
                Some(v) => v,
                None => self.get(addr),
            }
        };
        if self.log_reads && !is_dram(addr) {
            self.evs.push(Ev::R(32, addr as u64, v));
        }
        v
    }
    fn write32(&mut self, addr: usize, val: u32) {
        if is_dram(addr) {
            let a = (addr as u64) % self.dram_bytes;
            self.dram.insert(a, val);
            return;
        }
        let stored = if masked(addr) {
            let m = val >> 16;
            (self.get(addr) & !m) | (val & m & 0xffff)
        } else {
            val
        };
        self.regs.insert(addr, stored);
        self.evs.push(Ev::W(32, addr as u64, val));
        self.side_effects(addr, val);
    }
    fn read16(&mut self, addr: usize) -> u16 {
        self.read32(addr) as u16
    }
    fn write16(&mut self, addr: usize, val: u16) {
        self.write32(addr, val as u32)
    }
    fn udelay(&mut self, us: u32) {
        self.clock += us as u64;
        self.evs.push(Ev::Delay(us));
    }
    fn now_us(&mut self) -> u64 {
        self.clock += 1;
        self.clock
    }
    fn log(&mut self, args: fmt::Arguments) {
        self.text.push_str(&args.to_string());
    }
}
