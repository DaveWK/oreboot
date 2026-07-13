// SPDX-License-Identifier: GPL-2.0-only
//! dwc-ddrctl against verbatim copies of the A733 code it came from
//! (ore-edk-boot-radxa-a7a ed42550, tests/legacy/): identical register write
//! traces (address, value, order) and results.
#![allow(dead_code, unused_imports, clippy::all)]

#[path = "legacy/ctrl.rs"]
mod ctrl;
#[path = "legacy/hw.rs"]
mod hw;
#[path = "legacy/mr.rs"]
mod mr;
#[path = "legacy/para.rs"]
mod para;
#[path = "legacy/post.rs"]
mod post;
#[path = "legacy/timing.rs"]
mod timing;

/// The PHY functions the legacy post.rs calls, on the recorder (message
/// block bytes and the two latency CSRs are plain registers here).
mod phy {
    use crate::hw::Hw;
    pub const DMEM: usize = 0x0a95_0000;
    pub fn dmem_access<H: Hw>(hw: &mut H, on: bool) {
        hw.write16(0x0a9f_ff00, on as u16);
    }
    pub fn mb_byte<H: Hw>(hw: &mut H, off: usize) -> u32 {
        let h = hw.read16(DMEM + (off & !1)) as u32;
        if off & 1 != 0 {
            h >> 8
        } else {
            h & 0xff
        }
    }
    pub fn read_latency_csrs<H: Hw>(hw: &mut H) -> (u32, u32) {
        let a = hw.read16(0x0a9f_0002) as u32;
        let b = hw.read16(0x0a9f_0004) as u32;
        (a, b)
    }
}

use dwc_ddrctl::{addrmap, dfi, regs, Map};
use std::collections::HashMap;

/// A register model that records every access, implements both the legacy
/// `Hw` and `regio::RegIo`, and answers the controller's status polls.
#[derive(Default, Clone)]
struct Rec {
    regs: HashMap<usize, u32>,
    trace: Vec<(char, usize, u32)>,
    clock: u64,
}

const A733: Map = Map {
    main: ctrl::ddrc,
    freq: ctrl::freq,
    addrmap: ctrl::addrmap,
};

impl Rec {
    fn status(&self, addr: usize) -> Option<u32> {
        for ch in 0..2 {
            if addr == ctrl::ddrc(ch, ctrl::SWSTAT) {
                return Some(
                    self.regs
                        .get(&ctrl::ddrc(ch, ctrl::SWCTL))
                        .copied()
                        .unwrap_or(0)
                        & 1,
                );
            }
            if addr == ctrl::ddrc(ch, ctrl::DFISTAT) || addr == ctrl::ddrc(ch, ctrl::STAT) {
                return Some(1);
            }
            if addr == ctrl::ddrc(ch, ctrl::DDRC_B88) {
                return Some(0x3600_0000);
            }
        }
        None
    }
    fn rd(&mut self, addr: usize) -> u32 {
        let v = self
            .status(addr)
            .unwrap_or_else(|| self.regs.get(&addr).copied().unwrap_or(0));
        self.trace.push(('r', addr, v));
        v
    }
    fn wr(&mut self, addr: usize, v: u32) {
        self.trace.push(('w', addr, v));
        self.regs.insert(addr, v);
    }
}

impl hw::Hw for Rec {
    fn read32(&mut self, addr: usize) -> u32 {
        self.rd(addr)
    }
    fn write32(&mut self, addr: usize, val: u32) {
        self.wr(addr, val)
    }
    fn read16(&mut self, addr: usize) -> u16 {
        self.rd(addr) as u16
    }
    fn write16(&mut self, addr: usize, val: u16) {
        self.wr(addr, val as u32)
    }
    fn udelay(&mut self, us: u32) {
        self.trace.push(('d', 0, us));
        self.clock += us as u64;
    }
    fn now_us(&mut self) -> u64 {
        self.clock += 1;
        self.clock
    }
    fn set_voltage(&mut self, _: hw::Rail, _: u32) -> bool {
        true
    }
    fn log(&mut self, _: core::fmt::Arguments) {}
}

impl regio::RegIo for Rec {
    fn read32(&mut self, addr: usize) -> u32 {
        self.rd(addr)
    }
    fn write32(&mut self, addr: usize, val: u32) {
        self.wr(addr, val)
    }
    fn read16(&mut self, addr: usize) -> u16 {
        self.rd(addr) as u16
    }
    fn write16(&mut self, addr: usize, val: u16) {
        self.wr(addr, val as u32)
    }
    fn udelay(&mut self, us: u32) {
        self.trace.push(('d', 0, us));
        self.clock += us as u64;
    }
    fn now_us(&mut self) -> u64 {
        self.clock += 1;
        self.clock
    }
    fn log(&mut self, _: core::fmt::Arguments) {}
}

/// xorshift64*, seeded: reproducible inputs.
struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u32 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        (self.0.wrapping_mul(0x2545_f491_4f6c_dd1d) >> 32) as u32
    }
}

fn para(col: u32, row: u32, bank: u32, bg: u32, para2: u32, tpr13: u32, ty: u32) -> para::Para {
    let mut words = [0u32; 32];
    words[1] = ty;
    words[6] = col | (row << 4) | (bank << 12) | (bg << 14);
    words[7] = para2;
    words[30] = tpr13;
    para::Para::from_sysconfig(&words)
}

/// What a board passes in, from its parameter set.
fn geometry(p: &para::Para) -> addrmap::Geometry {
    addrmap::Geometry {
        col_bits: p.col_bits(),
        row_bits: p.row_bits(),
        bank_bits: p.bank_bits(),
        bg_bits: p.bg_bits(),
        rank_field: p.rank_bits(),
        dual_rank: p.dual_rank(),
        nonbinary: (p.tpr13() >> 16) & 7,
        lpddr5: p.is_lpddr5(),
    }
}

#[test]
fn address_map_matches_a733_for_every_geometry() {
    let mut n = 0;
    for col in 7..=12 {
        for row in 13..=19 {
            for bank in 1..=4 {
                for bg in 0..=3 {
                    for rank_field in [0u32, 1, 2, 7, 8, 15] {
                        for density in 0..=3u32 {
                            for ty in [para::TYPE_LPDDR4, para::TYPE_LPDDR5] {
                                let p = para(
                                    col,
                                    row,
                                    bank,
                                    bg,
                                    rank_field << 12,
                                    (density << 16) | (1 << 6),
                                    ty,
                                );
                                for ch in 0..2 {
                                    let mut a = Rec::default();
                                    let old = ctrl::set_address_map(&mut a, &p, ch);
                                    let mut b = Rec::default();
                                    let new =
                                        addrmap::set_address_map(&mut b, &A733, ch, &geometry(&p));
                                    match (old, new) {
                                        (Ok(()), Ok(())) => assert_eq!(a.trace, b.trace),
                                        (Err(hw::Error::Config(m)), Err(n)) => {
                                            assert_eq!(m, n);
                                            assert!(a.trace.is_empty() && b.trace.is_empty());
                                        }
                                        (o, n) => panic!("{col} {row} {bank} {bg} {rank_field} {density}: {o:?} vs {n:?}"),
                                    }
                                    n += 1;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    assert_eq!(n, 6 * 7 * 4 * 4 * 6 * 4 * 2 * 2);
}

#[test]
fn mstr0_odtmap_and_sw_done_match_a733() {
    let mut r = Rng(0x5eed);
    for _ in 0..2000 {
        let p = para(10, 16, 3, 2, r.next() & 0x1fff, 1 << 6, para::TYPE_LPDDR5);
        let ch = (r.next() & 1) as usize;
        let mut a = Rec::default();
        ctrl::controller_config(&mut a, &p, ch);
        ctrl::controller_odt(&mut a, &p, ch);
        ctrl::sw_done(&mut a, ch).unwrap();
        let mut b = Rec::default();
        let ranks = if p.dual_rank() { 2 } else { 1 };
        regio::RegIo::write32(
            &mut b,
            ctrl::ddrc(ch, regs::MSTR0),
            dwc_ddrctl::mstr0_lpddr5_bl16(ranks, p.para2() & 1 != 0),
        );
        regio::RegIo::write32(
            &mut b,
            ctrl::ddrc(ch, regs::ODTMAP),
            dwc_ddrctl::odtmap(ranks),
        );
        dwc_ddrctl::sw_done(&mut b, &A733, ch, 100_000).unwrap();
        assert_eq!(a.trace, b.trace);
    }
}

#[test]
fn dfi_init_matches_a733() {
    for channels in 0..=3 {
        let mut a = Rec::default();
        let old = post::dfi_init(&mut a, channels);
        let mut b = Rec::default();
        let new = dfi::dfi_init(&mut b, &A733, channels, 2, 100_000, |io| {
            regio::RegIo::set32(io, 0x0701_0250, 1)
        });
        assert_eq!(a.trace, b.trace, "channels {channels}");
        match (old, new) {
            (Ok(()), Ok(())) => {}
            (Err(hw::Error::Config(m)), Err(dfi::DfiError::Config(n))) => assert_eq!(m, n),
            (o, n) => panic!("{o:?} vs {n:?}"),
        }
    }
}

/// The board side of the post-training fold on the crate: the same message
/// block reads and A733 glue as the legacy code, the controller part from
/// dwc_ddrctl.
fn board_timing_after_phy(io: &mut Rec, ch: usize) {
    use crate::hw::Hw;
    let mb_max = |io: &mut Rec, a, b| phy::mb_byte(io, a).max(phy::mb_byte(io, b));
    phy::dmem_access(io, true);
    let cdd = dfi::Cdd {
        rr: mb_max(io, 0x30, 0x31).max(mb_max(io, 0x4b, 0x4c)),
        rr_alt: mb_max(io, 0x26, 0x27).max(mb_max(io, 0x41, 0x42)),
        ww: mb_max(io, 0x29, 0x2a).max(mb_max(io, 0x44, 0x45)),
        ww_alt: mb_max(io, 0x2d, 0x2e).max(mb_max(io, 0x48, 0x49)),
        rw: mb_max(io, 0x28, 0x2b).max(mb_max(io, 0x43, 0x46)),
        wr: mb_max(io, 0x2c, 0x2f).max(mb_max(io, 0x47, 0x4a)),
    };
    phy::dmem_access(io, false);
    io.set32(ctrl::ddrc(ch, ctrl::DDRC_B84), 1);
    hw::wait(io, "a", 100_000, |io| {
        (io.read32(ctrl::ddrc(0, ctrl::DDRC_B88)) >> 25) & 3 == 3
    })
    .unwrap();
    hw::wait(io, "b", 100_000, |io| {
        (io.read32(ctrl::ddrc(0, ctrl::DDRC_B88)) >> 28) & 3 == 3
    })
    .unwrap();
    io.write32(ctrl::ddrc(ch, ctrl::SWCTL), 0);
    hw::wait(io, "c", 100_000, |io| {
        io.read32(ctrl::ddrc(ch, ctrl::SWSTAT)) & 1 == 0
    })
    .unwrap();
    dfi::fold_cdd(io, &A733, ch, &cdd);
    let (ptr_init, rcv) = phy::read_latency_csrs(io);
    let rden = ((rcv >> 1) & 0xf) + (((ptr_init & 0xf) + 1) >> 3) * 4 + 21;
    dfi::set_rddata_en(io, &A733, ch, rden);
    dwc_ddrctl::sw_done(io, &A733, ch, 100_000).unwrap();
    io.clr32(ctrl::ddrc(ch, ctrl::DDRC_B84), 1);
}

#[test]
fn post_training_fold_matches_a733() {
    let mut r = Rng(0xcdd);
    for _ in 0..3000 {
        let ch = (r.next() & 1) as usize;
        let mut start = Rec::default();
        for off in (0x26..0x4e).step_by(2) {
            start.regs.insert(phy::DMEM + off, r.next() & 0xffff);
        }
        for off in [0xd04, 0xd08, 0x08, 0x60, 0x24, 0x584, 0x588] {
            start.regs.insert(ctrl::freq(ch, 0, off), r.next());
        }
        start.regs.insert(0x0a9f_0002, r.next() & 0xffff);
        start.regs.insert(0x0a9f_0004, r.next() & 0xffff);
        let mut a = start.clone();
        post::timing_after_phy(&mut a, ch).unwrap();
        let mut b = start;
        board_timing_after_phy(&mut b, ch);
        assert_eq!(a.trace, b.trace);
        assert_eq!(a.regs, b.regs);
    }
}

#[test]
fn dramset_timing_fields_pack_the_a733_values() {
    // The legacy timing writer's DRAMSET1TMG0/1/2/4 values, unpacked by the
    // crate's field layout and packed again, for every pstate and clock.
    use dwc_ddrctl::timing::tmg::*;
    for mhz in [100u32, 200, 300, 400, 533, 600, 800, 1066] {
        for long in [false, true] {
            let mut words = [0u32; 32];
            words[0] = mhz * 4;
            words[1] = para::TYPE_LPDDR5;
            words[24] = 0x0302_0100; // dividers 1,2,3,4 per pstate
            words[27] = (long as u32) << 31;
            words[30] = 5; // multi-pstate
            let mut p = para::Para::from_sysconfig(&words);
            let mut a = Rec::default();
            timing::set_channel_timing(&mut a, &mut p, 0);
            for ps in 0..4 {
                let get = |off| *a.regs.get(&ctrl::freq(0, ps, off)).unwrap();
                let split = |v: u32, f: [dwc_ddrctl::timing::Field; 4]| {
                    f.map(|x| (x, (v & x.mask()) >> x.shift))
                };
                for (off, f) in [
                    (0x00, [RAS_MIN, RAS_MAX, FAW, WR2PRE]),
                    (0x08, [WR2RD, RD2WR, READ_LATENCY, WRITE_LATENCY]),
                    (0x10, [RP, RRD, CCD, RCD]),
                ] {
                    let v = get(off);
                    assert_eq!(dwc_ddrctl::timing::pack(&split(v, f)), Ok(v));
                }
                let v1 = get(0x04);
                assert_eq!(
                    dwc_ddrctl::timing::pack(&[
                        (RC, v1 & 0xff),
                        (RD2PRE, (v1 >> 8) & 0xff),
                        (XP, (v1 >> 16) & 0xff)
                    ]),
                    Ok(v1)
                );
                assert_eq!(dwc_ddrctl::timing::dramset1tmg(4), 0x10);
            }
        }
    }
}

#[test]
fn pack_refuses_overflow_and_update_keeps_other_bits() {
    use dwc_ddrctl::timing::{pack, update, Field, Overflow};
    let f = Field::new(8, 4);
    assert_eq!(pack(&[(f, 0xf)]), Ok(0xf00));
    assert_eq!(
        pack(&[(f, 0x10)]),
        Err(Overflow {
            field: f,
            value: 0x10
        })
    );
    assert_eq!(update(0xffff_ffff, &[(f, 3)]), Ok(0xffff_f3ff));
    assert_eq!(Field::new(0, 32).mask(), u32::MAX);
}
