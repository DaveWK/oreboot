// SPDX-License-Identifier: GPL-2.0-only
//! The crate against verbatim copies of the board code it replaces
//! (tests/legacy/): same results over exhaustive or seeded-random inputs.

#[path = "legacy/a733.rs"]
mod a733;
#[path = "legacy/k1_train_eye.rs"]
mod k1;

use dram_train::io::{mock::Mock, RegIo};
use dram_train::{dwc_mr, edge, eye, launch, window};

/// xorshift64*, fixed seed: reproducible "random" inputs without a crate.
struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        self.0.wrapping_mul(0x2545_f491_4f6c_dd1d)
    }
    fn map(&mut self) -> u128 {
        // Mix dense, sparse and run-shaped maps.
        let a = (self.next() as u128) << 64 | self.next() as u128;
        match self.next() % 4 {
            0 => a,
            1 => a & ((self.next() as u128) << 64 | self.next() as u128),
            2 => a | ((self.next() as u128) << 64 | self.next() as u128),
            _ => {
                let s = self.next() % 128;
                let l = self.next() % 129;
                let run = if l == 128 {
                    u128::MAX
                } else {
                    (1u128 << l) - 1
                };
                run.rotate_left(s as u32)
            }
        }
    }
}

#[test]
fn longest_run_and_run_near_match_a733() {
    let mut rng = Rng(0xa733_c0de);
    let mut cases = 0;
    for n in 1..=128u32 {
        for _ in 0..400 {
            let map = rng.map();
            assert_eq!(
                window::longest_run(map, n).as_tuple(),
                a733::coldinit::longest_run(map, n),
                "longest_run({map:#x}, {n})"
            );
            let pref = (rng.next() % 300) as u32;
            assert_eq!(
                window::run_near(map, n, pref).as_tuple(),
                a733::coldinit::run_near(map, n, pref),
                "run_near({map:#x}, {n}, {pref})"
            );
            cases += 1;
        }
        for map in [0u128, u128::MAX, 1, 1 << (n - 1), window::full_mask(n) ^ 1] {
            assert_eq!(
                window::longest_run(map, n).as_tuple(),
                a733::coldinit::longest_run(map, n)
            );
            for pref in [0, n / 2, n - 1, n, 2 * n + 3] {
                assert_eq!(
                    window::run_near(map, n, pref).as_tuple(),
                    a733::coldinit::run_near(map, n, pref)
                );
            }
        }
    }
    // Every 8-bit map exhaustively.
    for n in 1..=8u32 {
        for map in 0..256u128 {
            for pref in 0..n {
                assert_eq!(
                    window::run_near(map, n, pref).as_tuple(),
                    a733::coldinit::run_near(map, n, pref)
                );
            }
            assert_eq!(
                window::longest_run(map, n).as_tuple(),
                a733::coldinit::longest_run(map, n)
            );
        }
    }
    assert!(cases > 50_000);
}

#[test]
fn pass_bits_matches_cbt_pass_bits() {
    let mut rng = Rng(7);
    for _ in 0..100_000 {
        let echo = [0, 1, 2, 3].map(|_| rng.next() as u16);
        let bits = rng.next() as u16;
        assert_eq!(
            window::pass_bits(&echo, &a733::CBT_EXPECT, bits),
            a733::coldinit::cbt_pass_bits(echo, bits)
        );
    }
    // The aligned echo passes every echo bit.
    let aligned = a733::CBT_EXPECT.map(|e| if e { 0xffff } else { 0 });
    assert_eq!(
        window::pass_bits(&aligned, &a733::CBT_EXPECT, a733::CBT_ECHO_BITS),
        a733::CBT_ECHO_BITS
    );
}

/// combine_channel rebuilt from the crate: per lane the window of the
/// intersection nearest the first block's delay, its centre in steps.
fn combine(blocks: &[a733::BlockTrain; 8], ch: usize) -> a733::ChannelCa {
    let (points, step) = (a733::LANE_SWEEP_POINTS, a733::LANE_SWEEP_STEP);
    let set = &blocks[4 * ch..4 * ch + 4];
    let mut out = a733::ChannelCa::EMPTY;
    out.lane_map_agrees = true;
    for lane in 0..7 {
        let bit = set[0].lane_bit[lane];
        if set.iter().any(|b| b.lane_bit[lane] != bit) || bit == 0xff {
            out.lane_map_agrees = false;
        }
        let pref = set[0].lane_delay[lane] as u32 / step as u32;
        let w = window::intersect_near(set.iter().map(|b| b.lane_maps[lane] as u128), points, pref);
        out.runs[lane] = w.as_tuple();
        out.delay[lane] = if w.is_empty() {
            set[0].lane_delay[lane]
        } else {
            w.center(points) as u16 * step
        };
    }
    out
}

#[test]
fn intersect_near_matches_combine_channel() {
    let mut rng = Rng(0xca);
    for _ in 0..20_000 {
        let mut blocks = [a733::BlockTrain {
            lane_bit: [0; 7],
            lane_maps: [0; 7],
            lane_delay: [0; 7],
        }; 8];
        for b in blocks.iter_mut() {
            for l in 0..7 {
                b.lane_bit[l] = if rng.next().is_multiple_of(10) {
                    0xff
                } else {
                    (rng.next() % 3) as u8
                };
                b.lane_maps[l] = rng.map() as u64;
                b.lane_delay[l] = (rng.next() % 512) as u16;
            }
        }
        for ch in 0..2 {
            assert_eq!(combine(&blocks, ch), a733::combine_channel(&blocks, ch));
        }
    }
}

#[test]
fn decide_matches_wl_decide() {
    let mut rng = Rng(0xd1);
    for _ in 0..50_000 {
        let n = (rng.next() % 84) as usize;
        let mut err = [0u16; 84];
        let shape = rng.next() % 3;
        for (i, e) in err.iter_mut().enumerate().take(n) {
            *e = match shape {
                0 => (rng.next() % 200) as u16,
                1 => {
                    let edge = 20 + (rng.next() % 4) as usize;
                    if i < edge {
                        180
                    } else {
                        (rng.next() % 5) as u16
                    }
                }
                _ => {
                    if (i / 7) % 2 == 0 {
                        3
                    } else {
                        150
                    }
                }
            };
        }
        let counter = match rng.next() % 3 {
            0 => None,
            _ => Some((rng.next() % 1400) as u16),
        };
        let new = edge::decide_legacy(&err[..n], 16, counter);
        let old = a733::decide(&err[..n], 16, counter);
        assert_eq!(
            (
                new.threshold,
                new.edges,
                new.n_edges,
                new.delay,
                new.counter_agrees
            ),
            (
                old.threshold,
                old.edges,
                old.n_edges,
                old.delay,
                old.counter_agrees
            )
        );
    }
}

#[test]
fn center_walk_out_worst_match_k1() {
    let mut rng = Rng(0x41);
    for _ in 0..30_000 {
        let n = (rng.next() % 82) as usize;
        let mut w = [None; 82];
        for x in w.iter_mut().take(n) {
            *x = match rng.next() % 5 {
                0 => None,
                1 => Some(0),
                _ => Some((rng.next() % 40) as u8),
            };
        }
        assert_eq!(
            eye::center_legacy(&w[..n]).map(|e| (
                e.lo,
                e.hi,
                e.center,
                e.peak,
                e.peak_width,
                e.level,
                e.center_width
            )),
            k1::center(&w[..n])
                .map(|e| (
                    e.lo,
                    e.hi,
                    e.center,
                    e.peak,
                    e.peak_width,
                    e.level,
                    e.center_width
                ))
                .map_err(|e| match e {
                    k1::EyeError::Empty => eye::EyeError::Empty,
                    k1::EyeError::Closed => eye::EyeError::Closed,
                })
        );
        // walk_out over a simulated eye with a fault at a random point.
        let (lo, hi) = {
            let a = (rng.next() % 82) as usize;
            let b = (rng.next() % 82) as usize;
            (a.min(b), a.max(b))
        };
        let fault = rng.next().is_multiple_of(4);
        let fault_at = (rng.next() % 82) as usize;
        let meas = |i: usize| -> Result<u8, usize> {
            if fault && i == fault_at {
                Err(i)
            } else if (lo..=hi).contains(&i) {
                Ok((1 + (i - lo).min(hi - i)) as u8)
            } else {
                Ok(0)
            }
        };
        let start = (rng.next() % 90) as usize;
        let stop = 1 + (rng.next() % 4) as usize;
        let (mut a, mut b) = ([None; 82], [None; 82]);
        let ra = eye::walk_out(&mut a[..n], start, stop, meas);
        let rb = k1::walk_out(&mut b[..n], start, stop, meas);
        assert_eq!((ra, a), (rb, b));
        let m: [u8; 32] = core::array::from_fn(|_| rng.next() as u8);
        assert_eq!(eye::worst(&m), k1::worst(&m));
    }
    assert_eq!(eye::worst(&[]), k1::worst(&[]));
}

/// A register model of an ACSM-like sequencer that records every access.
#[derive(Default)]
struct Seq {
    trace: Vec<(char, usize, u32)>,
    clock: u64,
    busy_polls: u32,
    left: u32,
    last: u16,
    stop: u16,
    witness_ok: bool,
}
const RUN: usize = 0x100;
const DONE: usize = 0x104;
const LAST: usize = 0x108;
impl RegIo for Seq {
    fn read32(&mut self, a: usize) -> u32 {
        let v = match a {
            DONE => {
                if self.left > 0 {
                    self.left -= 1;
                    0
                } else {
                    1
                }
            }
            LAST => {
                if self.witness_ok {
                    self.stop as u32
                } else {
                    self.last as u32
                }
            }
            _ => 0,
        };
        self.trace.push(('r', a, v));
        v
    }
    fn write32(&mut self, a: usize, v: u32) {
        if a == RUN && v & 1 != 0 {
            self.left = self.busy_polls;
        }
        self.trace.push(('w', a, v));
    }
    fn read16(&mut self, a: usize) -> u16 {
        self.read32(a) as u16
    }
    fn write16(&mut self, a: usize, v: u16) {
        self.write32(a, v as u32)
    }
    fn udelay(&mut self, _: u32) {}
    fn now_us(&mut self) -> u64 {
        self.clock += 7;
        self.trace.push(('t', 0, self.clock as u32));
        self.clock
    }
    fn log(&mut self, _: core::fmt::Arguments) {}
}

/// The polling core of the A733 `WlExec::launch` (wl.rs), verbatim except
/// that the CSR helpers are the model's addresses (stub).
fn legacy_launch(
    hw: &mut Seq,
    stop: u16,
    prior_last: u16,
    control: u16,
    timeout: u64,
) -> (Result<(), u8>, u32, u32) {
    let t0 = hw.now_us();
    hw.write16(RUN, control | 1);
    let mut polls: u32 = 0;
    let result = loop {
        if hw.read16(DONE) & 1 != 0 {
            if polls == 0 {
                let last = hw.read16(LAST);
                if last != stop || prior_last == stop {
                    break Err(1);
                }
            }
            break Ok(());
        }
        polls += 1;
        if hw.now_us().wrapping_sub(t0) > timeout {
            break Err(2);
        }
    };
    let us = hw.now_us().wrapping_sub(t0) as u32;
    hw.write16(RUN, control & !1);
    (result, us, polls)
}

#[test]
fn bounded_launch_matches_wl_launch() {
    let mut rng = Rng(0x1a);
    for _ in 0..20_000 {
        let busy = (rng.next() % 40) as u32;
        let stop = (rng.next() % 300) as u16;
        let prior = if rng.next().is_multiple_of(3) {
            stop
        } else {
            (rng.next() % 300) as u16
        };
        let witness_ok = rng.next().is_multiple_of(2);
        let timeout = rng.next() % 200;
        let control = (rng.next() as u16) & !1;
        let mk = || Seq {
            busy_polls: busy,
            stop,
            last: (stop + 1) % 300,
            witness_ok,
            ..Seq::default()
        };
        let (mut a, mut b) = (mk(), mk());
        let old = legacy_launch(&mut a, stop, prior, control, timeout);
        let new = launch::bounded_launch(
            &mut b,
            timeout,
            |io| io.write16(RUN, control | 1),
            |io| io.read16(DONE) & 1 != 0,
            |io| {
                let last = io.read16(LAST);
                !(last != stop || prior == stop)
            },
            |io| io.write16(RUN, control & !1),
        );
        let new_r = match new.result {
            Ok(()) => Ok(()),
            Err(launch::LaunchError::NoBusyObserved) => Err(1),
            Err(launch::LaunchError::Timeout) => Err(2),
        };
        assert_eq!((new_r, new.us, new.polls), old);
        assert_eq!(a.trace, b.trace, "register access order");
    }
}

/// The A733 controllers as a dwc_mr::Ctrl (the board's values).
const A733: dwc_mr::Ctrl = dwc_mr::Ctrl {
    reg: a733::mr::ctrl::ddrc,
    mrctrl0: a733::mr::ctrl::MRCTRL0,
    mrctrl1: a733::mr::ctrl::MRCTRL1,
    mrstat: a733::mr::ctrl::MRSTAT,
    mrr_data: |ch| (if ch == 0 { 0x0a02_002c } else { 0x0a02_0040 }, 8),
    channels: 2,
    ranks: 2,
    timeout_us: 100_000,
};

fn mr_responder(regs: &mut [(usize, u32)], a: usize) -> Option<u32> {
    // MRSTAT idle, MRCTRL0 done after the write; MRR data per channel.
    if a == a733::mr::ctrl::ddrc(0, 0x090) || a == a733::mr::ctrl::ddrc(1, 0x090) {
        return Some(0);
    }
    if a == a733::mr::ctrl::ddrc(0, 0x080) || a == a733::mr::ctrl::ddrc(1, 0x080) {
        return Some(regs.iter().find(|r| r.0 == a).map_or(0, |r| r.1) & 0x7fff_ffff);
    }
    if a == 0x0a02_002c || a == 0x0a02_0040 {
        return Some(0xa5_5a ^ a as u32);
    }
    None
}

#[test]
fn dwc_mr_matches_a733_mr() {
    for ch in 0..3usize {
        for rank in 0..3u32 {
            for (mr, v) in [
                (0u32, 0u32),
                (14, 0x4d),
                (0xff, 0xff),
                (0x100, 1),
                (1, 0x100),
            ] {
                let mut a: Mock<16> = Mock::default();
                let mut b: Mock<16> = Mock::default();
                a.responder = Some(mr_responder);
                b.responder = Some(mr_responder);
                let old = a733::mr::write_rank(&mut a, ch, rank, mr, v);
                let new = dwc_mr::write_rank(&mut b, &A733, ch, rank, mr, v);
                assert_eq!(
                    old.map_err(|e| format!("{e:?}")),
                    new.map_err(|e| format!("{e:?}"))
                );
                assert_eq!((a.reads, a.writes), (b.reads, b.writes));
                let old = a733::mr::read(&mut a, ch, rank, mr);
                let new = dwc_mr::read(&mut b, &A733, ch, rank, mr);
                assert_eq!(
                    old.map_err(|e| format!("{e:?}")),
                    new.map_err(|e| format!("{e:?}"))
                );
                let old = a733::mr::write(&mut a, ch, mr, v, v ^ 1);
                let new = dwc_mr::write(&mut b, &A733, ch, mr, v, v ^ 1);
                assert_eq!(
                    old.map_err(|e| format!("{e:?}")),
                    new.map_err(|e| format!("{e:?}"))
                );
                assert_eq!(
                    (a.reads, a.writes, a.clock_us),
                    (b.reads, b.writes, b.clock_us)
                );
                for addr in [0x080, 0x084, 0x090] {
                    assert_eq!(
                        a.get(a733::mr::ctrl::ddrc(ch, addr)),
                        b.get(a733::mr::ctrl::ddrc(ch, addr))
                    );
                }
            }
        }
    }
    // A stuck MRSTAT times out the same way.
    let (mut a, mut b): (Mock<16>, Mock<16>) = (Mock::default(), Mock::default());
    a.set(a733::mr::ctrl::ddrc(0, 0x090), 1);
    b.set(a733::mr::ctrl::ddrc(0, 0x090), 1);
    a.tick_us = 1000;
    b.tick_us = 1000;
    assert_eq!(
        format!("{:?}", a733::mr::write_rank(&mut a, 0, 0, 1, 2)),
        format!("{:?}", dwc_mr::write_rank(&mut b, &A733, 0, 0, 1, 2)).replace("MrError::", "")
    );
}
