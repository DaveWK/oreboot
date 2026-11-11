// SPDX-License-Identifier: GPL-2.0-only
//! P3-M10: the issue/compare/count engine loop on the plain-access engine
//! with fault plants.

use std::cell::RefCell;
use std::collections::HashMap;

use dram_train::engine::{run, Beat, EngineError, Job, MemEngine};

/// Memory with a stuck lane and an intermittent lane (fails on every
/// `every`-th read of one word).
#[derive(Default)]
struct Mem {
    words: HashMap<u64, Beat>,
    stuck: Option<(u32, bool)>,
    flaky: Option<(u64, u32, u32)>, // (addr, lane, every)
    reads: u32,
    writes: u32,
}
impl Mem {
    fn access(&mut self, a: u64, v: Option<Beat>) -> Beat {
        match v {
            Some(v) => {
                self.writes += 1;
                self.words.insert(a, v);
                0
            }
            None => {
                self.reads += 1;
                let mut v = *self.words.get(&a).unwrap_or(&0);
                if let Some((l, s)) = self.stuck {
                    v = v & !(1 << l) | (u64::from(s) << l);
                }
                if let Some((fa, l, every)) = self.flaky {
                    if fa == a && self.reads.is_multiple_of(every) {
                        v ^= 1 << l;
                    }
                }
                v
            }
        }
    }
}

fn job(passes: u32) -> Job {
    Job {
        rank: 0,
        addr: 0x1000,
        passes,
        stop_after: None,
        lanes: !0,
        write_each_pass: true,
    }
}

const PAT: [Beat; 8] = [
    0x5555_5555_5555_5555,
    0xaaaa_aaaa_aaaa_aaaa,
    0,
    !0,
    0x0123_4567_89ab_cdef,
    0xfedc_ba98_7654_3210,
    0x00ff_00ff_00ff_00ff,
    0xff00_ff00_ff00_ff00,
];

#[test]
fn a_good_memory_passes_every_loop() {
    let m = RefCell::new(Mem::default());
    let mut e = MemEngine::<_, 16>::new(|a, v| m.borrow_mut().access(a, v));
    let c = run(&mut e, &job(10), &PAT, &mut []).unwrap();
    assert_eq!((c.passes, c.fails, c.lanes, c.first_fail), (10, 0, 0, None));
    assert_eq!(m.borrow().writes, 80);
}

#[test]
fn a_stuck_lane_is_counted_and_the_first_fail_captured() {
    let m = RefCell::new(Mem {
        stuck: Some((37, true)),
        ..Mem::default()
    });
    let mut e = MemEngine::<_, 16>::new(|a, v| m.borrow_mut().access(a, v));
    let mut cap = [0u64; 8];
    let c = run(&mut e, &job(5), &PAT, &mut cap).unwrap();
    assert_eq!((c.fails, c.lanes, c.first_fail), (5, 1 << 37, Some(0)));
    assert_eq!(c.lane_fails[37], 5);
    assert_eq!(c.captured, 8);
    assert_eq!(cap[2], 1 << 37, "the zero beat read back with lane 37 set");
}

#[test]
fn stop_on_the_nth_fail_and_lane_masks() {
    let m = RefCell::new(Mem {
        flaky: Some((0x1010, 3, 7)),
        ..Mem::default()
    });
    let mut e = MemEngine::<_, 16>::new(|a, v| m.borrow_mut().access(a, v));
    let mut j = job(1000);
    j.stop_after = Some(3);
    let c = run(&mut e, &j, &PAT, &mut []).unwrap();
    assert!(c.stopped);
    assert_eq!(c.fails, 3);
    assert!(c.passes < 1000);
    assert_eq!(c.lanes, 1 << 3);
    // The same fault with lane 3 masked out is not counted.
    let m = RefCell::new(Mem {
        flaky: Some((0x1010, 3, 7)),
        ..Mem::default()
    });
    let mut e = MemEngine::<_, 16>::new(|a, v| m.borrow_mut().access(a, v));
    let mut j = job(100);
    j.lanes = !(1 << 3);
    let c = run(&mut e, &j, &PAT, &mut []).unwrap();
    assert_eq!((c.fails, c.passes), (0, 100));
}

#[test]
fn a_read_only_loop_writes_once() {
    let m = RefCell::new(Mem::default());
    let mut e = MemEngine::<_, 16>::new(|a, v| m.borrow_mut().access(a, v));
    let mut j = job(20);
    j.write_each_pass = false;
    run(&mut e, &j, &PAT, &mut []).unwrap();
    assert_eq!((m.borrow().writes, m.borrow().reads), (8, 160));
}

#[test]
fn the_expected_cache_bounds_the_pattern() {
    let m = RefCell::new(Mem::default());
    let mut e = MemEngine::<_, 4>::new(|a, v| m.borrow_mut().access(a, v));
    assert_eq!(
        run(&mut e, &job(1), &PAT, &mut []),
        Err(EngineError::Capacity)
    );
    assert_eq!(
        run(&mut e, &job(1), &[], &mut []),
        Err(EngineError::Capacity)
    );
}
