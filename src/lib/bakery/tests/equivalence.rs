// SPDX-License-Identifier: GPL-2.0-only
//! The crate against the verbatim A7A board copy: a scripted, deterministic
//! Access (peers' slots change pseudo-randomly as time advances) records every
//! call; acquire must make the same calls and give the same result.
#[path = "legacy/a7a_bakery.rs"]
mod legacy;

use std::cell::RefCell;

struct Script {
    n: usize,
    slots: RefCell<Vec<u32>>,
    t: RefCell<u64>,
    rng: RefCell<u64>,
    log: RefCell<Vec<String>>,
}
impl Script {
    fn new(n: usize, seed: u64) -> Self {
        let mut slots = vec![0u32; n];
        let mut x = seed | 1;
        for s in slots.iter_mut() {
            x ^= x << 13;
            x ^= x >> 7;
            x ^= x << 17;
            *s = match x % 4 {
                0 => 0,
                1 => 1 << 31,
                2 => (x >> 8) as u32 & 0x7,
                _ => 0x7fff_fffe + (x >> 9) as u32 % 2,
            };
        }
        Self {
            n,
            slots: RefCell::new(slots),
            t: RefCell::new(0),
            rng: RefCell::new(x),
            log: RefCell::new(Vec::new()),
        }
    }
    fn step(&self) {
        let mut x = self.rng.borrow_mut();
        *x ^= *x << 13;
        *x ^= *x >> 7;
        *x ^= *x << 17;
        let i = (*x as usize) % self.n;
        // Peers (not slot 0, ours) release or take a small ticket.
        if i != 0 {
            self.slots.borrow_mut()[i] = if *x & 0x100 != 0 {
                0
            } else {
                (*x >> 20) as u32 & 3
            };
        }
    }
    fn load(&self, i: usize) -> u32 {
        let v = self.slots.borrow().get(i).copied().unwrap_or(0xdead);
        self.log.borrow_mut().push(format!("load {i} {v:x}"));
        v
    }
    fn store(&self, i: usize, v: u32) {
        self.log.borrow_mut().push(format!("store {i} {v:x}"));
        if i < self.n {
            self.slots.borrow_mut()[i] = v;
        }
    }
    fn barrier(&self) {
        self.log.borrow_mut().push("barrier".into());
    }
    fn ticks(&self) -> u64 {
        *self.t.borrow_mut() += 1;
        self.step();
        *self.t.borrow()
    }
    fn relax(&self) {
        self.log.borrow_mut().push("relax".into());
        self.step();
    }
}
struct New(Script);
struct Old(Script);
impl bakery::Access for New {
    fn participants(&self) -> usize {
        self.0.n
    }
    fn load(&self, i: usize) -> u32 {
        self.0.load(i)
    }
    fn store(&self, i: usize, v: u32) {
        self.0.store(i, v)
    }
    fn barrier(&self) {
        self.0.barrier()
    }
    fn ticks(&self) -> u64 {
        self.0.ticks()
    }
    fn relax(&self) {
        self.0.relax()
    }
}
impl legacy::Access for Old {
    fn participants(&self) -> usize {
        self.0.n
    }
    fn load(&self, i: usize) -> u32 {
        self.0.load(i)
    }
    fn store(&self, i: usize, v: u32) {
        self.0.store(i, v)
    }
    fn barrier(&self) {
        self.0.barrier()
    }
    fn ticks(&self) -> u64 {
        self.0.ticks()
    }
    fn relax(&self) {
        self.0.relax()
    }
}

#[test]
fn scripted_runs_match_the_board_code() {
    let mut got = 0;
    for seed in 0..20_000u64 {
        let n = 1 + (seed % 8) as usize;
        let me = if seed % 37 == 0 { n } else { 0 };
        let budget = [0, 1, 5, 40, 400][(seed / 8 % 5) as usize];
        let a = New(Script::new(n, seed * 0x9e37_79b9 + 1));
        let b = Old(Script::new(n, seed * 0x9e37_79b9 + 1));
        let ra = bakery::acquire(&a, me, budget)
            .map(|_| ())
            .map_err(|e| format!("{e:?}"));
        let rb = legacy::acquire(&b, me, budget)
            .map(|_| ())
            .map_err(|e| format!("{e:?}"));
        assert_eq!(ra, rb, "seed {seed}");
        assert_eq!(*a.0.log.borrow(), *b.0.log.borrow(), "seed {seed}");
        got += ra.is_ok() as u32;
    }
    assert!(got > 1000, "only {got} acquisitions");
}
