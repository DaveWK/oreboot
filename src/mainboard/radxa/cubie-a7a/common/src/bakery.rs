//! Bounded Lamport bakery coordination without exclusive-memory instructions.
//!
//! One participant per physical CPU; aligned single-copy-atomic slots, ordered
//! loads/stores and no concurrent writer to a participant's own slot are required.
//! EL3 supplies uncached accesses/barriers; host controls use sequentially
//! consistent atomics solely to model those access semantics safely.
//! See pinned TF-A lib/locks/bakery/bakery_lock_normal.c for the MMU-off use case.

const CHOOSING: u32 = 1 << 31;
const TICKET: u32 = !CHOOSING;

pub trait Access {
    fn participants(&self) -> usize;
    fn load(&self, participant: usize) -> u32;
    fn store(&self, participant: usize, value: u32);
    fn barrier(&self);
    fn ticks(&self) -> u64;
    fn relax(&self);
}

#[derive(Debug, PartialEq, Eq)]
pub enum Error {
    InvalidParticipant,
    Reentrant,
    TimedOut,
}

pub struct Guard<'a, A: Access> {
    access: &'a A,
    participant: usize,
}
impl<A: Access> Drop for Guard<'_, A> {
    fn drop(&mut self) {
        release(self.access, self.participant);
    }
}
fn release<A: Access>(a: &A, me: usize) {
    a.barrier(); // All protected reads/writes finish before dropping the ticket.
    a.store(me, 0);
    a.barrier();
}

pub fn acquire<A: Access>(a: &A, me: usize, budget: u64) -> Result<Guard<'_, A>, Error> {
    if me >= a.participants() {
        return Err(Error::InvalidParticipant);
    }
    if a.load(me) != 0 {
        return Err(Error::Reentrant);
    }
    let start = a.ticks();
    let expired = || a.ticks().wrapping_sub(start) >= budget;
    loop {
        if expired() {
            return Err(Error::TimedOut);
        }
        a.store(me, CHOOSING);
        a.barrier();
        let mut highest = 0;
        for other in 0..a.participants() {
            highest = highest.max(a.load(other) & TICKET);
        }
        if highest == TICKET {
            // Never wrap an outstanding ticket through zero. Drop our choosing
            // flag and retry after contenders advance, subject to the same bound.
            release(a, me);
            a.relax();
            continue;
        }
        let ticket = highest + 1;
        a.store(me, ticket);
        a.barrier();
        for other in 0..a.participants() {
            if other == me {
                continue;
            }
            loop {
                let value = a.load(other);
                let theirs = value & TICKET;
                if value & CHOOSING == 0 && (theirs == 0 || (theirs, other) > (ticket, me)) {
                    break;
                }
                if expired() {
                    release(a, me);
                    return Err(Error::TimedOut);
                }
                a.relax();
            }
        }
        a.barrier();
        return Ok(Guard {
            access: a,
            participant: me,
        });
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use core::sync::atomic::{AtomicU32, AtomicU64, Ordering::SeqCst};
    use std::{sync::Arc, thread, vec::Vec};
    struct Memory {
        slots: [AtomicU32; 8],
        clock: AtomicU64,
    }
    impl Memory {
        fn new() -> Self {
            Self {
                slots: core::array::from_fn(|_| AtomicU32::new(0)),
                clock: AtomicU64::new(0),
            }
        }
    }
    impl Access for Memory {
        fn participants(&self) -> usize {
            self.slots.len()
        }
        fn load(&self, i: usize) -> u32 {
            self.slots[i].load(SeqCst)
        }
        fn store(&self, i: usize, v: u32) {
            self.slots[i].store(v, SeqCst);
        }
        fn barrier(&self) {
            core::sync::atomic::fence(SeqCst);
        }
        fn ticks(&self) -> u64 {
            self.clock.fetch_add(1, SeqCst)
        }
        fn relax(&self) {
            thread::yield_now();
        }
    }
    #[test]
    fn all_eight_contenders_exclude_and_release() {
        let mem = Arc::new(Memory::new());
        let busy = Arc::new(AtomicU32::new(0));
        let count = Arc::new(AtomicU32::new(0));
        let threads: Vec<_> = (0..8)
            .map(|id| {
                let (m, b, n) = (mem.clone(), busy.clone(), count.clone());
                thread::spawn(move || {
                    for _ in 0..512 {
                        let _guard = acquire(&*m, id, 100_000_000).unwrap();
                        assert_eq!(b.fetch_add(1, SeqCst), 0, "critical sections overlap");
                        thread::yield_now();
                        n.fetch_add(1, SeqCst);
                        assert_eq!(b.fetch_sub(1, SeqCst), 1);
                    }
                })
            })
            .collect();
        for t in threads {
            t.join().unwrap();
        }
        assert_eq!(count.load(SeqCst), 4096);
        assert!(mem.slots.iter().all(|s| s.load(SeqCst) == 0));
    }
    #[test]
    fn timeout_cleans_ticket_and_choosing_overflow() {
        for blocked in [CHOOSING, 1, TICKET] {
            let m = Memory::new();
            m.store(0, blocked);
            assert!(matches!(acquire(&m, 1, 32), Err(Error::TimedOut)));
            assert_eq!(m.load(1), 0);
            assert_eq!(m.load(0), blocked);
            m.store(0, 0);
            drop(acquire(&m, 1, 32).unwrap());
            assert_eq!(m.load(1), 0);
        }
    }
    #[test]
    fn invalid_and_recursive_acquisition_preserve_owner() {
        let m = Memory::new();
        assert!(matches!(acquire(&m, 8, 32), Err(Error::InvalidParticipant)));
        let guard = acquire(&m, 0, 32).unwrap();
        let before = m.load(0);
        assert!(matches!(acquire(&m, 0, 32), Err(Error::Reentrant)));
        assert_eq!(m.load(0), before);
        drop(guard);
        assert_eq!(m.load(0), 0);
        assert!(matches!(acquire(&m, 0, 0), Err(Error::TimedOut)));
        assert_eq!(m.load(0), 0);
    }
    #[test]
    fn timeout_counter_wraps_safely() {
        let m = Memory::new();
        m.clock.store(u64::MAX - 4, SeqCst);
        m.store(0, 1);
        assert!(matches!(acquire(&m, 1, 16), Err(Error::TimedOut)));
        assert_eq!(m.load(1), 0);
    }
}
