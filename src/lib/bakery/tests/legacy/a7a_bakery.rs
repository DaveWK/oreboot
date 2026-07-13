// SPDX-License-Identifier: GPL-2.0-only
// Verbatim copy of ore-edk-boot-radxa-a7a oreboot 4904019
// src/mainboard/radxa/cubie-a7a/common/src/bakery.rs, the oracle for the
// equivalence tests (its own unit tests removed).
#![allow(dead_code, clippy::all)]
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
