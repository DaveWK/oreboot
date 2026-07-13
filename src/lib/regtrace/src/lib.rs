// SPDX-License-Identifier: GPL-2.0-only
//! Register-level traces for host tests (no_std + alloc so it builds for
//! every target; meant for host tests, never linked into firmware): record
//! every [`regio::RegIo`] access of the code under test, read a reference
//! trace, render traces for reports and compare a generated register
//! program with a reference, register by register and field by field.
//!
//! Use it to prove that code computing a register program from
//! documentation and JEDEC rules writes what a recorded reference wrote,
//! and to explain every difference (masks, skipped ranges).
//!
//! From ore-edk-boot-radxa-a7a ed42550 `tools/dram-harness/src/trace.rs`
//! (Ev, fnv1a, render, render_annotated, CompareOpts, compare), with
//! 64-bit addresses, no SoC address ranges in the defaults (the A733's PHY
//! DMEM and PState SRAM are the board's options), per-address field masks,
//! a [`Recorder`] and a [`parse`]r for the unfolded text form.

#![no_std]
extern crate alloc;

use alloc::boxed::Box;
use alloc::collections::{BTreeMap, BTreeSet};
use alloc::format;
use alloc::string::{String, ToString};
use alloc::vec::Vec;
use core::fmt;

/// One observable action of the code under test.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Ev {
    /// Read: width in bits, address, value returned.
    R(u8, u64, u32),
    /// Write: width in bits, address, value.
    W(u8, u64, u32),
    /// Busy-wait of this many microseconds.
    Delay(u32),
    /// PMIC request: rail name, millivolts, on/off.
    Volt(String, i32, i32),
    /// Log text printed by the code (never compared).
    Print(String),
}

impl fmt::Display for Ev {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Ev::R(w, a, v) => write!(f, "R{w} {a:08x} {v:0width$x}", width = *w as usize / 4),
            Ev::W(w, a, v) => write!(f, "W{w} {a:08x} {v:0width$x}", width = *w as usize / 4),
            Ev::Delay(us) => write!(f, "D {us}"),
            Ev::Volt(n, mv, on) => write!(f, "V {n} {mv} {on}"),
            Ev::Print(s) => write!(f, "P {}", s.escape_default()),
        }
    }
}

/// Parse the unfolded text form ([`Ev`]'s `Display`, one event per line;
/// empty lines and `##`/`#` comment lines are skipped). Folded report lines
/// (`block of N writes`, `xN` polls) are refused: a reference must be exact.
pub fn parse(text: &str) -> Result<Vec<Ev>, String> {
    let mut out = Vec::new();
    for (n, line) in text.lines().enumerate() {
        let l = line.trim_end();
        if l.is_empty() || l.starts_with('#') {
            continue;
        }
        let bad = || format!("line {}: {l:?}", n + 1);
        let mut it = l.split(' ');
        let tag = it.next().unwrap_or("");
        let ev = match tag {
            "D" => Ev::Delay(it.next().ok_or_else(bad)?.parse().map_err(|_| bad())?),
            "P" => Ev::Print(l[2..].to_string()),
            "V" => {
                let name = it.next().ok_or_else(bad)?.to_string();
                let mv = it.next().ok_or_else(bad)?.parse().map_err(|_| bad())?;
                let on = it.next().ok_or_else(bad)?.parse().map_err(|_| bad())?;
                Ev::Volt(name, mv, on)
            }
            _ if tag.starts_with('R') || tag.starts_with('W') => {
                let w: u8 = tag[1..].parse().map_err(|_| bad())?;
                let a = u64::from_str_radix(it.next().ok_or_else(bad)?, 16).map_err(|_| bad())?;
                let v = u32::from_str_radix(it.next().ok_or_else(bad)?, 16).map_err(|_| bad())?;
                if it.next().is_some() || !matches!(w, 8 | 16 | 32) {
                    return Err(bad());
                }
                if tag.starts_with('R') {
                    Ev::R(w, a, v)
                } else {
                    Ev::W(w, a, v)
                }
            }
            _ => return Err(bad()),
        };
        out.push(ev);
    }
    Ok(out)
}

/// FNV-1a over a byte stream: a fingerprint for bulk loads in reports.
pub fn fnv1a(data: impl IntoIterator<Item = u8>) -> u64 {
    let mut h = 0xcbf2_9ce4_8422_2325u64;
    for b in data {
        h ^= b as u64;
        h = h.wrapping_mul(0x0100_0000_01b3);
    }
    h
}

/// Render a trace for a report: consecutive writes of the same width to
/// ascending addresses (firmware or table loads) and repeated identical
/// reads (polls) are folded into one line each.
pub fn render(evs: &[Ev]) -> String {
    render_annotated(evs, &[])
}

/// [`render`] with `## name` lines where the code that made the accesses
/// changes (`marks`: first event index and name, ascending).
pub fn render_annotated(evs: &[Ev], marks: &[(usize, String)]) -> String {
    let mut out = String::new();
    let mut i = 0;
    let mut m = 0;
    while i < evs.len() {
        while m < marks.len() && marks[m].0 <= i {
            out += &format!("## {}\n", marks[m].1);
            m += 1;
        }
        let limit = marks.get(m).map(|x| x.0).unwrap_or(evs.len());
        match &evs[i] {
            Ev::W(w, a, _) => {
                let step = *w as u64 / 8;
                let mut j = i + 1;
                while j < limit {
                    match &evs[j] {
                        Ev::W(w2, a2, _) if w2 == w && *a2 == a + step * (j - i) as u64 => j += 1,
                        _ => break,
                    }
                }
                if j - i >= 16 {
                    let bytes = evs[i..j].iter().flat_map(|e| match e {
                        Ev::W(_, _, v) => v.to_le_bytes()[..step as usize].to_vec(),
                        _ => unreachable!(),
                    });
                    out += &format!(
                        "W{w} {a:08x}..{:08x} block of {} writes, fnv1a {:016x}\n",
                        a + step * (j - i) as u64,
                        j - i,
                        fnv1a(bytes)
                    );
                    i = j;
                    continue;
                }
                out += &format!("{}\n", evs[i]);
            }
            Ev::R(..) => {
                let mut j = i + 1;
                while j < limit && evs[j] == evs[i] {
                    j += 1;
                }
                if j - i > 1 {
                    out += &format!("{} x{}\n", evs[i], j - i);
                } else {
                    out += &format!("{}\n", evs[i]);
                }
                i = j;
                continue;
            }
            e => out += &format!("{e}\n"),
        }
        i += 1;
    }
    out
}

/// How two traces are compared. The default compares everything in order
/// (polls folded); boards add their memories and documented differences.
#[derive(Clone, Debug)]
pub struct CompareOpts {
    /// Fold runs of identical reads (poll loops), so a poll that needs a
    /// different number of iterations still matches.
    pub fold_polls: bool,
    /// Compare reads at all (writes, delays and PMIC requests always are).
    pub reads: bool,
    /// Address ranges `[start, end)` whose reads are not compared (memories
    /// without read side effects).
    pub quiet_reads: Vec<(u64, u64)>,
    /// Address ranges whose writes are left out (documented, intended
    /// differences).
    pub skip_writes: Vec<(u64, u64)>,
    /// Plain memories whose final contents are compared instead of the
    /// order of their writes.
    pub unordered: Vec<(u64, u64)>,
    /// Per address: only these bits of reads and writes are compared
    /// (fields a documented difference leaves out).
    pub field_masks: Vec<(u64, u32)>,
}

impl Default for CompareOpts {
    fn default() -> Self {
        Self {
            fold_polls: true,
            reads: true,
            quiet_reads: Vec::new(),
            skip_writes: Vec::new(),
            unordered: Vec::new(),
            field_masks: Vec::new(),
        }
    }
}

fn within(a: u64, r: &[(u64, u64)]) -> bool {
    r.iter().any(|&(s, e)| (s..e).contains(&a))
}

fn mask(a: u64, v: u32, o: &CompareOpts) -> u32 {
    o.field_masks
        .iter()
        .filter(|(x, _)| *x == a)
        .fold(v, |v, (_, m)| v & m)
}

fn normalise(evs: &[Ev], o: &CompareOpts) -> Vec<Ev> {
    let mut v: Vec<Ev> = Vec::with_capacity(evs.len());
    for e in evs {
        let e = match e {
            Ev::R(w, a, x) => Ev::R(*w, *a, mask(*a, *x, o)),
            Ev::W(w, a, x) => Ev::W(*w, *a, mask(*a, *x, o)),
            e => e.clone(),
        };
        match &e {
            Ev::Print(_) => continue,
            Ev::R(..) if !o.reads => continue,
            Ev::R(_, a, _) if within(*a, &o.quiet_reads) => continue,
            Ev::W(_, a, _) if within(*a, &o.skip_writes) => continue,
            Ev::W(_, a, _) if within(*a, &o.unordered) => continue,
            Ev::R(..) if o.fold_polls && v.last() == Some(&e) => continue,
            _ => v.push(e),
        }
    }
    v
}

/// Final value of every written address in `ranges`.
pub fn final_contents(evs: &[Ev], ranges: &[(u64, u64)]) -> BTreeMap<u64, u32> {
    let mut m = BTreeMap::new();
    for e in evs {
        if let Ev::W(_, a, v) = e {
            if within(*a, ranges) {
                m.insert(*a, *v);
            }
        }
    }
    m
}

/// First difference between a reference `a` and a candidate `b` after
/// normalisation, with context, or None when they match.
pub fn compare(a: &[Ev], b: &[Ev], o: &CompareOpts) -> Option<String> {
    let (fa, fb) = (
        final_contents(a, &o.unordered),
        final_contents(b, &o.unordered),
    );
    if fa != fb {
        let diff: Vec<String> = fa
            .keys()
            .chain(fb.keys())
            .collect::<BTreeSet<_>>()
            .into_iter()
            .filter(|k| fa.get(k) != fb.get(k))
            .take(8)
            .map(|k| format!("{k:08x}: {:x?} / {:x?}", fa.get(k), fb.get(k)))
            .collect();
        return Some(format!(
            "final contents of unordered ranges differ: {}",
            diff.join(", ")
        ));
    }
    let (na, nb) = (normalise(a, o), normalise(b, o));
    let n = na.iter().zip(&nb).take_while(|(x, y)| x == y).count();
    if n == na.len() && n == nb.len() {
        return None;
    }
    let ctx = |v: &[Ev]| {
        v[n.saturating_sub(5)..(n + 5).min(v.len())]
            .iter()
            .enumerate()
            .map(|(k, e)| {
                let idx = n.saturating_sub(5) + k;
                format!("{}{idx:7} {e}", if idx == n { ">" } else { " " })
            })
            .collect::<Vec<_>>()
            .join("\n")
    };
    Some(format!(
        "traces differ at event {n} (lengths {} / {}):\n--- reference\n{}\n--- candidate\n{}",
        na.len(),
        nb.len(),
        ctx(&na),
        ctx(&nb)
    ))
}

/// Per-register summary of two traces' final written values: registers
/// whose (masked) final value differs, and registers only one side wrote.
pub fn register_diff(a: &[Ev], b: &[Ev], o: &CompareOpts) -> Vec<(u64, Option<u32>, Option<u32>)> {
    let all = |e: &[Ev]| {
        let mut m = BTreeMap::new();
        for x in e {
            if let Ev::W(_, a, v) = x {
                if !within(*a, &o.skip_writes) {
                    m.insert(*a, mask(*a, *v, o));
                }
            }
        }
        m
    };
    let (fa, fb) = (all(a), all(b));
    fa.keys()
        .chain(fb.keys())
        .collect::<BTreeSet<_>>()
        .into_iter()
        .filter(|k| fa.get(k) != fb.get(k))
        .map(|k| (*k, fa.get(k).copied(), fb.get(k).copied()))
        .collect()
}

/// Answers a read of an address from the registers written so far, or None.
pub type Responder = Box<dyn FnMut(u64, &BTreeMap<u64, u32>) -> Option<u32>>;

/// A register model that records every access as [`Ev`]s: reads return the
/// last written value (0 before), unless `responder` answers first; the
/// clock advances `tick_us` per `now_us` call.
pub struct Recorder {
    pub regs: BTreeMap<u64, u32>,
    pub evs: Vec<Ev>,
    pub clock_us: u64,
    pub tick_us: u64,
    pub responder: Option<Responder>,
}

impl Default for Recorder {
    fn default() -> Self {
        Self {
            regs: BTreeMap::new(),
            evs: Vec::new(),
            clock_us: 0,
            tick_us: 1,
            responder: None,
        }
    }
}

impl Recorder {
    fn read(&mut self, w: u8, addr: usize) -> u32 {
        let a = addr as u64;
        let answered = match self.responder.as_mut() {
            Some(f) => f(a, &self.regs),
            None => None,
        };
        let v = answered.unwrap_or_else(|| self.regs.get(&a).copied().unwrap_or(0));
        self.evs.push(Ev::R(w, a, v));
        v
    }
    fn write(&mut self, w: u8, addr: usize, v: u32) {
        let a = addr as u64;
        self.evs.push(Ev::W(w, a, v));
        self.regs.insert(a, v);
    }
}

impl regio::RegIo for Recorder {
    fn read32(&mut self, addr: usize) -> u32 {
        self.read(32, addr)
    }
    fn write32(&mut self, addr: usize, val: u32) {
        self.write(32, addr, val)
    }
    fn read16(&mut self, addr: usize) -> u16 {
        self.read(16, addr) as u16
    }
    fn write16(&mut self, addr: usize, val: u16) {
        self.write(16, addr, val as u32)
    }
    fn udelay(&mut self, us: u32) {
        self.evs.push(Ev::Delay(us));
        self.clock_us += us as u64;
    }
    fn now_us(&mut self) -> u64 {
        self.clock_us += self.tick_us;
        self.clock_us
    }
    fn log(&mut self, args: fmt::Arguments) {
        self.evs.push(Ev::Print(args.to_string()));
    }
}
