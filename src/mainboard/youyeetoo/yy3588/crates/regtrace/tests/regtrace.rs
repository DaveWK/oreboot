// SPDX-License-Identifier: GPL-2.0-only
//! regtrace against the A7A harness trace module it came from (verbatim in
//! tests/legacy/a7a_trace.rs) and its own additions.

#[path = "legacy/a7a_trace.rs"]
#[allow(dead_code)]
mod legacy;

use regio::RegIo;
use regtrace::{
    compare, parse, register_diff, render, render_annotated, CompareOpts, Ev, Recorder,
};

fn to_legacy(e: &Ev) -> legacy::Ev {
    match e {
        Ev::R(w, a, v) => legacy::Ev::R(*w, *a as u32, *v),
        Ev::W(w, a, v) => legacy::Ev::W(*w, *a as u32, *v),
        Ev::Delay(d) => legacy::Ev::Delay(*d),
        Ev::Volt(n, a, b) => legacy::Ev::Volt(n.clone(), *a, *b),
        Ev::Print(s) => legacy::Ev::Print(s.clone()),
    }
}

struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u32 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        (self.0 >> 16) as u32
    }
}

/// Random traces with the shapes the A733 runs have: register writes, polls,
/// block loads into DMEM and PState SRAM, delays, PMIC requests, prints.
fn trace(r: &mut Rng) -> Vec<Ev> {
    let mut v = Vec::new();
    let n = 1 + r.next() % 60;
    for _ in 0..n {
        match r.next() % 8 {
            0 => {
                let base = 0x0a9b_0000 + (r.next() % 0x100) as u64 * 2;
                for k in 0..(r.next() % 40) as u64 {
                    v.push(Ev::W(16, base + 2 * k, r.next() & 0xffff));
                }
            }
            1 => {
                let e = Ev::R(32, 0x0a10_0000 + (r.next() % 8) as u64 * 4, r.next() % 3);
                for _ in 0..1 + r.next() % 5 {
                    v.push(e.clone());
                }
            }
            2 => v.push(Ev::W(
                32,
                0x0aa4_0000 + (r.next() % 64) as u64 * 4,
                r.next(),
            )),
            3 => v.push(Ev::R(
                16,
                0x0a9b_0000 + (r.next() % 64) as u64 * 2,
                r.next() & 0xffff,
            )),
            4 => v.push(Ev::Delay(r.next() % 100)),
            5 => v.push(Ev::Volt("dcdc7".into(), 1050, 1)),
            6 => v.push(Ev::Print(format!("step {}", r.next() % 9))),
            _ => v.push(Ev::W(
                32,
                0x0a10_0000 + (r.next() % 16) as u64 * 4,
                r.next(),
            )),
        }
    }
    v
}

fn mutate(t: &[Ev], r: &mut Rng) -> Vec<Ev> {
    let mut v = t.to_vec();
    if v.is_empty() {
        return v;
    }
    let i = (r.next() as usize) % v.len();
    match r.next() % 4 {
        0 => {
            v.remove(i);
        }
        1 => v.insert(i, Ev::W(32, 0x0a10_0040, r.next())),
        2 => {
            if let Ev::W(w, a, x) = v[i] {
                v[i] = Ev::W(w, a, x ^ 1);
            }
        }
        _ => {
            let j = (r.next() as usize) % v.len();
            v.swap(i, j)
        }
    }
    v
}

#[test]
fn render_and_compare_match_the_a7a_harness() {
    let mut r = Rng(0x7ace);
    let a733 = CompareOpts {
        quiet_reads: vec![(0x0a9b_0000, 0x0a9c_0000)],
        unordered: vec![(0x0aa4_0000, 0x0aa4_8000)],
        ..CompareOpts::default()
    };
    for _ in 0..4000 {
        let a = trace(&mut r);
        let b = if r.next().is_multiple_of(3) {
            a.clone()
        } else {
            mutate(&a, &mut r)
        };
        let (la, lb): (Vec<_>, Vec<_>) = (
            a.iter().map(to_legacy).collect(),
            b.iter().map(to_legacy).collect(),
        );
        assert_eq!(render(&a), legacy::render(&la));
        let marks = vec![
            (a.len() / 3, "x".to_string()),
            (a.len() / 2, "y".to_string()),
        ];
        assert_eq!(
            render_annotated(&a, &marks),
            legacy::render_annotated(&la, &marks)
        );
        for reads in [true, false] {
            let o = CompareOpts {
                reads,
                ..a733.clone()
            };
            let lo = legacy::CompareOpts {
                reads,
                ..legacy::CompareOpts::default()
            };
            let new = compare(&a, &b, &o);
            let old = legacy::compare(&la, &lb, &lo).map(|s| {
                s.replace(
                    "final PState SRAM contents differ",
                    "final contents of unordered ranges differ",
                )
            });
            assert_eq!(new, old);
        }
    }
}

#[test]
fn parse_reads_what_display_writes() {
    let mut r = Rng(0x9a55);
    for _ in 0..500 {
        let t: Vec<Ev> = trace(&mut r)
            .into_iter()
            .filter(|e| !matches!(e, Ev::Print(_)))
            .collect();
        let text: String = t.iter().map(|e| format!("{e}\n")).collect();
        assert_eq!(parse(&text).unwrap(), t);
    }
    assert_eq!(
        parse("## step\n\nW32 0a100000 00000001\n").unwrap(),
        vec![Ev::W(32, 0x0a10_0000, 1)]
    );
    assert!(parse("W32 0a100000..0a100040 block of 16 writes, fnv1a 0\n").is_err());
    assert!(parse("R32 0a100000 00000000 x4\n").is_err());
}

#[test]
fn field_masks_and_register_diff_explain_differences() {
    let a = vec![Ev::W(32, 0x100, 0x0000_1234), Ev::W(32, 0x104, 5)];
    let b = vec![Ev::W(32, 0x100, 0xabcd_1234), Ev::W(32, 0x104, 5)];
    assert!(compare(&a, &b, &CompareOpts::default()).is_some());
    let masked = CompareOpts {
        field_masks: vec![(0x100, 0xffff)],
        ..CompareOpts::default()
    };
    assert_eq!(compare(&a, &b, &masked), None);
    assert_eq!(
        register_diff(&a, &b, &CompareOpts::default()),
        vec![(0x100, Some(0x1234), Some(0xabcd_1234))]
    );
    assert!(register_diff(&a, &b, &masked).is_empty());
    let c = vec![Ev::W(32, 0x108, 1)];
    assert_eq!(register_diff(&a, &c, &masked).len(), 3);
}

#[test]
fn recorder_records_regio_accesses() {
    let mut rec = Recorder {
        responder: Some(Box::new(|a, regs| {
            (a == 0x20).then(|| regs.get(&0x10).copied().unwrap_or(0) | 0x80)
        })),
        ..Recorder::default()
    };
    rec.write32(0x10, 3);
    assert_eq!(rec.read32(0x20), 0x83);
    rec.set32(0x10, 4);
    rec.write16(0x30, 0xbeef);
    rec.udelay(5);
    rio_log(&mut rec);
    assert_eq!(
        rec.evs,
        vec![
            Ev::W(32, 0x10, 3),
            Ev::R(32, 0x20, 0x83),
            Ev::R(32, 0x10, 3),
            Ev::W(32, 0x10, 7),
            Ev::W(16, 0x30, 0xbeef),
            Ev::Delay(5),
            Ev::Print("hello 1".into()),
        ]
    );
}

fn rio_log(r: &mut Recorder) {
    r.log(format_args!("hello {}", 1));
}
