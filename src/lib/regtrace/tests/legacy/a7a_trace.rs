// Verbatim copy of ore-edk-boot-radxa-a7a ed42550 tools/dram-harness/src/trace.rs
// (equivalence oracle for crates/regtrace).
//! The register-level trace of a DRAM init run, and its comparison.

use std::fmt;

/// One observable action of the DRAM init code.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Ev {
    /// MMIO or DRAM read: width in bits, address, value returned by the model.
    R(u8, u32, u32),
    /// MMIO or DRAM write.
    W(u8, u32, u32),
    /// Busy-wait of this many microseconds.
    Delay(u32),
    /// PMIC request: rail name, millivolts, on/off.
    Volt(String, i32, i32),
    /// Log text printed by the code (not compared).
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
/// ascending addresses (the PHY IMEM/DMEM loads, PIE images) and repeated
/// identical reads (polls) are folded into one line each.
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
                let step = *w as u32 / 8;
                let mut j = i + 1;
                while j < limit {
                    match &evs[j] {
                        Ev::W(w2, a2, _) if w2 == w && *a2 == a + step * (j - i) as u32 => j += 1,
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
                        a + step * (j - i) as u32,
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

/// How two traces are compared.
#[derive(Clone, Debug)]
pub struct CompareOpts {
    /// Fold runs of identical reads (poll loops), so a poll that needs a
    /// different number of iterations still matches.
    pub fold_polls: bool,
    /// Compare reads at all (writes, delays and PMIC requests always are).
    pub reads: bool,
    /// Address ranges whose reads are not compared: memories without read
    /// side effects (the PHY firmware's DMEM), where the number and order
    /// of reads is the compiler's business.
    pub quiet_reads: &'static [(u32, u32)],
    /// Address ranges whose writes are left out (documented, intended
    /// differences).
    pub skip_writes: Vec<(u32, u32)>,
    /// Plain memories whose final contents are compared instead of the
    /// order of their writes (the PHY's PState SRAM).
    pub unordered: &'static [(u32, u32)],
}

/// PHY PState SRAM (CSRs 0xa0000..0xa4000).
pub const PS_SRAM_RANGE: (u32, u32) = (0x0aa4_0000, 0x0aa4_8000);

/// PHY DMEM (message block and firmware data).
pub const DMEM_RANGE: (u32, u32) = (0x0a9b_0000, 0x0a9c_0000);

impl Default for CompareOpts {
    fn default() -> Self {
        Self {
            fold_polls: true,
            reads: true,
            quiet_reads: &[DMEM_RANGE],
            skip_writes: Vec::new(),
            unordered: &[PS_SRAM_RANGE],
        }
    }
}

fn normalise(evs: &[Ev], o: &CompareOpts) -> Vec<Ev> {
    let within = |a: u32, r: &[(u32, u32)]| r.iter().any(|&(s, e)| (s..e).contains(&a));
    let mut v: Vec<Ev> = Vec::with_capacity(evs.len());
    for e in evs {
        match e {
            Ev::Print(_) => continue,
            Ev::R(..) if !o.reads => continue,
            Ev::R(_, a, _) if within(*a, o.quiet_reads) => continue,
            Ev::W(_, a, _) if within(*a, &o.skip_writes) => continue,
            Ev::W(_, a, _) if within(*a, o.unordered) => continue,
            Ev::R(..) if o.fold_polls && v.last() == Some(e) => continue,
            _ => v.push(e.clone()),
        }
    }
    v
}

/// First difference between two traces after normalisation, with context,
/// or None when they match.
fn final_contents(evs: &[Ev], ranges: &[(u32, u32)]) -> std::collections::BTreeMap<u32, u32> {
    let mut m = std::collections::BTreeMap::new();
    for e in evs {
        if let Ev::W(_, a, v) = e {
            if ranges.iter().any(|&(s, e)| (s..e).contains(a)) {
                m.insert(*a, *v);
            }
        }
    }
    m
}

pub fn compare(a: &[Ev], b: &[Ev], o: &CompareOpts) -> Option<String> {
    let (fa, fb) = (
        final_contents(a, o.unordered),
        final_contents(b, o.unordered),
    );
    if fa != fb {
        let diff: Vec<String> = fa
            .keys()
            .chain(fb.keys())
            .collect::<std::collections::BTreeSet<_>>()
            .into_iter()
            .filter(|k| fa.get(k) != fb.get(k))
            .take(8)
            .map(|k| format!("{k:08x}: {:x?} / {:x?}", fa.get(k), fb.get(k)))
            .collect();
        return Some(format!(
            "final PState SRAM contents differ: {}",
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
