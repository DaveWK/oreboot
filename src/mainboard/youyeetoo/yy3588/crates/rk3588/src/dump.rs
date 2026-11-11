// SPDX-License-Identifier: GPL-2.0-only
//! dram-info: a parseable report of the DRAM state after either DDR init
//! (the rkbin DDR init of the stepping-stone image, or bt0's own).
//!
//! Text (UART), one record per line, `dram-info <kind> key=value ...`:
//! - `osreg`: OS_REG2..5 raw and decoded (crate::osreg);
//! - `mr`: MRR of MR0, MR5, MR6, MR7, MR8, MR12, MR14 per channel and rank;
//! - `ctl`: DDRCTL MSTR0, ADDRMAP0..12, DRAMSET1TMG0..14, DFITMG0/1,
//!   RFSHSET1TMG0/1, INITMR0..3, per channel;
//! - `phy`: DDRPHY words 0x0000..0x0200 and 0x0A00..0x0C00 per channel
//!   (training results: VWM/deskew/CAL status, MDLL lock value, ZQ).
//!
//! Binary record (the same values) at [`RECORD_ADDR`]: header {u32 magic
//! "RKDI" = 0x49444B52, u32 version 1, u32 entry count, u32 reserved},
//! then entries {u32 key, u32 value}. Keys are register addresses, except
//! MRR results: key = 0x4D52_0000 | ch << 12 | rank << 8 | MA.
//! EDK2 and Linux can read the record until they reuse the page (it is not
//! reserved in their memory maps; see the board README).
//!
//! LICENCE GUARD: values captured here from the rkbin DDR init's register
//! programming (timings, DFI settings, training results) are for
//! comparison and debugging only and must never be copied into bt0's own
//! init (crate::ddr). Identification data (MR5/6/7/8 from the DRAM, the
//! documented OS_REG handoff) may select configurations.

use crate::ddr::mr;
use crate::osreg;
use crate::regs::{base, ctl, OS_REG2, OS_REG3, OS_REG4, OS_REG5};
use core::fmt::{self, Write};
use regio::RegIo;

pub const RECORD_ADDR: usize = 0x001F_0000;
pub const RECORD_BYTES: usize = 0x1_0000;
pub const MAGIC: u32 = 0x4944_4B52;
pub const MRS: [u8; 7] = [0, 5, 6, 7, 8, 12, 14];

pub const fn mr_key(ch: usize, rank: u8, ma: u8) -> u32 {
    0x4D52_0000 | ((ch as u32) << 12) | ((rank as u32) << 8) | ma as u32
}

/// The DDRCTL registers reported, as offsets from a channel base.
pub fn ctl_regs() -> impl Iterator<Item = usize> {
    let main = [
        ctl::MSTR0,
        ctl::STAT,
        ctl::DFIMISC,
        ctl::INITTMG0,
        ctl::INITTMG1,
    ]
    .into_iter();
    let addrmap = (0..=12).map(|n| ctl::ADDRMAP_BASE + 4 * n);
    let tmg = (0..=14).map(ctl::dramset1tmg);
    let rest = [
        ctl::INITMR0,
        ctl::INITMR1,
        ctl::INITMR2,
        ctl::INITMR3,
        ctl::DFITMG0,
        ctl::DFITMG1,
        ctl::RFSHSET1TMG0,
        ctl::RFSHSET1TMG1,
    ]
    .into_iter();
    main.chain(addrmap).chain(tmg).chain(rest)
}

/// The DDRPHY word ranges reported.
pub fn phy_regs() -> impl Iterator<Item = usize> {
    (0x0000..0x0200)
        .step_by(4)
        .chain((0x0A00..0x0C00).step_by(4))
}

/// Binary record writer over a byte buffer.
pub struct Record<'a> {
    buf: &'a mut [u8],
    n: usize,
}

impl<'a> Record<'a> {
    pub fn new(buf: &'a mut [u8]) -> Self {
        let mut r = Record { buf, n: 0 };
        r.header();
        r
    }
    fn header(&mut self) {
        let h = [MAGIC, 1, self.n as u32, 0];
        for (i, w) in h.iter().enumerate() {
            self.buf[4 * i..4 * i + 4].copy_from_slice(&w.to_le_bytes());
        }
    }
    pub fn push(&mut self, key: u32, val: u32) {
        let at = 16 + 8 * self.n;
        if at + 8 > self.buf.len() {
            return;
        }
        self.buf[at..at + 4].copy_from_slice(&key.to_le_bytes());
        self.buf[at + 4..at + 8].copy_from_slice(&val.to_le_bytes());
        self.n += 1;
        self.header();
    }
    pub fn len(&self) -> usize {
        self.n
    }
    pub fn is_empty(&self) -> bool {
        self.n == 0
    }
}

/// Parse a record back (host tools, tests): yields (key, value).
pub fn parse(buf: &[u8]) -> Option<impl Iterator<Item = (u32, u32)> + '_> {
    let w = |i: usize| u32::from_le_bytes(buf[i..i + 4].try_into().unwrap());
    if buf.len() < 16 || w(0) != MAGIC || w(4) != 1 {
        return None;
    }
    let n = w(8) as usize;
    if 16 + 8 * n > buf.len() {
        return None;
    }
    Some((0..n).map(move |k| (w(16 + 8 * k), w(20 + 8 * k))))
}

/// Run the whole report: text to `out`, values into `rec`. `channels`
/// and `ranks` bound the MRR loop (from OS_REG when DRAM came up before).
pub fn report<H: RegIo + ?Sized, W: Write>(
    io: &mut H,
    out: &mut W,
    rec: &mut Record,
    channels: usize,
    ranks: [u8; 4],
) -> fmt::Result {
    let regs = [OS_REG2, OS_REG3, OS_REG4, OS_REG5];
    let v: [u32; 4] = core::array::from_fn(|i| io.read32(regs[i]));
    for (i, &r) in regs.iter().enumerate() {
        rec.push(r as u32, v[i]);
    }
    writeln!(
        out,
        "dram-info osreg raw={:08x},{:08x},{:08x},{:08x}",
        v[0], v[1], v[2], v[3]
    )?;
    for (pair, (a, b)) in [(v[0], v[1]), (v[2], v[3])].into_iter().enumerate() {
        let p = osreg::decode(a, b);
        for line in DisplayLines(&p) {
            if line.n != 0 {
                writeln!(out, "dram-info pair={pair} {line}")?;
            }
        }
    }
    for (ch, &nranks) in ranks.iter().enumerate().take(channels.min(4)) {
        for rank in 0..nranks.clamp(1, 2) {
            write!(out, "dram-info mr ch={ch} rank={rank}")?;
            for ma in MRS {
                match mr::read(io, ch, rank, ma) {
                    Ok(val) => {
                        rec.push(mr_key(ch, rank, ma), val as u32);
                        write!(out, " mr{ma}={val:#04x}")?;
                    }
                    Err(_) => write!(out, " mr{ma}=timeout")?,
                }
            }
            writeln!(out)?;
        }
        let cb = base::DDRCTL[ch];
        write!(out, "dram-info ctl ch={ch}")?;
        for off in ctl_regs() {
            let val = io.read32(cb + off);
            rec.push((cb + off) as u32, val);
            write!(out, " {off:05x}={val:08x}")?;
        }
        writeln!(out)?;
        let pb = base::DDRPHY[ch];
        let mut col = 0;
        for off in phy_regs() {
            if col == 0 {
                write!(out, "dram-info phy ch={ch} off={off:04x}")?;
            }
            let val = io.read32(pb + off);
            rec.push((pb + off) as u32, val);
            write!(out, " {val:08x}")?;
            col += 1;
            if col == 8 {
                writeln!(out)?;
                col = 0;
            }
        }
    }
    writeln!(
        out,
        "dram-info end entries={} record={:#x}",
        rec.len(),
        RECORD_ADDR
    )
}

/// Lines of an osreg pair's Display, without allocation.
struct DisplayLines<'a>(&'a osreg::Pair);
impl<'a> IntoIterator for DisplayLines<'a> {
    type Item = Line;
    type IntoIter = core::array::IntoIter<Line, 3>;
    fn into_iter(self) -> Self::IntoIter {
        let p = self.0;
        let mut lines = [Line::default(), Line::default(), Line::default()];
        let mut w = LineBuf::default();
        let _ = write!(w, "{p}");
        let mut k = 0;
        for part in w.as_str().split('\n') {
            if k < 3 {
                lines[k].set(part);
                k += 1;
            }
        }
        lines.into_iter()
    }
}

#[derive(Clone, Copy)]
pub struct Line {
    b: [u8; 200],
    n: usize,
}
impl Default for Line {
    fn default() -> Self {
        Line { b: [0; 200], n: 0 }
    }
}
impl Line {
    fn set(&mut self, s: &str) {
        let k = s.len().min(200);
        self.b[..k].copy_from_slice(&s.as_bytes()[..k]);
        self.n = k;
    }
}
impl fmt::Display for Line {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.n == 0 {
            return Ok(());
        }
        f.write_str(core::str::from_utf8(&self.b[..self.n]).unwrap_or(""))
    }
}

struct LineBuf {
    b: [u8; 600],
    n: usize,
}
impl Default for LineBuf {
    fn default() -> Self {
        LineBuf { b: [0; 600], n: 0 }
    }
}
impl LineBuf {
    fn as_str(&self) -> &str {
        core::str::from_utf8(&self.b[..self.n]).unwrap_or("")
    }
}
impl Write for LineBuf {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let k = s.len().min(self.b.len() - self.n);
        self.b[self.n..self.n + k].copy_from_slice(&s.as_bytes()[..k]);
        self.n += k;
        Ok(())
    }
}
