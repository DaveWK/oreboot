// SPDX-License-Identifier: GPL-2.0-only
//! Margin scan: a 2-D eye over (delay x Vref) for one rank and one bit,
//! nibble or byte, run after training on the trained settings, restoring
//! them afterwards.
//!
//! Interface shape: the Versal margin-check service (AMD AM019 v1.2
//! `ddrmc_lpddr4_xram___MGCHK_*`: mode read/write, unit nibble/byte/bit,
//! step count, rank, bit select, read/write Vref and the default Vref
//! restored after the check). Names only; the scan itself is ours.

use crate::result::{Axis, Margin, Unit};
use crate::window::{longest_run_linear, Window};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Direction {
    Read,
    Write,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Granularity {
    Bit,
    Nibble,
    Byte,
}

/// A linear range of settings: `start`, `start + step`, ... `count` points.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Range {
    pub start: i32,
    pub step: i32,
    pub count: u32,
}

impl Range {
    pub const fn at(self, i: u32) -> i32 {
        self.start + self.step * i as i32
    }
}

/// What to scan.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Request {
    pub dir: Direction,
    pub unit: Granularity,
    pub ch: u8,
    pub rank: u8,
    /// Bit, nibble or byte index.
    pub select: u8,
    /// Delay axis (at most 128 points).
    pub delay: Range,
    /// Vref axis (rows; at most the caller's row buffer).
    pub vref: Range,
    /// Index of the trained delay on the delay axis: [`scan`] walks out from
    /// it to each edge.
    pub trained: u32,
    /// Coarse step of [`scan`]'s edge search (10 in PG353; 1: every point).
    pub coarse: u32,
}

/// The board side of a scan.
pub trait Target {
    type Error;
    fn set_vref(&mut self, req: &Request, vref: i32) -> Result<(), Self::Error>;
    fn set_delay(&mut self, req: &Request, delay: i32) -> Result<(), Self::Error>;
    /// Run the test pattern at the current settings.
    fn test(&mut self, req: &Request) -> Result<bool, Self::Error>;
    /// Put back the trained delay and Vref.
    fn restore(&mut self, req: &Request) -> Result<(), Self::Error>;
    /// Apply the scan options the test depends on (pattern mode, bit
    /// mask) before the scan; the default ignores them.
    fn configure(&mut self, _req: &Request, _opts: &Options) -> Result<(), Self::Error> {
        Ok(())
    }
}

/// The test pattern of a margin check (AM019 `MGCHK_MODE`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Pattern {
    #[default]
    Simple,
    /// The victim/aggressor pattern ([`crate::complex`]).
    Complex,
}

/// Options of [`scan_with_options`] (AM019 `MGCHK_*`: mode, step count,
/// bit select, the Vref of the check).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Options {
    pub pattern: Pattern,
    /// Scan at most this many delay points each side of the trained one
    /// (`MGCHK_STEP`); None: the whole delay range.
    pub steps: Option<u32>,
    /// Bits of the nibble or byte that count (`MGCHK_BITSELECT`); None:
    /// all. The board applies it in [`Target::configure`].
    pub bits: Option<u16>,
    /// Check at this one Vref instead of the request's Vref axis.
    pub vref: Option<i32>,
    /// Test every point instead of searching edges from the trained one.
    pub full: bool,
    /// After restoring, test the trained settings once more and report it.
    pub verify_restore: bool,
}

/// How a margin check ended (AM019 `MGCHK_CTRL` done/abort/error).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CheckStatus {
    Done,
    /// `abort()` returned true; the rows before it are valid.
    Aborted,
}

/// What [`scan_with_options`] returns.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CheckReport {
    pub scan: Scan,
    pub status: CheckStatus,
    /// Rows completed.
    pub rows_done: u32,
    /// Points tested.
    pub points: u32,
    /// The trained settings passed after the restore (with
    /// `verify_restore`).
    pub restored_passes: Option<bool>,
}

/// The scan outcome: per Vref row the longest passing delay run, and the
/// row with the widest run.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Scan {
    pub rows: u32,
    pub best_row: Option<u32>,
    pub best: Window,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ScanError<E> {
    Target(E),
    /// More than 128 delay points or more rows than the buffer holds.
    Size,
}

/// Scan `req` into `maps` (one pass map per Vref row, bit i = delay point
/// i), then restore the trained settings (also after an error in the scan).
/// Default since v1.0.0: on each row the two edges are found from the
/// trained delay outwards, coarse then fine ([`crate::edge::coarse_fine`]),
/// and the row's map is that window; a row failing at the trained delay is
/// scanned in full. [`scan_full`] tests every point.
pub fn scan<T: Target>(
    t: &mut T,
    req: &Request,
    maps: &mut [u128],
) -> Result<Scan, ScanError<T::Error>> {
    scan_with_options(t, req, &Options::default(), maps, || false).map(|r| r.scan)
}

/// Scan every (delay, Vref) point.
pub fn scan_full<T: Target>(
    t: &mut T,
    req: &Request,
    maps: &mut [u128],
) -> Result<Scan, ScanError<T::Error>> {
    let opts = Options {
        full: true,
        ..Options::default()
    };
    scan_with_options(t, req, &opts, maps, || false).map(|r| r.scan)
}

struct Ctx<'a, T: Target, A: FnMut() -> bool> {
    t: &'a mut T,
    req: &'a Request,
    abort: A,
    aborted: bool,
    points: u32,
    lo: u32,
    hi: u32,
}

/// Abort from inside a probe: the probe reports a fail, the row is
/// dropped and the scan stops.
#[derive(Debug)]
enum Stop<E> {
    Target(E),
    Abort,
}

impl<T: Target, A: FnMut() -> bool> Ctx<'_, T, A> {
    fn probe(&mut self, i: u32) -> Result<bool, Stop<T::Error>> {
        if (self.abort)() {
            self.aborted = true;
            return Err(Stop::Abort);
        }
        self.points += 1;
        self.t
            .set_delay(self.req, self.req.delay.at(i))
            .map_err(Stop::Target)?;
        self.t.test(self.req).map_err(Stop::Target)
    }

    fn full_row(&mut self) -> Result<u128, Stop<T::Error>> {
        let mut m = 0u128;
        for d in self.lo..=self.hi {
            if self.probe(d)? {
                m |= 1 << d;
            }
        }
        Ok(m)
    }

    fn edge_row(&mut self) -> Result<u128, Stop<T::Error>> {
        let start = self.req.trained.clamp(self.lo, self.hi);
        let coarse = self.req.coarse;
        let (lo, hi) = (self.lo, self.hi);
        let up = crate::edge::coarse_fine(start, hi, coarse, 0, &mut |i| self.probe(i))?;
        if !up.start_pass {
            return self.full_row();
        }
        let down = crate::edge::coarse_fine(start, lo, coarse, 0, &mut |i| self.probe(i))?;
        let top = up.edge.map_or(hi, |e| e - 1);
        let bottom = down.edge.map_or(lo, |e| e + 1);
        let mut m = 0u128;
        for i in bottom..=top {
            m |= 1 << i;
        }
        Ok(m)
    }
}

/// The margin check with options (see [`Options`]): `abort()` is polled
/// before every point; on an abort the completed rows are kept and the
/// status says so. The trained settings are restored in every case.
pub fn scan_with_options<T: Target>(
    t: &mut T,
    req: &Request,
    opts: &Options,
    maps: &mut [u128],
    abort: impl FnMut() -> bool,
) -> Result<CheckReport, ScanError<T::Error>> {
    let rows = if opts.vref.is_some() {
        1
    } else {
        req.vref.count
    };
    if req.delay.count > 128 || rows as usize > maps.len() {
        return Err(ScanError::Size);
    }
    let n = req.delay.count;
    let trained = req.trained.min(n.saturating_sub(1));
    let (lo, hi) = match opts.steps {
        Some(k) => (
            trained.saturating_sub(k),
            trained.saturating_add(k).min(n.saturating_sub(1)),
        ),
        None => (0, n.saturating_sub(1)),
    };
    let mut c = Ctx {
        t,
        req,
        abort,
        aborted: false,
        points: 0,
        lo,
        hi,
    };
    let mut rows_done = 0;
    let res = (|| -> Result<(), Stop<T::Error>> {
        c.t.configure(req, opts).map_err(Stop::Target)?;
        if n == 0 {
            return Ok(());
        }
        for r in 0..rows {
            let v = opts.vref.unwrap_or_else(|| req.vref.at(r));
            c.t.set_vref(req, v).map_err(Stop::Target)?;
            maps[r as usize] = if opts.full {
                c.full_row()?
            } else {
                c.edge_row()?
            };
            rows_done += 1;
        }
        Ok(())
    })();
    let restored = c.t.restore(req);
    let restored_passes = if opts.verify_restore && restored.is_ok() {
        Some(c.t.test(req).map_err(ScanError::Target)?)
    } else {
        None
    };
    match res {
        Err(Stop::Target(e)) => return Err(ScanError::Target(e)),
        Err(Stop::Abort) | Ok(()) => {}
    }
    restored.map_err(ScanError::Target)?;
    let mut best_row = None;
    let mut best = Window::EMPTY;
    for r in 0..rows_done {
        let w = longest_run_linear(maps[r as usize], n);
        if w.len > best.len {
            best = w;
            best_row = Some(r);
        }
    }
    Ok(CheckReport {
        scan: Scan {
            rows: rows_done,
            best_row,
            best,
        },
        status: if c.aborted {
            CheckStatus::Aborted
        } else {
            CheckStatus::Done
        },
        rows_done,
        points: c.points,
        restored_passes,
    })
}

/// The delay margin of row `row` as an acceptance-record line (`unit` =
/// the delay range's unit), None for an empty row.
pub fn row_margin(
    req: &Request,
    maps: &[u128],
    row: u32,
    axis: Axis,
    unit: Unit,
) -> Option<Margin> {
    let w = longest_run_linear(*maps.get(row as usize)?, req.delay.count);
    if w.is_empty() {
        return None;
    }
    let lo = req.delay.at(w.start);
    let hi = req.delay.at(w.start + w.len - 1);
    Some(Margin {
        axis,
        ch: req.ch,
        rank: req.rank,
        lane: req.select,
        unit,
        lo,
        hi,
        center: req.delay.at(w.center(u32::MAX)),
    })
}
