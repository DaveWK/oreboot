// SPDX-License-Identifier: GPL-2.0-only
//! Deskew and centring rules.
//!
//! - [`per_bit`]: per-bit deskew that edge-aligns every bit of a byte or
//!   nibble to the latest one. A bit that passes gets its own delay
//!   incremented; a bit that fails from the start pushes the shared strobe;
//!   the shared strobe is never moved backwards. A second pass re-aligns
//!   the bits done before the strobe moved for a later bit.
//! - [`split_difference`]: write centring when delays can only be added:
//!   the left margin is found by delaying DQ, the right margin by delaying
//!   DQS, and the centre is reached by adding delay to whichever side has
//!   the larger margin.
//! - [`centre_from_edges`]: the centre of the data window from whichever of
//!   the four edges around it (the two of this window and the two of the
//!   next one, half a UI on) were found, for when an edge is outside the
//!   delay range.
//!
//! Method: AMD PG150 v1.4 p767-770 (per-bit deskew: never decrement the
//! shared strobe, push it to the latest bit, re-check the early bits; a
//! minimum passing window before an edge counts), PG353 v1.0 p265-266
//! (write DQS-to-DQ centring by left and right margins) and UG586 v4.2
//! Table 60 p132-133 (centre equations by the edges found); cited, not
//! copied. REPORT 5.1 (prior art) has the per-bit alignment use case.

/// The delays a per-bit deskew moves.
pub trait Target {
    type Error;
    /// Set the shared strobe delay (never called with a smaller value than
    /// before within one [`per_bit`] run).
    fn set_strobe(&mut self, delay: u16) -> Result<(), Self::Error>;
    /// Set the delay of data bit `bit`.
    fn set_bit(&mut self, bit: u8, delay: u16) -> Result<(), Self::Error>;
    /// Whether bit `bit` reads the expected data at the current delays.
    fn passes(&mut self, bit: u8) -> Result<bool, Self::Error>;
}

/// Per-bit deskew settings.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Params {
    pub bits: u8,
    /// Strobe delay to start from (normally the lowest).
    pub strobe_start: u16,
    pub strobe_max: u16,
    pub bit_max: u16,
    /// Re-probes that must all fail before a fail counts (noise guard).
    pub confirm: u8,
}

/// Per-bit deskew result.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Deskew<const N: usize> {
    pub strobe: u16,
    pub bit: [u16; N],
    /// Bits that still passed at `bit_max` (earlier than the delay line can
    /// reach): their edge is not aligned.
    pub saturated: u32,
    /// The bit that set the strobe (the latest bit).
    pub latest: Option<u8>,
}

/// Why a per-bit deskew failed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DeskewError<E> {
    Target(E),
    /// More bits than the result holds, or zero bits.
    Config,
    /// The strobe reached `strobe_max` and the bit still fails.
    StrobeRange(u8),
}

struct Run<'a, T: Target> {
    t: &'a mut T,
    confirm: u8,
}
impl<T: Target> Run<'_, T> {
    fn fails(&mut self, bit: u8) -> Result<bool, T::Error> {
        for _ in 0..=self.confirm {
            if self.t.passes(bit)? {
                return Ok(false);
            }
        }
        Ok(true)
    }
    /// Delay `bit` from `d` while it passes; leave it at the last passing
    /// delay. Returns (delay, saturated).
    fn walk_bit(&mut self, bit: u8, mut d: u16, max: u16) -> Result<(u16, bool), T::Error> {
        loop {
            if d == max {
                return Ok((d, true));
            }
            self.t.set_bit(bit, d + 1)?;
            if self.fails(bit)? {
                self.t.set_bit(bit, d)?;
                return Ok((d, false));
            }
            d += 1;
        }
    }
}

/// Per-bit deskew (see the module docs). All bit delays start at 0.
pub fn per_bit<T: Target, const N: usize>(
    t: &mut T,
    p: &Params,
) -> Result<Deskew<N>, DeskewError<T::Error>> {
    let n = usize::from(p.bits);
    if n == 0 || n > N || n > 32 {
        return Err(DeskewError::Config);
    }
    let mut out = Deskew {
        strobe: p.strobe_start,
        bit: [0; N],
        saturated: 0,
        latest: None,
    };
    let confirm = p.confirm;
    let mut r = Run { t, confirm };
    r.t.set_strobe(out.strobe).map_err(DeskewError::Target)?;
    for b in 0..p.bits {
        r.t.set_bit(b, 0).map_err(DeskewError::Target)?;
    }
    for b in 0..p.bits {
        // A failing bit pushes the strobe (later), never back.
        while r.fails(b).map_err(DeskewError::Target)? {
            if out.strobe >= p.strobe_max {
                return Err(DeskewError::StrobeRange(b));
            }
            out.strobe += 1;
            out.latest = Some(b);
            r.t.set_strobe(out.strobe).map_err(DeskewError::Target)?;
        }
        // A passing bit is delayed up to the strobe.
        let (d, sat) = r
            .walk_bit(b, out.bit[b as usize], p.bit_max)
            .map_err(DeskewError::Target)?;
        out.bit[b as usize] = d;
        if sat {
            out.saturated |= 1 << b;
        }
    }
    // Second pass: bits done before the strobe moved for a later bit have
    // slack now; delay them up to it.
    for b in 0..p.bits {
        let (d, sat) = r
            .walk_bit(b, out.bit[b as usize], p.bit_max)
            .map_err(DeskewError::Target)?;
        out.bit[b as usize] = d;
        out.saturated = (out.saturated & !(1 << b)) | (u32::from(sat) << b);
    }
    Ok(out)
}

/// What to add to reach the centre of a write window (delays can only be
/// added): `dqs` to the strobe, `dq` to every data bit (and DBI).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Split {
    pub dqs: u32,
    pub dq: u32,
}

/// Write centring by margins (PG353 p266): `left` taps of margin found by
/// delaying DQ, `right` taps found by delaying DQS, both from the same
/// start. The window centre is (right - left) / 2 taps of strobe delay from
/// the start: a larger right margin adds strobe delay, a larger left margin
/// adds data delay instead. (PG353 writes the strobe case from the
/// right-edge position: right edge - (left + right) / 2, the same point.)
pub fn split_difference(left: u32, right: u32) -> Split {
    if right >= left {
        Split {
            dqs: (right - left) / 2,
            dq: 0,
        }
    } else {
        Split {
            dqs: 0,
            dq: (left - right) / 2,
        }
    }
}

/// The edges around a data window, in strobe delay taps: `rise_left`,
/// `rise_right` of this window, `fall_left`, `fall_right` of the next one
/// (half a UI later). UG586 names them f2z, z2f, f2o, o2f.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Edges {
    pub rise_left: Option<i32>,
    pub rise_right: Option<i32>,
    pub fall_left: Option<i32>,
    pub fall_right: Option<i32>,
}

/// Which edges a centre came from.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CentreFrom {
    /// Both edges of this window.
    Window,
    /// This window's right edge and the next one's left edge: the noise
    /// between them, minus a quarter UI.
    Boundary,
    /// Both edges of the next window, minus half a UI.
    NextWindow,
    /// One edge and a delay limit.
    EdgeAndLimit,
}

/// The centre of this window from the edges found (UG586 Table 60, in
/// UI terms: `ui` taps per unit interval, 180 degrees; the half UI is 90
/// degrees). Preference: both edges of the window; the window/next
/// boundary; both edges of the next window; one edge and the delay limit
/// on its open side (`lo`, `hi`). None if no edge was found.
pub fn centre_from_edges(e: Edges, ui: i32, lo: i32, hi: i32) -> Option<(i32, CentreFrom)> {
    let mid = |a: i32, b: i32| a + (b - a) / 2;
    Some(match e {
        Edges {
            rise_left: Some(l),
            rise_right: Some(r),
            ..
        } => (mid(l, r), CentreFrom::Window),
        Edges {
            rise_right: Some(r),
            fall_left: Some(f),
            ..
        } => (mid(r, f) - ui / 2, CentreFrom::Boundary),
        Edges {
            fall_left: Some(l),
            fall_right: Some(r),
            ..
        } => (mid(l, r) - ui, CentreFrom::NextWindow),
        Edges {
            rise_left: Some(l), ..
        } => (mid(l, hi), CentreFrom::EdgeAndLimit),
        Edges {
            rise_right: Some(r),
            ..
        } => (mid(lo, r), CentreFrom::EdgeAndLimit),
        Edges {
            fall_left: Some(l), ..
        } => (mid(l, hi) - ui, CentreFrom::EdgeAndLimit),
        // Not in Table 60: only the next window's right edge (ours).
        Edges {
            fall_right: Some(r),
            ..
        } => (mid(lo, r) - ui, CentreFrom::EdgeAndLimit),
        _ => return None,
    })
}
