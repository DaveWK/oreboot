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
//! - [`per_bit_centre`]: per-bit **centre** alignment. Each bit's
//!   own window is measured at both edges on the shared strobe axis, every
//!   bit is delayed so that the window centres line up on the latest one
//!   (the strobe is not moved), and a re-measure hook checks the result:
//!   after the move the windows are measured again and a bit whose centre
//!   is still off by more than the tolerance is corrected once more, for a
//!   bounded number of rounds (the target follows the latest measured
//!   centre, so bits are only ever delayed). Bits the delay line cannot
//!   reach are reported saturated, bits still off after the last round
//!   unverified.
//!
//! Method: AMD PG150 v1.4 p767-770 (per-bit deskew: never decrement the
//! shared strobe, push it to the latest bit, re-check the early bits; a
//! minimum passing window before an edge counts), PG353 v1.0 p265-266
//! (write DQS-to-DQ centring by left and right margins) and UG586 v4.2
//! Table 60 p132-133 (centre equations by the edges found); cited, not
//! copied.

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
/// on its open side (`lo`, `hi`). None if no edge was found or the
/// resulting centre is outside the signed 32-bit tap axis.
pub fn centre_from_edges(e: Edges, ui: i32, lo: i32, hi: i32) -> Option<(i32, CentreFrom)> {
    let mid = |a: i32, b: i32| i64::from(a) + (i64::from(b) - i64::from(a)) / 2;
    let ui = i64::from(ui);
    let (centre, from) = match e {
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
    };
    Some((i32::try_from(centre).ok()?, from))
}

// ---------------------------------------------------------------------------
// Per-bit centre alignment with a re-measure hook.

/// Measuring one bit's pass window at both edges on the shared strobe axis
/// (the board sweeps the strobe, or reads its engine's left and right
/// edges), and setting that bit's own delay.
pub trait Measure {
    type Error;
    /// Both edges of `bit`'s pass window at the current delays, in taps of
    /// the strobe axis (`lo <= hi`); None when the bit has no window.
    /// Delaying a bit by k taps moves its window by +k on this axis.
    fn window(&mut self, bit: u8) -> Result<Option<(i32, i32)>, Self::Error>;
    /// Set the delay of data bit `bit`.
    fn set_bit(&mut self, bit: u8, delay: u16) -> Result<(), Self::Error>;
}

/// Per-bit centre alignment settings.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CentreParams {
    pub bits: u8,
    pub bit_max: u16,
    /// A centre within this many taps of the target counts as aligned.
    pub tolerance: u16,
    /// Re-measure rounds after the first move (0: move once, measure once,
    /// report; each further round corrects the bits still off).
    pub rounds: u8,
}

/// One bit's result.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct BitCentre {
    /// Window centre before the move (strobe-axis taps).
    pub before: i32,
    /// Window centre at the last measurement.
    pub after: i32,
    /// The delay now set.
    pub delay: u16,
    /// Within the tolerance of the target at the last measurement.
    pub verified: bool,
}

/// The aligned result.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Centred<const N: usize> {
    /// The common centre every bit was moved to: the latest bit's centre
    /// (bits can only be delayed); re-taken from the latest measured centre
    /// before every correction round, so it only grows.
    pub target: i32,
    pub bits: [BitCentre; N],
    /// Bits whose delay hit `bit_max` short of the target.
    pub saturated: u32,
    /// Bits still off by more than the tolerance after the last round.
    pub unverified: u32,
    /// Re-measure rounds actually run.
    pub rounds: u8,
    /// Window width (hi - lo) of the narrowest bit at the last measurement.
    /// Unsigned to represent the entire signed 32-bit tap axis exactly.
    pub min_width: u32,
}

/// Why a centre alignment failed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CentreError<E> {
    Target(E),
    /// More bits than the result holds, zero bits, or a start delay above
    /// `bit_max`.
    Config,
    /// A bit has no window at the current delays.
    NoWindow(u8),
    /// The measured low edge is above the high edge.
    InvalidWindow(u8),
}

fn measured_window<M: Measure>(m: &mut M, bit: u8) -> Result<(i32, u32), CentreError<M::Error>> {
    let (lo, hi) = m
        .window(bit)
        .map_err(CentreError::Target)?
        .ok_or(CentreError::NoWindow(bit))?;
    if lo > hi {
        return Err(CentreError::InvalidWindow(bit));
    }
    // A midpoint remains within its endpoints. Round a half tap early;
    // use a wider intermediate for both the sum and the unsigned width.
    let centre = (i64::from(lo) + i64::from(hi)).div_euclid(2) as i32;
    let width = (i64::from(hi) - i64::from(lo)) as u32;
    Ok((centre, width))
}

/// Per-bit centre alignment (see the module docs). `start` holds every
/// bit's current delay; they are set first so the measurement matches.
pub fn per_bit_centre<M: Measure, const N: usize>(
    m: &mut M,
    p: &CentreParams,
    start: &[u16],
) -> Result<Centred<N>, CentreError<M::Error>> {
    let n = usize::from(p.bits);
    if n == 0 || n > N || n > 32 || start.len() != n || start.iter().any(|&d| d > p.bit_max) {
        return Err(CentreError::Config);
    }
    let mut out = Centred {
        target: 0,
        bits: [BitCentre::default(); N],
        saturated: 0,
        unverified: 0,
        rounds: 0,
        min_width: u32::MAX,
    };
    for (b, &d) in start.iter().enumerate() {
        m.set_bit(b as u8, d).map_err(CentreError::Target)?;
        out.bits[b].delay = d;
    }
    // Measure once, find the latest centre.
    for (b, bit) in out.bits[..n].iter_mut().enumerate() {
        let (c, _) = measured_window(m, b as u8)?;
        bit.before = c;
        bit.after = c;
    }
    out.target = out.bits[..n].iter().map(|b| b.before).max().unwrap_or(0);
    // Move every bit to the target (delays only added here: the target is
    // the latest centre), then re-measure and correct, bounded.
    let tol = i64::from(p.tolerance);
    let mut round = 0u8;
    loop {
        for b in 0..n {
            let off = i64::from(out.target) - i64::from(out.bits[b].after);
            if off.abs() <= tol && round > 0 {
                continue;
            }
            let want = i64::from(out.bits[b].delay) + off;
            let clamped = want.clamp(0, i64::from(p.bit_max)) as u16;
            if i64::from(clamped) != want {
                out.saturated |= 1 << b;
            } else {
                out.saturated &= !(1 << b);
            }
            if clamped != out.bits[b].delay {
                m.set_bit(b as u8, clamped).map_err(CentreError::Target)?;
                out.bits[b].delay = clamped;
            }
        }
        // The re-measure hook: every bit again, at the new delays.
        out.unverified = 0;
        out.min_width = u32::MAX;
        for b in 0..n {
            let (c, width) = measured_window(m, b as u8)?;
            out.bits[b].after = c;
            out.bits[b].verified = (i64::from(out.target) - i64::from(c)).abs() <= tol;
            if !out.bits[b].verified {
                out.unverified |= 1 << b;
            }
            out.min_width = out.min_width.min(width);
        }
        if out.unverified == 0 || round >= p.rounds {
            break;
        }
        // Bits interact (a moved bit can shift a neighbour's window, the
        // latest bit's included): the target is re-taken from the latest
        // centre measured, so it only ever grows and bits are only ever
        // delayed to it.
        let latest = out.bits[..n].iter().map(|b| b.after).max().unwrap_or(0);
        if latest > out.target {
            out.target = latest;
            for b in 0..n {
                let v = (i64::from(out.target) - i64::from(out.bits[b].after)).abs() <= tol;
                out.bits[b].verified = v;
                if !v {
                    out.unverified |= 1 << b;
                }
            }
        }
        round += 1;
    }
    out.rounds = round;
    Ok(out)
}
