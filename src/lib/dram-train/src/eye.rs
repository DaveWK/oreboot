// SPDX-License-Identifier: GPL-2.0-only
//! Eye centring along one axis (a Vref or delay setting) of measured widths.
//!
//! For each axis point a measurement gives the worst window over the lanes
//! and ranks that share the setting (a "width"). Along the axis these widths
//! form the eye's profile. The chosen point is the middle of the eye at half
//! its peak opening, not the point of the largest width: a flat or skewed top
//! then still yields a centred setting, which keeps the most margin in both
//! directions.
//!
//! Since v1.0.0 [`center`] uses [`Policy::DEFAULT`]: when the half-height
//! region reaches an end of the scan (a nearly flat profile, R2S 2400 MT/s
//! write eye: REQ-K1-01), the point is the middle of the longest run of
//! points at the peak width (a single-point peak: a 1-tap band around it)
//! instead of the scan midpoint. [`center_legacy`] is the moved code
//! unchanged.
//!
//! First written for the SpacemiT K1 own training (ore-edk-boot-opi
//! `train_eye.rs`, from JEDEC LPDDR4/4X and generic eye-centering practice;
//! no logic or parameter from SpacemiT's training image).

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Eye {
    /// Lowest and highest axis point whose width is at least `level`,
    /// contiguous with the peak.
    pub lo: usize,
    pub hi: usize,
    /// Chosen axis point.
    pub center: usize,
    pub peak: usize,
    pub peak_width: u8,
    /// Half the peak width, at least 1.
    pub level: u8,
    pub center_width: u8,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EyeError {
    /// Nothing measured.
    Empty,
    /// Every measured point has a zero window.
    Closed,
}

/// Centre of the eye with the default [`Policy`] ([`Policy::DEFAULT`]): the
/// middle at half height, or the middle of the plateau around the peak when
/// the half-height region reaches an end of the scan. `widths[i]` is the
/// worst-case window at axis point `i`, `None` where it was not measured. The
/// first highest point is the peak.
pub fn center(widths: &[Option<u8>]) -> Result<Eye, EyeError> {
    match center_with(widths, Policy::DEFAULT) {
        Ok(c) => Ok(c.eye),
        Err(CenterError::Empty) => Err(EyeError::Empty),
        Err(CenterError::Closed | CenterError::EdgeOutsideScan(_)) => Err(EyeError::Closed),
    }
}

/// The centre as moved from the K1 (`train_eye.rs`), before v1.0.0: always
/// the middle of the half-height region ([`Policy::LEGACY`]).
pub fn center_legacy(widths: &[Option<u8>]) -> Result<Eye, EyeError> {
    let mut peak = None;
    for (i, w) in widths.iter().enumerate() {
        if let Some(w) = *w {
            match peak {
                Some((_, best)) if best >= w => {}
                _ => peak = Some((i, w)),
            }
        }
    }
    let (peak, peak_width) = peak.ok_or(EyeError::Empty)?;
    if peak_width == 0 {
        return Err(EyeError::Closed);
    }
    let level = peak_width.div_ceil(2).max(1);
    let open = |i: usize| matches!(widths[i], Some(w) if w >= level);
    let mut lo = peak;
    while lo > 0 && open(lo - 1) {
        lo -= 1;
    }
    let mut hi = peak;
    while hi + 1 < widths.len() && open(hi + 1) {
        hi += 1;
    }
    let mut center = (lo + hi) / 2;
    let width = |i: usize| widths[i].unwrap_or(0);
    if (lo + hi) % 2 == 1 && width(center + 1) > width(center) {
        center += 1;
    }
    Ok(Eye {
        lo,
        hi,
        center,
        peak,
        peak_width,
        level,
        center_width: width(center),
    })
}

/// Measure the axis outwards from `start`: first upwards, then downwards,
/// each direction stopping after `stop_after` consecutive closed (zero)
/// points or at the axis end. If nothing open was found, the remaining points
/// are measured as well (the start was outside the eye). A measurement error
/// stops the search and is returned.
pub fn walk_out<E>(
    out: &mut [Option<u8>],
    start: usize,
    stop_after: usize,
    mut measure: impl FnMut(usize) -> Result<u8, E>,
) -> Result<(), E> {
    for w in out.iter_mut() {
        *w = None;
    }
    let len = out.len();
    if len == 0 {
        return Ok(());
    }
    let start = start.min(len - 1);
    let mut closed = 0;
    for (i, slot) in out.iter_mut().enumerate().skip(start) {
        let w = measure(i)?;
        *slot = Some(w);
        closed = if w == 0 { closed + 1 } else { 0 };
        if closed >= stop_after {
            break;
        }
    }
    closed = if out[start] == Some(0) { 1 } else { 0 };
    for i in (0..start).rev() {
        if closed >= stop_after {
            break;
        }
        let w = measure(i)?;
        out[i] = Some(w);
        closed = if w == 0 { closed + 1 } else { 0 };
    }
    if out.iter().all(|w| matches!(w, None | Some(0))) {
        for (i, slot) in out.iter_mut().enumerate() {
            if slot.is_none() {
                *slot = Some(measure(i)?);
            }
        }
    }
    Ok(())
}

/// Smallest value of a margin window (e.g. one per DQ); 0 when empty.
pub fn worst(m: &[u8]) -> u8 {
    m.iter().copied().min().unwrap_or(0)
}

// ---------------------------------------------------------------------------
// Centring policy (REQ-K1-01). On the R2S at 2400 MT/s the write profile was
// nearly flat (widths 6-9 over the whole scan): every point is above half
// height, so the half-height region is the whole scan and [`center`] takes
// the scan midpoint instead of the widest plateau. A policy can centre on the
// plateau near the peak whenever the half-height region reaches a scan end.

/// How close to the peak width a point must be to count as plateau: within
/// `taps` of the peak, or within `percent` of the peak, whichever allows
/// more.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Plateau {
    pub taps: u8,
    pub percent: u8,
}

impl Plateau {
    /// 1 tap or 10 % (used only around a single-point peak).
    pub const DEFAULT: Plateau = Plateau {
        taps: 1,
        percent: 10,
    };
    /// 1 tap.
    pub const ONE_TAP: Plateau = Plateau {
        taps: 1,
        percent: 0,
    };
    /// The lowest width on the plateau of a peak `peak_width`.
    pub const fn floor(self, peak_width: u8) -> u8 {
        let pct = (peak_width as u32 * self.percent as u32 / 100) as u8;
        let tol = if pct > self.taps { pct } else { self.taps };
        let f = peak_width.saturating_sub(tol);
        if f == 0 {
            1
        } else {
            f
        }
    }
}

/// How [`center_with`] picks the point.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Policy {
    /// When the half-height region touches either end of the scan, centre on
    /// the longest run of points at the peak width instead; when the peak is
    /// a single point (possibly noise), on the band within this tolerance
    /// around it, preferring the widest of the band's middle and its
    /// neighbours (None: [`center_legacy`]).
    pub plateau_tolerance: Option<Plateau>,
    /// Fail with [`CenterError::EdgeOutsideScan`] when the half-height
    /// region touches an end of the scan (an edge of the eye was not seen).
    pub require_edges: bool,
}

impl Policy {
    /// The pre-v1.0.0 behaviour ([`center_legacy`]): half-height centre,
    /// edges not required.
    pub const LEGACY: Policy = Policy {
        plateau_tolerance: None,
        require_edges: false,
    };
    /// Plateau centring when an edge is outside the scan; a single-point
    /// peak uses a 1-tap band.
    pub const PLATEAU: Policy = Policy {
        plateau_tolerance: Some(Plateau::ONE_TAP),
        require_edges: false,
    };
    /// The default since v1.0.0: [`Policy::PLATEAU`].
    pub const DEFAULT: Policy = Policy::PLATEAU;
    pub const fn legacy() -> Policy {
        Policy::LEGACY
    }
}

impl Default for Policy {
    fn default() -> Self {
        Policy::DEFAULT
    }
}

/// [`center_with`]'s result: the eye and whether the half-height region
/// reached an end of the measured scan.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Centered {
    pub eye: Eye,
    /// The half-height region touches the first or last measured point, so
    /// at least one edge of the eye lies outside the scan.
    pub edge_outside_scan: bool,
    /// The point was chosen on the plateau (`eye.lo..=eye.hi` is then the
    /// plateau, not the half-height region).
    pub plateau: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CenterError {
    Empty,
    Closed,
    /// `require_edges` and an edge of the eye lies outside the scan.
    EdgeOutsideScan(Eye),
}

impl From<EyeError> for CenterError {
    fn from(e: EyeError) -> Self {
        match e {
            EyeError::Empty => CenterError::Empty,
            EyeError::Closed => CenterError::Closed,
        }
    }
}

fn measured_span(widths: &[Option<u8>]) -> (usize, usize) {
    let first = widths.iter().position(|w| w.is_some()).unwrap_or(0);
    let last = widths.iter().rposition(|w| w.is_some()).unwrap_or(0);
    (first, last)
}

/// The centre with a [`Policy`]. With [`Policy::LEGACY`] the eye is exactly
/// [`center_legacy`]'s.
pub fn center_with(widths: &[Option<u8>], policy: Policy) -> Result<Centered, CenterError> {
    let eye = center_legacy(widths)?;
    let (first, last) = measured_span(widths);
    let edge_outside_scan = eye.lo <= first || eye.hi >= last;
    if edge_outside_scan && policy.require_edges {
        return Err(CenterError::EdgeOutsideScan(eye));
    }
    let tol = match policy.plateau_tolerance {
        Some(t) if edge_outside_scan => t,
        _ => {
            return Ok(Centered {
                eye,
                edge_outside_scan,
                plateau: false,
            })
        }
    };
    // The longest run of points at the peak width (the first on a tie).
    let width = |i: usize| widths[i].unwrap_or(0);
    let at_peak = |i: usize| widths[i] == Some(eye.peak_width);
    let (mut lo, mut hi) = (eye.peak, eye.peak);
    let mut i = 0;
    while i < widths.len() {
        if at_peak(i) {
            let s = i;
            while i + 1 < widths.len() && at_peak(i + 1) {
                i += 1;
            }
            if i - s > hi - lo {
                (lo, hi) = (s, i);
            }
        }
        i += 1;
    }
    let mut floor = eye.peak_width;
    if lo == hi {
        // A single-point peak may be noise: the band within the tolerance
        // around it.
        floor = tol.floor(eye.peak_width);
        let on = |i: usize| matches!(widths[i], Some(w) if w >= floor);
        while lo > 0 && on(lo - 1) {
            lo -= 1;
        }
        while hi + 1 < widths.len() && on(hi + 1) {
            hi += 1;
        }
    }
    // The middle, then the widest of it and its neighbours in the band
    // (the middle on a tie).
    let mid = (lo + hi) / 2;
    let mut c = mid;
    for k in [mid.saturating_sub(1).max(lo), (mid + 1).min(hi)] {
        if width(k) > width(c) {
            c = k;
        }
    }
    Ok(Centered {
        eye: Eye {
            lo,
            hi,
            center: c,
            level: floor,
            center_width: width(c),
            ..eye
        },
        edge_outside_scan,
        plateau: true,
    })
}

/// The minimum of `n` measurements (at least one): a noisy point counts at
/// its worst. Stops at the first error.
pub fn min_of<E>(n: u8, mut measure: impl FnMut() -> Result<u8, E>) -> Result<u8, E> {
    let mut w = measure()?;
    for _ in 1..n.max(1) {
        w = w.min(measure()?);
    }
    Ok(w)
}
