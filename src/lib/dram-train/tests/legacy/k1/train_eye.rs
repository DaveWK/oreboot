// Verbatim copy of ore-edk-boot-opi 78479e21ba
// oreboot/src/mainboard/spacemit/k1x/bt0/src/train_eye.rs (equivalence oracle; never built into firmware).
// SPDX-License-Identifier: GPL-2.0-only
//! Vref eye search and centering for our own K1 LPDDR4/4X training.
//!
//! Pure functions, no MMIO. Written for ore-edk-boot-opi from JEDEC LPDDR4/4X
//! (MR14 VREF(DQ) settings and ranges) and generic eye-centering practice. It
//! takes no logic or parameter from SpacemiT's training image or its
//! translation; see doc/K1-OWN-TRAINING-PLAN.md in ore-edk-boot-opi.
//!
//! The training engines report, for one Vref, the timing window of each DQ.
//! The worst DQ (over all ranks) gives one "width" per Vref point. Along the
//! Vref axis these widths form the eye's vertical profile. We choose the Vref
//! in the middle of the eye at half its peak opening, not the point of the
//! largest width: a flat or skewed top then still yields a centred Vref, which
//! keeps the most Vref margin in both directions.

/// RX Vref codes: one 4-bit DQ field per byte lane in the PHY (GPL-2.0+
/// `lpddr4_silicon_init.c` `top_DDR_rx_ds_odt_vref`: "high 4 bit for DQS,
/// low 4 bit for DQ"). We scan all of them.
pub const RX_CODES: usize = 16;

/// TX Vref axis over JEDEC MR14 VREF(DQ). OP[5:0] is the setting (0..=50,
/// higher values reserved), OP[6] the range, OP[7] RFU. In both the LPDDR4
/// and the LPDDR4X tables range 1 setting 21 is the first one above range 0's
/// top (setting 50), so range 0 settings 0..=50 followed by range 1 settings
/// 21..=50 is monotonic in VDDQ percentage, and range 1 setting s sits at
/// about range 0 setting s + 30.
pub const TX_AXIS_LEN: usize = 51 + 30;

/// MR14 OP byte of TX axis point `i` (`i < TX_AXIS_LEN`).
pub const fn tx_axis_mr14(i: usize) -> u8 {
    if i <= 50 {
        i as u8
    } else {
        0x40 | (i - 30) as u8
    }
}

/// Axis point of an MR14 OP byte; `None` for reserved settings or OP[7] set.
/// Range 1 settings below 21 map to their range 0 equivalent.
pub const fn tx_axis_index(mr14: u8) -> Option<usize> {
    let s = (mr14 & 0x3f) as usize;
    if mr14 & 0x80 != 0 || s > 50 {
        None
    } else if mr14 & 0x40 == 0 {
        Some(s)
    } else {
        Some(s + 30)
    }
}

/// VREF(DQ) of an LPDDR4X MR14 OP byte in 0.1 % of VDDQ (JEDEC LPDDR4X:
/// range 0 15.0..45.0 %, range 1 32.9..62.9 %, 0.6 % steps). Log only.
pub const fn lp4x_mr14_permille(mr14: u8) -> u32 {
    let s = (mr14 & 0x3f) as u32;
    if mr14 & 0x40 == 0 {
        150 + 6 * s
    } else {
        329 + 6 * s
    }
}

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

/// Centre of the eye at half height. `widths[i]` is the worst-case window at
/// axis point `i`, `None` where it was not measured.
pub fn center(widths: &[Option<u8>]) -> Result<Eye, EyeError> {
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
    for i in start..len {
        let w = measure(i)?;
        out[i] = Some(w);
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
        for i in 0..len {
            if out[i].is_none() {
                out[i] = Some(measure(i)?);
            }
        }
    }
    Ok(())
}

/// Smallest value of a margin window (32 DQ).
pub fn worst(m: &[u8]) -> u8 {
    m.iter().copied().min().unwrap_or(0)
}
