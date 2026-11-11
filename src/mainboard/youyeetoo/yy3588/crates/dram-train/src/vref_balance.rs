// SPDX-License-Identifier: GPL-2.0-only
//! Vref by balance: at each Vref, measure the window of the samples that
//! should read 0 and the window of those that should read 1; a wider
//! 0-window means the reference is above the centre of the signal swing,
//! so step it down, a wider 1-window means step it up; stop when the two
//! differ by at most a tolerance. An alternative to the ratio-bounded
//! sweep ([`crate::vref_sweep`]) for engines that report both windows: it
//! visits only the points on the way to the balance, not the whole axis.
//!
//! Method: AMD PG456 v1.0 p157-158 (the Vref algorithm shared by CA, read
//! DQ and write DQ), cited, not copied. PG456 gives no tolerance or step;
//! here both are the caller's, and a search that steps back and forth
//! between two points settles on the better balanced of the two.

/// The axis: Vref points `0..len` (higher index = higher reference).
pub trait Axis {
    type Error;
    fn set(&mut self, point: usize) -> Result<(), Self::Error>;
    /// (0-window, 1-window) at the current point, in delay steps.
    fn windows(&mut self) -> Result<(u16, u16), Self::Error>;
}

/// The balanced point.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Balanced {
    pub point: usize,
    pub zero: u16,
    pub one: u16,
    /// Points measured.
    pub steps: u32,
    /// The search stopped between two points (the imbalance there was
    /// above the tolerance on both sides).
    pub straddled: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BalanceError<E> {
    Axis(E),
    /// Both windows are closed at a point on the way.
    Closed {
        point: usize,
    },
    /// The balance lies beyond an end of the axis.
    OutOfRange {
        point: usize,
        zero: u16,
        one: u16,
    },
    /// No balance within `max_steps` points.
    NoBalance,
}

/// Search from `start` on an axis of `len` points (see the module docs).
/// The axis is left at the result point.
pub fn balance<A: Axis>(
    a: &mut A,
    start: usize,
    len: usize,
    tolerance: u16,
    max_steps: u32,
) -> Result<Balanced, BalanceError<A::Error>> {
    if len == 0 {
        return Err(BalanceError::NoBalance);
    }
    let mut p = start.min(len - 1);
    let mut prev: Option<(usize, u16, u16)> = None;
    let mut steps = 0;
    loop {
        if steps == max_steps {
            return Err(BalanceError::NoBalance);
        }
        a.set(p).map_err(BalanceError::Axis)?;
        let (z, o) = a.windows().map_err(BalanceError::Axis)?;
        steps += 1;
        if z == 0 && o == 0 {
            return Err(BalanceError::Closed { point: p });
        }
        let done = |point, zero, one, straddled| Balanced {
            point,
            zero,
            one,
            steps,
            straddled,
        };
        if z.abs_diff(o) <= tolerance {
            return Ok(done(p, z, o, false));
        }
        let down = z > o;
        // Stepping back to where we came from: the balance is between the
        // two points; keep the better balanced one.
        if let Some((q, qz, qo)) = prev {
            if (down && q + 1 == p) || (!down && p + 1 == q) {
                let better_here = (z.abs_diff(o), u16::MAX - z.min(o))
                    <= (qz.abs_diff(qo), u16::MAX - qz.min(qo));
                if better_here {
                    return Ok(done(p, z, o, true));
                }
                a.set(q).map_err(BalanceError::Axis)?;
                return Ok(done(q, qz, qo, true));
            }
        }
        let next = if down { p.checked_sub(1) } else { Some(p + 1) };
        match next {
            Some(n) if n < len => {
                prev = Some((p, z, o));
                p = n;
            }
            _ => {
                return Err(BalanceError::OutOfRange {
                    point: p,
                    zero: z,
                    one: o,
                })
            }
        }
    }
}
