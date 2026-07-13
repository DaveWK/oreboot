// SPDX-License-Identifier: GPL-2.0-only
//! Pass windows in sampled bitmaps.
//!
//! A sweep of `n` points (n <= 128) gives a pass map: bit i set when point i
//! passed. Delay lines that wrap (a CA delay that passes once per 256 units,
//! a phase) are circular: a run may cross from point n-1 to point 0. Others
//! are linear. A [`Window`] is a run `start, start + 1, .., start + len - 1`
//! (mod n when circular).
//!
//! First written for the A733 CA training (ore-edk-boot-radxa-a7a
//! `coldinit.rs`, `blobless.rs`).

/// A run of passing points.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Window {
    pub start: u32,
    pub len: u32,
}

impl Window {
    pub const EMPTY: Window = Window { start: 0, len: 0 };

    pub const fn is_empty(self) -> bool {
        self.len == 0
    }
    /// Centre point (the upper middle for an even length), mod `n`.
    pub const fn center(self, n: u32) -> u32 {
        (self.start + self.len / 2) % n
    }
    /// Whether point `i` lies in the window of an `n`-point circular axis.
    pub const fn contains(self, i: u32, n: u32) -> bool {
        (i % n + n - self.start % n) % n < self.len
    }
    pub const fn as_tuple(self) -> (u32, u32) {
        (self.start, self.len)
    }
}

impl From<Window> for (u32, u32) {
    fn from(w: Window) -> Self {
        (w.start, w.len)
    }
}

impl From<(u32, u32)> for Window {
    fn from((start, len): (u32, u32)) -> Self {
        Window { start, len }
    }
}

/// All `n` points set (n <= 128).
pub const fn full_mask(n: u32) -> u128 {
    if n >= 128 {
        u128::MAX
    } else {
        (1u128 << n) - 1
    }
}

/// The longest run of set bits in a circular `n`-bit map. (0, n) when all
/// pass; the first longest run found after the first zero otherwise; (0, 0)
/// for an empty map.
pub fn longest_run(map: u128, n: u32) -> Window {
    let full = full_mask(n);
    let map = map & full;
    if map == full {
        return Window { start: 0, len: n };
    }
    let mut best = (0, 0);
    // Start counting after a zero so that a wrapping run is counted once.
    let mut first_zero = 0;
    while map >> first_zero & 1 != 0 {
        first_zero += 1;
    }
    let mut start = 0;
    let mut len = 0;
    for k in 1..=n {
        let i = (first_zero + k) % n;
        if map >> i & 1 != 0 {
            if len == 0 {
                start = i;
            }
            len += 1;
            if len > best.1 {
                best = (start, len);
            }
        } else {
            len = 0;
        }
    }
    best.into()
}

/// The longest run of set bits in a linear (non-wrapping) `n`-bit map, the
/// lowest one on a tie; (0, 0) for an empty map.
pub fn longest_run_linear(map: u128, n: u32) -> Window {
    let map = map & full_mask(n);
    let mut best = Window::EMPTY;
    let mut start = 0;
    let mut len = 0;
    for i in 0..n {
        if map >> i & 1 != 0 {
            if len == 0 {
                start = i;
            }
            len += 1;
            if len > best.len {
                best = Window { start, len };
            }
        } else {
            len = 0;
        }
    }
    best
}

/// The run of set bits in a circular `n`-bit map that contains point `pref`,
/// or else the run whose center is circularly nearest to it (ties: the
/// longer, then the first found); (0, 0) for an empty map. Picks one of
/// several equivalent windows consistently around a reference point.
pub fn run_near(map: u128, n: u32, pref: u32) -> Window {
    let full = full_mask(n);
    let map = map & full;
    if map == 0 {
        return Window::EMPTY;
    }
    if map == full {
        return Window { start: 0, len: n };
    }
    let bit = |i: u32| map >> (i % n) & 1 != 0;
    // Enumerate runs starting after a zero.
    let mut best: Option<((u32, u32), u32)> = None;
    for start in 0..n {
        if !bit(start) || bit(start + n - 1) {
            continue;
        }
        let mut len = 0;
        while len < n && bit(start + len) {
            len += 1;
        }
        let contains = (pref + n - start) % n < len;
        // Doubled coordinates: center = 2 start + len - 1.
        let c2 = (2 * start + len - 1) % (2 * n);
        let p2 = 2 * (pref % n);
        let d = c2.abs_diff(p2);
        let dist = if contains { 0 } else { d.min(2 * n - d) };
        let better = match best {
            None => true,
            Some(((_, bl), bd)) => dist < bd || (dist == bd && len > bl),
        };
        if better {
            best = Some(((start, len), dist));
        }
    }
    best.map(|b| b.0.into()).unwrap_or(Window::EMPTY)
}

/// Points that pass in every map (`n` points).
pub fn intersect(maps: impl IntoIterator<Item = u128>, n: u32) -> u128 {
    maps.into_iter().fold(full_mask(n), |acc, m| acc & m)
}

/// [`run_near`] of the intersection of `maps`: the window a setting shared
/// by several ranks, phases or blocks must lie in.
pub fn intersect_near(maps: impl IntoIterator<Item = u128>, n: u32, pref: u32) -> Window {
    run_near(intersect(maps, n), n, pref)
}

/// Bits of `bits` whose echo matched the expected level in every pattern:
/// pattern p expects all ones where `expect[p]` and all zeros elsewhere.
/// Patterns beyond the shorter of `echo` and `expect` are ignored.
pub const fn pass_bits(echo: &[u16], expect: &[bool], bits: u16) -> u16 {
    let mut pass = bits;
    let mut p = 0;
    while p < echo.len() && p < expect.len() {
        let want = if expect[p] { 0xffff } else { 0 };
        pass &= !(echo[p] ^ want);
        p += 1;
    }
    pass
}

/// The longest run of a pass map (`circular` or linear, `n` points) if it is
/// at least `min_len` points long: an edge is accepted only with a minimum
/// contiguous passing run behind it (AMD PG150 v1.4 p768: 10 taps, PG353
/// v1.0 p233: 15; method only).
pub fn min_run(map: u128, n: u32, min_len: u32, circular: bool) -> Option<Window> {
    let w = if circular {
        longest_run(map, n)
    } else {
        longest_run_linear(map, n)
    };
    if w.len >= min_len.max(1) {
        Some(w)
    } else {
        None
    }
}

/// Points a minimum window of `ui_permille` / 1000 of the unit interval
/// needs on an axis of `step_fs` femtoseconds per point (rounded up, at least
/// 1), for a unit interval of `ui_fs`. Gates belong in UI fractions or ps
/// with each board's step size, not in another PHY's tap counts (a literal
/// 15-point gate rejects 5 of the 56 recorded A7A CA bit windows).
pub const fn gate_points(ui_permille: u32, ui_fs: u64, step_fs: u64) -> u32 {
    if step_fs == 0 {
        return u32::MAX;
    }
    let need = (ui_permille as u64 * ui_fs).div_ceil(1000 * step_fs);
    if need == 0 {
        1
    } else if need > u32::MAX as u64 {
        u32::MAX
    } else {
        need as u32
    }
}

/// [`min_run`] with the gate as a fraction of the UI ([`gate_points`]).
pub fn min_run_ui(
    map: u128,
    n: u32,
    circular: bool,
    ui_permille: u32,
    ui_fs: u64,
    step_fs: u64,
) -> Option<Window> {
    min_run(map, n, gate_points(ui_permille, ui_fs, step_fs), circular)
}
