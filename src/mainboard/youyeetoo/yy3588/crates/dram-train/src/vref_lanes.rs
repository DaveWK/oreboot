// SPDX-License-Identifier: GPL-2.0-only
//! One Vref per lane group from one shared sweep, and the rule for keeping
//! it (per-lane RX Vref).
//!
//! A receiver Vref is often one register field per byte lane (the K1 PHY
//! RX Vref: four DQ nibbles for four byte lanes), while the sweep that
//! measures it moves every lane together and centres on the worst lane
//! over the whole bus ([`crate::vref_sweep`]). Every run of that sweep
//! already yields each lane's window at the point, so no extra engine run
//! is needed to know each lane group's own profile: [`LaneProfiles`] keeps
//! the minimum over ranks and repeats per group and point while the sweep
//! runs, [`centres`] centres every group's profile with the same
//! [`crate::eye::Policy`] as the common choice, and [`Verdict::judge`]
//! decides from a re-measurement at the per-group points whether to keep
//! them: only when the worst window over the bus is not smaller and the sum
//! of lane windows is not smaller than at the common point. Otherwise the
//! common point stays (one measured setting is never traded for an
//! unmeasured hope).
//!
//! Method: generic per-byte Vref practice (every PHY with per-byte receiver
//! Vref trains them independently); the accept-only-if-not-worse rule is
//! ours. No logic or value from any vendor training image.

use crate::eye::{center_with, CenterError, Eye, Policy};

/// Per-group worst-window profiles along one axis of `N` points for
/// `GROUPS` lane groups.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LaneProfiles<const GROUPS: usize, const N: usize> {
    /// `widths[g][i]`: the smallest window of group `g` measured at axis
    /// point `i` (over its lanes, ranks and repeats); `None` not measured.
    pub widths: [[Option<u8>; N]; GROUPS],
}

impl<const GROUPS: usize, const N: usize> Default for LaneProfiles<GROUPS, N> {
    fn default() -> Self {
        Self {
            widths: [[None; N]; GROUPS],
        }
    }
}

impl<const GROUPS: usize, const N: usize> LaneProfiles<GROUPS, N> {
    pub const fn new() -> Self {
        Self {
            widths: [[None; N]; GROUPS],
        }
    }

    /// Record one measurement at axis point `i`: `window[l]` is lane `l`'s
    /// window, `group(l)` its group (a group index at or above `GROUPS` is
    /// ignored). Repeated calls at the same point keep the minimum.
    pub fn record(&mut self, i: usize, window: &[u8], group: impl Fn(usize) -> usize) {
        if i >= N {
            return;
        }
        for (l, &w) in window.iter().enumerate() {
            let g = group(l);
            if g >= GROUPS {
                continue;
            }
            let slot = &mut self.widths[g][i];
            *slot = Some(match *slot {
                Some(v) => v.min(w),
                None => w,
            });
        }
    }

    /// A copy fit for choosing per-group points (measured on the R2S,
    /// 2026-10-16): a window at or above `saturated` is the
    /// engine's "no edge found" value, not a measurement, and counts as
    /// closed (one such value at a closed code won a lane's choice and broke
    /// the training); and in every group, each point outside the group's own
    /// contiguous open run around `centre` (the common choice) counts as
    /// closed, so a group moves only through codes where it was itself seen
    /// open, never to an isolated spike. A group closed at `centre` keeps
    /// its profile (centring it then fails as before, or finds its run).
    pub fn censored(&self, saturated: u8, centre: usize) -> Self {
        let mut out = *self;
        for g in out.widths.iter_mut() {
            for w in g.iter_mut().flatten() {
                if *w >= saturated {
                    *w = 0;
                }
            }
            let open = |g: &[Option<u8>; N], i: usize| matches!(g.get(i), Some(Some(w)) if *w > 0);
            if !open(g, centre) {
                continue;
            }
            let mut lo = centre;
            while lo > 0 && open(g, lo - 1) {
                lo -= 1;
            }
            let mut hi = centre;
            while hi + 1 < N && open(g, hi + 1) {
                hi += 1;
            }
            for (i, w) in g.iter_mut().enumerate() {
                if let Some(v) = w {
                    if i < lo || i > hi {
                        *v = 0;
                    }
                }
            }
        }
        out
    }

    /// The common (all-groups) worst profile: the minimum over groups at each
    /// measured point, `None` where no group was measured.
    pub fn common(&self) -> [Option<u8>; N] {
        let mut out = [None; N];
        for (i, o) in out.iter_mut().enumerate() {
            for g in 0..GROUPS {
                if let Some(w) = self.widths[g][i] {
                    *o = Some(o.map_or(w, |v: u8| v.min(w)));
                }
            }
        }
        out
    }
}

/// The per-group choice.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Choice<const GROUPS: usize> {
    pub eyes: [Eye; GROUPS],
    /// The chosen axis point per group.
    pub points: [usize; GROUPS],
    /// Some group's point differs from `common`.
    pub differs: bool,
}

/// Centre every group's profile with `policy` (the same way the common
/// sweep centred the bus profile); `common` is the common sweep's chosen
/// point. A group with a closed or empty profile is an error: the common
/// sweep would have failed on it too.
pub fn centres<const GROUPS: usize, const N: usize>(
    p: &LaneProfiles<GROUPS, N>,
    policy: Policy,
    common: usize,
) -> Result<Choice<GROUPS>, CenterError> {
    let mut eyes = [Eye {
        lo: 0,
        hi: 0,
        center: 0,
        peak: 0,
        peak_width: 0,
        level: 0,
        center_width: 0,
    }; GROUPS];
    let mut points = [common; GROUPS];
    for g in 0..GROUPS {
        let c = center_with(&p.widths[g], policy)?;
        eyes[g] = c.eye;
        points[g] = c.eye.center;
    }
    Ok(Choice {
        eyes,
        points,
        differs: points.iter().any(|&x| x != common),
    })
}

/// Whether the per-group setting is kept, from the bus windows measured at
/// the common point and at the per-group points.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Verdict {
    /// The per-group windows are not worse (worst not smaller, sum not
    /// smaller): keep the per-group setting.
    Kept,
    /// The per-group windows are worse somewhere: the common point stays.
    Reverted,
    /// Nothing to decide: every group chose the common point.
    Same,
}

impl Verdict {
    /// `common` and `per_group` are the final per-lane windows measured at
    /// the common point and at the per-group points (the minimum over ranks).
    pub fn judge(common: &[u8], per_group: &[u8]) -> Verdict {
        let worst = |w: &[u8]| w.iter().copied().min().unwrap_or(0);
        let sum = |w: &[u8]| w.iter().map(|&x| u32::from(x)).sum::<u32>();
        if worst(per_group) >= worst(common) && sum(per_group) >= sum(common) {
            Verdict::Kept
        } else {
            Verdict::Reverted
        }
    }

    /// [`judge`](Self::judge) with every window at or above `saturated`
    /// counted as closed (0) on both sides: a saturated re-measurement is no
    /// evidence that the per-group setting is better. A per-group setting
    /// left with a closed lane is never kept, even against an equally closed
    /// common point.
    pub fn judge_censored(common: &[u8], per_group: &[u8], saturated: u8) -> Verdict {
        let c = |w: &[u8], i: usize| if w[i] >= saturated { 0 } else { w[i] };
        let n = common.len().min(per_group.len()).min(64);
        let mut a = [0u8; 64];
        let mut b = [0u8; 64];
        for i in 0..n {
            a[i] = c(common, i);
            b[i] = c(per_group, i);
        }
        if b[..n].contains(&0) {
            return Verdict::Reverted;
        }
        Verdict::judge(&a[..n], &b[..n])
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn lane_group(l: usize) -> usize {
        l / 8
    }

    #[test]
    fn profiles_keep_the_minimum_per_group_and_point() {
        let mut p = LaneProfiles::<4, 16>::new();
        let mut w = [10u8; 32];
        w[3] = 7; // group 0
        w[20] = 9; // group 2
        p.record(5, &w, lane_group);
        assert_eq!(p.widths[0][5], Some(7));
        assert_eq!(p.widths[1][5], Some(10));
        assert_eq!(p.widths[2][5], Some(9));
        assert_eq!(p.widths[3][5], Some(10));
        // A repeat at the same point lowers but never raises.
        let mut w2 = [12u8; 32];
        w2[9] = 6; // group 1
        p.record(5, &w2, lane_group);
        assert_eq!(p.widths[0][5], Some(7));
        assert_eq!(p.widths[1][5], Some(6));
        assert_eq!(p.widths[3][5], Some(10));
        assert_eq!(p.widths[0][4], None);
        // Out-of-range points and groups are ignored.
        p.record(16, &w, lane_group);
        p.record(0, &w, |_| 7);
        assert!(p.widths.iter().all(|g| g[0].is_none()));
        let c = p.common();
        assert_eq!(c[5], Some(6));
        assert_eq!(c[4], None);
    }

    /// Two groups peak two codes apart: the common choice sits between them
    /// at the worst 9; each group's own centre gives 11 (or the peak band).
    #[test]
    fn groups_centred_on_their_own_profiles() {
        let mut p = LaneProfiles::<2, 16>::new();
        for code in 0..16usize {
            let a = 11u8.saturating_sub((code as i32 - 5).unsigned_abs() as u8);
            let b = 11u8.saturating_sub((code as i32 - 9).unsigned_abs() as u8);
            let mut w = [0u8; 16];
            w[..8].fill(a);
            w[8..].fill(b);
            p.record(code, &w, lane_group);
        }
        let common = p.common();
        let c0 = center_with(&common, Policy::DEFAULT).unwrap();
        assert_eq!(c0.eye.center, 7);
        assert_eq!(c0.eye.center_width, 9);
        let ch = centres(&p, Policy::DEFAULT, c0.eye.center).unwrap();
        assert!(ch.differs);
        assert_eq!(ch.points, [5, 9]);
        assert_eq!(ch.eyes[0].center_width, 11);
        assert_eq!(ch.eyes[1].center_width, 11);
    }

    #[test]
    fn equal_groups_choose_the_common_point() {
        let mut p = LaneProfiles::<4, 16>::new();
        for code in 0..16usize {
            let w = 11u8.saturating_sub((code as i32 - 7).unsigned_abs() as u8);
            p.record(code, &[w; 32], lane_group);
        }
        let c0 = center_with(&p.common(), Policy::DEFAULT).unwrap();
        let ch = centres(&p, Policy::DEFAULT, c0.eye.center).unwrap();
        assert!(!ch.differs);
        assert_eq!(ch.points, [c0.eye.center; 4]);
    }

    #[test]
    fn a_closed_group_is_an_error() {
        let mut p = LaneProfiles::<2, 4>::new();
        p.record(0, &[5, 0], |l| l);
        p.record(1, &[5, 0], |l| l);
        assert_eq!(
            centres(&p, Policy::DEFAULT, 0).unwrap_err(),
            CenterError::Closed
        );
        assert_eq!(
            centres(&LaneProfiles::<1, 4>::new(), Policy::DEFAULT, 0).unwrap_err(),
            CenterError::Empty
        );
    }

    /// The R2S fp2 read profiles of Oct 16 (3b): lane 0 shows the saturated
    /// 31 at code 14 between 4 and 0; the common eye is open at 2..=7.
    #[test]
    fn a_saturated_spike_outside_the_common_range_never_wins() {
        let lanes: [[u8; 16]; 4] = [
            [9, 9, 10, 11, 11, 11, 11, 10, 9, 9, 8, 8, 7, 4, 31, 0],
            [9, 9, 10, 10, 10, 11, 11, 11, 10, 9, 9, 9, 8, 5, 4, 4],
            [9, 9, 10, 10, 10, 10, 11, 10, 9, 9, 9, 8, 7, 4, 0, 0],
            [10, 10, 10, 10, 11, 11, 10, 10, 10, 9, 9, 9, 7, 4, 4, 0],
        ];
        let mut p = LaneProfiles::<4, 16>::new();
        for code in 0..16 {
            let mut w = [0u8; 32];
            for (l, lane) in lanes.iter().enumerate() {
                w[l * 8..l * 8 + 8].fill(lane[code]);
            }
            p.record(code, &w, lane_group);
        }
        // Uncensored, lane 0 goes to the spike (what failed on the board).
        let raw = centres(&p, Policy::DEFAULT, 5).unwrap();
        assert_eq!(raw.points[0], 14);
        // Censored (saturation 31, runs around the common 5): lane 0 back on
        // its real plateau 3..=6; the run of lane 0 ends at 13.
        let ch = centres(&p.censored(31, 5), Policy::DEFAULT, 5).unwrap();
        assert!((3..=6).contains(&ch.points[0]), "{:?}", ch.points);
        assert!(ch.points.iter().all(|&x| x <= 13), "{:?}", ch.points);
        // An isolated spike beyond a closed code never wins either.
        let mut q = LaneProfiles::<1, 8>::new();
        for (i, w) in [6u8, 8, 8, 6, 0, 0, 20, 0].iter().enumerate() {
            q.record(i, &[*w], |_| 0);
        }
        assert_eq!(centres(&q, Policy::DEFAULT, 2).unwrap().points[0], 6);
        let c = q.censored(31, 2);
        assert_eq!(c.widths[0][6], Some(0));
        assert!((1..=2).contains(&centres(&c, Policy::DEFAULT, 2).unwrap().points[0]));
        // Unmeasured points stay unmeasured.
        let q = LaneProfiles::<1, 4>::new().censored(31, 0);
        assert!(q.widths[0].iter().all(|w| w.is_none()));
    }

    #[test]
    fn a_saturated_remeasurement_is_not_kept() {
        let common = [11u8, 11, 10, 11];
        // Lane 0 saturated at the per-group setting: raw judge keeps it.
        assert_eq!(Verdict::judge(&common, &[31, 11, 10, 11]), Verdict::Kept);
        assert_eq!(
            Verdict::judge_censored(&common, &[31, 11, 10, 11], 31),
            Verdict::Reverted
        );
        assert_eq!(
            Verdict::judge_censored(&common, &[12, 11, 10, 11], 31),
            Verdict::Kept
        );
    }

    /// Both sides closed in the same lane: no evidence for the per-group
    /// setting, however the sums compare.
    #[test]
    fn a_setting_with_a_closed_lane_is_never_kept() {
        let v = Verdict::judge_censored(&[31, 11, 10, 12], &[31, 12, 10, 12], 31);
        assert_eq!(v, Verdict::Reverted);
        let v = Verdict::judge_censored(&[0, 11, 10, 12], &[0, 12, 10, 12], 31);
        assert_eq!(v, Verdict::Reverted);
    }

    #[test]
    fn kept_only_when_worst_and_sum_do_not_drop() {
        let common = [10u8, 10, 9, 10];
        assert_eq!(Verdict::judge(&common, &[10, 10, 10, 10]), Verdict::Kept);
        assert_eq!(Verdict::judge(&common, &[10, 10, 9, 10]), Verdict::Kept);
        // Worst equal, sum lower: reverted.
        assert_eq!(Verdict::judge(&common, &[9, 10, 9, 10]), Verdict::Reverted);
        // Sum higher, worst lower: reverted.
        assert_eq!(Verdict::judge(&common, &[12, 12, 8, 12]), Verdict::Reverted);
        assert_eq!(Verdict::judge(&[], &[]), Verdict::Kept);
    }
}
