// SPDX-License-Identifier: GPL-2.0-only
//! Read Vref: the PHY's receiver reference (VrefDAC0/VrefDAC1 per DBYTE
//! lane, `0x1n04e`/`0x1n04f + 0x100 lane`, 7-bit) chosen from our own read
//! eye instead of a seeded value (FEL lab). Replaces, when applied, the
//! init tables'
//! constant 0x35 (REFERENCE-DERIVED) and is the measured counterpart of the
//! vendor library's trained per-lane values (feature `av3c`, comparison
//! only).
//!
//! Method: a 2-D read eye, Vref x receive clock delay, done as a 1-D Vref
//! sweep whose every point is the read training's own per-lane receive
//! clock sweep (`rdtrain` step 4: T and C together, the RDC compare); per
//! lane the window width (the longest passing run, in sweep points) gives
//! the lane's Vref profile, and the Vref is the middle of the profile at
//! half its peak (`dram_train::eye::center`, the K1's eye centring;
//! PG313/PG456-style eye centring, cited, not copied). Per lane, because
//! the PHY has a reference per lane (the vendor's trained values differ
//! per lane by up to 0x15 codes). The per-byte common choice (the worst
//! lane's profile) is reported next to it.
//!
//! Not the AMD PG456 p157-158 balance search (`dram_train::vref_balance`):
//! that needs separate windows for samples expected 0 and 1, and the PHY's
//! DTSM compare counts errors per lane, not per expected value.
//!
//! At pstate 1 the training firmware leaves VrefDAC at the init value 0x35
//! on every lane (a board dump, MEASURED); its 2-D
//! read stage runs only at pstate 0 (REFERENCE-DERIVED). So this
//! stage at pstate 1 has no firmware reference to match, only the eye
//! width it reports.

use crate::wl::LANES;
use dram_train::eye;

/// Vref points of one sweep, at most.
pub const POINTS_MAX: usize = 48;
/// The VrefDAC field.
pub const DAC_MAX: u16 = 0x7f;

/// One sweep's plan (env `A7A_RD_VREF`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct VrefPlan {
    /// First code, step and number of points.
    pub start: u16,
    pub step: u16,
    pub n: u16,
    /// Bit 0: write VrefDAC0, bit 1: VrefDAC1 (both by default; RxDfeModeCfg
    /// is 0 on this board, which of the two the receiver uses then is not
    /// known: sweeping one alone tells).
    pub dacs: u8,
    /// Leave the per-lane choice applied (rank 0 blocks only; rank 1
    /// blocks measure) and re-centre the receive clocks there.
    pub apply: bool,
}

impl VrefPlan {
    /// 0x20..0x50 step 2 (25 points) around the init value 0x35 and the
    /// vendor's 0x2c..0x41, both DACs, measure only.
    pub const MEASURE: VrefPlan = VrefPlan {
        start: 0x20,
        step: 2,
        n: 25,
        dacs: 3,
        apply: false,
    };

    pub const fn code(&self, k: usize) -> u16 {
        self.start + self.step * k as u16
    }

    pub const fn valid(&self) -> bool {
        self.n >= 3
            && self.n as usize <= POINTS_MAX
            && self.step >= 1
            && self.dacs >= 1
            && self.dacs <= 3
            && self.start + self.step * (self.n - 1) <= DAC_MAX
    }

    /// `measure` or `apply`, optionally followed by `,<start>,<step>,<n>`
    /// and `,<dacs>` (hex). None: unset; Err: malformed.
    pub const fn parse(s: Option<&str>) -> Result<Option<VrefPlan>, ()> {
        let Some(s) = s else {
            return Ok(None);
        };
        let b = s.as_bytes();
        let mut p = VrefPlan::MEASURE;
        let mut i;
        if starts(b, b"measure") {
            i = 7;
        } else if starts(b, b"apply") {
            p.apply = true;
            i = 5;
        } else {
            return Err(());
        }
        let mut field = 0;
        while i < b.len() {
            if b[i] != b',' {
                return Err(());
            }
            i += 1;
            let mut v: u32 = 0;
            let mut digits = 0;
            while i < b.len() && b[i] != b',' {
                let d = match b[i] {
                    b'0'..=b'9' => b[i] - b'0',
                    b'a'..=b'f' => b[i] - b'a' + 10,
                    b'A'..=b'F' => b[i] - b'A' + 10,
                    _ => return Err(()),
                };
                v = v * 16 + d as u32;
                digits += 1;
                if v > 0xffff {
                    return Err(());
                }
                i += 1;
            }
            if digits == 0 {
                return Err(());
            }
            match field {
                0 => p.start = v as u16,
                1 => p.step = v as u16,
                2 => p.n = v as u16,
                3 => p.dacs = v as u8,
                _ => return Err(()),
            }
            field += 1;
        }
        if field != 0 && field < 3 {
            return Err(());
        }
        if !p.valid() {
            return Err(());
        }
        Ok(Some(p))
    }
}

const fn starts(b: &[u8], w: &[u8]) -> bool {
    if b.len() < w.len() {
        return false;
    }
    let mut i = 0;
    while i < w.len() {
        if b[i] != w[i] {
            return false;
        }
        i += 1;
    }
    b.len() == w.len() || b[w.len()] == b','
}

/// One lane's profile: window width (passing sweep points) per Vref point.
pub type Profile = [Option<u8>; POINTS_MAX];

/// One block's sweep.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct VrefResult {
    pub plan: VrefPlan,
    /// Per byte and lane.
    pub profile: [[Profile; LANES]; 2],
    /// VrefDAC0/VrefDAC1 per byte and lane as found.
    pub found: [[[u16; LANES]; 2]; 2],
    /// The per-lane choice, and the per-byte common one (worst lane).
    pub chosen: [[Option<u16>; LANES]; 2],
    pub common: [Option<u16>; 2],
    /// The choice was left applied and the receive clocks re-centred
    /// (`recentred`: the new joint T/C centre per lane, effective taps).
    pub applied: bool,
    pub recentred: [[Option<u16>; LANES]; 2],
    pub ran: bool,
}

impl VrefResult {
    pub const EMPTY: VrefResult = VrefResult {
        plan: VrefPlan::MEASURE,
        profile: [[[None; POINTS_MAX]; LANES]; 2],
        found: [[[0; LANES]; 2]; 2],
        chosen: [[None; LANES]; 2],
        common: [None; 2],
        applied: false,
        recentred: [[None; LANES]; 2],
        ran: false,
    };
}

/// The longest passing run of an `n`-point sweep mask, in points.
pub fn width(mask: u128, n: usize) -> u8 {
    let mut best = 0u32;
    let mut run = 0u32;
    for j in 0..n.min(128) {
        if mask >> j & 1 != 0 {
            run += 1;
            best = best.max(run);
        } else {
            run = 0;
        }
    }
    best.min(255) as u8
}

/// The Vref code for a profile of `plan` (None: closed everywhere or not
/// measured).
pub fn choose(profile: &[Option<u8>], plan: &VrefPlan) -> Option<u16> {
    let n = (plan.n as usize).min(profile.len());
    let e = eye::center(&profile[..n]).ok()?;
    Some(plan.code(e.center))
}

/// The worst (smallest) width per point over `lanes`.
pub fn worst(lanes: &[Profile], n: usize) -> Profile {
    let mut out = [None; POINTS_MAX];
    for (k, o) in out.iter_mut().enumerate().take(n.min(POINTS_MAX)) {
        let mut w: Option<u8> = None;
        for p in lanes {
            match (w, p[k]) {
                (_, None) => {
                    w = None;
                    break;
                }
                (None, Some(x)) => w = Some(x),
                (Some(a), Some(x)) => w = Some(a.min(x)),
            }
        }
        *o = w;
    }
    out
}

/// Both ranks of a channel share the references: the per-point minimum of
/// two blocks' profiles of a lane.
pub fn combine(a: &Profile, b: &Profile) -> Profile {
    worst(&[*a, *b], POINTS_MAX)
}

/// Fill a result's choices from its profiles.
pub fn choose_all(r: &mut VrefResult) {
    let n = r.plan.n as usize;
    for i in 0..2 {
        for lane in 0..LANES {
            r.chosen[i][lane] = choose(&r.profile[i][lane], &r.plan);
        }
        r.common[i] = choose(&worst(&r.profile[i], n), &r.plan);
    }
}

/// The two-dimensional search of `crate::rdeye` (feature `full-speed`;
/// declared here, below the code, so that no line of it moves). A plan for it is a
/// [`VrefPlan`] with [`VrefPlan::EYE_FLAG`] in `dacs`: `start`, `step`
/// and `n` are then the search's coarse reference rows.
#[cfg(any(test, feature = "full-speed"))]
impl VrefPlan {
    /// In `dacs`: the plan is the two-dimensional search's.
    pub const EYE_FLAG: u8 = 0x80;

    /// The search's default coarse rows: 0x18..0x58 step 8 (nine rows)
    /// around the init value 0x35, both DACs, measure only.
    pub const EYE: VrefPlan = VrefPlan {
        start: 0x18,
        step: 8,
        n: 9,
        dacs: 3 | Self::EYE_FLAG,
        apply: false,
    };

    pub const fn eye(&self) -> bool {
        self.dacs & Self::EYE_FLAG != 0
    }

    /// The DACs the plan writes (bit 0 VrefDAC0, bit 1 VrefDAC1).
    pub const fn dac_mask(&self) -> u8 {
        self.dacs & 3
    }

    /// The text of knob `A7A_RD_VREF`: `eye` or `eye-apply` (the
    /// two-dimensional search), optionally followed by
    /// `,<start>,<step>,<n>` and `,<dacs>` (hex) as for the sweep, or one
    /// of the sweep's forms ([`VrefPlan::parse`]). None: unset; Err:
    /// malformed, or coarse rows over the search's budget.
    pub const fn parse_knob(s: Option<&str>) -> Result<Option<VrefPlan>, ()> {
        let Some(text) = s else {
            return Ok(None);
        };
        let b = text.as_bytes();
        let mut p = VrefPlan::EYE;
        let mut i = if starts(b, b"eye-apply") {
            p.apply = true;
            9
        } else if starts(b, b"eye") {
            3
        } else {
            return VrefPlan::parse(s);
        };
        let mut field = 0;
        while i < b.len() {
            i += 1;
            let mut v: u32 = 0;
            let mut digits = 0;
            while i < b.len() && b[i] != b',' {
                let d = match b[i] {
                    b'0'..=b'9' => b[i] - b'0',
                    b'a'..=b'f' => b[i] - b'a' + 10,
                    b'A'..=b'F' => b[i] - b'A' + 10,
                    _ => return Err(()),
                };
                v = v * 16 + d as u32;
                digits += 1;
                if v > 0xff {
                    return Err(());
                }
                i += 1;
            }
            if digits == 0 {
                return Err(());
            }
            match field {
                0 => p.start = v as u16,
                1 => p.step = v as u16,
                2 => p.n = v as u16,
                3 if v >= 1 && v <= 3 => p.dacs = v as u8 | Self::EYE_FLAG,
                _ => return Err(()),
            }
            field += 1;
        }
        let rows = VrefPlan {
            dacs: p.dac_mask(),
            ..p
        };
        if (field != 0 && field < 3) || !rows.valid() || !crate::rdeye::rows_valid(&p) {
            return Err(());
        }
        Ok(Some(p))
    }
}

/// The parser of knob `A7A_RD_VREF` in a build with the search: bt0 reads
/// the knob through `VrefPlan::parse` without feature `full-speed` and
/// through this with it.
#[cfg(any(test, feature = "full-speed"))]
pub struct EyeKnob;

#[cfg(any(test, feature = "full-speed"))]
impl EyeKnob {
    /// [`VrefPlan::parse_knob`].
    pub const fn parse(s: Option<&str>) -> Result<Option<VrefPlan>, ()> {
        VrefPlan::parse_knob(s)
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;

    /// A lane's read eye: width `w0` points at its best reference `c`,
    /// narrowing by `slope` points per code away from it.
    fn lane(plan: &VrefPlan, c: f32, w0: f32, slope: f32) -> Profile {
        let mut p = [None; POINTS_MAX];
        for (k, x) in p.iter_mut().enumerate().take(plan.n as usize) {
            let v = plan.code(k) as f32;
            *x = Some((w0 - slope * (v - c).abs()).max(0.0) as u8);
        }
        p
    }

    #[test]
    fn per_lane_centres() {
        let plan = VrefPlan::MEASURE;
        for c in [0x2c as f32, 0x35 as f32, 0x41 as f32, 0x3a as f32] {
            let p = lane(&plan, c, 40.0, 2.0);
            let got = choose(&p, &plan).unwrap() as f32;
            assert!((got - c).abs() <= 2.0, "{c} -> {got}");
        }
    }

    #[test]
    fn common_follows_the_worst_lane() {
        let plan = VrefPlan::MEASURE;
        let mut r = VrefResult {
            plan,
            ..VrefResult::EMPTY
        };
        for l in 0..LANES {
            r.profile[0][l] = lane(&plan, 0x34 as f32, 40.0, 2.0);
            r.profile[1][l] = lane(&plan, 0x36 as f32, 40.0, 2.0);
        }
        // One narrow lane far off pulls the byte's common choice.
        r.profile[0][3] = lane(&plan, 0x40 as f32, 24.0, 2.0);
        choose_all(&mut r);
        assert!((r.chosen[0][3].unwrap() as i32 - 0x40).abs() <= 2);
        assert!(r.common[0].unwrap() > 0x36);
        assert!((r.common[1].unwrap() as i32 - 0x36).abs() <= 2);
    }

    #[test]
    fn closed_and_unmeasured() {
        let plan = VrefPlan::MEASURE;
        let mut p = [None; POINTS_MAX];
        assert_eq!(choose(&p, &plan), None);
        for x in p.iter_mut().take(plan.n as usize) {
            *x = Some(0);
        }
        assert_eq!(choose(&p, &plan), None);
        // A missing point in a lane makes the worst profile unmeasured there.
        let a = lane(&plan, 0x35 as f32, 30.0, 1.0);
        let mut b = a;
        b[4] = None;
        assert_eq!(worst(&[a, b], plan.n as usize)[4], None);
        assert_eq!(combine(&a, &b)[5], a[5]);
    }

    #[test]
    fn sweep_edge_and_flat_top() {
        let plan = VrefPlan::MEASURE;
        // The best reference below the sweep: the choice stays inside it,
        // at its low end.
        let p = lane(&plan, 0x10 as f32, 60.0, 1.0);
        let c = choose(&p, &plan);
        assert!(matches!(c, None | Some(0x20..=0x2a)), "{c:x?}");
        // A flat top: the middle of the plateau.
        let mut q = [None; POINTS_MAX];
        for (k, x) in q.iter_mut().enumerate().take(plan.n as usize) {
            *x = Some(if (6..=14).contains(&k) { 30 } else { 10 });
        }
        assert_eq!(choose(&q, &plan), Some(plan.code(10)));
    }

    #[test]
    fn widths() {
        assert_eq!(width(0, 65), 0);
        assert_eq!(width(0b0111_0011, 65), 3);
        assert_eq!(width(u128::MAX, 65), 65);
        assert_eq!(width(1 << 64, 64), 0);
    }

    #[test]
    fn knob_text() {
        assert_eq!(VrefPlan::parse(None), Ok(None));
        assert_eq!(
            VrefPlan::parse(Some("measure")),
            Ok(Some(VrefPlan::MEASURE))
        );
        let a = VrefPlan::parse(Some("apply")).unwrap().unwrap();
        assert!(a.apply && a.start == 0x20);
        let m = VrefPlan::parse(Some("measure,28,1,20,1")).unwrap().unwrap();
        assert_eq!(
            (m.start, m.step, m.n, m.dacs, m.apply),
            (0x28, 1, 0x20, 1, false)
        );
        for bad in [
            "measur",
            "measure,",
            "measure,20",
            "measure,20,2",
            "measure,70,2,19",
            "measure,20,2,40",
            "measure,20,0,10",
            "measure,20,2,10,4",
            "apply,20,2,10,1,1",
            "measure,2g,2,10",
            "applyx",
        ] {
            assert_eq!(VrefPlan::parse(Some(bad)), Err(()), "{bad}");
        }
    }
}
