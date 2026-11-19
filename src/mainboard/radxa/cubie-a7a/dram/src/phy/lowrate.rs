// SPDX-License-Identifier: GPL-2.0-only
//! The stage generators' forms at pstates 2 and 3 (DfiClk 200 and 100 MHz;
//! WCK 800 and 400 MHz on this board), where they differ from the pstate-1
//! forms the flow was built with: the CS initialization launch and the
//! WCK2CK measurement program. The clock values of the two pstates are in
//! [`crate::pstate`]. Nothing here is used at pstates 0 and 1.
//!
//! REFERENCE-DERIVED like the forms of the other pstates; not run on a
//! board.

use super::coldinit::Row;
use super::wcklevel::{held_edge, Rank, MEASUREMENT_ROWS_PS0};

/// Rows of the CS initialization launch at the build's pstate: the four of
/// `coldinit` at pstates 0 and 1; at pstates 2 and 3 the reference launches
/// the first two only (CS held on both ranks).
pub(crate) const CS_INIT_ROWS: usize = cs_init_rows_at(crate::pstate::ACTIVE);

const fn cs_init_rows_at(pstate: u32) -> usize {
    if pstate >= 2 {
        2
    } else {
        4
    }
}

/// The first `N` of the launch's four rows.
pub(crate) const fn cs_init<const N: usize>(four: [Row; 4]) -> [Row; N] {
    first(&four)
}

/// Rows of the measurement program at a build pstate other than 1
/// ([`alt_program`]).
pub(crate) const ALT_ROWS: usize = alt_rows_at(crate::pstate::ACTIVE);

const fn alt_rows_at(pstate: u32) -> usize {
    if pstate >= 2 {
        MEASUREMENT_ROWS_LOW
    } else {
        MEASUREMENT_ROWS_PS0
    }
}

/// The measurement program of `rank` at a build pstate other than 1: the
/// pstate-0 program `ps0` at pstate 0, [`measurement_program_low`] at
/// pstates 2 and 3.
pub(crate) const fn alt_program(ps0: [Row; MEASUREMENT_ROWS_PS0], rank: Rank) -> [Row; ALT_ROWS] {
    if crate::pstate::ACTIVE >= 2 {
        first(&measurement_program_low(rank))
    } else {
        first(&ps0)
    }
}

/// The first `N` rows of `from` (all of them when it has `N`).
const fn first<const N: usize>(from: &[Row]) -> [Row; N] {
    let mut rows = [Row::EMPTY; N];
    let mut i = 0;
    while i < N {
        rows[i] = from[i];
        i += 1;
    }
    rows
}

/// Rows of [`measurement_program_low`].
pub(crate) const MEASUREMENT_ROWS_LOW: usize = 112;

/// [`wcklevel::measurement_program`](super::wcklevel::measurement_program) at pstates 2 and 3 (DfiClk 200 and 100 MHz).
/// REFERENCE-DERIVED: the reference runs one program at both, and its rules
/// agree with the other two programs:
/// - the time waits are those of pstate 1 at 200 MHz with the MEASURED ACSM
///   unit (`pstate::acsm_scale`: 0x5f00/0x0c00 (29) becomes 0x3f00/0x0800
///   (19), 0x5e00/0x0000 (5) becomes 0x3e00/0x0000 (3)); at 100 MHz the
///   same counts wait twice as long;
/// - the waits in clocks stay (0x4f00/0x1400, 0x4f00/0x0800, 0x1e80, 0x0500);
/// - the held MR18 edges take four rows (the low-speed AC clock 0x182),
///   starting on an odd row; four idle row pairs before the first set, six
///   before the second, two after it;
/// - the eight WCK pattern periods take four rows each (two 0x2000, two
///   0x1000).
pub(crate) const fn measurement_program_low(rank: Rank) -> [Row; MEASUREMENT_ROWS_LOW] {
    let mut rows = [Row::EMPTY; MEASUREMENT_ROWS_LOW];
    let r = rank.control();
    rows[0] = Row::nop(0x3f00);
    rows[1] = Row::nop(0x0800);
    let mut i = 2;
    while i <= 8 {
        rows[i] = Row::nop(0x0400);
        i += 2;
    }
    held_edge(&mut rows, 9, 4, rank, 0x58, true);
    held_edge(&mut rows, 13, 4, rank, 18, true);
    held_edge(&mut rows, 17, 4, rank, 0x48, true);
    held_edge(&mut rows, 21, 4, rank, 0x48, false);
    rows[26] = Row::nop(0x3f00);
    rows[27] = Row::nop(0x0800);
    rows[28] = Row::nop(0x3e00);
    rows[30] = Row::nop(0x0500);
    // Eight periods, four rows each: two with hw1=0x2000, two=0x1000.
    i = 0;
    while i < 32 {
        let data = if i % 4 < 2 { 0x2000 } else { 0x1000 };
        rows[32 + i] = Row::raw(0, data, 0, r | if i % 2 == 0 { 0x0400 } else { 0 });
        i += 1;
    }
    rows[64] = Row::nop(0x0400);
    rows[66] = Row::nop(0x4f00);
    rows[67] = Row::nop(0x1400);
    rows[68] = Row::raw(0x81, 3, 0, 0x0400);
    rows[70] = Row::nop(0x4f00);
    rows[71] = Row::nop(0x0800);
    rows[72] = Row::raw(0, 3, 0, 0x0400);
    rows[74] = Row::nop(0x4f00);
    rows[75] = Row::nop(0x0800);
    rows[76] = Row::nop(0x1e80);
    rows[78] = Row::nop(0x0500);
    rows[80] = Row::nop(0x3f00);
    rows[81] = Row::nop(0x0800);
    i = 82;
    while i <= 92 {
        rows[i] = Row::nop(0x0400);
        i += 2;
    }
    held_edge(&mut rows, 93, 4, rank, 0x58, true);
    held_edge(&mut rows, 97, 4, rank, 18, true);
    held_edge(&mut rows, 101, 4, rank, 0x48, true);
    rows[106] = Row::nop(0x0400);
    rows[108] = Row::nop(0x0400);
    rows[110] = Row::nop(0x3f00);
    rows[111] = Row::nop(0x0800);
    rows
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::phy::acsm_row;
    use crate::phy::wcklevel::measurement_program;
    use crate::pstate::{acsm_scale, acsm_wait_ps};

    fn count(rows: &[Row], even: usize) -> u32 {
        acsm_row::repeat_count(rows[even], rows[even + 1]) as u32
    }

    /// FNV-1a over the rows' four halfwords, low byte first.
    fn fnv(rows: &[Row]) -> u32 {
        let mut h: u32 = 0x811c_9dc5;
        for row in rows {
            for hw in row.0 {
                for b in hw.to_le_bytes() {
                    h = (h ^ b as u32).wrapping_mul(0x0100_0193);
                }
            }
        }
        h
    }

    /// The reference's measurement launches of each rank as checksums of
    /// their rows (REFERENCE-DERIVED; pstates 2 and 3 launch the same rows).
    /// The pstate-1 pair checks the checksum against the program that has
    /// run on the board.
    #[test]
    fn programs_are_the_reference_rows() {
        assert_eq!(fnv(&measurement_program(Rank::R0)), FNV_PS1[0]);
        assert_eq!(fnv(&measurement_program(Rank::R1)), FNV_PS1[1]);
        assert_eq!(fnv(&measurement_program_low(Rank::R0)), FNV_LOW[0]);
        assert_eq!(fnv(&measurement_program_low(Rank::R1)), FNV_LOW[1]);
    }
    const FNV_PS1: [u32; 2] = [0x2395_c8b5, 0xb0c3_fdd5];
    const FNV_LOW: [u32; 2] = [0xe486_6bd5, 0xebba_c855];

    #[test]
    fn time_waits_follow_200_mhz_and_clock_waits_keep_their_count() {
        for rank in [Rank::R0, Rank::R1] {
            let p1 = measurement_program(rank);
            let lo = measurement_program_low(rank);
            // Time waits: pstate 1 rows 0, 38, 108, 150; here rows 0, 26,
            // 80, 110 (head, before the pattern, after it, tail).
            for (a, b) in [(0, 0), (38, 26), (108, 80), (150, 110)] {
                let (n1, n) = (count(&p1, a), count(&lo, b));
                assert_eq!(n, acsm_scale(n1, 300, 200), "rows {a}/{b}");
                assert_eq!(acsm_wait_ps(n1, 300), acsm_wait_ps(n, 200));
                assert_eq!(2 * acsm_wait_ps(n1, 300), acsm_wait_ps(n, 100));
            }
            assert_eq!(count(&lo, 28), acsm_scale(count(&p1, 40), 300, 200));
            // Clock waits keep their count.
            for (a, b) in [(94, 66), (98, 70), (102, 74)] {
                assert_eq!(count(&p1, a), count(&lo, b), "rows {a}/{b}");
            }
            // Two thirds of the pattern rows, the same pattern.
            let pat = |p: &[Row], from: usize, n: usize| {
                p[from..from + n]
                    .iter()
                    .filter(|x| x.0[1] == 0x2000)
                    .count()
            };
            assert_eq!(pat(&p1, 44, 48), 24);
            assert_eq!(pat(&lo, 32, 32), 16);
            // Held edges: two rows per low-speed divider step, as at the
            // other pstates (6 rows at divider 3, 12 at divider 6).
            let ls = crate::pstate::low_speed_any(200);
            assert_eq!(2 * ls.div as usize, 4);
            assert_eq!(crate::pstate::low_speed_any(100), ls);
        }
    }

    #[test]
    fn cs_init_rows_per_pstate() {
        const FOUR: [Row; 4] = [Row::cmd(3, 0), Row::cmd(3, 0), Row::cmd(3, 0), Row::EMPTY];
        assert_eq!([0, 1, 2, 3].map(cs_init_rows_at), [4, 4, 2, 2]);
        assert_eq!(cs_init::<4>(FOUR), FOUR);
        assert_eq!(cs_init::<2>(FOUR), [Row::cmd(3, 0); 2]);
        assert_eq!(cs_init::<2>([Row::cmd(3, 0); 4]), [Row::cmd(3, 0); 2]);
    }

    /// What a build at each pstate launches (run with `A7A_PSTATE` set to
    /// cover pstates 0, 2 and 3): the CS initialization launch and the
    /// low-speed clock of cold init, and the measurement launch.
    #[test]
    fn the_build_pstate_selects_the_forms() {
        extern crate std;
        use crate::phy::coldinit::{self, Mrs};
        use crate::phy::wcklevel;
        use std::vec::Vec;
        let ps = crate::pstate::ACTIVE;
        let (cs_rows, low_speed, rows) = [
            (4, (0x106, 4), 272),
            (4, (0x103, 1), 152),
            (2, (0x102, 0), 112),
            (2, (0x102, 0), 112),
        ][ps as usize];
        let mut launches = Vec::new();
        let mut writes = Vec::new();
        coldinit::part1(&Mrs::ORACLE_PSTATE1_CXMT_4G, &mut |s| match s {
            coldinit::Step::Launch { rows, .. } => launches.push(rows.len()),
            coldinit::Step::Write(csr, v) => writes.push((csr, v)),
            _ => {}
        });
        assert_eq!(launches[0], cs_rows, "pstate {ps}");
        let first = |csr: u32| writes.iter().find(|w| w.0 == csr).unwrap().1;
        assert_eq!((first(0x20042), first(0x20044)), low_speed, "pstate {ps}");
        for rank in [Rank::R0, Rank::R1] {
            let mut launched = Vec::new();
            wcklevel::measurement_once(rank, &mut |s| {
                if let wcklevel::Step::Launch { rows, .. } = s {
                    launched.extend_from_slice(rows);
                }
            });
            assert_eq!(launched.len(), rows, "pstate {ps}");
            if ps >= 2 {
                assert_eq!(launched[..], measurement_program_low(rank)[..]);
            }
        }
    }

    #[test]
    fn alt_program_per_pstate() {
        use crate::phy::wcklevel::measurement_program_ps0;
        assert_eq!([0, 2, 3].map(alt_rows_at), [272, 112, 112]);
        for rank in [Rank::R0, Rank::R1] {
            let got = alt_program(measurement_program_ps0(rank), rank);
            if crate::pstate::ACTIVE >= 2 {
                assert_eq!(got[..], measurement_program_low(rank)[..]);
            } else {
                assert_eq!(got[..], measurement_program_ps0(rank)[..]);
            }
        }
    }
}
