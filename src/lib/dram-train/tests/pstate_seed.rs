// SPDX-License-Identifier: GPL-2.0-only
//! P3-A6: cross-pstate seeding on a delay plant (fixed ps plus whole UIs,
//! codes in UI/64).

use dram_train::pstate_seed::{search_seeded, Model, Seed, Seeded, Trained};

/// UI per pstate (ps): four pstates, top first.
const UI: [f64; 4] = [104.2, 208.3, 312.5, 625.0];

fn true_code(t_ps: f64, n: i32, ui: f64) -> i32 {
    ((t_ps / ui + n as f64) * 64.0).round() as i32
}

#[test]
fn two_trained_pstates_predict_the_others_within_a_code() {
    for (t_ps, n) in [(180.0, 2), (35.5, 1), (410.0, 0), (75.0, 3)] {
        let tr = [UI[0], UI[1]].map(|ui| Trained {
            ui_ps: ui,
            code: true_code(t_ps, n, ui),
        });
        let m = Model::fit(&tr, 64).unwrap();
        assert_eq!(m.n_ui, n, "t {t_ps}");
        assert!((m.t_ps - t_ps).abs() < 1.5, "t {t_ps}: {}", m.t_ps);
        for ui in UI {
            assert!(
                (m.code(ui) - true_code(t_ps, n, ui)).abs() <= 1,
                "t {t_ps} ui {ui}"
            );
        }
    }
}

#[test]
fn one_pstate_with_the_latency_known() {
    let t = Trained {
        ui_ps: UI[0],
        code: true_code(180.0, 2, UI[0]),
    };
    let m = Model::from_one(t, 2, 64).unwrap();
    for ui in UI {
        assert!((m.code(ui) - true_code(180.0, 2, ui)).abs() <= 1);
    }
    assert!(Model::from_one(t, 2, 0).is_none());
    assert!(Model::fit(&[t], 64).is_none());
    assert!(Model::fit(&[t, t], 64).is_none(), "the same UI twice");
}

#[test]
fn the_bracket_cuts_the_search_and_falls_back_when_wrong() {
    let m = Model {
        t_ps: 180.0,
        n_ui: 2,
        fine_per_ui: 64,
    };
    let s = m.seed(UI[3], 20.0, 0, 1023);
    let truth = true_code(180.0, 2, UI[3]);
    assert!(s.lo <= truth && truth <= s.hi);
    assert!(s.hi - s.lo <= 6, "{s:?}: a few codes instead of 1024");
    // A plant search: the trained code if it is in the range.
    let mut probes = 0;
    let search = |t: i32| {
        move |lo: i32, hi: i32| -> Result<Option<i32>, ()> {
            Ok((lo..=hi).contains(&t).then_some(t))
        }
    };
    assert_eq!(
        search_seeded(s, 0, 1023, |lo, hi| {
            probes += 1;
            search(truth)(lo, hi)
        }),
        Ok(Seeded::InBracket(truth))
    );
    assert_eq!(probes, 1);
    // The board moved (a different part): the bracket misses, the full
    // range finds it.
    assert_eq!(
        search_seeded(s, 0, 1023, search(truth + 40)),
        Ok(Seeded::Full(Some(truth + 40)))
    );
    // A result on the bracket's edge may lie beyond it: full search.
    assert_eq!(
        search_seeded(s, 0, 1023, search(s.hi)),
        Ok(Seeded::Full(Some(s.hi)))
    );
    // At the range end the bracket edge is final.
    let end = Seed {
        code: 2,
        lo: 0,
        hi: 4,
    };
    assert_eq!(
        search_seeded(end, 0, 1023, search(0)),
        Ok(Seeded::InBracket(0))
    );
}
