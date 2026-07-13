// SPDX-License-Identifier: GPL-2.0-only
//! P3-A5: the design-around rules of the temperature-aware cache, enforced
//! (orchestrator PATENT-COMPARISON-A5.md, D1 D4 D5 D6 D7 D9).

use dram_train::cache::{
    CacheError, DataTest, Field, Key, Outcome, Policy, Restore, WarmFailure, Why,
};
use dram_train::cache_temp::{boot, encode_temp, log_of, Band, Copies, Source};

fn crc32(data: &[u8]) -> u32 {
    let mut c = !0u32;
    for &b in data {
        c ^= b as u32;
        for _ in 0..8 {
            c = if c & 1 != 0 {
                (c >> 1) ^ 0xedb8_8320
            } else {
                c >> 1
            };
        }
    }
    !c
}

fn key() -> Key {
    Key {
        board: 0x4137_4100, // "A7A"
        crate_version: 1 << 16 | 3 << 8,
        ranks: 2,
        density: 0x0c,
        mr5_8: [
            [0x01, 0x05, 0x00, 0x0c],
            [0x01, 0x05, 0x00, 0x0c],
            [0; 4],
            [0; 4],
        ],
        freqs: 4,
        freq_mhz: [2400, 1200, 800, 400, 0, 0, 0, 0],
    }
}

fn record(band: u8, payload: &[u8], log: &[u8]) -> Vec<u8> {
    let mut scratch = [0u8; 512];
    let mut out = [0u8; 600];
    let n = encode_temp(&key(), band, payload, log, crc32, &mut scratch, &mut out).unwrap();
    out[..n].to_vec()
}

/// A board that records every call; its band reading is set per test.
#[derive(Default)]
struct Board {
    band: Option<u8>,
    applied: Vec<Vec<u8>>,
    calls: Vec<&'static str>,
}
impl Restore for Board {
    type Error = &'static str;
    fn apply(&mut self, p: &[u8]) -> Result<(), &'static str> {
        self.calls.push("apply");
        self.applied.push(p.to_vec());
        Ok(())
    }
    fn warm(&mut self) -> Result<(), WarmFailure> {
        self.calls.push("warm");
        Ok(())
    }
    // The data test takes nothing: no stored margin, no offset, no width.
    fn data_test(&mut self) -> Result<DataTest, &'static str> {
        self.calls.push("data_test");
        Ok(DataTest::Clean)
    }
    fn train(&mut self) -> Result<(), &'static str> {
        self.calls.push("train");
        Ok(())
    }
}
impl Band for Board {
    fn band(&mut self) -> Result<u8, &'static str> {
        self.calls.push("band");
        self.band.ok_or("mr4")
    }
}

const P: Policy = Policy {
    forced: false,
    boots_since_train: 1,
    every: 64,
};

fn run(copies: Copies, now: Option<u8>, tol: u8) -> (Outcome, Option<Source>, Board) {
    let mut b = Board {
        band: now,
        ..Board::default()
    };
    let r = boot(copies, &key(), crc32, P, tol, &mut b).unwrap();
    (r.outcome, r.copy, b)
}

#[test]
fn a_band_change_beyond_the_tolerance_always_invalidates() {
    for tol in 0..3u8 {
        for stored in 0..8u8 {
            for now in 0..8u8 {
                let rec = record(stored, b"regs", b"");
                let (o, copy, b) = run(
                    Copies {
                        rolling: &rec,
                        golden: None,
                    },
                    Some(now),
                    tol,
                );
                if stored.abs_diff(now) > tol {
                    assert_eq!(
                        o,
                        Outcome::Trained(Why::Band {
                            stored,
                            now: Some(now)
                        })
                    );
                    assert!(b.applied.is_empty(), "nothing restored");
                    assert_eq!(copy, None);
                } else {
                    assert_eq!(o, Outcome::Restored);
                    assert_eq!(b.applied, [b"regs".to_vec()], "values never adjusted");
                }
            }
        }
    }
}

#[test]
fn the_band_never_selects_a_copy() {
    // Rolling intact with band 3, golden intact with band 5, now 5: the
    // rolling copy is chosen (integrity and identity), its band does not
    // match, so: retrain. The matching golden copy is not used.
    let rolling = record(3, b"rolling", b"");
    let golden = record(5, b"golden", b"");
    let (o, copy, b) = run(
        Copies {
            rolling: &rolling,
            golden: Some(&golden),
        },
        Some(5),
        0,
    );
    assert_eq!(
        o,
        Outcome::Trained(Why::Band {
            stored: 3,
            now: Some(5)
        })
    );
    assert_eq!((copy, b.applied.len()), (None, 0));
    // A damaged rolling copy falls back to the golden one by integrity.
    let mut bad = rolling.clone();
    bad[40] ^= 1;
    let (o, copy, b) = run(
        Copies {
            rolling: &bad,
            golden: Some(&golden),
        },
        Some(5),
        0,
    );
    assert_eq!((o, copy), (Outcome::Restored, Some(Source::Golden)));
    assert_eq!(b.applied, [b"golden".to_vec()]);
    // ... and the golden copy's band still only invalidates.
    let (o, _, _) = run(
        Copies {
            rolling: &bad,
            golden: Some(&golden),
        },
        Some(1),
        0,
    );
    assert_eq!(
        o,
        Outcome::Trained(Why::Band {
            stored: 5,
            now: Some(1)
        })
    );
    // Nothing usable: the rolling copy's reason.
    let (o, _, _) = run(
        Copies {
            rolling: &bad,
            golden: None,
        },
        Some(5),
        0,
    );
    assert_eq!(o, Outcome::Trained(Why::Record(CacheError::Crc)));
}

#[test]
fn stored_margins_never_reach_the_decision() {
    // The same record with different log sections (margins): the same
    // decision and calls, and the board only ever sees the payload.
    let mut seen = Vec::new();
    for log in [
        &b""[..],
        b"margins: wide",
        b"margins: closed 0 0 0",
        &[0xff; 200],
    ] {
        let rec = record(2, b"regs", log);
        assert_eq!(log_of(&rec, crc32).unwrap(), log, "kept for diagnostics");
        let (o, copy, b) = run(
            Copies {
                rolling: &rec,
                golden: None,
            },
            Some(2),
            0,
        );
        assert_eq!(b.applied, [b"regs".to_vec()]);
        seen.push((o, copy, b.calls));
    }
    assert!(seen.windows(2).all(|w| w[0] == w[1]), "{seen:?}");
}

#[test]
fn the_retrain_happens_in_the_same_boot() {
    let rec = record(2, b"regs", b"");
    for (now, want) in [
        (
            None,
            Why::Band {
                stored: 2,
                now: None,
            },
        ),
        (
            Some(6),
            Why::Band {
                stored: 2,
                now: Some(6),
            },
        ),
    ] {
        let (o, _, b) = run(
            Copies {
                rolling: &rec,
                golden: None,
            },
            now,
            0,
        );
        assert_eq!(o, Outcome::Trained(want));
        assert_eq!(b.calls.last(), Some(&"train"));
        assert_eq!(b.calls.iter().filter(|c| **c == "train").count(), 1);
    }
    // The band is read before anything is restored.
    let (_, _, b) = run(
        Copies {
            rolling: &rec,
            golden: None,
        },
        Some(2),
        0,
    );
    assert_eq!(b.calls, ["band", "apply", "warm", "data_test"]);
}

/// The retrain reasons are a closed list: this match has no wildcard, so a
/// new reason fails to compile here until it is reviewed against
/// PATENT-COMPARISON-A5 (functional failures only, never a width).
#[test]
fn the_retrain_reasons_are_a_closed_list() {
    fn functional(w: Why) -> bool {
        match w {
            Why::Forced | Why::Periodic | Why::Apply | Why::DataTest => true,
            Why::Record(_) | Why::Key(_) | Why::Band { .. } => true,
            Why::Warm(WarmFailure::NoGate) | Why::Warm(WarmFailure::Timeout) => true,
        }
    }
    assert!(functional(Why::Key(Field::Board)));
    let forced = Policy { forced: true, ..P };
    let rec = record(2, b"regs", b"");
    let mut b = Board {
        band: Some(2),
        ..Board::default()
    };
    let r = boot(
        Copies {
            rolling: &rec,
            golden: None,
        },
        &key(),
        crc32,
        forced,
        0,
        &mut b,
    )
    .unwrap();
    assert_eq!(r.outcome, Outcome::Trained(Why::Forced));
}
