// SPDX-License-Identifier: GPL-2.0-only
//! P3-13: the plain training cache: record round trip, corruption and key
//! checks, and the boot decision with its fall-backs.

use dram_train::cache::{
    boot, decode, encode, CacheError, DataTest, Field, Key, Outcome, Policy, Restore, WarmFailure,
    Why, HEADER_BYTES, KEY_BYTES,
};

/// CRC-32 (IEEE, reflected), as storage::crc::crc32.
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
        board: 0x5232_5300, // "R2S"
        crate_version: 1 << 16 | 2 << 8,
        ranks: 1,
        density: 0x10,
        mr5_8: [[0x06, 0x01, 0x00, 0x10], [0; 4], [0; 4], [0; 4]],
        freqs: 3,
        freq_mhz: [800, 1066, 1200, 0, 0, 0, 0, 0],
    }
}

#[test]
fn records_round_trip_and_reject_damage() {
    assert_eq!(crc32(b"123456789"), 0xcbf4_3926);
    let payload: Vec<u8> = (0..200u8).collect();
    let mut buf = [0xffu8; 512];
    let n = encode(&key(), &payload, crc32, &mut buf).unwrap();
    assert_eq!(n, HEADER_BYTES + KEY_BYTES + 200);
    let (k, p) = decode(&buf, crc32).unwrap();
    assert_eq!((k, p), (key(), &payload[..]));
    // Any flipped bit in key or payload fails the CRC.
    for i in HEADER_BYTES..n {
        let mut b = buf;
        b[i] ^= 0x10;
        assert_eq!(decode(&b, crc32), Err(CacheError::Crc), "byte {i}");
    }
    let mut b = buf;
    b[0] = 0xff;
    assert_eq!(decode(&b, crc32), Err(CacheError::Empty));
    assert_eq!(
        decode(&[0xff; 512], crc32),
        Err(CacheError::Empty),
        "erased"
    );
    let mut b = buf;
    b[8] = 9;
    assert_eq!(decode(&b, crc32), Err(CacheError::Format(9)));
    let mut b = buf;
    b[13] = 0x7f; // length far beyond the bytes
    assert_eq!(decode(&b, crc32), Err(CacheError::Length));
    assert_eq!(decode(&buf[..n - 1], crc32), Err(CacheError::Length));
    assert_eq!(
        encode(&key(), &payload, crc32, &mut [0u8; 100]),
        Err(CacheError::Space)
    );
}

#[test]
fn every_key_field_is_checked() {
    let k = key();
    type Edit = fn(&mut Key);
    let cases: [(Edit, Field); 6] = [
        (|k| k.board ^= 1, Field::Board),
        (|k| k.crate_version += 1, Field::CrateVersion),
        (|k| k.ranks = 2, Field::Ranks),
        (|k| k.density = 0x18, Field::Density),
        (|k| k.mr5_8[0][1] = 2, Field::ModeRegisters),
        (|k| k.freq_mhz[2] = 1333, Field::Frequencies),
    ];
    for (f, field) in cases {
        let mut o = k;
        f(&mut o);
        assert_eq!(o.mismatch(&k), Some(field));
    }
    assert_eq!(k.mismatch(&k), None);
}

#[derive(Default)]
struct Board {
    applied: Option<Vec<u8>>,
    warm_fails: bool,
    verify_ok: bool,
    apply_fails: bool,
    log: Vec<&'static str>,
}
impl Restore for Board {
    type Error = &'static str;
    fn apply(&mut self, p: &[u8]) -> Result<(), &'static str> {
        self.log.push("apply");
        if self.apply_fails {
            return Err("apply");
        }
        self.applied = Some(p.to_vec());
        Ok(())
    }
    fn warm(&mut self) -> Result<(), WarmFailure> {
        self.log.push("warm");
        if self.warm_fails {
            Err(WarmFailure::NoGate)
        } else {
            Ok(())
        }
    }
    fn data_test(&mut self) -> Result<DataTest, &'static str> {
        self.log.push("data_test");
        Ok(if self.verify_ok {
            DataTest::Clean
        } else {
            DataTest::Errors
        })
    }
    fn train(&mut self) -> Result<(), &'static str> {
        self.log.push("train");
        Ok(())
    }
}

const NORMAL: Policy = Policy {
    forced: false,
    boots_since_train: 3,
    every: 100,
};

#[test]
fn boot_restores_or_falls_back_to_full_training() {
    let mut rec = [0u8; 300];
    let n = encode(&key(), b"csr+mr+margins", crc32, &mut rec).unwrap();
    let rec = &rec[..n];
    let good = || Board {
        verify_ok: true,
        ..Board::default()
    };

    let mut b = good();
    assert_eq!(
        boot(rec, &key(), crc32, NORMAL, &mut b),
        Ok(Outcome::Restored)
    );
    assert_eq!(b.log, ["apply", "warm", "data_test"]);
    assert_eq!(b.applied.as_deref(), Some(&b"csr+mr+margins"[..]));

    let mut other = key();
    other.freq_mhz[0] = 933;
    let mut b = good();
    assert_eq!(
        boot(rec, &other, crc32, NORMAL, &mut b),
        Ok(Outcome::Trained(Why::Key(Field::Frequencies)))
    );
    assert_eq!(b.log, ["train"]);

    for (mut b, why) in [
        (
            Board {
                apply_fails: true,
                ..good()
            },
            Why::Apply,
        ),
        (
            Board {
                warm_fails: true,
                ..good()
            },
            Why::Warm(WarmFailure::NoGate),
        ),
        (Board::default(), Why::DataTest),
    ] {
        assert_eq!(
            boot(rec, &key(), crc32, NORMAL, &mut b),
            Ok(Outcome::Trained(why))
        );
        assert_eq!(b.log.last(), Some(&"train"));
    }

    let forced = Policy {
        forced: true,
        ..NORMAL
    };
    assert_eq!(
        boot(rec, &key(), crc32, forced, &mut good()),
        Ok(Outcome::Trained(Why::Forced))
    );
    let periodic = Policy {
        boots_since_train: 99,
        ..NORMAL
    };
    assert_eq!(
        boot(rec, &key(), crc32, periodic, &mut good()),
        Ok(Outcome::Trained(Why::Periodic))
    );
    assert_eq!(
        boot(&[0xff; 64], &key(), crc32, NORMAL, &mut good()),
        Ok(Outcome::Trained(Why::Record(CacheError::Empty)))
    );
}
