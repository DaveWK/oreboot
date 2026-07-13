// SPDX-License-Identifier: GPL-2.0-only
//! tlv-eeprom against the verbatim K1 eeprom.rs: the K1 parse rebuilt on
//! the crate (as the board does it) returns what the old parse returns, on
//! valid, truncated, mutated and random records. The copy's own tests (TLV
//! parse, truncation, corruption, CRC vector, profile) run through it too.

use tlv_eeprom::{exact, walk, write_record, Once, ParseError};

// Host stand-ins for the board imports.
mod log {
    macro_rules! println {
        ($($t:tt)*) => {
            let _ = format_args!($($t)*);
        };
    }
    pub(crate) use println;
}
mod util {
    pub mod mmio {
        pub fn read32(_: usize) -> u32 {
            0
        }
        pub fn write32(_: usize, _: u32) {}
    }
}

#[allow(dead_code, unused_imports, clippy::all)]
mod k1 {
    use super::{log, util};
    include!("legacy/k1_eeprom.rs");
}

/// The K1 DDR TLVs on the crate (board code in Phase 2).
fn k1_parse_on_crate(e: &[u8]) -> Result<(Option<u32>, Option<u32>), ParseError> {
    let (mut cs, mut rate) = (Once::default(), Once::default());
    walk(e, 256, |code, v| match code {
        0x41 => cs.set(exact(v, 1)?[0] as u32),
        0x43 => {
            let v = exact(v, 2)?;
            rate.set(u16::from_be_bytes([v[0], v[1]]) as u32)
        }
        _ => Ok(()),
    })?;
    Ok((cs.0, rate.0))
}

fn legacy(e: &[u8]) -> Result<(Option<u32>, Option<u32>), ParseError> {
    k1::parse(e)
        .map(|i| (i.cs_num, i.data_rate))
        .map_err(|e| match e {
            k1::ParseError::Header => ParseError::Header,
            k1::ParseError::Length => ParseError::Length,
            k1::ParseError::Crc => ParseError::Crc,
            k1::ParseError::Tlv => ParseError::Tlv,
            k1::ParseError::Duplicate => ParseError::Duplicate,
        })
}

struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }
}

fn random_tlvs(rng: &mut Rng) -> Vec<u8> {
    let mut t = Vec::new();
    for _ in 0..rng.next() % 6 {
        let code = [0x21, 0x41, 0x43, 0x41, 0x43, 0xfd, 0xfe][(rng.next() % 7) as usize];
        let n = match rng.next() % 4 {
            0 => 1,
            1 => 2,
            _ => (rng.next() % 9) as usize,
        };
        t.push(code);
        t.push(n as u8);
        for _ in 0..n {
            t.push(rng.next() as u8);
        }
    }
    t
}

#[test]
fn matches_on_valid_and_mutated_records() {
    let mut rng = Rng(0x5eed_71f0);
    let mut out = [0u8; 300];
    for _ in 0..20_000 {
        let tlvs = random_tlvs(&mut rng);
        let n = write_record(&tlvs, &mut out).unwrap();
        let rec = &out[..n];
        assert_eq!(k1_parse_on_crate(rec), legacy(rec));
        // truncation, a flipped byte, a random length field
        let cut = (rng.next() as usize) % (n + 1);
        assert_eq!(k1_parse_on_crate(&rec[..cut]), legacy(&rec[..cut]));
        let mut bad = rec.to_vec();
        let i = (rng.next() as usize) % n;
        bad[i] ^= 1 << (rng.next() % 8);
        assert_eq!(k1_parse_on_crate(&bad), legacy(&bad));
        let mut len = rec.to_vec();
        len[9] = rng.next() as u8;
        len[10] = rng.next() as u8;
        assert_eq!(k1_parse_on_crate(&len), legacy(&len));
    }
}

#[test]
fn matches_on_random_bytes_with_a_valid_header() {
    let mut rng = Rng(0x5eed_00ee);
    for _ in 0..20_000 {
        let mut e: Vec<u8> = b"TlvInfo\0\x01".to_vec();
        let len = (rng.next() % 300) as usize;
        e.extend_from_slice(&(len as u16).to_be_bytes());
        for _ in 0..(rng.next() % 320) {
            e.push(rng.next() as u8);
        }
        assert_eq!(k1_parse_on_crate(&e), legacy(&e));
    }
}

#[test]
fn total_len_and_writer() {
    let mut out = [0u8; 64];
    let n = write_record(&[0x21, 3, b'a', b'b', b'c'], &mut out).unwrap();
    assert_eq!(tlv_eeprom::total_len(&out[..11]), Some(n));
    assert_eq!(tlv_eeprom::total_len(b"TlvInfo\0\x02\0\0"), None);
    assert!(write_record(&[0; 60], &mut out).is_none());
    let mut seen = Vec::new();
    walk(&out[..n], 256, |c, v| {
        seen.push((c, v.to_vec()));
        Ok(())
    })
    .unwrap();
    assert_eq!(seen, [(0x21, b"abc".to_vec())]);
}
