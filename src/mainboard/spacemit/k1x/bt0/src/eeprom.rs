//! Read the board's ONIE TLV EEPROM (a 24c02 at 0x50 on TWSI2) for the DRAM
//! settings SpacemiT's U-Boot SPL takes from it. Polled, byte at a time, as
//! U-Boot's drivers/i2c/k1_i2c.c; every wait is bounded so a missing or
//! silent EEPROM only costs the fallback.

use log::println;
use util::mmio::{read32, write32};

const TWSI2_BASE: usize = 0xd401_2000;
const ICR: usize = TWSI2_BASE + 0x00;
const ISR: usize = TWSI2_BASE + 0x04;
const IDBR: usize = TWSI2_BASE + 0x0c;

const ICR_START: u32 = 1 << 0;
const ICR_STOP: u32 = 1 << 1;
const ICR_ACKNAK: u32 = 1 << 2;
const ICR_TB: u32 = 1 << 3;
const ICR_UR: u32 = 1 << 10;
const ICR_SCLE: u32 = 1 << 13;
const ICR_IUE: u32 = 1 << 14;
const ICR_ALDIE: u32 = 1 << 18;
const ICR_ITEIE: u32 = 1 << 19;
const ICR_IRFIE: u32 = 1 << 20;
const ICR_GCD: u32 = 1 << 21;
const ICR_BEIE: u32 = 1 << 22;
const ICR_MODE_MASK: u32 = 0x300;
// Standard mode (100 kHz).
const ICR_INIT: u32 = ICR_BEIE | ICR_IRFIE | ICR_ITEIE | ICR_GCD | ICR_SCLE;

const ISR_ACKNAK: u32 = 1 << 14;
const ISR_IBB: u32 = 1 << 16;
const ISR_ITE: u32 = 1 << 19;
const ISR_IRF: u32 = 1 << 20;
const ISR_INIT: u32 = 0x1FD_E000;

const EEPROM_ADDR: u8 = 0x50;
const EEPROM_SIZE: usize = 256;

// ONIE TLV codes; 0x41.. are SpacemiT's (U-Boot include/configs/k1-x.h).
const TLV_CODE_DDR_CSNUM: u8 = 0x41;
const TLV_CODE_DDR_DATARATE: u8 = 0x43;
// The TLV record format (header, CRC, walk) is the shared tlv_eeprom crate
// (ore-edk2-boot); the K1 DRAM records above stay here.
pub use tlv_eeprom::ParseError;
use tlv_eeprom::{exact, walk, Once, HEADER_LEN as TLV_HEADER};

// About 100 us or more of bus wait at any core clock bt0 runs at.
const SPIN: u32 = 5_000;
// Per-byte status wait; a byte at 100 kHz takes about 90 us.
const POLL: u32 = 200_000;

fn spin(n: u32) {
    for _ in 0..n {
        let _ = read32(ISR);
    }
}

fn reset() {
    let mode = read32(ICR) & ICR_MODE_MASK;
    write32(ICR, read32(ICR) & !ICR_IUE);
    spin(SPIN / 10);
    write32(ICR, read32(ICR) | ICR_UR);
    spin(SPIN);
    write32(ICR, read32(ICR) & !ICR_IUE);
    write32(ISR, 0);
    write32(ICR, ICR_INIT | mode);
    write32(ISR, ISR_INIT);
    write32(ICR, read32(ICR) | ICR_IUE);
    spin(SPIN);
}

fn wait(set: u32, clear: u32) -> Result<(), ()> {
    for _ in 0..POLL {
        let isr = read32(ISR);
        if isr & set == set && isr & clear == 0 {
            return Ok(());
        }
    }
    Err(())
}

#[derive(PartialEq)]
enum Cond {
    Normal,
    Start,
    Stop,
}

#[derive(PartialEq)]
enum Ack {
    Wait,
    Send,
    Nak,
}

/// One byte on the bus; returns the byte read (reads) or 0 (writes).
fn transfer(write: Option<u8>, cond: Cond, ack: Ack) -> Result<u8, ()> {
    let r = (|| {
        wait(0, ISR_IBB)?;
        write32(ICR, read32(ICR) & !(ICR_START | ICR_STOP));
        if let Some(b) = write {
            write32(IDBR, b as u32);
        }
        let mut icr = read32(ICR);
        match cond {
            Cond::Start => icr |= ICR_START,
            Cond::Stop => icr |= ICR_STOP,
            Cond::Normal => {}
        }
        match ack {
            Ack::Nak => icr |= ICR_ACKNAK,
            Ack::Send => icr &= !ICR_ACKNAK,
            Ack::Wait => {}
        }
        write32(ICR, (icr & !ICR_ALDIE) | ICR_TB);
        if write.is_some() {
            wait(ISR_ITE, 0)?;
            write32(ISR, read32(ISR) | ISR_ITE);
            if ack == Ack::Wait {
                wait(0, ISR_ACKNAK)?;
            }
            Ok(0)
        } else {
            wait(ISR_IRF, 0)?;
            let b = read32(IDBR) as u8;
            write32(ISR, read32(ISR) | ISR_IRF);
            Ok(b)
        }
    })();
    if r.is_err() {
        reset();
    }
    r
}

fn read(offset: u8, buf: &mut [u8]) -> Result<(), ()> {
    reset();
    transfer(Some(EEPROM_ADDR << 1), Cond::Start, Ack::Wait)?;
    transfer(Some(offset), Cond::Normal, Ack::Wait)?;
    transfer(Some((EEPROM_ADDR << 1) | 1), Cond::Start, Ack::Wait)?;
    let n = buf.len();
    for (i, b) in buf.iter_mut().enumerate() {
        *b = if i + 1 == n {
            transfer(None, Cond::Stop, Ack::Nak)?
        } else {
            transfer(None, Cond::Normal, Ack::Send)?
        };
    }
    Ok(())
}

/// DRAM settings from the EEPROM, where present.
#[derive(Default, Debug, PartialEq, Eq)]
pub struct DdrInfo {
    pub cs_num: Option<u32>,
    pub data_rate: Option<u32>,
}

/// Read and check the TLV EEPROM. TWSI2 must be set up (eeprom_i2c_init).
pub fn ddr_info() -> DdrInfo {
    let info = DdrInfo::default();
    let mut e = [0u8; EEPROM_SIZE];
    if read(0, &mut e[..TLV_HEADER]).is_err() {
        println!("[bt0] EEPROM: no response");
        return info;
    }
    let Some(total) = tlv_eeprom::total_len(&e[..TLV_HEADER]) else {
        println!("[bt0] EEPROM: no TLV header");
        return info;
    };
    let len = total - TLV_HEADER;
    if total > EEPROM_SIZE {
        println!("[bt0] EEPROM: TLV length {len} too large");
        return info;
    }
    if read(TLV_HEADER as u8, &mut e[TLV_HEADER..TLV_HEADER + len]).is_err() {
        println!("[bt0] EEPROM: read failed");
        return info;
    }
    match parse(&e[..TLV_HEADER + len]) {
        Ok(v) => v,
        Err(e) => {
            println!("[bt0] EEPROM invalid ({e:?}); using board profile");
            info
        }
    }
}

/// Validate the entire TLV record before returning any settings. Missing/invalid
/// EEPROMs fall back only to the explicit, size-checked board profile.
pub fn parse(e: &[u8]) -> Result<DdrInfo, ParseError> {
    let (mut cs_num, mut data_rate) = (Once(None), Once(None));
    walk(e, EEPROM_SIZE, |code, v| match code {
        TLV_CODE_DDR_CSNUM => cs_num.set(exact(v, 1)?[0] as u32),
        TLV_CODE_DDR_DATARATE => {
            let v = exact(v, 2)?;
            data_rate.set(u16::from_be_bytes([v[0], v[1]]) as u32)
        }
        _ => Ok(()),
    })?;
    Ok(DdrInfo {
        cs_num: cs_num.0,
        data_rate: data_rate.0,
    })
}
#[cfg(test)]
mod tests {
    use super::*;
    use storage::crc::crc32;
    fn record(payload: &[u8]) -> std::vec::Vec<u8> {
        let mut e = b"TlvInfo\0\x01".to_vec();
        e.extend_from_slice(&((payload.len() + 6) as u16).to_be_bytes());
        e.extend_from_slice(payload);
        e.extend_from_slice(&[0xfe, 4]);
        let crc = crc32(&e);
        e.extend_from_slice(&crc.to_be_bytes());
        e
    }
    #[test]
    fn valid_rank_rate_and_unknown_tlv() {
        let e = record(&[0x21, 3, b'f', b'o', b'o', 0x41, 1, 1, 0x43, 2, 9, 0x60]);
        assert_eq!(
            parse(&e),
            Ok(DdrInfo {
                cs_num: Some(1),
                data_rate: Some(2400)
            })
        );
        assert_eq!(parse(&record(&[])), Ok(DdrInfo::default()));
    }
    #[test]
    fn truncated_every_byte() {
        let e = record(&[0x41, 1, 1]);
        for n in 0..e.len() {
            assert!(parse(&e[..n]).is_err());
        }
    }
    #[test]
    fn malformed_tlv_never_returns_partial_rank() {
        for payload in [
            &[0x41, 1, 1, 0x43, 2, 9][..],
            &[0x41, 1, 1, 0x42][..],
            &[0x41, 1, 1, 0x41, 1, 2][..],
            &[0x41, 0][..],
            &[0xfe, 0][..],
        ] {
            assert!(parse(&record(payload)).is_err());
        }
    }
    #[test]
    fn crc_and_header_corruption() {
        let e = record(&[0x41, 1, 1]);
        for i in 0..e.len() {
            let mut bad = e.clone();
            bad[i] ^= 1;
            assert!(parse(&bad).is_err());
        }
    }
    #[test]
    fn crc_standard_vector() {
        assert_eq!(crc32(b"123456789"), 0xcbf43926);
    }
}

#[derive(Debug, PartialEq, Eq)]
pub enum ProfileError {
    Rank,
    Rate,
}
pub fn resolve_profile(info: &DdrInfo, rank: u32, rate: u32) -> Result<u32, ProfileError> {
    if !matches!(rank, 1 | 2) || info.cs_num.is_some_and(|n| n != rank) {
        return Err(ProfileError::Rank);
    }
    if info.data_rate.is_some_and(|r| r != rate) {
        return Err(ProfileError::Rate);
    }
    Ok(rank)
}
#[cfg(test)]
mod profile_tests {
    use super::*;
    #[test]
    fn explicit_board_fallback_and_mismatch() {
        {
            assert_eq!(resolve_profile(&DdrInfo::default(), 1, 2400), Ok(1));
            assert_eq!(
                resolve_profile(
                    &DdrInfo {
                        cs_num: Some(1),
                        data_rate: Some(2400)
                    },
                    1,
                    2400
                ),
                Ok(1)
            );
        }
        assert_eq!(
            resolve_profile(
                &DdrInfo {
                    cs_num: Some(2),
                    data_rate: None
                },
                1,
                2400
            ),
            Err(ProfileError::Rank)
        );
        for rate in [0, 1200, 1600, 3200] {
            assert_eq!(
                resolve_profile(
                    &DdrInfo {
                        cs_num: None,
                        data_rate: Some(rate)
                    },
                    1,
                    2400
                ),
                Err(ProfileError::Rate)
            );
        }
        assert_eq!(
            resolve_profile(&DdrInfo::default(), 0, 2400),
            Err(ProfileError::Rank)
        );
    }
}
