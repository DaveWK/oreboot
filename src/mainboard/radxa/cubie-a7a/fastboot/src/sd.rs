//! Read-only SD card access over an abstract host controller ([`SdBus`]):
//! card detect, identification, capacity and block reads, for `fetch:sd`.
//!
//! Read-only by construction: every command goes through [`cmd`], which
//! refuses anything outside [`ALLOWED`]: the identification and read
//! commands (0, 2, 3, 7, 8, 9, 13, 16, 17, 18, 12 for the controller's
//! auto stop, 55 and ACMD41). No write (24, 25), erase (32, 33, 38), lock
//! (42), switch (6, ACMD6 bus width), CSD/CID programming (26, 27), write
//! protection (28, 29) or general command (56) can be sent; the unit tests
//! check the list and that a refused command never reaches the bus. The
//! card stays on its power-on 1-bit bus at default speed.
//!
//! The sequence is the one bt0's SD boot (`smhc.rs` `init_card`) runs, plus
//! CMD9 in standby for the capacity.

/// The commands this module can send (ACMD41 is 41 after 55).
pub const ALLOWED: [u32; 13] = [0, 2, 3, 7, 8, 9, 12, 13, 16, 17, 18, 41, 55];

pub fn command_allowed(idx: u32) -> bool {
    ALLOWED.contains(&idx)
}

pub const BLOCK: usize = 512;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Resp {
    None,
    R1,
    R1b,
    /// 136-bit: CID or CSD, as four words, most significant first.
    R2,
    /// OCR, no CRC.
    R3,
    R6,
    R7,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BusError {
    /// No response in time (no card, or the card ignored the command).
    Timeout,
    /// Response or data error (CRC, end bit, FIFO).
    Error,
}

/// The host controller, as identification and reading need it.
pub trait SdBus {
    /// Card-detect switch: Some(true) when a card is in, None if the board
    /// has none.
    fn card_present(&mut self) -> Option<bool>;
    /// Reset the controller and clock the card at the identification
    /// (<= 400 kHz, `ident`) or transfer rate.
    fn clock(&mut self, ident: bool) -> Result<(), BusError>;
    fn cmd(&mut self, idx: u32, arg: u32, resp: Resp) -> Result<[u32; 4], BusError>;
    /// A read command (17 or 18) with its data, `buf.len() / 128` blocks.
    fn read(&mut self, idx: u32, arg: u32, buf: &mut [u32]) -> Result<(), BusError>;
    /// Wait between ACMD41 polls.
    fn delay_ms(&mut self, _ms: u32) {}
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    /// No card: card detect open, or no answer to ACMD41.
    NoCard,
    Refused(u32),
    Bus(u32, BusError),
    Unsupported(&'static str),
    /// Past the end of the card.
    Range,
}

impl Error {
    pub fn message(&self) -> &'static str {
        match self {
            Error::NoCard => "no SD card",
            Error::Refused(_) => "SD command not allowed (read-only)",
            Error::Bus(..) => "SD command failed (see the console)",
            Error::Unsupported(w) => w,
            Error::Range => "read past the end of the SD card",
        }
    }
}

/// Every command passes here.
fn cmd<B: SdBus>(b: &mut B, idx: u32, arg: u32, resp: Resp) -> Result<[u32; 4], Error> {
    if !command_allowed(idx) {
        return Err(Error::Refused(idx));
    }
    b.cmd(idx, arg, resp).map_err(|e| Error::Bus(idx, e))
}

/// Bits [hi:lo] of a 128-bit register given most significant word first.
pub fn bits(r: &[u32; 4], hi: u32, lo: u32) -> u32 {
    let mut v = 0u64;
    for bit in (lo..=hi).rev() {
        let w = r[3 - (bit / 32) as usize];
        v = (v << 1) | ((w >> (bit % 32)) & 1) as u64;
    }
    v as u32
}

/// Capacity in 512-byte blocks from the CSD (versions 1.0 and 2.0).
pub fn csd_blocks(csd: &[u32; 4]) -> Result<u64, Error> {
    match bits(csd, 127, 126) {
        0 => {
            let c_size = bits(csd, 73, 62) as u64;
            let mult = bits(csd, 49, 47);
            let bl_len = bits(csd, 83, 80);
            let bytes = (c_size + 1) << (mult + 2) << bl_len;
            Ok(bytes / BLOCK as u64)
        }
        1 => Ok((bits(csd, 69, 48) as u64 + 1) * 1024),
        _ => Err(Error::Unsupported("unknown CSD structure")),
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Card {
    pub rca: u32,
    pub high_capacity: bool,
    pub blocks: u64,
    pub ocr: u32,
}

impl Card {
    /// Card detect, then identification into the transfer state.
    pub fn identify<B: SdBus>(b: &mut B) -> Result<Card, Error> {
        if b.card_present() == Some(false) {
            return Err(Error::NoCard);
        }
        b.clock(true).map_err(|e| Error::Bus(0, e))?;
        cmd(b, 0, 0, Resp::None)?;
        let v2 = match cmd(b, 8, 0x1aa, Resp::R7) {
            Ok(r) if r[0] & 0xfff == 0x1aa => true,
            Ok(_) => return Err(Error::Unsupported("bad CMD8 echo")),
            Err(Error::Bus(..)) => false,
            Err(e) => return Err(e),
        };
        let hcs = if v2 { 1 << 30 } else { 0 };
        let mut ocr = 0;
        for _ in 0..1000 {
            match cmd(b, 55, 0, Resp::R1) {
                Ok(_) => {}
                Err(Error::Bus(_, BusError::Timeout)) => return Err(Error::NoCard),
                Err(e) => return Err(e),
            }
            ocr = cmd(b, 41, hcs | 0x00ff_8000, Resp::R3)?[0];
            if ocr & (1 << 31) != 0 {
                break;
            }
            b.delay_ms(1);
        }
        if ocr & (1 << 31) == 0 {
            return Err(Error::Unsupported("card stays busy (ACMD41)"));
        }
        let high_capacity = ocr & (1 << 30) != 0;
        cmd(b, 2, 0, Resp::R2)?;
        let rca = cmd(b, 3, 0, Resp::R6)?[0] >> 16;
        let csd = cmd(b, 9, rca << 16, Resp::R2)?;
        let blocks = csd_blocks(&csd)?;
        cmd(b, 7, rca << 16, Resp::R1b)?;
        if !high_capacity {
            cmd(b, 16, BLOCK as u32, Resp::R1)?;
        }
        b.clock(false).map_err(|e| Error::Bus(0, e))?;
        Ok(Card {
            rca,
            high_capacity,
            blocks,
            ocr,
        })
    }

    /// Read `buf.len() / 128` blocks from block `lba`, at most 128 per
    /// command (CMD17 for one, CMD18 with auto stop for more).
    pub fn read<B: SdBus>(&self, b: &mut B, lba: u64, buf: &mut [u32]) -> Result<(), Error> {
        const WORDS: usize = BLOCK / 4;
        if !buf.len().is_multiple_of(WORDS) {
            return Err(Error::Unsupported("partial block"));
        }
        let n = (buf.len() / WORDS) as u64;
        if lba.checked_add(n).is_none_or(|e| e > self.blocks) {
            return Err(Error::Range);
        }
        let mut lba = lba;
        for chunk in buf.chunks_mut(WORDS * 128) {
            let blocks = (chunk.len() / WORDS) as u64;
            let arg = if self.high_capacity {
                lba
            } else {
                lba * BLOCK as u64
            };
            let arg = u32::try_from(arg).map_err(|_| Error::Range)?;
            let idx = if blocks > 1 { 18 } else { 17 };
            if !command_allowed(idx) {
                return Err(Error::Refused(idx));
            }
            b.read(idx, arg, chunk).map_err(|e| Error::Bus(idx, e))?;
            lba += blocks;
        }
        Ok(())
    }
}

#[cfg(test)]
pub(crate) mod model {
    //! A host-side SD card: state machine, CSD, data, and a log of every
    //! command it saw.
    use super::*;
    use std::vec::Vec;

    #[derive(Clone, Copy, PartialEq, Debug)]
    enum State {
        Idle,
        Ready,
        Ident,
        Stby,
        Tran,
    }

    pub struct SdModel {
        pub present: bool,
        pub detect: Option<bool>,
        pub high_capacity: bool,
        pub csd: [u32; 4],
        pub blocks: u64,
        pub log: Vec<u32>,
        pub fail_read_at: Option<u64>,
        state: State,
        app: bool,
        busy_polls: u32,
    }

    impl SdModel {
        /// An SDHC card of `blocks` blocks (CSD 2.0).
        pub fn sdhc(blocks: u64) -> Self {
            let c_size = (blocks / 1024 - 1) as u32;
            // CSD 2.0: structure 01 at [127:126], C_SIZE at [69:48].
            let csd = [
                0x4000_0000,
                (c_size >> 16) & 0x3f,
                (c_size & 0xffff) << 16,
                0,
            ];
            SdModel {
                present: true,
                detect: Some(true),
                high_capacity: true,
                csd,
                blocks,
                log: Vec::new(),
                fail_read_at: None,
                state: State::Idle,
                app: false,
                busy_polls: 3,
            }
        }
        pub fn byte(&self, addr: u64) -> u8 {
            (addr ^ (addr >> 9) ^ (addr >> 17)) as u8
        }
    }

    impl SdBus for SdModel {
        fn card_present(&mut self) -> Option<bool> {
            self.detect
        }
        fn clock(&mut self, _ident: bool) -> Result<(), BusError> {
            Ok(())
        }
        fn cmd(&mut self, idx: u32, arg: u32, _resp: Resp) -> Result<[u32; 4], BusError> {
            assert!(command_allowed(idx), "CMD{idx} reached the card");
            self.log.push(idx);
            if !self.present {
                return if idx == 0 {
                    Ok([0; 4])
                } else {
                    Err(BusError::Timeout)
                };
            }
            let app = core::mem::take(&mut self.app);
            let r = match (idx, self.state) {
                (0, _) => {
                    self.state = State::Idle;
                    [0; 4]
                }
                (8, State::Idle) => [arg & 0xfff, 0, 0, 0],
                (55, _) => {
                    self.app = true;
                    [0x120, 0, 0, 0]
                }
                (41, State::Idle | State::Ready) if app => {
                    self.state = State::Ready;
                    if self.busy_polls > 0 {
                        self.busy_polls -= 1;
                        [0x00ff_8000, 0, 0, 0]
                    } else {
                        let ccs = if self.high_capacity && arg & (1 << 30) != 0 {
                            1 << 30
                        } else {
                            0
                        };
                        [0x80ff_8000 | ccs, 0, 0, 0]
                    }
                }
                (2, State::Ready) => {
                    self.state = State::Ident;
                    [1, 2, 3, 4]
                }
                (3, State::Ident) => {
                    self.state = State::Stby;
                    [0x1234_0000, 0, 0, 0]
                }
                (9, State::Stby) if arg >> 16 == 0x1234 => self.csd,
                (7, State::Stby) if arg >> 16 == 0x1234 => {
                    self.state = State::Tran;
                    [0x700, 0, 0, 0]
                }
                (16, State::Tran) => [0x900, 0, 0, 0],
                _ => return Err(BusError::Error),
            };
            Ok(r)
        }
        fn read(&mut self, idx: u32, arg: u32, buf: &mut [u32]) -> Result<(), BusError> {
            assert!(command_allowed(idx));
            self.log.push(idx);
            if self.state != State::Tran {
                return Err(BusError::Error);
            }
            let lba = if self.high_capacity {
                arg as u64
            } else {
                arg as u64 / 512
            };
            let n = (buf.len() / 128) as u64;
            if lba + n > self.blocks {
                return Err(BusError::Error);
            }
            if self
                .fail_read_at
                .is_some_and(|f| (lba..lba + n).contains(&f))
            {
                return Err(BusError::Error);
            }
            for (i, w) in buf.iter_mut().enumerate() {
                let a = lba * 512 + 4 * i as u64;
                *w = u32::from_le_bytes([
                    self.byte(a),
                    self.byte(a + 1),
                    self.byte(a + 2),
                    self.byte(a + 3),
                ]);
            }
            Ok(())
        }
    }
}

#[cfg(test)]
mod tests {
    use super::model::SdModel;
    use super::*;
    use std::vec;

    #[test]
    fn allow_list_is_read_only() {
        for idx in 0..64 {
            let write_like = matches!(
                idx,
                6 | 20 | 24 | 25 | 26 | 27 | 28 | 29 | 30 | 32 | 33 | 38 | 42 | 56
            );
            if write_like {
                assert!(!command_allowed(idx), "CMD{idx}");
            }
        }
        for idx in ALLOWED {
            assert!(!matches!(idx, 6 | 24 | 25 | 32 | 33 | 38 | 42 | 56));
        }
        // A refused command never reaches the bus.
        let mut m = SdModel::sdhc(1 << 20);
        assert_eq!(cmd(&mut m, 24, 0, Resp::R1), Err(Error::Refused(24)));
        assert_eq!(cmd(&mut m, 6, 0, Resp::R1), Err(Error::Refused(6)));
        assert!(m.log.is_empty());
    }

    #[test]
    fn csd_capacity() {
        // CSD 2.0, 32 GiB card: C_SIZE = 65535 -> 65536 * 1024 blocks.
        let m = SdModel::sdhc(65536 * 1024);
        assert_eq!(csd_blocks(&m.csd), Ok(65536 * 1024));
        // CSD 1.0: C_SIZE 4095, C_SIZE_MULT 7, READ_BL_LEN 10: 2 GiB.
        let mut c = [0u32; 4];
        let set = |c: &mut [u32; 4], hi: u32, lo: u32, v: u32| {
            for bit in lo..=hi {
                if v >> (bit - lo) & 1 != 0 {
                    c[3 - (bit / 32) as usize] |= 1 << (bit % 32);
                }
            }
        };
        set(&mut c, 73, 62, 4095);
        set(&mut c, 49, 47, 7);
        set(&mut c, 83, 80, 10);
        assert_eq!(bits(&c, 73, 62), 4095);
        assert_eq!(csd_blocks(&c), Ok(4096 * 512 * 1024 / 512));
        set(&mut c, 127, 126, 2);
        assert!(csd_blocks(&c).is_err());
    }

    #[test]
    fn identify_and_read() {
        let mut m = SdModel::sdhc(1 << 20);
        let card = Card::identify(&mut m).unwrap();
        assert_eq!(
            (card.rca, card.high_capacity, card.blocks),
            (0x1234, true, 1 << 20)
        );
        assert_eq!(m.log, [0, 8, 55, 41, 55, 41, 55, 41, 55, 41, 2, 3, 9, 7]);
        let mut buf = vec![0u32; 128 * 200];
        card.read(&mut m, 1000, &mut buf).unwrap();
        // 200 blocks: one CMD18 of 128 and one of 72.
        assert_eq!(&m.log[14..], [18, 18]);
        let b0 = buf[0].to_le_bytes()[1];
        assert_eq!(b0, m.byte(1000 * 512 + 1));
        let last = buf[128 * 200 - 1].to_le_bytes()[3];
        assert_eq!(last, m.byte(1200 * 512 - 1));
        card.read(&mut m, 5, &mut buf[..128]).unwrap();
        assert_eq!(*m.log.last().unwrap(), 17);
    }

    #[test]
    fn no_card_short_card_and_errors() {
        // Card detect open: nothing is sent at all.
        let mut m = SdModel::sdhc(1 << 20);
        m.detect = Some(false);
        assert_eq!(Card::identify(&mut m), Err(Error::NoCard));
        assert!(m.log.is_empty());
        // No detect switch and no card (the board's dead slot): CMD0, CMD8
        // and CMD55 time out -> NoCard.
        let mut m = SdModel::sdhc(1 << 20);
        m.detect = None;
        m.present = false;
        assert_eq!(Card::identify(&mut m), Err(Error::NoCard));
        assert_eq!(m.log, [0, 8, 55]);
        // Short card: reads past the end are refused before the bus.
        let mut m = SdModel::sdhc(2048);
        let card = Card::identify(&mut m).unwrap();
        let n = m.log.len();
        let mut buf = vec![0u32; 128 * 2];
        assert_eq!(card.read(&mut m, 2047, &mut buf), Err(Error::Range));
        assert_eq!(card.read(&mut m, u64::MAX, &mut buf), Err(Error::Range));
        assert_eq!(m.log.len(), n);
        card.read(&mut m, 2046, &mut buf).unwrap();
        // A read error mid-way is reported, not hidden.
        m.fail_read_at = Some(10);
        let mut big = vec![0u32; 128 * 300];
        assert_eq!(
            card.read(&mut m, 0, &mut big[..128 * 20]),
            Err(Error::Bus(18, BusError::Error))
        );
        assert!(
            card.read(&mut m, 0, &mut big[..100]).is_err(),
            "partial block"
        );
    }
}
