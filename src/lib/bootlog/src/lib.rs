// SPDX-License-Identifier: GPL-2.0-only
//! Firmware console log in RAM, so a boot can be read back (from the next
//! stage, EDK2 or Linux) when no UART is attached, and boot-progress markers
//! that let a first stage notice that the previous boot never finished.
//!
//! [`ring`]: the A7A log area format and the operations bt0/main use on it
//! (header check and set-up, append, copy to a "previous boot" area, early
//! buffer hand-over), over an [`Area`] accessor, plus [`ring::read`], the
//! reader. [`progress`]: the attempt marker from the A7A recovery path
//! ("previous boot never confirmed"), generalised to milestones, and
//! [`progress::stamps`], a timestamp per milestone (P3-01).
//!
//! From ore-edk-boot-radxa-a7a oreboot 4904019 `common/src/fwlog.rs` and
//! `bt0/src/recovery.rs` + `common/src/lib.rs` `mod recovery`. The A7A
//! addresses, the log sink selection and the watchdog stay board code.
#![no_std]
#[cfg(test)]
extern crate std;

/// Byte and aligned 32-bit word access to one log area, at offsets from its
/// base. Firmware implements it with volatile accesses (with the MMU off,
/// unaligned accesses fault, so words are always 4-byte aligned).
pub trait Area {
    fn read8(&self, off: usize) -> u8;
    fn write8(&mut self, off: usize, v: u8);
    fn read32(&self, off: usize) -> u32;
    fn write32(&mut self, off: usize, v: u32);
}

pub mod ring {
    //! Layout of a log area (little endian; EDK2's OrebootLogSerialPortLib
    //! and the Linux reader `a7a-fwlog` mirror it):
    //!
    //! | Offset | Size | Field |
    //! |---|---|---|
    //! | 0  | 8 | magic (A7A: `"A7AFWLOG"`) |
    //! | 8  | 4 | version (1) |
    //! | 12 | 4 | data size in bytes (area size - 64) |
    //! | 16 | 8 | bytes written so far; the ring position is `written % size` |
    //! | 24 | 4 | boot sequence number, +1 per boot while the log survives |
    //! | 28 | 4 | flags, 0 |
    //! | 32 | 32 | reserved, 0 |
    //! | 64 | size | ring data |
    use super::Area;

    pub const VERSION: u32 = 1;
    pub const HEADER_SIZE: usize = 64;
    const OFF_VERSION: usize = 8;
    const OFF_SIZE: usize = 12;
    const OFF_WRITTEN: usize = 16;
    const OFF_SEQ: usize = 24;

    /// One log format: its magic and the data size of its areas.
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    pub struct Format {
        pub magic: [u8; 8],
        pub data_size: usize,
    }

    /// The A7A areas: magic "A7AFWLOG", 256 KiB areas.
    pub const A7A: Format = Format {
        magic: *b"A7AFWLOG",
        data_size: 0x0004_0000 - HEADER_SIZE,
    };

    fn r64(a: &impl Area, off: usize) -> u64 {
        a.read32(off) as u64 | (a.read32(off + 4) as u64) << 32
    }
    fn w64(a: &mut impl Area, off: usize, v: u64) {
        a.write32(off, v as u32);
        a.write32(off + 4, (v >> 32) as u32);
    }

    /// The area holds a log of this format.
    pub fn header_valid(a: &impl Area, f: &Format) -> bool {
        (0..8).all(|i| a.read8(i) == f.magic[i])
            && a.read32(OFF_VERSION) == VERSION
            && a.read32(OFF_SIZE) as usize == f.data_size
    }

    /// Start an empty log with boot sequence `seq`.
    pub fn init_header(a: &mut impl Area, f: &Format, seq: u32) {
        for i in 0..HEADER_SIZE / 4 {
            a.write32(4 * i, 0);
        }
        a.write32(OFF_VERSION, VERSION);
        a.write32(OFF_SIZE, f.data_size as u32);
        w64(a, OFF_WRITTEN, 0);
        a.write32(OFF_SEQ, seq);
        // Magic last, so a reader never sees a valid header over stale fields.
        for (i, &b) in f.magic.iter().enumerate() {
            a.write8(i, b);
        }
    }

    /// Append one byte.
    pub fn putc(a: &mut impl Area, f: &Format, c: u8) {
        let n = r64(a, OFF_WRITTEN);
        a.write8(HEADER_SIZE + (n % f.data_size as u64) as usize, c);
        w64(a, OFF_WRITTEN, n + 1);
    }

    /// Bytes written since the log started.
    pub fn written(a: &impl Area) -> u64 {
        r64(a, OFF_WRITTEN)
    }

    /// Boot sequence number of the log.
    pub fn seq(a: &impl Area) -> u32 {
        a.read32(OFF_SEQ)
    }

    /// Copy a whole area word by word (the previous boot's log, kept before
    /// a new one starts).
    pub fn copy(from: &impl Area, to: &mut impl Area, f: &Format) {
        for i in (0..HEADER_SIZE + f.data_size).step_by(4) {
            to.write32(i, from.read32(i));
        }
    }

    /// Make the area unusable as a log (a stale copy must not pass for the
    /// last boot).
    pub fn invalidate(a: &mut impl Area) {
        a.write32(0, 0);
    }

    /// The new boot's log in `cur`: if `cur` held a valid log, copy it to
    /// `prev` and continue its sequence number, else invalidate `prev`; then
    /// start `cur` empty. Returns the new sequence number.
    pub fn rotate(cur: &mut impl Area, prev: &mut impl Area, f: &Format) -> u32 {
        let mut seq = 0;
        if header_valid(cur, f) {
            seq = self::seq(cur).wrapping_add(1);
            copy(cur, prev, f);
        } else {
            invalidate(prev);
        }
        init_header(cur, f, seq);
        seq
    }

    /// Read the log in order into `out`: the last `out.len()` bytes if the
    /// ring wrapped or `out` is short. Returns the bytes copied, or None
    /// when the area holds no log of this format.
    pub fn read(a: &impl Area, f: &Format, out: &mut [u8]) -> Option<usize> {
        if !header_valid(a, f) {
            return None;
        }
        let n = written(a);
        let kept = n.min(f.data_size as u64);
        let take = kept.min(out.len() as u64);
        let first = n - take;
        for (k, o) in out[..take as usize].iter_mut().enumerate() {
            let pos = (first + k as u64) % f.data_size as u64;
            *o = a.read8(HEADER_SIZE + pos as usize);
        }
        Some(take as usize)
    }

    /// The early linear buffer bt0 uses before DRAM works: its length is in
    /// the first word, the bytes follow. Append one byte (dropped when full).
    pub fn early_putc(a: &mut impl Area, size: usize, c: u8) {
        let n = a.read32(0) as usize;
        if 4 + n < size {
            a.write8(4 + n, c);
            a.write32(0, (n + 1) as u32);
        }
    }

    /// Move the early buffer's bytes into the ring.
    pub fn drain_early(early: &impl Area, size: usize, ring: &mut impl Area, f: &Format) {
        let n = (early.read32(0) as usize).min(size - 4);
        for i in 0..n {
            putc(ring, f, early.read8(4 + i));
        }
    }
}

pub mod progress {
    //! Boot-progress markers in a register or word that survives a reset
    //! (A7A: RTC GP_DATA_REG6). bt0 sets an "attempt" marker and arms the
    //! watchdog; the OS-facing firmware clears it once the boot is
    //! confirmed. If the next bt0 finds the marker still set, the previous
    //! boot never got that far, and it can pick a fallback.
    //!
    //! A milestone variant stores how far the boot came: `tag << 8 |
    //! milestone`, so a board can tell "hung in DRAM training" from "hung in
    //! EDK2".

    /// The reset-surviving word.
    pub trait Marker {
        fn read(&mut self) -> u32;
        fn write(&mut self, v: u32);
    }

    /// True if the last boot set `attempt` and never confirmed. Clears the
    /// marker, so a failed fallback leads back to the normal chain (A7A
    /// `previous_boot_failed`).
    pub fn previous_boot_failed(m: &mut impl Marker, attempt: u32) -> bool {
        let failed = m.read() == attempt;
        m.write(0);
        failed
    }

    /// Mark this boot as in progress (A7A `begin_attempt`, without the
    /// watchdog, which the board arms itself).
    pub fn begin_attempt(m: &mut impl Marker, attempt: u32) {
        m.write(attempt);
    }

    /// The boot is confirmed: clear the marker.
    pub fn confirm(m: &mut impl Marker) {
        m.write(0);
    }

    /// How far a boot came, in boot order.
    #[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
    #[repr(u8)]
    pub enum Milestone {
        Bt0 = 1,
        DramReady = 2,
        NextStage = 3,
        Edk2Dxe = 4,
        BootManager = 5,
    }
    impl Milestone {
        pub const ALL: [Milestone; 5] = [
            Milestone::Bt0,
            Milestone::DramReady,
            Milestone::NextStage,
            Milestone::Edk2Dxe,
            Milestone::BootManager,
        ];
        pub fn from_u8(v: u8) -> Option<Self> {
            Self::ALL.into_iter().find(|m| *m as u8 == v)
        }
    }

    /// Record that this boot reached `ms` (tag: a board constant, 24 bits).
    pub fn reach(m: &mut impl Marker, tag: u32, ms: Milestone) {
        m.write((tag & 0x00ff_ffff) << 8 | ms as u32);
    }

    /// A milestone's name in the `[progress]` line.
    pub fn name(ms: Milestone) -> &'static str {
        match ms {
            Milestone::Bt0 => "bt0",
            Milestone::DramReady => "dram_ready",
            Milestone::NextStage => "next_stage",
            Milestone::Edk2Dxe => "edk2_dxe",
            Milestone::BootManager => "boot_manager",
        }
    }

    pub mod stamps {
        //! Boot-progress timestamps (P3-01): one tick count per milestone in
        //! a small table in memory that the later stages can reach (SRAM or
        //! a reserved DRAM word block), beside the marker word, which keeps
        //! holding only tag and milestone. Ticks are the board's free-running
        //! timer (K1 CLINT mtime, A7A CNTPCT; both 24 MHz), counted from
        //! reset, so milestones of one boot compare directly.
        //!
        //! Layout (little-endian 32-bit words at [`crate::Area`] offsets):
        //!
        //! | word | content |
        //! |---|---|
        //! | 0 | magic `BPT1` (0x3154_5042) |
        //! | 1 | tag (the board constant of [`super::reach`]) |
        //! | 2 | timer frequency, Hz |
        //! | 3 | bit mask of the milestones stamped (bit = milestone value) |
        //! | 4 + 2 (m - 1), 5 + 2 (m - 1) | milestone m's ticks, low and high word |
        //! | 14 | check: the wrapping sum of words 0..14, inverted |
        //!
        //! A table from another board, another boot's leftovers or random
        //! memory fails the magic, tag or check word and reads as `None`.
        //! [`line`] formats `[progress] tag=0x.. hz=.. bt0=..ms ...`, the line
        //! the acceptance harness reads into `boot_ms`.

        use super::Milestone;
        use crate::Area;

        pub const MAGIC: u32 = 0x3154_5042;
        /// Table size in bytes.
        pub const SIZE: usize = 15 * 4;
        const CHECK: usize = 14 * 4;

        fn slot(ms: Milestone) -> usize {
            (4 + 2 * (ms as usize - 1)) * 4
        }

        fn check(a: &impl Area) -> u32 {
            !(0..14).fold(0u32, |s, w| s.wrapping_add(a.read32(w * 4)))
        }

        fn seal(a: &mut impl Area) {
            let c = check(a);
            a.write32(CHECK, c);
        }

        /// Start a table for this boot (bt0, first thing after the timer
        /// runs). Clears every slot.
        pub fn init(a: &mut impl Area, tag: u32, timer_hz: u32) {
            a.write32(0, MAGIC);
            a.write32(4, tag & 0x00ff_ffff);
            a.write32(8, timer_hz);
            a.write32(12, 0);
            for w in 4..14 {
                a.write32(w * 4, 0);
            }
            seal(a);
        }

        /// Record `ticks` for `ms`. Returns false (and writes nothing) when
        /// the table is not valid, so a stage never stamps into memory it
        /// did not set up.
        pub fn stamp(a: &mut impl Area, tag: u32, ms: Milestone, ticks: u64) -> bool {
            if !valid(a, tag) {
                return false;
            }
            let o = slot(ms);
            a.write32(o, ticks as u32);
            a.write32(o + 4, (ticks >> 32) as u32);
            let mask = a.read32(12) | 1 << ms as u32;
            a.write32(12, mask);
            seal(a);
            true
        }

        /// Whether the table is intact and carries `tag`.
        pub fn valid(a: &impl Area, tag: u32) -> bool {
            a.read32(0) == MAGIC
                && a.read32(4) == tag & 0x00ff_ffff
                && a.read32(8) != 0
                && a.read32(CHECK) == check(a)
        }

        /// The stamps of a valid table.
        #[derive(Clone, Copy, Debug, PartialEq, Eq)]
        pub struct Stamps {
            pub tag: u32,
            pub timer_hz: u32,
            pub ticks: [Option<u64>; 5],
        }

        impl Stamps {
            pub fn get(&self, ms: Milestone) -> Option<u64> {
                self.ticks[ms as usize - 1]
            }
            /// Milliseconds since reset (rounded down).
            pub fn ms(&self, ms: Milestone) -> Option<u64> {
                self.get(ms)
                    .map(|t| (u128::from(t) * 1000 / u128::from(self.timer_hz)) as u64)
            }
        }

        pub fn read(a: &impl Area, tag: u32) -> Option<Stamps> {
            if !valid(a, tag) {
                return None;
            }
            let mask = a.read32(12);
            let mut ticks = [None; 5];
            for ms in Milestone::ALL {
                if mask & 1 << ms as u32 != 0 {
                    let o = slot(ms);
                    ticks[ms as usize - 1] =
                        Some(u64::from(a.read32(o)) | u64::from(a.read32(o + 4)) << 32);
                }
            }
            Some(Stamps {
                tag: a.read32(4),
                timer_hz: a.read32(8),
                ticks,
            })
        }

        /// `[progress] tag=0x00a7a0 hz=24000000 bt0=12ms dram_ready=2835ms`
        /// (stamped milestones only, in boot order).
        pub fn line(s: &Stamps, out: &mut impl core::fmt::Write) -> core::fmt::Result {
            write!(out, "[progress] tag={:#08x} hz={}", s.tag, s.timer_hz)?;
            for ms in Milestone::ALL {
                if let Some(v) = s.ms(ms) {
                    write!(out, " {}={}ms", super::name(ms), v)?;
                }
            }
            Ok(())
        }
    }

    /// The milestone the previous boot left, if the marker carries `tag`;
    /// clears the marker. `Some(BootManager)` means it finished.
    pub fn previous_milestone(m: &mut impl Marker, tag: u32) -> Option<Milestone> {
        let v = m.read();
        m.write(0);
        if v >> 8 != tag & 0x00ff_ffff {
            return None;
        }
        Milestone::from_u8(v as u8)
    }
}

#[cfg(test)]
mod tests {
    use super::progress::{self, Marker, Milestone};
    use super::ring::{self, Format};
    use super::Area;
    use std::{vec, vec::Vec};

    struct Buf(Vec<u8>);
    impl Area for Buf {
        fn read8(&self, o: usize) -> u8 {
            self.0[o]
        }
        fn write8(&mut self, o: usize, v: u8) {
            self.0[o] = v;
        }
        fn read32(&self, o: usize) -> u32 {
            assert_eq!(o % 4, 0, "unaligned word");
            u32::from_le_bytes(self.0[o..o + 4].try_into().unwrap())
        }
        fn write32(&mut self, o: usize, v: u32) {
            assert_eq!(o % 4, 0, "unaligned word");
            self.0[o..o + 4].copy_from_slice(&v.to_le_bytes());
        }
    }
    const F: Format = Format {
        magic: *b"TESTLOG!",
        data_size: 16,
    };
    #[test]
    fn ring_wraps_and_reads_back_in_order() {
        let mut a = Buf(vec![0; 64 + 16]);
        assert_eq!(ring::read(&a, &F, &mut [0; 8]), None);
        ring::init_header(&mut a, &F, 7);
        for c in b"0123456789abcdefXYZ" {
            ring::putc(&mut a, &F, *c);
        }
        let mut out = [0u8; 32];
        let n = ring::read(&a, &F, &mut out).unwrap();
        assert_eq!(&out[..n], b"3456789abcdefXYZ");
        let mut short = [0u8; 4];
        assert_eq!(ring::read(&a, &F, &mut short), Some(4));
        assert_eq!(&short, b"fXYZ");
        assert_eq!((ring::written(&a), ring::seq(&a)), (19, 7));
    }
    #[test]
    fn rotate_keeps_the_previous_log_and_counts_boots() {
        let mut cur = Buf(vec![0xee; 80]);
        let mut prev = Buf(vec![0xee; 80]);
        assert_eq!(ring::rotate(&mut cur, &mut prev, &F), 0);
        assert!(!ring::header_valid(&prev, &F));
        ring::putc(&mut cur, &F, b'a');
        assert_eq!(ring::rotate(&mut cur, &mut prev, &F), 1);
        let mut out = [0u8; 4];
        assert_eq!(ring::read(&prev, &F, &mut out), Some(1));
        assert_eq!(out[0], b'a');
        assert_eq!(ring::read(&cur, &F, &mut out), Some(0));
    }
    #[test]
    fn early_buffer_moves_into_the_ring() {
        let mut early = Buf(vec![0; 8]);
        for c in b"hello" {
            ring::early_putc(&mut early, 8, *c);
        }
        let mut cur = Buf(vec![0; 80]);
        ring::init_header(&mut cur, &F, 0);
        ring::drain_early(&early, 8, &mut cur, &F);
        let mut out = [0u8; 8];
        let n = ring::read(&cur, &F, &mut out).unwrap();
        assert_eq!(&out[..n], b"hell");
    }
    struct Words([u32; 15]);
    impl Area for Words {
        fn read8(&self, o: usize) -> u8 {
            (self.0[o / 4] >> (8 * (o % 4))) as u8
        }
        fn write8(&mut self, o: usize, v: u8) {
            let w = &mut self.0[o / 4];
            *w = *w & !(0xff << (8 * (o % 4))) | u32::from(v) << (8 * (o % 4));
        }
        fn read32(&self, o: usize) -> u32 {
            self.0[o / 4]
        }
        fn write32(&mut self, o: usize, v: u32) {
            self.0[o / 4] = v;
        }
    }

    #[test]
    fn stamps_round_trip_and_line() {
        use progress::stamps;
        let mut a = Words([0xdead_beef; 15]);
        // Random memory is no table, and nothing is stamped into it.
        assert!(stamps::read(&a, 0xa7a).is_none());
        assert!(!stamps::stamp(&mut a, 0xa7a, Milestone::Bt0, 1));
        assert_eq!(a.0, [0xdead_beef; 15]);
        stamps::init(&mut a, 0xa7a, 24_000_000);
        assert!(stamps::stamp(&mut a, 0xa7a, Milestone::Bt0, 288_000));
        // 871011209 cycles at ~307 MHz is ~2.84 s; in 24 MHz ticks:
        assert!(stamps::stamp(
            &mut a,
            0xa7a,
            Milestone::DramReady,
            68_040_000
        ));
        // Past 2^32 ticks (179 s at 24 MHz): the high word carries.
        assert!(stamps::stamp(
            &mut a,
            0xa7a,
            Milestone::BootManager,
            (1 << 32) + 24_000
        ));
        let s = stamps::read(&a, 0xa7a).unwrap();
        assert_eq!(s.ms(Milestone::Bt0), Some(12));
        assert_eq!(s.ms(Milestone::DramReady), Some(2835));
        assert_eq!(s.get(Milestone::NextStage), None);
        assert_eq!(s.ms(Milestone::BootManager), Some(178_956 + 1));
        let mut line = std::string::String::new();
        stamps::line(&s, &mut line).unwrap();
        assert_eq!(
            line,
            "[progress] tag=0x000a7a hz=24000000 bt0=12ms dram_ready=2835ms boot_manager=178957ms"
        );
        // Another board's tag, or one flipped bit: not this boot's table.
        assert!(stamps::read(&a, 0x0b1).is_none());
        for w in 0..15 {
            let mut b = Words(a.0);
            b.0[w] ^= 1 << (w % 32);
            assert!(stamps::read(&b, 0xa7a).is_none(), "word {w}");
        }
        // init starts over.
        stamps::init(&mut a, 0xa7a, 24_000_000);
        assert_eq!(stamps::read(&a, 0xa7a).unwrap().ticks, [None; 5]);
    }

    struct Word(u32);
    impl Marker for Word {
        fn read(&mut self) -> u32 {
            self.0
        }
        fn write(&mut self, v: u32) {
            self.0 = v;
        }
    }
    #[test]
    fn attempt_and_milestones() {
        let attempt = u32::from_le_bytes(*b"A7At");
        let mut w = Word(0);
        assert!(!progress::previous_boot_failed(&mut w, attempt));
        progress::begin_attempt(&mut w, attempt);
        assert!(progress::previous_boot_failed(&mut w, attempt));
        assert!(!progress::previous_boot_failed(&mut w, attempt), "cleared");
        progress::begin_attempt(&mut w, attempt);
        progress::confirm(&mut w);
        assert!(!progress::previous_boot_failed(&mut w, attempt));
        progress::reach(&mut w, 0xa7a, Milestone::DramReady);
        assert_eq!(
            progress::previous_milestone(&mut w, 0xa7a),
            Some(Milestone::DramReady)
        );
        assert_eq!(progress::previous_milestone(&mut w, 0xa7a), None);
        progress::reach(&mut w, 0xa7a, Milestone::BootManager);
        assert_eq!(progress::previous_milestone(&mut w, 0x123), None);
    }
}
