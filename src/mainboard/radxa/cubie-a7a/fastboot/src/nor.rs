//! The Cubie A7A's SPI NOR (W25Q64FW, 8 MiB) as fastboot partitions, and
//! the guards every erase and program passes.
//!
//! The partitions keep the SD card's offsets (the SPI NOR boot layout):
//! bt0 (eGON) at 0, the layout table
//! frame at 0x1ff000, oreboot main at 0x200000, the EDK2 FD at 0x300000.
//! Nothing at or above [`WRITE_LIMIT`] is ever erased or programmed.
//!
//! A `flash:<part>` or `erase:<part>` goes ahead only when all of these
//! hold, checked in this order, before the first erase:
//!
//! 1. the board enabled NOR writes ([`Guard::new`]; bt0 only with its
//!    compile-time feature `fastboot-nor-write`);
//! 2. the host armed exactly this operation with `oem nor-arm` within the
//!    last [`ARM_TTL_MS`]: for `flash`, naming the image's length and
//!    CRC-32, which the download must match; every command other than
//!    `getvar` and `download` disarms, and every flash or erase attempt
//!    uses the arm up, whatever its outcome;
//! 3. the partition exists, and the image fits it and passes its content
//!    check (bt0: [`crate::egon::check`]; the layout frame: the board's
//!    decoder; main and EDK2: the board's check, if any);
//! 4. the flash identifies as the expected part ([`JEDEC_ID`]), is idle,
//!    and has no protection bit set in status register 1 (never cleared
//!    here: there is no status register write in [`NorFlash`]).
//!
//! Then the whole partition is erased (64 KiB blocks where aligned, 4 KiB
//! sectors elsewhere) and read back as 0xFF, the image is programmed page
//! by page with its first page last (a partial bt0 write never carries an
//! eGON header), and read back and compared. Every erase and program call
//! is checked against the partition and [`WRITE_LIMIT`] once more on its
//! way to the flash ([`in_window`]); bt0's driver checks again.
use crate::proto::{Command, Info};
use crate::{crc32, egon, proto};

pub const NOR_SIZE: u32 = 0x0080_0000;
/// Erase and program only below this (the top 1 MiB is never touched;
/// the same bound as the FEL write payload's WR_HI).
pub const WRITE_LIMIT: u32 = 0x0070_0000;
pub const PAGE: u32 = 256;
pub const SECTOR: u32 = 0x1000;
pub const BLOCK: u32 = 0x1_0000;
/// W25Q64FW: Winbond, 1.8 V, 64 Mbit.
pub const JEDEC_ID: [u8; 3] = [0xef, 0x60, 0x17];
/// Status register 1: BUSY, WEL, then BP0-2, TB, SEC, SRP0.
pub const SR1_BUSY: u8 = 0x01;
pub const SR1_WEL: u8 = 0x02;
pub const SR1_PROTECT: u8 = 0xfc;
/// How long an `oem nor-arm` stays valid.
pub const ARM_TTL_MS: u64 = 60_000;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Content {
    /// A boot0 for the mask ROM: [`egon::check`].
    Egon,
    /// The boot layout table frame: the board's decoder.
    Layout,
    /// An image the layout table names.
    Raw,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Partition {
    pub name: &'static str,
    pub offset: u32,
    pub size: u32,
    pub content: Content,
}

pub const PARTITIONS: [Partition; 4] = [
    Partition {
        name: "bt0",
        offset: 0,
        size: 0x4_0000,
        content: Content::Egon,
    },
    Partition {
        name: "layout",
        offset: 0x1f_f000,
        size: SECTOR,
        content: Content::Layout,
    },
    Partition {
        name: "main",
        offset: 0x20_0000,
        size: 0x10_0000,
        content: Content::Raw,
    },
    Partition {
        name: "edk2",
        offset: 0x30_0000,
        size: 0x40_0000,
        content: Content::Raw,
    },
];

const fn table_ok() -> bool {
    let mut i = 0;
    while i < PARTITIONS.len() {
        let p = PARTITIONS[i];
        if !p.offset.is_multiple_of(SECTOR) || !p.size.is_multiple_of(SECTOR) || p.size == 0 {
            return false;
        }
        if p.offset as u64 + p.size as u64 > WRITE_LIMIT as u64 {
            return false;
        }
        if i > 0 && PARTITIONS[i - 1].offset + PARTITIONS[i - 1].size > p.offset {
            return false;
        }
        i += 1;
    }
    WRITE_LIMIT <= NOR_SIZE
}
const _: () = assert!(
    table_ok(),
    "NOR partitions must be sector aligned, sorted, disjoint and below WRITE_LIMIT"
);

pub fn partition(name: &str) -> Option<&'static Partition> {
    PARTITIONS.iter().find(|p| p.name == name)
}

/// The partition of `content` kind that holds all of [offset, offset+size).
pub fn holding(offset: u32, size: u32, content: Content) -> Option<&'static Partition> {
    let end = offset as u64 + size as u64;
    PARTITIONS.iter().find(|p| {
        p.content == content && offset >= p.offset && end <= p.offset as u64 + p.size as u64
    })
}

/// The last check before an erase or program reaches the flash.
pub fn in_window(offset: u32, len: u32) -> bool {
    len != 0
        && offset
            .checked_add(len)
            .is_some_and(|end| end <= WRITE_LIMIT)
}

/// The board's content check of an image for a partition.
pub type ContentCheck<'a> = dyn FnMut(&Partition, &[u8]) -> Result<(), &'static str> + 'a;

/// What an arm allows.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Op {
    Flash { len: u32, crc32: u32 },
    Erase,
}

#[derive(Clone, Copy, Debug)]
struct Arm {
    part: &'static Partition,
    op: Op,
    at_ms: u64,
}

/// The arming state (rule 1 and 2 of the module documentation).
pub struct Guard {
    enabled: bool,
    arm: Option<Arm>,
}

impl Guard {
    pub const fn new(enabled: bool) -> Self {
        Guard { enabled, arm: None }
    }

    pub fn enabled(&self) -> bool {
        self.enabled
    }

    pub fn armed(&self) -> Option<(&'static str, Op)> {
        self.arm.map(|a| (a.part.name, a.op))
    }

    /// Every command passes here first: only `getvar` and `download` keep
    /// an arm (the fastboot tool sends both between `oem nor-arm` and the
    /// `flash`); flash and erase use it up in [`Guard::take`].
    pub fn observe(&mut self, c: &Command) {
        match c {
            Command::Getvar(_) | Command::Download(_) | Command::Flash(_) | Command::Erase(_) => {}
            _ => self.arm = None,
        }
    }

    /// `oem nor-arm <part> flash <len hex> <crc32 hex>` or
    /// `oem nor-arm <part> erase` (the words after `nor-arm`).
    pub fn arm(
        &mut self,
        args: &str,
        now_ms: u64,
    ) -> Result<(&'static Partition, Op), &'static str> {
        self.arm = None;
        if !self.enabled {
            return Err("NOR writes are not built in");
        }
        let mut w = args.split(' ').filter(|s| !s.is_empty());
        let part = w
            .next()
            .and_then(partition)
            .ok_or("unknown NOR partition")?;
        let op = match (w.next(), w.next(), w.next(), w.next()) {
            (Some("flash"), Some(len), Some(crc), None) => {
                let len = proto::hex_u32(len).ok_or("bad length")?;
                let crc32 = proto::hex_u32(crc).ok_or("bad CRC-32")?;
                if len == 0 || len > part.size {
                    return Err("length does not fit the partition");
                }
                Op::Flash { len, crc32 }
            }
            (Some("erase"), None, None, None) => Op::Erase,
            _ => return Err("usage: oem nor-arm <part> flash <len> <crc32> | <part> erase"),
        };
        self.arm = Some(Arm {
            part,
            op,
            at_ms: now_ms,
        });
        Ok((part, op))
    }

    /// Use the arm up for `flash` (`image` Some) or `erase` (None) of
    /// `name`: Ok only for exactly the armed operation, in time.
    pub fn take(
        &mut self,
        name: &str,
        image: Option<&[u8]>,
        now_ms: u64,
    ) -> Result<&'static Partition, &'static str> {
        let arm = self.arm.take();
        if !self.enabled {
            return Err("NOR writes are not built in");
        }
        let arm = arm.ok_or("not armed (oem nor-arm first)")?;
        if arm.part.name != name {
            return Err("armed for another partition");
        }
        if now_ms.saturating_sub(arm.at_ms) > ARM_TTL_MS {
            return Err("arm expired");
        }
        match (arm.op, image) {
            (Op::Erase, None) => {}
            (Op::Flash { len, crc32: crc }, Some(image)) => {
                if image.len() != len as usize || crc32(image) != crc {
                    return Err("download does not match the armed length and CRC-32");
                }
            }
            _ => return Err("armed for another operation"),
        }
        Ok(arm.part)
    }
}

/// The flash, as the writer needs it. Implementations send only the
/// opcodes these methods name; there is no status register write.
pub trait NorFlash {
    type Error: Copy + core::fmt::Debug;
    fn jedec_id(&mut self) -> Result<[u8; 3], Self::Error>;
    fn status(&mut self) -> Result<u8, Self::Error>;
    fn read(&mut self, offset: u32, buf: &mut [u8]) -> Result<(), Self::Error>;
    /// Write enable, erase the [`SECTOR`] or [`BLOCK`] at `offset` (aligned
    /// to `len`) and wait until it is done.
    fn erase(&mut self, offset: u32, len: u32) -> Result<(), Self::Error>;
    /// Write enable, program `data` (1 to [`PAGE`] bytes, inside one page)
    /// at `offset` and wait until it is done.
    fn program(&mut self, offset: u32, data: &[u8]) -> Result<(), Self::Error>;
    /// Keep the watchdog quiet during long operations.
    fn kick(&mut self) {}
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error<E> {
    /// A guard said no; nothing was erased or programmed.
    Refused(&'static str),
    /// The flash failed a command (the partition may be partly written).
    Flash(E),
    /// Read-back mismatch at this NOR offset.
    Verify(u32),
}

impl<E> Error<E> {
    pub fn message(&self) -> &'static str {
        match self {
            Error::Refused(why) => why,
            Error::Flash(_) => "SPI NOR command failed (see the console)",
            Error::Verify(_) => "SPI NOR read-back mismatch (see the console)",
        }
    }
}

/// The erase steps for [offset, offset + size) (both sector aligned):
/// 64 KiB blocks where aligned and inside, 4 KiB sectors elsewhere.
pub fn erase_plan(offset: u32, size: u32) -> impl Iterator<Item = (u32, u32)> {
    let end = offset + size;
    let mut pos = offset;
    core::iter::from_fn(move || {
        if pos >= end {
            return None;
        }
        let len = if pos.is_multiple_of(BLOCK) && end - pos >= BLOCK {
            BLOCK
        } else {
            SECTOR
        };
        let step = (pos, len);
        pos += len;
        Some(step)
    })
}

fn preflight<F: NorFlash>(f: &mut F) -> Result<(), Error<F::Error>> {
    if f.jedec_id().map_err(Error::Flash)? != JEDEC_ID {
        return Err(Error::Refused("unexpected SPI NOR (JEDEC ID)"));
    }
    let sr = f.status().map_err(Error::Flash)?;
    if sr & SR1_BUSY != 0 {
        return Err(Error::Refused("SPI NOR busy"));
    }
    if sr & SR1_PROTECT != 0 {
        return Err(Error::Refused("SPI NOR write protection set; not touched"));
    }
    Ok(())
}

/// Erase all of `p` and check that it reads back erased.
fn erase_part<F: NorFlash>(
    f: &mut F,
    p: &Partition,
    scratch: &mut [u8],
) -> Result<(), Error<F::Error>> {
    for (offset, len) in erase_plan(p.offset, p.size) {
        if !in_window(offset, len) || offset < p.offset || offset + len > p.offset + p.size {
            return Err(Error::Refused("erase outside the partition"));
        }
        f.erase(offset, len).map_err(Error::Flash)?;
        f.kick();
    }
    compare(f, p.offset, p.size, scratch, |_| 0xff)
}

/// Read [offset, offset + len) back in `scratch`-sized pieces and compare
/// each byte with `want(i)` (i from 0).
fn compare<F: NorFlash>(
    f: &mut F,
    offset: u32,
    len: u32,
    scratch: &mut [u8],
    want: impl Fn(usize) -> u8,
) -> Result<(), Error<F::Error>> {
    let mut done = 0usize;
    while done < len as usize {
        let n = scratch.len().min(len as usize - done);
        let at = offset + done as u32;
        f.read(at, &mut scratch[..n]).map_err(Error::Flash)?;
        if let Some(i) = (0..n).find(|&i| scratch[i] != want(done + i)) {
            return Err(Error::Verify(at + i as u32));
        }
        done += n;
        f.kick();
    }
    Ok(())
}

/// `flash:<name>` of `image`: [`Guard::take`], then [`program_partition`].
#[allow(clippy::too_many_arguments)]
pub fn flash<F: NorFlash>(
    f: &mut F,
    guard: &mut Guard,
    name: &str,
    image: &[u8],
    now_ms: u64,
    check: &mut ContentCheck,
    scratch: &mut [u8],
    info: &mut dyn Info,
) -> Result<(), Error<F::Error>> {
    let p = guard
        .take(name, Some(image), now_ms)
        .map_err(Error::Refused)?;
    program_partition(f, p, image, check, scratch, info)
}

/// Write `image` to `p`, whose arm the caller has just used up: rules 3
/// and 4 of the module documentation, then erase, program and read back.
/// `check` is the board's content check for [`Content::Layout`] and
/// [`Content::Raw`]; `scratch` (at least one byte) holds read-backs.
pub fn program_partition<F: NorFlash>(
    f: &mut F,
    p: &Partition,
    image: &[u8],
    check: &mut ContentCheck,
    scratch: &mut [u8],
    info: &mut dyn Info,
) -> Result<(), Error<F::Error>> {
    if image.is_empty() || image.len() > p.size as usize {
        return Err(Error::Refused("image does not fit the partition"));
    }
    match p.content {
        Content::Egon => egon::check(image, p.size),
        Content::Layout | Content::Raw => check(p, image),
    }
    .map_err(Error::Refused)?;
    preflight(f)?;

    info.info(format_args!(
        "erasing {} ({} KiB at 0x{:06x})",
        p.name,
        p.size / 1024,
        p.offset
    ));
    erase_part(f, p, scratch)?;

    info.info(format_args!("programming {} bytes", image.len()));
    let pages = image.len().div_ceil(PAGE as usize);
    // The first page last: until it is written the partition has no header.
    for page in (1..pages).chain(0..1) {
        let start = page * PAGE as usize;
        let data = &image[start..image.len().min(start + PAGE as usize)];
        if data.iter().all(|&b| b == 0xff) {
            continue;
        }
        let at = p.offset + start as u32;
        if !in_window(at, data.len() as u32) || at + data.len() as u32 > p.offset + p.size {
            return Err(Error::Refused("program outside the partition"));
        }
        f.program(at, data).map_err(Error::Flash)?;
        if page % 64 == 0 {
            f.kick();
        }
    }
    compare(f, p.offset, image.len() as u32, scratch, |i| image[i])?;
    info.info(format_args!("{}: written and read back", p.name));
    Ok(())
}

/// `erase:<name>`: [`Guard::take`], then [`erase_partition`].
pub fn erase<F: NorFlash>(
    f: &mut F,
    guard: &mut Guard,
    name: &str,
    now_ms: u64,
    scratch: &mut [u8],
    info: &mut dyn Info,
) -> Result<(), Error<F::Error>> {
    let p = guard.take(name, None, now_ms).map_err(Error::Refused)?;
    erase_partition(f, p, scratch, info)
}

/// Erase all of `p` (its arm just used up) and check it reads back erased.
pub fn erase_partition<F: NorFlash>(
    f: &mut F,
    p: &Partition,
    scratch: &mut [u8],
    info: &mut dyn Info,
) -> Result<(), Error<F::Error>> {
    preflight(f)?;
    info.info(format_args!(
        "erasing {} ({} KiB at 0x{:06x})",
        p.name,
        p.size / 1024,
        p.offset
    ));
    erase_part(f, p, scratch)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::{format, string::String, vec, vec::Vec};

    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    struct Fault;

    /// An 8 MiB NOR: erase sets 0xFF, program can only clear bits.
    struct Model {
        mem: Vec<u8>,
        id: [u8; 3],
        sr: u8,
        stuck: Option<u32>,
        ops: Vec<(char, u32, u32)>,
    }

    impl Model {
        fn new() -> Self {
            Model {
                mem: vec![0x5a; NOR_SIZE as usize],
                id: JEDEC_ID,
                sr: 0,
                stuck: None,
                ops: Vec::new(),
            }
        }
        fn writes(&self) -> impl Iterator<Item = &(char, u32, u32)> {
            self.ops.iter().filter(|o| o.0 != 'r')
        }
    }

    impl NorFlash for Model {
        type Error = Fault;
        fn jedec_id(&mut self) -> Result<[u8; 3], Fault> {
            Ok(self.id)
        }
        fn status(&mut self) -> Result<u8, Fault> {
            Ok(self.sr)
        }
        fn read(&mut self, offset: u32, buf: &mut [u8]) -> Result<(), Fault> {
            self.ops.push(('r', offset, buf.len() as u32));
            buf.copy_from_slice(&self.mem[offset as usize..offset as usize + buf.len()]);
            Ok(())
        }
        fn erase(&mut self, offset: u32, len: u32) -> Result<(), Fault> {
            assert!(len == SECTOR || len == BLOCK);
            assert_eq!(offset % len, 0);
            assert!(in_window(offset, len));
            self.ops.push(('e', offset, len));
            self.mem[offset as usize..(offset + len) as usize].fill(0xff);
            Ok(())
        }
        fn program(&mut self, offset: u32, data: &[u8]) -> Result<(), Fault> {
            assert!(!data.is_empty() && data.len() <= PAGE as usize);
            assert_eq!(
                offset / PAGE,
                (offset + data.len() as u32 - 1) / PAGE,
                "crosses a page"
            );
            assert!(in_window(offset, data.len() as u32));
            self.ops.push(('p', offset, data.len() as u32));
            for (i, b) in data.iter().enumerate() {
                let a = offset as usize + i;
                self.mem[a] &= *b;
                if self.stuck == Some(a as u32) {
                    self.mem[a] = 0x00;
                }
            }
            Ok(())
        }
    }

    #[derive(Default)]
    struct Log(Vec<String>);
    impl Info for Log {
        fn info(&mut self, msg: core::fmt::Arguments) {
            self.0.push(format!("{msg}"));
        }
    }

    fn no_check(_: &Partition, _: &[u8]) -> Result<(), &'static str> {
        Ok(())
    }

    fn armed(img: &[u8], part: &str) -> Guard {
        let mut g = Guard::new(true);
        let args = format!("{part} flash {:x} {:08x}", img.len(), crc32(img));
        g.arm(&args, 1000).unwrap();
        g
    }

    fn do_flash(m: &mut Model, g: &mut Guard, part: &str, img: &[u8]) -> Result<(), Error<Fault>> {
        let mut scratch = [0u8; 4096];
        flash(
            m,
            g,
            part,
            img,
            2000,
            &mut no_check,
            &mut scratch,
            &mut Log::default(),
        )
    }

    #[test]
    fn partition_table() {
        assert!(table_ok());
        assert_eq!(partition("edk2").unwrap().offset, 0x300000);
        assert!(partition("spl").is_none());
        assert_eq!(
            holding(0x200000, 0x1000, Content::Raw).unwrap().name,
            "main"
        );
        assert_eq!(
            holding(0x300000, 0x400000, Content::Raw).unwrap().name,
            "edk2"
        );
        assert!(
            holding(0x2ff000, 0x2000, Content::Raw).is_none(),
            "straddles main and edk2"
        );
        assert!(holding(0x0, 0x100, Content::Raw).is_none());
        assert!(holding(0x6ff000, 0x2000, Content::Raw).is_none());
    }

    #[test]
    fn window() {
        assert!(in_window(0, 1));
        assert!(in_window(WRITE_LIMIT - SECTOR, SECTOR));
        assert!(!in_window(WRITE_LIMIT - SECTOR, SECTOR + 1));
        assert!(!in_window(WRITE_LIMIT, 1));
        assert!(!in_window(0, 0));
        assert!(!in_window(u32::MAX, 2));
    }

    #[test]
    fn erase_plans() {
        let v: Vec<_> = erase_plan(0x1ff000, 0x1000).collect();
        assert_eq!(v, [(0x1ff000, SECTOR)]);
        let v: Vec<_> = erase_plan(0x300000, 0x400000).collect();
        assert_eq!(v.len(), 64);
        assert!(v.iter().all(|&(_, l)| l == BLOCK));
        let v: Vec<_> = erase_plan(0xf000, 0x12000).collect();
        assert_eq!(v, [(0xf000, SECTOR), (0x10000, BLOCK), (0x20000, SECTOR)]);
        let total: u32 = erase_plan(0, 0x40000).map(|s| s.1).sum();
        assert_eq!(total, 0x40000);
    }

    #[test]
    fn arm_parsing_and_observe() {
        let mut g = Guard::new(true);
        assert!(g.arm("", 0).is_err());
        assert!(g.arm("spl erase", 0).is_err());
        assert!(g.arm("main flash 10", 0).is_err());
        assert!(g.arm("main flash 10 zz", 0).is_err());
        assert!(g.arm("main flash 0 0", 0).is_err());
        assert!(g.arm("main flash 100001 0", 0).is_err(), "longer than main");
        assert!(g.arm("main erase now", 0).is_err());
        assert!(g.arm("main flash 10 abc extra", 0).is_err());
        assert_eq!(g.armed(), None, "a refused arm leaves nothing armed");
        g.arm("main  flash 0x10 0xabc", 0).unwrap();
        assert_eq!(
            g.armed(),
            Some((
                "main",
                Op::Flash {
                    len: 0x10,
                    crc32: 0xabc
                }
            ))
        );
        // getvar and download keep it; anything else drops it.
        g.observe(&Command::Getvar("max-download-size"));
        g.observe(&Command::Download(Some(16)));
        assert!(g.armed().is_some());
        for c in [
            Command::Boot,
            Command::Continue,
            Command::Oem("log"),
            Command::Reboot,
            Command::Unknown("x"),
        ] {
            let mut g = Guard::new(true);
            g.arm("edk2 erase", 0).unwrap();
            g.observe(&c);
            assert_eq!(g.armed(), None, "{c:?}");
        }
        let mut off = Guard::new(false);
        assert_eq!(off.arm("main erase", 0), Err("NOR writes are not built in"));
    }

    #[test]
    fn take_rules() {
        let img = [1u8, 2, 3];
        let mut g = armed(&img, "main");
        assert_eq!(
            g.take("edk2", Some(&img), 1000),
            Err("armed for another partition")
        );
        assert_eq!(
            g.take("main", Some(&img), 1000),
            Err("not armed (oem nor-arm first)"),
            "used up"
        );

        let mut g = armed(&img, "main");
        assert!(g.take("main", Some(&[1, 2, 4]), 1000).is_err(), "CRC");
        let mut g = armed(&img, "main");
        assert!(g.take("main", Some(&[1, 2]), 1000).is_err(), "length");
        let mut g = armed(&img, "main");
        assert_eq!(
            g.take("main", None, 1000),
            Err("armed for another operation")
        );
        let mut g = armed(&img, "main");
        assert_eq!(
            g.take("main", Some(&img), 1000 + ARM_TTL_MS + 1),
            Err("arm expired")
        );
        let mut g = armed(&img, "main");
        assert_eq!(
            g.take("main", Some(&img), 1000 + ARM_TTL_MS).unwrap().name,
            "main"
        );
    }

    #[test]
    fn flash_main_happy_path() {
        let mut m = Model::new();
        let mut img: Vec<u8> = (0..5000u32).map(|i| (i % 251) as u8).collect();
        img[256..512].fill(0xff); // a blank page is skipped
        let mut g = armed(&img, "main");
        let mut log = Log::default();
        let mut scratch = [0u8; 1000];
        flash(
            &mut m,
            &mut g,
            "main",
            &img,
            2000,
            &mut no_check,
            &mut scratch,
            &mut log,
        )
        .unwrap();
        let p = partition("main").unwrap();
        assert_eq!(&m.mem[0x200000..0x200000 + img.len()], &img[..]);
        assert!(m.mem[0x200000 + img.len()..0x300000]
            .iter()
            .all(|&b| b == 0xff));
        assert!(
            m.mem[..0x200000].iter().all(|&b| b == 0x5a),
            "below untouched"
        );
        assert!(
            m.mem[0x300000..].iter().all(|&b| b == 0x5a),
            "above untouched"
        );
        // Every write inside the partition; 16 block erases; page 0 last.
        for &(_, o, l) in m.writes() {
            assert!(o >= p.offset && o + l <= p.offset + p.size);
        }
        assert_eq!(m.writes().filter(|o| o.0 == 'e').count(), 16);
        let progs: Vec<_> = m.writes().filter(|o| o.0 == 'p').collect();
        assert_eq!(progs.len(), 19, "20 pages, one blank");
        assert_eq!(progs.last().unwrap().1, 0x200000);
        assert_eq!(
            progs[progs.len() - 2],
            &('p', 0x200000 + 19 * 256, 5000 - 19 * 256)
        );
        assert_eq!(g.armed(), None);
        assert_eq!(log.0.len(), 3);
    }

    #[test]
    fn refusals_touch_nothing() {
        let img = vec![0u8; 100];
        // Not enabled.
        let mut m = Model::new();
        let mut g = Guard::new(false);
        assert_eq!(
            do_flash(&mut m, &mut g, "main", &img),
            Err(Error::Refused("NOR writes are not built in"))
        );
        // Not armed.
        let mut g = Guard::new(true);
        assert!(matches!(
            do_flash(&mut m, &mut g, "main", &img),
            Err(Error::Refused(_))
        ));
        // bt0 that is not an eGON image.
        let mut g = armed(&img, "bt0");
        assert_eq!(
            do_flash(&mut m, &mut g, "bt0", &img),
            Err(Error::Refused("not an eGON.BT0 image"))
        );
        // Board check refuses the layout frame.
        let mut g = armed(&img, "layout");
        let mut scratch = [0u8; 64];
        let r = flash(
            &mut m,
            &mut g,
            "layout",
            &img,
            2000,
            &mut |_, _| Err("bad frame"),
            &mut scratch,
            &mut Log::default(),
        );
        assert_eq!(r, Err(Error::Refused("bad frame")));
        // Too big for the layout partition (arm refuses already; force it).
        let big = vec![0u8; 0x1001];
        let mut g = Guard::new(true);
        assert!(g
            .arm(
                &format!("layout flash {:x} {:08x}", big.len(), crc32(&big)),
                0
            )
            .is_err());
        // Protection bits, busy, wrong part.
        for (sr, id, why) in [
            (0x04, JEDEC_ID, "SPI NOR write protection set; not touched"),
            (0x80, JEDEC_ID, "SPI NOR write protection set; not touched"),
            (0x01, JEDEC_ID, "SPI NOR busy"),
            (0x00, [0xef, 0x40, 0x17], "unexpected SPI NOR (JEDEC ID)"),
        ] {
            m.sr = sr;
            m.id = id;
            let mut g = armed(&img, "main");
            assert_eq!(
                do_flash(&mut m, &mut g, "main", &img),
                Err(Error::Refused(why))
            );
        }
        assert_eq!(m.writes().count(), 0);
        assert!(m.mem.iter().all(|&b| b == 0x5a));
    }

    #[test]
    fn bt0_needs_a_media_egon_and_goes_header_last() {
        let mut m = Model::new();
        let fel = egon::make(0x2000, 0x90000);
        let mut g = armed(&fel, "bt0");
        assert!(matches!(
            do_flash(&mut m, &mut g, "bt0", &fel),
            Err(Error::Refused(_))
        ));
        assert_eq!(m.writes().count(), 0);
        let img = egon::make(0x2000, egon::MEDIA_RUN_ADDR);
        let mut g = armed(&img, "bt0");
        do_flash(&mut m, &mut g, "bt0", &img).unwrap();
        assert_eq!(&m.mem[..0x2000], &img[..]);
        let last = m.writes().filter(|o| o.0 == 'p').last().unwrap();
        assert_eq!(last.1, 0);
        assert_eq!(m.writes().filter(|o| o.0 == 'e').count(), 4);
    }

    #[test]
    fn verify_catches_a_bad_cell() {
        let mut m = Model::new();
        m.stuck = Some(0x200123);
        let img = vec![0xa5u8; 0x400];
        let mut g = armed(&img, "main");
        assert_eq!(
            do_flash(&mut m, &mut g, "main", &img),
            Err(Error::Verify(0x200123))
        );
    }

    #[test]
    fn erase_partition() {
        let mut m = Model::new();
        let mut g = Guard::new(true);
        g.arm("layout erase", 0).unwrap();
        let mut scratch = [0u8; 512];
        erase(
            &mut m,
            &mut g,
            "layout",
            10,
            &mut scratch,
            &mut Log::default(),
        )
        .unwrap();
        assert!(m.mem[0x1ff000..0x200000].iter().all(|&b| b == 0xff));
        assert!(m.mem[0x1fe000..0x1ff000].iter().all(|&b| b == 0x5a));
        assert_eq!(m.writes().collect::<Vec<_>>(), [&('e', 0x1ff000, SECTOR)]);
        // Used up; and a flash arm does not allow an erase.
        assert!(erase(
            &mut m,
            &mut g,
            "layout",
            10,
            &mut scratch,
            &mut Log::default()
        )
        .is_err());
        let mut g = armed(&[1], "layout");
        assert!(erase(
            &mut m,
            &mut g,
            "layout",
            10,
            &mut scratch,
            &mut Log::default()
        )
        .is_err());
    }
}
