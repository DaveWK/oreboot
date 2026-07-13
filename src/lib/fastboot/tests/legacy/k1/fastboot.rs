// SPDX-License-Identifier: GPL-2.0-only
// Verbatim copy of ore-edk-boot-opi c91bbc4e40
// oreboot/src/mainboard/spacemit/k1x/bt0/src/fastboot.rs (equivalence oracle).
//! fastboot over USB, for writing the board's storage from a workstation.
//!
//! bt0 serves this when the BootROM started it from USB download mode
//! (`fastboot stage bt0.bin && fastboot continue`): it re-enumerates as a
//! fastboot device and takes
//!
//!   fastboot getvar <name>
//!   fastboot flash <target> <file>   raw or Android sparse images
//!   fastboot continue                boot the board's normal chain
//! Reboot commands fail: this firmware has no proven hardware reset path.
//!
//! Targets: `emmc` (eMMC user area), `boot0` and `boot1` (eMMC boot
//! partitions), `sd` (microSD card), and `disk`, the board's boot medium.
//! Images larger than max-download-size arrive as several sparse pieces,
//! which the fastboot tool makes itself.

use core::fmt::Write;

use crate::mmc::{Kind, Mmc, Partition};
use crate::usb::{Descriptors, Usb};

const SECTOR: usize = 512;
/// Download buffer in DRAM.
const BUF: usize = 0x1000_0000;
const MAX_DOWNLOAD: usize = 0x2000_0000;
/// Pattern buffer for sparse FILL chunks, after the download buffer.
const FILL_BUF: usize = BUF + MAX_DOWNLOAD;
const FILL_LEN: usize = 1 << 20;

const SPARSE_MAGIC: u32 = 0xed26_ff3a;
pub struct Board {
    /// Reported as `product`.
    pub name: &'static str,
    /// The medium `disk` names.
    pub disk: Kind,
}

/// A response line: OKAY, FAIL, INFO or DATA plus up to 60 bytes.
struct Line {
    buf: [u8; 64],
    len: usize,
}

impl Line {
    fn new(kind: &str) -> Self {
        let mut l = Self {
            buf: [0; 64],
            len: 0,
        };
        let _ = l.write_str(kind);
        l
    }
}

impl Write for Line {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let n = s.len().min(self.buf.len() - self.len);
        self.buf[self.len..self.len + n].copy_from_slice(&s.as_bytes()[..n]);
        self.len += n;
        Ok(())
    }
}

fn reply_status(
    usb: &mut Usb,
    kind: &str,
    msg: core::fmt::Arguments,
) -> Result<(), crate::usb::Error> {
    let mut l = Line::new(kind);
    let _ = l.write_fmt(msg);
    if kind != "INFO" {
        println!(
            "[fastboot] {}",
            core::str::from_utf8(&l.buf[..l.len]).unwrap_or("?")
        );
    }
    usb.send(&l.buf[..l.len])
}

fn reply(usb: &mut Usb, kind: &str, msg: core::fmt::Arguments) {
    // Non-handoff replies retain the command handler's simple interface.
    // Usb latches fatal quiescence faults; run checks this before each command.
    let _ = reply_status(usb, kind, msg);
}

fn target(name: &str, disk: Kind) -> Option<(Kind, Partition)> {
    Some(match name {
        "emmc" | "user" | "mmc0" => (Kind::Emmc, Partition::User),
        "boot0" => (Kind::Emmc, Partition::Boot0),
        "boot1" => (Kind::Emmc, Partition::Boot1),
        "sd" => (Kind::Sd, Partition::User),
        "disk" => (disk, Partition::User),
        _ => return None,
    })
}

fn le32(a: usize) -> u32 {
    unsafe { core::ptr::read_unaligned(a as *const u32) }
}

/// The open card: initialised once per medium, with the bus widened and
/// the clock raised as far as a test read allows.
struct Disk {
    kind: Kind,
    mmc: Mmc,
    width: u8,
    khz: u32,
    /// TRIM leaves zeros, so zero fills can be trimmed instead of written.
    trim: bool,
    /// Index into FALLBACK once a write has failed.
    fallback: usize,
    /// Writes go through SDMA rather than the data port.
    dma: bool,
    capacity: u64,
}

/// Bus settings tried in turn when a write fails: width, TX hold time,
/// clock in kHz (above 26 MHz with eMMC high-speed timing), SDMA.
const FALLBACK: [(u8, bool, u32, bool); 8] = [
    (8, true, 25_000, true),
    (4, true, 25_000, true),
    (8, true, 25_000, false),
    (4, true, 25_000, false),
    (1, true, 25_000, true),
    (1, true, 25_000, false),
    (1, false, 25_000, false),
    (1, true, 5_000, false),
];

impl Disk {
    fn open(slot: &mut Option<Disk>, kind: Kind) -> Result<&mut Disk, &'static str> {
        if slot.as_ref().map(|d| d.kind) != Some(kind) {
            *slot = None;
            let mut mmc = Mmc::init(kind).map_err(|_| "card did not initialise")?;
            mmc.tune_for_writes();
            let width = mmc.widen();
            // High speed where the card takes it, checked with a read.
            let mut khz = 0;
            if kind == Kind::Emmc {
                if let Ok(k) = mmc.set_khz(52_000) {
                    if mmc.ext_csd().is_ok() {
                        khz = k;
                    }
                }
            }
            if khz == 0 {
                khz = mmc
                    .set_khz(25_000)
                    .map_err(|_| "cannot set the card clock")?;
            }
            let trim = mmc.trim_zeroes();
            println!("[fastboot] {width}-bit bus at {khz} kHz, trim for zeros: {trim}");
            // PIO writes fail on wide eMMC buses (R2S), so start with SDMA.
            *slot = Some(Disk {
                kind,
                mmc,
                width,
                khz,
                trim,
                fallback: 0,
                dma: true,
                capacity: 0,
            });
        }
        Ok(slot.as_mut().unwrap())
    }

    /// Write, stepping through FALLBACK when the card rejects the data.
    fn write(&mut self, lba: u32, count: usize, src: usize) -> Result<(), &'static str> {
        let end = (lba as u64)
            .checked_add(count as u64)
            .ok_or("write extent overflow")?;
        if count == 0 {
            return Ok(());
        }
        if end > self.capacity || end > 1u64 << 32 {
            return Err("write beyond partition");
        }
        loop {
            let r = if self.dma {
                self.mmc.write_dma(lba, count, src)
            } else {
                self.mmc.write(lba, count, src)
            };
            match r {
                Ok(()) => return Ok(()),
                Err(e) => {
                    println!(
                        "[fastboot] write error at {lba} ({}-bit, {} kHz, {}): {e:?}",
                        self.width,
                        self.khz,
                        if self.dma { "SDMA" } else { "PIO" }
                    );
                    loop {
                        if self.fallback == FALLBACK.len() {
                            return Err("write failed (see the serial console)");
                        }
                        let (width, hold, khz, dma) = FALLBACK[self.fallback];
                        self.fallback += 1;
                        if width > 4 && self.kind == Kind::Sd {
                            continue;
                        }
                        self.mmc.set_tx_hold(hold);
                        if self.mmc.set_width(width).is_err() {
                            continue;
                        }
                        if let Ok(k) = self.mmc.set_khz(khz) {
                            self.width = width;
                            self.khz = k;
                            self.dma = dma;
                            println!(
                                "[fastboot] retrying: {width}-bit bus, TX hold {hold}, {k} kHz, {}",
                                if dma { "SDMA" } else { "PIO" }
                            );
                            break;
                        }
                    }
                }
            }
        }
    }

    /// Zero `count` sectors: TRIM them when that leaves zeros (checked on
    /// the first and last sector), else write zeros from `zeros`.
    fn zero(
        &mut self,
        lba: u32,
        count: usize,
        zeros: usize,
        zeros_len: usize,
    ) -> Result<(), &'static str> {
        if count == 0 {
            return Ok(());
        }
        if (lba as u64)
            .checked_add(count as u64)
            .filter(|e| *e <= self.capacity && *e <= 1u64 << 32)
            .is_none()
        {
            return Err("zero beyond partition");
        }
        if self.trim {
            let mut ok = true;
            let mut done = 0;
            while done < count {
                let n = (count - done).min(1 << 17);
                if self.mmc.trim(lba + done as u32, n).is_err() {
                    ok = false;
                    break;
                }
                done += n;
            }
            if ok {
                let mut probe = [0xffu8; SECTOR];
                for s in [lba, lba + count as u32 - 1] {
                    let p = probe.as_mut_ptr() as usize;
                    if self.mmc.read_sector(s, p).is_err() || probe.iter().any(|b| *b != 0) {
                        ok = false;
                    }
                    probe = [0xff; SECTOR];
                }
            }
            if ok {
                return Ok(());
            }
            println!("[fastboot] TRIM did not leave zeros; writing them");
            self.trim = false;
        }
        let per = zeros_len / SECTOR;
        let mut done = 0;
        while done < count {
            let n = (count - done).min(per);
            self.write(lba + done as u32, n, zeros)?;
            done += n;
        }
        Ok(())
    }
}

/// Capacity in 512-byte sectors, for both eMMC partitions and SD CSD.
fn capacity(d: &Disk, part: Partition) -> Option<u64> {
    d.mmc.capacity_sectors(part).ok()
}

fn write_raw(
    usb: &mut Usb,
    d: &mut Disk,
    lba: u64,
    src: usize,
    len: usize,
) -> Result<(), &'static str> {
    let sectors = len.div_ceil(SECTOR);
    if lba
        .checked_add(sectors as u64)
        .filter(|e| *e <= d.capacity && *e <= 1u64 << 32)
        .is_none()
    {
        return Err("raw write beyond partition");
    }
    let mut done = 0;
    // 32 MiB at a time, with progress for the host.
    while done < sectors {
        let n = (sectors - done).min(65536);
        d.write((lba + done as u64) as u32, n, src + done * SECTOR)?;
        done += n;
        if sectors > 65536 {
            reply(
                usb,
                "INFO",
                format_args!("{} of {} MiB", done >> 11, sectors >> 11),
            );
        }
    }
    Ok(())
}

fn fill(value: u32) {
    for i in 0..FILL_LEN / 4 {
        unsafe { core::ptr::write_volatile((FILL_BUF + 4 * i) as *mut u32, value) };
    }
}

/// The sparse parser owns the validation pass; this adapter owns real I/O.
struct SparseDisk<'a, 'u> {
    usb: &'a mut Usb<'u>,
    disk: &'a mut Disk,
    pattern: Option<u32>,
}
impl crate::sparse::Writer for SparseDisk<'_, '_> {
    fn raw(&mut self, lba: u32, data: &[u8]) -> Result<(), &'static str> {
        write_raw(
            self.usb,
            self.disk,
            lba as u64,
            data.as_ptr() as usize,
            data.len(),
        )
    }
    fn fill(&mut self, lba: u32, sectors: u64, value: u32) -> Result<(), &'static str> {
        if self.pattern != Some(value) {
            fill(value);
            self.pattern = Some(value);
        }
        let sectors = usize::try_from(sectors).map_err(|_| "fill extent overflow")?;
        if value == 0 {
            return self.disk.zero(lba, sectors, FILL_BUF, FILL_LEN);
        }
        let mut done = 0;
        while done < sectors {
            let n = (sectors - done).min(FILL_LEN / SECTOR);
            let at = u32::try_from(lba as u64 + done as u64).map_err(|_| "fill LBA overflow")?;
            self.disk.write(at, n, FILL_BUF)?;
            done += n;
        }
        Ok(())
    }
}
fn write_sparse(usb: &mut Usb, d: &mut Disk, cap: u64, len: usize) -> Result<(), &'static str> {
    let bytes = unsafe { core::slice::from_raw_parts(BUF as *const u8, len) };
    crate::sparse::write(
        bytes,
        cap,
        &mut SparseDisk {
            usb,
            disk: d,
            pattern: None,
        },
    )
}

fn flash(
    usb: &mut Usb,
    disk: &mut Option<Disk>,
    board: &Board,
    name: &str,
    len: usize,
) -> Result<(), &'static str> {
    if len == 0 {
        return Err("download an image first");
    }
    let (kind, part) = target(name, board.disk).ok_or("unknown target")?;
    let d = Disk::open(disk, kind)?;
    if kind == Kind::Emmc {
        d.mmc
            .select(part)
            .map_err(|_| "cannot select the eMMC partition")?;
    }
    let cap = capacity(d, part).ok_or("cannot determine partition capacity")?;
    d.capacity = cap;
    if len >= 4 && le32(BUF) == SPARSE_MAGIC {
        return write_sparse(usb, d, cap, len);
    }
    if len.div_ceil(SECTOR) as u64 > cap {
        return Err("image larger than the partition");
    }
    // Zero the tail of a partial last sector.
    for a in BUF + len..BUF + len.next_multiple_of(SECTOR) {
        unsafe { core::ptr::write_volatile(a as *mut u8, 0) };
    }
    write_raw(usb, d, 0, BUF, len)
}

fn getvar(usb: &mut Usb, disk: &mut Option<Disk>, board: &Board, var: &str) {
    match var {
        "version" => reply(usb, "OKAY", format_args!("0.4")),
        "product" => reply(usb, "OKAY", format_args!("{}", board.name)),
        "serialno" => reply(usb, "OKAY", format_args!("k1")),
        "max-download-size" => reply(usb, "OKAY", format_args!("0x{:08x}", MAX_DOWNLOAD)),
        "is-userspace" | "secure" | "unlocked" => reply(
            usb,
            "OKAY",
            format_args!("{}", if var == "unlocked" { "yes" } else { "no" }),
        ),
        _ => {
            if let Some(p) = var.strip_prefix("partition-type:") {
                if target(p, board.disk).is_some() {
                    return reply(usb, "OKAY", format_args!("raw"));
                }
            } else if let Some(p) = var.strip_prefix("partition-size:") {
                if let Some((kind, part)) = target(p, board.disk) {
                    if let Ok(d) = Disk::open(disk, kind) {
                        if let Some(c) = capacity(d, part) {
                            return reply(usb, "OKAY", format_args!("0x{:x}", c * SECTOR as u64));
                        }
                    }
                }
            } else if var.starts_with("has-slot:") || var.starts_with("is-logical:") {
                return reply(usb, "OKAY", format_args!("no"));
            }
            reply(usb, "FAIL", format_args!("unknown variable"))
        }
    }
}

fn hex(s: &str) -> Option<usize> {
    usize::from_str_radix(s.trim_start_matches("0x"), 16).ok()
}

/// Only an acknowledged, quiesced `continue` may return to the boot chain.
pub fn run(board: &Board) -> Result<(), crate::usb::Error> {
    let mut usb = Usb::init(Descriptors {
        vendor: 0x18d1, // the IDs the fastboot tool and U-Boot use
        product: 0x4ee0,
        class: [0xff, 0x42, 0x03],
        strings: ["oreboot", board.name, "k1", "fastboot"],
    })?;
    let mut disk: Option<Disk> = None;
    let mut downloaded = 0;
    let mut cmd = [0u8; 64];
    loop {
        usb.wait_configured()?;
        let n = match usb.recv(&mut cmd) {
            Ok(n) => n,
            Err(crate::usb::Error::Reset) => continue,
            Err(e) => return Err(e),
        };
        let c = core::str::from_utf8(&cmd[..n]).unwrap_or("");
        println!("[fastboot] < {c}");
        if let Some(var) = c.strip_prefix("getvar:") {
            getvar(&mut usb, &mut disk, board, var);
        } else if let Some(size) = c.strip_prefix("download:") {
            match hex(size) {
                Some(len) if len > 0 && len <= MAX_DOWNLOAD => {
                    downloaded = 0;
                    reply_status(&mut usb, "DATA", format_args!("{:08x}", len))?;
                    match usb.recv_into(BUF, len) {
                        Ok(()) => {
                            downloaded = len;
                            reply_status(&mut usb, "OKAY", format_args!(""))?;
                        }
                        Err(e) => {
                            downloaded = 0;
                            println!("[fastboot] download failed: {e:?}");
                            return Err(e);
                        }
                    }
                }
                _ => reply(&mut usb, "FAIL", format_args!("bad download size")),
            }
        } else if let Some(name) = c.strip_prefix("flash:") {
            match flash(&mut usb, &mut disk, board, name, downloaded) {
                Ok(()) => reply(&mut usb, "OKAY", format_args!("")),
                Err(e) => reply(&mut usb, "FAIL", format_args!("{e}")),
            }
        } else if c == "continue" {
            let ack = reply_status(&mut usb, "OKAY", format_args!(""));
            let stopped = usb.disconnect();
            ack?;
            stopped?;
            return Ok(());
        } else if c == "reboot" || c.starts_with("reboot-") {
            reply_status(
                &mut usb,
                "FAIL",
                format_args!("reset unsupported; use continue to boot"),
            )?;
        } else {
            reply(&mut usb, "FAIL", format_args!("unknown command"));
        }
    }
}
