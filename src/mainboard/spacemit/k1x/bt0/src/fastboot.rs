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
//!
//! The protocol (commands, replies, downloads, the acknowledged `continue`)
//! and the raw/sparse image dispatch are the shared `fastboot` crate, the USB
//! transport is `ci_udc` (ore-edk2-boot). The K1 side stays here: the
//! targets, the card set-up and write fallbacks, TRIM zeroing, the sparse
//! writer with progress replies, and the download buffers in DRAM.

use core::cell::RefCell;
use core::fmt::Write;

use crate::mmc::{Kind, Mmc, Partition};
use crate::usb::{Descriptors, Usb};
use fastboot::{Line, Transport, SECTOR};

/// Download buffer in DRAM.
const BUF: usize = 0x1000_0000;
const MAX_DOWNLOAD: usize = 0x2000_0000;
/// Pattern buffer for sparse FILL chunks, after the download buffer.
const FILL_BUF: usize = BUF + MAX_DOWNLOAD;
const FILL_LEN: usize = 1 << 20;

pub struct Board {
    /// Reported as `product`.
    pub name: &'static str,
    /// The medium `disk` names.
    pub disk: Kind,
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
            let mut mmc = crate::mmc::init(kind).map_err(|_| "card did not initialise")?;
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
fn capacity(d: &mut Disk, part: Partition) -> Option<u64> {
    d.mmc.capacity_sectors(part).ok()
}

/// The USB transport with the K1 console log; shared with the device for
/// progress replies during long writes (never borrowed by both at once:
/// `fastboot::serve` holds no transport borrow while it calls the device).
struct K1Transport<'u, 'a>(&'u RefCell<Usb<'a>>);

impl Transport for K1Transport<'_, '_> {
    type Error = crate::usb::Error;
    fn wait_configured(&mut self) -> Result<(), Self::Error> {
        Transport::wait_configured(&mut *self.0.borrow_mut())
    }
    fn recv(&mut self, out: &mut [u8]) -> Result<usize, Self::Error> {
        Transport::recv(&mut *self.0.borrow_mut(), out)
    }
    fn recv_exact(&mut self, out: &mut [u8]) -> Result<(), Self::Error> {
        let r = Transport::recv_exact(&mut *self.0.borrow_mut(), out);
        if let Err(e) = r {
            println!("[fastboot] download failed: {e:?}");
        }
        r
    }
    fn send(&mut self, data: &[u8]) -> Result<(), Self::Error> {
        Transport::send(&mut *self.0.borrow_mut(), data)
    }
    fn disconnect(&mut self) -> Result<(), Self::Error> {
        Transport::disconnect(&mut *self.0.borrow_mut())
    }
    fn is_reset(e: &Self::Error) -> bool {
        <Usb as Transport>::is_reset(e)
    }
    fn log(&mut self, what: &str, line: &[u8]) {
        let s = core::str::from_utf8(line).unwrap_or("?");
        if what == "<" {
            println!("[fastboot] < {s}");
        } else {
            println!("[fastboot] {s}");
        }
    }
}

/// An INFO reply (progress) while the device is busy.
fn info(usb: &RefCell<Usb>, msg: core::fmt::Arguments) {
    let mut l = Line::new("INFO");
    let _ = l.write_fmt(msg);
    let _ = Transport::send(&mut *usb.borrow_mut(), l.as_bytes());
}

fn write_raw(
    usb: &RefCell<Usb>,
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
            info(usb, format_args!("{} of {} MiB", done >> 11, sectors >> 11));
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
struct SparseDisk<'a, 'u, 'b> {
    usb: &'a RefCell<Usb<'u>>,
    disk: &'b mut Disk,
    pattern: Option<u32>,
}
impl sparse::Writer for SparseDisk<'_, '_, '_> {
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

/// The K1 board's fastboot device: product name, targets and their storage.
struct K1Device<'u, 'a> {
    usb: &'u RefCell<Usb<'a>>,
    board: &'u Board,
    disk: Option<Disk>,
}

impl fastboot::Device for K1Device<'_, '_> {
    fn getvar(&mut self, var: &str, out: &mut Line) -> bool {
        match var {
            "product" => write!(out, "{}", self.board.name).is_ok(),
            "serialno" => write!(out, "k1").is_ok(),
            _ => {
                if let Some(p) = var.strip_prefix("partition-type:") {
                    return target(p, self.board.disk).is_some() && write!(out, "raw").is_ok();
                }
                if let Some(p) = var.strip_prefix("partition-size:") {
                    if let Some((kind, part)) = target(p, self.board.disk) {
                        if let Ok(d) = Disk::open(&mut self.disk, kind) {
                            if let Some(c) = capacity(d, part) {
                                return write!(out, "0x{:x}", c * SECTOR as u64).is_ok();
                            }
                        }
                    }
                }
                false
            }
        }
    }
    fn flash(
        &mut self,
        name: &str,
        image_len: usize,
        padded: &mut [u8],
    ) -> Result<(), &'static str> {
        let (kind, part) = target(name, self.board.disk).ok_or("unknown target")?;
        let d = Disk::open(&mut self.disk, kind)?;
        if kind == Kind::Emmc {
            d.mmc
                .select(part)
                .map_err(|_| "cannot select the eMMC partition")?;
        }
        let cap = capacity(d, part).ok_or("cannot determine partition capacity")?;
        d.capacity = cap;
        let mut w = SparseDisk {
            usb: self.usb,
            disk: d,
            pattern: None,
        };
        fastboot::flash_image(padded, image_len, cap, &mut w)
    }
}

/// Only an acknowledged, quiesced `continue` may return to the boot chain.
pub fn run(board: &Board) -> Result<(), crate::usb::Error> {
    let usb = RefCell::new(crate::usb::init(Descriptors {
        vendor: 0x18d1, // the IDs the fastboot tool and U-Boot use
        product: 0x4ee0,
        class: [0xff, 0x42, 0x03],
        strings: ["oreboot", board.name, "k1", "fastboot"],
    })?);
    let mut device = K1Device {
        usb: &usb,
        board,
        disk: None,
    };
    // SAFETY: the download buffer is DRAM bt0 owns; nothing else uses it.
    let buf = unsafe { core::slice::from_raw_parts_mut(BUF as *mut u8, MAX_DOWNLOAD) };
    fastboot::serve(
        &mut K1Transport(&usb),
        &mut device,
        &fastboot::Config { unlocked: true },
        buf,
    )
}
