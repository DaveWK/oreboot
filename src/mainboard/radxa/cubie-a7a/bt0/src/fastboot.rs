//! fastboot over FEL (feature `fastboot`): after DRAM is up, bt0 leaves the
//! mask ROM's FEL session on the USB-C port (USB0) and enumerates as a
//! standard fastboot device (18d1:4ee0), so the host uses the stock
//! `fastboot` tool instead of chained xfel calls (fel.rs entry 4).
//!
//! | Command                     | Here                                          |
//! |-----------------------------|-----------------------------------------------|
//! | `getvar`                    | product, serialno, DRAM, partitions, `all`    |
//! | `download`                  | to DRAM at [`BUF`], up to [`BUF_MAX`]         |
//! | `boot`                      | a bundle (layout table + images): checked,    |
//! |                             | then the FEL entry 2 hand-over to main        |
//! | `continue`                  | `boot` of the last download                   |
//! | `reboot`                    | watchdog reset of the board                   |
//! | `reboot-bootloader`, `oem fel` | back to the mask ROM's FEL (DRAM stays up) |
//! | `oem help/log/read32`       | diagnostics                                   |
//! | `flash`/`erase`, `oem nor-*`| SPI NOR, only with `fastboot-nor-write`       |
//! | `fetch:emmc*`, `oem emmc`   | eMMC module, read-only (DRAM extension)       |
//! | `flash`/`erase:emmc-*`, `oem emmc-arm/bootcfg` | eMMC, only with `fastboot-emmc-write` |
//!
//! A bundle is any cut of the boot medium that holds the layout table and
//! the images it names (cubie_a7a_fastboot::bundle): the image's
//! `firmware.bin`, the 8 MiB NOR image, or the medium from the table on.
//! `fastboot boot` wraps a plain file in an Android boot image; the kernel
//! of one is the bundle. Everything `boot` checks (table, extents against
//! DRAM, overlaps with the download, every CRC) is checked before the OKAY;
//! after it the images are copied, checked again in place and bt0 hands
//! over exactly as FEL entry 2 does.
use core::fmt::Write;
use core::ops::Range;
use core::sync::atomic::{AtomicBool, Ordering};

use cubie_a7a_common::{crc32, fwlog};
use cubie_a7a_fastboot::proto::{self, Command, Device, Exit, Info, Line};
use cubie_a7a_fastboot::{fetch, nor, plan, usb};
use util::mmio::{read32, read64le, read8};

use crate::musb::{self, Musb};
use crate::{soc, Bringup};

/// The download buffer in DRAM: above main (0x40000000), its log and
/// handoff (below 0x40200000) and the EDK2 FD (0x41000000, 4 MiB), so a
/// bundle's images never land in it (plan::check checks that too).
pub const BUF: usize = 0x4800_0000;
pub const BUF_MAX: usize = 0x0800_0000;
/// Read-back scratch for NOR and eMMC writes, after the download buffer.
#[cfg(any(feature = "fastboot-nor-write", feature = "fastboot-emmc-write"))]
const SCRATCH: usize = BUF + BUF_MAX;
#[cfg(any(feature = "fastboot-nor-write", feature = "fastboot-emmc-write"))]
const SCRATCH_LEN: usize = 0x1_0000;

/// USB sessions (take-over, serve) before bt0 gives up and goes back to FEL.
const SESSIONS: u32 = 4;

/// The checked plan of a `boot`: a bundle in the download buffer.
#[derive(Clone, Copy)]
pub struct Plan {
    checked: plan::Checked,
    /// The download.
    image: usize,
    len: usize,
}

impl Plan {
    fn image(&self) -> &'static [u8] {
        // SAFETY: the download buffer, which only fastboot writes.
        unsafe { core::slice::from_raw_parts(self.image as *const u8, self.len) }
    }
}

/// After the acknowledgement: copy the images, check them in place, hand
/// over to main (fel.rs, as entry 2). Back to FEL if a copy is bad.
fn execute(b: &Bringup, p: Plan) -> ! {
    let mut handoff = crate::handoff(b);
    let mut main_entry = None;
    let mut bad = false;
    let placed = plan::for_each_image(&p.checked, p.image(), b.dram_end as u64, |e, addr, src| {
        let t0 = soc::time_us();
        // SAFETY: plan::check validated the extent against DRAM, the other
        // images and the download buffer.
        let dst = unsafe { core::slice::from_raw_parts_mut(addr as *mut u8, src.len()) };
        dst.copy_from_slice(src);
        let crc = crc32(dst);
        if e.crc32 != 0 && crc != e.crc32 {
            println!(
                "[fastboot] kind {} at 0x{addr:08x}: CRC {crc:08x} after the copy, table {:08x}",
                e.kind, e.crc32
            );
            bad = true;
            return;
        }
        println!(
            "[fastboot] kind {} ({} KiB) at 0x{addr:08x}{} ({} ms)",
            e.kind,
            e.size / 1024,
            if e.crc32 != 0 { ", CRC ok" } else { "" },
            (soc::time_us() - t0) / 1000
        );
        if let Some(entry) = crate::place(&mut handoff, e, addr) {
            main_entry = Some(entry);
        }
    });
    match (placed, main_entry, bad) {
        (Ok(()), Some(entry), false) => crate::fel::hand_over(b, handoff, entry),
        _ => {
            println!("[fastboot] the images in DRAM do not match the bundle; back to FEL");
            crate::fel::halt_to_fel()
        }
    }
}

const NAMES: &[&str] = &[
    "product",
    "serialno",
    "version-bootloader",
    "secure",
    "unlocked",
    "hw-revision",
    "boot-mode",
    "dram-size",
    "dram-mhz",
    "usb-speed",
    "nor-writes",
    "emmc-writes",
    "partition-type:bt0",
    "partition-size:bt0",
    "partition-type:layout",
    "partition-size:layout",
    "partition-type:main",
    "partition-size:main",
    "partition-type:edk2",
    "partition-size:edk2",
    "partition-type:nor",
    "partition-size:nor",
];

const HELP: &[&str] = &[
    "oem log [hex bytes]: tail of the bt0 console log",
    "oem read32 <hex addr>: one 32-bit register read",
    "oem fel: back to the mask ROM's FEL (as reboot-bootloader)",
    "oem nor-id: SPI NOR JEDEC ID and status register 1",
    "oem usb0: open the blue-port USB disk, show it and its GPT",
    "oem sd: identify the SD card",
    "oem emmc: identify the eMMC (CID, CSD, EXT_CSD fields, GPT)",
    "fetch: nor bt0 layout main edk2 usb0 usb0p<N> sd [@hexoff+hexlen]",
    "fetch: emmc emmc-boot0 emmc-boot1 emmcp<N> emmc-extcsd [@hexoff+hexlen]",
];
#[cfg(feature = "fastboot-emmc-write")]
const HELP_EMMC: &[&str] = &[
    "oem emmc-arm <emmc-fw|emmc-boot0|emmc-boot1|emmc@off+len> flash <len> <crc32> | <t> erase",
    "oem emmc-arm bootcfg <pc> <bb>; oem emmc-bootcfg <pc> <bb>",
];
#[cfg(not(feature = "fastboot-emmc-write"))]
const HELP_EMMC: &[&str] = &[];
#[cfg(feature = "fastboot-nor-write")]
const HELP_NOR: &[&str] = &["oem nor-arm <part> flash <hex len> <hex crc32> | <part> erase"];
#[cfg(not(feature = "fastboot-nor-write"))]
const HELP_NOR: &[&str] = &[];

/// A fresh eMMC write guard, built at compile time (no call into the DRAM
/// extension).
#[cfg(feature = "fastboot-emmc-write")]
const EMMC_GUARD: cubie_a7a_fastboot::emmcarm::Guard =
    cubie_a7a_fastboot::emmcarm::Guard::new(true);

/// The negotiated speed, for `getvar:usb-speed`.
static HIGH_SPEED: AtomicBool = AtomicBool::new(true);

struct A7a<'b> {
    b: &'b Bringup,
    serial: &'b str,
    guard: nor::Guard,
    nor: Option<crate::spinor::SpiNor>,
    usb: Option<crate::ext::UsbDisk>,
    sd: Option<crate::ext::SdCard>,
    emmc: Option<crate::ext::Emmc>,
    #[cfg(feature = "fastboot-emmc-write")]
    eguard: cubie_a7a_fastboot::emmcarm::Guard,
}

/// A read-only device behind a fetch target.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Dev {
    Nor,
    Usb,
    Sd,
    /// Sizes only; eMMC reads go through `ext::emmc_fetch`.
    Emmc,
}

fn now_ms() -> u64 {
    soc::time_us() / 1000
}

/// The tail of bt0's console log (cubie_a7a_common::fwlog), `n` bytes,
/// as INFO lines.
fn oem_log(n: usize, info: &mut dyn Info) -> Result<(), &'static str> {
    let base = fwlog::CUR_BASE;
    if (0..8).any(|i| read8(base + i) != fwlog::MAGIC[i]) {
        return Err("no console log in DRAM");
    }
    let written = read64le(base + 16);
    let n = (n as u64).min(written).min(fwlog::DATA_SIZE as u64);
    let mut line = [0u8; 56];
    let mut len = 0;
    let mut emit = |line: &[u8]| {
        info.info(format_args!(
            "{}",
            core::str::from_utf8(line).unwrap_or("?")
        ));
    };
    for i in written - n..written {
        let c = read8(base + fwlog::HEADER_SIZE + (i % fwlog::DATA_SIZE as u64) as usize);
        if c == b'\n' || len == line.len() {
            emit(&line[..len]);
            len = 0;
            if c == b'\n' {
                continue;
            }
        }
        if (0x20..0x7f).contains(&c) {
            line[len] = c;
            len += 1;
        }
    }
    if len > 0 {
        emit(&line[..len]);
    }
    Ok(())
}

/// `oem read32 <addr>`: one aligned 32-bit read. Not the USB0 FIFOs
/// (reading one would take fastboot's own packets).
fn oem_read32(arg: &str, info: &mut dyn Info) -> Result<(), &'static str> {
    let a = proto::hex_u32(arg).ok_or("usage: oem read32 <hex address>")? as usize;
    if !a.is_multiple_of(4) {
        return Err("address not 4-byte aligned");
    }
    if (0x0410_0000..0x0410_0040).contains(&a) {
        return Err("USB0 FIFO window: refused");
    }
    let v = read32(a);
    println!("[fastboot] read32 0x{a:08x}: 0x{v:08x}");
    info.info(format_args!("0x{a:08x}: 0x{v:08x}"));
    Ok(())
}

impl A7a<'_> {
    fn nor(&mut self) -> Result<&mut crate::spinor::SpiNor, &'static str> {
        if self.nor.is_none() {
            match crate::spinor::SpiNor::new() {
                Ok(n) => self.nor = Some(n),
                Err(e) => {
                    let _ = e;
                    println!("[fastboot] SPI NOR controller init failed");
                    return Err("SPI NOR controller init failed");
                }
            }
        }
        Ok(self.nor.as_mut().unwrap())
    }

    /// The device, base offset and size in bytes of a target name (no
    /// window).
    fn target(&mut self, name: &str) -> Result<(Dev, u64, u64), &'static str> {
        if name == "nor" {
            return Ok((Dev::Nor, 0, nor::NOR_SIZE as u64));
        }
        if let Some(p) = nor::partition(name) {
            return Ok((Dev::Nor, p.offset as u64, p.size as u64));
        }
        if name == "usb0" || name.starts_with("usb0p") {
            let i = match name.strip_prefix("usb0p") {
                Some(n) => Some(n.parse::<usize>().map_err(|_| "unknown partition")?),
                None => None,
            };
            let u = self.usb()?;
            let (at, size) = crate::ext::usb_extent(u, i)?;
            return Ok((Dev::Usb, at, size));
        }
        if name == "sd" {
            let c = self.sd()?;
            return Ok((Dev::Sd, 0, crate::ext::sd_size(c)));
        }
        if name.starts_with("emmc") {
            let d = self.emmc()?;
            return Ok((Dev::Emmc, 0, crate::ext::emmc_size(d, name)?));
        }
        Err("unknown partition")
    }

    /// Read [start, start + len) of `dev` into `buf`; where the bytes are.
    fn read(
        &mut self,
        dev: Dev,
        start: u64,
        len: u64,
        buf: &mut [u8],
    ) -> Result<Range<usize>, &'static str> {
        let len = len as usize;
        match dev {
            Dev::Nor => {
                let f = self.nor()?;
                f.read(start as u32, &mut buf[..len]).map_err(|_| {
                    println!("[fastboot] nor read at {start:#x} failed");
                    "SPI NOR read failed"
                })?;
                Ok(0..len)
            }
            Dev::Usb => {
                let u = self.usb()?;
                crate::ext::usb_read(u, start, len as u64, buf)
            }
            Dev::Sd => {
                let c = self.sd()?;
                crate::ext::sd_read(c, start, len as u64, buf)
            }
            Dev::Emmc => Err("eMMC reads go through emmc_fetch"),
        }
    }

    /// The blue-port disk, opened on first use (needs the DRAM part).
    fn usb(&mut self) -> Result<&mut crate::ext::UsbDisk, &'static str> {
        if self.usb.is_none() {
            crate::extcheck::ready()?;
            self.usb = Some(crate::ext::usb_open()?);
        }
        Ok(self.usb.as_mut().unwrap())
    }

    /// The SD card, identified on first use (needs the DRAM part).
    fn sd(&mut self) -> Result<&mut crate::ext::SdCard, &'static str> {
        if self.sd.is_none() {
            crate::extcheck::ready()?;
            self.sd = Some(crate::ext::sd_open()?);
        }
        Ok(self.sd.as_mut().unwrap())
    }

    /// The eMMC module, identified on first use (needs the DRAM part).
    fn emmc(&mut self) -> Result<&mut crate::ext::Emmc, &'static str> {
        if self.emmc.is_none() {
            crate::extcheck::ready()?;
            self.emmc = Some(crate::ext::emmc_open()?);
        }
        Ok(self.emmc.as_mut().unwrap())
    }

    #[cfg(feature = "fastboot-emmc-write")]
    fn emmc_flash(
        &mut self,
        part: &str,
        image: &[u8],
        info: &mut dyn Info,
    ) -> Result<(), &'static str> {
        // The arm first: no eMMC access unless this exact flash is armed
        // (the guard lives in the DRAM extension, as the writer).
        crate::extcheck::ready()?;
        let t = self.eguard.take(part, Some(image), now_ms())?;
        let dram_end = self.b.dram_end as u64;
        let d = self.emmc()?;
        crate::ext::emmc_flash(d, &t, image, dram_end, scratch_words(), info)
    }

    #[cfg(feature = "fastboot-emmc-write")]
    fn emmc_erase(&mut self, part: &str, info: &mut dyn Info) -> Result<(), &'static str> {
        crate::extcheck::ready()?;
        let t = self.eguard.take(part, None, now_ms())?;
        let d = self.emmc()?;
        crate::ext::emmc_erase(d, &t, scratch_words(), info)
    }

    #[cfg(not(feature = "fastboot-emmc-write"))]
    fn emmc_flash(
        &mut self,
        _p: &str,
        _i: &[u8],
        _info: &mut dyn Info,
    ) -> Result<(), &'static str> {
        Err("eMMC writes are not built in (bt0 feature fastboot-emmc-write)")
    }

    #[cfg(not(feature = "fastboot-emmc-write"))]
    fn emmc_erase(&mut self, _p: &str, _info: &mut dyn Info) -> Result<(), &'static str> {
        Err("eMMC writes are not built in (bt0 feature fastboot-emmc-write)")
    }

    /// Stop what fastboot started besides USB0 (before the hand-over or
    /// the return to FEL).
    fn quiesce(&mut self) {
        if let Some(u) = self.usb.as_mut() {
            crate::ext::usb_close(u);
        }
        self.usb = None;
    }
}

#[cfg(feature = "fastboot-nor-write")]
fn scratch() -> &'static mut [u8] {
    // SAFETY: DRAM after the download buffer, used only here.
    unsafe { core::slice::from_raw_parts_mut(SCRATCH as *mut u8, SCRATCH_LEN) }
}

#[cfg(feature = "fastboot-emmc-write")]
fn scratch_words() -> &'static mut [u32] {
    // SAFETY: DRAM after the download buffer (aligned), used only here.
    unsafe { core::slice::from_raw_parts_mut(SCRATCH as *mut u32, SCRATCH_LEN / 4) }
}

/// The layout partition's content check: a valid frame for this board
/// whose images all lie in the main and EDK2 partitions.
#[cfg(feature = "fastboot-nor-write")]
fn check_nor_image(b: &Bringup, p: &nor::Partition, image: &[u8]) -> Result<(), &'static str> {
    use cubie_a7a_common::layout;
    use cubie_a7a_fastboot::bundle;
    if p.content != nor::Content::Layout {
        return Ok(());
    }
    let frame = image
        .get(..bundle::SECTOR)
        .ok_or("layout frame shorter than 512 bytes")?;
    let t = layout::decode(frame).map_err(|_| "not a valid A7ABOOT1 layout frame")?;
    layout::validate(&t, b.dram_end as u64).map_err(|_| "layout table does not fit this board")?;
    for e in &t.entries[..t.count as usize] {
        if e.kind == layout::KIND_VENDOR_SPL {
            return Err("vendor SPL entry: not on the NOR");
        }
        if nor::holding(e.offset, e.size, nor::Content::Raw).is_none() {
            return Err("an image lies outside the main and edk2 partitions");
        }
    }
    Ok(())
}

impl Device for A7a<'_> {
    type Boot = Plan;

    fn getvar(&mut self, name: &str, out: &mut Line) -> bool {
        let b = self.b;
        match name {
            "product" => write!(out, "cubie-a7a"),
            "serialno" => write!(out, "{}", self.serial),
            "version-bootloader" => {
                write!(out, "oreboot-bt0-{}-fastboot", env!("CARGO_PKG_VERSION"))
            }
            "secure" => write!(out, "no"),
            "unlocked" => write!(out, "yes"),
            "hw-revision" => write!(out, "{}", (b'A' + b.version as u8) as char),
            "boot-mode" => write!(out, "fel"),
            "dram-size" => write!(out, "{} MiB", b.dram_mb),
            "dram-mhz" => write!(out, "{}", b.dram_mhz),
            "usb-speed" => write!(
                out,
                "{}",
                if HIGH_SPEED.load(Ordering::Relaxed) {
                    "high"
                } else {
                    "full"
                }
            ),
            "emmc-writes" => write!(
                out,
                "{}",
                if cfg!(feature = "fastboot-emmc-write") {
                    "built in"
                } else {
                    "no"
                }
            ),
            "nor-writes" => write!(
                out,
                "{}",
                if self.guard.enabled() {
                    "built in"
                } else {
                    "no"
                }
            ),
            _ => {
                let (p, size) = if let Some(p) = name.strip_prefix("partition-type:") {
                    (p, false)
                } else if let Some(p) = name.strip_prefix("partition-size:") {
                    (p, true)
                } else {
                    return false;
                };
                let Ok((base, window)) = fetch::split_window(p) else {
                    return false;
                };
                // Sizes open usb0/sd on first use (seconds).
                let Ok((_, _, part)) = self.target(base) else {
                    return false;
                };
                match fetch::target_size(part, window) {
                    Ok(n) if size => write!(out, "0x{n:x}"),
                    Ok(_) => write!(out, "raw"),
                    Err(_) => return false,
                }
            }
        }
        .is_ok()
    }

    fn getvar_names(&self) -> &'static [&'static str] {
        NAMES
    }

    fn command(&mut self, c: &Command) {
        self.guard.observe(c);
        #[cfg(feature = "fastboot-emmc-write")]
        self.eguard.observe(c);
    }

    #[cfg(feature = "fastboot-nor-write")]
    fn flash(&mut self, part: &str, image: &[u8], info: &mut dyn Info) -> Result<(), &'static str> {
        if part.starts_with("emmc") {
            return self.emmc_flash(part, image, info);
        }
        let b = self.b;
        // The arm first: no SPI controller access unless this exact
        // flash is armed (and the arm is used up whatever happens next).
        let p = self.guard.take(part, Some(image), now_ms())?;
        let f = self.nor()?;
        nor::program_partition(
            f,
            p,
            image,
            &mut |p, i| check_nor_image(b, p, i),
            scratch(),
            info,
        )
        .map_err(|e| {
            println!("[fastboot] flash {part}: {e:?}");
            e.message()
        })
    }

    #[cfg(not(feature = "fastboot-nor-write"))]
    fn flash(&mut self, part: &str, image: &[u8], info: &mut dyn Info) -> Result<(), &'static str> {
        if part.starts_with("emmc") {
            return self.emmc_flash(part, image, info);
        }
        Err("NOR writes are not built in (bt0 feature fastboot-nor-write)")
    }

    #[cfg(feature = "fastboot-nor-write")]
    fn erase(&mut self, part: &str, info: &mut dyn Info) -> Result<(), &'static str> {
        if part.starts_with("emmc") {
            return self.emmc_erase(part, info);
        }
        let p = self.guard.take(part, None, now_ms())?;
        let f = self.nor()?;
        nor::erase_partition(f, p, scratch(), info).map_err(|e| {
            println!("[fastboot] erase {part}: {e:?}");
            e.message()
        })
    }

    #[cfg(not(feature = "fastboot-nor-write"))]
    fn erase(&mut self, part: &str, info: &mut dyn Info) -> Result<(), &'static str> {
        if part.starts_with("emmc") {
            return self.emmc_erase(part, info);
        }
        Err("NOR writes are not built in (bt0 feature fastboot-nor-write)")
    }

    fn boot(&mut self, image: Option<&[u8]>, info: &mut dyn Info) -> Result<Plan, &'static str> {
        let image = image.ok_or("download a bundle first")?;
        let t0 = soc::time_us();
        let r = plan::check(image, image.as_ptr() as u64, self.b.dram_end as u64, info).map(
            |checked| Plan {
                checked,
                image: image.as_ptr() as usize,
                len: image.len(),
            },
        );
        match &r {
            Ok(_) => {
                println!(
                    "[fastboot] bundle checked in {} ms",
                    (soc::time_us() - t0) / 1000
                );
            }
            Err(e) => {
                println!("[fastboot] bundle refused: {e}");
            }
        }
        r
    }

    fn continue_boot(
        &mut self,
        image: Option<&[u8]>,
        info: &mut dyn Info,
    ) -> Result<Plan, &'static str> {
        if image.is_none() {
            return Err("nothing staged; fastboot stage <bundle> first");
        }
        self.boot(image, info)
    }

    fn reboot(&mut self, _bootloader: bool) -> Result<(), &'static str> {
        Ok(())
    }

    fn fetch(
        &mut self,
        part: &str,
        offset: u64,
        size: Option<u64>,
        buf: &mut [u8],
        _info: &mut dyn Info,
    ) -> Result<Range<usize>, &'static str> {
        let (base, window) = fetch::split_window(part)?;
        let max = proto::MAX_FETCH.min(buf.len()) as u64;
        let t0 = soc::time_us();
        let (r, len) = if base.starts_with("emmc") {
            let d = self.emmc()?;
            let r = crate::ext::emmc_fetch(d, base, window, offset, size, max, buf)?;
            let n = r.len() as u64;
            (r, n)
        } else {
            let (dev, at, part_size) = self.target(base)?;
            let (start, len) = fetch::resolve(part_size, window, offset, size, max)?;
            (self.read(dev, at + start, len, buf)?, len)
        };
        let us = (soc::time_us() - t0).max(1);
        if offset == 0 || (offset & 0xfff_ffff) == 0 {
            println!(
                "[fastboot] fetch {part} +{offset:#x}: {len} bytes in {} ms ({} KiB/s)",
                us / 1000,
                len * 1_000_000 / 1024 / us
            );
        }
        Ok(r)
    }

    fn oem(
        &mut self,
        args: &str,
        _image: Option<&[u8]>,
        info: &mut dyn Info,
    ) -> Result<Option<Exit<Plan>>, &'static str> {
        let (cmd, rest) = args.split_once(' ').unwrap_or((args, ""));
        match cmd {
            "help" | "" => {
                for l in HELP.iter().chain(HELP_NOR).chain(HELP_EMMC) {
                    info.info(format_args!("{l}"));
                }
                Ok(None)
            }
            "log" => {
                let n = if rest.is_empty() {
                    4096
                } else {
                    proto::hex_u32(rest).ok_or("usage: oem log [hex bytes]")? as usize
                };
                oem_log(n.min(0x8000), info).map(|()| None)
            }
            "read32" => oem_read32(rest, info).map(|()| None),
            "fel" => Ok(Some(Exit::Bootloader)),
            "nor-id" => {
                let f = self.nor()?;
                let id = f.id().map_err(|_| "RDID failed")?;
                let sr = f.status1().map_err(|_| "RDSR failed")?;
                info.info(format_args!(
                    "JEDEC ID {:02x} {:02x} {:02x}, SR1 0x{sr:02x}",
                    id[0], id[1], id[2]
                ));
                Ok(None)
            }
            "usb0" => {
                let u = self.usb()?;
                crate::ext::usb_info(u, info);
                Ok(None)
            }
            "sd" => {
                let c = self.sd()?;
                crate::ext::sd_info(c, info);
                Ok(None)
            }
            "emmc" => {
                // Identify afresh (after a write the GPT may have changed).
                self.emmc = None;
                let d = self.emmc()?;
                crate::ext::emmc_info(d, info);
                Ok(None)
            }
            #[cfg(feature = "fastboot-emmc-write")]
            "emmc-arm" => {
                crate::extcheck::ready()?;
                self.eguard.arm(rest, now_ms())?;
                println!("[fastboot] eMMC armed: {rest}");
                info.info(format_args!(
                    "armed: {rest} (next matching command only, 60 s)"
                ));
                Ok(None)
            }
            #[cfg(feature = "fastboot-emmc-write")]
            "emmc-bootcfg" => {
                crate::extcheck::ready()?;
                let (pc, bb) = self.eguard.take_boot_config(rest, now_ms())?;
                let d = self.emmc()?;
                crate::ext::emmc_boot_config(d, pc, bb, info).map(|()| None)
            }
            #[cfg(feature = "fastboot-nor-write")]
            "nor-arm" => {
                let (p, op) = self.guard.arm(rest, now_ms())?;
                println!("[fastboot] NOR armed: {} {op:?}", p.name);
                info.info(format_args!(
                    "armed: {} {:?} (next flash/erase only, 60 s)",
                    p.name, op
                ));
                Ok(None)
            }
            _ => Err("unknown oem command (oem help)"),
        }
    }
}

/// Why fastboot gave the board back to FEL.
#[derive(Clone, Copy, Debug)]
pub enum Back {
    /// `reboot-bootloader` or `oem fel`.
    Asked = 1,
    /// The USB sessions kept failing.
    Usb = 2,
}

/// Serve fastboot until the host boots a bundle (no return), resets the
/// board (no return), or asks for FEL (returns; the caller puts the ROM's
/// USB set-up back and returns to the ROM).
pub fn run(b: &Bringup) -> Back {
    let mut hex = [0u8; 16];
    let serial = usb::serial(soc::chip_sid(), &mut hex);
    let desc = usb::Descriptors {
        vendor: usb::FASTBOOT_VID,
        product: usb::FASTBOOT_PID,
        class: usb::FASTBOOT_CLASS,
        strings: ["oreboot", "Radxa Cubie A7A bt0", serial, "fastboot"],
    };
    let max = BUF_MAX.min(b.dram_end.saturating_sub(BUF + 0x10_0000));
    // SAFETY: DRAM bt0 tested and owns; nothing else uses this range.
    let buf = unsafe { core::slice::from_raw_parts_mut(BUF as *mut u8, max) };
    for session in 1..=SESSIONS {
        let (mut usb, found) = Musb::take_over(desc);
        println!(
            "[fastboot] USB0 {} (ROM: POWER {:#04x} FADDR {} ISCR {:#010x}, CCU USB0 {:#010x}/{:#010x}); \
             fastboot 18d1:4ee0 serial {serial}, download buffer 0x{BUF:08x} + 0x{max:x}",
            if found.rom_ready { "taken over from the mask ROM" } else { "brought up (the ROM had it off)" },
            found.rom_power,
            found.rom_faddr,
            found.iscr,
            found.usb0_clk,
            found.usb0_bgr
        );
        let mut dev = A7a {
            b,
            serial,
            guard: nor::Guard::new(cfg!(feature = "fastboot-nor-write")),
            nor: None,
            usb: None,
            sd: None,
            emmc: None,
            #[cfg(feature = "fastboot-emmc-write")]
            eguard: EMMC_GUARD,
        };
        let r = proto::serve(&mut Speed(&mut usb, 0), &mut dev, buf);
        dev.quiesce();
        match r {
            Ok(Exit::Boot(plan)) => {
                musb::release();
                println!("[fastboot] boot: USB0 off the bus, copying the images");
                execute(b, plan)
            }
            Ok(Exit::Reset) => {
                musb::release();
                println!("[fastboot] reboot: watchdog reset");
                cubie_a7a_common::uart::Uart::attach(crate::UART0_BASE).drain();
                soc::watchdog_reset()
            }
            Ok(Exit::Bootloader) => {
                println!("[fastboot] back to the mask ROM's FEL");
                return Back::Asked;
            }
            Err(e) => {
                println!(
                    "[fastboot] USB session {session} ended: {}; enumerating again",
                    e.message()
                );
            }
        }
    }
    println!("[fastboot] giving up after {SESSIONS} USB sessions; back to FEL");
    Back::Usb
}

/// The controller, with the negotiated speed noted for `getvar` and the
/// download rate on the console. The second field counts consecutive
/// `fetch` commands: their command lines are logged one in 64 (a disk
/// dump is thousands of them, and the UART is slow).
struct Speed<'u, 'a>(&'u mut Musb<'a>, u32);

impl proto::Transport for Speed<'_, '_> {
    type Error = musb::Error;
    fn wait_configured(&mut self) -> Result<(), musb::Error> {
        let r = self.0.wait_configured();
        HIGH_SPEED.store(self.0.high_speed(), Ordering::Relaxed);
        r
    }
    fn recv(&mut self, out: &mut [u8]) -> Result<usize, musb::Error> {
        self.0.recv(out)
    }
    fn recv_exact(&mut self, out: &mut [u8]) -> Result<(), musb::Error> {
        let t0 = soc::time_us();
        let r = self.0.recv_exact(out);
        let us = (soc::time_us() - t0).max(1);
        match r {
            Ok(()) => {
                println!(
                    "[fastboot] downloaded {} bytes in {} ms ({} KiB/s)",
                    out.len(),
                    us / 1000,
                    out.len() as u64 * 1_000_000 / 1024 / us
                );
            }
            Err(e) => {
                println!("[fastboot] download failed: {}", e.message());
            }
        }
        r
    }
    fn send(&mut self, data: &[u8]) -> Result<(), musb::Error> {
        self.0.send(data)
    }
    fn send_data(&mut self, data: &[u8]) -> Result<(), musb::Error> {
        self.0.send_data(data)
    }
    fn disconnect(&mut self) -> Result<(), musb::Error> {
        self.0.disconnect()
    }
    fn is_reset(e: &musb::Error) -> bool {
        <Musb as proto::Transport>::is_reset(e)
    }
    fn log(&mut self, dir: &str, line: &[u8]) {
        if dir == "<" {
            self.1 = if line.starts_with(b"fetch:") {
                self.1 + 1
            } else {
                0
            };
        }
        if self.1 <= 1 || self.1.is_multiple_of(64) {
            self.0.log(dir, line)
        }
    }
}
