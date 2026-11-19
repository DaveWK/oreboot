// SPDX-License-Identifier: GPL-2.0-only

//! The DRAM part of the fastboot bt0 (feature `fastboot`): the block
//! device dumps (`fetch:usb0`, `usb0p<N>`, `sd`, `emmc*`) and the guarded
//! eMMC writer (`fastboot-emmc-write`), which do not fit the FEL
//! SRAM window beside the DRAM bring-up.
//!
//! build.rs links this module, `ehci.rs` and the dump modules of
//! cubie-a7a-fastboot (`msc`, `gpt`, `usbhost`, `sd`, `emmc`, including
//! the `SdBus` and `MmcBus` impls in smhc.rs) into the output section `.dram` at [`BASE`]:
//! the same ELF, split by objcopy into `bt0-*.bin` (the SRAM image, as
//! before) and `bt0-*-ext.bin`. The host writes the extension to [`BASE`]
//! once DRAM is up (after FEL entry 1) and its length and CRC-32 to the
//! mailbox; entry 4 checks them (`extcheck.rs`, in SRAM) and everything
//! here is reached only after `extcheck::ready`. Every entry point is
//! `#[inline(never)]`, so LTO cannot pull its body into SRAM callers. Without the extension, fastboot runs as before,
//! NOR dumps included (`spinor.rs` stays in SRAM); `usb0` and `sd` answer
//! "DRAM extension not loaded".
//!
//! Calls between SRAM and DRAM go through the linker's long-branch thunks.
//! The extension's exception index entries are discarded (bt0 never
//! unwinds; PREL31 cannot span SRAM to 0x59000000).
use core::ops::Range;

use cubie_a7a_fastboot::emmcarm::Part;
use cubie_a7a_fastboot::proto::Info;
use cubie_a7a_fastboot::{emmc, fetch, gpt, msc, sd};

use crate::{ehci, smhc, soc};

/// The blue-port disk: EHCI1, the Bulk-Only disk, its GPT partitions.
pub struct UsbDisk {
    host: ehci::Ehci,
    disk: msc::Disk,
    parts: GptParts,
}

type GptParts = Result<[Option<gpt::Part>; gpt::MAX_PARTS], &'static str>;

fn gpt_part(parts: &GptParts, index: usize) -> Result<gpt::Part, &'static str> {
    parts
        .as_ref()
        .map_err(|e| *e)?
        .get(index.checked_sub(1).ok_or("no such GPT partition")?)
        .copied()
        .flatten()
        .ok_or("no such GPT partition")
}

/// DRAM for the GPT reads (not the download buffer: `getvar` may open the
/// disk while a bundle is staged).
const GPT_BUF: usize = ehci::DMA + 0x1_0000;
const GPT_BUF_LEN: usize = 0x1_0000;

/// Open the blue-port disk (EHCI1, enumeration, SCSI, GPT): ~1-4 s.
#[inline(never)]
pub fn usb_open() -> Result<UsbDisk, &'static str> {
    let t0 = soc::time_us();
    let (mut host, found) = ehci::Ehci::open().map_err(|e| {
        println!("[fastboot] usb0: {} ({:#x})", e.message(), e.detail());
        e.message()
    })?;
    let mut disk = match msc::Disk::open(&mut host, 0) {
        Ok(d) => d,
        Err(e) => {
            println!("[fastboot] usb0: {}", e.message());
            host.shutdown();
            return Err(e.message());
        }
    };
    // SAFETY: DRAM reserved for this (ehci::DMA layout).
    let g = unsafe { core::slice::from_raw_parts_mut(GPT_BUF as *mut u8, GPT_BUF_LEN) };
    let bs = disk.block_size as usize;
    let parts = disk
        .read(&mut host, 1, &mut g[..bs])
        .map_err(|e| e.message())
        .and_then(|()| {
            let h = gpt::header(&mut g[..bs], disk.blocks)?;
            let array = g
                .get_mut(..h.entries_read_len())
                .ok_or("GPT entry array too large")?;
            disk.read(&mut host, h.entries_lba, array)
                .map_err(|e| e.message())?;
            gpt::parts(&h, array, disk.blocks)
        });
    let note = parts.as_ref().map_or_else(|e| *e, |_| "GPT ok");
    println!(
        "[fastboot] usb0: {:04x}:{:04x} '{}', {} blocks of {} ({} MiB), {note}, {} ms (RESCAL_CTRL was {:#010x})",
        found.vid,
        found.pid,
        core::str::from_utf8(&disk.ident).unwrap_or("?").trim_end(),
        disk.blocks,
        disk.block_size,
        (disk.blocks * disk.block_size as u64) >> 20,
        (soc::time_us() - t0) / 1000,
        found.rescal_before
    );
    Ok(UsbDisk { host, disk, parts })
}

/// Byte offset and size of the whole disk (`part` None) or GPT partition
/// `part` (1-based).
#[inline(never)]
pub fn usb_extent(u: &UsbDisk, part: Option<usize>) -> Result<(u64, u64), &'static str> {
    let bs = u.disk.block_size as u64;
    match part {
        None => Ok((0, u.disk.blocks * bs)),
        Some(i) => {
            let p = gpt_part(&u.parts, i)?;
            Ok((p.first * bs, (p.last - p.first + 1) * bs))
        }
    }
}

/// Read [start, start + len) into `buf` (whole blocks); where the bytes
/// are.
#[inline(never)]
pub fn usb_read(
    u: &mut UsbDisk,
    start: u64,
    len: u64,
    buf: &mut [u8],
) -> Result<Range<usize>, &'static str> {
    let bs = u.disk.block_size;
    let (lba, count, skip) = fetch::blocks(start, len, bs);
    let n = count as usize * bs as usize;
    if n > buf.len() {
        return Err("fetch too large for the buffer");
    }
    u.disk.read(&mut u.host, lba, &mut buf[..n]).map_err(|e| {
        println!("[fastboot] usb0 read at block {lba}: {}", e.message());
        e.message()
    })?;
    Ok(skip..skip + len as usize)
}

#[inline(never)]
pub fn usb_info(u: &UsbDisk, info: &mut dyn Info) {
    let d = u.disk;
    info.info(format_args!(
        "{} blocks of {}, {} MiB",
        d.blocks,
        d.block_size,
        (d.blocks * d.block_size as u64) >> 20
    ));
    info.info(format_args!(
        "{}",
        core::str::from_utf8(&d.ident).unwrap_or("?").trim_end()
    ));
    gpt_info(&u.parts, "usb0p", info);
}

#[inline(never)]
pub fn usb_close(u: &mut UsbDisk) {
    u.host.shutdown();
}

/// The SD card in SMHC0, identified.
pub struct SdCard {
    host: smhc::Smhc,
    card: sd::Card,
}

#[inline(never)]
pub fn sd_open() -> Result<SdCard, &'static str> {
    let mut host = smhc::Smhc::new_smhc0(soc::SMHC0_BASE);
    let card = sd::Card::identify(&mut host).map_err(|e| {
        println!("[fastboot] sd: {}", e.message());
        e.message()
    })?;
    println!(
        "[fastboot] sd: {} blocks, OCR {:08x}, RCA {:04x}",
        card.blocks, card.ocr, card.rca
    );
    Ok(SdCard { host, card })
}

#[inline(never)]
pub fn sd_size(c: &SdCard) -> u64 {
    c.card.blocks * sd::BLOCK as u64
}

#[inline(never)]
pub fn sd_read(
    c: &mut SdCard,
    start: u64,
    len: u64,
    buf: &mut [u8],
) -> Result<Range<usize>, &'static str> {
    let (lba, count, skip) = fetch::blocks(start, len, sd::BLOCK as u32);
    let words = count as usize * sd::BLOCK / 4;
    if words * 4 > buf.len() || !(buf.as_ptr() as usize).is_multiple_of(4) {
        return Err("fetch too large for the buffer");
    }
    // SAFETY: 4-byte aligned (the download buffer), in bounds.
    let w = unsafe { core::slice::from_raw_parts_mut(buf.as_mut_ptr().cast::<u32>(), words) };
    c.card.read(&mut c.host, lba, w).map_err(|e| {
        println!("[fastboot] sd read at block {lba}: {}", e.message());
        e.message()
    })?;
    Ok(skip..skip + len as usize)
}

#[inline(never)]
pub fn sd_info(c: &SdCard, info: &mut dyn Info) {
    info.info(format_args!(
        "{} blocks, OCR {:08x}",
        c.card.blocks, c.card.ocr
    ));
}

/// The eMMC module on SMHC2, identified, with its user-area GPT.
pub struct Emmc {
    host: smhc::Smhc,
    card: emmc::Card,
    ext: [u32; emmc::WORDS],
    parts: GptParts,
}

const EMMC_WRITE: bool = cfg!(feature = "fastboot-emmc-write");

/// Detect, identify (up to 8-bit, read-only bus test), EXT_CSD, GPT.
#[inline(never)]
pub fn emmc_open() -> Result<Emmc, &'static str> {
    let t0 = soc::time_us();
    let mut host = smhc::Smhc::new_smhc2(soc::SMHC2_BASE, true);
    let mut ext = [0u32; emmc::WORDS];
    let mut card = emmc::Card::identify(&mut host, &mut ext, 8, EMMC_WRITE).map_err(|e| {
        println!("[fastboot] emmc: {e:?}");
        e.message()
    })?;
    // SAFETY: DRAM reserved for GPT reads (as usb_open).
    let g = unsafe { core::slice::from_raw_parts_mut(GPT_BUF as *mut u32, GPT_BUF_LEN / 4) };
    let parts = card
        .read(&mut host, Part::User, 1, &mut g[..emmc::WORDS])
        .map_err(|e| e.message())
        .and_then(|()| {
            let header = &mut g[..emmc::WORDS];
            // SAFETY: bytes of the initialized header words, only borrowed for parsing.
            let header = unsafe {
                core::slice::from_raw_parts_mut(header.as_mut_ptr().cast::<u8>(), emmc::BLOCK)
            };
            let h = gpt::header(header, card.user_blocks)?;
            let array = g
                .get_mut(..h.entries_read_len() / 4)
                .ok_or("GPT entry array too large")?;
            card.read(&mut host, Part::User, h.entries_lba, array)
                .map_err(|e| e.message())?;
            gpt::parts(&h, words_as_bytes(array), card.user_blocks)
        });
    let note = parts.as_ref().map_or_else(|e| *e, |_| "GPT ok");
    let f = card.fields;
    println!(
        "[fastboot] emmc: CID {:08x}{:08x}{:08x}{:08x}, {} MiB, boot 2 x {} KiB, EXT_CSD rev {}, \
         PARTITION_CONFIG {:#04x}, BOOT_BUS {:#04x}, {}-bit, {note}, {} ms",
        card.cid[0],
        card.cid[1],
        card.cid[2],
        card.cid[3],
        card.user_blocks >> 11,
        card.boot_blocks / 2,
        f.rev,
        f.part_config,
        f.boot_bus,
        card.width,
        (soc::time_us() - t0) / 1000
    );
    Ok(Emmc {
        host,
        card,
        ext,
        parts,
    })
}

fn words_as_bytes(w: &[u32]) -> &[u8] {
    // SAFETY: plain memory, u8 has no alignment.
    unsafe { core::slice::from_raw_parts(w.as_ptr().cast::<u8>(), w.len() * 4) }
}

/// Partition and extent of an eMMC fetch target: `emmc` (user area),
/// `emmc-boot0`/`emmc-boot1` (boot partitions 1/2), `emmcp<N>` (user-area
/// GPT entry N), `emmc-extcsd` (the 512-byte EXT_CSD).
fn emmc_extent(d: &Emmc, name: &str) -> Result<(Option<Part>, u64, u64), &'static str> {
    let b = emmc::BLOCK as u64;
    match name {
        "emmc" => Ok((Some(Part::User), 0, d.card.user_blocks * b)),
        "emmc-boot0" if d.card.boot_blocks > 0 => {
            Ok((Some(Part::Boot1), 0, d.card.boot_blocks * b))
        }
        "emmc-boot1" if d.card.boot_blocks > 0 => {
            Ok((Some(Part::Boot2), 0, d.card.boot_blocks * b))
        }
        "emmc-extcsd" => Ok((None, 0, b)),
        _ => {
            let i = name
                .strip_prefix("emmcp")
                .and_then(|n| n.parse::<usize>().ok())
                .ok_or("unknown eMMC partition")?;
            let p = gpt_part(&d.parts, i)?;
            Ok((Some(Part::User), p.first * b, (p.last - p.first + 1) * b))
        }
    }
}

#[inline(never)]
pub fn emmc_size(d: &Emmc, name: &str) -> Result<u64, &'static str> {
    emmc_extent(d, name).map(|(_, _, size)| size)
}

/// `fetch` of an eMMC target (window, offset, size as the other devices).
#[inline(never)]
pub fn emmc_fetch(
    d: &mut Emmc,
    name: &str,
    window: fetch::Window,
    offset: u64,
    size: Option<u64>,
    max: u64,
    buf: &mut [u8],
) -> Result<Range<usize>, &'static str> {
    let (part, at, part_size) = emmc_extent(d, name)?;
    let (start, len) = fetch::resolve(part_size, window, offset, size, max)?;
    let Some(part) = part else {
        // EXT_CSD, read fresh.
        d.card
            .refresh(&mut d.host, &mut d.ext)
            .map_err(|e| e.message())?;
        buf[..len as usize]
            .copy_from_slice(&words_as_bytes(&d.ext)[start as usize..(start + len) as usize]);
        return Ok(0..len as usize);
    };
    let (lba, count, skip) = fetch::blocks(at + start, len, emmc::BLOCK as u32);
    let words = count as usize * emmc::WORDS;
    if words * 4 > buf.len() || !(buf.as_ptr() as usize).is_multiple_of(4) {
        return Err("fetch too large for the buffer");
    }
    // SAFETY: 4-byte aligned (the download buffer), in bounds.
    let w = unsafe { core::slice::from_raw_parts_mut(buf.as_mut_ptr().cast::<u32>(), words) };
    d.card.read(&mut d.host, part, lba, w).map_err(|e| {
        println!("[fastboot] emmc {part:?} read at block {lba}: {e:?}");
        e.message()
    })?;
    Ok(skip..skip + len as usize)
}

/// `oem emmc`: identity, sizes, the boot-relevant EXT_CSD fields, GPT.
#[inline(never)]
pub fn emmc_info(d: &Emmc, info: &mut dyn Info) {
    let c = &d.card;
    let f = &c.fields;
    info.info(format_args!(
        "CID {:08x}{:08x}{:08x}{:08x}",
        c.cid[0], c.cid[1], c.cid[2], c.cid[3]
    ));
    info.info(format_args!(
        "CSD {:08x}{:08x}{:08x}{:08x}",
        c.csd[0], c.csd[1], c.csd[2], c.csd[3]
    ));
    info.info(format_args!(
        "OCR {:08x}, {} MiB user, boot 2 x {} KiB, {}-bit",
        c.ocr,
        c.user_blocks >> 11,
        c.boot_blocks / 2,
        c.width
    ));
    info.info(format_args!(
        "EXT_CSD rev {} type {:#04x} PART_CFG {:#04x} BOOT_BUS {:#04x} BOOT_INFO {:#04x}",
        f.rev, f.device_type, f.part_config, f.boot_bus, f.boot_info
    ));
    info.info(format_args!(
        "BOOT_WP {:#04x}/{:#04x} USER_WP {:#04x} CFG_PROT {:#04x} RST_n {:#04x} PART_DONE {:#04x}",
        f.boot_wp,
        f.boot_wp_status,
        f.user_wp,
        f.boot_config_prot,
        f.rst_n,
        f.partition_setting_completed
    ));
    gpt_info(&d.parts, "emmcp", info);
}

fn gpt_info(parts: &GptParts, prefix: &str, info: &mut dyn Info) {
    let entries = match parts {
        Ok(entries) => entries,
        Err(e) => {
            info.info(format_args!("GPT unavailable: {e}"));
            return;
        }
    };
    for (i, p) in entries.iter().enumerate() {
        if let Some(p) = p {
            info.info(format_args!(
                "{prefix}{}: blocks {}..={}",
                i + 1,
                p.first,
                p.last
            ));
        }
    }
}

/// The layout check of a `firmware.bin` for `emmc-fw`: the frame decodes
/// and fits this board's DRAM (cubie_a7a_fastboot::emmc checks the rest).
#[cfg(feature = "fastboot-emmc-write")]
fn emmc_layout_check(image: &[u8], dram_end: u64) -> Result<(), &'static str> {
    use cubie_a7a_common::layout;
    let at = (layout::TABLE_OFFSET as u64 - cubie_a7a_fastboot::emmcarm::FW_START) as usize;
    let frame = image.get(at..at + emmc::BLOCK).ok_or("no layout frame")?;
    let t = layout::decode(frame).map_err(|_| "not a valid A7ABOOT1 layout frame")?;
    layout::validate(&t, dram_end).map_err(|_| "layout table does not fit this board")
}

/// `flash` of an armed eMMC target (the arm already used up).
#[cfg(feature = "fastboot-emmc-write")]
#[inline(never)]
pub fn emmc_flash(
    d: &mut Emmc,
    t: &cubie_a7a_fastboot::emmcarm::Target,
    image: &[u8],
    dram_end: u64,
    scratch: &mut [u32],
    info: &mut dyn Info,
) -> Result<(), &'static str> {
    let r = d.card.program(
        &mut d.host,
        t,
        image,
        &mut |i| emmc_layout_check(i, dram_end),
        scratch,
        info,
    );
    r.map_err(|e| {
        println!("[fastboot] emmc flash at block {}: {e:?}", t.first);
        e.message()
    })
}

/// `erase` (zeros) of an armed eMMC target.
#[cfg(feature = "fastboot-emmc-write")]
#[inline(never)]
pub fn emmc_erase(
    d: &mut Emmc,
    t: &cubie_a7a_fastboot::emmcarm::Target,
    scratch: &mut [u32],
    info: &mut dyn Info,
) -> Result<(), &'static str> {
    let r = d.card.erase(&mut d.host, t, scratch, info);
    r.map_err(|e| {
        println!("[fastboot] emmc erase at block {}: {e:?}", t.first);
        e.message()
    })
}

/// `oem emmc-bootcfg`, armed: PARTITION_CONFIG bits 6:3 and
/// BOOT_BUS_CONDITIONS, read back.
#[cfg(feature = "fastboot-emmc-write")]
#[inline(never)]
pub fn emmc_boot_config(
    d: &mut Emmc,
    part_config: u8,
    boot_bus: u8,
    info: &mut dyn Info,
) -> Result<(), &'static str> {
    d.card
        .boot_config(&mut d.host, part_config, boot_bus, &mut d.ext)
        .map_err(|e| {
            println!("[fastboot] emmc bootcfg: {e:?}");
            e.message()
        })?;
    info.info(format_args!(
        "PARTITION_CONFIG {:#04x}, BOOT_BUS_CONDITIONS {:#04x} (read back)",
        d.card.fields.part_config, d.card.fields.boot_bus
    ));
    Ok(())
}
