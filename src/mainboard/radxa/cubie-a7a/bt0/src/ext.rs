//! The DRAM part of the fastboot bt0 (feature `fastboot`): the block
//! device dumps (`fetch:usb0`, `usb0p<N>`, `sd`), which do not fit the FEL
//! SRAM window beside the DRAM bring-up.
//!
//! build.rs links this module, `ehci.rs` and the dump modules of
//! cubie-a7a-fastboot (`msc`, `gpt`, `usbhost`, `sd`, including the
//! `SdBus` impl in smhc.rs) into the output section `.dram` at [`BASE`]:
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

use cubie_a7a_fastboot::proto::Info;
use cubie_a7a_fastboot::{fetch, gpt, msc, sd};

use crate::{ehci, smhc, soc};

/// The blue-port disk: EHCI1, the Bulk-Only disk, its GPT partitions.
pub struct UsbDisk {
    host: ehci::Ehci,
    disk: msc::Disk,
    parts: [Option<gpt::Part>; gpt::MAX_PARTS],
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
    let mut parts = [None; gpt::MAX_PARTS];
    let note = match disk
        .read(&mut host, 1, &mut g[..bs])
        .map_err(|e| e.message())
        .and_then(|()| {
            let h = gpt::header(&g[..bs], disk.blocks)?;
            let n = gpt::entries_len(&h).div_ceil(bs) * bs;
            if n > GPT_BUF_LEN {
                return Err("GPT entry array too large");
            }
            disk.read(&mut host, h.entries_lba, &mut g[..n])
                .map_err(|e| e.message())?;
            gpt::parts(&h, &g[..n], disk.blocks)
        }) {
        Ok(p) => {
            parts = p;
            "GPT ok"
        }
        Err(e) => e,
    };
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
            let p = i
                .checked_sub(1)
                .and_then(|i| u.parts.get(i).copied().flatten())
                .ok_or("no such GPT partition on usb0")?;
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
    for (i, p) in u.parts.iter().enumerate() {
        if let Some(p) = p {
            info.info(format_args!(
                "usb0p{}: blocks {}..={}",
                i + 1,
                p.first,
                p.last
            ));
        }
    }
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
