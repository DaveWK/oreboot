// SPDX-License-Identifier: GPL-2.0-only
//! Load stage: eMMC -> FIT -> placement -> TF-A BL31 entry.
//!
//! Memory map used (all DRAM, below EDK2's region except where noted):
//! bt0 itself at 0x0..0x30000 (stepping-stone build), BL31 segments at
//! 0x40000 / 0x65000 (DRAM, TF-A TZRAM 0..1 MiB) and 0xff100000 (PMU SRAM),
//! the dram-info record at 0x1F0000, EDK2 at 0x200000, the UEFI variable
//! window at 0x7C0000, the BL31 FDT at 0x3000000, bl_params at 0x3100000,
//! the FIT staging buffer at 0x4000000.
//!
//! Caches and MMU are off (strict alignment: the target is built with
//! +strict-align and copies are word-wise when aligned).

use crate::emmc::{EmmcIo, Rk3588Emmc};
use crate::mmio::Mmio;
use core::fmt::Write;
use regio::RegIo;
use rk3588::{fit, handoff, regs::base};
use sdhci::{Kind, Sdhci};

pub const FIT_LBA: u32 = 0x100000 / 512;
pub const STAGING: usize = 0x0400_0000;
pub const STAGING_MAX: usize = 0x0080_0000;
pub const FDT_ADDR: usize = 0x0300_0000;
pub const PARAMS_ADDR: usize = 0x0310_0000;
/// bt0 itself (image, .bss, stack): the linker script's RAM region.
#[cfg(feature = "stepping-stone")]
const BT0_REGION: (usize, usize) = (0, 0x3_0000);
#[cfg(not(feature = "stepping-stone"))]
const BT0_REGION: (usize, usize) = (0xff00_1000, 0x3_F000);
/// The dram-info record (crate rk3588::dump).
const RECORD: (usize, usize) = (rk3588::dump::RECORD_ADDR, rk3588::dump::RECORD_BYTES);
const CHUNK: usize = 128; // sectors per read

/// Quick, non-destructive-to-boot DRAM check over the staging buffer.
pub fn memcheck(io: &mut Mmio) -> bool {
    let words = 0x10_0000 / 8;
    let p = STAGING as *mut u64;
    for i in 0..words {
        unsafe {
            p.add(i)
                .write_volatile((i as u64).wrapping_mul(0x9E37_79B9_7F4A_7C15) ^ 0x5555_aaaa)
        };
    }
    for i in 0..words {
        let want = (i as u64).wrapping_mul(0x9E37_79B9_7F4A_7C15) ^ 0x5555_aaaa;
        let got = unsafe { p.add(i).read_volatile() };
        if got != want {
            let _ = writeln!(
                io,
                "dram: mismatch at {:#x}: {got:#x} != {want:#x}",
                STAGING + 8 * i
            );
            return false;
        }
    }
    let _ = writeln!(io, "dram: quick check ok (1 MiB at {STAGING:#x})");
    true
}

fn copy(dst: usize, src: usize, len: usize) {
    unsafe {
        if (dst | src) & 7 == 0 {
            let n = len / 8;
            for i in 0..n {
                let v = ((src + 8 * i) as *const u64).read_volatile();
                ((dst + 8 * i) as *mut u64).write_volatile(v);
            }
            for i in 8 * n..len {
                ((dst + i) as *mut u8).write_volatile(((src + i) as *const u8).read_volatile());
            }
        } else {
            for i in 0..len {
                ((dst + i) as *mut u8).write_volatile(((src + i) as *const u8).read_volatile());
            }
        }
    }
}

pub fn load_and_enter(io: &mut Mmio, dram_bytes: u64) -> Result<(), &'static str> {
    let (plan, edk2_size) = {
        let eio = EmmcIo(io);
        let mut mmc =
            Sdhci::init(eio, Rk3588Emmc, base::EMMC, Kind::Emmc).map_err(|_| "eMMC init")?;
        // FIT structure first (8 sectors cover the edk2-rk3588 FIT header).
        mmc.read(FIT_LBA, 8, STAGING)
            .map_err(|_| "eMMC read (FIT header)")?;
        let head = unsafe { core::slice::from_raw_parts(STAGING as *const u8, 8 * 512) };
        let mut plan = fit::plan(head).map_err(|_| "FIT structure")?;
        if plan.end > STAGING_MAX {
            return Err("FIT larger than the staging buffer");
        }
        let sectors = plan.end.div_ceil(512);
        let mut done = 8;
        while done < sectors {
            let n = CHUNK.min(sectors - done);
            mmc.read(FIT_LBA + done as u32, n, STAGING + done * 512)
                .map_err(|_| "eMMC read (FIT)")?;
            done += n;
        }
        let fitbuf = unsafe { core::slice::from_raw_parts(STAGING as *const u8, sectors * 512) };
        plan.resolve(fitbuf).map_err(|_| "FIT data base / hash")?;
        plan.verify(fitbuf).map_err(|_| "FIT hash")?;
        let edk2 = plan
            .images()
            .iter()
            .find(|i| i.name() == "edk2")
            .map(|i| i.size)
            .unwrap_or(0);
        (plan, edk2)
    };
    plan.check_overlaps(&[
        BT0_REGION,
        RECORD,
        (STAGING, STAGING_MAX),
        (FDT_ADDR, 0x20_0000),
    ])
    .map_err(|_| "FIT destinations overlap bt0, staging or handoff")?;
    for i in plan.images() {
        let _ = writeln!(
            io,
            "load {:<8} {:#010x} <- fit+{:#x} ({} bytes)",
            i.name(),
            i.load,
            i.offset,
            i.size
        );
        copy(i.load, STAGING + i.offset, i.size);
    }
    let fdt = unsafe { core::slice::from_raw_parts_mut(FDT_ADDR as *mut u8, 2048) };
    let fdt_len = handoff::bl31_fdt(fdt, dram_bytes);
    let mut params = [0u8; handoff::TOTAL];
    handoff::bl_params(
        &mut params,
        PARAMS_ADDR as u64,
        0x20_0000,
        edk2_size as u32,
        0,
    );
    copy(PARAMS_ADDR, params.as_ptr() as usize, params.len());
    let _ = writeln!(
        io,
        "enter BL31 at {:#x} (bl_params {PARAMS_ADDR:#x}, fdt {FDT_ADDR:#x} {fdt_len} bytes)",
        plan.entry
    );
    io.udelay(2_000); // let the console drain
    unsafe {
        core::arch::asm!("dsb sy", "ic iallu", "dsb sy", "isb");
        let entry: extern "C" fn(usize, usize, usize, usize) -> ! =
            core::mem::transmute(plan.entry);
        entry(PARAMS_ADDR, FDT_ADDR, 0, 0);
    }
}
