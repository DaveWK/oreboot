// SPDX-License-Identifier: GPL-2.0-only
//! Boot a FIT that is already in memory (the fastboot download buffer):
//! the steps of bt0's eMMC path (boards/rk3588/yy3588/bt0/src/boot.rs
//! `load_and_enter`, after its eMMC read) on a buffer, with the memory and
//! the jump behind [`Machine`] so the handoff can be checked on the host.
//!
//! [`prepare`] checks the image before the fastboot reply (so a bad image
//! is a FAIL and the session goes on): FDT magic, `rk3588::fit::plan`,
//! `resolve` (data base), complete data inside the download, `verify`
//! (SHA-256 of every hashed image), `check_overlaps` against what must
//! survive. The one exception to "complete data" is the UEFI variable store
//! window ([`fillable`]): see [`NV_FILL`]. [`enter`] then copies every
//! image to its load address (filling the missing part of that window),
//! writes
//! the BL31 FDT (`rk3588::handoff::bl31_fdt`) and `bl_params`
//! (`rk3588::handoff::bl_params`, BL33 = EDK2) and enters BL31 with x0 =
//! bl_params, x1 = FDT, exactly as boot.rs does.

use rk3588::{fit, handoff};

/// Where the handoff structures go, and what FIT images must not overwrite.
#[derive(Clone, Copy, Debug)]
pub struct Layout<'a> {
    /// The BL31 FDT (boot.rs FDT_ADDR = 0x0300_0000).
    pub fdt_addr: usize,
    /// bl_params (boot.rs PARAMS_ADDR = 0x0310_0000).
    pub params_addr: usize,
    /// BL33 entry when the FIT has no image named "edk2" (boot.rs passes
    /// 0x20_0000, edk2-rk3588's EDK2 load address).
    pub bl33_default: usize,
    /// Regions (base, size) no image may overlap: bt0, the dram-info
    /// record, the handoff window, the download buffer.
    pub keep: &'a [(usize, usize)],
}

/// boot.rs: the bt0 region of the stepping-stone build (DRAM 0, 192 KiB),
/// the dram-info record, and the FDT + bl_params window.
pub const BT0_STEPPING_STONE: (usize, usize) = (0, 0x3_0000);
pub const FDT_ADDR: usize = 0x0300_0000;
pub const PARAMS_ADDR: usize = 0x0310_0000;
pub const HANDOFF_WINDOW: (usize, usize) = (FDT_ADDR, 0x20_0000);

/// Memory and the jump.
pub trait Machine {
    /// Copy `src` to physical address `dst`.
    fn copy(&mut self, dst: usize, src: &[u8]);
    /// Set `len` bytes at physical address `dst` to `value` (default: in
    /// 256-byte [`Machine::copy`]s).
    fn fill(&mut self, dst: usize, len: usize, value: u8) {
        let chunk = [value; 256];
        let mut done = 0;
        while done < len {
            let n = (len - done).min(chunk.len());
            self.copy(dst + done, &chunk[..n]);
            done += n;
        }
    }
    /// Enter BL31 at EL3 with x0 = `params`, x1 = `fdt`. Never returns on
    /// hardware; a host model records it and returns.
    fn enter_bl31(&mut self, entry: usize, params: usize, fdt: usize);
}

/// FDT header magic (big-endian 0xd00dfeed): a FIT.
pub const FDT_MAGIC: [u8; 4] = [0xd0, 0x0d, 0xfe, 0xed];

/// Android boot image magic.
pub const BOOT_MAGIC: &[u8; 8] = b"ANDROID!";

/// The FIT inside what the host downloaded. `fastboot boot <file>` wraps a
/// file that is not an Android boot image into one (AOSP
/// system/core/fastboot fastboot.cpp LoadBootableImage, with the file as
/// the kernel), while `fastboot stage <file>` sends it as is. Header
/// layout (AOSP bootimg.h, all versions): kernel_size at byte 8,
/// header_version at 40; versions 0-2 carry page_size at 36 and the
/// kernel at one page, versions 3-4 use 4096-byte pages.
pub fn unwrap(image: &[u8]) -> Result<&[u8], &'static str> {
    if image.len() < 48 || &image[..8] != BOOT_MAGIC {
        return Ok(image);
    }
    let le = |o: usize| u32::from_le_bytes([image[o], image[o + 1], image[o + 2], image[o + 3]]);
    let kernel = le(8) as usize;
    let page = if le(40) >= 3 { 4096 } else { le(36) as usize };
    if page == 0 || page > 65536 {
        return Err("bad boot image page size");
    }
    image
        .get(page..page.checked_add(kernel).ok_or("bad boot image")?)
        .ok_or("boot image shorter than its kernel")
}

/// What the missing part of the UEFI variable store window is filled with
/// when the downloaded FIT ends before it: 0xFF, the erased-flash state.
/// edk2-rk3588 edk2-rockchip/Silicon/Rockchip/Drivers/RkFvbDxe/RkFvbDxe.c
/// (read at edk2-rk3588 6283923b99; the file last changed in be4d361,
/// 2026-09-19, before the YY3588 pin 2f3f826559 of 2026-10-25, so taken
/// as identical there [I]): FvbConfigureFlashInstance uses the window at
/// its load address as the store (":1087 U-Boot maps NV data into memory
/// at the same address as in flash"); FvbPrepareFvHeader (:1009) runs
/// FvbValidateFvHeader (:258, needs EFI_FVH_SIGNATURE), and on failure
/// FvbEraseBlocks (:1025; :944 SetMem (..., 0xFF), EFI_FVB2_ERASE_POLARITY)
/// and FvbInitFvAndVariableStoreHeaders format a fresh store. 0xFF is that
/// erased state, so the window looks like never-written flash.
pub const NV_FILL: u8 = 0xFF;

/// The FIT image whose data may lie past the downloaded bytes: edk2-rk3588's
/// `nvdata` (misc/uefi_rk3588.its: `data-position` 0x6c0000, size 0x30000,
/// load 0x7c0000), the UEFI variable store window, and only while it has
/// no `hash` node. A FIT cut from a built image ends before that window
/// (the eMMC path reads 7 MiB and always has it).
pub fn fillable(i: &fit::Image) -> bool {
    i.name() == "nvdata" && i.hash.is_none()
}

/// Bytes of image `i` that a FIT of `len` bytes holds, and the bytes past
/// its end, for a resolved plan.
pub fn present(i: &fit::Image, len: usize) -> (usize, usize) {
    let have = len.saturating_sub(i.offset).min(i.size);
    (have, i.size - have)
}

/// The [`fillable`] image a FIT of `len` bytes ends inside or before, and
/// how many bytes [`enter`] fills with [`NV_FILL`] (for the log).
pub fn filled(plan: &fit::Plan, len: usize) -> Option<(&fit::Image, usize)> {
    plan.images()
        .iter()
        .map(|i| (i, present(i, len).1))
        .find(|&(i, n)| n > 0 && fillable(i))
}

/// Check `image` and plan its placement; Err is a fastboot FAIL message.
pub fn prepare(image: &[u8], keep: &[(usize, usize)]) -> Result<fit::Plan, &'static str> {
    if image.len() < 40 || image[..4] != FDT_MAGIC {
        return Err("not a FIT image");
    }
    let mut plan = fit::plan(image).map_err(|_| "FIT structure")?;
    plan.resolve(image).map_err(|_| "FIT data base / hash")?;
    for i in plan.images() {
        if present(i, image.len()).1 > 0 && !fillable(i) {
            return Err("FIT data past the download");
        }
    }
    plan.verify(image).map_err(|_| "FIT hash")?;
    plan.check_overlaps(keep)
        .map_err(|_| "FIT destinations overlap bt0, the buffer or the handoff")?;
    Ok(plan)
}

/// Place the images of a [`prepare`]d FIT and enter BL31.
pub fn enter<M: Machine>(
    m: &mut M,
    image: &[u8],
    plan: &fit::Plan,
    dram_bytes: u64,
    layout: &Layout,
) {
    for i in plan.images() {
        let (have, missing) = present(i, image.len());
        if have > 0 {
            m.copy(i.load, &image[i.offset..i.offset + have]);
        }
        if missing > 0 {
            // Only a fillable image gets here (prepare refused the rest).
            m.fill(i.load + have, missing, NV_FILL);
        }
    }
    let edk2 = plan.images().iter().find(|i| i.name() == "edk2");
    let (pc, size) = edk2.map_or((layout.bl33_default, 0), |i| (i.load, i.size));
    let mut fdt = [0u8; 2048];
    let n = handoff::bl31_fdt(&mut fdt, dram_bytes);
    m.copy(layout.fdt_addr, &fdt[..n]);
    let mut params = [0u8; handoff::TOTAL];
    handoff::bl_params(
        &mut params,
        layout.params_addr as u64,
        pc as u64,
        size as u32,
        0,
    );
    m.copy(layout.params_addr, &params);
    m.enter_bl31(plan.entry, layout.params_addr, layout.fdt_addr);
}
