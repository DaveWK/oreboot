// SPDX-License-Identifier: GPL-2.0-only
//! The YY3588 fastboot device: what crate `fastboot`'s protocol loop asks
//! the board.
//!
//! Commands (host side: `fastboot <cmd>`):
//! - `getvar`: version, max-download-size, unlocked, is-userspace, secure,
//!   has-slot:*, is-logical:* (crate fastboot); product, serialno,
//!   version-bootloader, partition-type:<target>, partition-size:<target>
//!   (here).
//! - `download` (also what `flash`, `boot` and `stage` send first).
//! - `flash:<target>`: raw or Android sparse images (crate fastboot
//!   `flash_image`) to the eMMC through the board's [`Board`] hook.
//! - `boot`: a downloaded FIT (as is, or as the kernel of the Android
//!   boot image the fastboot tool wraps it in, [`crate::stage::unwrap`]) is checked ([`crate::stage::prepare`]) and,
//!   once the OKAY is acknowledged and the controller quiesced, booted
//!   ([`Next::Boot`]). `stage <fit>` + `continue` does the same.
//! - `continue` without a staged FIT: [`Next::Continue`] (bt0 then boots
//!   the board's normal chain from eMMC, like the K1 bt0 after fastboot).
//! - `reboot*`: the board's reset ([`Board::reset`]).
//!
//! Targets, those of the K1 bt0 (fastboot.rs `target`) for the eMMC, plus
//! two regions of the RK3588 boot layout (boards/rk3588/yy3588/README.md:
//! idblock at 0x8000, FIT at 0x100000, UEFI variables up to 0x7F0000):
//!
//! | target | where | size |
//! |---|---|---|
//! | `emmc`, `user`, `mmc0`, `disk` | user area from LBA 0 | the user area |
//! | `boot0`, `boot1` | eMMC boot partitions | as reported |
//! | `idblock` | user area LBA 64 (0x8000) | to LBA 2048 (1984 sectors) |
//! | `fit` | user area LBA 2048 (0x100000) | 7 MiB, to 0x800000 [I] |
//!
//! The YY3588's microSD slot (the K1's `sd`) is not offered.

use core::fmt::Write;
use fastboot::Line;

use crate::stage;

pub const SECTOR: usize = 512;

/// eMMC hardware partitions.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Part {
    User,
    Boot0,
    Boot1,
}

/// A flash target: partition, first sector, size limit in sectors (None:
/// the partition's size).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Target {
    pub part: Part,
    pub lba: u32,
    pub sectors: Option<u64>,
}

pub fn target(name: &str) -> Option<Target> {
    let (part, lba, sectors) = match name {
        "emmc" | "user" | "mmc0" | "disk" => (Part::User, 0, None),
        "boot0" => (Part::Boot0, 0, None),
        "boot1" => (Part::Boot1, 0, None),
        "idblock" => (Part::User, 64, Some(2048 - 64)),
        "fit" => (Part::User, 2048, Some(0x3800)),
        _ => return None,
    };
    Some(Target { part, lba, sectors })
}

/// The board's hooks: the eMMC (the bt0 eMMC driver) and the reset.
pub trait Board {
    /// Open the eMMC if needed and select hardware partition `part`.
    fn emmc_select(&mut self, part: Part) -> Result<(), &'static str>;
    /// Sectors of hardware partition `part` (after `emmc_select`).
    fn emmc_sectors(&mut self, part: Part) -> Result<u64, &'static str>;
    /// Write whole sectors `data` at `lba` of the selected partition.
    fn emmc_write(&mut self, lba: u32, data: &[u8]) -> Result<(), &'static str>;
    /// Zero `sectors` from `lba` without writing them (TRIM where the card
    /// reads trimmed sectors back as zeros). Ok(false): not done, write the
    /// zeros instead.
    fn emmc_zero(&mut self, _lba: u32, _sectors: u64) -> Result<bool, &'static str> {
        Ok(false)
    }
    /// Board variables for `getvar` (e.g. the eMMC bus mode); false when
    /// `name` is not one.
    fn getvar(&mut self, _name: &str, _out: &mut Line) -> bool {
        false
    }
    /// Reset the SoC (never returns on hardware) or refuse.
    fn reset(&mut self) -> Result<(), &'static str>;
}

/// What bt0 does after `fastboot::serve` returned Ok.
// One value per session and no heap in bt0: the plan stays inline.
#[allow(clippy::large_enum_variant)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Next {
    /// `continue` with nothing staged: the normal boot chain.
    Continue,
    /// `boot` or `stage` + `continue` of a FIT at `addr` (`len` bytes, in
    /// the download buffer), planned and verified.
    Boot {
        plan: rk3588::fit::Plan,
        addr: usize,
        len: usize,
    },
}

pub struct Rk3588Device<'a, B: Board> {
    pub board: B,
    pub product: &'a str,
    pub serial: &'a str,
    /// Regions a booted FIT must not overwrite (see [`stage::Layout`]).
    pub keep: &'a [(usize, usize)],
    pub next: Next,
}

impl<'a, B: Board> Rk3588Device<'a, B> {
    pub fn new(board: B, product: &'a str, serial: &'a str, keep: &'a [(usize, usize)]) -> Self {
        Self {
            board,
            product,
            serial,
            keep,
            next: Next::Continue,
        }
    }

    fn capacity(&mut self, t: &Target) -> Result<u64, &'static str> {
        self.board.emmc_select(t.part)?;
        let n = self.board.emmc_sectors(t.part)?;
        let n = n
            .checked_sub(t.lba as u64)
            .ok_or("target outside the eMMC")?;
        Ok(t.sectors.map_or(n, |s| s.min(n)))
    }

    fn take(&mut self, image: &[u8]) -> Result<(), &'static str> {
        let fit = stage::unwrap(image)?;
        let plan = stage::prepare(fit, self.keep)?;
        self.next = Next::Boot {
            plan,
            addr: fit.as_ptr() as usize,
            len: fit.len(),
        };
        Ok(())
    }
}

/// A target region as a `sparse::Writer`: LBAs relative to its start.
struct Region<'b, B: Board> {
    board: &'b mut B,
    base: u32,
}

impl<B: Board> sparse::Writer for Region<'_, B> {
    fn raw(&mut self, lba: u32, data: &[u8]) -> Result<(), &'static str> {
        let lba = self.base.checked_add(lba).ok_or("LBA overflow")?;
        // 32 MiB per call, like the K1 bt0's write_raw.
        let mut done = 0;
        for chunk in data.chunks(65536 * SECTOR) {
            self.board.emmc_write(lba + (done / SECTOR) as u32, chunk)?;
            done += chunk.len();
        }
        Ok(())
    }
    fn fill(&mut self, lba: u32, sectors: u64, value: u32) -> Result<(), &'static str> {
        if value == 0 {
            let at = self.base.checked_add(lba).ok_or("LBA overflow")?;
            if self.board.emmc_zero(at, sectors)? {
                return Ok(());
            }
        }
        const PER: usize = 8;
        let mut pattern = [0u8; PER * SECTOR];
        for w in pattern.as_chunks_mut::<4>().0 {
            w.copy_from_slice(&value.to_le_bytes());
        }
        let mut done = 0u64;
        while done < sectors {
            let n = (sectors - done).min(PER as u64) as usize;
            let at = self
                .base
                .checked_add(lba)
                .and_then(|l| l.checked_add(done as u32))
                .ok_or("LBA overflow")?;
            self.board.emmc_write(at, &pattern[..n * SECTOR])?;
            done += n as u64;
        }
        Ok(())
    }
}

impl<B: Board> fastboot::Device for Rk3588Device<'_, B> {
    fn getvar(&mut self, name: &str, out: &mut Line) -> bool {
        match name {
            "product" => write!(out, "{}", self.product).is_ok(),
            "serialno" => write!(out, "{}", self.serial).is_ok(),
            "version-bootloader" => write!(out, "oreboot-bt0").is_ok(),
            _ => {
                if let Some(t) = name.strip_prefix("partition-type:").and_then(target) {
                    let _ = t;
                    return write!(out, "raw").is_ok();
                }
                if let Some(t) = name.strip_prefix("partition-size:").and_then(target) {
                    if let Ok(n) = self.capacity(&t) {
                        return write!(out, "0x{:x}", n * SECTOR as u64).is_ok();
                    }
                }
                self.board.getvar(name, out)
            }
        }
    }

    fn flash(
        &mut self,
        name: &str,
        image_len: usize,
        padded: &mut [u8],
    ) -> Result<(), &'static str> {
        let t = target(name).ok_or("unknown target")?;
        let cap = self.capacity(&t)?;
        let mut w = Region {
            board: &mut self.board,
            base: t.lba,
        };
        fastboot::flash_image(padded, image_len, cap, &mut w)
    }

    fn boot(&mut self, image: &[u8]) -> Result<(), &'static str> {
        if image.is_empty() {
            return Err("download an image first");
        }
        self.take(image)
    }

    fn staged(&mut self, image: &[u8]) -> Result<(), &'static str> {
        self.take(image)
    }

    fn reboot(&mut self) -> Result<(), &'static str> {
        self.board.reset()
    }
}
