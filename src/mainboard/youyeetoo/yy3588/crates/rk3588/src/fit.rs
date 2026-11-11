// SPDX-License-Identifier: GPL-2.0-only
//! The edk2-rk3588 FIT (`mkimage -f uefi_rk3588.its -E`), read from the
//! boot medium at 0x100000 and placed by bt0.
//!
//! The default configuration names `firmware` (BL31's first segment, also
//! its entry point) and `loadables` (EDK2, further BL31 segments, OP-TEE
//! when built in, and `nvdata`). Each image has `load` and either
//! `data-offset` (from the end of the FIT structure, rounded up: to 4 bytes
//! by mainline mkimage 2026.04, to 512 bytes by the mkimage edk2-rk3588
//! ships, observed on a built image; [`Plan::resolve`] finds which by the
//! hashes) or `data-position` (from the start of the FIT: `nvdata`, the
//! UEFI variable store window), `data-size`, and usually a `hash` node
//! (`algo` = "sha256", `value`). Layout: edk2-rk3588 misc/uefi_rk3588.its
//! and build.sh `_build_fit` (BSD-2-Clause-Patent project files).
//! Placement mirrors what the U-Boot SPL does with a FIT (load every
//! loadable, enter the firmware image) without the SPL.

use fdt_checked::{self as fdt, Fdt};

pub const MAX_IMAGES: usize = 8;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    Fdt,
    Missing(&'static str),
    TooMany,
    Bounds,
    Hash,
    /// The image is not one bt0 loads (e.g. OP-TEE in a stepping-stone FIT
    /// built without BL32_BIN=none is reported, not loaded).
    Unexpected,
}

impl From<fdt::Error> for Error {
    fn from(_: fdt::Error) -> Self {
        Error::Fdt
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct Image {
    /// Name, NUL-padded (first 15 bytes).
    pub name: [u8; 16],
    /// Byte offset of the data from the start of the FIT (after
    /// [`Plan::resolve`]; before it, from the data base when `relative`).
    pub offset: usize,
    /// `data-offset` (true) or `data-position` (false).
    pub relative: bool,
    pub size: usize,
    pub load: usize,
    pub hash: Option<[u8; 32]>,
    /// `os = "op-tee"` (never loaded by bt0).
    pub optee: bool,
}

impl Image {
    pub fn name(&self) -> &str {
        let n = self.name.iter().position(|&b| b == 0).unwrap_or(16);
        core::str::from_utf8(&self.name[..n]).unwrap_or("?")
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Plan {
    /// Bytes from the FIT start the data reaches, for the largest data
    /// base (512-byte alignment); exact after [`Plan::resolve`].
    pub end: usize,
    /// FDT totalsize of the FIT structure.
    pub total: usize,
    /// The data base chosen by [`Plan::resolve`] (0 before).
    pub data_base: usize,
    /// BL31 entry (the firmware image's load address).
    pub entry: usize,
    pub count: usize,
    /// images[0] is the firmware image.
    pub images: [Image; MAX_IMAGES],
}

fn name16(n: &[u8]) -> [u8; 16] {
    let mut a = [0u8; 16];
    let k = n.len().min(15);
    a[..k].copy_from_slice(&n[..k]);
    a
}

fn image(f: &Fdt, images: usize, name: &[u8]) -> Result<Image, Error> {
    let n = f.child(images, name)?;
    let size = fdt::number(f.prop(n, b"data-size")?)?;
    let (offset, relative) = match f.prop(n, b"data-offset") {
        Ok(v) => (fdt::number(v)?, true),
        Err(_) => (
            fdt::number(
                f.prop(n, b"data-position")
                    .map_err(|_| Error::Missing("data-offset/position"))?,
            )?,
            false,
        ),
    };
    let load = fdt::number(f.prop(n, b"load").map_err(|_| Error::Missing("load"))?)?;
    let hash = match f.child(n, b"hash").or_else(|_| f.child(n, b"hash-1")) {
        Ok(h) => {
            if fdt::string(f.prop(h, b"algo")?)? != b"sha256" {
                return Err(Error::Hash);
            }
            Some(f.prop(h, b"value")?.try_into().map_err(|_| Error::Hash)?)
        }
        Err(_) => None,
    };
    let optee =
        matches!(f.prop(n, b"os"), Ok(v) if fdt::string(v).ok() == Some(b"op-tee".as_slice()));
    Ok(Image {
        name: name16(name),
        offset,
        relative,
        size,
        load,
        hash,
        optee,
    })
}

/// Plan the placements from the FIT structure (`meta`, at least the FDT
/// header's totalsize bytes). Reads no image data.
pub fn plan(meta: &[u8]) -> Result<Plan, Error> {
    let f = Fdt::new(meta)?;
    let images = f.child(f.h.structure, b"images")?;
    let configs = f.child(f.h.structure, b"configurations")?;
    let default = fdt::string(f.prop(configs, b"default")?)?;
    let c = f.child(configs, default)?;
    let fw_name = fdt::string(f.prop(c, b"firmware")?)?;
    let mut p = Plan {
        end: 0,
        total: f.h.total,
        data_base: 0,
        entry: 0,
        count: 0,
        images: [Image::default(); MAX_IMAGES],
    };
    let fw = image(&f, images, fw_name)?;
    p.entry = fw.load;
    p.images[0] = fw;
    p.count = 1;
    let loadables = f.prop(c, b"loadables")?;
    for name in loadables.split(|&b| b == 0).filter(|s| !s.is_empty()) {
        if p.count == MAX_IMAGES {
            return Err(Error::TooMany);
        }
        p.images[p.count] = image(&f, images, name)?;
        p.count += 1;
    }
    let worst = (p.total + 511) & !511;
    for i in &p.images[..p.count] {
        let start = if i.relative {
            worst + i.offset
        } else {
            i.offset
        };
        p.end = p.end.max(start.checked_add(i.size).ok_or(Error::Bounds)?);
    }
    Ok(p)
}

impl Plan {
    pub fn images(&self) -> &[Image] {
        &self.images[..self.count]
    }

    /// Choose the data base (FIT totalsize rounded to 4, else to 512) by
    /// the first hashed relative image, and make every offset absolute.
    pub fn resolve(&mut self, fit: &[u8]) -> Result<(), Error> {
        let probe = self
            .images()
            .iter()
            .find(|i| i.relative && i.hash.is_some())
            .copied();
        let mut chosen = None;
        for align in [4usize, 512] {
            let base = (self.total + align - 1) & !(align - 1);
            match probe {
                Some(i) => {
                    if let Some(d) = fit.get(base + i.offset..base + i.offset + i.size) {
                        if Some(sha256::digest(d)) == i.hash {
                            chosen = Some(base);
                            break;
                        }
                    }
                }
                None => {
                    chosen = Some(base);
                    break;
                }
            }
        }
        let base = chosen.ok_or(Error::Hash)?;
        self.data_base = base;
        self.end = 0;
        for i in &mut self.images[..self.count] {
            if i.relative {
                i.offset += base;
                i.relative = false;
            }
            self.end = self.end.max(i.offset + i.size);
        }
        Ok(())
    }

    /// Verify every hashed image in `fit` (the whole FIT in memory) before
    /// anything is copied. Call after [`Plan::resolve`].
    pub fn verify(&self, fit: &[u8]) -> Result<(), Error> {
        for i in self.images() {
            if let Some(h) = i.hash {
                let data = fit.get(i.offset..i.offset + i.size).ok_or(Error::Bounds)?;
                if sha256::digest(data) != h {
                    return Err(Error::Hash);
                }
            }
        }
        Ok(())
    }

    /// Destinations must not overlap each other or `keep` (bt0 itself, the
    /// staging buffer, the handoff area).
    pub fn check_overlaps(&self, keep: &[(usize, usize)]) -> Result<(), Error> {
        let imgs = self.images();
        let over = |a: (usize, usize), b: (usize, usize)| a.0 < b.0 + b.1 && b.0 < a.0 + a.1;
        for (k, a) in imgs.iter().enumerate() {
            if a.optee {
                return Err(Error::Unexpected);
            }
            for b in &imgs[..k] {
                if over((a.load, a.size), (b.load, b.size)) {
                    return Err(Error::Bounds);
                }
            }
            for &r in keep {
                if over((a.load, a.size), r) {
                    return Err(Error::Bounds);
                }
            }
        }
        Ok(())
    }
}
