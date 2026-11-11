// SPDX-License-Identifier: GPL-2.0-only
//! The Rockchip loader container `rkdeveloptool db` sends to a board in
//! MaskROM mode, written from GPL sources only:
//!
//! - layout: rkdeveloptool (GPL-2.0+, github.com/rockchip-linux/rkdeveloptool
//!   304f0737) boot_merger.h / RKBoot.h (`rk_boot_header`, `rk_boot_entry`,
//!   both `#pragma pack(1)`), main.cpp `mergeBoot` / `getBoothdr` /
//!   `saveEntry` (entries, then the data of each entry in the same order,
//!   each padded to 2048 bytes, then a CRC_32 of everything before it);
//! - reader checks: RKBoot.cpp `CRKBoot::CRKBoot` (tag "BOOT" or "LDR ",
//!   CRC_32, `rc4Flag` != 0 means the data is stored and sent unencrypted);
//! - transfer: RKDevice.cpp `DownloadBoot` sends each 471 entry, then each
//!   472 entry, as stored (padded size), with Boot_VendorRequest 0x471 /
//!   0x472, sleeping `dataDelay` ms after each;
//! - CRC_32: crc.cpp `CRC_32` (MSB first, polynomial 0x04C10DB7, initial 0,
//!   no final xor), table generated here.
//!
//! RK3588 (rkbin RKBOOT/RK3588MINIALL.ini `NEWIDB=true`) loaders start the
//! 471 list with "UsbHead": the 2048-byte idblock v2 header describing the
//! 471 data image and the 472 image. Checked on a Rockchip-built loader:
//! that entry is byte-identical to the header `rk-idblock pack --init <471>
//! --boot <472>` writes, and the 471/472 data entries are the input files
//! unchanged, zero-padded to 2048 (container inspection only; nothing in
//! any blob was disassembled or decoded).
//!
//! Mainline U-Boot (GPL-2.0+) builds the same pair without UsbHead
//! (arch/arm/dts/rockchip-u-boot.dtsi, CONFIG_ROCKCHIP_MASKROM_IMAGE:
//! u-boot-rockchip-usb471.bin = the TPL / rkbin DDR init,
//! u-boot-rockchip-usb472.bin = SPL linked at SPL_TEXT_BASE, 0 on RK3588,
//! arch/arm/mach-rockchip/Kconfig): the BootROM runs the 471 data, then
//! loads the 472 data to DRAM 0x0 and enters it. [`Head::None`] builds that
//! variant as a fallback.

pub const TAG_BOOT: u32 = 0x544F_4F42; // "BOOT"
pub const TAG_LDR: u32 = 0x2052_444C; // "LDR "
pub const HEADER_SIZE: usize = 102;
pub const ENTRY_SIZE: usize = 57;
pub const ALIGN: usize = 2048;
/// rkdeveloptool boot_merger.h MERGER_VERSION.
pub const MERGER_VERSION: u32 = 0x0103_0000;
/// rkdeveloptool getChipType("RK3588") = convertChipType("3588") (bytes
/// '3','5','8','8' big-endian into a u32).
pub const CHIP_RK3588: u32 = 0x3335_3838;
pub const NAME_CHARS: usize = 20;
pub const RESERVED: usize = HEADER_SIZE - 45;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    /// Sent first (DDR init); `rk_boot_entry.type` 1.
    E471 = 1,
    /// Sent second (usbplug, here bt0); type 2.
    E472 = 2,
    /// Written to flash by `rkdeveloptool ul`; type 4.
    Loader = 4,
}

impl Kind {
    fn from_u32(v: u32) -> Option<Kind> {
        match v {
            1 => Some(Kind::E471),
            2 => Some(Kind::E472),
            4 => Some(Kind::Loader),
            _ => None,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Entry {
    pub kind: Kind,
    /// At most 20 UTF-16 code units (longer names are truncated, as
    /// rkdeveloptool does).
    pub name: String,
    /// Stored data (padded to [`ALIGN`] by [`build`]).
    pub data: Vec<u8>,
    pub delay_ms: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Time {
    pub year: u16,
    pub month: u8,
    pub day: u8,
    pub hour: u8,
    pub minute: u8,
    pub second: u8,
}

impl Time {
    /// UTC calendar time of a Unix timestamp (for SOURCE_DATE_EPOCH).
    pub fn from_unix(secs: u64) -> Time {
        let days = (secs / 86_400) as i64;
        let rem = secs % 86_400;
        // Howard Hinnant's civil_from_days.
        let z = days + 719_468;
        let era = z.div_euclid(146_097);
        let doe = z - era * 146_097;
        let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
        let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
        let mp = (5 * doy + 2) / 153;
        let day = (doy - (153 * mp + 2) / 5 + 1) as u8;
        let month = if mp < 10 { mp + 3 } else { mp - 9 } as u8;
        let year = (yoe + era * 400 + if month <= 2 { 1 } else { 0 }) as u16;
        Time {
            year,
            month,
            day,
            hour: (rem / 3600) as u8,
            minute: (rem / 60 % 60) as u8,
            second: (rem % 60) as u8,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Loader {
    pub tag: u32,
    /// `major << 8 | minor` (rkdeveloptool getBoothdr).
    pub version: u32,
    pub merge_version: u32,
    pub time: Time,
    pub chip: u32,
    /// `rc4Flag`: true = data stored and sent unencrypted.
    pub rc4_disabled: bool,
    pub sign_flag: u8,
    /// `reserved[57]` after rc4Flag. Rockchip-built RK3588 loaders have
    /// reserved[0] = 1 (meaning not documented; rkdeveloptool ignores the
    /// field and the BootROM never sees the header). Kept for exact repacks.
    pub reserved: [u8; RESERVED],
    /// In file order: all 471, then all 472, then all loader entries.
    pub entries: Vec<Entry>,
}

pub fn crc32(data: &[u8]) -> u32 {
    const fn table() -> [u32; 256] {
        let mut t = [0u32; 256];
        let mut i = 0;
        while i < 256 {
            let mut c = (i as u32) << 24;
            let mut k = 0;
            while k < 8 {
                c = if c & 0x8000_0000 != 0 {
                    (c << 1) ^ 0x04C1_0DB7
                } else {
                    c << 1
                };
                k += 1;
            }
            t[i] = c;
            i += 1;
        }
        t
    }
    const T: [u32; 256] = table();
    data.iter().fold(0u32, |acc, &b| {
        (acc << 8) ^ T[((acc >> 24) as u8 ^ b) as usize]
    })
}

fn pad(n: usize) -> usize {
    n.div_ceil(ALIGN) * ALIGN
}

/// Serialise. Entries must already be grouped 471, 472, loader.
pub fn build(l: &Loader) -> Result<Vec<u8>, String> {
    let order = |k: Kind| match k {
        Kind::E471 => 0,
        Kind::E472 => 1,
        Kind::Loader => 2,
    };
    if !l
        .entries
        .windows(2)
        .all(|w| order(w[0].kind) <= order(w[1].kind))
    {
        return Err("entries must be grouped 471, 472, loader".into());
    }
    let count = |k: Kind| l.entries.iter().filter(|e| e.kind == k).count();
    let (n471, n472, nld) = (count(Kind::E471), count(Kind::E472), count(Kind::Loader));
    if n471 > 255 || n472 > 255 || nld > 255 {
        return Err("too many entries".into());
    }
    let mut out = vec![0u8; HEADER_SIZE + ENTRY_SIZE * l.entries.len()];
    let put = |o: &mut [u8], at: usize, v: &[u8]| o[at..at + v.len()].copy_from_slice(v);
    put(&mut out, 0, &l.tag.to_le_bytes());
    put(&mut out, 4, &(HEADER_SIZE as u16).to_le_bytes());
    put(&mut out, 6, &l.version.to_le_bytes());
    put(&mut out, 10, &l.merge_version.to_le_bytes());
    let t = l.time;
    put(&mut out, 14, &t.year.to_le_bytes());
    put(&mut out, 16, &[t.month, t.day, t.hour, t.minute, t.second]);
    put(&mut out, 21, &l.chip.to_le_bytes());
    let mut at = HEADER_SIZE;
    for (n, field) in [(n471, 25), (n472, 31), (nld, 37)] {
        put(&mut out, field, &[n as u8]);
        put(&mut out, field + 1, &(at as u32).to_le_bytes());
        put(&mut out, field + 5, &[ENTRY_SIZE as u8]);
        at += ENTRY_SIZE * n;
    }
    put(&mut out, 43, &[l.sign_flag, l.rc4_disabled as u8]);
    put(&mut out, 45, &l.reserved);
    let mut data_at = out.len();
    for (i, e) in l.entries.iter().enumerate() {
        let size = pad(e.data.len());
        let o = HEADER_SIZE + ENTRY_SIZE * i;
        put(&mut out, o, &[ENTRY_SIZE as u8]);
        put(&mut out, o + 1, &(e.kind as u32).to_le_bytes());
        for (j, u) in e.name.encode_utf16().take(NAME_CHARS).enumerate() {
            put(&mut out, o + 5 + 2 * j, &u.to_le_bytes());
        }
        put(&mut out, o + 45, &(data_at as u32).to_le_bytes());
        put(&mut out, o + 49, &(size as u32).to_le_bytes());
        put(&mut out, o + 53, &e.delay_ms.to_le_bytes());
        data_at += size;
    }
    // Data follows the entries directly (no alignment of the start); each
    // entry's data is zero-padded to a multiple of ALIGN.
    for e in &l.entries {
        let end = out.len() + pad(e.data.len());
        out.extend_from_slice(&e.data);
        out.resize(end, 0);
    }
    let c = crc32(&out);
    out.extend_from_slice(&c.to_le_bytes());
    Ok(out)
}

/// Parse and check (tag, CRC_32, bounds), as CRKBoot does.
pub fn parse(b: &[u8]) -> Result<Loader, String> {
    if b.len() < HEADER_SIZE + 4 {
        return Err("too short".into());
    }
    let u16_ = |at: usize| u16::from_le_bytes([b[at], b[at + 1]]);
    let u32_ = |at: usize| u32::from_le_bytes(b[at..at + 4].try_into().unwrap());
    let tag = u32_(0);
    if tag != TAG_BOOT && tag != TAG_LDR {
        return Err(format!("bad tag {tag:#010x}"));
    }
    let stored = u32_(b.len() - 4);
    let calc = crc32(&b[..b.len() - 4]);
    if stored != calc {
        return Err(format!(
            "CRC_32 mismatch: stored {stored:#010x}, computed {calc:#010x}"
        ));
    }
    let mut entries = Vec::new();
    for (field, want) in [(25, Kind::E471), (31, Kind::E472), (37, Kind::Loader)] {
        let n = b[field] as usize;
        let off = u32_(field + 1) as usize;
        let esz = b[field + 5] as usize;
        if n > 0 && esz < ENTRY_SIZE {
            return Err("entry size too small".into());
        }
        for k in 0..n {
            let o = off + k * esz;
            let e = b.get(o..o + ENTRY_SIZE).ok_or("entry out of bounds")?;
            let kind = Kind::from_u32(u32::from_le_bytes(e[1..5].try_into().unwrap()))
                .ok_or("unknown entry type")?;
            if kind != want {
                return Err(format!("entry type {kind:?} in the {want:?} list"));
            }
            let units: Vec<u16> = (0..NAME_CHARS)
                .map(|j| u16::from_le_bytes([e[5 + 2 * j], e[6 + 2 * j]]))
                .collect();
            let len = units.iter().position(|&u| u == 0).unwrap_or(NAME_CHARS);
            let name = String::from_utf16_lossy(&units[..len]);
            let doff = u32::from_le_bytes(e[45..49].try_into().unwrap()) as usize;
            let dsz = u32::from_le_bytes(e[49..53].try_into().unwrap()) as usize;
            let delay_ms = u32::from_le_bytes(e[53..57].try_into().unwrap());
            let data = b
                .get(doff..doff + dsz)
                .filter(|_| doff + dsz <= b.len() - 4)
                .ok_or("entry data out of bounds")?
                .to_vec();
            entries.push(Entry {
                kind,
                name,
                data,
                delay_ms,
            });
        }
    }
    Ok(Loader {
        tag,
        version: u32_(6),
        merge_version: u32_(10),
        time: Time {
            year: u16_(14),
            month: b[16],
            day: b[17],
            hour: b[18],
            minute: b[19],
            second: b[20],
        },
        chip: u32_(21),
        rc4_disabled: b[44] != 0,
        sign_flag: b[43],
        reserved: b[45..HEADER_SIZE].try_into().unwrap(),
        entries,
    })
}

/// The RK3588 MaskROM loader: 471 = [UsbHead, `ddr`], 472 = [`next`].
/// `ddr` is the DDR init (rkbin's in the stepping-stone flow), `next` the
/// image the BootROM loads after it returns (our bt0, linked at DRAM 0x0).
/// No loader (flash) entries: this file is for `rkdeveloptool db` only.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Head {
    /// UsbHead first, as rkbin's boot_merger (NEWIDB) writes.
    UsbHead,
    /// The raw 471 data only, as mainline U-Boot's usb471 image.
    None,
}

pub fn rk3588_usb(
    ddr: &[u8],
    ddr_name: &str,
    next: &[u8],
    next_name: &str,
    time: Time,
) -> Result<Loader, String> {
    rk3588_usb_with(Head::UsbHead, ddr, ddr_name, next, next_name, time)
}

pub fn rk3588_usb_with(
    head: Head,
    ddr: &[u8],
    ddr_name: &str,
    next: &[u8],
    next_name: &str,
    time: Time,
) -> Result<Loader, String> {
    let idb = rk_idblock::build("rk3588", ddr, Some(next)).map_err(|e| format!("{e:?}"))?;
    let mut l = Loader {
        tag: TAG_LDR,
        version: 0x0100,
        merge_version: MERGER_VERSION,
        time,
        chip: CHIP_RK3588,
        rc4_disabled: true,
        sign_flag: 0,
        reserved: {
            let mut r = [0u8; RESERVED];
            r[0] = 1;
            r
        },
        entries: vec![
            Entry {
                kind: Kind::E471,
                name: "UsbHead".into(),
                data: idb[..rk_idblock::HEADER_SIZE].to_vec(),
                delay_ms: 1,
            },
            Entry {
                kind: Kind::E471,
                name: ddr_name.into(),
                data: ddr.to_vec(),
                delay_ms: 1,
            },
            Entry {
                kind: Kind::E472,
                name: next_name.into(),
                data: next.to_vec(),
                delay_ms: 0,
            },
        ],
    };
    if head == Head::None {
        l.entries.remove(0);
    }
    Ok(l)
}

/// For an RK3588 USB loader: rebuild the idblock the UsbHead entry
/// describes (UsbHead + 471[1] + 472[0]) and verify its hashes with
/// rk-idblock. Returns the (offset, size) of the two images.
pub fn check_usbhead(l: &Loader) -> Result<[(usize, usize); 2], String> {
    let e471: Vec<&Entry> = l.entries.iter().filter(|e| e.kind == Kind::E471).collect();
    let e472: Vec<&Entry> = l.entries.iter().filter(|e| e.kind == Kind::E472).collect();
    match (e471.as_slice(), e472.as_slice()) {
        ([head, ddr], [next]) if head.name == "UsbHead" => {
            let mut idb = head.data.clone();
            idb.extend_from_slice(&ddr.data);
            idb.resize(idb.len().div_ceil(ALIGN) * ALIGN, 0);
            idb.extend_from_slice(&next.data);
            idb.resize(idb.len().div_ceil(ALIGN) * ALIGN, 0);
            rk_idblock::verify(&idb)
        }
        _ => Err("not a UsbHead + 471 + 472 loader".into()),
    }
}
