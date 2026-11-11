// SPDX-License-Identifier: GPL-2.0-only
//! `fastboot boot` / `stage` + `continue` end to end on mock hardware: a
//! scripted host downloads an edk2-rk3588 shaped FIT (crate rk3588's
//! mkimage fixture), the device checks it, `serve` returns, and placing it
//! produces exactly the copies, the BL31 FDT, the bl_params and the one
//! BL31 entry bt0's eMMC path makes. Also `flash:<target>` through the
//! eMMC hook (raw and sparse, the RK3588 regions) and the refusals.

use fastboot::{Config, Device, Transport};
use rk3588::handoff;
use rk3588_fastboot::device::{Board, Next, Part, Rk3588Device};
use rk3588_fastboot::stage::{self, Layout, Machine};
use std::collections::VecDeque;

const FIT: &[u8] = include_bytes!("../../rk3588/tests/fixtures/mini.itb");

struct Host {
    script: VecDeque<Vec<u8>>,
    replies: Vec<String>,
    disconnects: u32,
}
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Gone;
impl Transport for Host {
    type Error = Gone;
    fn wait_configured(&mut self) -> Result<(), Gone> {
        Ok(())
    }
    fn recv(&mut self, out: &mut [u8]) -> Result<usize, Gone> {
        let c = self.script.pop_front().ok_or(Gone)?;
        out[..c.len()].copy_from_slice(&c);
        Ok(c.len())
    }
    fn recv_exact(&mut self, out: &mut [u8]) -> Result<(), Gone> {
        let d = self.script.pop_front().ok_or(Gone)?;
        out.copy_from_slice(&d);
        Ok(())
    }
    fn send(&mut self, data: &[u8]) -> Result<(), Gone> {
        self.replies
            .push(String::from_utf8_lossy(data).into_owned());
        Ok(())
    }
    fn disconnect(&mut self) -> Result<(), Gone> {
        self.disconnects += 1;
        Ok(())
    }
    fn is_reset(_: &Gone) -> bool {
        false
    }
}
fn host(script: Vec<Vec<u8>>) -> Host {
    Host {
        script: script.into(),
        replies: Vec::new(),
        disconnects: 0,
    }
}

/// An eMMC of 8192 user sectors and two 256-sector boot partitions.
struct FakeBoard {
    part: Part,
    user: Vec<u8>,
    boot0: Vec<u8>,
    resets: u32,
}
impl FakeBoard {
    fn new() -> Self {
        Self {
            part: Part::User,
            user: vec![0xaa; 8192 * 512],
            boot0: vec![0xaa; 256 * 512],
            resets: 0,
        }
    }
}
impl Board for FakeBoard {
    fn emmc_select(&mut self, part: Part) -> Result<(), &'static str> {
        self.part = part;
        Ok(())
    }
    fn emmc_sectors(&mut self, part: Part) -> Result<u64, &'static str> {
        Ok(match part {
            Part::User => 8192,
            _ => 256,
        })
    }
    fn emmc_write(&mut self, lba: u32, data: &[u8]) -> Result<(), &'static str> {
        assert_eq!(data.len() % 512, 0);
        let disk = match self.part {
            Part::User => &mut self.user,
            Part::Boot0 => &mut self.boot0,
            Part::Boot1 => return Err("boot1 is read-only here"),
        };
        let o = lba as usize * 512;
        disk.get_mut(o..o + data.len())
            .ok_or("write past the end")?
            .copy_from_slice(data);
        Ok(())
    }
    fn reset(&mut self) -> Result<(), &'static str> {
        self.resets += 1;
        Err("reset (model)")
    }
}

#[derive(Default)]
struct Recorder {
    copies: Vec<(usize, Vec<u8>)>,
    fills: Vec<(usize, usize, u8)>,
    entries: Vec<(usize, usize, usize)>,
}
impl Machine for Recorder {
    fn copy(&mut self, dst: usize, src: &[u8]) {
        assert!(self.entries.is_empty(), "copy after the jump");
        self.copies.push((dst, src.to_vec()));
    }
    fn fill(&mut self, dst: usize, len: usize, value: u8) {
        assert!(self.entries.is_empty(), "fill after the jump");
        self.fills.push((dst, len, value));
    }
    fn enter_bl31(&mut self, entry: usize, params: usize, fdt: usize) {
        self.entries.push((entry, params, fdt));
    }
}

const DRAM: u64 = 8 << 30;

/// bt0's keep list with the download buffer at `buf`.
fn keep(buf: &[u8]) -> [(usize, usize); 4] {
    [
        stage::BT0_STEPPING_STONE,
        (rk3588::dump::RECORD_ADDR, rk3588::dump::RECORD_BYTES),
        stage::HANDOFF_WINDOW,
        (buf.as_ptr() as usize, buf.len()),
    ]
}

fn layout<'a>(keep: &'a [(usize, usize)]) -> Layout<'a> {
    Layout {
        fdt_addr: stage::FDT_ADDR,
        params_addr: stage::PARAMS_ADDR,
        bl33_default: 0x20_0000,
        keep,
    }
}

fn download(data: &[u8]) -> Vec<Vec<u8>> {
    vec![
        format!("download:{:08x}", data.len()).into_bytes(),
        data.to_vec(),
    ]
}

/// Serve `script`; on Next::Boot, place and enter on a Recorder.
fn session(script: Vec<Vec<u8>>) -> (Host, Next, Recorder, FakeBoard) {
    let mut buf = vec![0u8; 0x10000];
    let k = keep(&buf);
    let mut h = host(script);
    let mut d = Rk3588Device::new(FakeBoard::new(), "yy3588", "0123", &k);
    let r = fastboot::serve(&mut h, &mut d, &Config { unlocked: true }, &mut buf);
    let mut m = Recorder::default();
    if r.is_ok() {
        if let Next::Boot { plan, addr, len } = d.next {
            let off = addr - buf.as_ptr() as usize;
            assert!(off == 0 || off == 2048 || off == 4096);
            stage::enter(&mut m, &buf[off..off + len], &plan, DRAM, &layout(&k));
        }
    }
    let next = d.next;
    (h, next, m, d.board)
}

fn check_handoff(m: &Recorder) {
    let mut plan = rk3588::fit::plan(FIT).unwrap();
    plan.resolve(FIT).unwrap();
    // Every image, at its load address, with its bytes from the FIT.
    let images = plan.images();
    assert_eq!(m.copies.len(), images.len() + 2);
    for (c, i) in m.copies.iter().zip(images) {
        assert_eq!(c.0, i.load, "{}", i.name());
        assert_eq!(c.1, &FIT[i.offset..i.offset + i.size]);
    }
    // The BL31 FDT and bl_params, as handoff builds them for 8 GiB.
    let mut fdt = [0u8; 2048];
    let n = handoff::bl31_fdt(&mut fdt, DRAM);
    assert_eq!(m.copies[images.len()], (stage::FDT_ADDR, fdt[..n].to_vec()));
    let mut p = [0u8; handoff::TOTAL];
    handoff::bl_params(&mut p, stage::PARAMS_ADDR as u64, 0x20_0000, 3000, 0);
    assert_eq!(m.copies[images.len() + 1], (stage::PARAMS_ADDR, p.to_vec()));
    // BL33 = EDK2 at 0x200000 in the entry point info.
    let ep = handoff::BL_PARAMS_SIZE + handoff::NODE_SIZE;
    assert_eq!(&p[ep + 8..ep + 16], &0x20_0000u64.to_le_bytes());
    // One entry: BL31 (atf-1's load address), x0 = bl_params, x1 = FDT.
    assert_eq!(m.entries, [(0x40000, stage::PARAMS_ADDR, stage::FDT_ADDR)]);
}

#[test]
fn download_then_boot_enters_bl31_with_the_handoff() {
    let mut s = vec![b"getvar:product".to_vec()];
    s.extend(download(FIT));
    s.push(b"boot".to_vec());
    let (h, next, m, _) = session(s);
    assert!(matches!(next, Next::Boot { len, .. } if len == FIT.len()));
    assert_eq!(
        h.replies,
        [
            "OKAYyy3588".to_string(),
            format!("DATA{:08x}", FIT.len()),
            "OKAY".into(),
            "OKAY".into()
        ]
    );
    assert_eq!(h.disconnects, 1);
    check_handoff(&m);
}

#[test]
fn stage_then_continue_boots_the_same_way() {
    let mut s = download(FIT);
    s.push(b"continue".to_vec());
    let (h, _, m, _) = session(s);
    assert_eq!(h.replies.last().unwrap(), "OKAY");
    check_handoff(&m);
}

#[test]
fn bad_images_are_refused_and_the_session_goes_on() {
    let mut tampered = FIT.to_vec();
    let mut plan = rk3588::fit::plan(FIT).unwrap();
    plan.resolve(FIT).unwrap();
    tampered[plan.images()[1].offset + 5] ^= 1;
    let mut s = vec![b"boot".to_vec()];
    s.extend(download(
        b"not a fit at all, just bytes......................",
    ));
    s.push(b"boot".to_vec());
    s.extend(download(&tampered));
    s.push(b"boot".to_vec());
    // Cut inside a hashed image (cutting only into the unhashed nvdata
    // window is allowed: see the nvdata tests).
    let hashed_end = plan
        .images()
        .iter()
        .filter(|i| i.hash.is_some())
        .map(|i| i.offset + i.size)
        .max()
        .unwrap();
    s.extend(download(&FIT[..hashed_end - 100]));
    s.push(b"boot".to_vec());
    s.push(b"continue".to_vec());
    let (h, next, m, _) = session(s);
    let fails: Vec<_> = h.replies.iter().filter(|r| r.starts_with("FAIL")).collect();
    assert_eq!(fails.len(), 5, "{:?}", h.replies);
    assert_eq!(fails[0], "FAILdownload an image first");
    assert_eq!(fails[1], "FAILnot a FIT image");
    // The resolve probe (atf-1) matches; the edk2 hash does not.
    assert_eq!(fails[2], "FAILFIT hash");
    // The truncated FIT is staged, so the final continue tries it too.
    assert!(fails[3].starts_with("FAILFIT"));
    assert!(fails[4].starts_with("FAILFIT"));
    assert_eq!(next, Next::Continue);
    assert!(m.entries.is_empty());
}

#[test]
fn a_fit_landing_on_the_buffer_or_bt0_is_refused() {
    let buf = vec![0u8; 16];
    // The download buffer covering EDK2's load address.
    let k = [(0x1f_0000, 0x20_0000)];
    assert_eq!(
        stage::prepare(FIT, &k).err(),
        Some("FIT destinations overlap bt0, the buffer or the handoff")
    );
    assert!(stage::prepare(FIT, &keep(&buf)).is_ok());
}

#[test]
fn flash_targets_write_where_the_layout_says() {
    // idblock: LBA 64; fit: LBA 2048; a sparse image to emmc.
    let idb = vec![0x11u8; 700];
    let fit = vec![0x22u8; 1024];
    // Sparse: 4096-byte blocks, RAW 1 block of 0x33, FILL 1 block 0x44444444.
    let mut sp = Vec::new();
    sp.extend(0xed26_ff3au32.to_le_bytes());
    sp.extend(1u16.to_le_bytes()); // major
    sp.extend(0u16.to_le_bytes()); // minor
    sp.extend(28u16.to_le_bytes()); // file header
    sp.extend(12u16.to_le_bytes()); // chunk header
    sp.extend(4096u32.to_le_bytes());
    sp.extend(3u32.to_le_bytes()); // total blocks
    sp.extend(3u32.to_le_bytes()); // chunks
    sp.extend(0u32.to_le_bytes()); // checksum
    sp.extend(0xcac1u16.to_le_bytes());
    sp.extend(0u16.to_le_bytes());
    sp.extend(1u32.to_le_bytes());
    sp.extend((12 + 4096u32).to_le_bytes());
    sp.extend([0x33u8; 4096]);
    sp.extend(0xcac3u16.to_le_bytes());
    sp.extend(0u16.to_le_bytes());
    sp.extend(1u32.to_le_bytes());
    sp.extend(12u32.to_le_bytes());
    sp.extend(0xcac2u16.to_le_bytes());
    sp.extend(0u16.to_le_bytes());
    sp.extend(1u32.to_le_bytes());
    sp.extend(16u32.to_le_bytes());
    sp.extend(0x4444_4444u32.to_le_bytes());
    let mut s = download(&idb);
    s.push(b"flash:idblock".to_vec());
    s.extend(download(&fit));
    s.push(b"flash:fit".to_vec());
    s.extend(download(&sp));
    s.push(b"flash:emmc".to_vec());
    s.extend(download(&[0x55u8; 512]));
    s.push(b"flash:boot0".to_vec());
    s.push(b"flash:sd".to_vec());
    s.push(b"getvar:partition-size:idblock".to_vec());
    s.push(b"getvar:partition-size:emmc".to_vec());
    s.push(b"getvar:partition-type:fit".to_vec());
    s.push(b"reboot".to_vec());
    s.push(b"continue".to_vec());
    let (h, next, _, b) = session(s);
    assert_eq!(next, Next::Continue);
    let r: Vec<&str> = h.replies.iter().map(|s| s.as_str()).collect();
    assert_eq!(
        &r[11..],
        [
            "OKAY",
            "FAILunknown target",
            "OKAY0xf8000",
            "OKAY0x400000",
            "OKAYraw",
            "FAILreset (model)",
            "OKAY"
        ]
    );
    assert_eq!(b.resets, 1);
    // idblock: 700 bytes at 0x8000, the rest of its last sector zeroed.
    assert!(b.user[0x8000..0x8000 + 700].iter().all(|v| *v == 0x11));
    assert!(b.user[0x8000 + 700..0x8000 + 1024].iter().all(|v| *v == 0));
    assert_eq!(b.user[0x8000 + 1024], 0xaa);
    assert!(b.user[0x10_0000..0x10_0400].iter().all(|v| *v == 0x22));
    // Sparse to the user area: RAW block 0, block 1 untouched, FILL block 2
    // (the idblock written above lies in block 8 and later).
    assert!(b.user[..4096].iter().all(|v| *v == 0x33));
    assert!(b.user[4096..8192].iter().all(|v| *v == 0xaa));
    assert!(b.user[8192..12288].iter().all(|v| *v == 0x44));
    assert!(b.boot0[..512].iter().all(|v| *v == 0x55));
}

#[test]
fn an_idblock_larger_than_its_region_is_refused() {
    let big = vec![1u8; 1985 * 512];
    let mut buf = vec![0u8; 2 << 20];
    let k = keep(&buf);
    let mut d = Rk3588Device::new(FakeBoard::new(), "yy3588", "0", &k);
    buf[..big.len()].copy_from_slice(&big);
    assert_eq!(
        d.flash("idblock", big.len(), &mut buf[..big.len()]),
        Err("image larger than the partition")
    );
}

/// What the fastboot tool sends for `fastboot boot mini.itb`: an Android
/// boot image (header version 0, 2048-byte pages) with the FIT as kernel.
fn android_boot_image(kernel: &[u8], version: u32) -> Vec<u8> {
    let page = if version >= 3 { 4096 } else { 2048 };
    let mut v = vec![0u8; page];
    v[..8].copy_from_slice(b"ANDROID!");
    v[8..12].copy_from_slice(&(kernel.len() as u32).to_le_bytes());
    if version < 3 {
        v[36..40].copy_from_slice(&(page as u32).to_le_bytes());
    }
    v[40..44].copy_from_slice(&version.to_le_bytes());
    v.extend_from_slice(kernel);
    v.resize(v.len().next_multiple_of(page), 0);
    v
}

#[test]
fn fastboot_boot_of_a_wrapped_fit_boots_the_fit() {
    for version in [0, 2, 4] {
        let img = android_boot_image(FIT, version);
        let mut s = download(&img);
        s.push(b"boot".to_vec());
        let (h, next, m, _) = session(s);
        assert!(matches!(next, Next::Boot { len, .. } if len == FIT.len()));
        assert_eq!(h.replies.last().unwrap(), "OKAY");
        check_handoff(&m);
    }
    assert_eq!(
        stage::unwrap(&android_boot_image(FIT, 0)[..3000]).err(),
        Some("boot image shorter than its kernel")
    );
    assert_eq!(stage::unwrap(FIT).unwrap().len(), FIT.len());
}

/// An edk2-rk3588 shaped FIT as edk2-rk3588's mkimage writes it: hashed
/// images by `data-offset` from the 512-byte aligned end of the structure,
/// and `nvdata` by `data-position` 0x6c0000, size 0x30000, load 0x7c0000
/// (misc/uefi_rk3588.its). The file ends `nv_in_file` bytes into the
/// window when the window starts right after the hashed data
/// (`nv_at_end`), else well before it, as a FIT cut from a built image
/// does. `nv_hash` gives nvdata a hash node (never filled); `atf2_hash`
/// false leaves atf-2 unhashed.
struct NvFit {
    bytes: Vec<u8>,
    nv_pos: usize,
}
const NV_SIZE: usize = 0x3_0000;
const NV_LOAD: usize = 0x7c_0000;
fn nv_fit(nv_at_end: bool, nv_in_file: usize, nv_hash: bool, atf2_hash: bool) -> NvFit {
    let edk2: Vec<u8> = (0..3000u32).map(|i| (i * 7) as u8).collect();
    let atf1: Vec<u8> = (0..1500u32).map(|i| (i * 3 + 1) as u8).collect();
    let atf2: Vec<u8> = (0..700u32).map(|i| (i * 5 + 2) as u8).collect();
    let dtb = vec![0u8; 64];
    let payloads = [&edk2, &atf1, &atf2, &dtb];
    let rel: Vec<usize> = payloads
        .iter()
        .scan(0, |o, p| {
            let at = *o;
            *o += p.len().next_multiple_of(4);
            Some(at)
        })
        .collect();
    let data_len = rel[3] + dtb.len();
    let nv_file: Vec<u8> = vec![0x5a; nv_in_file];
    let structure = |nv_pos: usize| {
        let mut w = fdt_writer::Fdt::new();
        w.begin_node("");
        w.prop_str("description", "nvdata past the download");
        w.prop_u32("#address-cells", 1);
        w.begin_node("images");
        let names = ["edk2", "atf-1", "atf-2", "fdt"];
        let loads = [0x20_0000u32, 0x4_0000, 0xff10_0000, 0];
        for k in 0..4 {
            w.begin_node(names[k]);
            w.prop_u32("data-offset", rel[k] as u32);
            w.prop_u32("data-size", payloads[k].len() as u32);
            if k < 3 {
                w.prop_u32("load", loads[k]);
            }
            if k != 2 || atf2_hash {
                w.begin_node("hash");
                w.prop_str("algo", "sha256");
                w.prop("value", &sha256::digest(payloads[k]));
                w.end_node();
            }
            w.end_node();
        }
        w.begin_node("nvdata");
        w.prop_u32("data-position", nv_pos as u32);
        w.prop_u32("data-size", NV_SIZE as u32);
        w.prop_u32("load", NV_LOAD as u32);
        if nv_hash {
            w.begin_node("hash");
            w.prop_str("algo", "sha256");
            w.prop("value", &[0u8; 32]);
            w.end_node();
        }
        w.end_node();
        w.end_node();
        w.begin_node("configurations");
        w.prop_str("default", "conf");
        w.begin_node("conf");
        w.prop_str("firmware", "atf-1");
        w.prop("loadables", b"edk2\0atf-2\0nvdata\0");
        w.end_node();
        w.end_node();
        w.end_node();
        let mut out = vec![0u8; 4096];
        let n = w.finish(&mut out);
        out.truncate(n);
        out
    };
    // Data base: the structure size rounded up to 512 (edk2-rk3588's
    // mkimage); data-position does not change the structure size.
    let base = structure(0).len().next_multiple_of(512);
    let nv_pos = if nv_at_end {
        base + data_len.next_multiple_of(512)
    } else {
        0x6c_0000
    };
    let mut out = structure(nv_pos);
    out.resize(base, 0);
    for (k, p) in payloads.iter().enumerate() {
        out.resize(base + rel[k], 0);
        out.extend_from_slice(p);
    }
    if nv_at_end {
        out.resize(nv_pos, 0);
        out.extend_from_slice(&nv_file);
    }
    NvFit { bytes: out, nv_pos }
}

fn nv_copies_and_fill(m: &Recorder, fit: &[u8], nv_pos: usize) -> (usize, usize) {
    let mut plan = rk3588::fit::plan(fit).unwrap();
    plan.resolve(fit).unwrap();
    assert_eq!(plan.data_base % 512, 0);
    // Every hashed image placed whole.
    for i in plan.images().iter().filter(|i| i.hash.is_some()) {
        assert!(m
            .copies
            .iter()
            .any(|c| c.0 == i.load && c.1 == fit[i.offset..i.offset + i.size]));
    }
    let have = fit.len().saturating_sub(nv_pos).min(NV_SIZE);
    if have > 0 {
        assert!(m
            .copies
            .iter()
            .any(|c| c.0 == NV_LOAD && c.1 == fit[nv_pos..nv_pos + have]));
    }
    assert_eq!(m.fills, [(NV_LOAD + have, NV_SIZE - have, 0xff)]);
    assert_eq!(m.entries, [(0x40000, stage::PARAMS_ADDR, stage::FDT_ADDR)]);
    (have, NV_SIZE - have)
}

#[test]
fn nvdata_window_past_the_download_is_filled_raw_and_wrapped() {
    let f = nv_fit(false, 0, false, true);
    assert!(f.bytes.len() < 0x2000 && f.nv_pos == 0x6c_0000);
    // stage + continue (raw) and boot (wrapped in an Android boot image).
    let mut raw = download(&f.bytes);
    raw.push(b"continue".to_vec());
    let mut wrapped = download(&android_boot_image(&f.bytes, 0));
    wrapped.push(b"boot".to_vec());
    for s in [raw, wrapped] {
        let (h, next, m, _) = session(s);
        assert_eq!(h.replies.last().unwrap(), "OKAY", "{:?}", h.replies);
        let Next::Boot { plan, len, .. } = next else {
            panic!("not booted")
        };
        assert_eq!(len, f.bytes.len());
        let (i, n) = stage::filled(&plan, len).unwrap();
        assert_eq!((i.name(), n), ("nvdata", NV_SIZE));
        assert_eq!(nv_copies_and_fill(&m, &f.bytes, f.nv_pos), (0, NV_SIZE));
    }
}

#[test]
fn nvdata_window_partly_in_the_download_keeps_its_bytes() {
    let f = nv_fit(true, 0x200, false, true);
    let (have, missing) = {
        let mut s = download(&f.bytes);
        s.push(b"boot".to_vec());
        let (h, _, m, _) = session(s);
        assert_eq!(h.replies.last().unwrap(), "OKAY", "{:?}", h.replies);
        nv_copies_and_fill(&m, &f.bytes, f.nv_pos)
    };
    assert_eq!((have, missing), (0x200, NV_SIZE - 0x200));
}

#[test]
fn only_an_unhashed_nvdata_window_may_be_missing() {
    let buf = vec![0u8; 16];
    let k = keep(&buf);
    // nvdata with a hash: refused.
    let f = nv_fit(false, 0, true, true);
    assert_eq!(
        stage::prepare(&f.bytes, &k).err(),
        Some("FIT data past the download")
    );
    // An unhashed image other than nvdata (atf-2) cut off: refused.
    let f = nv_fit(false, 0, false, false);
    let mut p = rk3588::fit::plan(&f.bytes).unwrap();
    p.resolve(&f.bytes).unwrap();
    let e = p.images().iter().find(|i| i.name() == "atf-2").unwrap();
    assert!(e.hash.is_none());
    let short = &f.bytes[..e.offset + e.size - 10];
    assert_eq!(
        stage::prepare(short, &k).err(),
        Some("FIT data past the download")
    );
    // The complete FIT with its window present needs no fill.
    let f = nv_fit(true, NV_SIZE, false, true);
    let plan = stage::prepare(&f.bytes, &k).unwrap();
    assert!(stage::filled(&plan, f.bytes.len()).is_none());
}

/// A FIT cut from a real built image (YY3588_FIT, e.g. the 0x52C200 bytes
/// `dd`'d from YY3588_rkbinddr_DEBUG.img) now prepares, with the whole
/// nvdata window filled.
#[test]
fn real_cut_image_if_given() {
    let Ok(path) = std::env::var("YY3588_FIT") else {
        return;
    };
    let bytes = std::fs::read(path).unwrap();
    let buf = vec![0u8; 16];
    let plan = stage::prepare(&bytes, &keep(&buf)).unwrap();
    let (i, n) = stage::filled(&plan, bytes.len()).expect("the window is past the file");
    assert_eq!((i.name(), i.offset, i.load), ("nvdata", 0x6c_0000, NV_LOAD));
    assert_eq!(n, NV_SIZE);
    let wrapped = android_boot_image(&bytes, 0);
    assert_eq!(stage::unwrap(&wrapped).unwrap(), &bytes[..]);
}
