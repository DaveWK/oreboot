// SPDX-License-Identifier: GPL-2.0-only
//! FIT planning and verification (crate::fit) on a mkimage -E fixture
//! shaped like edk2-rk3588's, and on a real image when YY3588_FIT names one.
use rk3588::fit;

const MINI: &[u8] = include_bytes!("fixtures/mini.itb");

#[test]
fn mini_plan() {
    let mut p = fit::plan(MINI).unwrap();
    p.resolve(MINI).unwrap();
    // mainline mkimage 2026.04: data base = totalsize rounded up to 4.
    assert_eq!(p.data_base, (p.total + 3) & !3);
    assert_eq!(p.entry, 0x40000);
    let names: Vec<_> = p.images().iter().map(|i| i.name().to_string()).collect();
    assert_eq!(names, ["atf-1", "edk2", "atf-2", "nvdata"]);
    let loads: Vec<_> = p.images().iter().map(|i| i.load).collect();
    assert_eq!(loads, [0x40000, 0x200000, 0xff10_0000, 0x7c_0000]);
    let sizes: Vec<_> = p.images().iter().map(|i| i.size).collect();
    assert_eq!(sizes, [1500, 3000, 700, 0x400]);
    // nvdata carries no hash; mainline mkimage turns its data-position
    // into a data-offset (the edk2-rk3588 mkimage keeps data-position, see
    // real_image_if_given).
    let nv = p.images()[3];
    assert_eq!(nv.hash, None);
    assert_eq!(p.end, MINI.len());
    p.verify(MINI).unwrap();
    // The payload bytes are where the plan says.
    let edk2 = p.images()[1];
    assert_eq!(MINI[edk2.offset + 1], 7);
}

#[test]
fn tamper_and_overlap() {
    let mut p = fit::plan(MINI).unwrap();
    p.resolve(MINI).unwrap();
    let mut bad = MINI.to_vec();
    bad[p.images()[2].offset] ^= 0xff;
    assert_eq!(p.verify(&bad), Err(fit::Error::Hash));
    // A FIT whose data matches neither base is refused before placement.
    let mut q = fit::plan(MINI).unwrap();
    let mut worse = MINI.to_vec();
    let off = p.images()[0].offset;
    worse[off] ^= 0xff;
    assert_eq!(q.resolve(&worse), Err(fit::Error::Hash));
    // bt0 at DRAM 0 (stepping stone) must not overlap BL31 at 0x40000.
    assert!(p.check_overlaps(&[(0, 0x3_0000)]).is_ok());
    assert_eq!(p.check_overlaps(&[(0, 0x5_0000)]), Err(fit::Error::Bounds));
}

#[test]
fn real_image_if_given() {
    let Ok(path) = std::env::var("YY3588_FIT") else {
        return;
    };
    let bytes = std::fs::read(path).unwrap();
    let mut p = fit::plan(&bytes).unwrap();
    p.resolve(&bytes).unwrap();
    // edk2-rk3588's mkimage: data base = totalsize rounded up to 512.
    assert_eq!(p.data_base, (p.total + 511) & !511);
    assert_eq!(p.entry, 0x40000);
    assert!(p.images().iter().all(|i| !i.optee), "BL32_BIN=none FIT");
    assert!(p
        .images()
        .iter()
        .any(|i| i.load == 0x20_0000 && i.name() == "edk2"));
    assert!(p.images().iter().any(|i| i.load == 0xff10_0000));
    assert!(p
        .images()
        .iter()
        .any(|i| i.name() == "nvdata" && i.offset == 0x6c_0000));
    p.verify(&bytes).unwrap();
    p.check_overlaps(&[(0, 0x3_0000), (0x0400_0000, 0x80_0000)])
        .unwrap();
}
