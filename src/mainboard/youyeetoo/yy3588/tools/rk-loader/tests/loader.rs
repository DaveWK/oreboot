// SPDX-License-Identifier: GPL-2.0-only
//! Container tests. RK_LOADER_REF (optional) names a Rockchip-built RK3588
//! loader (e.g. rkbin boot_merger output): it must parse, pass the CRC,
//! and rebuild byte-identically from its parsed fields, which checks this
//! writer's layout against Rockchip's tool. RK_LOADER_DDR / RK_LOADER_472
//! (optional) are the rkbin files it was merged from, to check the 471/472
//! data and the UsbHead equivalence with rk-idblock.

use rk_loader::*;

fn sample() -> Loader {
    rk3588_usb(
        &[0xAA; 3000],
        "ddr",
        &[0x55; 5000],
        "bt0",
        Time::from_unix(0),
    )
    .unwrap()
}

#[test]
fn crc32_matches_rkdeveloptool_table() {
    // crc.cpp gTable_Crc32[1] and [2] are the CRC of single bytes 1, 2.
    assert_eq!(crc32(&[1]), 0x04c1_0db7);
    assert_eq!(crc32(&[2]), 0x0982_1b6e);
    assert_eq!(crc32(&[]), 0);
}

#[test]
fn layout() {
    let b = build(&sample()).unwrap();
    // header, 3 entries, 2048 + 4096 + 6144 bytes of data, CRC.
    assert_eq!(
        b.len(),
        HEADER_SIZE + 3 * ENTRY_SIZE + 2048 + 4096 + 6144 + 4
    );
    assert_eq!(&b[0..4], b"LDR ");
    assert_eq!(u16::from_le_bytes([b[4], b[5]]) as usize, HEADER_SIZE);
    assert_eq!((b[25], b[31], b[37]), (2, 1, 0));
    assert_eq!(b[44], 1, "rc4Flag: data unencrypted");
    let data0 = HEADER_SIZE + 3 * ENTRY_SIZE;
    assert_eq!(&b[data0..data0 + 4], b"RKNS");
    let ddr = data0 + 2048;
    assert!(b[ddr..ddr + 3000].iter().all(|&x| x == 0xAA));
    assert!(b[ddr + 3000..ddr + 4096].iter().all(|&x| x == 0));
}

#[test]
fn roundtrip_and_usbhead() {
    let l = sample();
    let b = build(&l).unwrap();
    let p = parse(&b).unwrap();
    assert_eq!(p.entries.len(), 3);
    assert_eq!(p.entries[1].data[..3000], l.entries[1].data[..]);
    assert_eq!(build(&p).unwrap(), b);
    let imgs = check_usbhead(&p).unwrap();
    assert_eq!(imgs[0].1, 4096);
    assert_eq!(imgs[1].1, 6144);
}

#[test]
fn no_usbhead_variant() {
    let l = rk3588_usb_with(
        Head::None,
        &[1; 100],
        "ddr",
        &[2; 100],
        "bt0",
        Time::from_unix(0),
    )
    .unwrap();
    let b = build(&l).unwrap();
    assert_eq!((b[25], b[31]), (1, 1));
    let data0 = HEADER_SIZE + 2 * ENTRY_SIZE;
    assert_eq!(b[data0], 1);
    assert!(check_usbhead(&parse(&b).unwrap()).is_err());
}

#[test]
fn corrupt_crc_rejected() {
    let mut b = build(&sample()).unwrap();
    let n = b.len();
    b[n / 2] ^= 1;
    assert!(parse(&b).unwrap_err().contains("CRC"));
}

#[test]
fn unix_time() {
    let t = Time::from_unix(1_762_646_400); // 2025-11-09T00:00:00Z
    assert_eq!((t.year, t.month, t.day, t.hour), (2025, 11, 9, 0));
}

#[test]
fn rockchip_reference() {
    let Ok(path) = std::env::var("RK_LOADER_REF") else {
        eprintln!("RK_LOADER_REF not set: skipped");
        return;
    };
    let b = std::fs::read(path).unwrap();
    let l = parse(&b).unwrap();
    let r = build(&l).unwrap();
    let diff = r.iter().zip(&b).position(|(x, y)| x != y);
    assert!(
        r.len() == b.len() && diff.is_none(),
        "rebuild must be byte-identical: len {} vs {}, first difference at {diff:#x?}",
        r.len(),
        b.len()
    );
    let (Ok(ddr), Ok(e472)) = (
        std::env::var("RK_LOADER_DDR"),
        std::env::var("RK_LOADER_472"),
    ) else {
        return;
    };
    let ddr = std::fs::read(ddr).unwrap();
    let next = std::fs::read(e472).unwrap();
    let ours = rk3588_usb(&ddr, "x", &next, "y", l.time).unwrap();
    // UsbHead and the 471/472 data are what we would write.
    for k in 0..3 {
        assert_eq!(ours.entries[k].kind, l.entries[k].kind);
        let mut d = ours.entries[k].data.clone();
        d.resize(d.len().div_ceil(ALIGN) * ALIGN, 0);
        assert_eq!(d, l.entries[k].data, "entry {k}");
    }
    check_usbhead(&l).unwrap();
}
