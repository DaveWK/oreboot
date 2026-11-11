// SPDX-License-Identifier: GPL-2.0-only
//! Byte-for-byte check against U-Boot's `mkimage -T rksd`, the public
//! reference for the v2 header, when mkimage is installed (the Fedora 44
//! runners have uboot-tools). Skipped otherwise.

use std::path::PathBuf;
use std::process::Command;

fn mkimage(soc: &str, files: &[&PathBuf], out: &PathBuf) -> Option<Vec<u8>> {
    let d = files
        .iter()
        .map(|p| p.display().to_string())
        .collect::<Vec<_>>()
        .join(":");
    let st = Command::new("mkimage")
        .args(["-n", soc, "-T", "rksd", "-d", &d])
        .arg(out)
        .output()
        .ok()?;
    assert!(st.status.success(), "mkimage failed: {st:?}");
    Some(std::fs::read(out).unwrap())
}

#[test]
fn matches_mkimage_rksd() {
    let dir = PathBuf::from(env!("CARGO_TARGET_TMPDIR")).join("rk-idblock-mkimage");
    std::fs::create_dir_all(&dir).unwrap();
    // Deterministic pseudo-random contents, odd sizes to exercise padding.
    let mut x = 0x2545_f491u32;
    let mut bytes = |n: usize| -> Vec<u8> {
        (0..n)
            .map(|_| {
                x ^= x << 13;
                x ^= x >> 17;
                x ^= x << 5;
                x as u8
            })
            .collect()
    };
    let init = bytes(50_001);
    let boot = bytes(3_333);
    let (pi, pb) = (dir.join("init.bin"), dir.join("boot.bin"));
    std::fs::write(&pi, &init).unwrap();
    std::fs::write(&pb, &boot).unwrap();

    let Some(reference) = mkimage("rk3588", &[&pi], &dir.join("ref1.img")) else {
        eprintln!("mkimage not installed: skipped");
        return;
    };
    let ours = rk_idblock::build("rk3588", &init, None).unwrap();
    assert_eq!(ours, reference, "init only");

    let reference = mkimage("rk3588", &[&pi, &pb], &dir.join("ref2.img")).unwrap();
    let ours = rk_idblock::build("rk3588", &init, Some(&boot)).unwrap();
    assert_eq!(ours, reference, "init and boot");
}
