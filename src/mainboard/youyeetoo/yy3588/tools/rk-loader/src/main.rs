// SPDX-License-Identifier: GPL-2.0-only
//! rk-loader: the MaskROM USB loader for `rkdeveloptool db`.
//!
//!     rk-loader pack --chip rk3588 --471 DDR.bin --472 NEXT.bin --out LOADER.bin [--no-usbhead]
//!     rk-loader list LOADER.bin
//!     rk-loader verify LOADER.bin
//!
//! The header time comes from SOURCE_DATE_EPOCH (default 0) so the output
//! is reproducible. rkdeveloptool treats a file as a bare loader only when
//! its name contains ".bin" (RKImage.cpp), so `--out` must.

use rk_loader::{Head, Kind, Time, check_usbhead, parse, rk3588_usb_with};
use std::process::ExitCode;

fn usage() -> ExitCode {
    eprintln!(
        "usage: rk-loader pack --chip rk3588 --471 FILE --472 FILE --out FILE.bin [--no-usbhead]\n       rk-loader list FILE\n       rk-loader verify FILE"
    );
    ExitCode::from(2)
}

fn base(p: &str) -> String {
    let b = std::path::Path::new(p)
        .file_name()
        .map(|s| s.to_string_lossy().into_owned());
    let b = b.unwrap_or_else(|| p.to_string());
    b.trim_end_matches(".bin").to_string()
}

fn read(p: &str) -> Result<Vec<u8>, String> {
    std::fs::read(p).map_err(|e| format!("{p}: {e}"))
}

fn pack(args: &[String]) -> Result<(), String> {
    let (mut chip, mut ddr, mut next, mut out) = (None, None, None, None);
    let mut head = Head::UsbHead;
    let mut it = args.iter();
    while let Some(a) = it.next() {
        if a == "--no-usbhead" {
            head = Head::None;
            continue;
        }
        let v = it.next().cloned();
        match (a.as_str(), v) {
            ("--chip", Some(v)) => chip = Some(v),
            ("--471", Some(v)) => ddr = Some(v),
            ("--472", Some(v)) => next = Some(v),
            ("--out", Some(v)) => out = Some(v),
            _ => return Err(format!("bad argument {a}")),
        }
    }
    let (Some(chip), Some(ddr), Some(next), Some(out)) = (chip, ddr, next, out) else {
        return Err("pack needs --chip, --471, --472 and --out".into());
    };
    if chip != "rk3588" {
        return Err(format!("chip {chip}: only rk3588 is supported"));
    }
    if !out.contains(".bin") {
        return Err(
            "--out must contain \".bin\" (rkdeveloptool loads only *.bin as a loader)".into(),
        );
    }
    let epoch = match std::env::var("SOURCE_DATE_EPOCH") {
        Ok(s) => s
            .parse::<u64>()
            .map_err(|_| "SOURCE_DATE_EPOCH is not a number")?,
        Err(_) => 0,
    };
    let l = rk3588_usb_with(
        head,
        &read(&ddr)?,
        &base(&ddr),
        &read(&next)?,
        &base(&next),
        Time::from_unix(epoch),
    )?;
    let bytes = rk_loader::build(&l)?;
    let p = parse(&bytes)?;
    if head == Head::UsbHead {
        check_usbhead(&p)?;
    }
    std::fs::write(&out, &bytes).map_err(|e| format!("{out}: {e}"))?;
    println!("{out}: {} bytes", bytes.len());
    Ok(())
}

fn list(path: &str, check: bool) -> Result<(), String> {
    let l = parse(&read(path)?)?;
    let tag = String::from_utf8_lossy(&l.tag.to_le_bytes()).into_owned();
    let t = l.time;
    println!(
        "tag={tag:?} version={:#06x} merge={:#010x} time={:04}-{:02}-{:02}T{:02}:{:02}:{:02} chip={:#010x} rc4_disabled={} sign={:#04x} crc=ok",
        l.version,
        l.merge_version,
        t.year,
        t.month,
        t.day,
        t.hour,
        t.minute,
        t.second,
        l.chip,
        l.rc4_disabled,
        l.sign_flag
    );
    for e in &l.entries {
        let kind = match e.kind {
            Kind::E471 => "471",
            Kind::E472 => "472",
            Kind::Loader => "loader",
        };
        println!(
            "{kind:>6} {:<20} size={:#08x} delay_ms={}",
            e.name,
            e.data.len(),
            e.delay_ms
        );
    }
    if check {
        let imgs = check_usbhead(&l)?;
        println!("UsbHead idblock hashes ok: images {imgs:x?}");
    }
    Ok(())
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let r = match (args.first().map(String::as_str), args.get(1)) {
        (Some("pack"), _) => pack(&args[1..]),
        (Some("list"), Some(p)) => list(p, false),
        (Some("verify"), Some(p)) => list(p, true),
        _ => return usage(),
    };
    match r {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("rk-loader: {e}");
            ExitCode::FAILURE
        }
    }
}
