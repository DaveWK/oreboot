// SPDX-License-Identifier: GPL-2.0-only
//! rk-idblock: pack our own first stage into a Rockchip BootROM idblock.
//!
//!     rk-idblock pack --soc rk3588 --init bt0.bin [--boot next.bin] --out idblock.bin
//!     rk-idblock verify idblock.bin

use std::process::ExitCode;

fn usage() -> ExitCode {
    eprintln!(
        "usage: rk-idblock pack --soc SOC --init FILE [--boot FILE] --out FILE\n       rk-idblock verify FILE"
    );
    ExitCode::from(2)
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    match args.first().map(String::as_str) {
        Some("pack") => {
            let mut soc = None;
            let mut init = None;
            let mut boot = None;
            let mut out = None;
            let mut it = args[1..].iter();
            while let Some(a) = it.next() {
                let v = it.next();
                match (a.as_str(), v) {
                    ("--soc", Some(v)) => soc = Some(v.clone()),
                    ("--init", Some(v)) => init = Some(v.clone()),
                    ("--boot", Some(v)) => boot = Some(v.clone()),
                    ("--out", Some(v)) => out = Some(v.clone()),
                    _ => return usage(),
                }
            }
            let (Some(soc), Some(init), Some(out)) = (soc, init, out) else {
                return usage();
            };
            let read = |p: &str| std::fs::read(p).map_err(|e| format!("{p}: {e}"));
            let result = read(&init).and_then(|i| {
                let b = boot.as_deref().map(read).transpose()?;
                rk_idblock::build(&soc, &i, b.as_deref()).map_err(|e| e.to_string())
            });
            match result
                .and_then(|blob| std::fs::write(&out, blob).map_err(|e| format!("{out}: {e}")))
            {
                Ok(()) => ExitCode::SUCCESS,
                Err(e) => {
                    eprintln!("rk-idblock: {e}");
                    ExitCode::FAILURE
                }
            }
        }
        Some("verify") if args.len() == 2 => match std::fs::read(&args[1])
            .map_err(|e| e.to_string())
            .and_then(|b| rk_idblock::verify(&b))
        {
            Ok([(io, il), (bo, bl)]) => {
                println!("ok: init {il:#x} bytes at {io:#x}, boot {bl:#x} bytes at {bo:#x}");
                ExitCode::SUCCESS
            }
            Err(e) => {
                eprintln!("rk-idblock: {e}");
                ExitCode::FAILURE
            }
        },
        _ => usage(),
    }
}
