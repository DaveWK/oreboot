// SPDX-License-Identifier: GPL-2.0-only
//! `stage` + `continue` (chain-load): a download not followed by a flash is
//! handed to `Device::staged` on `continue`; a flash clears the staged
//! state; a refused image is a FAIL and the session goes on; the default
//! device ignores staged data. `boot` hands the download to `Device::boot`
//! and ends the session like `continue` when the device accepts it.

use fastboot::{Config, Device, Line, Transport};
use std::collections::VecDeque;

/// A scripted host: commands in, replies out; `continue` ends the session.
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
        assert_eq!(d.len(), out.len());
        out.copy_from_slice(&d);
        Ok(())
    }
    fn send(&mut self, data: &[u8]) -> Result<(), Gone> {
        self.replies.push(
            String::from_utf8_lossy(data)
                .trim_end_matches('\0')
                .to_string(),
        );
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

#[derive(Default)]
struct Dev {
    staged: Vec<Vec<u8>>,
    flashed: Vec<(String, usize)>,
    booted: Vec<Vec<u8>>,
    refuse: bool,
}

impl Device for Dev {
    fn getvar(&mut self, _: &str, _: &mut Line) -> bool {
        false
    }
    fn flash(&mut self, target: &str, len: usize, _: &mut [u8]) -> Result<(), &'static str> {
        self.flashed.push((target.into(), len));
        Ok(())
    }
    fn staged(&mut self, image: &[u8]) -> Result<(), &'static str> {
        if self.refuse {
            return Err("not a bt0 image");
        }
        self.staged.push(image.to_vec());
        Ok(())
    }
    fn boot(&mut self, image: &[u8]) -> Result<(), &'static str> {
        if image.is_empty() {
            return Err("download an image first");
        }
        if self.refuse {
            return Err("not a FIT");
        }
        self.booted.push(image.to_vec());
        Ok(())
    }
}

struct Plain;
impl Device for Plain {
    fn getvar(&mut self, _: &str, _: &mut Line) -> bool {
        false
    }
    fn flash(&mut self, _: &str, _: usize, _: &mut [u8]) -> Result<(), &'static str> {
        Ok(())
    }
}

fn host(script: &[&[u8]]) -> Host {
    Host {
        script: script.iter().map(|s| s.to_vec()).collect(),
        replies: Vec::new(),
        disconnects: 0,
    }
}

fn serve<D: Device>(h: &mut Host, d: &mut D) -> Result<(), Gone> {
    let mut buf = vec![0u8; 4096];
    fastboot::serve(h, d, &Config { unlocked: true }, &mut buf)
}

#[test]
fn a_download_then_continue_is_staged() {
    let img = [7u8; 96];
    let mut h = host(&[b"download:00000060", &img, b"continue"]);
    let mut d = Dev::default();
    assert_eq!(serve(&mut h, &mut d), Ok(()));
    assert_eq!(d.staged, vec![img.to_vec()]);
    assert!(d.flashed.is_empty());
    assert_eq!(h.replies, ["DATA00000060", "OKAY", "OKAY"]);
    assert_eq!(h.disconnects, 1);
}

#[test]
fn a_flashed_download_is_not_staged_and_a_new_download_is() {
    let a = [1u8; 64];
    let b = [2u8; 32];
    let mut h = host(&[b"download:00000040", &a, b"flash:boot0", b"continue"]);
    let mut d = Dev::default();
    assert_eq!(serve(&mut h, &mut d), Ok(()));
    assert!(d.staged.is_empty());
    assert_eq!(d.flashed, vec![("boot0".to_string(), 64)]);
    let mut h = host(&[
        b"download:00000040",
        &a,
        b"flash:boot0",
        b"download:00000020",
        &b,
        b"continue",
    ]);
    let mut d = Dev::default();
    assert_eq!(serve(&mut h, &mut d), Ok(()));
    assert_eq!(d.staged, vec![b.to_vec()]);
}

#[test]
fn a_refused_image_fails_the_continue_and_the_session_goes_on() {
    let img = [9u8; 32];
    let mut h = host(&[
        b"download:00000020",
        &img,
        b"continue",
        b"getvar:version",
        b"continue",
    ]);
    let mut d = Dev {
        refuse: true,
        ..Dev::default()
    };
    // The second continue is refused again (still staged), so the script
    // runs out: the host went away.
    assert_eq!(serve(&mut h, &mut d), Err(Gone));
    assert_eq!(
        h.replies,
        [
            "DATA00000020",
            "OKAY",
            "FAILnot a bt0 image",
            "OKAY0.4",
            "FAILnot a bt0 image"
        ]
    );
    assert_eq!(h.disconnects, 0);
    assert!(d.staged.is_empty());
}

#[test]
fn plain_continue_and_the_default_device_are_unchanged() {
    let mut h = host(&[b"continue"]);
    assert_eq!(serve(&mut h, &mut Dev::default()), Ok(()));
    assert_eq!(h.replies, ["OKAY"]);
    let img = [3u8; 32];
    let mut h = host(&[b"download:00000020", &img, b"continue"]);
    assert_eq!(serve(&mut h, &mut Plain), Ok(()));
    assert_eq!(h.replies, ["DATA00000020", "OKAY", "OKAY"]);
    assert_eq!(h.disconnects, 1);
}

#[test]
fn boot_hands_over_the_download_and_ends_the_session() {
    let img = [5u8; 48];
    let mut h = host(&[b"boot", b"download:00000030", &img, b"boot"]);
    let mut d = Dev::default();
    assert_eq!(serve(&mut h, &mut d), Ok(()));
    assert_eq!(d.booted, vec![img.to_vec()]);
    assert!(d.staged.is_empty());
    assert_eq!(
        h.replies,
        [
            "FAILdownload an image first",
            "DATA00000030",
            "OKAY",
            "OKAY"
        ]
    );
    assert_eq!(h.disconnects, 1);
}

#[test]
fn a_refused_boot_is_a_fail_and_the_session_goes_on() {
    let img = [5u8; 16];
    let mut h = host(&[b"download:00000010", &img, b"boot", b"getvar:version"]);
    let mut d = Dev {
        refuse: true,
        ..Dev::default()
    };
    assert_eq!(serve(&mut h, &mut d), Err(Gone));
    assert_eq!(
        h.replies,
        ["DATA00000010", "OKAY", "FAILnot a FIT", "OKAY0.4"]
    );
    assert_eq!(h.disconnects, 0);
    assert!(d.booted.is_empty());
}
