// SPDX-License-Identifier: GPL-2.0-only
//! fastboot (the Android flashing protocol) for a first stage that stages
//! images from a workstation: `fastboot getvar`, `download`, `flash` (raw or
//! Android sparse images, see [`flash_image`]) and `continue`.
//!
//! The protocol loop ([`serve`]) runs over two board seams:
//! - [`Transport`]: the SoC's USB device controller (K1 DWC2-like udc, A733
//!   musb/FEL, ...), bulk packets in and out, plus the configured/reset and
//!   quiescence (disconnect) states;
//! - [`Device`]: the board's targets (names, capacities, block writes),
//!   product and serial strings, and whether reset is supported.
//!
//! From ore-edk-boot-opi c91bbc4e40 `oreboot/src/mainboard/spacemit/k1x/bt0/src/fastboot.rs`:
//! the dispatcher, reply lines and download/continue/reboot semantics are
//! the K1's, with its USB, MMC and fixed DRAM buffer replaced by the seams
//! (equivalence-tested against the K1 code on its host model). Only an
//! acknowledged, quiesced `continue` returns `Ok`; a failed download or a
//! failed continue acknowledgement is terminal.
#![no_std]
#[cfg(test)]
extern crate std;

use core::fmt::Write;

pub const SECTOR: usize = 512;
/// First word of an Android sparse image.
pub const SPARSE_MAGIC: u32 = 0xed26_ff3a;
/// fastboot protocol version reported by `getvar:version`.
pub const PROTOCOL_VERSION: &str = "0.4";

/// A response line: OKAY, FAIL, INFO or DATA plus up to 60 bytes.
pub struct Line {
    buf: [u8; 64],
    len: usize,
}
impl Line {
    pub fn new(kind: &str) -> Self {
        let mut l = Self {
            buf: [0; 64],
            len: 0,
        };
        let _ = l.write_str(kind);
        l
    }
    pub fn as_bytes(&self) -> &[u8] {
        &self.buf[..self.len]
    }
}
impl Write for Line {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let n = s.len().min(self.buf.len() - self.len);
        self.buf[self.len..self.len + n].copy_from_slice(&s.as_bytes()[..n]);
        self.len += n;
        Ok(())
    }
}

/// One host command.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Command<'a> {
    Getvar(&'a str),
    /// `download:<hex size>`; None if the size is not hex.
    Download(Option<usize>),
    Flash(&'a str),
    Erase(&'a str),
    Continue,
    /// `reboot` or `reboot-*`.
    Reboot(&'a str),
    Unknown(&'a str),
}

/// Parse one command (`c` is the received packet as text).
pub fn parse(c: &str) -> Command<'_> {
    if let Some(var) = c.strip_prefix("getvar:") {
        Command::Getvar(var)
    } else if let Some(size) = c.strip_prefix("download:") {
        Command::Download(hex(size))
    } else if let Some(name) = c.strip_prefix("flash:") {
        Command::Flash(name)
    } else if let Some(name) = c.strip_prefix("erase:") {
        Command::Erase(name)
    } else if c == "continue" {
        Command::Continue
    } else if c == "reboot" || c.starts_with("reboot-") {
        Command::Reboot(c)
    } else {
        Command::Unknown(c)
    }
}

/// A hex number with an optional `0x` prefix (the K1 parser).
pub fn hex(s: &str) -> Option<usize> {
    usize::from_str_radix(s.trim_start_matches("0x"), 16).ok()
}

/// The USB device controller, as fastboot needs it.
pub trait Transport {
    type Error: Copy;
    /// Wait until the host configured the device (after a bus reset too).
    fn wait_configured(&mut self) -> Result<(), Self::Error>;
    /// One command packet into `out`; its length.
    fn recv(&mut self, out: &mut [u8]) -> Result<usize, Self::Error>;
    /// Exactly `out.len()` bytes of download data.
    fn recv_exact(&mut self, out: &mut [u8]) -> Result<(), Self::Error>;
    fn send(&mut self, data: &[u8]) -> Result<(), Self::Error>;
    /// Stop the controller and wait until it is quiescent (before handing
    /// over to the next stage).
    fn disconnect(&mut self) -> Result<(), Self::Error>;
    /// The error is a bus reset: wait for configuration again and continue.
    fn is_reset(e: &Self::Error) -> bool;
    /// Log a received command or a sent non-INFO reply (default: nothing).
    fn log(&mut self, _what: &str, _line: &[u8]) {}
}

/// The board's side.
pub trait Device {
    /// Answer a variable the protocol layer does not know (product, serialno,
    /// partition-type:X, partition-size:X, ...): write it to `out` and return
    /// true, or false for "unknown variable".
    fn getvar(&mut self, name: &str, out: &mut Line) -> bool;
    /// Write `image` (raw or sparse; [`flash_image`] does both) to `target`.
    /// `image` is the downloaded bytes; `padded` the same buffer extended to
    /// a whole sector, which the device may zero-fill and write.
    fn flash(
        &mut self,
        target: &str,
        image_len: usize,
        padded: &mut [u8],
    ) -> Result<(), &'static str>;
    /// `erase:<target>`; the default refuses it like an unknown command.
    fn erase(&mut self, _target: &str) -> Result<(), &'static str> {
        Err("unknown command")
    }
    /// `reboot*`: reset the board (never returns) or refuse. The default
    /// refuses: no proven hardware reset path.
    fn reboot(&mut self) -> Result<(), &'static str> {
        Err("reset unsupported; use continue to boot")
    }
}

/// Protocol-level settings.
#[derive(Clone, Copy, Debug)]
pub struct Config {
    /// Reported by `getvar:unlocked` (`is-userspace` and `secure` are "no").
    pub unlocked: bool,
}

fn reply_status<T: Transport>(
    t: &mut T,
    kind: &str,
    msg: core::fmt::Arguments,
) -> Result<(), T::Error> {
    let mut l = Line::new(kind);
    let _ = l.write_fmt(msg);
    if kind != "INFO" {
        t.log(">", l.as_bytes());
    }
    t.send(l.as_bytes())
}

fn reply<T: Transport>(t: &mut T, kind: &str, msg: core::fmt::Arguments) {
    // Non-handoff replies keep the simple interface; a fatal transport fault
    // shows up at the next wait_configured/recv.
    let _ = reply_status(t, kind, msg);
}

fn getvar<T: Transport, D: Device>(t: &mut T, d: &mut D, cfg: &Config, max: usize, var: &str) {
    match var {
        "version" => reply(t, "OKAY", format_args!("{PROTOCOL_VERSION}")),
        "max-download-size" => reply(t, "OKAY", format_args!("0x{:08x}", max)),
        "is-userspace" | "secure" => reply(t, "OKAY", format_args!("no")),
        "unlocked" => reply(
            t,
            "OKAY",
            format_args!("{}", if cfg.unlocked { "yes" } else { "no" }),
        ),
        _ if var.starts_with("has-slot:") || var.starts_with("is-logical:") => {
            reply(t, "OKAY", format_args!("no"))
        }
        _ => {
            let mut l = Line::new("OKAY");
            if d.getvar(var, &mut l) {
                t.log(">", l.as_bytes());
                let _ = t.send(l.as_bytes());
            } else {
                reply(t, "FAIL", format_args!("unknown variable"))
            }
        }
    }
}

/// Serve fastboot until an acknowledged, quiesced `continue` (Ok) or a
/// terminal transport fault (Err). Downloads land in `buf`; the advertised
/// max-download-size is `buf.len()` rounded down to a whole sector, so the
/// device can always pad the last sector in place.
pub fn serve<T: Transport, D: Device>(
    t: &mut T,
    d: &mut D,
    cfg: &Config,
    buf: &mut [u8],
) -> Result<(), T::Error> {
    let max = buf.len() / SECTOR * SECTOR;
    let mut downloaded = 0;
    let mut cmd = [0u8; 64];
    loop {
        t.wait_configured()?;
        let n = match t.recv(&mut cmd) {
            Ok(n) => n,
            Err(e) if T::is_reset(&e) => continue,
            Err(e) => return Err(e),
        };
        let c = core::str::from_utf8(&cmd[..n]).unwrap_or("");
        t.log("<", c.as_bytes());
        match parse(c) {
            Command::Getvar(var) => getvar(t, d, cfg, max, var),
            Command::Download(size) => match size {
                Some(len) if len > 0 && len <= max => {
                    // A failed download is terminal (return below), so the
                    // previous image can never be flashed after it.
                    reply_status(t, "DATA", format_args!("{:08x}", len))?;
                    t.recv_exact(&mut buf[..len])?;
                    downloaded = len;
                    reply_status(t, "OKAY", format_args!(""))?;
                }
                _ => reply(t, "FAIL", format_args!("bad download size")),
            },
            Command::Flash(name) => {
                let r = if downloaded == 0 {
                    Err("download an image first")
                } else {
                    let padded = downloaded.next_multiple_of(SECTOR);
                    d.flash(name, downloaded, &mut buf[..padded])
                };
                match r {
                    Ok(()) => reply(t, "OKAY", format_args!("")),
                    Err(e) => reply(t, "FAIL", format_args!("{e}")),
                }
            }
            Command::Erase(name) => match d.erase(name) {
                Ok(()) => reply(t, "OKAY", format_args!("")),
                Err(e) => reply(t, "FAIL", format_args!("{e}")),
            },
            Command::Continue => {
                let ack = reply_status(t, "OKAY", format_args!(""));
                let stopped = t.disconnect();
                ack?;
                stopped?;
                return Ok(());
            }
            Command::Reboot(_) => match d.reboot() {
                Ok(()) => reply_status(t, "OKAY", format_args!(""))?,
                Err(e) => reply_status(t, "FAIL", format_args!("{e}"))?,
            },
            Command::Unknown(_) => reply(t, "FAIL", format_args!("unknown command")),
        }
    }
}

/// Write a downloaded image to a block device of `capacity` sectors: an
/// Android sparse image through [`sparse::write`] (validated completely
/// before the first write), otherwise raw from sector 0, with the tail of a
/// partial last sector zeroed. `padded` is the image (`image_len` bytes)
/// extended to a whole sector, as [`serve`] passes it to [`Device::flash`].
pub fn flash_image<W: sparse::Writer>(
    padded: &mut [u8],
    image_len: usize,
    capacity: u64,
    w: &mut W,
) -> Result<(), &'static str> {
    if image_len == 0 {
        return Err("download an image first");
    }
    if image_len >= 4
        && u32::from_le_bytes([padded[0], padded[1], padded[2], padded[3]]) == SPARSE_MAGIC
    {
        return sparse::write(&padded[..image_len], capacity, w);
    }
    if image_len.div_ceil(SECTOR) as u64 > capacity {
        return Err("image larger than the partition");
    }
    let end = image_len.next_multiple_of(SECTOR);
    padded[image_len..end].fill(0);
    w.raw(0, &padded[..end])
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::{collections::VecDeque, vec, vec::Vec};

    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    enum E {
        Reset,
        Timeout,
        Flush,
    }
    #[derive(Default)]
    struct Script {
        commands: VecDeque<&'static str>,
        replies: Vec<Vec<u8>>,
        received: usize,
        disconnected: usize,
        ack_fail: bool,
        flush_fail: bool,
        download_fail: bool,
        resets: usize,
    }
    impl Transport for Script {
        type Error = E;
        fn wait_configured(&mut self) -> Result<(), E> {
            Ok(())
        }
        fn recv(&mut self, out: &mut [u8]) -> Result<usize, E> {
            if self.resets > 0 {
                self.resets -= 1;
                return Err(E::Reset);
            }
            self.received += 1;
            let c = self
                .commands
                .pop_front()
                .expect("unexpected command request");
            out[..c.len()].copy_from_slice(c.as_bytes());
            Ok(c.len())
        }
        fn recv_exact(&mut self, out: &mut [u8]) -> Result<(), E> {
            if self.download_fail {
                return Err(E::Timeout);
            }
            for (i, b) in out.iter_mut().enumerate() {
                *b = i as u8;
            }
            Ok(())
        }
        fn send(&mut self, data: &[u8]) -> Result<(), E> {
            self.replies.push(data.to_vec());
            if data == b"OKAY" && self.ack_fail {
                Err(E::Timeout)
            } else {
                Ok(())
            }
        }
        fn disconnect(&mut self) -> Result<(), E> {
            self.disconnected += 1;
            if self.flush_fail {
                Err(E::Flush)
            } else {
                Ok(())
            }
        }
        fn is_reset(e: &E) -> bool {
            *e == E::Reset
        }
    }
    struct Disk {
        sectors: Vec<u8>,
    }
    impl sparse::Writer for Disk {
        fn raw(&mut self, lba: u32, data: &[u8]) -> Result<(), &'static str> {
            let o = lba as usize * SECTOR;
            self.sectors[o..o + data.len()].copy_from_slice(data);
            Ok(())
        }
        fn fill(&mut self, lba: u32, sectors: u64, value: u32) -> Result<(), &'static str> {
            for k in 0..sectors as usize * SECTOR / 4 {
                let o = lba as usize * SECTOR + 4 * k;
                self.sectors[o..o + 4].copy_from_slice(&value.to_le_bytes());
            }
            Ok(())
        }
    }
    struct Board {
        disk: Disk,
    }
    impl Device for Board {
        fn getvar(&mut self, name: &str, out: &mut Line) -> bool {
            match name {
                "product" => write!(out, "host-model").is_ok(),
                "partition-type:disk" => write!(out, "raw").is_ok(),
                _ => false,
            }
        }
        fn flash(
            &mut self,
            target: &str,
            len: usize,
            padded: &mut [u8],
        ) -> Result<(), &'static str> {
            if target != "disk" {
                return Err("unknown target");
            }
            let cap = (self.disk.sectors.len() / SECTOR) as u64;
            flash_image(padded, len, cap, &mut self.disk)
        }
    }
    fn run(s: &mut Script) -> Result<(), E> {
        let mut b = Board {
            disk: Disk {
                sectors: vec![0xaa; 4 * SECTOR],
            },
        };
        let mut buf = vec![0u8; 2048];
        serve(s, &mut b, &Config { unlocked: true }, &mut buf)
    }
    fn script(c: &[&'static str]) -> Script {
        Script {
            commands: c.iter().copied().collect(),
            ..Default::default()
        }
    }
    #[test]
    fn reboot_does_not_alias_continue() {
        for cmd in [
            "reboot",
            "reboot-bootloader",
            "reboot-fastboot",
            "reboot-recovery",
            "rebootx",
        ] {
            let mut s = script(&[cmd, "continue"]);
            assert!(run(&mut s).is_ok());
            assert_eq!(s.received, 2);
            assert!(s.replies[0].starts_with(b"FAIL"));
            assert_eq!(s.replies[1], b"OKAY");
            assert_eq!(s.disconnected, 1);
        }
    }
    #[test]
    fn continue_needs_ack_and_quiescence() {
        let mut s = script(&["continue"]);
        assert!(run(&mut s).is_ok());
        let mut s = script(&["continue"]);
        s.ack_fail = true;
        assert_eq!(run(&mut s), Err(E::Timeout));
        assert_eq!(s.disconnected, 1);
        let mut s = script(&["continue"]);
        s.flush_fail = true;
        assert_eq!(run(&mut s), Err(E::Flush));
    }
    #[test]
    fn download_failure_is_terminal_and_bus_reset_is_not() {
        let mut s = script(&["download:00000020", "continue"]);
        s.download_fail = true;
        assert_eq!(run(&mut s), Err(E::Timeout));
        assert_eq!(s.replies, vec![b"DATA00000020".to_vec()]);
        let mut s = script(&["continue"]);
        s.resets = 2;
        assert!(run(&mut s).is_ok());
    }
    #[test]
    fn getvar_and_unknown_command() {
        let mut s = script(&[
            "getvar:version",
            "getvar:product",
            "getvar:unlocked",
            "getvar:max-download-size",
            "getvar:has-slot:disk",
            "getvar:nope",
            "anything",
            "erase:disk",
            "continue",
        ]);
        run(&mut s).unwrap();
        assert_eq!(
            s.replies,
            vec![
                b"OKAY0.4".to_vec(),
                b"OKAYhost-model".to_vec(),
                b"OKAYyes".to_vec(),
                b"OKAY0x00000800".to_vec(),
                b"OKAYno".to_vec(),
                b"FAILunknown variable".to_vec(),
                b"FAILunknown command".to_vec(),
                b"FAILunknown command".to_vec(),
                b"OKAY".to_vec(),
            ]
        );
    }
    #[test]
    fn flash_raw_pads_the_last_sector_and_checks_size() {
        let mut s = script(&[
            "flash:disk",
            "download:0x300",
            "flash:disk",
            "flash:sd",
            "download:801",
            "continue",
        ]);
        run(&mut s).unwrap();
        assert_eq!(s.replies[0], b"FAILdownload an image first");
        assert_eq!(s.replies[1], b"DATA00000300");
        assert_eq!(s.replies[3], b"OKAY");
        assert_eq!(s.replies[4], b"FAILunknown target");
        assert_eq!(s.replies[5], b"FAILbad download size");
        let mut b = Board {
            disk: Disk {
                sectors: vec![0xaa; 2 * SECTOR],
            },
        };
        let mut img = vec![1u8; 3 * SECTOR];
        assert_eq!(
            flash_image(&mut img, 3 * SECTOR, 2, &mut b.disk),
            Err("image larger than the partition")
        );
        let mut img = vec![1u8; 2 * SECTOR];
        flash_image(&mut img, SECTOR + 3, 2, &mut b.disk).unwrap();
        assert_eq!(&b.disk.sectors[SECTOR..SECTOR + 4], &[1, 1, 1, 0]);
        assert_eq!(b.disk.sectors[2 * SECTOR - 1], 0);
    }
}
