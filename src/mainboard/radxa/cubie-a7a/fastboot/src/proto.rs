// SPDX-License-Identifier: GPL-2.0-only

//! The fastboot protocol: command parsing, replies and the serve loop.
//!
//! The host sends one command per bulk OUT packet (printable ASCII, at most
//! 64 bytes); the device answers with `OKAY`, `FAIL`, `INFO` (progress, any
//! number before the final answer) or `DATA<8 hex digits>` (ready for that
//! many download bytes), each followed by up to 60 bytes of text.
//!
//! [`serve`] runs until the host asks for something that leaves fastboot
//! ([`Exit`]): `boot` or `continue` (the board's [`Device::Boot`] plan),
//! `reboot` or `reboot-bootloader`. Every exit is acknowledged with `OKAY`
//! and the controller is disconnected (quiescent) before [`serve`] returns
//! it; a failed acknowledgement or disconnect is an error instead, so a
//! hand-over never follows a reply the host did not get. A bus reset,
//! disconnect, or failed download ends the session. Everything a board
//! refuses is a `FAIL` and the loop goes on.
use core::fmt::{self, Write};

/// Reported by `getvar:version`.
pub const PROTOCOL_VERSION: &str = "0.4";
/// The longest command the protocol allows.
pub const MAX_COMMAND: usize = 64;
/// The command receive buffer: one high-speed bulk packet.
pub const PACKET: usize = 512;

/// A reply: `OKAY`, `FAIL`, `INFO` or `DATA` plus up to 60 bytes; longer
/// text is cut off.
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
    fn write_str(&mut self, s: &str) -> fmt::Result {
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
    /// `download:<hex size>`; None if the size is not 1 to 8 hex digits.
    Download(Option<u32>),
    /// `fetch:<part>[:<hex offset>[:<hex size>]]` (the stock tool always
    /// sends both, `0x%08lx`); None for a field that is present but not
    /// 1 to 16 hex digits.
    Fetch {
        part: &'a str,
        offset: Option<Option<u64>>,
        size: Option<Option<u64>>,
    },
    Flash(&'a str),
    Erase(&'a str),
    Boot,
    Continue,
    Reboot,
    RebootBootloader,
    /// `reboot-recovery`, `reboot-fastboot`, ...: modes this board lacks.
    RebootOther(&'a str),
    /// `oem <args>` (args may be empty).
    Oem(&'a str),
    Unknown(&'a str),
}

/// Why a packet is not a command at all.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Malformed {
    Empty,
    TooLong,
    NotText,
}

impl Malformed {
    pub fn message(self) -> &'static str {
        match self {
            Malformed::Empty => "empty command",
            Malformed::TooLong => "command longer than 64 bytes",
            Malformed::NotText => "command is not printable ASCII",
        }
    }
}

/// Parse one command packet.
pub fn parse(packet: &[u8]) -> Result<Command<'_>, Malformed> {
    if packet.is_empty() {
        return Err(Malformed::Empty);
    }
    if packet.len() > MAX_COMMAND {
        return Err(Malformed::TooLong);
    }
    if !packet.iter().all(|b| (0x20..0x7f).contains(b)) {
        return Err(Malformed::NotText);
    }
    // Printable ASCII is UTF-8.
    let c = core::str::from_utf8(packet).map_err(|_| Malformed::NotText)?;
    Ok(if let Some(v) = c.strip_prefix("getvar:") {
        Command::Getvar(v)
    } else if let Some(size) = c.strip_prefix("download:") {
        Command::Download(hex_u32(size))
    } else if let Some(rest) = c.strip_prefix("fetch:") {
        let mut f = rest.split(':');
        let part = f.next().unwrap_or("");
        let offset = f.next().map(hex_u64);
        let size = f.next().map(hex_u64);
        if f.next().is_some() {
            Command::Unknown(c)
        } else {
            Command::Fetch { part, offset, size }
        }
    } else if let Some(p) = c.strip_prefix("flash:") {
        Command::Flash(p)
    } else if let Some(p) = c.strip_prefix("erase:") {
        Command::Erase(p)
    } else if c == "boot" {
        Command::Boot
    } else if c == "continue" {
        Command::Continue
    } else if c == "reboot" {
        Command::Reboot
    } else if c == "reboot-bootloader" {
        Command::RebootBootloader
    } else if c.starts_with("reboot-") {
        Command::RebootOther(c)
    } else if c == "oem" {
        Command::Oem("")
    } else if let Some(args) = c.strip_prefix("oem ") {
        Command::Oem(args)
    } else {
        Command::Unknown(c)
    })
}

/// 1 to 16 hex digits with an optional `0x`; no sign, space or prefix.
pub fn hex_u64(s: &str) -> Option<u64> {
    let d = s
        .strip_prefix("0x")
        .or_else(|| s.strip_prefix("0X"))
        .unwrap_or(s);
    if d.is_empty() || d.len() > 16 {
        return None;
    }
    let mut v = 0u64;
    for b in d.bytes() {
        let n = match b {
            b'0'..=b'9' => b - b'0',
            b'a'..=b'f' => b - b'a' + 10,
            b'A'..=b'F' => b - b'A' + 10,
            _ => return None,
        };
        v = (v << 4) | n as u64;
    }
    Some(v)
}

/// 1 to 8 hex digits with an optional `0x`; no sign, space or other
/// prefix (`usize::from_str_radix` would take a leading `+`).
pub fn hex_u32(s: &str) -> Option<u32> {
    let d = s
        .strip_prefix("0x")
        .or_else(|| s.strip_prefix("0X"))
        .unwrap_or(s);
    if d.is_empty() || d.len() > 8 {
        return None;
    }
    let mut v = 0u32;
    for b in d.bytes() {
        let n = match b {
            b'0'..=b'9' => b - b'0',
            b'a'..=b'f' => b - b'a' + 10,
            b'A'..=b'F' => b - b'A' + 10,
            _ => return None,
        };
        v = (v << 4) | n as u32;
    }
    Some(v)
}

/// How [`serve`] ended: always after an acknowledged command and a
/// quiescent controller.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Exit<B> {
    /// `boot` or `continue`: hand over as the board planned.
    Boot(B),
    /// `reboot-bootloader` (or the board's equivalent oem command).
    Bootloader,
    /// `reboot`.
    Reset,
}

/// The USB device controller, as fastboot needs it.
pub trait Transport {
    type Error: Copy + fmt::Debug;
    /// Wait for the initial configuration. After that, reset or disconnect
    /// is terminal and must be reported by every subsequent transfer.
    fn wait_configured(&mut self) -> Result<(), Self::Error>;
    /// One command packet into `out` (at least [`PACKET`] bytes); its length.
    fn recv(&mut self, out: &mut [u8]) -> Result<usize, Self::Error>;
    /// Exactly `out.len()` bytes of download data, no more.
    fn recv_exact(&mut self, out: &mut [u8]) -> Result<(), Self::Error>;
    fn send(&mut self, data: &[u8]) -> Result<(), Self::Error>;
    /// The data phase of an upload (`fetch`): exactly `data`, as full
    /// packets and a short last one; no zero-length packet (the host reads
    /// exactly the size `DATA` announced).
    fn send_data(&mut self, data: &[u8]) -> Result<(), Self::Error>;
    /// Wait until the host took the last reply, then leave the bus.
    fn disconnect(&mut self) -> Result<(), Self::Error>;
    /// A received command ("<") or a sent final reply (">").
    fn log(&mut self, _dir: &str, _line: &[u8]) {}
}

/// Progress text for the host (`INFO` lines) while a command runs.
pub trait Info {
    fn info(&mut self, msg: fmt::Arguments);
}

struct Sink<'a, T: Transport>(&'a mut T);

impl<T: Transport> Info for Sink<'_, T> {
    fn info(&mut self, msg: fmt::Arguments) {
        let mut l = Line::new("INFO");
        let _ = l.write_fmt(msg);
        // A session-ending fault must also be reported by the final reply.
        let _ = self.0.send(l.as_bytes());
    }
}

/// The board.
pub trait Device {
    /// What `boot` and `continue` prepared, for the hand-over after the
    /// acknowledgement.
    type Boot;
    /// Append the value of `name` to `out` and return true, or false for an
    /// unknown variable. The protocol layer answers `version`,
    /// `max-download-size`, `is-userspace`, `has-slot:*`, `is-logical:*` and
    /// `all` itself.
    fn getvar(&mut self, name: &str, out: &mut Line) -> bool;
    /// The board's variables `getvar:all` lists.
    fn getvar_names(&self) -> &'static [&'static str];
    /// Every command, before it runs (malformed packets excepted).
    fn command(&mut self, _c: &Command) {}
    /// `flash:<part>` of the downloaded `image` (never empty).
    fn flash(&mut self, part: &str, image: &[u8], info: &mut dyn Info) -> Result<(), &'static str>;
    fn erase(&mut self, part: &str, info: &mut dyn Info) -> Result<(), &'static str>;
    /// `boot`: check the download (None if there is none) and plan the
    /// hand-over; an error is a `FAIL` and fastboot goes on.
    fn boot(
        &mut self,
        image: Option<&[u8]>,
        info: &mut dyn Info,
    ) -> Result<Self::Boot, &'static str>;
    /// `continue`: boot what is staged, by the board's rules.
    fn continue_boot(
        &mut self,
        image: Option<&[u8]>,
        info: &mut dyn Info,
    ) -> Result<Self::Boot, &'static str>;
    /// May `reboot` (false) or `reboot-bootloader` (true) go ahead?
    fn reboot(&mut self, bootloader: bool) -> Result<(), &'static str>;
    /// `fetch`: read `size` bytes (None: to the end of the partition) at
    /// `offset` of `part` into `buf` (the download buffer, which the read
    /// overwrites) and return where they are in it. Every read error is a
    /// `FAIL` before any data moves.
    fn fetch(
        &mut self,
        part: &str,
        offset: u64,
        size: Option<u64>,
        buf: &mut [u8],
        info: &mut dyn Info,
    ) -> Result<core::ops::Range<usize>, &'static str>;
    /// `oem <args>`: Ok(None) answers `OKAY` and goes on; Ok(Some(exit))
    /// leaves fastboot like the standard commands.
    fn oem(
        &mut self,
        args: &str,
        image: Option<&[u8]>,
        info: &mut dyn Info,
    ) -> Result<Option<Exit<Self::Boot>>, &'static str>;
}

fn send_line<T: Transport>(t: &mut T, kind: &str, msg: fmt::Arguments) -> Result<(), T::Error> {
    let mut l = Line::new(kind);
    let _ = l.write_fmt(msg);
    if kind != "INFO" {
        t.log(">", l.as_bytes());
    }
    t.send(l.as_bytes())
}

/// Acknowledge an exit and quiesce; either failing is an error.
fn finish<T: Transport, B>(t: &mut T, exit: Exit<B>) -> Result<Exit<B>, T::Error> {
    let ack = send_line(t, "OKAY", format_args!(""));
    let stopped = t.disconnect();
    ack?;
    stopped?;
    Ok(exit)
}

/// Variables the protocol layer answers.
const PROTOCOL_VARS: [&str; 4] = [
    "version",
    "max-download-size",
    "max-fetch-size",
    "is-userspace",
];

/// `fetch:download`: the last download, sent back from the buffer as it
/// is (a lossless round trip for throughput tests; `getvar
/// partition-size:download` is its length).
pub const DOWNLOAD_TARGET: &str = "download";

/// The largest `fetch` chunk: one read into the download buffer, then one
/// data phase. Small enough that the slowest medium (the SPI NOR at
/// ~0.7 MB/s) answers within a couple of seconds; the stock tool loops
/// over the partition in chunks of this size.
pub const MAX_FETCH: usize = 0x10_0000;

fn protocol_var(name: &str, max: usize, staged: usize, out: &mut Line) -> bool {
    match name {
        "partition-size:download" if staged > 0 => write!(out, "0x{staged:x}").is_ok(),
        "version" => write!(out, "{PROTOCOL_VERSION}").is_ok(),
        "max-download-size" => write!(out, "0x{max:08x}").is_ok(),
        "max-fetch-size" => write!(out, "0x{:08x}", MAX_FETCH.min(max)).is_ok(),
        "is-userspace" => write!(out, "no").is_ok(),
        _ if name.starts_with("has-slot:") || name.starts_with("is-logical:") => {
            write!(out, "no").is_ok()
        }
        _ => false,
    }
}

fn getvar<T: Transport, D: Device>(
    t: &mut T,
    d: &mut D,
    max: usize,
    staged: usize,
    name: &str,
) -> Result<(), T::Error> {
    if name == "all" {
        for n in PROTOCOL_VARS.iter().chain(d.getvar_names()) {
            let mut l = Line::new("INFO");
            let _ = write!(l, "{n}:");
            if protocol_var(n, max, staged, &mut l) || d.getvar(n, &mut l) {
                t.send(l.as_bytes())?;
            }
        }
        return send_line(t, "OKAY", format_args!(""));
    }
    let mut l = Line::new("OKAY");
    if protocol_var(name, max, staged, &mut l) || d.getvar(name, &mut l) {
        t.log(">", l.as_bytes());
        t.send(l.as_bytes())
    } else {
        send_line(t, "FAIL", format_args!("unknown variable"))
    }
}

/// Serve fastboot until an acknowledged exit (Ok) or a terminal transport
/// fault (Err). Downloads land in `buf`; max-download-size is its length
/// (at most 4 GiB - 1). Each `download` command drops the previous image,
/// even one it refuses. The download is only staged for this invocation;
/// the caller must also discard board-local session state before serving again.
pub fn serve<T: Transport, D: Device>(
    t: &mut T,
    d: &mut D,
    buf: &mut [u8],
) -> Result<Exit<D::Boot>, T::Error> {
    let max = buf.len().min(u32::MAX as usize);
    let mut downloaded = 0usize;
    let mut packet = [0u8; PACKET];
    t.wait_configured()?;
    loop {
        let n = t.recv(&mut packet)?.min(PACKET);
        // A zero-length packet ends a host transfer that filled whole
        // packets (macOS and Windows hosts after such a download): no command.
        if n == 0 {
            continue;
        }
        t.log("<", &packet[..n]);
        let cmd = match parse(&packet[..n]) {
            Ok(c) => c,
            Err(m) => {
                send_line(t, "FAIL", format_args!("{}", m.message()))?;
                continue;
            }
        };
        d.command(&cmd);
        match cmd {
            Command::Getvar(name) => getvar(t, d, max, downloaded, name)?,
            Command::Download(size) => {
                downloaded = 0;
                match size {
                    Some(len) if len > 0 && len as usize <= max => {
                        let len = len as usize;
                        send_line(t, "DATA", format_args!("{len:08x}"))?;
                        t.recv_exact(&mut buf[..len])?;
                        downloaded = len;
                        send_line(t, "OKAY", format_args!(""))?;
                    }
                    Some(len) if len as usize > max => send_line(
                        t,
                        "FAIL",
                        format_args!("download larger than max-download-size 0x{max:x}"),
                    )?,
                    _ => send_line(t, "FAIL", format_args!("bad download size"))?,
                }
            }
            Command::Flash(part) => {
                let r = if downloaded == 0 {
                    Err("download an image first")
                } else {
                    d.flash(part, &buf[..downloaded], &mut Sink(t))
                };
                match r {
                    Ok(()) => send_line(t, "OKAY", format_args!(""))?,
                    Err(e) => send_line(t, "FAIL", format_args!("{e}"))?,
                }
            }
            Command::Erase(part) => match d.erase(part, &mut Sink(t)) {
                Ok(()) => send_line(t, "OKAY", format_args!(""))?,
                Err(e) => send_line(t, "FAIL", format_args!("{e}"))?,
            },
            Command::Boot | Command::Continue => {
                let image = (downloaded > 0).then(|| &buf[..downloaded]);
                let r = if cmd == Command::Boot {
                    d.boot(image, &mut Sink(t))
                } else {
                    d.continue_boot(image, &mut Sink(t))
                };
                match r {
                    Ok(plan) => return finish(t, Exit::Boot(plan)),
                    Err(e) => send_line(t, "FAIL", format_args!("{e}"))?,
                }
            }
            Command::Reboot | Command::RebootBootloader => {
                let bootloader = cmd == Command::RebootBootloader;
                match d.reboot(bootloader) {
                    Ok(()) if bootloader => return finish(t, Exit::Bootloader),
                    Ok(()) => return finish(t, Exit::Reset),
                    Err(e) => send_line(t, "FAIL", format_args!("{e}"))?,
                }
            }
            Command::RebootOther(_) => send_line(
                t,
                "FAIL",
                format_args!("no such mode; use reboot or reboot-bootloader"),
            )?,
            Command::Oem(args) => {
                let image = (downloaded > 0).then(|| &buf[..downloaded]);
                match d.oem(args, image, &mut Sink(t)) {
                    Ok(Some(exit)) => return finish(t, exit),
                    Ok(None) => send_line(t, "OKAY", format_args!(""))?,
                    Err(e) => send_line(t, "FAIL", format_args!("{e}"))?,
                }
            }
            Command::Fetch { part, offset, size } => {
                let staged = downloaded;
                // The read lands in the download buffer (except the
                // download itself, sent back as it is).
                if part != DOWNLOAD_TARGET {
                    downloaded = 0;
                }
                let (Some(offset), Some(size)) = (
                    offset.unwrap_or(Some(0)),
                    size.map_or(Some(None), |s| s.map(Some)),
                ) else {
                    send_line(t, "FAIL", format_args!("bad fetch offset or size"))?;
                    continue;
                };
                let room = MAX_FETCH.min(max);
                if size.is_some_and(|s| s == 0 || s > room as u64) {
                    send_line(
                        t,
                        "FAIL",
                        format_args!("fetch size must be 1..=max-fetch-size"),
                    )?;
                    continue;
                }
                let r = if part == DOWNLOAD_TARGET {
                    crate::fetch::resolve(staged as u64, None, offset, size, room as u64)
                        .map(|(s, l)| s as usize..(s + l) as usize)
                } else {
                    d.fetch(part, offset, size, buf, &mut Sink(t))
                };
                match r {
                    Ok(r) if r.is_empty() || r.len() > room || r.end > buf.len() => send_line(
                        t,
                        "FAIL",
                        format_args!("fetch: nothing to send or too large"),
                    )?,
                    Ok(r) => {
                        send_line(t, "DATA", format_args!("{:08x}", r.len()))?;
                        t.send_data(&buf[r])?;
                        send_line(t, "OKAY", format_args!(""))?;
                    }
                    Err(e) => send_line(t, "FAIL", format_args!("{e}"))?,
                }
            }
            Command::Unknown(_) => send_line(t, "FAIL", format_args!("unknown command"))?,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::{collections::VecDeque, string::String, vec, vec::Vec};

    #[test]
    fn parse_commands() {
        fn p(s: &str) -> Command<'_> {
            parse(s.as_bytes()).unwrap()
        }
        assert_eq!(p("getvar:version"), Command::Getvar("version"));
        assert_eq!(p("getvar:"), Command::Getvar(""));
        assert_eq!(p("download:0000abcd"), Command::Download(Some(0xabcd)));
        assert_eq!(p("download:0x10"), Command::Download(Some(0x10)));
        assert_eq!(p("download:FFFFFFFF"), Command::Download(Some(u32::MAX)));
        assert_eq!(p("flash:bt0"), Command::Flash("bt0"));
        assert_eq!(p("erase:edk2"), Command::Erase("edk2"));
        assert_eq!(p("boot"), Command::Boot);
        assert_eq!(p("continue"), Command::Continue);
        assert_eq!(p("reboot"), Command::Reboot);
        assert_eq!(p("reboot-bootloader"), Command::RebootBootloader);
        assert_eq!(
            p("reboot-recovery"),
            Command::RebootOther("reboot-recovery")
        );
        assert_eq!(p("oem"), Command::Oem(""));
        assert_eq!(p("oem read32 0x3000"), Command::Oem("read32 0x3000"));
        // Near misses are not commands.
        for s in [
            "Boot",
            "boot ",
            "bootx",
            "rebootx",
            "oemx",
            "continue:",
            "getvar",
            "flash",
        ] {
            assert_eq!(p(s), Command::Unknown(s), "{s}");
        }
    }

    #[test]
    fn malformed_packets() {
        assert_eq!(parse(b""), Err(Malformed::Empty));
        assert_eq!(parse(&[b'a'; 65]), Err(Malformed::TooLong));
        assert!(parse(&[b'a'; 64]).is_ok());
        assert_eq!(parse(b"boot\0"), Err(Malformed::NotText));
        assert_eq!(parse(b"getvar:\xff"), Err(Malformed::NotText));
        assert_eq!(parse(b"boot\n"), Err(Malformed::NotText));
        assert_eq!(parse(b"\x7f"), Err(Malformed::NotText));
    }

    #[test]
    fn hex_sizes() {
        assert_eq!(hex_u32("0"), Some(0));
        assert_eq!(hex_u32("00001000"), Some(0x1000));
        assert_eq!(hex_u32("0X1f"), Some(0x1f));
        for bad in [
            "",
            "0x",
            "+10",
            "-1",
            " 10",
            "10 ",
            "123456789",
            "0x123456789",
            "g",
            "0xx1",
        ] {
            assert_eq!(hex_u32(bad), None, "{bad:?}");
        }
    }

    #[test]
    fn line_truncates_at_64_bytes() {
        let mut l = Line::new("INFO");
        let _ = write!(l, "{}", "x".repeat(100));
        assert_eq!(l.as_bytes().len(), 64);
        assert!(l.as_bytes().starts_with(b"INFOxxx"));
    }

    // ---- serve() on a scripted host ----

    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    enum E {
        Reset,
        Timeout,
        Flush,
    }

    #[derive(Default)]
    struct Host {
        packets: VecDeque<Vec<u8>>,
        replies: Vec<String>,
        resets: usize,
        reset_after_replies: Option<usize>,
        reset_on_reply: Option<&'static [u8]>,
        ended: bool,
        download_fail: bool,
        ack_fail: bool,
        flush_fail: bool,
        disconnected: usize,
        upload_fail: bool,
        uploaded: Vec<u8>,
    }

    impl Host {
        fn new(cmds: &[&str]) -> Self {
            Host {
                packets: cmds.iter().map(|c| c.as_bytes().to_vec()).collect(),
                ..Default::default()
            }
        }
    }

    impl Transport for Host {
        type Error = E;
        fn wait_configured(&mut self) -> Result<(), E> {
            self.ended = false;
            Ok(())
        }
        fn recv(&mut self, out: &mut [u8]) -> Result<usize, E> {
            if self.ended {
                return Err(E::Reset);
            }
            if self.reset_after_replies == Some(self.replies.len()) {
                self.reset_after_replies = None;
                return Err(E::Reset);
            }
            if self.resets > 0 {
                self.resets -= 1;
                return Err(E::Reset);
            }
            let p = self.packets.pop_front().ok_or(E::Timeout)?;
            out[..p.len()].copy_from_slice(&p);
            Ok(p.len())
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
        fn send_data(&mut self, data: &[u8]) -> Result<(), E> {
            if self.upload_fail {
                return Err(E::Timeout);
            }
            self.uploaded.extend_from_slice(data);
            Ok(())
        }
        fn send(&mut self, data: &[u8]) -> Result<(), E> {
            if self.reset_on_reply == Some(data) {
                self.ended = true;
            }
            if self.ended {
                return Err(E::Reset);
            }
            self.replies.push(String::from_utf8(data.to_vec()).unwrap());
            if self.ack_fail && data == b"OKAY" {
                return Err(E::Timeout);
            }
            Ok(())
        }
        fn disconnect(&mut self) -> Result<(), E> {
            self.disconnected += 1;
            if self.flush_fail {
                Err(E::Flush)
            } else {
                Ok(())
            }
        }
    }

    #[derive(Default)]
    struct Board {
        seen: Vec<String>,
        flashed: Vec<(String, usize)>,
        booted_len: Option<usize>,
        nor: Option<crate::nor::Guard>,
        emmc: Option<crate::emmcarm::Guard>,
    }

    impl Device for Board {
        type Boot = usize;
        fn getvar(&mut self, name: &str, out: &mut Line) -> bool {
            match name {
                "product" => write!(out, "model").is_ok(),
                _ => false,
            }
        }
        fn getvar_names(&self) -> &'static [&'static str] {
            &["product", "missing"]
        }
        fn command(&mut self, c: &Command) {
            self.seen.push(std::format!("{c:?}"));
            if let Some(guard) = self.nor.as_mut() {
                guard.observe(c);
            }
            if let Some(guard) = self.emmc.as_mut() {
                guard.observe(c);
            }
        }
        fn flash(
            &mut self,
            part: &str,
            image: &[u8],
            info: &mut dyn Info,
        ) -> Result<(), &'static str> {
            if part != "main" {
                return Err("unknown partition");
            }
            info.info(format_args!("writing {}", image.len()));
            self.flashed.push((part.into(), image.len()));
            Ok(())
        }
        fn erase(&mut self, part: &str, _info: &mut dyn Info) -> Result<(), &'static str> {
            if part.starts_with("emmc") {
                return self
                    .emmc
                    .as_mut()
                    .ok_or("not armed")?
                    .take(part, None, 0)
                    .map(|_| ());
            }
            if let Some(guard) = self.nor.as_mut() {
                return guard.take(part, None, 0).map(|_| ());
            }
            Err("not armed")
        }
        fn boot(
            &mut self,
            image: Option<&[u8]>,
            _info: &mut dyn Info,
        ) -> Result<usize, &'static str> {
            let image = image.ok_or("download a bundle first")?;
            if image[0] != 0 {
                return Err("bad bundle");
            }
            self.booted_len = Some(image.len());
            Ok(image.len())
        }
        fn continue_boot(
            &mut self,
            image: Option<&[u8]>,
            info: &mut dyn Info,
        ) -> Result<usize, &'static str> {
            match image {
                Some(_) => self.boot(image, info),
                None => Ok(0),
            }
        }
        fn reboot(&mut self, bootloader: bool) -> Result<(), &'static str> {
            if bootloader {
                Ok(())
            } else {
                Err("no reset")
            }
        }
        fn fetch(
            &mut self,
            part: &str,
            offset: u64,
            size: Option<u64>,
            buf: &mut [u8],
            _info: &mut dyn Info,
        ) -> Result<core::ops::Range<usize>, &'static str> {
            if part == "usb0p1" {
                // Model the board's partition lookup with a disk whose array
                // starts in the last sector but extends past the disk.
                let (mut header, entries) = crate::gpt::build(1024, &[(50, 60)]);
                header[72..80].copy_from_slice(&1023u64.to_le_bytes());
                crate::gpt::fix_header_crc(&mut header);
                let hdr = crate::gpt::header(&mut header, 1024)?;
                crate::gpt::parts(&hdr, &entries, 1024)?;
                return Err("test: invalid geometry accepted");
            }
            // A 0x3000-byte device whose bytes are their offset's low byte.
            if part != "dev" {
                return Err("unknown partition");
            }
            let len = size.unwrap_or(0x3000u64.saturating_sub(offset));
            if offset.checked_add(len).is_none_or(|e| e > 0x3000) {
                return Err("past the end");
            }
            let len = len as usize;
            if len > buf.len() {
                // Claims more than fits: serve() must refuse it.
                return Ok(0..len);
            }
            let start = 16.min(buf.len() - len);
            for i in 0..len {
                buf[start + i] = (offset as usize + i) as u8;
            }
            Ok(start..start + len)
        }
        fn oem(
            &mut self,
            args: &str,
            _image: Option<&[u8]>,
            info: &mut dyn Info,
        ) -> Result<Option<Exit<usize>>, &'static str> {
            match args {
                "fel" => Ok(Some(Exit::Bootloader)),
                "hello" => {
                    info.info(format_args!("hi"));
                    Ok(None)
                }
                "nor-arm main erase" => {
                    let guard = self.nor.get_or_insert_with(|| crate::nor::Guard::new(true));
                    guard.arm("main erase", 0)?;
                    Ok(None)
                }
                "emmc-arm emmc-boot0 erase" => {
                    let guard = self
                        .emmc
                        .get_or_insert_with(|| crate::emmcarm::Guard::new(true));
                    guard.arm("emmc-boot0 erase", 0)?;
                    Ok(None)
                }
                "emmc-arm bootcfg 08 02" => {
                    let guard = self
                        .emmc
                        .get_or_insert_with(|| crate::emmcarm::Guard::new(true));
                    guard.arm("bootcfg 08 02", 0)?;
                    Ok(None)
                }
                "emmc-bootcfg 08 02" => {
                    self.emmc
                        .as_mut()
                        .ok_or("not armed")?
                        .take_boot_config("08 02", 0)?;
                    Ok(None)
                }
                _ => Err("unknown oem command"),
            }
        }
    }

    fn run(h: &mut Host, b: &mut Board) -> Result<Exit<usize>, E> {
        let mut buf = vec![0u8; 0x1000];
        serve(h, b, &mut buf)
    }

    #[test]
    fn getvar_known_unknown_and_all() {
        let mut h = Host::new(&[
            "getvar:version",
            "getvar:max-download-size",
            "getvar:product",
            "getvar:is-userspace",
            "getvar:has-slot:bt0",
            "getvar:is-logical:edk2",
            "getvar:nope",
            "getvar:all",
            "reboot-bootloader",
        ]);
        let mut b = Board::default();
        assert_eq!(run(&mut h, &mut b), Ok(Exit::Bootloader));
        assert_eq!(
            h.replies,
            [
                "OKAY0.4",
                "OKAY0x00001000",
                "OKAYmodel",
                "OKAYno",
                "OKAYno",
                "OKAYno",
                "FAILunknown variable",
                "INFOversion:0.4",
                "INFOmax-download-size:0x00001000",
                "INFOmax-fetch-size:0x00001000",
                "INFOis-userspace:no",
                "INFOproduct:model",
                "OKAY",
                "OKAY",
            ]
        );
        assert_eq!(h.disconnected, 1);
    }

    #[test]
    fn download_sizes_and_malformed_input() {
        let mut h = Host::new(&[
            "download:00000000",
            "download:00001001",
            "download:+10",
            "download:",
            "download:0x123456789",
            "download:00000800",
            "",
            "",
            "getvar:\u{1}",
            "flash:main",
            "reboot-recovery",
            "frobnicate",
            "reboot-bootloader",
        ]);
        // A packet of exactly 64 bytes is still a command; 65 is not.
        h.packets.insert(7, vec![b'a'; 65]);
        let mut b = Board::default();
        assert_eq!(run(&mut h, &mut b), Ok(Exit::Bootloader));
        assert_eq!(
            h.replies,
            [
                "FAILbad download size",
                "FAILdownload larger than max-download-size 0x1000",
                "FAILbad download size",
                "FAILbad download size",
                "FAILbad download size",
                "DATA00000800",
                "OKAY",
                "FAILcommand longer than 64 bytes",
                "FAILcommand is not printable ASCII",
                "INFOwriting 2048",
                "OKAY",
                "FAILno such mode; use reboot or reboot-bootloader",
                "FAILunknown command",
                "OKAY",
            ]
        );
        assert_eq!(b.flashed, [("main".into(), 0x800)]);
        // Malformed packets never reach the board.
        assert_eq!(b.seen.len(), 10);
    }

    #[test]
    fn refused_download_drops_the_previous_image() {
        let mut h = Host::new(&[
            "download:10",
            "download:2000",
            "flash:main",
            "boot",
            "reboot-bootloader",
        ]);
        let mut b = Board::default();
        assert_eq!(run(&mut h, &mut b), Ok(Exit::Bootloader));
        assert_eq!(h.replies[3], "FAILdownload an image first");
        assert_eq!(h.replies[4], "FAILdownload a bundle first");
        assert!(b.flashed.is_empty());
    }

    #[test]
    fn download_failure_and_bus_reset_are_terminal() {
        let mut h = Host::new(&["download:10", "boot"]);
        h.download_fail = true;
        let mut b = Board::default();
        assert_eq!(run(&mut h, &mut b), Err(E::Timeout));
        assert_eq!(h.replies, ["DATA00000010"]);
        assert_eq!(b.booted_len, None);

        let mut h = Host::new(&["reboot-bootloader"]);
        h.resets = 3;
        assert_eq!(run(&mut h, &mut Board::default()), Err(E::Reset));
    }

    #[test]
    fn reset_ends_session_and_reconnect_cannot_use_staged_image_or_arms() {
        for (arm, action) in [
            ("oem nor-arm main erase", "erase:main"),
            ("oem emmc-arm emmc-boot0 erase", "erase:emmc-boot0"),
            ("oem emmc-arm bootcfg 08 02", "oem emmc-bootcfg 08 02"),
        ] {
            let mut buf = [0; 64];
            let mut first = Host::new(&["download:10", arm, action, "flash:main"]);
            first.reset_after_replies = Some(3);
            let mut board = Board::default();
            assert_eq!(serve(&mut first, &mut board, &mut buf), Err(E::Reset));
            assert_eq!(first.replies, ["DATA00000010", "OKAY", "OKAY"]);
            assert_eq!(first.packets.len(), 2);
            assert!(
                board
                    .nor
                    .as_ref()
                    .and_then(crate::nor::Guard::armed)
                    .is_some()
                    || board
                        .emmc
                        .as_ref()
                        .and_then(crate::emmcarm::Guard::armed)
                        .is_some()
            );

            // Reuse the DRAM but instantiate a fresh board session as bt0 does.
            let mut next = Host::new(&[action, "flash:main", "reboot-bootloader"]);
            assert_eq!(
                serve(&mut next, &mut Board::default(), &mut buf),
                Ok(Exit::Bootloader)
            );
            assert_eq!(
                next.replies,
                ["FAILnot armed", "FAILdownload an image first", "OKAY"]
            );
        }
    }

    #[test]
    fn reset_during_progress_is_terminal() {
        let mut h = Host::new(&["oem hello", "reboot-bootloader"]);
        h.reset_on_reply = Some(b"INFOhi");
        assert_eq!(run(&mut h, &mut Board::default()), Err(E::Reset));
        assert_eq!(h.packets.len(), 1);
    }

    #[test]
    fn boot_refusal_keeps_serving_and_success_exits_after_ack() {
        let mut h = Host::new(&["boot", "download:4", "boot"]);
        let mut b = Board::default();
        // The model's bundle check passes (first byte 0).
        assert_eq!(run(&mut h, &mut b), Ok(Exit::Boot(4)));
        assert_eq!(
            h.replies,
            [
                "FAILdownload a bundle first",
                "DATA00000004",
                "OKAY",
                "OKAY"
            ]
        );
        assert_eq!(h.disconnected, 1);

        let mut h = Host::new(&["continue"]);
        assert_eq!(run(&mut h, &mut Board::default()), Ok(Exit::Boot(0)));
    }

    #[test]
    fn exits_need_ack_and_quiescence() {
        let mut h = Host::new(&["download:4", "boot"]);
        h.ack_fail = true;
        // The download's OKAY fails first: terminal as well.
        assert_eq!(run(&mut h, &mut Board::default()), Err(E::Timeout));

        let mut h = Host::new(&["reboot-bootloader"]);
        h.ack_fail = true;
        assert_eq!(run(&mut h, &mut Board::default()), Err(E::Timeout));
        assert_eq!(h.disconnected, 1);

        let mut h = Host::new(&["reboot-bootloader"]);
        h.flush_fail = true;
        assert_eq!(run(&mut h, &mut Board::default()), Err(E::Flush));
    }

    #[test]
    fn reboot_refusal_and_oem() {
        let mut h = Host::new(&["reboot", "oem hello", "oem", "oem fel"]);
        let mut b = Board::default();
        assert_eq!(run(&mut h, &mut b), Ok(Exit::Bootloader));
        assert_eq!(
            h.replies,
            [
                "FAILno reset",
                "INFOhi",
                "OKAY",
                "FAILunknown oem command",
                "OKAY"
            ]
        );
    }

    #[test]
    fn parse_fetch_forms() {
        fn p(s: &str) -> Command<'_> {
            parse(s.as_bytes()).unwrap()
        }
        assert_eq!(
            p("fetch:nor"),
            Command::Fetch {
                part: "nor",
                offset: None,
                size: None
            }
        );
        assert_eq!(
            p("fetch:usb0:0x00000000:0x00100000"),
            Command::Fetch {
                part: "usb0",
                offset: Some(Some(0)),
                size: Some(Some(0x100000))
            }
        );
        assert_eq!(
            p("fetch:usb0:0x1d00000000:0x00100000"),
            Command::Fetch {
                part: "usb0",
                offset: Some(Some(0x1d_0000_0000)),
                size: Some(Some(0x100000))
            }
        );
        assert_eq!(
            p("fetch:sd:zz"),
            Command::Fetch {
                part: "sd",
                offset: Some(None),
                size: None
            }
        );
        assert_eq!(p("fetch:a:1:2:3"), Command::Unknown("fetch:a:1:2:3"));
        assert_eq!(hex_u64("ffffffffffffffff"), Some(u64::MAX));
        assert_eq!(hex_u64("10000000000000000"), None);
        assert_eq!(hex_u64("+1"), None);
    }

    #[test]
    fn fetch_chunks_like_the_stock_tool() {
        // getvar max-fetch-size, then the partition in max-fetch-size chunks
        // (the board's buffer here is 0x1000, so chunks of 0x1000).
        let mut h = Host::new(&[
            "getvar:max-fetch-size",
            "fetch:dev:0x00000000:0x00001000",
            "fetch:dev:0x00001000:0x00001000",
            "fetch:dev:0x00002000:0x00001000",
            "reboot-bootloader",
        ]);
        let mut b = Board::default();
        assert_eq!(run(&mut h, &mut b), Ok(Exit::Bootloader));
        assert_eq!(h.replies[0], "OKAY0x00001000");
        assert_eq!(&h.replies[1..4], ["DATA00001000", "OKAY", "DATA00001000"]);
        assert_eq!(h.uploaded.len(), 0x3000);
        assert!(h.uploaded.iter().enumerate().all(|(i, &b)| b == i as u8));
    }

    #[test]
    fn fetch_refusals_and_abort() {
        let mut h = Host::new(&[
            "fetch:dev:0x0:0x0",
            "fetch:dev:0x0:0x1001",
            "fetch:dev",
            "fetch:dev:0x2f00:0x200",
            "fetch:nope:0x0:0x10",
            "fetch:dev:0xg:0x10",
            "fetch:dev:0x2f00",
            "reboot-bootloader",
        ]);
        assert_eq!(run(&mut h, &mut Board::default()), Ok(Exit::Bootloader));
        assert_eq!(
            h.replies,
            [
                "FAILfetch size must be 1..=max-fetch-size",
                "FAILfetch size must be 1..=max-fetch-size",
                "FAILfetch: nothing to send or too large",
                "FAILpast the end",
                "FAILunknown partition",
                "FAILbad fetch offset or size",
                "DATA00000100",
                "OKAY",
                "OKAY",
            ]
        );
        // A transfer that breaks during the data phase ends the session.
        let mut h = Host::new(&["fetch:dev:0x0:0x100", "reboot-bootloader"]);
        h.upload_fail = true;
        assert_eq!(run(&mut h, &mut Board::default()), Err(E::Timeout));
        assert_eq!(h.replies, ["DATA00000100"]);
        // A fetch drops the staged download.
        let mut h = Host::new(&["download:10", "fetch:dev:0x0:0x10", "continue"]);
        let mut b = Board::default();
        // The model's continue boots "nothing" (0) when no image is staged.
        assert_eq!(run(&mut h, &mut b), Ok(Exit::Boot(0)));
        assert_eq!(b.booted_len, None);
    }

    #[test]
    fn bad_gpt_geometry_fails_fetch_but_not_session() {
        let mut h = Host::new(&["fetch:usb0p1", "getvar:version", "reboot-bootloader"]);
        assert_eq!(run(&mut h, &mut Board::default()), Ok(Exit::Bootloader));
        assert_eq!(
            h.replies,
            ["FAILGPT entry array outside the disk", "OKAY0.4", "OKAY"]
        );
        assert!(h.uploaded.is_empty());
    }

    #[test]
    fn download_round_trip() {
        let mut h = Host::new(&[
            "download:00000c00",
            "getvar:partition-size:download",
            "fetch:download:0x00000000:0x00000800",
            "fetch:download:0x00000800:0x00000400",
            "fetch:download:0x00000800:0x00000401",
            "boot",
        ]);
        let mut b = Board::default();
        // The staged image is still there for boot after the fetches.
        assert_eq!(run(&mut h, &mut b), Ok(Exit::Boot(0xc00)));
        assert_eq!(h.replies[2], "OKAY0xc00");
        assert_eq!(
            &h.replies[3..8],
            [
                "DATA00000800",
                "OKAY",
                "DATA00000400",
                "OKAY",
                "FAILfetch past the end"
            ]
        );
        assert_eq!(h.uploaded.len(), 0xc00);
        assert!(h.uploaded.iter().enumerate().all(|(i, &v)| v == i as u8));
        // Nothing downloaded: no such target.
        let mut h = Host::new(&[
            "fetch:download:0x0:0x10",
            "getvar:partition-size:download",
            "reboot-bootloader",
        ]);
        assert_eq!(run(&mut h, &mut Board::default()), Ok(Exit::Bootloader));
        assert_eq!(h.replies[0], "FAILfetch offset past the end");
        assert_eq!(h.replies[1], "FAILunknown variable");
    }

    #[test]
    fn host_silence_is_an_error() {
        let mut h = Host::new(&["getvar:version"]);
        assert_eq!(run(&mut h, &mut Board::default()), Err(E::Timeout));
    }
}
