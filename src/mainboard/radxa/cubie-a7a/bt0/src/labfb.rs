// SPDX-License-Identifier: GPL-2.0-only

//! DRAM lab mode over fastboot (feature `dram-lab`; the host-side scripts
//! are not part of this tree).
//!
//! One experiment per SoC reset: FEL entry 4 brings up clocks, UART and the
//! PMIC (not DRAM), takes USB0 over and serves fastboot. The host sends
//! `oem dram-run <A7A_* knobs>`: the knobs (`cubie_a7a_dram::knobs`, the
//! build-time names and formats) apply, and the DRAM init runs once, with
//! the build's code path (the open driver, `hybrid` or `bl-mission`). That
//! path ends in bt0's halt (or returns); either way the stack is reset and
//! fastboot goes on (the pending `dram-run` gets its OKAY then). A second
//! run in the same session is refused: retraining in a running session
//! leaves the PHY read path broken, so every run starts from a SoC reset.
//! `dram-run` checks that itself ([`dram_touched`]).
//!
//! # Leaving a session
//!
//! `fastboot reboot` (and `reboot-bootloader`, and four failed USB
//! sessions) end the session ([`leave`]). bt0 does not reset the SoC for
//! it: on the board, WDT0 written by bt0 after a lab DRAM run never reset
//! it (the soft reset register, the shortest interval, and the same after
//! the clean-up of a return to FEL: four runs out of four, each ended by a
//! power cycle), while the same registers written through the mask ROM's
//! FEL have reset it after every DRAM run of the FEL flow. So the session
//! ends in the ROM's FEL, entered afresh at its entry address
//! (`fel::lab_enter_rom_fel`) with USB0 back in reset, and the host resets
//! the SoC from there before the next run, as it does between FEL flow
//! runs. The previous way out, the WDT0 writes from bt0, is kept as
//! `A7A_LAB_EXIT=wdt` (build, or in a run's knobs) for comparison on the
//! board. A run that hangs still ends in the 16 s watchdog bt0 armed for
//! it.
//!
//! DRAM is the device under test, so nothing of the lab mode lives there:
//! the fastboot buffer, the run log (everything bt0 prints from entry 4 on)
//! and the knob text are in the mask ROM's FEL SRAM at 0x47000..0x77200,
//! which the session never gives back (no return into the FEL session that
//! started it); lab op lists and their output use `lab.rs`'s SRAM
//! (0xe0000..0xee000). Results:
//!
//! - `getvar lab-state | lab-knobs | lab-dram | lab-ops`;
//! - `oem lab-verdict`: the host's check (MRR MR5/MR8 on four ranks, three
//!   16 MiB memtests) and diagnosis (MR16/18/2) op lists in firmware, the
//!   same lines, plus a VERDICT line;
//! - `oem lab-summary`: the run log lines the host scripts look for;
//! - `stage <op list>` + `oem lab-ops`: any lab op list, then
//!   `fetch labout`;
//! - `fetch runlog | labout | phycsr | ctlregs`.
use core::arch::global_asm;
use core::fmt::Write;
use core::ops::Range;

use cubie_a7a_fastboot::proto::{self, Command, Device, Exit, Info, Line, Transport};
use cubie_a7a_fastboot::{fetch, usb};
use util::mmio::{read16, read32, write16, write32};

use crate::musb::{self, Musb};
use crate::{lab, soc, Pre};

// The ROM's FEL SRAM, 0x47000..0x87000: the open driver's PHY firmware
// (build.rs `.phyfw`, 0x47000..0x5b000), then:
/// The fastboot buffer (downloads, fetch staging).
pub const BUF: usize = 0x0005_b000;
pub const BUF_LEN: usize = 0x1_8000;
/// The run log (fwlog SRAM sink: length word, then the bytes).
pub const LOG: usize = 0x0007_3000;
pub const LOG_LEN: usize = 0x1_0000;
/// The knob text of the run (it must outlive the knobs that borrow it).
const KNOBS: usize = 0x0008_3000;
const KNOBS_LEN: usize = 0x200;

const PHY_BASE: usize = 0x0a90_0000;
const fn ctl(ch: usize, off: usize) -> usize {
    0x0a11_0000 + 0x40_0000 * ch + off
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum State {
    Idle,
    Running,
    /// The DRAM init returned.
    Done,
    /// bt0 halted (the end of every hybrid and bl-mission run, a failure,
    /// an exception or a panic).
    Halted,
}

impl State {
    fn name(self) -> &'static str {
        match self {
            State::Idle => "idle",
            State::Running => "running",
            State::Done => "done",
            State::Halted => "halted",
        }
    }
}

// .bss: cleared when entry 4 starts, kept across the stack reset.
static mut STATE: State = State::Idle;
static mut ACTIVE: bool = false;
static mut PRE: Option<Pre> = None;
static mut DRAM: Option<(u32, u32)> = None;
static mut KNOBS_N: usize = 0;
static mut OPS: (u32, u32, u32) = (0, 0, 0);
static mut T0: u64 = 0;

fn state() -> State {
    unsafe { core::ptr::read_volatile(core::ptr::addr_of!(STATE)) }
}

fn set_state(s: State) {
    unsafe { core::ptr::write_volatile(core::ptr::addr_of_mut!(STATE), s) }
}

pub fn active() -> bool {
    unsafe { core::ptr::read_volatile(core::ptr::addr_of!(ACTIVE)) }
}

fn knob_text() -> &'static str {
    let n = unsafe { core::ptr::read_volatile(core::ptr::addr_of!(KNOBS_N)) };
    // SAFETY: SRAM the lab mode owns; written once by dram-run.
    let b = unsafe { core::slice::from_raw_parts(KNOBS as *const u8, n) };
    core::str::from_utf8(b).unwrap_or("")
}

fn log_bytes() -> &'static [u8] {
    let n = (read32(LOG) as usize).min(LOG_LEN - 4);
    // SAFETY: the SRAM log the fwlog sink writes.
    unsafe { core::slice::from_raw_parts((LOG + 4) as *const u8, n) }
}

// After a run: a fresh stack, then fastboot again (the halt that ended the
// run may have been deep inside the DRAM code).
global_asm!(
    r#"
    .section .text.labfb, "ax"
    .arm
    .global labfb_reenter
labfb_reenter:
    ldr     sp, =_stack_top
    bl      labfb_after_run
1:  b       1b
    .ltorg
    "#
);

// The run itself on a fresh stack too: `dram-run` is served from
// deep inside the fastboot protocol loop, and the blob-free flow's frames
// (WL, read training, write stage) on top of it overran bt0's 16 KiB stack;
// the first lab run hung in WCK2CK leveling, mid-println. Nothing below the
// run is needed again: it always ends in `resume` (a fresh stack as well).
global_asm!(
    r#"
    .section .text.labfb, "ax"
    .arm
    .global labfb_fresh_run
labfb_fresh_run:
    ldr     sp, =_stack_top
    bl      labfb_run_body
1:  b       1b
    .ltorg
    "#
);

unsafe extern "C" {
    fn labfb_reenter() -> !;
    fn labfb_fresh_run() -> !;
}

#[unsafe(no_mangle)]
extern "C" fn labfb_run_body() -> ! {
    // SAFETY: dram_run stored it right before switching stacks.
    let Some(pre) = (unsafe { (*core::ptr::addr_of_mut!(RUN_PRE)).take() }) else {
        resume(State::Halted)
    };
    // WDT0 freshly armed (16 s, the whole system) for the run: a hang that
    // stops the kicks resets the board into FEL. The run's own waits and,
    // after the run, the fastboot loop (musb) keep kicking it.
    crate::recovery::arm_watchdog();
    let b = crate::bring_up_dram(pre);
    unsafe { *core::ptr::addr_of_mut!(DRAM) = Some((b.dram_mb, b.dram_mhz)) };
    resume(State::Done)
}

/// The pre-DRAM state handed across the stack switch.
static mut RUN_PRE: Option<Pre> = None;

/// End the run (bt0's halt, or the init returned) and serve again.
pub fn resume(end: State) -> ! {
    set_state(end);
    let ms = (soc::ticks() - unsafe { T0 }) / (soc::TIMER_HZ / 1000);
    println!("[lab] run {}: {} after {ms} ms", knob_text(), end.name());
    unsafe { labfb_reenter() }
}

#[unsafe(no_mangle)]
extern "C" fn labfb_after_run() -> ! {
    serve(true)
}

/// Entry 4 of a lab build, after the pre-DRAM bring-up.
pub fn run(pre: Pre) -> ! {
    unsafe {
        *core::ptr::addr_of_mut!(PRE) = Some(pre);
        core::ptr::write_volatile(core::ptr::addr_of_mut!(ACTIVE), true);
    }
    println!(
        "[lab] DRAM lab mode: oem dram-run <knobs>, results in SRAM, fastboot reboot between runs"
    );
    serve(false)
}

fn serve(after_run: bool) -> ! {
    let mut hex = [0u8; 16];
    let serial = usb::serial(soc::chip_sid(), &mut hex);
    let desc = usb::Descriptors {
        vendor: usb::FASTBOOT_VID,
        product: usb::FASTBOOT_PID,
        class: usb::FASTBOOT_CLASS,
        strings: ["oreboot", "Radxa Cubie A7A DRAM lab", serial, "fastboot"],
    };
    // SAFETY: the ROM's FEL SRAM, owned by the lab session.
    let buf = unsafe { core::slice::from_raw_parts_mut(BUF as *mut u8, BUF_LEN) };
    let mut dev = Lab;
    let mut resumed = after_run;
    for _ in 0..4 {
        let mut usb = if resumed {
            let mut u = Musb::resume(desc);
            // The answer to the `oem dram-run` that started the run.
            let mut l = Line::new("INFO");
            let _ = write!(l, "run {}", state().name());
            let _ = u.send(l.as_bytes());
            let _ = u.send(b"OKAY");
            u
        } else {
            Musb::take_over(desc).0
        };
        resumed = false;
        match proto::serve(&mut usb, &mut dev, buf) {
            Ok(Exit::Boot(())) => unreachable!(),
            Ok(_) => leave(BACK_ASKED),
            Err(e) => {
                println!(
                    "[lab] USB session ended: {}; enumerating again",
                    e.message()
                );
            }
        }
    }
    leave(BACK_USB)
}

/// Why the session ended, as fastboot.rs `Back` tells the host's mailbox.
const BACK_ASKED: u32 = 1;
const BACK_USB: u32 = 2;

/// A run's `A7A_LAB_EXIT=value` (Some(None) for `-A7A_LAB_EXIT`), which
/// `dram-run` takes out of the knob text: the knob is the lab mode's own,
/// not one of the DRAM crate's. None: the build's value.
static mut EXIT: Option<Option<&'static str>> = None;

/// `A7A_LAB_EXIT=wdt`: leave through bt0's own WDT0 reset, as before
/// ([`fel_style_reset`]). Anything else, or unset: the mask ROM's FEL
/// entry.
fn exit_by_wdt() -> bool {
    let run = unsafe { core::ptr::read_volatile(core::ptr::addr_of!(EXIT)) };
    run.unwrap_or(option_env!("A7A_LAB_EXIT")) == Some("wdt")
}

/// End the session (module comment, "Leaving a session").
fn leave(back: u32) -> ! {
    let uart = cubie_a7a_common::uart::Uart::attach(crate::UART0_BASE);
    if exit_by_wdt() {
        println!("[lab] reboot: watchdog reset (back into FEL)");
        musb::release();
        uart.drain();
        fel_style_reset()
    }
    musb::reset_for_rom();
    println!(
        "[lab] reboot: the mask ROM's FEL entry, no reset (ISR {:#x}); reset the SoC from FEL before the next run",
        crate::isr()
    );
    crate::fel::lab_enter_rom_fel(back)
}

/// RTC GP_DATA_REG2 and the vendor's reboot-into-FEL value (felreboot.rs,
/// which the FEL builds do not carry): a media bt0 that finds it enters
/// FEL instead of booting.
const FEL_FLAG_REG: usize = soc::RTC_BASE + 0x108;
const FEL_FLAG: u32 = 0x5aa5_a55a;

/// WDT0 whole-system reset with exactly the writes the host's felreset
/// script (scripts/felreset.sh in radxa-a7a-workspace) makes from FEL
/// (CFG = key | 1 whole system, MODE = key | 1 enable 0.5 s, CTRL
/// restart), which has brought the board back into FEL after every
/// blob-free run. `soc::watchdog_reset` also writes WDT0 +0x08 (key | 1)
/// first; after a lab DRAM run that path left the board out of FEL twice,
/// while without a run it came back in 2.3 s.
///
/// Kept as `A7A_LAB_EXIT=wdt`: with the clean-up below it left the board
/// out of FEL as well. It sets the reboot-into-FEL flag first, so that a
/// reset that does happen ends in FEL on a board whose eMMC or SD card
/// boots, too.
fn fel_style_reset() -> ! {
    write32(FEL_FLAG_REG, FEL_FLAG);
    // Measured: felreset's writes alone did not help either; first leave
    // the state entry 1's return to FEL leaves (USB0 to the ROM, its bus
    // clock, watchdog stopped, caches off), then the reset.
    crate::fel::lab_prepare_reset();
    const WDT0: usize = soc::WDT0_BASE;
    write32(WDT0 + 0x10, 0x16aa_0001);
    write32(WDT0 + 0x14, 0x16aa_0001);
    write32(WDT0 + 0x0c, 0x0000_14af);
    loop {
        unsafe { core::arch::asm!("wfi") };
    }
}

struct Lab;

const NAMES: &[&str] = &["product", "lab-state", "lab-knobs", "lab-dram", "lab-ops"];

/// `oem lab-verdict`: the host's check, then its diagnosis, as lab op lists.
fn verdict(info: &mut dyn Info) -> Result<(), &'static str> {
    if state() == State::Idle {
        return Err("no DRAM run yet");
    }
    // 8 MRRs, 3 memtests and 12 MRRs: 92 words with the END (the 64 of the
    // first board run panicked).
    let mut cmd = [0u32; 128];
    let mut n = 0;
    let mut push = |w: &[u32]| {
        cmd[n..n + w.len()].copy_from_slice(w);
        n += w.len();
    };
    for ch in 0..2 {
        for rk in 0..2 {
            for mr in [5, 8] {
                push(&[lab::op::MRR, ch, rk, mr]);
            }
        }
    }
    for base in [0x4010_0000, 0x8000_0000, 0xbf00_0000u32] {
        push(&[lab::op::MEMTEST, base, 16 << 20, 0x5a17]);
    }
    for mr in [16, 18, 2] {
        for (ch, rk) in [(0, 0), (0, 1), (1, 0), (1, 1)] {
            push(&[lab::op::MRR, ch, rk, mr]);
        }
    }
    push(&[lab::op::END]);
    for (i, w) in cmd[..n].iter().enumerate() {
        write32(lab::CMD + 4 * i, *w);
    }
    let (ops, outn, err) = lab::run();
    unsafe { *core::ptr::addr_of_mut!(OPS) = (ops, outn, err) };
    if err != 0 || outn != 8 + 18 + 12 {
        return Err("lab ops failed");
    }
    let out = |i: usize| read32(lab::OUT + 4 * i);
    let mut line = Line::new("");
    let _ = write!(line, "MRR MR5/MR8:");
    for i in 0..8 {
        let _ = write!(line, " {:#x}", out(i));
    }
    report(info, line);
    let mut bad = (0..8).any(|i| out(i) >> 16 == 0xdead);
    for (k, base) in [0x4010_0000u32, 0x8000_0000, 0xbf00_0000]
        .iter()
        .enumerate()
    {
        let o = 8 + 6 * k;
        let mut line = Line::new("");
        let _ = write!(
            line,
            "memtest {base:#x} 16MiB: errors {} first {:#x} want {:#x} got {:#x} xor-or {:#x} ({} ms)",
            out(o),
            out(o + 1),
            out(o + 2),
            out(o + 3),
            out(o + 4),
            out(o + 5)
        );
        report(info, line);
        bad |= out(o) != 0;
    }
    for (k, mr) in [16, 18, 2].iter().enumerate() {
        let mut line = Line::new("");
        let _ = write!(line, "post-verdict MR{mr}:");
        for r in 0..4 {
            let _ = write!(line, " {:#x}", out(26 + 4 * k + r));
        }
        report(info, line);
    }
    let mut line = Line::new("");
    let _ = write!(line, "VERDICT {}", if bad { "FAIL" } else { "PASS" });
    report(info, line);
    Ok(())
}

/// An INFO line to the host and the same line in the run log.
fn report(info: &mut dyn Info, l: Line) {
    let s = core::str::from_utf8(l.as_bytes()).unwrap_or("?");
    println!("[lab] {s}");
    info.info(format_args!("{s}"));
}

/// The run log lines the host scripts look for, as INFO lines (cut at 60).
const SUMMARY: &[&str] = &[
    "[dram] HYBRID",
    "[dram] end of init",
    "[dram] WL: lane maps restored",
    "[dram] WL: MR18 ops",
    "[dram] MISSION",
    "[dram] RD: skipped",
    "[dram] blobless: CA train",
    "[bt0] DRAM: open init variant",
    "[bt0] FEL: halted",
    "pstate 1: training",
    "WL ch ",
    "[lab] run ",
];

/// A fetch target's length without reading it (`getvar partition-size`).
fn target_size(name: &str) -> Option<usize> {
    let ran = state() != State::Idle;
    match name {
        "runlog" => Some(log_bytes().len()),
        "labout" => Some(4 * unsafe { OPS.1 } as usize),
        "phycsr" if ran => Some(
            cubie_a7a_dram::dump::RANGES
                .iter()
                .map(|&(a, b)| 2 * (b - a) as usize)
                .sum(),
        ),
        "ctlregs" if ran => Some(2 * 4 * (0x200 / 4 + 0xa0 / 4 + 11)),
        _ => None,
    }
}

/// Fill `buf` with a fetch target's bytes; its length.
fn target(name: &str, buf: &mut [u8]) -> Result<usize, &'static str> {
    let ran = state() != State::Idle;
    match name {
        "runlog" => {
            let l = log_bytes();
            buf[..l.len()].copy_from_slice(l);
            Ok(l.len())
        }
        "labout" => {
            let n = unsafe { OPS.1 } as usize;
            for i in 0..n {
                buf[4 * i..4 * i + 4].copy_from_slice(&read32(lab::OUT + 4 * i).to_le_bytes());
            }
            Ok(4 * n)
        }
        // PHY CSRs (dram dump::RANGES) as u16 LE, range after range; only
        // after a run (the PHY is not clocked before).
        "phycsr" if ran => {
            write16(PHY_BASE + 2 * 0xd0000, 0);
            let hclk = read16(PHY_BASE + 2 * 0xc0080);
            write16(PHY_BASE + 2 * 0xc0080, 3);
            let mut n = 0;
            for &(a, b) in cubie_a7a_dram::dump::RANGES.iter() {
                for c in a..b {
                    let v = read16(PHY_BASE + 2 * c as usize);
                    buf[n..n + 2].copy_from_slice(&v.to_le_bytes());
                    n += 2;
                }
                crate::recovery::kick();
            }
            write16(PHY_BASE + 2 * 0xc0080, hclk);
            write16(PHY_BASE + 2 * 0xd0000, 1);
            Ok(n)
        }
        // Controller registers in the host dump tool's order, u32 LE, ch0 then ch1.
        "ctlregs" if ran => {
            let mut n = 0;
            for ch in 0..2 {
                let regs = (0..0x200)
                    .step_by(4)
                    .chain((0x500..0x5a0).step_by(4))
                    .chain([
                        0xb80, 0xb84, 0xb88, 0xc80, 0xc84, 0xd00, 0xd04, 0xd08, 0xd0c, 0xd10, 0xd14,
                    ]);
                for o in regs {
                    buf[n..n + 4].copy_from_slice(&read32(ctl(ch, o)).to_le_bytes());
                    n += 4;
                }
            }
            Ok(n)
        }
        "phycsr" | "ctlregs" => Err("no DRAM run yet (the PHY is not clocked)"),
        _ => Err("lab mode targets: runlog labout phycsr ctlregs"),
    }
}

impl Device for Lab {
    type Boot = ();

    fn getvar(&mut self, name: &str, out: &mut Line) -> bool {
        match name {
            "product" => write!(out, "cubie-a7a-dram-lab"),
            "lab-state" => write!(out, "{}", state().name()),
            "lab-knobs" => write!(out, "{}", knob_text()),
            "lab-dram" => match unsafe { DRAM } {
                Some((mb, mhz)) => write!(out, "{mb} MiB {mhz} MHz"),
                None => write!(out, "none"),
            },
            "lab-ops" => {
                let (o, n, e) = unsafe { OPS };
                write!(out, "{o} {n} {e:#x}")
            }
            // The stock fastboot tool sizes a fetch with partition-size
            // first (measured: `fetch runlog` failed without it).
            _ => match name.strip_prefix("partition-size:").and_then(target_size) {
                Some(n) => write!(out, "0x{n:x}"),
                None => return false,
            },
        }
        .is_ok()
    }

    fn getvar_names(&self) -> &'static [&'static str] {
        NAMES
    }

    fn command(&mut self, _c: &Command) {}

    fn flash(&mut self, _: &str, _: &[u8], _: &mut dyn Info) -> Result<(), &'static str> {
        Err("DRAM lab mode: no flash")
    }

    fn erase(&mut self, _: &str, _: &mut dyn Info) -> Result<(), &'static str> {
        Err("DRAM lab mode: no erase")
    }

    fn boot(&mut self, _: Option<&[u8]>, _: &mut dyn Info) -> Result<(), &'static str> {
        Err("DRAM lab mode: no hand-over")
    }

    fn continue_boot(&mut self, _: Option<&[u8]>, _: &mut dyn Info) -> Result<(), &'static str> {
        Err("DRAM lab mode: no hand-over")
    }

    fn reboot(&mut self, _bootloader: bool) -> Result<(), &'static str> {
        // Both end the session the same way ([`leave`]): the FEL session
        // that started it cannot be resumed (its SRAM is in use).
        Ok(())
    }

    fn fetch(
        &mut self,
        part: &str,
        offset: u64,
        size: Option<u64>,
        buf: &mut [u8],
        _info: &mut dyn Info,
    ) -> Result<Range<usize>, &'static str> {
        let n = target(part, buf)? as u64;
        let (start, len) = fetch::resolve(n, None, offset, size, buf.len() as u64)?;
        Ok(start as usize..(start + len) as usize)
    }

    fn oem(
        &mut self,
        args: &str,
        image: Option<&[u8]>,
        info: &mut dyn Info,
    ) -> Result<Option<Exit<()>>, &'static str> {
        let (cmd, rest) = args.split_once(' ').unwrap_or((args, ""));
        match cmd {
            "dram-run" => {
                // Knobs on the command line, or (longer than a 64-byte
                // command allows) as staged text.
                let text = if rest.is_empty() {
                    let img = image.ok_or("knobs: oem dram-run <knobs>, or stage a knob text")?;
                    core::str::from_utf8(img).map_err(|_| "staged knobs are not text")?
                } else {
                    rest
                };
                dram_run(text.trim_end_matches(['\n', '\r', ' '])).map(|_| None)
            }
            #[cfg(not(dram_blobfree))]
            "phyfw" => {
                let img = image.ok_or("stage the build's phy-fw-lab.bin first")?;
                crate::dram::open::lab_fw::load(img)?;
                info.info(format_args!("PHY firmware loaded ({} bytes)", img.len()));
                Ok(None)
            }
            "lab-verdict" => verdict(info).map(|()| None),
            "lab-summary" => {
                for line in log_bytes().split(|&b| b == b'\n') {
                    if SUMMARY
                        .iter()
                        .any(|p| line.windows(p.len()).any(|w| w == p.as_bytes()))
                    {
                        let s = core::str::from_utf8(line)
                            .unwrap_or("?")
                            .trim_end_matches('\r');
                        info.info(format_args!("{s}"));
                    }
                }
                Ok(None)
            }
            "lab-ops" => {
                let img = image.ok_or("stage a lab op list first")?;
                if img.len() % 4 != 0 || img.len() > lab::CMD_WORDS * 4 - 4 {
                    return Err("op list: whole words, at most 4 KiB");
                }
                for (i, w) in img.chunks_exact(4).enumerate() {
                    write32(
                        lab::CMD + 4 * i,
                        u32::from_le_bytes([w[0], w[1], w[2], w[3]]),
                    );
                }
                write32(lab::CMD + img.len(), lab::op::END);
                let r = lab::run();
                unsafe { *core::ptr::addr_of_mut!(OPS) = r };
                info.info(format_args!("ops {} out {} err {:#x}", r.0, r.1, r.2));
                if r.2 != 0 {
                    return Err("lab op error (getvar lab-ops)");
                }
                Ok(None)
            }
            "help" | "" => {
                for l in [
                    "stage phy-fw-lab.bin; oem phyfw (open driver images)",
                    "oem dram-run <A7A_*=v | -A7A_*>...: one DRAM run",
                    "  (or stage the knob text, then oem dram-run)",
                    "oem lab-verdict: MRR, 3 memtests, MR16/18/2",
                    "oem lab-summary: run log lines as run-one.sh",
                    "stage <ops>; oem lab-ops; fetch labout",
                    "fetch runlog | labout | phycsr | ctlregs",
                ] {
                    info.info(format_args!("{l}"));
                }
                Ok(None)
            }
            _ => Err("unknown oem command (oem help)"),
        }
    }
}

/// R_PRCM +0x250 [0], the DRAM PHY's isolation release: 0 after every SoC
/// reset, set by every DRAM init and kept until the next reset. Set before
/// a run, it means bt0 was loaded again without a reset (`reboot` leaves
/// through the ROM's FEL entry, not a reset), and a DRAM init on top of the
/// last one is not a lab result.
fn dram_touched() -> bool {
    read32(soc::R_PRCM_BASE + 0x250) & 1 != 0
}

/// `oem dram-run <knobs>`: once per session. Knob errors are refused
/// before anything runs; then the run ends in [`resume`] (no return).
fn dram_run(text: &str) -> Result<(), &'static str> {
    if state() != State::Idle {
        return Err("one DRAM run per session: fastboot reboot first");
    }
    if dram_touched() {
        return Err("DRAM was up since the last SoC reset: reset from FEL first");
    }
    #[cfg(not(dram_blobfree))]
    if !crate::dram::open::lab_fw::loaded() {
        return Err("PHY firmware not loaded (stage phy-fw-lab.bin; oem phyfw)");
    }
    if text.len() > KNOBS_LEN {
        return Err("knob text too long");
    }
    for (i, b) in text.bytes().enumerate() {
        unsafe { core::ptr::write_volatile((KNOBS + i) as *mut u8, b) };
    }
    unsafe { core::ptr::write_volatile(core::ptr::addr_of_mut!(KNOBS_N), text.len()) };
    let text = knob_text();
    let mut k = cubie_a7a_dram::knobs::get();
    let mut exit = None;
    for tok in text.split([' ', '\n']).filter(|t| !t.is_empty()) {
        // The lab's own knob (`A7A_LAB_EXIT=value`, `-A7A_LAB_EXIT`);
        // everything else is the DRAM crate's.
        if let Some(v) = tok.strip_prefix("A7A_LAB_EXIT=") {
            exit = Some(Some(v));
        } else if tok == "-A7A_LAB_EXIT" {
            exit = Some(None);
        } else {
            k.apply(tok)?;
        }
    }
    unsafe { core::ptr::write_volatile(core::ptr::addr_of_mut!(EXIT), exit) };
    cubie_a7a_dram::knobs::set(k);
    // SAFETY: entry 4 stored it; taken once (state Idle).
    let Some(pre) = (unsafe { (*core::ptr::addr_of_mut!(PRE)).take() }) else {
        return Err("no pre-DRAM state");
    };
    println!(
        "[lab] dram-run: {text} (A7A_OPEN_OPTS {:#x}, endinit {}, skip RD {}, EI {:?})",
        k.open_opts, k.hybrid_endinit, k.hybrid_skip_rd, k.ei
    );
    set_state(State::Running);
    unsafe { T0 = soc::ticks() };
    unsafe {
        *core::ptr::addr_of_mut!(RUN_PRE) = Some(pre);
        labfb_fresh_run()
    }
}
