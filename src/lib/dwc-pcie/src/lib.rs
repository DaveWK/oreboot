// SPDX-License-Identifier: GPL-2.0-only
//! DesignWare PCIe root complex bring-up, so the payload only has to
//! enumerate.
//!
//! From ore-edk-boot-radxa-a7a `main/src/pcie.rs` (A733: DesignWare PCIe 3.0
//! RC behind a Cadence combo PHY). The DesignWare core part is here: the
//! express capability walk, RC set-up (link capability, lane mode, BARs
//! disabled through DBI2, bus numbers, bridge class), LTSSM training with
//! empty-slot detection, the directed speed change with a Gen1 fall-back,
//! unrolled iATU outbound regions and the endpoint probe through the config
//! window, in the board's order and with its bounded polls. The SoC part is
//! a [`Platform`]: slot power, WAKE# and PERST#, the power domain, clocks,
//! the PHY and the vendor glue block that starts the LTSSM and reports the
//! link. Every register access goes through [`regio::RegIo`].
//!
//! On success the link is trained, iATU region 1 maps MEM and region 2 I/O;
//! region 0 is left for the payload to retarget per config access. On
//! failure [`init`] puts the controller back into reset, asserts PERST# and
//! switches the slot off.
#![no_std]

use regio::RegIo;

/// DesignWare port logic and config space offsets (from DBI).
pub mod reg {
    pub const PORT_LINK_CTRL: usize = 0x710;
    /// [5:0] LTSSM state.
    pub const PL_DEBUG0: usize = 0x728;
    /// [16:8] lane width, bit 17 directed speed change.
    pub const GEN2_CTRL: usize = 0x80c;
    pub const SPEED_CHANGE: u32 = 1 << 17;
    /// Bit 0: DBI read-only write enable.
    pub const MISC_CONTROL_1: usize = 0x8bc;
    pub const ATU_TYPE_MEM: u32 = 0;
    pub const ATU_TYPE_IO: u32 = 2;
    pub const ATU_TYPE_CFG0: u32 = 4;
    pub const ATU_ENABLE: u32 = 1 << 31;
    /// Unrolled iATU: one 0x200-byte block per outbound region.
    pub const ATU_REGION_STRIDE: usize = 0x200;
}
use reg::*;

/// An outbound window: CPU address, PCI address, size.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Window {
    pub cpu: u64,
    pub pci: u64,
    pub size: u64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Config {
    /// DBI, DBI2 (shadow/mask registers) and unrolled iATU bases.
    pub dbi: usize,
    pub dbi2: usize,
    pub atu: usize,
    /// Target speed after the Gen1 link is up.
    pub max_gen: u32,
    /// 1, 2, 4 or 8.
    pub lanes: u32,
    /// Slot power stable to PERST# release (PCIe CEM T_PVPERL >= 100 ms).
    pub perst_hold_us: u64,
    /// Config window decoded by iATU region 0 (CPU address, size); bus 1
    /// device 0 is probed through it.
    pub config: Window,
    /// Windows of iATU regions 1 (MEM) and 2 (I/O).
    pub mem: Window,
    pub io: Window,
}

/// The SoC around the DesignWare core.
pub trait Platform<R: RegIo> {
    type Error: Copy + core::fmt::Debug;
    /// PERST#: `true` asserts it (drives it low).
    fn perst(&mut self, io: &mut R, asserted: bool);
    /// Slot power (and WAKE#, if the board drives it) on or off.
    fn slot_power(&mut self, io: &mut R, on: bool);
    fn power_domain_on(&mut self, io: &mut R) -> Result<(), Self::Error>;
    /// Clocks and resets of controller and PHY. No DBI access before this.
    fn clocks_on(&mut self, io: &mut R);
    /// PHY programming up to "ready".
    fn phy_init(&mut self, io: &mut R) -> Result<(), Self::Error>;
    /// Diagnostics after the PHY is ready (log glue registers); optional.
    fn after_phy(&mut self, _io: &mut R) {}
    /// Start or stop LTSSM training (vendor glue).
    fn ltssm_enable(&mut self, io: &mut R, on: bool);
    /// Data link up (the glue's link status, e.g. SMLH and RDLH).
    fn link_up(&mut self, io: &mut R) -> bool;
    /// Raw glue link status, for the error report.
    fn link_status(&mut self, io: &mut R) -> u32;
    /// Controller back into reset, clocks gated (after [`Self::clocks_on`]).
    fn clocks_off(&mut self, io: &mut R);
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error<E> {
    Platform(E),
    /// No PCI Express capability in the RC's config space.
    NoExpressCap,
    /// The LTSSM never left Detect: nothing in the slot.
    Empty,
    /// Glue link status and the last LTSSM state.
    LinkTimeout(u32, u32),
    /// The endpoint never answered config reads (last value).
    NoEndpoint(u32),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Link {
    pub gen: u32,
    pub width: u32,
    /// Endpoint vendor ID | device ID << 16.
    pub id: u32,
    pub class: u32,
}

fn set32<R: RegIo>(io: &mut R, a: usize, bits: u32) {
    let v = io.read32(a);
    io.write32(a, v | bits);
}
fn clr32<R: RegIo>(io: &mut R, a: usize, bits: u32) {
    let v = io.read32(a);
    io.write32(a, v & !bits);
}
fn rmw32<R: RegIo>(io: &mut R, a: usize, clr: u32, set: u32) {
    let v = io.read32(a);
    io.write32(a, (v & !clr) | set);
}

/// Poll `cond` every `step_us` until it holds or `max_us` passed; the last
/// check is made after the time ran out (the board's semantics).
pub fn wait<R: RegIo>(
    io: &mut R,
    max_us: u64,
    step_us: u32,
    mut cond: impl FnMut(&mut R) -> bool,
) -> bool {
    let t = io.now_us();
    loop {
        if cond(io) {
            return true;
        }
        if io.now_us().wrapping_sub(t) >= max_us {
            return cond(io);
        }
        io.udelay(step_us);
    }
}

/// Offset of the PCI Express capability in the RC's config space.
pub fn find_express_cap<R: RegIo>(io: &mut R, c: &Config) -> Option<usize> {
    let mut ptr = (io.read16(c.dbi + 0x34) & 0xff) as usize;
    for _ in 0..48 {
        if ptr == 0 || ptr > 0xfc {
            return None;
        }
        let hdr = io.read16(c.dbi + ptr);
        if hdr & 0xff == 0x10 {
            return Some(ptr);
        }
        ptr = (hdr >> 8) as usize;
    }
    None
}

/// Port link mode field of PORT_LINK_CTRL for `lanes`.
pub const fn link_mode(lanes: u32) -> u32 {
    match lanes {
        1 => 0x1,
        2 => 0x3,
        4 => 0x7,
        _ => 0xf,
    }
}

/// RC set-up for training at Gen1; BARs disabled through their DBI2 masks so
/// the payload sees a plain bridge. Returns the express capability offset.
pub fn setup_rc<R: RegIo, E>(io: &mut R, c: &Config) -> Result<usize, Error<E>> {
    let dbi = c.dbi;
    set32(io, dbi + MISC_CONTROL_1, 1);
    let Some(cap) = find_express_cap(io, c) else {
        clr32(io, dbi + MISC_CONTROL_1, 1);
        return Err(Error::NoExpressCap);
    };
    rmw32(io, dbi + cap + 0x0c, 0xf, c.max_gen); // LNKCAP: max link speed
    rmw32(io, dbi + cap + 0x30, 0xf, 1); // LNKCTL2: target Gen1 for training
    rmw32(
        io,
        dbi + PORT_LINK_CTRL,
        0x3f << 16,
        link_mode(c.lanes) << 16,
    );
    rmw32(io, dbi + GEN2_CTRL, 0x1ff << 8, c.lanes << 8);
    io.write32(c.dbi2 + 0x10, 0);
    io.write32(c.dbi2 + 0x14, 0);
    io.write32(dbi + 0x10, 0);
    io.write32(dbi + 0x14, 0);
    rmw32(io, dbi + 0x3c, 0xff00, 0x0100); // interrupt pin INTA
    rmw32(io, dbi + 0x18, 0x00ff_ffff, 0x00ff_0100); // buses 0 / 1 / 0xff
    rmw32(
        io,
        dbi + 0x04,
        0xffff,
        (1 << 0) | (1 << 1) | (1 << 2) | (1 << 8),
    );
    io.write16(dbi + 0x0a, 0x0604); // class: PCI-to-PCI bridge
    clr32(io, dbi + MISC_CONTROL_1, 1);
    Ok(cap)
}

pub fn ltssm<R: RegIo>(io: &mut R, c: &Config) -> u32 {
    io.read32(c.dbi + PL_DEBUG0) & 0x3f
}

/// LTSSM on, then wait for the link. A slot whose LTSSM stays in Detect
/// (states 0/1) for 100 ms is empty; a link that started training gets 1 s.
pub fn train<R: RegIo, P: Platform<R>>(
    io: &mut R,
    p: &mut P,
    c: &Config,
) -> Result<(), Error<P::Error>> {
    p.ltssm_enable(io, true);
    let t = io.now_us();
    let mut seen = false;
    loop {
        if p.link_up(io) {
            return Ok(());
        }
        let s = ltssm(io, c);
        seen |= s > 1;
        let us = io.now_us().wrapping_sub(t);
        if !seen && us >= 100_000 {
            return Err(Error::Empty);
        }
        if us >= 1_000_000 {
            return Err(Error::LinkTimeout(p.link_status(io), s));
        }
        io.udelay(100);
    }
}

/// Current link speed and width (LNKSTA).
pub fn link_status<R: RegIo>(io: &mut R, c: &Config, cap: usize) -> (u32, u32) {
    let sta = io.read16(c.dbi + cap + 0x12) as u32;
    (sta & 0xf, (sta >> 4) & 0x3f)
}

/// Raise the target speed and request a directed speed change; if the link
/// does not come back within 100 ms, retrain at Gen1 rather than lose it.
pub fn speed_up<R: RegIo, P: Platform<R>>(
    io: &mut R,
    p: &mut P,
    c: &Config,
    cap: usize,
) -> Result<(), Error<P::Error>> {
    if c.max_gen <= 1 {
        return Ok(());
    }
    let dbi = c.dbi;
    set32(io, dbi + MISC_CONTROL_1, 1);
    rmw32(io, dbi + cap + 0x30, 0xf, c.max_gen);
    clr32(io, dbi + GEN2_CTRL, SPEED_CHANGE);
    set32(io, dbi + GEN2_CTRL, SPEED_CHANGE);
    clr32(io, dbi + MISC_CONTROL_1, 1);
    let done = wait(io, 20_000, 100, |io| {
        io.read32(dbi + GEN2_CTRL) & SPEED_CHANGE == 0
    });
    let back = wait(io, 100_000, 100, |io| {
        p.link_up(io) && io.read16(dbi + cap + 0x12) & (1 << 11) == 0 // not training
    });
    if back {
        if !done {
            io.log(format_args!(
                "[main] PCIe: speed change bit did not clear\n"
            ));
        }
        return Ok(());
    }
    io.log(format_args!(
        "[main] PCIe: link lost after the Gen{} change; retraining at Gen1\n",
        c.max_gen
    ));
    rmw32(io, dbi + cap + 0x30, 0xf, 1);
    set32(io, dbi + cap + 0x10, 1 << 5); // LNKCTL: retrain link
    if wait(io, 100_000, 100, |io| p.link_up(io)) {
        Ok(())
    } else {
        let s = p.link_status(io);
        Err(Error::LinkTimeout(s, ltssm(io, c)))
    }
}

/// Program and enable unrolled iATU outbound region `region`.
pub fn atu_outbound<R: RegIo>(io: &mut R, c: &Config, region: usize, kind: u32, w: Window) {
    let r = c.atu + region * ATU_REGION_STRIDE;
    io.write32(r + 0x08, w.cpu as u32);
    io.write32(r + 0x0c, (w.cpu >> 32) as u32);
    io.write32(r + 0x10, (w.cpu + w.size - 1) as u32);
    io.write32(r + 0x14, w.pci as u32);
    io.write32(r + 0x18, (w.pci >> 32) as u32);
    io.write32(r, kind); // region control 1: type
    io.write32(r + 0x04, ATU_ENABLE);
    if !wait(io, 20_000, 1000, |io| io.read32(r + 0x04) & ATU_ENABLE != 0) {
        io.log(format_args!(
            "[main] PCIe: iATU region {region} did not enable\n"
        ));
    }
}

/// Read bus 1 device 0 through iATU region 0. After training, devices may
/// answer Configuration Request Retry (vendor 0x0001 or all ones) for 1 s.
/// Returns (vendor | device << 16, class code).
pub fn endpoint_id<R: RegIo, E>(io: &mut R, c: &Config) -> Result<(u32, u32), Error<E>> {
    atu_outbound(
        io,
        c,
        0,
        ATU_TYPE_CFG0,
        Window {
            cpu: c.config.cpu,
            pci: 1 << 24,
            size: c.config.size,
        },
    );
    let cfg = c.config.cpu as usize;
    let mut id = 0;
    let ok = wait(io, 1_000_000, 10_000, |io| {
        id = io.read32(cfg);
        id != 0xffff_ffff && id != 0 && id & 0xffff != 0x0001
    });
    if !ok {
        return Err(Error::NoEndpoint(id));
    }
    Ok((id, io.read32(cfg + 0x08) >> 8))
}

/// The whole bring-up: slot power with PERST# held, power domain, clocks,
/// PHY, PERST# release after the hold time, RC set-up, training, speed
/// change, the 100 ms config-request settle (PCIe base 6.6.1), endpoint
/// probe and the MEM/I/O windows. `clocked` tells [`shut_down`] how far it
/// got.
pub fn bring_up<R: RegIo, P: Platform<R>>(
    io: &mut R,
    p: &mut P,
    c: &Config,
    clocked: &mut bool,
) -> Result<Link, Error<P::Error>> {
    p.perst(io, true);
    p.slot_power(io, true);
    let t_power = io.now_us();
    p.power_domain_on(io).map_err(Error::Platform)?;
    p.clocks_on(io);
    *clocked = true;
    p.phy_init(io).map_err(Error::Platform)?;
    p.after_phy(io);
    p.ltssm_enable(io, false);
    let held = io.now_us().wrapping_sub(t_power);
    if held < c.perst_hold_us {
        io.udelay((c.perst_hold_us - held) as u32);
    }
    p.perst(io, false);
    let cap = setup_rc(io, c)?;
    train(io, p, c)?;
    speed_up(io, p, c, cap)?;
    io.udelay(100_000);
    let (gen, width) = link_status(io, c, cap);
    let (id, class) = endpoint_id(io, c)?;
    atu_outbound(io, c, 1, ATU_TYPE_MEM, c.mem);
    atu_outbound(io, c, 2, ATU_TYPE_IO, c.io);
    Ok(Link {
        gen,
        width,
        id,
        class,
    })
}

/// Back to the state a cold boot leaves: training off, endpoint in reset,
/// controller in reset with its clocks gated, slot off.
pub fn shut_down<R: RegIo, P: Platform<R>>(io: &mut R, p: &mut P, clocked: bool) {
    if clocked {
        p.ltssm_enable(io, false);
    }
    p.perst(io, true);
    if clocked {
        p.clocks_off(io);
    }
    p.slot_power(io, false);
}

/// Bring the root complex and the link up; on failure shut everything down
/// again. Never panics; the caller decides whether boot goes on.
pub fn init<R: RegIo, P: Platform<R>>(
    io: &mut R,
    p: &mut P,
    c: &Config,
) -> Result<Link, Error<P::Error>> {
    let mut clocked = false;
    let r = bring_up(io, p, c, &mut clocked);
    if r.is_err() {
        shut_down(io, p, clocked);
    }
    r
}
