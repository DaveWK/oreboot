// SPDX-License-Identifier: GPL-2.0-only
//! A behavioural DWC3 device-mode model plus a scripted USB host, for host
//! tests of the driver: registers (with the self-clearing bits, endpoint
//! command completion and transfer resource indices the TRM describes),
//! the event buffer in the driver's memory, TRB write-back, and a host that
//! resets the bus, runs control transfers through their SETUP, data and
//! status stages, and moves bulk data. Every register access is recorded as
//! a `regtrace::Ev` for the trace tests.
#![allow(dead_code)]

use dwc3_udc::{event, regs::*, trb, Dwc3Platform, PhyTimeout};
use regio::RegIo;
use regtrace::Ev;
use std::collections::{BTreeMap, VecDeque};
use std::fmt;

pub const BASE: usize = 0xfc00_0000;
/// The RK3588 USB3OTG0 core: DWC_usb3 3.00a, GSNPSID read on the first
/// hardware run (2026-11-09). The model had assumed 3.30a; both are run.
pub const SNPSID_300A: u32 = 0x5533_300a;
pub const SNPSID_330A: u32 = 0x5533_330a;
pub const SNPSID: u32 = SNPSID_300A;

impl Model {
    /// The default model with GSNPSID = `id`.
    pub fn with_snpsid(id: u32) -> Self {
        let mut m = Self::default();
        m.regs.insert(GSNPSID, id);
        m
    }
}

/// One host action.
#[derive(Clone, Debug)]
pub enum Host {
    /// Bus reset, then ConnectDone (High-Speed unless `full_speed`).
    Reset,
    /// A control transfer: the SETUP packet; the model runs the stages.
    Setup([u8; 8]),
    /// Bulk OUT data, consumed by as many device transfers as it takes.
    BulkOut(Vec<u8>),
    /// One bulk IN transfer read by the host.
    BulkIn,
    /// Cable pulled (DisconnEvt).
    Unplug,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Stage {
    Idle,
    /// SETUP delivered; wLength and direction decide what follows.
    AfterSetup {
        length: u16,
        dir_in: bool,
    },
    /// Waiting for the device's IN data stage TRB.
    DataIn,
    /// Waiting for the status stage TRB on this physical endpoint.
    Status(usize),
}

/// What the host saw of a control transfer.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Control {
    pub setup: [u8; 8],
    pub data: Vec<u8>,
    pub stalled: bool,
    /// TRB control type of the status stage (3 two-stage, 4 three-stage).
    pub status_type: u32,
}

pub struct Model {
    pub regs: BTreeMap<usize, u32>,
    pub evs: Vec<Ev>,
    pub clock_us: u64,
    queue: VecDeque<u32>,
    ev_pos: usize,
    count: u32,
    active: [Option<usize>; 4],
    pub cmds: Vec<(usize, u32, [u32; 3])>,
    pub script: VecDeque<Host>,
    stage: Stage,
    current: Option<Control>,
    pub controls: Vec<Control>,
    pub bulk_in: Vec<Vec<u8>>,
    /// Bytes of each bulk OUT transfer the device asked for.
    pub bulk_out_sizes: Vec<u32>,
    pub full_speed: bool,
    /// The model panics after this many polls without host progress.
    pub max_idle: usize,
    idle: usize,
    /// Return 0 from DSTS.DevCtrlHlt forever (a halt that never comes).
    pub never_halt: bool,
    /// Keep DCTL.CSFTRST set forever.
    pub stuck_reset: bool,
    /// Pending events the BootROM left (acknowledged during the halt).
    pub stale_events: u32,
    /// Silent polls allowed after the script ran out (for timeouts).
    pub allow_idle_end: bool,
}

impl Default for Model {
    fn default() -> Self {
        let mut regs = BTreeMap::new();
        // Reset values, TRM pp.2190-2228.
        regs.insert(GSNPSID, SNPSID);
        regs.insert(GCTL, 0x1351_2000); // PwrDnScale 0x26a, U2RstECN, PrtCapDir 2
        regs.insert(DCFG, 0x0008_0000);
        regs.insert(DCTL, 0x00f0_0000);
        regs.insert(DSTS, 0x0052_0000);
        regs.insert(GUSB2PHYCFG, 0x4010_2400);
        Self {
            regs,
            evs: Vec::new(),
            clock_us: 0,
            queue: VecDeque::new(),
            ev_pos: 0,
            count: 0,
            active: [None; 4],
            cmds: Vec::new(),
            script: VecDeque::new(),
            stage: Stage::Idle,
            current: None,
            controls: Vec::new(),
            bulk_in: Vec::new(),
            bulk_out_sizes: Vec::new(),
            full_speed: false,
            max_idle: 10_000,
            idle: 0,
            never_halt: false,
            stuck_reset: false,
            stale_events: 0,
            allow_idle_end: false,
        }
    }
}

pub struct Plat {
    pub bring_ups: usize,
    pub flushes: usize,
}
impl Dwc3Platform for Plat {
    fn bring_up<IO: RegIo>(&mut self, _io: &mut IO) -> Result<(), PhyTimeout> {
        self.bring_ups += 1;
        Ok(())
    }
    fn flush_dcache(&mut self, _: usize, _: usize) {
        self.flushes += 1;
    }
    fn invalidate_dcache(&mut self, _: usize, _: usize) {}
}

pub fn plat() -> Plat {
    Plat {
        bring_ups: 0,
        flushes: 0,
    }
}

fn trb_at(a: usize) -> *mut dwc3_udc::Trb {
    a as *mut dwc3_udc::Trb
}

impl Model {
    fn reg(&self, o: usize) -> u32 {
        self.regs.get(&o).copied().unwrap_or(0)
    }

    fn deliver(&mut self, ev: u32) {
        self.queue.push_back(ev);
    }

    fn flush_queue(&mut self) {
        let base = ((self.reg(GEVNTADRHI) as u64) << 32 | self.reg(GEVNTADRLO) as u64) as usize;
        let size = (self.reg(GEVNTSIZ) & 0xffff) as usize;
        assert!(base != 0 && size != 0, "event buffer not set up");
        while let Some(ev) = self.queue.pop_front() {
            // SAFETY: the driver's event buffer (a leaked Dma in the test).
            unsafe { ((base + self.ev_pos) as *mut u32).write_volatile(ev) };
            self.ev_pos = (self.ev_pos + 4) % size;
            self.count += 4;
        }
    }

    fn trb(&self, phys: usize) -> Option<(usize, dwc3_udc::Trb)> {
        self.active[phys].map(|a| (a, unsafe { trb_at(a).read_volatile() }))
    }

    fn complete(&mut self, phys: usize, left: u32, status: u32) {
        let (a, mut t) = self.trb(phys).expect("no active transfer");
        assert!(t.ctrl & trb::HWO != 0, "TRB not owned by the controller");
        t.size = (t.size & !trb::SIZE_MASK) | left;
        t.ctrl &= !trb::HWO;
        unsafe { trb_at(a).write_volatile(t) };
        self.active[phys] = None;
        self.deliver(event::ep(phys as u32, event::XFERCOMPLETE, status));
    }

    fn buf(t: &dwc3_udc::Trb) -> usize {
        ((t.bph as u64) << 32 | t.bpl as u64) as usize
    }

    /// Make one step of host progress; false when the host waits for the
    /// device.
    fn step(&mut self) -> bool {
        let running = self.reg(DCTL) & DCTL_RUN_STOP != 0;
        if !running {
            return false;
        }
        match self.stage {
            Stage::AfterSetup { length, dir_in } => {
                if self.current.as_ref().is_some_and(|c| c.stalled) {
                    self.finish();
                    return true;
                }
                if length == 0 {
                    self.deliver(event::ep(
                        1,
                        event::XFERNOTREADY,
                        event::STATUS_CONTROL_STATUS,
                    ));
                    self.stage = Stage::Status(1);
                } else if dir_in {
                    self.stage = Stage::DataIn;
                } else {
                    panic!("the model has no OUT data stages");
                }
                return true;
            }
            Stage::DataIn => {
                if self.current.as_ref().is_some_and(|c| c.stalled) {
                    self.finish();
                    return true;
                }
                let Some((_, t)) = self.trb(1) else {
                    return false;
                };
                assert_eq!(t.ctrl & (0x3f << 4), trb::CONTROL_DATA);
                let n = (t.size & trb::SIZE_MASK) as usize;
                let p = Self::buf(&t) as *const u8;
                let data: Vec<u8> = (0..n)
                    .map(|i| unsafe { p.add(i).read_volatile() })
                    .collect();
                self.current.as_mut().unwrap().data = data;
                self.complete(1, 0, 0);
                self.deliver(event::ep(
                    0,
                    event::XFERNOTREADY,
                    event::STATUS_CONTROL_STATUS,
                ));
                self.stage = Stage::Status(0);
                return true;
            }
            Stage::Status(phys) => {
                if self.current.as_ref().is_some_and(|c| c.stalled) {
                    self.finish();
                    return true;
                }
                let Some((_, t)) = self.trb(phys) else {
                    return false;
                };
                let ty = (t.ctrl >> 4) & 0x3f;
                assert!(ty == 3 || ty == 4, "status stage TRB type {ty}");
                self.current.as_mut().unwrap().status_type = ty;
                self.complete(phys, 0, 0);
                self.finish();
                return true;
            }
            Stage::Idle => {}
        }
        let Some(h) = self.script.front().cloned() else {
            return false;
        };
        match h {
            Host::Reset => {
                self.script.pop_front();
                let spd = if self.full_speed { 1 } else { 0 };
                let v = self.reg(DSTS) & !7;
                self.regs.insert(DSTS, v | spd);
                self.deliver(event::dev(event::DEV_RESET));
                self.deliver(event::dev(event::DEV_CONNECT_DONE));
                true
            }
            Host::Unplug => {
                self.script.pop_front();
                self.deliver(event::dev(event::DEV_DISCONNECT));
                true
            }
            Host::Setup(s) => {
                let Some((_, t)) = self.trb(0) else {
                    return false;
                };
                if t.ctrl & (0x3f << 4) != trb::CONTROL_SETUP {
                    return false;
                }
                self.script.pop_front();
                let p = Self::buf(&t) as *mut u8;
                for (i, b) in s.iter().enumerate() {
                    unsafe { p.add(i).write_volatile(*b) };
                }
                self.current = Some(Control {
                    setup: s,
                    data: Vec::new(),
                    stalled: false,
                    status_type: 0,
                });
                self.complete(0, 0, 0);
                self.stage = Stage::AfterSetup {
                    length: u16::from_le_bytes([s[6], s[7]]),
                    dir_in: s[0] & 0x80 != 0,
                };
                true
            }
            Host::BulkOut(mut data) => {
                let Some((_, t)) = self.trb(2) else {
                    return false;
                };
                let size = t.size & trb::SIZE_MASK;
                self.bulk_out_sizes.push(size);
                let n = data.len().min(size as usize);
                let p = Self::buf(&t) as *mut u8;
                for (i, b) in data[..n].iter().enumerate() {
                    unsafe { p.add(i).write_volatile(*b) };
                }
                data.drain(..n);
                if data.is_empty() {
                    self.script.pop_front();
                } else {
                    *self.script.front_mut().unwrap() = Host::BulkOut(data);
                }
                let status = if (n as u32) < size {
                    event::STATUS_SHORT
                } else {
                    0
                };
                self.complete(2, size - n as u32, status);
                true
            }
            Host::BulkIn => {
                let Some((_, t)) = self.trb(3) else {
                    return false;
                };
                self.script.pop_front();
                let n = (t.size & trb::SIZE_MASK) as usize;
                let p = Self::buf(&t) as *const u8;
                self.bulk_in.push(
                    (0..n)
                        .map(|i| unsafe { p.add(i).read_volatile() })
                        .collect(),
                );
                self.complete(3, 0, 0);
                true
            }
        }
    }

    fn finish(&mut self) {
        let c = self.current.take().unwrap();
        self.controls.push(c);
        self.stage = Stage::Idle;
    }

    fn gevntcount(&mut self) -> u32 {
        if self.reg(GEVNTADRLO) == 0 && self.reg(GEVNTADRHI) == 0 {
            return std::mem::take(&mut self.stale_events) * 4;
        }
        if self.count == 0 {
            loop {
                if !self.queue.is_empty() {
                    self.flush_queue();
                    self.idle = 0;
                    break;
                }
                if !self.step() {
                    self.idle += 1;
                    if self.idle > self.max_idle && !self.allow_idle_end {
                        panic!(
                            "host model stuck: stage {:?}, next {:?}, active {:?}",
                            self.stage,
                            self.script.front(),
                            self.active
                        );
                    }
                    break;
                }
            }
        }
        self.count
    }

    fn command(&mut self, phys: usize, v: u32) {
        let cmd = v & 0xf;
        let par = [
            self.reg(depcmdpar0(phys)),
            self.reg(depcmdpar1(phys)),
            self.reg(depcmdpar2(phys)),
        ];
        self.cmds.push((phys, v & !DEPCMD_CMDACT, par));
        let mut back = v & !DEPCMD_CMDACT;
        match cmd {
            DEPCMD_STARTTRANSFER => {
                assert!(
                    self.active[phys].is_none(),
                    "transfer already active on {phys}"
                );
                let a = ((par[0] as u64) << 32 | par[1] as u64) as usize;
                self.active[phys] = Some(a);
                back = (back & 0xffff) | ((phys as u32 + 1) << 16);
            }
            DEPCMD_ENDTRANSFER => {
                assert_eq!((v >> 16) & 0x7f, phys as u32 + 1, "wrong XferRscIdx");
                self.active[phys] = None;
            }
            DEPCMD_SETSTALL => {
                if let Some(c) = self.current.as_mut() {
                    c.stalled = true;
                }
            }
            _ => {}
        }
        self.regs.insert(depcmd(phys), back);
    }

    /// Endpoint commands of one type, in order.
    pub fn cmds_of(&self, cmd: u32) -> Vec<(usize, u32, [u32; 3])> {
        self.cmds
            .iter()
            .filter(|c| c.1 & 0xf == cmd)
            .copied()
            .collect()
    }
}

impl RegIo for Model {
    fn read32(&mut self, addr: usize) -> u32 {
        let o = addr - BASE;
        let v = match o {
            GEVNTCOUNT => self.gevntcount(),
            DSTS => {
                let mut v = self.reg(DSTS) & !DSTS_DEVCTRLHLT;
                let halted = self.reg(DCTL) & DCTL_RUN_STOP == 0 && self.stale_events == 0;
                if halted && !self.never_halt {
                    v |= DSTS_DEVCTRLHLT;
                }
                v
            }
            DCTL => {
                let v = self.reg(DCTL);
                if self.stuck_reset {
                    v
                } else {
                    v & !DCTL_CSFTRST
                }
            }
            _ => self.reg(o),
        };
        self.evs.push(Ev::R(32, addr as u64, v));
        v
    }
    fn write32(&mut self, addr: usize, val: u32) {
        self.evs.push(Ev::W(32, addr as u64, val));
        let o = addr - BASE;
        match o {
            GEVNTCOUNT => {
                self.count = self.count.saturating_sub(val);
                return;
            }
            o if (0xc80c..0xc80c + 0x10 * 16).contains(&o) && (o - 0xc80c) % 0x10 == 0 => {
                let phys = (o - 0xc80c) / 0x10;
                if val & DEPCMD_CMDACT != 0 {
                    self.command(phys, val);
                    return;
                }
            }
            _ => {}
        }
        self.regs.insert(o, val);
    }
    fn read16(&mut self, addr: usize) -> u16 {
        self.read32(addr) as u16
    }
    fn write16(&mut self, addr: usize, val: u16) {
        self.write32(addr, val as u32)
    }
    fn udelay(&mut self, us: u32) {
        self.evs.push(Ev::Delay(us));
        self.clock_us += us as u64;
    }
    fn now_us(&mut self) -> u64 {
        self.clock_us += 1;
        self.clock_us
    }
    fn log(&mut self, args: fmt::Arguments) {
        self.evs.push(Ev::Print(args.to_string()));
    }
}

/// SETUP packets.
pub fn setup(rt: u8, req: u8, value: u16, index: u16, length: u16) -> [u8; 8] {
    let [vl, vh] = value.to_le_bytes();
    let [il, ih] = index.to_le_bytes();
    let [ll, lh] = length.to_le_bytes();
    [rt, req, vl, vh, il, ih, ll, lh]
}

/// What Linux does to enumerate a device, roughly: reset, device
/// descriptor (64), reset, address, device descriptor, configuration
/// (9, then full), strings, SET_CONFIGURATION(1).
pub fn enumeration() -> Vec<Host> {
    vec![
        Host::Reset,
        Host::Setup(setup(0x80, 6, 0x0100, 0, 64)),
        Host::Reset,
        Host::Setup(setup(0x00, 5, 7, 0, 0)),
        Host::Setup(setup(0x80, 6, 0x0100, 0, 18)),
        Host::Setup(setup(0x80, 6, 0x0200, 0, 9)),
        Host::Setup(setup(0x80, 6, 0x0200, 0, 32)),
        Host::Setup(setup(0x80, 6, 0x0300, 0, 255)),
        Host::Setup(setup(0x80, 6, 0x0302, 0x0409, 255)),
        Host::Setup(setup(0x00, 9, 1, 0, 0)),
    ]
}

pub fn dma() -> &'static mut dwc3_udc::Dma {
    Box::leak(Box::new(dwc3_udc::Dma::new()))
}
