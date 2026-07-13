// SPDX-License-Identifier: GPL-2.0-only
//! A controller init sequence as data, run with bounded polls and a log
//! line per step.

use regio::{wait, RegIo};

/// One step (absolute register addresses, computed by the board).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Step {
    Write {
        addr: usize,
        val: u32,
    },
    /// Read-modify-write: clear, then set.
    Modify {
        addr: usize,
        clear: u32,
        set: u32,
    },
    /// Wait until `(read & mask) == want`, at most `timeout_us`.
    Poll {
        addr: usize,
        mask: u32,
        want: u32,
        timeout_us: u64,
        what: &'static str,
    },
    Delay {
        us: u32,
    },
    /// A mode-register write through the board's controller.
    Mrw {
        rank: u32,
        mr: u32,
        val: u32,
    },
    /// A named point in the log.
    Note(&'static str),
    /// Read-modify-write of a controller field with its programming mode
    /// ([`crate::dwc::ProgMode`]); [`run_checked`] enforces the mode.
    Field {
        addr: usize,
        clear: u32,
        set: u32,
        mode: crate::dwc::ProgMode,
        what: &'static str,
    },
    /// The preceding steps established these controller states.
    Assert(Conditions),
    /// These states no longer hold.
    Clear(Conditions),
    /// SWCTL.sw_done = 0: open quasi-dynamic programming.
    SwDoneClear {
        swctl: usize,
    },
    /// SWCTL.sw_done = 1, then wait for SWSTAT.sw_done_ack.
    SwDoneSet {
        swctl: usize,
        swstat: usize,
        timeout_us: u64,
    },
    /// Toggle RFSHCTL3.refresh_update_level after refresh-related writes.
    RefreshUpdate {
        rfshctl3: usize,
    },
}

/// Mode-register writes for [`Step::Mrw`] (the controller's MR path).
pub trait MrPort<H: RegIo + ?Sized> {
    fn mrw(&mut self, io: &mut H, rank: u32, mr: u32, val: u32) -> Result<(), &'static str>;
}

/// A board without MR steps.
pub struct NoMr;
impl<H: RegIo + ?Sized> MrPort<H> for NoMr {
    fn mrw(&mut self, _: &mut H, _: u32, _: u32, _: u32) -> Result<(), &'static str> {
        Err("no mode-register path")
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct StepError {
    /// Index of the failing step.
    pub index: usize,
    pub what: &'static str,
}

/// Run `steps` in order; stop at the first failure (poll timeout or MR
/// error) and name it. With `log`, every step except writes and modifies
/// is logged (`[dram-ctrl] i: ...`), so a hang or timeout points at a step.
pub fn run<H: RegIo + ?Sized>(
    io: &mut H,
    steps: &[Step],
    mr: &mut impl MrPort<H>,
    log: bool,
) -> Result<usize, StepError> {
    for (i, s) in steps.iter().enumerate() {
        exec(io, i, s, mr, log)?;
    }
    Ok(steps.len())
}

/// Run one step (`i`: its index, for the log and errors).
fn exec<H: RegIo + ?Sized>(
    io: &mut H,
    i: usize,
    s: &Step,
    mr: &mut impl MrPort<H>,
    log: bool,
) -> Result<(), StepError> {
    match *s {
        Step::Write { addr, val } => io.write32(addr, val),
        Step::Modify { addr, clear, set } => io.clrset32(addr, clear, set),
        Step::Poll {
            addr,
            mask,
            want,
            timeout_us,
            what,
        } => {
            if log {
                io.log(format_args!("[dram-ctrl] {i}: wait {what}"));
            }
            wait(io, what, timeout_us, |io| io.read32(addr) & mask == want).map_err(|t| {
                StepError {
                    index: i,
                    what: t.0,
                }
            })?;
        }
        Step::Delay { us } => io.udelay(us),
        Step::Mrw { rank, mr: m, val } => {
            if log {
                io.log(format_args!(
                    "[dram-ctrl] {i}: MRW rank {rank} MR{m} = {val:#04x}"
                ));
            }
            mr.mrw(io, rank, m, val)
                .map_err(|what| StepError { index: i, what })?;
        }
        Step::Note(n) => {
            if log {
                io.log(format_args!("[dram-ctrl] {i}: {n}"));
            }
        }
        Step::Field {
            addr, clear, set, ..
        } => io.clrset32(addr, clear, set),
        Step::Assert(_) | Step::Clear(_) => {}
        Step::SwDoneClear { swctl } => io.write32(swctl, 0),
        Step::SwDoneSet {
            swctl,
            swstat,
            timeout_us,
        } => {
            io.write32(swctl, 1);
            wait(io, "SWSTAT.sw_done_ack", timeout_us, |io| {
                io.read32(swstat) & 1 != 0
            })
            .map_err(|t| StepError {
                index: i,
                what: t.0,
            })?;
        }
        Step::RefreshUpdate { rfshctl3 } => {
            let v = io.read32(rfshctl3);
            io.write32(
                rfshctl3,
                v ^ crate::dwc::bits::RFSHCTL3_REFRESH_UPDATE_LEVEL,
            );
        }
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// Programming-mode checked steps (DesignWare DDRCTL, crate::dwc).

/// Controller states a board has established ([`Step::Assert`]).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Conditions(pub u8);

impl Conditions {
    /// The controller is held in reset: every field may be written.
    pub const RESET: Conditions = Conditions(1 << 0);
    /// Quasi-dynamic group 1: no DFI read/write traffic.
    pub const GROUP1: Conditions = Conditions(1 << 1);
    /// Group 2: DRAM in self-refresh, deep power-down or MPSM.
    pub const GROUP2: Conditions = Conditions(1 << 2);
    /// Group 3: controller empty.
    pub const GROUP3: Conditions = Conditions(1 << 3);
    /// Group 4: writing the shadow set of the inactive frequency.
    pub const GROUP4: Conditions = Conditions(1 << 4);
    pub const fn or(self, o: Conditions) -> Conditions {
        Conditions(self.0 | o.0)
    }
    pub const fn has(self, o: Conditions) -> bool {
        self.0 & o.0 == o.0
    }
    /// Whether any quasi-dynamic group of `g` has its state established.
    pub const fn allows(self, g: crate::dwc::Groups) -> bool {
        (g.has(1) && self.has(Self::GROUP1))
            || (g.has(2) && self.has(Self::GROUP2))
            || (g.has(3) && self.has(Self::GROUP3))
            || (g.has(4) && self.has(Self::GROUP4))
    }
}

/// What [`run_checked`] tracks between steps (and across step lists).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Guard {
    pub conds: Conditions,
    /// SWCTL.sw_done is 0 (quasi-dynamic programming open).
    pub sw_open: bool,
    /// A refresh-related field was written and not yet followed by
    /// [`Step::RefreshUpdate`].
    pub refresh_pending: bool,
}

/// Run `steps` like [`run`], and check every [`Step::Field`] write against
/// its programming mode first: a static field only in reset; a
/// quasi-dynamic field outside reset only with sw_done open and one of its
/// groups established; refresh-related writes must be followed by a
/// [`Step::RefreshUpdate`] before sw_done is set. Plain writes and modifies
/// are not checked. `guard` carries the state between calls.
pub fn run_checked<H: RegIo + ?Sized>(
    io: &mut H,
    steps: &[Step],
    mr: &mut impl MrPort<H>,
    log: bool,
    guard: &mut Guard,
) -> Result<usize, StepError> {
    use crate::dwc::ProgMode;
    for (i, s) in steps.iter().enumerate() {
        let err = |what| StepError { index: i, what };
        match *s {
            Step::Field { mode, what, .. } => {
                let in_reset = guard.conds.has(Conditions::RESET);
                match mode {
                    ProgMode::Static if !in_reset => {
                        if log {
                            io.log(format_args!("[dram-ctrl] {i}: static {what} outside reset"));
                        }
                        return Err(err("static field written outside reset"));
                    }
                    ProgMode::QuasiDynamic(g) if !in_reset => {
                        if !guard.sw_open {
                            return Err(err("quasi-dynamic field without sw_done = 0"));
                        }
                        if !guard.conds.allows(g) {
                            return Err(err("quasi-dynamic group state not established"));
                        }
                    }
                    ProgMode::DynamicRefresh => guard.refresh_pending = true,
                    _ => {}
                }
            }
            Step::SwDoneSet { .. } if guard.refresh_pending => {
                return Err(err(
                    "refresh-related write without refresh_update_level toggle",
                ));
            }
            _ => {}
        }
        exec(io, i, s, mr, log)?;
        match *s {
            Step::Assert(c) => guard.conds = guard.conds.or(c),
            Step::Clear(c) => guard.conds = Conditions(guard.conds.0 & !c.0),
            Step::SwDoneClear { .. } => guard.sw_open = true,
            Step::SwDoneSet { .. } => guard.sw_open = false,
            Step::RefreshUpdate { .. } => guard.refresh_pending = false,
            _ => {}
        }
    }
    Ok(steps.len())
}

/// Kani proofs (P3-A7): `run_checked` never panics and only lets a field
/// write through when its programming mode allows it.
#[cfg(kani)]
mod proofs {
    use super::*;
    use crate::dwc::{Groups, ProgMode};
    use core::fmt;

    struct AnyIo {
        t: u64,
    }
    impl RegIo for AnyIo {
        fn read32(&mut self, _: usize) -> u32 {
            kani::any()
        }
        fn write32(&mut self, _: usize, _: u32) {}
        fn read16(&mut self, _: usize) -> u16 {
            kani::any()
        }
        fn write16(&mut self, _: usize, _: u16) {}
        fn udelay(&mut self, _: u32) {}
        fn now_us(&mut self) -> u64 {
            self.t = self.t.wrapping_add(1);
            self.t
        }
        fn log(&mut self, _: fmt::Arguments) {}
    }

    fn any_mode() -> ProgMode {
        match kani::any::<u8>() % 4 {
            0 => ProgMode::Static,
            1 => ProgMode::Dynamic,
            2 => ProgMode::DynamicRefresh,
            _ => ProgMode::QuasiDynamic(Groups(kani::any::<u8>() & 0xf)),
        }
    }

    fn any_step() -> Step {
        match kani::any::<u8>() % 7 {
            0 => Step::Field {
                addr: 0x100,
                clear: kani::any(),
                set: kani::any(),
                mode: any_mode(),
                what: "f",
            },
            1 => Step::Assert(Conditions(kani::any::<u8>() & 0x1f)),
            2 => Step::Clear(Conditions(kani::any::<u8>() & 0x1f)),
            3 => Step::SwDoneClear { swctl: 0x320 },
            4 => Step::SwDoneSet {
                swctl: 0x320,
                swstat: 0x324,
                timeout_us: 2,
            },
            5 => Step::RefreshUpdate { rfshctl3: 0x60 },
            _ => Step::Note("n"),
        }
    }

    /// Any three steps from any guard state: no panic; an accepted static
    /// write happened in reset; an accepted quasi-dynamic write in reset or
    /// with sw_done open and one of its groups established; sw_done is
    /// never set over a pending refresh update.
    #[kani::proof]
    #[kani::unwind(6)]
    fn run_checked_enforces_the_modes() {
        let steps = [any_step(), any_step(), any_step()];
        let mut g = Guard {
            conds: Conditions(kani::any::<u8>() & 0x1f),
            sw_open: kani::any(),
            refresh_pending: kani::any(),
        };
        let mut io = AnyIo { t: 0 };
        for s in steps {
            let before = g;
            let r = run_checked(&mut io, &[s], &mut NoMr, false, &mut g);
            match s {
                Step::Field { mode, .. } if r.is_ok() => {
                    let reset = before.conds.has(Conditions::RESET);
                    match mode {
                        ProgMode::Static => assert!(reset),
                        ProgMode::QuasiDynamic(gr) => {
                            assert!(reset || (before.sw_open && before.conds.allows(gr)))
                        }
                        _ => {}
                    }
                }
                Step::SwDoneSet { .. } if before.refresh_pending => assert!(r.is_err()),
                _ => {}
            }
            if r.is_err() {
                break;
            }
        }
    }
}
