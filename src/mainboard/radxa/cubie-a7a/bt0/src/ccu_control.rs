// SPDX-License-Identifier: GPL-2.0-only
//! Production early clock control, also compiled by host fault controls.
//! Original bounded error handling around the existing A733 programming
//! sequence. Register fields follow A733 User Manual1.00 sections4.1/5.1.
//! Software budgets are not physical timing/operating-point acceptance.

pub const CCU: usize = 0x0200_2000;
pub const CPU: usize = 0x0887_0000;
pub const CPU_PLLS: [usize; 3] = [CPU + 0x1000, CPU + 0x2000, CPU + 0x3000];
pub const SLOW_SELECT: usize = CPU + 0x3034;
pub const NSI: usize = CCU + 0x580;
pub const SOURCE_MASK: u32 = 7 << 24;
pub const UPDATE: u32 = 1 << 26;
pub const LOCK: u32 = 1 << 28;
pub const OUTPUT: u32 = 1 << 27;
pub const NSI_UPDATE: u32 = 1 << 27;
pub const POLL_READS: u32 = 1_000_000;
pub const POLL_TICKS: u64 = 24_000; // 1ms at the established SYS_CLK24M.
pub const DELAY_READS: u32 = 1_000_000;
const ENABLE: u32 = 1 << 31;
const LDO: u32 = 1 << 30;
const LOCK_ENABLE: u32 = 1 << 29;
const FACTORS: u32 = (3 << 20) | (15 << 16) | (255 << 8) | 15;
const REF_FACTORS_MASK: u32 = (127 << 16) | (255 << 8) | (1 << 1);
const REF_26M: u32 = (103 << 16) | (95 << 8);
const REFERENCE_PLLS: [usize; 13] = [
    0x020, 0x0a0, 0x0c0, 0x0e0, 0x120, 0x140, 0x160, 0x220, 0x240, 0x260, 0x280, 0x2a0, 0x2e0,
];

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum Phase {
    Configuration = 1,
    SlowSource,
    Park,
    Update,
    Lock,
    Delay,
    ReferenceLock,
    NsiDivider,
    NsiSource,
    Switch,
    BusAssert,
    BusRelease,
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Error {
    pub phase: Phase,
    pub addr: usize,
    pub last: u32,
    pub reads: u32,
}
pub trait Hw {
    fn read32(&mut self, addr: usize) -> u32;
    fn write32(&mut self, addr: usize, value: u32);
    fn ticks(&mut self) -> u64;
}
fn change<H: Hw>(h: &mut H, a: usize, clear: u32, set: u32) {
    let v = h.read32(a);
    h.write32(a, (v & !clear) | set);
}
fn error(phase: Phase, addr: usize, last: u32, reads: u32) -> Error {
    Error {
        phase,
        addr,
        last,
        reads,
    }
}

/// No timer dependency while establishing its reference clock. Output gate
/// is never cleared: SYS_CLK24M/UART/timer may already depend on it. Failure
/// never newly enables the gate, and does not claim to restore the old PLL.
pub fn reference<H: Hw>(h: &mut H, hosc: u32) -> Result<u32, Error> {
    let found = h.read32(CCU);
    if hosc != 26_000 {
        return Ok(found);
    }
    let on = ENABLE | LDO | LOCK_ENABLE | LOCK | OUTPUT;
    if found & on == on && found & REF_FACTORS_MASK == REF_26M {
        return Ok(found);
    }
    let mut v = (found & !REF_FACTORS_MASK) | REF_26M | LDO;
    h.write32(CCU, v);
    spin(10_000);
    v |= ENABLE | LOCK_ENABLE;
    h.write32(CCU, v);
    let mut last = 0;
    for reads in 1..=POLL_READS {
        last = h.read32(CCU);
        if last & LOCK != 0 {
            h.write32(CCU, v | OUTPUT);
            spin(10_000);
            return Ok(found);
        }
        if reads == POLL_READS {
            return Err(error(Phase::ReferenceLock, CCU, last, reads));
        }
    }
    Err(error(Phase::ReferenceLock, CCU, last, POLL_READS))
}
fn spin(n: u32) {
    for _ in 0..n {
        core::hint::spin_loop();
    }
}

/// Real timer delay with an independent read cap. It either observes the
/// requested elapsed ticks or fails; a frozen clock cannot silently shorten
/// settling or trap execution forever. Caller establishes SYS_CLK24M first.
pub fn delay<H: Hw>(h: &mut H, us: u32) -> Result<(), Error> {
    let start = h.ticks();
    let needed = u64::from(us) * 24;
    if needed == 0 {
        return Ok(());
    }
    for reads in 1..=DELAY_READS {
        let now = h.ticks();
        if now.wrapping_sub(start) >= needed {
            return Ok(());
        }
        if reads == DELAY_READS {
            return Err(error(Phase::Delay, 0, now as u32, reads));
        }
    }
    unreachable!()
}
pub(crate) fn wait<H: Hw>(
    h: &mut H,
    a: usize,
    mask: u32,
    want: u32,
    phase: Phase,
    stable: u32,
) -> Result<(), Error> {
    let start = h.ticks();
    let mut streak = 0;
    for reads in 1..=POLL_READS {
        let v = h.read32(a);
        if v & mask == want {
            streak += 1;
        } else {
            streak = 0;
        }
        if streak == stable {
            return Ok(());
        }
        if reads == POLL_READS || h.ticks().wrapping_sub(start) >= POLL_TICKS {
            return Err(error(phase, a, v, reads));
        }
    }
    unreachable!()
}
/// Mux000 is CPU_SLOW_CLK, not necessarily DCXO. Leave its established
/// independent DCXO/32K/16MRC source untouched; reject reserved selector3.
/// Park ALL CPU/DSU consumers and verify the full3-bit mux before any PLL
/// output/factor changes. In particular incoming sources4/5 cannot survive
/// the old two-bit clear. On failure no PLL output is blindly disabled.
fn park<H: Hw>(h: &mut H) -> Result<(), Error> {
    let slow = h.read32(SLOW_SELECT);
    if slow & 3 == 3 {
        return Err(error(Phase::SlowSource, SLOW_SELECT, slow, 1));
    }
    for a in CPU_PLLS {
        change(h, a + 0x1c, SOURCE_MASK, 0);
        let v = h.read32(a + 0x1c);
        if v & SOURCE_MASK != 0 {
            return Err(error(Phase::Park, a + 0x1c, v, 1));
        }
    }
    Ok(())
}
fn n_value(hosc: u32, mhz: u32) -> Result<u32, Error> {
    if ![19_200, 24_000, 26_000].contains(&hosc) {
        return Err(error(Phase::Configuration, 0, hosc, 0));
    }
    let n = u64::from(mhz) * 1000 / u64::from(hosc);
    // Field fit only, not a characterized VCO, voltage or stable rate claim.
    if !(1..=255).contains(&n) {
        return Err(error(Phase::Configuration, 0, mhz, 0));
    }
    Ok(n as u32)
}
fn locked<H: Hw>(h: &mut H, a: usize) -> Result<(), Error> {
    wait(h, a, UPDATE, 0, Phase::Update, 1)?;
    wait(h, a, LOCK, LOCK, Phase::Lock, 3)
}
fn enable<H: Hw>(h: &mut H, a: usize, n: u32) -> Result<(), Error> {
    change(h, a, 0, LDO);
    change(h, a, 0, OUTPUT);
    change(h, a, FACTORS, n << 8);
    delay(h, 20)?;
    change(h, a, 0, ENABLE);
    change(h, a, 0, LOCK_ENABLE);
    change(h, a, 0, UPDATE);
    locked(h, a)?;
    delay(h, 20)
}
fn retune_one<H: Hw>(h: &mut H, a: usize, n: u32) -> Result<(), Error> {
    // park() has verified every known CPU/DSU consumer before any gate change.
    change(h, a, OUTPUT | LOCK_ENABLE, 0);
    change(h, a, FACTORS, n << 8);
    delay(h, 20)?;
    change(h, a, 0, LOCK_ENABLE);
    change(h, a, 0, UPDATE);
    locked(h, a)?;
    delay(h, 20)?;
    change(h, a, 0, OUTPUT);
    delay(h, 20)
}
fn switch<H: Hw>(h: &mut H, a: usize) -> Result<(), Error> {
    change(h, a + 0x1c, SOURCE_MASK | (3 << 16), 3 << 24);
    let v = h.read32(a + 0x1c);
    if v & (SOURCE_MASK | (3 << 16)) != 3 << 24 {
        return Err(error(Phase::Switch, a + 0x1c, v, 1));
    }
    delay(h, 20)
}
pub fn init<H: Hw>(h: &mut H, hosc: u32) -> Result<(), Error> {
    let (cpu, dsu) = if hosc == 26_000 {
        (1014, 780)
    } else {
        (1008, 744)
    };
    let ns = [
        n_value(hosc, cpu)?,
        n_value(hosc, cpu)?,
        n_value(hosc, dsu)?,
    ];
    park(h)?;
    for offset in REFERENCE_PLLS {
        change(h, CCU + offset, 0, 1 << 24);
    }
    for (a, n) in CPU_PLLS.into_iter().zip(ns) {
        enable(h, a, (480_000u32).div_ceil(hosc))?;
        retune_one(h, a, n)?;
    }
    delay(h, 20)?;
    for a in CPU_PLLS {
        switch(h, a)?;
    }
    change(h, CCU + 0x518, (3 << 24) | 31, 0);
    change(h, NSI, (1 << 31) | 31, NSI_UPDATE);
    wait(h, NSI, NSI_UPDATE, 0, Phase::NsiDivider, 1)?;
    change(h, NSI, 7 << 24, (3 << 24) | (1 << 31) | NSI_UPDATE);
    wait(h, NSI, NSI_UPDATE, 0, Phase::NsiSource, 1)
}
/// Rates in CPU_L/CPU_B/DSU order. Validate every factor before any write;
/// park all clusters, retune DSU first, and switch only after all locks pass.
/// Failure leaves verified parked consumers or the already switched prefix;
/// no blanket gate-off or rollback to an unverified old PLL is attempted.
pub fn retune<H: Hw>(h: &mut H, hosc: u32, mhz: [u32; 3]) -> Result<(), Error> {
    let ns = [
        n_value(hosc, mhz[0])?,
        n_value(hosc, mhz[1])?,
        n_value(hosc, mhz[2])?,
    ];
    park(h)?;
    for i in [2, 0, 1] {
        retune_one(h, CPU_PLLS[i], ns[i])?;
    }
    for i in [2, 0, 1] {
        switch(h, CPU_PLLS[i])?;
    }
    Ok(())
}

/// Preserve the existing peripheral bus reset/gate sequence, but require both
/// settling intervals to complete. Failure leaves the last programmed state;
/// no speculative rollback or peripheral readiness guarantee is implied.
pub fn bus_enable<H: Hw>(h: &mut H, bgr: usize) -> Result<(), Error> {
    change(h, bgr, 1 << 16, 0);
    change(h, bgr, 1, 0);
    delay(h, 5).map_err(|e| Error {
        phase: Phase::BusAssert,
        addr: bgr,
        ..e
    })?;
    change(h, bgr, 0, 1 << 16);
    change(h, bgr, 0, 1);
    delay(h, 5).map_err(|e| Error {
        phase: Phase::BusRelease,
        addr: bgr,
        ..e
    })
}
