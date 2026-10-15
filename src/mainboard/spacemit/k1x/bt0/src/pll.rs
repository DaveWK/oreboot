// SPDX-License-Identifier: GPL-2.0-only
//! Ownership of the K1 system PLLs PLL1-3 (APBS, `0xd409_0000`).
//!
//! Nothing before bt0 except the mask BootROM touches these PLLs, and the
//! next stages (OpenSBI, EDK2) do not program them either. The stock Linux
//! clock driver (`drivers/clk/spacemit/ccu_pll.c`, `ccu_pll_init`) accepts a
//! PLL only if its SWCR1 and SWCR3[30:0] equal an entry of its rate table
//! (`drivers/clk/spacemit/ccu-k1.c`); otherwise it gates the PLL and writes
//! the first table entry. Gating a PLL that the CPU or the bus runs from
//! stops the SoC without a message. bt0 therefore puts every PLL into a state
//! from that table before any OS sees it, and reports what it found.
//!
//! Targets, from the GPL sources that define them. "Vendor" is SpacemiT's
//! uboot-2022.10 as shipped by Orange Pi (orangepi-xunlong/u-boot-orangepi
//! `v2022.10-ky` `89bff4a7e4`, where the K1 is named "Ky X1"):
//! - PLL1 2457.6 MHz `0x0050dd64`/`0x330ccccd`: the only PLL1 entry in Linux
//!   `pll1_rate_tbl`; the vendor clock driver (`drivers/clk/ky/ccu-x1.c`,
//!   GPL-2.0-only) has no PLL1 table and treats PLL1 as fixed at this rate.
//! - PLL2 3000 MHz `0x0050dd66`/`0x3fe00000`: the only PLL2 entry in Linux
//!   `pll2_rate_tbl`, and the first entry of the vendor `pll2_rate_tbl`.
//! - PLL3 3200 MHz `0x0050dd67`/`0x43eaaaab`: in Linux `pll3_rate_tbl` and the
//!   vendor table; the vendor SPL (`board/ky/x1/spl.c`, GPL-2.0+,
//!   `raise_cpu_frequency`) only sets the gate bit of PLL3 ("enable
//!   PLL3(3200Mhz)") and PLL3_DIV2, so the BootROM leaves these values; its
//!   `pll3_d2` (1.6 GHz) feeds the CPU clusters' top operating point.
//!   FreeBSD on our chain read exactly these PLL3 values at boot (R2S,
//!   October 8 2026).
//! The vendor stages program SWCR1/SWCR3 of no PLL; neither does EDK2.
//!
//! Policy: a PLL that already matches is left alone. A gated PLL that does
//! not match is reprogrammed while it stays gated (the parameters may only
//! change while the PLL is off; Linux marks these clocks CLK_SET_RATE_GATE)
//! and read back. A running PLL that does not match is never touched here:
//! the CPU, UART and buses may be running from it. It is reported, because
//! the kernel would gate it.

use log::println;

pub const APBS_BASE: usize = 0xd409_0000;
pub const MPMU_BASE: usize = 0xd405_0000;
/// PLL lock status (Linux `MPMU_POSR`), bits 27..29 = PLL1..3 locked.
pub const MPMU_POSR: usize = MPMU_BASE + 0x0010;
/// MPMU clock gates (Linux `MPMU_ACGR`); reported only.
pub const MPMU_ACGR: usize = MPMU_BASE + 0x1024;
/// CPU cluster 0/1 clock control (Linux `APMU_CPU_C0/C1_CLK_CTRL`); reported only.
pub const APMU_CPU_C0_CLK_CTRL: usize = 0xd428_2800 + 0x38c;
pub const APMU_CPU_C1_CLK_CTRL: usize = 0xd428_2800 + 0x390;

/// SWCR3 bit 31 gates the PLL; bits 30:0 hold the divider.
pub const SWCR3_EN: u32 = 1 << 31;
pub const SWCR3_MASK: u32 = !SWCR3_EN;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Pll {
    pub name: &'static str,
    pub swcr1: usize,
    pub swcr2: usize,
    pub swcr3: usize,
    pub lock_bit: u32,
    pub khz: u32,
    pub want_swcr1: u32,
    pub want_swcr3: u32,
}

pub const PLLS: [Pll; 3] = [
    Pll {
        name: "PLL1",
        swcr1: APBS_BASE + 0x100,
        swcr2: APBS_BASE + 0x104,
        swcr3: APBS_BASE + 0x108,
        lock_bit: 1 << 27,
        khz: 2_457_600,
        want_swcr1: 0x0050_dd64,
        want_swcr3: 0x330c_cccd,
    },
    Pll {
        name: "PLL2",
        swcr1: APBS_BASE + 0x118,
        swcr2: APBS_BASE + 0x11c,
        swcr3: APBS_BASE + 0x120,
        lock_bit: 1 << 28,
        khz: 3_000_000,
        want_swcr1: 0x0050_dd66,
        want_swcr3: 0x3fe0_0000,
    },
    Pll {
        name: "PLL3",
        swcr1: APBS_BASE + 0x124,
        swcr2: APBS_BASE + 0x128,
        swcr3: APBS_BASE + 0x12c,
        lock_bit: 1 << 29,
        khz: 3_200_000,
        want_swcr1: 0x0050_dd67,
        want_swcr3: 0x43ea_aaab,
    },
];

pub trait Regs {
    fn read(&mut self, address: usize) -> u32;
    fn write(&mut self, address: usize, value: u32);
}

pub struct Mmio;
impl Regs for Mmio {
    fn read(&mut self, address: usize) -> u32 {
        util::mmio::read32(address)
    }
    fn write(&mut self, address: usize, value: u32) {
        util::mmio::write32(address, value)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Action {
    /// SWCR1/SWCR3 already equal the target.
    Kept,
    /// Gated and different: target written and read back.
    Programmed,
    /// Running and different: left as is (the kernel would gate it).
    RunningMismatch,
    /// Gated, target written, but the readback differs.
    ReadbackMismatch,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct State {
    pub swcr1: u32,
    pub swcr2: u32,
    pub swcr3: u32,
}

impl State {
    pub fn enabled(&self) -> bool {
        self.swcr3 & SWCR3_EN != 0
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Report {
    pub pll: Pll,
    pub before: State,
    pub after: State,
    pub action: Action,
}

impl Report {
    /// True if the stock kernel's `ccu_pll_init` will accept this PLL as is.
    pub fn kernel_accepts(&self) -> bool {
        matches(&self.pll, &self.after)
    }
}

pub fn matches(p: &Pll, s: &State) -> bool {
    s.swcr1 == p.want_swcr1 && s.swcr3 & SWCR3_MASK == p.want_swcr3
}

fn read_state(r: &mut impl Regs, p: &Pll) -> State {
    State {
        swcr1: r.read(p.swcr1),
        swcr2: r.read(p.swcr2),
        swcr3: r.read(p.swcr3),
    }
}

/// Bring one PLL into its target state under the policy above.
pub fn own_one(r: &mut impl Regs, p: &Pll) -> Report {
    let before = read_state(r, p);
    if matches(p, &before) {
        return Report {
            pll: *p,
            before,
            after: before,
            action: Action::Kept,
        };
    }
    if before.enabled() {
        return Report {
            pll: *p,
            before,
            after: before,
            action: Action::RunningMismatch,
        };
    }
    // Gated: SWCR1 first, then the divider with the gate bit left clear,
    // the order ccu_pll_update_param uses.
    r.write(p.swcr1, p.want_swcr1);
    r.write(p.swcr3, (before.swcr3 & SWCR3_EN) | p.want_swcr3);
    let after = read_state(r, p);
    let action = if matches(p, &after) && !after.enabled() {
        Action::Programmed
    } else {
        Action::ReadbackMismatch
    };
    Report {
        pll: *p,
        before,
        after,
        action,
    }
}

pub fn own(r: &mut impl Regs) -> [Report; 3] {
    [
        own_one(r, &PLLS[0]),
        own_one(r, &PLLS[1]),
        own_one(r, &PLLS[2]),
    ]
}

/// Program and log PLL1-3; returns false if the kernel would reprogram any.
pub fn init() -> bool {
    let mut r = Mmio;
    let reports = own(&mut r);
    let posr = r.read(MPMU_POSR);
    let mut all = true;
    for rep in reports.iter() {
        let b = rep.before;
        let a = rep.after;
        let locked = posr & rep.pll.lock_bit != 0;
        println!(
            "[bt0] {} {} kHz: swcr1 {:08x}->{:08x} swcr2 {:08x} swcr3 {:08x}->{:08x} en {} lock {}: {:?}",
            rep.pll.name,
            rep.pll.khz,
            b.swcr1,
            a.swcr1,
            a.swcr2,
            b.swcr3,
            a.swcr3,
            a.enabled() as u8,
            locked as u8,
            rep.action
        );
        all &= rep.kernel_accepts();
    }
    println!(
        "[bt0] POSR {:08x} ACGR {:08x} CPU_C0 {:08x} CPU_C1 {:08x}",
        posr,
        r.read(MPMU_ACGR),
        r.read(APMU_CPU_C0_CLK_CTRL),
        r.read(APMU_CPU_C1_CLK_CTRL)
    );
    if !all {
        println!("[bt0] WARNING: a PLL differs from the stock kernel's table; Linux will gate and reprogram it");
    }
    all
}
