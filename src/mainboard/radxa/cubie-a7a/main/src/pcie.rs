//! PCIe Gen3 x1 root complex bring-up (DesignWare PCIe 3.0 RC behind the
//! Cadence combo PHY 1), so the payload only has to enumerate.
//!
//! Sources, in order of authority:
//! - Radxa's vendor BSP (github.com/radxa/allwinner-bsp, cubie-aiot-v1.4.8):
//!   drivers/pcie/pcie-sunxi-{plat,rc}.c and drivers/phy/
//!   sunxi-cadence-combophy.c. The PHY table below is generated from
//!   `sunxi_cadence_phy_pcie_phy_init()` there; v1.4.6 and v1.5.0
//!   (`..._pcie_internal_100m_init`, the A7A's default refclk) are
//!   identical. The writes, their order and the reset/clock order follow
//!   those drivers; only the bounded polls and the logging are new.
//! - The vendor kernel's DT on an A7A (captures/cubie-a7a-vendor-6.6.98):
//!   `allwinner,sunxi-pcie-v300-rc`, DBI 0x06000000 + 0x480000, windows
//!   CFG 0x20000000 (16 MiB), IO 0x21000000 (16 MiB), MEM 0x22000000
//!   (96 MiB), PE11 power, PE12 wake, PE13 PERST#, max-link-speed 3.
//! - vehoelite/edk2-a733 @ 44aeea5 SunxiPcieDxe and its research notes,
//!   which brought an NVMe up from cold on the Orange Pi 4 Pro (same SoC):
//!   the keyed bus-gate stray bits, the serdes reset after its clocks, the
//!   100 MHz serdes config clock, training at Gen1 first.
//!
//! Rails: DCDC1 (3.3 V, always on), BLDO1 (PCIe 1.8 V) and CLDO5 (SERDES
//! 1.8 V) are on before main runs; bt0 reports BLDO1/CLDO5 (axp.rs
//! BOARD_LDOS) and DCDC1 is the board's main 3.3 V supply.
//!
//! Hazard: a DBI, app-block or PHY access before the power domain, resets
//! and clocks are up hangs the bus with no output. Nothing below touches
//! 0x06xxxxxx before `clocks_on()` and every poll is bounded.
//!
//! On success the link is trained, iATU region 1 maps MEM32 and region 2
//! maps I/O; region 0 is left for the payload to retarget per config
//! access (see `fdt_node`). On failure the controller is put back into
//! reset, PERST# is asserted, the slot rail is switched off and boot goes
//! on without PCIe.

use cubie_a7a_common::time::{ticks, udelay, TIMER_HZ};
use util::mmio::{read16, read32, write16, write32};

use crate::fdt::Fdt;
use crate::ppu;

// --- CCU (vendor ccu-sun60iw2.c) -------------------------------------------
const CCU: usize = 0x0200_2000;
/// AHB/MBUS master gates take a key in the same write; its low bits land
/// on real gates, which edk2-a733 measured against a vendor boot and puts
/// back (`*_STRAY`) unless they were already on.
const AHB_GATE: usize = CCU + 0x05c0;
const AHB_KEY: u32 = 0x0100_00ff;
const AHB_STRAY: u32 = 0x0000_0007;
const AHB_SERDES: u32 = 1 << 8;
const MBUS_GATE: usize = CCU + 0x05e0;
const MBUS_KEY: u32 = 0x4105_5800;
const MBUS_STRAY: u32 = 0x0100_5000;
const MBUS_SERDES: u32 = 1 << 28;
const ITS_BGR: usize = CCU + 0x0574; // gate bit 1, reset bit 16
const PCIE_AUX_CLK: usize = CCU + 0x1380; // sys24M, gate bit 31
const PCIE_AXI_SLV_CLK: usize = CCU + 0x1384; // mux 2 = PERI0 400M, gate bit 31
const PCIE_BGR: usize = CCU + 0x138c; // bit 17 PCIE0_RST, bit 16 PWRUP_RST
const SERDES_CFG_CLK: usize = CCU + 0x13c0; // mux 1 = PERI0 600M, M[4:0], gate bit 31
const SERDES_BGR: usize = CCU + 0x13c4; // bit 16 SERDES_RST
const GATE: u32 = 1 << 31;
/// RTC DCXO serdes clock gates (third reg of the vendor serdes node).
const DCXO_SERDES: usize = 0x0709_016c;
const DCXO_SERDES1: u32 = 1 << 5;

// --- Serdes subsystem and combo PHY 1 ----------------------------------------
const SUBSYS: usize = 0x06c0_0000;
const SUBSYS_PCIE_BGR: usize = SUBSYS + 0x04;
const SUBSYS_DBG_CTL: usize = SUBSYS + 0xf0;
const SUBSYS_COMB1_PIPE: usize = 0x06c0_6000 + 0xc44; // 1 = PCIe
const C1_TOP: usize = 0x06c0_2000;
const C1_PHY: usize = 0x06ca_0000;

// --- PIO bank E (A733 manual 18.14.5: PE_CFG1 0x284, PE_DAT 0x290) -----------
const PE_CFG1: usize = 0x0200_0284; // pins 8..15
const PE_DAT: usize = 0x0200_0290;
const PIN_POWER: u32 = 11;
const PIN_WAKE: u32 = 12;
const PIN_PERST: u32 = 13; // PERST#: low = asserted

// --- DesignWare RC -------------------------------------------------------------
pub const DBI: usize = 0x0600_0000;
pub const DBI_SIZE: usize = 0x0048_0000;
const DBI2: usize = DBI + 0x0010_0000;
const ATU: usize = DBI + 0x0030_0000;
const APP: usize = DBI + 0x0040_0000;
const APP_PHY_CFG: usize = APP + 0x800;
const APP_LTSSM_CTRL: usize = APP + 0xc00;
const LTSSM_EN: u32 = 1 << 0;
const APP_LINK_STAT: usize = APP + 0xe0c;
const LINK_UP: u32 = 0b11; // SMLH | RDLH

const PORT_LINK_CTRL: usize = DBI + 0x710;
const PL_DEBUG0: usize = DBI + 0x728; // [5:0] LTSSM state
const GEN2_CTRL: usize = DBI + 0x80c; // [16:8] width, bit 17 speed change
const SPEED_CHANGE: u32 = 1 << 17;
const MISC_CONTROL_1: usize = DBI + 0x8bc; // bit 0 DBI_RO_WR_EN

// --- Windows (vendor DT `ranges`), CPU address == PCI address except I/O --------
pub const CFG_BASE: u64 = 0x2000_0000;
/// The part of the 16 MiB config window iATU region 0 decodes.
pub const CFG_WINDOW: u64 = 0x1_0000;
pub const IO_CPU: u64 = 0x2100_0000;
/// The root port decodes 16-bit I/O only, so I/O space is 0..64 KiB.
pub const IO_SIZE: u64 = 0x1_0000;
pub const MEM_BASE: u64 = 0x2200_0000;
pub const MEM_SIZE: u64 = 0x0600_0000;

const ATU_TYPE_MEM: u32 = 0;
const ATU_TYPE_IO: u32 = 2;
const ATU_TYPE_CFG0: u32 = 4;
const ATU_ENABLE: u32 = 1 << 31;

/// Target speed after the Gen1 link is up (the vendor DT's max-link-speed).
const MAX_GEN: u32 = 3;

/// PCIe CEM T_PVPERL: power stable to PERST# release, at least 100 ms. The
/// Cubie A7Z overlay waits 500 ms for its FPC rail; raise this if an
/// adapter here turns out to need the same.
const PERST_HOLD_US: u64 = 100_000;

// The fields are only read through `{:?}` in the log.
#[allow(dead_code)]
#[derive(Debug, Clone, Copy)]
enum Error {
    PowerDomain(ppu::PpuError),
    /// Combo PHY PMA never reported ready (top + 0x900).
    PmaTimeout(u32),
    /// No PCIe express capability in the RC's config space.
    NoExpressCap,
    /// The LTSSM never left Detect: nothing in the slot.
    Empty,
    /// LINK_STAT and the last LTSSM state.
    LinkTimeout(u32, u32),
    /// The endpoint never answered config reads (last value).
    NoEndpoint(u32),
}

#[derive(Clone, Copy)]
struct Link {
    gen: u32,
    width: u32,
    /// Endpoint vendor ID | device ID << 16.
    id: u32,
    class: u32,
}

static mut LINK: Option<Link> = None;

// The A7A's vendor-BSP combo PHY 1 PCIe table. 85 literal writes, generated
// from the BSP source and identical to edk2-a733's independently generated
// table; the read-modify-writes and the PMA poll follow in `phy_init`.
#[derive(Clone, Copy)]
enum Op {
    Top32(u16, u32),
    Phy16(u32, u16),
    /// combo1 top + 0x100 &= !0x30 (lane mode) at this point.
    TopLaneMode,
}
use Op::*;

#[rustfmt::skip]
const PHY_TABLE: &[Op] = &[
    Top32(0x004, 0x0110_0001),
    Phy16(0x00044, 0x001a),
    Phy16(0x00054, 0x0034), Phy16(0x00058, 0x00da),
    Phy16(0x00064, 0x0034), Phy16(0x00068, 0x00da),
    Phy16(0x000c8, 0x0082), Phy16(0x000ca, 0x0082),
    Phy16(0x000e8, 0x001a),
    Phy16(0x00208, 0x0020), Phy16(0x0020a, 0x0007),
    Phy16(0x00218, 0x0020), Phy16(0x0021a, 0x0007),
    Phy16(0x00228, 0x030c), Phy16(0x0022a, 0x0007),
    Phy16(0x00248, 0x0007), Phy16(0x0024a, 0x0003),
    Phy16(0x0024c, 0x000f), Phy16(0x00250, 0x0132),
    Phy16(0x08180, 0x0208), Phy16(0x08184, 0x009c),
    Phy16(0x10088, 0x001a), Phy16(0x1008a, 0x0082),
    Phy16(0x10098, 0x001a), Phy16(0x1009a, 0x0082),
    Phy16(0x08246, 0x0a28),
    TopLaneMode,
    Phy16(0x00128, 0x0004), Phy16(0x00148, 0x0004), Phy16(0x001a8, 0x0004),
    Phy16(0x00348, 0x0509), Phy16(0x00368, 0x0509), Phy16(0x00388, 0x0509),
    Phy16(0x0034a, 0x0f00), Phy16(0x0036a, 0x0f00), Phy16(0x0038a, 0x0f00),
    Phy16(0x0034c, 0x0f08), Phy16(0x0036c, 0x0f08), Phy16(0x0038c, 0x0f08),
    Phy16(0x00120, 0x0180), Phy16(0x00140, 0x0133), Phy16(0x001a0, 0x0133),
    Phy16(0x00122, 0x9d8a), Phy16(0x00142, 0xb13b), Phy16(0x001a2, 0xb13b),
    Phy16(0x00124, 0x0002), Phy16(0x00144, 0x0002), Phy16(0x001a4, 0x0002),
    Phy16(0x00126, 0x0102), Phy16(0x00146, 0x00ce), Phy16(0x001a6, 0x00ce),
    Phy16(0x00340, 0x0022), Phy16(0x00360, 0x0022), Phy16(0x00380, 0x0022),
    Phy16(0x00130, 0x0001), Phy16(0x00150, 0x0001), Phy16(0x001b0, 0x0001),
    Phy16(0x00132, 0x045f), Phy16(0x00152, 0x02f0), Phy16(0x001b2, 0x0399),
    Phy16(0x00134, 0x006b), Phy16(0x00154, 0x0068), Phy16(0x001b4, 0x0068),
    Phy16(0x00136, 0x0004), Phy16(0x00156, 0x0004), Phy16(0x001b6, 0x0004),
    Phy16(0x00108, 0x0104), Phy16(0x00188, 0x0104),
    Phy16(0x0010a, 0x0005), Phy16(0x0018a, 0x0005),
    Phy16(0x0010c, 0x0337), Phy16(0x0018c, 0x0337),
    Phy16(0x00110, 0x3dbe), Phy16(0x00190, 0x3dbe),
    Phy16(0x00104, 0x0003), Phy16(0x00184, 0x0003),
    Phy16(0x00138, 0x0014), Phy16(0x001b8, 0x0014),
    Phy16(0x0013c, 0x0192), Phy16(0x001bc, 0x0192),
    Phy16(0x0013e, 0x0006), Phy16(0x001be, 0x0006),
    Phy16(0x103c0, 0x0000),
    Phy16(0x102e2, 0x0019), Phy16(0x102e4, 0x0019),
    Phy16(0x103fe, 0x0001),
];

fn set32(a: usize, bits: u32) {
    write32(a, read32(a) | bits);
}

fn clr32(a: usize, bits: u32) {
    write32(a, read32(a) & !bits);
}

fn rmw32(a: usize, clr: u32, set: u32) {
    write32(a, (read32(a) & !clr) | set);
}

fn us_since(t: u64) -> u64 {
    (ticks() - t) / (TIMER_HZ / 1_000_000)
}

/// Poll `cond` every `step_us` until it holds or `max_us` passed.
fn wait(max_us: u64, step_us: u64, mut cond: impl FnMut() -> bool) -> bool {
    let t = ticks();
    loop {
        if cond() {
            return true;
        }
        if us_since(t) >= max_us {
            return cond();
        }
        udelay(step_us);
    }
}

/// Drive a PE pin (8..15) as an output; the level is set before the mux so
/// the pad never glitches the other way.
fn pe_out(pin: u32, high: bool) {
    if high {
        set32(PE_DAT, 1 << pin);
    } else {
        clr32(PE_DAT, 1 << pin);
    }
    let shift = (pin - 8) * 4;
    rmw32(PE_CFG1, 0xf << shift, 1 << shift);
}

fn keyed_gate(reg: usize, key: u32, stray: u32, gate: u32) {
    let before = read32(reg);
    write32(reg, before | key | gate);
    let strays = stray & !before;
    if strays != 0 {
        write32(reg, read32(reg) & !strays);
    }
}

/// sunxi_pcie_plat_clk_setup() plus the serdes side of
/// sunxi_cadence_phy_combo1_pcie_init(), up to the PHY table.
fn clocks_on() {
    keyed_gate(AHB_GATE, AHB_KEY, AHB_STRAY, AHB_SERDES);
    keyed_gate(MBUS_GATE, MBUS_KEY, MBUS_STRAY, MBUS_SERDES);
    // serdes config clock: PERI0 600M / 6 = 100 MHz (clk_set_rate 100 MHz)
    rmw32(SERDES_CFG_CLK, (7 << 24) | 0x1f, (1 << 24) | 5);
    set32(SERDES_CFG_CLK, GATE);
    // controller resets first, then aux, then the 400 MHz AXI slave clock;
    // the vendor form (OR in mux 2) is kept: edk2-a733 saw a hang when the
    // field was cleared first.
    set32(PCIE_BGR, (1 << 17) | (1 << 16));
    set32(PCIE_AUX_CLK, GATE);
    let slv = read32(PCIE_AXI_SLV_CLK);
    set32(PCIE_AXI_SLV_CLK, (2 << 24) | GATE);
    set32(ITS_BGR, (1 << 16) | (1 << 1));
    set32(DCXO_SERDES, DCXO_SERDES1);
    // serdes reset only once its clocks run (Linux order)
    set32(SERDES_BGR, 1 << 16);
    udelay(1000);
    set32(SUBSYS_PCIE_BGR, 7 << 16);
    set32(SUBSYS_DBG_CTL, 1 << 29); // no combo1 auto clock gating
    write32(SUBSYS_COMB1_PIPE, 1);
    println!(
        "[main] PCIe: clocks on (AXI slave {slv:#010x} -> {:#010x}, serdes cfg {:#010x})",
        read32(PCIE_AXI_SLV_CLK),
        read32(SERDES_CFG_CLK)
    );
}

fn phy_init() -> Result<(), Error> {
    for op in PHY_TABLE {
        match *op {
            Top32(off, v) => write32(C1_TOP + off as usize, v),
            Phy16(off, v) => write16(C1_PHY + off as usize, v),
            TopLaneMode => clr32(C1_TOP + 0x100, 0x30),
        }
    }
    for off in [0x98, 0xa8, 0xd8] {
        rmw32(C1_PHY + off, 0x3, 0x2);
    }
    set32(C1_TOP, 1);
    set32(C1_TOP + 0x100, 1);
    if !wait(100_000, 100, || read32(C1_TOP + 0x900) & 1 != 0) {
        return Err(Error::PmaTimeout(read32(C1_TOP + 0x900)));
    }
    write16(C1_PHY + 0xa0, 0x270);
    write16(C1_PHY + 0x98, read16(C1_PHY + 0x98) | (1 << 4));
    write16(C1_PHY + 0x18000, read16(C1_PHY + 0x18000) | 1);
    set32(C1_TOP + 0x4, 1 << 28); // reference clock out to the slot
    Ok(())
}

fn find_express_cap() -> Option<usize> {
    let mut ptr = (read16(DBI + 0x34) & 0xff) as usize;
    for _ in 0..48 {
        if ptr == 0 || ptr > 0xfc {
            return None;
        }
        let hdr = read16(DBI + ptr);
        if hdr & 0xff == 0x10 {
            return Some(ptr);
        }
        ptr = (hdr >> 8) as usize;
    }
    None
}

/// sunxi_pcie_host_setup_rc(), training first at Gen1. RC BARs are
/// disabled through their DBI2 masks (the vendor EP driver's BAR_ENABLE
/// register) so the payload sees a plain bridge.
fn setup_rc() -> Result<usize, Error> {
    set32(MISC_CONTROL_1, 1);
    let Some(cap) = find_express_cap() else {
        clr32(MISC_CONTROL_1, 1);
        return Err(Error::NoExpressCap);
    };
    rmw32(DBI + cap + 0x0c, 0xf, MAX_GEN); // LNKCAP: max link speed
    rmw32(DBI + cap + 0x30, 0xf, 1); // LNKCTL2: target Gen1 for training
    rmw32(PORT_LINK_CTRL, 0x3f << 16, 1 << 16); // x1
    rmw32(GEN2_CTRL, 0x1ff << 8, 1 << 8);
    write32(DBI2 + 0x10, 0);
    write32(DBI2 + 0x14, 0);
    write32(DBI + 0x10, 0);
    write32(DBI + 0x14, 0);
    rmw32(DBI + 0x3c, 0xff00, 0x0100); // interrupt pin INTA
    rmw32(DBI + 0x18, 0x00ff_ffff, 0x00ff_0100); // buses 0 / 1 / 0xff
    rmw32(DBI + 0x04, 0xffff, (1 << 0) | (1 << 1) | (1 << 2) | (1 << 8));
    write16(DBI + 0x0a, 0x0604); // class: PCI-to-PCI bridge
    clr32(MISC_CONTROL_1, 1);
    Ok(cap)
}

fn ltssm() -> u32 {
    read32(PL_DEBUG0) & 0x3f
}

fn link_up() -> bool {
    read32(APP_LINK_STAT) & LINK_UP == LINK_UP
}

/// LTSSM on, then wait for SMLH + RDLH. A slot whose LTSSM stays in
/// Detect (states 0/1) for 100 ms is empty; a link that started training
/// gets 1 s, as the vendor's 20 x 100 ms.
fn train() -> Result<(), Error> {
    set32(APP_LTSSM_CTRL, LTSSM_EN);
    let t = ticks();
    let mut seen = false;
    loop {
        if link_up() {
            return Ok(());
        }
        let s = ltssm();
        seen |= s > 1;
        let us = us_since(t);
        if !seen && us >= 100_000 {
            return Err(Error::Empty);
        }
        if us >= 1_000_000 {
            return Err(Error::LinkTimeout(read32(APP_LINK_STAT), s));
        }
        udelay(100);
    }
}

fn link_status(cap: usize) -> (u32, u32) {
    let sta = read16(DBI + cap + 0x12) as u32;
    (sta & 0xf, (sta >> 4) & 0x3f)
}

/// sunxi_pcie_host_speed_change(): raise the target speed and request a
/// directed speed change. If the link does not come back within 100 ms,
/// retrain at Gen1 rather than lose it.
fn speed_up(cap: usize) -> Result<(), Error> {
    if MAX_GEN <= 1 {
        return Ok(());
    }
    set32(MISC_CONTROL_1, 1);
    rmw32(DBI + cap + 0x30, 0xf, MAX_GEN);
    clr32(GEN2_CTRL, SPEED_CHANGE);
    set32(GEN2_CTRL, SPEED_CHANGE);
    clr32(MISC_CONTROL_1, 1);
    let done = wait(20_000, 100, || read32(GEN2_CTRL) & SPEED_CHANGE == 0);
    let back = wait(100_000, 100, || {
        link_up() && read16(DBI + cap + 0x12) & (1 << 11) == 0 // not training
    });
    if back {
        if !done {
            println!("[main] PCIe: speed change bit did not clear");
        }
        return Ok(());
    }
    println!("[main] PCIe: link lost after the Gen{MAX_GEN} change; retraining at Gen1");
    rmw32(DBI + cap + 0x30, 0xf, 1);
    set32(DBI + cap + 0x10, 1 << 5); // LNKCTL: retrain link
    if wait(100_000, 100, link_up) {
        Ok(())
    } else {
        Err(Error::LinkTimeout(read32(APP_LINK_STAT), ltssm()))
    }
}

fn atu_outbound(region: usize, kind: u32, cpu: u64, pci: u64, size: u64) {
    let r = ATU + region * 0x200;
    write32(r + 0x08, cpu as u32);
    write32(r + 0x0c, (cpu >> 32) as u32);
    write32(r + 0x10, (cpu + size - 1) as u32);
    write32(r + 0x14, pci as u32);
    write32(r + 0x18, (pci >> 32) as u32);
    write32(r + 0x00, kind);
    write32(r + 0x04, ATU_ENABLE);
    if !wait(20_000, 1000, || read32(r + 0x04) & ATU_ENABLE != 0) {
        println!("[main] PCIe: iATU region {region} did not enable");
    }
}

/// Read bus 1 device 0 through iATU region 0. After training, devices may
/// answer Configuration Request Retry (seen as vendor 0x0001 or all ones)
/// for up to 1 s.
fn endpoint_id() -> Result<(u32, u32), Error> {
    atu_outbound(0, ATU_TYPE_CFG0, CFG_BASE, 1 << 24, CFG_WINDOW);
    let cfg = CFG_BASE as usize;
    let mut id = 0;
    let ok = wait(1_000_000, 10_000, || {
        id = read32(cfg);
        id != 0xffff_ffff && id != 0 && id & 0xffff != 0x0001
    });
    if !ok {
        return Err(Error::NoEndpoint(id));
    }
    Ok((id, read32(cfg + 0x08) >> 8))
}

fn bring_up(clocked: &mut bool) -> Result<Link, Error> {
    pe_out(PIN_PERST, false);
    pe_out(PIN_POWER, true);
    pe_out(PIN_WAKE, true); // the vendor driver drives WAKE# high too
    let t_power = ticks();

    ppu::power_on(ppu::PCIE).map_err(Error::PowerDomain)?;
    clocks_on();
    *clocked = true;
    phy_init()?;
    println!(
        "[main] PCIe: PHY ready; LTSSM_CTRL {:#x}, PHY_CFG {:#010x}",
        read32(APP_LTSSM_CTRL),
        read32(APP_PHY_CFG)
    );
    clr32(APP_LTSSM_CTRL, LTSSM_EN);

    let held = us_since(t_power);
    if held < PERST_HOLD_US {
        udelay(PERST_HOLD_US - held);
    }
    pe_out(PIN_PERST, true);

    let cap = setup_rc()?;
    train()?;
    speed_up(cap)?;
    // PCIe base spec 6.6.1: 100 ms after training before the first config
    // request (above 5 GT/s; also covers PERST# release at lower speeds).
    udelay(100_000);
    let (gen, width) = link_status(cap);
    let (id, class) = endpoint_id()?;
    atu_outbound(1, ATU_TYPE_MEM, MEM_BASE, MEM_BASE, MEM_SIZE);
    atu_outbound(2, ATU_TYPE_IO, IO_CPU, 0, IO_SIZE);
    Ok(Link { gen, width, id, class })
}

/// Back to the state a cold boot leaves: training off, endpoint in reset,
/// controller in reset with its clocks gated, slot rail off. The serdes
/// subsystem and the shared AHB/MBUS serdes gates stay on (USB uses them).
fn shut_down(clocked: bool) {
    if clocked {
        clr32(APP_LTSSM_CTRL, LTSSM_EN);
    }
    pe_out(PIN_PERST, false);
    if clocked {
        clr32(ITS_BGR, (1 << 16) | (1 << 1));
        clr32(PCIE_AXI_SLV_CLK, GATE);
        clr32(PCIE_AUX_CLK, GATE);
        clr32(PCIE_BGR, (1 << 17) | (1 << 16));
    }
    pe_out(PIN_POWER, false);
}

/// Bring the root complex and the link up. Never fails the boot.
pub fn init() {
    let t = ticks();
    let mut clocked = false;
    match bring_up(&mut clocked) {
        Ok(l) => {
            println!(
                "[main] PCIe: link up Gen{} x{}, endpoint {:04x}:{:04x} class {:06x} ({} ms)",
                l.gen,
                l.width,
                l.id & 0xffff,
                l.id >> 16,
                l.class,
                us_since(t) / 1000
            );
            // SAFETY: only the boot CPU runs this, once, before the payload.
            unsafe { LINK = Some(l) };
        }
        Err(e) => {
            shut_down(clocked);
            match e {
                Error::Empty => {
                    println!("[main] PCIe: slot empty ({} ms)", us_since(t) / 1000);
                }
                e => {
                    println!("[main] PCIe: {e:?}; left off ({} ms)", us_since(t) / 1000);
                }
            }
        }
    }
}

/// Describe a trained link to the payload, inside /firmware/oreboot. The
/// node exists only when the link is up, and EDK2 touches the controller
/// only if it finds it. iATU region 0 belongs to the payload: it retargets
/// it to bus/device/function for each config access through
/// `config-window`; regions 1 and 2 already decode `mem-window` and
/// `io-window` (cells: CPU address, PCI address, size).
pub fn fdt_node(f: &mut Fdt) {
    // SAFETY: written once by init() on the boot CPU before this runs.
    let Some(l) = (unsafe { LINK }) else {
        return;
    };
    f.begin_node("pcie@6000000");
    f.prop_str("compatible", "oreboot,a733-pcie-rc");
    f.prop_u64_cells("reg", &[DBI as u64, DBI_SIZE as u64]);
    f.prop_u32_cells("link", &[l.gen, l.width]);
    f.prop_u32_cells("endpoint", &[l.id, l.class]);
    f.prop_u32_cells("bus-range", &[0, 0xff]);
    f.prop_u64_cells("config-window", &[CFG_BASE, CFG_WINDOW]);
    f.prop_u32("config-atu-region", 0);
    f.prop_u64_cells("mem-window", &[MEM_BASE, MEM_BASE, MEM_SIZE]);
    f.prop_u64_cells("io-window", &[IO_CPU, 0, IO_SIZE]);
    f.end_node();
}
