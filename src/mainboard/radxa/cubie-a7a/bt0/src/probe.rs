// SPDX-License-Identifier: GPL-2.0-only

//! Warm-reset research probes (FEL lab). Build-time knobs, all off by
//! default; with none set this module does nothing.
//!
//! - `A7A_FETCH_PROBE=1`: instruction fetch from DRAM, from AArch32 bt0,
//!   before the warm reset. Every bt0 DRAM access so far is a 32-bit
//!   single access with the MMU off (Strongly-ordered); oreboot main's
//!   first instructions are the first instruction fetches and, once
//!   SCTLR_EL3.I is set, the first cacheable line fills (bursts) the DRAM
//!   ever serves. With the MMU off and SCTLR.I = 1 an AArch32 instruction
//!   fetch is Normal, Inner/Outer Write-Through cacheable (Arm ARM,
//!   behaviour with stage 1 translation disabled), so bt0 itself can make
//!   the same kind of traffic: a sled of `add r0, r0, #1` ending in
//!   `bx lr` is written with single 32-bit stores, checked with single
//!   reads, then called with SCTLR.I = 0 (non-cacheable fetch) and = 1
//!   (line fills). The return value counts the instructions that ran.
//!   A wrong count or an undefined-instruction exception: fetched data is
//!   corrupt; a prefetch abort: the bus answered with an error; silence
//!   and the watchdog: the fetch never completes.
//! - `A7A_SYS_SNAP=1`: one line per register of the controllers (both
//!   channels, main block and the active frequency set), MCTL_COM, the
//!   NSI/port registers the hand-over writes, the DRAM/MBUS/NSI clocks and
//!   a few PHY master CSRs, as `[snap <tag>] <addr>=<value>`: the same list
//!   on every backend, for a host-side diff of the vendor, PMU and
//!   blob-free states right before the warm reset.
//! - `A7A_NO_BOOST=1` (in main.rs): no CPU clock boost after DRAM, i.e. the
//!   DSU/CPU clocks of the FEL lab states.
//! - `A7A_RD_GAPS=<hex>`: a fix test for back-to-back traffic. In the
//!   active frequency set of both controllers, inside a software
//!   self-refresh (quasi-dynamic group 2: public DDRCTL), DRAMSET1TMG4
//!   t_ccd [21:16] raised to at least the value (cap 0x3f), and the value
//!   added to DRAMSET1TMG2 wr2rd [7:0] / rd2wr [15:8], RANKTMG0
//!   diff_rank_rd_gap [7:0] / diff_rank_wr_gap [15:8] and RANKTMG1
//!   wr2rd_dr [7:0] / rd2wr_dr [15:8] (each capped at 0xff): consecutive
//!   bursts get idle gaps between them, as bt0's single accesses always
//!   have. Run at the `init` hook, before the fetch probe and the boot.
//!
//! Hooks: `bring_up` after bt0's DRAM test (before and after the boost),
//! and FEL entry 2 after the image checks, right before the GIC quiesce and
//! the warm reset (with the watchdog armed: entry 1 stopped it).

use cubie_a7a_common::uart::{Uart, UART0_BASE};
use util::mmio::{read16, read32, write16, write32};

/// Whether any probe knob can be on: one is in the build's environment, or
/// the DRAM lab mode may set them at run time. The hooks are compiled out
/// of every other image (as before the probes existed).
pub const ANY: bool = cfg!(feature = "dram-lab")
    || crate::knob!("A7A_FETCH_PROBE").is_some()
    || crate::knob!("A7A_SYS_SNAP").is_some()
    || crate::knob!("A7A_RD_GAPS").is_some()
    || crate::knob!("A7A_NO_BOOST").is_some();

#[allow(non_snake_case)]
pub fn FETCH_PROBE() -> bool {
    cubie_a7a_dram::knobs::env("A7A_FETCH_PROBE", crate::knob!("A7A_FETCH_PROBE")).is_some()
}
#[allow(non_snake_case)]
pub fn SYS_SNAP() -> bool {
    cubie_a7a_dram::knobs::env("A7A_SYS_SNAP", crate::knob!("A7A_SYS_SNAP")).is_some()
}
#[allow(non_snake_case)]
pub fn NO_BOOST() -> bool {
    cubie_a7a_dram::knobs::env("A7A_NO_BOOST", crate::knob!("A7A_NO_BOOST")).is_some()
}
#[allow(non_snake_case)]
fn RD_GAPS() -> Option<u32> {
    match cubie_a7a_dram::knobs::env("A7A_RD_GAPS", crate::knob!("A7A_RD_GAPS")) {
        Some(s) => match u32::from_str_radix(s, 16) {
            Ok(v) if (1..=0x3f).contains(&v) => Some(v),
            _ => panic!("A7A_RD_GAPS is a hex value 1..3f"),
        },
        None => None,
    }
}

/// Sled bases: away from main (0x40000000), the EL3 area and fwlog
/// (0x40000000-0x401fffff), the EDK2 FD (0x41000000, 4 MiB), the write
/// stage's probe blocks (0x40100000.., and rank 1's 0xc0100000.. or
/// 0x80100000..), the warm-reset
/// stub's FILL region (0x48000000, 1 MiB) and inside the range bt0 tests
/// (below 0xc0000000). A base at +0x80 in its 128-byte block: the other
/// channel (measured: the write stage's blocks 1 and 9). Each sled length
/// gets its own 128 KiB slot above its base, so no line was ever cached.
const SLED_BASES: [usize; 4] = [0x4c00_0000, 0x4c10_0080, 0x7fe0_0000, 0xbfe0_0000];
const SLOT: usize = 0x2_0000;
/// Instructions per sled (plus `mov r0, #0` and `bx lr`): one line, 4 KiB,
/// 64 KiB.
const SLED_LENGTHS: [usize; 3] = [16, 1024, 16384];
const A32_MOV_R0_0: u32 = 0xe3a0_0000;
const A32_ADD_R0_1: u32 = 0xe280_0001;
const A32_BX_LR: u32 = 0xe12f_ff1e;
const LINE: usize = 64;

fn drain() {
    Uart::attach(UART0_BASE).drain();
}

fn sctlr() -> u32 {
    let v: u32;
    unsafe { core::arch::asm!("mrc p15, 0, {}, c1, c0, 0", out(reg) v) };
    v
}

fn set_sctlr(v: u32) {
    unsafe { core::arch::asm!("dsb sy", "mcr p15, 0, {}, c1, c0, 0", "isb", in(reg) v) };
}

/// Clean and invalidate [a, a + len) to the point of coherency by MVA (all
/// cache levels, unified L2/L3 included; MMU off: VA = PA), then the
/// instruction cache and branch predictor.
fn flush_range(a: usize, len: usize) {
    let mut p = a & !(LINE - 1);
    while p < a + len {
        unsafe { core::arch::asm!("mcr p15, 0, {}, c7, c14, 1", in(reg) p) }; // DCCIMVAC
        p += LINE;
    }
    unsafe {
        core::arch::asm!(
            "dsb sy",
            "mcr p15, 0, {z}, c7, c5, 0", // ICIALLU
            "mcr p15, 0, {z}, c7, c5, 6", // BPIALL
            "dsb sy",
            "isb",
            z = in(reg) 0u32,
        )
    };
}

/// Write the sled at `base` with single stores; the number of words that
/// read back wrong with single loads.
fn write_sled(base: usize, n: usize) -> u32 {
    write32(base, A32_MOV_R0_0);
    for i in 0..n {
        write32(base + 4 + 4 * i, A32_ADD_R0_1);
    }
    write32(base + 4 + 4 * n, A32_BX_LR);
    let mut bad = 0;
    for i in 0..n + 2 {
        let want = match i {
            0 => A32_MOV_R0_0,
            i if i == n + 1 => A32_BX_LR,
            _ => A32_ADD_R0_1,
        };
        if read32(base + 4 * i) != want {
            bad += 1;
        }
    }
    bad
}

fn call_sled(base: usize) -> u32 {
    // SAFETY: base holds a complete A32 leaf function (written and read
    // back above); bit 0 clear, so blx stays in A32 state.
    let f: extern "C" fn() -> u32 = unsafe { core::mem::transmute(base) };
    f()
}

/// The instruction-fetch probe (`A7A_FETCH_PROBE`).
pub fn fetch(tag: &str) {
    if !FETCH_PROBE() {
        return;
    }
    let s0 = sctlr();
    println!(
        "[probe {tag}] FETCH: SCTLR {s0:08x} (M {}, C {}, I {})",
        s0 & 1,
        (s0 >> 2) & 1,
        (s0 >> 12) & 1
    );
    let mut fails = 0u32;
    for (k, &n) in SLED_LENGTHS.iter().enumerate() {
        for &b in SLED_BASES.iter() {
            let base = b + k * SLOT;
            crate::recovery::kick();
            let bad = write_sled(base, n);
            flush_range(base, 4 * (n + 2));
            for icache in [false, true] {
                let s = if icache {
                    s0 | (1 << 12)
                } else {
                    s0 & !(1 << 12)
                };
                println!(
                    "[probe {tag}] FETCH {base:08x} n {n} I {}: calling (sled readback errors {bad})",
                    icache as u32
                );
                drain();
                set_sctlr(s);
                let got = call_sled(base);
                set_sctlr(s0);
                let ok = got == n as u32;
                if !ok {
                    fails += 1;
                }
                println!(
                    "[probe {tag}] FETCH {base:08x} n {n} I {}: returned {got} {}",
                    icache as u32,
                    if ok { "ok" } else { "WRONG" }
                );
                flush_range(base, 4 * (n + 2));
            }
        }
    }
    println!("[probe {tag}] FETCH done: {fails} wrong results");
    drain();
}

const PHY: usize = 0x0a90_0000;
const fn csr(c: usize) -> usize {
    PHY + 2 * c
}
const MICRO_CONT_MUX_SEL: usize = 0xd0000;
const DDRC0: usize = 0x0a11_0000;
const DDRC_STRIDE: usize = 0x40_0000;
const CTRL0: usize = 0x0a10_0000;

/// DDRC main block offsets (public DDRCTL names, EIC7700X TRM Part 2,
/// block REGB_DDRC_CH0).
const DDRC_MAIN: [(usize, &str); 33] = [
    (0x000, "MSTR0"),
    (0x008, "MSTR2"),
    (0x010, "MSTR4"),
    (0x014, "STAT"),
    (0x090, "MRSTAT"),
    (0x180, "PWRCTL"),
    (0x184, "HWLPCTL"),
    (0x18c, "CLKGATECTL"),
    (0x200, "RFSHMOD0"),
    (0x208, "RFSHCTL0"),
    (0x220, "RFMMOD0"),
    (0x224, "+224"),
    (0x280, "ZQCTL0"),
    (0x284, "ZQCTL1"),
    (0x288, "ZQCTL2"),
    (0x300, "DQSOSCRUNTIME"),
    (0x308, "DQSOSCCFG0"),
    (0x380, "SCHED0"),
    (0x500, "DFILPCFG0"),
    (0x508, "DFIUPD0"),
    (0x510, "DFIMISC"),
    (0x514, "DFISTAT"),
    (0x518, "DFIPHYMSTR"),
    (0xb80, "OPCTRL0"),
    (0xb84, "OPCTRL1"),
    (0xb88, "OPCTRLCAM"),
    (0xb90, "OPCTRLSTAT"),
    (0xc80, "SWCTL"),
    (0xc84, "SWSTAT"),
    (0xc94, "DBICTL"),
    (0xc9c, "ODTMAP"),
    (0xd00, "INITTMG0"),
    (0x01c, "+01c"),
];

/// Only offsets the vendor library or our code already access (an APB read
/// of an offset this controller configuration lacks might abort);
/// DFIUPDTMG2 (+0x5b4), HWFFCCTL, RANKCTL, PPT2 and OPREFSTAT0 are left to
/// a lab R32 first.
/// Frequency-set block offsets (public names, EIC7700X TRM Part 2,
/// REGB_FREQf): DRAMSET1TMG0-14 (+0x000..), DFITMG (+0x580..),
/// RFSHSET1TMG0-2 (+0x600..), RFMSET1TMG0
/// (+0x650), ZQSET1TMG0/1 (+0x800/+0x804), DQSOSCCTL0 (+0xa80),
/// HWLPTMG0 (+0xb80), SCHEDTMG0 (+0xc00), TMGCFG (+0xd00), RANKTMG0/1
/// (+0xd04/+0xd08: the rank-to-rank gaps the CDD fold raises), PWRTMG
/// (+0xd0c).
const FREQ: [usize; 37] = [
    0x000, 0x004, 0x008, 0x00c, 0x010, 0x014, 0x018, 0x01c, 0x020, 0x024, 0x030, 0x034, 0x038,
    0x05c, 0x060, 0x064, 0x078, 0x580, 0x584, 0x588, 0x590, 0x594, 0x5a0, 0x5b0, 0x600, 0x604,
    0x608, 0x650, 0x800, 0x804, 0xa80, 0xb80, 0xc00, 0xd00, 0xd04, 0xd08, 0xd0c,
];

/// MCTL_COM, the per-channel port blocks and NSI registers the hand-over
/// writes (`post::controller_after_phy`), clocks and resets.
const SYS: [(usize, &str); 46] = [
    (0x0a02_0000, "COM+000"),
    (0x0a02_0004, "COM+004"),
    (0x0a02_0008, "COM+008"),
    (0x0a02_000c, "COM+00c"),
    (0x0a02_0010, "COM+010"),
    (0x0a02_0014, "COM+014"),
    (0x0a02_0018, "COM+018"),
    (0x0a02_001c, "COM+01c"),
    (0x0a02_0020, "COM+020"),
    (0x0a02_0024, "COM+024"),
    (0x0a02_0200, "COM+200"),
    (0x0a02_0204, "COM+204"),
    (0x0a02_021c, "COM+21c"),
    (0x0a02_0220, "COM+220"),
    (0x0a02_0224, "COM+224"),
    (0x0a02_0228, "COM+228"),
    (0x0a02_1800, "COM+1800"),
    (0x0a03_0000, "P0+000"),
    (0x0a03_0008, "P0+008"),
    (0x0a03_0018, "P0+018"),
    (0x0a03_0068, "P0+068"),
    (0x0a04_0000, "P1+000"),
    (0x0a04_0008, "P1+008"),
    (0x0a04_0018, "P1+018"),
    (0x0a04_0068, "P1+068"),
    (0x0202_4818, "NSI+4818"),
    (0x0202_481c, "NSI+481c"),
    (0x0202_4838, "NSI+4838"),
    (0x0202_448c, "NSI+448c"),
    (0x0202_468c, "NSI+468c"),
    (0x0202_4490, "NSI+4490"),
    (0x0202_4494, "NSI+4494"),
    (0x0202_3a04, "NSI+3a04"),
    (0x0202_3c04, "NSI+3c04"),
    (0x0202_3e04, "NSI+3e04"),
    (0x0202_4004, "NSI+4004"),
    (0x0202_4204, "NSI+4204"),
    (0x0200_2020, "PLL_DDR"),
    (0x0200_2518, "AHB"),
    (0x0200_2580, "NSI_CLK"),
    (0x0200_2584, "NSI_BGR"),
    (0x0200_2588, "MBUS_CLK"),
    (0x0200_2c00, "DRAM_CLK"),
    (0x0200_2c0c, "DRAM_BGR"),
    (0x0701_0250, "R_PRCM+250"),
    (0x0300_0160, "SYS+160"),
];

/// PHY CSRs (16-bit): mission-mode switches, MRL, PLL lock, the
/// training-firmware-owned HWT overrides.
const PHY_CSRS: [(usize, &str); 26] = [
    (0xd0000, "MicroContMuxSel"),
    (0xc0080, "UcclkHclkEnables"),
    (0x20054, "PMIEnable"),
    (0x20060, "MemResetL"),
    (0x2006e, "+2006e"),
    (0x2002a, "ACSMWckEnControl"),
    (0x200f8, "TxRdPtrInit"),
    (0x200a2, "ClockingCtrl"),
    (0x200c3, "PllPwrDn"),
    (0x200d5, "PllLockedStatus"),
    (0x20310, "ZCalReset"),
    (0x200f9, "DfiInitComplete"),
    (0x2000d, "HwtMRL"),
    (0x10000, "DFIMRL0"),
    (0x11000, "DFIMRL1"),
    (0x12000, "DFIMRL2"),
    (0x13000, "DFIMRL3"),
    (0x30000, "AcDFIMRL0"),
    (0x31000, "AcDFIMRL1"),
    (0x20072, "HwtLpCsEnA"),
    (0x2007a, "HwtControlOvr"),
    (0x20049, "PllCtrl3"),
    (0x2004e, "PllDacValIn"),
    (0x2000a, "AcDllLockParam"),
    (0x20017, "DxDllLockParam"),
    (0x2001f, "+2001f"),
];

/// The register snapshot (`A7A_SYS_SNAP`).
pub fn snap(tag: &str) {
    if !SYS_SNAP() {
        return;
    }
    for ch in 0..2 {
        let d = DDRC0 + ch * DDRC_STRIDE;
        for (off, name) in DDRC_MAIN {
            println!(
                "[snap {tag}] {:08x}={:08x} ch{ch} {name}",
                d + off,
                read32(d + off)
            );
        }
        let set = (read32(d + 0x008) & 0x1f) as usize;
        let f = CTRL0 + ch * DDRC_STRIDE + set * 0x10_0000;
        for off in FREQ {
            println!(
                "[snap {tag}] {:08x}={:08x} ch{ch} set{set}+{off:03x}",
                f + off,
                read32(f + off)
            );
        }
        crate::recovery::kick();
    }
    for (a, name) in SYS {
        println!("[snap {tag}] {a:08x}={:08x} {name}", read32(a));
    }
    // PHY CSR reads with the host CSR mux, put back as found.
    let mux = read16(csr(MICRO_CONT_MUX_SEL));
    write16(csr(MICRO_CONT_MUX_SEL), 0);
    let mut v = [0u16; PHY_CSRS.len()];
    for (k, (c, _)) in PHY_CSRS.iter().enumerate() {
        v[k] = if *c == MICRO_CONT_MUX_SEL {
            mux
        } else {
            read16(csr(*c))
        };
    }
    write16(csr(MICRO_CONT_MUX_SEL), mux);
    for (k, (c, name)) in PHY_CSRS.iter().enumerate() {
        println!("[snap {tag}] phy {c:05x}={:04x} {name}", v[k]);
    }
    drain();
}

/// Poll `addr` until `f` holds, at most about `n` reads; false on timeout.
fn poll(addr: usize, n: u32, f: impl Fn(u32) -> bool) -> bool {
    for _ in 0..n {
        if f(read32(addr)) {
            return true;
        }
    }
    false
}

/// `A7A_RD_GAPS` ([`RD_GAPS()`]): wider command spacing in the active set.
pub fn gaps(tag: &str) {
    let Some(g) = RD_GAPS() else {
        return;
    };
    const N: u32 = 10_000_000;
    let add8 = |v: u32, shift: u32| {
        let f = ((v >> shift) & 0xff) + g;
        (v & !(0xff << shift)) | (f.min(0xff) << shift)
    };
    for ch in 0..2 {
        let d = DDRC0 + ch * DDRC_STRIDE;
        let set = (read32(d + 0x008) & 0x1f) as usize;
        let f = CTRL0 + ch * DDRC_STRIDE + set * 0x10_0000;
        let before = [
            read32(f + 0x010),
            read32(f + 0x008),
            read32(f + 0xd04),
            read32(f + 0xd08),
        ];
        // PWRCTL.selfref_sw, STAT.selfref_type = 2'b10 (software).
        write32(d + 0x180, read32(d + 0x180) | 0x800);
        let sre = poll(d + 0x014, N, |v| v & 0x30 == 0x20);
        write32(d + 0xc80, 0);
        let ccd = ((before[0] >> 16) & 0x3f).max(g).min(0x3f);
        write32(f + 0x010, (before[0] & !(0x3f << 16)) | (ccd << 16));
        write32(f + 0x008, add8(add8(before[1], 0), 8));
        write32(f + 0xd04, add8(add8(before[2], 0), 8));
        write32(f + 0xd08, add8(add8(before[3], 0), 8));
        write32(d + 0xc80, 1);
        let done = poll(d + 0xc84, N, |v| v & 1 != 0);
        write32(d + 0x180, read32(d + 0x180) & !0x800);
        let srx = poll(d + 0x014, N, |v| v & 0x30 == 0 && v & 3 == 1);
        println!(
            "[probe {tag}] RD_GAPS() {g:#x} ch{ch} set{set}: TMG4/TMG2/RANKTMG0/RANKTMG1 {before:08x?} -> {:08x?}; SRE {sre} sw_done {done} SRX {srx}",
            [read32(f + 0x010), read32(f + 0x008), read32(f + 0xd04), read32(f + 0xd08)]
        );
    }
    drain();
}
