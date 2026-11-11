// SPDX-License-Identifier: GPL-2.0-only
//! RK3588 DDRCTL programming through the shared crate dwc-ddrctl.
//!
//! The RK3588 DDRCTL is a Synopsys DWC DDRCTL of the LPDDR4/5 generation
//! (frequency sets FREQ0..3 at 0x100000 strides holding DRAMSET1TMGn at
//! +0x0, the main block at +0x10000, the AXI port at +0x20000, ADDRMAP1..12
//! at +0x30004..+0x30030; RK3588 TRM Part 2 §2.4.1 register summary,
//! pp.57-108 and pp.181-190). Its ADDRMAP and DRAMSET1TMG field layout is
//! the one dwc-ddrctl implements for the Allwinner A733 (same controller
//! generation), so [`MAP`] plugs RK3588 into that crate: the address-map
//! writer and the quasi-dynamic `sw_done` handshake are reused as is.

use super::detect::ChannelGeometry;
use super::timing::Plan;
use crate::regs::{base, ctl};
use dwc_ddrctl::{addrmap, Map};
use regio::{wait, RegIo, Timeout};

fn main(ch: usize, off: usize) -> usize {
    base::DDRCTL[ch] + 0x1_0000 + off
}
fn freq(ch: usize, ps: usize, off: usize) -> usize {
    base::DDRCTL[ch] + ctl::freq(ps, off)
}
fn addrmap_reg(ch: usize, n: usize) -> usize {
    base::DDRCTL[ch] + ctl::ADDRMAP_BASE + 4 * n
}

/// Where the four RK3588 controllers' register blocks are.
pub const MAP: Map = Map {
    main,
    freq,
    addrmap: addrmap_reg,
};

pub const T_US: u64 = 100_000;

/// Quasi-dynamic write (TRM §2.6.1 steps 17-19 pattern): sw_done = 0, `f`,
/// sw_done = 1 and wait for sw_done_ack (dwc_ddrctl::sw_done).
pub fn quasi<H: RegIo + ?Sized>(
    io: &mut H,
    ch: usize,
    f: impl FnOnce(&mut H),
) -> Result<(), Timeout> {
    io.write32(main(ch, dwc_ddrctl::regs::SWCTL), 0);
    f(io);
    dwc_ddrctl::sw_done(io, &MAP, ch, T_US)
}

/// The dwc-ddrctl geometry of one channel (LPDDR4: no bank groups, binary
/// densities; 3/4-row parts are refused before this point).
pub fn dwc_geometry(g: &ChannelGeometry, ranks: u8) -> addrmap::Geometry {
    addrmap::Geometry {
        col_bits: g.col,
        row_bits: g.row,
        bank_bits: g.bank_bits,
        bg_bits: 0,
        rank_field: (ranks >= 2) as u32,
        dual_rank: ranks >= 2,
        nonbinary: 0,
        lpddr5: false,
    }
}

/// TRM §2.6.1 step 14: the controller registers, written while the core
/// and AXI resets are still asserted (APB is out of reset since step 3).
pub fn program<H: RegIo + ?Sized>(
    io: &mut H,
    ch: usize,
    p: &Plan,
    g: &ChannelGeometry,
) -> Result<(), &'static str> {
    let f = |off| freq(ch, 0, off);
    let m = |off: usize| base::DDRCTL[ch] + off;
    io.write32(m(ctl::MSTR0), p.mstr0);
    for (i, v) in p.dramset1tmg.iter().enumerate() {
        io.write32(f(ctl::dramset1tmg(i)), *v);
    }
    io.write32(f(ctl::DRAMSET1TMG12), p.tmg12);
    io.write32(f(ctl::DRAMSET1TMG13), p.tmg13);
    io.write32(f(ctl::DRAMSET1TMG14), p.tmg14);
    io.write32(f(ctl::DRAMSET1TMG25), p.tmg25);
    for (i, v) in p.initmr.iter().enumerate() {
        io.write32(f(ctl::INITMR0 + 4 * i), *v);
    }
    io.write32(f(ctl::DFITMG0), p.dfitmg0);
    io.write32(f(ctl::RFSHSET1TMG0), p.rfshset1tmg0);
    io.write32(f(ctl::RFSHSET1TMG1), p.rfshset1tmg1);
    io.write32(f(ctl::ZQSET1TMG0), p.zqset1tmg0);
    io.write32(f(ctl::ZQSET1TMG1), p.zqset1tmg1);
    io.write32(f(ctl::TMGCFG), 0); // 1:2 frequency ratio (p.105)
    io.write32(m(ctl::INITTMG0), p.inittmg0);
    io.write32(m(ctl::INITTMG1), p.inittmg1);
    io.write32(m(ctl::ODTMAP), p.odtmap);
    // No hardware low-power entry and no ZQ/ctrlupd from the controller
    // until training is done (TRM §2.6.3: "Controller should not assert
    // dfi_ctrlupd_req during training"); re-enabled by seq::finish.
    io.write32(m(ctl::HWLPCTL), 0);
    io.set32(m(ctl::ZQCTL0), ctl::ZQCTL0_DIS_AUTO_ZQ);
    io.set32(m(ctl::DFIUPD0), ctl::DFIUPD0_DIS_AUTO_CTRLUPD);
    // The dwc-ddrctl address map, with RK3588's reserved bits cleared:
    // ADDRMAP1 has only cs_bit0 [5:0] here (A733 also has cs_bit1 [13:8]).
    let w = addrmap::address_map(&dwc_geometry(g, p.ranks))?;
    for &(t, v) in w.as_slice() {
        match t {
            addrmap::Target::AddrMap(1) => io.write32(addrmap_reg(ch, 1), v & 0x3f),
            addrmap::Target::AddrMap(n) => io.write32(addrmap_reg(ch, n), v),
            addrmap::Target::Main(_) => return Err("LPDDR5 address-map tail on RK3588 LPDDR4"),
        }
    }
    Ok(())
}

/// Enter (true) or leave software self-refresh and wait for the state.
pub fn self_refresh<H: RegIo + ?Sized>(io: &mut H, ch: usize, enter: bool) -> Result<(), Timeout> {
    let b = base::DDRCTL[ch];
    if enter {
        io.set32(b + ctl::PWRCTL, ctl::PWRCTL_SELFREF_SW);
        wait(io, "STAT.operating_mode self-refresh", T_US, |io| {
            io.read32(b + ctl::STAT) & ctl::STAT_MODE_MASK == ctl::STAT_MODE_SELFREF
        })
    } else {
        io.clr32(b + ctl::PWRCTL, ctl::PWRCTL_SELFREF_SW);
        wait(io, "STAT.operating_mode normal", T_US, |io| {
            io.read32(b + ctl::STAT) & ctl::STAT_MODE_MASK == ctl::STAT_MODE_NORMAL
        })
    }
}

/// Update DFITMG0's latencies (quasi-dynamic, used by the DFI sweep).
pub fn set_dfitmg0<H: RegIo + ?Sized>(io: &mut H, ch: usize, v: u32) -> Result<(), Timeout> {
    quasi(io, ch, |io| io.write32(freq(ch, 0, ctl::DFITMG0), v))
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn map_addresses() {
        assert_eq!((MAP.main)(1, 0x14), 0xF801_0014);
        assert_eq!((MAP.freq)(2, 1, 0x500), 0xF910_0500);
        assert_eq!((MAP.addrmap)(3, 1), 0xFA03_0004);
    }
}
