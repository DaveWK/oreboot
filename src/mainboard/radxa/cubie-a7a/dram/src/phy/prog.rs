//! PHY programs: the CSR writes of each PhyInit step as data, run by
//! [`run`].
//!
//! The steps' CSR values come from the vendor library's register-level
//! behaviour (see `tables.rs`): most are constants of the LPDDR5 PHY
//! configuration; the few that depend on the operating point are
//! [`COMPUTED`] here from the pstate's clock, the board's drive/ODT
//! parameters, the rank count and the speed bin.
//!
//! The PHY replays each pstate's CSRs from its PState SRAM on a frequency
//! change: a record per CSR pair holds the two 16-bit values, the pair's
//! APB word address and which halves to write. Records are written beside
//! the CSRs they replay; their slots are numbered in program order and a
//! slot the configuration skips (second-rank CSRs on a single-rank board)
//! is not used.

use crate::hw::Hw;

use super::csr;

pub const COMPUTED: u8 = 1 << 0;
/// Only on dual-rank boards.
pub const DUAL_RANK: u8 = 1 << 1;
/// One CSR per pstate (base + pstate), written with the pstate number.
pub const PSTATE_INDEXED: u8 = 1 << 2;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    /// Write `csr` with `val` (^ `xr` on one rank, ^ `xs` on the slow
    /// speed bin).
    Write,
    /// PState SRAM record `val` (slot) for the CSR pair at `csr`, `hi` the
    /// enables; data from the ops at indices `a` (low half) and `b`.
    RecDerived,
    /// PState SRAM record with literal data: `a` low, `b` high, `hi`
    /// enables; `xr`/`xs` apply to (low << 16 | high), `xen` to the
    /// enables on one rank.
    RecLiteral,
    Delay,
    /// Copy firmware blob `a` to consecutive CSRs from `csr`.
    Blob,
}

#[derive(Clone, Copy, Debug)]
pub struct Op {
    pub kind: Kind,
    pub flags: u8,
    pub csr: u32,
    pub val: u16,
    pub hi: u16,
    pub a: u16,
    pub b: u16,
    pub xr: u32,
    pub xs: u32,
    pub xen: u8,
}

const NONE: Op = Op {
    kind: Kind::Write,
    flags: 0,
    csr: 0,
    val: 0,
    hi: 0,
    a: 0,
    b: 0,
    xr: 0,
    xs: 0,
    xen: 0,
};

pub const fn w(csr: u32, val: u16) -> Op {
    Op { csr, val, ..NONE }
}
pub const fn wx(csr: u32, val: u16, xr: u32, xs: u32, flags: u8) -> Op {
    Op {
        csr,
        val,
        xr,
        xs,
        flags,
        ..NONE
    }
}
pub const fn rd(slot: u16, csr: u32, en: u16, lo_op: u16, hi_op: u16, flags: u8) -> Op {
    Op {
        kind: Kind::RecDerived,
        csr,
        val: slot,
        hi: en,
        a: lo_op,
        b: hi_op,
        flags,
        ..NONE
    }
}
#[allow(clippy::too_many_arguments)]
pub const fn rl(
    slot: u16,
    csr: u32,
    en: u16,
    lo: u16,
    hi: u16,
    xr: u32,
    xs: u32,
    xen: u8,
    flags: u8,
) -> Op {
    Op {
        kind: Kind::RecLiteral,
        csr,
        val: slot,
        hi: en,
        a: lo,
        b: hi,
        xr,
        xs,
        xen,
        flags,
    }
}
pub const fn delay(us: u16) -> Op {
    Op {
        kind: Kind::Delay,
        val: us,
        ..NONE
    }
}
pub const fn blob(csr: u32, b: Blob) -> Op {
    Op {
        kind: Kind::Blob,
        csr,
        a: b as u16,
        ..NONE
    }
}

/// The PHY's PIE sequencer images (data from the PHY firmware input).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[allow(non_camel_case_types)]
pub enum Blob {
    PIE_A = 0,
    PIE_B = 1,
    PIE_A_1RANK = 2,
    PIE_B_1RANK = 3,
}

/// What the programs need to know about the configuration.
pub struct Ctx<'a> {
    /// Pstate this program runs for (0..3), selecting the PState SRAM.
    pub pstate: u32,
    /// Controller (DFI) clock of the pstate in MHz.
    pub mhz: u32,
    /// DRAM data clock in MHz (4 x DFI clock: 2400 at pstate 0).
    pub data_mhz: u32,
    pub dual_rank: bool,
    /// tpr10[31]: the slower speed bin (RL 14).
    pub slow_bin: bool,
    pub dx_odt: u32,
    pub dx_dri: u32,
    pub ca_dri: u32,
    pub pie: [&'a [u8]; 4],
}

/// PState SRAM of `pstate`: CSR 0xa0000 + 0x1000 * pstate, 4 CSRs a slot.
fn ps_sram(pstate: u32, slot: u32) -> usize {
    csr(0xa0000 + 0x1000 * pstate + 4 * slot)
}

/// PState SRAM slots that only second-rank CSRs use, in the per-pstate
/// programs: a single-rank configuration leaves them out and packs the
/// later slots down.
fn rank1_slots(progs: &[&[Op]], out: &mut [u16; 64]) -> usize {
    let mut n = 0;
    for prog in progs {
        for op in prog.iter() {
            if op.kind != Kind::RecDerived {
                continue;
            }
            let only_rank1 = |idx: u16, bit: u16| {
                op.hi & bit == 0 || idx == 0xffff || prog[idx as usize].flags & DUAL_RANK != 0
            };
            if only_rank1(op.a, 1) && only_rank1(op.b, 2) && n < out.len() {
                out[n] = op.val;
                n += 1;
            }
        }
    }
    n
}

/// Slot `slot` of the dual-rank layout in the current configuration.
fn slot_of(slot: u16, dual_rank: bool) -> u32 {
    if dual_rank {
        return slot as u32;
    }
    let mut skipped = [0u16; 64];
    let n = rank1_slots(
        &[super::tables::C_PSTATE, super::tables::I_PSTATE],
        &mut skipped,
    );
    slot as u32 - skipped[..n].iter().filter(|&&s| s < slot).count() as u32
}

/// Run `prog`. Returns nothing; the PHY reports nothing back here.
pub fn run<H: Hw>(hw: &mut H, prog: &[Op], ctx: &Ctx, formula: fn(&Op, &Ctx) -> u16) {
    // Values written by each op of this program (for derived records), and
    // whether each op ran.
    let mut vals = [0u16; 800];
    let mut ran = [false; 800];
    for (i, op) in prog.iter().enumerate() {
        match op.kind {
            Kind::Write => {
                if op.flags & DUAL_RANK != 0 && !ctx.dual_rank {
                    continue;
                }
                let (addr, v) = if op.flags & PSTATE_INDEXED != 0 {
                    (op.csr + ctx.pstate, ctx.pstate as u16)
                } else if op.flags & COMPUTED != 0 {
                    (op.csr, formula(op, ctx))
                } else {
                    let mut v = op.val as u32;
                    if !ctx.dual_rank {
                        v ^= op.xr;
                    }
                    if ctx.slow_bin {
                        v ^= op.xs;
                    }
                    (op.csr, v as u16)
                };
                hw.write16(csr(addr), v);
                vals[i] = v;
                ran[i] = true;
            }
            Kind::RecDerived => {
                let mut en = op.hi;
                let half = |idx: u16, bit: u16, en: &mut u16| -> u16 {
                    if *en & bit == 0 || idx == 0xffff {
                        return 0;
                    }
                    if !ran[idx as usize] {
                        // A second-rank CSR this board skips.
                        *en &= !bit;
                        return 0;
                    }
                    vals[idx as usize]
                };
                let lo = half(op.a, 1, &mut en);
                let hi = half(op.b, 2, &mut en);
                if en == 0 {
                    continue;
                }
                let slot = slot_of(op.val, ctx.dual_rank);
                write_record(hw, ctx.pstate, slot, op.csr, en, lo, hi);
            }
            Kind::RecLiteral => {
                let mut v = (op.a as u32) << 16 | op.b as u32;
                let mut en = op.hi;
                if !ctx.dual_rank {
                    v ^= op.xr;
                    en ^= op.xen as u16;
                }
                if ctx.slow_bin {
                    v ^= op.xs;
                }
                if op.flags & COMPUTED != 0 {
                    // The literal mirrors a computed CSR (low half).
                    let lo = formula(op, ctx) as u32;
                    v = (v & 0xffff) | lo << 16;
                }
                write_record(
                    hw,
                    ctx.pstate,
                    slot_of(op.val, ctx.dual_rank),
                    op.csr,
                    en,
                    (v >> 16) as u16,
                    v as u16,
                );
            }
            Kind::Delay => hw.udelay(op.val as u32),
            Kind::Blob => {
                let data = ctx.pie[op.a as usize];
                for (k, h) in data.chunks_exact(2).enumerate() {
                    hw.write16(csr(op.csr + k as u32), u16::from_le_bytes([h[0], h[1]]));
                }
            }
        }
    }
}

/// Record: low half, high half, APB word address [15:0], and [4:3] the
/// half enables with [2:0] the word address [18:16].
fn write_record<H: Hw>(hw: &mut H, pstate: u32, slot: u32, csr: u32, en: u16, lo: u16, hi: u16) {
    let word = csr >> 1;
    let a = ps_sram(pstate, slot);
    hw.write16(a, lo);
    hw.write16(a + 2, hi);
    hw.write16(a + 4, word as u16);
    hw.write16(a + 6, (en << 3) | ((word >> 16) & 7) as u16);
}
