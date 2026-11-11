// SPDX-License-Identifier: GPL-2.0-only
//! Where each DRAM address bit sits in the controller's host-interface
//! (HIF) word address, for any rank count and density (DRAM variants as
//! parameters, D18).
//!
//! The layout is the one the A733 controller programs (and the common
//! DesignWare default): from bit 0 up, column bits 0..2, bank-group bits,
//! the remaining column bits, bank bits, row bits, then the rank bit. A
//! controller's ADDRMAP encoding is computed from these positions.

use jedec_lpddr::geometry::Geometry;

/// One kind of DRAM address bit.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Bit {
    Col(u32),
    BankGroup(u32),
    Bank(u32),
    Row(u32),
    Rank(u32),
}

/// HIF bit of every address bit of `g` (at most 40 bits), in HIF order.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Layout {
    bits: [Bit; 40],
    len: usize,
}

impl Layout {
    /// The layout of `g`; None when it has fewer than 3 column bits or more
    /// than 40 address bits.
    pub fn new(g: &Geometry) -> Option<Layout> {
        if g.col_bits < 3 || g.address_bits() > 40 {
            return None;
        }
        let mut l = Layout {
            bits: [Bit::Col(0); 40],
            len: 0,
        };
        let mut push = |b| {
            l.bits[l.len] = b;
            l.len += 1;
        };
        for c in 0..3 {
            push(Bit::Col(c));
        }
        for b in 0..g.bg_bits {
            push(Bit::BankGroup(b));
        }
        for c in 3..g.col_bits {
            push(Bit::Col(c));
        }
        for b in 0..g.bank_bits {
            push(Bit::Bank(b));
        }
        for r in 0..g.row_bits {
            push(Bit::Row(r));
        }
        for r in 0..g.rank_bits {
            push(Bit::Rank(r));
        }
        Some(l)
    }
    pub fn bits(&self) -> &[Bit] {
        &self.bits[..self.len]
    }
    /// HIF bit of `b`, if the geometry has it.
    pub fn hif(&self, b: Bit) -> Option<u32> {
        self.bits().iter().position(|x| *x == b).map(|i| i as u32)
    }
    /// HIF bit of the rank select (None for one rank).
    pub fn rank_hif(&self) -> Option<u32> {
        self.hif(Bit::Rank(0))
    }
}

/// The controller's active-ranks mask for `ranks` (MSTR active_ranks,
/// chip-select masks).
pub const fn active_ranks_mask(ranks: u8) -> u32 {
    (1u32 << ranks) - 1
}
