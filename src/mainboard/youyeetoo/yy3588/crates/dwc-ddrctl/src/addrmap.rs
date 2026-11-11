// SPDX-License-Identifier: GPL-2.0-only
//! ADDRMAPn: which HIF (host interface word address) bit drives each
//! column, bank, bank-group, row and rank bit. Each field holds the HIF bit
//! minus that field's base; 0x1f/0x3f mark an unused bit.
//!
//! With C column, BG bank-group and BA bank bits the HIF word address is
//! laid out, from bit 0 up: column bits 0..2 (fixed), bank group, column
//! bits 3.., bank, row, rank. The encodings follow the A733 vendor init for
//! the geometries of its parameter sets (columns 8..11, rows 14..18,
//! non-binary densities), which is what the writer below implements; the
//! geometry check rejects everything else before any write.

use regio::RegIo;

/// One rank of one channel, as the controller addresses it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Geometry {
    pub col_bits: u32,
    pub row_bits: u32,
    pub bank_bits: u32,
    pub bg_bits: u32,
    /// Rank address field (A733: para2[15:12]): 0 one rank, 1 two ranks, 7
    /// the non-binary two-rank layout.
    pub rank_field: u32,
    pub dual_rank: bool,
    /// Non-binary density code (A733: tpr13[18:16]): 0 binary, 1 or 2.
    pub nonbinary: u32,
    pub lpddr5: bool,
}

/// Where a write of [`address_map`] goes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Target {
    AddrMap(usize),
    /// A main-block register (the LPDDR5 tail: +0x220, +0x224).
    Main(usize),
}

/// The ordered writes of one channel's address map (at most 14).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Writes {
    pub list: [(Target, u32); 14],
    pub len: usize,
}

impl Writes {
    const EMPTY: Writes = Writes {
        list: [(Target::Main(0), 0); 14],
        len: 0,
    };
    fn push(&mut self, t: Target, v: u32) {
        self.list[self.len] = (t, v);
        self.len += 1;
    }
    pub fn as_slice(&self) -> &[(Target, u32)] {
        &self.list[..self.len]
    }
}

/// The geometries the writer implements. This validates the software
/// representation, not the fitted DRAM.
pub fn validate(g: &Geometry) -> Result<(), &'static str> {
    if !(8..=11).contains(&g.col_bits)
        || !(14..=18).contains(&g.row_bits)
        || !matches!(g.bank_bits, 2 | 3)
        || g.bg_bits > 2
        || !matches!(g.rank_field, 0 | 1 | 7)
        || g.nonbinary > 2
    {
        return Err("geometry outside implemented ADDRMAP fields");
    }
    Ok(())
}

/// The address-map writes for `g`, in the order the controller gets them.
pub fn address_map(g: &Geometry) -> Result<Writes, &'static str> {
    validate(g)?;
    let mut w = Writes::EMPTY;
    let am = Target::AddrMap;
    let col = g.col_bits;
    let row = g.row_bits;
    let ba = g.bank_bits;
    let bg = g.bg_bits;
    let rank_bits = g.rank_field & 7;
    let dual_rank_adj = if rank_bits == 7 {
        2
    } else {
        (rank_bits == 1) as u32
    };
    let rep = |v: u32, n: u32| (0..n).fold(0, |acc, i| acc | (v << (8 * i)));

    // ADDRMAP6: column bits 3..6.
    w.push(am(6), rep(bg, 4) & !0xff);
    // ADDRMAP5: column bits 7..10.
    let am5 = match col {
        8 => 0x1f1f_1f00 | bg,
        9 => 0x1f1f_0000 | rep(bg, 2),
        10 => 0x1f00_0000 | rep(bg, 3),
        _ => rep(bg, 4),
    };
    w.push(am(5), am5);
    // ADDRMAP4: bank group bits 0..1.
    w.push(
        am(4),
        match bg {
            2 => 0x0101,
            1 => 0x3f01,
            _ => 0x3f3f,
        },
    );
    // ADDRMAP3: bank bits 0..2, after the column and bank-group bits.
    let b = col - 3 + bg;
    w.push(
        am(3),
        if ba == 3 {
            rep(b, 3)
        } else {
            0x3f_0000 | rep(b, 2)
        },
    );
    // ADDRMAP8..11: row bits 0..13.
    let r = col + bg + ba - 6;
    w.push(am(11), rep(r, 2));
    w.push(am(10), rep(r, 4));
    w.push(am(9), rep(r, 4));
    w.push(am(8), rep(r, 4));
    // ADDRMAP7: row bits 14..17; ADDRMAP1: the rank bit. A non-binary
    // density puts the top row bit and the rank bit higher.
    let nb15 = matches!(g.nonbinary, 1 | 2);
    let nb = g.nonbinary == 1;
    let base = col + bg + ba;
    let x = r + dual_rank_adj;
    let mut rank_hif: Option<u32> = None;
    let am7 = match row {
        14 => 0x1f1f_1f1f,
        15 if nb15 => {
            w.push(am(8), rep(r, 3) | (x << 24));
            rank_hif = Some(base + 7);
            0x1f1f_1f00 | x
        }
        15 => 0x1f1f_1f00 | r,
        16 if nb => {
            rank_hif = Some(base + 8);
            0x1f1f_0000 | rep(x, 2)
        }
        16 => 0x1f1f_0000 | rep(r, 2),
        17 if nb => {
            rank_hif = Some(base + 9);
            0x1f00_0000 | r | (x << 8) | (x << 16)
        }
        17 => 0x1f00_0000 | rep(r, 3),
        _ if nb => {
            rank_hif = Some(base + row - 8);
            rep(r, 2) | (x << 16) | (x << 24)
        }
        _ => rep(r, 4),
    };
    w.push(am(7), am7);
    if row == 16 && nb && g.lpddr5 {
        // ADDRMAP12: non-binary device density.
        w.push(am(12), 3);
    }
    if let Some(v) = rank_hif {
        w.push(am(1), v | 0x3f00);
    }
    if g.dual_rank {
        if !nb15 {
            w.push(am(1), (base + row - 6) | 0x3f00);
        }
    } else {
        w.push(am(1), 0x3f3f);
    }
    if g.lpddr5 {
        w.push(Target::Main(0x220), 0x0a09_0e01);
        w.push(Target::Main(0x224), 0);
    }
    Ok(w)
}

/// Program channel `ch`'s address map for `g`.
pub fn set_address_map<H: RegIo + ?Sized>(
    io: &mut H,
    map: &crate::Map,
    ch: usize,
    g: &Geometry,
) -> Result<(), &'static str> {
    let w = address_map(g)?;
    for &(t, v) in w.as_slice() {
        let addr = match t {
            Target::AddrMap(n) => (map.addrmap)(ch, n),
            Target::Main(off) => (map.main)(ch, off),
        };
        io.write32(addr, v);
    }
    Ok(())
}
