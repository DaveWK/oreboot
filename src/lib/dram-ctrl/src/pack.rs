// SPDX-License-Identifier: GPL-2.0-only
//! Register-field packing tables: which value goes into which field of
//! which register, for one controller.

use crate::values::{Name, Values};
use dwc_ddrctl::timing::Field;

/// Where a field's value comes from.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Src {
    Value(Name),
    /// A constant the controller or the board fixes.
    Const(u32),
}

/// One field of one register (offset within the controller's block).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Entry {
    pub reg: usize,
    pub field: Field,
    pub src: Src,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PackError {
    /// A value the table needs is not set.
    Missing { reg: usize, name: Name },
    /// A value does not fit its field.
    Overflow {
        reg: usize,
        field: Field,
        value: u32,
    },
    /// More registers than [`MAX_REGS`].
    TooManyRegisters,
}

pub const MAX_REGS: usize = 32;

/// Register values in the order of their first entry in the table.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Packed {
    regs: [(usize, u32); MAX_REGS],
    len: usize,
}

impl Packed {
    pub fn as_slice(&self) -> &[(usize, u32)] {
        &self.regs[..self.len]
    }
    pub fn get(&self, reg: usize) -> Option<u32> {
        self.as_slice().iter().find(|r| r.0 == reg).map(|r| r.1)
    }
}

/// Pack `values` by `table`. Every field is checked; nothing is truncated.
pub fn pack(table: &[Entry], values: &Values) -> Result<Packed, PackError> {
    let mut p = Packed {
        regs: [(0, 0); MAX_REGS],
        len: 0,
    };
    for e in table {
        let x = match e.src {
            Src::Const(c) => c,
            Src::Value(n) => values.get(n).ok_or(PackError::Missing {
                reg: e.reg,
                name: n,
            })?,
        };
        if e.field.width < 32 && x >> e.field.width != 0 {
            return Err(PackError::Overflow {
                reg: e.reg,
                field: e.field,
                value: x,
            });
        }
        let i = match p.as_slice().iter().position(|r| r.0 == e.reg) {
            Some(i) => i,
            None => {
                if p.len == MAX_REGS {
                    return Err(PackError::TooManyRegisters);
                }
                p.regs[p.len] = (e.reg, 0);
                p.len += 1;
                p.len - 1
            }
        };
        p.regs[i].1 |= x << e.field.shift;
    }
    Ok(p)
}
