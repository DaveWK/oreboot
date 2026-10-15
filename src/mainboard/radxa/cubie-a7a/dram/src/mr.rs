//! LPDDR5 mode register access through the controller (MRCTRL0/1, MRSTAT),
//! the shared DesignWare path (`dwc_ddrctl::mr`) on the A733's two
//! controllers.

use crate::ctrl::{ddrc, MRCTRL0, MRCTRL1, MRSTAT};
use crate::hw::{Error, Hw};
use dwc_ddrctl::mr::{self as dwc_mr, Ctrl, MrError};

/// The A733's controllers: channel registers at `ddrc`, MRR data in
/// MCTL_COM +0x2c [15:8] for channel 0, +0x40 [15:8] for channel 1.
pub const A733: Ctrl = Ctrl {
    reg: ddrc,
    mrctrl0: MRCTRL0,
    mrctrl1: MRCTRL1,
    mrstat: MRSTAT,
    mrr_data: |ch| (if ch == 0 { 0x0a02_002c } else { 0x0a02_0040 }, 8),
    channels: 2,
    ranks: 2,
    timeout_us: 100_000,
};

fn error(e: MrError) -> Error {
    match e {
        MrError::Config(what) => Error::Config(what),
        MrError::Timeout(what) => Error::Timeout(what),
    }
}

/// Write one explicitly selected rank using the existing controller MR path.
/// Does not enter a training mode, infer applicability, or prove MR timing.
pub fn write_rank<H: Hw>(
    hw: &mut H,
    ch: usize,
    rank: u32,
    mr: u32,
    value: u32,
) -> Result<(), Error> {
    dwc_mr::write_rank(hw, &A733, ch, rank, mr, value).map_err(error)
}

/// Write MR `mr` of both ranks of channel `ch` (rank 0 gets `v0`, rank 1
/// `v1`). All fields are validated before the first command. A rank-1 timeout
/// does not undo rank 0: callers must retain that partial-operation failure.
pub fn write<H: Hw>(hw: &mut H, ch: usize, mr: u32, v0: u32, v1: u32) -> Result<(), Error> {
    dwc_mr::write(hw, &A733, ch, mr, v0, v1).map_err(error)
}

/// Read MR `mr` of `rank` of channel `ch`.
pub fn read<H: Hw>(hw: &mut H, ch: usize, rank: u32, mr: u32) -> Result<u32, Error> {
    dwc_mr::read(hw, &A733, ch, rank, mr).map_err(error)
}
