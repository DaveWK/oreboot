//! FEL lab observation of the PHY's read path around the lean hand-over
//! (`A7A_OBS_ARM`), for the blob-free read-capture question: does a
//! controller (DFI) read put strobes into the DBYTE RX FIFO, and do the
//! FIFO's two write pointers agree?
//!
//! Public basis (EIC7700X TRM Part 2, the same DWC LPDDR5/4 PHY family):
//! - RX FIFO pointer check: `RxDfe0/1FifoCheckStatus` 0x1n052/0x1n053
//!   (per lane: the strobe-side write pointer disagrees with the PUB's own
//!   write pointer when the PHY is idle), `RxDfe0FifoChkErrVal` 0x1n054
//!   (the first mismatch: PUB pointer [14:10], strobe-side odd/even [9:5]
//!   and [4:0]); checked at sideband transactions and dfi_init_start, and
//!   with `RxFifoChecks` 0x200fb = 1 at every break in read traffic;
//!   posted only when `PhyInterruptEnable` 0x2011b bit 10 is set; cleared
//!   by a 0->1 on `PhyInterruptClear` 0x2011e bit 10 (self-clearing);
//!   `PhyInterruptMask` 0x2011d bit 10 = 1 keeps it off the
//!   dwc_ddrphy_int_n pin.
//! - RX FIFO pointers `RxDfe0FifoInfo` 0x1n070 (RdLoc [15:10], WrLocOdd
//!   [9:5], WrLocEvn [4:0]) with `RxFifoVisibility` 0x1n072 =
//!   ObsSelOvrd|ObsSel 0 (0x200); ObsSelOvrd 0 hands the select back to
//!   the hardware's check, so it is restored at once (l.25755-25909).
//! - `Dfi0Status`/`Dfi1Status` 0x2005a/b (l.30804-30856), `RxFifoInit`
//!   0x200a0, `PUBMODE` 0x2006e, `PhyInterruptStatus` 0x2011f.
//!
//! Nothing here writes a delay, a pointer or a mode; the only writes are
//! the interrupt enable/mask/clear and RxFifoChecks (arming) and the
//! visibility select (put back as found). Default (knob unset): nothing.

use crate::hw::Hw;
use crate::phy::{csr, csr_access};

/// `A7A_OBS_ARM=<hex>`: bit 0 arms the RX FIFO pointer check before the
/// DFI init of the lean hand-over; bit 1 prints a snapshot of the read-path
/// observation CSRs before the DFI init and after the hand-over. Unset: 0.
#[allow(non_snake_case)]
pub fn OBS_ARM() -> u16 {
    obs_knob(crate::knobs::env("A7A_OBS_ARM", crate::knob!("A7A_OBS_ARM")))
}
/// Arm the RX FIFO pointer check.
pub const ARM_CHECK: u16 = 1;
/// Print the snapshots.
pub const ARM_SNAP: u16 = 2;

/// `A7A_HO_RDCSLAT=<hex>`: the controller's dfi_tphy_rdcslat (DFITMG2
/// +0x588 [14:8]) in set 0 and the active set, written by the lean
/// hand-over before the DFI init (the rank-select partner of
/// `A7A_HO_RDDATA_EN`; the timing code and the vendor leave 0x2f, equal to
/// dfi_t_rddata_en). Unset: unchanged.
#[allow(non_snake_case)]
pub fn HO_RDCSLAT() -> Option<u32> {
    match crate::knobs::env("A7A_HO_RDCSLAT", crate::knob!("A7A_HO_RDCSLAT")) {
        Some(s) => match u32::from_str_radix(s, 16) {
            Ok(v) if v <= 0x7f => Some(v),
            _ => panic!("A7A_HO_RDCSLAT is a hex value up to 0x7f"),
        },
        None => None,
    }
}
/// DFITMG2 dfi_tphy_rdcslat field.
pub const RDCSLAT_SHIFT: u32 = 8;
pub const RDCSLAT_MASK: u32 = 0x7f << RDCSLAT_SHIFT;

const fn obs_knob(s: Option<&str>) -> u16 {
    let Some(s) = s else {
        return 0;
    };
    let b = s.as_bytes();
    if b.len() != 1 {
        panic!("A7A_OBS_ARM is one hex digit");
    }
    match b[0] {
        b'0'..=b'3' => (b[0] - b'0') as u16,
        _ => panic!("A7A_OBS_ARM is 0-3"),
    }
}

const DBYTES: usize = 4;
const INT_ENABLE: u32 = 0x2011b;
const INT_MASK: u32 = 0x2011d;
const INT_CLEAR: u32 = 0x2011e;
const INT_STATUS: u32 = 0x2011f;
const RX_FIFO_CHECK_BIT: u16 = 1 << 10;
const RX_FIFO_CHECKS: u32 = 0x200fb;
const DFI0_STATUS: u32 = 0x2005a;
const DFI1_STATUS: u32 = 0x2005b;
const RX_FIFO_INIT: u32 = 0x200a0;
const PUBMODE: u32 = 0x2006e;
/// RxFifoObsSelOvrd = 1, RxFifoObsSel = 0 (DFE0 pointers).
const VIS_DFE0: u16 = 0x200;

const fn db(n: usize, off: u32) -> u32 {
    0x10000 + 0x1000 * n as u32 + off
}

/// Arm the RX FIFO pointer check: mask first (the pin stays quiet), then
/// enable, clear any old status, and check at every break in read
/// traffic. CSR access must be on.
pub fn arm<H: Hw>(hw: &mut H) {
    let m = hw.read16(csr(INT_MASK));
    hw.write16(csr(INT_MASK), m | RX_FIFO_CHECK_BIT);
    let e = hw.read16(csr(INT_ENABLE));
    hw.write16(csr(INT_ENABLE), e | RX_FIFO_CHECK_BIT);
    hw.write16(csr(INT_CLEAR), 0);
    hw.write16(csr(INT_CLEAR), RX_FIFO_CHECK_BIT);
    hw.write16(csr(RX_FIFO_CHECKS), 1);
}

/// One read-path snapshot.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Snap {
    /// RxDfe0FifoInfo lane 0 and lane 8 per DBYTE.
    pub info: [[u16; 2]; DBYTES],
    /// RxDfe0FifoCheckStatus, RxDfe1FifoCheckStatus per DBYTE.
    pub check: [[u16; 2]; DBYTES],
    /// RxDfe0FifoChkErrVal lane 0 per DBYTE.
    pub err_val: [u16; DBYTES],
    /// PhyInterruptStatus, Dfi0Status, Dfi1Status, RxFifoInit, PUBMODE.
    pub master: [u16; 5],
}

/// Take a snapshot with CSR access on, then put the visibility selects
/// and the CSR mux back as found.
pub fn snap<H: Hw>(hw: &mut H) -> Snap {
    let mux = hw.read16(crate::phy::MICRO_CONT_MUX_SEL);
    csr_access(hw, true);
    let mut s = Snap::default();
    let mut vis = [0u16; DBYTES];
    for (n, v) in vis.iter_mut().enumerate() {
        *v = hw.read16(csr(db(n, 0x072)));
    }
    for n in 0..DBYTES {
        hw.write16(csr(db(n, 0x072)), VIS_DFE0);
    }
    for n in 0..DBYTES {
        s.info[n] = [hw.read16(csr(db(n, 0x070))), hw.read16(csr(db(n, 0x870)))];
    }
    for (n, &v) in vis.iter().enumerate() {
        hw.write16(csr(db(n, 0x072)), v);
    }
    for n in 0..DBYTES {
        s.check[n] = [hw.read16(csr(db(n, 0x052))), hw.read16(csr(db(n, 0x053)))];
        s.err_val[n] = hw.read16(csr(db(n, 0x054)));
    }
    for (k, c) in [INT_STATUS, DFI0_STATUS, DFI1_STATUS, RX_FIFO_INIT, PUBMODE]
        .into_iter()
        .enumerate()
    {
        s.master[k] = hw.read16(csr(c));
    }
    hw.write16(crate::phy::MICRO_CONT_MUX_SEL, mux);
    s
}

/// Print a snapshot as one line (`[dram] OBS <label>: ...`).
pub fn print<H: Hw>(hw: &mut H, label: &str, s: &Snap) {
    hw.log(format_args!(
        "[dram] OBS {label}: info l0/l8 (Rd.WrOdd.WrEvn) {:?}; check dfe0/dfe1 {:03x?}; err_val l0 {:04x?}; IntStatus {:04x} Dfi0 {:04x} Dfi1 {:04x} RxFifoInit {:x} PUBMODE {:x}\n",
        s.info.map(|l| l.map(decode)),
        s.check,
        s.err_val,
        s.master[0],
        s.master[1],
        s.master[2],
        s.master[3],
        s.master[4],
    ));
}

/// RxDfe0FifoInfo as (RdLoc, WrLocOdd, WrLocEvn).
pub const fn decode(v: u16) -> (u16, u16, u16) {
    (v >> 10, (v >> 5) & 0x1f, v & 0x1f)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decode_fields() {
        // h-ctl after one MRR: Rd 13, WrOdd 12, WrEvn 12.
        assert_eq!(decode((13 << 10) | (12 << 5) | 12), (13, 12, 12));
        assert_eq!(decode(0xfc00), (63, 0, 0));
        assert_eq!(decode(0x9484), (37, 4, 4));
    }

    #[test]
    fn defaults_off() {
        if crate::knobs::env("A7A_OBS_ARM", crate::knob!("A7A_OBS_ARM")).is_none() {
            assert_eq!(OBS_ARM(), 0);
        }
        if crate::knobs::env("A7A_HO_RDCSLAT", crate::knob!("A7A_HO_RDCSLAT")).is_none() {
            assert_eq!(HO_RDCSLAT(), None);
        }
    }
}
