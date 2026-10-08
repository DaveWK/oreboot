//! Native combo0 USB3.1 PHY for Cubie A7A's fixed blue Type-A port.
//!
//! Pinned BSP alexcaoys/allwinner-bsp 656a550c826787e7e72fae58ca793df9e0988ac8,
//! drivers/phy/sunxi-cadence-combophy.c (blob ebce3fb5e92b3575d5807ffd5bb81f490b0f1f1b).
//! Captured A7A DT lacks udp-disable: use combo0_configure_usb_dp(), normal
//! orientation (TYPEC_ORIENTATION_NONE), initial ssc_en=0. This is NOT the
//! alternate udp-disable USB table. USB PLL SSC stays enabled as in that
//! exact default branch. No DP connector/mode or combo1 PCIe lane is claimed.
//!
//! The USB caller first powers USB2 domain and opens shared SERDES bus/reset.
//! bt0's CLDO5 supplies the captured serdes1v8 rail. Do not reset the shared
//! SERDES or turn off shared clocks on failure: combo1 may own PCIe already.
//! Manual 18.2.6: orientation must be static while PHY reset is released.
//! Hold PHY/PIPE/DP resets before warm reconfiguration, use 16-bit PHY CSRs,
//! bound every handshake, and park only combo0 on failure.

use cubie_a7a_common::time::udelay;
use util::mmio::{read16, read32, write16, write32};

const TOP: usize = 0x06c0_1000;
const PHY: usize = 0x06c8_0000;
const MAP: usize = 0x06c0_6000;
const CCU_CFG: usize = 0x0200_33c0;
const DCXO: usize = 0x0709_016c;
const RESET: u32 = 1;
const PIPE: u32 = 1 << 4;
const PIPE_MASK: u32 = 1 << 5;
const DP: u32 = 1 << 8;
const ORIENTATION: u32 = 1 << 12;
const PLL: u32 = 1;
const POWER: u32 = 0x3f << 4;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    Reset,
    PllEnable,
    PmaReady,
}

#[derive(Clone, Copy)]
enum Op {
    Phy16(u32, u16),
    Top32(u16, u32, u32),
}
use Op::*;

// Exact compiled default BSP branch: retained source-parity controls compare
// every ordered 16-bit CSR write. Top fields use masked updates so reserved
// fields survive; the owned reference fields match the BSP's 0x01100001.
#[rustfmt::skip]
const TABLE: &[Op] = &[
    Top32(0x008, 0x00000003, 0x00000000),
    Phy16(0x1c00a, 0x0003),
    Phy16(0x1c00c, 0x1124),
    Phy16(0x08200, 0x00ff),
    Phy16(0x08600, 0x00ff),
    Phy16(0x08204, 0x04ae),
    Phy16(0x08604, 0x04ae),
    Phy16(0x08206, 0x04ae),
    Phy16(0x08606, 0x04ae),
    Phy16(0x10000, 0x0000),
    Phy16(0x10400, 0x091d),
    Phy16(0x10004, 0x0000),
    Phy16(0x10404, 0x0900),
    Phy16(0x10006, 0x0000),
    Phy16(0x10406, 0x0000),
    Phy16(0x1000c, 0x0000),
    Phy16(0x1040c, 0x010f),
    Phy16(0x081d4, 0x00ff),
    Phy16(0x085d4, 0x00ff),
    Phy16(0x10210, 0x0000),
    Phy16(0x10610, 0x0009),
    Phy16(0x10220, 0x0000),
    Phy16(0x10620, 0x0009),
    Phy16(0x10230, 0x0000),
    Phy16(0x10630, 0x000d),
    Phy16(0x08080, 0x2a84),
    Phy16(0x08480, 0x2a84),
    Phy16(0x0818c, 0x00a3),
    Phy16(0x0858c, 0x00a3),
    Phy16(0x080a0, 0x0000),
    Phy16(0x084a0, 0x0000),
    Phy16(0x08098, 0x001c),
    Phy16(0x08498, 0x001c),
    Top32(0x004, 0x01113331, 0x01100001),
    Phy16(0x00044, 0x001a),
    Phy16(0x00054, 0x0034),
    Phy16(0x00058, 0x00da),
    Phy16(0x00064, 0x0034),
    Phy16(0x00068, 0x00da),
    Phy16(0x000c8, 0x0082),
    Phy16(0x000ca, 0x0082),
    Phy16(0x000e8, 0x001a),
    Phy16(0x00208, 0x0020),
    Phy16(0x0020a, 0x0007),
    Phy16(0x00218, 0x0020),
    Phy16(0x0021a, 0x0007),
    Phy16(0x00228, 0x030c),
    Phy16(0x0022a, 0x0007),
    Phy16(0x00248, 0x0007),
    Phy16(0x0024a, 0x0003),
    Phy16(0x0024c, 0x000f),
    Phy16(0x00250, 0x0132),
    Phy16(0x10088, 0x001a),
    Phy16(0x10488, 0x001a),
    Phy16(0x10888, 0x001a),
    Phy16(0x10c88, 0x001a),
    Phy16(0x1008a, 0x0082),
    Phy16(0x1048a, 0x0082),
    Phy16(0x1088a, 0x0082),
    Phy16(0x10c8a, 0x0082),
    Phy16(0x10098, 0x001a),
    Phy16(0x10498, 0x001a),
    Phy16(0x10898, 0x001a),
    Phy16(0x10c98, 0x001a),
    Phy16(0x1009a, 0x0082),
    Phy16(0x1049a, 0x0082),
    Phy16(0x1089a, 0x0082),
    Phy16(0x10c9a, 0x0082),
    Phy16(0x08246, 0x0a28),
    Phy16(0x08646, 0x0a28),
    Phy16(0x08a46, 0x0a28),
    Phy16(0x08e46, 0x0a28),
    Phy16(0x00342, 0x8600),
    Phy16(0x00382, 0x0701),
    Phy16(0x089ca, 0x0012),
    Phy16(0x08dca, 0x0012),
    Phy16(0x081cc, 0x0000),
    Phy16(0x085cc, 0x0000),
    Phy16(0x081ce, 0x0001),
    Phy16(0x085ce, 0x0001),
    Phy16(0x081ca, 0x0012),
    Phy16(0x085ca, 0x0012),
    Phy16(0x081ca, 0x0041),
    Phy16(0x085ca, 0x0041),
    Phy16(0x089cc, 0x0001),
    Phy16(0x08dcc, 0x0001),
    Phy16(0x089ce, 0x0001),
    Phy16(0x08dce, 0x0001),
    Phy16(0x089ca, 0x0019),
    Phy16(0x08dca, 0x0019),
    Phy16(0x00128, 0x0004),
    Phy16(0x001a8, 0x0004),
    Phy16(0x00348, 0x0509),
    Phy16(0x00388, 0x0509),
    Phy16(0x0034a, 0x0f00),
    Phy16(0x0038a, 0x0f00),
    Phy16(0x0034c, 0x0f08),
    Phy16(0x0038c, 0x0f08),
    Phy16(0x00120, 0x0180),
    Phy16(0x001a0, 0x019f),
    Phy16(0x00122, 0x9d8a),
    Phy16(0x001a2, 0x6276),
    Phy16(0x00124, 0x0002),
    Phy16(0x001a4, 0x0002),
    Phy16(0x00126, 0x0102),
    Phy16(0x001a6, 0x0116),
    Phy16(0x00340, 0x0002),
    Phy16(0x00380, 0x0002),
    Phy16(0x00130, 0x0001),
    Phy16(0x001b0, 0x0002),
    Phy16(0x00132, 0x045f),
    Phy16(0x001b2, 0x04c4),
    Phy16(0x00134, 0x006b),
    Phy16(0x001b4, 0x006a),
    Phy16(0x00136, 0x0004),
    Phy16(0x001b6, 0x0004),
    Phy16(0x00108, 0x0104),
    Phy16(0x00188, 0x0104),
    Phy16(0x0010a, 0x0005),
    Phy16(0x0018a, 0x0005),
    Phy16(0x0010c, 0x0337),
    Phy16(0x0018c, 0x0337),
    Phy16(0x00110, 0x0335),
    Phy16(0x00190, 0x0335),
    Phy16(0x00104, 0x0003),
    Phy16(0x00184, 0x0003),
    Phy16(0x00138, 0x00cf),
    Phy16(0x001b8, 0x00cf),
    Phy16(0x0013c, 0x00ce),
    Phy16(0x001bc, 0x00ce),
    Phy16(0x0013e, 0x0005),
    Phy16(0x001be, 0x0005),
    Top32(0x000, 0x00001000, 0x00000000),
    Phy16(0x18020, 0x5100),
    Phy16(0x18022, 0x0100),
    Phy16(0x18030, 0x0a0a),
    Phy16(0x18034, 0x1000),
    Phy16(0x18036, 0x0010),
    Phy16(0x00082, 0x8200),
    Phy16(0x0008e, 0x8200),
    Phy16(0x08e00, 0x00ff),
    Phy16(0x08e02, 0x04af),
    Phy16(0x08e04, 0x04ae),
    Phy16(0x08e06, 0x04ae),
    Phy16(0x10800, 0x091d),
    Phy16(0x10802, 0x091d),
    Phy16(0x10804, 0x0900),
    Phy16(0x10806, 0x0000),
    Phy16(0x08c80, 0x2a84),
    Phy16(0x08c9a, 0x0011),
    Phy16(0x10920, 0x000c),
    Phy16(0x10a10, 0x0009),
    Phy16(0x10a92, 0x0c02),
    Phy16(0x10aee, 0x06f6),
    Phy16(0x10af0, 0x4606),
    Phy16(0x089d6, 0x0006),
    Phy16(0x10ae2, 0x0519),
    Phy16(0x10ae4, 0x0519),
    Phy16(0x10bd0, 0x1002),
    Phy16(0x10bca, 0x0b98),
    Phy16(0x10bc4, 0x0c01),
    Phy16(0x10bc6, 0x0000),
    Phy16(0x10bea, 0x0000),
    Phy16(0x10be8, 0x0311),
    Phy16(0x10bfe, 0x0000),
    Phy16(0x10ffe, 0x0000),
    Phy16(0x10900, 0x010a),
    Phy16(0x10904, 0x0003),
    Phy16(0x08dd4, 0x00ff),
    Phy16(0x089d4, 0x00ff),
    Phy16(0x08200, 0x02ff),
    Phy16(0x08202, 0x06af),
    Phy16(0x08204, 0x06ae),
    Phy16(0x08206, 0x06ae),
    Phy16(0x10400, 0x0d1d),
    Phy16(0x10402, 0x0d1d),
    Phy16(0x10404, 0x0d00),
    Phy16(0x10406, 0x0500),
    Phy16(0x08080, 0x2a82),
    Phy16(0x0809a, 0x0014),
    Phy16(0x10520, 0x0013),
    Phy16(0x10610, 0x0000),
    Phy16(0x10692, 0x0c02),
    Phy16(0x106ee, 0x0330),
    Phy16(0x106f0, 0x0300),
    Phy16(0x085d6, 0x0003),
    Phy16(0x106e2, 0x0019),
    Phy16(0x106e4, 0x0019),
    Phy16(0x107d0, 0x1004),
    Phy16(0x107ca, 0x00f9),
    Phy16(0x107c4, 0x0c01),
    Phy16(0x107c6, 0x0002),
    Phy16(0x107ea, 0x0000),
    Phy16(0x107e8, 0x0031),
    Phy16(0x107fe, 0x0001),
    Phy16(0x103fe, 0x0002),
    Phy16(0x10500, 0x018c),
    Phy16(0x10504, 0x0003),
    Phy16(0x081d4, 0x000f),
    Phy16(0x085d4, 0x00f0),
    Phy16(0x1c00a, 0x000b),
    Phy16(0x1c00c, 0x2224),
    Phy16(0x08a00, 0x00fb),
    Phy16(0x08e00, 0x00fb),
    Phy16(0x08a04, 0x04aa),
    Phy16(0x08e04, 0x04aa),
    Phy16(0x08a06, 0x04aa),
    Phy16(0x08e06, 0x04aa),
    Phy16(0x10800, 0x0000),
    Phy16(0x10c00, 0x0000),
    Phy16(0x10804, 0x0000),
    Phy16(0x10c04, 0x0000),
    Phy16(0x10806, 0x0000),
    Phy16(0x10c06, 0x0000),
    Phy16(0x1080c, 0x0000),
    Phy16(0x10c0c, 0x0000),
    Phy16(0x089d4, 0x000f),
    Phy16(0x08dd4, 0x000f),
    Phy16(0x10a10, 0x0000),
    Phy16(0x10e10, 0x0000),
    Phy16(0x10a20, 0x0000),
    Phy16(0x10e20, 0x0000),
    Phy16(0x10a30, 0x0000),
    Phy16(0x10e30, 0x0000),
    Phy16(0x08880, 0x08a4),
    Phy16(0x08c80, 0x08a4),
    Phy16(0x0898c, 0x0003),
    Phy16(0x08d8c, 0x0003),
    Phy16(0x088a0, 0x002a),
    Phy16(0x08ca0, 0x002a),
    Phy16(0x08898, 0x0000),
    Phy16(0x08c98, 0x0000),
    Phy16(0x00206, 0x007f),
    Phy16(0x00216, 0x007f),
];

fn update(a: usize, mask: u32, v: u32) {
    write32(a, (read32(a) & !mask) | (v & mask));
}
fn poll(a: usize, mask: u32, want: u32, us: u32) -> bool {
    for _ in 0..us / 5 {
        if read32(a) & mask == want {
            return true;
        }
        udelay(5);
    }
    read32(a) & mask == want
}

fn configure() {
    for op in TABLE {
        match *op {
            Phy16(off, value) => write16(PHY + off as usize, value),
            Top32(off, mask, value) => update(TOP + off as usize, mask, value),
        }
    }
}

/// Park only combo0. Retain USB2 UTMI, shared SERDES and combo1 PCIe resources.
pub fn disable() {
    update(TOP, RESET | PIPE | DP, 0);
    update(TOP + 0x0c, PLL | POWER, 0);
    update(TOP + 8, 3, 3); // IDDQ selector and software value: PHY off
    update(MAP + 0x100, 0xf, 0xf);
    update(MAP + 0x104, 0xf, 0xf);
}

fn start() -> Result<(), Error> {
    // Existing shared PCIe/SERDES clock is the same 100 MHz (PERI0 600 / 6).
    // Preserve unrelated fields and never cycle the shared reset here.
    update(
        CCU_CFG,
        (1 << 31) | (7 << 24) | 0x1f,
        (1 << 31) | (1 << 24) | 5,
    );
    update(DCXO, 1 << 4, 1 << 4); // exact RTC DCXO_SERDES0 gate; leave PHY1 on
    udelay(10);
    // Warm firmware may have left reverse orientation, masked PIPE reset or
    // a different power state. Establish all owned mode fields while reset.
    update(TOP, RESET | PIPE | PIPE_MASK | DP, 0);
    udelay(1000);
    update(TOP + 8, 3, 0);
    update(
        TOP + 4,
        (1 << 24) | (1 << 20) | 1,
        (1 << 24) | (1 << 20) | 1,
    );
    update(MAP + 0x100, 0xf, 0); // PIPE clock from combo0
    update(MAP + 0x104, 0xf, 0); // PIPE data from combo0
    configure();
    update(TOP, ORIENTATION, 0); // static fixed Type-A normal routing
    update(TOP, RESET, RESET);
    // BSP waits for this readback but continues on failure. Fail closed for
    // SuperSpeed rather than publishing a ready PHY after a lost write.
    if !poll(TOP, RESET, RESET, 1000) {
        return Err(Error::Reset);
    }
    update(
        MAP + 4,
        1 | (0xf << 4) | (0xf << 8),
        1 | (0xf << 4) | (0xf << 8),
    );
    udelay(1000);
    update(TOP, PIPE, PIPE);
    update(TOP + 0x0c, PLL, PLL);
    if !poll(TOP + 0x0c, PLL, PLL, 1000) {
        return Err(Error::PllEnable);
    }
    // Replace the complete request field, not OR(1) into a stale warm state.
    update(TOP + 0x0c, POWER, 1 << 4);
    update(TOP, DP, DP); // required by the exact BSP combo0 reset sequence
    if read32(TOP + 0x900) & 1 == 0 {
        udelay(10_000);
        let old = read16(PHY + (0xc008 << 1));
        write16(PHY + (0xc008 << 1), old & !(0xf << 8));
        if !poll(TOP + 0x900, 1, 1, 5000) {
            return Err(Error::PmaReady);
        }
    }
    Ok(())
}

pub fn init() -> Result<(), Error> {
    let result = start();
    if result.is_err() {
        disable();
    }
    result
}
