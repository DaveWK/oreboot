//! The Synopsys DesignWare LPDDR5/4/4X PHY of the A733 ("DRAM PHY" at
//! 0x0A900000 in the A733 manual).
//!
//! The PHY's CSRs are 16-bit registers on a 16-bit APB address space; the
//! A733 maps CSR `a` at CPU address `0x0A900000 + 2 * a`. CSR addresses
//! follow Synopsys' block layout: bits [19:12] select the block type and
//! instance (DBYTE n at 0x1n000, MASTER at 0x20000, ...), the training
//! firmware's instruction memory sits at 0x50000 and its data memory
//! (with the message block at its start) at 0x58000; the APB-only block at
//! 0xD0000 holds the microcontroller mux, reset and mailbox. Names below
//! are Synopsys' where the same CSR exists in the DDR4/LPDDR4 multiPHY
//! (whose names STMicroelectronics publishes in TF-A's
//! drivers/st/ddr/phy/phyinit, BSD-3-Clause).

pub mod acsm;
pub mod coldinit;
pub mod devinit;
pub mod freq;
pub mod fw;
pub mod init;
pub mod msgblock;
pub mod prefix;
pub mod prog;
pub mod sequence;
pub mod wckalias;
pub mod wcklevel;
#[rustfmt::skip]
pub mod tables;

pub use init::PhyFirmware;

use crate::hw::Hw;

pub const PHY_BASE: usize = 0x0a90_0000;

/// CPU address of PHY CSR `apb`.
pub const fn csr(apb: u32) -> usize {
    PHY_BASE + 2 * apb as usize
}

// APB-only block.
pub const MICRO_CONT_MUX_SEL: usize = csr(0xd0000);
pub const UCT_SHADOW_REGS: usize = csr(0xd0004);
pub const DCT_WRITE_PROT: usize = csr(0xd0031);
pub const UCT_WRITE_ONLY_SHADOW: usize = csr(0xd0032);
pub const UCT_WRITE_PROT: usize = csr(0xd0033);
pub const UCT_DAT_WRITE_ONLY_SHADOW: usize = csr(0xd0034);
pub const MICRO_RESET: usize = csr(0xd0099);
/// DRTUB block: UcclkHclkEnables, the clocks of the PMU's memories for APB
/// access.
pub const UCCLK_HCLK_ENABLES: usize = csr(0xc0080);

pub const DMEM: usize = csr(0x58000);
pub const IMEM: usize = csr(0x50000);

/// Select host (0) or microcontroller (1) CSR writes. This does not establish
/// exclusive ownership: the matching LPDDR5 register description in ESWIN's
/// EIC7700X TRM Part 2, PDF pp. 907-908, retains PIE/DMA access in both modes
/// and states that DMA can suppress host writes even in host mode. The caller
/// must establish the appropriate handoff/quiescence contract separately.
pub fn csr_access<H: Hw>(hw: &mut H, cpu: bool) {
    hw.write16(MICRO_CONT_MUX_SEL, if cpu { 0 } else { 1 });
}

/// CSR access plus the clocks of the PMU memories, for reading the message
/// block after training.
pub fn dmem_access<H: Hw>(hw: &mut H, on: bool) {
    if on {
        csr_access(hw, true);
        hw.write16(UCCLK_HCLK_ENABLES, 3);
    } else {
        hw.write16(UCCLK_HCLK_ENABLES, 0);
        csr_access(hw, false);
    }
}

/// Byte `off` of the training firmware's message block (DMEM holds it as
/// little-endian halfwords).
pub fn mb_byte<H: Hw>(hw: &mut H, off: usize) -> u32 {
    let h = hw.read16(DMEM + (off & !1)) as u32;
    if off & 1 != 0 {
        h >> 8
    } else {
        h & 0xff
    }
}

/// DBYTE `db` trained delay CSR of `lane` (0..8, the ninth being DMI):
/// CSR 0x1n04e + 0x100 * lane (low) and the next CSR (high).
pub const fn dbyte_lane_delay(db: u32, lane: u32) -> usize {
    csr(0x1004e + 0x1000 * db + 0x100 * lane)
}

/// The PHY's trained read pointer init (MASTER 0x20002 [3:0]) and DBYTE0's
/// receive-enable timing (0x1002a), from which the controller's DFI read
/// data enable delay follows.
pub fn read_latency_csrs<H: Hw>(hw: &mut H) -> (u32, u32) {
    csr_access(hw, true);
    let a = hw.read16(csr(0x20002)) as u32;
    let b = hw.read16(csr(0x1002a)) as u32;
    csr_access(hw, false);
    (a, b)
}
