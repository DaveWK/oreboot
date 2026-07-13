// SPDX-License-Identifier: GPL-2.0-only
//! DesignWare DDRCTL register offsets within a channel's main block and the
//! fields this crate uses. Offsets named in the DesignWare register set; the
//! main block's base is the SoC's ([`crate::Map::main`]).

pub const MSTR0: usize = 0x000;
pub const MSTR4: usize = 0x010;
pub const STAT: usize = 0x014;
pub const MRCTRL0: usize = 0x080;
pub const MRCTRL1: usize = 0x084;
pub const MRSTAT: usize = 0x090;
pub const PWRCTL: usize = 0x180;
pub const DFILPCFG0: usize = 0x500;
pub const DFIUPD0: usize = 0x508;
pub const DFIMISC: usize = 0x510;
pub const DFISTAT: usize = 0x514;
pub const DFIPHYMSTR: usize = 0x518;
pub const SWCTL: usize = 0xc80;
pub const SWSTAT: usize = 0xc84;
pub const ODTMAP: usize = 0xc9c;

/// DFIMISC fields.
pub const DFI_INIT_COMPLETE_EN: u32 = 1 << 0;
pub const DFI_INIT_START: u32 = 1 << 5;
pub const DFI_FREQUENCY_MASK: u32 = 0x1f << 8;
/// PWRCTL.selfref_sw: the controller holds the DRAM in software self-refresh.
pub const PWRCTL_SELFREF_SW: u32 = 0x800;
/// STAT fields: selfref_type [5:4], operating_mode [2:0] (1: normal).
pub const STAT_SELFREF_TYPE: u32 = 0x30;
pub const STAT_OPERATING_MODE: u32 = 0x3;
pub const OPERATING_MODE_NORMAL: u32 = 1;
