// SPDX-License-Identifier: GPL-2.0-only
//! Access to the K1 DDR PHY's fixed-function training engines.
//!
//! The K1 has no PHY firmware: write leveling, read gate and the read/write
//! eye engines are hardware, started through the DFI PHY user command
//! register and polled in the PHY status register. This module only moves
//! bits; the algorithm is in `train_own.rs`.
//!
//! Provenance of every register below (full map with sources:
//! doc/K1-DDR-REGISTERS.md; lookups in the translation are logged in
//! doc/K1-OWN-TRAINING-PLAN.md "Translation lookups"):
//!   GPL  SpacemiT uboot-2022.10 `drivers/ddr/spacemit/k1x/` at d0e22fd5b0
//!        (GPL-2.0+): `lpddr4_silicon_init.c`, `ddr_init_asic.h`,
//!        `ddr_freq.c`; GPL+ = also its later revision 90317d2789
//!   TR   identified by reading bt0's translated training (logged lookup);
//!        to be confirmed by the train-probe build on hardware
//!   JEDEC LPDDR4/4X

use util::mmio::{read32, write32};

/// DDR controller base (GPL `DDR_BASE`).
pub const DDRC_BASE: usize = 0xc000_0000;
/// User command 0: bit 28 command, [27:24] chip-select mask (GPL).
pub const MC_USER_CMD: usize = 0x0020;
/// Mode register command. MRW from the controller's shadow value:
/// `0x1002_0000 | cs_mask << 24 | mr` (GPL init and DFC table, e.g.
/// "Init MR14, update Vref(dq)"); MRR: `0x1001_0000 | (cs + 1) << 24 |
/// ch << 18 | mr`, data in `0x234` [7:0] once `0x370` bit 31 is set (GPL
/// `ddr_freq.c`).
pub const MC_MR_CMD: usize = 0x0024;
/// Frequency-point select for the per-point MC registers: [31:28] (GPL
/// `fp_sel`). Logged only.
pub const MC_FP_SEL: usize = 0x0304;
/// Per-point mode register values (GPL+ `fp_timing_init`: `ca_vref << 24 |
/// tx_vref << 16 | rx_soc_odt << 12`; d0e22fd5b0 writes `0x9d194314`). An MRW
/// of MR14 sends OP[5:0] (the VREF(DQ) setting) from [21:16] and OP[6] (the
/// range) from bit 23; bit 22 is not sent and is left alone. Measured on the
/// R2S, Oct 14: the CA byte `0x9d` reads back as MR12 `0x5d` (e-probe-02),
/// and with the range written to bit 22 the range-1 half of the MR14 axis
/// repeated range 0 exactly (f run). Before that fix [22:16] was used.
pub const MC_FP_MR_VALUES: usize = 0x030c;
pub const MR14_SETTING_SHIFT: u32 = 16;
pub const MR14_SETTING_MASK: u32 = 0x3f << MR14_SETTING_SHIFT;
pub const MR14_RANGE_BIT: u32 = 1 << 23;
/// DFI PHY user command 0 (GPL `DFI_PHY_USER_COMMAND_0` = 0x13d0: bit 0
/// init_start, bit 8 clear, [27:24] chip-select mask). The training trigger
/// bits are TR (see `Engine::command`).
pub const PHY_USER_CMD: usize = 0x13d0;
/// PHY status (GPL `MC_CH0_PHY_BASE + 0x3fc`: bit 31 PHY init done). Training
/// done/pass bits are TR (see `Engine::done`).
pub const PHY_STATUS: usize = 0x13fc;
/// Training engine parameter registers (TR: register identity only). Our
/// training does not write them unless a build sets a value (see
/// `train_own::Params`); the probe characterises their fields.
pub const PHY_PARAM: [usize; 3] = [0x13e8, 0x13ec, 0x13f8];

/// DDR PHY 0 (GPL `DDRC_BASE + 0x40000`).
pub const DPHY0: usize = 0x0004_0000;
/// Per frequency point copy of the PHY registers (GPL `FREQ_POINT_OFFSET`).
pub const DPHY_FP_STRIDE: usize = 0x4000;
/// Sub-PHYs A and B (GPL `subPHY_A_OFFSET`, `subPHY_B_OFFSET`).
pub const DPHY_SUBPHY: [usize; 2] = [0x0000, 0x0200];
/// Data PHYs within a sub-PHY (GPL `DATAPHY0_OFFSET`, `DATAPHY1_OFFSET`).
pub const DPHY_DATAPHY: [usize; 2] = [0x0000, 0x1000];
/// Rank offset (GPL `CS0_OFFSET`, `CS1_OFFSET`).
pub const DPHY_CS_STRIDE: usize = 0x100;
/// Common block (GPL `COMMON_OFFSET`).
pub const DPHY_COMMON: usize = 0x3000;
/// RX Vref, common block + 4 (GPL `top_DDR_rx_ds_odt_vref`): [23:16]
/// `rx_ref_d1`, [31:24] `rx_ref_d2`, each "high 4 bit for DQS, low 4 bit for
/// DQ"; the GPL init writes 0x55 to both. The low 16 bits hold other fields
/// (GPL `top_DDR_amble_config` sets bits 11, 13, 15).
pub const DPHY_RX_VREF: usize = DPHY_COMMON + 0x4;
pub const RX_VREF_DQ_MASK: u32 = 0x0f0f_0000;
/// Training block (GPL `TRAINING_CONTROL_OFFSET` = 0x18000 from DPHY0, so
/// `DDRC_BASE + 0x58000`). Its layout is TR: summary status at +0, per-DQ
/// read windows at +0x200 (32 bytes), write windows at +0x300 (32 bytes).
pub const DPHY_TRAINING: usize = DPHY0 + 0x1_8000;
pub const TRAINING_SUMMARY: usize = DPHY_TRAINING;
pub const TRAINING_RX_WINDOW: usize = DPHY_TRAINING + 0x200;
pub const TRAINING_TX_WINDOW: usize = DPHY_TRAINING + 0x300;
/// Other-control block (GPL `OTHER_CONTROL_OFFSET`); +0x28 is written by the
/// translation before training (TR); our training does not touch it, the
/// probe tests whether the engines need it.
pub const DPHY_OTHER: usize = DPHY0 + 0x1_0000;

/// A finite poll budget; a stuck engine becomes an error, not a hang.
pub const POLL_LIMIT: u32 = 10_000_000;
/// Delay after an MR14 write before the next engine run: JEDEC tVREF_LONG is
/// 250 ns; this many CPU no-ops are several microseconds at any K1 core clock.
pub const VREF_SETTLE_NOPS: usize = 20_000;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Engine {
    WriteLeveling,
    ReadGate,
    Read,
    Write,
}

impl Engine {
    pub const ALL: [Engine; 4] = [
        Engine::WriteLeveling,
        Engine::ReadGate,
        Engine::Read,
        Engine::Write,
    ];
    /// Trigger bit in the DFI PHY user command (TR).
    pub const fn command(self) -> u32 {
        match self {
            Engine::WriteLeveling => 1 << 22,
            Engine::Read => 1 << 21,
            Engine::ReadGate => 1 << 20,
            Engine::Write => 1 << 19,
        }
    }
    /// Done bits in the PHY status register (TR).
    pub const fn done(self) -> u32 {
        match self {
            Engine::WriteLeveling => 1 << 0,
            Engine::ReadGate => 1 << 1,
            Engine::Read => 3 << 1,
            Engine::Write => 3 << 5,
        }
    }
    pub const fn name(self) -> &'static str {
        match self {
            Engine::WriteLeveling => "write-leveling",
            Engine::ReadGate => "read-gate",
            Engine::Read => "read-eye",
            Engine::Write => "write-eye",
        }
    }
}

/// Read gate pass: PHY status [2:1] = 0b11 after the gate engine (TR).
pub const GATE_PASS: u32 = 3 << 1;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    /// An engine did not report done within the poll budget.
    Timeout {
        engine: Engine,
        rank: u32,
        last: u32,
    },
    /// The read gate engine finished without its pass bits.
    Gate { rank: u32, status: u32 },
    /// No open Vref point (fp, which: 0 read, 1 write).
    Closed { fp: u32, which: u8 },
    /// The re-run at the chosen Vref fell below the minimum window.
    Margin { fp: u32, which: u8, worst: u8 },
    /// Mode register read did not complete.
    Mrr,
    /// A knob-response check of the shared sweep failed (dead knob,
    /// aliased axis, ...; logged), no decision (fp, which: 0 read, 1 write).
    Sanity { fp: u32, which: u8 },
}

/// What the training algorithm needs from the PHY. `HwPhy` drives the K1;
/// host tests use a simulated eye.
pub trait Phy {
    fn fp(&self) -> u32;
    fn ranks(&self) -> u32;
    /// Current DQ RX Vref code of byte lane 0.
    fn rx_vref(&self) -> u8;
    /// Set the DQ RX Vref code of all byte lanes (DQS fields untouched).
    fn set_rx_vref(&mut self, code: u8);
    /// MR14 OP value in the controller's per-point register.
    fn tx_mr14(&self) -> u8;
    /// Program MR14 OP into the controller and send it to every rank.
    fn set_tx_mr14(&mut self, op: u8);
    /// Run one engine on one rank (0-based) and wait for it.
    fn run(&mut self, engine: Engine, rank: u32) -> Result<(), Error>;
    /// Per-DQ windows the read or write engine reported last.
    fn window(&self, engine: Engine) -> [u8; 32];
    /// Mode register read, rank `rank`, channel `ch`.
    fn mr_read(&mut self, mr: u32, ch: u32, rank: u32) -> Result<u8, Error>;
    /// Value of a register for the log (`None` in host models).
    fn peek(&self, _offset: usize) -> Option<u32> {
        None
    }
}

/// Chip-select mask of `ranks` ranks for MRW (GPL: 1 rank 0x1, 2 ranks 0x3).
pub use jedec_lpddr::lp4::cs_mask;

/// MR14 OP[6:0] held in an `MC_FP_MR_VALUES` register value.
pub const fn mr14_from_register(v: u32) -> u8 {
    (((v & MR14_SETTING_MASK) >> MR14_SETTING_SHIFT) | ((v & MR14_RANGE_BIT) >> 17)) as u8
}

/// `v` with MR14 OP[6:0] = `op` (OP[5:0] to [21:16], OP[6] to bit 23); every
/// other bit, bit 22 included, is kept.
pub const fn mr14_into_register(v: u32, op: u8) -> u32 {
    let op = op as u32;
    (v & !(MR14_SETTING_MASK | MR14_RANGE_BIT))
        | ((op & 0x3f) << MR14_SETTING_SHIFT)
        | ((op & 0x40) << 17)
}

pub struct HwPhy {
    pub ddrc: usize,
    pub fp: u32,
    pub ranks: u32,
    pub poll_limit: u32,
    /// Mode register read (bt0's GPL-derived `mode_register_read`).
    pub mrr: fn(u32, u32, u32) -> Option<u32>,
}

impl HwPhy {
    pub fn new(ddrc: usize, fp: u32, ranks: u32, mrr: fn(u32, u32, u32) -> Option<u32>) -> Self {
        Self {
            ddrc,
            fp,
            ranks,
            poll_limit: POLL_LIMIT,
            mrr,
        }
    }
    /// This frequency point's PHY register copy.
    pub fn pp(&self) -> usize {
        self.ddrc + DPHY0 + self.fp as usize * DPHY_FP_STRIDE
    }
    /// The two RX Vref registers (sub-PHY A and B).
    pub fn rx_vref_regs(&self) -> [usize; 2] {
        let pp = self.pp();
        [
            pp + DPHY_SUBPHY[0] + DPHY_RX_VREF,
            pp + DPHY_SUBPHY[1] + DPHY_RX_VREF,
        ]
    }
    /// Log the engine parameter registers and write the ones a build sets
    /// (`train_own::PARAMS`); the others keep their hardware value.
    pub fn apply_params(&self, params: &[Option<u32>; 3]) {
        for (reg, p) in PHY_PARAM.iter().zip(params) {
            let hw = read32(self.ddrc + reg);
            match p {
                Some(v) => {
                    write32(self.ddrc + reg, *v);
                    log::println!(
                        "[k1t] fp{} engine param 0x{reg:04x}: 0x{hw:08x} -> 0x{v:08x} (build)",
                        self.fp
                    );
                }
                None => {
                    log::println!(
                        "[k1t] fp{} engine param 0x{reg:04x}: 0x{hw:08x} (hardware value kept)",
                        self.fp
                    );
                }
            }
        }
    }
    /// Trigger an engine and poll; returns the last status on timeout.
    pub fn trigger(&self, engine: Engine, rank: u32) -> Result<u32, u32> {
        let cmd = 0x1000_0000 | ((1 << rank) << 24) | engine.command();
        write32(self.ddrc + PHY_USER_CMD, cmd);
        let mut last = 0;
        for _ in 0..self.poll_limit {
            last = read32(self.ddrc + PHY_STATUS);
            if last & engine.done() == engine.done() {
                return Ok(last);
            }
        }
        Err(last)
    }
}

impl Phy for HwPhy {
    fn fp(&self) -> u32 {
        self.fp
    }
    fn ranks(&self) -> u32 {
        self.ranks
    }
    fn rx_vref(&self) -> u8 {
        ((read32(self.rx_vref_regs()[0]) >> 16) & 0xf) as u8
    }
    fn set_rx_vref(&mut self, code: u8) {
        let c = (code & 0xf) as u32;
        for r in self.rx_vref_regs() {
            let v = read32(r) & !RX_VREF_DQ_MASK;
            write32(r, v | (c << 16) | (c << 24));
        }
    }
    fn tx_mr14(&self) -> u8 {
        mr14_from_register(read32(self.ddrc + MC_FP_MR_VALUES))
    }
    fn set_tx_mr14(&mut self, op: u8) {
        let r = self.ddrc + MC_FP_MR_VALUES;
        write32(r, mr14_into_register(read32(r), op));
        write32(
            self.ddrc + MC_MR_CMD,
            0x1002_0000 | (cs_mask(self.ranks) << 24) | 14,
        );
        util::nop_delay(VREF_SETTLE_NOPS);
    }
    fn run(&mut self, engine: Engine, rank: u32) -> Result<(), Error> {
        let status =
            self.trigger(engine, rank)
                .map_err(|last| Error::Timeout { engine, rank, last })?;
        if engine == Engine::ReadGate && status & GATE_PASS != GATE_PASS {
            return Err(Error::Gate { rank, status });
        }
        Ok(())
    }
    fn window(&self, engine: Engine) -> [u8; 32] {
        let base = self.ddrc
            + match engine {
                Engine::Write => TRAINING_TX_WINDOW,
                _ => TRAINING_RX_WINDOW,
            };
        let mut w = [0u8; 32];
        for (k, chunk) in w.chunks_mut(4).enumerate() {
            chunk.copy_from_slice(&read32(base + k * 4).to_le_bytes());
        }
        w
    }
    fn mr_read(&mut self, mr: u32, ch: u32, rank: u32) -> Result<u8, Error> {
        (self.mrr)(mr, ch, rank).map(|v| v as u8).ok_or(Error::Mrr)
    }
    fn peek(&self, offset: usize) -> Option<u32> {
        Some(read32(self.ddrc + offset))
    }
}
