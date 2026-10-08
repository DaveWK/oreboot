//! The board's DRAM parameters.
//!
//! Allwinner describes a board's DRAM with 32 words in `sys_config.fex`
//! order (`[dram_para]`: dram_clk, dram_type, dram_dx_odt, dram_dx_dri,
//! dram_ca_dri, dram_para0..2, dram_mr0..6, dram_mr11..14, dram_mr16,
//! dram_mr17, dram_mr22, dram_tpr0..3, dram_tpr6, dram_tpr10..14). The Cubie
//! A7A's sets come from Radxa's sys_config (see bt0 `dram.rs`). The meaning of
//! each word as this driver uses it is below; the init also stores what it
//! learns (DRAM size, MR14/MR15 per rank) back into the set, as the vendor
//! boot0 hands it on.

/// Words 0..31: the sys_config parameters. Words 32..51: results.
pub const WORDS: usize = 64;

#[derive(Clone, Debug)]
pub struct Para {
    pub w: [u32; WORDS],
}

/// DRAM type codes of `dram_type`.
pub const TYPE_LPDDR4: u32 = 8;
pub const TYPE_LPDDR5: u32 = 9;

impl Para {
    pub fn from_sysconfig(words: &[u32; 32]) -> Self {
        let mut w = [0; WORDS];
        w[..32].copy_from_slice(words);
        Self { w }
    }

    /// dram_clk: the top pstate's DRAM clock in MHz (CK; LPDDR5 WCK is 4x
    /// ... per JEDEC; here the PHY's DfiClk x 4 / divider, see pstate_mhz).
    pub fn clk(&self) -> u32 {
        self.w[0]
    }
    pub fn ty(&self) -> u32 {
        self.w[1]
    }
    pub fn is_lpddr5(&self) -> bool {
        self.ty() == TYPE_LPDDR5
    }
    /// dram_dx_odt: DQ ODT per byte lane.
    pub fn dx_odt(&self) -> u32 {
        self.w[2]
    }
    /// dram_dx_dri: DQ drive strength per byte lane.
    pub fn dx_dri(&self) -> u32 {
        self.w[3]
    }
    /// dram_ca_dri: CA drive strength.
    pub fn ca_dri(&self) -> u32 {
        self.w[4]
    }
    /// dram_para1: geometry of one rank of one channel:
    /// [3:0] column address bits, [11:4] row address bits,
    /// [13:12] bank address bits, [15:14] bank group address bits.
    pub fn para1(&self) -> u32 {
        self.w[6]
    }
    pub fn col_bits(&self) -> u32 {
        self.para1() & 0xf
    }
    pub fn row_bits(&self) -> u32 {
        (self.para1() >> 4) & 0xff
    }
    pub fn bank_bits(&self) -> u32 {
        (self.para1() >> 12) & 3
    }
    pub fn bg_bits(&self) -> u32 {
        (self.para1() >> 14) & 3
    }
    /// dram_para2: [3:0] nonzero for 16-bit (half) channels, [0] also the
    /// controller's half-bus-width mode; [12] two ranks ([15:12] the rank
    /// address bits); [30:16] the total size in MiB once known, with [31]
    /// set (written back by init); [31:30] == 2 marks a binary density.
    pub fn para2(&self) -> u32 {
        self.w[7]
    }
    pub fn set_para2(&mut self, v: u32) {
        self.w[7] = v;
    }
    pub fn rank_bits(&self) -> u32 {
        (self.para2() >> 12) & 0xf
    }
    pub fn dual_rank(&self) -> bool {
        self.para2() & (1 << 12) != 0
    }
    /// dram_mrN as given (LPDDR5 MR values for training).
    pub fn mr(&self, idx: usize) -> u32 {
        self.w[8 + idx]
    }
    /// dram_tpr0: [16] VDD2H at 1200 mV instead of 1080 mV.
    pub fn tpr0(&self) -> u32 {
        self.w[22]
    }
    /// dram_tpr1: signed; >= 0 enables the trained-result save.
    pub fn tpr1(&self) -> u32 {
        self.w[23]
    }
    /// dram_tpr2: per-pstate clock dividers, 5 bits per byte: pstate n runs
    /// at 4 * dram_clk / (tpr2[8n+4:8n] + 1).
    pub fn tpr2(&self) -> u32 {
        self.w[24]
    }
    /// dram_tpr3: [15:8] VDDQ in units of 10 mV (0: 500 mV);
    /// [20] the alternate PMIC rail set; [7:0] VDD2 for LPDDR4.
    pub fn tpr3(&self) -> u32 {
        self.w[25]
    }
    pub fn tpr6(&self) -> u32 {
        self.w[26]
    }
    /// dram_tpr10: [29] run the 2D-training result save, [28:16] ...
    pub fn tpr10(&self) -> u32 {
        self.w[27]
    }
    pub fn set_tpr10(&mut self, v: u32) {
        self.w[27] = v;
    }
    pub fn tpr11(&self) -> u32 {
        self.w[28]
    }
    pub fn tpr12(&self) -> u32 {
        self.w[29]
    }
    /// dram_tpr13: [0] geometry given (no auto scan), [2] DFS/multi-pstate
    /// training, [5] ..., [6] two channels, [18:16] non-binary density.
    pub fn tpr13(&self) -> u32 {
        self.w[30]
    }
    pub fn tpr14(&self) -> u32 {
        self.w[31]
    }
    pub fn channels(&self) -> usize {
        if self.tpr13() & (1 << 6) != 0 {
            2
        } else {
            1
        }
    }
    /// DRAM clock of pstate `ps` in MHz.
    pub fn pstate_mhz(&self, ps: usize) -> u32 {
        let div = (self.tpr2() >> (8 * ps)) & 0x1f;
        4 * self.clk() / (div + 1)
    }
}
