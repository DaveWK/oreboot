// SPDX-License-Identifier: GPL-2.0-only
//! Pure, host-testable core of the cached EL3 full-burst memory test
//! (`memtest-cached`):
//! - cache-line-aligned plan builder preserving EL3, payload and DTB;
//! - rank address decoding from channel 0 ADDRMAP1 (`ctrl::RankMap` formula);
//! - 64-bit word bit/beat/channel error decoding and per-GiB accounting;
//! - `1 0 x 0` beat-shape pattern pass targeting bits 5, 21, 37 and 53;
//! - 16 MiB cross-rank and above-4-GiB copy pass with chained SHA-256 compare.

use ::memtest::arch::aarch64::{ATTR1, BLOCK};
use ::memtest::stress::{splitmix64, Record};
use ::memtest::{Memory, Plan, Region};

#[cfg(test)]
use crate::{mem, sha256, Handoff};
#[cfg(not(test))]
use cubie_a7a_common::{mem, sha256, Handoff};

/// MAIR_EL3 value: attr0 = Device-nGnRnE (0x00), attr1 = Normal inner/outer
/// write-back non-transient read/write-allocate (0xff).
pub const MAIR_WB: u64 = 0xff00;
/// SCTLR_EL3.M: stage-1 EL3 MMU enable.
pub const SCTLR_M: u64 = 1 << 0;
/// SCTLR_EL3.C: data cacheability enable.
pub const SCTLR_C: u64 = 1 << 2;
/// SCTLR_EL3.I: instruction cacheability enable.
pub const SCTLR_I: u64 = 1 << 12;

/// Minimum alignment (bytes) for cached test range boundaries so no excluded
/// object shares a 64-byte cache line with tested memory.
pub const CACHE_LINE_ALIGN: u64 = 64;
/// Cache-maintenance block size for `stress::Runner` and the bit-37 pass.
pub const STRESS_BLOCK_BYTES: u64 = 64 * 1024;
/// Block size for the cross-rank and above-4-GiB copy pass.
pub const COPY_BLOCK_BYTES: u64 = 16 * 1024 * 1024;
/// Number of 16 MiB copy pairs planned for each copy leg.
pub const COPY_BLOCKS_PER_LEG: usize = 4;

/// Channel 0 DRAM controller ADDRMAP1 MMIO address
/// (`cubie-a7a-dram/src/ctrl.rs`: `CTRL0 = 0x0a10_0000`, `addrmap(0, 1)`).
pub const ADDRMAP1_CH0: usize = 0x0a13_0004;
/// WDT0_CTRL reload register (`main/src/soc.rs:12,148-150`,
/// `bt0/src/recovery.rs:16,64-66`): Device-nGnRnE MMIO in L1 block 0.
pub const WDT0_CTRL_ADDR: u64 = 0x0205_000c;
/// WDT0_CTRL reload value (`(0x0a57 << 1) | 1`, `main/src/soc.rs:149`,
/// `bt0/src/recovery.rs:65`).
pub const WDT0_RELOAD_VAL: u32 = (0x0a57 << 1) | 1;
/// Watchdog timeout armed by bt0 (`bt0/src/recovery.rs:50-56`: 16 s).
pub const WDT0_TIMEOUT_US: u64 = 16 * 1_000_000;
/// Reload WDT0 from inside the teardown `DC CIVAC` loop every 524,288 lines
/// (32 MiB): at 1 us/line that is 0.524 s between reloads (>30x under 16 s).
pub const TEARDOWN_WDT_RELOAD_LINES: u64 = 1 << 19;

/// Worst-case microseconds between consecutive WDT0 reloads during the
/// teardown sweep when each `DC CIVAC` line takes `us_per_line` microseconds.
pub const fn teardown_wdt_max_interval_us(reload_lines: u64, us_per_line: u64) -> Option<u64> {
    if reload_lines == 0 {
        return None;
    }
    reload_lines.checked_mul(us_per_line)
}

const _: () = {
    match teardown_wdt_max_interval_us(TEARDOWN_WDT_RELOAD_LINES, 1) {
        Some(us) if us < WDT0_TIMEOUT_US => {}
        _ => panic!("teardown WDT reload interval exceeds 16 s at 1 us/line"),
    }
};
/// Base bit offset of the chip-select field in ADDRMAP1
/// (`cubie-a7a-dram/src/ctrl.rs`: `CS_BIT0_BASE = 6`).
pub const CS_BIT0_BASE: u32 = 6;
/// System-word shift for dual 16-bit channels (32-bit system word).
pub const DUAL_CH_SHIFT: u32 = 2;

/// 4 GiB physical address threshold.
pub const ABOVE_4G: u64 = 1u64 << 32;
/// Maximum DRAM end supported by the 39-bit EL3 flat map (512 GiB).
const MAX_FLAT_END: u64 = 512u64 << 30;
/// Maximum number of 1 GiB DRAM buckets tracked individually.
pub const MAX_GIB: usize = 16;

/// Mask of the four beats of byte-0 bit 5 across one 64-bit word
/// (bits 5, 21, 37 and 53 = beats 0, 1, 2 and 3 of the 16-bit channel).
pub const BIT37_LANE_MASK: u64 = (1 << 5) | (1 << 21) | (1 << 37) | (1 << 53);
/// `1 0 1 0` on bits 5, 21, 37, 53: single-UI high on beat 2 (bit 37).
pub const BIT37_SHAPE_1010: u64 = (1 << 5) | (1 << 37);
/// `1 0 0 0` on bits 5, 21, 37, 53: low on beat 2 (bit 37).
pub const BIT37_SHAPE_1000: u64 = 1 << 5;

/// Fixed seeds for the two bit-37 sub-passes.
pub const BIT37_SEED_1010: u64 = 0xb137_1010_a733_0001;
pub const BIT37_SEED_1000: u64 = 0xb137_1000_a733_0002;
/// Base seed for the copy-pass source blocks.
pub const COPY_SEED_BASE: u64 = 0xc097_a733_2400_0000;

/// Chunk size (bytes) for the chained SHA-256 hash in the copy pass.
const HASH_CHUNK_BYTES: usize = 1024;
/// Maximum candidate 16 MiB blocks scanned per pool when planning copies.
const MAX_POOL_BLOCKS: usize = 256;

/// Descriptor field masks matching `memtest::arch::aarch64::l1_table`.
const L1_DESC_TYPE_MASK: u64 = 0b11;
const L1_ATTR_INDX_MASK: u64 = 0b111 << 2;

/// Compute the contiguous `[start, end)` virtual/physical address range of all
/// 1 GiB blocks mapped Normal write-back (`ATTR1`) in a level-1 flat-map table.
/// Used by cache teardown to `DC CIVAC` every cacheable-mapped block to PoC.
pub fn wb_mapped_range(table: &[u64; 512]) -> Result<(u64, u64), &'static str> {
    let mut range: Option<(usize, usize)> = None;
    let mut closed = false;
    for (gib, &desc) in table.iter().enumerate() {
        let is_wb = (desc & L1_DESC_TYPE_MASK) == BLOCK && (desc & L1_ATTR_INDX_MASK) == ATTR1;
        if is_wb {
            if closed {
                return Err("discontiguous cacheable L1 blocks");
            }
            range = match range {
                None => Some((gib, gib + 1)),
                Some((s, _)) => Some((s, gib + 1)),
            };
        } else if range.is_some() {
            closed = true;
        }
    }
    match range {
        Some((s, e)) => Ok(((s as u64) << 30, (e as u64) << 30)),
        None => Err("no cacheable L1 blocks mapped"),
    }
}

/// Decode the DRAM-offset rank bit from an ADDRMAP1 register value, matching
/// `cubie_a7a_dram::ctrl::RankMap::bit_from_addrmap1`.
pub const fn rank_bit_from_addrmap1(addrmap1: u32, shift: u32) -> Option<u32> {
    let cs = addrmap1 & 0x3f;
    if cs == 0x3f {
        None
    } else {
        Some(cs + CS_BIT0_BASE + shift)
    }
}

/// Validated live rank-map decode from channel-0 `ADDRMAP1` + usable `dram_size`
/// (`cubie-a7a-dram/src/ctrl.rs` `RankMap`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LiveRankMap {
    /// Supported dual-rank CXMT map: 4 GB (rank bit 31) or 6 GB (rank bit 30).
    DualRank { rank_bit: u32 },
    /// Single-rank map (`ADDRMAP1` chip-select field `0x3f`, e.g. HYNIX 4 GB).
    SingleRank,
}

impl LiveRankMap {
    pub const fn rank_bit(self) -> Option<u32> {
        match self {
            Self::DualRank { rank_bit } => Some(rank_bit),
            Self::SingleRank => None,
        }
    }
}

/// Chip-select fields of `ADDRMAP1`: `cs0` in `[5:0]`, `cs1` in `[13:8]`.
const ADDRMAP1_CS_MASK: u32 = 0x3f;
const ADDRMAP1_CS1_SHIFT: u32 = 8;
const ADDRMAP1_CS_UNUSED: u32 = 0x3f;

/// Validate the live channel-0 `ADDRMAP1` readback against `dram_size` and the
/// supported A7A board maps (`cubie-a7a-dram/src/ctrl.rs` `RankMap` and
/// `dwc-ddrctl/src/addrmap.rs`). The writer sets the unused upper field to
/// `0x3f`, but task-000095 read `0x0017` on the 4 GiB board after `0x3f17`
/// was written. Admit either readback, not an arbitrary second CS mapping.
pub fn validate_live_rank_map(addrmap1: u32, dram_size: u64) -> Result<LiveRankMap, &'static str> {
    const GIB: u64 = 1u64 << 30;
    if addrmap1 == 0 || addrmap1 == u32::MAX {
        return Err("unreadable ADDRMAP1 register value");
    }
    let cs1 = (addrmap1 >> ADDRMAP1_CS1_SHIFT) & ADDRMAP1_CS_MASK;
    if cs1 != 0 && cs1 != ADDRMAP1_CS_UNUSED {
        return Err("incompatible ADDRMAP1 second CS field");
    }
    match rank_bit_from_addrmap1(addrmap1, DUAL_CH_SHIFT) {
        None => Ok(LiveRankMap::SingleRank),
        Some(31) if dram_size == 4 * GIB => Ok(LiveRankMap::DualRank { rank_bit: 31 }),
        Some(30) if dram_size == 6 * GIB => Ok(LiveRankMap::DualRank { rank_bit: 30 }),
        Some(_) => Err("unsupported dual-rank ADDRMAP1 / DRAM size geometry"),
    }
}

/// Check that `plan` includes testable words in both rank 0 and rank 1 (and at
/// least one word above 4 GiB) under `rank_bit`.
pub fn plan_covers_both_ranks_and_above_4g(plan: &Plan, dram_base: u64, rank_bit: u32) -> bool {
    if rank_bit >= 63 {
        return false;
    }
    let stride = 1u64 << rank_bit;
    let mut saw_r0 = false;
    let mut saw_r1 = false;
    let mut saw_hi = false;
    for r in plan.ranges() {
        if r.bytes() == 0 {
            continue;
        }
        let mut a = r.start;
        while a < r.end {
            let rk = rank_of_addr(a, dram_base, Some(rank_bit));
            if rk == 0 {
                saw_r0 = true;
            } else {
                saw_r1 = true;
            }
            if a >= ABOVE_4G {
                saw_hi = true;
            }
            // Jump to the next rank boundary or above-4-GiB boundary inside `r`.
            let off = a - dram_base;
            let next_rank = dram_base + ((off / stride) + 1) * stride;
            let next = if a < ABOVE_4G && ABOVE_4G < next_rank {
                ABOVE_4G
            } else {
                next_rank
            };
            a = next.min(r.end);
        }
    }
    saw_r0 && saw_r1 && saw_hi
}

/// Three-state verdict of a `memtest-cached` run.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Verdict {
    /// Zero errors and full dual-rank + above-4-GiB coverage (all passes + all
    /// three 16 MiB copy legs with non-zero bytes).
    Pass,
    /// Zero errors on a single-rank geometry (dual-rank copy legs unavailable);
    /// never reported as `PASS`.
    Limited,
    /// Any DRAM error, unusable/unsupported rank map, or missing required leg.
    Fail,
}

impl Verdict {
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Pass => "PASS",
            Self::Limited => "LIMITED",
            Self::Fail => "FAIL",
        }
    }
}

/// Coverage facts collected across a `memtest-cached` run.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct RunCoverage {
    pub stress_ran_both_ranks: bool,
    pub march_ran_both_ranks: bool,
    pub bit37_ran_both_ranks: bool,
    pub copy_r0_to_r1_bytes: u64,
    pub copy_r1_to_r0_bytes: u64,
    pub copy_r1_above4g_bytes: u64,
}

/// Evaluate the run verdict from the live rank map, error total and per-phase
/// / per-leg coverage.
pub fn evaluate_verdict(
    rank_map: Result<LiveRankMap, &'static str>,
    total_errors: u64,
    cov: &RunCoverage,
) -> (Verdict, &'static str) {
    let Ok(map) = rank_map else {
        return (Verdict::Fail, "invalid or unsupported rank map");
    };
    if total_errors > 0 {
        return (Verdict::Fail, "DRAM errors detected");
    }
    match map {
        LiveRankMap::SingleRank => (
            Verdict::Limited,
            "single-rank geometry; dual-rank copy legs unavailable",
        ),
        LiveRankMap::DualRank { .. } => {
            if !cov.stress_ran_both_ranks || !cov.march_ran_both_ranks || !cov.bit37_ran_both_ranks
            {
                return (
                    Verdict::Fail,
                    "plan did not cover both ranks and above 4 GiB",
                );
            }
            if cov.copy_r0_to_r1_bytes == 0
                || cov.copy_r1_to_r0_bytes == 0
                || cov.copy_r1_above4g_bytes == 0
            {
                return (
                    Verdict::Fail,
                    "missing required dual-rank or above-4-GiB copy leg",
                );
            }
            (
                Verdict::Pass,
                "full dual-rank and above-4-GiB coverage clean",
            )
        }
    }
}

/// Rank (0 or 1) that system address `addr` selects on a map starting at
/// `dram_base`.
pub const fn rank_of_addr(addr: u64, dram_base: u64, rank_bit: Option<u32>) -> u8 {
    match rank_bit {
        Some(b) if addr >= dram_base && b < 64 => (((addr - dram_base) >> b) & 1) as u8,
        _ => 0,
    }
}

/// Round `[start, end)` outward to `CACHE_LINE_ALIGN`, clamped inside `dram`.
fn round_outward_to_line(r: Region, dram: Region) -> Result<Region, &'static str> {
    let start = (r.start & !(CACHE_LINE_ALIGN - 1)).max(dram.start);
    let end = r
        .end
        .checked_add(CACHE_LINE_ALIGN - 1)
        .map(|v| (v & !(CACHE_LINE_ALIGN - 1)).min(dram.end))
        .ok_or("excluded range overflow")?;
    Ok(Region { start, end })
}

/// Build the cached test plan from `h`: all DRAM except EL3, the loaded
/// payload and any supplied DTB, with every exclusion rounded outward to
/// 64-byte cache lines so `DC IVAC` / `DC CIVAC` never share a cache line
/// with excluded memory.
pub fn cached_plan_from_handoff(h: &Handoff) -> Result<Plan, &'static str> {
    if h.dram_base != mem::DRAM_BASE as u64 {
        return Err("unexpected A7A DRAM base");
    }
    let dram_end = h
        .dram_base
        .checked_add(h.dram_size)
        .ok_or("DRAM range overflow")?;
    if h.dram_size == 0 || dram_end > MAX_FLAT_END || !h.dram_size.is_multiple_of(CACHE_LINE_ALIGN)
    {
        return Err("unexpected A7A DRAM size");
    }
    let dram = Region::new(h.dram_base, h.dram_size)?;
    let el3 = Region::new(mem::EL3_BASE as u64, mem::EL3_SIZE as u64)?;
    let payload =
        Region::covering(h.payload_addr, h.payload_size).map_err(|_| "invalid payload range")?;

    let mut raw = [Region::default(); 3];
    let mut raw_len = 2usize;
    raw[0] = el3;
    raw[1] = payload;
    if h.dtb_addr != 0 || h.dtb_size != 0 {
        if h.dtb_addr == 0 || h.dtb_size == 0 {
            return Err("invalid DTB range");
        }
        raw[2] = Region::covering(h.dtb_addr, h.dtb_size).map_err(|_| "DTB overflow")?;
        raw_len = 3;
    }

    // Validate the unrounded exclusions against DRAM and each other first.
    let _ = Plan::new(dram, &raw[..raw_len])?;

    // Round each exclusion outward to 64-byte cache lines and merge any that
    // now touch or overlap on a shared cache line.
    let mut rounded = [Region::default(); 3];
    for i in 0..raw_len {
        rounded[i] = round_outward_to_line(raw[i], dram)?;
    }
    rounded[..raw_len].sort_unstable_by_key(|r| r.start);

    let mut merged = [Region::default(); 3];
    let mut merged_len = 0usize;
    for r in rounded[..raw_len].iter().copied() {
        if merged_len > 0 && r.start <= merged[merged_len - 1].end {
            merged[merged_len - 1].end = merged[merged_len - 1].end.max(r.end);
        } else {
            merged[merged_len] = r;
            merged_len += 1;
        }
    }

    Plan::new(dram, &merged[..merged_len])
}

/// Decoded physical/burst position of one failing bit inside a 64-bit word.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BitDecode {
    /// Failing bit position inside the 64-bit word (`0..64`).
    pub bit: u8,
    /// Byte offset inside the 64-bit word (`0..8`, `bit / 8`).
    pub byte_in_word: u8,
    /// Bit position inside that byte (`0..8`, `bit % 8`).
    pub bit_in_byte: u8,
    /// 16-bit beat inside the 64-bit word (`0..4`, `bit / 16`).
    pub beat_in_word: u8,
    /// 16-bit beat inside the 32-byte (16-beat) burst (`0..16`).
    pub burst_beat: u8,
    /// Failing byte's address mod 8: `(addr + bit / 8) % 8`.
    pub byte_addr_mod_8: u8,
    /// Failing 32-bit word's address mod 8: `(addr + (bit / 32) * 4) % 8`.
    pub word32_addr_mod_8: u8,
    /// Channel indicator from address bit 7 (`0` = channel A, `1` = channel B).
    pub channel: u8,
}

/// Decode one bit index (`0..64`) of a 64-bit word at `addr`.
pub const fn decode_bit(addr: u64, bit: u8) -> BitDecode {
    let byte_in_word = bit / 8;
    let bit_in_byte = bit % 8;
    let beat_in_word = bit / 16;
    let word_in_burst = ((addr % 32) / 8) as u8;
    let burst_beat = (word_in_burst * 4 + beat_in_word) % 16;
    let byte_addr_mod_8 = ((addr.wrapping_add(byte_in_word as u64)) % 8) as u8;
    let word32_off = (bit / 32) * 4;
    let word32_addr_mod_8 = ((addr.wrapping_add(word32_off as u64)) % 8) as u8;
    let channel = ((addr >> 7) & 1) as u8;
    BitDecode {
        bit,
        byte_in_word,
        bit_in_byte,
        beat_in_word,
        burst_beat,
        byte_addr_mod_8,
        word32_addr_mod_8,
        channel,
    }
}

/// Full decoded first-error record for a 64-bit word mismatch.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ErrorDecode {
    pub addr: u64,
    pub addr_mod_8: u8,
    pub wanted: u64,
    pub got: u64,
    pub xor: u64,
    pub bit_count: u8,
    pub bits: [u8; 64],
    pub first_bit: Option<BitDecode>,
}

impl ErrorDecode {
    /// Failing bit positions (`0..64`), ascending.
    pub fn failing_bits(&self) -> &[u8] {
        &self.bits[..self.bit_count as usize]
    }
}

/// Decode `(addr, wanted, got)` into failing bit positions, address mod 8,
/// beat index and channel.
pub fn decode_error(addr: u64, wanted: u64, got: u64) -> ErrorDecode {
    let xor = wanted ^ got;
    let mut bits = [0u8; 64];
    let mut bit_count = 0usize;
    let mut rem = xor;
    while rem != 0 {
        let b = rem.trailing_zeros() as u8;
        bits[bit_count] = b;
        bit_count += 1;
        rem &= rem - 1;
    }
    let first_bit = if bit_count > 0 {
        Some(decode_bit(addr, bits[0]))
    } else {
        None
    };
    ErrorDecode {
        addr,
        addr_mod_8: (addr % 8) as u8,
        wanted,
        got,
        xor,
        bit_count: bit_count as u8,
        bits,
        first_bit,
    }
}

/// A copy-pass block digest mismatch whose word-by-word rescan found no
/// persistent word error: counted as an error without decoding a physical bit.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct UnlocalizedMismatch {
    pub leg: &'static str,
    pub block_base: u64,
    pub block_len: u64,
    pub src_digest: [u8; 32],
    pub dst_digest: [u8; 32],
}

/// Per-pass mismatch summary: total errors, first decoded error, first
/// unlocalized block-hash mismatch, and error count per GiB of DRAM.
/// Implements `memtest::stress::Record` so `stress::Runner` populates it
/// directly.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PassReport {
    pub dram_base: u64,
    pub gib_count: usize,
    pub errors: u64,
    pub first: Option<ErrorDecode>,
    pub unlocalized_count: u64,
    pub first_unlocalized: Option<UnlocalizedMismatch>,
    pub per_gib: [u64; MAX_GIB],
    pub out_of_range: u64,
}

impl PassReport {
    pub fn new(dram_base: u64, dram_size: u64) -> Self {
        let gib_count = dram_size.div_ceil(1u64 << 30).min(MAX_GIB as u64) as usize;
        Self {
            dram_base,
            gib_count,
            errors: 0,
            first: None,
            unlocalized_count: 0,
            first_unlocalized: None,
            per_gib: [0; MAX_GIB],
            out_of_range: 0,
        }
    }

    pub fn gib_counts(&self) -> &[u64] {
        &self.per_gib[..self.gib_count]
    }

    /// Record an unlocalized block-hash mismatch (a copy-pass digest mismatch
    /// that disappeared on word-by-word rescan): counts as an error and in the
    /// block's GiB bucket without populating `self.first` or decoding any bit.
    pub fn record_unlocalized(&mut self, mismatch: UnlocalizedMismatch) {
        self.errors = self.errors.saturating_add(1);
        self.unlocalized_count = self.unlocalized_count.saturating_add(1);
        if self.first_unlocalized.is_none() {
            self.first_unlocalized = Some(mismatch);
        }
        if let Some(off) = mismatch.block_base.checked_sub(self.dram_base) {
            let gib = (off >> 30) as usize;
            if gib < self.gib_count {
                self.per_gib[gib] = self.per_gib[gib].saturating_add(1);
                return;
            }
        }
        self.out_of_range = self.out_of_range.saturating_add(1);
    }

    /// Fold another pass's totals into an aggregate report.
    pub fn merge(&mut self, other: &Self) {
        self.errors = self.errors.saturating_add(other.errors);
        if self.first.is_none() {
            self.first = other.first;
        }
        self.unlocalized_count = self
            .unlocalized_count
            .saturating_add(other.unlocalized_count);
        if self.first_unlocalized.is_none() {
            self.first_unlocalized = other.first_unlocalized;
        }
        for i in 0..MAX_GIB {
            self.per_gib[i] = self.per_gib[i].saturating_add(other.per_gib[i]);
        }
        self.out_of_range = self.out_of_range.saturating_add(other.out_of_range);
    }
}

impl Record for PassReport {
    fn record(&mut self, addr: u64, expected: u64, got: u64) {
        self.errors = self.errors.saturating_add(1);
        if self.first.is_none() {
            self.first = Some(decode_error(addr, expected, got));
        }
        if let Some(off) = addr.checked_sub(self.dram_base) {
            let gib = (off >> 30) as usize;
            if gib < self.gib_count {
                self.per_gib[gib] = self.per_gib[gib].saturating_add(1);
                return;
            }
        }
        self.out_of_range = self.out_of_range.saturating_add(1);
    }

    fn errors(&self) -> u64 {
        self.errors
    }
}

/// Which `1 0 x 0` beat shape to enforce on bits 5, 21, 37 and 53.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Bit37Shape {
    /// `1 0 1 0`: bit 5 = 1, bit 21 = 0, bit 37 = 1, bit 53 = 0.
    OneZeroOneZero,
    /// `1 0 0 0`: bit 5 = 1, bit 21 = 0, bit 37 = 0, bit 53 = 0.
    OneZeroZeroZero,
}

impl Bit37Shape {
    pub const fn name(self) -> &'static str {
        match self {
            Self::OneZeroOneZero => "bit37-1010",
            Self::OneZeroZeroZero => "bit37-1000",
        }
    }

    pub const fn fixed_bits(self) -> u64 {
        match self {
            Self::OneZeroOneZero => BIT37_SHAPE_1010,
            Self::OneZeroZeroZero => BIT37_SHAPE_1000,
        }
    }
}

/// Deterministic 64-bit word for the bit-37 pattern pass: bits 5, 21, 37 and 53
/// hold `shape`, and all other 60 bits come from `splitmix64(addr ^ seed)`.
#[inline(always)]
pub fn bit37_word(addr: u64, seed: u64, shape: Bit37Shape) -> u64 {
    (splitmix64(addr ^ seed) & !BIT37_LANE_MASK) | shape.fixed_bits()
}

fn pace_kick<M: Memory>(mem: &mut M, kick_ticks: u64, last_kick: &mut u64) {
    let now = mem.ticks();
    if now.wrapping_sub(*last_kick) >= kick_ticks {
        mem.kick();
        *last_kick = now;
    }
}

/// Run one cached fill-then-verify pass of `bit37_word` over `plan` in
/// `block`-byte cache-maintenance chunks (`DC CIVAC` after fill, `DC IVAC`
/// before verify).
pub fn run_bit37_pass<M: Memory>(
    mem: &mut M,
    plan: &Plan,
    block: u64,
    kick_ticks: u64,
    seed: u64,
    shape: Bit37Shape,
    report: &mut PassReport,
) -> Result<(), &'static str> {
    if block < CACHE_LINE_ALIGN || !block.is_multiple_of(CACHE_LINE_ALIGN) {
        return Err("block must be a non-zero multiple of 64 bytes");
    }
    let mut last_kick = mem.ticks();
    mem.kick();

    // Fill phase.
    for r in plan.ranges() {
        let mut bstart = r.start;
        while bstart < r.end {
            let bend = (bstart - bstart % block + block).min(r.end);
            let mut a = bstart;
            while a < bend {
                mem.write(a, bit37_word(a, seed, shape));
                a += 8;
            }
            mem.clean_invalidate(bstart, bend - bstart);
            pace_kick(mem, kick_ticks, &mut last_kick);
            bstart = bend;
        }
    }
    mem.complete();

    // Verify phase.
    for r in plan.ranges() {
        let mut bstart = r.start;
        while bstart < r.end {
            let bend = (bstart - bstart % block + block).min(r.end);
            mem.invalidate(bstart, bend - bstart);
            let mut a = bstart;
            while a < bend {
                let want = bit37_word(a, seed, shape);
                let got = mem.read(a);
                if got != want {
                    report.record(a, want, got);
                }
                a += 8;
            }
            pace_kick(mem, kick_ticks, &mut last_kick);
            bstart = bend;
        }
    }
    mem.kick();
    Ok(())
}

/// One source-to-destination block copy pair.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CopyPair {
    pub src: u64,
    pub dst: u64,
    pub len: u64,
}

/// One directional leg of the copy pass.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CopyLeg {
    pub name: &'static str,
    pub pairs: [CopyPair; COPY_BLOCKS_PER_LEG],
    pub count: usize,
}

impl CopyLeg {
    pub const fn new(name: &'static str) -> Self {
        Self {
            name,
            pairs: [CopyPair {
                src: 0,
                dst: 0,
                len: 0,
            }; COPY_BLOCKS_PER_LEG],
            count: 0,
        }
    }

    pub fn pairs(&self) -> &[CopyPair] {
        &self.pairs[..self.count]
    }

    fn push(&mut self, pair: CopyPair) {
        if self.count < COPY_BLOCKS_PER_LEG {
            self.pairs[self.count] = pair;
            self.count += 1;
        }
    }
}

/// Planned copy legs across ranks and above 4 GiB.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CopyPlan {
    pub r0_to_r1: CopyLeg,
    pub r1_to_r0: CopyLeg,
    pub r1_above4g: CopyLeg,
}

struct BlockPool {
    addrs: [u64; MAX_POOL_BLOCKS],
    len: usize,
}

impl BlockPool {
    const fn new() -> Self {
        Self {
            addrs: [0; MAX_POOL_BLOCKS],
            len: 0,
        }
    }

    fn push(&mut self, addr: u64) {
        if self.len < MAX_POOL_BLOCKS {
            self.addrs[self.len] = addr;
            self.len += 1;
        }
    }

    fn pick(&self, idx: usize) -> Option<u64> {
        if self.len == 0 {
            return None;
        }
        // Spread selections across the available blocks in the pool.
        let stride = (self.len / COPY_BLOCKS_PER_LEG).max(1);
        let pos = (idx * stride) % self.len;
        Some(self.addrs[pos])
    }
}

/// Plan 16 MiB (or `block_size`) non-overlapping copy pairs inside `plan` for:
/// 1. rank 0 -> rank 1 (covering both `<4 GiB` and `>=4 GiB` rank 1 when present);
/// 2. rank 1 -> rank 0 (from both `<4 GiB` and `>=4 GiB` rank 1 when present);
/// 3. within rank 1 above 4 GiB (disjoint `src != dst` pairs).
pub fn build_copy_plan(
    plan: &Plan,
    dram_base: u64,
    rank_bit: Option<u32>,
    block_size: u64,
) -> CopyPlan {
    let mut out = CopyPlan {
        r0_to_r1: CopyLeg::new("copy-r0-to-r1"),
        r1_to_r0: CopyLeg::new("copy-r1-to-r0"),
        r1_above4g: CopyLeg::new("copy-r1-above4g"),
    };
    if block_size < CACHE_LINE_ALIGN || !block_size.is_power_of_two() {
        return out;
    }

    let mut r0 = BlockPool::new();
    let mut r1_lo = BlockPool::new();
    let mut r1_hi = BlockPool::new();

    for r in plan.ranges() {
        let mut a = (r.start + block_size - 1) & !(block_size - 1);
        while a.checked_add(block_size).is_some_and(|e| e <= r.end) {
            let rk_start = rank_of_addr(a, dram_base, rank_bit);
            let rk_end = rank_of_addr(a + block_size - 8, dram_base, rank_bit);
            let above_start = a >= ABOVE_4G;
            let above_end = (a + block_size - 8) >= ABOVE_4G;
            if rk_start == rk_end && above_start == above_end {
                if rk_start == 0 {
                    r0.push(a);
                } else if above_start {
                    r1_hi.push(a);
                } else {
                    r1_lo.push(a);
                }
            }
            a += block_size;
        }
    }

    let pick_r1 = |i: usize| -> Option<u64> {
        match (r1_lo.len > 0, r1_hi.len > 0) {
            (true, true) => {
                if i.is_multiple_of(2) {
                    r1_lo.pick(i / 2)
                } else {
                    r1_hi.pick(i / 2)
                }
            }
            (true, false) => r1_lo.pick(i),
            (false, true) => r1_hi.pick(i),
            (false, false) => None,
        }
    };

    // Leg 1: rank 0 -> rank 1.
    if r0.len > 0 && (r1_lo.len > 0 || r1_hi.len > 0) {
        let n = COPY_BLOCKS_PER_LEG.min(r0.len).min(r1_lo.len + r1_hi.len);
        for i in 0..n {
            if let (Some(src), Some(dst)) = (r0.pick(i), pick_r1(i)) {
                if src != dst {
                    out.r0_to_r1.push(CopyPair {
                        src,
                        dst,
                        len: block_size,
                    });
                }
            }
        }
    }

    // Leg 2: rank 1 -> rank 0.
    if r0.len > 0 && (r1_lo.len > 0 || r1_hi.len > 0) {
        let n = COPY_BLOCKS_PER_LEG.min(r0.len).min(r1_lo.len + r1_hi.len);
        for i in 0..n {
            // Offset the rank-0 index by 1 when possible so destinations differ
            // from Leg 1's sources.
            let dst_idx = if r0.len > 1 { i + 1 } else { i };
            if let (Some(src), Some(dst)) = (pick_r1(i), r0.pick(dst_idx)) {
                if src != dst {
                    out.r1_to_r0.push(CopyPair {
                        src,
                        dst,
                        len: block_size,
                    });
                }
            }
        }
    }

    // Leg 3: within rank 1 above 4 GiB (requires at least two disjoint blocks).
    if r1_hi.len >= 2 {
        let half = r1_hi.len / 2;
        let n = COPY_BLOCKS_PER_LEG.min(half);
        let stride = (half / n).max(1);
        for i in 0..n {
            let src = r1_hi.addrs[i * stride];
            let dst = r1_hi.addrs[half + i * stride];
            if src != dst {
                out.r1_above4g.push(CopyPair {
                    src,
                    dst,
                    len: block_size,
                });
            }
        }
    }

    out
}

/// Deterministic 64-bit word for copy-block offset `off` (independent of the
/// block's base address so source and destination hold identical bytes).
#[inline(always)]
pub fn copy_word(seed: u64, off: u64) -> u64 {
    splitmix64(off ^ seed)
}

/// Chained SHA-256 over `[base, base + len)` read from `mem` in 1 KiB chunks.
pub fn hash_region<M: Memory>(
    mem: &mut M,
    base: u64,
    len: u64,
    kick_ticks: u64,
    last_kick: &mut u64,
) -> [u8; 32] {
    let mut acc = sha256::digest(&len.to_le_bytes());
    let mut buf = [0u8; 32 + HASH_CHUNK_BYTES];
    let mut off = 0u64;
    while off < len {
        let chunk = ((len - off) as usize).min(HASH_CHUNK_BYTES);
        buf[..32].copy_from_slice(&acc);
        let mut p = 0usize;
        while p < chunk {
            let w = mem.read(base + off + p as u64);
            buf[32 + p..32 + p + 8].copy_from_slice(&w.to_le_bytes());
            p += 8;
        }
        acc = sha256::digest(&buf[..32 + chunk]);
        off += chunk as u64;
        if off.is_multiple_of(STRESS_BLOCK_BYTES) {
            pace_kick(mem, kick_ticks, last_kick);
        }
    }
    acc
}

/// Expected chained SHA-256 of a `len`-byte block filled with `copy_word(seed, off)`.
pub fn hash_expected(len: u64, seed: u64) -> [u8; 32] {
    let mut acc = sha256::digest(&len.to_le_bytes());
    let mut buf = [0u8; 32 + HASH_CHUNK_BYTES];
    let mut off = 0u64;
    while off < len {
        let chunk = ((len - off) as usize).min(HASH_CHUNK_BYTES);
        buf[..32].copy_from_slice(&acc);
        let mut p = 0usize;
        while p < chunk {
            let w = copy_word(seed, off + p as u64);
            buf[32 + p..32 + p + 8].copy_from_slice(&w.to_le_bytes());
            p += 8;
        }
        acc = sha256::digest(&buf[..32 + chunk]);
        off += chunk as u64;
    }
    acc
}

fn clean_invalidate_range<M: Memory>(
    mem: &mut M,
    base: u64,
    len: u64,
    block: u64,
    kick_ticks: u64,
    last_kick: &mut u64,
) {
    let mut off = 0u64;
    while off < len {
        let n = (len - off).min(block);
        mem.clean_invalidate(base + off, n);
        pace_kick(mem, kick_ticks, last_kick);
        off += n;
    }
}

fn invalidate_range<M: Memory>(
    mem: &mut M,
    base: u64,
    len: u64,
    block: u64,
    kick_ticks: u64,
    last_kick: &mut u64,
) {
    let mut off = 0u64;
    while off < len {
        let n = (len - off).min(block);
        mem.invalidate(base + off, n);
        pace_kick(mem, kick_ticks, last_kick);
        off += n;
    }
}

/// Summary counters returned by [`run_copy_leg`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CopyLegStats {
    pub blocks: usize,
    pub bytes: u64,
    pub hash_mismatches: u64,
    pub last_src_hash: [u8; 32],
    pub last_dst_hash: [u8; 32],
}

/// Run one copy leg: fill source -> flush -> stream-copy `src -> dst` ->
/// flush destination -> invalidate and hash both `dst` and `src` from DRAM ->
/// compare `dst_hash == src_hash == expected_hash`. On any hash mismatch,
/// scan word-by-word so `report` captures exact error counts, first address,
/// bit/beat decode and per-GiB counts.
pub fn run_copy_leg<M: Memory>(
    mem: &mut M,
    leg: &CopyLeg,
    maint_block: u64,
    kick_ticks: u64,
    seed_base: u64,
    report: &mut PassReport,
) -> Result<CopyLegStats, &'static str> {
    if maint_block < CACHE_LINE_ALIGN || !maint_block.is_multiple_of(CACHE_LINE_ALIGN) {
        return Err("maint_block must be a non-zero multiple of 64 bytes");
    }
    let mut last_kick = mem.ticks();
    mem.kick();
    let mut stats = CopyLegStats::default();

    for (idx, pair) in leg.pairs().iter().copied().enumerate() {
        if pair.len == 0 || !pair.len.is_multiple_of(CACHE_LINE_ALIGN) {
            return Err("copy block length must be a non-zero multiple of 64 bytes");
        }
        let seed = seed_base.wrapping_add((idx as u64).wrapping_mul(0x9e37_79b9_7f4a_7c15));

        // 1. Fill source and push it to DRAM.
        let mut off = 0u64;
        while off < pair.len {
            let chunk_end = (off + maint_block).min(pair.len);
            let mut p = off;
            while p < chunk_end {
                mem.write(pair.src + p, copy_word(seed, p));
                p += 8;
            }
            mem.clean_invalidate(pair.src + off, chunk_end - off);
            pace_kick(mem, kick_ticks, &mut last_kick);
            off = chunk_end;
        }
        mem.complete();

        // 2. Evict any cached lines for both source and destination before streaming.
        invalidate_range(
            mem,
            pair.src,
            pair.len,
            maint_block,
            kick_ticks,
            &mut last_kick,
        );
        invalidate_range(
            mem,
            pair.dst,
            pair.len,
            maint_block,
            kick_ticks,
            &mut last_kick,
        );

        // 3. Stream-copy src -> dst across the full block so L1/L2/L3 overflow
        //    drives concurrent line fills from src and write-back bursts to dst.
        let mut off = 0u64;
        while off < pair.len {
            let v = mem.read(pair.src + off);
            mem.write(pair.dst + off, v);
            off += 8;
            if off.is_multiple_of(maint_block) {
                pace_kick(mem, kick_ticks, &mut last_kick);
            }
        }

        // 4. Push destination to DRAM and evict both blocks from cache.
        clean_invalidate_range(
            mem,
            pair.dst,
            pair.len,
            maint_block,
            kick_ticks,
            &mut last_kick,
        );
        invalidate_range(
            mem,
            pair.src,
            pair.len,
            maint_block,
            kick_ticks,
            &mut last_kick,
        );
        mem.complete();

        // 5. Re-read both blocks from DRAM and hash destination against source.
        invalidate_range(
            mem,
            pair.dst,
            pair.len,
            maint_block,
            kick_ticks,
            &mut last_kick,
        );
        let dst_hash = hash_region(mem, pair.dst, pair.len, kick_ticks, &mut last_kick);

        invalidate_range(
            mem,
            pair.src,
            pair.len,
            maint_block,
            kick_ticks,
            &mut last_kick,
        );
        let src_hash = hash_region(mem, pair.src, pair.len, kick_ticks, &mut last_kick);

        let exp_hash = hash_expected(pair.len, seed);
        stats.blocks += 1;
        stats.bytes += pair.len;
        stats.last_src_hash = src_hash;
        stats.last_dst_hash = dst_hash;

        if dst_hash != src_hash || dst_hash != exp_hash {
            stats.hash_mismatches += 1;
            let before = report.errors;
            invalidate_range(
                mem,
                pair.dst,
                pair.len,
                maint_block,
                kick_ticks,
                &mut last_kick,
            );
            invalidate_range(
                mem,
                pair.src,
                pair.len,
                maint_block,
                kick_ticks,
                &mut last_kick,
            );
            let mut off = 0u64;
            while off < pair.len {
                let want = copy_word(seed, off);
                let got_dst = mem.read(pair.dst + off);
                if got_dst != want {
                    report.record(pair.dst + off, want, got_dst);
                }
                let got_src = mem.read(pair.src + off);
                if got_src != want {
                    report.record(pair.src + off, want, got_src);
                }
                off += 8;
                if off.is_multiple_of(maint_block) {
                    pace_kick(mem, kick_ticks, &mut last_kick);
                }
            }
            if report.errors == before {
                // Hash differed during streaming read, then rescan passed
                // (transient read error): record an unlocalized block-hash
                // mismatch with both digests without feeding digest bytes into
                // the physical bit/address decoder.
                report.record_unlocalized(UnlocalizedMismatch {
                    leg: leg.name,
                    block_base: pair.dst,
                    block_len: pair.len,
                    src_digest: src_hash,
                    dst_digest: dst_hash,
                });
            }
        }
    }
    mem.kick();
    Ok(stats)
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use ::memtest::arch::aarch64::{l1_table, AF, ATTR1, BLOCK, SH_INNER, XN};
    use std::collections::BTreeMap;

    const GIB: u64 = 1u64 << 30;
    const MIB: u64 = 1u64 << 20;

    fn sample_handoff(dram_size: u64, payload_size: u64, dtb: Option<(u64, u64)>) -> Handoff {
        let (dtb_addr, dtb_size) = dtb.unwrap_or((0, 0));
        Handoff {
            magic: crate::HANDOFF_MAGIC,
            version: crate::HANDOFF_VERSION,
            dram_base: mem::DRAM_BASE as u64,
            dram_size,
            soc_version: 0,
            hosc_mhz: 24,
            bt0_done_ticks: 0,
            payload_addr: mem::PAYLOAD_BASE as u64,
            payload_size,
            dtb_addr,
            dtb_size,
            cpu_mhz: [1416, 1800, 1200],
            dram_mhz: 2400,
        }
    }

    struct FakeMem {
        words: BTreeMap<u64, u64>,
        tick: u64,
        kicks: usize,
        clean_inv_calls: usize,
        inv_calls: usize,
        completes: usize,
        fault_read: Option<(u64, u64)>,
        fault_once: bool,
    }

    impl FakeMem {
        fn new() -> Self {
            Self {
                words: BTreeMap::new(),
                tick: 0,
                kicks: 0,
                clean_inv_calls: 0,
                inv_calls: 0,
                completes: 0,
                fault_read: None,
                fault_once: false,
            }
        }
    }

    impl Memory for FakeMem {
        fn write(&mut self, addr: u64, value: u64) {
            self.words.insert(addr, value);
        }

        fn read(&mut self, addr: u64) -> u64 {
            let mut v = *self.words.get(&addr).unwrap_or(&0);
            if let Some((fa, mask)) = self.fault_read {
                if fa == addr {
                    v ^= mask;
                    if self.fault_once {
                        self.fault_read = None;
                    }
                }
            }
            v
        }

        fn complete(&mut self) {
            self.completes += 1;
        }

        fn ticks(&mut self) -> u64 {
            self.tick += 1;
            self.tick
        }

        fn kick(&mut self) {
            self.kicks += 1;
        }

        fn clean_invalidate(&mut self, _addr: u64, _len: u64) {
            self.clean_inv_calls += 1;
        }

        fn invalidate(&mut self, _addr: u64, _len: u64) {
            self.inv_calls += 1;
        }
    }

    #[test]
    fn bit37_pattern_enforces_beat_shape_and_randomizes_other_bits() {
        let mut or_other_1010 = 0u64;
        let mut and_other_1010 = !BIT37_LANE_MASK;
        let mut or_other_1000 = 0u64;
        let mut and_other_1000 = !BIT37_LANE_MASK;

        for i in 0..1024u64 {
            let addr = 0x4020_0000 + i * 8;
            let w1 = bit37_word(addr, BIT37_SEED_1010, Bit37Shape::OneZeroOneZero);
            let w0 = bit37_word(addr, BIT37_SEED_1000, Bit37Shape::OneZeroZeroZero);

            // Bits 5, 21, 37, 53 must be exactly 1, 0, 1, 0 for OneZeroOneZero.
            assert_eq!(w1 & BIT37_LANE_MASK, BIT37_SHAPE_1010);
            assert_eq!((w1 >> 5) & 1, 1);
            assert_eq!((w1 >> 21) & 1, 0);
            assert_eq!((w1 >> 37) & 1, 1);
            assert_eq!((w1 >> 53) & 1, 0);

            // Bits 5, 21, 37, 53 must be exactly 1, 0, 0, 0 for OneZeroZeroZero.
            assert_eq!(w0 & BIT37_LANE_MASK, BIT37_SHAPE_1000);
            assert_eq!((w0 >> 5) & 1, 1);
            assert_eq!((w0 >> 21) & 1, 0);
            assert_eq!((w0 >> 37) & 1, 0);
            assert_eq!((w0 >> 53) & 1, 0);

            or_other_1010 |= w1 & !BIT37_LANE_MASK;
            and_other_1010 &= w1 & !BIT37_LANE_MASK;
            or_other_1000 |= w0 & !BIT37_LANE_MASK;
            and_other_1000 &= w0 & !BIT37_LANE_MASK;
        }

        // Every non-lane bit must vary across addresses (both 0 and 1 seen).
        assert_eq!(or_other_1010, !BIT37_LANE_MASK);
        assert_eq!(and_other_1010, 0);
        assert_eq!(or_other_1000, !BIT37_LANE_MASK);
        assert_eq!(and_other_1000, 0);

        // Deterministic from address and seed alone.
        let a = 0x1_0000_0080;
        assert_eq!(
            bit37_word(a, BIT37_SEED_1010, Bit37Shape::OneZeroOneZero),
            bit37_word(a, BIT37_SEED_1010, Bit37Shape::OneZeroOneZero)
        );
        assert_eq!(Bit37Shape::OneZeroOneZero.name(), "bit37-1010");
        assert_eq!(Bit37Shape::OneZeroZeroZero.name(), "bit37-1000");
    }

    #[test]
    fn bit_and_beat_decoder_matches_bit37_geometry() {
        // Bit 37 on all four 64-bit word offsets within a 32-byte burst must
        // map to burst beats 2, 6, 10 and 14, byte_addr % 8 == 4, w32 % 8 == 4.
        for (word_idx, expected_burst_beat) in [(0u64, 2u8), (1, 6), (2, 10), (3, 14)] {
            let addr = 0x1_0000_0000 + word_idx * 8;
            let want = bit37_word(addr, BIT37_SEED_1010, Bit37Shape::OneZeroOneZero);
            let got = want ^ (1u64 << 37);
            let d = decode_error(addr, want, got);
            assert_eq!(d.addr, addr);
            assert_eq!(d.addr_mod_8, 0);
            assert_eq!(d.xor, 1u64 << 37);
            assert_eq!(d.failing_bits(), &[37]);
            let b = d.first_bit.unwrap();
            assert_eq!(b.bit, 37);
            assert_eq!(b.byte_in_word, 4);
            assert_eq!(b.bit_in_byte, 5);
            assert_eq!(b.beat_in_word, 2);
            assert_eq!(b.burst_beat, expected_burst_beat);
            assert_eq!(b.byte_addr_mod_8, 4);
            assert_eq!(b.word32_addr_mod_8, 4);
            assert_eq!(b.channel, 0);
        }

        // Channel B is selected by address bit 7 (0x80).
        let d_ch_b = decode_error(0x1_0000_0088, 1u64 << 37, 0);
        assert_eq!(d_ch_b.first_bit.unwrap().channel, 1);
        assert_eq!(d_ch_b.first_bit.unwrap().burst_beat, 6);

        // Multiple failing bits are listed in ascending order.
        let d_multi = decode_error(0x4020_0000, 0, (1 << 5) | (1 << 21) | (1 << 37) | (1 << 53));
        assert_eq!(d_multi.failing_bits(), &[5, 21, 37, 53]);
        assert_eq!(d_multi.first_bit.unwrap().byte_addr_mod_8, 0);
        assert_eq!(decode_bit(0x4020_0000, 21).byte_addr_mod_8, 2);
        assert_eq!(decode_bit(0x4020_0000, 53).byte_addr_mod_8, 6);

        // Non-zero addr mod 8 is preserved directly.
        let d_unaligned = decode_error(0x4020_0004, 1 << 5, 0);
        assert_eq!(d_unaligned.addr_mod_8, 4);
        assert_eq!(d_unaligned.first_bit.unwrap().byte_addr_mod_8, 4);
    }

    #[test]
    fn pass_report_tracks_first_error_and_per_gib_counts() {
        let mut r = PassReport::new(mem::DRAM_BASE as u64, 4 * GIB);
        assert_eq!(r.gib_counts(), &[0, 0, 0, 0]);

        // GiB 0 (0x4000_0000..0x8000_0000)
        r.record(0x4020_0000, 0xaa, 0xab);
        // GiB 2 (rank 1 below 4 GiB: 0xc000_0000..0x1_0000_0000)
        r.record(0xc000_0080, 1 << 37, 0);
        // GiB 3 (rank 1 above 4 GiB: 0x1_0000_0000..0x1_4000_0000)
        r.record(0x1_0000_0000, 1 << 37, 0);
        r.record(0x1_3fff_fff8, 1 << 37, 0);

        assert_eq!(r.errors, 4);
        assert_eq!(r.gib_counts(), &[1, 0, 1, 2]);
        assert_eq!(r.first.unwrap().addr, 0x4020_0000);
        assert_eq!(r.first.unwrap().failing_bits(), &[0]);

        let mut total = PassReport::new(mem::DRAM_BASE as u64, 4 * GIB);
        total.merge(&r);
        let mut r2 = PassReport::new(mem::DRAM_BASE as u64, 4 * GIB);
        r2.record(0x8000_0000, 1, 0);
        total.merge(&r2);
        assert_eq!(total.errors, 5);
        assert_eq!(total.gib_counts(), &[1, 1, 1, 2]);
        assert_eq!(total.first.unwrap().addr, 0x4020_0000);
    }

    #[test]
    fn cached_plan_honours_exclusions_cache_lines_both_ranks_and_above_4g() {
        // 4 GB CXMT board: DRAM 0x4000_0000..0x1_4000_0000, ADDRMAP1 low byte
        // 0x17 -> rank bit 31 (cubie-a7a-dram/src/ctrl.rs:440, 450-462).
        let rank_bit_4g = rank_bit_from_addrmap1(0x3f17, DUAL_CH_SHIFT);
        assert_eq!(rank_bit_4g, Some(31));
        assert_eq!(
            validate_live_rank_map(0x3f17, 4 * GIB),
            Ok(LiveRankMap::DualRank { rank_bit: 31 })
        );

        // Unaligned payload/DTB sizes test outward 64-byte cache-line rounding.
        let payload_size = 4 * MIB - 13;
        let dtb_addr = 0x4800_0010;
        let dtb_size = 0x1234;
        let h = sample_handoff(4 * GIB, payload_size, Some((dtb_addr, dtb_size)));
        let plan = cached_plan_from_handoff(&h).unwrap();

        let el3 = Region::new(mem::EL3_BASE as u64, mem::EL3_SIZE as u64).unwrap();
        let payload = Region::covering(h.payload_addr, h.payload_size).unwrap();
        let dtb = Region::covering(dtb_addr, dtb_size).unwrap();

        let mut covers_r0 = false;
        let mut covers_r1_lo = false;
        let mut covers_r1_hi = false;

        for r in plan.ranges() {
            assert!(r.start.is_multiple_of(CACHE_LINE_ALIGN));
            assert!(r.end.is_multiple_of(CACHE_LINE_ALIGN));
            assert!(!r.overlaps(el3));
            assert!(!r.overlaps(payload));
            assert!(!r.overlaps(dtb));
            // Must not even share a 64-byte cache line with any exclusion.
            for ex in [el3, payload, dtb] {
                let ex_lo = ex.start & !(CACHE_LINE_ALIGN - 1);
                let ex_hi = (ex.end + CACHE_LINE_ALIGN - 1) & !(CACHE_LINE_ALIGN - 1);
                assert!(r.end <= ex_lo || r.start >= ex_hi);
            }
            if r.start < 0xc000_0000 {
                covers_r0 = true;
            }
            if r.start < ABOVE_4G && r.end > 0xc000_0000 {
                covers_r1_lo = true;
            }
            if r.end > ABOVE_4G {
                covers_r1_hi = true;
            }
        }
        assert!(covers_r0);
        assert!(covers_r1_lo);
        assert!(covers_r1_hi);
        // End of DRAM above 4 GiB is reached exactly.
        assert_eq!(plan.ranges().last().unwrap().end, 0x1_4000_0000);
        assert_eq!(rank_of_addr(0x4020_0000, h.dram_base, rank_bit_4g), 0);
        assert_eq!(rank_of_addr(0xbfff_fff8, h.dram_base, rank_bit_4g), 0);
        assert_eq!(rank_of_addr(0xc000_0000, h.dram_base, rank_bit_4g), 1);
        assert_eq!(rank_of_addr(0x1_0000_0000, h.dram_base, rank_bit_4g), 1);
        assert_eq!(rank_of_addr(0x1_3fff_fff8, h.dram_base, rank_bit_4g), 1);
    }

    #[test]
    fn copy_plan_covers_all_three_legs_on_4g_and_6g_maps() {
        // 4 GB CXMT map (rank bit 31, ctrl.rs:450-462).
        let h4 = sample_handoff(4 * GIB, 4 * MIB, None);
        let plan4 = cached_plan_from_handoff(&h4).unwrap();
        let rb4 = rank_bit_from_addrmap1(0x3f17, DUAL_CH_SHIFT);
        let cp4 = build_copy_plan(&plan4, h4.dram_base, rb4, COPY_BLOCK_BYTES);

        assert_eq!(cp4.r0_to_r1.count, COPY_BLOCKS_PER_LEG);
        assert_eq!(cp4.r1_to_r0.count, COPY_BLOCKS_PER_LEG);
        assert_eq!(cp4.r1_above4g.count, COPY_BLOCKS_PER_LEG);

        let in_plan = |p: &Plan, addr: u64, len: u64| {
            p.ranges()
                .iter()
                .any(|r| addr >= r.start && addr + len <= r.end)
        };

        let mut r0_to_r1_hits_above4g = false;
        for p in cp4.r0_to_r1.pairs() {
            assert_eq!(p.len, COPY_BLOCK_BYTES);
            assert!(in_plan(&plan4, p.src, p.len));
            assert!(in_plan(&plan4, p.dst, p.len));
            assert_eq!(rank_of_addr(p.src, h4.dram_base, rb4), 0);
            assert_eq!(rank_of_addr(p.dst, h4.dram_base, rb4), 1);
            if p.dst >= ABOVE_4G {
                r0_to_r1_hits_above4g = true;
            }
        }
        assert!(r0_to_r1_hits_above4g);

        let mut r1_to_r0_hits_above4g = false;
        for p in cp4.r1_to_r0.pairs() {
            assert_eq!(p.len, COPY_BLOCK_BYTES);
            assert!(in_plan(&plan4, p.src, p.len));
            assert!(in_plan(&plan4, p.dst, p.len));
            assert_eq!(rank_of_addr(p.src, h4.dram_base, rb4), 1);
            assert_eq!(rank_of_addr(p.dst, h4.dram_base, rb4), 0);
            if p.src >= ABOVE_4G {
                r1_to_r0_hits_above4g = true;
            }
        }
        assert!(r1_to_r0_hits_above4g);

        for p in cp4.r1_above4g.pairs() {
            assert_eq!(p.len, COPY_BLOCK_BYTES);
            assert_ne!(p.src, p.dst);
            assert!(in_plan(&plan4, p.src, p.len));
            assert!(in_plan(&plan4, p.dst, p.len));
            assert!(p.src >= ABOVE_4G && p.dst >= ABOVE_4G);
            assert_eq!(rank_of_addr(p.src, h4.dram_base, rb4), 1);
            assert_eq!(rank_of_addr(p.dst, h4.dram_base, rb4), 1);
        }

        // 6 GB CXMT map (ADDRMAP1 = 0x3f16 -> rank bit 30, ctrl.rs:478, 503-516).
        let h6 = sample_handoff(6 * GIB, 4 * MIB, None);
        let plan6 = cached_plan_from_handoff(&h6).unwrap();
        let rb6 = rank_bit_from_addrmap1(0x3f16, DUAL_CH_SHIFT);
        assert_eq!(rb6, Some(30));
        let cp6 = build_copy_plan(&plan6, h6.dram_base, rb6, COPY_BLOCK_BYTES);
        assert_eq!(cp6.r0_to_r1.count, COPY_BLOCKS_PER_LEG);
        assert_eq!(cp6.r1_to_r0.count, COPY_BLOCKS_PER_LEG);
        assert_eq!(cp6.r1_above4g.count, COPY_BLOCKS_PER_LEG);

        // Single-rank map (ADDRMAP1 = 0x3f3f -> None, ctrl.rs:538-546).
        let rb1 = rank_bit_from_addrmap1(0x3f3f, DUAL_CH_SHIFT);
        assert_eq!(rb1, None);
        let cp1 = build_copy_plan(&plan4, h4.dram_base, rb1, COPY_BLOCK_BYTES);
        assert_eq!(cp1.r0_to_r1.count, 0);
        assert_eq!(cp1.r1_to_r0.count, 0);
        assert_eq!(cp1.r1_above4g.count, 0);
    }

    #[test]
    fn bit37_pass_and_copy_pass_detect_injected_errors() {
        let mut mem = FakeMem::new();
        let dram = Region::new(0x4000_0000, 16 * 1024).unwrap();
        let plan = Plan::new(dram, &[]).unwrap();
        let mut rep = PassReport::new(0x4000_0000, 4 * GIB);

        // Clean bit37 pass exercises clean_invalidate and invalidate.
        run_bit37_pass(
            &mut mem,
            &plan,
            4096,
            100,
            BIT37_SEED_1010,
            Bit37Shape::OneZeroOneZero,
            &mut rep,
        )
        .unwrap();
        assert_eq!(rep.errors, 0);
        assert_eq!(mem.clean_inv_calls, 4);
        assert_eq!(mem.inv_calls, 4);
        assert_eq!(mem.completes, 1);

        // Inject a bit-37 flip at 0x4000_0080 (channel B, burst beat 2).
        mem.fault_read = Some((0x4000_0080, 1u64 << 37));
        let mut rep_err = PassReport::new(0x4000_0000, 4 * GIB);
        run_bit37_pass(
            &mut mem,
            &plan,
            4096,
            100,
            BIT37_SEED_1010,
            Bit37Shape::OneZeroOneZero,
            &mut rep_err,
        )
        .unwrap();
        assert_eq!(rep_err.errors, 1);
        assert_eq!(rep_err.gib_counts(), &[1, 0, 0, 0]);
        let first = rep_err.first.unwrap();
        assert_eq!(first.addr, 0x4000_0080);
        assert_eq!(first.failing_bits(), &[37]);
        assert_eq!(first.first_bit.unwrap().byte_addr_mod_8, 4);
        assert_eq!(first.first_bit.unwrap().channel, 1);
        mem.fault_read = None;

        // Clean copy-pass hash compare.
        let mut leg = CopyLeg::new("test-copy");
        leg.push(CopyPair {
            src: 0x4000_0000,
            dst: 0x1_0000_0000,
            len: 4096,
        });
        let mut copy_ok = PassReport::new(0x4000_0000, 4 * GIB);
        let stats_ok =
            run_copy_leg(&mut mem, &leg, 1024, 100, COPY_SEED_BASE, &mut copy_ok).unwrap();
        assert_eq!(copy_ok.errors, 0);
        assert_eq!(stats_ok.blocks, 1);
        assert_eq!(stats_ok.bytes, 4096);
        assert_eq!(stats_ok.hash_mismatches, 0);
        assert_eq!(stats_ok.last_src_hash, stats_ok.last_dst_hash);

        // Persistent destination flip is caught by hash compare and rescan.
        mem.fault_read = Some((0x1_0000_0108, 1u64 << 37));
        let mut copy_bad = PassReport::new(0x4000_0000, 4 * GIB);
        let stats_bad =
            run_copy_leg(&mut mem, &leg, 1024, 100, COPY_SEED_BASE, &mut copy_bad).unwrap();
        assert_eq!(stats_bad.hash_mismatches, 1);
        assert_ne!(stats_bad.last_src_hash, stats_bad.last_dst_hash);
        assert_eq!(copy_bad.errors, 1);
        assert_eq!(copy_bad.gib_counts(), &[0, 0, 0, 1]);
        let cf = copy_bad.first.unwrap();
        assert_eq!(cf.addr, 0x1_0000_0108);
        assert_eq!(cf.failing_bits(), &[37]);
        assert_eq!(cf.first_bit.unwrap().burst_beat, 6);

        // Transient single-read flip during dst hash yields an unlocalized
        // block-hash mismatch, counts as an error, and decodes no physical bit.
        mem.fault_read = Some((0x1_0000_0200, 1u64 << 37));
        mem.fault_once = true;
        let mut copy_transient = PassReport::new(0x4000_0000, 4 * GIB);
        let stats_tr = run_copy_leg(
            &mut mem,
            &leg,
            1024,
            100,
            COPY_SEED_BASE,
            &mut copy_transient,
        )
        .unwrap();
        assert_eq!(stats_tr.hash_mismatches, 1);
        assert_eq!(copy_transient.errors, 1);
        assert_eq!(copy_transient.gib_counts(), &[0, 0, 0, 1]);
        assert!(copy_transient.first.is_none());
        assert_eq!(copy_transient.unlocalized_count, 1);
        let u = copy_transient.first_unlocalized.unwrap();
        assert_eq!(u.leg, "test-copy");
        assert_eq!(u.block_base, 0x1_0000_0000);
        assert_eq!(u.block_len, 4096);
        assert_eq!(u.src_digest, stats_tr.last_src_hash);
        assert_eq!(u.dst_digest, stats_tr.last_dst_hash);
        assert_ne!(u.src_digest, u.dst_digest);
    }

    #[test]
    fn flat_map_descriptors_and_mair_wb_match_spec() {
        let mut table = [0u64; 512];
        l1_table(&mut table, 0x1_4000_0000, Some(1)).unwrap();

        // Block 0: MMIO Device-nGnRnE (AttrIndx 0), execute-never.
        assert_eq!(table[0], BLOCK | AF | XN);
        // Block 1: DRAM 0x4000_0000..0x8000_0000, AttrIndx 1, executable.
        assert_eq!(table[1], (1u64 << 30) | BLOCK | AF | SH_INNER | ATTR1);
        // Blocks 2..=4: DRAM up to 0x1_4000_0000 (including above 4 GiB), XN.
        for (gib, &entry) in table.iter().enumerate().take(5).skip(2) {
            assert_eq!(
                entry,
                ((gib as u64) << 30) | BLOCK | AF | SH_INNER | ATTR1 | XN
            );
        }
        assert_eq!(table[5], 0);
        // MAIR_WB puts 0x00 in attr0 (Device) and 0xff in attr1 (Normal WB).
        assert_eq!(MAIR_WB & 0xff, 0x00);
        assert_eq!((MAIR_WB >> 8) & 0xff, 0xff);
        // SCTLR_EL3.M (0), .C (2), .I (12) and channel 0 ADDRMAP1 MMIO address.
        assert_eq!(SCTLR_M | SCTLR_C | SCTLR_I, 0x1005);
        assert_eq!(ADDRMAP1_CH0, 0x0a10_0000 + 0x3_0000 + 4);
    }

    #[test]
    fn cached_plan_rejects_malformed_handoffs_and_merges_shared_lines() {
        let mut bad_base = sample_handoff(4 * GIB, 4 * MIB, None);
        bad_base.dram_base = 0x5000_0000;
        assert!(cached_plan_from_handoff(&bad_base).is_err());

        let bad_zero = sample_handoff(0, 4 * MIB, None);
        assert!(cached_plan_from_handoff(&bad_zero).is_err());

        let bad_align = sample_handoff(4 * GIB + 8, 4 * MIB, None);
        assert!(cached_plan_from_handoff(&bad_align).is_err());

        let bad_dtb = sample_handoff(4 * GIB, 4 * MIB, Some((0x4800_0000, 0)));
        assert!(cached_plan_from_handoff(&bad_dtb).is_err());

        let bad_dtb2 = sample_handoff(4 * GIB, 4 * MIB, Some((0, 4096)));
        assert!(cached_plan_from_handoff(&bad_dtb2).is_err());

        // Payload ending at +0x10 and DTB starting at +0x20 are disjoint in
        // bytes but share one 64-byte cache line; outward rounding merges them.
        let shared = sample_handoff(4 * GIB, 0x1010, Some((0x4100_1020, 0x20)));
        let plan = cached_plan_from_handoff(&shared).unwrap();
        for r in plan.ranges() {
            assert!(r.start.is_multiple_of(CACHE_LINE_ALIGN));
            assert!(r.end.is_multiple_of(CACHE_LINE_ALIGN));
            assert!(r.end <= 0x4100_0000 || r.start >= 0x4100_1040);
        }

        // Unreadable or out-of-range rank decodes fail validation explicitly.
        assert!(validate_live_rank_map(0, 4 * GIB).is_err());
        assert!(validate_live_rank_map(u32::MAX, 4 * GIB).is_err());
        assert!(validate_live_rank_map(0x3f18, 4 * GIB).is_err());
    }

    #[test]
    fn verdict_requires_valid_map_and_all_three_copy_legs() {
        let full_cov = RunCoverage {
            stress_ran_both_ranks: true,
            march_ran_both_ranks: true,
            bit37_ran_both_ranks: true,
            copy_r0_to_r1_bytes: 64 * MIB,
            copy_r1_to_r0_bytes: 64 * MIB,
            copy_r1_above4g_bytes: 64 * MIB,
        };

        // 1. Decode failure -> FAIL, never PASS.
        for (am1, size) in [
            (0u32, 4 * GIB),
            (u32::MAX, 4 * GIB),
            (0x3f16, 4 * GIB), // 6 GB rank bit on 4 GB size
            (0x3f17, 6 * GIB), // 4 GB rank bit on 6 GB size
            (0x3f18, 4 * GIB), // bit 32 out of range for 4 GB
        ] {
            let map = validate_live_rank_map(am1, size);
            assert!(map.is_err(), "expected Err for am1={am1:#x} size={size:#x}");
            let (v, _) = evaluate_verdict(map, 0, &full_cov);
            assert_eq!(v, Verdict::Fail);
            assert_eq!(v.as_str(), "FAIL");
        }

        // 2. Missing or empty above-4-GiB (or cross-rank) leg -> FAIL.
        let map_4g = validate_live_rank_map(0x3f17, 4 * GIB);
        let no_hi = RunCoverage {
            copy_r1_above4g_bytes: 0,
            ..full_cov
        };
        assert_eq!(evaluate_verdict(map_4g, 0, &no_hi).0, Verdict::Fail);
        let no_r01 = RunCoverage {
            copy_r0_to_r1_bytes: 0,
            ..full_cov
        };
        assert_eq!(evaluate_verdict(map_4g, 0, &no_r01).0, Verdict::Fail);
        let no_r10 = RunCoverage {
            copy_r1_to_r0_bytes: 0,
            ..full_cov
        };
        assert_eq!(evaluate_verdict(map_4g, 0, &no_r10).0, Verdict::Fail);
        let no_both = RunCoverage {
            march_ran_both_ranks: false,
            ..full_cov
        };
        assert_eq!(evaluate_verdict(map_4g, 0, &no_both).0, Verdict::Fail);

        // 3. Single-rank map -> LIMITED (never PASS); errors still FAIL.
        let map_1r = validate_live_rank_map(0x3f3f, 4 * GIB);
        assert_eq!(map_1r, Ok(LiveRankMap::SingleRank));
        let empty_copy = RunCoverage::default();
        let (v_1r, _) = evaluate_verdict(map_1r, 0, &empty_copy);
        assert_eq!(v_1r, Verdict::Limited);
        assert_eq!(v_1r.as_str(), "LIMITED");
        assert_eq!(evaluate_verdict(map_1r, 1, &empty_copy).0, Verdict::Fail);

        // 4. Full 4 GB and 6 GB CXMT plans give PASS when error-free.
        for (am1, size, expected_bit) in [(0x3f17u32, 4 * GIB, 31u32), (0x3f16u32, 6 * GIB, 30u32)]
        {
            let map = validate_live_rank_map(am1, size).unwrap();
            assert_eq!(
                map,
                LiveRankMap::DualRank {
                    rank_bit: expected_bit
                }
            );
            assert_eq!(map.rank_bit(), Some(expected_bit));
            let h = sample_handoff(size, 4 * MIB, None);
            let plan = cached_plan_from_handoff(&h).unwrap();
            assert!(plan_covers_both_ranks_and_above_4g(
                &plan,
                h.dram_base,
                expected_bit
            ));
            let cp = build_copy_plan(&plan, h.dram_base, map.rank_bit(), COPY_BLOCK_BYTES);
            let cov = RunCoverage {
                stress_ran_both_ranks: true,
                march_ran_both_ranks: true,
                bit37_ran_both_ranks: true,
                copy_r0_to_r1_bytes: cp.r0_to_r1.pairs().iter().map(|p| p.len).sum(),
                copy_r1_to_r0_bytes: cp.r1_to_r0.pairs().iter().map(|p| p.len).sum(),
                copy_r1_above4g_bytes: cp.r1_above4g.pairs().iter().map(|p| p.len).sum(),
            };
            let (v, _) = evaluate_verdict(Ok(map), 0, &cov);
            assert_eq!(v, Verdict::Pass);
            assert_eq!(v.as_str(), "PASS");
            assert_eq!(evaluate_verdict(Ok(map), 1, &cov).0, Verdict::Fail);
        }
    }

    #[test]
    fn teardown_wb_range_covers_every_cacheable_block_at_4g_and_6g() {
        // 4 GB map: 0x4000_0000..0x1_4000_0000 (blocks 1..=4).
        let mut t4 = [0u64; 512];
        l1_table(&mut t4, 0x1_4000_0000, Some(1)).unwrap();
        let (s4, e4) = wb_mapped_range(&t4).unwrap();
        assert_eq!((s4, e4), (0x4000_0000, 0x1_4000_0000));
        // Every WB block is inside [s4, e4), and block 0 (Device MMIO) is not.
        assert!(0x3fff_ffc0 < s4);
        assert!(0x4000_0000 >= s4 && 0x1_3fff_ffc0 < e4);

        // 6 GB map: 0x4000_0000..0x1_c000_0000 (blocks 1..=6).
        let mut t6 = [0u64; 512];
        l1_table(&mut t6, 0x1_c000_0000, Some(1)).unwrap();
        assert_eq!(wb_mapped_range(&t6).unwrap(), (0x4000_0000, 0x1_c000_0000));

        // Empty table and discontiguous WB blocks are rejected.
        let empty = [0u64; 512];
        assert!(wb_mapped_range(&empty).is_err());
        let mut gap = t4;
        gap[6] = (6u64 << 30) | BLOCK | AF | SH_INNER | ATTR1 | XN;
        assert!(wb_mapped_range(&gap).is_err());
    }

    #[test]
    fn teardown_wdt_reload_interval_stays_under_16s_at_1us_per_line() {
        assert_eq!(WDT0_CTRL_ADDR, 0x0205_000c);
        assert_eq!(WDT0_RELOAD_VAL, 0x14af);
        assert_eq!(WDT0_TIMEOUT_US, 16_000_000);
        // Zero reload_lines is rejected.
        assert_eq!(teardown_wdt_max_interval_us(0, 1), None);
        // At 1 us/line, worst-case time between reloads stays strictly under 16 s.
        let max_us_1 = teardown_wdt_max_interval_us(TEARDOWN_WDT_RELOAD_LINES, 1).unwrap();
        assert_eq!(max_us_1, 524_288);
        assert!(max_us_1 < WDT0_TIMEOUT_US);
        // Even at 10 us/line (10x slower), reloads stay well under 16 s.
        let max_us_10 = teardown_wdt_max_interval_us(TEARDOWN_WDT_RELOAD_LINES, 10).unwrap();
        assert!(max_us_10 < WDT0_TIMEOUT_US);
        // Reload counts across 4 GB and 6 GB (64-byte lines).
        let lines_4g = (4 * GIB) / CACHE_LINE_ALIGN;
        let lines_6g = (6 * GIB) / CACHE_LINE_ALIGN;
        assert_eq!(lines_4g / TEARDOWN_WDT_RELOAD_LINES, 128);
        assert_eq!(lines_6g / TEARDOWN_WDT_RELOAD_LINES, 192);
    }

    #[test]
    fn addrmap1_4g_readback_written_and_mapped_second_cs() {
        let expected = Ok(LiveRankMap::DualRank { rank_bit: 31 });
        for raw in [0x3f17, 0x8000_3fd7, 0x17] {
            assert_eq!(validate_live_rank_map(raw, 4 * GIB), expected);
        }
        for raw in [0x0517, 0x1f17] {
            assert!(validate_live_rank_map(raw, 4 * GIB).is_err());
        }
    }

    #[test]
    fn addrmap1_6g_readback_written_and_mapped_second_cs() {
        let expected = Ok(LiveRankMap::DualRank { rank_bit: 30 });
        for raw in [0x3f16, 0x4000_3f56, 0x16] {
            assert_eq!(validate_live_rank_map(raw, 6 * GIB), expected);
        }
        for raw in [0x0516, 0x2016] {
            assert!(validate_live_rank_map(raw, 6 * GIB).is_err());
        }
    }

    #[test]
    fn addrmap1_single_rank_readback_or_written() {
        for raw in [0x3f3f, 0x3f] {
            assert_eq!(
                validate_live_rank_map(raw, 4 * GIB),
                Ok(LiveRankMap::SingleRank)
            );
        }
        assert!(validate_live_rank_map(0x053f, 4 * GIB).is_err());
    }
}
