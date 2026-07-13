// SPDX-License-Identifier: GPL-2.0-only
// Verbatim copies of the board code this crate replaces, kept as the test
// oracle for the equivalence tests. Source: ore-edk-boot-radxa-a7a oreboot
// 2d79de7 (= ore-edk2-boot patches/oreboot/a733-cubie-a7a up to "a7a: one-boot
// TxWckDly alias probe ...", tree 6507a1ab86cf with the K1 series), path
// src/mainboard/radxa/cubie-a7a/dram/src/<file>. Only the stubs marked
// "stub:" are not from the source.
#![allow(dead_code, clippy::all)]

// ---- phy/coldinit.rs ------------------------------------------------------
// stub: CBT_ECHO_BITS = CBT_A.echo_bits() = 0xff & !0x110; CBT_EXPECT verbatim.
pub const CBT_ECHO_BITS: u16 = 0xef;
pub const CBT_EXPECT: [bool; 4] = [false, true, true, false];
pub mod coldinit {
    use super::*;
    pub const fn cbt_pass_bits(echo: [u16; 4], bits: u16) -> u16 {
        let mut pass = bits;
        let mut p = 0;
        while p < 4 {
            let want = if CBT_EXPECT[p] { 0xffff } else { 0 };
            pass &= !(echo[p] ^ want);
            p += 1;
        }
        pass
    }

    /// The run of set bits in a circular `n`-bit map that contains point `pref`,
    /// or else the run whose center is circularly nearest to it (ties: the
    /// longer, then the first found): (start, len); (0, 0) for an empty map.
    /// Used to pick one of several equivalent windows (the CA delay passes once
    /// per 256 units) consistently around a reference point.
    pub fn run_near(map: u128, n: u32, pref: u32) -> (u32, u32) {
        let full = if n == 128 {
            u128::MAX
        } else {
            (1u128 << n) - 1
        };
        let map = map & full;
        if map == 0 {
            return (0, 0);
        }
        if map == full {
            return (0, n);
        }
        let bit = |i: u32| map >> (i % n) & 1 != 0;
        // Enumerate runs starting after a zero.
        let mut best: Option<((u32, u32), u32)> = None;
        for start in 0..n {
            if !bit(start) || bit(start + n - 1) {
                continue;
            }
            let mut len = 0;
            while len < n && bit(start + len) {
                len += 1;
            }
            let contains = (pref + n - start) % n < len;
            // Doubled coordinates: center = 2 start + len - 1.
            let c2 = (2 * start + len - 1) % (2 * n);
            let p2 = 2 * (pref % n);
            let d = c2.abs_diff(p2);
            let dist = if contains { 0 } else { d.min(2 * n - d) };
            let better = match best {
                None => true,
                Some(((_, bl), bd)) => dist < bd || (dist == bd && len > bl),
            };
            if better {
                best = Some(((start, len), dist));
            }
        }
        best.map(|b| b.0).unwrap_or((0, 0))
    }

    pub fn longest_run(map: u128, n: u32) -> (u32, u32) {
        let full = if n == 128 {
            u128::MAX
        } else {
            (1u128 << n) - 1
        };
        let map = map & full;
        if map == full {
            return (0, n);
        }
        let mut best = (0, 0);
        // Start counting after a zero so that a wrapping run is counted once.
        let mut first_zero = 0;
        while map >> first_zero & 1 != 0 {
            first_zero += 1;
        }
        let mut start = 0;
        let mut len = 0;
        for k in 1..=n {
            let i = (first_zero + k) % n;
            if map >> i & 1 != 0 {
                if len == 0 {
                    start = i;
                }
                len += 1;
                if len > best.1 {
                    best = (start, len);
                }
            } else {
                len = 0;
            }
        }
        best
    }
}

// ---- blobless.rs ----------------------------------------------------------
// stub: BlockTrain reduced to the fields combine_channel reads.
pub const LANE_SWEEP_POINTS: u32 = 64;
pub const LANE_SWEEP_STEP: u16 = 8;
#[derive(Clone, Copy, Debug)]
pub struct BlockTrain {
    pub lane_bit: [u8; 7],
    pub lane_maps: [u64; 7],
    pub lane_delay: [u16; 7],
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ChannelCa {
    /// Lane-sweep run of the intersection per lane (length 0: no overlap or
    /// a block without a window for that lane).
    pub runs: [(u32, u32); 7],
    pub delay: [u16; 7],
    /// Every lane drives the same echo bit in all four blocks.
    pub lane_map_agrees: bool,
}

impl ChannelCa {
    pub const EMPTY: ChannelCa = ChannelCa {
        runs: [(0, 0); 7],
        delay: [0; 7],
        lane_map_agrees: false,
    };
}

pub fn combine_channel(blocks: &[BlockTrain; 8], ch: usize) -> ChannelCa {
    let full = u64::MAX >> (64 - LANE_SWEEP_POINTS);
    let set = &blocks[4 * ch..4 * ch + 4];
    let mut out = ChannelCa {
        lane_map_agrees: true,
        ..ChannelCa::EMPTY
    };
    for lane in 0..7 {
        let bit = set[0].lane_bit[lane];
        let mut map = full;
        for b in set {
            if b.lane_bit[lane] != bit || bit == 0xff {
                out.lane_map_agrees = false;
            }
            map &= b.lane_maps[lane];
        }
        let pref = set[0].lane_delay[lane] as u32 / LANE_SWEEP_STEP as u32;
        out.runs[lane] = coldinit::run_near(map as u128, LANE_SWEEP_POINTS, pref);
        let (s, l) = out.runs[lane];
        out.delay[lane] = if l == 0 {
            set[0].lane_delay[lane]
        } else {
            (((s + l / 2) % LANE_SWEEP_POINTS) as u16) * LANE_SWEEP_STEP
        };
    }
    out
}

// ---- wl.rs ----------------------------------------------------------------
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Decision {
    /// Fail threshold: counts above it are "fail" (half the maximum).
    pub threshold: u16,
    /// Points (index i: the change happens between i-1 and i) where pass/fail
    /// changes, and the fail state after each, first [`MAX_EDGES`].
    pub edges: [(u8, bool); MAX_EDGES],
    pub n_edges: u8,
    /// Chosen TxWckDly (linear UI/64), None without a usable transition.
    pub delay: Option<u16>,
    /// The hardware counter's delay lies on a measured transition.
    pub counter_agrees: bool,
}

pub const MAX_EDGES: usize = 8;

impl Decision {
    pub const EMPTY: Decision = Decision {
        threshold: 0,
        edges: [(0, false); MAX_EDGES],
        n_edges: 0,
        delay: None,
        counter_agrees: false,
    };
}

pub fn decide(err: &[u16], step: u16, counter: Option<u16>) -> Decision {
    let mut d = Decision::EMPTY;
    let max = err.iter().copied().max().unwrap_or(0);
    let min = err.iter().copied().min().unwrap_or(0);
    if max == min {
        return d;
    }
    d.threshold = max / 2;
    let fail = |i: usize| err[i] > d.threshold;
    let mut all = [0usize; 64];
    let mut n_all = 0;
    for i in 1..err.len() {
        if fail(i) != fail(i - 1) {
            if (d.n_edges as usize) < MAX_EDGES {
                d.edges[d.n_edges as usize] = (i as u8, fail(i));
                d.n_edges += 1;
            }
            if n_all < all.len() {
                all[n_all] = i;
                n_all += 1;
            }
        }
    }
    let at = |i: usize| (i as u16 * step) - step / 2;
    if let Some(c) = counter {
        if all[..n_all]
            .iter()
            .any(|&i| !fail(i) && at(i).abs_diff(c) <= 2 * step)
        {
            d.counter_agrees = true;
            d.delay = Some(c);
            return d;
        }
    }
    for k in 0..n_all {
        let i = all[k];
        let prev = if k == 0 { 0 } else { all[k - 1] };
        let next = if k + 1 < n_all { all[k + 1] } else { err.len() };
        // Fail -> pass only: the delay at which the DRAM's sampled level
        // becomes the expected one (hypothesis until a data stage confirms).
        if !fail(i) && i >= prev + 2 && next >= i + 2 {
            d.delay = Some(at(i));
            break;
        }
    }
    d
}

// ---- timing.rs ------------------------------------------------------------
pub const fn ns(t_ns: u32, mhz: u32) -> u32 {
    (t_ns * mhz).div_ceil(1000)
}

pub mod lp5 {
    pub const T_FAW: u32 = 20;
    pub const T_RRD: u32 = 5;
    pub const T_RCD: u32 = 18;
    pub const T_XP: u32 = 7;
    pub const T_WTR_L: u32 = 12;
    pub const T_RTP: u32 = 8; // 7.5 ns
    pub const T_RPAB: u32 = 21;
    pub const T_RAS: u32 = 42;
    pub const T_REFI: u32 = 3906;
    pub const T_RFCAB: u32 = 380;
    /// Power-up: 200 us of RESET/CKE timing for the DFI init counters.
    pub const T_INIT_200US: u32 = 200_000;
    pub const T_ZQCAL: u32 = 260;
    pub const T_XSR_EXTRA: u32 = 90;
    pub const T_CKE: u32 = 3;
    pub const T_ESCKE: u32 = 10;
    pub const T_CCDMW_NS: u32 = 15;
    pub const T_CMD_1NS: u32 = 1;
    pub const T_MRW_40: u32 = 40;
    pub const T_ZQRESET_6US: u32 = 6000;
    pub const T_ZQLAT_30: u32 = 30;
    pub const T_ZQCS_50: u32 = 50;
}

// ---- size.rs (Para accessors replaced by parameters) -----------------------
// stub: the Para and Error of the board crate.
pub struct Para {
    pub row: u32,
    pub bank: u32,
    pub col: u32,
    pub bg: u32,
    pub rank: u32,
    pub para2: u32,
    pub channels: u32,
    pub tpr13: u32,
}
impl Para {
    pub fn row_bits(&self) -> u32 {
        self.row
    }
    pub fn bank_bits(&self) -> u32 {
        self.bank
    }
    pub fn col_bits(&self) -> u32 {
        self.col
    }
    pub fn bg_bits(&self) -> u32 {
        self.bg
    }
    pub fn rank_bits(&self) -> u32 {
        self.rank
    }
    pub fn para2(&self) -> u32 {
        self.para2
    }
    pub fn channels(&self) -> u32 {
        self.channels
    }
    pub fn tpr13(&self) -> u32 {
        self.tpr13
    }
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    Config(&'static str),
    Timeout(&'static str),
}
pub fn checked_size_mib(p: &Para) -> Result<u32, Error> {
    let bits = p.row_bits() + p.bank_bits() + p.col_bits() + p.bg_bits() + p.rank_bits();
    let half_width = p.para2() & 0xf != 0;
    let overhead = if !half_width || p.channels() == 2 {
        18
    } else {
        19
    };
    let shift = bits
        .checked_sub(overhead)
        .ok_or(Error::Config("DRAM geometry below size representation"))?;
    let mut mib = 1u32
        .checked_shl(shift)
        .ok_or(Error::Config("DRAM geometry exceeds size arithmetic"))?;
    if p.tpr13() & (7 << 16) != 0 && p.para2() >> 30 != 2 {
        mib = mib
            .checked_mul(3)
            .ok_or(Error::Config("non-binary DRAM size arithmetic overflow"))?
            >> 2;
    }
    validate_mib(mib)
}

fn validate_mib(mib: u32) -> Result<u32, Error> {
    // para2's handoff size is 15 bits; the bounded smoke check needs at least
    // two MiB. These are software bounds, not detected physical capacity.
    if !(2..=0x7fff).contains(&mib) {
        return Err(Error::Config("DRAM size outside boot handoff field"));
    }
    Ok(mib)
}

// ---- mr.rs -----------------------------------------------------------------
pub mod mr {
    // stub: the ctrl and hw items mr.rs uses.
    pub mod ctrl {
        pub const fn ddrc(ch: usize, off: usize) -> usize {
            0x0a02_0000 + 0x1000 * ch + 0x1_0000 + off
        }
        pub const MRCTRL0: usize = 0x080;
        pub const MRCTRL1: usize = 0x084;
        pub const MRSTAT: usize = 0x090;
    }
    pub mod hw {
        pub use super::super::Error;
        pub use dram_train::io::RegIo as Hw;
        pub fn wait<H: Hw>(
            hw: &mut H,
            what: &'static str,
            timeout_us: u64,
            cond: impl FnMut(&mut H) -> bool,
        ) -> Result<(), Error> {
            dram_train::io::wait(hw, what, timeout_us, cond).map_err(|t| Error::Timeout(t.0))
        }
    }
    // (mr.rs:) LPDDR5 mode register access through the controller (MRCTRL0/1, MRSTAT).

    use self::ctrl::{ddrc, MRCTRL0, MRCTRL1, MRSTAT};
    use self::hw::{wait, Error, Hw};

    const MR_WR: u32 = 1 << 31;
    const MR_TYPE_READ: u32 = 1;
    /// MRCTRL0.mr_rank: rank 0 is bit 4, rank 1 bit 5.
    const fn rank_bit(rank: u32) -> u32 {
        1 << (4 + rank)
    }
    const TIMEOUT_US: u64 = 100_000;

    /// Validate the software command fields before any controller access. A legal
    /// numeric channel/rank is not proof that it exists in the selected topology;
    /// callers must separately establish that and the DRAM command timing.
    fn validate(ch: usize, rank: u32, mr: u32, value: u32) -> Result<(), Error> {
        if ch > 1 || rank > 1 || mr > 0xff || value > 0xff {
            return Err(Error::Config(
                "MR channel/rank/address/data outside command fields",
            ));
        }
        Ok(())
    }

    fn idle<H: Hw>(hw: &mut H, ch: usize) -> Result<(), Error> {
        wait(hw, "MRSTAT.mr_wr_busy", TIMEOUT_US, |hw| {
            hw.read32(ddrc(ch, MRSTAT)) & 1 == 0
        })
    }

    fn done<H: Hw>(hw: &mut H, ch: usize) -> Result<(), Error> {
        wait(hw, "MRCTRL0.mr_wr", TIMEOUT_US, |hw| {
            hw.read32(ddrc(ch, MRCTRL0)) & MR_WR == 0
        })
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
        validate(ch, rank, mr, value)?;
        idle(hw, ch)?;
        hw.write32(ddrc(ch, MRCTRL0), rank_bit(rank));
        hw.write32(ddrc(ch, MRCTRL1), (mr << 8) | value);
        hw.write32(ddrc(ch, MRCTRL0), MR_WR | rank_bit(rank));
        done(hw, ch)
    }

    /// Write MR `mr` of both ranks of channel `ch` (rank 0 gets `v0`, rank 1
    /// `v1`). All fields are validated before the first command. A rank-1 timeout
    /// does not undo rank 0: callers must retain that partial-operation failure.
    pub fn write<H: Hw>(hw: &mut H, ch: usize, mr: u32, v0: u32, v1: u32) -> Result<(), Error> {
        validate(ch, 0, mr, v0)?;
        validate(ch, 1, mr, v1)?;
        write_rank(hw, ch, 0, mr, v0)?;
        write_rank(hw, ch, 1, mr, v1)
    }

    /// Read MR `mr` of `rank` of channel `ch`. The value arrives in MCTL_COM:
    /// +0x2c [15:8] for channel 0, +0x40 [15:8] for channel 1.
    pub fn read<H: Hw>(hw: &mut H, ch: usize, rank: u32, mr: u32) -> Result<u32, Error> {
        validate(ch, rank, mr, 0)?;
        idle(hw, ch)?;
        hw.write32(ddrc(ch, MRCTRL0), rank_bit(rank) | MR_TYPE_READ);
        hw.write32(ddrc(ch, MRCTRL1), mr << 8);
        hw.write32(ddrc(ch, MRCTRL0), MR_WR | rank_bit(rank) | MR_TYPE_READ);
        done(hw, ch)?;
        let reg = if ch == 0 { 0x0a02_002c } else { 0x0a02_0040 };
        Ok((hw.read32(reg) >> 8) & 0xff)
    }
}
