// SPDX-License-Identifier: GPL-2.0-only
//! RK3588 LPDDR4/4X bring-up: four DDRCTL + four DDRPHY, one frequency.
//! Entry point for bt0: [`init`].
//!
//! Flow (each step cites its source in the module that does it):
//! 1. prerequisites: DDR rails (crate::rk806, TRM P2 §2.6.1 step 1), DDR
//!    CRU clocks ungated, DxBPLL locked at the target (crate::pll, step 2);
//! 2. per channel, a probe pass ([`seq::init_channel`] with the largest
//!    geometry, untrained) to read MR5/6/7/8 per rank ([`detect`]) and,
//!    RK3568-RFC style, a gate-training probe of rank 1 ([`train`]);
//! 3. per channel, the real pass with the detected ranks, density and
//!    address map ([`ctl`] through crate dwc-ddrctl, [`phy`]), optional HW
//!    command bus training, the PHY training scheduler, post-training
//!    settings ([`seq::finish`]);
//! 4. DDR schedulers (crate::msch), a per-channel CPU test in the linear
//!    layout with software sweeps as fallback ([`fallback`], crate
//!    dram-train), then the 4-channel interleave, OS_REG2..5 handoff and
//!    a whole-memory smoke and aliasing test ([`verify`], crate memtest).
//!
//! Provenance: register map and sequence from the RK3588 TRM Part 1/2
//! V1.0; timing from JEDEC rules (crate jedec-lpddr, [`timing`]);
//! structure of the PHY steps and the GRF dfi_init_complete guard from
//! the RK3576 U-Boot RFC (Johan Axelsson, 2026-05, GPL-2.0+); the MR8 row
//! rule, the rank probe by gate training and the aliasing size check from
//! the RK3568 U-Boot RFC (Pavel Golikov, 2026-05, GPL-2.0+). Both RFCs are
//! C; what is reused is ported logic, attributed in each file.
//!
//! Licence guard: no value here comes from a Rockchip binary or from a
//! register dump of the rkbin DDR init. Identification data (MR5-8,
//! OS_REG) may select a configuration ([`Known`]).

pub mod ctl;
pub mod detect;
pub mod fallback;
pub mod mr;
pub mod phy;
pub mod seq;
pub mod timing;
pub mod train;
pub mod verify;

use crate::msch::{self, Layout};
use crate::osreg;
use regio::RegIo;

/// DRAM type to bring up.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DramType {
    Lpddr4,
    Lpddr4x,
}

impl DramType {
    pub const fn osreg_code(self) -> u32 {
        match self {
            DramType::Lpddr4 => osreg::LPDDR4,
            DramType::Lpddr4x => osreg::LPDDR4X,
        }
    }
}

/// DFI latencies relative to the DRAM latencies. The DDRCTL field texts say
/// "Refer to PHY specification" (TRM Part 2 p.85) and the PHY specification
/// is not public: explicit knobs, swept by [`fallback::dfi`] on failure.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DfiLatency {
    /// dfi_tphy_wrlat = WL - wrlat_sub.
    pub wrlat_sub: u32,
    /// dfi_tphy_wrdata.
    pub tphy_wrdata: u32,
    /// dfi_t_rddata_en = RL - rddata_en_sub.
    pub rddata_en_sub: u32,
}

/// How the PHY gets a clean read DQS (TRM §2.6.5).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DqsClean {
    /// Gate training in the scheduler run (phy_gttrn_en).
    Gate,
    /// Auto DQS clean (needs VSSQ read termination).
    Auto,
}

/// The fitted parts, when already known (from a serial log or dram-info):
/// skips the probe pass; [`init`] reads MR5..MR8 back after the real pass
/// and refuses a channel whose MR8 differs. Identification data only.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Known {
    pub ranks: u8,
    pub mr8: u8,
}

/// The YY3588's parts, identified on the first hardware run (2026-11-09,
/// MaskROM dram-info loader; DaveWK/yy3588-workspace
/// docs/firmware-out/archive/2026-11-09-debug/hw-logs/2026-11-09-maskrom-draminfo-loader.md): the rkbin banner ("LPDDR4X ...
/// BW=16 Col=10 Bk=8 CS0 Row=18 CS1 Row=18 CS=2 Die BW=8 Size=8192MB" on
/// all four channels), the OS_REG words it left (0x3a561a56, 0x300010f5
/// for both pairs) and MRR through our own code on every channel and rank:
/// MR5 0xff (Micron), MR6 0x07, MR7 0x00, MR8 0x58 = OP[7:6] 1 (x8,
/// byte mode), OP[5:2] 6 (16 Gb per die channel; a 16-bit channel of two
/// dies is 32 Gb = 4 GiB per rank). Two ranks: 8 GiB per channel, 32 GiB.
/// Identification only (licence guard): no training result or register
/// value of that run is used.
pub const YY3588_PARTS: Known = Known {
    ranks: 2,
    mr8: 0x58,
};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Config {
    /// DRAM clock (CK) in MHz; the data rate is twice this. In LPDDR4 1:2:2
    /// mode the PHY's clk_phy2x equals CK (TRM p.501 ctrl_phy_mode, p.558).
    pub ck_mhz: u32,
    pub dram_type: DramType,
    /// Channels to bring up (bit per channel). Interleave needs all four.
    pub channel_mask: u8,
    /// Ranks to probe (1 or 2).
    pub max_ranks: u8,
    /// Hardware command bus training (TRM §2.6.4).
    pub cbt: bool,
    pub dfi: DfiLatency,
    pub dqs_clean: DqsClean,
    /// MR11 (ODT), MR12 (CA Vref), MR14 (DQ Vref), MR22, written by MRW
    /// after init and given to the PHY for its own MRWs.
    pub mr11: u8,
    pub mr12: u8,
    pub mr14: u8,
    pub mr22: u8,
    /// Software self-refresh around the clock switch ([`seq`]).
    pub sr_freq_change: bool,
    /// GRF hold of dfi_init_complete while the DLL powers on ([`seq`]).
    pub grf_dfi_guard: bool,
    /// Confirm rank 1 by gate training it alone (RK3568 RFC method).
    pub rank_gate_probe: bool,
    /// Extra clocks on rd2wr / wr2rd (the TRM says trained values "may
    /// need to be increased").
    pub turnaround_pad: u8,
    /// CAL_CON4.num_repeat for write training (TRM §2.6.7: "increase").
    pub wr_num_repeat: u8,
    /// Enable the DDR rails through the RK806 first.
    pub rails: bool,
    pub known: Option<Known>,
    /// Per-channel and whole-memory CPU tests.
    pub memtest: bool,
    /// Software sweeps when a channel fails its test.
    pub fallback: bool,
    /// The address layout left for the OS.
    pub layout: Layout,
}

impl Config {
    /// First-contact defaults for the YY3588 ([`YY3588_PARTS`]): LPDDR4X
    /// at 1056 MHz (2112 MT/s, the lowest common data rate of the parts'
    /// speed range this code programs), all four channels, two ranks of
    /// x8 byte-mode dies (MR8 0x58: 18 rows, 10 columns, 8 banks, 4 GiB
    /// per rank, 8 GiB per channel, 32 GiB), ODT off, Vref near 50 % of
    /// VDDQ for an unterminated bus (MR12/MR14 range 1 setting 28 = 49.7 %
    /// on LPDDR4X), gate training for DQS, interleaved.
    ///
    /// Byte mode (JEDEC LPDDR4 x8 devices): two dies share each rank's CA
    /// bus and CS and each drives one byte lane with its own DQS. With
    /// MR11 = 0 (DQ ODT and CA ODT off on every die) and MR22 = 0 (no
    /// ODTE-CK/ODTE-CS/ODTD-CA override; the ODT_CA pad strapping does
    /// not matter with CA ODT off) the four dies on a channel's CA bus are
    /// all unterminated, the same for x8 and x16 parts, so no byte-mode
    /// specific MR11/MR22 value is needed at this rate. MRW goes to both
    /// dies of a rank (shared CS), MRR returns each die's value on its own
    /// byte (crate::ddr::mr takes byte 0), the PHY trains per byte lane
    /// (DQS0/DQS1) as for x16 parts, and the DDRCTL has no device-width
    /// setting for LPDDR4 (MSTR0 [31:26] reserved, TRM P2 pp.108-109).
    /// What byte mode changes here: the row count (18 for this density,
    /// detect::geometry), the tRFC class (per die channel, 16 Gb: 380 ns),
    /// PHY CAL_CON4.LP4_High_density_mode (R17 in ACT-2), the OS_REG DBW
    /// field (x8) and the 64 MiB DDRSCH granule count (64 per rank).
    pub const fn yy3588_default() -> Config {
        Config {
            known: Some(YY3588_PARTS),
            ..Config::yy3588_probe()
        }
    }

    /// [`Config::yy3588_default`] without the known parts: the probe pass
    /// identifies the parts by MRR (for other LPDDR4/4X fits).
    pub const fn yy3588_probe() -> Config {
        Config {
            ck_mhz: 1056,
            dram_type: DramType::Lpddr4x,
            channel_mask: 0xf,
            max_ranks: 2,
            cbt: false,
            dfi: DfiLatency {
                wrlat_sub: 2,
                tphy_wrdata: 2,
                rddata_en_sub: 2,
            },
            dqs_clean: DqsClean::Gate,
            mr11: 0x00,
            mr12: 0x40 | 28,
            mr14: 0x40 | 28,
            mr22: 0x00,
            sr_freq_change: true,
            grf_dfi_guard: true,
            rank_gate_probe: true,
            turnaround_pad: 1,
            wr_num_repeat: 4,
            rails: true,
            known: None,
            memtest: true,
            fallback: true,
            layout: Layout::Interleave,
        }
    }
}

/// What one channel turned out to be.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct ChannelInfo {
    pub present: bool,
    pub ranks: u8,
    pub mr5: u8,
    pub mr6: u8,
    pub mr7: u8,
    pub mr8: u8,
    pub geometry: Option<detect::ChannelGeometry>,
    pub train: train::Result,
    pub zq_fail: bool,
    pub cbt_fail: u32,
    /// Set when a fallback sweep chose a value.
    pub rx_vref: Option<u8>,
    pub dfi: Option<DfiLatency>,
}

/// The result of bring-up: per-channel info and the total size.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct DramInfo {
    pub ch: [ChannelInfo; 4],
    pub total_mib: u64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    BadFrequency,
    Rails,
    Pll(&'static str),
    NoChannel,
    Channel {
        ch: usize,
        what: &'static str,
    },
    Geometry {
        ch: usize,
        mr8: u8,
    },
    Training {
        ch: usize,
        fail: u32,
    },
    /// Channels differ in size or are missing, so they cannot interleave.
    Asymmetric,
    Layout(&'static str),
    Memtest {
        ch: Option<usize>,
        addr: usize,
    },
    Alias {
        offset: usize,
    },
}

/// Lowest and highest CK this code programs: the write-training modes of
/// §2.6.7 cover >= 533 MHz; 1600 MHz is the board's LPDDR4X-3200 limit.
pub const CK_MHZ_RANGE: core::ops::RangeInclusive<u32> = 533..=1600;

/// Bring up one channel with known ranks and geometry (real pass).
pub fn bring_up_channel<H: RegIo + ?Sized>(
    io: &mut H,
    ch: usize,
    cfg: &Config,
    ranks: u8,
    g: &detect::ChannelGeometry,
) -> Result<ChannelInfo, Error> {
    let e = |what| Error::Channel { ch, what };
    let p = timing::plan(cfg, ranks, g.density_gb);
    let obs = seq::init_channel(io, ch, &p, g, cfg).map_err(e)?;
    let mut info = ChannelInfo {
        present: true,
        ranks,
        geometry: Some(*g),
        zq_fail: obs.zq_fail,
        ..Default::default()
    };
    if cfg.cbt {
        info.cbt_fail = train::cbt_hw(io, ch, &p, cfg).map_err(e)?;
    }
    train::prepare(io, ch, &p, cfg);
    info.train = train::run(io, ch, train::rank_mask(ranks), cfg).map_err(e)?;
    seq::finish(io, ch).map_err(e)?;
    io.log(format_args!(
        "ddr ch{ch}: trained dq_fail={:#06x} wrlvl={} wlcal={} zq_fail={} cbt={:#x}\n",
        info.train.dq_fail,
        info.train.wrlvl_fail,
        info.train.wlcal_fail,
        info.zq_fail,
        info.cbt_fail
    ));
    Ok(info)
}

/// Identify channel `ch`: probe pass and MRR, or the configured [`Known`].
/// None when no LPDDR4 answers (channel not fitted or dead).
pub fn identify<H: RegIo + ?Sized>(
    io: &mut H,
    ch: usize,
    cfg: &Config,
) -> Result<Option<(u8, detect::Ids)>, Error> {
    if let Some(k) = cfg.known {
        let ids = detect::Ids {
            ranks: k.ranks,
            mr8: k.mr8,
            ..Default::default()
        };
        return Ok(Some((k.ranks, ids)));
    }
    let g = detect::ChannelGeometry::PROBE;
    let probe = timing::plan(cfg, cfg.max_ranks, g.density_gb);
    if let Err(what) = seq::init_channel(io, ch, &probe, &g, cfg) {
        io.log(format_args!("ddr ch{ch}: probe failed: {what}\n"));
        return Ok(None);
    }
    let ids = detect::read_ids(io, ch, cfg.max_ranks);
    io.log(format_args!(
        "ddr ch{ch}: mr5={:#04x} ({}) mr6={:#04x} mr7={:#04x} mr8={:#04x} raw={:#010x} mr8[r1]={:#04x}\n",
        ids.mr5,
        detect::vendor_name(ids.mr5),
        ids.mr6,
        ids.mr7,
        ids.mr8,
        ids.mr8_raw,
        ids.mr8_rank1
    ));
    if !detect::present(&ids) {
        return Ok(None);
    }
    let mut gate = None;
    if cfg.rank_gate_probe {
        // RK3568 RFC method: a rank exists when gate training on it alone
        // passes; rank 0 first (a dead channel fails it).
        train::prepare(io, ch, &probe, cfg);
        if !train::gate_probe(io, ch, 0) {
            io.log(format_args!("ddr ch{ch}: rank 0 gate probe failed\n"));
            return Ok(None);
        }
        if cfg.max_ranks >= 2 && ids.mr8_rank1 == ids.mr8 {
            gate = Some(train::gate_probe(io, ch, 1));
        }
    }
    let ranks = detect::ranks(&ids, cfg.max_ranks, gate);
    Ok(Some((ranks, ids)))
}

/// With [`Config::known`]: MRR MR5..MR8 after the real pass and refuse the
/// channel when MR8 (rank 0, and rank 1 when two ranks are known) is not
/// the known value. Identification only.
pub fn check_known<H: RegIo + ?Sized>(
    io: &mut H,
    ch: usize,
    k: &Known,
) -> Result<detect::Ids, Error> {
    let ids = detect::read_ids(io, ch, k.ranks);
    io.log(format_args!(
        "ddr ch{ch}: mr5={:#04x} ({}) mr6={:#04x} mr7={:#04x} mr8={:#04x} mr8[r1]={:#04x} (known mr8={:#04x})\n",
        ids.mr5,
        detect::vendor_name(ids.mr5),
        ids.mr6,
        ids.mr7,
        ids.mr8,
        ids.mr8_rank1,
        k.mr8
    ));
    if ids.mr8 != k.mr8 || (k.ranks >= 2 && ids.mr8_rank1 != k.mr8) {
        return Err(Error::Geometry { ch, mr8: ids.mr8 });
    }
    Ok(ids)
}

/// Base of channel `ch` in the linear layout with `mib` per channel.
pub const fn linear_base(ch: usize, mib: u64) -> usize {
    (ch as u64 * (mib << 20)) as usize
}

/// The 256 MiB register window at 0xF000_0000 (P1 Table 1-1): no DRAM
/// is reachable there.
pub const fn in_register_window(addr: usize) -> bool {
    addr >= 0xF000_0000 && (addr as u64) < 0x1_0000_0000
}

/// Where the whole-memory test also runs for `span_mib` MiB of DRAM from
/// 0: the middle, when the span is 8 GiB or more (a smaller span is
/// covered by the test at 0, which reaches 4 GiB) and the middle lies
/// above the register window.
pub const fn upper_half_test_base(span_mib: u64) -> Option<usize> {
    let base = (span_mib / 2) << 20;
    if span_mib >= 8192 && base >= 1 << 32 {
        Some(base as usize)
    } else {
        None
    }
}

/// Words per test location (memtest::quick) in the per-channel test.
pub const CHANNEL_TEST_WORDS: usize = 256;

/// Full bring-up. Writes OS_REG2..5 on success only, after the tests.
pub fn init<H: RegIo + ?Sized>(io: &mut H, cfg: &Config) -> Result<DramInfo, Error> {
    if !CK_MHZ_RANGE.contains(&cfg.ck_mhz) {
        return Err(Error::BadFrequency);
    }
    let pll = crate::pll::settings(cfg.ck_mhz).ok_or(Error::BadFrequency)?;
    if cfg.rails {
        let r = crate::rk806::ddr_rails(io).map_err(|_| Error::Rails)?;
        io.log(format_args!("ddr: rails {r:?}\n"));
    }
    let mut info = DramInfo::default();
    for ch in 0..4 {
        if cfg.channel_mask & (1 << ch) == 0 {
            continue;
        }
        crate::pll::program(io, crate::regs::base::DDR_CRU[ch], pll)
            .map_err(|t| Error::Pll(t.0))?;
        let Some((ranks, ids)) = identify(io, ch, cfg)? else {
            continue;
        };
        let g = match detect::geometry(ids.mr8) {
            Some(g) if !g.row_3_4 => g,
            // 3/4-row densities need ADDRMAP12 non-binary support [U].
            _ => return Err(Error::Geometry { ch, mr8: ids.mr8 }),
        };
        let mut c = bring_up_channel(io, ch, cfg, ranks, &g)?;
        let ids = match cfg.known {
            Some(k) => check_known(io, ch, &k)?,
            None => ids,
        };
        (c.mr5, c.mr6, c.mr7, c.mr8) = (ids.mr5, ids.mr6, ids.mr7, ids.mr8);
        if !c.train.ok() && !(cfg.fallback && cfg.memtest) {
            return Err(Error::Training {
                ch,
                fail: c.train.dq_fail,
            });
        }
        info.total_mib += g.channel_mib(ranks);
        info.ch[ch] = c;
    }
    let present: u8 = (0..4)
        .filter(|&c| info.ch[c].present)
        .fold(0, |m, c| m | 1 << c);
    if present == 0 {
        return Err(Error::NoChannel);
    }
    msch::configure(io, &info);
    let sizes = info.ch.map(|c| c.geometry.map(|g| g.channel_mib(c.ranks)));
    let ch_mib = sizes.iter().flatten().copied().max().unwrap_or(0);
    let equal = sizes.iter().flatten().all(|&m| m == ch_mib);
    if matches!(cfg.layout, Layout::Interleave) && (present != 0xf || !equal) {
        return Err(Error::Asymmetric);
    }
    if cfg.memtest {
        check_channels(io, cfg, &mut info, ch_mib)?;
    }
    msch::set_layout(io, cfg.layout).map_err(Error::Layout)?;
    if cfg.memtest {
        let span = match cfg.layout {
            Layout::Interleave => info.total_mib,
            Layout::Linear { mib } => mib.min(info.total_mib),
        };
        verify::quick(io, 0, span, 1024).map_err(|addr| Error::Memtest { ch: None, addr })?;
        if let Some(base) = upper_half_test_base(span) {
            // verify::quick covers at most 4 GiB from its base: also test
            // the upper half (rank 1 of every channel in the interleave
            // when ranks are equal [I]).
            verify::quick(io, base, span / 2, 1024)
                .map_err(|addr| Error::Memtest { ch: None, addr })?;
        }
        verify::alias(io, 0, span).map_err(|offset| Error::Alias { offset })?;
    }
    // Only after every test passed: a failed init leaves no plausible
    // handoff behind for a later stage (or a warm reset) to trust.
    write_osreg(io, cfg, &info);
    io.log(format_args!("ddr: {} MiB up\n", info.total_mib));
    Ok(info)
}

/// Per-channel CPU test in the linear layout, with the software sweeps of
/// [`fallback`] for a failing channel.
fn check_channels<H: RegIo + ?Sized>(
    io: &mut H,
    cfg: &Config,
    info: &mut DramInfo,
    ch_mib: u64,
) -> Result<(), Error> {
    let Some(win) = msch::test_window(ch_mib) else {
        io.log(format_args!(
            "ddr: no linear layout for {ch_mib} MiB channels: whole-memory test only\n"
        ));
        return Ok(());
    };
    if win < ch_mib {
        io.log(format_args!(
            "ddr: {ch_mib} MiB channels tested through {win} MiB linear windows (rank 0); the rest by the whole-memory test\n"
        ));
    }
    msch::set_layout(io, Layout::Linear { mib: win }).map_err(Error::Layout)?;
    for ch in 0..4 {
        let c = info.ch[ch];
        if !c.present {
            continue;
        }
        let base = linear_base(ch, win);
        let ch_mib = win;
        if in_register_window(base) || in_register_window(base + ((ch_mib as usize / 2) << 20)) {
            io.log(format_args!(
                "ddr ch{ch}: test addresses in the register window, skipped\n"
            ));
            continue;
        }
        let mut test = |io: &mut H| verify::quick(io, base, ch_mib, CHANNEL_TEST_WORDS).is_ok();
        if c.train.ok() && test(io) {
            continue;
        }
        if !cfg.fallback {
            return Err(Error::Memtest {
                ch: Some(ch),
                addr: base,
            });
        }
        io.log(format_args!("ddr ch{ch}: test failed, sweeping\n"));
        let p = timing::plan(cfg, c.ranks, c.geometry.map_or(16, |g| g.density_gb));
        info.ch[ch].dfi = fallback::dfi(io, ch, p.rl, p.wl, cfg.dfi.tphy_wrdata, &mut test);
        if info.ch[ch].dfi.is_none() {
            info.ch[ch].rx_vref = fallback::rx_vref(io, ch, &mut test);
        }
        if !test(io) {
            return Err(Error::Memtest {
                ch: Some(ch),
                addr: base,
            });
        }
    }
    Ok(())
}

/// Write OS_REG2..5 for the initialised channels (version 3 encoding):
/// OS_REG2/3 carry channels 0/1, OS_REG4/5 channels 2/3. A pair holds 1 or
/// 2 channels, so absent channels are left out (only debug layouts have
/// any); a pair with none is written as zeros.
pub fn write_osreg<H: RegIo + ?Sized>(io: &mut H, cfg: &Config, info: &DramInfo) {
    let geo = |c: &ChannelInfo| {
        c.geometry.filter(|_| c.present).map(|g| osreg::Geometry {
            rank: c.ranks.max(1) as u32,
            col: g.col,
            bk: g.bank_bits,
            cs0_row: g.row,
            cs1_row: g.row,
            bw: 1,
            dbw: if g.byte_mode {
                osreg::DBW_X8
            } else {
                osreg::DBW_X16
            },
        })
    };
    let t = cfg.dram_type.osreg_code();
    let pair = |a: &ChannelInfo, b: &ChannelInfo| {
        let mut v = [osreg::Geometry {
            rank: 1,
            col: 10,
            bk: 3,
            cs0_row: 16,
            cs1_row: 16,
            bw: 1,
            dbw: osreg::DBW_X16,
        }; 2];
        let mut n = 0;
        for g in [geo(a), geo(b)].into_iter().flatten() {
            v[n] = g;
            n += 1;
        }
        if n == 0 {
            (0, 0)
        } else {
            osreg::encode(t, &v[..n])
        }
    };
    let (r2, r3) = pair(&info.ch[0], &info.ch[1]);
    let (r4, r5) = pair(&info.ch[2], &info.ch[3]);
    io.write32(crate::regs::OS_REG2, r2);
    io.write32(crate::regs::OS_REG3, r3);
    io.write32(crate::regs::OS_REG4, r4);
    io.write32(crate::regs::OS_REG5, r5);
}

/// Read OS_REG2..5 back: Some when both pairs are plausible (DRAM
/// initialised before bt0, e.g. by the stepping-stone image's rkbin DDR
/// init). Identification only (licence guard).
pub fn handoff_present<H: RegIo + ?Sized>(io: &mut H) -> Option<(osreg::Pair, osreg::Pair)> {
    let a = osreg::decode(
        io.read32(crate::regs::OS_REG2),
        io.read32(crate::regs::OS_REG3),
    );
    let b = osreg::decode(
        io.read32(crate::regs::OS_REG4),
        io.read32(crate::regs::OS_REG5),
    );
    (osreg::plausible(&a) && osreg::plausible(&b)).then_some((a, b))
}
