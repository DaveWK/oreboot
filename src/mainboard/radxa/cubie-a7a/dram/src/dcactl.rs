// SPDX-License-Identifier: GPL-2.0-only
//! WCK duty-cycle adjuster (DCA) training through the DDR controller, in
//! mission mode: the duty-cycle monitor (DCM) is started, flipped and
//! stopped with the controller's own mode-register writes and read with
//! its mode-register read (`crate::mr`), after the hand-over. Lab stage
//! (feature `full-speed`): nothing here runs unless a build knob asks for
//! it (bt0 `A7A_DCA_CTL`).
//!
//! Why a second route next to [`crate::dca`] (the same sweep through the
//! PHY's ACSM, in the PHY's training mode): that one reads MR26 from the
//! RX FIFO's debug port, which has not returned a mode register on the
//! board yet; the controller's MRW and MRR are the paths every board run
//! uses (the hand-over's final mode-register writes, the host's MR5/MR8
//! check). The monitor needs WCK running while it measures: after the
//! hand-over the controller holds WCK on (MSTR4.wck_on with MR18 OP[4],
//! the hand-over's final state), which this stage checks before it writes
//! anything. Not checked by anything here: that the monitor then sees the
//! mission WCK; a monitor that does not answer shows as two flat curves
//! and no decision.
//!
//! The sequence per DCA code is the public one (coreboot MT8195
//! `DramcDutyCycleMonitor`, BSD-3; Renesas DBSC5 U-Boot
//! `dbsc5_dramdca_training`, GPL-2.0+): MR30 = code, MR26 start with
//! flip, wait tDCMM, MR26 start without flip, wait tDCMM, MR26 stop, MRR
//! MR26. The decision is [`crate::dca::decide`] (the mean of the two
//! flips' transitions).
//!
//! What this route cannot do: WCK2CK leveling runs in the PHY's training
//! mode, before the hand-over, so it cannot be repeated after a change
//! made here. [`Mode::Apply`] therefore changes the WCK duty under a
//! leveling result found at the old duty (the public flows level again
//! after DCA). It is offered for the measurement of that effect; the
//! write stage, which follows, does centre on the new duty.

use crate::ctrl::{ddrc, MSTR4};
use crate::dca::{
    dcm_bit, decide, final_mr30, mr16_write_operating, mr30_both, step_of, ByteDecision, ByteSweep,
    DcaPolicy, MR26_START, MR26_START_FLIP, MR26_STOP, STEPS,
};
use crate::hw::{Error, Hw};

/// What the stage does (bt0 `A7A_DCA_CTL`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    /// The sweep; MR30 put back as found.
    Measure,
    /// The sweep; the decided bytes' steps written to MR30 (module
    /// documentation: no leveling follows).
    Apply,
}

impl Mode {
    /// `measure`, `apply`; None for anything else.
    pub const fn parse(s: &str) -> Option<Mode> {
        match s.as_bytes() {
            b"measure" => Some(Mode::Measure),
            b"apply" => Some(Mode::Apply),
            _ => None,
        }
    }
}

/// The stage's inputs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Config {
    pub mode: Mode,
    /// MR16 as the hand-over left it (0x04: FSP-OP 1, FSP-WR 0).
    pub mr16_found: u8,
    /// Sweeps per block (results add up per step).
    pub iterations: u8,
    pub policy: DcaPolicy,
    /// MR5 the MRR must return before any write (CXMT 0x13).
    pub mr5_want: u8,
    /// Wait after each MR26 start, in us (tDCMM is 2 us; the timer's
    /// resolution and the command's own latency come on top).
    pub wait_us: u32,
}

impl Config {
    pub const DEFAULT: Config = Config {
        mode: Mode::Measure,
        mr16_found: 0x04,
        iterations: 1,
        policy: DcaPolicy::DEFAULT,
        mr5_want: 0x13,
        wait_us: 4,
    };
}

const _: () = assert!(Config::DEFAULT.wait_us * 1_000_000 >= jedec_lpddr::lp5::timing::T_DCMM_PS);

/// Why a block stopped before its sweep.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Stop {
    /// The controller does not hold WCK on (MSTR4.wck_on clear): the
    /// monitor would measure a stopped clock.
    WckOff,
    /// MR5 did not read as wanted: nothing was written.
    Mr5 { got: u32 },
    /// MR16 did not read as the configuration says, or cannot be moved to
    /// the operating set: nothing was written.
    Mr16 { got: u32 },
    /// A controller command failed; what could be put back was.
    Command(Error),
}

/// One block's (channel, rank) run.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Block {
    pub ch: usize,
    pub rank: u32,
    pub stop: Option<Stop>,
    /// MR30 of the operating set and MR26 before the sweep.
    pub mr30_found: Option<u8>,
    pub mr26_found: Option<u8>,
    /// Per DRAM byte: 0 upper (MR30 OP[7:4], DCMU), 1 lower (DCML).
    pub sweep: [ByteSweep; 2],
    pub decision: [Option<ByteDecision>; 2],
    /// Per sweep step, the last MR26 read.
    pub mr26: [Option<u8>; STEPS],
    pub mr30_final: Option<u8>,
    /// MR30 and MR16 read back as left (measure: as found).
    pub restored: bool,
    pub mrws: u16,
    pub mrrs: u16,
    pub elapsed_us: u32,
}

impl Block {
    pub const fn empty(ch: usize, rank: u32) -> Block {
        Block {
            ch,
            rank,
            stop: None,
            mr30_found: None,
            mr26_found: None,
            sweep: [ByteSweep::EMPTY; 2],
            decision: [None; 2],
            mr26: [None; STEPS],
            mr30_final: None,
            restored: false,
            mrws: 0,
            mrrs: 0,
            elapsed_us: 0,
        }
    }

    /// An applied change: MR30 left other than found.
    pub const fn changed(&self) -> bool {
        match (self.mr30_found, self.mr30_final) {
            (Some(a), Some(z)) => a != z,
            _ => false,
        }
    }
}

/// Mode-register commands of one block, counted.
struct Port<'a, H: Hw> {
    hw: &'a mut H,
    ch: usize,
    rank: u32,
    mrws: u16,
    mrrs: u16,
}

impl<H: Hw> Port<'_, H> {
    fn mrw(&mut self, ma: u32, op: u8) -> Result<(), Error> {
        self.mrws += 1;
        crate::mr::write_rank(self.hw, self.ch, self.rank, ma, op as u32)
    }
    fn mrr(&mut self, ma: u32) -> Result<u8, Error> {
        self.mrrs += 1;
        crate::mr::read(self.hw, self.ch, self.rank, ma).map(|v| v as u8)
    }
}

/// Run the stage on one block. Call in mission mode, after the hand-over's
/// own mode-register writes, with no traffic on the channel.
#[inline(never)]
pub fn run_block<H: Hw>(hw: &mut H, ch: usize, rank: u32, cfg: &Config) -> Block {
    let mut out = Block::empty(ch, rank);
    let t0 = hw.now_us();
    if hw.read32(ddrc(ch, MSTR4)) & 1 == 0 {
        out.stop = Some(Stop::WckOff);
        return out;
    }
    let mut p = Port {
        hw,
        ch,
        rank,
        mrws: 0,
        mrrs: 0,
    };
    if let Err(e) = body(&mut p, &mut out, cfg) {
        // Put back what is known, whatever failed in between.
        if let Some(found) = out.mr30_found {
            let _ = p.mrw(30, found);
            out.mr30_final = Some(found);
        }
        let _ = p.mrw(16, cfg.mr16_found);
        out.stop = Some(Stop::Command(e));
    }
    (out.mrws, out.mrrs) = (p.mrws, p.mrrs);
    out.elapsed_us = p.hw.now_us().wrapping_sub(t0) as u32;
    out
}

fn body<H: Hw>(p: &mut Port<'_, H>, out: &mut Block, cfg: &Config) -> Result<(), Error> {
    let mr5 = p.mrr(5)?;
    if mr5 != cfg.mr5_want {
        out.stop = Some(Stop::Mr5 { got: mr5 as u32 });
        return Ok(());
    }
    let mr16 = p.mrr(16)?;
    let Some(mr16_op) = mr16_write_operating(cfg.mr16_found).filter(|_| mr16 == cfg.mr16_found)
    else {
        out.stop = Some(Stop::Mr16 { got: mr16 as u32 });
        return Ok(());
    };
    // From here every MR30 write lies between this switch and the one back.
    p.mrw(16, mr16_op)?;
    let found = p.mrr(30)?;
    out.mr30_found = Some(found);
    out.mr26_found = Some(p.mrr(26)?);
    for _ in 0..cfg.iterations.max(1) {
        for i in 0..STEPS {
            p.hw.kick_watchdog();
            p.mrw(30, mr30_both(step_of(i)))?;
            for op in [MR26_START_FLIP, MR26_START] {
                p.mrw(26, op)?;
                p.hw.udelay(cfg.wait_us);
            }
            p.mrw(26, MR26_STOP)?;
            // As the PHY-side stage: the read-out at the found duty.
            p.mrw(30, found)?;
            let v = p.mrr(26)?;
            out.mr26[i] = Some(v);
            for flip in 0..2 {
                out.sweep[0].record(flip, i, Some(dcm_bit(v, true, flip)));
                out.sweep[1].record(flip, i, Some(dcm_bit(v, false, flip)));
            }
        }
    }
    let d = [
        decide(&out.sweep[0], &cfg.policy),
        decide(&out.sweep[1], &cfg.policy),
    ];
    out.decision = [Some(d[0]), Some(d[1])];
    let last = match cfg.mode {
        Mode::Apply => final_mr30(found, d[0].step, d[1].step),
        Mode::Measure => found,
    };
    p.mrw(30, last)?;
    out.mr30_final = Some(last);
    let back = p.mrr(30)?;
    p.mrw(16, cfg.mr16_found)?;
    out.restored = back == last && p.mrr(16)? == cfg.mr16_found;
    Ok(())
}

/// The lines of one block (`DCC <channel><rank> ...`).
pub fn log_block<H: Hw>(hw: &mut H, b: &Block) {
    let ch = ["A", "B"][b.ch & 1];
    hw.kick_watchdog();
    hw.log(format_args!(
        "[dram] DCC {ch}{}: stop {:?} restored {}; MR30 found {:02x?} final {:02x?} changed {}; MR26 before {:02x?}; {} MRW, {} MRR, {} us\n",
        b.rank, b.stop, b.restored, b.mr30_found, b.mr30_final, b.changed(), b.mr26_found, b.mrws, b.mrrs, b.elapsed_us
    ));
    if b.mr26.iter().any(|x| x.is_some()) {
        hw.log(format_args!(
            "[dram] DCC {ch}{} MR26 per step -7..+7: {:02x?}\n",
            b.rank, b.mr26
        ));
        for (k, byte) in ["upper", "lower"].iter().enumerate() {
            let s = &b.sweep[k];
            hw.kick_watchdog();
            hw.log(format_args!(
                "[dram] DCC {ch}{} {byte}: flip0 ones {:?} flip1 ones {:?} reads {:?}; decision {:?}\n",
                b.rank, s.ones[0], s.ones[1], s.reads[0], b.decision[k]
            ));
        }
    }
}

/// The stage on every channel and rank of `channels` x `ranks`, logged.
/// Returns how many blocks left another MR30 than they found.
pub fn run<H: Hw>(hw: &mut H, channels: usize, ranks: u32, cfg: &Config) -> u32 {
    hw.log(format_args!(
        "[dram] DCC: WCK duty-cycle monitor through the controller, mission mode: {cfg:x?}\n"
    ));
    let mut changed = 0;
    for rank in 0..ranks {
        for ch in 0..channels {
            let b = run_block(hw, ch, rank, cfg);
            log_block(hw, &b);
            changed += b.changed() as u32;
        }
    }
    changed
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use crate::ctrl::{MRCTRL0, MRCTRL1};
    use crate::dca::{mr30, NoDecision};
    use crate::hw::{Rail, RegIo};
    use core::fmt;
    use jedec_lpddr::lp5::mr::{dca_step, Mr16};
    use std::string::String;
    use std::vec::Vec;

    const MRR_DATA: [usize; 2] = [0x0a02_002c, 0x0a02_0040];

    /// One rank: MR16, MR30 per FSP, the monitor's result bits, and the
    /// WCK duty error per byte (upper, lower) in DCA steps.
    struct Dram {
        mr16: u8,
        mr30: [u8; 3],
        mr26: u8,
        err: [f32; 2],
    }

    /// The controller's mode-register commands on the DRAM model, with a
    /// duty-cycle monitor that has an offset and needs tDCMM.
    struct Model {
        reg: std::collections::HashMap<usize, u32>,
        dram: [[Dram; 2]; 2],
        off: f32,
        mr5: u8,
        /// The monitor answers (WCK reaches it).
        wck: bool,
        /// Microseconds since the last MR26 start.
        since_start: u32,
        short_waits: u32,
        /// Fail the n-th MRW from now with a timeout (the busy bit stays).
        fail_at: Option<u32>,
        mrws: Vec<(usize, u32, u32, u8)>,
        clock: u64,
        log: String,
    }

    impl Model {
        fn new(err: [[[f32; 2]; 2]; 2], mr30_op: u8) -> Model {
            let d = |e: [f32; 2]| Dram {
                mr16: 0x04,
                mr30: [0, mr30_op, 0],
                mr26: 0,
                err: e,
            };
            let mut reg = std::collections::HashMap::new();
            for ch in 0..2 {
                reg.insert(ddrc(ch, MSTR4), 1);
            }
            Model {
                reg,
                dram: [[d(err[0][0]), d(err[0][1])], [d(err[1][0]), d(err[1][1])]],
                off: 1.3,
                mr5: 0x13,
                wck: true,
                since_start: 0,
                short_waits: 0,
                fail_at: None,
                mrws: Vec::new(),
                clock: 0,
                log: String::new(),
            }
        }
        fn channel(addr: usize) -> usize {
            (addr - 0x0a10_0000) / 0x40_0000
        }
        fn mrw(&mut self, ch: usize, rank: u32, ma: u32, op: u8) {
            self.mrws.push((ch, rank, ma, op));
            let (off, wck) = (self.off, self.wck);
            if ma == 26 && self.dram[ch][rank as usize].mr26 & 1 != 0 && self.since_start < 2 {
                // The monitor was running and got less than tDCMM.
                self.short_waits += 1;
            }
            let d = &mut self.dram[ch][rank as usize];
            let m = Mr16::decode(d.mr16);
            match ma {
                16 => d.mr16 = op,
                30 => d.mr30[m.fsp_wr as usize] = op,
                26 => {
                    d.mr26 = (d.mr26 & !3) | (op & 3);
                    if op & 1 != 0 && wck {
                        let flip = (op >> 1 & 1) as usize;
                        let code = d.mr30[m.fsp_op as usize];
                        for (k, upper) in [(0, true), (1, false)] {
                            let nib = if upper { code >> 4 } else { code & 0xf };
                            let x = d.err[k] + dca_step(nib).unwrap() as f32;
                            let b = if flip == 0 {
                                x + off > 0.0
                            } else {
                                x - off < 0.0
                            };
                            let bit = (if upper { 4 } else { 2 }) + flip;
                            d.mr26 = (d.mr26 & !(1 << bit)) | (b as u8) << bit;
                        }
                    }
                    self.since_start = 0;
                }
                _ => {}
            }
        }
    }

    impl RegIo for Model {
        fn read32(&mut self, addr: usize) -> u32 {
            if let Some(ch) = MRR_DATA.iter().position(|&a| a == addr) {
                let c0 = self.reg[&ddrc(ch, MRCTRL0)];
                let ma = self.reg[&ddrc(ch, MRCTRL1)] >> 8;
                let d = &self.dram[ch][(c0 & 0x20 != 0) as usize];
                let v = match ma {
                    5 => self.mr5,
                    16 => d.mr16,
                    26 => d.mr26,
                    30 => d.mr30[Mr16::decode(d.mr16).fsp_wr as usize],
                    _ => 0,
                };
                return (v as u32) << 8;
            }
            *self.reg.get(&addr).unwrap_or(&0)
        }
        fn write32(&mut self, addr: usize, val: u32) {
            let ch = Self::channel(addr);
            if addr == ddrc(ch, MRCTRL0) && val & (1 << 31) != 0 {
                if val & 1 == 0 {
                    if let Some(n) = self.fail_at.as_mut() {
                        if *n == 0 {
                            // The command never completes.
                            self.reg.insert(addr, val);
                            self.fail_at = None;
                            return;
                        }
                        *n -= 1;
                    }
                    let c1 = self.reg[&ddrc(ch, MRCTRL1)];
                    let rank = if val & 0x20 != 0 { 1 } else { 0 };
                    self.mrw(ch, rank, c1 >> 8, c1 as u8);
                }
                self.reg.insert(addr, val & !(1 << 31));
                return;
            }
            self.reg.insert(addr, val);
        }
        fn read16(&mut self, _: usize) -> u16 {
            0
        }
        fn write16(&mut self, _: usize, _: u16) {}
        fn udelay(&mut self, us: u32) {
            self.clock += us as u64;
            self.since_start += us;
        }
        fn now_us(&mut self) -> u64 {
            self.clock += 1000;
            self.clock
        }
        fn log(&mut self, a: fmt::Arguments) {
            use core::fmt::Write;
            self.log.write_fmt(a).unwrap();
        }
    }
    impl Hw for Model {
        fn set_voltage(&mut self, _: Rail, _: u32) -> bool {
            false
        }
    }

    const ERR: [[[f32; 2]; 2]; 2] = [[[2.2, -1.1], [0.4, 3.6]], [[-2.7, 0.0], [5.2, -4.4]]];
    const APPLY: Config = Config {
        mode: Mode::Apply,
        ..Config::DEFAULT
    };

    #[test]
    fn measure_reports_the_decision_and_changes_nothing() {
        let mut m = Model::new(ERR, 0x9a);
        assert_eq!(run(&mut m, 2, 2, &Config::DEFAULT), 0);
        for ch in 0..2 {
            for rank in 0..2 {
                let d = &m.dram[ch][rank];
                assert_eq!((d.mr30, d.mr16), ([0, 0x9a, 0], 0x04));
                assert_eq!(d.mr26 & 1, 0, "the monitor is stopped");
            }
        }
        assert_eq!(m.short_waits, 0);
        let t = &m.log;
        assert!(
            t.contains(
                "DCC A0: stop None restored true; MR30 found Some(9a) final Some(9a) changed false"
            ),
            "{t}"
        );
        assert!(t.contains("DCC B1 lower: flip0 ones"));
        // The decision is there although nothing was applied: err 2.2
        // upper on A0 wants -2.
        let b = run_block(&mut m, 0, 0, &Config::DEFAULT);
        assert_eq!(b.decision[0].unwrap().step, Ok(-2));
        assert_eq!(b.decision[1].unwrap().step, Ok(1));
        assert_eq!((b.mrws, b.mrrs), (2 + 15 * 5 + 1, 5 + 15 + 1));
    }

    #[test]
    fn apply_trains_the_operating_set_inside_the_mr16_window() {
        let mut m = Model::new(ERR, 0x00);
        assert_eq!(run(&mut m, 2, 2, &APPLY), 4);
        for ch in 0..2 {
            for rank in 0..2 {
                let d = &m.dram[ch][rank];
                let want = mr30((-d.err[0]).round() as i8, (-d.err[1]).round() as i8).unwrap();
                let got = d.mr30[1];
                for (g, w) in [(got >> 4, want >> 4), (got & 0xf, want & 0xf)] {
                    let (g, w) = (dca_step(g).unwrap(), dca_step(w).unwrap());
                    assert!(
                        (g - w).abs() <= 1,
                        "ch {ch} rank {rank}: {got:02x} vs {want:02x}"
                    );
                }
                assert_eq!((d.mr30[0], d.mr30[2], d.mr16), (0, 0, 0x04));
            }
        }
        let mut open = [[false; 2]; 2];
        for &(ch, rank, ma, op) in &m.mrws {
            match ma {
                16 => open[ch][rank as usize] = op == 0x05,
                30 | 26 => assert!(open[ch][rank as usize], "MR{ma} outside the window"),
                _ => panic!("MR{ma} written"),
            }
        }
    }

    #[test]
    fn nothing_is_written_without_wck_mr5_or_the_expected_mr16() {
        let mut m = Model::new(ERR, 0);
        m.reg.insert(ddrc(1, MSTR4), 0);
        assert_eq!(run_block(&mut m, 1, 0, &APPLY).stop, Some(Stop::WckOff));
        m.mr5 = 0xff;
        assert_eq!(
            run_block(&mut m, 0, 1, &APPLY).stop,
            Some(Stop::Mr5 { got: 0xff })
        );
        m.mr5 = 0x13;
        m.dram[0][0].mr16 = 0x45;
        assert_eq!(
            run_block(&mut m, 0, 0, &APPLY).stop,
            Some(Stop::Mr16 { got: 0x45 })
        );
        // A CBT value is refused even when the DRAM agrees with it.
        m.dram[0][0].mr16 = 0x14;
        let cbt = Config {
            mr16_found: 0x14,
            ..APPLY
        };
        assert_eq!(
            run_block(&mut m, 0, 0, &cbt).stop,
            Some(Stop::Mr16 { got: 0x14 })
        );
        assert!(m.mrws.is_empty());
    }

    #[test]
    fn a_monitor_without_wck_gives_no_decision_and_no_change() {
        let mut m = Model::new(ERR, 0x21);
        m.wck = false;
        let b = run_block(&mut m, 0, 0, &APPLY);
        assert_eq!(b.stop, None);
        for d in b.decision {
            assert_eq!(d.unwrap().step, Err(NoDecision::Flat));
        }
        assert!(!b.changed() && b.restored);
        assert_eq!(m.dram[0][0].mr30, [0, 0x21, 0]);
    }

    #[test]
    fn a_failed_command_puts_mr30_and_mr16_back() {
        // The 20th MRW of the block never completes (inside the sweep).
        let mut m = Model::new(ERR, 0x21);
        m.fail_at = Some(20);
        let b = run_block(&mut m, 1, 1, &APPLY);
        assert!(
            matches!(b.stop, Some(Stop::Command(Error::Timeout(_)))),
            "{:?}",
            b.stop
        );
        // The later commands go through: the found MR30 and MR16 are back
        // in the DRAM, and the block reports that it did not finish.
        assert!(!b.restored);
        assert_eq!(b.mr30_final, Some(0x21));
        assert!(!b.changed());
        assert_eq!((m.dram[1][1].mr30, m.dram[1][1].mr16), ([0, 0x21, 0], 0x04));
    }

    #[test]
    fn mode_text() {
        assert_eq!(Mode::parse("measure"), Some(Mode::Measure));
        assert_eq!(Mode::parse("apply"), Some(Mode::Apply));
        assert_eq!(Mode::parse("mrr"), None);
    }
}
