// SPDX-License-Identifier: GPL-2.0-only
//! RK806 over SPI2: the corrected register sequence and a wire-level model.
//!
//! The model is a Rockchip SPI controller (TRM P1 Chapter 31; CTRLR0.bht
//! p.1933: 0 = "APB 16bit write/read, spi 8bit write/read", one FIFO access
//! is two 8-bit frames, little endian; 1 = one access per 8-bit frame)
//! wired to an RK806 that decodes command, address low, address high
//! (Linux drivers/mfd/rk8xx-spi.c) and shifts the addressed register out
//! after the three header bytes. It reproduces the first hardware run's
//! `chip_name=0x00` with the old CTRLR0 = 0x1 and reads the real values
//! with the corrected CTRLR0.
//!
//! The model also knows receive-only transfers (CTRLR0.xfm = 2, CTRLR1 + 1
//! frames clocked on ENR = 1) and, for the third hardware run, a board on
//! which MISO and MOSI are one line (`shared`): whoever drives it wins,
//! the SoC's MOSI driver over the RK806, so a full-duplex read returns the
//! bytes sent. Whether receive-only mode still drives MOSI is unknown
//! (`ro_drives_mosi`); GPIO0_A6 muxed to GPIO (mux 0) never drives it.

use std::collections::{BTreeMap, VecDeque};

use regio::RegIo;
use regtrace::{Ev, Recorder};
use rk3588::regs::{base, ioc, spi, wm, GPIO0};
use rk3588::rk806::{self, ReadMode, Readback, Spi2};

const SPI: usize = base::SPI2;

/// RK806 register file as a board would have it: id 0x8061 (an RK806
/// revision 1 [I]), BUCK5 0.85 V on, BUCK6 1.1 V on, BUCK9 0.6 V on, NLDO2
/// 0.85 V on. Values are model inputs, not hardware readings.
fn pmic_regs() -> BTreeMap<u16, u8> {
    BTreeMap::from([
        (rk806::CHIP_NAME as u16, 0x80),
        (rk806::CHIP_VER as u16, 0x61),
        (rk806::POWER_EN1 as u16, 0x03),
        (rk806::POWER_EN2 as u16, 0x01),
        (rk806::POWER_EN3 as u16, 0x02),
        (rk806::BUCK5_ON_VSEL as u16, rk806::buck_vsel(850_000)),
        (rk806::BUCK6_ON_VSEL as u16, rk806::buck_vsel(1_100_000)),
        (rk806::BUCK9_ON_VSEL as u16, rk806::buck_vsel(600_000)),
        (rk806::NLDO2_ON_VSEL as u16, rk806::ldo_vsel(850_000)),
    ])
}

#[derive(Default)]
struct Model {
    other: BTreeMap<usize, u32>,
    ctrlr0: u32,
    ctrlr1: u32,
    enr: u32,
    /// MISO and MOSI joined on the board.
    shared: bool,
    /// Receive-only mode drives MOSI (low) while SPI2_MOSI_M2 is muxed.
    ro_drives_mosi: bool,
    /// GPIO0_A6 iomux (1 = SPI2_MOSI_M2).
    mosi_mux: u32,
    ser: u32,
    rx: VecDeque<u32>,
    /// MOSI bytes of the current ss_n-low window.
    frame: Vec<u8>,
    pmic: BTreeMap<u16, u8>,
    /// Every frame seen on the wire (MOSI bytes per ss_n window).
    wire: Vec<Vec<u8>>,
    clock: u64,
    /// Log lines printed by the code under test.
    log: String,
}

impl Model {
    fn new() -> Self {
        Model {
            pmic: pmic_regs(),
            mosi_mux: 1,
            ..Default::default()
        }
    }

    /// MISO and MOSI joined; receive-only drives MOSI or not.
    fn shared(ro_drives_mosi: bool) -> Self {
        Model {
            shared: true,
            ro_drives_mosi,
            ..Model::new()
        }
    }

    /// One frame on the wire: `mosi` is what the SoC drives (None:
    /// nothing), returns what the SoC samples on MISO.
    fn frame_io(&mut self, mosi: Option<u8>) -> u8 {
        // An undriven line idles high (pull-ups); the RK806 only listens
        // to the 3 header bytes, which the SoC always drives.
        let out = self.shift(mosi.unwrap_or(0xff));
        match (self.shared, mosi) {
            (true, Some(m)) => m,
            _ => out,
        }
    }

    fn xfm(&self) -> u32 {
        (self.ctrlr0 & spi::CR0_XFM_MASK) >> spi::CR0_XFM_SHIFT
    }

    /// One 8-bit frame: MOSI in, MISO out (the RK806 side).
    fn shift(&mut self, mosi: u8) -> u8 {
        let n = self.frame.len();
        self.frame.push(mosi);
        if n < 3 {
            return 0;
        }
        let cmd = self.frame[0];
        let reg = u16::from_le_bytes([self.frame[1], self.frame[2]]) + (n as u16 - 3);
        if cmd & 0x80 == 0 {
            // Read: unknown addresses read 0.
            self.pmic.get(&reg).copied().unwrap_or(0)
        } else {
            if reg < 0x100 {
                let old = self.pmic.get(&reg).copied().unwrap_or(0);
                // POWER_ENx: high nibble is the write mask.
                let v = if reg <= 5 {
                    (old & !(mosi >> 4)) | (mosi & (mosi >> 4) & 0x0f)
                } else {
                    mosi
                };
                self.pmic.insert(reg, v);
            }
            0
        }
    }

    fn tx(&mut self, v: u32) {
        assert!(
            self.enr == 1 && self.ser & 1 == 1,
            "TXDR write with ss_n high"
        );
        if self.ctrlr0 & spi::CR0_LBK != 0 {
            // Loopback: rxd = txd, nothing on the bus (TRM P1 p.1932).
            self.rx.push_back(v & 0xff);
        } else if self.ctrlr0 & spi::CR0_BHT_8BIT != 0 {
            assert_eq!(
                self.xfm(),
                spi::XFM_TR,
                "TXDR write outside transmit+receive"
            );
            assert_eq!(self.mosi_mux, 1, "TXDR write with MOSI not muxed");
            let m = self.frame_io(Some(v as u8));
            self.rx.push_back(m as u32);
        } else {
            let lo = self.shift(v as u8);
            let hi = self.shift((v >> 8) as u8);
            self.rx.push_back(lo as u32 | (hi as u32) << 8);
        }
    }

    /// ENR set in receive-only mode: CTRLR1 + 1 frames.
    fn receive_only(&mut self) {
        assert_eq!(self.ser & 1, 1, "receive-only with ss_n high");
        for _ in 0..=self.ctrlr1 {
            let drive = (self.ro_drives_mosi && self.mosi_mux == 1).then_some(0x00);
            let m = self.frame_io(drive);
            self.rx.push_back(m as u32);
        }
    }
}

impl RegIo for Model {
    fn read32(&mut self, addr: usize) -> u32 {
        match addr.wrapping_sub(SPI) {
            spi::SR => {
                if self.rx.is_empty() {
                    spi::SR_RFE | spi::SR_TFE
                } else {
                    spi::SR_TFE
                }
            }
            spi::RXDR => self.rx.pop_front().expect("RXDR read with empty FIFO"),
            spi::VERSION => 0x0011_0002,
            _ => self.other.get(&addr).copied().unwrap_or(0),
        }
    }
    fn write32(&mut self, addr: usize, val: u32) {
        match addr.wrapping_sub(SPI) {
            spi::CTRLR0 => self.ctrlr0 = val,
            spi::CTRLR1 => self.ctrlr1 = val,
            spi::ENR => {
                let was = self.enr;
                self.enr = val;
                if val == 0 {
                    self.rx.clear();
                } else if was == 0 && self.xfm() == spi::XFM_RO {
                    self.receive_only();
                }
            }
            spi::SER => {
                if self.ser & 1 == 0 && val & 1 == 1 {
                    self.frame.clear();
                }
                if self.ser & 1 == 1 && val & 1 == 0 {
                    self.wire.push(core::mem::take(&mut self.frame));
                }
                self.ser = val;
            }
            spi::TXDR => self.tx(val),
            _ if addr == base::PMU1_IOC + ioc::PMU1_GPIO0A_SEL_H => {
                if val & (0xF << 24) != 0 {
                    self.mosi_mux = (val >> 8) & 0xF;
                }
                self.other.insert(addr, val);
            }
            _ => {
                self.other.insert(addr, val);
            }
        }
    }
    fn read16(&mut self, _: usize) -> u16 {
        unreachable!()
    }
    fn write16(&mut self, _: usize, _: u16) {
        unreachable!()
    }
    fn udelay(&mut self, us: u32) {
        self.clock += us as u64;
    }
    fn now_us(&mut self) -> u64 {
        self.clock += 1;
        self.clock
    }
    fn log(&mut self, args: core::fmt::Arguments) {
        self.log.push_str(&args.to_string());
    }
}

#[test]
fn old_ctrlr0_reads_zero_like_the_first_hardware_run() {
    let mut m = Model::new();
    let mut s = Spi2::init(&mut m);
    s.mode = ReadMode::Duplex;
    // The pre-fix init wrote CTRLR0 = 1 (bht = 0).
    m.write32(SPI + spi::CTRLR0, 1);
    assert_eq!(s.read(&mut m, rk806::CHIP_NAME).unwrap(), 0x00);
    // The RK806 saw command 0x00, then address 0x5A00, not 0x005A.
    assert_eq!(&m.wire[0][..4], &[0x00, 0x00, 0x5A, 0x00]);
}

#[test]
fn corrected_ctrlr0_reads_the_pmic() {
    let mut m = Model::new();
    let r = rk806::readback(&mut m).unwrap();
    assert_eq!(r.spi_version, 0x0011_0002);
    assert_eq!((r.chip_name, r.chip_ver), (0x80, 0x61));
    assert!(r.id_ok());
    // The Linux-style read worked first time: only it was tried.
    assert_eq!(r.mode, ReadMode::Split);
    assert_eq!(r.ident.tried, 1);
    assert_eq!(r.ident.to_string(), "split=8061");
    // Each register read is one ss_n window: command, address low, high,
    // then one receive-only frame (MOSI undriven, idles high).
    assert_eq!(m.wire[0], vec![0x00, rk806::CHIP_NAME, 0x00, 0xff]);
    // id (2) + enables and VSELs (7) + state registers + every rail VSEL.
    assert_eq!(
        m.wire.len(),
        9 + rk806::STATE_REGS.len() + rk806::ALL_RAILS.len()
    );
    assert_eq!(r.state[1], 0x03); // POWER_EN1
    assert_eq!(r.state[13], 0x00); // 0xC3, not in the model
    let rails = r.rails();
    assert_eq!(rails[0].2, 850_000);
    // BUCK6/BUCK9: external feedback, VSEL unused.
    assert_eq!(rails[1].2, 0);
    assert_eq!(rails[2].2, 0);
    assert_eq!(rails[3].2, 850_000);
    assert!(rails.iter().all(|x| x.3));
    // No write frame on the read-only path.
    assert!(m.wire.iter().all(|f| f[0] & 0x80 == 0));
}

#[test]
fn ddr_rails_refuses_to_write_an_unidentified_pmic() {
    let mut m = Model::new();
    m.pmic.insert(rk806::CHIP_NAME as u16, 0x00);
    assert!(rk806::ddr_rails(&mut m).is_err());
    assert!(m.wire.iter().all(|f| f[0] & 0x80 == 0));
}

#[test]
fn ddr_rails_enables_an_off_rail() {
    let mut m = Model::new();
    m.pmic.insert(rk806::POWER_EN2 as u16, 0x00); // BUCK9 off
    let rep = rk806::ddr_rails(&mut m).unwrap();
    assert_eq!(rep.enabled_now, 4);
    assert_eq!(m.pmic[&(rk806::POWER_EN2 as u16)] & 1, 1);
    assert!(m.wire.contains(&vec![0x80, rk806::POWER_EN2, 0x00, 0x11]));
}

/// The exact init and one full-duplex read, as a register trace.
#[test]
fn init_and_read_trace() {
    let mut rec = Recorder {
        responder: Some(Box::new(|a, _| match a as usize {
            x if x == SPI + spi::SR => Some(spi::SR_TFE),
            x if x == SPI + spi::RXDR => Some(0x80),
            _ => None,
        })),
        ..Recorder::default()
    };
    let mut s = Spi2::init(&mut rec);
    s.mode = ReadMode::Duplex;
    assert_eq!(s.read(&mut rec, rk806::CHIP_NAME).unwrap(), 0x80);
    let a = |off: usize| (SPI + off) as u64;
    let cru = base::CRU as u64;
    let ioc = base::PMU1_IOC as u64;
    let want = vec![
        // Pinmux SPI2_CLK/MOSI_M2 (GPIO0_A5/A6), CS0/MISO_M2 (GPIO0_B1/B3).
        Ev::W(32, ioc + 0x4, wm(4, 4, 1) | wm(8, 4, 1)),
        Ev::W(32, ioc + 0x8, wm(4, 4, 1) | wm(12, 4, 1)),
        // Pull-ups (pe = ps = 1) on A5/A6 and B1/B3.
        Ev::W(32, ioc + 0x20, 0x3C00_3C00),
        Ev::W(32, ioc + 0x24, 0x00CC_00CC),
        // Ungate pclk_spi2/clk_spi2, clk_spi2 = 24 MHz, reset pulse.
        Ev::W(32, cru + 0x838, 0x2100_0000),
        Ev::W(32, cru + 0x3EC, 0x00C0_0080),
        Ev::W(32, cru + 0xA38, 0x2100_2100),
        Ev::Delay(10),
        Ev::W(32, cru + 0xA38, 0x2100_0000),
        Ev::Delay(10),
        Ev::W(32, a(spi::ENR), 0),
        Ev::W(32, a(spi::SER), 0),
        Ev::W(32, a(spi::IMR), 0),
        Ev::W(32, a(spi::CTRLR0), 0x2C01),
        Ev::W(32, a(spi::CTRLR1), 0),
        Ev::W(32, a(spi::BAUDR), 24),
        // Read CHIP_NAME: ss_n0 low, 00 5A 00 + one dummy byte, all four
        // in the FIFO before the first RX byte is read.
        Ev::W(32, a(spi::SER), 1),
        Ev::W(32, a(spi::ENR), 1),
        Ev::W(32, a(spi::TXDR), 0x00),
        Ev::W(32, a(spi::TXDR), 0x5A),
        Ev::W(32, a(spi::TXDR), 0x00),
        Ev::W(32, a(spi::TXDR), 0x00),
        Ev::R(32, a(spi::SR), spi::SR_TFE),
        Ev::R(32, a(spi::RXDR), 0x80),
        Ev::R(32, a(spi::SR), spi::SR_TFE),
        Ev::R(32, a(spi::RXDR), 0x80),
        Ev::R(32, a(spi::SR), spi::SR_TFE),
        Ev::R(32, a(spi::RXDR), 0x80),
        Ev::R(32, a(spi::SR), spi::SR_TFE),
        Ev::R(32, a(spi::RXDR), 0x80),
        Ev::R(32, a(spi::SR), spi::SR_TFE),
        Ev::W(32, a(spi::ENR), 0),
        Ev::W(32, a(spi::SER), 0),
        Ev::Delay(2),
    ];
    assert_eq!(rec.evs, want, "\n{}", regtrace::render(&rec.evs));
}

#[test]
fn readback_default_is_not_an_rk806() {
    assert!(!Readback::default().id_ok());
}

/// Every SPI2 MOSI byte the diagnostics send, from a register trace in
/// which the controller returns `rx` for every RXDR read.
fn diag_trace(rx: u32) -> Recorder {
    let mut rec = Recorder {
        responder: Some(Box::new(move |a, _| match a as usize {
            x if x == SPI + spi::SR => Some(spi::SR_TFE),
            x if x == SPI + spi::RXDR => Some(rx),
            _ => None,
        })),
        ..Recorder::default()
    };
    rk806::diagnose(&mut rec);
    rec
}

fn writes(rec: &Recorder) -> Vec<(usize, u32)> {
    rec.evs
        .iter()
        .filter_map(|e| match e {
            Ev::W(32, a, v) => Some((*a as usize, *v)),
            _ => None,
        })
        .collect()
}

/// The diagnostics are read-only for the RK806: every byte on MOSI has
/// bit 7 clear (an RK806 write command needs bit 7, Linux rk8xx-spi.c
/// RK806_CMD_WRITE), whatever the RK806 takes as the command byte.
#[test]
fn diagnose_sends_no_rk806_write_command() {
    let rec = diag_trace(0);
    let tx: Vec<u32> = writes(&rec)
        .into_iter()
        .filter(|&(a, _)| a == SPI + spi::TXDR)
        .map(|(_, v)| v)
        .collect();
    // Loopback (3) + 4 full-duplex reads of 4 bytes + 2 split headers.
    assert_eq!(tx.len(), 3 + 4 * 4 + 2 * 3);
    assert!(tx.iter().all(|&v| v < 0x80), "{tx:x?}");
    // Every frame is the CHIP_NAME read (or the loopback bytes).
    for c in tx[3..19].chunks(4) {
        assert_eq!(c, [0x00, 0x5A, 0x00, 0x00]);
    }
    for c in tx[19..].chunks(3) {
        assert_eq!(c, [0x00, 0x5A, 0x00]);
    }
}

/// The only IOC and GPIO0 writes are to the SPI2 M2 pins' mux and pull
/// fields and to GPIO0_B1 (bit 9), and the pins end as Spi2::init sets
/// them (the last writes are a full init).
#[test]
fn diagnose_touches_only_the_spi2_pins() {
    let rec = diag_trace(0);
    let ioc = base::PMU1_IOC;
    let gpio = rk3588::regs::GPIO0;
    for (a, v) in writes(&rec) {
        let en = v >> 16;
        match a {
            x if x == ioc + 0x4 => assert_eq!(en & !0x0FF0, 0, "{v:#x}"),
            x if x == ioc + 0x8 => assert_eq!(en & !0xF0F0, 0, "{v:#x}"),
            x if x == ioc + 0x20 => assert_eq!(en & !0x3C00, 0, "{v:#x}"),
            x if x == ioc + 0x24 => assert_eq!(en & !0x00CC, 0, "{v:#x}"),
            x if x == gpio || x == gpio + 0x8 => {
                assert!(en == 1 << 9 || en == 1 << 6, "{v:#x}")
            }
            x if (ioc..ioc + 0x1_0000).contains(&x) || (gpio..gpio + 0x100).contains(&x) => {
                panic!("unexpected write {x:#x} = {v:#x}")
            }
            _ => {}
        }
    }
    // Software CS: B1 back to SPI2_CS0_M2 and GPIO input before the end.
    let w = writes(&rec);
    for bit in [9, 6] {
        let last = w
            .iter()
            .rposition(|&(a, v)| a == gpio + 0x8 && v >> 16 == 1 << bit)
            .unwrap();
        assert_eq!(w[last].1, 0x1_0000 << bit, "GPIO0 bit {bit} left an output");
    }
    let mut fresh = Recorder::default();
    Spi2::init(&mut fresh);
    let init = writes(&fresh);
    assert_eq!(&w[w.len() - init.len()..], &init[..]);
}

/// GPIO0_B1 driven high before it is switched to GPIO, low only for the
/// software-CS read, and high again before it goes back to SPI2_CS0_M2.
#[test]
fn diagnose_software_cs_sequence() {
    let rec = diag_trace(0);
    let gpio = rk3588::regs::GPIO0;
    let ioc = base::PMU1_IOC;
    let seq: Vec<(usize, u32)> = writes(&rec)
        .into_iter()
        .filter(|&(a, v)| {
            ((a == gpio || a == gpio + 0x8) && v >> 16 == 1 << 9)
                || (a == ioc + 0x8 && v >> 16 == 0xF0)
        })
        .collect();
    let dr = |l: u32| (gpio, 0x0200_0000 | l << 9);
    let ddr = |o: u32| (gpio + 0x8, 0x0200_0000 | o << 9);
    let mux = |f: u32| (ioc + 0x8, 0x00F0_0000 | f << 4);
    let mut want = Vec::new();
    for level in [1, 0] {
        // Loopback with CS high, then the read with CS low.
        want.extend([dr(1), ddr(1), mux(0), dr(level), dr(1), mux(1), ddr(0)]);
    }
    // (The inits at the start and the end mux B1 and B3 in one write.)
    assert_eq!(seq, want);
}

/// Against the controller + RK806 model: the loopback passes and every
/// read shows the header bytes as 00 and CHIP_NAME in the 4th byte.
#[test]
fn diagnose_log_against_the_model() {
    let mut m = Model::new();
    rk806::diagnose(&mut m);
    let lines: Vec<&str> = m.log.lines().collect();
    assert_eq!(lines.len(), 8, "{}", m.log);
    assert!(lines.iter().all(|l| l.starts_with("pmic diag ")));
    assert!(
        m.log.contains("loopback tx=5a 3c 69 rx=[5a 3c 69] ok"),
        "{}",
        m.log
    );
    assert!(
        m.log.contains(
            "read tx=00 5a 00 00 hw1M=[00 00 00 80] hw250k=[00 00 00 80] \
             hw250k-m3=[00 00 00 80] sw250k=[00 00 00 80]"
        ),
        "{}",
        m.log
    );
    assert!(
        m.log
            .contains("split1M tx=00 5a 00 +ro1 split=[00 00 00] val=80 tri=[00 00 00] val=80"),
        "{}",
        m.log
    );
    // Only reads reached the RK806: every command byte has bit 7 clear.
    assert!(m
        .wire
        .iter()
        .all(|f| f.first().is_none_or(|&c| c & 0x80 == 0)));
    assert_eq!(m.pmic, pmic_regs());
}

/// A timed-out read prints the wait that failed.
#[test]
fn diagnose_reports_timeouts() {
    let mut rec = Recorder {
        // RX FIFO never fills.
        responder: Some(Box::new(|a, _| match a as usize {
            x if x == SPI + spi::SR => Some(spi::SR_RFE | spi::SR_TFE),
            _ => None,
        })),
        ..Recorder::default()
    };
    rk806::diagnose(&mut rec);
    let log: String = rec
        .evs
        .iter()
        .filter_map(|e| match e {
            Ev::Print(s) => Some(s.as_str()),
            _ => None,
        })
        .collect();
    assert!(log.contains("hw1M=[spi rx: 00 00 00 00]"), "{log}");
    assert!(
        log.contains("loopback tx=5a 3c 69 rx=[spi rx: 00 00 00] BAD"),
        "{log}"
    );
}

#[test]
fn pin_decode() {
    use rk806::PinCfg;
    // TRM reset values (P1 pp.957-964): mux GPIO, A5 pull-down, others
    // none, drive 1, input enabled.
    let reset = PinCfg::decode([0, 0, 0xC751, 0x0030, 0x1111, 0x1111, 0xFF, 0x0F]);
    assert_eq!(reset.mux, [0; 4]);
    assert_eq!(reset.pull, [1, 0, 0, 0]);
    assert_eq!(reset.ds, [1; 4]);
    assert_eq!(reset.ie, [1; 4]);
    assert!(!reset.ok());
    // After Spi2::init (the write-masked values applied to the reset ones).
    let set = PinCfg::decode([0x0110, 0x1010, 0xFF51, 0x00FC, 0x1111, 0x1111, 0xFF, 0x0F]);
    assert_eq!(set.mux, [1; 4]);
    assert_eq!(set.pull, [3; 4]);
    assert!(set.ok());
}

#[test]
fn diag_frame() {
    assert_eq!(rk806::DIAG_TX[..3], rk806::read_frame(rk806::CHIP_NAME));
    // Bit palindromes: the RK806 gets the same request MSB- or LSB-first.
    assert!(rk806::DIAG_TX.iter().all(|b| b.reverse_bits() == *b));
    // 250 kHz and 1 MHz from 24 MHz with an even BAUDR (P1 p.1935).
    for d in rk806::DIAG_READS {
        assert_eq!(d.baudr % 2, 0);
        assert!(24_000_000 / d.baudr <= 1_000_000);
    }
}

/// The Linux-style read (spi_write_then_read: header transmit+receive,
/// value receive-only, ss_n0 held), as a register trace.
#[test]
fn split_read_trace() {
    let mut rec = Recorder {
        responder: Some(Box::new(|a, _| match a as usize {
            x if x == SPI + spi::SR => Some(spi::SR_TFE),
            x if x == SPI + spi::RXDR => Some(0x80),
            _ => None,
        })),
        ..Recorder::default()
    };
    let mut s = Spi2::init(&mut rec);
    rec.evs.clear();
    assert_eq!(s.mode, ReadMode::Split);
    assert_eq!(s.read(&mut rec, rk806::CHIP_NAME).unwrap(), 0x80);
    let a = |off: usize| (SPI + off) as u64;
    let ro = 0x2C01 | spi::XFM_RO << spi::CR0_XFM_SHIFT;
    let rx = |n: usize| {
        (0..n)
            .flat_map(|_| {
                [
                    Ev::R(32, a(spi::SR), spi::SR_TFE),
                    Ev::R(32, a(spi::RXDR), 0x80),
                ]
            })
            .collect::<Vec<_>>()
    };
    let mut want = vec![
        Ev::W(32, a(spi::SER), 1),
        // Header: transmit+receive, 3 frames.
        Ev::W(32, a(spi::ENR), 0),
        Ev::W(32, a(spi::CTRLR0), 0x2C01),
        Ev::W(32, a(spi::CTRLR1), 2),
        Ev::W(32, a(spi::ENR), 1),
        Ev::W(32, a(spi::TXDR), 0x00),
        Ev::W(32, a(spi::TXDR), 0x5A),
        Ev::W(32, a(spi::TXDR), 0x00),
    ];
    want.extend(rx(3));
    want.extend([
        Ev::R(32, a(spi::SR), spi::SR_TFE),
        // Value: receive only, CTRLR1 = 0, no TXDR write.
        Ev::W(32, a(spi::ENR), 0),
        Ev::W(32, a(spi::CTRLR0), ro),
        Ev::W(32, a(spi::CTRLR1), 0),
        Ev::W(32, a(spi::ENR), 1),
    ]);
    want.extend(rx(1));
    want.extend([
        Ev::R(32, a(spi::SR), spi::SR_TFE),
        Ev::W(32, a(spi::ENR), 0),
        Ev::W(32, a(spi::CTRLR0), 0x2C01),
        Ev::W(32, a(spi::CTRLR1), 0),
        Ev::W(32, a(spi::SER), 0),
        Ev::Delay(2),
    ]);
    assert_eq!(rec.evs, want, "\n{}", regtrace::render(&rec.evs));
}

/// The tri-state variant: GPIO0_A6 becomes a GPIO input (direction, then
/// mux) between the header and the receive-only byte, and is muxed back
/// to SPI2_MOSI_M2 before ss_n0 rises.
#[test]
fn split_tristate_trace() {
    let mut rec = Recorder {
        responder: Some(Box::new(|a, _| match a as usize {
            x if x == SPI + spi::SR => Some(spi::SR_TFE),
            x if x == SPI + spi::RXDR => Some(0x80),
            _ => None,
        })),
        ..Recorder::default()
    };
    let mut s = Spi2::init(&mut rec);
    rec.evs.clear();
    s.mode = ReadMode::SplitTristate;
    assert_eq!(s.read(&mut rec, rk806::CHIP_NAME).unwrap(), 0x80);
    let w = writes(&rec);
    let pos = |a: usize, v: u32| w.iter().position(|&x| x == (a, v)).unwrap();
    let ddr_in = pos(GPIO0 + 0x8, 0x0040_0000);
    let mux_gpio = pos(base::PMU1_IOC + 0x4, 0x0F00_0000);
    let mux_spi = pos(base::PMU1_IOC + 0x4, 0x0F00_0100);
    let ro = w
        .iter()
        .position(|&(a, v)| a == SPI + spi::CTRLR0 && v & spi::CR0_XFM_MASK != 0)
        .unwrap();
    let last_tx = w.iter().rposition(|&(a, _)| a == SPI + spi::TXDR).unwrap();
    let ser0 = w.iter().rposition(|&x| x == (SPI + spi::SER, 0)).unwrap();
    assert!(last_tx < ddr_in && ddr_in < mux_gpio && mux_gpio < ro);
    assert!(ro < mux_spi && mux_spi < ser0);
    // Nothing else touches the IOC or GPIO0.
    let other = w
        .iter()
        .filter(|&&(a, _)| !(SPI..SPI + 0x1000).contains(&a))
        .count();
    assert_eq!(other, 3);
}

/// The third hardware run, reproduced: MISO and MOSI joined, so the
/// full-duplex read returns the bytes sent (`00 5a 00 00`). If
/// receive-only mode leaves MOSI alone, the Linux-style read works; if it
/// drives MOSI, only the tri-state variant does. readback() finds the
/// working mode either way.
#[test]
fn shared_line_board() {
    for (ro_drives, mode, tried) in [
        (false, ReadMode::Split, 1),
        (true, ReadMode::SplitTristate, 2),
    ] {
        let mut m = Model::shared(ro_drives);
        let mut s = Spi2::init(&mut m);
        s.mode = ReadMode::Duplex;
        let mut rx = [0u8; 4];
        s.duplex(&mut m, &rk806::DIAG_TX, &mut rx).unwrap();
        assert_eq!(rx, [0x00, 0x5A, 0x00, 0x00]);

        let mut m = Model::shared(ro_drives);
        let r = rk806::readback(&mut m).unwrap();
        assert!(r.id_ok(), "ro_drives_mosi={ro_drives}");
        assert_eq!(r.mode, mode);
        assert_eq!(r.ident.tried, tried);
        assert_eq!(r.rails()[0].2, 850_000);
        assert_eq!(m.mosi_mux, 1);
        if ro_drives {
            assert_eq!(r.ident.to_string(), "split=0000 tri=8061");
        }

        let mut m = Model::shared(ro_drives);
        rk806::diagnose(&mut m);
        let want = if ro_drives {
            "split=[00 5a 00] val=00 tri=[00 5a 00] val=80"
        } else {
            "split=[00 5a 00] val=80 tri=[00 5a 00] val=80"
        };
        assert!(m.log.contains(want), "{}", m.log);
        assert!(m.log.contains("hw1M=[00 5a 00 00]"), "{}", m.log);
        assert_eq!(m.pmic, pmic_regs());
    }
}

/// No mode reads an RK806: every mode was tried, the id is reported as
/// read, and ddr_rails() writes nothing.
#[test]
fn no_rk806_in_any_mode() {
    let mut m = Model::shared(true);
    m.pmic.insert(rk806::CHIP_NAME as u16, 0x00);
    let r = rk806::readback(&mut m).unwrap();
    assert!(!r.id_ok());
    assert_eq!(r.ident.tried, 3);
    assert_eq!(r.mode, ReadMode::Split);
    assert_eq!(r.state, [0; rk806::STATE_REGS.len()]);
    let mut m = Model::shared(true);
    m.pmic.insert(rk806::CHIP_NAME as u16, 0x00);
    assert!(rk806::ddr_rails(&mut m).is_err());
    assert!(m.wire.iter().all(|f| f[0] & 0x80 == 0));
    assert_eq!(m.mosi_mux, 1);
}

/// readback() is read-only on every path, including the state registers.
#[test]
fn readback_writes_nothing_to_the_pmic() {
    for mut m in [Model::new(), Model::shared(false), Model::shared(true)] {
        rk806::readback(&mut m).unwrap();
        assert!(m.wire.iter().all(|f| f[0] & 0x80 == 0));
        assert_eq!(m.pmic, pmic_regs());
    }
}

/// ddr_rails() on the joined-line board: the write goes out as one
/// 4-byte transfer after a good id.
#[test]
fn ddr_rails_on_a_shared_line_board() {
    let mut m = Model::shared(true);
    m.pmic.insert(rk806::POWER_EN2 as u16, 0x00);
    let rep = rk806::ddr_rails(&mut m).unwrap();
    assert_eq!(rep.chip, 0x8061);
    assert_eq!(rep.enabled_now, 4);
    assert!(m.wire.contains(&vec![0x80, rk806::POWER_EN2, 0x00, 0x11]));
}

/// The 2026-11-09 03:17 hardware state: all 21 outputs on, PWRCTRL1-3
/// null (polarity 1), no DEV_SLP/DEV_OFF: plain ACTIVE.
#[test]
fn decode_the_0317_state() {
    let st = [
        0x0f, 0x0f, 0x03, 0x0f, 0x0f, 0x07, 0x80, 0x88, 0x08, 0x00, 0x00, 0x40, 0x00, 0x4a, 0x09,
        0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00,
    ];
    let d = rk806::Decoded::from_state(&st);
    assert_eq!(d.rails_on, 21);
    assert_eq!(d.pwrctrl_fun, [0, 0, 0]);
    assert_eq!(d.pwrctrl_pol, [1, 1, 1]);
    assert!(!d.dev_slp && !d.dev_off);
    assert!(d.active());
    // PWRCTRL1 as sleep (EDK2 sets power-off, function 2) is not plain ACTIVE.
    let mut slp = st;
    slp[7] = 0x89;
    assert!(!rk806::Decoded::from_state(&slp).active());
    let mut off = st;
    off[4] = 0x0e; // PLDO6 off
    assert_eq!(rk806::Decoded::from_state(&off).rails_on, 20);
}

/// ON_SOURCE / OFF_SOURCE / SYS_STS / INT_STS0 names and the SYS_CFG0/1,
/// SYS_OPTION settings (RK806 datasheet Rev 1.4 register description).
#[test]
fn power_decode() {
    let reg = |a: u8| {
        rk806::STATE_REGS
            .iter()
            .position(|r| r.1 == a)
            .unwrap_or_else(|| panic!("{a:#04x} not in STATE_REGS"))
    };
    let state = |v: &[(u8, u8)]| {
        let mut st = [0u8; rk806::STATE_REGS.len()];
        for &(a, x) in v {
            st[reg(a)] = x;
        }
        st
    };
    // The abnormal restart seen on the board; SYS_CFG0 at its reset value.
    let p = rk806::Power::from_state(&state(&[(0x74, 0x20), (0x75, 0x02), (0x5E, 0x0c)]));
    assert_eq!((p.on_src, p.off_src, p.sys_cfg0), (0x20, 0x02, 0x0c));
    assert_eq!(
        p.to_string(),
        "on=ON_ABNORMAL off=OFF_ABNORMAL sts=none int=none"
    );
    assert_eq!(
        p.cfg().to_string(),
        "vb_uv=2700mV/5us vb_lo=3200mV/irq abnordet=on ov_sd=off uvlock=off vccxdet=0"
    );
    // ... with the NRESPWRON restart bit, VDC high, VB_LO and VDC events.
    let p = rk806::Power::from_state(&state(&[
        (0x74, 0x30),
        (0x75, 0x02),
        (0x5D, 0x40),
        (0x77, 0xa0),
    ]));
    assert_eq!(
        p.to_string(),
        "on=ON_ABNORMAL|RESTART_RESETB off=OFF_ABNORMAL sts=VDC int=VB_LO|VDC_RISE"
    );
    // A plain VDC power-on.
    let p = rk806::Power::from_state(&state(&[(0x74, 0x40)]));
    assert_eq!(p.to_string(), "on=ON_VDC off=none sts=none int=none");
    // Overvoltage: OFF_SOURCE VB_SYS_OV [6], SYS_STS VB_OV [0]; undervoltage
    // and low: SYS_STS VB_UV [5], VB_LO [4]; OFF_VB_LO [0].
    let p = rk806::Power::from_state(&state(&[(0x75, 0x41), (0x5D, 0x31), (0x77, 0x40)]));
    assert_eq!(
        p.to_string(),
        "on=none off=VB_SYS_OV|OFF_VB_LO sts=VB_UV|VB_LO|VB_OV int=VDC_FALL"
    );
    // Every bit has a name, bit 7 first.
    let p = rk806::Power::from_state(&state(&[(0x74, 0x81), (0x75, 0x80)]));
    assert_eq!(
        p.to_string(),
        "on=ON_PWRON|RESTART_WDT off=OFF_PWRCTRL sts=none int=none"
    );
    // Settings: VB_UV 3.4 V / 50 us, VB_LO 2.8 V shutting down, abnormal
    // detect disabled, OV shutdown on, UV lock on, VCCA only.
    let p = rk806::Power::from_state(&state(&[(0x5E, 0xf0), (0x5F, 0x88), (0x61, 0xb0)]));
    assert_eq!((p.vb_uv_mv(), p.vb_lo_mv()), (3400, 2800));
    assert_eq!(
        p.cfg().to_string(),
        "vb_uv=3400mV/50us vb_lo=2800mV/off abnordet=off ov_sd=on uvlock=on vccxdet=3"
    );
}

/// The new state registers are read (and only read) by readback().
#[test]
fn readback_reads_the_power_registers() {
    let mut m = Model::new();
    for (a, v) in [
        (0x74u16, 0x20u8),
        (0x75, 0x02),
        (0x5E, 0x0c),
        (0x77, 0x80),
        (0x79, 0x08),
    ] {
        m.pmic.insert(a, v);
    }
    let r = rk806::readback(&mut m).unwrap();
    assert_eq!(
        r.power().to_string(),
        "on=ON_ABNORMAL off=OFF_ABNORMAL sts=none int=VB_LO"
    );
    assert_eq!(r.power().vb_lo_mv(), 3200);
    assert_eq!(r.state[rk806::STATE_REGS.len() - 1], 0x08); // INT_STS1
    assert!(m.wire.iter().all(|f| f[0] & 0x80 == 0));
}

/// The carrier probe only reads, and skips GPIO3 / USB2PHY0_GRF when their
/// pclk is gated.
#[test]
fn carrier_probe_reads_only() {
    use rk3588::regs::{CRU_GATE_CON17, CRU_GATE_CON72, GPIO3, USB2PHY0_GRF};
    for gated in [false, true] {
        let mut rec = Recorder {
            responder: Some(Box::new(move |a, _| match a as usize {
                x if x == GPIO0 + 0x70 => Some(1 << 24),
                x if x == GPIO3 + 0x70 => Some(1 << 19),
                x if x == USB2PHY0_GRF + 0xC0 => Some(1 << 11),
                x if x == CRU_GATE_CON17 || x == CRU_GATE_CON72 => {
                    Some(if gated { 0xffff } else { 0 })
                }
                _ => None,
            })),
            ..Recorder::default()
        };
        let p = rk806::carrier_probe(&mut rec);
        assert!(writes(&rec).is_empty());
        assert!(p.present());
        assert_eq!((p.scl, p.sda), (1, 0));
        if gated {
            assert_eq!((p.mdio, p.sessvld), (None, None));
            assert!(!rec
                .evs
                .iter()
                .any(|e| matches!(e, Ev::R(_, a, _) if *a as usize == GPIO3 + 0x70)));
        } else {
            assert_eq!((p.mdio, p.sessvld), (Some(1), Some(1)));
        }
    }
    let dead = rk806::CarrierProbe {
        mdio: Some(0),
        sessvld: Some(0),
        ..Default::default()
    };
    assert_eq!(
        dead.to_string(),
        "i2c6 scl=0 sda=0 gmac1 mdio=0 u2phy0 sessvld=0 -> carrier 3.3 V ABSENT"
    );
}
