// SPDX-License-Identifier: GPL-2.0-only
//! Equivalence of `ci-udc` (+ `usb-device`) with the SpacemiT K1 bt0 USB
//! driver it was moved from (`tests/legacy/k1_usb.rs`, verbatim). Both run
//! every scenario of the K1 host model (tests/usb-model.rs of
//! ore-edk-boot-opi) plus a full enumeration through injected SETUP packets
//! against one model; the register access trace (hash and count), the
//! model's counters, what ep0 sent and every result must be the same. The
//! crate runs with the K1 platform below (the K1 board's side).

extern crate self as util;

mod model;
use model::{change, reset, state, Transfer};

pub mod mmio {
    pub fn read32(a: usize) -> u32 {
        super::model::read32(a)
    }
    pub fn write32(a: usize, v: u32) {
        super::model::write32(a, v)
    }
}
mod cache {
    pub fn flush(a: usize, n: usize) {
        super::model::flush(a, n)
    }
    pub fn invalidate(a: usize, n: usize) {
        super::model::invalidate(a, n)
    }
}
#[allow(dead_code, unused, static_mut_refs, clippy::all)]
#[path = "legacy/k1_usb.rs"]
mod legacy;

use ci_udc::{Config, Descriptors, Dma, PhyTimeout, Udc, UdcPlatform};
use regio::RegIo;
use std::sync::Mutex;

/// The legacy driver keeps its DMA area in one static: one scenario at a time.
static SERIAL: Mutex<()> = Mutex::new(());

struct Io;
impl RegIo for Io {
    fn read32(&mut self, a: usize) -> u32 {
        model::read32(a)
    }
    fn write32(&mut self, a: usize, v: u32) {
        model::write32(a, v)
    }
    fn read16(&mut self, a: usize) -> u16 {
        model::read32(a) as u16
    }
    fn write16(&mut self, a: usize, v: u16) {
        model::write32(a, v as u32)
    }
    fn udelay(&mut self, _: u32) {}
    fn now_us(&mut self) -> u64 {
        0
    }
    fn log(&mut self, _: core::fmt::Arguments) {}
}

/// The K1 board's side: APMU clock/reset, the UTMI PHY, cache maintenance.
struct K1;
const APMU_USB: usize = model::APMU_USB;
const PHY_REG01: usize = 0xc094_0004;
const PHY_REG0D: usize = 0xc094_0034;
impl UdcPlatform for K1 {
    fn bring_up<IO: RegIo>(&mut self, io: &mut IO) -> Result<(), PhyTimeout> {
        let v = io.read32(APMU_USB);
        io.write32(APMU_USB, v | 2);
        io.write32(APMU_USB, (v | 2) & !1);
        io.write32(APMU_USB, v | 2 | 1);
        self.spin(200_000);
        let mut ready = false;
        for _ in 0..1_000_000 {
            if io.read32(PHY_REG01) & 1 != 0 {
                ready = true;
                break;
            }
        }
        if !ready {
            return Err(PhyTimeout);
        }
        io.write32(PHY_REG01, 0x60ef);
        io.write32(PHY_REG0D, 0x1c);
        Ok(())
    }
    fn flush_dcache(&mut self, a: usize, n: usize) {
        model::flush(a, n)
    }
    fn invalidate_dcache(&mut self, a: usize, n: usize) {
        model::invalidate(a, n)
    }
    fn spin(&mut self, _: usize) {}
}

const DESC: Descriptors = Descriptors {
    vendor: 0x18d1,
    product: 0x4ee0,
    class: [0xff, 0x42, 3],
    strings: ["OrangePi", "R2S", "0123456789", "fastboot"],
};

/// The operations both drivers offer.
trait Dev {
    fn send(&mut self, d: &[u8]) -> String;
    fn recv(&mut self, n: usize) -> String;
    fn recv_into(&mut self, len: usize) -> String;
    fn wait_configured(&mut self) -> String;
    fn poll(&mut self) -> String;
    fn disconnect(&mut self) -> String;
    fn configured(&self) -> bool;
}

fn buffer(len: usize) -> usize {
    let v = vec![0u8; len + 4096].leak();
    (v.as_mut_ptr() as usize + 4095) & !4095
}

impl Dev for legacy::Usb<'static> {
    fn send(&mut self, d: &[u8]) -> String {
        format!("{:?}", legacy::Usb::send(self, d))
    }
    fn recv(&mut self, n: usize) -> String {
        let mut b = vec![0u8; n];
        // Only the result: the model delivers no OUT data, and the K1
        // driver's static buffer keeps bytes from earlier scenarios.
        format!("{:?}", legacy::Usb::recv(self, &mut b))
    }
    fn recv_into(&mut self, len: usize) -> String {
        format!("{:?}", legacy::Usb::recv_into(self, buffer(len), len))
    }
    fn wait_configured(&mut self) -> String {
        format!("{:?}", legacy::Usb::wait_configured(self))
    }
    fn poll(&mut self) -> String {
        format!("{:?}", legacy::Usb::poll(self))
    }
    fn disconnect(&mut self) -> String {
        format!("{:?}", legacy::Usb::disconnect(self))
    }
    fn configured(&self) -> bool {
        legacy::Usb::configured(self)
    }
}

impl Dev for Udc<'static, Io, K1> {
    fn send(&mut self, d: &[u8]) -> String {
        format!("{:?}", Udc::send(self, d))
    }
    fn recv(&mut self, n: usize) -> String {
        let mut b = vec![0u8; n];
        // Only the result: the model delivers no OUT data, and the K1
        // driver's static buffer keeps bytes from earlier scenarios.
        format!("{:?}", Udc::recv(self, &mut b))
    }
    fn recv_into(&mut self, len: usize) -> String {
        format!("{:?}", Udc::recv_into(self, buffer(len), len))
    }
    fn wait_configured(&mut self) -> String {
        format!("{:?}", Udc::wait_configured(self))
    }
    fn poll(&mut self) -> String {
        format!("{:?}", Udc::poll(self))
    }
    fn disconnect(&mut self) -> String {
        format!("{:?}", Udc::disconnect(self))
    }
    fn configured(&self) -> bool {
        Udc::configured(self)
    }
}

type Outcome = (Vec<String>, u64, u64, [usize; 9], Vec<Vec<u8>>);

fn outcome(log: Vec<String>) -> Outcome {
    state(|m| {
        (
            log,
            m.hash,
            m.accesses,
            [
                m.phy_reads,
                m.reset_reads,
                m.flush_reads,
                m.flush_writes,
                m.list_writes,
                m.run_writes,
                m.prime_writes,
                m.cache_writes,
                m.invalidations,
            ],
            m.ep0_log.clone(),
        )
    })
}

/// Run `scenario` on both drivers after `setup`, and compare.
fn compare(setup: impl Fn(), scenario: impl Fn(&mut dyn Dev, &mut Vec<String>)) -> Outcome {
    let _g = SERIAL.lock().unwrap_or_else(|e| e.into_inner());
    let run = |init: &dyn Fn() -> Result<Box<dyn Dev>, String>| {
        reset();
        setup();
        let mut log = Vec::new();
        match init() {
            Err(e) => log.push(e),
            Ok(mut d) => {
                log.push("init ok".into());
                scenario(d.as_mut(), &mut log);
                log.push(format!("configured {}", d.configured()));
            }
        }
        outcome(log)
    };
    let old = run(&|| {
        legacy::Usb::init(legacy::Descriptors {
            vendor: DESC.vendor,
            product: DESC.product,
            class: DESC.class,
            strings: DESC.strings,
        })
        .map(|u| Box::new(u) as Box<dyn Dev>)
        .map_err(|e| format!("{e:?}"))
    });
    let new = run(&|| {
        let dma: &'static mut Dma = Box::leak(Box::new(Dma::new()));
        Udc::init(Io, K1, Config::new(0xc090_0000), dma, DESC)
            .map(|u| Box::new(u) as Box<dyn Dev>)
            .map_err(|e| format!("{e:?}"))
    });
    assert_eq!(old.0, new.0, "results differ");
    assert_eq!(old.3, new.3, "model counters differ");
    assert_eq!(old.4, new.4, "ep0 data differs");
    assert_eq!((old.1, old.2), (new.1, new.2), "register traces differ");
    assert!(old.2 > 0);
    old
}

fn transfer(t: Transfer) {
    change(|m| {
        m.transfer = t;
        m.tds.clear();
        m.invalidations = 0
    })
}

#[test]
fn init_phy_timeout() {
    compare(|| change(|m| m.phy_at = usize::MAX), |_, _| {});
}

#[test]
fn init_reset_timeout() {
    compare(|| change(|m| m.reset_at = usize::MAX), |_, _| {});
}

#[test]
fn init_last_allowed_phy_and_reset_read() {
    compare(
        || {
            change(|m| {
                m.phy_at = 1_000_000;
                m.reset_at = 10_000_000
            })
        },
        |_, _| {},
    );
}

#[test]
fn init_flush_timeout_and_stuck_status() {
    compare(|| change(|m| m.flush_stuck = true), |_, _| {});
    compare(|| change(|m| m.stat_stuck = true), |_, _| {});
}

#[test]
fn send_completes_then_active_send_timeout_is_terminal() {
    compare(
        || {},
        |u, log| {
            transfer(Transfer::Complete);
            log.push(u.send(b"OKAY"));
            transfer(Transfer::Never);
            log.push(u.send(b"OKAY"));
            log.push(u.send(b"again"));
            log.push(u.recv(64));
            log.push(u.recv_into(512));
            log.push(u.wait_configured());
        },
    );
}

#[test]
fn active_download_timeout_is_bounded() {
    compare(
        || {},
        |u, log| {
            transfer(Transfer::Never);
            log.push(u.recv_into(512));
        },
    );
}

#[test]
fn idle_command_wait_remains_host_driven() {
    compare(
        || {},
        |u, log| {
            transfer(Transfer::After(100_000_001));
            log.push(u.recv(64));
        },
    );
}

#[test]
fn short_out_paths() {
    compare(
        || {},
        |u, log| {
            transfer(Transfer::Short);
            change(|m| m.flush_stuck = true);
            log.push(u.recv_into(32768));
            log.push(u.send(b"again"));
        },
    );
    compare(
        || {},
        |u, log| {
            transfer(Transfer::Short);
            log.push(u.recv_into(32768));
        },
    );
}

#[test]
fn chain_error_and_large_download() {
    compare(
        || {},
        |u, log| {
            transfer(Transfer::Error);
            log.push(u.recv_into(32768));
        },
    );
    compare(
        || {},
        |u, log| {
            transfer(Transfer::Complete);
            // Two full descriptor chains and a tail.
            log.push(u.recv_into(2 * 64 * 16384 + 4096));
            log.push(u.recv(512));
        },
    );
}

#[test]
fn disconnect_and_bus_resets() {
    compare(
        || {},
        |u, log| {
            change(|m| m.flush_stuck = true);
            log.push(u.disconnect());
            log.push(u.send(b"again"));
        },
    );
    compare(
        || {},
        |u, log| {
            change(|m| m.status = 1 << 6);
            log.push(u.poll());
            transfer(Transfer::Complete);
            log.push(u.send(b"after reset"));
        },
    );
    compare(
        || {},
        |u, log| {
            change(|m| {
                m.status = 1 << 6;
                m.flush_stuck = true
            });
            log.push(u.poll());
            log.push(u.send(b"again"));
        },
    );
    compare(
        || {},
        |u, log| {
            change(|m| {
                m.setup = 1;
                m.flush_stuck = true
            });
            log.push(u.poll());
        },
    );
}

#[test]
fn active_in_final_allowed_completion() {
    compare(
        || {},
        |u, log| {
            transfer(Transfer::After(50_000_000));
            log.push(u.send(b"OKAY"));
        },
    );
}

fn setup(u: &mut dyn Dev, log: &mut Vec<String>, s: [u8; 8]) {
    change(|m| {
        m.setup = 1;
        m.setup_bytes = Some(s);
    });
    log.push(format!("{s:02x?} {}", u.poll()));
}

#[test]
fn enumeration_at_full_and_high_speed() {
    for portsc in [0u32, 2 << 26] {
        let out = compare(
            || {},
            |u, log| {
                change(|m| {
                    m.status = 1 << 2;
                    m.portsc = portsc
                });
                log.push(u.poll());
                for s in [
                    [0x80, 6, 0, 1, 0, 0, 64, 0],
                    [0x80, 6, 0, 1, 0, 0, 8, 0],
                    [0x80, 6, 0, 2, 0, 0, 255, 0],
                    [0x80, 6, 0, 2, 0, 0, 9, 0],
                    [0x80, 6, 0, 3, 0, 0, 255, 0],
                    [0x80, 6, 1, 3, 9, 4, 255, 0],
                    [0x80, 6, 2, 3, 9, 4, 255, 0],
                    [0x80, 6, 3, 3, 9, 4, 255, 0],
                    [0x80, 6, 4, 3, 9, 4, 255, 0],
                    [0x80, 6, 5, 3, 9, 4, 255, 0],
                    [0x80, 6, 0, 6, 0, 0, 10, 0],
                    [0x80, 6, 0, 7, 0, 0, 10, 0],
                    [0x00, 5, 9, 0, 0, 0, 0, 0],
                    [0x00, 9, 1, 0, 0, 0, 0, 0],
                    [0x80, 8, 0, 0, 0, 0, 1, 0],
                    [0x80, 0, 0, 0, 0, 0, 2, 0],
                    [0x81, 0, 0, 0, 0, 0, 2, 0],
                    [0x82, 0, 0, 0, 0x81, 0, 2, 0],
                    [0x02, 1, 0, 0, 0x81, 0, 0, 0],
                    [0x02, 1, 0, 0, 0x01, 0, 0, 0],
                    [0x02, 1, 0, 0, 0x82, 0, 0, 0],
                    [0x01, 11, 0, 0, 0, 0, 0, 0],
                    [0x21, 0x22, 0, 0, 0, 0, 0, 0],
                ] {
                    setup(u, log, s);
                }
                log.push(u.wait_configured());
                log.push(u.send(b"OKAY"));
                log.push(u.recv(64));
                setup(u, log, [0x00, 9, 0, 0, 0, 0, 0, 0]);
            },
        );
        // Device, two configurations, five strings, qualifier, configuration
        // and three status replies were sent (and compared above).
        // (The zero-length status stages are logged too.)
        let sent: Vec<_> = out.4.iter().filter(|b| !b.is_empty()).collect();
        assert_eq!(sent.len(), 14);
        assert_eq!(&sent[0][..8], &[18, 1, 0, 2, 0, 0, 0, 64]);
        let mps: u16 = if portsc == 0 { 64 } else { 512 };
        assert_eq!(&sent[2][22..24], &mps.to_le_bytes());
    }
}
