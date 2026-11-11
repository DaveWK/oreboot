// SPDX-License-Identifier: GPL-2.0-only
//! The crate against the real K1 dispatcher on its host model
//! (tests/legacy/k1/): random command scripts and transport faults that stay
//! off storage, run once through the K1 `fastboot::run` and once through
//! `fastboot::serve` with a K1-like Device over the same scripted USB. The
//! replies, commands consumed, disconnects and result must match.
#![allow(dead_code, unused_imports, unused_assignments, clippy::all)]
include!("legacy/k1/model.rs");
#[path = "legacy/k1/fastboot.rs"]
mod legacy_fastboot;
#[path = "legacy/k1/mmc.rs"]
mod mmc;
#[path = "legacy/k1/sd_capacity.rs"]
mod sd_capacity;
#[path = "legacy/k1/sparse.rs"]
mod sparse;

use core::fmt::Write as _;

struct K1Usb<'a>(usb::Usb<'a>);
impl fastboot::Transport for K1Usb<'_> {
    type Error = usb::Error;
    fn wait_configured(&mut self) -> Result<(), usb::Error> {
        self.0.wait_configured()
    }
    fn recv(&mut self, out: &mut [u8]) -> Result<usize, usb::Error> {
        self.0.recv(out)
    }
    fn recv_exact(&mut self, out: &mut [u8]) -> Result<(), usb::Error> {
        self.0.recv_into(out.as_ptr() as usize, out.len())
    }
    fn send(&mut self, data: &[u8]) -> Result<(), usb::Error> {
        self.0.send(data)
    }
    fn disconnect(&mut self) -> Result<(), usb::Error> {
        self.0.disconnect()
    }
    fn is_reset(e: &usb::Error) -> bool {
        matches!(e, usb::Error::Reset)
    }
}

/// The K1 board's answers (fastboot.rs `getvar`/`target`), storage excluded.
struct K1Device;
impl fastboot::Device for K1Device {
    fn getvar(&mut self, name: &str, out: &mut fastboot::Line) -> bool {
        const TARGETS: [&str; 7] = ["emmc", "user", "mmc0", "boot0", "boot1", "sd", "disk"];
        match name {
            "product" => write!(out, "host-model").is_ok(),
            "serialno" => write!(out, "k1").is_ok(),
            _ => match name.strip_prefix("partition-type:") {
                Some(p) if TARGETS.contains(&p) => write!(out, "raw").is_ok(),
                _ => false,
            },
        }
    }
    fn flash(&mut self, target: &str, _len: usize, _padded: &mut [u8]) -> Result<(), &'static str> {
        match target {
            "emmc" | "user" | "mmc0" | "boot0" | "boot1" | "sd" | "disk" => {
                unreachable!("scripts never flash a real target")
            }
            _ => Err("unknown target"),
        }
    }
}

const POOL: [&str; 27] = [
    "getvar:version",
    "getvar:product",
    "getvar:serialno",
    "getvar:max-download-size",
    "getvar:unlocked",
    "getvar:secure",
    "getvar:is-userspace",
    "getvar:partition-type:emmc",
    "getvar:partition-type:sd",
    "getvar:partition-type:nope",
    "getvar:has-slot:boot",
    "getvar:is-logical:system",
    "getvar:foo",
    "download:00000020",
    "download:0",
    "download:zz",
    "download:0x10",
    "download:20000001",
    "download:20000000",
    "flash:nope",
    "flash:",
    "reboot",
    "reboot-bootloader",
    "rebootx",
    "anything",
    "erase:emmc",
    "",
];

#[test]
fn random_scripts_match_the_k1_dispatcher() {
    let mut x: u64 = 0xfa57_b007_0000_0001;
    let mut rnd = |n: u64| {
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        x % n
    };
    let mut buf = vec![0u8; 0x2000_0000];
    for round in 0..3000 {
        let mut cmds: Vec<&'static str> = Vec::new();
        let mut downloaded = false;
        for _ in 0..rnd(8) {
            let c = POOL[rnd(POOL.len() as u64) as usize];
            downloaded |= c.starts_with("download:")
                && !["download:0", "download:zz", "download:20000001"].contains(&c);
            // flash with nothing downloaded never reaches storage in either.
            if c == "flash:emmc" && downloaded {
                continue;
            }
            cmds.push(c);
        }
        if rnd(4) != 0 {
            cmds.push(if downloaded {
                "continue"
            } else {
                ["flash:emmc", "continue"][rnd(2) as usize]
            });
        }
        cmds.push("continue");
        let faults = rnd(16);
        let set = |s: &mut Script| {
            s.ack_fail = faults == 1;
            s.flush_fail = faults == 2;
            s.wait_fail = faults == 3;
            s.recv_fail = faults == 4;
            s.download_fail = faults == 5;
        };
        reset(cmds.clone());
        edit(set);
        let board = legacy_fastboot::Board {
            name: "host-model",
            disk: mmc::Kind::Sd,
        };
        let old = legacy_fastboot::run(&board).map_err(|e| format!("{e:?}"));
        let old_state = view(|s| (s.replies.clone(), s.received, s.disconnected));

        reset(cmds.clone());
        edit(set);
        let mut t = K1Usb(
            usb::Usb::init(usb::Descriptors {
                vendor: 0x18d1,
                product: 0x4ee0,
                class: [0xff, 0x42, 0x03],
                strings: ["oreboot", "host-model", "k1", "fastboot"],
            })
            .unwrap(),
        );
        let new = fastboot::serve(
            &mut t,
            &mut K1Device,
            &fastboot::Config { unlocked: true },
            &mut buf,
        )
        .map_err(|e| format!("{e:?}"));
        let new_state = view(|s| (s.replies.clone(), s.received, s.disconnected));
        assert_eq!(new, old, "round {round}: {cmds:?} faults {faults}");
        assert_eq!(
            new_state, old_state,
            "round {round}: {cmds:?} faults {faults}"
        );
    }
}
