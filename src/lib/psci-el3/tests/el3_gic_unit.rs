// SPDX-License-Identifier: GPL-2.0-only
//! Host tests of `el3` and `gicv3` beyond the board's behaviour: bounded
//! polls, another GIC layout, the errata data.

use psci_el3::el3::{self, El3Regs, PatchSlot, Trap, ERRATA};
use psci_el3::gicv3::{self, Gic, GicError, IccEl3};
use regio::mock::Mock;

struct NoIcc(Vec<(u8, u64)>);
impl IccEl3 for NoIcc {
    fn write_sre(&mut self, v: u64) {
        self.0.push((0, v))
    }
    fn write_ctlr(&mut self, v: u64) {
        self.0.push((1, v))
    }
    fn write_igrpen1(&mut self, v: u64) {
        self.0.push((2, v))
    }
}

const G4: Gic = Gic {
    gicd: 0x1000_0000,
    gicr: 0x1100_0000,
    gicr_stride: 0x2_0000,
    max_cpus: 4,
};

#[test]
fn a_stuck_distributor_times_out_instead_of_hanging() {
    fn rwp(_: &mut [(usize, u32)], addr: usize) -> Option<u32> {
        (addr == 0x1000_0000).then_some(gicv3::gicd::CTLR_RWP)
    }
    let mut io: Mock<64> = Mock::default();
    io.responder = Some(rwp);
    assert_eq!(
        gicv3::init_distributor(&mut io, &G4),
        Err(GicError::Timeout("GICD_CTLR.RWP"))
    );
}

#[test]
fn another_layout_finds_the_redistributor_by_scanning() {
    // Redistributor 2 reports affinity 0.0.1.0 (core 1's MPIDR); it is also
    // the last one. Unwritten PWRR reads 0: powered and settled.
    fn typer(_: &mut [(usize, u32)], addr: usize) -> Option<u32> {
        let base = 0x1100_0000;
        if addr < base || addr >= base + 4 * 0x2_0000 {
            return None;
        }
        let (i, off) = ((addr - base) / 0x2_0000, (addr - base) % 0x2_0000);
        match off {
            0x8 => Some(if i == 2 { 1 << 4 } else { 0 }),
            0xc => Some(if i == 2 { 0x100 } else { 0x1_0000 + i as u32 }),
            _ => None,
        }
    }
    let mut io: Mock<256> = Mock::default();
    io.responder = Some(typer);
    let mut icc = NoIcc(Vec::new());
    let rd = gicv3::init_cpu(&mut io, &mut icc, &G4, 0x100, 1).unwrap();
    assert_eq!(rd, 0x1100_0000 + 2 * 0x2_0000);
    assert_eq!(icc.0, vec![(0, 0xf), (1, 0), (2, 0x3)]);
    // No redistributor for Aff1 = 3.
    assert_eq!(
        gicv3::init_cpu(&mut io, &mut icc, &G4, 0x300, 3),
        Err(GicError::NoRedistributor { mpidr: 0x300 })
    );
    gicv3::cpu_off(&mut io, &mut icc, rd);
    assert_eq!(icc.0.last(), Some(&(2, 0)));
    assert_ne!(
        io.get(rd + gicv3::gicr::WAKER) & gicv3::gicr::WAKER_PROCESSOR_SLEEP,
        0
    );
}

struct Rec {
    midr: u64,
    log: Vec<String>,
}
impl El3Regs for Rec {
    fn midr(&mut self) -> u64 {
        self.midr
    }
    fn patch_slot(&mut self, p: &PatchSlot) {
        self.log.push(format!("patch{}", p.slot))
    }
    fn isb(&mut self) {
        self.log.push("isb".into())
    }
    fn write_scr(&mut self, v: u64) {
        self.log.push(format!("scr {v:#x}"))
    }
    fn write_cptr(&mut self, v: u64) {
        self.log.push(format!("cptr {v:#x}"))
    }
    fn write_mdcr(&mut self, v: u64) {
        self.log.push(format!("mdcr {v:#x}"))
    }
    fn write_cntfrq(&mut self, v: u64) {
        self.log.push(format!("cntfrq {v}"))
    }
}

#[test]
fn errata_apply_to_the_a76_r4p1_only_where_listed() {
    let a76_r4p1 = 0x41 << 24 | 4 << 20 | 0xf << 16 | 0xd0b << 4 | 1;
    let mut r = Rec {
        midr: a76_r4p1,
        log: Vec::new(),
    };
    el3::el3_cpu_init(&mut r, 24_000_000);
    assert_eq!(
        r.log,
        [
            "patch3",
            "patch4",
            "patch5",
            "isb",
            "scr 0x731",
            "cptr 0x0",
            "mdcr 0x0",
            "cntfrq 24000000",
            "isb"
        ]
    );
    let a55_r2p0 = 0x41 << 24 | 2 << 20 | 0xf << 16 | 0xd05 << 4;
    let mut r = Rec {
        midr: a55_r2p0,
        log: Vec::new(),
    };
    el3::el3_cpu_init(&mut r, 1);
    assert_eq!(r.log[0], "scr 0x731");
    assert!(ERRATA.iter().all(|e| e.rev_min <= e.rev_max));
    assert_eq!(el3::classify(0x17 << 26), Trap::Smc);
    assert_eq!(el3::classify(0x16 << 26), Trap::Other(0x16));
}
