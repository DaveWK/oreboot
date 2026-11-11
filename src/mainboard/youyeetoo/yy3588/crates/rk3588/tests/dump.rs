// SPDX-License-Identifier: GPL-2.0-only
//! dram-info report over a register model (crate::dump).
mod common;
use common::Model;
use rk3588::dump::{self, Record};
use rk3588::osreg::{self, Geometry};
use rk3588::regs::{base, ctl, OS_REG2, OS_REG3, OS_REG4, OS_REG5};

fn model() -> Model {
    let mut m = Model::default();
    let g = Geometry {
        rank: 1,
        col: 10,
        bk: 3,
        cs0_row: 16,
        cs1_row: 16,
        bw: 1,
        dbw: 1,
    };
    let (a, b) = osreg::encode(osreg::LPDDR4X, &[g, g]);
    for (r, v) in [(OS_REG2, a), (OS_REG3, b), (OS_REG4, a), (OS_REG5, b)] {
        m.regs.insert(r, v);
    }
    // The DDRCTL answers MRR: mrr_done set, data = 0x10 * ch + MA, MR5 = 0xff.
    m.responder = Some(Box::new(|regs, addr| {
        for ch in 0..4 {
            let b = base::DDRCTL[ch];
            if addr == b + ctl::MRSTAT {
                return Some(ctl::MRSTAT_MRR_DONE);
            }
            if addr == b + ctl::MRRDATA0 {
                let ma = (regs.get(&(b + ctl::MRCTRL1)).copied().unwrap_or(0) >> 8) as u8;
                return Some(if ma == 5 {
                    0xff
                } else {
                    0x10 * ch as u32 + ma as u32
                });
            }
        }
        None
    }));
    m
}

#[test]
fn report_text_and_record() {
    let mut m = model();
    let mut buf = vec![0u8; dump::RECORD_BYTES];
    let mut rec = Record::new(&mut buf);
    let mut text = String::new();
    dump::report(&mut m, &mut text, &mut rec, 4, [1; 4]).unwrap();
    let n = rec.len();
    assert!(text.contains("dram-info pair=0 osreg version=3 type=LPDDR4X channels=2 size_mib=2048"));
    assert!(text.contains("dram-info mr ch=2 rank=0 mr0=0x20 mr5=0xff mr6=0x26"));
    assert!(text
        .lines()
        .last()
        .unwrap()
        .starts_with("dram-info end entries="));
    for line in text.lines() {
        assert!(line.starts_with("dram-info "), "{line}");
        let kind = line.split_whitespace().nth(1).unwrap();
        assert!(
            ["osreg", "pair=0", "pair=1", "mr", "ctl", "phy", "end"].contains(&kind)
                || kind.starts_with("raw="),
            "{line}"
        );
    }
    // The record holds the same MRR values and the OS_REG words.
    let entries: Vec<_> = dump::parse(&buf).unwrap().collect();
    assert_eq!(entries.len(), n);
    assert!(entries.contains(&(dump::mr_key(1, 0, 8), 0x18)));
    assert!(entries.contains(&(OS_REG2 as u32, m.regs[&OS_REG2])));
    // 4 OS_REG + 4 channels x (7 MR + ctl + phy) entries.
    let per_ch = 7 + dump::ctl_regs().count() + dump::phy_regs().count();
    assert_eq!(n, 4 + 4 * per_ch);
    assert!(16 + 8 * n <= dump::RECORD_BYTES);
}

#[test]
fn record_rejects_garbage() {
    assert!(dump::parse(&[0u8; 64]).is_none());
}
