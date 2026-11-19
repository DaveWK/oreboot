// SPDX-License-Identifier: GPL-2.0-only
//! What `tables::DFS_TABLE` says: the command list the vendor init leaves
//! for the controller block's frequency-change sequencer, read as the
//! frequency-change procedure of the DesignWare controller (EIC7700X TRM
//! Part 2, "Change Clock Frequencies with Frequency Set Point"). Nothing
//! in the boot runs the list; these tests only fix its reading, which is
//! what a software frequency change would have to do on the controller
//! side.
//!
//! Layout as read here (REFERENCE-DERIVED, not documented): one command
//! per 16 bytes from 0x0a021800: word 0 = [15:0] the controller register
//! (0x8000 + its offset in the main block), [17:16] 3 write / 2 poll,
//! [27:24] flags (8: the value comes from the per-pstate table [23:20];
//! 4: a delay word follows at +0xc; 0xc: the clock change, with table 3;
//! 0x10: on DFIMISC's dfi_frequency field), bit 31 the last command; word
//! 1 the value; word 2 the bits kept. Four per-pstate tables of four words
//! from 0x0a021d00.

extern crate std;
use super::tables::DFS_TABLE;
use std::vec::Vec;

const LIST: u32 = 0x0a02_1800;
const TABLES: u32 = 0x0a02_1d00;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Cmd {
    reg: u32,
    op: u32,
    flags: u32,
    table: u32,
    value: u32,
    keep: u32,
}

fn word(addr: u32) -> Option<u32> {
    DFS_TABLE.iter().find(|e| e.0 == addr).map(|e| e.1)
}

fn commands() -> Vec<Cmd> {
    let mut out = Vec::new();
    let mut a = LIST;
    while let Some(w0) = word(a) {
        out.push(Cmd {
            reg: w0 & 0x7fff,
            op: w0 >> 16 & 3,
            flags: w0 >> 24,
            table: w0 >> 20 & 0xf,
            value: word(a + 4).unwrap_or(0),
            keep: word(a + 8).unwrap_or(0),
        });
        a += 0x10;
    }
    out
}

fn table(n: u32) -> [u32; 4] {
    [0, 1, 2, 3].map(|ps| word(TABLES + 0x10 * n + 4 * ps).unwrap())
}

// Controller registers (offsets in the main block).
const MSTR2: u32 = 0x008;
const STAT: u32 = 0x014;
const MRCTRL0: u32 = 0x080;
const MRCTRL1: u32 = 0x084;
const PWRCTL: u32 = 0x180;
const RFSHCTL0: u32 = 0x208;
const DFIMISC: u32 = 0x510;
const DFISTAT: u32 = 0x514;
const SWCTL: u32 = 0xc80;
const WRITE: u32 = 3;
const POLL: u32 = 2;

/// Index of the first command at or after `from` that matches.
fn find(c: &[Cmd], from: usize, f: impl Fn(&Cmd) -> bool) -> usize {
    from + c[from..]
        .iter()
        .position(f)
        .expect("command not in the list")
}

#[test]
fn the_list_is_the_documented_frequency_change() {
    let c = commands();
    assert_eq!(c.len(), 79);
    assert_eq!(c.iter().filter(|x| x.flags & 0x80 != 0).count(), 1);
    assert!(c[c.len() - 1].flags & 0x80 != 0);
    let set = |reg, bits| {
        move |x: &Cmd| {
            x.reg == reg
                && x.op == WRITE
                && x.value & bits == bits
                && x.keep & bits == 0
                && bits != 0
        }
    };
    let clear = |reg, bits| {
        move |x: &Cmd| x.reg == reg && x.op == WRITE && x.value & bits == 0 && x.keep & bits == 0
    };
    let poll = |reg, bits, want| {
        move |x: &Cmd| {
            x.reg == reg && x.op == POLL && !x.keep & bits == bits && x.value & bits == want
        }
    };
    // Low-power features off, then normal mode read three times.
    assert!(clear(PWRCTL, 0xff)(&c[0]));
    let normal = find(&c, 0, poll(STAT, 7, 1));
    assert!(poll(STAT, 7, 1)(&c[normal + 1]) && poll(STAT, 7, 1)(&c[normal + 2]));
    // Self-refresh by software, staying in self-refresh.
    let sr = find(&c, normal, set(PWRCTL, 0x8800));
    let in_sr = find(&c, sr, poll(STAT, 0x7000, 0x1000));
    // One mode-register write to every rank, its data from table 0.
    let mrw_data = find(&c, in_sr, |x| {
        x.reg == MRCTRL1 && x.flags & 8 != 0 && x.table == 0
    });
    let mrw = find(&c, mrw_data, set(MRCTRL0, 1 << 31));
    // dfi_init_complete_en off; then, with sw_done low, the frequency
    // set-point (table 1 into DFIMISC [15:14]), the frequency set (table 2
    // into MSTR2 [4:0]) and dfi_init_start.
    let complete_en_off = find(&c, mrw, clear(DFIMISC, 1));
    let fsp = find(&c, complete_en_off, |x| {
        x.reg == DFIMISC && x.flags & 8 != 0 && x.table == 1
    });
    assert_eq!(!c[fsp].keep, 0xc000);
    let set_sel = find(&c, fsp, |x| {
        x.reg == MSTR2 && x.flags & 8 != 0 && x.table == 2
    });
    assert_eq!(!c[set_sel].keep, 0x1f);
    let start = find(&c, set_sel, set(DFIMISC, 0x20));
    assert!(c[fsp - 1].reg == SWCTL && c[fsp - 1].value == 0);
    assert!(c[start + 1].reg == SWCTL && c[start + 1].value == 1);
    // The PHY drops dfi_init_complete; only then the clock changes (table
    // 3) and the refresh timing is reloaded.
    let dropped = find(&c, start, poll(DFISTAT, 1, 0));
    let clock = find(&c, dropped, |x| x.flags & 0xc == 0xc && x.table == 3);
    let refresh = find(&c, clock, |x| x.reg == RFSHCTL0 && x.keep == !0x10);
    // dfi_init_start off, the PHY raises dfi_init_complete again,
    // dfi_init_complete_en back on.
    let stop = find(&c, refresh, clear(DFIMISC, 0x20));
    let complete = find(&c, stop, poll(DFISTAT, 1, 1));
    let complete_en = find(&c, complete, set(DFIMISC, 1));
    // Three more mode-register writes, then out of self-refresh.
    let later_mrws = c[complete_en..]
        .iter()
        .filter(|x| set(MRCTRL0, 1 << 31)(x))
        .count();
    assert_eq!(later_mrws, 3);
    let srx = find(&c, complete_en, clear(PWRCTL, 0x800));
    let out_of_sr = find(&c, srx, poll(STAT, 0x7000, 0));
    assert!(out_of_sr < c.len());
    // Exactly one dfi_init_start pulse and one clock change in the list.
    assert_eq!(c.iter().filter(|x| set(DFIMISC, 0x20)(x)).count(), 1);
    assert_eq!(c.iter().filter(|x| x.flags & 0xc == 0xc).count(), 1);
}

#[test]
fn the_per_pstate_tables() {
    // Table 0, the mode-register write before the change: MR16 = 0x01
    // (FSP-WR 1, FSP-OP 0) whatever the target pstate.
    assert_eq!(table(0), [0x1001; 4]);
    // Table 1: dfi_freq_fsp 1 for every pstate (one DRAM set-point serves
    // all four clocks).
    assert_eq!(table(1), [1 << 14; 4]);
    // Table 2: the controller's frequency set is the pstate.
    assert_eq!(table(2), [0, 1, 2, 3]);
    // Table 3, the clock words: [23:16] is 8 x the pstate's divider of the
    // 4 GB parameter sets (3, 7, 11, 23), as `post::dfs_table` writes it.
    assert_eq!(
        table(3).map(|w| w >> 16 & 0xff),
        [3, 7, 11, 23].map(|d| 8 * d)
    );
    // The mode-register writes after the change, as the list holds them:
    // MR0 = 0 (a read-only register: no effect), MR11 = 0x12, MR0 = 0.
    let c = commands();
    let data: Vec<u32> = c
        .iter()
        .filter(|x| x.reg == MRCTRL1 && x.flags & 8 == 0)
        .map(|x| x.value)
        .collect();
    assert_eq!(data, [0x0000, 0x0b12, 0x0000]);
}
