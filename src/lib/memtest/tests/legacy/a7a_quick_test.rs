// SPDX-License-Identifier: GPL-2.0-only
// Verbatim copy of ore-edk-boot-radxa-a7a oreboot 4904019
// src/mainboard/radxa/cubie-a7a/dram/src/size.rs from `const DRAM_BASE` to the
// end (quick_test), the oracle for the equivalence tests. Only the stubs
// marked "stub:" are not from the source.
#![allow(dead_code, clippy::all)]
// stub: the A733 Hw trait's two accessors quick_test uses, and its Error.
pub trait Hw {
    fn read32(&mut self, addr: usize) -> u32;
    fn write32(&mut self, addr: usize, val: u32);
}
#[derive(Debug, PartialEq, Eq)]
pub enum Error {
    Config(&'static str),
    Test {
        addr: usize,
        expected: u32,
        got: u32,
    },
}
const DRAM_BASE: usize = 0x4000_0000;
/// Existing boot smoke-test footprint per region. This bounds software work;
/// it is not full-RAM coverage or an independently measured timing margin.
pub const MAX_QUICK_WORDS: usize = 4096;

/// Write `words` incrementing patterns at the start of DRAM and at its
/// middle (`mib / 2`), read both back. Catches a dead or swapped channel,
/// rank or high address line, not marginal timing.
pub fn quick_test<H: Hw>(hw: &mut H, mib: u32, words: usize) -> Result<(), Error> {
    // The AArch32 boot consumer checks at most the first 4096 MiB, using two
    // disjoint <=16KiB regions at base and midpoint. Reject empty checks,
    // overlapping/invalid regions and oversized work before the first access.
    // These limits keep every address representable on the actual 32-bit CPU.
    if !(2..=4096).contains(&mib) || words == 0 || words > MAX_QUICK_WORDS {
        return Err(Error::Config(
            "quick-test size/word count outside bounded boot footprint",
        ));
    }
    let half = ((mib / 2) as usize) << 20;
    for i in 0..words {
        hw.write32(DRAM_BASE + 4 * i, 0x0123_4567u32.wrapping_add(i as u32));
        hw.write32(
            DRAM_BASE + half + 4 * i,
            0xfedc_ba98u32.wrapping_add(i as u32),
        );
    }
    for i in 0..words {
        for (base, pat) in [(DRAM_BASE + half, 0xfedc_ba98u32), (DRAM_BASE, 0x0123_4567)] {
            let addr = base + 4 * i;
            let expected = pat.wrapping_add(i as u32);
            let got = hw.read32(addr);
            if got != expected {
                return Err(Error::Test {
                    addr,
                    expected,
                    got,
                });
            }
        }
    }
    Ok(())
}
