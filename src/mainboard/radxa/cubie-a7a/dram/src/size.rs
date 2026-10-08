//! DRAM size from the geometry, and the quick check after init.

use crate::hw::{Error, Hw};
use crate::para::Para;

/// Total size in MiB: one rank of one channel has 2^(row + col + bank +
/// bank group) 16-bit words; times ranks and channels; three quarters of
/// that for the non-binary densities (6/12 GB) unless para2[31:30] says
/// otherwise.
pub fn size_mib(p: &Para) -> u32 {
    let bits = p.row_bits() + p.bank_bits() + p.col_bits() + p.bg_bits() + p.rank_bits();
    let half_width = p.para2() & 0xf != 0;
    let shift = if !half_width || p.channels() == 2 {
        bits - 18
    } else {
        bits - 19
    };
    let mut mib = 1u32 << shift;
    if p.tpr13() & (7 << 16) != 0 && p.para2() >> 30 != 2 {
        mib = (mib * 3) >> 2;
    }
    mib
}

const DRAM_BASE: usize = 0x4000_0000;

/// Write `words` incrementing patterns at the start of DRAM and at its
/// middle (`mib / 2`), read both back. Catches a dead or swapped channel,
/// rank or high address line, not marginal timing.
pub fn quick_test<H: Hw>(hw: &mut H, mib: u32, words: usize) -> Result<(), Error> {
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
