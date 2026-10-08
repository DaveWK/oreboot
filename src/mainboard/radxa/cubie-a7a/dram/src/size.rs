//! DRAM size from the geometry, and the quick check after init.

use crate::hw::{Error, Hw};
use crate::para::Para;

/// Total size in MiB: one rank of one channel has 2^(row + col + bank +
/// bank group) 16-bit words; times ranks and channels; three quarters of
/// that for the non-binary densities (6/12 GB) unless para2[31:30] says
/// otherwise.
pub fn checked_size_mib(p: &Para) -> Result<u32, Error> {
    let bits = p.row_bits() + p.bank_bits() + p.col_bits() + p.bg_bits() + p.rank_bits();
    let half_width = p.para2() & 0xf != 0;
    let overhead = if !half_width || p.channels() == 2 {
        18
    } else {
        19
    };
    let shift = bits
        .checked_sub(overhead)
        .ok_or(Error::Config("DRAM geometry below size representation"))?;
    let mut mib = 1u32
        .checked_shl(shift)
        .ok_or(Error::Config("DRAM geometry exceeds size arithmetic"))?;
    if p.tpr13() & (7 << 16) != 0 && p.para2() >> 30 != 2 {
        mib = mib
            .checked_mul(3)
            .ok_or(Error::Config("non-binary DRAM size arithmetic overflow"))?
            >> 2;
    }
    validate_mib(mib)
}

fn validate_mib(mib: u32) -> Result<u32, Error> {
    // para2's handoff size is 15 bits; the bounded smoke check needs at least
    // two MiB. These are software bounds, not detected physical capacity.
    if !(2..=0x7fff).contains(&mib) {
        return Err(Error::Config("DRAM size outside boot handoff field"));
    }
    Ok(mib)
}

/// Respect the explicit size flag or calculate the geometry, rejecting values
/// that cannot be handed off and checked safely before any hardware setup.
pub fn boot_size_mib(p: &Para) -> Result<u32, Error> {
    if p.para2() & (1 << 31) != 0 {
        validate_mib((p.para2() >> 16) & 0x7fff)
    } else {
        checked_size_mib(p)
    }
}

/// Compatibility for the reference harness's integer ABI. Invalid geometry
/// returns zero; the actual boot path uses the checked Result API above.
pub fn size_mib(p: &Para) -> u32 {
    checked_size_mib(p).unwrap_or(0)
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
