//! DRAM size from the geometry, and the quick check after init.

use crate::hw::{Error, Hw};
use crate::para::Para;

/// Total size in MiB: one rank of one channel has 2^(row + col + bank +
/// bank group) 16-bit words; times ranks and channels; three quarters of
/// that for the non-binary densities (6/12 GB) unless para2[31:30] says
/// otherwise.
pub fn checked_size_mib(p: &Para) -> Result<u32, Error> {
    let g = jedec_lpddr::geometry::Geometry {
        row_bits: p.row_bits(),
        col_bits: p.col_bits(),
        bank_bits: p.bank_bits(),
        bg_bits: p.bg_bits(),
        rank_bits: p.rank_bits(),
    };
    // A 32-bit bus (two x16 channels, or a full-width one) unless a single
    // half-width channel is populated.
    let half_width = p.para2() & 0xf != 0;
    let bus_bytes_log2 = if !half_width || p.channels() == 2 {
        2
    } else {
        1
    };
    let bits = g.address_bits() + bus_bytes_log2;
    if bits < 20 {
        return Err(Error::Config("DRAM geometry below size representation"));
    }
    let mut mib = jedec_lpddr::geometry::size_mib(g.address_bits(), bus_bytes_log2)
        .ok_or(Error::Config("DRAM geometry exceeds size arithmetic"))?;
    if p.tpr13() & (7 << 16) != 0 && p.para2() >> 30 != 2 {
        mib = jedec_lpddr::geometry::non_binary_mib(mib)
            .ok_or(Error::Config("non-binary DRAM size arithmetic overflow"))?;
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
pub use memtest::MAX_QUICK_WORDS;

/// The boot smoke check (`memtest::quick`) at the A733's DRAM base: `words`
/// incrementing patterns at the start of DRAM and at its middle (`mib / 2`),
/// both read back. Catches a dead or swapped channel, rank or high address
/// line, not marginal timing. `mib` 2..=4096 (the AArch32 boot consumer's
/// addresses stay 32-bit), `words` 1..=[`MAX_QUICK_WORDS`].
pub fn quick_test<H: Hw>(hw: &mut H, mib: u32, words: usize) -> Result<(), Error> {
    memtest::quick(hw, DRAM_BASE, mib, words).map_err(|e| match e {
        memtest::QuickError::Config(m) => Error::Config(m),
        memtest::QuickError::Mismatch {
            addr,
            expected,
            got,
        } => Error::Test {
            addr,
            expected,
            got,
        },
    })
}
