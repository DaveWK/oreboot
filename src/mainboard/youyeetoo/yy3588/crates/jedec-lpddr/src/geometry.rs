// SPDX-License-Identifier: GPL-2.0-only
//! DRAM density from the address geometry.
//!
//! A rank of one channel holds 2^(row + column + bank + bank-group) words of
//! the channel's width. The total over ranks and channels is
//! 2^(row + col + bank + bg + rank) words of the whole bus. The board decides
//! the bus width it actually populates (one or two x16 channels, ...).

/// Address bits of one configuration (rank bits: log2 of the rank count).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Geometry {
    pub row_bits: u32,
    pub col_bits: u32,
    pub bank_bits: u32,
    pub bg_bits: u32,
    pub rank_bits: u32,
}

impl Geometry {
    pub const fn address_bits(self) -> u32 {
        self.row_bits + self.bank_bits + self.col_bits + self.bg_bits + self.rank_bits
    }
}

/// Total size in MiB of `address_bits` words of `bus_bytes_log2` (1: 16-bit
/// bus, 2: 32-bit bus, ...). None when it is below 1 MiB or overflows u32.
pub const fn size_mib(address_bits: u32, bus_bytes_log2: u32) -> Option<u32> {
    let bits = address_bits + bus_bytes_log2;
    match bits.checked_sub(20) {
        Some(shift) => 1u32.checked_shl(shift),
        None => None,
    }
}

/// Three quarters of `mib`: the non-binary densities (6, 12, 24 Gb dies use
/// a 3/4 populated row space). None on overflow.
pub const fn non_binary_mib(mib: u32) -> Option<u32> {
    match mib.checked_mul(3) {
        Some(v) => Some(v >> 2),
        None => None,
    }
}

/// Why a [`Topology`] was refused.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TopologyError {
    /// Rank count other than 1 or 2.
    Ranks,
    /// Channel count other than 1, 2 or 4.
    Channels,
    /// Zero per-channel density, or a total that overflows u32 MiB.
    Size,
    /// An LPDDR4 MR8 value the decode refused.
    Mr8(crate::lp4::density::DensityError),
}

/// Ranks, channels and per-channel density of one memory configuration: the
/// DRAM variants (one or two ranks, any density) as parameters, so board
/// code computes sizes, rank bits and address maps instead of hard-coding
/// one variant.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Topology {
    pub ranks: u8,
    pub channels: u8,
    /// MiB of one channel of one rank.
    pub channel_mib: u32,
}

impl Topology {
    pub const fn new(ranks: u8, channels: u8, channel_mib: u32) -> Result<Topology, TopologyError> {
        if ranks != 1 && ranks != 2 {
            return Err(TopologyError::Ranks);
        }
        if channels != 1 && channels != 2 && channels != 4 {
            return Err(TopologyError::Channels);
        }
        if channel_mib == 0 {
            return Err(TopologyError::Size);
        }
        let t = Topology {
            ranks,
            channels,
            channel_mib,
        };
        match t.total_mib() {
            Some(_) => Ok(t),
            None => Err(TopologyError::Size),
        }
    }

    /// LPDDR4/4X: every rank and channel reports `mr8`
    /// ([`crate::lp4::density::channel_mib`]).
    pub const fn lp4(ranks: u8, channels: u8, mr8: u8) -> Result<Topology, TopologyError> {
        match crate::lp4::density::channel_mib(mr8) {
            Ok(mib) => Topology::new(ranks, channels, mib),
            Err(e) => Err(TopologyError::Mr8(e)),
        }
    }

    /// log2 of the rank count (the rank address bit).
    pub const fn rank_bits(self) -> u32 {
        (self.ranks / 2) as u32
    }

    /// MiB of one rank over all channels.
    pub const fn rank_mib(self) -> Option<u32> {
        self.channel_mib.checked_mul(self.channels as u32)
    }

    /// MiB over all ranks and channels.
    pub const fn total_mib(self) -> Option<u32> {
        match self.rank_mib() {
            Some(r) => r.checked_mul(self.ranks as u32),
            None => None,
        }
    }
}
