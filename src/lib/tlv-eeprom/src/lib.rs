// SPDX-License-Identifier: GPL-2.0-only
//! ONIE TlvInfo EEPROM records (`TlvInfo\0`, version 1, big-endian length,
//! TLVs, CRC-32 TLV last), validated in full before any value is used.
//!
//! Moved from ore-edk-boot-opi 78479e21ba `oreboot/src/mainboard/spacemit/
//! k1x/bt0/src/eeprom.rs` (`parse`, `crc32`, the TLV constants): the
//! record checks and the TLV walk are here; what a board does with its own
//! codes (the K1's DDR rank and data rate, 0x41/0x43) stays in the board,
//! as a closure given to [`walk`]. The I2C controller that reads the EEPROM
//! is board code too. CRC-32 is `storage::crc`.
#![no_std]

pub const MAGIC: &[u8; 8] = b"TlvInfo\0";
pub const VERSION: u8 = 1;
/// Magic, version and the 16-bit big-endian TLV area length.
pub const HEADER_LEN: usize = 11;

/// ONIE type codes.
pub mod code {
    pub const PRODUCT_NAME: u8 = 0x21;
    pub const PART_NUMBER: u8 = 0x22;
    pub const SERIAL_NUMBER: u8 = 0x23;
    pub const MAC_BASE: u8 = 0x24;
    pub const MANUFACTURE_DATE: u8 = 0x25;
    pub const DEVICE_VERSION: u8 = 0x26;
    pub const LABEL_REVISION: u8 = 0x27;
    pub const PLATFORM_NAME: u8 = 0x28;
    pub const ONIE_VERSION: u8 = 0x29;
    pub const MAC_SIZE: u8 = 0x2a;
    pub const MANUFACTURER: u8 = 0x2b;
    pub const COUNTRY_CODE: u8 = 0x2c;
    pub const VENDOR: u8 = 0x2d;
    pub const DIAG_VERSION: u8 = 0x2e;
    pub const SERVICE_TAG: u8 = 0x2f;
    pub const VENDOR_EXTENSION: u8 = 0xfd;
    pub const CRC_32: u8 = 0xfe;
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ParseError {
    /// Too short for a header, wrong magic or version.
    Header,
    /// TLV area shorter than the CRC TLV, or beyond `max` or the input.
    Length,
    /// The last TLV is not a 4-byte CRC-32, or the CRC does not match.
    Crc,
    /// A TLV runs past the CRC TLV, or a second CRC TLV.
    Tlv,
    /// A code the caller accepts once appeared twice (see [`Once`]).
    Duplicate,
}

/// Total record length (header + TLV area) announced by a header, or None
/// if `header` is not a valid header. For reading a record in two steps.
pub fn total_len(header: &[u8]) -> Option<usize> {
    if header.len() < HEADER_LEN || &header[..8] != MAGIC || header[8] != VERSION {
        return None;
    }
    Some(HEADER_LEN + u16::from_be_bytes([header[9], header[10]]) as usize)
}

/// Validate the whole record `e` (at most `max` bytes: the EEPROM size),
/// then call `f(code, value)` for every TLV before the CRC TLV, in order.
/// Nothing is passed to `f` unless header, length and CRC are valid; a TLV
/// structure error or an error from `f` stops the walk.
pub fn walk<'a>(
    e: &'a [u8],
    max: usize,
    mut f: impl FnMut(u8, &'a [u8]) -> Result<(), ParseError>,
) -> Result<(), ParseError> {
    if e.len() < HEADER_LEN || &e[..8] != MAGIC || e[8] != VERSION {
        return Err(ParseError::Header);
    }
    let len = u16::from_be_bytes([e[9], e[10]]) as usize;
    let end = HEADER_LEN + len;
    if len < 6 || end > max || end > e.len() {
        return Err(ParseError::Length);
    }
    let stop = end - 6;
    if e[stop..stop + 2] != [code::CRC_32, 4] {
        return Err(ParseError::Crc);
    }
    let want = u32::from_be_bytes([e[end - 4], e[end - 3], e[end - 2], e[end - 1]]);
    if storage::crc::crc32(&e[..end - 4]) != want {
        return Err(ParseError::Crc);
    }
    let mut p = HEADER_LEN;
    while p < stop {
        if stop - p < 2 {
            return Err(ParseError::Tlv);
        }
        let (c, n) = (e[p], e[p + 1] as usize);
        p += 2;
        if n > stop - p {
            return Err(ParseError::Tlv);
        }
        if c == code::CRC_32 {
            return Err(ParseError::Tlv);
        }
        f(c, &e[p..p + n])?;
        p += n;
    }
    Ok(())
}

/// A value a record may carry at most once.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Once<T>(pub Option<T>);

impl<T> Once<T> {
    /// Store `v`; [`ParseError::Duplicate`] if a value is already stored.
    pub fn set(&mut self, v: T) -> Result<(), ParseError> {
        if self.0.is_some() {
            return Err(ParseError::Duplicate);
        }
        self.0 = Some(v);
        Ok(())
    }
}

/// `value` must be exactly `n` bytes long ([`ParseError::Tlv`] otherwise).
pub fn exact(value: &[u8], n: usize) -> Result<&[u8], ParseError> {
    if value.len() == n {
        Ok(value)
    } else {
        Err(ParseError::Tlv)
    }
}

/// Write a record with the TLV area `tlvs` (already encoded type, length,
/// value triples) and its CRC TLV into `out`; returns its length, or None if
/// `out` is too small or the area too long.
pub fn write_record(tlvs: &[u8], out: &mut [u8]) -> Option<usize> {
    let len = tlvs.len() + 6;
    let total = HEADER_LEN + len;
    if len > u16::MAX as usize || out.len() < total {
        return None;
    }
    out[..8].copy_from_slice(MAGIC);
    out[8] = VERSION;
    out[9..11].copy_from_slice(&(len as u16).to_be_bytes());
    out[HEADER_LEN..HEADER_LEN + tlvs.len()].copy_from_slice(tlvs);
    let c = HEADER_LEN + tlvs.len();
    out[c] = code::CRC_32;
    out[c + 1] = 4;
    let crc = storage::crc::crc32(&out[..c + 2]);
    out[c + 2..total].copy_from_slice(&crc.to_be_bytes());
    Some(total)
}
