// SPDX-License-Identifier: GPL-2.0-only

//! USB 2.0 chapter 9 for a one-interface bulk function (fastboot): SETUP
//! packets, the standard requests and the descriptors.
//!
//! Follows the shared `usb-device` crate of the SpacemiT K1 bt0 (first its
//! `usb.rs`; kept outside this tree), which src/lib does not include yet; same
//! descriptors and request set, so the two can merge later.

/// Control endpoint maximum packet size.
pub const EP0_MAX: usize = 64;
/// The bulk endpoint pair: EP1 IN (0x81) and EP1 OUT (0x01).
pub const EP1: u8 = 1;

/// What the function offers: IDs, interface class and strings.
#[derive(Clone, Copy, Debug)]
pub struct Descriptors<'a> {
    pub vendor: u16,
    pub product: u16,
    /// Interface class, subclass, protocol (fastboot: ff/42/03).
    pub class: [u8; 3],
    /// Manufacturer, product, serial number, interface.
    pub strings: [&'a str; 4],
}

/// The fastboot IDs the stock `fastboot` tool and U-Boot use.
pub const FASTBOOT_VID: u16 = 0x18d1;
pub const FASTBOOT_PID: u16 = 0x4ee0;
pub const FASTBOOT_CLASS: [u8; 3] = [0xff, 0x42, 0x03];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Setup {
    pub request_type: u8,
    pub request: u8,
    pub value: u16,
    pub index: u16,
    pub length: u16,
}

impl Setup {
    pub fn parse(s: [u8; 8]) -> Self {
        Self {
            request_type: s[0],
            request: s[1],
            value: u16::from_le_bytes([s[2], s[3]]),
            index: u16::from_le_bytes([s[4], s[5]]),
            length: u16::from_le_bytes([s[6], s[7]]),
        }
    }
}

/// The standard requests a simple function answers.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Request {
    /// `value` is type << 8 | index; answer [`descriptor`].
    GetDescriptor {
        value: u16,
    },
    /// Applied after the status stage.
    SetAddress(u8),
    /// 1 configures, 0 deconfigures, anything else stalls.
    SetConfiguration(u16),
    GetConfiguration,
    /// Device status: self powered.
    GetStatusDevice,
    /// Interface or endpoint status: zero.
    GetStatusOther,
    /// CLEAR_FEATURE(ENDPOINT_HALT) of endpoint `index` (bit 7: IN).
    ClearEndpointHalt {
        index: u16,
    },
    /// Only alternate setting 0 exists.
    SetInterface,
    /// Anything else: stall.
    Unsupported,
}

pub fn decode(s: &Setup) -> Request {
    match (s.request_type, s.request) {
        (0x80, 6) => Request::GetDescriptor { value: s.value },
        (0x00, 5) if s.value <= 127 => Request::SetAddress(s.value as u8),
        (0x00, 9) if s.value <= 1 => Request::SetConfiguration(s.value),
        (0x80, 8) => Request::GetConfiguration,
        (0x80, 0) => Request::GetStatusDevice,
        (0x81, 0) | (0x82, 0) => Request::GetStatusOther,
        (0x02, 1) if s.value == 0 => Request::ClearEndpointHalt { index: s.index },
        (0x01, 11) if s.value == 0 => Request::SetInterface,
        _ => Request::Unsupported,
    }
}

/// Build descriptor `value` (type << 8 | index) into `buf` for a bulk
/// maximum packet size `max_packet` (64 at full speed, 512 at high speed).
/// Its length, or None when the request must be stalled.
pub fn descriptor(
    d: &Descriptors,
    value: u16,
    max_packet: u16,
    buf: &mut [u8; 64],
) -> Option<usize> {
    let kind = (value >> 8) as u8;
    let index = (value & 0xff) as usize;
    let n = match kind {
        // Device
        1 => {
            let [vl, vh] = d.vendor.to_le_bytes();
            let [pl, ph] = d.product.to_le_bytes();
            buf[..18].copy_from_slice(&[
                18,
                1,
                0x00,
                0x02,
                0,
                0,
                0,
                EP0_MAX as u8,
                vl,
                vh,
                pl,
                ph,
                0x00,
                0x01,
                1,
                2,
                3,
                1,
            ]);
            18
        }
        // Configuration: interface and the two bulk endpoints
        2 if index == 0 => {
            let [ml, mh] = max_packet.to_le_bytes();
            let c = d.class;
            buf[..32].copy_from_slice(&[
                9,
                2,
                32,
                0,
                1,
                1,
                0,
                0xc0,
                50, // configuration
                9,
                4,
                0,
                0,
                2,
                c[0],
                c[1],
                c[2],
                4, // interface
                7,
                5,
                0x80 | EP1,
                2,
                ml,
                mh,
                0, // EP1 IN
                7,
                5,
                EP1,
                2,
                ml,
                mh,
                1, // EP1 OUT
            ]);
            32
        }
        // String: language list, then UTF-16LE strings (ASCII, at most 30)
        3 => {
            if index == 0 {
                buf[..4].copy_from_slice(&[4, 3, 0x09, 0x04]);
                4
            } else if index <= 4 {
                let s = d.strings[index - 1].as_bytes();
                let n = s.len().min(30);
                buf[0] = (2 + 2 * n) as u8;
                buf[1] = 3;
                for (i, c) in s[..n].iter().enumerate() {
                    buf[2 + 2 * i] = *c;
                    buf[3 + 2 * i] = 0;
                }
                2 + 2 * n
            } else {
                return None;
            }
        }
        // Device qualifier: the other speed has the same layout
        6 => {
            buf[..10].copy_from_slice(&[10, 6, 0x00, 0x02, 0, 0, 0, EP0_MAX as u8, 1, 0]);
            10
        }
        _ => return None,
    };
    Some(n)
}

/// Hex digits of a 128-bit chip ID for the serial number string.
pub fn serial(sid: [u32; 4], out: &mut [u8; 16]) -> &str {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    // The first two words differ between chips of a lot; enough for a name.
    let v = ((sid[0] as u64) << 32) | sid[1] as u64;
    for (i, o) in out.iter_mut().enumerate() {
        *o = HEX[((v >> (60 - 4 * i)) & 0xf) as usize];
    }
    core::str::from_utf8(out).unwrap_or("0")
}

#[cfg(test)]
mod tests {
    use super::*;
    const D: Descriptors = Descriptors {
        vendor: FASTBOOT_VID,
        product: FASTBOOT_PID,
        class: FASTBOOT_CLASS,
        strings: ["Maker", "Board", "0123", "fastboot"],
    };

    #[test]
    fn setup_fields_are_little_endian() {
        let s = Setup::parse([0x80, 6, 0x00, 0x01, 0x09, 0x04, 0x12, 0x00]);
        assert_eq!((s.value, s.index, s.length), (0x0100, 0x0409, 18));
        assert_eq!(decode(&s), Request::GetDescriptor { value: 0x0100 });
    }

    #[test]
    fn requests() {
        let r = |t, q, v: u16, i| {
            let [v0, v1] = v.to_le_bytes();
            decode(&Setup::parse([t, q, v0, v1, i, 0, 0, 0]))
        };
        assert_eq!(r(0, 5, 7, 0), Request::SetAddress(7));
        assert_eq!(r(0, 5, 128, 0), Request::Unsupported);
        assert_eq!(r(0, 9, 1, 0), Request::SetConfiguration(1));
        assert_eq!(r(0, 9, 0, 0), Request::SetConfiguration(0));
        assert_eq!(r(0, 9, 2, 0), Request::Unsupported);
        assert_eq!(r(0x80, 8, 0, 0), Request::GetConfiguration);
        assert_eq!(r(0x80, 0, 0, 0), Request::GetStatusDevice);
        assert_eq!(r(0x82, 0, 0, 0x81), Request::GetStatusOther);
        assert_eq!(r(2, 1, 0, 0x81), Request::ClearEndpointHalt { index: 0x81 });
        assert_eq!(r(2, 1, 1, 0x81), Request::Unsupported, "only ENDPOINT_HALT");
        assert_eq!(r(1, 11, 0, 0), Request::SetInterface);
        assert_eq!(r(1, 11, 1, 0), Request::Unsupported);
        assert_eq!(r(0x21, 0x22, 0, 0), Request::Unsupported);
        assert_eq!(r(0x00, 3, 1, 0), Request::Unsupported, "SET_FEATURE");
    }

    #[test]
    fn descriptors() {
        let mut b = [0u8; 64];
        assert_eq!(descriptor(&D, 0x0100, 512, &mut b), Some(18));
        assert_eq!(&b[8..12], &[0xd1, 0x18, 0xe0, 0x4e]);
        let mut b = [0u8; 64];
        assert_eq!(descriptor(&D, 0x0200, 512, &mut b), Some(32));
        assert_eq!((b[14], b[15], b[16]), (0xff, 0x42, 3));
        assert_eq!(&b[22..24], &[0, 2]);
        assert_eq!((b[20], b[27]), (0x81, 0x01));
        assert_eq!(descriptor(&D, 0x0201, 512, &mut b), None);
        let mut b = [0u8; 64];
        assert_eq!(descriptor(&D, 0x0200, 64, &mut b), Some(32));
        assert_eq!(&b[22..24], &[64, 0]);
        let mut b = [0xaau8; 64];
        assert_eq!(descriptor(&D, 0x0304, 64, &mut b), Some(18));
        assert_eq!(&b[..6], &[18, 3, b'f', 0, b'a', 0]);
        assert_eq!(descriptor(&D, 0x0305, 64, &mut [0; 64]), None);
        assert_eq!(descriptor(&D, 0x0600, 64, &mut [0; 64]), Some(10));
        assert_eq!(descriptor(&D, 0x0700, 64, &mut [0; 64]), None);
        let long = Descriptors {
            strings: ["0123456789012345678901234567890123456789"; 4],
            ..D
        };
        assert_eq!(descriptor(&long, 0x0301, 64, &mut [0; 64]), Some(62));
    }

    #[test]
    fn serial_is_hex_of_the_first_two_words() {
        let mut b = [0u8; 16];
        assert_eq!(
            serial([0x0123_4567, 0x89ab_cdef, 9, 9], &mut b),
            "0123456789abcdef"
        );
    }
}
