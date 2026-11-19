// SPDX-License-Identifier: GPL-2.0-only

//! The host side of USB enumeration for one mass storage device: the
//! control requests bt0's host controller driver may send, and finding the
//! Bulk-Only interface in a configuration descriptor.
//!
//! [`control_allowed`] is the only gate to the control pipe: GET_DESCRIPTOR,
//! SET_ADDRESS, SET_CONFIGURATION, CLEAR_FEATURE(ENDPOINT_HALT), and the
//! mass storage class's GET_MAX_LUN and Bulk-Only Mass Storage Reset.
//! Nothing else (no SET_FEATURE, SET_DESCRIPTOR, vendor requests, ADSC
//! for other subclasses) can reach the device.
use crate::usb::Setup;

pub const GET_MAX_LUN: (u8, u8) = (0xa1, 0xfe);
pub const BOT_RESET: (u8, u8) = (0x21, 0xff);

pub fn control_allowed(s: &Setup) -> bool {
    match (s.request_type, s.request) {
        (0x80, 6) => true,
        (0x00, 5) => s.value <= 127 && s.length == 0,
        (0x00, 9) => s.length == 0,
        (0x02, 1) => s.value == 0 && s.length == 0,
        GET_MAX_LUN => s.value == 0 && s.length == 1,
        BOT_RESET => s.value == 0 && s.length == 0,
        _ => false,
    }
}

pub fn setup(request_type: u8, request: u8, value: u16, index: u16, length: u16) -> Setup {
    Setup {
        request_type,
        request,
        value,
        index,
        length,
    }
}

pub fn setup_bytes(s: &Setup) -> [u8; 8] {
    let [v0, v1] = s.value.to_le_bytes();
    let [i0, i1] = s.index.to_le_bytes();
    let [l0, l1] = s.length.to_le_bytes();
    [s.request_type, s.request, v0, v1, i0, i1, l0, l1]
}

/// The mass storage function of a configuration.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MscInterface {
    pub configuration: u8,
    pub interface: u8,
    pub ep_in: u8,
    pub ep_out: u8,
    pub maxp_in: u16,
    pub maxp_out: u16,
}

/// Find a Bulk-Only SCSI interface (class 8, subclass 6, protocol 0x50,
/// alternate setting 0) and its two bulk endpoints in a full
/// configuration descriptor.
pub fn find_msc(cfg: &[u8]) -> Option<MscInterface> {
    if cfg.len() < 9 || cfg[1] != 2 {
        return None;
    }
    let configuration = cfg[5];
    let mut found: Option<MscInterface> = None;
    let mut i = 0;
    while i + 2 <= cfg.len() {
        let len = cfg[i] as usize;
        if len < 2 || i + len > cfg.len() {
            return None;
        }
        let d = &cfg[i..i + len];
        match d[1] {
            4 if len >= 9 => {
                if found.is_some_and(|f| f.ep_in != 0 && f.ep_out != 0) {
                    break;
                }
                found =
                    (d[3] == 0 && d[5] == 8 && d[6] == 6 && d[7] == 0x50).then_some(MscInterface {
                        configuration,
                        interface: d[2],
                        ep_in: 0,
                        ep_out: 0,
                        maxp_in: 0,
                        maxp_out: 0,
                    });
            }
            5 if len >= 7 => {
                if let Some(f) = found.as_mut() {
                    if d[3] & 3 == 2 {
                        let maxp = u16::from_le_bytes([d[4], d[5]]) & 0x7ff;
                        if d[2] & 0x80 != 0 && f.ep_in == 0 {
                            f.ep_in = d[2];
                            f.maxp_in = maxp;
                        } else if d[2] & 0x80 == 0 && f.ep_out == 0 {
                            f.ep_out = d[2];
                            f.maxp_out = maxp;
                        }
                    }
                }
            }
            _ => {}
        }
        i += len;
    }
    found.filter(|f| f.ep_in != 0 && f.ep_out != 0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::vec::Vec;

    #[test]
    fn allow_list() {
        assert!(control_allowed(&setup(0x80, 6, 0x0100, 0, 18)));
        assert!(control_allowed(&setup(0x00, 5, 1, 0, 0)));
        assert!(control_allowed(&setup(0x00, 9, 1, 0, 0)));
        assert!(control_allowed(&setup(0x02, 1, 0, 0x81, 0)));
        assert!(control_allowed(&setup(0xa1, 0xfe, 0, 0, 1)));
        assert!(control_allowed(&setup(0x21, 0xff, 0, 0, 0)));
        for (t, r, v, l) in [
            (0x00, 3, 1, 0),      // SET_FEATURE
            (0x02, 3, 0, 0),      // SET_FEATURE(halt)
            (0x00, 7, 0x100, 18), // SET_DESCRIPTOR
            (0x02, 1, 1, 0),      // CLEAR_FEATURE other than halt
            (0x21, 0x00, 0, 8),   // ADSC (CBI)
            (0x40, 0x01, 0, 0),   // vendor
            (0xc0, 0x01, 0, 4),   // vendor in
            (0x00, 5, 128, 0),    // bad address
            (0x00, 9, 1, 2),      // SET_CONFIGURATION with data
        ] {
            assert!(!control_allowed(&setup(t, r, v, 0, l)), "{t:#x} {r:#x}");
        }
        assert_eq!(
            setup_bytes(&setup(0x80, 6, 0x0200, 0, 255)),
            [0x80, 6, 0, 2, 0, 0, 255, 0]
        );
    }

    /// (number, class, subclass, protocol, endpoints (address, attributes, maxp)).
    type Iface<'a> = (u8, u8, u8, u8, &'a [(u8, u8, u16)]);

    fn cfg(ifaces: &[Iface]) -> Vec<u8> {
        let mut v = std::vec![9, 2, 0, 0, ifaces.len() as u8, 3, 0, 0x80, 50];
        for &(num, class, sub, proto, eps) in ifaces {
            v.extend_from_slice(&[9, 4, num, 0, eps.len() as u8, class, sub, proto, 0]);
            for &(addr, attr, maxp) in eps {
                let [m0, m1] = maxp.to_le_bytes();
                v.extend_from_slice(&[7, 5, addr, attr, m0, m1, 0]);
                // SuperSpeed companion descriptors are skipped.
                v.extend_from_slice(&[6, 0x30, 0, 0, 0, 0]);
            }
        }
        let n = v.len() as u16;
        v[2..4].copy_from_slice(&n.to_le_bytes());
        v
    }

    #[test]
    fn finds_the_bulk_only_interface() {
        // A UAS-capable disk: interface 0 alt 0 BOT; a HID interface first.
        let c = cfg(&[
            (1, 3, 0, 0, &[(0x83, 3, 8)]),
            (0, 8, 6, 0x50, &[(0x81, 2, 512), (0x02, 2, 512)]),
        ]);
        assert_eq!(
            find_msc(&c),
            Some(MscInterface {
                configuration: 3,
                interface: 0,
                ep_in: 0x81,
                ep_out: 0x02,
                maxp_in: 512,
                maxp_out: 512
            })
        );
        // UAS only (protocol 0x62), or a missing endpoint: none.
        assert_eq!(
            find_msc(&cfg(&[(0, 8, 6, 0x62, &[(0x81, 2, 512), (0x02, 2, 512)])])),
            None
        );
        assert_eq!(find_msc(&cfg(&[(0, 8, 6, 0x50, &[(0x81, 2, 512)])])), None);
        // Truncated or malformed descriptors.
        let mut c = cfg(&[(0, 8, 6, 0x50, &[(0x81, 2, 512), (0x02, 2, 512)])]);
        assert!(find_msc(&c[..c.len() - 3]).is_none());
        c[9] = 0;
        assert!(find_msc(&c).is_none());
        assert!(find_msc(&[]).is_none());
    }
}
