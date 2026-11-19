// SPDX-License-Identifier: GPL-2.0-only
//! The lab's six PHY images, staged as one validated, immutable payload.

use core::sync::atomic::{AtomicU8, Ordering};

#[cfg(test)]
use crate::crc32;
#[cfg(not(test))]
use cubie_a7a_common::crc32;

#[cfg(not(test))]
include!(concat!(env!("OUT_DIR"), "/phy-fw-lab.rs"));
#[cfg(test)]
const LEN: [usize; 6] = [1; 6];
#[cfg(test)]
const CRC: [u32; 6] = [0xd202_ef8d; 6]; // CRC-32 of a single zero byte.

const HEADER: usize = 8 + 8 * LEN.len();
const SIZE: usize = LEN[0] + LEN[1] + LEN[2] + LEN[3] + LEN[4] + LEN[5];
const EMPTY: u8 = 0;
const WRITING: u8 = 1;
const READY: u8 = 2;

#[unsafe(link_section = ".phyfw")]
static mut PAYLOAD: [u8; SIZE] = [0; SIZE];
// Initialized with bt0's BSS, not stored in the NOLOAD payload section.
static STATE: AtomicU8 = AtomicU8::new(EMPTY);

pub fn loaded() -> bool {
    STATE.load(Ordering::Acquire) == READY
}

fn validate(blob: &[u8]) -> Result<&[u8], &'static str> {
    let le = |o: usize| u32::from_le_bytes(blob[o..o + 4].try_into().unwrap());
    if blob.len() != HEADER + SIZE {
        return Err("wrong PHY firmware blob length");
    }
    if &blob[..4] != b"A7AP" || le(4) != LEN.len() as u32 {
        return Err("not this build's PHY firmware blob");
    }
    let data = &blob[HEADER..];
    let mut at = 0;
    for (i, (&len, &crc)) in LEN.iter().zip(&CRC).enumerate() {
        if le(8 + 8 * i) != len as u32 || le(12 + 8 * i) != crc || crc32(&data[at..at + len]) != crc
        {
            return Err("PHY firmware image does not match this build");
        }
        at += len;
    }
    Ok(data)
}

/// Validate and publish the host blob once.
///
/// # Safety
/// Calls must be serialized. The FEL lab command loop runs on the boot CPU
/// with IRQ and FIQ masked by its entry stub (`fel.rs`, `cpsid if`).
pub unsafe fn load(blob: &[u8]) -> Result<(), &'static str> {
    let data = validate(blob)?;
    // No RMW: as in dram/wrstage.rs, exclusive accesses abort with MMU off.
    if STATE.load(Ordering::Relaxed) != EMPTY {
        return Err("PHY firmware already staged");
    }
    STATE.store(WRITING, Ordering::Relaxed);
    // SAFETY: there is only one writer, and readers acquire READY before
    // taking a reference. Nothing mutates PAYLOAD after that publication.
    unsafe {
        core::ptr::copy_nonoverlapping(
            data.as_ptr(),
            core::ptr::addr_of_mut!(PAYLOAD).cast::<u8>(),
            SIZE,
        );
    }
    STATE.store(READY, Ordering::Release);
    Ok(())
}

pub(super) fn images() -> Option<[&'static [u8]; 6]> {
    if !loaded() {
        return None;
    }
    // SAFETY: READY means PAYLOAD was fully written and is immutable until
    // bt0 restarts (at which point no Rust references from this boot remain).
    let data = unsafe { &*core::ptr::addr_of!(PAYLOAD) };
    let mut start = 0;
    Some(LEN.map(|len| {
        let image = &data[start..start + len];
        start += len;
        image
    }))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn validates_all_images_before_publication_and_never_rewrites() {
        // SAFETY: this is the only test staging a payload; calls are sequential.
        let load = |blob: &[u8]| unsafe { super::load(blob) };
        let mut good = [0; HEADER + SIZE];
        good[..4].copy_from_slice(b"A7AP");
        good[4..8].copy_from_slice(&6u32.to_le_bytes());
        for entry in good[8..HEADER].chunks_exact_mut(8) {
            entry[..4].copy_from_slice(&1u32.to_le_bytes());
            entry[4..].copy_from_slice(&CRC[0].to_le_bytes());
        }

        let mut bad = good;
        for (index, label) in [
            (0, "magic"),
            (4, "count"),
            (8, "entry length"),
            (12, "entry checksum"),
            (good.len() - 1, "last image payload"),
        ] {
            bad[index] ^= 1;
            assert!(load(&bad).is_err(), "accepted wrong {label}");
            assert!(!loaded(), "published wrong {label}");
            assert_eq!(STATE.load(Ordering::Relaxed), EMPTY);
            bad[index] ^= 1;
        }
        assert!(load(&good[..good.len() - 1]).is_err());
        let mut trailing = [0; HEADER + SIZE + 1];
        trailing[..good.len()].copy_from_slice(&good);
        assert!(load(&trailing).is_err());
        assert!(images().is_none());

        assert_eq!(load(&good), Ok(()));
        assert!(loaded());
        assert_eq!(images().unwrap(), [&[0][..]; 6]);
        assert_eq!(load(&good), Err("PHY firmware already staged"));
        assert_eq!(images().unwrap(), [&[0][..]; 6]);
    }
}
