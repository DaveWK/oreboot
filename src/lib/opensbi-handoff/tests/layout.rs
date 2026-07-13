// SPDX-License-Identifier: GPL-2.0-only
//! The info structure against the K1 board's definition (verbatim from
//! ore-edk-boot-opi 78479e21ba oreboot/src/mainboard/spacemit/k1x/bt0/src/fit.rs)
//! and OpenSBI's fw_dynamic.h field order.
use core::mem::{offset_of, size_of};
use opensbi_handoff::{FwDynamicInfo, NextMode, MAGIC, VERSION};

// --- verbatim from the K1 board (the static's initialiser) ---
#[repr(C)]
struct LegacyFwDynamicInfo {
    magic: usize,
    version: usize,
    next_addr: usize,
    next_mode: usize,
    options: usize,
    boot_hart: usize,
}
const LEGACY: LegacyFwDynamicInfo = LegacyFwDynamicInfo {
    magic: 0x4942_534f,
    version: 2,
    next_addr: 0,
    next_mode: 1, // S-mode
    options: 0,
    boot_hart: 0,
};
// --- end of the board copy ---

fn bytes<T>(v: &T) -> Vec<u8> {
    let p = v as *const T as *const u8;
    unsafe { core::slice::from_raw_parts(p, size_of::<T>()) }.to_vec()
}

#[test]
fn same_layout_and_bytes_as_the_board() {
    assert_eq!(size_of::<FwDynamicInfo>(), size_of::<LegacyFwDynamicInfo>());
    assert_eq!(size_of::<FwDynamicInfo>(), 6 * size_of::<usize>());
    let w = size_of::<usize>();
    assert_eq!(offset_of!(FwDynamicInfo, magic), 0);
    assert_eq!(offset_of!(FwDynamicInfo, version), w);
    assert_eq!(offset_of!(FwDynamicInfo, next_addr), 2 * w);
    assert_eq!(offset_of!(FwDynamicInfo, next_mode), 3 * w);
    assert_eq!(offset_of!(FwDynamicInfo, options), 4 * w);
    assert_eq!(offset_of!(FwDynamicInfo, boot_hart), 5 * w);
    for (next, hart) in [
        (0usize, 0usize),
        (0x20_0000, 0),
        (0x20_0000, 3),
        (usize::MAX, 7),
    ] {
        let mut old = LEGACY;
        old.next_addr = next;
        old.boot_hart = hart;
        let new = FwDynamicInfo::new(next, NextMode::Supervisor, hart);
        assert_eq!(bytes(&old), bytes(&new));
    }
    assert_eq!((MAGIC, VERSION), (LEGACY.magic, LEGACY.version));
    assert_eq!(NextMode::User as usize, 0);
    assert_eq!(NextMode::Machine as usize, 3);
}
