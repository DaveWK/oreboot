// SPDX-License-Identifier: GPL-2.0-only
//! bl_params v2 layout and the BL31 FDT (crate::handoff).
use fdt_checked::{self as fdt, Fdt};
use rk3588::handoff::{self, TOTAL};

fn u64_at(b: &[u8], at: usize) -> u64 {
    u64::from_le_bytes(b[at..at + 8].try_into().unwrap())
}
fn u32_at(b: &[u8], at: usize) -> u32 {
    u32::from_le_bytes(b[at..at + 4].try_into().unwrap())
}

#[test]
fn bl_params_v2_layout() {
    let addr = 0x0310_0000u64;
    let mut b = [0u8; TOTAL];
    handoff::bl_params(&mut b, addr, 0x20_0000, 0x50_0000, 0);
    assert_eq!(TOTAL, 16 + 32 + 88 + 24);
    // bl_params header: type 5, version 2, size 16.
    assert_eq!(&b[0..4], &[5, 2, 16, 0]);
    let node = u64_at(&b, 8) - addr;
    assert_eq!(node, 16);
    let node = node as usize;
    assert_eq!(u32_at(&b, node), 5); // BL33_IMAGE_ID
    let info = (u64_at(&b, node + 8) - addr) as usize;
    let ep = (u64_at(&b, node + 16) - addr) as usize;
    assert_eq!(u64_at(&b, node + 24), 0); // last node
                                          // entry_point_info: type 1, version 2, size 88, NON_SECURE.
    assert_eq!(&b[ep..ep + 4], &[1, 2, 88, 0]);
    assert_eq!(u32_at(&b, ep + 4), 1);
    assert_eq!(u64_at(&b, ep + 8), 0x20_0000);
    assert_eq!(u32_at(&b, ep + 16), 0x3c9);
    // image_info: type 2, version 2, size 24.
    assert_eq!(&b[info..info + 4], &[2, 2, 24, 0]);
    assert_eq!(u64_at(&b, info + 8), 0x20_0000);
}

#[test]
fn banks() {
    assert_eq!(handoff::banks(2 << 30).1, 1);
    let (b, n) = handoff::banks(4 << 30);
    assert_eq!((n, b[0]), (1, (0, 0xF000_0000)));
    let (b, n) = handoff::banks(8 << 30);
    assert_eq!((n, b[1]), (2, (1 << 32, 4 << 30)));
    let (b, n) = handoff::banks(32 << 30);
    assert_eq!(n, 3);
    assert_eq!(b[1].0 + b[1].1, 0x3_FC00_0000);
    assert_eq!(b[2], (0x4_0000_0000, (32u64 << 30) - 0x4_0000_0000));
}

#[test]
fn bl31_fdt_has_console_and_memory() {
    let mut out = [0u8; 2048];
    let n = handoff::bl31_fdt(&mut out, 8 << 30);
    let f = Fdt::new(&out[..n]).unwrap();
    let chosen = f.child(f.h.structure, b"chosen").unwrap();
    assert_eq!(
        fdt::string(f.prop(chosen, b"stdout-path").unwrap()).unwrap(),
        b"serial2:1500000n8"
    );
    let m0 = f.child(f.h.structure, b"memory@0").unwrap();
    assert_eq!(
        fdt::string(f.prop(m0, b"device_type").unwrap()).unwrap(),
        b"memory"
    );
    let reg = f.prop(m0, b"reg").unwrap();
    assert_eq!(reg.len(), 16);
    assert_eq!(
        u64::from_be_bytes(reg[8..16].try_into().unwrap()),
        0xF000_0000
    );
    assert!(f.child(f.h.structure, b"memory@100000000").is_ok());
}

/// The YY3588's 32 GiB (4 x 8 GiB): what OS_REG decodes to and the banks
/// TF-A gets. DRAM below the MMIO hole, then from 4 GiB up to the
/// [0x3FC000000, 0x400000000) gap (mainline U-Boot rk3588, 16 GiB+), then
/// 16..32 GiB. edk2-rk3588 SdramLib/Rk3588Mem.c compute the same 32 GiB
/// from OS_REG and reserve BAD1/BAD2 inside that gap.
#[test]
fn yy3588_32gib_layout() {
    let p = rk3588::osreg::decode(0x3a56_1a56, 0x3000_10f5);
    let total = (2 * p.size_mib()) << 20;
    assert_eq!(total, 32 << 30);
    let (b, n) = handoff::banks(total);
    assert_eq!(n, 3);
    assert_eq!(
        b,
        [
            (0, 0xF000_0000),
            (0x1_0000_0000, 0x2_FC00_0000),
            (0x4_0000_0000, 0x4_0000_0000)
        ]
    );
    // Nothing in the gap or above 32 GiB; 256 MiB under the hole and the
    // 64 MiB gap are not DRAM to the OS.
    let sum: u64 = b.iter().map(|x| x.1).sum();
    assert_eq!(sum, (32 << 30) - 0x1000_0000 - 0x400_0000);
    assert!(b
        .iter()
        .all(|&(s, l)| s + l <= handoff::GAP_START || s >= handoff::GAP_END));
    let mut out = [0u8; 2048];
    let len = handoff::bl31_fdt(&mut out, total);
    let f = Fdt::new(&out[..len]).unwrap();
    for (name, base, size) in [
        (&b"memory@0"[..], 0u64, 0xF000_0000u64),
        (b"memory@100000000", 0x1_0000_0000, 0x2_FC00_0000),
        (b"memory@400000000", 0x4_0000_0000, 0x4_0000_0000),
    ] {
        let m = f.child(f.h.structure, name).unwrap();
        let reg = f.prop(m, b"reg").unwrap();
        assert_eq!(u64::from_be_bytes(reg[0..8].try_into().unwrap()), base);
        assert_eq!(u64::from_be_bytes(reg[8..16].try_into().unwrap()), size);
    }
}
