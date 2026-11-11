// SPDX-License-Identifier: GPL-2.0-only
//! The TF-A BL31 handoff: `bl_params` version 2 in x0, an FDT in x1.
//!
//! Structures from TF-A include/export/common/{param_header_exp.h,
//! ep_info_exp.h, bl_common_exp.h} (BSD-3-Clause), AArch64 layout:
//! param_header {u8 type, u8 version, u16 size, u32 attr} (8 bytes);
//! entry_point_info {header, u64 pc, u32 spsr, pad, u64 args[8]} (88);
//! image_info {header, u64 image_base, u32 image_size, u32 image_max_size}
//! (24); bl_params_node {u32 image_id, pad, u64 image_info, u64 ep_info,
//! u64 next} (32); bl_params {header, u64 head} (16). BL31 parses them with
//! bl31_params_parse_helper (common/desc_image_load.c): BL33 only, no BL32.
//!
//! The FDT carries what plat/rockchip/common/params_setup.c reads:
//! /chosen stdout-path "serial2:1500000" (UART2 console) and /memory
//! nodes (the top of DRAM for the DDR firewall alias window).
//!
//! DRAM banks follow mainline U-Boot (GPL-2.0+) arch/arm/mach-rockchip/
//! sdram.c / rk3588/rk3588.c: [0, 0xF0000000) below the MMIO hole, the rest
//! from 4 GiB, minus the [0x3FC000000, 0x400000000) gap on 16 GiB+.

pub const PARAM_EP: u8 = 0x01;
pub const PARAM_IMAGE_BINARY: u8 = 0x02;
pub const PARAM_BL_PARAMS: u8 = 0x05;
pub const VERSION_2: u8 = 0x02;
pub const BL33_IMAGE_ID: u32 = 5;
pub const EP_NON_SECURE: u32 = 0x1;
/// SPSR_64(EL2, SP_ELx, all exceptions masked) = 0x3C9: EDK2 at EL2h.
pub const SPSR_EL2H_DAIF: u32 = (0xf << 6) | (2 << 2) | 1;

pub const BL_PARAMS_SIZE: usize = 16;
pub const NODE_SIZE: usize = 32;
pub const EP_SIZE: usize = 88;
pub const IMAGE_INFO_SIZE: usize = 24;
/// Bytes [`bl_params`] writes.
pub const TOTAL: usize = BL_PARAMS_SIZE + NODE_SIZE + EP_SIZE + IMAGE_INFO_SIZE;

fn put(b: &mut [u8], at: usize, v: &[u8]) {
    b[at..at + v.len()].copy_from_slice(v);
}
fn header(b: &mut [u8], at: usize, ty: u8, size: u16, attr: u32) {
    put(b, at, &[ty, VERSION_2]);
    put(b, at + 2, &size.to_le_bytes());
    put(b, at + 4, &attr.to_le_bytes());
}

/// Write bl_params v2 for one BL33 image at `pc` into `out`, which will
/// live at address `addr` (the embedded pointers are absolute).
pub fn bl_params(out: &mut [u8; TOTAL], addr: u64, pc: u64, image_size: u32, arg0: u64) {
    out.fill(0);
    let node = BL_PARAMS_SIZE;
    let ep = node + NODE_SIZE;
    let info = ep + EP_SIZE;
    header(out, 0, PARAM_BL_PARAMS, BL_PARAMS_SIZE as u16, 0);
    put(out, 8, &(addr + node as u64).to_le_bytes());
    put(out, node, &BL33_IMAGE_ID.to_le_bytes());
    put(out, node + 8, &(addr + info as u64).to_le_bytes());
    put(out, node + 16, &(addr + ep as u64).to_le_bytes());
    put(out, node + 24, &0u64.to_le_bytes());
    header(out, ep, PARAM_EP, EP_SIZE as u16, EP_NON_SECURE);
    put(out, ep + 8, &pc.to_le_bytes());
    put(out, ep + 16, &SPSR_EL2H_DAIF.to_le_bytes());
    put(out, ep + 24, &arg0.to_le_bytes());
    header(out, info, PARAM_IMAGE_BINARY, IMAGE_INFO_SIZE as u16, 0);
    put(out, info + 8, &pc.to_le_bytes());
    put(out, info + 16, &image_size.to_le_bytes());
    put(out, info + 20, &image_size.to_le_bytes());
}

pub const MMIO_HOLE: u64 = 0xF000_0000;
pub const GAP_START: u64 = 0x3_FC00_0000;
pub const GAP_END: u64 = 0x4_0000_0000;

/// DRAM banks (base, size) for `total` bytes of DRAM.
pub fn banks(total: u64) -> ([(u64, u64); 3], usize) {
    let mut b = [(0u64, 0u64); 3];
    if total <= MMIO_HOLE {
        b[0] = (0, total);
        return (b, 1);
    }
    b[0] = (0, MMIO_HOLE);
    let top = total; // DRAM above the hole continues at 4 GiB (U-Boot)
    if top <= 1 << 32 {
        return (b, 1); // exactly 4 GiB: the 256 MiB under the hole is lost
    }
    if top <= GAP_START {
        b[1] = (1 << 32, top - (1 << 32));
        return (b, 2);
    }
    b[1] = (1 << 32, GAP_START - (1 << 32));
    if top > GAP_END {
        b[2] = (GAP_END, top - GAP_END);
        return (b, 3);
    }
    (b, 2)
}

/// The FDT for BL31 into `out`; returns its size.
pub fn bl31_fdt(out: &mut [u8], total: u64) -> usize {
    let mut f: fdt_writer::FdtWriter<1024, 256> = fdt_writer::FdtWriter::new();
    f.begin_node("");
    f.prop_u32("#address-cells", 2);
    f.prop_u32("#size-cells", 2);
    f.begin_node("chosen");
    f.prop_str("stdout-path", "serial2:1500000n8");
    f.end_node();
    let (b, n) = banks(total);
    for (base, size) in &b[..n] {
        // Node names need not be unique per spec for parsing here, but keep
        // them distinct: memory@<base>.
        let mut name = [0u8; 32];
        let s = fmt_name(&mut name, *base);
        f.begin_node(s);
        f.prop_str("device_type", "memory");
        f.prop_u64_cells("reg", &[*base, *size]);
        f.end_node();
    }
    f.end_node();
    f.finish(out)
}

fn fmt_name(buf: &mut [u8; 32], base: u64) -> &str {
    let prefix = b"memory@";
    buf[..prefix.len()].copy_from_slice(prefix);
    let mut n = prefix.len();
    let hex = b"0123456789abcdef";
    let mut started = false;
    for i in (0..16).rev() {
        let d = ((base >> (4 * i)) & 0xf) as usize;
        if d != 0 || started || i == 0 {
            buf[n] = hex[d];
            n += 1;
            started = true;
        }
    }
    core::str::from_utf8(&buf[..n]).unwrap_or("memory")
}
