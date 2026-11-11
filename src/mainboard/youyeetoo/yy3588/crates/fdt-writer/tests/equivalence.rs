// SPDX-License-Identifier: GPL-2.0-only
//! fdt-writer against the verbatim A733 fdt.rs: the same random operation
//! scripts give byte-identical blobs, which fdt-checked accepts.

#[allow(dead_code, clippy::all)]
mod a7a {
    include!("legacy/a7a_fdt.rs");
}

struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }
    fn below(&mut self, n: u64) -> usize {
        (self.next() % n) as usize
    }
}

const NAMES: [&str; 8] = [
    "reg",
    "compatible",
    "#address-cells",
    "#size-cells",
    "method",
    "stdout-path",
    "linux,usable-memory-range",
    "status",
];
const NODES: [&str; 6] = ["", "memory@40000000", "psci", "chosen", "cpus", "cpu@0"];

#[test]
fn random_scripts_give_identical_blobs() {
    let mut rng = Rng(0x5eed_fd7a);
    for _ in 0..3000 {
        let mut old = a7a::Fdt::new();
        let mut new = fdt_writer::Fdt::new();
        let mut depth = 0;
        old.begin_node("");
        new.begin_node("");
        depth += 1;
        for _ in 0..rng.below(24) {
            let name = NAMES[rng.below(8)];
            match rng.below(8) {
                0 if depth < 4 => {
                    let n = NODES[1 + rng.below(5)];
                    old.begin_node(n);
                    new.begin_node(n);
                    depth += 1;
                }
                1 if depth > 1 => {
                    old.end_node();
                    new.end_node();
                    depth -= 1;
                }
                2 => {
                    let v = rng.next() as u32;
                    old.prop_u32(name, v);
                    new.prop_u32(name, v);
                }
                3 => {
                    let cells: Vec<u32> = (0..rng.below(20)).map(|_| rng.next() as u32).collect();
                    old.prop_u32_cells(name, &cells);
                    new.prop_u32_cells(name, &cells);
                }
                4 => {
                    let cells: Vec<u64> = (0..rng.below(9)).map(|_| rng.next()).collect();
                    old.prop_u64_cells(name, &cells);
                    new.prop_u64_cells(name, &cells);
                }
                5 => {
                    let s =
                        ["arm,psci-1.0", "smc", "serial0:115200n8", "okay\0x", ""][rng.below(5)];
                    old.prop_str(name, s);
                    new.prop_str(name, s);
                }
                _ => {
                    let v: Vec<u8> = (0..rng.below(13)).map(|_| rng.next() as u8).collect();
                    old.prop(name, &v);
                    new.prop(name, &v);
                }
            }
        }
        while depth > 0 {
            old.end_node();
            new.end_node();
            depth -= 1;
        }
        let (mut a, mut b) = ([0u8; 8192], [0u8; 8192]);
        let (na, nb) = (old.finish(&mut a), new.finish(&mut b));
        assert_eq!(na, nb);
        assert_eq!(a[..na], b[..nb]);
        assert!(
            fdt_checked::Fdt::new(&b[..nb]).is_ok(),
            "fdt-checked reads it"
        );
    }
}

#[test]
fn overflow_panics_like_the_board() {
    let r = std::panic::catch_unwind(|| {
        let mut f = fdt_writer::FdtWriter::<16, 16>::new();
        f.begin_node("a-node-name-longer-than-sixteen-bytes");
    });
    assert!(r.is_err(), "struct block overflow");
    let r = std::panic::catch_unwind(|| {
        let mut f = fdt_writer::Fdt::new();
        f.begin_node("");
        let mut out = [0u8; 8];
        f.finish(&mut out);
    });
    assert!(r.is_err(), "unbalanced nodes");
}
