// SPDX-License-Identifier: GPL-2.0-only
//! The crate against verbatim copies of the K1 bt0 code it replaces.

use startup::{env, memory, riscv};

#[allow(dead_code, clippy::all)]
mod k1_startup {
    include!("legacy/k1_startup.rs");
}
#[allow(dead_code, clippy::all)]
mod k1_main {
    include!("legacy/k1_main_excerpt.rs");
    // Callable wrappers (the board's functions are private).
    pub fn eq(v: Option<&str>, want: &str) -> bool {
        env_eq(v, want)
    }
    pub fn u32(v: Option<&str>) -> u32 {
        env_u32(v)
    }
    pub fn vendor(v: usize) -> &'static str {
        vendorid_to_name(v)
    }
    pub fn implementation(v: usize) -> &'static str {
        impid_to_name(v)
    }
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

#[test]
fn initialize_matches_on_random_layouts() {
    let mut rng = Rng(0x5eed_57a7);
    for _ in 0..20_000 {
        // [bss][gap][data] in 256 bytes, the load image elsewhere or in place.
        let sbss = rng.below(64);
        let ebss = sbss + rng.below(64);
        let sdata = ebss + rng.below(32);
        let edata = sdata + rng.below(64);
        let in_place = rng.next() & 1 == 0;
        let fill: [u8; 64] = core::array::from_fn(|i| (i as u8) ^ (rng.next() as u8));
        let mut a = [0xa5u8; 256];
        let mut b = [0xa5u8; 256];
        unsafe {
            let (pa, pb) = (a.as_mut_ptr(), b.as_mut_ptr());
            if in_place {
                k1_startup::initialize(
                    pa.add(sbss),
                    pa.add(ebss),
                    pa.add(sdata),
                    pa.add(edata),
                    pa.add(sdata),
                );
                memory::initialize(
                    pb.add(sbss),
                    pb.add(ebss),
                    pb.add(sdata),
                    pb.add(edata),
                    pb.add(sdata),
                );
            } else {
                k1_startup::initialize(
                    pa.add(sbss),
                    pa.add(ebss),
                    pa.add(sdata),
                    pa.add(edata),
                    fill.as_ptr(),
                );
                memory::initialize(
                    pb.add(sbss),
                    pb.add(ebss),
                    pb.add(sdata),
                    pb.add(edata),
                    fill.as_ptr(),
                );
            }
        }
        assert_eq!(a, b);
    }
}

#[test]
fn env_parsers_match() {
    let mut rng = Rng(0x5eed_e1e1);
    let words = [
        "emmc", "sd", "nor", "", "emmc0", "EMMC", "8192", "0", "4294967",
    ];
    for w in words {
        for want in words {
            assert_eq!(env::eq(Some(w), want), k1_main::eq(Some(w), want));
        }
        assert_eq!(env::eq(None, w), k1_main::eq(None, w));
    }
    assert_eq!(env::u32(None), k1_main::u32(None));
    for _ in 0..100_000 {
        let n = rng.next() % 429_496_729;
        let s = n.to_string();
        assert_eq!(env::u32(Some(&s)), k1_main::u32(Some(&s)));
        assert_eq!(env::u32(Some(&s)) as u64, n);
    }
}

#[test]
fn riscv_names_match() {
    let mut rng = Rng(0x5eed_01d5);
    let known = [0usize, 0x0489, 0x0710, 0x0421_0427, 0x1000_0000_4977_2200];
    for v in known
        .into_iter()
        .chain((0..10_000).map(|_| rng.next() as usize))
    {
        assert_eq!(riscv::vendor_name(v), k1_main::vendor(v));
        assert_eq!(riscv::impl_name(v as u64), k1_main::implementation(v));
    }
    assert_eq!(riscv::cycles(), 0, "host");
}
