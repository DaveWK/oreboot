// SPDX-License-Identifier: GPL-2.0-only
//! The crate with the K1 layout against the verbatim K1 board copy
//! (`legacy/k1_fit_checked.rs`): FIT plans of valid and mutated K1-style
//! metadata, payload verification, the extent helpers, and the memory fixup
//! of valid and mutated board DTs. Results and errors must be identical.

// The legacy file says `crate::fdt_checked` and `crate::sha256`.
pub use ::fdt_checked;
pub use ::sha256;

#[path = "legacy/k1_fit_checked.rs"]
mod legacy;

use fit::{Layout, MemoryLayout, Region, Reserved};

const K1: Layout = Layout {
    arch: b"riscv",
    max_fit: legacy::MAX_FIT,
    max_metadata: legacy::MAX_METADATA,
    data_base: Some(0x1000),
    firmware_load: 0,
    next_load: 0x20_0000,
    next_size: Some(legacy::EDK2_CODE_SIZE),
    reserved: Some(Reserved {
        node: b"nvram-reserved",
        offset: legacy::EDK2_CODE_SIZE,
        size: legacy::EDK2_FD_SIZE - legacy::EDK2_CODE_SIZE,
    }),
    next_slot: legacy::EDK2_FD_SIZE,
    dt_align: 0x10000,
    firmware_limit: 0x20_0000,
    payload_limit: 0x1000_0000,
};
const K1_MEMORY: MemoryLayout = MemoryLayout {
    low: b"memory@0",
    high: b"memory@100000000",
    low_base: 0,
    low_max: 0x8000_0000,
    high_base: 0x1_0000_0000,
    sizes: &[0x4000_0000, 0x8000_0000, 0x1_0000_0000, 0x2_0000_0000],
};

struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }
    fn below(&mut self, n: u64) -> u64 {
        self.next() % n
    }
}

/// A DT node for the builder: name, properties, children.
#[derive(Clone)]
struct Node {
    name: Vec<u8>,
    props: Vec<(String, Vec<u8>)>,
    kids: Vec<Node>,
}
fn n(name: &str, props: Vec<(&str, Vec<u8>)>, kids: Vec<Node>) -> Node {
    Node {
        name: name.as_bytes().to_vec(),
        props: props.into_iter().map(|(k, v)| (k.to_string(), v)).collect(),
        kids,
    }
}
fn s(v: &str) -> Vec<u8> {
    let mut b = v.as_bytes().to_vec();
    b.push(0);
    b
}
fn u32p(v: u32) -> Vec<u8> {
    v.to_be_bytes().to_vec()
}
fn u64p(v: u64) -> Vec<u8> {
    v.to_be_bytes().to_vec()
}

fn be(v: &mut Vec<u8>, x: u32) {
    v.extend(x.to_be_bytes());
}
fn pad(v: &mut Vec<u8>) {
    while !v.len().is_multiple_of(4) {
        v.push(0);
    }
}
fn emit(node: &Node, st: &mut Vec<u8>, strings: &mut Vec<u8>) {
    be(st, 1);
    st.extend(&node.name);
    st.push(0);
    pad(st);
    for (k, v) in &node.props {
        let off = match strings
            .windows(k.len() + 1)
            .position(|w| &w[..k.len()] == k.as_bytes() && w[k.len()] == 0)
        {
            Some(o) => o,
            None => {
                let o = strings.len();
                strings.extend(k.as_bytes());
                strings.push(0);
                o
            }
        };
        be(st, 3);
        be(st, v.len() as u32);
        be(st, off as u32);
        st.extend(v);
        pad(st);
    }
    for k in &node.kids {
        emit(k, st, strings);
    }
    be(st, 2);
}
/// FDT v17 blob of `root`, padded with zeros to `total` bytes if given.
fn dtb(root: &Node, total: Option<usize>) -> Vec<u8> {
    let mut st = Vec::new();
    let mut strings = Vec::new();
    emit(root, &mut st, &mut strings);
    be(&mut st, 9);
    let rsv = 40usize;
    let structure = rsv + 16;
    let strings_off = structure + st.len();
    let size = strings_off + strings.len();
    let total = total.unwrap_or(size).max(size);
    let mut b = Vec::new();
    for x in [
        0xd00d_feedu32,
        total as u32,
        structure as u32,
        strings_off as u32,
        rsv as u32,
        17,
        16,
        0,
        strings.len() as u32,
        st.len() as u32,
    ] {
        be(&mut b, x);
    }
    b.extend([0u8; 16]);
    b.extend(&st);
    b.extend(&strings);
    b.resize(total, 0);
    b
}

fn img(kind: &str, offset: u32, size: u32, load: Option<u32>, hash: &[u8; 32]) -> Node {
    let mut props = vec![
        ("type", s(kind)),
        ("compression", s("none")),
        ("data-offset", u32p(offset)),
        ("data-size", u32p(size)),
    ];
    if let Some(l) = load {
        props.push(("arch", s("riscv")));
        props.push(("load", u32p(l)));
        props.push(("entry", u32p(l)));
    }
    n(
        kind,
        props,
        vec![n(
            "hash-1",
            vec![("algo", s("sha256")), ("value", hash.to_vec())],
            vec![],
        )],
    )
}

/// A valid K1 FIT metadata blob (4 KiB, so data starts at 0x1000) and a
/// random mutation of one of its fields when `mutate` is set.
fn k1_fit(r: &mut Rng, mutate: bool) -> Vec<u8> {
    let code = legacy::EDK2_CODE_SIZE as u32;
    let fd = legacy::EDK2_FD_SIZE as u32;
    let fw_size = 0x4_0000 + (r.below(0x1000) as u32) * 4;
    let dt_size = 0x8000 + (r.below(0x100) as u32) * 4;
    let mut fw = img("firmware", fd, fw_size, Some(0), &[1; 32]);
    let mut next = img("standalone", 0, code, Some(0x20_0000), &[2; 32]);
    let mut dt = img("flat_dt", fd + fw_size, dt_size, None, &[3; 32]);
    let mut nv = n(
        "nvram-reserved",
        vec![
            ("type", s("firmware")),
            ("compression", s("none")),
            ("data-offset", u32p(code)),
            ("data-size", u32p(fd - code)),
        ],
        vec![],
    );
    fw.name = b"opensbi".to_vec();
    next.name = b"edk2".to_vec();
    dt.name = b"fdt-1".to_vec();
    let mut conf = n(
        "conf-1",
        vec![
            ("firmware", s("opensbi")),
            ("loadables", s("edk2")),
            ("fdt", s("fdt-1")),
        ],
        vec![],
    );
    let mut total = Some(0x1000);
    if mutate {
        let which = r.below(14);
        let target: &mut Node = match which % 4 {
            0 => &mut fw,
            1 => &mut next,
            2 => &mut dt,
            _ => &mut nv,
        };
        match which {
            0..=3 => {
                // a numeric field of the chosen node set to a random value
                let fields = ["data-offset", "data-size", "load", "entry"];
                let f = fields[r.below(4) as usize];
                let v = match r.below(4) {
                    0 => 0,
                    1 => r.next() as u32,
                    2 => 0x20_0000,
                    _ => 0x1000_0000 - 0x1000,
                };
                if let Some(p) = target.props.iter_mut().find(|p| p.0 == f) {
                    p.1 = u32p(v);
                } else {
                    target.props.push((f.into(), u32p(v)));
                }
            }
            4..=7 => {
                // a string field changed
                let fields = ["type", "compression", "arch"];
                let f = fields[r.below(3) as usize];
                if let Some(p) = target.props.iter_mut().find(|p| p.0 == f) {
                    p.1 = s("other");
                }
            }
            8 => target.props.push(("data".into(), vec![0; 4])),
            9 => {
                if let Some(h) = target.kids.first_mut() {
                    h.props[0].1 = s("crc32");
                }
            }
            10 => total = Some(0x2000),
            11 => conf.props[r.below(3) as usize].1 = s("missing"),
            12 => {
                target.props.retain(|p| p.0 != "data-size");
            }
            _ => {
                if let Some(h) = target.kids.first_mut() {
                    h.props[1].1 = vec![9; 31];
                }
            }
        }
    }
    let root = n(
        "",
        vec![("description", s("K1 FIT"))],
        vec![
            n("images", vec![], vec![fw, next, dt, nv]),
            n("configurations", vec![("default", s("conf-1"))], vec![conf]),
        ],
    );
    dtb(&root, total)
}

fn same_plan(a: &Result<legacy::Plan, legacy::Error>, b: &Result<fit::Plan, fit::Error>) {
    match (a, b) {
        (Ok(x), Ok(y)) => {
            assert_eq!((x.end, x.metadata), (y.end, y.metadata));
            for (i, j) in x.images.iter().zip(y.images.iter()) {
                assert_eq!(
                    (
                        i.data.start,
                        i.data.size,
                        i.destination.start,
                        i.destination.size,
                        i.hash
                    ),
                    (
                        j.data.start,
                        j.data.size,
                        j.destination.start,
                        j.destination.size,
                        j.hash
                    )
                );
            }
        }
        (Err(x), Err(y)) => assert_eq!(format!("{x:?}"), format!("{y:?}")),
        _ => panic!("legacy {a:?} vs crate {b:?}"),
    }
}

#[test]
fn plans_match_the_board_code() {
    let mut r = Rng(0x9e37_79b9_7f4a_7c15);
    let mut ok = 0;
    for i in 0..1500 {
        let meta = k1_fit(&mut r, i % 3 != 0);
        let staging_start = [0x1000_0000usize, 0x10_0000, 0x20_0000, 0][r.below(4) as usize];
        let staging_size = [legacy::MAX_FIT, 0x10_0000, 0x80_0000][r.below(3) as usize];
        let a = legacy::plan(
            &meta,
            legacy::Region {
                start: staging_start,
                size: staging_size,
            },
        );
        let b = fit::plan(
            &K1,
            &meta,
            Region {
                start: staging_start,
                size: staging_size,
            },
        );
        same_plan(&a, &b);
        ok += a.is_ok() as u32;
    }
    assert!(ok > 80, "too few valid plans ({ok}) to mean anything");
}

#[test]
fn verify_matches_the_board_code() {
    let mut r = Rng(42);
    let mut ok = 0;
    for i in 0..24 {
        // A valid plan with real hashes, then optionally corrupt one byte.
        let code = legacy::EDK2_CODE_SIZE as u32;
        let fd = legacy::EDK2_FD_SIZE as u32;
        let fw_data = vec![0x5a; 0x400];
        let next_data = vec![0x11; code as usize];
        let dt_root = n("", vec![("model", s("test"))], vec![]);
        let dt_data = dtb(&dt_root, None);
        let mut fw = img(
            "firmware",
            fd,
            fw_data.len() as u32,
            Some(0),
            &sha256::digest(&fw_data),
        );
        let mut next = img(
            "standalone",
            0,
            code,
            Some(0x20_0000),
            &sha256::digest(&next_data),
        );
        let mut dt = img(
            "flat_dt",
            fd + fw_data.len() as u32,
            dt_data.len() as u32,
            None,
            &sha256::digest(&dt_data),
        );
        fw.name = b"opensbi".to_vec();
        next.name = b"edk2".to_vec();
        dt.name = b"fdt-1".to_vec();
        let nv = n(
            "nvram-reserved",
            vec![
                ("type", s("firmware")),
                ("compression", s("none")),
                ("data-offset", u32p(code)),
                ("data-size", u32p(fd - code)),
            ],
            vec![],
        );
        let conf = n(
            "conf-1",
            vec![
                ("firmware", s("opensbi")),
                ("loadables", s("edk2")),
                ("fdt", s("fdt-1")),
            ],
            vec![],
        );
        let root = n(
            "",
            vec![],
            vec![
                n("images", vec![], vec![fw, next, dt, nv]),
                n("configurations", vec![("default", s("conf-1"))], vec![conf]),
            ],
        );
        let meta = dtb(&root, Some(0x1000));
        let mut bytes = meta.clone();
        bytes.resize(0x1000 + fd as usize, 0);
        bytes[0x1000..0x1000 + code as usize].copy_from_slice(&next_data);
        bytes.extend(&fw_data);
        bytes.extend(&dt_data);
        if i % 2 == 1 {
            let at = 0x1000 + r.below((bytes.len() - 0x1000) as u64) as usize;
            bytes[at] ^= 1 + r.below(255) as u8;
        }
        let staging = (0x1000_0000, legacy::MAX_FIT);
        let a = legacy::plan(
            &meta,
            legacy::Region {
                start: staging.0,
                size: staging.1,
            },
        )
        .unwrap();
        let b = fit::plan(
            &K1,
            &meta,
            Region {
                start: staging.0,
                size: staging.1,
            },
        )
        .unwrap();
        let va = a.verify(&bytes);
        let vb = b.verify(&bytes);
        assert_eq!(format!("{va:?}"), format!("{vb:?}"));
        ok += va.is_ok() as u32;
    }
    assert!(ok >= 12);
}

#[test]
fn extents_match_the_board_code() {
    let mut r = Rng(7);
    let edge = [
        0u64,
        1,
        511,
        512,
        8192,
        u32::MAX as u64,
        u32::MAX as u64 + 1,
        u64::MAX,
    ];
    for _ in 0..20_000 {
        let pick = |r: &mut Rng| {
            if r.below(3) == 0 {
                edge[r.below(edge.len() as u64) as usize]
            } else {
                r.below(1 << 34)
            }
        };
        let first = pick(&mut r) as u32;
        let sectors = pick(&mut r);
        let rel = pick(&mut r);
        let count = pick(&mut r) as usize;
        assert_eq!(
            format!("{:?}", legacy::read_extent(first, sectors, rel, count)),
            format!("{:?}", fit::read_extent(first, sectors, rel, count))
        );
        let found = if r.below(2) == 0 {
            None
        } else {
            Some((first, sectors))
        };
        let cap = pick(&mut r);
        let fb = pick(&mut r) as u32;
        assert_eq!(
            format!("{:?}", legacy::source_extent(found, cap, fb)),
            format!("{:?}", fit::source_extent(found, cap, fb))
        );
    }
}

fn board_dt(r: &mut Rng, mutate: bool) -> Vec<u8> {
    let mem = |name: &str| {
        n(
            name,
            vec![
                ("device_type", s("memory")),
                ("reg", [u64p(0), u64p(0)].concat()),
            ],
            vec![],
        )
    };
    let mut kids = vec![
        n("cpus", vec![], vec![]),
        mem("memory@0"),
        mem("memory@100000000"),
        n("soc", vec![("ranges", vec![])], vec![]),
    ];
    let mut props = vec![("#address-cells", u32p(2)), ("#size-cells", u32p(2))];
    if mutate {
        match r.below(6) {
            0 => props[0].1 = u32p(1),
            1 => kids.push(mem("memory@200000000")),
            2 => {
                kids.remove(2);
            }
            3 => kids[1].props[1].1 = u64p(0),
            4 => kids[2].props[0].1 = s("reserved"),
            _ => kids[1].name = b"memory@80000000".to_vec(),
        }
    }
    dtb(&n("", props, kids), None)
}

#[test]
fn memory_fixup_matches_the_board_code() {
    let mut r = Rng(99);
    let sizes = [
        0x4000_0000u64,
        0x8000_0000,
        0x1_0000_0000,
        0x2_0000_0000,
        0,
        0x6000_0000,
        0x4_0000_0000,
    ];
    for i in 0..1200 {
        let dt = board_dt(&mut r, i % 2 == 1);
        let size = sizes[r.below(sizes.len() as u64) as usize];
        let mut a = dt.clone();
        let mut b = dt.clone();
        let ra = legacy::fix_memory(&mut a, size);
        let rb = fit::fix_memory(&K1_MEMORY, &mut b, size);
        assert_eq!(format!("{ra:?}"), format!("{rb:?}"));
        assert_eq!(a, b, "the fixed DT bytes differ");
    }
}
