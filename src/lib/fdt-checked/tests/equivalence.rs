// SPDX-License-Identifier: GPL-2.0-only
//! The crate against the verbatim K1 board copy: random FDT v17 blobs, valid
//! and mutated, through Header::parse, Fdt::new, the token walk and the
//! child/prop/node_end lookups.
#[path = "legacy/k1_fdt_checked.rs"]
mod legacy;

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

const NAMES: [&[u8]; 5] = [
    b"images",
    b"kernel",
    b"fdt-1",
    b"configurations",
    b"memory@40000000",
];
const PROPS: [&str; 5] = ["data", "description", "reg", "compatible", "hash"];

fn be(v: &mut Vec<u8>, x: u32) {
    v.extend(x.to_be_bytes());
}
fn pad(v: &mut Vec<u8>) {
    while !v.len().is_multiple_of(4) {
        v.push(0);
    }
}

fn node(r: &mut Rng, s: &mut Vec<u8>, strings: &mut Vec<u8>, depth: u32) {
    be(s, 1);
    s.extend(if depth == 0 {
        &b""[..]
    } else {
        NAMES[r.below(5) as usize]
    });
    s.push(0);
    pad(s);
    for _ in 0..r.below(4) {
        if r.below(6) == 0 {
            be(s, 4);
        }
        let name = PROPS[r.below(5) as usize];
        let off = match strings
            .windows(name.len() + 1)
            .position(|w| &w[..name.len()] == name.as_bytes() && w[name.len()] == 0)
        {
            Some(o) => o,
            None => {
                let o = strings.len();
                strings.extend(name.as_bytes());
                strings.push(0);
                o
            }
        };
        let len = r.below(24) as usize;
        be(s, 3);
        be(s, len as u32);
        be(s, off as u32);
        for _ in 0..len {
            s.push(r.next() as u8);
        }
        pad(s);
    }
    if depth < 3 {
        for _ in 0..r.below(3) {
            node(r, s, strings, depth + 1);
        }
    }
    be(s, 2);
}

fn blob(r: &mut Rng) -> Vec<u8> {
    let mut s = Vec::new();
    let mut strings = Vec::new();
    node(r, &mut s, &mut strings, 0);
    be(&mut s, 9);
    let rsv = 40usize;
    let structure = rsv + 16;
    let strings_off = structure + s.len();
    let total = strings_off + strings.len();
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
        s.len() as u32,
    ] {
        be(&mut b, x);
    }
    b.extend([0u8; 16]);
    b.extend(&s);
    b.extend(&strings);
    b
}

fn walk_new(b: &[u8]) -> String {
    let f = match fdt_checked::Fdt::new(b) {
        Ok(f) => f,
        Err(e) => return format!("new {e:?}"),
    };
    let mut out = format!("{:?}", f.h);
    let mut t = f.tokens();
    loop {
        match t.next() {
            Ok(Some(tok)) => out += &format!(" {tok:?}"),
            Ok(None) => break,
            Err(e) => {
                out += &format!(" err {e:?}");
                break;
            }
        }
    }
    for name in NAMES {
        out += &format!(" child {:?}", f.child(f.h.structure, name));
        if let Ok(n) = f.child(f.h.structure, name) {
            for p in PROPS {
                out += &format!(
                    " prop {:?} {:?}",
                    f.prop(n, p.as_bytes()),
                    f.prop_offset(n, p.as_bytes())
                );
            }
            out += &format!(" end {:?}", f.node_end(n));
        }
    }
    out
}

fn walk_old(b: &[u8]) -> String {
    let f = match legacy::Fdt::new(b) {
        Ok(f) => f,
        Err(e) => return format!("new {e:?}"),
    };
    let mut out = format!("{:?}", f.h);
    let mut t = f.tokens();
    loop {
        match t.next() {
            Ok(Some(tok)) => out += &format!(" {tok:?}"),
            Ok(None) => break,
            Err(e) => {
                out += &format!(" err {e:?}");
                break;
            }
        }
    }
    for name in NAMES {
        out += &format!(" child {:?}", f.child(f.h.structure, name));
        if let Ok(n) = f.child(f.h.structure, name) {
            for p in PROPS {
                out += &format!(
                    " prop {:?} {:?}",
                    f.prop(n, p.as_bytes()),
                    f.prop_offset(n, p.as_bytes())
                );
            }
            out += &format!(" end {:?}", f.node_end(n));
        }
    }
    out
}

#[test]
fn random_blobs_match_the_board_code() {
    let mut r = Rng(0xfd70_0000_0000_0001);
    let mut valid = 0;
    for _ in 0..5000 {
        let b = blob(&mut r);
        for cap in [b.len(), b.len() - 1, 1 << 20] {
            assert_eq!(
                format!("{:?}", fdt_checked::Header::parse(&b, cap)),
                format!("{:?}", legacy::Header::parse(&b, cap))
            );
        }
        let n = walk_new(&b);
        assert_eq!(n, walk_old(&b));
        valid += !n.starts_with("new ") as u32;
        let mut m = b.clone();
        let i = r.below(m.len() as u64) as usize;
        m[i] ^= 1 << r.below(8);
        assert_eq!(walk_new(&m), walk_old(&m));
        let cut = r.below(b.len() as u64) as usize;
        assert_eq!(walk_new(&b[..cut]), walk_old(&b[..cut]));
    }
    assert!(valid > 2500, "only {valid} blobs parsed");
    for v in [
        &b"ab\0"[..],
        b"",
        b"x",
        &[0, 0, 0, 7],
        &[1, 2, 3, 4, 5, 6, 7, 8],
        &[1, 2, 3],
    ] {
        assert_eq!(
            format!("{:?}", fdt_checked::string(v)),
            format!("{:?}", legacy::string(v))
        );
        assert_eq!(
            format!("{:?}", fdt_checked::number(v)),
            format!("{:?}", legacy::number(v))
        );
    }
}
