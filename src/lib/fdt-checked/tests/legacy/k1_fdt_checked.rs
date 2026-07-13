// SPDX-License-Identifier: GPL-2.0-only
// Verbatim copy of ore-edk-boot-opi c91bbc4e40
// oreboot/src/mainboard/spacemit/k1x/bt0/src/fdt_checked.rs, the oracle for the
// equivalence tests.
#![allow(dead_code, clippy::all)]
//! Bounded FDT v17 reader for the supported FIT/board-DT contract. No unchecked
//! pointer walking: validate every block, string, token, depth and reservation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    Header,
    Bounds,
    Structure,
    Missing,
    Duplicate,
    Value,
}
pub type Result<T> = core::result::Result<T, Error>;
pub fn be32(b: &[u8], p: usize) -> Result<u32> {
    Ok(u32::from_be_bytes(
        b.get(p..p.checked_add(4).ok_or(Error::Bounds)?)
            .ok_or(Error::Bounds)?
            .try_into()
            .unwrap(),
    ))
}
fn end(start: usize, size: usize, cap: usize) -> Result<usize> {
    start
        .checked_add(size)
        .filter(|&n| n <= cap)
        .ok_or(Error::Bounds)
}
fn align4(n: usize) -> Result<usize> {
    Ok(n.checked_add(3).ok_or(Error::Bounds)? & !3)
}
fn overlap(a: (usize, usize), b: (usize, usize)) -> bool {
    a.0 < b.1 && b.0 < a.1
}
#[derive(Debug, Clone, Copy)]
pub struct Header {
    pub total: usize,
    pub structure: usize,
    pub structure_end: usize,
    pub strings: usize,
    pub strings_end: usize,
    reserve: usize,
}
impl Header {
    /// Safe to call on the first sector before performing a larger read.
    pub fn parse(b: &[u8], cap: usize) -> Result<Self> {
        if b.len() < 40 || be32(b, 0)? != 0xd00dfeed {
            return Err(Error::Header);
        }
        let total = be32(b, 4)? as usize;
        if total < 40 || total > cap || be32(b, 20)? != 17 || be32(b, 24)? > 17 {
            return Err(Error::Header);
        }
        let structure = be32(b, 8)? as usize;
        let strings = be32(b, 12)? as usize;
        let reserve = be32(b, 16)? as usize;
        let structure_end = end(structure, be32(b, 36)? as usize, total)?;
        let strings_end = end(strings, be32(b, 32)? as usize, total)?;
        if structure < 40
            || structure % 4 != 0
            || structure_end - structure < 16
            || strings < 40
            || reserve < 40
            || reserve % 8 != 0
            || end(reserve, 16, total).is_err()
            || overlap((structure, structure_end), (strings, strings_end))
        {
            return Err(Error::Header);
        }
        Ok(Self {
            total,
            structure,
            structure_end,
            strings,
            strings_end,
            reserve,
        })
    }
}
#[derive(Debug, Clone, Copy)]
pub enum Kind<'a> {
    Begin(&'a [u8]),
    End,
    Property(&'a [u8], &'a [u8]),
    Nop,
    Finish,
}
#[derive(Debug, Clone, Copy)]
pub struct Token<'a> {
    pub at: usize,
    pub end: usize,
    pub depth: usize,
    pub kind: Kind<'a>,
}
pub struct Fdt<'a> {
    b: &'a [u8],
    pub h: Header,
}
impl<'a> Fdt<'a> {
    pub fn new(b: &'a [u8]) -> Result<Self> {
        let h = Header::parse(b, b.len())?;
        let f = Self {
            b: &b[..h.total],
            h,
        };
        let mut p = h.reserve;
        let mut terminator = false;
        for _ in 0..64 {
            let q = end(p, 16, h.total)?;
            if overlap((p, q), (h.structure, h.structure_end))
                || overlap((p, q), (h.strings, h.strings_end))
            {
                return Err(Error::Header);
            }
            if f.b[p..q].iter().all(|&x| x == 0) {
                terminator = true;
                break;
            }
            // Address + size must not wrap.
            let a = u64::from_be_bytes(f.b[p..p + 8].try_into().unwrap());
            let n = u64::from_be_bytes(f.b[p + 8..q].try_into().unwrap());
            a.checked_add(n).ok_or(Error::Bounds)?;
            p = q;
        }
        if !terminator {
            return Err(Error::Header);
        }
        let mut it = f.tokens();
        let mut root = false;
        let mut closed = false;
        let mut finish = false;
        while let Some(t) = it.next()? {
            match t.kind {
                Kind::Begin(n) => {
                    if closed || (!root && (t.depth != 1 || !n.is_empty())) {
                        return Err(Error::Structure);
                    }
                    root = true;
                }
                Kind::End if t.depth == 1 => closed = true,
                Kind::Finish => {
                    if !root || !closed {
                        return Err(Error::Structure);
                    }
                    finish = true;
                }
                Kind::Property(..) if closed || !root => return Err(Error::Structure),
                _ => {}
            }
        }
        if !finish {
            return Err(Error::Structure);
        }
        Ok(f)
    }
    pub fn bytes(&self) -> &'a [u8] {
        self.b
    }
    pub fn tokens(&self) -> Tokens<'a> {
        Tokens {
            b: self.b,
            h: self.h,
            p: self.h.structure,
            depth: 0,
            finished: false,
        }
    }
    pub fn child(&self, parent: usize, name: &[u8]) -> Result<usize> {
        let mut it = self.tokens();
        let mut parent_depth = None;
        let mut found = None;
        while let Some(t) = it.next()? {
            if t.at == parent {
                if !matches!(t.kind, Kind::Begin(_)) {
                    return Err(Error::Structure);
                }
                parent_depth = Some(t.depth);
                continue;
            }
            if let Some(d) = parent_depth {
                if let Kind::Begin(n) = t.kind {
                    if t.depth == d + 1 && n == name {
                        if found.replace(t.at).is_some() {
                            return Err(Error::Duplicate);
                        }
                    }
                }
                if matches!(t.kind, Kind::End) && t.depth == d {
                    break;
                }
            }
        }
        found.ok_or(Error::Missing)
    }
    pub fn prop(&self, node: usize, name: &[u8]) -> Result<&'a [u8]> {
        let mut it = self.tokens();
        let mut depth = None;
        let mut value = None;
        while let Some(t) = it.next()? {
            if t.at == node {
                depth = Some(t.depth);
                continue;
            }
            if let Some(d) = depth {
                if let Kind::Property(n, v) = t.kind {
                    if t.depth == d && n == name {
                        if value.replace(v).is_some() {
                            return Err(Error::Duplicate);
                        }
                    }
                }
                if matches!(t.kind, Kind::End) && t.depth == d {
                    break;
                }
            }
        }
        value.ok_or(Error::Missing)
    }
    pub fn prop_offset(&self, node: usize, name: &[u8]) -> Result<usize> {
        Ok(self.prop(node, name)?.as_ptr() as usize - self.b.as_ptr() as usize)
    }
    pub fn node_end(&self, node: usize) -> Result<usize> {
        let mut it = self.tokens();
        let mut d = None;
        while let Some(t) = it.next()? {
            if t.at == node {
                d = Some(t.depth);
            } else if matches!(t.kind, Kind::End) && Some(t.depth) == d {
                return Ok(t.end);
            }
        }
        Err(Error::Structure)
    }
}
pub fn string(b: &[u8]) -> Result<&[u8]> {
    if b.is_empty() || b.last() != Some(&0) || b[..b.len() - 1].contains(&0) {
        return Err(Error::Value);
    }
    Ok(&b[..b.len() - 1])
}
pub fn number(b: &[u8]) -> Result<usize> {
    match b.len() {
        4 => Ok(u32::from_be_bytes(b.try_into().unwrap()) as usize),
        8 => usize::try_from(u64::from_be_bytes(b.try_into().unwrap())).map_err(|_| Error::Value),
        _ => Err(Error::Value),
    }
}
fn cstr(b: &[u8], start: usize, stop: usize) -> Result<(&[u8], usize)> {
    let tail = b.get(start..stop).ok_or(Error::Bounds)?;
    let len = tail.iter().position(|&x| x == 0).ok_or(Error::Structure)?;
    Ok((&tail[..len], start + len + 1))
}
pub struct Tokens<'a> {
    b: &'a [u8],
    h: Header,
    p: usize,
    depth: usize,
    finished: bool,
}
impl<'a> Tokens<'a> {
    pub fn next(&mut self) -> Result<Option<Token<'a>>> {
        if self.finished {
            return Ok(None);
        }
        let at = self.p;
        end(at, 4, self.h.structure_end)?;
        self.p += 4;
        let kind = match be32(self.b, at)? {
            1 => {
                if self.depth >= 32 {
                    return Err(Error::Structure);
                }
                let (n, q) = cstr(self.b, self.p, self.h.structure_end)?;
                self.p = align4(q)?;
                if self.p > self.h.structure_end {
                    return Err(Error::Bounds);
                }
                self.depth += 1;
                Kind::Begin(n)
            }
            2 => {
                if self.depth == 0 {
                    return Err(Error::Structure);
                }
                Kind::End
            }
            3 => {
                if self.depth == 0 {
                    return Err(Error::Structure);
                }
                end(self.p, 8, self.h.structure_end)?;
                let n = be32(self.b, self.p)? as usize;
                let name = be32(self.b, self.p + 4)? as usize;
                let (name, _) = cstr(
                    self.b,
                    end(self.h.strings, name, self.h.strings_end)?,
                    self.h.strings_end,
                )?;
                self.p += 8;
                let q = end(self.p, n, self.h.structure_end)?;
                let value = &self.b[self.p..q];
                self.p = align4(q)?;
                if self.p > self.h.structure_end {
                    return Err(Error::Bounds);
                }
                Kind::Property(name, value)
            }
            4 => Kind::Nop,
            9 => {
                if self.depth != 0 || self.p != self.h.structure_end {
                    return Err(Error::Structure);
                }
                self.finished = true;
                Kind::Finish
            }
            _ => return Err(Error::Structure),
        };
        let depth = self.depth;
        if matches!(kind, Kind::End) {
            self.depth -= 1;
        }
        Ok(Some(Token {
            at,
            end: self.p,
            depth,
            kind,
        }))
    }
}
