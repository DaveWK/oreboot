//! Minimal flattened device tree writer, enough to describe memory, PSCI
//! and the console to the payload.

const FDT_MAGIC: u32 = 0xd00d_feed;
const FDT_BEGIN_NODE: u32 = 1;
const FDT_END_NODE: u32 = 2;
const FDT_PROP: u32 = 3;
const FDT_END: u32 = 9;

const HEADER_SIZE: usize = 40;
const RSVMAP_SIZE: usize = 16; // one terminating empty entry

const STRUCT_MAX: usize = 4096;
const STRINGS_MAX: usize = 1024;

pub struct Fdt {
    dt_struct: [u8; STRUCT_MAX],
    struct_len: usize,
    strings: [u8; STRINGS_MAX],
    strings_len: usize,
    depth: usize,
}

impl Fdt {
    pub const fn new() -> Self {
        Self {
            dt_struct: [0; STRUCT_MAX],
            struct_len: 0,
            strings: [0; STRINGS_MAX],
            strings_len: 0,
            depth: 0,
        }
    }

    fn put(&mut self, bytes: &[u8]) {
        let end = self.struct_len + bytes.len();
        assert!(end <= STRUCT_MAX, "FDT struct block overflow");
        self.dt_struct[self.struct_len..end].copy_from_slice(bytes);
        self.struct_len = end;
        // keep 4-byte alignment
        while self.struct_len % 4 != 0 {
            self.dt_struct[self.struct_len] = 0;
            self.struct_len += 1;
        }
    }

    fn put_u32(&mut self, v: u32) {
        self.put(&v.to_be_bytes());
    }

    fn string_offset(&mut self, name: &str) -> u32 {
        let name = name.as_bytes();
        let mut off = 0;
        while off < self.strings_len {
            let end = off + self.strings[off..self.strings_len]
                .iter()
                .position(|&b| b == 0)
                .unwrap();
            if &self.strings[off..end] == name {
                return off as u32;
            }
            off = end + 1;
        }
        let start = self.strings_len;
        assert!(start + name.len() < STRINGS_MAX, "FDT strings block overflow");
        self.strings[start..start + name.len()].copy_from_slice(name);
        self.strings[start + name.len()] = 0;
        self.strings_len += name.len() + 1;
        start as u32
    }

    pub fn begin_node(&mut self, name: &str) {
        self.put_u32(FDT_BEGIN_NODE);
        let mut buf = [0u8; 64];
        buf[..name.len()].copy_from_slice(name.as_bytes());
        self.put(&buf[..name.len() + 1]);
        self.depth += 1;
    }

    pub fn end_node(&mut self) {
        assert!(self.depth > 0);
        self.put_u32(FDT_END_NODE);
        self.depth -= 1;
    }

    pub fn prop(&mut self, name: &str, value: &[u8]) {
        let nameoff = self.string_offset(name);
        self.put_u32(FDT_PROP);
        self.put_u32(value.len() as u32);
        self.put_u32(nameoff);
        if !value.is_empty() {
            self.put(value);
        }
    }

    pub fn prop_u32(&mut self, name: &str, v: u32) {
        self.prop(name, &v.to_be_bytes());
    }

    /// A list of cells, each u64 split into two big-endian u32 cells.
    pub fn prop_u64_cells(&mut self, name: &str, vals: &[u64]) {
        let mut buf = [0u8; 64];
        for (i, v) in vals.iter().enumerate() {
            buf[i * 8..i * 8 + 8].copy_from_slice(&v.to_be_bytes());
        }
        self.prop(name, &buf[..vals.len() * 8]);
    }

    /// A string or, with embedded `\0` separators, a string list.
    pub fn prop_str(&mut self, name: &str, s: &str) {
        let mut buf = [0u8; 128];
        buf[..s.len()].copy_from_slice(s.as_bytes());
        self.prop(name, &buf[..s.len() + 1]);
    }

    /// Serialize into `out`, returning the total size.
    pub fn finish(&mut self, out: &mut [u8]) -> usize {
        assert!(self.depth == 0, "unbalanced FDT nodes");
        self.put_u32(FDT_END);
        let off_rsvmap = HEADER_SIZE;
        let off_struct = off_rsvmap + RSVMAP_SIZE;
        let off_strings = off_struct + self.struct_len;
        let total = off_strings + self.strings_len;
        assert!(total <= out.len(), "FDT does not fit");

        let header = [
            FDT_MAGIC,
            total as u32,
            off_struct as u32,
            off_strings as u32,
            off_rsvmap as u32,
            17, // version
            16, // last compatible version
            0,  // boot CPU
            self.strings_len as u32,
            self.struct_len as u32,
        ];
        for (i, w) in header.iter().enumerate() {
            out[i * 4..i * 4 + 4].copy_from_slice(&w.to_be_bytes());
        }
        out[off_rsvmap..off_struct].fill(0);
        out[off_struct..off_strings].copy_from_slice(&self.dt_struct[..self.struct_len]);
        out[off_strings..total].copy_from_slice(&self.strings[..self.strings_len]);
        total
    }
}
