//! FEL lab knobs: the build-time `A7A_*` environment knobs of the open
//! driver and the hybrid/blobless lab stages, as one runtime value.
//!
//! [`BUILD`] is what the build's environment says, parsed exactly as the
//! knob consts did (same parsers, same panics on malformed values); every
//! consumer reads [`get`], which is [`BUILD`] unless the bt0 DRAM lab mode
//! (fastboot `oem dram-run`) installed overrides with `set` before the
//! DRAM init. Only feature `lab-knobs` (bt0 `dram-lab`) has the runtime
//! store and `set`; without it [`get`] and [`env`] are the build's values,
//! which fold to constants, so a build without the lab mode behaves and
//! compiles as before (the host-side reference checks see the same
//! values), and the lab mode can run any knob combination from one image.
//!
//! Runtime syntax ([`Knobs::apply`]): `NAME=value` with the build-time
//! names and value formats, or `-NAME` to unset. The end-of-initialization
//! variant is rebuilt from its four knobs (`A7A_HYBRID_EI_SKIP`,
//! `_EI_NO_MRW`, `_EI_MR16`, `A7A_EI_FSPWR0`) whenever one of them changes,
//! with [`crate::phy::endinit::Variant::parse`] as at build time.
use crate::phy::endinit::Variant;
use crate::phy::wcklevel::{parse_mr18_ops, Mr18Op};

/// Mode-register overrides of `A7A_BL_MRS` (`mr=hex` pairs).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MrList {
    pub n: usize,
    pub pairs: [(u8, u8); 24],
}

impl MrList {
    pub const EMPTY: MrList = MrList {
        n: 0,
        pairs: [(0, 0); 24],
    };

    /// As `A7A_BL_MRS` was read: comma-separated `mr=hex`, malformed pairs
    /// skipped (`strict`: refused).
    pub fn parse(spec: &str, strict: bool) -> Option<MrList> {
        let mut l = MrList::EMPTY;
        for pair in spec.split(',') {
            let mut it = pair.split('=');
            let parsed = match (it.next(), it.next()) {
                (Some(a), Some(v)) => match (a.parse::<u8>(), u8::from_str_radix(v, 16)) {
                    (Ok(a), Ok(v)) => Some((a, v)),
                    _ => None,
                },
                _ => None,
            };
            match parsed {
                Some(p) if l.n < l.pairs.len() => {
                    l.pairs[l.n] = p;
                    l.n += 1;
                }
                Some(_) | None if strict => return None,
                _ => {}
            }
        }
        Some(l)
    }

    pub fn iter(&self) -> impl Iterator<Item = &(u8, u8)> {
        self.pairs[..self.n].iter()
    }
}

/// The raw end-of-initialization knob strings (kept so a change to one
/// rebuilds the variant from all four, as at build time).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EiSpec {
    pub skip: Option<&'static str>,
    pub no_mrw: Option<&'static str>,
    pub mr16: Option<&'static str>,
    pub fspwr0: Option<&'static str>,
}

/// The later lab knobs, read at run time through
/// [`env`]: the lab mode's override when one was given (`NAME=value`, or
/// `-NAME` for unset), else the build's `option_env!` value. Each knob's
/// parser runs where it is used, as at build time (a malformed value
/// panics there; the lab mode turns a panic into a halted run).
pub const EXTRA: [&str; 34] = [
    "A7A_BL_BOOT",
    "A7A_CBT_CA_ADD",
    "A7A_EXEC_HOLD_US",
    "A7A_FETCH_PROBE",
    "A7A_FSP0_MRS",
    "A7A_HO_CDD",
    "A7A_HO_CSRS",
    "A7A_HO_DBICTL",
    "A7A_HO_EPILOGUE",
    "A7A_HO_FIFOINIT",
    "A7A_HO_FIU",
    "A7A_HO_MR2",
    "A7A_HO_MRL_ADD",
    "A7A_HO_MRS",
    "A7A_HO_PULSE_RESTORE",
    "A7A_HO_RDCSLAT",
    "A7A_HO_RDDATA_EN",
    "A7A_HO_RXEN_ADD",
    "A7A_HO_SRCYCLE",
    "A7A_HO_TXWCK_ADD",
    "A7A_HO_WCKON",
    "A7A_HYBRID_COLD",
    "A7A_NO_BOOST",
    "A7A_OBS_ARM",
    "A7A_RD_EXIT",
    "A7A_RD_FSPWR",
    "A7A_RD_GAPS",
    "A7A_RD_LANES",
    "A7A_RD_PULSE_RESTORE",
    "A7A_SYS_SNAP",
    "A7A_WL_TXWCK_ADD",
    "A7A_WR_APPLY",
    "A7A_WR_BLIND_CENTER",
    "A7A_WR_STAGE",
];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Knobs {
    /// Overrides of [`EXTRA`] (by index): None = the build's value.
    pub extra: [Option<Option<&'static str>>; EXTRA.len()],
    /// `A7A_OPEN_OPTS` (hex bit mask; bt0 `dram/open.rs`).
    pub open_opts: u32,
    /// `A7A_HYBRID_ENDINIT`, `_NO_WL`, `_SKIP_WL`, `_SKIP_RD` (set or not).
    pub hybrid_endinit: bool,
    pub hybrid_no_wl: bool,
    pub hybrid_skip_wl: bool,
    pub hybrid_skip_rd: bool,
    /// `A7A_BL_RELOCK` (set or not).
    pub bl_relock: bool,
    /// `A7A_RD_MRL` (decimal).
    pub rd_mrl: Option<u16>,
    /// `A7A_HYBRID_WL_MR18` (hex byte).
    pub wl_mr18: Option<u8>,
    /// `A7A_BL_MRS`; None when unset.
    pub bl_mrs: Option<MrList>,
    /// The end-of-initialization variant and its knob strings.
    pub ei: Variant,
    pub ei_spec: EiSpec,
    /// `A7A_BL_INIT0` (0..3).
    pub init0: Option<u32>,
    /// `A7A_HYBRID_WL_MR18_OPS`.
    pub wl_mr18_ops: [Mr18Op; 4],
}

const fn hex_u32(s: &str) -> Option<u32> {
    let b = s.as_bytes();
    if b.is_empty() {
        return None;
    }
    let mut v: u32 = 0;
    let mut i = 0;
    while i < b.len() {
        let d = match b[i] {
            c @ b'0'..=b'9' => c - b'0',
            c @ b'a'..=b'f' => c - b'a' + 10,
            c @ b'A'..=b'F' => c - b'A' + 10,
            _ => return None,
        };
        if v > (u32::MAX >> 4) {
            return None;
        }
        v = v * 16 + d as u32;
        i += 1;
    }
    Some(v)
}

const fn dec_u16(s: &str) -> Option<u16> {
    let b = s.as_bytes();
    if b.is_empty() {
        return None;
    }
    let mut v: u32 = 0;
    let mut i = 0;
    while i < b.len() {
        if b[i] < b'0' || b[i] > b'9' {
            return None;
        }
        v = v * 10 + (b[i] - b'0') as u32;
        if v > u16::MAX as u32 {
            return None;
        }
        i += 1;
    }
    Some(v as u16)
}

const fn ei_variant(e: &EiSpec) -> Option<Variant> {
    match Variant::parse(e.skip, e.no_mrw, e.mr16) {
        Some(v) => v.with_fspwr0(e.fspwr0),
        None => None,
    }
}

/// The build environment's knobs (the former consts, same panics).
pub const BUILD: Knobs = Knobs {
    extra: [None; EXTRA.len()],
    open_opts: match option_env!("A7A_OPEN_OPTS") {
        Some(s) => match hex_u32(s) {
            Some(v) => v,
            None => panic!("A7A_OPEN_OPTS is not a hex number"),
        },
        None => 0,
    },
    hybrid_endinit: option_env!("A7A_HYBRID_ENDINIT").is_some(),
    hybrid_no_wl: option_env!("A7A_HYBRID_NO_WL").is_some(),
    hybrid_skip_wl: option_env!("A7A_HYBRID_SKIP_WL").is_some(),
    hybrid_skip_rd: option_env!("A7A_HYBRID_SKIP_RD").is_some(),
    bl_relock: option_env!("A7A_BL_RELOCK").is_some(),
    rd_mrl: match option_env!("A7A_RD_MRL") {
        Some(s) => match dec_u16(s) {
            Some(v) => Some(v),
            None => panic!("A7A_RD_MRL is not a number"),
        },
        None => None,
    },
    wl_mr18: match option_env!("A7A_HYBRID_WL_MR18") {
        Some(s) => match hex_u32(s) {
            Some(v) if v <= 0xff => Some(v as u8),
            _ => panic!("A7A_HYBRID_WL_MR18 is not a hex byte"),
        },
        None => None,
    },
    // Parsed at first use (not const); see `bl_mrs()`.
    bl_mrs: None,
    ei: match ei_variant(&BUILD_EI) {
        Some(v) => v,
        None => panic!("A7A_HYBRID_EI_SKIP/_NO_MRW/_MR16/A7A_EI_FSPWR0 malformed"),
    },
    ei_spec: BUILD_EI,
    init0: match option_env!("A7A_BL_INIT0") {
        Some(s) => Some((s.as_bytes()[0] - b'0') as u32 & 3),
        None => None,
    },
    wl_mr18_ops: match parse_mr18_ops(option_env!("A7A_HYBRID_WL_MR18_OPS")) {
        Some(o) => o,
        None => panic!("A7A_HYBRID_WL_MR18_OPS malformed (four fields: empty, -, or hex)"),
    },
};

const BUILD_EI: EiSpec = EiSpec {
    skip: option_env!("A7A_HYBRID_EI_SKIP"),
    no_mrw: option_env!("A7A_HYBRID_EI_NO_MRW"),
    mr16: option_env!("A7A_HYBRID_EI_MR16"),
    fspwr0: option_env!("A7A_EI_FSPWR0"),
};

/// The build's `A7A_BL_MRS` text (lenient, as before).
pub const BUILD_BL_MRS: Option<&str> = option_env!("A7A_BL_MRS");

#[cfg(feature = "lab-knobs")]
static mut CURRENT: Option<Knobs> = None;

/// The knobs in force: the runtime ones if the lab mode set them, else the
/// build's (with `A7A_BL_MRS` parsed leniently, as it was).
#[cfg(feature = "lab-knobs")]
pub fn get() -> Knobs {
    // SAFETY: written only by the boot CPU before the DRAM init.
    if let Some(k) = unsafe { core::ptr::read_volatile(core::ptr::addr_of!(CURRENT)) } {
        return k;
    }
    build()
}

/// Without the lab mode: the build's knobs, which fold to constants at
/// every use (images without `dram-lab` keep the build-time code).
#[cfg(not(feature = "lab-knobs"))]
#[inline(always)]
pub fn get() -> Knobs {
    build()
}

/// [`BUILD`] with `A7A_BL_MRS` parsed leniently, as it was.
#[inline(always)]
fn build() -> Knobs {
    let mut k = BUILD;
    k.bl_mrs = BUILD_BL_MRS.and_then(|s| MrList::parse(s, false));
    k
}

/// An [`EXTRA`] knob's value: the runtime override if the lab mode set
/// one, else `build` (the caller's `option_env!`).
#[cfg(feature = "lab-knobs")]
pub fn env(name: &str, build: Option<&'static str>) -> Option<&'static str> {
    // SAFETY: written only by the boot CPU before the DRAM init.
    let cur = unsafe { &*core::ptr::addr_of!(CURRENT) };
    if let Some(k) = cur {
        if let Some(i) = EXTRA.iter().position(|n| *n == name) {
            if let Some(v) = k.extra[i] {
                return v;
            }
        }
    }
    build
}

/// Without the lab mode: the build's value (folds to a constant).
#[cfg(not(feature = "lab-knobs"))]
#[inline(always)]
pub const fn env(_name: &str, build: Option<&'static str>) -> Option<&'static str> {
    build
}

/// Install runtime knobs (bt0 lab mode, before the DRAM init).
#[cfg(feature = "lab-knobs")]
pub fn set(k: Knobs) {
    unsafe { core::ptr::write_volatile(core::ptr::addr_of_mut!(CURRENT), Some(k)) };
}

impl Knobs {
    /// One runtime knob, `NAME=value` or `-NAME`. Values must live as long
    /// as the knobs (the lab mode keeps its command text in SRAM).
    pub fn apply(&mut self, token: &'static str) -> Result<(), &'static str> {
        let (name, value) = match token.strip_prefix('-') {
            Some(n) => (n, None),
            None => match token.split_once('=') {
                Some((n, v)) => (n, Some(v)),
                None => return Err("knob: NAME=value or -NAME"),
            },
        };
        let flag = value.is_some();
        match name {
            "A7A_OPEN_OPTS" => {
                self.open_opts = match value {
                    Some(v) => hex_u32(v).ok_or("A7A_OPEN_OPTS: hex")?,
                    None => 0,
                }
            }
            "A7A_HYBRID_ENDINIT" => self.hybrid_endinit = flag,
            "A7A_HYBRID_NO_WL" => self.hybrid_no_wl = flag,
            "A7A_HYBRID_SKIP_WL" => self.hybrid_skip_wl = flag,
            "A7A_HYBRID_SKIP_RD" => self.hybrid_skip_rd = flag,
            "A7A_BL_RELOCK" => self.bl_relock = flag,
            "A7A_RD_MRL" => {
                self.rd_mrl = match value {
                    Some(v) => Some(dec_u16(v).ok_or("A7A_RD_MRL: decimal")?),
                    None => None,
                }
            }
            "A7A_HYBRID_WL_MR18" => {
                self.wl_mr18 = match value {
                    Some(v) => match hex_u32(v) {
                        Some(x) if x <= 0xff => Some(x as u8),
                        _ => return Err("A7A_HYBRID_WL_MR18: hex byte"),
                    },
                    None => None,
                }
            }
            "A7A_BL_MRS" => {
                self.bl_mrs = match value {
                    Some(v) => Some(MrList::parse(v, true).ok_or("A7A_BL_MRS: mr=hex,...")?),
                    None => None,
                }
            }
            "A7A_BL_INIT0" => {
                self.init0 = match value {
                    Some(v) if v.len() == 1 && (b'0'..=b'3').contains(&v.as_bytes()[0]) => {
                        Some((v.as_bytes()[0] - b'0') as u32)
                    }
                    Some(_) => return Err("A7A_BL_INIT0: 0..3"),
                    None => None,
                }
            }
            "A7A_HYBRID_WL_MR18_OPS" => {
                self.wl_mr18_ops =
                    parse_mr18_ops(value).ok_or("A7A_HYBRID_WL_MR18_OPS: four fields")?
            }
            "A7A_HYBRID_EI_SKIP"
            | "A7A_HYBRID_EI_NO_MRW"
            | "A7A_HYBRID_EI_MR16"
            | "A7A_EI_FSPWR0" => {
                let mut e = self.ei_spec;
                match name {
                    "A7A_HYBRID_EI_SKIP" => e.skip = value,
                    "A7A_HYBRID_EI_NO_MRW" => e.no_mrw = value,
                    "A7A_HYBRID_EI_MR16" => e.mr16 = value,
                    _ => e.fspwr0 = value,
                }
                self.ei = ei_variant(&e).ok_or("end-of-init knobs malformed or conflicting")?;
                self.ei_spec = e;
            }
            _ => match EXTRA.iter().position(|n| *n == name) {
                Some(i) => self.extra[i] = Some(value),
                None => return Err("unknown knob"),
            },
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn build_defaults_match_the_unset_environment() {
        // The host tests run without any A7A_* knob in the environment.
        let k = get();
        assert_eq!(k.open_opts, 0);
        assert!(!k.hybrid_endinit && !k.hybrid_skip_rd && !k.bl_relock);
        assert_eq!(
            (k.rd_mrl, k.wl_mr18, k.bl_mrs, k.init0),
            (None, None, None, None)
        );
        assert!(k.ei.is_reference());
        assert_eq!(k.wl_mr18_ops, [Mr18Op::Reference; 4]);
    }

    /// Every knob set of a lab run list (A7A_RUNS_TXT names the file; kept
    /// outside this tree), through `apply`, gives the variant the build path would.
    #[test]
    fn every_bisect_run_parses_at_runtime() {
        extern crate std;
        let Ok(path) = std::env::var("A7A_RUNS_TXT") else {
            return;
        };
        let text = std::fs::read_to_string(path).unwrap();
        let mut n = 0;
        for line in text
            .lines()
            .filter(|l| !l.starts_with('#') && !l.trim().is_empty())
        {
            let spec = line.split('|').next().unwrap();
            let mut k = BUILD;
            let mut e = (None, None, None, None);
            for tok in spec.split_whitespace().skip(1) {
                let tok: &'static str =
                    std::boxed::Box::leak(std::string::String::from(tok).into_boxed_str());
                match tok {
                    "-" | "BL" => {}
                    "EI" => {
                        k.apply("A7A_HYBRID_ENDINIT=1").unwrap();
                        k.apply(
                            "A7A_BL_MRS=18=00,2=00,3=06,20=00,10=00,28=00,41=00,13=00,22=04,19=20",
                        )
                        .unwrap();
                    }
                    t => {
                        k.apply(t).unwrap_or_else(|e| panic!("{line}: {t}: {e}"));
                        let (name, v) = t.split_once('=').unwrap();
                        match name {
                            "A7A_HYBRID_EI_SKIP" => e.0 = Some(v),
                            "A7A_HYBRID_EI_NO_MRW" => e.1 = Some(v),
                            "A7A_HYBRID_EI_MR16" => e.2 = Some(v),
                            "A7A_EI_FSPWR0" => e.3 = Some(v),
                            _ => {}
                        }
                    }
                }
            }
            let want = Variant::parse(e.0, e.1, e.2)
                .unwrap()
                .with_fspwr0(e.3)
                .unwrap();
            assert_eq!(k.ei, want, "{line}");
            n += 1;
        }
        assert!(n >= 20, "{n} runs");
    }

    #[test]
    fn runtime_knobs_parse_as_the_build_did() {
        let mut k = BUILD;
        k.apply("A7A_OPEN_OPTS=2de").unwrap();
        k.apply("A7A_HYBRID_SKIP_RD=1").unwrap();
        k.apply("A7A_HYBRID_ENDINIT=1").unwrap();
        k.apply("A7A_BL_MRS=18=00,2=00,3=06,20=00,10=00,28=00,41=00,13=00,22=04,19=20")
            .unwrap();
        k.apply("A7A_HYBRID_EI_SKIP=2b00").unwrap();
        assert_eq!(k.open_opts, 0x2de);
        assert!(k.hybrid_skip_rd && k.hybrid_endinit);
        assert_eq!(k.bl_mrs.unwrap().n, 10);
        assert_eq!(k.ei, Variant::parse(Some("2b00"), None, None).unwrap());
        k.apply("A7A_HYBRID_EI_NO_MRW=all").unwrap();
        assert_eq!(
            k.ei,
            Variant::parse(Some("2b00"), Some("all"), None).unwrap()
        );
        k.apply("A7A_HYBRID_EI_MR16=41,04").unwrap();
        // FSP-WR 0 fix cannot combine with a switch value: refused, unchanged.
        let before = k;
        assert!(k.apply("A7A_EI_FSPWR0=04").is_err());
        assert_eq!(k, before);
        k.apply("-A7A_HYBRID_EI_MR16").unwrap();
        k.apply("A7A_EI_FSPWR0=04").unwrap();
        k.apply("A7A_HYBRID_WL_MR18_OPS=,-,0c,").unwrap();
        assert_eq!(k.wl_mr18_ops[1], Mr18Op::Skip);
        k.apply("-A7A_HYBRID_SKIP_RD").unwrap();
        assert!(!k.hybrid_skip_rd);
        for bad in [
            "A7A_OPEN_OPTS=zz",
            "A7A_RD_MRL=x",
            "A7A_HYBRID_WL_MR18=100",
            "A7A_BL_MRS=2",
            "A7A_BL_INIT0=4",
            "A7A_HYBRID_WL_MR18_OPS=1,2",
            "A7A_NOPE=1",
            "A7A_OPEN_OPTS",
        ] {
            assert!(k.apply(bad).is_err(), "{bad}");
        }
    }
}
