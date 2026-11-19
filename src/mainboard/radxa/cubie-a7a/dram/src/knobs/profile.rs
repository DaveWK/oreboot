// SPDX-License-Identifier: GPL-2.0-only
//! The settings of the blob-free boot, per DRAM clock.
//!
//! The firmware-free stages take their choices from build-time `A7A_*`
//! knobs (`option_env!`), which is how the lab images are built: one knob
//! line per image. A boot image must not depend on the build's
//! environment, so the knob lines the board runs validated are kept here,
//! one [`Profile`] per DRAM clock, and every knob is read through
//! [`value`] (macro [`crate::knob!`]): the environment's value when the
//! knob is set there, else the active profile's, else none. A profile is
//! therefore exactly the image its knob line builds, and a knob in the
//! environment still overrides it ([`UNSET`] turns one off).
//!
//! [`ACTIVE`] is the profile of this build. A build has one with feature
//! `boot-profile` (bt0's default feature `blobfree`). bt0's lab features
//! and its other DRAM backends are built without it: there
//! [`crate::knob!`] is `option_env!` itself, every knob comes from the
//! environment alone, and the crate compiles as it did before the
//! profiles existed. Which profile is build variable `A7A_PROFILE`: the
//! DRAM clock, `1200`, `2400`, `800` or `400`; unset: [`DEFAULT`]; `none`: no profile
//! in a build that would have one (the same code, the knobs from the
//! environment alone).
//!
//! What the knobs do is documented where each one is read; the summary
//! here is per profile.

/// One boot profile: the knob values and the one rail the knobs do not
/// cover.
// Nothing in this module is compiled into a build without a profile: the
// functions are `#[inline]` (instantiated where they are used, and they
// are used at compile time only) and the derives are the tests', so the
// images of such builds are those from before the module existed.
#[cfg_attr(test, derive(Debug, PartialEq, Eq))]
pub struct Profile {
    /// The DRAM clock (WCK) of the pstate the stages train, in MHz.
    pub mhz: u32,
    /// VDD2L (ELDO1) in mV. Every board run of the blob-free flow was a
    /// USB-loaded image, whose rails are the vendor library's (1080 mV,
    /// `crate::power`); the profile keeps that rail on every boot medium.
    pub vdd2l_mv: u32,
    /// `(knob, value)`, the values in the knobs' own text formats.
    pub knobs: &'static [(&'static str, &'static str)],
}

/// WCK 1200 MHz (2400 Mb/s, pstate 1), the clock the flow was built at:
///
/// - the end of initialization writes MR3 0x06 (DBI off) and MR18 0x00 and
///   leaves FSP-WR 0 (MR16 0x04);
/// - the read training passes on the eight DQ lanes (with read DBI off the
///   DMI lane carries no pattern), runs inside an FSP-WR = FSP-OP window
///   (MR16 0x05 before, 0x04 after), with the ACSM read pulses put back
///   to the training firmware's values first, and leaves MR18 0x1c (WCK
///   always on) and MR10 0x54;
/// - the hand-over applies the trained read latency plus one, makes the
///   training firmware's final DRAM state through the controller and
///   resets the RX FIFO pointers;
/// - the write stage finds a clean TxDqDly per DBYTE and centres the
///   lanes it can separate, every compare applying its delay writes;
/// - after a clean write stage the boot continues.
///
/// The write Vref stays at the parameter set's 0x2e (measured on the 4 GB
/// board: clean from 0x10 to 0x4a at this clock).
///
/// On the 4 GB board, loaded over USB FEL: Linux boots.
pub const BOOT_1200: Profile = Profile {
    mhz: 1200,
    vdd2l_mv: 1080,
    knobs: &[
        ("A7A_EI_FSPWR0", "04"),
        ("A7A_BL_MRS", "3=06,18=00"),
        ("A7A_RD_LANES", "ff"),
        ("A7A_RD_FSPWR", "05,04"),
        ("A7A_RD_EXIT", "1c,54"),
        ("A7A_HO_EPILOGUE", "1"),
        ("A7A_HO_FIFOINIT", "1"),
        ("A7A_RD_PULSE_RESTORE", "1"),
        ("A7A_WR_STAGE", "11"),
        ("A7A_WR_APPLY", "1"),
        ("A7A_HO_MRL_ADD", "1"),
        ("A7A_BL_BOOT", "1"),
    ],
};

/// WCK 2400 MHz (4800 Mb/s, pstate 0), the parameter sets' clock: as
/// [`BOOT_1200`] at pstate 0, with
///
/// - the write Vref (MR14 and MR15, every channel and rank) at 0x26, the
///   centre of the clean range measured on the 4 GB board with this
///   flow's write timing (0x1f to 0x2d; the parameter set's 0x2e fails);
/// - the write stage's clean start moved to the middle of its clean run
///   (at this clock the first clean delay sits within a few 1/64 UI of
///   the window's early edge).
///
/// On the 4 GB board, loaded over USB FEL: the bring-up, bt0's own DRAM
/// test and the host's memory tests pass and the firmware runs, but the
/// kernel then fails to decompress (rare data errors under load). Not a
/// boot clock yet; selectable for that work.
pub const BOOT_2400: Profile = Profile {
    mhz: 2400,
    vdd2l_mv: 1080,
    knobs: &[
        ("A7A_PSTATE", "0"),
        ("A7A_BL_MRS", "3=06,18=00,14=26,15=26"),
        ("A7A_EI_FSPWR0", "04"),
        ("A7A_RD_LANES", "ff"),
        ("A7A_RD_FSPWR", "05,04"),
        ("A7A_RD_EXIT", "1c,54"),
        ("A7A_HO_EPILOGUE", "1"),
        ("A7A_HO_FIFOINIT", "1"),
        ("A7A_RD_PULSE_RESTORE", "1"),
        ("A7A_WR_STAGE", "11"),
        ("A7A_WR_APPLY", "1"),
        ("A7A_HO_MRL_ADD", "1"),
        ("A7A_WR_BLIND_CENTER", "1"),
        ("A7A_BL_BOOT", "1"),
    ],
};

/// WCK 800 MHz (1600 Mb/s, pstate 2): [`BOOT_1200`]'s settings with the
/// stages generated for pstate 2. The write Vref stays at the parameter
/// set's 0x2e, as at 1200 MHz.
///
/// Not run on a board: the line is derived, not validated. Selectable for
/// that work.
pub const BOOT_800: Profile = Profile {
    mhz: 800,
    vdd2l_mv: 1080,
    knobs: &[
        ("A7A_PSTATE", "2"),
        ("A7A_EI_FSPWR0", "04"),
        ("A7A_BL_MRS", "3=06,18=00"),
        ("A7A_RD_LANES", "ff"),
        ("A7A_RD_FSPWR", "05,04"),
        ("A7A_RD_EXIT", "1c,54"),
        ("A7A_HO_EPILOGUE", "1"),
        ("A7A_HO_FIFOINIT", "1"),
        ("A7A_RD_PULSE_RESTORE", "1"),
        ("A7A_WR_STAGE", "11"),
        ("A7A_WR_APPLY", "1"),
        ("A7A_HO_MRL_ADD", "1"),
        ("A7A_BL_BOOT", "1"),
    ],
};

/// WCK 400 MHz (800 Mb/s, pstate 3), the lowest clock of the parameter
/// sets: as [`BOOT_800`] at pstate 3. Not run on a board.
pub const BOOT_400: Profile = Profile {
    mhz: 400,
    vdd2l_mv: 1080,
    knobs: &[
        ("A7A_PSTATE", "3"),
        ("A7A_EI_FSPWR0", "04"),
        ("A7A_BL_MRS", "3=06,18=00"),
        ("A7A_RD_LANES", "ff"),
        ("A7A_RD_FSPWR", "05,04"),
        ("A7A_RD_EXIT", "1c,54"),
        ("A7A_HO_EPILOGUE", "1"),
        ("A7A_HO_FIFOINIT", "1"),
        ("A7A_RD_PULSE_RESTORE", "1"),
        ("A7A_WR_STAGE", "11"),
        ("A7A_WR_APPLY", "1"),
        ("A7A_HO_MRL_ADD", "1"),
        ("A7A_BL_BOOT", "1"),
    ],
};

/// The profile of a build that names none (`A7A_PROFILE` unset). This
/// line is the default DRAM clock of the blob-free boot: the clock Linux
/// boots at.
pub const DEFAULT: &Profile = &BOOT_1200;

/// Whether this build has a boot profile at all (module docs).
pub const AVAILABLE: bool = cfg!(feature = "boot-profile");

/// The profile `choice` (the text of `A7A_PROFILE`) selects in a build
/// that has one (`available`); `Err` for a choice the build cannot honour.
#[inline]
pub const fn select(
    available: bool,
    choice: Option<&str>,
) -> Result<Option<&'static Profile>, &'static str> {
    let named = match choice {
        None if available => return Ok(Some(DEFAULT)),
        None => return Ok(None),
        Some(s) if same(s, "none") => return Ok(None),
        Some(s) if same(s, "1200") => &BOOT_1200,
        Some(s) if same(s, "2400") => &BOOT_2400,
        Some(s) if same(s, "800") => &BOOT_800,
        Some(s) if same(s, "400") => &BOOT_400,
        Some(_) => return Err("A7A_PROFILE is 1200, 2400, 800, 400 or none"),
    };
    if available {
        Ok(Some(named))
    } else {
        Err("A7A_PROFILE names a boot profile, but this build has none (feature boot-profile is off)")
    }
}

/// The profile in force, if any (module docs).
pub const ACTIVE: Option<&Profile> = match select(AVAILABLE, option_env!("A7A_PROFILE")) {
    Ok(p) => p,
    Err(why) => panic!("{}", why),
};

/// A knob set to this in the build's environment is unset, whatever the
/// profile says (`A7A_BL_BOOT=-`: the profile's bring-up, then the halt).
/// No knob takes it as a value. Only in a build with feature
/// `boot-profile`.
pub const UNSET: &str = "-";

#[inline]
const fn same(a: &str, b: &str) -> bool {
    let (a, b) = (a.as_bytes(), b.as_bytes());
    if a.len() != b.len() {
        return false;
    }
    let mut i = 0;
    while i < a.len() {
        if a[i] != b[i] {
            return false;
        }
        i += 1;
    }
    true
}

impl Profile {
    /// The profile's value of knob `name`.
    #[inline]
    pub const fn get(&self, name: &str) -> Option<&'static str> {
        let mut i = 0;
        while i < self.knobs.len() {
            if same(self.knobs[i].0, name) {
                return Some(self.knobs[i].1);
            }
            i += 1;
        }
        None
    }
}

/// Knob `name` with `env` in the build's environment and `profile` in
/// force: the environment wins, [`UNSET`] there is no value.
#[inline]
pub const fn resolve(
    profile: Option<&Profile>,
    name: &str,
    env: Option<&'static str>,
) -> Option<&'static str> {
    match env {
        Some(v) if same(v, UNSET) => None,
        Some(v) => Some(v),
        None => match profile {
            Some(p) => p.get(name),
            None => None,
        },
    }
}

/// Knob `name` of this build ([`resolve`] with [`ACTIVE`]); `env` is the
/// caller's `option_env!(name)`. Use [`crate::knob!`].
#[inline]
pub const fn value(name: &str, env: Option<&'static str>) -> Option<&'static str> {
    resolve(ACTIVE, name, env)
}

/// A build-time `A7A_*` knob: the environment's value, else the boot
/// profile's ([`value`]), evaluated at compile time, so a profile's
/// value compiles as the same knob set in the environment does.
#[cfg(feature = "boot-profile")]
#[macro_export]
macro_rules! knob {
    ($name:literal) => {
        const { $crate::knobs::profile::value($name, ::core::option_env!($name)) }
    };
}

/// A build-time `A7A_*` knob. This build has no boot profile: the
/// environment's value, as `option_env!` gives it.
#[cfg(not(feature = "boot-profile"))]
#[macro_export]
macro_rules! knob {
    ($name:literal) => {
        ::core::option_env!($name)
    };
}

#[cfg(test)]
mod tests {
    use super::*;

    extern crate std;
    use std::string::String;
    use std::vec::Vec;

    /// A profile as the knob line that builds the same image.
    fn line(p: &Profile) -> String {
        p.knobs
            .iter()
            .map(|(k, v)| std::format!("{k}={v}"))
            .collect::<Vec<_>>()
            .join(" ")
    }

    #[test]
    fn the_profiles_are_the_validated_knob_lines() {
        // The lines the board ran, each with the boot continuation
        // (A7A_BL_BOOT) the boot image needs.
        assert_eq!(
            line(&BOOT_2400),
            "A7A_PSTATE=0 A7A_BL_MRS=3=06,18=00,14=26,15=26 A7A_EI_FSPWR0=04 A7A_RD_LANES=ff \
             A7A_RD_FSPWR=05,04 A7A_RD_EXIT=1c,54 A7A_HO_EPILOGUE=1 A7A_HO_FIFOINIT=1 \
             A7A_RD_PULSE_RESTORE=1 A7A_WR_STAGE=11 A7A_WR_APPLY=1 A7A_HO_MRL_ADD=1 \
             A7A_WR_BLIND_CENTER=1 A7A_BL_BOOT=1"
        );
        assert_eq!(
            line(&BOOT_1200),
            "A7A_EI_FSPWR0=04 A7A_BL_MRS=3=06,18=00 A7A_RD_LANES=ff A7A_RD_FSPWR=05,04 \
             A7A_RD_EXIT=1c,54 A7A_HO_EPILOGUE=1 A7A_HO_FIFOINIT=1 A7A_RD_PULSE_RESTORE=1 \
             A7A_WR_STAGE=11 A7A_WR_APPLY=1 A7A_HO_MRL_ADD=1 A7A_BL_BOOT=1"
        );
        assert_eq!((BOOT_2400.mhz, BOOT_1200.mhz), (2400, 1200));
    }

    #[test]
    fn the_lower_clocks_are_the_1200_line_at_their_pstate() {
        // Derived lines, not validated ones: the 1200 MHz line with the
        // stages generated for the pstate.
        for (p, pstate) in [(&BOOT_800, "2"), (&BOOT_400, "3")] {
            assert_eq!(line(p), std::format!("A7A_PSTATE={pstate} {}", line(&BOOT_1200)));
            let ps: usize = pstate.parse().unwrap();
            assert_eq!(4 * crate::pstate::BOARD_DFI_MHZ[ps], p.mhz);
            assert_eq!(p.vdd2l_mv, BOOT_1200.vdd2l_mv);
        }
    }

    #[test]
    fn the_two_clocks_differ_in_the_pstate_the_write_vref_and_the_centred_start() {
        let only = |a: &Profile, b: &Profile| -> Vec<&'static str> {
            a.knobs
                .iter()
                .filter(|(k, v)| b.get(k) != Some(*v))
                .map(|(k, _)| *k)
                .collect()
        };
        assert_eq!(
            only(&BOOT_2400, &BOOT_1200),
            ["A7A_PSTATE", "A7A_BL_MRS", "A7A_WR_BLIND_CENTER"]
        );
        assert_eq!(only(&BOOT_1200, &BOOT_2400), ["A7A_BL_MRS"]);
        // The pstate each profile's stages are generated for.
        for p in [&BOOT_2400, &BOOT_1200] {
            let pstate = match p.get("A7A_PSTATE") {
                Some(s) => s.parse::<usize>().unwrap(),
                None => 1,
            };
            assert_eq!(4 * crate::pstate::BOARD_DFI_MHZ[pstate], p.mhz);
            assert_eq!(p.vdd2l_mv, 1080);
        }
    }

    #[test]
    fn every_profile_knob_is_a_knob_and_parses() {
        for p in [&BOOT_2400, &BOOT_1200, &BOOT_800, &BOOT_400] {
            let mut seen: Vec<&str> = Vec::new();
            let mut k = crate::knobs::BUILD;
            for &(name, v) in p.knobs {
                assert!(!seen.contains(&name), "{name} twice");
                seen.push(name);
                assert_ne!(v, UNSET);
                if name == "A7A_PSTATE" {
                    // Build-time only (the stage generators follow it).
                    assert!(matches!(v, "0" | "1" | "2" | "3"));
                    continue;
                }
                let tok: &'static str =
                    std::boxed::Box::leak(std::format!("{name}={v}").into_boxed_str());
                k.apply(tok).unwrap_or_else(|e| panic!("{name}={v}: {e}"));
            }
            // The fields with their own parsers.
            let mrs = k.bl_mrs.unwrap();
            assert_eq!(mrs.n, if p.mhz == 2400 { 4 } else { 2 });
            assert_eq!(&mrs.pairs[..2], &[(3, 0x06), (18, 0x00)]);
            if p.mhz == 2400 {
                assert_eq!(&mrs.pairs[2..4], &[(14, 0x26), (15, 0x26)]);
            }
            assert!(!k.ei.is_reference());
        }
    }

    #[test]
    fn the_environment_overrides_a_profile_and_a_dash_unsets() {
        let p = Some(&BOOT_2400);
        assert_eq!(resolve(p, "A7A_HO_MRL_ADD", None), Some("1"));
        assert_eq!(resolve(p, "A7A_HO_MRL_ADD", Some("2")), Some("2"));
        assert_eq!(resolve(p, "A7A_BL_BOOT", Some(UNSET)), None);
        assert_eq!(resolve(p, "A7A_HO_CDD", None), None);
        assert_eq!(resolve(p, "A7A_HO_CDD", Some("1")), Some("1"));
        // No profile: the environment alone, as the lab images are built.
        assert_eq!(resolve(None, "A7A_HO_MRL_ADD", None), None);
        assert_eq!(resolve(None, "A7A_HO_MRL_ADD", Some("1")), Some("1"));
        assert_eq!(resolve(None, "A7A_BL_BOOT", Some(UNSET)), None);
        // A name is matched whole.
        assert_eq!(resolve(p, "A7A_BL", None), None);
        assert_eq!(resolve(p, "A7A_BL_BOOT_", None), None);
    }

    #[test]
    fn the_build_variable_selects_the_profile() {
        assert_eq!(select(true, None), Ok(Some(DEFAULT)));
        assert_eq!(select(true, Some("1200")), Ok(Some(&BOOT_1200)));
        assert_eq!(select(true, Some("2400")), Ok(Some(&BOOT_2400)));
        assert_eq!(select(true, Some("800")), Ok(Some(&BOOT_800)));
        assert_eq!(select(true, Some("400")), Ok(Some(&BOOT_400)));
        assert_eq!(select(true, Some("none")), Ok(None));
        assert!(select(true, Some("1800")).is_err());
        assert!(select(true, Some("")).is_err());
        // A build without a profile (a lab feature, another backend, this
        // crate alone): nothing to select, and a named clock is refused.
        assert_eq!(select(false, None), Ok(None));
        assert_eq!(select(false, Some("none")), Ok(None));
        assert!(select(false, Some("1200")).is_err());
        assert!(select(false, Some("2400")).is_err());
        assert!(select(false, Some("800")).is_err());
        assert!(select(false, Some("400")).is_err());
    }

    #[test]
    fn the_host_tests_run_without_a_profile() {
        // The unit tests are written against the environment-only values.
        assert!(!AVAILABLE);
        assert_eq!(ACTIVE, None);
    }
}
