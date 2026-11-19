//! eMMC write targets and the arming rules every eMMC flash, erase and
//! boot-configuration change passes (`fastboot-emmc-write` builds only).
//!
//! It names targets and keeps the arm; it never talks to the card. In bt0
//! it is linked into the DRAM extension with [`crate::emmc`]: every caller
//! checks the extension first, except [`Guard::new`] and [`Guard::observe`],
//! which are `#[inline(always)]` so they stay in SRAM (they run on every
//! command, before the extension is known to be loaded).
//!
//! Targets (byte offsets in the eMMC user area unless named otherwise):
//!
//! | name | where | content check |
//! |---|---|---|
//! | `emmc-fw` | user area 0x20000..0x800000 (LBA 256..16383): the SD card's firmware gap minus the EDK2 variable store at 8-12 MiB | `firmware.bin`: eGON.BT0 for a medium at its start, the A7ABOOT1 frame at 0x1df000 into it, every image inside the target |
//! | `emmc-boot0`, `emmc-boot1` | boot partitions 1 and 2 (Linux `mmcblkXboot0/1`), whole | eGON.BT0 for a medium at offset 0 |
//! | `emmc@<hex off>+<hex len>` | any whole-block window of the user area that does not touch 0x20000..0x1000000 (the firmware gap) | none (raw: a GPT, an OS image) |
//!
//! Rules, in order (the NOR rules of [`crate::nor`], for the eMMC):
//! 1. compile time: bt0 feature `fastboot-emmc-write` ([`Guard::new`]);
//! 2. `oem emmc-arm <target> flash <hex len> <hex crc32>`, `<target> erase`
//!    or `bootcfg <hex PARTITION_CONFIG> <hex BOOT_BUS_CONDITIONS>`, valid
//!    [`ARM_TTL_MS`], for exactly the next such operation; only `getvar`
//!    and `download` keep it; the download must match length and CRC-32;
//! 3. the content check above and the card's write-protect state
//!    ([`crate::emmc`]), before the first write command.
use crate::fetch::split_window;
use crate::proto::{self, Command};

pub const BLOCK: u64 = 512;
/// The firmware gap of the SD card contract: 128 KiB to 16 MiB. Raw windows never touch it.
pub const GAP_START: u64 = 0x2_0000;
pub const GAP_END: u64 = 0x100_0000;
/// `emmc-fw`: the gap up to the EDK2 variable store (8 MiB).
pub const FW_START: u64 = GAP_START;
pub const FW_END: u64 = 0x80_0000;
/// The largest eGON boot0 accepted (the vendor SD boot0's length).
pub const EGON_MAX: u32 = 0x4_a000;

/// How long an `oem emmc-arm` stays valid.
pub const ARM_TTL_MS: u64 = 60_000;

/// The eMMC hardware partitions bt0 can address (PARTITION_ACCESS values).
/// RPMB (3) and the general purpose partitions (4-7) are never selected.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Part {
    User = 0,
    Boot1 = 1,
    Boot2 = 2,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Content {
    /// `firmware.bin`: eGON boot0, then the layout frame and its images.
    Firmware,
    /// A boot0 for the mask ROM alone.
    Egon,
    /// No check.
    Raw,
}

/// A write target: partition, first block and length in blocks (`blocks`
/// 0: the whole boot partition, sized from EXT_CSD at write time).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Target {
    pub part: Part,
    pub first: u64,
    pub blocks: u64,
    pub content: Content,
}

/// The target a name stands for.
pub fn target(name: &str) -> Result<Target, &'static str> {
    match name {
        "emmc-fw" => Ok(Target {
            part: Part::User,
            first: FW_START / BLOCK,
            blocks: (FW_END - FW_START) / BLOCK,
            content: Content::Firmware,
        }),
        "emmc-boot0" | "emmc-boot1" => Ok(Target {
            part: if name == "emmc-boot0" {
                Part::Boot1
            } else {
                Part::Boot2
            },
            first: 0,
            blocks: 0,
            content: Content::Egon,
        }),
        _ => {
            let (base, window) = split_window(name)?;
            let (off, len) = match (base, window) {
                ("emmc", Some(w)) => w,
                _ => return Err("not an eMMC write target (emmc-fw, emmc-boot0/1, emmc@off+len)"),
            };
            if !off.is_multiple_of(BLOCK) || !len.is_multiple_of(BLOCK) {
                return Err("eMMC window must be whole 512-byte blocks");
            }
            let end = off.checked_add(len).ok_or("eMMC window overflows")?;
            if off < GAP_END && end > GAP_START {
                return Err("window touches the firmware gap (use emmc-fw)");
            }
            Ok(Target {
                part: Part::User,
                first: off / BLOCK,
                blocks: len / BLOCK,
                content: Content::Raw,
            })
        }
    }
}

/// What an arm allows.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Op {
    Flash {
        len: u32,
        crc32: u32,
    },
    Erase,
    /// The two persistent (but rewritable) boot fields: PARTITION_CONFIG
    /// bits 6:3 (BOOT_ACK, BOOT_PARTITION_ENABLE) and BOOT_BUS_CONDITIONS.
    BootConfig {
        part_config: u8,
        boot_bus: u8,
    },
}

#[derive(Clone, Copy, Debug)]
struct Arm {
    target: Option<Target>,
    op: Op,
    at_ms: u64,
}

pub struct Guard {
    enabled: bool,
    arm: Option<Arm>,
}

const OFF: &str = "eMMC writes are not built in (bt0 feature fastboot-emmc-write)";

impl Guard {
    #[inline(always)]
    pub const fn new(enabled: bool) -> Self {
        Guard { enabled, arm: None }
    }

    pub fn enabled(&self) -> bool {
        self.enabled
    }

    pub fn armed(&self) -> Option<Op> {
        self.arm.map(|a| a.op)
    }

    /// Every command passes here first; `getvar` and `download` keep the
    /// arm, `flash`/`erase` use it up in [`Guard::take`], an `oem` command
    /// in [`Guard::take_boot_config`] (any other `oem` disarms).
    #[inline(always)]
    pub fn observe(&mut self, c: &Command) {
        match c {
            Command::Getvar(_) | Command::Download(_) | Command::Flash(_) | Command::Erase(_) => {}
            Command::Oem(a) if a.starts_with("emmc-bootcfg") => {}
            _ => self.arm = None,
        }
    }

    /// The words after `oem emmc-arm`.
    pub fn arm(&mut self, args: &str, now_ms: u64) -> Result<Op, &'static str> {
        self.arm = None;
        if !self.enabled {
            return Err(OFF);
        }
        let mut w = args.split(' ').filter(|s| !s.is_empty());
        let first = w.next().ok_or(USAGE)?;
        let (target, op) = if first == "bootcfg" {
            let (Some(pc), Some(bb), None) = (w.next(), w.next(), w.next()) else {
                return Err(USAGE);
            };
            let part_config = hex_u8(pc).ok_or("bad PARTITION_CONFIG")?;
            let boot_bus = hex_u8(bb).ok_or("bad BOOT_BUS_CONDITIONS")?;
            boot_config_ok(part_config, boot_bus)?;
            (
                None,
                Op::BootConfig {
                    part_config,
                    boot_bus,
                },
            )
        } else {
            let t = target(first)?;
            let op = match (w.next(), w.next(), w.next(), w.next()) {
                (Some("flash"), Some(len), Some(crc), None) => {
                    let len = proto::hex_u32(len).ok_or("bad length")?;
                    let crc32 = proto::hex_u32(crc).ok_or("bad CRC-32")?;
                    if len == 0 || (t.blocks != 0 && len as u64 > t.blocks * BLOCK) {
                        return Err("length does not fit the target");
                    }
                    if t.content == Content::Raw && len as u64 != t.blocks * BLOCK {
                        return Err("a window is written whole: length = window");
                    }
                    Op::Flash { len, crc32 }
                }
                (Some("erase"), None, None, None) => Op::Erase,
                _ => return Err(USAGE),
            };
            (Some(t), op)
        };
        self.arm = Some(Arm {
            target,
            op,
            at_ms: now_ms,
        });
        Ok(op)
    }

    fn take_arm(&mut self, now_ms: u64) -> Result<Arm, &'static str> {
        let arm = self.arm.take();
        if !self.enabled {
            return Err(OFF);
        }
        let arm = arm.ok_or("not armed (oem emmc-arm first)")?;
        if now_ms.saturating_sub(arm.at_ms) > ARM_TTL_MS {
            return Err("arm expired");
        }
        Ok(arm)
    }

    /// Use the arm up for `flash` (`image` Some) or `erase` (None) of
    /// `name`: Ok only for exactly the armed operation, in time.
    pub fn take(
        &mut self,
        name: &str,
        image: Option<&[u8]>,
        now_ms: u64,
    ) -> Result<Target, &'static str> {
        let arm = self.take_arm(now_ms)?;
        let t = target(name)?;
        if arm.target != Some(t) {
            return Err("armed for another target");
        }
        match (arm.op, image) {
            (Op::Erase, None) => {}
            (Op::Flash { len, crc32 }, Some(image)) => {
                if image.len() != len as usize || crate::crc32(image) != crc32 {
                    return Err("download does not match the armed length and CRC-32");
                }
            }
            _ => return Err("armed for another operation"),
        }
        Ok(t)
    }

    /// Use the arm up for `oem emmc-bootcfg <pc> <bb>`.
    pub fn take_boot_config(&mut self, args: &str, now_ms: u64) -> Result<(u8, u8), &'static str> {
        let arm = self.take_arm(now_ms)?;
        let mut w = args.split(' ').filter(|s| !s.is_empty());
        let (Some(pc), Some(bb), None) = (w.next(), w.next(), w.next()) else {
            return Err("usage: oem emmc-bootcfg <hex PARTITION_CONFIG> <hex BOOT_BUS_CONDITIONS>");
        };
        let want = (
            hex_u8(pc).ok_or("bad PARTITION_CONFIG")?,
            hex_u8(bb).ok_or("bad BOOT_BUS_CONDITIONS")?,
        );
        match arm.op {
            Op::BootConfig {
                part_config,
                boot_bus,
            } if (part_config, boot_bus) == want => Ok(want),
            _ => Err("armed for another operation"),
        }
    }
}

const USAGE: &str =
    "usage: oem emmc-arm <target> flash <len> <crc32> | <target> erase | bootcfg <pc> <bb>";

fn hex_u8(s: &str) -> Option<u8> {
    proto::hex_u32(s).and_then(|v| u8::try_from(v).ok())
}

/// The values `bootcfg` may write. PARTITION_CONFIG: bit 7 reserved,
/// BOOT_ACK (6), BOOT_PARTITION_ENABLE (5:3) none, boot 1, boot 2 or user,
/// and the access bits (2:0) 0 (they are the session's, not the arm's).
/// BOOT_BUS_CONDITIONS: bits 7:5 reserved, BOOT_MODE (4:3) 0-2,
/// RESET_BOOT_BUS_CONDITIONS (2), BOOT_BUS_WIDTH (1:0) x1, x4, x8.
pub fn boot_config_ok(part_config: u8, boot_bus: u8) -> Result<(), &'static str> {
    if part_config & 0x87 != 0 || !matches!((part_config >> 3) & 7, 0 | 1 | 2 | 7) {
        return Err("PARTITION_CONFIG: only BOOT_ACK and BOOT_PARTITION_ENABLE 0/1/2/7");
    }
    if boot_bus & 0xe0 != 0 || boot_bus & 3 == 3 || (boot_bus >> 3) & 3 == 3 {
        return Err("BOOT_BUS_CONDITIONS: reserved value");
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn targets() {
        let fw = target("emmc-fw").unwrap();
        assert_eq!((fw.part, fw.first, fw.blocks), (Part::User, 256, 16128));
        assert_eq!(target("emmc-boot0").unwrap().part, Part::Boot1);
        assert_eq!(target("emmc-boot1").unwrap().part, Part::Boot2);
        // Windows: whole blocks, outside the gap.
        let w = target("emmc@0+20000").unwrap();
        assert_eq!((w.first, w.blocks, w.content), (0, 256, Content::Raw));
        let w = target("emmc@1000000+4000000").unwrap();
        assert_eq!((w.first, w.blocks), (0x8000, 0x20000));
        for bad in [
            "emmc@0+20200",    // reaches into the gap
            "emmc@1f000+2000", // straddles its start
            "emmc@ffe00+400",  // inside
            "emmc@fffe00+400", // straddles its end
            "emmc@100+200",    // not block aligned
            "emmc@0+300",      // not whole blocks
            "emmc@ffffffffffffff00+200",
            "emmc", // the whole device: never a write target
            "emmc-boot2",
            "emmc-rpmb",
            "emmc-gp1",
            "nor",
            "sd@0+200",
        ] {
            assert!(target(bad).is_err(), "{bad}");
        }
    }

    #[test]
    fn arm_and_take() {
        let img = [7u8; 1024];
        let crc = crate::crc32(&img);
        let mut g = Guard::new(false);
        assert_eq!(g.arm("emmc-fw erase", 0), Err(OFF));
        assert_eq!(g.take("emmc-fw", None, 0), Err(OFF));
        let mut g = Guard::new(true);
        assert!(g.take("emmc-fw", None, 0).is_err(), "not armed");
        g.arm(&std::format!("emmc-fw flash 400 {crc:x}"), 0)
            .unwrap();
        g.observe(&Command::Getvar("max-download-size"));
        g.observe(&Command::Download(Some(0x400)));
        assert_eq!(
            g.take("emmc-fw", Some(&img), 1000),
            Ok(target("emmc-fw").unwrap())
        );
        // Used up.
        assert!(g.take("emmc-fw", Some(&img), 1000).is_err());
        // Wrong target, wrong CRC, wrong length, wrong op, expired.
        g.arm(&std::format!("emmc-fw flash 400 {crc:x}"), 0)
            .unwrap();
        assert_eq!(
            g.take("emmc-boot0", Some(&img), 0),
            Err("armed for another target")
        );
        g.arm(&std::format!("emmc-fw flash 400 {:x}", crc ^ 1), 0)
            .unwrap();
        assert!(g.take("emmc-fw", Some(&img), 0).is_err());
        g.arm(&std::format!("emmc-fw flash 401 {crc:x}"), 0)
            .unwrap();
        assert!(g.take("emmc-fw", Some(&img), 0).is_err());
        g.arm(&std::format!("emmc-fw flash 400 {crc:x}"), 0)
            .unwrap();
        assert_eq!(
            g.take("emmc-fw", None, 0),
            Err("armed for another operation")
        );
        g.arm("emmc-boot0 erase", 0).unwrap();
        assert_eq!(
            g.take("emmc-boot0", None, ARM_TTL_MS + 1),
            Err("arm expired")
        );
        // Any other command disarms.
        g.arm("emmc-boot0 erase", 0).unwrap();
        g.observe(&Command::Oem("emmc"));
        assert!(g.armed().is_none());
        g.arm("emmc-boot0 erase", 0).unwrap();
        g.observe(&Command::Boot);
        assert!(g.armed().is_none());
        // Bad arms leave nothing armed.
        for bad in [
            "",
            "emmc-fw",
            "emmc-fw flash",
            "emmc-fw flash 400",
            "emmc-fw flash 0 0",
            "emmc-fw flash 7e0001 0",
            "emmc@0+200 flash 100 0", // windows are written whole
            "emmc-fw erase now",
            "emmc@100000+200 erase",
            "bootcfg",
            "bootcfg 8",
            "bootcfg 9 0",  // access bits set
            "bootcfg 20 0", // BOOT_PARTITION_ENABLE 4
            "bootcfg 80 0", // reserved bit
            "bootcfg 8 3",  // bus width 3
            "bootcfg 8 18", // boot mode 3
            "bootcfg 8 20", // reserved
            "bootcfg 100 0",
        ] {
            g.arm("emmc-boot0 erase", 0).unwrap();
            assert!(g.arm(bad, 0).is_err(), "{bad}");
            assert!(g.armed().is_none(), "{bad}");
        }
        // Windows of the right length.
        g.arm(&std::format!("emmc@0+400 flash 400 {crc:x}"), 0)
            .unwrap();
        assert!(g.take("emmc@0+400", Some(&img), 0).is_ok());
    }

    #[test]
    fn boot_config_arm() {
        let mut g = Guard::new(true);
        g.arm("bootcfg 8 2", 0).unwrap();
        g.observe(&Command::Oem("emmc-bootcfg 8 2"));
        assert_eq!(g.take_boot_config("8 2", 10), Ok((8, 2)));
        assert!(g.take_boot_config("8 2", 10).is_err(), "used up");
        g.arm("bootcfg 8 2", 0).unwrap();
        assert!(g.take_boot_config("10 2", 10).is_err(), "other values");
        g.arm("emmc-fw erase", 0).unwrap();
        assert!(g.take_boot_config("8 2", 10).is_err(), "other op");
        g.arm("bootcfg 48 1a", 0).unwrap_err();
        g.arm("bootcfg 78 12", 0).unwrap();
        assert!(
            g.take("emmc-fw", None, 0).is_err(),
            "a bootcfg arm is no erase arm"
        );
        for (pc, bb) in [
            (0x00, 0x00),
            (0x08, 0x02),
            (0x10, 0x01),
            (0x38, 0x00),
            (0x48, 0x12),
        ] {
            assert!(boot_config_ok(pc, bb).is_ok(), "{pc:x} {bb:x}");
        }
    }
}
