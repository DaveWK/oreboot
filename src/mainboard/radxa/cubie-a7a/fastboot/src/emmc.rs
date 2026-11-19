// SPDX-License-Identifier: GPL-2.0-only

//! eMMC (JEDEC 4.4 to 5.1) over an abstract host controller ([`MmcBus`]):
//! module detect, identification, EXT_CSD, partition selection, block
//! reads, and (write builds only) guarded block writes and the two
//! rewritable boot-configuration fields. Used by bt0's eMMC boot path
//! (feature `emmc`) and by fastboot (`fetch:emmc*`, `oem emmc`, and with
//! `fastboot-emmc-write` `flash`/`erase:emmc-*`, `oem emmc-bootcfg`).
//!
//! Read-only by construction outside the write builds: every command goes
//! through [`cmd`] (and bt0's controller checks again), which refuses
//! anything [`command_allowed`] does not list. The read path sends CMD0
//! (GO_IDLE only: no pre-idle or boot initiation argument), 1, 2, 3, 7, 9,
//! 13, the EXT_CSD read (8), block reads (17, 18, 12 as the controller's
//! auto stop) and CMD6 SWITCH limited to two *volatile* session fields:
//!
//! - PARTITION_CONFIG's PARTITION_ACCESS bits (2:0) to user, boot 1 or
//!   boot 2, written the way Linux does (`mmc_blk_part_switch`): the byte
//!   as read from EXT_CSD with only those bits changed, so BOOT_ACK and
//!   BOOT_PARTITION_ENABLE keep their values; [`Card::switch`] refuses any
//!   CMD6 to this byte that would change them;
//! - BUS_WIDTH 1, 4 or 8 bit SDR (no DDR, no enhanced strobe).
//!
//! Both reset to user / 1-bit at power-up, hardware reset and CMD0 (the eMMC
//! standard's "E_P" fields). Nothing else is ever switched: no HS_TIMING (the bus stays
//! at legacy speed), and in particular no field that is one-time
//! programmable or irreversible ([`NEVER`]: partitioning, enhanced area,
//! RST_n, reliable write, permanent/power-on write protection, boot config
//! protection, FFU, sanitize, ...), in any build.
//!
//! The write builds (`write` true) add CMD24/25 and, through
//! [`Card::boot_config`] only, PARTITION_CONFIG bits 6:3 and
//! BOOT_BUS_CONDITIONS: persistent but rewritable (R/W/E) fields. Erase is
//! done by writing zeros: no CMD35/36/38 (erase groups, TRIM, discard,
//! secure erase) in any build.
use crate::egon;
use crate::emmcarm::Part;
#[cfg(any(test, feature = "emmc-session"))]
use crate::emmcarm::{Content, Target, BLOCK as BLOCK64};
use crate::emmcarm::{EGON_MAX, FW_END, FW_START};
#[cfg(any(test, feature = "emmc-session"))]
use crate::proto::Info;
use crate::sd::{bits, BusError, Resp};

pub const BLOCK: usize = 512;
pub const WORDS: usize = BLOCK / 4;
/// Blocks per CMD17/18/24/25, bounding a single transfer.
pub const MAX_BLOCKS: usize = 128;

/// CMD1 argument: sector mode (bit 30), 2.7-3.6 V (bits 23:15) and
/// 1.70-1.95 V (bit 7). The A7A's module gets VCC 3.3 V (VCC-eMMC: the
/// PMIC's SWOUT2/DC1SW2, shared with VCC-CARD) and VCCQ 1.8 V (VCC-PC,
/// CLDO5) (schematic sheets 4 and 9).
pub const OCR_ARG: u32 = 0x40ff_8080;
/// The relative card address the host assigns (CMD3).
pub const RCA: u32 = 1;

/// EXT_CSD byte indices (JEDEC eMMC 5.1, the Extended CSD register).
pub mod ext_csd {
    pub const PARTITION_SETTING_COMPLETED: usize = 155;
    pub const PARTITIONS_ATTRIBUTE: usize = 156;
    pub const RST_N_FUNCTION: usize = 162;
    pub const USER_WP: usize = 171;
    pub const BOOT_WP: usize = 173;
    pub const BOOT_WP_STATUS: usize = 174;
    pub const BOOT_BUS_CONDITIONS: usize = 177;
    pub const BOOT_CONFIG_PROT: usize = 178;
    pub const PARTITION_CONFIG: usize = 179;
    pub const BUS_WIDTH: usize = 183;
    pub const HS_TIMING: usize = 185;
    pub const EXT_CSD_REV: usize = 192;
    pub const DEVICE_TYPE: usize = 196;
    pub const SEC_COUNT: usize = 212;
    pub const BOOT_SIZE_MULT: usize = 226;
    pub const BOOT_INFO: usize = 228;
}

/// EXT_CSD fields no build ever writes, with the reason. The allow-list
/// already refuses every index but 177, 179 and 183; this list pins the
/// irreversible ones in the tests.
pub const NEVER: [(usize, &str); 34] = [
    (16, "SECURE_REMOVAL_TYPE"),
    (17, "PRODUCT_STATE_AWARENESS_ENABLEMENT"),
    (29, "MODE_OPERATION_CODES (field firmware update)"),
    (30, "MODE_CONFIG (field firmware update mode)"),
    (52, "EXT_PARTITIONS_ATTRIBUTE (one-time)"),
    (53, "EXT_PARTITIONS_ATTRIBUTE (one-time)"),
    (134, "SEC_BAD_BLK_MGMNT"),
    (136, "ENH_START_ADDR (one-time)"),
    (137, "ENH_START_ADDR (one-time)"),
    (138, "ENH_START_ADDR (one-time)"),
    (139, "ENH_START_ADDR (one-time)"),
    (140, "ENH_SIZE_MULT (one-time)"),
    (141, "ENH_SIZE_MULT (one-time)"),
    (142, "ENH_SIZE_MULT (one-time)"),
    (143, "GP_SIZE_MULT (one-time)"),
    (146, "GP_SIZE_MULT (one-time)"),
    (149, "GP_SIZE_MULT (one-time)"),
    (152, "GP_SIZE_MULT (one-time)"),
    (154, "GP_SIZE_MULT (one-time)"),
    (155, "PARTITION_SETTING_COMPLETED (one-time)"),
    (156, "PARTITIONS_ATTRIBUTE (one-time)"),
    (162, "RST_n_FUNCTION (one-time)"),
    (163, "BKOPS_EN (MANUAL_EN one-time)"),
    (164, "BKOPS_START"),
    (165, "SANITIZE_START (destroys unmapped data)"),
    (167, "WR_REL_SET (one-time)"),
    (169, "FW_CONFIG (update disable, one-time)"),
    (171, "USER_WP (permanent / power-on write protection)"),
    (173, "BOOT_WP (permanent / power-on write protection)"),
    (175, "ERASE_GROUP_DEF"),
    (178, "BOOT_CONFIG_PROT (permanent boot config protection)"),
    (185, "HS_TIMING (not used: legacy speed)"),
    (187, "POWER_CLASS"),
    (191, "CMD_SET"),
];

/// R1 card status: error bits (JEDEC eMMC 5.1 device status; CARD_IS_LOCKED and
/// ERASE_RESET are states, not errors), SWITCH_ERROR (7), READY_FOR_DATA
/// (8), CURRENT_STATE (12:9).
pub const R1_ERRORS: u32 = 0xfdf9_8080;
pub const R1_SWITCH_ERROR: u32 = 1 << 7;
pub const R1_READY: u32 = 1 << 8;
pub const STATE_TRAN: u32 = 4;
pub const STATE_PRG: u32 = 7;

/// A CMD6 SWITCH this module can send (access mode Write Byte only).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Switch {
    /// EXT_CSD[179]; [`Card::switch`] allows only the access bits to
    /// change, [`Card::boot_config`] (write builds) bits 6:3 too.
    PartitionConfig(u8),
    /// EXT_CSD[183]: 0, 1, 2 = 1, 4, 8-bit SDR.
    BusWidth(u8),
    /// EXT_CSD[177], write builds only, through [`Card::boot_config`].
    BootBus(u8),
}

impl Switch {
    pub fn arg(self) -> u32 {
        let (index, value) = match self {
            Switch::PartitionConfig(v) => (ext_csd::PARTITION_CONFIG, v),
            Switch::BusWidth(v) => (ext_csd::BUS_WIDTH, v),
            Switch::BootBus(v) => (ext_csd::BOOT_BUS_CONDITIONS, v),
        };
        (3 << 24) | ((index as u32) << 16) | ((value as u32) << 8)
    }
}

/// The CMD6 argument as a [`Switch`] this build may send, or None.
pub fn decode_switch(arg: u32, write: bool) -> Option<Switch> {
    // [31:26] 0, [25:24] access = 3 (Write Byte), [7:0] 0 (command set).
    if arg >> 24 != 3 || arg & 0xff != 0 {
        return None;
    }
    let index = ((arg >> 16) & 0xff) as usize;
    let value = (arg >> 8) as u8;
    match index {
        ext_csd::PARTITION_CONFIG
            if value & 0x80 == 0 && value & 7 <= 2 && matches!((value >> 3) & 7, 0 | 1 | 2 | 7) =>
        {
            Some(Switch::PartitionConfig(value))
        }
        ext_csd::BUS_WIDTH if value <= 2 => Some(Switch::BusWidth(value)),
        ext_csd::BOOT_BUS_CONDITIONS
            if write && value & 0xe0 == 0 && value & 3 != 3 && (value >> 3) & 3 != 3 =>
        {
            Some(Switch::BootBus(value))
        }
        _ => None,
    }
}

/// The bus-level allow-list (bt0's controller checks it on every command,
/// whatever called it): index and argument shape. `write`: the
/// `fastboot-emmc-write` build.
pub fn command_allowed(idx: u32, arg: u32, write: bool) -> bool {
    match idx {
        0 => arg == 0,
        8 => arg == 0,
        1 | 2 | 3 | 7 | 9 | 12 | 13 | 17 | 18 => true,
        6 => decode_switch(arg, write).is_some(),
        24 | 25 => write,
        _ => false,
    }
}

/// The host controller, as this module needs it.
pub trait MmcBus {
    /// Is a module fitted? Some(true/false) when the board can tell (the
    /// A7A: the BOOT-SEL GPADC reading), None when it cannot.
    fn present(&mut self) -> Option<bool>;
    /// Reset the controller and clock the card at the identification
    /// (<= 400 kHz, `ident`) or transfer rate; the bus is 1-bit after
    /// `ident`.
    fn clock(&mut self, ident: bool) -> Result<(), BusError>;
    /// The controller's bus width (1, 4 or 8); the card is switched first.
    fn set_width(&mut self, width: u32) -> Result<(), BusError>;
    fn cmd(&mut self, idx: u32, arg: u32, resp: Resp) -> Result<[u32; 4], BusError>;
    /// A read command (8, 17, 18) with its data, `buf.len() / 128` blocks.
    fn read(&mut self, idx: u32, arg: u32, buf: &mut [u32]) -> Result<(), BusError>;
    /// A write command (24, 25) with its data. Only the write builds
    /// implement it.
    fn write(&mut self, _idx: u32, _arg: u32, _buf: &[u32]) -> Result<(), BusError> {
        Err(BusError::Error)
    }
    fn delay_ms(&mut self, _ms: u32) {}
    /// Keep the watchdog quiet during long transfers.
    fn kick(&mut self) {}
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    /// No module (detect), or no answer to CMD1.
    NoCard,
    Refused(u32),
    RefusedSwitch(u32),
    Bus(u32, BusError),
    /// An R1 error or a state other than expected (the status word).
    Status(u32, u32),
    Unsupported(&'static str),
    Range,
    /// A write guard refused; nothing was written.
    Guard(&'static str),
    /// Read-back mismatch at this block of the partition.
    Verify(u64),
}

impl Error {
    pub fn message(&self) -> &'static str {
        match self {
            Error::NoCard => "no eMMC module",
            Error::Refused(_) | Error::RefusedSwitch(_) => "eMMC command not allowed",
            Error::Bus(..) => "eMMC command failed (see the console)",
            Error::Status(..) => "eMMC status error (see the console)",
            Error::Unsupported(w) | Error::Guard(w) => w,
            Error::Range => "past the end of the eMMC partition",
            Error::Verify(_) => "eMMC read-back mismatch (see the console)",
        }
    }
}

/// Every command passes here (`write`: the build's write flag).
fn cmd<B: MmcBus>(
    b: &mut B,
    write: bool,
    idx: u32,
    arg: u32,
    resp: Resp,
) -> Result<[u32; 4], Error> {
    if !command_allowed(idx, arg, write) {
        return Err(if idx == 6 {
            Error::RefusedSwitch(arg)
        } else {
            Error::Refused(idx)
        });
    }
    b.cmd(idx, arg, resp).map_err(|e| Error::Bus(idx, e))
}

fn byte(e: &[u32; WORDS], i: usize) -> u8 {
    e[i / 4].to_le_bytes()[i % 4]
}

/// The EXT_CSD fields the boot and the board plan look at.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Fields {
    pub rev: u8,
    pub device_type: u8,
    pub sec_count: u32,
    pub boot_size_mult: u8,
    pub boot_info: u8,
    pub part_config: u8,
    pub boot_bus: u8,
    pub boot_config_prot: u8,
    pub boot_wp: u8,
    pub boot_wp_status: u8,
    pub user_wp: u8,
    pub rst_n: u8,
    pub partition_setting_completed: u8,
    pub partitions_attribute: u8,
    pub bus_width: u8,
    pub hs_timing: u8,
}

impl Fields {
    pub fn parse(e: &[u32; WORDS]) -> Fields {
        use ext_csd::*;
        Fields {
            rev: byte(e, EXT_CSD_REV),
            device_type: byte(e, DEVICE_TYPE),
            sec_count: e[SEC_COUNT / 4],
            boot_size_mult: byte(e, BOOT_SIZE_MULT),
            boot_info: byte(e, BOOT_INFO),
            part_config: byte(e, PARTITION_CONFIG),
            boot_bus: byte(e, BOOT_BUS_CONDITIONS),
            boot_config_prot: byte(e, BOOT_CONFIG_PROT),
            boot_wp: byte(e, BOOT_WP),
            boot_wp_status: byte(e, BOOT_WP_STATUS),
            user_wp: byte(e, USER_WP),
            rst_n: byte(e, RST_N_FUNCTION),
            partition_setting_completed: byte(e, PARTITION_SETTING_COMPLETED),
            partitions_attribute: byte(e, PARTITIONS_ATTRIBUTE),
            bus_width: byte(e, BUS_WIDTH),
            hs_timing: byte(e, HS_TIMING),
        }
    }
}

/// Capacity of a byte-addressed (<= 2 GB) card from its CSD.
fn csd_blocks(csd: &[u32; 4]) -> u64 {
    let c_size = bits(csd, 73, 62) as u64;
    let mult = bits(csd, 49, 47);
    let bl_len = bits(csd, 83, 80);
    ((c_size + 1) << (mult + 2) << bl_len) / BLOCK as u64
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Card {
    pub ocr: u32,
    pub cid: [u32; 4],
    pub csd: [u32; 4],
    pub sector_mode: bool,
    pub width: u32,
    pub user_blocks: u64,
    /// Blocks in each boot partition (0: none).
    pub boot_blocks: u64,
    pub fields: Fields,
    #[cfg(not(any(test, feature = "emmc-session")))]
    part: Part,
    /// The selected partition (PARTITION_ACCESS), unknown after an ambiguous CMD6.
    #[cfg(any(test, feature = "emmc-session"))]
    part: Option<Part>,
    /// Write commands allowed (the `fastboot-emmc-write` build).
    pub write: bool,
}

impl Card {
    /// Module detect, identification into the transfer state, EXT_CSD
    /// (into `ext`), then the widest of `want_width` (8, 4) or 1 bits that
    /// reads EXT_CSD back identically (a read-only bus test). `write`: the
    /// build may write (only [`Card::program`], [`Card::erase`] and
    /// [`Card::boot_config`] use it).
    pub fn identify<B: MmcBus>(
        b: &mut B,
        ext: &mut [u32; WORDS],
        want_width: u32,
        write: bool,
    ) -> Result<Card, Error> {
        if b.present() == Some(false) {
            return Err(Error::NoCard);
        }
        b.clock(true).map_err(|e| Error::Bus(0, e))?;
        cmd(b, write, 0, 0, Resp::None)?;
        let mut ocr = 0;
        for i in 0..1000 {
            ocr = match cmd(b, write, 1, OCR_ARG, Resp::R3) {
                Ok(r) => r[0],
                Err(Error::Bus(_, BusError::Timeout)) if i == 0 => return Err(Error::NoCard),
                Err(e) => return Err(e),
            };
            if ocr & (1 << 31) != 0 {
                break;
            }
            b.delay_ms(1);
        }
        if ocr & (1 << 31) == 0 {
            return Err(Error::Unsupported("eMMC stays busy (CMD1)"));
        }
        let sector_mode = (ocr >> 29) & 3 == 2;
        let cid = cmd(b, write, 2, 0, Resp::R2)?;
        cmd(b, write, 3, RCA << 16, Resp::R1)?;
        let csd = cmd(b, write, 9, RCA << 16, Resp::R2)?;
        cmd(b, write, 7, RCA << 16, Resp::R1b)?;
        b.clock(false).map_err(|e| Error::Bus(0, e))?;
        read_ext_csd(b, write, ext)?;
        let fields = Fields::parse(ext);
        let user_blocks = if sector_mode {
            fields.sec_count as u64
        } else {
            csd_blocks(&csd)
        };
        let mut card = Card {
            ocr,
            cid,
            csd,
            sector_mode,
            width: 1,
            user_blocks,
            boot_blocks: fields.boot_size_mult as u64 * 256,
            fields,
            #[cfg(not(any(test, feature = "emmc-session")))]
            part: Part::User,
            #[cfg(any(test, feature = "emmc-session"))]
            part: Some(Part::User),
            write,
        };
        if fields.part_config & 7 != 0 {
            // CMD0 resets the access bits; a card that kept them is odd.
            return Err(Error::Unsupported("eMMC not in the user area after CMD0"));
        }
        for width in [8u32, 4] {
            if width > want_width {
                continue;
            }
            if card.try_width(b, ext, width)? {
                break;
            }
        }
        Ok(card)
    }

    /// Switch to `width` and compare a second EXT_CSD read with `ext`;
    /// back to 1 bit on any mismatch. Ok(true) if the wide bus works.
    /// BUS_WIDTH [183] itself is W/E_P in the eMMC standard (written, not
    /// readable):
    /// a device may return 0 or the written code there, and both are
    /// accepted; every other byte must read back identical. Comparing
    /// [183] with the written code kept every JEDEC-conforming module on
    /// 1 bit (seen with a SanDisk iNAND). Linux (mmc_compare_ext_csds)
    /// and U-Boot compare a few read-only fields only; this stays stricter.
    fn try_width<B: MmcBus>(
        &mut self,
        b: &mut B,
        ext: &mut [u32; WORDS],
        width: u32,
    ) -> Result<bool, Error> {
        let code = if width == 8 { 2 } else { 1 };
        let mut again = [0u32; WORDS];
        let ok = self.switch(b, Switch::BusWidth(code)).is_ok()
            && b.set_width(width).is_ok()
            && read_ext_csd(b, self.write, &mut again).is_ok()
            && {
                // BUS_WIDTH reads 0 or the code; everything else must match.
                let got = byte(&again, ext_csd::BUS_WIDTH);
                let mut want = *ext;
                set_byte(&mut want, ext_csd::BUS_WIDTH, got);
                (got == 0 || got == code) && want == again
            };
        if ok {
            self.width = width;
            *ext = again;
            self.fields = Fields::parse(ext);
            return Ok(true);
        }
        b.set_width(1).map_err(|e| Error::Bus(0, e))?;
        // The card may or may not have switched: put it back to 1 bit.
        let _ = self.switch(b, Switch::BusWidth(0));
        read_ext_csd(b, self.write, ext)?;
        #[cfg(not(any(test, feature = "emmc-session")))]
        {
            self.fields = Fields::parse(ext);
        }
        #[cfg(any(test, feature = "emmc-session"))]
        self.update_ext(ext);
        Ok(false)
    }

    /// Send a volatile CMD6 and check the status after it.
    pub fn switch<B: MmcBus>(&mut self, b: &mut B, s: Switch) -> Result<(), Error> {
        match s {
            // Only the access bits may change here.
            Switch::PartitionConfig(v) if v & 0x78 == self.fields.part_config & 0x78 => {}
            Switch::BusWidth(_) => {}
            _ => return Err(Error::RefusedSwitch(s.arg())),
        }
        self.send_switch(b, s)
    }

    fn send_switch<B: MmcBus>(&mut self, b: &mut B, s: Switch) -> Result<(), Error> {
        #[cfg(any(test, feature = "emmc-session"))]
        let part = self.part.take();
        let response = cmd(b, self.write, 6, s.arg(), Resp::R1b)?[0];
        if cfg!(any(test, feature = "emmc-session")) && response & R1_ERRORS != 0 {
            return Err(Error::Status(6, response));
        }
        let st = self.status(b)?;
        if st & R1_SWITCH_ERROR != 0 {
            return Err(Error::Status(6, st));
        }
        match s {
            Switch::PartitionConfig(v) => {
                self.fields.part_config = v;
                #[cfg(any(test, feature = "emmc-session"))]
                {
                    self.part = access(v);
                }
            }
            Switch::BusWidth(v) => self.fields.bus_width = v,
            Switch::BootBus(v) => self.fields.boot_bus = v,
        }
        #[cfg(any(test, feature = "emmc-session"))]
        if !matches!(s, Switch::PartitionConfig(_)) {
            self.part = part;
        }
        Ok(())
    }

    /// CMD13: Ok(status) in the transfer state without error bits.
    pub fn status<B: MmcBus>(&self, b: &mut B) -> Result<u32, Error> {
        for _ in 0..1000 {
            let st = cmd(b, self.write, 13, RCA << 16, Resp::R1)?[0];
            if st & R1_ERRORS & !R1_SWITCH_ERROR != 0 {
                return Err(Error::Status(13, st));
            }
            match (st >> 9) & 0xf {
                STATE_TRAN if st & R1_READY != 0 => return Ok(st),
                STATE_PRG | STATE_TRAN => b.delay_ms(1),
                _ => return Err(Error::Status(13, st)),
            }
        }
        Err(Error::Unsupported("eMMC stays busy (CMD13)"))
    }

    pub fn blocks(&self, part: Part) -> u64 {
        match part {
            Part::User => self.user_blocks,
            Part::Boot1 | Part::Boot2 => self.boot_blocks,
        }
    }

    /// Select `part` (PARTITION_ACCESS), keeping every other bit.
    pub fn select<B: MmcBus>(&mut self, b: &mut B, part: Part) -> Result<(), Error> {
        #[cfg(any(test, feature = "emmc-session"))]
        if self.part.is_none() {
            self.refresh(b, &mut [0; WORDS])?;
        }
        #[cfg(any(test, feature = "emmc-session"))]
        let selected = self.part == Some(part);
        #[cfg(not(any(test, feature = "emmc-session")))]
        let selected = self.part == part;
        if selected {
            return Ok(());
        }
        if self.blocks(part) == 0 {
            return Err(Error::Unsupported("the eMMC has no boot partitions"));
        }
        let v = (self.fields.part_config & !7) | part as u8;
        self.switch(b, Switch::PartitionConfig(v))?;
        #[cfg(not(any(test, feature = "emmc-session")))]
        {
            self.part = part;
        }
        Ok(())
    }

    fn arg(&self, lba: u64) -> Result<u32, Error> {
        let a = if self.sector_mode {
            lba
        } else {
            lba * BLOCK as u64
        };
        u32::try_from(a).map_err(|_| Error::Range)
    }

    /// Read `buf.len() / 128` blocks of `part` from `lba`.
    pub fn read<B: MmcBus>(
        &mut self,
        b: &mut B,
        part: Part,
        lba: u64,
        buf: &mut [u32],
    ) -> Result<(), Error> {
        if !buf.len().is_multiple_of(WORDS) {
            return Err(Error::Unsupported("partial block"));
        }
        let n = (buf.len() / WORDS) as u64;
        if lba.checked_add(n).is_none_or(|e| e > self.blocks(part)) {
            return Err(Error::Range);
        }
        self.select(b, part)?;
        let mut lba = lba;
        for chunk in buf.chunks_mut(WORDS * MAX_BLOCKS) {
            let blocks = (chunk.len() / WORDS) as u64;
            let idx = if blocks > 1 { 18 } else { 17 };
            let arg = self.arg(lba)?;
            if !command_allowed(idx, arg, self.write) {
                return Err(Error::Refused(idx));
            }
            b.read(idx, arg, chunk).map_err(|e| Error::Bus(idx, e))?;
            b.kick();
            lba += blocks;
        }
        Ok(())
    }

    /// Re-read EXT_CSD and reconcile the cached fields and access bits, without
    /// changing the selected partition.
    #[cfg(any(test, feature = "emmc-session"))]
    pub fn refresh<B: MmcBus>(&mut self, b: &mut B, ext: &mut [u32; WORDS]) -> Result<(), Error> {
        self.part = None;
        read_ext_csd(b, self.write, ext)?;
        self.update_ext(ext);
        Ok(())
    }

    #[cfg(any(test, feature = "emmc-session"))]
    fn update_ext(&mut self, ext: &[u32; WORDS]) {
        self.fields = Fields::parse(ext);
        self.part = access(self.fields.part_config);
    }

    /// Refresh before trusting the cached selection, then confirm a deliberate
    /// switch with another EXT_CSD read. After ambiguity or unexpected drift,
    /// do not switch and write as part of the same destructive command.
    #[cfg(any(test, feature = "emmc-session"))]
    fn select_for_write<B: MmcBus>(
        &mut self,
        b: &mut B,
        part: Part,
        ext: &mut [u32; WORDS],
    ) -> Result<(), Error> {
        if !self.write {
            return Err(Error::Guard("eMMC writes are not built in"));
        }
        let cached = self.part;
        self.refresh(b, ext)?;
        if let Some(why) = self.protected(part) {
            return Err(Error::Guard(why));
        }
        if self.part != Some(part) && self.part.is_some() && self.part == cached {
            self.select(b, part)?;
            self.refresh(b, ext)?;
            if let Some(why) = self.protected(part) {
                return Err(Error::Guard(why));
            }
        }
        if self.part != Some(part) {
            return Err(Error::Guard(
                "eMMC partition access not confirmed; not touched",
            ));
        }
        Ok(())
    }

    /// The write protection that would refuse writes to `part`, if any.
    pub fn protected(&self, part: Part) -> Option<&'static str> {
        let f = &self.fields;
        // CSD PERM_WRITE_PROTECT (13), TMP_WRITE_PROTECT (12).
        if bits(&self.csd, 13, 12) != 0 {
            return Some("eMMC CSD write protection set; not touched");
        }
        match part {
            // US_PERM_WP_EN (2), US_PWR_WP_EN (0) and their disables.
            Part::User if f.user_wp & 0x05 != 0 => {
                Some("eMMC user area write protection enabled; not touched")
            }
            Part::Boot1 if f.boot_wp_status & 0x03 != 0 || f.boot_wp & 0x05 != 0 => {
                Some("eMMC boot partition 1 is write protected; not touched")
            }
            Part::Boot2 if f.boot_wp_status & 0x0c != 0 || f.boot_wp & 0x05 != 0 => {
                Some("eMMC boot partition 2 is write protected; not touched")
            }
            _ => None,
        }
    }

    /// Write `blocks` blocks from `words` at `lba` of the selected
    /// partition; every call is checked against `t` once more.
    #[cfg(any(test, feature = "emmc-session"))]
    fn write_blocks<B: MmcBus>(
        &mut self,
        b: &mut B,
        t: &Target,
        t_blocks: u64,
        lba: u64,
        words: &[u32],
    ) -> Result<(), Error> {
        let n = (words.len() / WORDS) as u64;
        if !self.write
            || n == 0
            || n > MAX_BLOCKS as u64
            || self.part != Some(t.part)
            || lba < t.first
            || lba + n > t.first + t_blocks
            || lba + n > self.blocks(t.part)
        {
            return Err(Error::Guard("eMMC write outside the target"));
        }
        let idx = if n > 1 { 25 } else { 24 };
        let arg = self.arg(lba)?;
        if !command_allowed(idx, arg, self.write) {
            return Err(Error::Refused(idx));
        }
        b.write(idx, arg, words).map_err(|e| Error::Bus(idx, e))?;
        self.status(b)?;
        b.kick();
        Ok(())
    }

    /// The target's length in blocks on this card.
    #[cfg(any(test, feature = "emmc-session"))]
    pub fn target_blocks(&self, t: &Target) -> Result<u64, Error> {
        let n = if t.blocks == 0 {
            self.blocks(t.part)
        } else {
            t.blocks
        };
        if n == 0
            || t.first
                .checked_add(n)
                .is_none_or(|e| e > self.blocks(t.part))
        {
            return Err(Error::Guard("target past the end of the eMMC partition"));
        }
        Ok(n)
    }

    /// `flash` of `image` to `t`, whose arm the caller has just used up:
    /// size, content (`board`: the layout check of a `firmware.bin`),
    /// write protection, then write with the first block last (a partial
    /// boot0 write never carries an eGON header), and read back.
    /// `scratch`: at least [`MAX_BLOCKS`] blocks.
    #[cfg(any(test, feature = "emmc-session"))]
    pub fn program<B: MmcBus>(
        &mut self,
        b: &mut B,
        t: &Target,
        image: &[u8],
        board: &mut dyn FnMut(&[u8]) -> Result<(), &'static str>,
        scratch: &mut [u32],
        info: &mut dyn Info,
    ) -> Result<(), Error> {
        let tb = self.target_blocks(t)?;
        if image.is_empty() || image.len() as u64 > tb * BLOCK64 {
            return Err(Error::Guard("image does not fit the target"));
        }
        match t.content {
            Content::Egon => egon::check(image, EGON_MAX),
            Content::Firmware => firmware_check(image).and_then(|()| board(image)),
            Content::Raw if image.len() as u64 != tb * BLOCK64 => Err("a window is written whole"),
            Content::Raw => Ok(()),
        }
        .map_err(Error::Guard)?;
        if scratch.len() < WORDS * MAX_BLOCKS {
            return Err(Error::Guard("scratch buffer too small"));
        }
        self.select_for_write(b, t.part, &mut [0; WORDS])?;
        let blocks = image.len().div_ceil(BLOCK) as u64;
        info.info(format_args!(
            "writing {blocks} blocks to {:?} at block {}",
            t.part, t.first
        ));
        let header_last = t.content != Content::Raw;
        let chunks = blocks.div_ceil(MAX_BLOCKS as u64);
        // Block 0 is its own last chunk when the header goes last.
        let order = (0..chunks)
            .map(|c| (c, false))
            .chain(header_last.then_some((0, true)));
        for (c, header) in order {
            let mut start = c * MAX_BLOCKS as u64;
            let mut end = (start + MAX_BLOCKS as u64).min(blocks);
            if header_last {
                if header {
                    end = 1;
                } else if start == 0 {
                    start = 1;
                }
            }
            if start >= end {
                continue;
            }
            let n = (end - start) as usize;
            fill(&mut scratch[..n * WORDS], image, start as usize * BLOCK);
            self.write_blocks(b, t, tb, t.first + start, &scratch[..n * WORDS])?;
        }
        self.compare(b, t, blocks, scratch, |i| {
            image.get(i).copied().unwrap_or(0)
        })?;
        info.info(format_args!(
            "written and read back ({} bytes)",
            image.len()
        ));
        Ok(())
    }

    /// `erase` of `t`: zeros, the first block first (the target stops
    /// being bootable before anything else changes), then read back.
    #[cfg(any(test, feature = "emmc-session"))]
    pub fn erase<B: MmcBus>(
        &mut self,
        b: &mut B,
        t: &Target,
        scratch: &mut [u32],
        info: &mut dyn Info,
    ) -> Result<(), Error> {
        let tb = self.target_blocks(t)?;
        if scratch.len() < WORDS * MAX_BLOCKS {
            return Err(Error::Guard("scratch buffer too small"));
        }
        self.select_for_write(b, t.part, &mut [0; WORDS])?;
        info.info(format_args!(
            "zeroing {tb} blocks of {:?} at block {}",
            t.part, t.first
        ));
        scratch.fill(0);
        let mut at = 0u64;
        while at < tb {
            // 1 block first, then whole chunks.
            let n = if at == 0 {
                1
            } else {
                (tb - at).min(MAX_BLOCKS as u64)
            };
            self.write_blocks(b, t, tb, t.first + at, &scratch[..n as usize * WORDS])?;
            at += n;
        }
        self.compare(b, t, tb, scratch, |_| 0)?;
        info.info(format_args!("zeroed and read back"));
        Ok(())
    }

    #[cfg(any(test, feature = "emmc-session"))]
    fn compare<B: MmcBus>(
        &mut self,
        b: &mut B,
        t: &Target,
        blocks: u64,
        scratch: &mut [u32],
        want: impl Fn(usize) -> u8,
    ) -> Result<(), Error> {
        let mut at = 0u64;
        while at < blocks {
            let n = (blocks - at).min(MAX_BLOCKS as u64) as usize;
            let w = &mut scratch[..n * WORDS];
            self.read(b, t.part, t.first + at, w)?;
            for (i, word) in w.iter().enumerate() {
                let base = at as usize * BLOCK + 4 * i;
                let exp = u32::from_le_bytes([
                    want(base),
                    want(base + 1),
                    want(base + 2),
                    want(base + 3),
                ]);
                if *word != exp {
                    return Err(Error::Verify(at + (4 * i / BLOCK) as u64));
                }
            }
            at += n as u64;
        }
        Ok(())
    }

    /// `oem emmc-bootcfg`, its arm just used up: write PARTITION_CONFIG
    /// bits 6:3 (`part_config`, access bits ignored) and
    /// BOOT_BUS_CONDITIONS, then read EXT_CSD back. Refused while
    /// BOOT_CONFIG_PROT is set (never cleared here).
    #[cfg(any(test, feature = "emmc-session"))]
    pub fn boot_config<B: MmcBus>(
        &mut self,
        b: &mut B,
        part_config: u8,
        boot_bus: u8,
        ext: &mut [u32; WORDS],
    ) -> Result<(), Error> {
        if !self.write {
            return Err(Error::Guard("eMMC writes are not built in"));
        }
        crate::emmcarm::boot_config_ok(part_config, boot_bus).map_err(Error::Guard)?;
        self.refresh(b, ext)?;
        if self.part.is_none() {
            return Err(Error::Guard(
                "eMMC partition access not confirmed; not touched",
            ));
        }
        if self.fields.boot_config_prot != 0 {
            return Err(Error::Guard(
                "BOOT_CONFIG_PROT set; boot configuration not touched",
            ));
        }
        let pc = (part_config & 0x78) | (self.fields.part_config & 7);
        self.send_switch(b, Switch::PartitionConfig(pc))?;
        self.send_switch(b, Switch::BootBus(boot_bus))?;
        self.refresh(b, ext)?;
        if self.fields.part_config != pc || self.fields.boot_bus != boot_bus {
            return Err(Error::Verify(0));
        }
        Ok(())
    }
}

#[cfg(any(test, feature = "emmc-session"))]
fn access(config: u8) -> Option<Part> {
    match config & 7 {
        0 => Some(Part::User),
        1 => Some(Part::Boot1),
        2 => Some(Part::Boot2),
        _ => None,
    }
}

fn set_byte(e: &mut [u32; WORDS], i: usize, v: u8) {
    let mut w = e[i / 4].to_le_bytes();
    w[i % 4] = v;
    e[i / 4] = u32::from_le_bytes(w);
}

fn read_ext_csd<B: MmcBus>(b: &mut B, write: bool, ext: &mut [u32; WORDS]) -> Result<(), Error> {
    if !command_allowed(8, 0, write) {
        return Err(Error::Refused(8));
    }
    b.read(8, 0, ext).map_err(|e| Error::Bus(8, e))
}

/// Copy `words.len() * 4` bytes of `image` from `at` into `words`,
/// zero-padded past its end.
#[cfg(any(test, feature = "emmc-session"))]
fn fill(words: &mut [u32], image: &[u8], at: usize) {
    for (i, w) in words.iter_mut().enumerate() {
        let o = at + 4 * i;
        let g = |k: usize| image.get(o + k).copied().unwrap_or(0);
        *w = u32::from_le_bytes([g(0), g(1), g(2), g(3)]);
    }
}

/// `firmware.bin` for `emmc-fw` (written at user-area 0x20000): a boot0
/// for a medium at its start, and the layout frame where the medium's
/// table offset falls, whose images lie inside the target. The frame's
/// decode and the DRAM contract are the board's check.
pub fn firmware_check(image: &[u8]) -> Result<(), &'static str> {
    use cubie_a7a_common::layout;
    egon::check(image, EGON_MAX)?;
    let at = (layout::TABLE_OFFSET as u64 - FW_START) as usize;
    let frame = image
        .get(at..at + BLOCK)
        .ok_or("firmware.bin too short for the layout table")?;
    let t = layout::decode(frame).map_err(|_| "no valid A7ABOOT1 frame in firmware.bin")?;
    for e in &t.entries[..t.count as usize] {
        if e.kind == layout::KIND_VENDOR_SPL {
            return Err("vendor SPL entry: not on the eMMC");
        }
        let end = e.offset as u64 + e.size as u64;
        if (e.offset as u64) < FW_START || end > FW_END || end > FW_START + image.len() as u64 {
            return Err("an image lies outside firmware.bin or emmc-fw");
        }
    }
    Ok(())
}

#[cfg(test)]
pub(crate) mod model {
    //! A host-side eMMC: state machine, EXT_CSD with the JEDEC field
    //! semantics this module relies on, user and boot partitions, a bus
    //! that can be broken above 1 bit, and a log of every command. It
    //! panics on any command the allow-list does not list for its build,
    //! and on any change to a persistent EXT_CSD byte that was not
    //! explicitly permitted.
    use super::*;
    use std::vec;
    use std::vec::Vec;

    #[derive(Clone, Copy, PartialEq, Debug)]
    enum State {
        Idle,
        Ready,
        Ident,
        Stby,
        Tran,
    }

    pub struct EmmcModel {
        pub detect: Option<bool>,
        pub present: bool,
        pub write_build: bool,
        /// Persistent PARTITION_CONFIG bits / BOOT_BUS may change.
        pub allow_persistent: bool,
        pub ext: [u8; 512],
        pub user: Vec<u8>,
        pub boot: [Vec<u8>; 2],
        pub log: Vec<(u32, u32)>,
        pub broken_wide: bool,
        /// EXT_CSD[183] BUS_WIDTH as CMD8 returns it: false (default) as
        /// the eMMC standard types it, W/E_P, written but not readable, so
        /// it reads 0 (seen on a SanDisk iNAND module: always 1-bit);
        /// true for a device that echoes the written width.
        pub bus_width_readable: bool,
        pub width: u32,
        pub fail_write_at: Option<u64>,
        pub fail_status_once: bool,
        pub ignore_access_switch: bool,
        pub corrupt_at: Option<u64>,
        state: State,
        busy: u32,
    }

    impl EmmcModel {
        /// A sector-mode eMMC 5.1: `user` blocks, boot partitions of
        /// `boot_mult` * 128 KiB.
        pub fn new(user_blocks: u64, boot_mult: u8, write_build: bool) -> Self {
            let mut ext = [0u8; 512];
            ext[ext_csd::EXT_CSD_REV] = 8;
            ext[ext_csd::DEVICE_TYPE] = 0x57;
            ext[ext_csd::SEC_COUNT..ext_csd::SEC_COUNT + 4]
                .copy_from_slice(&(user_blocks as u32).to_le_bytes());
            ext[ext_csd::BOOT_SIZE_MULT] = boot_mult;
            ext[ext_csd::BOOT_INFO] = 7;
            let pattern = |n: usize, seed: u8| -> Vec<u8> {
                (0..n)
                    .map(|i| (i ^ (i >> 9) ^ seed as usize) as u8)
                    .collect()
            };
            let bb = boot_mult as usize * 128 * 1024;
            EmmcModel {
                detect: Some(true),
                present: true,
                write_build,
                allow_persistent: false,
                ext,
                user: pattern(user_blocks as usize * 512, 0),
                boot: [pattern(bb, 0x55), pattern(bb, 0xaa)],
                log: Vec::new(),
                broken_wide: false,
                bus_width_readable: false,
                width: 1,
                fail_write_at: None,
                fail_status_once: false,
                ignore_access_switch: false,
                corrupt_at: None,
                state: State::Idle,
                busy: 2,
            }
        }

        pub fn cmds(&self) -> Vec<u32> {
            self.log.iter().map(|c| c.0).collect()
        }

        fn area(&mut self) -> &mut Vec<u8> {
            match self.ext[ext_csd::PARTITION_CONFIG] & 7 {
                0 => &mut self.user,
                1 => &mut self.boot[0],
                2 => &mut self.boot[1],
                p => panic!("partition {p} selected"),
            }
        }

        fn r1(&self) -> u32 {
            let st = match self.state {
                State::Tran => STATE_TRAN,
                State::Stby => 3,
                State::Ident => 2,
                State::Ready => 1,
                State::Idle => 0,
            };
            (st << 9) | R1_READY
        }

        fn switch(&mut self, arg: u32) -> Result<[u32; 4], BusError> {
            let s = decode_switch(arg, self.write_build)
                .unwrap_or_else(|| panic!("CMD6 {arg:#x} reached the card"));
            let index = ((arg >> 16) & 0xff) as usize;
            let value = (arg >> 8) as u8;
            let old = self.ext[index];
            match s {
                Switch::PartitionConfig(_) => {
                    if old & 0x78 != value & 0x78 {
                        assert!(
                            self.allow_persistent && self.write_build,
                            "persistent PARTITION_CONFIG bits changed: {old:#x} -> {value:#x}"
                        );
                    }
                    if value & 7 != 0 && self.ext[ext_csd::BOOT_SIZE_MULT] == 0 {
                        return Ok([self.r1() | R1_SWITCH_ERROR, 0, 0, 0]);
                    }
                    if self.ignore_access_switch {
                        return Ok([self.r1(), 0, 0, 0]);
                    }
                }
                Switch::BootBus(_) => {
                    assert!(
                        self.allow_persistent && self.write_build,
                        "BOOT_BUS_CONDITIONS written"
                    );
                }
                Switch::BusWidth(w) => {
                    self.width = [1, 4, 8][w as usize];
                }
            }
            self.ext[index] = value;
            Ok([self.r1(), 0, 0, 0])
        }
    }

    impl MmcBus for EmmcModel {
        fn present(&mut self) -> Option<bool> {
            self.detect
        }
        fn clock(&mut self, _ident: bool) -> Result<(), BusError> {
            Ok(())
        }
        fn set_width(&mut self, _w: u32) -> Result<(), BusError> {
            Ok(())
        }
        fn cmd(&mut self, idx: u32, arg: u32, _resp: Resp) -> Result<[u32; 4], BusError> {
            assert!(
                command_allowed(idx, arg, self.write_build),
                "CMD{idx} {arg:#x} reached the card"
            );
            self.log.push((idx, arg));
            if !self.present {
                return if idx == 0 {
                    Ok([0; 4])
                } else {
                    Err(BusError::Timeout)
                };
            }
            let r = match (idx, self.state) {
                (0, _) => {
                    self.state = State::Idle;
                    // E_P fields back to their power-on values.
                    self.ext[ext_csd::PARTITION_CONFIG] &= !7;
                    self.ext[ext_csd::BUS_WIDTH] = 0;
                    self.width = 1;
                    [0; 4]
                }
                (1, State::Idle | State::Ready) => {
                    self.state = State::Ready;
                    if self.busy > 0 {
                        self.busy -= 1;
                        [0x00ff_8080, 0, 0, 0]
                    } else {
                        [0xc0ff_8080, 0, 0, 0]
                    }
                }
                (2, State::Ready) => {
                    self.state = State::Ident;
                    [0x1500_0001, 2, 3, 4]
                }
                (3, State::Ident) if arg == RCA << 16 => {
                    self.state = State::Stby;
                    [0x500, 0, 0, 0]
                }
                (9, State::Stby) if arg == RCA << 16 => [0xd000_0000, 0, 0, 0],
                (7, State::Stby) if arg == RCA << 16 => {
                    self.state = State::Tran;
                    [0x700, 0, 0, 0]
                }
                (13, State::Tran) if arg == RCA << 16 => {
                    if core::mem::take(&mut self.fail_status_once) {
                        return Err(BusError::Timeout);
                    }
                    [self.r1(), 0, 0, 0]
                }
                (6, State::Tran) => return self.switch(arg),
                _ => return Err(BusError::Error),
            };
            Ok(r)
        }
        fn read(&mut self, idx: u32, arg: u32, buf: &mut [u32]) -> Result<(), BusError> {
            assert!(command_allowed(idx, arg, self.write_build));
            self.log.push((idx, arg));
            if self.state != State::Tran {
                return Err(BusError::Error);
            }
            let bytes: Vec<u8> = if idx == 8 {
                let mut e = self.ext.to_vec();
                if !self.bus_width_readable {
                    e[ext_csd::BUS_WIDTH] = 0;
                }
                e
            } else {
                let n = buf.len() * 4;
                let a = arg as usize * 512;
                let area = self.area();
                if a + n > area.len() {
                    return Err(BusError::Error);
                }
                area[a..a + n].to_vec()
            };
            for (i, w) in buf.iter_mut().enumerate() {
                *w = u32::from_le_bytes(bytes[4 * i..4 * i + 4].try_into().unwrap());
                if self.broken_wide && self.width > 1 && i % 7 == 3 {
                    *w ^= 0x10;
                }
            }
            Ok(())
        }
        fn write(&mut self, idx: u32, arg: u32, buf: &[u32]) -> Result<(), BusError> {
            assert!(self.write_build, "write command in a read-only build");
            assert!(command_allowed(idx, arg, true));
            assert!(matches!(idx, 24 | 25));
            assert_eq!(idx == 25, buf.len() > WORDS);
            self.log.push((idx, arg));
            if self.state != State::Tran {
                return Err(BusError::Error);
            }
            let lba = arg as u64;
            let n = (buf.len() / WORDS) as u64;
            if self
                .fail_write_at
                .is_some_and(|f| (lba..lba + n).contains(&f))
            {
                return Err(BusError::Error);
            }
            let corrupt = self.corrupt_at;
            let a = arg as usize * 512;
            let area = self.area();
            assert!(a + buf.len() * 4 <= area.len(), "write past the end");
            for (i, w) in buf.iter().enumerate() {
                area[a + 4 * i..a + 4 * i + 4].copy_from_slice(&w.to_le_bytes());
            }
            if let Some(c) = corrupt {
                if (lba..lba + n).contains(&c) {
                    area[c as usize * 512] ^= 1;
                }
            }
            Ok(())
        }
    }

    pub fn ext_words(m: &EmmcModel) -> [u32; WORDS] {
        let mut w = [0u32; WORDS];
        for (i, x) in w.iter_mut().enumerate() {
            *x = u32::from_le_bytes(m.ext[4 * i..4 * i + 4].try_into().unwrap());
        }
        w
    }

    pub fn image(len: usize, seed: u8) -> Vec<u8> {
        let mut v = vec![0u8; len];
        for (i, b) in v.iter_mut().enumerate() {
            *b = (i * 13 + seed as usize) as u8;
        }
        v
    }
}

#[cfg(test)]
mod tests {
    use super::model::{image, EmmcModel};
    use super::*;
    use crate::emmcarm::target;
    use std::string::String;
    use std::vec;
    use std::vec::Vec;

    struct Log(Vec<String>);
    impl Info for Log {
        fn info(&mut self, msg: core::fmt::Arguments) {
            self.0.push(std::format!("{msg}"));
        }
    }

    const USER: u64 = 0x1_0000; // 32 MiB

    fn open(m: &mut EmmcModel) -> Card {
        let mut ext = [0u32; WORDS];
        Card::identify(m, &mut ext, 8, m.write_build).unwrap()
    }

    #[test]
    fn allow_list() {
        for write in [false, true] {
            for idx in 0..64 {
                let ok = command_allowed(idx, 0, write) || command_allowed(idx, 0x03b3_0000, write);
                let expect = matches!(idx, 0 | 1 | 2 | 3 | 6 | 7 | 8 | 9 | 12 | 13 | 17 | 18)
                    || (write && matches!(idx, 24 | 25));
                assert_eq!(ok, expect, "CMD{idx} write={write}");
            }
            // Never: sleep, bus test, inactive, erase, write protect, lock,
            // CSD/CID programming, general and command-queue commands.
            for idx in [
                5, 14, 15, 19, 21, 23, 26, 27, 28, 29, 30, 31, 35, 36, 38, 39, 40, 42, 44, 45, 46,
                47, 48, 49, 53, 54, 55, 56,
            ] {
                for arg in [0, 1, 0xffff_ffff] {
                    assert!(!command_allowed(idx, arg, write), "CMD{idx}");
                }
            }
            // CMD0: GO_IDLE only (no pre-idle, no boot initiation).
            assert!(command_allowed(0, 0, write));
            assert!(!command_allowed(0, 0xf0f0_f0f0, write));
            assert!(!command_allowed(0, 0xffff_fffa, write));
            assert!(!command_allowed(8, 1, write));
        }
        assert!(!command_allowed(24, 0, false) && !command_allowed(25, 0, false));
    }

    /// Every CMD6 argument with a zero command set: exactly the volatile
    /// fields in the read-only build; plus BOOT_BUS_CONDITIONS in the
    /// write build; never an index from NEVER.
    #[test]
    fn switch_allow_list_exhaustive() {
        let mut count = [0usize; 2];
        for write in [false, true] {
            for access in 0..4u32 {
                for index in 0..256u32 {
                    for value in 0..256u32 {
                        let arg = (access << 24) | (index << 16) | (value << 8);
                        let ok = command_allowed(6, arg, write);
                        let v = value as u8;
                        let expect = access == 3
                            && match index {
                                179 => {
                                    v & 0x80 == 0
                                        && v & 7 <= 2
                                        && matches!((v >> 3) & 7, 0 | 1 | 2 | 7)
                                }
                                183 => v <= 2,
                                177 => write && v & 0xe0 == 0 && v & 3 != 3 && (v >> 3) & 3 != 3,
                                _ => false,
                            };
                        assert_eq!(ok, expect, "{arg:#x} write={write}");
                        count[write as usize] += ok as usize;
                        // Command set bits and the top bits are never accepted.
                        assert!(!command_allowed(6, arg | 1, write));
                        assert!(!command_allowed(6, arg | 1 << 26, write));
                    }
                }
            }
            for (index, why) in NEVER {
                for value in 0..256u32 {
                    for access in 0..4u32 {
                        let arg = (access << 24) | ((index as u32) << 16) | (value << 8);
                        assert!(!command_allowed(6, arg, write), "{why} write={write}");
                    }
                }
            }
        }
        // 179: 4 enables x 2 ack x 3 access; 183: 3; 177: 3 x 2 x 3.
        assert_eq!(count, [24 + 3, 24 + 3 + 18]);
    }

    #[test]
    fn identify_8bit_and_read() {
        let mut m = EmmcModel::new(USER, 32, false);
        let card = open(&mut m);
        assert_eq!(card.width, 8);
        assert_eq!(card.user_blocks, USER);
        assert_eq!(card.boot_blocks, 32 * 256);
        assert!(card.sector_mode);
        assert_eq!(m.cmds()[..9], [0, 1, 1, 1, 2, 3, 9, 7, 8]);
        // Then the width switch, status, and the EXT_CSD read back.
        assert_eq!(m.log[9], (6, Switch::BusWidth(2).arg()));
        assert_eq!(m.cmds()[10..], [13, 8]);
        let mut card = card;
        let mut buf = vec![0u32; WORDS * 200];
        card.read(&mut m, Part::User, 1000, &mut buf).unwrap();
        assert_eq!(buf[0].to_le_bytes()[1], m.user[1000 * 512 + 1]);
        assert_eq!(
            buf[WORDS * 200 - 1].to_le_bytes()[3],
            m.user[1200 * 512 - 1]
        );
        assert_eq!(m.cmds()[12..], [18, 18]);
        // Boot partition 2, then the user area again.
        card.read(&mut m, Part::Boot2, 5, &mut buf[..WORDS])
            .unwrap();
        assert_eq!(buf[0].to_le_bytes()[0], m.boot[1][5 * 512]);
        card.read(&mut m, Part::User, 0, &mut buf[..WORDS]).unwrap();
        assert_eq!(m.ext[ext_csd::PARTITION_CONFIG], 0);
        // Ranges are checked before the bus.
        let n = m.log.len();
        assert_eq!(
            card.read(&mut m, Part::Boot1, 32 * 256, &mut buf[..WORDS]),
            Err(Error::Range)
        );
        assert_eq!(
            card.read(&mut m, Part::User, USER - 1, &mut buf[..2 * WORDS]),
            Err(Error::Range)
        );
        assert_eq!(
            card.read(&mut m, Part::User, u64::MAX, &mut buf[..WORDS]),
            Err(Error::Range)
        );
        assert_eq!(m.log.len(), n);
    }

    #[test]
    fn select_keeps_persistent_bits() {
        let mut m = EmmcModel::new(USER, 16, false);
        // A card someone configured to boot from boot partition 1 with ack.
        m.ext[ext_csd::PARTITION_CONFIG] = 0x48;
        let mut card = open(&mut m);
        let mut buf = vec![0u32; WORDS];
        card.read(&mut m, Part::Boot1, 0, &mut buf).unwrap();
        assert_eq!(m.ext[ext_csd::PARTITION_CONFIG], 0x49);
        card.read(&mut m, Part::User, 0, &mut buf).unwrap();
        assert_eq!(m.ext[ext_csd::PARTITION_CONFIG], 0x48);
        // A switch that would change bits 6:3 never reaches the card
        // (the model would panic).
        let n = m.log.len();
        assert!(card.switch(&mut m, Switch::PartitionConfig(0x08)).is_err());
        assert!(card.switch(&mut m, Switch::PartitionConfig(0x41)).is_err());
        assert!(card.switch(&mut m, Switch::BootBus(2)).is_err());
        assert_eq!(m.log.len(), n);
        // The read-only build has no route to BOOT_BUS at all.
        let mut ext = [0u32; WORDS];
        assert_eq!(
            card.boot_config(&mut m, 0x08, 2, &mut ext),
            Err(Error::Guard("eMMC writes are not built in"))
        );
        // No boot partitions: selecting one is refused before the bus.
        let mut m = EmmcModel::new(USER, 0, false);
        let mut card = open(&mut m);
        let n = m.log.len();
        assert!(card.read(&mut m, Part::Boot1, 0, &mut buf).is_err());
        assert_eq!(m.log.len(), n);
    }

    #[test]
    fn bus_width_byte_reads_zero_or_the_code() {
        // W/E_P as the eMMC standard types it (reads 0, the model's default
        // and the SanDisk module) and a device that echoes the code: 8-bit,
        // with the same commands either way.
        for readable in [false, true] {
            let mut m = EmmcModel::new(USER, 16, false);
            m.bus_width_readable = readable;
            let card = open(&mut m);
            assert_eq!(card.width, 8, "readable {readable}");
            assert_eq!(card.fields.bus_width, if readable { 2 } else { 0 });
            assert_eq!(m.cmds(), [0, 1, 1, 1, 2, 3, 9, 7, 8, 6, 13, 8]);
            assert_eq!(m.width, 8);
        }
        // 4-bit only: same, through the 4-bit switch.
        for readable in [false, true] {
            let mut m = EmmcModel::new(USER, 16, false);
            m.bus_width_readable = readable;
            let mut ext = [0u32; WORDS];
            let c = Card::identify(&mut m, &mut ext, 4, false).unwrap();
            assert_eq!(c.width, 4);
            assert_eq!(m.log[9], (6, Switch::BusWidth(1).arg()));
        }
    }

    #[test]
    fn bus_width_byte_with_another_value_falls_back() {
        // [183] reading back neither 0 nor the code (a garbled bus that
        // only hits that byte) still fails the wide bus.
        let mut m = EmmcModel::new(USER, 16, false);
        m.bus_width_readable = true;
        let mut ext = [0u32; WORDS];
        // Identify on 1 bit, then pretend the 8-bit read returned 1 at 183.
        let mut c = Card::identify(&mut m, &mut ext, 1, false).unwrap();
        m.ext[ext_csd::BUS_WIDTH] = 0;
        set_byte(&mut ext, ext_csd::BUS_WIDTH, 0);
        struct Odd<'a>(&'a mut EmmcModel);
        impl MmcBus for Odd<'_> {
            fn present(&mut self) -> Option<bool> {
                self.0.present()
            }
            fn clock(&mut self, i: bool) -> Result<(), BusError> {
                self.0.clock(i)
            }
            fn set_width(&mut self, w: u32) -> Result<(), BusError> {
                self.0.set_width(w)
            }
            fn cmd(&mut self, i: u32, a: u32, r: Resp) -> Result<[u32; 4], BusError> {
                self.0.cmd(i, a, r)
            }
            fn read(&mut self, i: u32, a: u32, buf: &mut [u32]) -> Result<(), BusError> {
                self.0.read(i, a, buf)?;
                if i == 8 && self.0.width == 8 {
                    set_byte(
                        (&mut buf[..WORDS]).try_into().unwrap(),
                        ext_csd::BUS_WIDTH,
                        1,
                    );
                }
                Ok(())
            }
        }
        assert!(!c.try_width(&mut Odd(&mut m), &mut ext, 8).unwrap());
        assert_eq!(c.width, 1);
        assert_eq!(m.ext[ext_csd::BUS_WIDTH], 0);
    }

    #[test]
    fn wide_bus_falls_back_to_1_bit() {
        let mut m = EmmcModel::new(USER, 16, false);
        m.broken_wide = true;
        let mut card = open(&mut m);
        assert_eq!(card.width, 1);
        assert_eq!(m.ext[ext_csd::BUS_WIDTH], 0);
        let mut buf = vec![0u32; WORDS * 3];
        card.read(&mut m, Part::User, 7, &mut buf).unwrap();
        assert_eq!(buf[3].to_le_bytes()[0], m.user[7 * 512 + 12]);
        // Asking for 1 bit sends no CMD6 at all.
        let mut m = EmmcModel::new(USER, 16, false);
        let mut ext = [0u32; WORDS];
        let c = Card::identify(&mut m, &mut ext, 1, false).unwrap();
        assert_eq!(c.width, 1);
        assert!(!m.cmds().contains(&6));
    }

    #[test]
    fn no_module() {
        // Detect says no module: nothing is sent.
        let mut m = EmmcModel::new(USER, 16, false);
        m.detect = Some(false);
        let mut ext = [0u32; WORDS];
        assert_eq!(
            Card::identify(&mut m, &mut ext, 8, false),
            Err(Error::NoCard)
        );
        assert!(m.log.is_empty());
        // No detect and no card: CMD0, then CMD1 times out.
        let mut m = EmmcModel::new(USER, 16, false);
        m.detect = None;
        m.present = false;
        assert_eq!(
            Card::identify(&mut m, &mut ext, 8, false),
            Err(Error::NoCard)
        );
        assert_eq!(m.cmds(), [0, 1]);
    }

    fn fw_image() -> Vec<u8> {
        use cubie_a7a_common::{crc32, layout, mem};
        use layout::{Entry, Table, MAX_ENTRIES, TABLE_SIZE};
        let mut v = image(0x300000 + 0x10000 - FW_START as usize, 3);
        let bt0 = egon::make(0x4_0000, egon::MEDIA_RUN_ADDR);
        v[..bt0.len()].copy_from_slice(&bt0);
        let main = (0x200000 - FW_START as usize)..(0x200000 - FW_START as usize + 0x8000);
        let fd = (0x300000 - FW_START as usize)..v.len();
        let mut t = Table {
            magic: layout::MAGIC,
            version: layout::VERSION,
            count: 2,
            crc32: 0,
            _reserved: 0,
            entries: [Entry::default(); MAX_ENTRIES],
        };
        t.entries[0] = Entry {
            kind: layout::KIND_MAIN,
            offset: 0x200000,
            size: 0x8000,
            load_addr: mem::MAIN_BASE as u64,
            crc32: crc32(&v[main]),
            ..Entry::default()
        };
        t.entries[1] = Entry {
            kind: layout::KIND_PAYLOAD,
            offset: 0x300000,
            size: 0x10000,
            load_addr: mem::PAYLOAD_BASE as u64,
            crc32: crc32(&v[fd]),
            ..Entry::default()
        };
        let frame = bootlayout::encode(&t);
        let at = (layout::TABLE_OFFSET as u64 - FW_START) as usize;
        v[at..at + TABLE_SIZE].copy_from_slice(&frame[..TABLE_SIZE]);
        v
    }

    fn ok_board(_: &[u8]) -> Result<(), &'static str> {
        Ok(())
    }

    #[test]
    fn firmware_check_rules() {
        let fw = fw_image();
        assert_eq!(firmware_check(&fw), Ok(()));
        let mut bad = fw.clone();
        bad[100] ^= 1; // eGON checksum
        assert!(firmware_check(&bad).is_err());
        let fel = {
            let mut v = fw.clone();
            let b = egon::make(0x4_0000, 0x9_0000);
            v[..b.len()].copy_from_slice(&b);
            v
        };
        assert!(firmware_check(&fel).is_err(), "FEL build");
        assert!(firmware_check(&fw[..0x1d0000]).is_err(), "no table");
        assert!(
            firmware_check(&fw[..0x2e0000]).is_err(),
            "images past the end"
        );
        let mut t = fw.clone();
        t[0x1df000 + 30] ^= 1; // table CRC
        assert!(firmware_check(&t).is_err());
    }

    /// The built image (scripts/mk-emmc-fw.py in radxa-a7a-workspace), when
    /// given:
    /// `EMMC_FW_IMAGE=.../emmc-fw.bin cargo test`.
    #[test]
    fn built_image_if_given() {
        if let Some(p) = std::env::var_os("EMMC_FW_IMAGE") {
            let v = std::fs::read(p).unwrap();
            assert_eq!(firmware_check(&v), Ok(()));
            assert!(FW_START + v.len() as u64 <= FW_END);
        }
    }

    #[test]
    fn ambiguous_switch_refuses_later_armed_write() {
        use crate::emmcarm::Guard;

        let mut m = EmmcModel::new(256, 1, true);
        let mut card = open(&mut m);
        m.fail_status_once = true;
        assert_eq!(
            card.select(&mut m, Part::Boot1),
            Err(Error::Bus(13, BusError::Timeout))
        );
        assert_eq!(m.ext[ext_csd::PARTITION_CONFIG] & 7, 1);

        let mut guard = Guard::new(true);
        let data = image(BLOCK, 12);
        let name = "emmc@0+200";
        guard
            .arm(
                &std::format!("{name} flash 200 {:08x}", crate::crc32(&data)),
                0,
            )
            .unwrap();
        let target = guard.take(name, Some(&data), 1).unwrap();
        let mut scratch = vec![0; WORDS * MAX_BLOCKS];
        let mut log = Log(Vec::new());
        assert_eq!(
            card.program(
                &mut m,
                &target,
                &data,
                &mut ok_board,
                &mut scratch,
                &mut log
            ),
            Err(Error::Guard(
                "eMMC partition access not confirmed; not touched"
            ))
        );
        assert!(!m.cmds().iter().any(|&i| matches!(i, 24 | 25)));
        // Explicit recovery, then a newly armed, confirmed write.
        card.select(&mut m, Part::User).unwrap();
        guard
            .arm(
                &std::format!("{name} flash 200 {:08x}", crate::crc32(&data)),
                2,
            )
            .unwrap();
        let target = guard.take(name, Some(&data), 3).unwrap();
        card.program(
            &mut m,
            &target,
            &data,
            &mut ok_board,
            &mut scratch,
            &mut log,
        )
        .unwrap();
        assert_eq!(&m.user[..BLOCK], &data);
    }

    #[test]
    fn erase_requires_matching_access_after_successful_status() {
        let mut m = EmmcModel::new(256, 1, true);
        let mut card = open(&mut m);
        m.ignore_access_switch = true;
        let mut scratch = vec![0; WORDS * MAX_BLOCKS];
        let mut log = Log(Vec::new());
        let target = target("emmc-boot0").unwrap();
        assert_eq!(
            card.erase(&mut m, &target, &mut scratch, &mut log),
            Err(Error::Guard(
                "eMMC partition access not confirmed; not touched"
            ))
        );
        assert_eq!(&m.cmds()[12..], [8, 6, 13, 8]);
        assert_eq!(m.ext[ext_csd::PARTITION_CONFIG] & 7, 0);
    }

    #[test]
    fn program_firmware_header_last_and_verify() {
        let mut m = EmmcModel::new(USER, 16, true);
        let mut card = open(&mut m);
        let fw = fw_image();
        let t = target("emmc-fw").unwrap();
        let mut scratch = vec![0u32; WORDS * MAX_BLOCKS];
        let mut log = Log(Vec::new());
        let before_gpt = m.user[..0x20000].to_vec();
        let before_var = m.user[0x800000..0x1000000].to_vec();
        card.program(&mut m, &t, &fw, &mut ok_board, &mut scratch, &mut log)
            .unwrap();
        assert_eq!(&m.user[0x20000..0x20000 + fw.len()], &fw[..]);
        assert_eq!(&m.user[..0x20000], &before_gpt[..], "GPT area untouched");
        assert_eq!(
            &m.user[0x800000..0x1000000],
            &before_var[..],
            "variable store untouched"
        );
        // Writes: block 257 onward first, block 256 (the eGON header) last.
        let writes: Vec<(u32, u32)> = m
            .log
            .iter()
            .copied()
            .filter(|c| matches!(c.0, 24 | 25))
            .collect();
        assert_eq!(writes[0], (25, 257));
        assert_eq!(*writes.last().unwrap(), (24, 256));
        assert!(writes
            .iter()
            .all(|w| w.1 >= 256 && (w.1 as u64) < FW_END / 512));
        assert!(log.0.last().unwrap().contains("read back"));
    }

    #[test]
    fn program_refusals_touch_nothing() {
        let fw = fw_image();
        let t = target("emmc-fw").unwrap();
        let mut scratch = vec![0u32; WORDS * MAX_BLOCKS];
        let mut log = Log(Vec::new());
        let writes = |m: &EmmcModel| {
            m.cmds()
                .iter()
                .filter(|&&c| matches!(c, 24 | 25 | 6))
                .count()
        };
        // Read-only build: refused before any write, even with an image.
        let mut m = EmmcModel::new(USER, 16, false);
        let mut card = open(&mut m);
        let w0 = writes(&m);
        assert!(card
            .program(&mut m, &t, &fw, &mut ok_board, &mut scratch, &mut log)
            .is_err());
        assert_eq!(writes(&m), w0);
        // Content refusals.
        let mut m = EmmcModel::new(USER, 16, true);
        let mut card = open(&mut m);
        let w0 = writes(&m);
        let mut bad = fw.clone();
        bad[64] ^= 1;
        for (tgt, img) in [
            ("emmc-fw", bad.as_slice()),
            ("emmc-fw", &fw[..1000]),
            ("emmc-boot0", &fw[..0x1000]), // eGON length past the image
            ("emmc@1000000+200", &fw[..0x100]), // window written whole
            ("emmc@1000000+200", &fw[..0x400]), // too long
        ] {
            let t = target(tgt).unwrap();
            assert!(
                card.program(&mut m, &t, img, &mut ok_board, &mut scratch, &mut log)
                    .is_err(),
                "{tgt}"
            );
        }
        // The board's layout check can refuse too.
        assert!(card
            .program(&mut m, &t, &fw, &mut |_| Err("no"), &mut scratch, &mut log)
            .is_err());
        // Write protection: refused, nothing cleared.
        m.ext[ext_csd::BOOT_WP_STATUS] = 0x01;
        let mut ext = [0u32; WORDS];
        card.refresh(&mut m, &mut ext).unwrap();
        let bt0 = egon::make(0x4_0000, egon::MEDIA_RUN_ADDR);
        let b0 = target("emmc-boot0").unwrap();
        assert_eq!(
            card.program(&mut m, &b0, &bt0, &mut ok_board, &mut scratch, &mut log),
            Err(Error::Guard(
                "eMMC boot partition 1 is write protected; not touched"
            ))
        );
        m.ext[ext_csd::USER_WP] = 0x01;
        card.refresh(&mut m, &mut ext).unwrap();
        assert!(card
            .program(&mut m, &t, &fw, &mut ok_board, &mut scratch, &mut log)
            .is_err());
        assert_eq!(writes(&m), w0);
        // A target past the end of a small card.
        let mut m = EmmcModel::new(0x1000, 16, true); // 2 MiB: emmc-fw does not fit
        let mut card = open(&mut m);
        assert!(card
            .program(&mut m, &t, &fw, &mut ok_board, &mut scratch, &mut log)
            .is_err());
    }

    #[test]
    fn program_boot_partition_and_detect_bad_cells() {
        let mut m = EmmcModel::new(USER, 16, true);
        let mut card = open(&mut m);
        let bt0 = egon::make(0x4_0000, egon::MEDIA_RUN_ADDR);
        let b1 = target("emmc-boot1").unwrap();
        let mut scratch = vec![0u32; WORDS * MAX_BLOCKS];
        let mut log = Log(Vec::new());
        let user = m.user.clone();
        let boot0 = m.boot[0].clone();
        card.program(&mut m, &b1, &bt0, &mut ok_board, &mut scratch, &mut log)
            .unwrap();
        assert_eq!(&m.boot[1][..bt0.len()], &bt0[..]);
        assert_eq!(m.user, user);
        assert_eq!(m.boot[0], boot0);
        // Persistent PARTITION_CONFIG bits never changed (the model checks
        // that on every CMD6); access is back to whatever the next read
        // selects.
        assert_eq!(m.ext[ext_csd::PARTITION_CONFIG] & 0x78, 0);
        // A cell that does not take the data is found by the read-back.
        m.corrupt_at = Some(3);
        assert_eq!(
            card.program(&mut m, &b1, &bt0, &mut ok_board, &mut scratch, &mut log),
            Err(Error::Verify(3))
        );
        // A failing write stops the run with the bus error.
        m.corrupt_at = None;
        m.fail_write_at = Some(200);
        assert_eq!(
            card.program(&mut m, &b1, &bt0, &mut ok_board, &mut scratch, &mut log),
            Err(Error::Bus(25, BusError::Error))
        );
    }

    #[test]
    fn erase_zeroes_header_first() {
        let mut m = EmmcModel::new(USER, 16, true);
        let mut card = open(&mut m);
        let t = target("emmc-fw").unwrap();
        let mut scratch = vec![0u32; WORDS * MAX_BLOCKS];
        let mut log = Log(Vec::new());
        let n0 = m.log.len();
        let before = m.user.clone();
        card.erase(&mut m, &t, &mut scratch, &mut log).unwrap();
        let writes: Vec<(u32, u32)> = m.log[n0..]
            .iter()
            .copied()
            .filter(|c| matches!(c.0, 24 | 25))
            .collect();
        assert_eq!(writes[0], (24, 256));
        assert!(m.user[0x20000..0x800000].iter().all(|&b| b == 0));
        assert_eq!(
            &m.user[0x800000..],
            &before[0x800000..],
            "variable store and OS untouched"
        );
        assert_eq!(&m.user[..0x20000], &before[..0x20000], "GPT area untouched");
        // A raw window.
        let w = target("emmc@1000000+20000").unwrap();
        card.erase(&mut m, &w, &mut scratch, &mut log).unwrap();
        assert!(m.user[0x1000000..0x1020000].iter().all(|&b| b == 0));
        assert_eq!(&m.user[0x1020000..], &before[0x1020000..]);
    }

    #[test]
    fn raw_window_write() {
        let mut m = EmmcModel::new(USER, 16, true);
        let mut card = open(&mut m);
        let gpt = image(0x20000, 9);
        let t = target("emmc@0+20000").unwrap();
        let mut scratch = vec![0u32; WORDS * MAX_BLOCKS];
        let mut log = Log(Vec::new());
        card.program(&mut m, &t, &gpt, &mut ok_board, &mut scratch, &mut log)
            .unwrap();
        assert_eq!(&m.user[..0x20000], &gpt[..]);
        // In order, no header-last pass for raw data.
        let writes: Vec<u32> = m
            .log
            .iter()
            .filter(|c| matches!(c.0, 24 | 25))
            .map(|c| c.1)
            .collect();
        assert_eq!(writes, [0, 128]);
    }

    #[test]
    fn boot_config_is_the_only_persistent_write() {
        let mut m = EmmcModel::new(USER, 16, true);
        m.allow_persistent = true;
        let mut card = open(&mut m);
        let mut ext = [0u32; WORDS];
        card.boot_config(&mut m, 0x08, 0x02, &mut ext).unwrap();
        assert_eq!(m.ext[ext_csd::PARTITION_CONFIG], 0x08);
        assert_eq!(m.ext[ext_csd::BOOT_BUS_CONDITIONS], 0x02);
        // Access bits in the request are ignored (kept as the session's).
        card.boot_config(&mut m, 0x00, 0x00, &mut ext).unwrap();
        assert_eq!(m.ext[ext_csd::PARTITION_CONFIG], 0x00);
        assert!(card.boot_config(&mut m, 0x09, 0, &mut ext).is_err());
        assert!(card.boot_config(&mut m, 0x20, 0, &mut ext).is_err());
        assert!(card.boot_config(&mut m, 0x08, 0x03, &mut ext).is_err());
        // BOOT_CONFIG_PROT set: refused, and never cleared.
        m.ext[ext_csd::BOOT_CONFIG_PROT] = 0x01;
        let n = m.log.iter().filter(|c| c.0 == 6).count();
        assert_eq!(
            card.boot_config(&mut m, 0x08, 0x02, &mut ext),
            Err(Error::Guard(
                "BOOT_CONFIG_PROT set; boot configuration not touched"
            ))
        );
        assert_eq!(m.log.iter().filter(|c| c.0 == 6).count(), n);
        assert_eq!(m.ext[ext_csd::BOOT_CONFIG_PROT], 0x01);
    }

    #[test]
    fn fields_parse() {
        let m = EmmcModel::new(USER, 16, false);
        let f = Fields::parse(&model::ext_words(&m));
        assert_eq!(
            (f.rev, f.sec_count, f.boot_size_mult, f.boot_info),
            (8, USER as u32, 16, 7)
        );
    }
}
