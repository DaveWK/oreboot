// SPDX-License-Identifier: GPL-2.0-only
//! Polled SD/eMMC driver for an SDHCI (v3) host controller.
//!
//! Moved from the SpacemiT K1 bt0 (`ore-edk-boot-opi` `mmc.rs`, commit
//! 78479e21ba): the card is re-initialised at a low clock in 1-bit mode and
//! read in 512-byte sectors (CMD17); the fastboot flasher also writes
//! (CMD24/CMD25, by PIO or SDMA), widens the bus, raises the clock and TRIMs.
//! Everything SoC-specific (controller clock and reset, PHY and pad set-up,
//! vendor registers, the source clock when the capabilities register reports
//! none, cache maintenance before SDMA) is the board's [`SdhciPlatform`].
//! Register and memory access goes through [`regio::RegIo`]: data buffers are
//! addresses the controller (SDMA) or the CPU (PIO) reads and writes. The
//! byte registers (host and power control, software reset) are accessed with
//! `read8`/`write8`, which the board's `RegIo` must implement as real byte
//! accesses (the trait's defaults are word read-modify-writes).
#![no_std]

use regio::RegIo;

// SDHCI registers
const SDMA_ADDR: usize = 0x00;
const BLOCK_SIZE: usize = 0x04;
const BLOCK_COUNT: usize = 0x06;
const ARGUMENT: usize = 0x08;
const TRANSFER_MODE: usize = 0x0c;
const COMMAND: usize = 0x0e;
const RESPONSE: usize = 0x10;
const BUFFER: usize = 0x20;
const PRESENT_STATE: usize = 0x24;
const HOST_CONTROL: usize = 0x28;
const POWER_CONTROL: usize = 0x29;
const CLOCK_CONTROL: usize = 0x2c;
const TIMEOUT_CONTROL: usize = 0x2e;
const SOFTWARE_RESET: usize = 0x2f;
const INT_STATUS: usize = 0x30;
const INT_ENABLE: usize = 0x34;
const SIGNAL_ENABLE: usize = 0x38;
const CAPABILITIES: usize = 0x40;

const PRESENT_CMD_INHIBIT: u32 = 1 << 0;
const PRESENT_DAT_INHIBIT: u32 = 1 << 1;

const INT_CMD_COMPLETE: u32 = 1 << 0;
const INT_XFER_COMPLETE: u32 = 1 << 1;
const INT_DMA: u32 = 1 << 3;
const INT_BUF_WRITE_READY: u32 = 1 << 4;
const INT_BUF_READ_READY: u32 = 1 << 5;
const INT_ERROR: u32 = 1 << 15;

// Transfer mode
const TM_DMA: u16 = 1 << 0;
const TM_BLOCK_COUNT: u16 = 1 << 1;
const TM_AUTO_CMD12: u16 = 1 << 2;
const TM_READ: u16 = 1 << 4;
const TM_MULTI: u16 = 1 << 5;

// Host control 1
const HC_4BIT: u8 = 1 << 1;
const HC_HIGH_SPEED: u8 = 1 << 2;
const HC_8BIT: u8 = 1 << 5;

const CLOCK_INT_EN: u16 = 1 << 0;
const CLOCK_INT_STABLE: u16 = 1 << 1;
const CLOCK_CARD_EN: u16 = 1 << 2;

// Command register flags
const RESP_NONE: u16 = 0;
const RESP_136: u16 = 1;
const RESP_48: u16 = 2;
const RESP_48_BUSY: u16 = 3;
const CHECK_CRC: u16 = 1 << 3;
const CHECK_INDEX: u16 = 1 << 4;
const DATA_PRESENT: u16 = 1 << 5;

const R1: u16 = RESP_48 | CHECK_CRC | CHECK_INDEX;
const R1B: u16 = RESP_48_BUSY | CHECK_CRC | CHECK_INDEX;
const R2: u16 = RESP_136 | CHECK_CRC;
const R3: u16 = RESP_48;
const R6: u16 = R1;
const R7: u16 = R1;

/// RCA given to an eMMC card (SD cards publish their own).
const EMMC_RCA: u32 = 2;
/// Sector size.
pub const SECTOR: usize = 512;
/// Polls of a controller status before a [`Error::Timeout`].
pub const WAIT_POLLS: usize = 50_000_000;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    Emmc,
    Sd,
}

/// eMMC hardware partitions (EXT_CSD PARTITION_ACCESS).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Partition {
    User = 0,
    Boot0 = 1,
    Boot1 = 2,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    Timeout(&'static str),
    Command(u8, u32),
    NotReady,
    Capacity,
}

/// The source clock (MHz) the identification-mode clock is divided from:
/// the capabilities base clock, or 200 MHz when it reads 0. (Later clock
/// changes, [`Sdhci::set_khz`], ask the platform's clock tree instead.)
pub const fn init_source_mhz(caps_base_mhz: u32) -> u32 {
    if caps_base_mhz == 0 {
        200
    } else {
        caps_base_mhz
    }
}

/// The bus voltage selected in POWER_CONTROL bits 3:1.
pub const fn bus_volts(power_control: u8) -> &'static str {
    match (power_control >> 1) & 7 {
        0b111 => "3.3 V",
        0b110 => "3.0 V",
        0b101 => "1.8 V",
        _ => "unset",
    }
}

/// What the SoC adds around a standard SDHCI. Every hook has a default that
/// does nothing (or the SDHCI value), so a plain SDHCI needs no code.
pub trait SdhciPlatform {
    /// Clock the controller and release its reset before use.
    fn enable<IO: RegIo>(&mut self, _io: &mut IO, _base: usize, _kind: Kind) {}
    /// Vendor set-up before the controller reset (PHY mode, pads, clock
    /// gating) for a card of `kind`.
    fn init_quirks<IO: RegIo>(&mut self, _io: &mut IO, _base: usize, _kind: Kind) {}
    /// The controller's source clock in kHz when the capabilities register
    /// reports none (base clock 0).
    fn source_khz<IO: RegIo>(&mut self, _io: &mut IO, _base: usize, _kind: Kind) -> u32 {
        200_000
    }
    /// Pad and PHY tuning before writes.
    fn tune_for_writes<IO: RegIo>(&mut self, _io: &mut IO, _base: usize, _kind: Kind) {}
    /// The write data hold-time control, on or off.
    fn set_tx_hold<IO: RegIo>(&mut self, _io: &mut IO, _base: usize, _on: bool) {}
    /// Clean `len` bytes at `addr` from the data cache before SDMA reads them.
    fn flush_dcache(&mut self, _addr: usize, _len: usize) {}
    /// Busy-wait about `n` loop iterations (settling delays).
    fn spin(&mut self, n: usize) {
        for _ in 0..n {
            core::hint::spin_loop();
        }
    }
}

/// The inclusive last sector of a TRIM of `count` sectors from `lba`.
pub fn trim_last(lba: u32, count: usize) -> Result<u32, Error> {
    let last = (lba as u64)
        .checked_add(count as u64)
        .and_then(|e| e.checked_sub(1))
        .ok_or(Error::Capacity)?;
    u32::try_from(last).map_err(|_| Error::Capacity)
}

/// SDHCI v3 10-bit divided clock field for divider `n` (f = base / 2n).
const fn divider(n: u32) -> u16 {
    (((n & 0xff) << 8) | ((n >> 8) & 0x3) << 6) as u16
}

pub struct Sdhci<IO: RegIo, P: SdhciPlatform> {
    io: IO,
    plat: P,
    base: usize,
    kind: Kind,
    rca: u32,
    /// Sector (block) addressing: eMMC sector mode, SDHC/SDXC.
    block_addressing: bool,
    sd_sectors: u64,
}

impl<IO: RegIo, P: SdhciPlatform> Sdhci<IO, P> {
    fn r8(&mut self, o: usize) -> u8 {
        self.io.read8(self.base + o)
    }
    fn w8(&mut self, o: usize, v: u8) {
        self.io.write8(self.base + o, v)
    }
    fn r16(&mut self, o: usize) -> u16 {
        self.io.read16(self.base + o)
    }
    fn w16(&mut self, o: usize, v: u16) {
        self.io.write16(self.base + o, v)
    }
    fn r32(&mut self, o: usize) -> u32 {
        self.io.read32(self.base + o)
    }
    fn w32(&mut self, o: usize, v: u32) {
        self.io.write32(self.base + o, v)
    }

    fn wait(&mut self, what: &'static str, done: impl Fn(&mut Self) -> bool) -> Result<(), Error> {
        for _ in 0..WAIT_POLLS {
            if done(self) {
                return Ok(());
            }
        }
        Err(Error::Timeout(what))
    }

    /// The card kind.
    pub fn kind(&self) -> Kind {
        self.kind
    }
    /// The register access, for the board's own vendor registers.
    pub fn io(&mut self) -> &mut IO {
        &mut self.io
    }
    /// Give the register access and platform back.
    pub fn into_parts(self) -> (IO, P) {
        (self.io, self.plat)
    }

    /// Base clock in MHz from the capabilities register; 0 means unknown.
    fn base_clock_mhz(&mut self) -> u32 {
        (self.r32(CAPABILITIES) >> 8) & 0xff
    }

    fn program_clock(&mut self, n: u32) -> Result<(), Error> {
        self.w16(CLOCK_CONTROL, 0);
        let div = divider(n);
        self.w16(CLOCK_CONTROL, div | CLOCK_INT_EN);
        self.wait("internal clock", |m| {
            m.r16(CLOCK_CONTROL) & CLOCK_INT_STABLE != 0
        })?;
        self.w16(CLOCK_CONTROL, div | CLOCK_INT_EN | CLOCK_CARD_EN);
        self.plat.spin(100_000);
        Ok(())
    }

    fn set_clock(&mut self, khz: u32) -> Result<(), Error> {
        let base = init_source_mhz(self.base_clock_mhz()) * 1000;
        let n = base.div_ceil(2 * khz).min(0x3ff);
        self.program_clock(n)
    }

    fn command(&mut self, index: u8, arg: u32, flags: u16) -> Result<[u32; 4], Error> {
        // A data command here is a single-block read.
        self.command_data(index, arg, flags, TM_READ, 1)
    }

    /// Issue a command; with DATA_PRESENT, `mode` and `blocks` program the
    /// transfer (the data itself is moved by the caller).
    fn command_data(
        &mut self,
        index: u8,
        arg: u32,
        flags: u16,
        mode: u16,
        blocks: usize,
    ) -> Result<[u32; 4], Error> {
        let busy = flags & 3 == RESP_48_BUSY || flags & DATA_PRESENT != 0;
        self.wait("cmd inhibit", |m| {
            m.r32(PRESENT_STATE) & PRESENT_CMD_INHIBIT == 0
        })?;
        if busy {
            self.wait("dat inhibit", |m| {
                m.r32(PRESENT_STATE) & PRESENT_DAT_INHIBIT == 0
            })?;
        }
        self.w32(INT_STATUS, 0xffff_ffff);
        self.w32(ARGUMENT, arg);
        if flags & DATA_PRESENT != 0 {
            // SDMA pauses at 512 KiB boundaries (BLOCK_SIZE bits 14:12 = 7).
            let boundary = if mode & TM_DMA != 0 { 7 << 12 } else { 0 };
            self.w16(BLOCK_SIZE, SECTOR as u16 | boundary);
            self.w16(BLOCK_COUNT, blocks as u16);
            self.w16(TRANSFER_MODE, mode);
        } else {
            self.w16(TRANSFER_MODE, 0);
        }
        self.w16(COMMAND, ((index as u16) << 8) | flags);
        self.wait("command complete", |m| {
            m.r32(INT_STATUS) & (INT_CMD_COMPLETE | INT_ERROR) != 0
        })?;
        let status = self.r32(INT_STATUS);
        if status & INT_ERROR != 0 {
            self.w32(INT_STATUS, 0xffff_ffff);
            // Reset the command and data lines after an error.
            self.w8(SOFTWARE_RESET, 0x06);
            let _ = self.wait("reset", |m| m.r8(SOFTWARE_RESET) & 0x06 == 0);
            return Err(Error::Command(index, status));
        }
        self.w32(INT_STATUS, INT_CMD_COMPLETE);
        let resp = [
            self.r32(RESPONSE),
            self.r32(RESPONSE + 4),
            self.r32(RESPONSE + 8),
            self.r32(RESPONSE + 12),
        ];
        if flags & 3 == RESP_48_BUSY {
            self.wait("busy", |m| {
                m.r32(INT_STATUS) & (INT_XFER_COMPLETE | INT_ERROR) != 0
            })?;
            self.w32(INT_STATUS, INT_XFER_COMPLETE);
        }
        Ok(resp)
    }

    /// The card is back in the transfer state (CMD13), after at most
    /// `polls` status reads.
    fn wait_tran(&mut self, polls: usize) -> Result<(), Error> {
        for _ in 0..polls {
            let r = self.command(13, self.rca << 16, R1)?[0];
            if (r >> 9) & 0xf == 4 && r & (1 << 8) != 0 {
                return Ok(());
            }
        }
        Err(Error::NotReady)
    }

    /// EXT_CSD write with CMD6 (access: 1 set bits, 2 clear bits, 3 write byte).
    fn switch(&mut self, access: u32, index: u32, value: u32) -> Result<(), Error> {
        self.command(6, (access << 24) | (index << 16) | (value << 8), R1B)?;
        self.wait_tran(100_000)
    }

    /// Re-initialise the card at `base` in 1-bit legacy mode.
    pub fn init(io: IO, plat: P, base: usize, kind: Kind) -> Result<Self, Error> {
        let mut m = Self {
            io,
            plat,
            base,
            kind,
            rca: 0,
            block_addressing: true,
            sd_sectors: 0,
        };
        m.plat.enable(&mut m.io, base, kind);
        m.plat.spin(100_000);
        m.plat.init_quirks(&mut m.io, base, kind);
        m.w8(SOFTWARE_RESET, 0x06);
        m.wait("reset", |m| m.r8(SOFTWARE_RESET) & 0x06 == 0)?;
        m.w32(INT_ENABLE, 0xffff_ffff);
        m.w32(SIGNAL_ENABLE, 0);
        m.w8(TIMEOUT_CONTROL, 0x0e);
        // 1-bit, normal speed, no DMA.
        m.w8(HOST_CONTROL, 0);
        if m.r8(POWER_CONTROL) & 1 == 0 {
            // The bus is unpowered: eMMC I/O is 1.8 V, SD starts at 3.3 V.
            let volts = match kind {
                Kind::Emmc => 0x5,
                Kind::Sd => 0x7,
            };
            m.w8(POWER_CONTROL, (volts << 1) | 1);
            m.plat.spin(1_000_000);
        }
        let (caps, pwr) = (m.base_clock_mhz(), m.r8(POWER_CONTROL));
        m.io.log(format_args!(
            "[mmc] {} clock source {} MHz (capabilities base clock {}{}), bus power {} {} (0x{:02x})\n",
            if kind == Kind::Emmc { "eMMC" } else { "SD" },
            init_source_mhz(caps),
            caps,
            if caps == 0 { ", so assumed" } else { "" },
            bus_volts(pwr),
            if pwr & 1 != 0 { "on" } else { "off" },
            pwr
        ));
        m.set_clock(400)?;
        m.command(0, 0, RESP_NONE)?;
        m.plat.spin(1_000_000);
        match kind {
            Kind::Emmc => m.init_emmc()?,
            Kind::Sd => m.init_sd()?,
        }
        m.command(7, m.rca << 16, R1B)?;
        if !m.block_addressing {
            m.command(16, SECTOR as u32, R1)?;
        }
        m.set_clock(25_000)?;
        let rca = m.rca;
        m.io.log(format_args!("[mmc] card ready (RCA {rca})\n"));
        Ok(m)
    }

    fn init_emmc(&mut self) -> Result<(), Error> {
        // Sector addressing, 1.7-1.95 V and 2.7-3.6 V.
        let mut ready = false;
        for _ in 0..1000 {
            let ocr = self.command(1, 0x40ff_8080, R3)?[0];
            if ocr & (1 << 31) != 0 {
                ready = true;
                break;
            }
            self.plat.spin(1_000_000);
        }
        if !ready {
            return Err(Error::NotReady);
        }
        self.command(2, 0, R2)?;
        self.rca = EMMC_RCA;
        self.command(3, self.rca << 16, R1)?;
        Ok(())
    }

    fn init_sd(&mut self) -> Result<(), Error> {
        // CMD8: 2.7-3.6 V, check pattern 0xaa. Version 1 cards do not answer.
        let v2 = match self.command(8, 0x1aa, R7) {
            Ok(r) => r[0] & 0xfff == 0x1aa,
            Err(_) => false,
        };
        let hcs = if v2 { 1 << 30 } else { 0 };
        let mut ocr = 0;
        for _ in 0..1000 {
            self.command(55, 0, R1)?;
            ocr = self.command(41, hcs | 0x00ff_8000, R3)?[0];
            if ocr & (1 << 31) != 0 {
                break;
            }
            self.plat.spin(1_000_000);
        }
        if ocr & (1 << 31) == 0 {
            return Err(Error::NotReady);
        }
        // CCS: SDHC/SDXC use block addresses.
        self.block_addressing = ocr & (1 << 30) != 0;
        self.command(2, 0, R2)?;
        self.rca = self.command(3, 0, R6)?[0] >> 16;
        let csd = self.command(9, self.rca << 16, R2)?;
        self.sd_sectors =
            storage::sd::sectors(csd, self.block_addressing).ok_or(Error::Capacity)?;
        Ok(())
    }

    /// Select an eMMC hardware partition, keeping the boot configuration bits.
    pub fn select(&mut self, p: Partition) -> Result<(), Error> {
        if self.kind != Kind::Emmc {
            return Ok(());
        }
        // PARTITION_CONFIG (EXT_CSD 179): clear PARTITION_ACCESS, then set it.
        self.switch(2, 179, 0x07)?;
        if p as u32 != 0 {
            self.switch(1, 179, p as u32)?;
        }
        Ok(())
    }

    /// Bound command addressing before narrowing or byte-address conversion.
    pub fn transfer_range(&self, lba: u32, count: usize) -> Result<(), Error> {
        let end = (lba as u64)
            .checked_add(count as u64)
            .ok_or(Error::Capacity)?;
        let limit = if self.block_addressing {
            1u64 << 32
        } else {
            1u64 << 23
        };
        if end > limit || (self.kind == Kind::Sd && end > self.sd_sectors) {
            return Err(Error::Capacity);
        }
        Ok(())
    }

    fn arg(&self, sector: u32) -> u32 {
        if self.block_addressing {
            sector
        } else {
            sector * SECTOR as u32
        }
    }

    /// Read `count` sectors starting at `lba` into memory at `dest`.
    pub fn read(&mut self, lba: u32, count: usize, dest: usize) -> Result<(), Error> {
        if count == 0 {
            return Ok(());
        }
        self.transfer_range(lba, count)?;
        for i in 0..count {
            let arg = self.arg(lba + i as u32);
            self.read_block(17, arg, dest + i * SECTOR)?;
            if i % 2048 == 2047 {
                self.io.log(format_args!("."));
            }
        }
        Ok(())
    }

    /// One single-block read command (CMD17, or CMD8 for EXT_CSD).
    fn read_block(&mut self, index: u8, arg: u32, dest: usize) -> Result<(), Error> {
        self.command(index, arg, R1 | DATA_PRESENT)?;
        self.wait("read buffer", |m| {
            m.r32(INT_STATUS) & (INT_BUF_READ_READY | INT_ERROR) != 0
        })?;
        let status = self.r32(INT_STATUS);
        if status & INT_ERROR != 0 {
            self.w32(INT_STATUS, 0xffff_ffff);
            return Err(Error::Command(index, status));
        }
        self.w32(INT_STATUS, INT_BUF_READ_READY);
        for w in 0..SECTOR / 4 {
            let v = self.r32(BUFFER);
            self.io.write32(dest + w * 4, v);
        }
        self.wait("transfer complete", |m| {
            m.r32(INT_STATUS) & (INT_XFER_COMPLETE | INT_ERROR) != 0
        })?;
        self.w32(INT_STATUS, INT_XFER_COMPLETE);
        Ok(())
    }

    /// The card's EXT_CSD register (eMMC only).
    pub fn ext_csd(&mut self) -> Result<[u8; SECTOR], Error> {
        let mut ext = [0u8; SECTOR];
        self.read_block(8, 0, ext.as_mut_ptr() as usize)?;
        Ok(ext)
    }

    /// Actual medium/partition capacity in 512-byte sectors; unknown fails closed.
    pub fn capacity_sectors(&mut self, p: Partition) -> Result<u64, Error> {
        let n = if self.kind == Kind::Sd {
            if p as u32 != 0 {
                return Err(Error::Capacity);
            }
            self.sd_sectors
        } else {
            self.sectors(p)?
        };
        if n == 0 || n > 1u64 << 32 {
            return Err(Error::Capacity);
        }
        Ok(n)
    }

    /// Size in sectors of an eMMC hardware partition.
    pub fn sectors(&mut self, p: Partition) -> Result<u64, Error> {
        let ext = self.ext_csd()?;
        Ok(match p {
            Partition::User => u32::from_le_bytes([ext[212], ext[213], ext[214], ext[215]]) as u64,
            // BOOT_SIZE_MULT x 128 KiB
            Partition::Boot0 | Partition::Boot1 => ext[226] as u64 * 256,
        })
    }

    /// Recover the controller and card after a failed data transfer.
    fn abort(&mut self) {
        self.w32(INT_STATUS, 0xffff_ffff);
        self.w8(SOFTWARE_RESET, 0x06);
        let _ = self.wait("reset", |m| m.r8(SOFTWARE_RESET) & 0x06 == 0);
        let _ = self.command(12, 0, R1B);
        let _ = self.wait_tran(10_000_000);
    }

    /// Write `count` sectors from memory at `src` starting at `lba`, with
    /// multiple-block writes (CMD25 and an automatic CMD12).
    pub fn write(&mut self, lba: u32, count: usize, src: usize) -> Result<(), Error> {
        if count == 0 {
            return Ok(());
        }
        self.transfer_range(lba, count)?;
        let mut done = 0;
        while done < count {
            let n = (count - done).min(2048);
            let arg = self.arg(lba + done as u32);
            if n == 1 {
                self.command_data(24, arg, R1 | DATA_PRESENT, 0, 1)?;
            } else {
                self.command_data(
                    25,
                    arg,
                    R1 | DATA_PRESENT,
                    TM_BLOCK_COUNT | TM_AUTO_CMD12 | TM_MULTI,
                    n,
                )?;
            }
            for i in 0..n {
                let ready = self.wait("write buffer", |m| {
                    m.r32(INT_STATUS) & (INT_BUF_WRITE_READY | INT_ERROR) != 0
                });
                let status = self.r32(INT_STATUS);
                if ready.is_err() || status & INT_ERROR != 0 {
                    self.abort();
                    return Err(Error::Command(if n == 1 { 24 } else { 25 }, status));
                }
                self.w32(INT_STATUS, INT_BUF_WRITE_READY);
                let base = src + (done + i) * SECTOR;
                for w in 0..SECTOR / 4 {
                    let v = self.io.read32(base + w * 4);
                    self.w32(BUFFER, v);
                }
            }
            let complete = self.wait("write complete", |m| {
                m.r32(INT_STATUS) & (INT_XFER_COMPLETE | INT_ERROR) != 0
            });
            let status = self.r32(INT_STATUS);
            if complete.is_err() || status & INT_ERROR != 0 {
                self.abort();
                return Err(Error::Command(25, status));
            }
            self.w32(INT_STATUS, INT_XFER_COMPLETE);
            self.wait_tran(10_000_000)?;
            done += n;
        }
        Ok(())
    }

    /// Write like [`Sdhci::write`], but let the controller fetch the data
    /// from memory (SDMA) instead of pushing it through the data port.
    pub fn write_dma(&mut self, lba: u32, count: usize, src: usize) -> Result<(), Error> {
        if count == 0 {
            return Ok(());
        }
        self.transfer_range(lba, count)?;
        // SDMA, not ADMA (host control bits 4:3 = 0).
        let hc = self.r8(HOST_CONTROL) & !(3 << 3);
        self.w8(HOST_CONTROL, hc);
        let mut done = 0;
        while done < count {
            let n = (count - done).min(2048);
            let arg = self.arg(lba + done as u32);
            let addr = src + done * SECTOR;
            self.plat.flush_dcache(addr, n * SECTOR);
            self.w32(SDMA_ADDR, addr as u32);
            let mode = TM_DMA | TM_BLOCK_COUNT | if n > 1 { TM_AUTO_CMD12 | TM_MULTI } else { 0 };
            let index = if n > 1 { 25 } else { 24 };
            self.command_data(index, arg, R1 | DATA_PRESENT, mode, n)?;
            loop {
                let r = self.wait("dma write", |m| {
                    m.r32(INT_STATUS) & (INT_XFER_COMPLETE | INT_DMA | INT_ERROR) != 0
                });
                let status = self.r32(INT_STATUS);
                if r.is_err() || status & INT_ERROR != 0 {
                    self.abort();
                    return Err(Error::Command(index, status));
                }
                if status & INT_XFER_COMPLETE != 0 {
                    self.w32(INT_STATUS, INT_XFER_COMPLETE | INT_DMA);
                    break;
                }
                // Boundary reached: writing the address register resumes.
                self.w32(INT_STATUS, INT_DMA);
                let a = self.r32(SDMA_ADDR);
                self.w32(SDMA_ADDR, a);
            }
            self.wait_tran(10_000_000)?;
            done += n;
        }
        Ok(())
    }

    /// Set the data bus width (8, 4 or 1 bits) on the card and the host.
    pub fn set_width(&mut self, width: u8) -> Result<(), Error> {
        match self.kind {
            Kind::Emmc => {
                let v = match width {
                    8 => 2,
                    4 => 1,
                    _ => 0,
                };
                self.switch(3, 183, v)?;
            }
            Kind::Sd => {
                self.command(55, self.rca << 16, R1)?;
                self.command(6, if width == 4 { 2 } else { 0 }, R1)?;
            }
        }
        let hc = match width {
            8 => HC_8BIT,
            4 => HC_4BIT,
            _ => 0,
        };
        let v = (self.r8(HOST_CONTROL) & !(HC_4BIT | HC_8BIT)) | hc;
        self.w8(HOST_CONTROL, v);
        Ok(())
    }

    /// The controller's source clock in kHz: the capabilities register, or
    /// the platform's clock tree.
    pub fn source_khz(&mut self) -> u32 {
        // Read again for the value, as the K1 driver did (keeps the bus
        // access sequence of the equivalence tests).
        if self.base_clock_mhz() != 0 {
            return self.base_clock_mhz() * 1000;
        }
        self.plat.source_khz(&mut self.io, self.base, self.kind)
    }

    /// Set the card clock to at most `khz` from the source clock, and eMMC
    /// high-speed timing above 26 MHz. Returns the clock set.
    pub fn set_khz(&mut self, khz: u32) -> Result<u32, Error> {
        let hs = khz > 26_000;
        if self.kind == Kind::Emmc {
            self.switch(3, 185, hs as u32)?;
        }
        let hc = self.r8(HOST_CONTROL) & !HC_HIGH_SPEED;
        self.w8(HOST_CONTROL, hc | if hs { HC_HIGH_SPEED } else { 0 });
        let src = self.source_khz();
        let n = src.div_ceil(2 * khz).clamp(1, 0x3ff);
        self.program_clock(n)?;
        Ok(src / (2 * n))
    }

    /// The platform's pad and PHY tuning for writes.
    pub fn tune_for_writes(&mut self) {
        self.plat
            .tune_for_writes(&mut self.io, self.base, self.kind);
        let khz = self.source_khz();
        self.io.log(format_args!("[mmc] source clock {khz} kHz\n"));
    }

    /// Whether TRIM leaves zeros: TRIM supported and erased memory reads 0.
    pub fn trim_zeroes(&mut self) -> bool {
        match self.ext_csd() {
            // SEC_FEATURE_SUPPORT bit 4 (SEC_GB_CL_EN: TRIM), ERASED_MEM_CONT
            Ok(ext) => self.kind == Kind::Emmc && ext[231] & (1 << 4) != 0 && ext[181] == 0,
            Err(_) => false,
        }
    }

    /// TRIM `count` sectors from `lba` (CMD35, CMD36, CMD38 with arg 1).
    pub fn trim(&mut self, lba: u32, count: usize) -> Result<(), Error> {
        if count == 0 {
            return Ok(());
        }
        self.transfer_range(lba, count)?;
        // CMD35/36 here are eMMC sector-mode TRIM only. The inclusive end is
        // computed in u64 so a valid exclusive end of 2^32 does not wrap.
        if self.kind != Kind::Emmc || !self.block_addressing {
            return Err(Error::Capacity);
        }
        let last = trim_last(lba, count)?;
        self.command(35, lba, R1)?;
        self.command(36, last, R1)?;
        let r = self.command(38, 1, R1B);
        let ready = self.wait_tran(10_000_000);
        r?;
        ready
    }

    /// Read one sector (for checks).
    pub fn read_sector(&mut self, lba: u32, dest: usize) -> Result<(), Error> {
        self.transfer_range(lba, 1)?;
        let arg = self.arg(lba);
        self.read_block(17, arg, dest)
    }

    /// The platform's write data hold-time control, on or off.
    pub fn set_tx_hold(&mut self, on: bool) {
        self.plat.set_tx_hold(&mut self.io, self.base, on);
    }

    /// Move to the widest data bus the card answers on (eMMC 8 then 4 bits,
    /// SD 4 bits), keeping 1 bit when a test read fails. Returns the width.
    pub fn widen(&mut self) -> u8 {
        let mut probe = [0u8; SECTOR];
        let addr = probe.as_mut_ptr() as usize;
        match self.kind {
            Kind::Emmc => {
                for (width, ext_value, hc) in [(8u8, 2u32, HC_8BIT), (4, 1, HC_4BIT)] {
                    if self.switch(3, 183, ext_value).is_err() {
                        continue;
                    }
                    let v = (self.r8(HOST_CONTROL) & !(HC_4BIT | HC_8BIT)) | hc;
                    self.w8(HOST_CONTROL, v);
                    if self.read_block(8, 0, addr).is_ok() {
                        return width;
                    }
                    let v = self.r8(HOST_CONTROL) & !(HC_4BIT | HC_8BIT);
                    self.w8(HOST_CONTROL, v);
                    let _ = self.switch(3, 183, 0);
                }
                1
            }
            Kind::Sd => {
                let ok =
                    self.command(55, self.rca << 16, R1).is_ok() && self.command(6, 2, R1).is_ok();
                if ok {
                    let v = (self.r8(HOST_CONTROL) & !HC_8BIT) | HC_4BIT;
                    self.w8(HOST_CONTROL, v);
                    if self.read_block(17, 0, addr).is_ok() {
                        return 4;
                    }
                    let v = self.r8(HOST_CONTROL) & !HC_4BIT;
                    self.w8(HOST_CONTROL, v);
                    let _ = self.command(55, self.rca << 16, R1);
                    let _ = self.command(6, 0, R1);
                }
                1
            }
        }
    }
}

#[cfg(test)]
mod range_controls {
    //! The K1 driver's range controls, on the crate.
    use super::*;
    use regio::mock::Mock;

    struct Plain;
    impl SdhciPlatform for Plain {}

    fn card(block_addressing: bool, kind: Kind, n: u64) -> Sdhci<Mock<4>, Plain> {
        Sdhci {
            io: Mock::default(),
            plat: Plain,
            base: 0,
            kind,
            rca: 0,
            block_addressing,
            sd_sectors: n,
        }
    }
    #[test]
    fn inclusive_trim_end_at_command_limit() {
        assert_eq!(trim_last(1, u32::MAX as usize).unwrap(), u32::MAX);
        assert_eq!(trim_last(u32::MAX, 1).unwrap(), u32::MAX);
        assert!(trim_last(u32::MAX, 2).is_err());
        assert!(trim_last(1, usize::MAX).is_err());
    }
    #[test]
    fn address_extent_limits() {
        let m = card(true, Kind::Emmc, 0);
        assert!(m.transfer_range(u32::MAX, 1).is_ok());
        assert!(m.transfer_range(u32::MAX, 2).is_err());
        let m = card(false, Kind::Sd, 1 << 23);
        assert!(m.transfer_range((1 << 23) - 1, 1).is_ok());
        assert!(m.transfer_range(1 << 23, 1).is_err());
        let m = card(true, Kind::Sd, 100);
        assert!(m.transfer_range(99, 1).is_ok());
        assert!(m.transfer_range(99, 2).is_err());
    }
    #[test]
    fn unsupported_trim_does_no_io() {
        let mut m = card(false, Kind::Emmc, 0);
        assert!(m.trim(0, 1).is_err());
        let mut m = card(true, Kind::Sd, 100);
        assert!(m.trim(0, 1).is_err());
        assert!(m.trim(0, 0).is_ok());
        assert_eq!(m.io.reads + m.io.writes, 0);
    }
    #[test]
    fn clock_divider_field() {
        assert_eq!(divider(1), 0x0100);
        assert_eq!(divider(0x3ff), 0xffc0);
        assert_eq!(divider(0x1ff), 0xff40);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clock_source_and_bus_power_decode() {
        assert_eq!(init_source_mhz(0), 200);
        assert_eq!(init_source_mhz(50), 50);
        assert_eq!(bus_volts(0x0b), "1.8 V");
        assert_eq!(bus_volts(0x0f), "3.3 V");
        assert_eq!(bus_volts(0x0d), "3.0 V");
        assert_eq!(bus_volts(0x00), "unset");
    }
}
