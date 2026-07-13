// SPDX-License-Identifier: GPL-2.0-only
// Verbatim copy of ore-edk-boot-opi c91bbc4e40
// oreboot/src/mainboard/spacemit/k1x/bt0/src/mmc.rs (equivalence oracle).
//! Minimal polled (PIO) eMMC / SD driver for the SpacemiT K1 SDHCI.
//!
//! eMMC is SDH2 at 0xd428_1000 (OrangePi R2S), SD is SDH0 at 0xd428_0000
//! (OrangePi RV2). When the mask ROM booted from the card, the controller is
//! clocked, pinmuxed and the card powered; for download-mode boots the
//! controller clock and reset are enabled here. The card is re-initialised at
//! a low clock in 1-bit mode and read in 512-byte sectors with CMD17, which
//! is all the boot path needs. The fastboot flasher also writes (CMD25) and
//! widens the bus first.

use util::mmio::{read16, read32, read8, write16, write32, write8};

pub const EMMC_BASE: usize = 0xd428_1000;
pub const SD_BASE: usize = 0xd428_0000;

// APMU clock/reset control (U-Boot include/soc/spacemit/k1-syscon.h)
const APMU: usize = 0xd428_2800;
// SDH AXI: bit 3 clock, bit 0 reset release; SDH0: bit 4 clock, bit 1 reset release
const APMU_SDH0_CLK_RES_CTRL: usize = APMU + 0x054;
// SDH2: bit 4 clock, bit 1 reset release
const APMU_SDH2_CLK_RES_CTRL: usize = APMU + 0x0e0;

// SDHCI registers
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

// SpacemiT K1 vendor registers (see U-Boot drivers/mmc/spacemit_sdhci.c)
const OP_EXT: usize = 0x108;
const LEGACY_CTRL: usize = 0x10c;
const MMC_CTRL: usize = 0x114;
const TX_CFG: usize = 0x11c;
const PHY_CTRL: usize = 0x160;
const PHY_PADCFG: usize = 0x178;

const PRESENT_CMD_INHIBIT: u32 = 1 << 0;
const PRESENT_DAT_INHIBIT: u32 = 1 << 1;

const INT_CMD_COMPLETE: u32 = 1 << 0;
const INT_XFER_COMPLETE: u32 = 1 << 1;
const INT_BUF_WRITE_READY: u32 = 1 << 4;
const INT_BUF_READ_READY: u32 = 1 << 5;
const INT_ERROR: u32 = 1 << 15;

const SDMA_ADDR: usize = 0x00;
const INT_DMA: u32 = 1 << 3;

// Transfer mode
const TM_DMA: u16 = 1 << 0;
const TM_BLOCK_COUNT: u16 = 1 << 1;
const TM_AUTO_CMD12: u16 = 1 << 2;
const TM_READ: u16 = 1 << 4;
const TM_MULTI: u16 = 1 << 5;

// Host control 1: data bus width
const HC_4BIT: u8 = 1 << 1;
const HC_8BIT: u8 = 1 << 5;
const HC_HIGH_SPEED: u8 = 1 << 2;

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

// RCA given to an eMMC card (SD cards publish their own).
const EMMC_RCA: u32 = 2;
const SECTOR: usize = 512;

#[derive(Clone, Copy, PartialEq)]
pub enum Kind {
    Emmc,
    Sd,
}

/// eMMC hardware partitions (EXT_CSD PARTITION_ACCESS).
#[derive(Clone, Copy)]
pub enum Partition {
    User = 0,
    Boot0 = 1,
    Boot1 = 2,
}

#[derive(Debug)]
pub enum Error {
    Timeout(&'static str),
    Command(u8, u32),
    NotReady,
    Capacity,
}

fn trim_last(lba: u32, count: usize) -> Result<u32, Error> {
    let last = (lba as u64)
        .checked_add(count as u64)
        .and_then(|e| e.checked_sub(1))
        .ok_or(Error::Capacity)?;
    u32::try_from(last).map_err(|_| Error::Capacity)
}

fn spin(n: usize) {
    for _ in 0..n {
        core::hint::spin_loop();
    }
}

fn wait<F: Fn() -> bool>(what: &'static str, done: F) -> Result<(), Error> {
    for _ in 0..50_000_000 {
        if done() {
            return Ok(());
        }
    }
    Err(Error::Timeout(what))
}

pub struct Mmc {
    base: usize,
    kind: Kind,
    rca: u32,
    // Sector (block) addressing: eMMC sector mode, SDHC/SDXC.
    block_addressing: bool,
    sd_sectors: u64,
}

impl Mmc {
    fn r8(&self, o: usize) -> u8 {
        read8(self.base + o)
    }
    fn w8(&self, o: usize, v: u8) {
        write8(self.base + o, v)
    }
    fn r16(&self, o: usize) -> u16 {
        read16(self.base + o)
    }
    fn w16(&self, o: usize, v: u16) {
        write16(self.base + o, v)
    }
    fn r32(&self, o: usize) -> u32 {
        read32(self.base + o)
    }
    fn w32(&self, o: usize, v: u32) {
        write32(self.base + o, v)
    }

    /// Base clock in MHz from the capabilities register; 0 means unknown.
    fn base_clock_mhz(&self) -> u32 {
        (self.r32(CAPABILITIES) >> 8) & 0xff
    }

    fn set_clock(&self, khz: u32) -> Result<(), Error> {
        let base = match self.base_clock_mhz() {
            0 => 200,
            b => b,
        } * 1000;
        // SDHCI v3 10-bit divided clock: f = base / (2 * N), N = 0 means base.
        let mut n = base.div_ceil(2 * khz);
        if n > 0x3ff {
            n = 0x3ff;
        }
        self.w16(CLOCK_CONTROL, 0);
        let div = (((n & 0xff) << 8) | ((n >> 8) & 0x3) << 6) as u16;
        self.w16(CLOCK_CONTROL, div | CLOCK_INT_EN);
        wait("internal clock", || {
            self.r16(CLOCK_CONTROL) & CLOCK_INT_STABLE != 0
        })?;
        self.w16(CLOCK_CONTROL, div | CLOCK_INT_EN | CLOCK_CARD_EN);
        spin(100_000);
        Ok(())
    }

    fn command(&self, index: u8, arg: u32, flags: u16) -> Result<[u32; 4], Error> {
        // A data command here is a single-block read.
        self.command_data(index, arg, flags, TM_READ, 1)
    }

    /// Issue a command; with DATA_PRESENT, `mode` and `blocks` program the
    /// transfer (the data itself is moved by the caller).
    fn command_data(
        &self,
        index: u8,
        arg: u32,
        flags: u16,
        mode: u16,
        blocks: usize,
    ) -> Result<[u32; 4], Error> {
        let busy = flags & 3 == RESP_48_BUSY || flags & DATA_PRESENT != 0;
        wait("cmd inhibit", || {
            self.r32(PRESENT_STATE) & PRESENT_CMD_INHIBIT == 0
        })?;
        if busy {
            wait("dat inhibit", || {
                self.r32(PRESENT_STATE) & PRESENT_DAT_INHIBIT == 0
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
        wait("command complete", || {
            self.r32(INT_STATUS) & (INT_CMD_COMPLETE | INT_ERROR) != 0
        })?;
        let status = self.r32(INT_STATUS);
        if status & INT_ERROR != 0 {
            self.w32(INT_STATUS, 0xffff_ffff);
            // Reset the command and data lines after an error.
            self.w8(SOFTWARE_RESET, 0x06);
            let _ = wait("reset", || self.r8(SOFTWARE_RESET) & 0x06 == 0);
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
            wait("busy", || {
                self.r32(INT_STATUS) & (INT_XFER_COMPLETE | INT_ERROR) != 0
            })?;
            self.w32(INT_STATUS, INT_XFER_COMPLETE);
        }
        Ok(resp)
    }

    /// EXT_CSD write with CMD6 (access: 1 = set bits, 2 = clear bits, 3 = write byte).
    fn switch(&self, access: u32, index: u32, value: u32) -> Result<(), Error> {
        self.command(6, (access << 24) | (index << 16) | (value << 8), R1B)?;
        // Wait until the card is back in the transfer state.
        for _ in 0..100_000 {
            let r = self.command(13, self.rca << 16, R1)?[0];
            if (r >> 9) & 0xf == 4 && r & (1 << 8) != 0 {
                return Ok(());
            }
        }
        Err(Error::NotReady)
    }

    /// Re-initialise the card in 1-bit legacy mode.
    pub fn init(kind: Kind) -> Result<Self, Error> {
        let base = match kind {
            Kind::Emmc => EMMC_BASE,
            Kind::Sd => SD_BASE,
        };
        let mut m = Self {
            base,
            kind,
            rca: 0,
            block_addressing: true,
            sd_sectors: 0,
        };
        // The mask ROM does not touch a card it did not boot from (or any
        // card in download mode): make sure the controller is clocked and
        // out of reset.
        write32(
            APMU_SDH0_CLK_RES_CTRL,
            read32(APMU_SDH0_CLK_RES_CTRL) | (1 << 3) | (1 << 0),
        );
        match kind {
            Kind::Emmc => write32(
                APMU_SDH2_CLK_RES_CTRL,
                read32(APMU_SDH2_CLK_RES_CTRL) | (1 << 4) | (1 << 1),
            ),
            Kind::Sd => write32(
                APMU_SDH0_CLK_RES_CTRL,
                read32(APMU_SDH0_CLK_RES_CTRL) | (1 << 4) | (1 << 1),
            ),
        }
        spin(100_000);
        // Controller set-up as U-Boot's spacemit_sdhci_phy_init() does.
        match kind {
            Kind::Emmc => {
                // eMMC card mode and the PHY in functional mode.
                m.w32(MMC_CTRL, m.r32(MMC_CTRL) | (1 << 12));
                m.w32(PHY_CTRL, m.r32(PHY_CTRL) | 0x3);
                m.w32(PHY_PADCFG, m.r32(PHY_PADCFG) | (1 << 5));
            }
            Kind::Sd => {}
        }
        // TX_INT_CLK_SEL: the data hold time writes need at legacy speed
        // (U-Boot's k1x_sdhci.c sets it for legacy and HS modes).
        m.w32(TX_CFG, m.r32(TX_CFG) | (1 << 30));
        // Keep the card clock running while we poll.
        m.w32(OP_EXT, m.r32(OP_EXT) | (1 << 11) | (1 << 12));

        m.w8(SOFTWARE_RESET, 0x06);
        wait("reset", || m.r8(SOFTWARE_RESET) & 0x06 == 0)?;
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
            spin(1_000_000);
        }
        println!(
            "[mmc] {} base clock {} MHz, power {:02x}",
            if kind == Kind::Emmc { "eMMC" } else { "SD" },
            m.base_clock_mhz(),
            m.r8(POWER_CONTROL)
        );
        m.set_clock(400)?;

        m.command(0, 0, RESP_NONE)?;
        spin(1_000_000);
        match kind {
            Kind::Emmc => m.init_emmc()?,
            Kind::Sd => m.init_sd()?,
        }
        m.command(7, m.rca << 16, R1B)?;
        if !m.block_addressing {
            m.command(16, SECTOR as u32, R1)?;
        }
        m.set_clock(25_000)?;
        println!("[mmc] card ready (RCA {})", m.rca);
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
            spin(1_000_000);
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
            spin(1_000_000);
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
            crate::sd_capacity::sectors(csd, self.block_addressing).ok_or(Error::Capacity)?;
        Ok(())
    }

    /// Select an eMMC hardware partition, keeping the boot configuration bits.
    pub fn select(&self, p: Partition) -> Result<(), Error> {
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

    // Bound command addressing before narrowing or byte-address conversion.
    fn transfer_range(&self, lba: u32, count: usize) -> Result<(), Error> {
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

    /// Read `count` sectors starting at `lba` into memory at `dest`.
    pub fn read(&self, lba: u32, count: usize, dest: usize) -> Result<(), Error> {
        if count == 0 {
            return Ok(());
        }
        self.transfer_range(lba, count)?;
        for i in 0..count {
            let sector = lba + i as u32;
            let arg = if self.block_addressing {
                sector
            } else {
                sector * SECTOR as u32
            };
            self.read_block(17, arg, dest + i * SECTOR)?;
            if i % 2048 == 2047 {
                print!(".");
            }
        }
        Ok(())
    }

    /// One single-block read command (CMD17, or CMD8 for EXT_CSD).
    fn read_block(&self, index: u8, arg: u32, dest: usize) -> Result<(), Error> {
        self.command(index, arg, R1 | DATA_PRESENT)?;
        wait("read buffer", || {
            self.r32(INT_STATUS) & (INT_BUF_READ_READY | INT_ERROR) != 0
        })?;
        let status = self.r32(INT_STATUS);
        if status & INT_ERROR != 0 {
            self.w32(INT_STATUS, 0xffff_ffff);
            return Err(Error::Command(index, status));
        }
        self.w32(INT_STATUS, INT_BUF_READ_READY);
        for w in 0..SECTOR / 4 {
            write32(dest + w * 4, self.r32(BUFFER));
        }
        wait("transfer complete", || {
            self.r32(INT_STATUS) & (INT_XFER_COMPLETE | INT_ERROR) != 0
        })?;
        self.w32(INT_STATUS, INT_XFER_COMPLETE);
        Ok(())
    }

    /// The card's EXT_CSD register (eMMC only).
    pub fn ext_csd(&self) -> Result<[u8; SECTOR], Error> {
        let mut ext = [0u8; SECTOR];
        self.read_block(8, 0, ext.as_mut_ptr() as usize)?;
        Ok(ext)
    }

    /// Actual medium/partition capacity in 512-byte sectors; unknown fails closed.
    pub fn capacity_sectors(&self, p: Partition) -> Result<u64, Error> {
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
    pub fn sectors(&self, p: Partition) -> Result<u64, Error> {
        let ext = self.ext_csd()?;
        Ok(match p {
            Partition::User => u32::from_le_bytes([ext[212], ext[213], ext[214], ext[215]]) as u64,
            // BOOT_SIZE_MULT x 128 KiB
            Partition::Boot0 | Partition::Boot1 => ext[226] as u64 * 256,
        })
    }

    /// Wait until the card has finished programming and is back in the
    /// transfer state.
    fn wait_ready(&self) -> Result<(), Error> {
        for _ in 0..10_000_000 {
            let r = self.command(13, self.rca << 16, R1)?[0];
            if (r >> 9) & 0xf == 4 && r & (1 << 8) != 0 {
                return Ok(());
            }
        }
        Err(Error::NotReady)
    }

    /// Recover the controller and card after a failed data transfer.
    fn abort(&self) {
        self.w32(INT_STATUS, 0xffff_ffff);
        self.w8(SOFTWARE_RESET, 0x06);
        let _ = wait("reset", || self.r8(SOFTWARE_RESET) & 0x06 == 0);
        let _ = self.command(12, 0, R1B);
        let _ = self.wait_ready();
    }

    /// Write `count` sectors from memory at `src` starting at `lba`, with
    /// multiple-block writes (CMD25 and an automatic CMD12).
    pub fn write(&self, lba: u32, count: usize, src: usize) -> Result<(), Error> {
        if count == 0 {
            return Ok(());
        }
        self.transfer_range(lba, count)?;
        let mut done = 0;
        while done < count {
            let n = (count - done).min(2048);
            let sector = lba + done as u32;
            let arg = if self.block_addressing {
                sector
            } else {
                sector * SECTOR as u32
            };
            let r = if n == 1 {
                self.command_data(24, arg, R1 | DATA_PRESENT, 0, 1)
            } else {
                self.command_data(
                    25,
                    arg,
                    R1 | DATA_PRESENT,
                    TM_BLOCK_COUNT | TM_AUTO_CMD12 | TM_MULTI,
                    n,
                )
            };
            r?;
            for i in 0..n {
                let ready = wait("write buffer", || {
                    self.r32(INT_STATUS) & (INT_BUF_WRITE_READY | INT_ERROR) != 0
                });
                let status = self.r32(INT_STATUS);
                if ready.is_err() || status & INT_ERROR != 0 {
                    self.abort();
                    return Err(Error::Command(if n == 1 { 24 } else { 25 }, status));
                }
                self.w32(INT_STATUS, INT_BUF_WRITE_READY);
                let base = src + (done + i) * SECTOR;
                for w in 0..SECTOR / 4 {
                    self.w32(BUFFER, read32(base + w * 4));
                }
            }
            let complete = wait("write complete", || {
                self.r32(INT_STATUS) & (INT_XFER_COMPLETE | INT_ERROR) != 0
            });
            let status = self.r32(INT_STATUS);
            if complete.is_err() || status & INT_ERROR != 0 {
                self.abort();
                return Err(Error::Command(25, status));
            }
            self.w32(INT_STATUS, INT_XFER_COMPLETE);
            self.wait_ready()?;
            done += n;
        }
        Ok(())
    }

    /// Write like write(), but let the controller fetch the data from DRAM
    /// (SDMA) instead of pushing it through the data port.
    pub fn write_dma(&self, lba: u32, count: usize, src: usize) -> Result<(), Error> {
        if count == 0 {
            return Ok(());
        }
        self.transfer_range(lba, count)?;
        // SDMA, not ADMA (host control bits 4:3 = 0).
        self.w8(HOST_CONTROL, self.r8(HOST_CONTROL) & !(3 << 3));
        let mut done = 0;
        while done < count {
            let n = (count - done).min(2048);
            let sector = lba + done as u32;
            let arg = if self.block_addressing {
                sector
            } else {
                sector * SECTOR as u32
            };
            let addr = src + done * SECTOR;
            crate::cache::flush(addr, n * SECTOR);
            self.w32(SDMA_ADDR, addr as u32);
            let mode = TM_DMA | TM_BLOCK_COUNT | if n > 1 { TM_AUTO_CMD12 | TM_MULTI } else { 0 };
            let index = if n > 1 { 25 } else { 24 };
            self.command_data(index, arg, R1 | DATA_PRESENT, mode, n)?;
            loop {
                let r = wait("dma write", || {
                    self.r32(INT_STATUS) & (INT_XFER_COMPLETE | INT_DMA | INT_ERROR) != 0
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
                self.w32(SDMA_ADDR, self.r32(SDMA_ADDR));
            }
            self.wait_ready()?;
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
        self.w8(
            HOST_CONTROL,
            (self.r8(HOST_CONTROL) & !(HC_4BIT | HC_8BIT)) | hc,
        );
        Ok(())
    }

    /// The controller's source clock in kHz, from its APMU mux and divider
    /// (Linux ccu-k1.c: mux bits 7:5, divider bits 10:8 + 1).
    pub fn source_khz(&self) -> u32 {
        if self.base_clock_mhz() != 0 {
            return self.base_clock_mhz() * 1000;
        }
        // pll1_d6, pll1_d4, pll2_d8, pll1_d3 (SDH2) or pll2_d5 (SDH0),
        // pll1_d11, pll1_d13, pll1_d23
        let (reg, third) = match self.kind {
            Kind::Emmc => (read32(APMU_SDH2_CLK_RES_CTRL), 819_200),
            Kind::Sd => (read32(APMU_SDH0_CLK_RES_CTRL), 600_000),
        };
        let parents = [409_600, 614_400, 375_000, third, 223_400, 189_000, 106_800];
        let mux = ((reg >> 5) & 7) as usize;
        let div = ((reg >> 8) & 7) + 1;
        parents.get(mux).copied().unwrap_or(409_600) / div
    }

    /// Set the card clock to at most `khz` from the real source clock, and
    /// eMMC high-speed timing above 26 MHz. Returns the clock set.
    pub fn set_khz(&self, khz: u32) -> Result<u32, Error> {
        let hs = khz > 26_000;
        if self.kind == Kind::Emmc {
            self.switch(3, 185, hs as u32)?;
        }
        let hc = self.r8(HOST_CONTROL) & !HC_HIGH_SPEED;
        self.w8(HOST_CONTROL, hc | if hs { HC_HIGH_SPEED } else { 0 });
        let src = self.source_khz();
        let mut n = src.div_ceil(2 * khz).max(1);
        if n > 0x3ff {
            n = 0x3ff;
        }
        self.w16(CLOCK_CONTROL, 0);
        let div = (((n & 0xff) << 8) | ((n >> 8) & 0x3) << 6) as u16;
        self.w16(CLOCK_CONTROL, div | CLOCK_INT_EN);
        wait("internal clock", || {
            self.r16(CLOCK_CONTROL) & CLOCK_INT_STABLE != 0
        })?;
        self.w16(CLOCK_CONTROL, div | CLOCK_INT_EN | CLOCK_CARD_EN);
        spin(100_000);
        Ok(src / (2 * n))
    }

    /// Pad and PHY set-up for writing, as FreeBSD's sdhci_fdt_spacemit does
    /// for this slot: PHY in functional mode, pad drive strength 4 and RX
    /// bias, GEN_PAD_CLK_ON and TX_INT_CLK_SEL. In download mode the BootROM
    /// has not touched the card's pads.
    pub fn tune_for_writes(&self) {
        println!(
            "[mmc] before: APMU {:08x} PHY_CTRL {:08x} PAD {:08x} LEGACY {:08x} TX {:08x} OP_EXT {:08x}",
            read32(match self.kind {
                Kind::Emmc => APMU_SDH2_CLK_RES_CTRL,
                Kind::Sd => APMU_SDH0_CLK_RES_CTRL,
            }),
            self.r32(PHY_CTRL),
            self.r32(PHY_PADCFG),
            self.r32(LEGACY_CTRL),
            self.r32(TX_CFG),
            self.r32(OP_EXT)
        );
        if self.kind == Kind::Emmc {
            self.w32(PHY_CTRL, self.r32(PHY_CTRL) | 0x3);
            self.w32(PHY_PADCFG, (self.r32(PHY_PADCFG) & !0x7) | 4 | (1 << 5));
        }
        self.w32(LEGACY_CTRL, self.r32(LEGACY_CTRL) | (1 << 6));
        self.set_tx_hold(true);
        println!("[mmc] source clock {} kHz", self.source_khz());
    }

    /// Whether TRIM leaves zeros: TRIM supported and erased memory reads 0.
    pub fn trim_zeroes(&self) -> bool {
        match self.ext_csd() {
            // SEC_FEATURE_SUPPORT bit 4 (SEC_GB_CL_EN: TRIM), ERASED_MEM_CONT
            Ok(ext) => self.kind == Kind::Emmc && ext[231] & (1 << 4) != 0 && ext[181] == 0,
            Err(_) => false,
        }
    }

    /// TRIM `count` sectors from `lba` (CMD35, CMD36, CMD38 with arg 1).
    pub fn trim(&self, lba: u32, count: usize) -> Result<(), Error> {
        if count == 0 {
            return Ok(());
        }
        self.transfer_range(lba, count)?;
        // CMD35/36 here are eMMC sector-mode TRIM only. Compute inclusive
        // end in u64 so a valid exclusive end2^32 does not wrap first.
        if self.kind != Kind::Emmc || !self.block_addressing {
            return Err(Error::Capacity);
        }
        let last = trim_last(lba, count)?;
        self.command(35, lba, R1)?;
        self.command(36, last, R1)?;
        let r = self.command(38, 1, R1B);
        let ready = self.wait_ready();
        r?;
        ready
    }

    /// Read one sector (for checks).
    pub fn read_sector(&self, lba: u32, dest: usize) -> Result<(), Error> {
        self.transfer_range(lba, 1)?;
        let arg = if self.block_addressing {
            lba
        } else {
            lba * SECTOR as u32
        };
        self.read_block(17, arg, dest)
    }

    /// TX_INT_CLK_SEL on or off (write data hold time).
    pub fn set_tx_hold(&self, on: bool) {
        let v = self.r32(TX_CFG) & !(1 << 30);
        self.w32(TX_CFG, v | if on { 1 << 30 } else { 0 });
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
                    self.w8(
                        HOST_CONTROL,
                        (self.r8(HOST_CONTROL) & !(HC_4BIT | HC_8BIT)) | hc,
                    );
                    if self.read_block(8, 0, addr).is_ok() {
                        return width;
                    }
                    self.w8(HOST_CONTROL, self.r8(HOST_CONTROL) & !(HC_4BIT | HC_8BIT));
                    let _ = self.switch(3, 183, 0);
                }
                1
            }
            Kind::Sd => {
                let ok =
                    self.command(55, self.rca << 16, R1).is_ok() && self.command(6, 2, R1).is_ok();
                if ok {
                    self.w8(HOST_CONTROL, (self.r8(HOST_CONTROL) & !HC_8BIT) | HC_4BIT);
                    let arg = 0;
                    if self.read_block(17, arg, addr).is_ok() {
                        return 4;
                    }
                    self.w8(HOST_CONTROL, self.r8(HOST_CONTROL) & !HC_4BIT);
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
    use super::*;
    fn card(block_addressing: bool, kind: Kind, n: u64) -> Mmc {
        Mmc {
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
        assert!(card(false, Kind::Emmc, 0).trim(0, 1).is_err());
        assert!(card(true, Kind::Sd, 100).trim(0, 1).is_err());
        assert!(card(true, Kind::Sd, 100).trim(0, 0).is_ok());
    }
}
