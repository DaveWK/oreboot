// SPDX-License-Identifier: GPL-2.0-only
//! Allwinner SMHC (SD/MMC host) driver, polled PIO: the microSD card in
//! SMHC0 (read-only) and the eMMC module in SMHC2.
//!
//! SD: just enough to read the boot layout and images from the card: SD v2
//! identification, 4-bit bus (ACMD6), default speed. The module clock
//! comes straight from the 24 MHz oscillator; in "new timing" mode the
//! controller halves it, so data transfers run at 12 MHz. PF0..PF5 are all
//! SMHC0 (the earlier PF4/DAT3 workaround, a GPIO held high and a 1-bit
//! bus for one board, is gone).
//!
//! eMMC (SMHC2, `new_smhc2`): the controller side of
//! `cubie_a7a_fastboot::emmc` (identification, EXT_CSD, partition access,
//! bus width): PC0..PC16 function 3 except the SPI0 pins PC2/3/4/12, 1-bit
//! then up to 8-bit, at the same 12 MHz legacy rate.
//!
//! Read-only by construction for SD: `cmd_data`, the only writer of the
//! command register for reads, refuses every command index outside
//! `cubie_a7a_fastboot::sd::ALLOWED` (identification and read commands;
//! no write, erase, lock, switch or programming command), and after CMD55
//! everything but ACMD41 and ACMD6 1/4-bit
//! (`cubie_a7a_fastboot::sd::app_command_allowed`), in every build.
//! For the eMMC it applies `cubie_a7a_fastboot::emmc::command_allowed`
//! (index and argument: CMD6 only to PARTITION_ACCESS and BUS_WIDTH), and
//! the block write path (`cmd_write`, CMD24/25 only) exists only in
//! `fastboot-emmc-write` builds.

use util::mmio::{read32, write32};

use crate::ccu;
use crate::gpio::{self, Port, Pull};
use crate::soc::{time_us, udelay};

const GCTRL: usize = 0x00;
const CLKDIV: usize = 0x04;
const TMOUT: usize = 0x08;
const CTYPE: usize = 0x0c;
#[cfg(any(medium, feature = "fastboot"))]
const HWRST: usize = 0x78;
/// EMMC_DDR_SBIT_DET: HALF_START_BIT, set for eMMC 4.5 and later, clear
/// for SD.
#[cfg(any(medium, feature = "fastboot"))]
const EMMC_DDR_SBIT_DET: usize = 0x10c;
const BLKSZ: usize = 0x10;
const BYTECNT: usize = 0x14;
const CMD: usize = 0x18;
const CMDARG: usize = 0x1c;
const RESP0: usize = 0x20;
const RESP1: usize = 0x24;
const RESP2: usize = 0x28;
const RESP3: usize = 0x2c;
const IMASK: usize = 0x30;
const RINT: usize = 0x38;
const STATUS: usize = 0x3c;
const NTSR: usize = 0x5c;
const SAMP_DL: usize = 0x144;
const FIFO: usize = 0x200;

const GCTRL_SOFT_RST: u32 = 1 << 0;
const GCTRL_FIFO_RST: u32 = 1 << 1;
const GCTRL_DMA_RST: u32 = 1 << 2;
const GCTRL_FIFO_AHB: u32 = 1 << 31;

const CLKDIV_CCLK_EN: u32 = 1 << 16;

const CMD_RESP: u32 = 1 << 6;
const CMD_LONG_RESP: u32 = 1 << 7;
const CMD_CHK_CRC: u32 = 1 << 8;
const CMD_DATA: u32 = 1 << 9;
#[cfg(feature = "fastboot-emmc-write")]
const CMD_WRITE: u32 = 1 << 10;
const CMD_AUTO_STOP: u32 = 1 << 12;
const CMD_WAIT_PRE_OVER: u32 = 1 << 13;
const CMD_SEND_INIT_SEQ: u32 = 1 << 15;
const CMD_UPCLK_ONLY: u32 = 1 << 21;
const CMD_START: u32 = 1 << 31;

const RINT_RESP_ERR: u32 = 1 << 1;
const RINT_CMD_DONE: u32 = 1 << 2;
const RINT_DATA_OVER: u32 = 1 << 3;
const RINT_RESP_CRC: u32 = 1 << 6;
const RINT_DATA_CRC: u32 = 1 << 7;
const RINT_RESP_TIMEOUT: u32 = 1 << 8;
const RINT_DATA_TIMEOUT: u32 = 1 << 9;
const RINT_VOLT_SWITCH: u32 = 1 << 10;
const RINT_FIFO_RUN_ERR: u32 = 1 << 11;
const RINT_HW_LOCKED: u32 = 1 << 12;
const RINT_START_BIT_ERR: u32 = 1 << 13;
const RINT_AUTO_CMD_DONE: u32 = 1 << 14;
const RINT_END_BIT_ERR: u32 = 1 << 15;
const RINT_ERRORS: u32 = RINT_RESP_ERR
    | RINT_RESP_CRC
    | RINT_DATA_CRC
    | RINT_RESP_TIMEOUT
    | RINT_DATA_TIMEOUT
    | RINT_VOLT_SWITCH
    | RINT_FIFO_RUN_ERR
    | RINT_HW_LOCKED
    | RINT_START_BIT_ERR
    | RINT_END_BIT_ERR;

const STATUS_FIFO_EMPTY: u32 = 1 << 2;
#[cfg(feature = "fastboot-emmc-write")]
const STATUS_FIFO_FULL: u32 = 1 << 3;
const STATUS_CARD_BUSY: u32 = 1 << 9;

const NTSR_MODE_NEW: u32 = 1 << 31;
/// Enable sample-delay calibration with zero delay (as U-Boot on A733).
const SAMP_DL_CAL_SW_EN: u32 = 1 << 7;

pub const BLOCK_SIZE: usize = 512;
/// Blocks per CMD18, bounding the time a single transfer can take.
const MAX_BLOCKS_PER_CMD: u32 = 128;

// Fields are only read through Debug when reporting failures.
#[allow(dead_code)]
#[derive(Debug)]
pub enum Error {
    Timeout(&'static str),
    Cmd { idx: u32, rint: u32 },
    NoCard,
    Unsupported(&'static str),
}

#[derive(Clone, Copy, PartialEq)]
enum Resp {
    None,
    R1,
    R1b,
    R2,
    /// R3: OCR, no CRC
    R3,
    R6,
    R7,
}

pub struct Smhc {
    base: usize,
    rca: u32,
    high_capacity: bool,
    /// The next command is an application command (CMD55 just went out).
    app: core::cell::Cell<bool>,
    /// SMHC2 with an eMMC: the eMMC allow-list applies.
    #[cfg(any(medium, feature = "fastboot"))]
    emmc: bool,
    /// The eMMC module detect (GPADC0) is consulted.
    #[cfg(any(medium, feature = "fastboot"))]
    detect: bool,
}

/// The eMMC write commands exist in this build.
#[cfg(any(medium, feature = "fastboot"))]
const EMMC_WRITE: bool = cfg!(feature = "fastboot-emmc-write");

/// SDC2 pins on port C, function 3 (SoC pinout): DS PC0, RST PC1, CLK PC5,
/// CMD PC6, D0 PC10, D1 PC13, D2 PC15, D3 PC8, D4 PC9, D5 PC11, D6 PC14,
/// D7 PC16. PC2/3/4/12 are SPI0 (the NOR), PC7 the EEPROM write protect.
#[cfg(any(medium, feature = "fastboot"))]
const SDC2_PINS: [u32; 12] = [0, 1, 5, 6, 8, 9, 10, 11, 13, 14, 15, 16];

impl Smhc {
    /// Clock, reset and pin setup for SMHC0 on PF0..PF5.
    pub fn new_smhc0(base: usize) -> Self {
        // PF0 D1, PF1 D0, PF2 CLK, PF3 CMD, PF4 D3, PF5 D2: function 2.
        for pin in 0..6 {
            gpio::set_function(Port::F, pin, 2);
            gpio::set_pull(Port::F, pin, if pin == 2 { Pull::None } else { Pull::Up });
            gpio::set_drive(Port::F, pin, 2);
        }
        // Module clock: gate on, source OSC24M, N = M = 1.
        write32(ccu::SMHC0_CLK, 1 << 31);
        ccu::bus_enable(ccu::SMHC0_BGR).unwrap_or_else(|error| crate::clock_failure(error));
        Self {
            base,
            rca: 0,
            high_capacity: false,
            app: core::cell::Cell::new(false),
            #[cfg(any(medium, feature = "fastboot"))]
            emmc: false,
            #[cfg(any(medium, feature = "fastboot"))]
            detect: false,
        }
    }

    /// Clock, reset and pin setup for SMHC2 and the eMMC module (VCC-PC,
    /// 1.8 V, CLDO5; the module's VCC is SWOUT2, on from power-up and
    /// switched on again by `axp` as dc1sw2). `detect`: consult the
    /// module detect (BOOT-SEL on GPADC0) before identification; the eMMC
    /// boot leaves it off (the mask ROM just loaded bt0 from the module).
    #[cfg(any(medium, feature = "fastboot"))]
    pub fn new_smhc2(base: usize, detect: bool) -> Self {
        for pin in SDC2_PINS {
            // CLK unpulled; DS (unused below HS400) pulled down; CMD, data
            // and RST pulled up.
            let pull = match pin {
                5 => Pull::None,
                0 => Pull::Down,
                _ => Pull::Up,
            };
            gpio::set_pull(Port::C, pin, pull);
            gpio::set_drive(Port::C, pin, if pin == 5 { 3 } else { 2 });
            gpio::set_function(Port::C, pin, 3);
        }
        // Module clock: gate on, source OSC24M, N = M = 1 (12 MHz card
        // clock in new timing mode, as SMHC0).
        write32(ccu::SMHC2_CLK, 1 << 31);
        ccu::bus_enable(ccu::SMHC2_BGR).unwrap_or_else(|error| crate::clock_failure(error));
        let s = Self {
            base,
            rca: 0,
            high_capacity: false,
            app: core::cell::Cell::new(false),
            emmc: true,
            detect,
        };
        // RST_n high (the card ignores it unless RST_n_FUNCTION enables it;
        // never set here).
        s.w(HWRST, 1);
        s
    }

    fn r(&self, off: usize) -> u32 {
        read32(self.base + off)
    }

    fn w(&self, off: usize, v: u32) {
        write32(self.base + off, v)
    }

    fn wait<F: Fn(&Self) -> bool>(
        &self,
        what: &'static str,
        us: u64,
        cond: F,
    ) -> Result<(), Error> {
        let t0 = time_us();
        while !cond(self) {
            if time_us() - t0 > us {
                return Err(Error::Timeout(what));
            }
        }
        Ok(())
    }

    fn reset_controller(&self) -> Result<(), Error> {
        self.w(GCTRL, GCTRL_SOFT_RST | GCTRL_FIFO_RST | GCTRL_DMA_RST);
        self.wait("gctrl reset", 10_000, |s| {
            s.r(GCTRL) & (GCTRL_SOFT_RST | GCTRL_FIFO_RST | GCTRL_DMA_RST) == 0
        })
    }

    fn update_clock(&self) -> Result<(), Error> {
        self.w(CMD, CMD_START | CMD_UPCLK_ONLY | CMD_WAIT_PRE_OVER);
        let r = self.wait("clock update", 20_000, |s| s.r(CMD) & CMD_START == 0);
        self.w(RINT, 0xffff_ffff);
        r
    }

    /// Card clock = 12 MHz / (2 * div), or 12 MHz for div 0.
    fn set_card_clock(&self, div: u32) -> Result<(), Error> {
        self.w(CLKDIV, self.r(CLKDIV) & !CLKDIV_CCLK_EN);
        self.update_clock()?;
        self.w(CLKDIV, div & 0xff);
        self.w(SAMP_DL, SAMP_DL_CAL_SW_EN);
        self.update_clock()?;
        self.w(CLKDIV, (div & 0xff) | CLKDIV_CCLK_EN);
        self.update_clock()
    }

    fn cmd(&self, idx: u32, arg: u32, resp: Resp) -> Result<[u32; 4], Error> {
        self.cmd_data(idx, arg, resp, None)
    }

    fn cmd_data(
        &self,
        idx: u32,
        arg: u32,
        resp: Resp,
        data: Option<&mut [u32]>,
    ) -> Result<[u32; 4], Error> {
        let sd_allowed = if self.app.get() {
            cubie_a7a_fastboot::sd::app_command_allowed(idx, arg)
        } else {
            cubie_a7a_fastboot::sd::command_allowed(idx)
        };
        #[cfg(not(any(medium, feature = "fastboot")))]
        if !sd_allowed {
            return Err(Error::Unsupported("SD command not allowed (read-only)"));
        }
        #[cfg(any(medium, feature = "fastboot"))]
        if self.emmc {
            if !cubie_a7a_fastboot::emmc::command_allowed(idx, arg, EMMC_WRITE)
                || matches!(idx, 24 | 25)
            {
                return Err(Error::Unsupported("eMMC command not allowed"));
            }
        } else if !sd_allowed {
            return Err(Error::Unsupported("SD command not allowed (read-only)"));
        }
        let mut c = CMD_START | idx;
        if idx == 0 {
            c |= CMD_SEND_INIT_SEQ;
        }
        match resp {
            Resp::None => {}
            Resp::R2 => c |= CMD_RESP | CMD_LONG_RESP | CMD_CHK_CRC,
            Resp::R3 => c |= CMD_RESP,
            _ => c |= CMD_RESP | CMD_CHK_CRC,
        }
        let words = data.as_ref().map(|d| d.len()).unwrap_or(0);
        if words > 0 {
            let bytes = (words * 4) as u32;
            c |= CMD_DATA | CMD_WAIT_PRE_OVER;
            if bytes > BLOCK_SIZE as u32 {
                c |= CMD_AUTO_STOP;
            }
            self.w(BLKSZ, BLOCK_SIZE as u32);
            self.w(BYTECNT, bytes);
            self.w(GCTRL, self.r(GCTRL) | GCTRL_FIFO_AHB);
        }
        self.w(RINT, 0xffff_ffff);
        self.w(CMDARG, arg);
        self.w(CMD, c);

        let fail = |rint: u32| {
            self.recover();
            Error::Cmd { idx, rint }
        };

        if let Some(buf) = data {
            let t0 = time_us();
            for word in buf.iter_mut() {
                while self.r(STATUS) & STATUS_FIFO_EMPTY != 0 {
                    let rint = self.r(RINT);
                    if rint & RINT_ERRORS != 0 {
                        return Err(fail(rint));
                    }
                    if time_us() - t0 > 1_000_000 {
                        self.recover();
                        return Err(Error::Timeout("data fifo"));
                    }
                }
                *word = self.r(FIFO);
            }
        }

        let mut done = RINT_CMD_DONE;
        if words > 0 {
            done |= RINT_DATA_OVER;
            if words * 4 > BLOCK_SIZE {
                done |= RINT_AUTO_CMD_DONE;
            }
        }
        let t0 = time_us();
        loop {
            let rint = self.r(RINT);
            if rint & RINT_ERRORS != 0 {
                return Err(fail(rint));
            }
            if rint & done == done {
                break;
            }
            if time_us() - t0 > 1_000_000 {
                self.recover();
                return Err(Error::Timeout("command"));
            }
        }
        if resp == Resp::R1b {
            self.wait("card busy", 1_000_000, |s| {
                s.r(STATUS) & STATUS_CARD_BUSY == 0
            })?;
        }
        let out = if resp == Resp::R2 {
            [self.r(RESP3), self.r(RESP2), self.r(RESP1), self.r(RESP0)]
        } else {
            [self.r(RESP0), 0, 0, 0]
        };
        self.w(RINT, 0xffff_ffff);
        Ok(out)
    }

    fn recover(&self) {
        let _ = self.reset_controller();
        let _ = self.update_clock();
        self.w(RINT, 0xffff_ffff);
    }

    fn app_cmd(&self, idx: u32, arg: u32, resp: Resp) -> Result<[u32; 4], Error> {
        self.cmd(55, self.rca << 16, Resp::R1)?;
        self.app.set(true);
        let r = self.cmd(idx, arg, resp);
        self.app.set(false);
        r
    }

    /// Identify the card and put it into 4-bit transfer state.
    pub fn init_card(&mut self) -> Result<(), Error> {
        self.reset_controller()?;
        self.w(NTSR, self.r(NTSR) | NTSR_MODE_NEW);
        self.w(TMOUT, 0xffff_ff40);
        self.w(IMASK, 0);
        self.w(RINT, 0xffff_ffff);
        self.w(CTYPE, 0);
        // <= 400 kHz for identification regardless of the /2 in new mode.
        self.set_card_clock(30)?;
        udelay(1000);

        self.cmd(0, 0, Resp::None)?;
        // SEND_IF_COND: 2.7-3.6 V, check pattern 0xaa
        let v2 = match self.cmd(8, 0x1aa, Resp::R7) {
            Ok(r) if r[0] & 0xfff == 0x1aa => true,
            Ok(_) => return Err(Error::Unsupported("bad CMD8 echo")),
            Err(_) => false,
        };
        let hcs = if v2 { 1 << 30 } else { 0 };
        let t0 = time_us();
        let ocr = loop {
            let ocr = self
                .app_cmd(41, hcs | 0x00ff_8000, Resp::R3)
                .map_err(|e| match e {
                    Error::Cmd { idx: 55, .. } => Error::NoCard,
                    e => e,
                })?[0];
            if ocr & (1 << 31) != 0 {
                break ocr;
            }
            if time_us() - t0 > 1_000_000 {
                return Err(Error::Timeout("ACMD41"));
            }
            udelay(1000);
        };
        self.high_capacity = ocr & (1 << 30) != 0;
        self.cmd(2, 0, Resp::R2)?;
        self.rca = self.cmd(3, 0, Resp::R6)?[0] >> 16;
        self.cmd(7, self.rca << 16, Resp::R1b)?;
        // ACMD6 SET_BUS_WIDTH 4-bit, then the controller.
        self.app_cmd(6, 2, Resp::R1)?;
        self.w(CTYPE, 1);
        if !self.high_capacity {
            self.cmd(16, BLOCK_SIZE as u32, Resp::R1)?;
        }
        self.set_card_clock(0)?;
        println!(
            "[bt0] SD: {} card, RCA {:04x}, OCR {ocr:08x}, 4-bit bus",
            if self.high_capacity {
                "SDHC/SDXC"
            } else {
                "SDSC"
            },
            self.rca
        );
        Ok(())
    }

    /// Read `buf.len() * 4 / 512` blocks starting at block `lba`.
    pub fn read_blocks(&self, mut lba: u32, buf: &mut [u32]) -> Result<(), Error> {
        let words_per_block = BLOCK_SIZE / 4;
        assert!(buf.len() % words_per_block == 0);
        for chunk in buf.chunks_mut(words_per_block * MAX_BLOCKS_PER_CMD as usize) {
            let blocks = (chunk.len() / words_per_block) as u32;
            let addr = if self.high_capacity {
                lba
            } else {
                lba * BLOCK_SIZE as u32
            };
            let idx = if blocks > 1 { 18 } else { 17 };
            self.cmd_data(idx, addr, Resp::R1, Some(chunk))?;
            lba += blocks;
        }
        Ok(())
    }
}

impl Smhc {
    /// A block write (CMD24, CMD25 with auto stop) of `data`, then wait
    /// until the card releases DAT0: the only path that moves data to a
    /// card, eMMC only, in `fastboot-emmc-write` builds only.
    #[cfg(feature = "fastboot-emmc-write")]
    fn cmd_write(&self, idx: u32, arg: u32, data: &[u32]) -> Result<(), Error> {
        if !self.emmc
            || !matches!(idx, 24 | 25)
            || !cubie_a7a_fastboot::emmc::command_allowed(idx, arg, EMMC_WRITE)
            || data.is_empty()
            || !(data.len() * 4).is_multiple_of(BLOCK_SIZE)
            || (idx == 25) != (data.len() * 4 > BLOCK_SIZE)
        {
            return Err(Error::Unsupported("write not allowed"));
        }
        let bytes = (data.len() * 4) as u32;
        let mut c =
            CMD_START | idx | CMD_RESP | CMD_CHK_CRC | CMD_DATA | CMD_WRITE | CMD_WAIT_PRE_OVER;
        if idx == 25 {
            c |= CMD_AUTO_STOP;
        }
        self.w(BLKSZ, BLOCK_SIZE as u32);
        self.w(BYTECNT, bytes);
        self.w(GCTRL, self.r(GCTRL) | GCTRL_FIFO_AHB);
        self.w(RINT, 0xffff_ffff);
        self.w(CMDARG, arg);
        self.w(CMD, c);
        let fail = |rint: u32| {
            self.recover();
            Error::Cmd { idx, rint }
        };
        let t0 = time_us();
        for &word in data {
            while self.r(STATUS) & STATUS_FIFO_FULL != 0 {
                let rint = self.r(RINT);
                if rint & RINT_ERRORS != 0 {
                    return Err(fail(rint));
                }
                if time_us() - t0 > 2_000_000 {
                    self.recover();
                    return Err(Error::Timeout("write fifo"));
                }
            }
            self.w(FIFO, word);
        }
        let mut done = RINT_CMD_DONE | RINT_DATA_OVER;
        if idx == 25 {
            done |= RINT_AUTO_CMD_DONE;
        }
        loop {
            let rint = self.r(RINT);
            if rint & RINT_ERRORS != 0 {
                return Err(fail(rint));
            }
            if rint & done == done {
                break;
            }
            if time_us() - t0 > 2_000_000 {
                self.recover();
                return Err(Error::Timeout("write"));
            }
        }
        self.wait("card busy after write", 2_000_000, |s| {
            s.r(STATUS) & STATUS_CARD_BUSY == 0
        })?;
        self.w(RINT, 0xffff_ffff);
        Ok(())
    }
}

/// The eMMC controller for cubie_a7a_fastboot::emmc (the eMMC boot path
/// and fastboot's eMMC targets; the gate in `cmd_data` applies).
#[cfg(any(medium, feature = "fastboot"))]
impl cubie_a7a_fastboot::emmc::MmcBus for Smhc {
    fn present(&mut self) -> Option<bool> {
        if !self.detect {
            return None;
        }
        // BOOT-SEL-ADC: R1 10K to 1.8 V; an eMMC module grounds eMMC_PLUG
        // through R216 3.9K (~505 mV, the mask ROM's EMMC_BOOT/EMMC_USER
        // band 0x22c..0x57b); none: ~1800 mV (schematic sheets 2 and 9).
        let mv = crate::gpadc::read_mv(0)?;
        println!("[bt0] eMMC detect: BOOT-SEL (GPADC0) {mv} mV");
        match mv {
            200..=700 => Some(true),
            1600.. => Some(false),
            _ => None,
        }
    }

    fn clock(&mut self, ident: bool) -> Result<(), cubie_a7a_fastboot::sd::BusError> {
        let r = if ident {
            self.reset_controller().and_then(|()| {
                self.w(NTSR, self.r(NTSR) | NTSR_MODE_NEW);
                self.w(TMOUT, 0xffff_ff40);
                self.w(IMASK, 0);
                self.w(RINT, 0xffff_ffff);
                self.w(CTYPE, 0);
                self.w(EMMC_DDR_SBIT_DET, 1);
                self.set_card_clock(30)
            })
        } else {
            self.set_card_clock(0)
        };
        udelay(1000);
        r.map_err(|_| cubie_a7a_fastboot::sd::BusError::Error)
    }

    fn set_width(&mut self, width: u32) -> Result<(), cubie_a7a_fastboot::sd::BusError> {
        let v = match width {
            1 => 0,
            4 => 1,
            8 => 2,
            _ => return Err(cubie_a7a_fastboot::sd::BusError::Error),
        };
        self.w(CTYPE, v);
        Ok(())
    }

    fn cmd(
        &mut self,
        idx: u32,
        arg: u32,
        resp: cubie_a7a_fastboot::sd::Resp,
    ) -> Result<[u32; 4], cubie_a7a_fastboot::sd::BusError> {
        self.cmd_data(idx, arg, map_resp(resp), None)
            .map_err(bus_error)
    }

    fn read(
        &mut self,
        idx: u32,
        arg: u32,
        buf: &mut [u32],
    ) -> Result<(), cubie_a7a_fastboot::sd::BusError> {
        self.cmd_data(idx, arg, Resp::R1, Some(buf))
            .map(|_| ())
            .map_err(bus_error)
    }

    #[cfg(feature = "fastboot-emmc-write")]
    fn write(
        &mut self,
        idx: u32,
        arg: u32,
        buf: &[u32],
    ) -> Result<(), cubie_a7a_fastboot::sd::BusError> {
        self.cmd_write(idx, arg, buf).map_err(|e| {
            println!("[fastboot] eMMC CMD{idx} {arg:#x}: {e:?}");
            bus_error(e)
        })
    }

    fn delay_ms(&mut self, ms: u32) {
        udelay(ms as u64 * 1000);
    }

    fn kick(&mut self) {
        crate::recovery::kick();
    }
}

/// The controller for fastboot's `fetch:sd` (cubie_a7a_fastboot::sd runs
/// identification and reads over it; the same gate in `cmd_data` applies).
#[cfg(feature = "fastboot")]
impl cubie_a7a_fastboot::sd::SdBus for Smhc {
    fn card_present(&mut self) -> Option<bool> {
        // PF6, the card-detect switch: low with a card in (board DT
        // cd-gpios, active low, pull-up).
        gpio::set_function(Port::F, SD_CD_PIN, 0);
        gpio::set_pull(Port::F, SD_CD_PIN, Pull::Up);
        udelay(100);
        Some(!gpio::read(Port::F, SD_CD_PIN))
    }

    fn clock(&mut self, ident: bool) -> Result<(), cubie_a7a_fastboot::sd::BusError> {
        use cubie_a7a_fastboot::sd::BusError;
        let r = if ident {
            self.reset_controller().and_then(|()| {
                self.w(NTSR, self.r(NTSR) | NTSR_MODE_NEW);
                self.w(TMOUT, 0xffff_ff40);
                self.w(IMASK, 0);
                self.w(RINT, 0xffff_ffff);
                self.w(CTYPE, 0);
                self.set_card_clock(30)
            })
        } else {
            self.set_card_clock(0)
        };
        udelay(1000);
        r.map_err(|_| BusError::Error)
    }

    fn cmd(
        &mut self,
        idx: u32,
        arg: u32,
        resp: cubie_a7a_fastboot::sd::Resp,
    ) -> Result<[u32; 4], cubie_a7a_fastboot::sd::BusError> {
        let r = self
            .cmd_data(idx, arg, map_resp(resp), None)
            .map_err(bus_error);
        if idx == 3 {
            if let Ok(v) = r {
                self.rca = v[0] >> 16;
            }
        }
        r
    }

    fn read(
        &mut self,
        idx: u32,
        arg: u32,
        buf: &mut [u32],
    ) -> Result<(), cubie_a7a_fastboot::sd::BusError> {
        self.cmd_data(idx, arg, Resp::R1, Some(buf))
            .map(|_| ())
            .map_err(bus_error)
    }

    fn delay_ms(&mut self, ms: u32) {
        udelay(ms as u64 * 1000);
    }
}

#[cfg(feature = "fastboot")]
const SD_CD_PIN: u32 = 6;

#[cfg(any(medium, feature = "fastboot"))]
fn map_resp(r: cubie_a7a_fastboot::sd::Resp) -> Resp {
    use cubie_a7a_fastboot::sd::Resp as R;
    match r {
        R::None => Resp::None,
        R::R1 => Resp::R1,
        R::R1b => Resp::R1b,
        R::R2 => Resp::R2,
        R::R3 => Resp::R3,
        R::R6 => Resp::R6,
        R::R7 => Resp::R7,
    }
}

#[cfg(any(medium, feature = "fastboot"))]
fn bus_error(e: Error) -> cubie_a7a_fastboot::sd::BusError {
    use cubie_a7a_fastboot::sd::BusError;
    match e {
        Error::Cmd { rint, .. } if rint & RINT_RESP_TIMEOUT != 0 => BusError::Timeout,
        Error::Timeout(_) | Error::NoCard => BusError::Timeout,
        _ => BusError::Error,
    }
}
