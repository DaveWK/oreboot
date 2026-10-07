//! Allwinner TWI (Marvell mv64xxx-style I2C), polled, register-at-a-time.
//! Used on R_TWI0 (PL0 = SCK, PL1 = SDA) to reach the AXP8191 PMIC: by bt0
//! for setup and by main for power-off. bt0 muxes the pins and ungates the
//! bus clock; [`Twi::new`] only resets and programs the controller.

use util::mmio::{read32, write32};

use crate::time::udelay;

const DATA: usize = 0x08;
const CNTR: usize = 0x0c;
const STAT: usize = 0x10;
const CCR: usize = 0x14;
const SRST: usize = 0x18;
const EFR: usize = 0x1c;

const CNTR_A_ACK: u32 = 1 << 2;
const CNTR_INT_FLAG: u32 = 1 << 3;
const CNTR_M_STP: u32 = 1 << 4;
const CNTR_M_STA: u32 = 1 << 5;
const CNTR_BUS_EN: u32 = 1 << 6;

// Status codes
const ST_START: u32 = 0x08;
const ST_RESTART: u32 = 0x10;
const ST_ADDR_W_ACK: u32 = 0x18;
const ST_DATA_W_ACK: u32 = 0x28;
const ST_ADDR_R_ACK: u32 = 0x40;
const ST_DATA_R_NACK: u32 = 0x58;
const ST_IDLE: u32 = 0xf8;

const TIMEOUT_US: u64 = 10_000;

// Fields are only read through Debug when reporting failures.
#[allow(dead_code)]
#[derive(Debug)]
pub enum Error {
    Timeout,
    Status(u32),
}

pub struct Twi {
    base: usize,
}

impl Twi {
    /// Set up for 400 kHz from a 24 MHz parent: F1 = 24 MHz / 2^N,
    /// SCL = F1 / (10 * (M + 1)) with N = 1, M = 2.
    pub fn new(base: usize) -> Self {
        write32(base + SRST, 1);
        udelay(10);
        write32(base + SRST, 0);
        write32(base + CCR, (2 << 3) | 1);
        write32(base + EFR, 0);
        write32(base + CNTR, CNTR_BUS_EN);
        Self { base }
    }

    fn wait_int(&self) -> Result<u32, Error> {
        let mut t = 0;
        while read32(self.base + CNTR) & CNTR_INT_FLAG == 0 {
            if t >= TIMEOUT_US {
                return Err(Error::Timeout);
            }
            udelay(1);
            t += 1;
        }
        Ok(read32(self.base + STAT))
    }

    /// Clear INT_FLAG (write 1) while keeping the bus enabled, plus `extra`.
    fn next(&self, extra: u32) {
        let c = read32(self.base + CNTR) & CNTR_A_ACK;
        write32(self.base + CNTR, c | CNTR_BUS_EN | CNTR_INT_FLAG | extra);
    }

    fn start(&self, first: bool) -> Result<(), Error> {
        if first {
            write32(self.base + CNTR, CNTR_BUS_EN | CNTR_M_STA);
        } else {
            self.next(CNTR_M_STA);
        }
        match self.wait_int()? {
            ST_START | ST_RESTART => Ok(()),
            s => Err(Error::Status(s)),
        }
    }

    fn send(&self, byte: u8, expect: u32) -> Result<(), Error> {
        write32(self.base + DATA, byte as u32);
        self.next(0);
        match self.wait_int()? {
            s if s == expect => Ok(()),
            s => Err(Error::Status(s)),
        }
    }

    fn stop(&self) {
        self.next(CNTR_M_STP);
        let mut t = 0;
        while read32(self.base + CNTR) & CNTR_M_STP != 0 && t < TIMEOUT_US {
            udelay(1);
            t += 1;
        }
        let mut t = 0;
        while read32(self.base + STAT) != ST_IDLE && t < TIMEOUT_US {
            udelay(1);
            t += 1;
        }
    }

    fn transfer(&self, f: impl FnOnce(&Self) -> Result<u8, Error>) -> Result<u8, Error> {
        let r = f(self);
        self.stop();
        r
    }

    pub fn read_reg(&self, chip: u8, reg: u8) -> Result<u8, Error> {
        self.transfer(|t| {
            t.start(true)?;
            t.send(chip << 1, ST_ADDR_W_ACK)?;
            t.send(reg, ST_DATA_W_ACK)?;
            t.start(false)?;
            t.send((chip << 1) | 1, ST_ADDR_R_ACK)?;
            // Single byte: receive without ACK.
            write32(t.base + CNTR, CNTR_BUS_EN | CNTR_INT_FLAG);
            match t.wait_int()? {
                ST_DATA_R_NACK => Ok(read32(t.base + DATA) as u8),
                s => Err(Error::Status(s)),
            }
        })
    }

    pub fn write_reg(&self, chip: u8, reg: u8, val: u8) -> Result<(), Error> {
        self.transfer(|t| {
            t.start(true)?;
            t.send(chip << 1, ST_ADDR_W_ACK)?;
            t.send(reg, ST_DATA_W_ACK)?;
            t.send(val, ST_DATA_W_ACK)?;
            Ok(0)
        })
        .map(|_| ())
    }
}
