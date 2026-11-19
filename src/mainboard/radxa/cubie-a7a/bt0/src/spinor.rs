//! SPI NOR boot (feature `spinor`): read the boot layout table and the
//! images it names from the board's SPI NOR instead of the SD card.
//!
//! The Cubie A7A carries a 64 Mbit 1.8 V NOR (schematic sheet 9, U7,
//! W25Q64FW) on SPI0: PC2 MOSI, PC3 CS0, PC4 MISO, PC12 CLK, all function
//! 5, VCC-PC (CLDO5, 1.8 V). WP#/IO2 and HOLD#/IO3 only have pull-ups to
//! the flash supply, so the bus is single-bit. The mask ROM loads bt0 from
//! NOR offset 0 (eGON header, as on the SD card); everything else keeps
//! the SD card's absolute offsets, so the same layout table and images
//! work unchanged: table at `layout::TABLE_OFFSET` (0x1ff000), oreboot
//! main at 0x200000, the EDK2 FD at 0x300000, all below the 8 MiB end.
//!
//! Read-only by construction: the only opcodes this module ever puts in
//! the TX FIFO are READ (0x03) and RDID (0x9f). Each command is loaded
//! after a FIFO reset, and the data phase sends no TX bytes (MTC = command
//! length), so no other byte can reach the flash as an opcode.
//!
//! Controller settings are the ones the read-only FEL dump of this NOR used
//! (two identical 8 MiB passes): SYS_CLK24M / 4 = 6 MHz,
//! master, mode 0, manual CS, normal sample mode. GCR.TP_EN stops the
//! clock while the RX FIFO is full, so long bursts cannot overrun it.

use util::mmio::{read32, read8, write32, write8};

use crate::ccu;
use crate::gpio::{self, Port};
use crate::recovery;
use crate::soc::time_us;

const BASE: usize = 0x0254_0000;
const GCR: usize = BASE + 0x04;
const TCR: usize = BASE + 0x08;
const IER: usize = BASE + 0x10;
const ISR: usize = BASE + 0x14;
const FCR: usize = BASE + 0x18;
const FSR: usize = BASE + 0x1c;
const MBC: usize = BASE + 0x30;
const MTC: usize = BASE + 0x34;
const BCC: usize = BASE + 0x38;
const TXD: usize = BASE + 0x200;
const RXD: usize = BASE + 0x300;

const GCR_EN: u32 = 1 << 0;
const GCR_MASTER: u32 = 1 << 1;
const GCR_TP: u32 = 1 << 7;
const GCR_SRST: u32 = 1 << 31;
const TCR_SS_OWNER: u32 = 1 << 6;
const TCR_SS_LEVEL: u32 = 1 << 7;
const TCR_SDM: u32 = 1 << 13;
const TCR_XCH: u32 = 1 << 31;
const FCR_RF_RST: u32 = 1 << 15;
const FCR_TF_RST: u32 = 1 << 31;
const FSR_RF_CNT: u32 = 0xff;
const FSR_TF_CNT: u32 = 0xff << 16;

/// SPI0_CLK: gate, source SYS_CLK24M (0), N = 1, M = FACTOR_M + 1.
const CLK_FACTOR_M: u32 = 3; // 24 MHz / 4 = 6 MHz
const CLK_GATE: u32 = 1 << 31;

/// The part on the board: W25Q64FW, 8 MiB, 3-byte addresses.
pub const NOR_SIZE: u32 = 0x0080_0000;
const OP_READ: u8 = 0x03;
const OP_RDID: u8 = 0x9f;

/// Bytes per READ command; the watchdog is restarted after each.
const CHUNK: usize = 0x1_0000;
/// Per command: 64 KiB at 6 MHz take ~90 ms.
const CMD_TIMEOUT_US: u64 = 1_000_000;

// Fields are only read through Debug when reporting failures.
#[allow(dead_code)]
#[derive(Debug)]
pub enum Error {
    Timeout { what: &'static str, fsr: u32 },
    Range { offset: u32, len: usize },
    Fifo(u32),
}

pub struct SpiNor;

fn wait_clear(reg: usize, bits: u32, what: &'static str) -> Result<(), Error> {
    let t0 = time_us();
    while read32(reg) & bits != 0 {
        if time_us() - t0 > CMD_TIMEOUT_US {
            return Err(Error::Timeout {
                what,
                fsr: read32(FSR),
            });
        }
    }
    Ok(())
}

fn cs(high: bool) {
    let t = read32(TCR) & !TCR_XCH;
    write32(TCR, if high { t | TCR_SS_LEVEL } else { t & !TCR_SS_LEVEL });
}

impl SpiNor {
    /// Clock, reset, controller and pins for SPI0. The controller drives
    /// CS high before the pins are switched to it.
    pub fn new() -> Result<Self, Error> {
        write32(ccu::SPI0_CLK, CLK_FACTOR_M);
        write32(ccu::SPI0_CLK, CLK_GATE | CLK_FACTOR_M);
        ccu::bus_enable(ccu::SPI0_BGR).unwrap_or_else(|error| crate::clock_failure(error));

        write32(GCR, GCR_SRST);
        wait_clear(GCR, GCR_SRST, "soft reset")?;
        write32(GCR, GCR_MASTER | GCR_TP);
        write32(IER, 0);
        write32(TCR, TCR_SS_OWNER | TCR_SS_LEVEL | TCR_SDM); // mode 0, CS0
        write32(FCR, FCR_RF_RST | FCR_TF_RST);
        wait_clear(FCR, FCR_RF_RST | FCR_TF_RST, "FIFO reset")?;
        write32(GCR, GCR_MASTER | GCR_TP | GCR_EN);

        for pin in [2, 3, 4, 12] {
            gpio::set_function(Port::C, pin, 5);
        }
        Ok(SpiNor)
    }

    /// One command: `cmd` (opcode and address) from the TX FIFO, then
    /// `out.len()` bytes clocked in with no TX bytes, CS low throughout.
    fn command(&self, cmd: &[u8], out: &mut [u8]) -> Result<(), Error> {
        debug_assert!(matches!(cmd[0], OP_READ | OP_RDID) && cmd.len() <= 4);
        write32(FCR, FCR_RF_RST | FCR_TF_RST);
        wait_clear(FCR, FCR_RF_RST | FCR_TF_RST, "FIFO reset")?;
        let fsr = read32(FSR);
        if fsr & (FSR_RF_CNT | FSR_TF_CNT) != 0 {
            return Err(Error::Fifo(fsr));
        }
        for &b in cmd {
            write8(TXD, b);
        }
        let total = cmd.len() + out.len();
        write32(ISR, 0xffff_ffff);
        write32(MBC, total as u32);
        write32(MTC, cmd.len() as u32);
        write32(BCC, cmd.len() as u32);
        cs(false);
        write32(TCR, read32(TCR) | TCR_XCH);
        // Every clocked byte lands in the RX FIFO, the command's too:
        // drop the first cmd.len() bytes.
        let t0 = time_us();
        let mut got = 0;
        while got < total {
            let n = (read32(FSR) & FSR_RF_CNT) as usize;
            for _ in 0..n.min(total - got) {
                let b = read8(RXD);
                if got >= cmd.len() {
                    out[got - cmd.len()] = b;
                }
                got += 1;
            }
            if n == 0 && time_us() - t0 > CMD_TIMEOUT_US {
                cs(true);
                return Err(Error::Timeout {
                    what: "receive",
                    fsr: read32(FSR),
                });
            }
        }
        let done = wait_clear(TCR, TCR_XCH, "burst end");
        cs(true);
        done
    }

    /// JEDEC ID (manufacturer, type, capacity), for the boot log.
    pub fn id(&self) -> Result<[u8; 3], Error> {
        let mut id = [0u8; 3];
        self.command(&[OP_RDID], &mut id)?;
        Ok(id)
    }

    /// Read `buf.len()` bytes from NOR offset `offset`, restarting the
    /// watchdog after each chunk.
    pub fn read(&self, offset: u32, buf: &mut [u8]) -> Result<(), Error> {
        if offset as u64 + buf.len() as u64 > NOR_SIZE as u64 {
            return Err(Error::Range {
                offset,
                len: buf.len(),
            });
        }
        let mut addr = offset;
        for chunk in buf.chunks_mut(CHUNK) {
            let a = addr.to_be_bytes();
            self.command(&[OP_READ, a[1], a[2], a[3]], chunk)?;
            recovery::kick();
            addr += chunk.len() as u32;
        }
        Ok(())
    }
}
