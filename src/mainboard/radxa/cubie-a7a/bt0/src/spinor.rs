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
//! Read-only by construction unless built with `fastboot-nor-write`: the
//! only opcodes the read path ever puts in the TX FIFO are READ (0x03),
//! RDID (0x9f) and RDSR1 (0x05), checked at run time against
//! `cubie_a7a_fastboot::nor::READ_OPCODES`. Each command is loaded after a FIFO reset, and the data
//! phase sends no TX bytes (MTC = command length), so no other byte can
//! reach the flash as an opcode. The erase/program part at the end of this
//! file exists only in `fastboot-nor-write` builds (fastboot `flash` and
//! `erase` behind cubie_a7a_fastboot::nor's guards).
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
const OP_RDSR1_RO: u8 = 0x05;

/// Bytes per READ command; the watchdog is restarted after each.
const CHUNK: usize = 0x1_0000;
/// Per command: 64 KiB at 6 MHz take ~90 ms.
const CMD_TIMEOUT_US: u64 = 1_000_000;

// Fields are only read through Debug when reporting failures.
#[allow(dead_code)]
#[derive(Debug, Clone, Copy)]
pub enum Error {
    Timeout { what: &'static str, fsr: u32 },
    Range { offset: u32, len: usize },
    Fifo(u32),
    /// A guard in this driver refused (never sent).
    Refused(&'static str),
    /// Status register 1 after WREN or a busy timeout.
    #[cfg(feature = "fastboot-nor-write")]
    Status(u8),
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
        // READ, RDID and RDSR1 only (cubie_a7a_fastboot::nor::READ_OPCODES),
        // checked at run time: no other byte goes out as an opcode here.
        if cmd.is_empty() || cmd.len() > 4 || !cubie_a7a_fastboot::nor::read_opcode_allowed(cmd[0]) {
            return Err(Error::Refused("opcode not allowed on the read path"));
        }
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

    /// Status register 1 (RDSR1), read only.
    pub fn status1(&self) -> Result<u8, Error> {
        let mut sr = [0u8];
        self.command(&[OP_RDSR1_RO], &mut sr)?;
        Ok(sr[0])
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

// ---------------------------------------------------------------------------
// Erase and program (feature `fastboot-nor-write`, for fastboot's guarded
// `flash`/`erase`; cubie_a7a_fastboot::nor has the guards and the plan).
// Built for the bench only.
//
// Opcodes this part can send, besides READ and RDID above: 06 WREN, 05 RDSR1,
// 20 4 KiB sector erase, D8 64 KiB block erase, 02 page program. No chip
// erase (C7/60), no status or configuration register write (01/31/11), no
// security/OTP register access, no lock, reset, 4-byte or quad command
// exists here. Every erase and program checks its range against the write
// window again, and WEL must read back set after each WREN.

#[cfg(feature = "fastboot-nor-write")]
mod write {
    use cubie_a7a_fastboot::nor::{self, NorFlash};
    use util::mmio::{read32, read8, write32, write8};

    use super::*;

    const OP_WREN: u8 = 0x06;
    const OP_RDSR1: u8 = 0x05;
    const OP_SE: u8 = 0x20;
    const OP_BE: u8 = 0xd8;
    const OP_PP: u8 = 0x02;
    const FIFO_DEPTH: u32 = 64;
    /// W25Q64FW maxima: sector erase 400 ms, block erase 2 s, page 3 ms.
    const ERASE_TIMEOUT_US: u64 = 3_000_000;
    const PROGRAM_TIMEOUT_US: u64 = 50_000;

    fn allowed(op: u8) -> bool {
        matches!(op, OP_READ | OP_RDID | OP_WREN | OP_RDSR1 | OP_SE | OP_BE | OP_PP)
    }

    impl SpiNor {
        /// One command, CS low throughout: `hdr` (opcode and address, 1 to 4
        /// bytes), `tx` data bytes, then `rx.len()` bytes clocked in. The TX
        /// FIFO is refilled while the burst runs; every clocked byte also
        /// lands in the RX FIFO (TP_EN pauses the clock while it is full).
        fn xfer(&self, hdr: &[u8], tx: &[u8], rx: &mut [u8]) -> Result<(), Error> {
            if hdr.is_empty() || hdr.len() > 4 || !allowed(hdr[0]) {
                return Err(Error::Refused("opcode not allowed"));
            }
            write32(FCR, FCR_RF_RST | FCR_TF_RST);
            wait_clear(FCR, FCR_RF_RST | FCR_TF_RST, "FIFO reset")?;
            let fsr = read32(FSR);
            if fsr & (FSR_RF_CNT | FSR_TF_CNT) != 0 {
                return Err(Error::Fifo(fsr));
            }
            let ntx = hdr.len() + tx.len();
            let total = ntx + rx.len();
            let byte = |i: usize| if i < hdr.len() { hdr[i] } else { tx[i - hdr.len()] };
            let mut sent = 0;
            while sent < ntx && sent < FIFO_DEPTH as usize {
                write8(TXD, byte(sent));
                sent += 1;
            }
            write32(ISR, 0xffff_ffff);
            write32(MBC, total as u32);
            write32(MTC, ntx as u32);
            write32(BCC, ntx as u32);
            cs(false);
            write32(TCR, read32(TCR) | TCR_XCH);
            let t0 = time_us();
            let mut got = 0;
            while got < total {
                while sent < ntx && (read32(FSR) & FSR_TF_CNT) >> 16 < FIFO_DEPTH {
                    write8(TXD, byte(sent));
                    sent += 1;
                }
                let n = (read32(FSR) & FSR_RF_CNT) as usize;
                for _ in 0..n.min(total - got) {
                    let b = read8(RXD);
                    if got >= ntx {
                        rx[got - ntx] = b;
                    }
                    got += 1;
                }
                if n == 0 && time_us() - t0 > CMD_TIMEOUT_US {
                    cs(true);
                    return Err(Error::Timeout {
                        what: "transfer",
                        fsr: read32(FSR),
                    });
                }
            }
            let done = wait_clear(TCR, TCR_XCH, "burst end");
            cs(true);
            done
        }

        fn rdsr1(&self) -> Result<u8, Error> {
            let mut sr = [0u8];
            self.xfer(&[OP_RDSR1], &[], &mut sr)?;
            Ok(sr[0])
        }

        /// WREN, then WEL must be set and no protection bit.
        fn write_enable(&self) -> Result<(), Error> {
            self.xfer(&[OP_WREN], &[], &mut [])?;
            let sr = self.rdsr1()?;
            if sr & nor::SR1_PROTECT != 0 {
                return Err(Error::Status(sr));
            }
            if sr & nor::SR1_WEL == 0 {
                return Err(Error::Status(sr));
            }
            Ok(())
        }

        fn wait_ready(&self, timeout_us: u64) -> Result<(), Error> {
            let t0 = time_us();
            loop {
                let sr = self.rdsr1()?;
                if sr & nor::SR1_BUSY == 0 {
                    return Ok(());
                }
                recovery::kick();
                if time_us() - t0 > timeout_us {
                    return Err(Error::Status(sr));
                }
            }
        }
    }

    fn addr(op: u8, a: u32) -> [u8; 4] {
        let b = a.to_be_bytes();
        [op, b[1], b[2], b[3]]
    }

    impl NorFlash for SpiNor {
        type Error = Error;

        fn jedec_id(&mut self) -> Result<[u8; 3], Error> {
            self.id()
        }

        fn status(&mut self) -> Result<u8, Error> {
            self.rdsr1()
        }

        fn read(&mut self, offset: u32, buf: &mut [u8]) -> Result<(), Error> {
            SpiNor::read(self, offset, buf)
        }

        fn erase(&mut self, offset: u32, len: u32) -> Result<(), Error> {
            let op = match len {
                nor::SECTOR => OP_SE,
                nor::BLOCK => OP_BE,
                _ => return Err(Error::Refused("erase size")),
            };
            if !offset.is_multiple_of(len) || !nor::in_window(offset, len) {
                return Err(Error::Refused("erase outside the write window"));
            }
            self.write_enable()?;
            self.xfer(&addr(op, offset), &[], &mut [])?;
            self.wait_ready(ERASE_TIMEOUT_US)
        }

        fn program(&mut self, offset: u32, data: &[u8]) -> Result<(), Error> {
            let n = data.len() as u32;
            if n == 0
                || n > nor::PAGE
                || offset / nor::PAGE != (offset + n - 1) / nor::PAGE
                || !nor::in_window(offset, n)
            {
                return Err(Error::Refused("program outside one page of the write window"));
            }
            self.write_enable()?;
            self.xfer(&addr(OP_PP, offset), data, &mut [])?;
            self.wait_ready(PROGRAM_TIMEOUT_US)
        }

        fn kick(&mut self) {
            recovery::kick();
        }
    }
}
