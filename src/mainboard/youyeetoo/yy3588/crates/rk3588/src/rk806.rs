// SPDX-License-Identifier: GPL-2.0-only
//! RK806 PMIC over SPI2 M2: check and, if needed, enable the DDR rails.
//!
//! Protocol and register map from mainline Linux (GPL-2.0):
//! drivers/mfd/rk8xx-spi.c (command byte = WRITE 0x80 / READ 0x00 |
//! CRC off | (value bytes - 1), then the register address low byte and
//! RK806_REG_H = 0x00) and include/linux/mfd/rk808.h / drivers/regulator/
//! rk808-regulator.c (POWER_EN0..5 with enable value BIT(id) and write
//! mask BIT(4 + id); BUCK5/6 = POWER_EN1 bits 0/1, BUCK9 = POWER_EN2 bit 0,
//! NLDO2 = POWER_EN3 bit 1; buck VSEL 500 mV plus 6.25 mV per step, LDO
//! VSEL 500 mV plus 12.5 mV per step; CHIP_NAME 0x5A / CHIP_VER 0x5B). SPI controller: RK3588
//! TRM Part 1 Chapter 31; pins SPI2_*_M2 on GPIO0_A5/A6/B1/B3 (P1 pp.957-958,
//! Linux rk3588-base-pinctrl.dtsi spi2m2_*); clk_spi2 from 24 MHz
//! (CRU_CLKSEL_CON59 [7:6] = 2).
//!
//! Rails (YY3588 DTS): BUCK5 vdd_ddr_s0, BUCK6 vdd2_ddr_s3, BUCK9
//! vddq_ddr_s0, NLDO2 vdd_ddr_pll_s0 (0.85 V). BUCK5 has a range in the
//! DTS (675-900 mV; 850 mV is used), NLDO2 a fixed 0.85 V; BUCK6/BUCK9 are
//! enabled only if off, their voltage is left as found. [`readback`] is
//! read-only (the bt0 log on every path); [`ddr_rails`] writes and is only
//! used by the own-DDR init.
//!
//! First hardware run (2026-11-09, usb-loader): CHIP_NAME read 0x00. Root
//! cause [I]: CTRLR0 was written as 0x1 (8-bit frames only), leaving
//! CTRLR0.bht = 0, "APB 16bit write/read, spi 8bit write/read" (TRM P1
//! p.1933): every TXDR write then puts two 8-bit frames on the wire, so the
//! command/address bytes went out as `00 00 5A 00 ...` and the RK806 saw
//! address 0x5A00. Linux drivers/spi/spi-rockchip.c rockchip_spi_config()
//! always sets bht = 1 (8-bit APB), ssd = 1 and em = 1 (big endian); this
//! driver now writes the same CTRLR0 and also ungates and resets SPI2
//! before use (CRU_GATE_CON14 / CRU_SOFTRST_CON14, reset values already
//! "running", P1 pp.123/174) so the state the BootROM or rkbin left does
//! not matter.
//!
//! Second run (2026-11-09 00:41, stepping-stone, CTRLR0 = 0x2C01): SPI
//! VERSION 0x00110002 but every RK806 register 0x00. Rechecked against the
//! TRM, Linux, vendor U-Boot and edk2-rockchip: pin mux (PMU1_IOC, not
//! PMU2), CTRLR0, BAUDR 24 from 24 MHz, SER-held CS, TX+RX mode and the
//! frame (value in the 4th byte) all agree. What differed from Linux: no
//! pull-ups, so an undriven MISO floats (now set as the DTS asks). The
//! rest is outside what software can see without a scope, so when the id
//! check fails bt0 now prints [`diagnose`]: pins as read back, pad levels
//! with CS released and asserted, MISO under pull-up/down, a loopback,
//! and the raw bytes of reads at 1 MHz, 250 kHz, mode 3 and GPIO CS.
//!
//! Third run (2026-11-09 02:51, clean power-up): every read returned the
//! transmitted bytes (`00 5a 00 00`) at every speed, mode and CS method,
//! outside loopback, while MISO alone follows its pull (nothing drives it
//! while idle). So MISO sees MOSI: on this SoM the RK806's SDI/SDO look
//! joined (or MOSI is coupled to MISO), and a full-duplex read, which
//! drives MOSI through the data byte, masks the RK806's answer. Linux
//! never drives MOSI there: drivers/mfd/rk8xx-spi.c rk806_spi_bus_read()
//! is spi_write_then_read(cmd, addr, REG_H; value), i.e. a transmit
//! transfer and then a receive-only one with ss_n held, and
//! drivers/spi/spi-rockchip.c runs the second as CTRLR0.xfm = receive
//! only, CTRLR1 = len - 1, nothing in TXDR (vendor U-Boot rk8xx_spi.c /
//! rk_spi.c and edk2-rockchip SpiLib RK806.c do the same). Reads now
//! default to that ([`ReadMode::Split`]); [`ReadMode::SplitTristate`] also
//! makes GPIO0_A6 (MOSI) a GPIO input for the receive-only transfer, in
//! case receive-only mode still drives MOSI. [`Spi2::identify`] tries
//! split, split-tristate, then the old full-duplex read and keeps the
//! first that reads an RK806 id; [`diagnose`] prints the raw bytes of
//! both split variants and whether driving MOSI as a GPIO moves MISO.
//!
//! EXT_EN (PMIC_EXT_EN_OUT, the carrier's VCC5V0_SYS / VCC3V3_SYS enable):
//! no register controls it in any public source. Linux (rk806-core.c,
//! rk806-regulator.c, pinctrl-rk806.c, include/linux/mfd/rk806.h), vendor
//! U-Boot (rk8xx_spi.c) and edk2-rockchip never write it. Rockchip's RK806
//! Developer Guide (CN) section 1.5 describes EXT_EN as the master/slave
//! strap sampled at first power-up and as the output a slave's VDC pin
//! can be tied to, i.e. an output of the power-up sequence; Rockchip
//! reference-derived schematics (FriendlyElec NanoPi R6C, net
//! PMIC_EXT_EN_OUT on RK806 pin 39 "(M/S)EXT_EN") drive external buck
//! enables (RK860 VDD_CPU_BIG/NPU, "Slot:6A", off in sleep) with it. So
//! EXT_EN follows the RK806 power state, and bt0 writes nothing for it:
//! after a good id it prints the RK806 state registers ([`STATE_REGS`]:
//! enables, SYS_STS, PWRCTRL config, SYS_CFG3, ON/OFF source, the unnamed
//! sequence register 0xC3, FSM) and the SoC PMIC_SLEEP1/2 pins
//! (GPIO0_A2/A3), which show whether the RK806 is in ACTIVE or held in
//! SLEEP by a PWRCTRL pin.
//!
//! Fourth run (2026-11-09 03:17, 12 V in): split reads work (id 0x8062).
//! State decoded with the Linux field map (rk806-core.c): POWER_EN0..5 =
//! 0f 0f 03 0f 0f 07, i.e. all 21 outputs on (bucks 1-10, NLDO1-5,
//! PLDO1-6; PLDO/NLDO5 bit layout as edk2-rockchip RK806.c
//! pldo_set_enable()/nldo_set_enable()); SLEEP_CONFIG0/1 = 88/08:
//! PWRCTRL1/2/3 function 0 (null, pin ignored), polarity 1; SYS_CFG3 = 0
//! (no DEV_SLP/DEV_OFF); SYS_STS = 0x80 (PWRON_STS). So the RK806 is in
//! ACTIVE with its PWRCTRL pins ignored, and GPIO0_A2/A3 (GPIO, low)
//! cannot hold it in SLEEP. ON_SOURCE 0x40, 0xC3 = 0x4a and MAIN_FSM_STS
//! 0x09 have no public decode (Linux names 0xB2-0xC2 shutdown-sequence
//! fields only; 0xC3 none). A power-sequence EXT_EN is therefore expected
//! high, and the DEBUG log of the same build agrees indirectly: TF-A
//! (plat/rockchip/rk3588/drivers/scmi/rk3588_clk.c
//! clk_scmi_cpub01_get_rate) read the big-core PVTPLL ring oscillators at
//! 1954 / 1969 MHz, so VDD_CPU_BIG0/1 (RK8602/RK8603, EN = PMIC_EXT_EN_OUT
//! on the Rockchip reference design) are powered. No register write or
//! sleep-pin change is indicated, and none is made. BUCK6 (vdd2_ddr_s3)
//! and BUCK9 (vddq_ddr_s0) read VSEL 0: both are set by external feedback
//! ("ADJ, FB=0.5V" in the reference power table, no voltage in the DTS),
//! so VSEL is unused and [`Readback::rails`] reports them as `ext-fb`
//! (microvolts 0). bt0 also prints [`carrier_probe`], the carrier 3.3 V
//! probe the EDK2 platform code runs, at bt0 time.

use crate::regs::{
    base, ioc, spi, wm, CRU_CLKSEL_CON59, CRU_GATE_CON14, CRU_SOFTRST_CON14, GPIO0, GPIO_DDR_L,
    GPIO_DR_L, GPIO_EXT_PORT, GPIO_VERSION_ID, SPI2_CLK_BIT, SPI2_PCLK_BIT,
};
use core::fmt;
use regio::{wait, RegIo, Timeout};

pub const POWER_EN1: u8 = 0x01;
pub const POWER_EN2: u8 = 0x02;
pub const POWER_EN3: u8 = 0x03;
pub const BUCK5_ON_VSEL: u8 = 0x1E;
pub const BUCK6_ON_VSEL: u8 = 0x1F;
pub const BUCK9_ON_VSEL: u8 = 0x22;
pub const NLDO2_ON_VSEL: u8 = 0x44;
pub const CHIP_NAME: u8 = 0x5A;
pub const CHIP_VER: u8 = 0x5B;
/// `(CHIP_NAME << 8 | CHIP_VER) & 0xfff0` of an RK806 (Linux
/// include/linux/mfd/rk808.h RK806_ID / RK8XX_ID_MSK; U-Boot
/// drivers/power/pmic/rk8xx.c reads CHIP_NAME/CHIP_VER over SPI) [I].
pub const RK806_ID: u16 = 0x8060;
pub const ID_MASK: u16 = 0xfff0;

/// CTRLR0 for the RK806: 8-bit frames, 8-bit APB accesses, one sclk of
/// ss_n setup, big endian, Motorola mode 0, transmit+receive, master. The
/// bits Linux spi-rockchip.c rockchip_spi_config() sets for an 8-bit,
/// mode-0 transfer with tx and rx buffers.
pub const CTRLR0_VAL: u32 =
    spi::CR0_DFS_8BIT | spi::CR0_SSD_ONE | spi::CR0_EM_BIG | spi::CR0_BHT_8BIT;
/// clk_spi2 = 24 MHz (CRU_CLKSEL_CON59 [7:6] = 2); BAUDR 24 gives 1 MHz,
/// the DTS spi-max-frequency of the RK806 (BAUDR must be even, p.1936).
pub const BAUDR_VAL: u32 = 24;
/// Bytes one frame may carry: the 64-entry FIFO (P1 p.1928) holds them all.
pub const FIFO_BYTES: usize = 16;

pub const fn buck_vsel(uv: u32) -> u8 {
    ((uv - 500_000) / 6_250) as u8
}
pub const fn ldo_vsel(uv: u32) -> u8 {
    ((uv - 500_000) / 12_500) as u8
}
/// Buck VSEL to microvolts: 500 mV + 6.25 mV steps to 159, 1.5 V + 25 mV
/// steps from 160, 3.4 V from 236 (Linux drivers/regulator/rk808-regulator.c
/// rk806_buck_voltage_ranges).
pub const fn buck_uv(vsel: u8) -> u32 {
    match vsel {
        0..=159 => 500_000 + vsel as u32 * 6_250,
        160..=235 => 1_500_000 + (vsel as u32 - 160) * 25_000,
        _ => 3_400_000,
    }
}
/// LDO VSEL to microvolts: 500 mV + 12.5 mV steps to 231, then 3.4 V
/// (rk806_ldo_voltage_ranges, same file).
pub const fn ldo_uv(vsel: u8) -> u32 {
    match vsel {
        0..=231 => 500_000 + vsel as u32 * 12_500,
        _ => 3_400_000,
    }
}

/// The RK806 command byte for `n` value bytes.
pub const fn cmd(write: bool, n: u8) -> u8 {
    (if write { 0x80 } else { 0 }) | ((n - 1) & 0x0f)
}

/// The bytes of a one-register read frame (then one byte is clocked in):
/// command, address low, address high (RK806_REG_H = 0). Linux
/// rk8xx-spi.c rk806_spi_bus_read() with reg_format_endian_default =
/// REGMAP_ENDIAN_LITTLE.
pub const fn read_frame(reg: u8) -> [u8; 3] {
    [cmd(false, 1), reg, 0x00]
}
/// The bytes of a one-register write frame (rk806_spi_bus_write()).
pub const fn write_frame(reg: u8, val: u8) -> [u8; 4] {
    [cmd(true, 1), reg, 0x00, val]
}

/// How [`Spi2::read`] puts a register read on the wire.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum ReadMode {
    /// Linux/U-Boot/EDK2: a transmit+receive transfer of the 3 header
    /// bytes (received bytes dropped), then a receive-only transfer
    /// (CTRLR0.xfm = 2, CTRLR1 = 0) for the value, ss_n0 low across both.
    #[default]
    Split,
    /// [`ReadMode::Split`] with GPIO0_A6 (SPI2_MOSI_M2) switched to a GPIO
    /// input for the receive-only transfer: no SoC driver on MOSI while
    /// the RK806 answers.
    SplitTristate,
    /// One full-duplex window: header and a 0x00 clocked out while the
    /// value comes in (bt0 up to 2026-11-09 02:51).
    Duplex,
}

impl ReadMode {
    pub fn name(self) -> &'static str {
        match self {
            ReadMode::Split => "split",
            ReadMode::SplitTristate => "tri",
            ReadMode::Duplex => "duplex",
        }
    }
}

/// The order [`Spi2::identify`] tries the read modes in.
pub const READ_MODES: [ReadMode; 3] = [ReadMode::Split, ReadMode::SplitTristate, ReadMode::Duplex];

/// GPIO0_A6 = SPI2_MOSI_M2 = GPIO0 bit 6.
const MOSI_BIT: u32 = 6;

/// GPIO0_A6 to a GPIO input (direction first, then the mux), so the SoC
/// stops driving MOSI.
fn mosi_release<H: RegIo + ?Sized>(io: &mut H) {
    io.write32(GPIO0 + GPIO_DDR_L, wm(MOSI_BIT, 1, 0));
    io.write32(base::PMU1_IOC + ioc::PMU1_GPIO0A_SEL_H, wm(8, 4, 0));
}

/// GPIO0_A6 back to SPI2_MOSI_M2.
fn mosi_spi<H: RegIo + ?Sized>(io: &mut H) {
    io.write32(base::PMU1_IOC + ioc::PMU1_GPIO0A_SEL_H, wm(8, 4, 1));
}

/// What [`Spi2::identify`] read with each mode it tried.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Ident {
    /// The id, if a mode read an RK806.
    pub chip: Option<u16>,
    /// `CHIP_NAME << 8 | CHIP_VER` per [`READ_MODES`] entry, `None` for a
    /// timeout; only the first `tried` entries were tried.
    pub ids: [Option<u16>; 3],
    pub tried: usize,
}

impl fmt::Display for Ident {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        for (i, m) in READ_MODES.iter().enumerate().take(self.tried) {
            if i > 0 {
                f.write_str(" ")?;
            }
            match self.ids[i] {
                Some(v) => write!(f, "{}={v:04x}", m.name())?,
                None => write!(f, "{}=timeout", m.name())?,
            }
        }
        Ok(())
    }
}

pub struct Spi2 {
    pub mode: ReadMode,
    /// CTRLR0 without the transfer mode (xfm = transmit+receive).
    cr0: u32,
}

impl Spi2 {
    /// Pins, clock, reset and controller: 8-bit frames, mode 0, master, 1 MHz.
    pub fn init<H: RegIo + ?Sized>(io: &mut H) -> Spi2 {
        io.write32(
            base::PMU1_IOC + ioc::PMU1_GPIO0A_SEL_H,
            wm(4, 4, 1) | wm(8, 4, 1),
        );
        io.write32(
            base::PMU1_IOC + ioc::PMU1_GPIO0B_SEL_L,
            wm(4, 4, 1) | wm(12, 4, 1),
        );
        // Pull-ups on all four pins, as the DTS pinctrl asks (vendor
        // rk3588s-pinctrl.dtsi:2792-2807 spi2m2_pins/spi2m2_cs0,
        // pcfg_pull_up_drv_level_1; drive level 1 is the reset value). At
        // reset A5 has a pull-down and A6/B1/B3 none, so an undriven MISO
        // floats (P1 pp.961-963).
        io.write32(
            base::PMU1_IOC + ioc::PMU1_GPIO0A_P,
            wm(10, 2, ioc::PULL_UP) | wm(12, 2, ioc::PULL_UP),
        );
        io.write32(
            base::PMU1_IOC + ioc::PMU1_GPIO0B_P,
            wm(2, 2, ioc::PULL_UP) | wm(6, 2, ioc::PULL_UP),
        );
        io.write32(
            CRU_GATE_CON14,
            wm(SPI2_PCLK_BIT, 1, 0) | wm(SPI2_CLK_BIT, 1, 0),
        );
        io.write32(CRU_CLKSEL_CON59, wm(6, 2, 2));
        let rst = wm(SPI2_PCLK_BIT, 1, 1) | wm(SPI2_CLK_BIT, 1, 1);
        io.write32(CRU_SOFTRST_CON14, rst);
        io.udelay(10);
        io.write32(CRU_SOFTRST_CON14, rst & 0xffff_0000);
        io.udelay(10);
        let b = base::SPI2;
        io.write32(b + spi::ENR, 0);
        io.write32(b + spi::SER, 0);
        io.write32(b + spi::IMR, 0);
        io.write32(b + spi::CTRLR0, CTRLR0_VAL);
        io.write32(b + spi::CTRLR1, 0);
        io.write32(b + spi::BAUDR, BAUDR_VAL);
        Spi2 {
            mode: ReadMode::Split,
            cr0: CTRLR0_VAL,
        }
    }

    /// SPI2 disabled, then CTRLR0 = [`CTRLR0_VAL`] | `extra` and BAUDR.
    pub fn configure<H: RegIo + ?Sized>(&mut self, io: &mut H, extra: u32, baudr: u32) {
        let b = base::SPI2;
        self.cr0 = CTRLR0_VAL | extra;
        io.write32(b + spi::ENR, 0);
        io.write32(b + spi::CTRLR0, self.cr0);
        io.write32(b + spi::BAUDR, baudr);
    }

    /// SPI_VERSION, a register that reads non-zero only if the controller
    /// is clocked and out of reset (a diagnostic for the log).
    pub fn version<H: RegIo + ?Sized>(&self, io: &mut H) -> u32 {
        io.read32(base::SPI2 + spi::VERSION)
    }

    /// One frame with ss_n0 held low: send `tx`, then clock `rx.len()`
    /// more bytes in (0x00 out).
    pub fn xfer<H: RegIo + ?Sized>(
        &mut self,
        io: &mut H,
        tx: &[u8],
        rx: &mut [u8],
    ) -> Result<(), Timeout> {
        let n = tx.len() + rx.len();
        let mut out = [0u8; FIFO_BYTES];
        let mut inb = [0u8; FIFO_BYTES];
        if n > FIFO_BYTES {
            return Err(Timeout("spi frame longer than the FIFO"));
        }
        out[..tx.len()].copy_from_slice(tx);
        self.duplex(io, &out[..n], &mut inb[..n])?;
        rx.copy_from_slice(&inb[tx.len()..n]);
        Ok(())
    }

    /// Full duplex, `out.len()` bytes, every received byte kept. ss_n0
    /// follows SER: Linux spi-rockchip.c rockchip_spi_set_cs() asserts it
    /// through SER and keeps it across transfers while ENR is toggled per
    /// transfer, and U-Boot rk_spi.c spi_cs_activate() and edk2-rockchip
    /// SpiLib RK806.c _spi_read() do the same (the TRM's "ss_n held high
    /// when idle or disabled", P1 p.1928, is not what those drivers rely
    /// on). All bytes still go into the FIFO before the first is read
    /// back, as rockchip_spi_pio_writer() does, so SCLK runs without gaps
    /// ("stop transmitting SCLK if transmit FIFO is empty", p.1928). After
    /// SER = 0, ss_n0 stays high for 2 us (2 SCLK at 1 MHz) before the
    /// next frame can start.
    pub fn duplex<H: RegIo + ?Sized>(
        &mut self,
        io: &mut H,
        out: &[u8],
        inb: &mut [u8],
    ) -> Result<(), Timeout> {
        let b = base::SPI2;
        io.write32(b + spi::SER, 1);
        io.write32(b + spi::ENR, 1);
        let r = (|| {
            for &o in out {
                io.write32(b + spi::TXDR, o as u32);
            }
            for v in inb.iter_mut() {
                wait(io, "spi rx", 10_000, |io| {
                    io.read32(b + spi::SR) & spi::SR_RFE == 0
                })?;
                *v = io.read32(b + spi::RXDR) as u8;
            }
            wait(io, "spi idle", 10_000, |io| {
                io.read32(b + spi::SR) & spi::SR_BUSY == 0
            })
        })();
        io.write32(b + spi::ENR, 0);
        io.write32(b + spi::SER, 0);
        io.udelay(2);
        r
    }

    /// One transfer of a split frame with ss_n0 already low: CTRLR0 with
    /// xfm = transmit+receive (`out` given, every byte also received into
    /// `inb`) or receive only (`out` None, CTRLR1 = len - 1, nothing
    /// written to TXDR), as spi-rockchip.c rockchip_spi_config() and
    /// rockchip_spi_prepare_irq() run one spi_transfer. ENR is left set.
    fn phase<H: RegIo + ?Sized>(
        &mut self,
        io: &mut H,
        out: Option<&[u8]>,
        inb: &mut [u8],
    ) -> Result<(), Timeout> {
        let b = base::SPI2;
        let xfm = if out.is_some() {
            spi::XFM_TR
        } else {
            spi::XFM_RO
        };
        io.write32(b + spi::ENR, 0);
        io.write32(b + spi::CTRLR0, self.cr0 | xfm << spi::CR0_XFM_SHIFT);
        io.write32(b + spi::CTRLR1, inb.len() as u32 - 1);
        io.write32(b + spi::ENR, 1);
        if let Some(out) = out {
            for &o in out {
                io.write32(b + spi::TXDR, o as u32);
            }
        }
        for v in inb.iter_mut() {
            wait(io, "spi rx", 10_000, |io| {
                io.read32(b + spi::SR) & spi::SR_RFE == 0
            })?;
            *v = io.read32(b + spi::RXDR) as u8;
        }
        wait(io, "spi idle", 10_000, |io| {
            io.read32(b + spi::SR) & spi::SR_BUSY == 0
        })
    }

    /// Linux spi_write_then_read(): send `out` (what comes back goes to
    /// `hdr`, same length), then receive `inb` in receive-only mode, ss_n0
    /// low across both (SER, as rockchip_spi_set_cs() keeps it for a
    /// message without cs_change). With `tristate` GPIO0_A6 is a GPIO
    /// input during the receive-only transfer. Ends with SPI2 disabled,
    /// CTRLR0 transmit+receive, CTRLR1 = 0 and MOSI muxed back.
    pub fn split<H: RegIo + ?Sized>(
        &mut self,
        io: &mut H,
        out: &[u8],
        hdr: &mut [u8],
        inb: &mut [u8],
        tristate: bool,
    ) -> Result<(), Timeout> {
        if out.len() != hdr.len() || out.is_empty() || inb.is_empty() || inb.len() > FIFO_BYTES {
            return Err(Timeout("spi split frame"));
        }
        let b = base::SPI2;
        io.write32(b + spi::SER, 1);
        let r = match self.phase(io, Some(out), hdr) {
            Ok(()) => {
                if tristate {
                    mosi_release(io);
                }
                let r = self.phase(io, None, inb);
                if tristate {
                    mosi_spi(io);
                }
                r
            }
            e => e,
        };
        io.write32(b + spi::ENR, 0);
        io.write32(b + spi::CTRLR0, self.cr0);
        io.write32(b + spi::CTRLR1, 0);
        io.write32(b + spi::SER, 0);
        io.udelay(2);
        r
    }

    /// One register, as [`Spi2::mode`] says.
    pub fn read<H: RegIo + ?Sized>(&mut self, io: &mut H, reg: u8) -> Result<u8, Timeout> {
        let mut v = [0u8];
        match self.mode {
            ReadMode::Duplex => self.xfer(io, &read_frame(reg), &mut v)?,
            m => {
                let mut h = [0u8; 3];
                self.split(
                    io,
                    &read_frame(reg),
                    &mut h,
                    &mut v,
                    m == ReadMode::SplitTristate,
                )?
            }
        }
        Ok(v[0])
    }

    /// CHIP_NAME and CHIP_VER with each of [`READ_MODES`] in turn until one
    /// reads an RK806 id; that mode is kept for later reads (else
    /// [`ReadMode::Split`]). Reads only.
    pub fn identify<H: RegIo + ?Sized>(&mut self, io: &mut H) -> Ident {
        let mut id = Ident::default();
        for (i, &m) in READ_MODES.iter().enumerate() {
            self.mode = m;
            id.tried = i + 1;
            let r = self.read(io, CHIP_NAME).and_then(|n| {
                self.read(io, CHIP_VER)
                    .map(|v| ((n as u16) << 8) | v as u16)
            });
            id.ids[i] = r.ok();
            if let Ok(c) = r {
                if c & ID_MASK == RK806_ID {
                    id.chip = Some(c);
                    return id;
                }
            }
        }
        self.mode = ReadMode::Split;
        id
    }

    /// One register: a single 4-byte transfer (command, address low,
    /// RK806_REG_H, value), as Linux rk806_spi_bus_write() / vendor U-Boot
    /// _spi_write() send it; transmit+receive like spi-rockchip.c in IRQ
    /// mode, received bytes dropped.
    pub fn write<H: RegIo + ?Sized>(
        &mut self,
        io: &mut H,
        reg: u8,
        val: u8,
    ) -> Result<(), Timeout> {
        self.xfer(io, &write_frame(reg, val), &mut [])
    }
}

/// A read-only snapshot of the RK806 identity and the DDR rails, for the
/// bt0 log (no PMIC writes: safe on the stepping-stone path).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Readback {
    pub spi_version: u32,
    /// The read mode [`Spi2::identify`] settled on, and what each tried
    /// mode read.
    pub mode: ReadMode,
    pub ident: Ident,
    pub chip_name: u8,
    pub chip_ver: u8,
    pub en1: u8,
    pub en2: u8,
    pub en3: u8,
    pub buck5: u8,
    pub buck6: u8,
    pub buck9: u8,
    pub nldo2: u8,
    /// [`STATE_REGS`] in order; read only with a good id (else zero).
    pub state: [u8; STATE_REGS.len()],
    /// PMU1_IOC GPIO0A_IOMUX_SEL_L (GPIO0_A2/A3 = PMIC_SLEEP1/2 mux) and
    /// the GPIO0_A2/A3 pad levels (bit 1 = A2, bit 0 = A3): SoC side of
    /// the RK806 PWRCTRL1/2 pins.
    pub sleep_sel: u32,
    pub sleep_pads: u32,
    /// ON_VSEL of every [`ALL_RAILS`] entry; read only with a good id.
    pub vsel_all: [u8; ALL_RAILS.len()],
}

/// RK806 registers printed after a good id, for the EXT_EN question
/// (register addresses and fields: vendor/mainline Linux
/// drivers/mfd/rk806-core.c rk806_reg_fields and include/linux/mfd/
/// rk806.h): POWER_EN0..5; SYS_STS 0x5D (PWRON_STS [7], VDC_STS [6],
/// VB_UV [5], VB_LO [4], HOTDIE [3], TSD [2], VB_OV [0]: datasheet Rev
/// 1.4); SLEEP_CONFIG0/1 0x62/0x63 (PWRCTRLn
/// FUN / POL: 0 null, 1 sleep, 2 power-off, 3 reset, 4 DVS, 5 GPIO);
/// SLEEP_GPIO 0x71; SYS_CFG3 0x72 (RST_FUN [7:6], DEV_RST [5], DEV_SLP
/// [4], DEV_OFF [0]); ON_SOURCE / OFF_SOURCE 0x74/0x75; BUCK_SEQ_REG17
/// 0xC3 (the one sequence register Linux names no field in); MAIN_FSM_STS
/// 0xDD; OTP_VER 0x5C [3:0]; BUCK_FB_RES_OTP_REG1 0xAC (rk806.h, no field
/// decode); and, for the abnormal power-off question, SYS_CFG0 0x5E
/// (VB_UV / VB_LO thresholds, VB_LO_ACT), SYS_CFG1 0x5F (ABNORDET_ENB,
/// SYS_OV_SD_EN), SYS_OPTION 0x61, INT_STS0/1 0x77/0x79 (decoded by
/// [`Power`]). Reads only.
/// Every RK806 rail, for the IO-supply question (the UART2 pads behave
/// as if their supply were missing): (name, ON_VSEL register, LDO
/// voltage scale, POWER_EN index, enable bit). Registers and enable bits:
/// Linux include/linux/mfd/rk806.h and drivers/regulator/rk806-regulator.c
/// (BUCK1..10 ON_VSEL 0x1A..0x23 in POWER_EN0..2; NLDO1..4 0x43..0x46 in
/// POWER_EN3, NLDO5 0x47 = POWER_EN5 bit 2; PLDO1..3 0x4E..0x50 = POWER_EN4
/// bits 1..3, PLDO4/5 0x51/0x52 = POWER_EN5 bits 0/1, PLDO6 0x53 =
/// POWER_EN4 bit 0). Names: YY3588 vendor DTS rk3588-rk806-single.dtsi.
pub const ALL_RAILS: [(&str, u8, bool, u8, u8); 21] = [
    ("buck1 vdd_gpu_s0", 0x1A, false, 0, 0),
    ("buck2 vdd_cpu_lit_s0", 0x1B, false, 0, 1),
    ("buck3 vdd_log_s0", 0x1C, false, 0, 2),
    ("buck4 vdd_vdenc_s0", 0x1D, false, 0, 3),
    ("buck5 vdd_ddr_s0", 0x1E, false, 1, 0),
    ("buck6 vdd2_ddr_s3", 0x1F, false, 1, 1),
    ("buck7 vdd_2v0_pldo_s3", 0x20, false, 1, 2),
    ("buck8 vcc_3v3_s3", 0x21, false, 1, 3),
    ("buck9 vddq_ddr_s0", 0x22, false, 2, 0),
    ("buck10 vcc_1v8_s3", 0x23, false, 2, 1),
    ("nldo1 vdd_0v75_s3", 0x43, true, 3, 0),
    ("nldo2 vdd_ddr_pll_s0", 0x44, true, 3, 1),
    ("nldo3 avdd_0v75_s0", 0x45, true, 3, 2),
    ("nldo4 vdd_0v85_s0", 0x46, true, 3, 3),
    ("nldo5 vdd_0v75_s0", 0x47, true, 5, 2),
    ("pldo1 avcc_1v8_s0", 0x4E, true, 4, 1),
    ("pldo2 vcc_1v8_s0", 0x4F, true, 4, 2),
    ("pldo3 avdd_1v2_s0", 0x50, true, 4, 3),
    ("pldo4 vcc_3v3_s0", 0x51, true, 5, 0),
    ("pldo5 vccio_sd_s0", 0x52, true, 5, 1),
    ("pldo6 pldo6_s3", 0x53, true, 4, 0),
];

pub const STATE_REGS: [(&str, u8); 22] = [
    ("en0", 0x00),
    ("en1", 0x01),
    ("en2", 0x02),
    ("en3", 0x03),
    ("en4", 0x04),
    ("en5", 0x05),
    ("sys_sts", 0x5D),
    ("slp_cfg0", 0x62),
    ("slp_cfg1", 0x63),
    ("slp_gpio", 0x71),
    ("sys_cfg3", 0x72),
    ("on_src", 0x74),
    ("off_src", 0x75),
    ("seq17", 0xC3),
    ("fsm", 0xDD),
    ("otp_ver", 0x5C),
    ("fb_otp", 0xAC),
    // Appended (indices above are used by Decoded and the tests): the
    // input-voltage supervision, RK806 datasheet Rev 1.4 register tables.
    ("sys_cfg0", 0x5E),
    ("sys_cfg1", 0x5F),
    ("sys_opt", 0x61),
    ("int_sts0", 0x77),
    ("int_sts1", 0x79),
];

/// Index of register `reg` in [`STATE_REGS`] (compile-time; a register
/// that is not in the table fails the build).
const fn state_idx(reg: u8) -> usize {
    let mut i = 0;
    while i < STATE_REGS.len() {
        if STATE_REGS[i].1 == reg {
            return i;
        }
        i += 1;
    }
    panic!("register not in STATE_REGS")
}

/// Bit names, bit 7 first (RK806 datasheet Rev 1.4 register description;
/// the Linux rk806.h field order for 0x74/0x75 differs and is not used).
pub const ON_SOURCE_BITS: [&str; 8] = [
    "ON_PWRON",
    "ON_VDC",
    "ON_ABNORMAL",
    "RESTART_RESETB",
    "RESTART_PWRON_LP",
    "RESTART_PWRCTRL",
    "RESTART_DEV_RST",
    "RESTART_WDT",
];
pub const OFF_SOURCE_BITS: [&str; 8] = [
    "OFF_PWRCTRL",
    "VB_SYS_OV",
    "OFF_TSD",
    "OFF_SYNC",
    "OFF_DEV_OFF",
    "OFF_PWRON_LP",
    "OFF_ABNORMAL",
    "OFF_VB_LO",
];
/// SYS_STS 0x5D (live status, read only).
pub const SYS_STS_BITS: [&str; 8] = [
    "PWRON", "VDC", "VB_UV", "VB_LO", "HOTDIE", "TSD", "bit1", "VB_OV",
];
/// INT_STS0 0x77 (latched events).
pub const INT_STS0_BITS: [&str; 8] = [
    "VB_LO",
    "VDC_FALL",
    "VDC_RISE",
    "HOTDIE",
    "PWRON_LP",
    "PWRON",
    "PWRON_RISE",
    "PWRON_FALL",
];

/// The set bits of `v` by name, bit 7 first, `|`-separated; `none` for 0.
fn bit_names(f: &mut fmt::Formatter<'_>, v: u8, names: &[&str; 8]) -> fmt::Result {
    if v == 0 {
        return f.write_str("none");
    }
    let mut sep = "";
    for (i, n) in names.iter().enumerate() {
        if v & (0x80 >> i) != 0 {
            write!(f, "{sep}{n}")?;
            sep = "|";
        }
    }
    Ok(())
}

/// Why the RK806 last turned on and off, and the input-voltage
/// (VCCA/VCC1/VCC2) supervision state: ON_SOURCE 0x74, OFF_SOURCE 0x75,
/// SYS_STS 0x5D, INT_STS0 0x77, SYS_CFG0/1 0x5E/0x5F, SYS_OPTION 0x61.
/// OFF_ABNORMAL = input below VB_UV or above VB_OV (datasheet 4.1.3
/// item 7); VB_SYS_OV / VB_OV_STS are the bits that single out the
/// overvoltage case. Displays as
/// `on=<bits> off=<bits> sts=<bits> int=<bits>`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Power {
    pub on_src: u8,
    pub off_src: u8,
    pub sys_sts: u8,
    pub int_sts0: u8,
    pub sys_cfg0: u8,
    pub sys_cfg1: u8,
    pub sys_opt: u8,
}

impl Power {
    pub fn from_state(st: &[u8; STATE_REGS.len()]) -> Power {
        Power {
            on_src: st[const { state_idx(0x74) }],
            off_src: st[const { state_idx(0x75) }],
            sys_sts: st[const { state_idx(0x5D) }],
            int_sts0: st[const { state_idx(0x77) }],
            sys_cfg0: st[const { state_idx(0x5E) }],
            sys_cfg1: st[const { state_idx(0x5F) }],
            sys_opt: st[const { state_idx(0x61) }],
        }
    }
    /// VB_UV_SEL [6:4]: shutdown threshold, 2.7 V + 100 mV steps.
    pub fn vb_uv_mv(&self) -> u32 {
        2700 + 100 * ((self.sys_cfg0 >> 4) & 7) as u32
    }
    /// VB_LO_SEL [2:0]: low-voltage threshold, 2.8 V + 100 mV steps.
    pub fn vb_lo_mv(&self) -> u32 {
        2800 + 100 * (self.sys_cfg0 & 7) as u32
    }
    /// The supervision settings, displayed as
    /// `vb_uv=<mV>/<delay> vb_lo=<mV>/<irq|off> abnordet=.. ov_sd=..
    /// uvlock=.. vccxdet=<n>`.
    pub fn cfg(&self) -> PowerCfg {
        PowerCfg(*self)
    }
}

impl fmt::Display for Power {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("on=")?;
        bit_names(f, self.on_src, &ON_SOURCE_BITS)?;
        f.write_str(" off=")?;
        bit_names(f, self.off_src, &OFF_SOURCE_BITS)?;
        f.write_str(" sts=")?;
        bit_names(f, self.sys_sts, &SYS_STS_BITS)?;
        f.write_str(" int=")?;
        bit_names(f, self.int_sts0, &INT_STS0_BITS)
    }
}

/// [`Power::cfg`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PowerCfg(Power);

impl fmt::Display for PowerCfg {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let p = &self.0;
        let on = |b: bool| if b { "on" } else { "off" };
        write!(
            f,
            "vb_uv={}mV/{} vb_lo={}mV/{} abnordet={} ov_sd={} uvlock={} vccxdet={}",
            p.vb_uv_mv(),
            // VB_UV_DLY [7].
            if p.sys_cfg0 & 0x80 != 0 {
                "50us"
            } else {
                "5us"
            },
            p.vb_lo_mv(),
            // VB_LO_ACT [3]: 0 shut down, 1 interrupt.
            if p.sys_cfg0 & 0x08 != 0 { "irq" } else { "off" },
            // ABNORDET_ENB [7]: 0 enable, 1 disable.
            on(p.sys_cfg1 & 0x80 == 0),
            // SYS_OV_SD_EN [3].
            on(p.sys_cfg1 & 0x08 != 0),
            // VBUVLOCK_EN [7]; VCCXDET_DIS [5:4] (0 = VCCA, VCC1, VCC2;
            // 1 = VCCA, VCC2; 2 = VCCA, VCC1; 3 = VCCA only).
            on(p.sys_opt & 0x80 != 0),
            (p.sys_opt >> 4) & 3
        )
    }
}

/// The [`STATE_REGS`] fields that say whether the RK806 is ACTIVE.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Decoded {
    /// PWRCTRL1/2/3 function (0 null, 1 sleep, 2 power-off, 3 reset,
    /// 4 DVS, 5 GPIO) and polarity.
    pub pwrctrl_fun: [u8; 3],
    pub pwrctrl_pol: [u8; 3],
    /// SYS_CFG3 DEV_SLP [4] and DEV_OFF [0].
    pub dev_slp: bool,
    pub dev_off: bool,
    /// Outputs enabled, of 21.
    pub rails_on: u32,
}

impl Decoded {
    pub fn from_state(st: &[u8; STATE_REGS.len()]) -> Decoded {
        // POWER_EN0/1: bucks 1-8 [3:0]; EN2: bucks 9-10 [1:0]; EN3: NLDO1-4
        // [3:0]; EN4: PLDO6 [0], PLDO1-3 [3:1]; EN5: PLDO4 [0], PLDO5 [1],
        // NLDO5 [2].
        let mask = [0xf, 0xf, 0x3, 0xf, 0xf, 0x7];
        let rails_on = st[..6]
            .iter()
            .zip(mask)
            .map(|(v, m)| (v & m).count_ones())
            .sum();
        let (c0, c1, cfg3) = (st[7], st[8], st[10]);
        Decoded {
            pwrctrl_fun: [c0 & 7, (c0 >> 4) & 7, c1 & 7],
            pwrctrl_pol: [(c0 >> 3) & 1, c0 >> 7, (c1 >> 3) & 1],
            dev_slp: cfg3 & 0x10 != 0,
            dev_off: cfg3 & 0x01 != 0,
            rails_on,
        }
    }

    /// ACTIVE with nothing that sleeps it: no PWRCTRL pin with the sleep
    /// or power-off function, no DEV_SLP/DEV_OFF, every output on.
    pub fn active(&self) -> bool {
        !self.pwrctrl_fun.iter().any(|&f| f == 1 || f == 2)
            && !self.dev_slp
            && !self.dev_off
            && self.rails_on == 21
    }
}

impl Readback {
    pub fn decoded(&self) -> Decoded {
        Decoded::from_state(&self.state)
    }
    pub fn power(&self) -> Power {
        Power::from_state(&self.state)
    }

    pub fn chip(&self) -> u16 {
        ((self.chip_name as u16) << 8) | self.chip_ver as u16
    }
    pub fn id_ok(&self) -> bool {
        self.chip() & ID_MASK == RK806_ID
    }
    /// (name, VSEL, microvolts, enabled) of BUCK5, BUCK6, BUCK9 and NLDO2.
    /// BUCK6 and BUCK9 run on external feedback ("ADJ, FB=0.5V" in the
    /// Rockchip reference power table; the DTS gives them no voltage), so
    /// their VSEL means nothing: microvolts 0, printed `ext-fb`.
    /// POWER_ENx low nibble = enable state (rk808-regulator.c enable_mask).
    pub fn rails(&self) -> [(&'static str, u8, u32, bool); 4] {
        [
            (
                "buck5 vdd_ddr_s0",
                self.buck5,
                buck_uv(self.buck5),
                self.en1 & 1 != 0,
            ),
            ("buck6 vdd2_ddr_s3", self.buck6, 0, self.en1 & 2 != 0),
            ("buck9 vddq_ddr_s0", self.buck9, 0, self.en2 & 1 != 0),
            (
                "nldo2 vdd_ddr_pll_s0",
                self.nldo2,
                ldo_uv(self.nldo2),
                self.en3 & 2 != 0,
            ),
        ]
    }
}

impl Readback {
    /// Every rail as (name, VSEL, microvolts, enabled), from
    /// [`Readback::vsel_all`] and the POWER_EN bytes in [`Readback::state`]
    /// (STATE_REGS starts with en0..en5). BUCK6/BUCK9 use external
    /// feedback: their microvolts are decoded but mean nothing.
    pub fn all_rails(&self) -> impl Iterator<Item = (&'static str, u8, u32, bool)> + '_ {
        ALL_RAILS
            .iter()
            .zip(self.vsel_all)
            .map(|(&(name, _, ldo, en, bit), v)| {
                let uv = if ldo { ldo_uv(v) } else { buck_uv(v) };
                (name, v, uv, self.state[en as usize] >> bit & 1 != 0)
            })
    }
}

/// Bring SPI2 up and read the identity, enables and DDR rail VSELs.
pub fn readback<H: RegIo + ?Sized>(io: &mut H) -> Result<Readback, Timeout> {
    let mut s = Spi2::init(io);
    let mut r = Readback {
        spi_version: s.version(io),
        ..Readback::default()
    };
    r.ident = s.identify(io);
    r.mode = s.mode;
    let chip = r.ident.chip.or(r.ident.ids[0]).unwrap_or(0);
    r.chip_name = (chip >> 8) as u8;
    r.chip_ver = chip as u8;
    let sel = io.read32(base::PMU1_IOC + ioc::PMU1_GPIO0A_SEL_L);
    let pads = io.read32(GPIO0 + GPIO_EXT_PORT);
    r.sleep_sel = sel & 0xffff;
    r.sleep_pads = ((pads >> 2) & 1) << 1 | ((pads >> 3) & 1);
    if r.ident.chip.is_none() {
        // Timed out in every mode: nothing more to read.
        if r.ident.ids.iter().all(|i| i.is_none()) {
            return Err(Timeout("spi rx"));
        }
    }
    r.en1 = s.read(io, POWER_EN1)?;
    r.en2 = s.read(io, POWER_EN2)?;
    r.en3 = s.read(io, POWER_EN3)?;
    r.buck5 = s.read(io, BUCK5_ON_VSEL)?;
    r.buck6 = s.read(io, BUCK6_ON_VSEL)?;
    r.buck9 = s.read(io, BUCK9_ON_VSEL)?;
    r.nldo2 = s.read(io, NLDO2_ON_VSEL)?;
    if r.id_ok() {
        for (v, &(_, reg)) in r.state.iter_mut().zip(STATE_REGS.iter()) {
            *v = s.read(io, reg)?;
        }
        for (v, &(_, reg, ..)) in r.vsel_all.iter_mut().zip(ALL_RAILS.iter()) {
            *v = s.read(io, reg)?;
        }
    }
    Ok(r)
}

/// MOSI bytes of a diagnostic CHIP_NAME read: the [`read_frame`] header
/// and one byte to clock the value in, the same 4 SPI frames as Linux
/// spi_write_then_read(3, 1) and vendor U-Boot rk8xx_spi.c _spi_read()
/// (header `00 5A`, then 2 bytes read, value in the second). Every byte
/// has bit 7 clear, so however the RK806 frames it, it sees reads only;
/// and 0x00 / 0x5A are bit palindromes, so an LSB-first RK806 would get
/// the same request and answer 0x01 (0x80 reversed): no separate LSB pass.
pub const DIAG_TX: [u8; 4] = [cmd(false, 1), CHIP_NAME, 0x00, 0x00];
/// Loopback bytes (bit 7 clear as well).
pub const LOOP_TX: [u8; 3] = [0x5A, 0x3C, 0x69];
/// GPIO0 bit numbers of SPI2_CLK_M2, MOSI_M2, CS0_M2, MISO_M2 (A5, A6,
/// B1, B3).
pub const PAD_BITS: [u32; 4] = [5, 6, 9, 11];
/// GPIO0_B1 = SPI2_CS0_M2 = GPIO0 bit 9.
const CS_BIT: u32 = 9;

/// One CHIP_NAME read of [`diagnose`].
#[derive(Clone, Copy, Debug)]
pub struct DiagRead {
    pub name: &'static str,
    /// SPI mode (scpol << 1 | scph).
    pub mode: u32,
    /// 24 MHz clk_spi2 / BAUDR: 24 = 1 MHz, 96 = 250 kHz.
    pub baudr: u32,
    /// ss_n0 driven as GPIO0_B1 output instead of by the controller.
    pub sw_cs: bool,
}

const fn rd(name: &'static str, mode: u32, baudr: u32, sw_cs: bool) -> DiagRead {
    DiagRead {
        name,
        mode,
        baudr,
        sw_cs,
    }
}

pub const DIAG_READS: [DiagRead; 4] = [
    rd("hw1M", 0, 24, false),
    rd("hw250k", 0, 96, false),
    rd("hw250k-m3", 3, 96, false),
    rd("sw250k", 0, 96, true),
];

/// The SPI2 M2 pins' IOC fields, (clk, mosi, cs, miso) each.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct PinCfg {
    pub mux: [u32; 4],
    pub pull: [u32; 4],
    pub ds: [u32; 4],
    pub ie: [u32; 4],
}

impl PinCfg {
    /// Decode PMU1_IOC GPIO0A_IOMUX_SEL_H, GPIO0B_IOMUX_SEL_L, GPIO0A_P,
    /// GPIO0B_P, GPIO0A_DS_H, GPIO0B_DS_L, GPIO0A_IE, GPIO0B_IE (P1
    /// pp.957-964). A5/A6 are pins 1/2 of the A "high" registers, B1/B3
    /// pins 1/3 of the B "low" ones.
    pub fn decode(r: [u32; 8]) -> PinCfg {
        let f = |v: u32, lo: u32, w: u32| (v >> lo) & ((1 << w) - 1);
        PinCfg {
            mux: [f(r[0], 4, 4), f(r[0], 8, 4), f(r[1], 4, 4), f(r[1], 12, 4)],
            pull: [f(r[2], 10, 2), f(r[2], 12, 2), f(r[3], 2, 2), f(r[3], 6, 2)],
            ds: [f(r[4], 4, 2), f(r[4], 8, 2), f(r[5], 4, 2), f(r[5], 12, 2)],
            ie: [f(r[6], 5, 1), f(r[6], 6, 1), f(r[7], 1, 1), f(r[7], 3, 1)],
        }
    }
    /// What [`Spi2::init`] sets: function 1, pull-up, drive 1, input on.
    pub fn ok(&self) -> bool {
        self.mux == [1; 4] && self.pull == [ioc::PULL_UP; 4] && self.ie == [1; 4]
    }
}

struct Hex<'a>(&'a [u8]);
impl core::fmt::Display for Hex<'_> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        for (i, b) in self.0.iter().enumerate() {
            if i > 0 {
                f.write_str(" ")?;
            }
            write!(f, "{b:02x}")?;
        }
        Ok(())
    }
}

struct Quad([u32; 4]);
impl core::fmt::Display for Quad {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let q = self.0;
        write!(f, "{}/{}/{}/{}", q[0], q[1], q[2], q[3])
    }
}

/// One read's raw bytes, or the wait that timed out.
struct Rx<'a>(&'a Result<(), Timeout>, &'a [u8]);
impl core::fmt::Display for Rx<'_> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self.0 {
            Ok(()) => write!(f, "[{}]", Hex(self.1)),
            Err(t) => write!(f, "[{}: {}]", t.0, Hex(self.1)),
        }
    }
}

/// Pad levels of (clk, mosi, cs, miso) as 4 binary digits.
fn pads<H: RegIo + ?Sized>(io: &mut H) -> u32 {
    let x = io.read32(GPIO0 + GPIO_EXT_PORT);
    PAD_BITS.iter().fold(0, |a, &b| (a << 1) | ((x >> b) & 1))
}

/// MISO pad level (with pull-up, with pull-down): pulled down for 10 us,
/// read, then back to the pull-up for 10 us and read. Following the pull
/// means nothing drives MISO.
fn miso_pulls<H: RegIo + ?Sized>(io: &mut H) -> (u32, u32) {
    let p = base::PMU1_IOC + ioc::PMU1_GPIO0B_P;
    let miso = |io: &mut H| (io.read32(GPIO0 + GPIO_EXT_PORT) >> PAD_BITS[3]) & 1;
    io.write32(p, wm(6, 2, ioc::PULL_DOWN));
    io.udelay(10);
    let down = miso(io);
    io.write32(p, wm(6, 2, ioc::PULL_UP));
    io.udelay(10);
    (miso(io), down)
}

/// GPIO0_B1 as a GPIO output at `level` (high first, so it never glitches
/// low on the way), or back to SPI2_CS0_M2 with `None`.
fn sw_cs<H: RegIo + ?Sized>(io: &mut H, level: Option<u32>) {
    let mux = base::PMU1_IOC + ioc::PMU1_GPIO0B_SEL_L;
    match level {
        Some(l) => {
            io.write32(GPIO0 + GPIO_DR_L, wm(CS_BIT, 1, 1));
            io.write32(GPIO0 + GPIO_DDR_L, wm(CS_BIT, 1, 1));
            io.write32(mux, wm(4, 4, 0));
            io.write32(GPIO0 + GPIO_DR_L, wm(CS_BIT, 1, l));
        }
        None => {
            io.write32(GPIO0 + GPIO_DR_L, wm(CS_BIT, 1, 1));
            io.udelay(2);
            io.write32(mux, wm(4, 4, 1));
            io.write32(GPIO0 + GPIO_DDR_L, wm(CS_BIT, 1, 0));
        }
    }
    io.udelay(2);
}

/// MISO pad level with GPIO0_A6 (MOSI) driven as a GPIO, ss_n0 high (the
/// RK806 deselected): (MOSI 0 with MISO pulled up, MOSI 1 with MISO
/// pulled down). `(0, 1)` means MISO follows MOSI on the board.
fn mosi_couple<H: RegIo + ?Sized>(io: &mut H) -> (u32, u32) {
    let p = base::PMU1_IOC + ioc::PMU1_GPIO0B_P;
    let miso = |io: &mut H| (io.read32(GPIO0 + GPIO_EXT_PORT) >> PAD_BITS[3]) & 1;
    io.write32(GPIO0 + GPIO_DR_L, wm(MOSI_BIT, 1, 0));
    io.write32(GPIO0 + GPIO_DDR_L, wm(MOSI_BIT, 1, 1));
    io.write32(base::PMU1_IOC + ioc::PMU1_GPIO0A_SEL_H, wm(8, 4, 0));
    io.write32(p, wm(6, 2, ioc::PULL_UP));
    io.udelay(10);
    let low = miso(io);
    io.write32(GPIO0 + GPIO_DR_L, wm(MOSI_BIT, 1, 1));
    io.write32(p, wm(6, 2, ioc::PULL_DOWN));
    io.udelay(10);
    let high = miso(io);
    io.write32(p, wm(6, 2, ioc::PULL_UP));
    mosi_release(io);
    mosi_spi(io);
    io.udelay(2);
    (low, high)
}

/// Read-only SPI2/RK806 diagnostics for the bt0 log, for when CHIP_NAME
/// does not read as an RK806. Eight lines, one hardware run:
///
/// 1. `raw`: the IOC (PMU1 SPI2 M2 pins, BUS_IOC SPI2 M0/M1 MISO pads)
///    and CRU registers as read back.
/// 2. `pins`: those decoded per pin (clk/mosi/cs/miso), want mux 1,
///    pull 3 (up), ds 1, ie 1.
/// 3. `spi`: the controller registers after [`Spi2::init`].
/// 4. `pads`: GPIO0 pad levels (clk mosi cs miso, binary) idle and with
///    SER = 1 (cs should read 0), and MISO under pull-up/pull-down in
///    both states: `1/0` means nothing drives MISO, `0/0` that something
///    holds it low.
/// 5. `couple`: ss_n0 high, MOSI driven 0 then 1 as a GPIO, MISO read
///    under pull-up then pull-down: `0`/`1` (MISO FOLLOWS MOSI) means the
///    two lines are joined on the board.
/// 6. `loopback`: the controller's RX path alone (rxd = txd, ss_n0 pad
///    held high as a GPIO), so a failure there is inside the SoC.
/// 7. `read`: every raw byte of a full-duplex CHIP_NAME read
///    ([`DIAG_TX`], value in the 4th) at 1 MHz, at 250 kHz, at 250 kHz in
///    mode 3 and at 250 kHz with ss_n0 driven as a GPIO (if the
///    controller's CS does not reach the pad); the `cs` level during the
///    GPIO-driven read follows.
/// 8. `split1M`: the Linux-style CHIP_NAME read at 1 MHz ([`Spi2::split`]:
///    3 header bytes transmit+receive, what came back in `[..]`, then one
///    receive-only byte `val`), plain and with MOSI tri-stated (`tri`).
///
/// Writes only SoC registers (IOC pull/mux of the SPI2 M2 pins, GPIO0_A6
/// and GPIO0_B1, SPI2, CRU via [`Spi2::init`]); on the bus only
/// [`DIAG_TX`] and [`LOOP_TX`] bytes, all with bit 7 clear (RK806 reads);
/// MOSI moves as a GPIO only with ss_n0 high. Leaves SPI2 and the pins as
/// [`Spi2::init`] sets them.
pub fn diagnose<H: RegIo + ?Sized>(io: &mut H) {
    let mut s = Spi2::init(io);
    let p = base::PMU1_IOC;
    let bus = base::BUS_IOC;
    let r = |io: &mut H, a: usize| io.read32(a);
    let pins = [
        r(io, p + ioc::PMU1_GPIO0A_SEL_H),
        r(io, p + ioc::PMU1_GPIO0B_SEL_L),
        r(io, p + ioc::PMU1_GPIO0A_P),
        r(io, p + ioc::PMU1_GPIO0B_P),
        r(io, p + ioc::PMU1_GPIO0A_DS_H),
        r(io, p + ioc::PMU1_GPIO0B_DS_L),
        r(io, p + ioc::PMU1_GPIO0A_IE),
        r(io, p + ioc::PMU1_GPIO0B_IE),
    ];
    let m0 = r(io, bus + ioc::BUS_GPIO1A_SEL_H);
    let m1 = r(io, bus + ioc::BUS_GPIO4A_SEL_H);
    let cru = [
        r(io, CRU_CLKSEL_CON59),
        r(io, CRU_GATE_CON14),
        r(io, CRU_SOFTRST_CON14),
    ];
    io.log(format_args!(
        "pmic diag raw pmu1_ioc sel {:#06x} {:#06x} p {:#06x} {:#06x} ds {:#06x} {:#06x} ie {:#04x} {:#04x} bus_ioc 1a_h={:#06x} 4a_h={:#06x} cru clksel59={:#06x} gate14={:#06x} softrst14={:#06x}\n",
        pins[0], pins[1], pins[2], pins[3], pins[4], pins[5], pins[6], pins[7], m0, m1, cru[0], cru[1], cru[2]
    ));
    let c = PinCfg::decode(pins);
    io.log(format_args!(
        "pmic diag pins clk/mosi/cs/miso mux={} pull={} ds={} ie={} {} miso_m0/m1 mux={}/{}\n",
        Quad(c.mux),
        Quad(c.pull),
        Quad(c.ds),
        Quad(c.ie),
        if c.ok() { "ok" } else { "BAD" },
        m0 & 0xf,
        m1 & 0xf
    ));
    let b = base::SPI2;
    let sp = [
        r(io, b + spi::CTRLR0),
        r(io, b + spi::CTRLR1),
        r(io, b + spi::SER),
        r(io, b + spi::BAUDR),
        r(io, b + spi::SR),
        r(io, b + spi::RXFLR),
        r(io, b + spi::TXFLR),
        r(io, b + spi::ISR),
        r(io, b + spi::RISR),
        r(io, b + spi::VERSION),
    ];
    io.log(format_args!(
        "pmic diag spi ctrlr0={:#x} ctrlr1={:#x} ser={:#x} baudr={} sr={:#x} rxflr={} txflr={} isr={:#x} risr={:#x} ver={:#010x}\n",
        sp[0], sp[1], sp[2], sp[3], sp[4], sp[5], sp[6], sp[7], sp[8], sp[9]
    ));

    let gver = r(io, GPIO0 + GPIO_VERSION_ID);
    let idle = pads(io);
    let (iu, id) = miso_pulls(io);
    io.write32(b + spi::SER, 1);
    io.udelay(2);
    let ser = r(io, b + spi::SER);
    let sel = pads(io);
    let (su, sd) = miso_pulls(io);
    io.write32(b + spi::SER, 0);
    io.udelay(2);
    io.log(format_args!(
        "pmic diag pads clk,mosi,cs,miso idle={idle:04b} ser1={sel:04b} (ser={ser:#x}) miso pu/pd idle={iu}/{id} ser1={su}/{sd} gpio0_ver={gver:#010x}\n"
    ));
    let (c0, c1) = mosi_couple(io);
    io.log(format_args!(
        "pmic diag couple cs=1 mosi_gpio=0:miso(pu)={c0} mosi_gpio=1:miso(pd)={c1} {}\n",
        match (c0, c1) {
            (0, 1) => "MISO FOLLOWS MOSI",
            (1, 0) => "independent",
            _ => "?",
        }
    ));

    sw_cs(io, Some(1));
    s.configure(io, spi::CR0_LBK, 96);
    let mut lr = [0u8; LOOP_TX.len()];
    let res = s.duplex(io, &LOOP_TX, &mut lr);
    sw_cs(io, None);
    io.log(format_args!(
        "pmic diag loopback tx={} rx={} {}\n",
        Hex(&LOOP_TX),
        Rx(&res, &lr),
        if res.is_ok() && lr == LOOP_TX {
            "ok"
        } else {
            "BAD"
        }
    ));

    let mut rx = [[0u8; DIAG_TX.len()]; DIAG_READS.len()];
    let mut res = [Ok(()); DIAG_READS.len()];
    let mut cs_low = 0;
    for (i, d) in DIAG_READS.iter().enumerate() {
        s.configure(io, d.mode << spi::CR0_MODE_SHIFT, d.baudr);
        if d.sw_cs {
            sw_cs(io, Some(0));
            cs_low = (pads(io) >> 1) & 1;
        }
        res[i] = s.duplex(io, &DIAG_TX, &mut rx[i]);
        if d.sw_cs {
            sw_cs(io, None);
        }
    }
    io.log(format_args!(
        "pmic diag read tx={} {}={} {}={} {}={} {}={} cs={}\n",
        Hex(&DIAG_TX),
        DIAG_READS[0].name,
        Rx(&res[0], &rx[0]),
        DIAG_READS[1].name,
        Rx(&res[1], &rx[1]),
        DIAG_READS[2].name,
        Rx(&res[2], &rx[2]),
        DIAG_READS[3].name,
        Rx(&res[3], &rx[3]),
        cs_low
    ));

    s.configure(io, 0, BAUDR_VAL);
    let mut hdr = [[0u8; 3]; 2];
    let mut val = [[0u8; 1]; 2];
    let mut sres = [Ok(()); 2];
    for (i, tri) in [false, true].into_iter().enumerate() {
        sres[i] = s.split(io, &DIAG_TX[..3], &mut hdr[i], &mut val[i], tri);
    }
    io.log(format_args!(
        "pmic diag split1M tx={} +ro1 split={} val={} tri={} val={}\n",
        Hex(&DIAG_TX[..3]),
        Rx(&sres[0], &hdr[0]),
        Hex(&val[0]),
        Rx(&sres[1], &hdr[1]),
        Hex(&val[1])
    ));
    Spi2::init(io);
}

/// The carrier 3.3 V system rail seen from the SoC, read only (same
/// signals as the YY3588 EDK2 PowerRailProbe): pad levels of I2C6
/// SCL/SDA (GPIO0_D0/C7) and GMAC1 MDIO (GPIO3_C3), whose pull-ups sit on
/// that rail, and USB2PHY0 session valid (VBUS-detect pulled up to it).
/// GPIO3 / USB2PHY0_GRF are read only if their pclk is not gated.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CarrierProbe {
    pub scl: u32,
    pub sda: u32,
    pub mdio: Option<u32>,
    pub sessvld: Option<u32>,
}

impl CarrierProbe {
    pub fn present(&self) -> bool {
        self.scl != 0 || self.sda != 0 || self.mdio == Some(1)
    }
}

impl fmt::Display for CarrierProbe {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let o = |v: Option<u32>| match v {
            Some(0) => "0",
            Some(_) => "1",
            None => "gated",
        };
        write!(
            f,
            "i2c6 scl={} sda={} gmac1 mdio={} u2phy0 sessvld={} -> carrier 3.3 V {}",
            self.scl,
            self.sda,
            o(self.mdio),
            o(self.sessvld),
            if self.present() { "present" } else { "ABSENT" }
        )
    }
}

pub fn carrier_probe<H: RegIo + ?Sized>(io: &mut H) -> CarrierProbe {
    use crate::regs::{
        CRU_GATE_CON17, CRU_GATE_CON72, GPIO3, PCLK_GPIO3_BIT, PCLK_USB2PHY0_GRF_BIT, USB2PHY0_GRF,
        USB2PHY_GRF_STATUS0,
    };
    let g0 = io.read32(GPIO0 + GPIO_EXT_PORT);
    let mdio = (io.read32(CRU_GATE_CON17) >> PCLK_GPIO3_BIT & 1 == 0)
        .then(|| (io.read32(GPIO3 + GPIO_EXT_PORT) >> 19) & 1);
    let sessvld = (io.read32(CRU_GATE_CON72) >> PCLK_USB2PHY0_GRF_BIT & 1 == 0)
        .then(|| (io.read32(USB2PHY0_GRF + USB2PHY_GRF_STATUS0) >> 11) & 1);
    CarrierProbe {
        scl: (g0 >> 24) & 1,
        sda: (g0 >> 23) & 1,
        mdio,
        sessvld,
    }
}

/// What the PMIC step found and did.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Report {
    pub chip: u16,
    pub en1: u8,
    pub en2: u8,
    pub en3: u8,
    pub enabled_now: u8,
}

/// Enable a rail (`bit` of POWER_ENx) if it reads off.
fn ensure<H: RegIo + ?Sized>(
    s: &mut Spi2,
    io: &mut H,
    reg: u8,
    bit: u8,
    now: &mut u8,
    tag: u8,
) -> Result<u8, Timeout> {
    let v = s.read(io, reg)?;
    if v & (1 << bit) == 0 {
        s.write(io, reg, (1 << bit) | (1 << (4 + bit)))?;
        *now |= tag;
    }
    Ok(v)
}

pub fn ddr_rails<H: RegIo + ?Sized>(io: &mut H) -> Result<Report, Timeout> {
    let mut s = Spi2::init(io);
    // Never write to a PMIC that did not identify itself (a broken SPI
    // link reads 0x00, 0xff or the bytes sent).
    let Some(chip) = s.identify(io).chip else {
        return Err(Timeout("rk806 id"));
    };
    // vdd_ddr_s0 0.85 V (inside the DTS range 675-900 mV) and
    // vdd_ddr_pll_s0 0.85 V (the DTS value).
    s.write(io, BUCK5_ON_VSEL, buck_vsel(850_000))?;
    s.write(io, NLDO2_ON_VSEL, ldo_vsel(850_000))?;
    let mut now = 0;
    let en1 = ensure(&mut s, io, POWER_EN1, 0, &mut now, 1)?; // BUCK5
    ensure(&mut s, io, POWER_EN1, 1, &mut now, 2)?; // BUCK6
    let en2 = ensure(&mut s, io, POWER_EN2, 0, &mut now, 4)?; // BUCK9
    let en3 = ensure(&mut s, io, POWER_EN3, 1, &mut now, 8)?; // NLDO2
    if now != 0 {
        io.udelay(1_000); // rail ramp
    }
    Ok(Report {
        chip,
        en1,
        en2,
        en3,
        enabled_now: now,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn encodings() {
        assert_eq!(buck_vsel(850_000), 56);
        assert_eq!(ldo_vsel(850_000), 28);
        assert_eq!(cmd(true, 1), 0x80);
        assert_eq!(cmd(false, 1), 0x00);
        assert_eq!(read_frame(CHIP_NAME), [0x00, 0x5A, 0x00]);
        assert_eq!(write_frame(POWER_EN1, 0x11), [0x80, 0x01, 0x00, 0x11]);
    }
    #[test]
    fn ctrlr0_matches_linux() {
        // spi-rockchip.c: FRF_SPI | BHT_8BIT << 13 | SSD_ONE << 10 |
        // EM_BIG << 11 | DFS_8BIT, mode 0, XFM_TR, OPM master.
        assert_eq!(CTRLR0_VAL, 0x2C01);
        assert_ne!(CTRLR0_VAL & spi::CR0_BHT_8BIT, 0);
    }
    #[test]
    fn all_rails_table() {
        // 21 distinct rails, each enable bit used once, POWER_EN0..5 only.
        let mut seen = [0u8; 6];
        for &(_, _, _, en, bit) in ALL_RAILS.iter() {
            assert!(en < 6 && bit < 4);
            assert_eq!(seen[en as usize] & 1 << bit, 0);
            seen[en as usize] |= 1 << bit;
        }
        assert_eq!(seen, [0x0f, 0x0f, 0x03, 0x0f, 0x0f, 0x07]);
        for (i, r) in STATE_REGS[..6].iter().enumerate() {
            assert_eq!(r.1 as usize, i);
        }
        assert_eq!(buck_uv(232), 3_300_000);
        assert_eq!(ldo_uv(ldo_vsel(3_300_000)), 3_300_000);
    }

    #[test]
    fn voltage_decode() {
        assert_eq!(buck_uv(buck_vsel(850_000)), 850_000);
        assert_eq!(buck_uv(0x58), 1_050_000);
        assert_eq!(buck_uv(159), 1_493_750);
        assert_eq!(buck_uv(160), 1_500_000);
        assert_eq!(buck_uv(0xA4), 1_600_000);
        assert_eq!(buck_uv(255), 3_400_000);
        assert_eq!(ldo_uv(ldo_vsel(850_000)), 850_000);
        assert_eq!(ldo_uv(240), 3_400_000);
    }
}
