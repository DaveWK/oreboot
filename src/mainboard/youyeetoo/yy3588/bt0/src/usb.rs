// SPDX-License-Identifier: GPL-2.0-only
//! MaskROM USB launch (feature `usb-loader`: bt0 is the 472 part of the
//! loader `rkdeveloptool db` sends, after the rkbin DDR init as the 471
//! part): fastboot on the USB-C OTG port (USB3OTG0, crates dwc3-udc and
//! rk3588-fastboot), as the K1 bt0 serves fastboot when its BootROM started
//! it from USB download mode.
//!
//! - `fastboot boot <fit>` (or `stage <fit>` + `continue`): the FIT is
//!   checked, then placed and BL31 entered exactly as boot.rs does after
//!   its eMMC read (crate rk3588 fit + handoff, via rk3588_fastboot::stage).
//! - `fastboot flash <target> <file>`: eMMC user area, boot0/1, or the
//!   idblock / FIT regions, through the bt0 eMMC glue (emmc.rs + sdhci).
//! - `fastboot continue` with nothing staged: the normal eMMC boot
//!   (boot::load_and_enter), like the K1 bt0 after fastboot.
//! - `fastboot reboot`: CRU global soft reset.
//!
//! See boards/rk3588/yy3588/bt0/FASTBOOT.md.

use crate::boot;
use crate::dram;
use crate::emmc::{EmmcIo, Rk3588Emmc};
use crate::mmio::Mmio;
use crate::uart::Uart2;
use core::fmt::Write;
use core::ptr::addr_of_mut;
use dwc3_udc::{Dma, Udc};
use fastboot::Line;
use regio::RegIo;
use rk3588::regs::base;
use rk3588_fastboot::device::{Board, Next, Part, Rk3588Device};
use rk3588_fastboot::platform::Otg0;
use rk3588_fastboot::{stage, DOWNLOAD_BUF, USB3OTG0_BASE};
use sdhci::{Kind, Partition, Sdhci};

const PRODUCT: &str = "yy3588";
const SERIAL: &str = "rk3588";

/// The controller's event buffer, TRBs and control buffers (bt0 .bss in
/// DRAM, reachable by the DWC3's AXI master).
static mut DMA: Dma = Dma::new();
/// The register access the eMMC glue borrows for the whole session.
static mut EMMC_MMIO: Mmio = Mmio { uart: Uart2 };

/// CRU_GLB_SRST_FST_VALUE (CRU + 0x0C08) = 0xfdb9: first global software
/// reset, TRM Part 1 p.31 / p.203.
const CRU_GLB_SRST_FST: usize = base::CRU + 0x0c08;

/// FIREWALL_DDR_BASE 0xfe030000 (TF-A rk3588_def.h): RGN(i) at 4 * i,
/// MST(i) at 0x40 + 4 * i, CON (region enables, bit i) at 0xf0.
const FIREWALL_DDR: usize = 0xfe03_0000;
const FIREWALL_DDR_CON: usize = FIREWALL_DDR + 0xf0;

/// eMMC write modes, fastest first: bus width, card clock (above 26 MHz:
/// high-speed timing from the GPLL source, else 25 MHz from xin 24 MHz),
/// SDMA or PIO. A failed open or write re-initialises the card in the next
/// mode; the last is the 1-bit PIO path flashing always used.
const MODES: [(u8, u32, bool); 6] = [
    (8, 52_000, true),
    (8, 52_000, false),
    (8, 25_000, true),
    (8, 25_000, false),
    (4, 25_000, false),
    (1, 25_000, false),
];

/// The eMMC and the reset for rk3588_fastboot::device.
struct Bt0Board {
    mmc: Option<Sdhci<EmmcIo<'static, Mmio>, Rk3588Emmc>>,
    /// Index into [`MODES`].
    mode: usize,
    /// The card clock the open set, in kHz.
    khz: u32,
    /// The hardware partition last selected (restored after a re-open).
    part: Partition,
    /// The card reads TRIMmed sectors back as zeros (checked at the first
    /// open, cleared when a check fails).
    trim: Option<bool>,
    /// The first failure: mode index, LBA (u32::MAX for an open), error.
    first_error: Option<(usize, u32, sdhci::Error)>,
}

/// Room for a fastboot FAIL reason built at run time.
static mut REASON: [u8; 64] = [0; 64];

/// Format a FAIL reason into [`REASON`] (bt0 serves one request at a time).
fn reason(args: core::fmt::Arguments) -> &'static str {
    struct Buf(&'static mut [u8; 64], usize);
    impl Write for Buf {
        fn write_str(&mut self, s: &str) -> core::fmt::Result {
            let n = s.len().min(self.0.len() - self.1);
            self.0[self.1..self.1 + n].copy_from_slice(&s.as_bytes()[..n]);
            self.1 += n;
            Ok(())
        }
    }
    // SAFETY: single-threaded; the previous reason was already sent.
    let mut b = Buf(unsafe { &mut *addr_of_mut!(REASON) }, 0);
    let _ = b.write_fmt(args);
    let n = b.1;
    core::str::from_utf8(&b.0[..n]).unwrap_or("eMMC error")
}

impl Bt0Board {
    const fn new() -> Self {
        Self {
            mmc: None,
            mode: 0,
            khz: 0,
            part: Partition::User,
            trim: None,
            first_error: None,
        }
    }

    /// Initialise the card and set up [`MODES`]`[self.mode]`.
    fn open(&mut self) -> Result<Sdhci<EmmcIo<'static, Mmio>, Rk3588Emmc>, sdhci::Error> {
        // The rkbin DDR init leaves DDR firewall region 0 over all of DRAM
        // (FIREWALL_DDR_RGN0 0xffff0000, CON 0xffff) with nearly every
        // master denied (MST 0xffffffff; USB is let through), so the eMMC
        // SDMA reads fail with a bus (ADMA) error. Turn the regions off, as
        // nothing here needs them; BL31 sets its own up when it starts
        // (TF-A plat/rockchip/rk3588 secure.c secure_region_init).
        Mmio { uart: Uart2 }.write32(FIREWALL_DDR_CON, 0);
        // SAFETY: EMMC_MMIO is used by this board only, which exists once
        // per session, and only through the one open card.
        let io = EmmcIo(unsafe { &mut *addr_of_mut!(EMMC_MMIO) });
        let mut m = Sdhci::init(io, Rk3588Emmc, base::EMMC, Kind::Emmc)?;
        let (width, khz, _) = MODES[self.mode];
        if width != 1 {
            m.set_width(width)?;
            m.ext_csd()?;
        }
        self.khz = if khz > 26_000 {
            m.set_khz(5_000)?;
            if !crate::emmc::source_from_gpll(m.io()) {
                return Err(sdhci::Error::ClockSource);
            }
            let k = m.set_khz(khz)?;
            m.ext_csd()?;
            k
        } else {
            m.set_khz(khz)?
        };
        if self.part != Partition::User {
            m.select(self.part)?;
        }
        if self.trim.is_none() {
            self.trim = Some(m.trim_zeroes());
        }
        let _ = writeln!(
            Mmio { uart: Uart2 },
            "[fastboot] eMMC {width}-bit at {} kHz, {}, TRIM for zeros: {}",
            self.khz,
            if MODES[self.mode].2 { "SDMA" } else { "PIO" },
            self.trim == Some(true)
        );
        Ok(m)
    }

    /// Record a failure and move to the next mode (closing the card).
    fn step_down(&mut self, lba: u32, e: sdhci::Error) -> Result<(), &'static str> {
        let _ = writeln!(
            Mmio { uart: Uart2 },
            "[fastboot] eMMC mode {} failed at {lba:#x}: {e:?}",
            self.mode
        );
        self.first_error.get_or_insert((self.mode, lba, e));
        self.mmc = None;
        self.mode += 1;
        if self.mode == MODES.len() {
            self.mode -= 1;
            return Err(reason(format_args!("eMMC: {e:?} at {lba:#x}")));
        }
        Ok(())
    }

    fn mmc(&mut self) -> Result<&mut Sdhci<EmmcIo<'static, Mmio>, Rk3588Emmc>, &'static str> {
        while self.mmc.is_none() {
            match self.open() {
                Ok(m) => self.mmc = Some(m),
                Err(e) => self.step_down(u32::MAX, e)?,
            }
        }
        Ok(self.mmc.as_mut().unwrap())
    }
}

fn partition(p: Part) -> Partition {
    match p {
        Part::User => Partition::User,
        Part::Boot0 => Partition::Boot0,
        Part::Boot1 => Partition::Boot1,
    }
}

impl Board for Bt0Board {
    fn emmc_select(&mut self, part: Part) -> Result<(), &'static str> {
        self.part = partition(part);
        let p = self.part;
        self.mmc()?.select(p).map_err(|_| "eMMC partition select")
    }
    fn emmc_sectors(&mut self, part: Part) -> Result<u64, &'static str> {
        self.mmc()?
            .sectors(partition(part))
            .map_err(|_| "eMMC EXT_CSD")
    }
    fn emmc_write(&mut self, lba: u32, data: &[u8]) -> Result<(), &'static str> {
        let (count, src) = (data.len() / 512, data.as_ptr() as usize);
        loop {
            let dma = MODES[self.mode].2;
            let m = self.mmc()?;
            let r = if dma {
                m.write_dma(lba, count, src)
            } else {
                m.write(lba, count, src)
            };
            match r {
                Ok(()) => return Ok(()),
                Err(e) => self.step_down(lba, e)?,
            }
        }
    }
    fn emmc_zero(&mut self, lba: u32, sectors: u64) -> Result<bool, &'static str> {
        if self.trim != Some(true) || sectors == 0 {
            return Ok(false);
        }
        let count = usize::try_from(sectors).map_err(|_| "zero extent")?;
        let m = self.mmc()?;
        if m.trim(lba, count).is_err() {
            self.trim = Some(false);
            return Ok(false);
        }
        // Check the first and the last sector read back as zeros.
        let last = lba + (count - 1) as u32;
        let mut sector = [0u32; 128];
        for at in [lba, last] {
            let ok = m.read_sector(at, sector.as_mut_ptr() as usize).is_ok()
                && sector.iter().all(|w| *w == 0);
            if !ok {
                let _ = writeln!(
                    Mmio { uart: Uart2 },
                    "[fastboot] TRIM at {at} did not read back zeros; writing zeros from here"
                );
                self.trim = Some(false);
                return Ok(false);
            }
        }
        Ok(true)
    }
    fn getvar(&mut self, name: &str, out: &mut Line) -> bool {
        match name {
            // The carrier 3.3 V rail (VCC3V3_SYS: USB hub, Ethernet PHY,
            // RTC) as bt0 printed it at start, re-read now: the state can
            // be read over USB with the debug serial unplugged.
            "carrier" => {
                let p = rk3588::rk806::carrier_probe(&mut Mmio { uart: Uart2 });
                let _ = write!(out, "{} ({p})", if p.present() { "on" } else { "OFF" });
                return true;
            }
            "pmic" => {
                let _ = match rk3588::rk806::readback(&mut Mmio { uart: Uart2 }) {
                    Ok(r) if r.id_ok() => {
                        let d = r.decoded();
                        write!(
                            out,
                            "rk806 {:#06x} {} {}/21 rails on",
                            r.chip(),
                            if d.active() { "active" } else { "NOT active" },
                            d.rails_on
                        )
                    }
                    Ok(r) => write!(out, "rk806 not identified ({})", r.ident),
                    Err(t) => write!(out, "rk806 read failed: {}", t.0),
                };
                return true;
            }
            "emmc-error" => {
                let _ = match self.first_error {
                    Some((mode, lba, e)) => write!(out, "mode {mode} lba {lba:#x} {e:?}"),
                    None => write!(out, "none"),
                };
                return true;
            }
            "emmc-regs" => {
                // SDHCI HOST_VERSION (0xFE), HOST_CONTROL2 (0x3E: bit 12 v4
                // mode, bit 13 64-bit addressing), HOST_CONTROL (0x28).
                let mut io = Mmio { uart: Uart2 };
                let _ = write!(
                    out,
                    "ver {:#06x} hc2 {:#06x} hc {:#04x} caps {:#010x}",
                    io.read16(base::EMMC + 0xFE),
                    io.read16(base::EMMC + 0x3E),
                    io.read8(base::EMMC + 0x28),
                    io.read32(base::EMMC + 0x40)
                );
                return true;
            }
            "emmc" => {}
            _ => {
                // DDR firewall state left by the rkbin DDR init (BL31 sets
                // it up later; TF-A plat/rockchip/rk3588 secure.c layout):
                // fw-con = FIREWALL_DDR_CON, fw-rgn:<i> = RGN(i),
                // fw-mst:<i> = MST(i), from FIREWALL_DDR_BASE 0xfe030000.
                let mut io = Mmio { uart: Uart2 };
                let reg = if name == "fw-con" {
                    Some(0xf0)
                } else if let Some(i) = name
                    .strip_prefix("fw-rgn:")
                    .and_then(|s| s.parse::<usize>().ok())
                {
                    (i < 16).then_some(i * 4)
                } else if let Some(i) = name
                    .strip_prefix("fw-mst:")
                    .and_then(|s| s.parse::<usize>().ok())
                {
                    (i < 42).then_some(0x40 + i * 4)
                } else {
                    None
                };
                return match reg {
                    Some(off) => write!(out, "{:#010x}", io.read32(FIREWALL_DDR + off)).is_ok(),
                    None => false,
                };
            }
        }
        let (width, _, dma) = MODES[self.mode];
        let _ = write!(
            out,
            "mode {} {width}-bit {} kHz {} trim {:?}",
            self.mode,
            self.khz,
            if dma { "SDMA" } else { "PIO" },
            self.trim
        );
        if let Some((mode, lba, e)) = self.first_error {
            let _ = write!(out, "; first error mode {mode} at {lba:#x}: {e:?}");
        }
        true
    }
    fn reset(&mut self) -> Result<(), &'static str> {
        let mut io = Mmio { uart: Uart2 };
        let _ = writeln!(io, "fastboot: reboot");
        io.udelay(2_000);
        io.write32(CRU_GLB_SRST_FST, 0xfdb9);
        loop {
            core::hint::spin_loop();
        }
    }
}

/// bt0's memory and the jump into BL31 (MMU and caches off).
struct Bt0Machine<'a>(&'a mut Mmio);

impl stage::Machine for Bt0Machine<'_> {
    fn copy(&mut self, dst: usize, src: &[u8]) {
        let s = src.as_ptr() as usize;
        // SAFETY: stage::prepare checked that the destinations overlap
        // neither bt0, the handoff window nor the download buffer.
        unsafe {
            if (dst | s) & 7 == 0 {
                let n = src.len() / 8;
                for i in 0..n {
                    let v = ((s + 8 * i) as *const u64).read_volatile();
                    ((dst + 8 * i) as *mut u64).write_volatile(v);
                }
                for (i, b) in src.iter().enumerate().skip(8 * n) {
                    ((dst + i) as *mut u8).write_volatile(*b);
                }
            } else {
                for (i, b) in src.iter().enumerate() {
                    ((dst + i) as *mut u8).write_volatile(*b);
                }
            }
        }
    }
    fn enter_bl31(&mut self, entry: usize, params: usize, fdt: usize) {
        let _ = writeln!(
            self.0,
            "enter BL31 at {entry:#x} (bl_params {params:#x}, fdt {fdt:#x})"
        );
        self.0.udelay(2_000); // let the console drain
                              // SAFETY: the entry point of the BL31 image just placed (boot.rs).
        unsafe {
            core::arch::asm!("dsb sy", "ic iallu", "dsb sy", "isb");
            let f: extern "C" fn(usize, usize, usize, usize) -> ! = core::mem::transmute(entry);
            f(params, fdt, 0, 0);
        }
    }
}

/// Serve fastboot; never returns to the caller's eMMC path unless the host
/// sent `continue` without a staged image.
pub fn enter(io: &mut Mmio) {
    // main.rs read the OS_REG handoff already; read it again here rather
    // than change the hook's signature.
    let total = match dram::bring_up(io) {
        Ok(d) => d.total_bytes(),
        Err(why) => {
            let _ = writeln!(io, "fastboot: dram: {why}. stop.");
            return;
        }
    };
    // The BootROM's boot source id: USB for a MaskROM download (a check
    // for a later runtime selection; this build serves fastboot anyway).
    let id = io.read32(rk3588_fastboot::BOOTSOURCE_ID);
    let _ = writeln!(
        io,
        "fastboot: BootROM boot source {id:#x} ({})",
        if rk3588_fastboot::launched_over_usb(id) {
            "USB"
        } else {
            "not USB"
        }
    );
    let size = rk3588_fastboot::download_size(total);
    if size == 0 {
        let _ = writeln!(io, "fastboot: no DRAM for the download buffer. stop.");
        return;
    }
    let keep = [
        stage::BT0_STEPPING_STONE,
        (rk3588::dump::RECORD_ADDR, rk3588::dump::RECORD_BYTES),
        stage::HANDOFF_WINDOW,
        (DOWNLOAD_BUF, size),
    ];
    let _ = writeln!(
        io,
        "fastboot: USB-C OTG (USB3OTG0), 18d1:4ee0, download buffer {DOWNLOAD_BUF:#x} + {size:#x}"
    );
    // SAFETY: the only reference to DMA; the controller is quiesced
    // (disconnect) before bt0 leaves this function or jumps.
    let dma = unsafe { &mut *addr_of_mut!(DMA) };
    let mut udc = match Udc::init(
        Mmio { uart: Uart2 },
        Otg0::default(),
        dwc3_udc::Config::new(USB3OTG0_BASE),
        dma,
        rk3588_fastboot::descriptors(PRODUCT, SERIAL),
    ) {
        Ok(u) => u,
        Err(e) => {
            let _ = writeln!(io, "fastboot: USB controller: {e:?}. stop.");
            return;
        }
    };
    // SAFETY: DRAM is up and [DOWNLOAD_BUF, + size) is outside bt0 and
    // every other buffer (rk3588_fastboot::DOWNLOAD_BUF).
    let buf = unsafe { core::slice::from_raw_parts_mut(DOWNLOAD_BUF as *mut u8, size) };
    let mut dev = Rk3588Device::new(Bt0Board::new(), PRODUCT, SERIAL, &keep);
    if let Err(e) = fastboot::serve(
        &mut udc,
        &mut dev,
        &fastboot::Config { unlocked: true },
        buf,
    ) {
        let _ = writeln!(io, "fastboot: USB transport: {e:?}. stop.");
        return;
    }
    match dev.next {
        Next::Boot { plan, addr, len } => {
            // SAFETY: the downloaded image, still in the buffer.
            let fit = unsafe { core::slice::from_raw_parts(addr as *const u8, len) };
            for i in plan.images() {
                let _ = writeln!(
                    io,
                    "load {:<8} {:#010x} <- fit+{:#x} ({} bytes)",
                    i.name(),
                    i.load,
                    i.offset,
                    i.size
                );
            }
            if let Some((i, n)) = stage::filled(&plan, len) {
                let _ = writeln!(
                    io,
                    "fastboot: {} past download, filled {n:#x} bytes with {:#04x}",
                    i.name(),
                    stage::NV_FILL
                );
            }
            let layout = stage::Layout {
                fdt_addr: stage::FDT_ADDR,
                params_addr: stage::PARAMS_ADDR,
                bl33_default: 0x20_0000,
                keep: &keep,
            };
            stage::enter(&mut Bt0Machine(io), fit, &plan, total, &layout);
        }
        Next::Continue => {
            let _ = writeln!(io, "fastboot: continue: booting from eMMC");
            if let Err(e) = boot::load_and_enter(io, total) {
                let _ = writeln!(io, "boot failed: {e}. stop.");
            }
        }
    }
}
