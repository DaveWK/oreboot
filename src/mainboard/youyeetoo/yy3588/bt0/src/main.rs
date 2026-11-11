// SPDX-License-Identifier: GPL-2.0-only
//! youyeetoo YY3588 (Rockchip RK3588) bt0.
//!
//! eMMC boot image (feature `stepping-stone`, linked at DRAM 0x0): the
//! BootROM ran Rockchip's rkbin DDR init from the idblock's init image, got
//! control back and loaded bt0 (the boot image). bt0 checks the documented
//! OS_REG2..5 handoff, reads the edk2-rk3588 FIT from eMMC at 0x100000,
//! places TF-A BL31 and EDK2, and enters BL31 with bl_params (BL33 = EDK2,
//! no BL32). The image contains the rkbin DDR init blob.
//!
//! MaskROM USB loader (features `stepping-stone,usb-loader`): the 472 part
//! after the rkbin DDR init; fastboot on the USB-C port (usb.rs), used to
//! flash the eMMC or RAM-boot a FIT.
//!
//! Feature `diag` (implies `dram-info`): the full RK806 register and rail
//! dump, SPI2 diagnostics and the dram-info report. Off in the eMMC image
//! and the flashing loader.
//!
//! Default build (SYSTEM_SRAM 0xff001000): our own DRAM init (feature
//! `own-ddr`, dram.rs) is paused until the rkbin-based image is complete.

#![no_std]
#![no_main]

mod boot;
mod dram;
mod emmc;
mod mmio;
mod uart;
mod usb;

use core::fmt::Write;
use core::panic::PanicInfo;
use rk3588::{dump, rk806};

/// # Safety
/// Entry from the BootROM: counter frequency, stack, .bss, then main.
#[unsafe(naked)]
#[unsafe(no_mangle)]
#[unsafe(link_section = ".text.entry")]
pub unsafe extern "C" fn start() -> ! {
    core::arch::naked_asm!(
        "mov    w0, #0x3600",
        "movk   w0, #0x016e, lsl #16",
        "msr    cntfrq_el0, x0",
        "ldr    x0, =_stack_top",
        "mov    sp, x0",
        "ldr    x0, =_sbss",
        "ldr    x1, =_ebss",
        "2:",
        "cmp    x0, x1",
        "b.hs   3f",
        "str    xzr, [x0], #8",
        "b      2b",
        "3:",
        "bl     {main}",
        "4:",
        "wfe",
        "b      4b",
        main = sym main,
    )
}

fn current_el() -> u64 {
    let v: u64;
    unsafe { core::arch::asm!("mrs {}, CurrentEL", out(reg) v) };
    (v >> 2) & 3
}

extern "C" fn main() {
    mmio::stimer_start();
    let mut io = mmio::Mmio {
        uart: uart::Uart2::init(),
    };
    let flavour = if cfg!(feature = "stepping-stone") {
        "(rkbin DDR init)"
    } else {
        "(own DDR init, paused)"
    };
    let _ = writeln!(io, "\nyy3588 bt0 {flavour} at EL{}", current_el());
    if cfg!(feature = "diag") {
        pmic_verbose(&mut io);
    } else {
        pmic_brief(&mut io);
    }

    let dram = match dram::bring_up(&mut io) {
        Ok(d) => d,
        Err(why) => {
            let _ = writeln!(io, "dram: {why}. stop.");
            return;
        }
    };
    let total = dram.total_bytes();
    let _ = writeln!(
        io,
        "dram: {:?} {} {} MiB (pairs {} + {})",
        dram.source,
        dram.type_name(),
        total >> 20,
        dram.pairs[0].size_mib(),
        dram.pairs[1].size_mib()
    );

    if cfg!(feature = "dram-info") {
        // SAFETY: DRAM is up and the record window [0x1F0000, 0x200000) is
        // outside bt0, the staging buffer and every FIT destination
        // (boot::check_overlaps keeps it free).
        let rec_buf = unsafe {
            core::slice::from_raw_parts_mut(dump::RECORD_ADDR as *mut u8, dump::RECORD_BYTES)
        };
        let mut rec = dump::Record::new(rec_buf);
        let mut text = mmio::Mmio { uart: uart::Uart2 };
        let _ = dump::report(&mut io, &mut text, &mut rec, dram.channels(), dram.ranks());
    }

    if !boot::memcheck(&mut io) {
        let _ = writeln!(io, "dram: quick check FAILED. stop.");
        return;
    }
    soc_clocks(&mut io);
    if cfg!(feature = "usb-loader") {
        usb::enter(&mut io);
        return;
    }
    match boot::load_and_enter(&mut io, total) {
        Ok(()) => {}
        Err(e) => {
            let _ = writeln!(io, "boot failed: {e}. stop.");
        }
    }
}

/// CPLL / GPLL / PPLL to the rates Rockchip's SPL sets (bt0 takes the SPL's
/// place). EDK2 hard-codes CPLL 1500 / GPLL 1188 MHz in its divider maths
/// and the vendor kernel inherits CPLL as found; left at the post-DDR-init
/// 750 / 594 MHz (PPLL in slow mode), the GMAC 125 MHz and the combphy PCIe
/// reference come out at half rate or worse. UART2 runs from xin24m.
fn soc_clocks(io: &mut mmio::Mmio) {
    use regio::RegIo;
    use rk3588::pll::{soc_clocks_init, soc_pll_mhz, CPLL_1500, GPLL_1188, PPLL_1100};
    use rk3588::regs::base::CRU;
    let plls = [&CPLL_1500, &GPLL_1188, &PPLL_1100];
    let before = plls.map(|p| soc_pll_mhz(io, CRU, p));
    let regs = [0x280, 0x398, 0x324].map(|o| io.read32(CRU + o) & 0xFFFF);
    let _ = writeln!(
        io,
        "clocks: mode {:#06x} sel38 {:#06x} sel09 {:#06x}",
        regs[0], regs[1], regs[2]
    );
    let result = soc_clocks_init(io, CRU, |io, what| {
        let _ = writeln!(io, "clocks: {what}");
    });
    let after = plls.map(|p| soc_pll_mhz(io, CRU, p));
    let _ = writeln!(
        io,
        "clocks: CPLL {} -> {}, GPLL {} -> {}, PPLL {} -> {} MHz{}",
        before[0],
        after[0],
        before[1],
        after[1],
        before[2],
        after[2],
        match result {
            Ok(()) => "",
            Err(_) => " (lock timeout)",
        }
    );
}

/// One PMIC line and one carrier-rail line (read only).
fn pmic_brief(io: &mut mmio::Mmio) {
    match rk806::readback(io) {
        Ok(r) if r.id_ok() => {
            let d = r.decoded();
            let _ = writeln!(
                io,
                "pmic: rk806 {:#06x}, {}, {}/21 rails on",
                r.chip(),
                if d.active() { "active" } else { "NOT active" },
                d.rails_on
            );
        }
        Ok(r) => {
            let _ = writeln!(io, "pmic: rk806 not identified ({})", r.ident);
        }
        Err(t) => {
            let _ = writeln!(io, "pmic: rk806 read failed: {}", t.0);
        }
    }
    let probe = rk806::carrier_probe(io);
    let _ = writeln!(
        io,
        "carrier 3.3 V: {}",
        if probe.present() {
            "on"
        } else {
            "OFF (USB hub, Ethernet PHY, RTC unpowered)"
        }
    );
}

/// Feature `diag`: every RK806 register bt0 knows, every rail, the carrier
/// probe inputs, and the SPI2 diagnostics when the id is bad (read only).
fn pmic_verbose(io: &mut mmio::Mmio) {
    let pmic = rk806::readback(io);
    match pmic {
        Ok(r) => {
            let _ = writeln!(
                io,
                "pmic rk806 spi2 ver={:#010x} mode={} chip_name={:#04x} chip_ver={:#04x} id={} ({})",
                r.spi_version,
                r.mode.name(),
                r.chip_name,
                r.chip_ver,
                if r.id_ok() { "ok" } else { "BAD" },
                r.ident
            );
            let _ = writeln!(
                io,
                "pmic rk806 power_en1={:#04x} power_en2={:#04x} power_en3={:#04x}",
                r.en1, r.en2, r.en3
            );
            for (name, vsel, uv, on) in r.rails() {
                let on = if on { "on" } else { "off" };
                let _ = if uv == 0 {
                    // External feedback: VSEL unused (rk806::Readback::rails).
                    writeln!(io, "pmic rk806 {name} vsel={vsel:#04x} ext-fb {on}")
                } else {
                    writeln!(
                        io,
                        "pmic rk806 {name} vsel={vsel:#04x} {}.{:03} mV {on}",
                        uv / 1000,
                        uv % 1000
                    )
                };
            }
            // EXT_EN (carrier VCC5V0_SYS/VCC3V3_SYS enable) follows the
            // RK806 power state; no register drives it. Read-only state
            // and the SoC PMIC_SLEEP1/2 pins, to see which state it is in.
            if r.id_ok() {
                let _ = write!(io, "pmic rk806 state");
                for (&(name, _), v) in rk806::STATE_REGS.iter().zip(r.state) {
                    let _ = write!(io, " {name}={v:#04x}");
                }
                let _ = writeln!(io);
                // Why the RK806 last turned off / on, and the input
                // under/overvoltage status and settings.
                let p = r.power();
                let _ = writeln!(io, "pmic rk806 power {p}");
                let _ = writeln!(io, "pmic rk806 power cfg {}", p.cfg());
                // Every rail's voltage, read only: is the UART2 pad supply
                // (PMUIO2) at a sane level?
                for (name, vsel, uv, on) in r.all_rails() {
                    let _ = writeln!(
                        io,
                        "pmic rk806 rail {name} vsel={vsel:#04x} {}.{:03} mV {}",
                        uv / 1000,
                        uv % 1000,
                        if on { "on" } else { "off" }
                    );
                }
                let d = r.decoded();
                let _ = writeln!(
                    io,
                    "pmic rk806 decode pwrctrl1/2/3 fun={}/{}/{} pol={}/{}/{} dev_slp={} dev_off={} rails_on={}/21 -> {}",
                    d.pwrctrl_fun[0],
                    d.pwrctrl_fun[1],
                    d.pwrctrl_fun[2],
                    d.pwrctrl_pol[0],
                    d.pwrctrl_pol[1],
                    d.pwrctrl_pol[2],
                    d.dev_slp as u8,
                    d.dev_off as u8,
                    d.rails_on,
                    if d.active() {
                        "ACTIVE, no sleep source (EXT_EN expected high)"
                    } else {
                        "NOT plain ACTIVE"
                    }
                );
            }
            let _ = writeln!(
                io,
                "pmic soc sleep pins gpio0_a2/a3 mux={}/{} level={}/{}",
                (r.sleep_sel >> 8) & 0xf,
                (r.sleep_sel >> 12) & 0xf,
                (r.sleep_pads >> 1) & 1,
                r.sleep_pads & 1
            );
        }
        Err(t) => {
            let _ = writeln!(io, "pmic rk806 read failed: {}", t.0);
        }
    }
    // The carrier 3.3 V rail (enabled by PMIC_EXT_EN_OUT), as the EDK2
    // probe sees it, but at bt0 time. Read only.
    let probe = rk806::carrier_probe(io);
    let _ = writeln!(io, "pmic carrier probe [bt0] {probe}");
    // Not an RK806 answer: print the SPI2 pin/controller/bus diagnostics
    // (SoC registers and RK806 reads only).
    if !pmic.is_ok_and(|r| r.id_ok()) {
        rk806::diagnose(io);
    }
}

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    let mut u = uart::Uart2;
    let _ = writeln!(u, "\nbt0 panic: {info}");
    loop {
        core::hint::spin_loop();
    }
}
