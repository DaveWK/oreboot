//! LPDDR5 DRAM initialisation for the Allwinner A733 on the Radxa Cubie
//! A7A, written from the register-level behaviour of Allwinner's DRAM
//! library (recorded by the emulator harness in ore-edk-boot-radxa-a7a
//! `tools/dram-harness`), the A733 manual, the DesignWare controller and PHY
//! register conventions and JEDEC LPDDR5/5X. It links no vendor code; the PHY's
//! training firmware is loaded as data (see [`phy`]).
#![no_std]

pub mod ccu;
pub mod ctrl;
pub mod hw;
pub mod mr;
pub mod para;
pub mod phy;
pub mod post;
pub mod power;
pub mod sets;
pub mod size;
pub mod timing;

pub use hw::{Error, Hw, Rail};
pub use para::Para;
pub use phy::PhyFirmware;

use ctrl::{ddrc, DDRC_184, DDRC_B84, PWRCTL};

/// Controllers and PHY of every channel: clocks, controller set-up, PHY
/// training, DFI handshake. Returns after the controllers run normally.
pub fn core_init<H: Hw>(hw: &mut H, p: &mut Para, fw: &PhyFirmware) -> Result<(), Error> {
    ccu::sys_init(hw, p.clk())?;
    let channels = p.channels();
    ctrl::com_pre_init(hw, p);
    for ch in 0..channels {
        ctrl::bus_config(hw, ch);
        hw.set32(ddrc(ch, DDRC_B84), 1);
        hw.write32(ddrc(ch, DDRC_184), 2);
        ctrl::com_init(hw, p, ch)?;
    }
    let rl = if p.tpr10() & (1 << 31) != 0 { 14 } else { 13 };
    ctrl::com_post_init(hw, p);
    phy::init::init(hw, p, fw, rl)?;
    for ch in 0..channels {
        post::timing_after_phy(hw, ch)?;
    }
    post::dfi_init(hw, channels)?;
    post::controller_after_phy(hw, p)?;
    if p.tpr13() & (1 << 24) != 0 {
        for ch in 0..channels {
            hw.set32(ddrc(ch, PWRCTL), 0x11);
        }
    }
    if p.tpr13() & 4 != 0 {
        post::dfs_table(hw, p);
    }
    Ok(())
}

/// Bring up LPDDR5 with parameter set `p`. Returns the size in MiB; `p`
/// then also holds the size (para2) and the training results.
pub fn init<H: Hw>(hw: &mut H, p: &mut Para, fw: &PhyFirmware) -> Result<u32, Error> {
    hw.log(format_args!(
        "[dram] open LPDDR5 init: {} MHz, type {}\n",
        p.clk(),
        p.ty()
    ));
    if !p.is_lpddr5() {
        return Err(Error::Config("only LPDDR5 (dram_type 9) is supported"));
    }
    if p.tpr13() & 1 == 0 {
        return Err(Error::Config(
            "geometry scan (tpr13[0] clear) is not supported",
        ));
    }
    ccu::validate_pll_ddr_rate(p.clk())?;
    ccu::pad_calibration(hw);
    let vddq = power::set_rails(hw, p)?;
    hw.log(format_args!("[dram] VDDQ {vddq} mV\n"));
    core_init(hw, p, fw)?;
    if (p.tpr1() as i32) >= 0 && p.tpr10() & (1 << 29) != 0 {
        post::save_training_results(hw, p)?;
        if p.tpr10() & (1 << 30) == 0 {
            p.set_tpr10(p.tpr10() & !(1 << 29));
        }
    }
    let mib = if p.para2() & (1 << 31) != 0 {
        (p.para2() >> 16) & 0x7fff
    } else {
        let mib = size::size_mib(p);
        p.set_para2((p.para2() & 0xffff) | (mib << 16));
        mib
    };
    hw.log(format_args!(
        "[dram] {mib} MiB, para1 {:#x}, para2 {:#x}, tpr13 {:#x}\n",
        p.para1(),
        p.para2(),
        p.tpr13()
    ));
    size::quick_test(hw, mib.min(4096), 4096)?;
    Ok(mib)
}
