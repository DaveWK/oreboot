// SPDX-License-Identifier: GPL-2.0-only
//! USB device controller of the K1 (ChipIdea, USB2 OTG at 0xc090_0000): the
//! shared `ci_udc` driver and `usb_device` descriptors (ore-edk2-boot) with
//! the K1 platform side: APMU clock and reset, the UTMI PHY bring-up
//! (k1x_usb2_ci.h) and data cache maintenance for the controller's DMA. The
//! crate's equivalence test runs this platform against the previous K1
//! driver and requires the same register trace and results.

use crate::k1io::K1Io;
pub use ci_udc::Error;
use ci_udc::{Config, Dma, PhyTimeout, Udc, UdcPlatform};
use regio::RegIo;
pub use usb_device::Descriptors;

const USB: usize = 0xc090_0000;
const APMU_USB_CLK_RES_CTRL: usize = 0xd428_2800 + 0x5c;
const USB_AXICLK_EN: u32 = 1 << 1;
const USB_AXI_RST: u32 = 1 << 0; // reset released when set
const PHY_REG01: usize = 0xc094_0000 + 0x04;
const PHY_REG0D: usize = 0xc094_0000 + 0x34;
const PHY_PLL_READY: u32 = 1 << 0;
const PHY_POLLS: usize = 1_000_000;

/// The K1 board's side of the controller.
pub struct K1Usb;

impl UdcPlatform for K1Usb {
    fn bring_up<IO: RegIo>(&mut self, io: &mut IO) -> Result<(), PhyTimeout> {
        let v = io.read32(APMU_USB_CLK_RES_CTRL);
        io.write32(APMU_USB_CLK_RES_CTRL, v | USB_AXICLK_EN);
        io.write32(APMU_USB_CLK_RES_CTRL, (v | USB_AXICLK_EN) & !USB_AXI_RST);
        io.write32(APMU_USB_CLK_RES_CTRL, v | USB_AXICLK_EN | USB_AXI_RST);
        self.spin(200_000);
        if !(0..PHY_POLLS).any(|_| io.read32(PHY_REG01) & PHY_PLL_READY != 0) {
            return Err(PhyTimeout);
        }
        io.write32(PHY_REG01, 0x60ef);
        io.write32(PHY_REG0D, 0x1c);
        Ok(())
    }
    fn flush_dcache(&mut self, addr: usize, len: usize) {
        cache::zicbom::flush(addr, len)
    }
    fn invalidate_dcache(&mut self, addr: usize, len: usize) {
        cache::zicbom::invalidate(addr, len)
    }
}

pub type Usb<'a> = Udc<'a, K1Io, K1Usb>;

/// The controller's queue heads, descriptors and bounce buffers.
static mut DMA: Dma = Dma::new();

/// Reset the controller and connect to the host as a new device; in
/// download mode this drops the BootROM's own session. Call once.
pub fn init(desc: Descriptors<'_>) -> Result<Usb<'_>, Error> {
    // SAFETY: the one user of DMA; bt0 is single-threaded and calls this once.
    let dma = unsafe { &mut *core::ptr::addr_of_mut!(DMA) };
    Udc::init(K1Io, K1Usb, Config::new(USB), dma, desc)
}
