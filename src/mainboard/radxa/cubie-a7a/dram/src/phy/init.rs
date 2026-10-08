//! PHY initialisation and training, in the PhyInit step order: C (PHY
//! configuration), D (load the training firmware's IMEM), then per pstate
//! (1, 2, 3, then 0): C for the pstate, E (DFI clock), F (DMEM and message
//! block), the CA/DQ swizzle, G (run the firmware), I (PIE per pstate);
//! finally I (the PIE sequencer image).

use crate::ctrl::{ddrc, DFIMISC, DFI_FREQUENCY_MASK, SWCTL};
use crate::hw::{Error, Hw};
use crate::para::Para;

use super::prog::{self, Ctx};
use super::{csr_access, freq, fw, msgblock, tables, DMEM, IMEM};

/// The PHY's firmware: what runs inside it, loaded as data. All of it is
/// Synopsys' (via the vendor's DRAM library): the training firmware
/// (instruction memory image and the data that follows the message block
/// in data memory) and the PIE sequencer images for two and one rank.
pub struct PhyFirmware<'a> {
    pub imem: &'a [u8],
    pub dmem: &'a [u8],
    /// PIE images: two-rank A and B, one-rank A and B.
    pub pie: [&'a [u8]; 4],
}

const IMEM_WORDS: usize = 0x8000;
const DMEM_WORDS: usize = 0x8000;

fn ctx<'a>(p: &Para, fw: &PhyFirmware<'a>, pstate: u32) -> Ctx<'a> {
    let div = (p.tpr2() >> (8 * pstate)) & 0x1f;
    Ctx {
        pstate,
        mhz: p.clk() / (div + 1),
        data_mhz: 4 * p.clk() / (div + 1),
        dual_rank: p.dual_rank(),
        slow_bin: p.tpr10() & (1 << 31) != 0,
        dx_odt: p.dx_odt(),
        dx_dri: p.dx_dri(),
        ca_dri: p.ca_dri(),
        pie: fw.pie,
    }
}

/// Step D: the training firmware's instruction memory.
pub fn load_imem<H: Hw>(hw: &mut H, fw: &PhyFirmware) {
    let n = fw.imem.len() / 2;
    for k in 0..IMEM_WORDS {
        let v = if k < n {
            u16::from_le_bytes([fw.imem[2 * k], fw.imem[2 * k + 1]])
        } else {
            0
        };
        hw.write16(IMEM + 2 * k, v);
    }
}

/// Step E: the controller's DFI frequency (pstate) and DRAM0_CLK's divider
/// for `pstate`.
pub fn set_dfi_clk<H: Hw>(hw: &mut H, p: &Para, pstate: u32) {
    for ch in 0..2 {
        hw.write32(ddrc(ch, SWCTL), 0);
        hw.clrset32(ddrc(ch, DFIMISC), DFI_FREQUENCY_MASK, pstate << 8);
        hw.write32(ddrc(ch, SWCTL), 1);
    }
    const DRAM0_CLK: usize = 0x0200_2c00;
    hw.clr32(DRAM0_CLK, 7 << 24);
    let div = if p.tpr13() & 4 != 0 {
        (p.tpr2() >> (8 * pstate)) & 0x1f
    } else {
        3
    };
    let v = (hw.read32(DRAM0_CLK) & !0x1f) | div;
    hw.write32(DRAM0_CLK, v);
    hw.write32(DRAM0_CLK, v | (1 << 27));
    hw.udelay(1000);
}

/// Step F: the message block and the firmware's data image.
pub fn load_dmem<H: Hw>(hw: &mut H, p: &mut Para, fw: &PhyFirmware, pstate: u32, rl: u32) {
    let c = ctx(p, fw, pstate);
    let mb = msgblock::build(p, pstate, c.data_mhz, rl);
    csr_access(hw, true);
    hw.udelay(1);
    let data = fw.dmem.len() / 2;
    for k in 0..DMEM_WORDS {
        let v = if k < msgblock::WORDS {
            mb[k]
        } else if k - msgblock::WORDS < data {
            let i = 2 * (k - msgblock::WORDS);
            u16::from_le_bytes([fw.dmem[i], fw.dmem[i + 1]])
        } else {
            0
        };
        hw.write16(DMEM + 2 * k, v);
    }
    csr_access(hw, false);
}

/// Existing supported native path policy, checked before any clock, rail,
/// controller or PHY mutation. This is not a fitted-part or stable-rate claim.
pub fn validate(p: &Para) -> Result<(), Error> {
    if p.tpr10() & (1 << 29) == 0 {
        return Err(Error::Config(
            "only the trained (tpr10[29]) PHY path is supported",
        ));
    }
    if p.tpr13() & 4 == 0 {
        return Err(Error::Config(
            "only the four-pstate (tpr13[2]) set-up is supported",
        ));
    }
    for ps in 0..4 {
        let div = (p.tpr2() >> (8 * ps)) & 0x1f;
        let mhz = p.clk() / (div + 1);
        if !(100..=800).contains(&mhz) {
            return Err(Error::Config("pstate clock outside 100..800 MHz"));
        }
    }
    Ok(())
}

/// Steps C..I for every pstate and the PIE image: train the PHY.
pub fn init<H: Hw>(hw: &mut H, p: &mut Para, fw: &PhyFirmware, rl: u32) -> Result<(), Error> {
    validate(p)?;
    let c0 = ctx(p, fw, 0);
    prog::run(hw, tables::C_ONCE, &c0, freq::config);
    load_imem(hw, fw);
    for ps in [1, 2, 3, 0] {
        let c = ctx(p, fw, ps);
        prog::run(hw, tables::C_PSTATE, &c, freq::config);
        set_dfi_clk(hw, p, ps);
        load_dmem(hw, p, fw, ps, rl);
        csr_access(hw, true);
        prog::run(hw, tables::CA_REMAP, &c, freq::config);
        prog::run(hw, tables::DQ_REMAP, &c, freq::config);
        csr_access(hw, false);
        let mail = fw::run(hw, ps as u8)?;
        hw.log(format_args!(
            "[dram] pstate {ps}: training at {} MHz: message {mail:#x}\n",
            c.data_mhz
        ));
        if mail != fw::MSG_TRAINING_COMPLETE {
            return Err(Error::Training {
                pstate: ps as u8,
                message: mail,
            });
        }
        prog::run(hw, tables::I_PSTATE, &c, freq::pie);
    }
    let once = if p.dual_rank() {
        tables::I_ONCE
    } else {
        tables::I_ONCE_1RANK
    };
    prog::run(hw, once, &c0, freq::config);
    Ok(())
}
