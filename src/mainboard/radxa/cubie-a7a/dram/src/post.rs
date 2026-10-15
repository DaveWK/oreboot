//! Controller set-up after PHY training: the DFI handshake, the training
//! results folded into the pstate 0 timings, the AXI/NSI port set-up and the
//! training-result save.

use crate::ctrl::{ddrc, sw_done, DDRC_208, DDRC_B84, DDRC_B88, DFIMISC, MAP, SWCTL, SWSTAT};
use crate::hw::{wait, Error, Hw};
use crate::para::Para;
use crate::phy;
use dwc_ddrctl::dfi::{self, Cdd, DfiError};

const T: u64 = 100_000;

/// DFI initialisation of all channels (`dwc_ddrctl::dfi::dfi_init`): start
/// it, wait for the PHY to report dfi_init_complete, release the PHY's
/// R_PRCM isolation (+0x250 [0]), leave the controller's self-refresh and
/// wait for normal operation.
pub fn dfi_init<H: Hw>(hw: &mut H, channels: usize) -> Result<(), Error> {
    dfi::dfi_init(hw, &MAP, channels, 2, T, |hw| hw.set32(0x0701_0250, 1)).map_err(|e| match e {
        DfiError::Config(m) => Error::Config(m),
        DfiError::Timeout(m) => Error::Timeout(m),
    })
}

/// Larger of the two bytes of the message-block halfword holding byte
/// offsets `a` and `b`... (bytes of the training firmware's message block).
fn mb_max<H: Hw>(hw: &mut H, a: usize, b: usize) -> u32 {
    phy::mb_byte(hw, a).max(phy::mb_byte(hw, b))
}

/// Fold the trained critical delay differences (CDD) between ranks from the
/// PHY firmware's message block into channel `ch`'s pstate 0 turnaround
/// timings, and set the DFI read-data enable from the trained PHY latency.
pub fn timing_after_phy<H: Hw>(hw: &mut H, ch: usize) -> Result<(), Error> {
    phy::dmem_access(hw, true);
    // Read-to-read, write-to-write, read-to-write and write-to-read CDDs,
    // each the worst case over the rank/channel combinations.
    let cdd = Cdd {
        rr: mb_max(hw, 0x30, 0x31).max(mb_max(hw, 0x4b, 0x4c)),
        rr_alt: mb_max(hw, 0x26, 0x27).max(mb_max(hw, 0x41, 0x42)),
        ww: mb_max(hw, 0x29, 0x2a).max(mb_max(hw, 0x44, 0x45)),
        ww_alt: mb_max(hw, 0x2d, 0x2e).max(mb_max(hw, 0x48, 0x49)),
        rw: mb_max(hw, 0x28, 0x2b).max(mb_max(hw, 0x43, 0x46)),
        wr: mb_max(hw, 0x2c, 0x2f).max(mb_max(hw, 0x47, 0x4a)),
    };
    phy::dmem_access(hw, false);

    hw.set32(ddrc(ch, DDRC_B84), 1);
    // The library polls channel 0's acknowledge for both channels.
    wait(hw, "DDRC+0xb88 ack [26:25]", T, |hw| {
        (hw.read32(ddrc(0, DDRC_B88)) >> 25) & 3 == 3
    })?;
    wait(hw, "DDRC+0xb88 ack [29:28]", T, |hw| {
        (hw.read32(ddrc(0, DDRC_B88)) >> 28) & 3 == 3
    })?;
    hw.write32(ddrc(ch, SWCTL), 0);
    wait(hw, "SWSTAT.sw_done_ack clear", T, |hw| {
        hw.read32(ddrc(ch, SWSTAT)) & 1 == 0
    })?;
    dfi::fold_cdd(hw, &MAP, ch, &cdd);
    let (ptr_init, rcv) = phy::read_latency_csrs(hw);
    let rden = ((rcv >> 1) & 0xf) + (((ptr_init & 0xf) + 1) >> 3) * 4 + 21;
    dfi::set_rddata_en(hw, &MAP, ch, rden);
    sw_done(hw, ch)?;
    hw.clr32(ddrc(ch, DDRC_B84), 1);
    Ok(())
}

/// After the DFI handshake: DFIMISC [14] (DFI frequency-change handshake)
/// on every channel, then the AXI ports of the two CPU clusters and the NSI
/// QoS for one or two channels.
pub fn controller_after_phy<H: Hw>(hw: &mut H, p: &Para) -> Result<(), Error> {
    let channels = p.channels();
    for ch in 0..channels {
        hw.write32(ddrc(ch, SWCTL), 0);
    }
    for ch in 0..channels {
        hw.write32(ddrc(ch, DDRC_208), 0);
    }
    for ch in 0..channels {
        hw.clrset32(ddrc(ch, DFIMISC), 0xc000, 0x4000);
    }
    for ch in 0..channels {
        hw.write32(ddrc(ch, SWCTL), 1);
    }
    for ch in 0..channels {
        wait(hw, "SWSTAT.sw_done_ack", T, |hw| {
            hw.read32(ddrc(ch, SWSTAT)) & 1 != 0
        })?;
    }
    if channels == 2 {
        hw.write32(0x0202_4818, 0x8002);
        hw.write32(0x0202_481c, 0xffff_ffff);
        hw.write32(0x0a03_0018, 0x8002);
        hw.write32(0x0a04_0018, 0x8002);
        hw.write32(0x0a03_0068, 0xffff_ffff);
        hw.write32(0x0a04_0068, 0xffff_ffff);
        hw.write32(0x0202_4838, 2);
        hw.write32(0x0202_448c, 2);
        hw.write32(0x0202_468c, 2);
        for off in [0x000, 0x200, 0x400, 0x600, 0x800] {
            hw.write32(0x0202_3a04 + off, 0xffff_ffff);
        }
        hw.write32(0x0202_4490, 0x1010_2010);
        hw.write32(0x0202_4494, 0x0808_1008);
        hw.write32(0x0202_4690, 0x1010_2010);
        hw.write32(0x0202_4694, 0x0808_1008);
    } else {
        hw.write32(0x0202_44a8, 1);
        hw.write32(0x0202_46a8, 1);
        hw.write32(0x0a03_0008, 3);
        hw.write32(0x0a04_0008, 3);
    }
    Ok(())
}

/// Result-save failure, including each independently attempted restoration.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SaveFailure {
    /// Initial operation failure, before restoration (if one occurred).
    pub primary: Option<Error>,
    /// Restoration failure per channel/rank: ch0r0,ch0r1,ch1r0,ch1r1.
    pub restore: [Option<Error>; 4],
    /// A failed command may nevertheless have reached the DRAM.
    pub attempted_ranks: u8,
}
impl SaveFailure {
    pub fn recovery_failed(&self) -> bool {
        self.restore.iter().any(Option::is_some)
    }
}

/// Store per-lane VrefDAC0/1 (DBYTE +0x4e/+0x4f, nine lanes) in words 32..49 and
/// each rank's MR14/MR15 in words 50/51, preserving the inherited ABI.
/// Stage results until every operation and restoration succeeds. On any
/// error, invalidate the output, attempt MR16=4 on each possibly changed rank,
/// and return CSR ownership through the existing baseline close operation.
/// A successful restore command is not proof of physical state recovery.
// no_std without an allocator: the failure record cannot be boxed, and it
// stays on the caller's stack only on the error path.
#[allow(clippy::result_large_err)]
pub fn save_training_results<H: Hw>(hw: &mut H, p: &mut Para) -> Result<(), SaveFailure> {
    p.w[32..52].fill(0);
    let mut staged = [0u32; 20];
    let mut attempted_ranks = 0u8;
    phy::csr_access(hw, true);
    let primary = (|| -> Result<(), Error> {
        for lane in 0..9 {
            for (half, word) in [(0, 0), (2, 9)] {
                let mut v = 0;
                for db in 0..4 {
                    let a = phy::dbyte_vref_dac(db, lane) + half;
                    v |= (hw.read16(a) as u32 & 0x7f) << (8 * db);
                }
                staged[word + lane as usize] = v;
            }
        }
        for (index, (ch, rank)) in [(0, 0), (0, 1), (1, 0), (1, 1)].into_iter().enumerate() {
            attempted_ranks |= 1 << index;
            crate::mr::write_rank(hw, ch, rank, 16, 5)?;
        }
        for (mr, word) in [(14, 18), (15, 19)] {
            let mut v = 0;
            for (i, (ch, rank)) in [(0, 0), (0, 1), (1, 0), (1, 1)].into_iter().enumerate() {
                v |= crate::mr::read(hw, ch, rank, mr)? << (8 * i);
            }
            staged[word] = v;
        }
        Ok(())
    })()
    .err();
    let mut restore = [None; 4];
    for (index, (ch, rank)) in [(0, 0), (0, 1), (1, 0), (1, 1)].into_iter().enumerate() {
        if attempted_ranks & (1 << index) != 0 {
            // Continue best-effort restoration after an earlier restore failure.
            restore[index] = crate::mr::write_rank(hw, ch, rank, 16, 4).err();
        }
    }
    phy::csr_access(hw, false);
    if primary.is_some() || restore.iter().any(Option::is_some) {
        return Err(SaveFailure {
            primary,
            restore,
            attempted_ranks,
        });
    }
    p.w[32..52].copy_from_slice(&staged);
    Ok(())
}

/// MCTL_COM's hardware DFS sequencer program (0x0a021800..0x0a021d3f):
/// the command list the controller block replays to change the DRAM
/// frequency (self-refresh entry, clock switch, MR writes, exit). Nothing
/// starts it during boot; it is written as the vendor init leaves it so a
/// later frequency change (SCP firmware) finds it. The clock words of the
/// four pstates hold each pstate's PLL divider ([23:16] = 8 x tpr2 divider).
pub fn dfs_table<H: Hw>(hw: &mut H, p: &Para) {
    for &(addr, val) in phy::tables::DFS_TABLE.iter() {
        let v = match addr {
            0x0a02_1d30..=0x0a02_1d3c => {
                let ps = (addr - 0x0a02_1d30) / 4;
                let div = (p.tpr2() >> (8 * ps)) & 0x1f;
                (val & !0x00ff_0000) | (div * 8) << 16
            }
            _ => val,
        };
        hw.write32(addr as usize, v);
    }
}
