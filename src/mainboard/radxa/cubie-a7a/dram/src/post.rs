//! Controller set-up after PHY training: the DFI handshake, the training
//! results folded into the pstate 0 timings, the AXI/NSI port set-up and the
//! training-result save.

use crate::ctrl::{
    ddrc, freq, sw_done, DDRC_208, DDRC_B84, DDRC_B88, DFIMISC, DFISTAT, PWRCTL, STAT, SWCTL,
    SWSTAT,
};
use crate::hw::{wait, Error, Hw};
use crate::para::Para;
use crate::phy;

const T: u64 = 100_000;

/// DFI initialisation of all channels: start it, wait for the PHY to report
/// dfi_init_complete, release the PHY's R_PRCM isolation, leave the
/// controller's self-refresh and wait for normal operation.
pub fn dfi_init<H: Hw>(hw: &mut H, channels: usize) -> Result<(), Error> {
    if !(1..=2).contains(&channels) {
        return Err(Error::Config(
            "DFI channel count outside controller instances",
        ));
    }
    for ch in 0..channels {
        hw.write32(ddrc(ch, SWCTL), 0);
    }
    for ch in 0..channels {
        hw.set32(ddrc(ch, DFIMISC), 0x1001);
    }
    for ch in 0..channels {
        hw.set32(ddrc(ch, DFIMISC), crate::ctrl::DFI_INIT_START);
    }
    for ch in 0..channels {
        hw.write32(ddrc(ch, SWCTL), 1);
    }
    for ch in 0..channels {
        wait(hw, "SWSTAT.sw_done_ack", T, |hw| {
            hw.read32(ddrc(ch, SWSTAT)) & 1 != 0
        })?;
    }
    for ch in 0..channels {
        wait(hw, "DFISTAT.dfi_init_complete", T, |hw| {
            hw.read32(ddrc(ch, DFISTAT)) & 1 != 0
        })?;
    }
    hw.udelay(50);
    hw.set32(0x0701_0250, 1);
    hw.udelay(2000);
    for ch in 0..channels {
        hw.write32(ddrc(ch, SWCTL), 0);
    }
    for ch in 0..channels {
        hw.clr32(ddrc(ch, DFIMISC), crate::ctrl::DFI_INIT_START);
    }
    for ch in 0..channels {
        hw.clr32(ddrc(ch, PWRCTL), 0x800);
    }
    for ch in 0..channels {
        // STAT: selfref_type [5:4] back to 0, then operating_mode normal.
        wait(hw, "STAT.selfref_type", T, |hw| {
            hw.read32(ddrc(ch, STAT)) & 0x30 == 0
        })?;
        wait(hw, "STAT.operating_mode", T, |hw| {
            hw.read32(ddrc(ch, STAT)) & 3 == 1
        })?;
    }
    hw.udelay(50);
    for ch in 0..channels {
        hw.clr32(ddrc(ch, DFIMISC), crate::ctrl::DFI_INIT_COMPLETE_EN);
    }
    for ch in 0..channels {
        hw.write32(ddrc(ch, SWCTL), 1);
    }
    for ch in 0..channels {
        wait(hw, "SWSTAT.sw_done_ack", T, |hw| {
            hw.read32(ddrc(ch, SWSTAT)) & 1 != 0
        })?;
    }
    hw.udelay(2);
    Ok(())
}

/// Larger of the two bytes of the message-block halfword holding byte
/// offsets `a` and `b`... (bytes of the training firmware's message block).
fn mb_max<H: Hw>(hw: &mut H, a: usize, b: usize) -> u32 {
    phy::mb_byte(hw, a).max(phy::mb_byte(hw, b))
}

fn sat_add(field: u32, add: u32, max: u32) -> u32 {
    (field + add).min(max)
}

/// Fold the trained critical delay differences (CDD) between ranks from the
/// PHY firmware's message block into channel `ch`'s pstate 0 turnaround
/// timings, and set the DFI read-data enable from the trained PHY latency.
pub fn timing_after_phy<H: Hw>(hw: &mut H, ch: usize) -> Result<(), Error> {
    phy::dmem_access(hw, true);
    // Read-to-read, write-to-write, read-to-write and write-to-read CDDs,
    // each the worst case over the rank/channel combinations.
    let rr = mb_max(hw, 0x30, 0x31).max(mb_max(hw, 0x4b, 0x4c));
    let rr_alt = mb_max(hw, 0x26, 0x27).max(mb_max(hw, 0x41, 0x42));
    let ww = mb_max(hw, 0x29, 0x2a).max(mb_max(hw, 0x44, 0x45));
    let ww_alt = mb_max(hw, 0x2d, 0x2e).max(mb_max(hw, 0x48, 0x49));
    let rw = mb_max(hw, 0x28, 0x2b).max(mb_max(hw, 0x43, 0x46));
    let wr = mb_max(hw, 0x2c, 0x2f).max(mb_max(hw, 0x47, 0x4a));
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
    let f = |off| freq(ch, 0, off);
    let pair = |v: u32, hi: u32, lo: u32| {
        (v & 0xffff_0000) | (sat_add((v >> 8) & 0xff, hi, 0xff) << 8) | sat_add(v & 0xff, lo, 0xff)
    };
    hw.modify32(f(0xd04), |v| pair(v, rr, rr_alt));
    hw.modify32(f(0xd08), |v| pair(v, ww, ww_alt));
    hw.modify32(f(0x08), |v| pair(v, rw, wr));
    hw.modify32(f(0x60), |v| {
        (v & !0xff_0000) | (sat_add((v >> 16) & 0xff, ww, 0xff) << 16)
    });
    hw.modify32(f(0x24), |v| (v & !0xff) | sat_add(v & 0xff, ww_alt, 0xff));
    hw.modify32(f(0x584), |v| {
        (v & !0x1f_0000) | (sat_add((v >> 16) & 0xff, rr, 31) << 16)
    });
    let (ptr_init, rcv) = phy::read_latency_csrs(hw);
    hw.modify32(f(0x588), |v| {
        let rden = ((rcv >> 1) & 0xf) + (((ptr_init & 0xf) + 1) >> 3) * 4 + 21;
        (v & !0x3f_0000) | (rden << 16)
    });
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

/// Store per-lane delays (DBYTE +0x4e/+0x4f, nine lanes) in words 32..49 and
/// each rank's MR14/MR15 in words 50/51, preserving the inherited ABI.
/// Stage results until every operation and restoration succeeds. On any
/// error, invalidate the output, attempt MR16=4 on each possibly changed rank,
/// and return CSR ownership through the existing baseline close operation.
/// A successful restore command is not proof of physical state recovery.
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
                    let a = phy::dbyte_lane_delay(db, lane) + half;
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
