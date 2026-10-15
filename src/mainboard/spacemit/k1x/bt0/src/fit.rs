//! Load a U-Boot style FIT (external data, `mkimage -E`) from an eMMC hardware
//! partition or raw SD card sectors, and start OpenSBI's fw_dynamic firmware
//! with the FIT's next stage and DT.

use crate::fit_checked;
use crate::mmc::{Error as MmcError, Kind, Mmc, Partition};
use core::arch::asm;
use core::ptr::{addr_of, copy_nonoverlapping};
const SECTOR: usize = 512;
const NOR_MMIO_BASE: usize = 0xb800_0000;
#[derive(Debug)]
pub enum Error {
    Mmc(MmcError),
    Fit(fit_checked::Error),
    Gpt(storage::gpt::GptError),
}
impl From<MmcError> for Error {
    fn from(e: MmcError) -> Self {
        Self::Mmc(e)
    }
}
impl From<fit_checked::Error> for Error {
    fn from(e: fit_checked::Error) -> Self {
        Self::Fit(e)
    }
}
impl From<fdt_checked::Error> for Error {
    fn from(e: fdt_checked::Error) -> Self {
        Self::Fit(e.into())
    }
}
impl From<storage::gpt::GptError> for Error {
    fn from(e: storage::gpt::GptError) -> Self {
        Self::Gpt(e)
    }
}
pub struct Boot {
    pub firmware: usize,
    pub next: usize,
    pub fdt: usize,
    pub fdt_size: usize,
}
pub enum Medium {
    Mmc(Kind, Partition),
    Nor,
}
pub struct Source {
    pub medium: Medium,
    pub lba: u32,
    pub gpt_name: Option<&'static str>,
}
enum Device {
    // The GPT reader reads through &self; the card's commands need &mut.
    Mmc(core::cell::RefCell<Mmc>),
    Nor,
}
struct Reader {
    device: Device,
    capacity: u64,
}
impl Reader {
    fn read(&self, lba: u32, count: usize, dest: usize) -> Result<(), Error> {
        fit_checked::read_extent(0, self.capacity, lba as u64, count)?;
        let bytes = count
            .checked_mul(SECTOR)
            .ok_or(fit_checked::Error::Bounds)?;
        dest.checked_add(bytes).ok_or(fit_checked::Error::Bounds)?;
        match &self.device {
            Device::Mmc(m) => Ok(m.borrow_mut().read(lba, count, dest)?),
            Device::Nor => {
                let offset = (lba as usize)
                    .checked_mul(SECTOR)
                    .ok_or(fit_checked::Error::Bounds)?;
                let src = NOR_MMIO_BASE
                    .checked_add(offset)
                    .ok_or(fit_checked::Error::Bounds)?;
                unsafe { copy_nonoverlapping(src as *const u8, dest as *mut u8, bytes) };
                Ok(())
            }
        }
    }
    fn restore_user(&self) -> Result<(), Error> {
        match &self.device {
            Device::Mmc(m) => Ok(m.borrow_mut().select(Partition::User)?),
            Device::Nor => Ok(()),
        }
    }
}
impl storage::gpt::SectorReader for Reader {
    fn read_sector(&self, lba: u32, out: &mut [u8; 512]) -> Result<(), storage::gpt::GptError> {
        self.read(lba, 1, out.as_mut_ptr() as usize)
            .map_err(|_| storage::gpt::GptError::Read)
    }
}
pub fn load(src: &Source, staging: usize) -> Result<Boot, Error> {
    if staging != 0x1000_0000 {
        return Err(fit_checked::Error::Destination.into());
    }
    let r = match src.medium {
        Medium::Mmc(kind, partition) => {
            let mut m = crate::mmc::init(kind)?;
            m.select(partition)?;
            let capacity = match m.capacity_sectors(partition) {
                Ok(c) => c,
                Err(e) => {
                    let _ = m.select(Partition::User);
                    return Err(e.into());
                }
            };
            Reader {
                device: Device::Mmc(core::cell::RefCell::new(m)),
                capacity,
            }
        }
        Medium::Nor => Reader {
            device: Device::Nor,
            capacity: fit_checked::NOR_BYTES / SECTOR as u64,
        },
    };
    let result = load_from(&r, src, staging);
    // Restore eMMC partition selection on every path, including malformed FITs.
    let restored = r.restore_user();
    match result {
        Err(e) => Err(e),
        Ok(b) => {
            restored?;
            Ok(b)
        }
    }
}
fn load_from(r: &Reader, src: &Source, staging: usize) -> Result<Boot, Error> {
    let found = match src.gpt_name {
        Some(name) => storage::gpt::find(r, name, r.capacity)?,
        None => None,
    };
    let (first_lba, sectors) =
        fit_checked::source_extent(found.map(|e| (e.first_lba, e.sectors)), r.capacity, src.lba)?;
    let extent = storage::gpt::PartitionExtent { first_lba, sectors };
    let cap = extent
        .sectors
        .checked_mul(SECTOR as u64)
        .ok_or(fit_checked::Error::Bounds)?
        .min(fit_checked::MAX_FIT as u64) as usize;
    let read = |count: usize| -> Result<(), Error> {
        let (lba, count) = fit_checked::read_extent(extent.first_lba, extent.sectors, 0, count)?;
        Ok(r.read(lba, count, staging)?)
    };
    read(1)?;
    let first = unsafe { core::slice::from_raw_parts(staging as *const u8, SECTOR) };
    let header = fdt_checked::Header::parse(first, cap.min(fit_checked::MAX_METADATA))?;
    read(header.total.div_ceil(SECTOR))?;
    let metadata = unsafe { core::slice::from_raw_parts(staging as *const u8, header.total) };
    let plan = fit_checked::plan(
        metadata,
        fit_checked::Region {
            start: staging,
            size: fit_checked::MAX_FIT,
        },
    )?;
    let metadata_hash = sha256::digest(metadata);
    if plan.end > cap {
        return Err(fit_checked::Error::Bounds.into());
    }
    println!(
        "[bt0] reading checked FIT at sector {} ({} bytes)",
        extent.first_lba, plan.end
    );
    read(plan.end.div_ceil(SECTOR))?;
    let bytes = unsafe { core::slice::from_raw_parts(staging as *const u8, plan.end) };
    if sha256::digest(&bytes[..plan.metadata]) != metadata_hash {
        return Err(fit_checked::Error::Hash.into());
    }
    plan.verify(bytes)?;
    // Only the validated production layout can reach these destinations. No
    // destination is touched until every image hash and the nested DT pass.
    for image in plan.images {
        unsafe {
            copy_to_physical(
                staging + image.data.start,
                image.destination.start,
                image.data.size,
            )
        };
    }
    Ok(Boot {
        firmware: plan.images[0].destination.start,
        next: plan.images[1].destination.start,
        fdt: plan.images[2].destination.start,
        fdt_size: plan.images[2].data.size,
    })
}
pub fn fix_memory(fdt: usize, length: usize, size: u64) -> Result<(), Error> {
    let bytes = unsafe { core::slice::from_raw_parts_mut(fdt as *mut u8, length) };
    Ok(fit_checked::fix_memory(bytes, size)?)
}

// The OpenSBI fw_dynamic hand-off and the copy to physical address 0 are the
// shared opensbi_handoff crate (ore-edk2-boot). The info block must outlive
// the jump: OpenSBI reads it after bt0 has gone.
use opensbi_handoff::{copy_to_physical, FwDynamicInfo, NextMode};

static mut FW_DYNAMIC_INFO: FwDynamicInfo = FwDynamicInfo::new(0, NextMode::Supervisor, 0);

pub fn start_opensbi(hart: usize, b: &Boot) -> ! {
    unsafe {
        FW_DYNAMIC_INFO = FwDynamicInfo::new(b.next, NextMode::Supervisor, hart);
        opensbi_handoff::start(b.firmware, hart, b.fdt, addr_of!(FW_DYNAMIC_INFO))
    }
}
