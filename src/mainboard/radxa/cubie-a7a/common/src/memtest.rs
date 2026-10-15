//! Destructive memory-test planning and execution, on the shared `memtest`
//! crate. Only explicit, validated DRAM ranges are accessed; EL3 and loaded
//! images are excluded. The platform supplies uncached accesses and write
//! completion. This module keeps the A7A plan: which ranges the handoff
//! reserves.

pub use memtest::{pass, Memory, Pattern, Plan, Region, Report};

use crate::{mem, Handoff};

/// The test plan for the DRAM bt0 handed over: all of it except EL3 (main,
/// its stacks, log and handoff), the loaded payload and a supplied DTB
/// (rounded outward to whole test words).
pub fn plan_from_handoff(h: &Handoff) -> Result<Plan, &'static str> {
    if h.dram_base != mem::DRAM_BASE as u64 || h.dram_size > 4 << 30 {
        return Err("unexpected A7A DRAM map");
    }
    let dram = Region::new(h.dram_base, h.dram_size)?;
    let el3 = Region::new(mem::EL3_BASE as u64, mem::EL3_SIZE as u64)?;
    let payload = Region::new(h.payload_addr, h.payload_size)?;
    if h.dtb_addr != 0 || h.dtb_size != 0 {
        // Preserve the supplied tree even though main currently generates
        // its handoff DTB in EL3 memory.
        if h.dtb_addr == 0 || h.dtb_size == 0 {
            return Err("invalid DTB range");
        }
        let dtb = Region::covering(h.dtb_addr, h.dtb_size).map_err(|_| "DTB overflow")?;
        Plan::new(dram, &[el3, payload, dtb])
    } else {
        Plan::new(dram, &[el3, payload])
    }
}
