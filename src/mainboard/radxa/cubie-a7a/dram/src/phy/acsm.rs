//! Original LPDDR5 MRW CA/CS payload construction, without command execution.
//!
//! CA protocol: LiteDRAM 74522f715d7b10163ef6f6caa09887a90874be83,
//! `litedram/phy/lpddr5/commands.py`, MRW-1/MRW-2 truth table. Packing
//! corresponds to the A733 static record writer and the independent NXP
//! imx-oei 9aea3350e643da86081c9dfaf3a6e1a84a583bfd LPDDR5 timing sources:
//! two seven-bit CA edges and two CS bits occupy record halfwords 0 and 4.
//!
//! This constructs only those payload fields. It supplies no control words,
//! command spacing, device initialization, topology, PHY ownership, launch,
//! completion or MMIO operation. It is not an executable ACSM program or a
//! qualified physical A733 command backend.

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PayloadError {
    ModeAddress,
    ChipSelect,
}

/// The two CA/CS halfwords of a four-edge LPDDR5 MRW.
/// Other halfwords of the ACSM record require a separate reviewed contract.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct MrWritePayload {
    first: u16,
    second: u16,
}

impl MrWritePayload {
    /// Explicit CS mask 1..=3: no rank/channel mapping or broadcast policy is
    /// inferred. The caller must establish that the selected device(s), MR and
    /// operand are valid for the fitted part and current FSP.
    pub const fn new(address: u8, operand: u8, cs_mask: u8) -> Result<Self, PayloadError> {
        if address > 127 {
            return Err(PayloadError::ModeAddress);
        }
        if cs_mask == 0 || cs_mask > 3 {
            return Err(PayloadError::ChipSelect);
        }
        let cs = (cs_mask as u16) << 14;
        Ok(Self {
            first: cs | ((address as u16) << 7) | 0x58,
            second: cs | (((operand & 0x7f) as u16) << 7) | (((operand >> 7) as u16) << 6) | 0x08,
        })
    }

    /// Relative record halfword index plus CA/CS payload, not an APB address.
    /// Deliberately no conversion to a complete record or hardware write.
    pub const fn fields(self) -> [(usize, u16); 2] {
        [(0, self.first), (4, self.second)]
    }
}
