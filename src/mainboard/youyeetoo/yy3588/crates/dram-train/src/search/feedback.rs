// SPDX-License-Identifier: GPL-2.0-only
//! Checked, original planning for the statically observed acquisition feedback
//! selector. This is not a physical measurement backend or a CPU bus address.
//!
//! The retained A733 producer composes index<<12 and feedback slot<<8; unlike
//! its setting writes, it does not include the separate setting-bank selector.
//! The active-index mask is eight bits. Ordinary loops select records 0..7 or
//! 0..8; the special loop selects logical record 9 but reads feedback slot 8.
//! These are numeric software roles, not identified DQ/DM/DQS/channel/rank pins.
//! The caller-supplied mask does not prove silicon instantiation or CPU access.
//! No raw word is converted into a training Measurement, mask, pass or freshness.

use crate::search::BackendError;

/// Eight active-mask positions times nine distinct feedback slots.
pub const MAX_ROUTES: usize = 72;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Records {
    Eight,
    Nine,
    SpecialNine,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RoutingError {
    InactiveIndex,
    RecordOutsideMode,
    Empty,
    TooMany,
    Aliased,
}

/// Constructor-checked numeric route. No public fields permit unchecked bank
/// bits, oversized indices, or silently treating logical record 9 as slot 9.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Route {
    index: u8,
    logical_record: u8,
    slot: u8,
}
impl Route {
    pub fn new(
        active_mask: u8,
        index: u8,
        records: Records,
        logical_record: u8,
    ) -> Result<Self, RoutingError> {
        if index >= 8 || active_mask & (1 << index) == 0 {
            return Err(RoutingError::InactiveIndex);
        }
        let slot = match records {
            Records::Eight if logical_record < 8 => logical_record,
            Records::Nine if logical_record < 9 => logical_record,
            Records::SpecialNine if logical_record == 9 => 8,
            _ => return Err(RoutingError::RecordOutsideMode),
        };
        Ok(Self {
            index,
            logical_record,
            slot,
        })
    }
    pub const fn index(self) -> u8 {
        self.index
    }
    pub const fn logical_record(self) -> u8 {
        self.logical_record
    }
    pub const fn slot(self) -> u8 {
        self.slot
    }
    /// Selector fields only. Deliberately no CSR base, CPU address, or setting
    /// bank input; these bits alone authorize no access.
    pub const fn selector_bits(self) -> u16 {
        ((self.index as u16) << 12) | ((self.slot as u16) << 8)
    }
}

/// Distinct observed selectors, bounded before any port interaction. Mixing
/// ordinary record 8 and special logical record 9 at the same index aliases
/// feedback slot 8 and is rejected even though the logical labels differ.
pub struct Plan<'a> {
    routes: &'a [Route],
}
impl<'a> Plan<'a> {
    pub fn new(routes: &'a [Route]) -> Result<Self, RoutingError> {
        if routes.is_empty() {
            return Err(RoutingError::Empty);
        }
        if routes.len() > MAX_ROUTES {
            return Err(RoutingError::TooMany);
        }
        for (i, route) in routes.iter().enumerate() {
            if routes[..i]
                .iter()
                .any(|prior| prior.selector_bits() == route.selector_bits())
            {
                return Err(RoutingError::Aliased);
            }
        }
        Ok(Self { routes })
    }
    pub const fn routes(&self) -> &[Route] {
        self.routes
    }
}

/// Even zero/nonzero classification does not establish analog pass polarity,
/// field validity, command generation, or current-setting freshness.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Unclassified {
    pub route: Route,
    pub raw_word: u16,
}
impl Unclassified {
    pub const fn is_zero(self) -> bool {
        self.raw_word == 0
    }
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Failure {
    Port(BackendError),
}

/// A real port must separately prove topology, exclusive ownership, access
/// width/order/barriers and safe prerequisites. Preflight must not mutate.
/// Each callback must terminate; no command issue or freshness is inferred.
pub trait Port {
    fn preflight(&self, plan: &Plan<'_>) -> Result<(), BackendError>;
    fn read_raw(&mut self, route: Route) -> Result<u16, BackendError>;
}

/// Collect at most 72 raw halfwords after one preflight. Fixed-capacity output
/// is fully invalidated on every failure, including a fault
/// after a successful prefix. This does not promise a simultaneous snapshot.
pub fn observe<P: Port>(
    port: &mut P,
    plan: &Plan<'_>,
    output: &mut [Option<Unclassified>; MAX_ROUTES],
) -> Result<(), Failure> {
    output.fill(None);
    port.preflight(plan).map_err(Failure::Port)?;
    for (route, result) in plan.routes.iter().zip(output.iter_mut()) {
        match port.read_raw(*route) {
            Ok(raw_word) => {
                *result = Some(Unclassified {
                    route: *route,
                    raw_word,
                })
            }
            Err(error) => {
                output.fill(None);
                return Err(Failure::Port(error));
            }
        }
    }
    Ok(())
}

impl Port for crate::search::Unbound {
    fn preflight(&self, _: &Plan<'_>) -> Result<(), BackendError> {
        Err(BackendError::Unsupported)
    }
    fn read_raw(&mut self, _: Route) -> Result<u16, BackendError> {
        Err(BackendError::Unsupported)
    }
}
