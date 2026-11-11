// SPDX-License-Identifier: GPL-2.0-only
//! Original transactional LPDDR5 DRAM Vref application. No PHY binding or scan.
//!
//! MR numbers and seven-bit values use the existing value-only LPDDR5 contract.
//! An explicit reviewed port must prove part/FSP applicability, limits, command
//! timing, exclusive ownership and fresh full-byte reads. There is no default
//! part range, FSP transition, training entry, address, timeout or delay unit.

use crate::search::lpddr5::{DramVref, VrefCode};
use crate::search::BackendError;

pub const MAX_SETTINGS: usize = 12; // Software capacity: 2 channels x 2 ranks x 3 MRs.

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Target {
    pub channel: u8,
    pub rank: u8,
    pub register: DramVref,
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Setting {
    pub target: Target,
    pub value: VrefCode,
    /// Explicit caller-supplied code limits; preflight must prove their origin
    /// and applicability. Merely being within 0..127 is not a part contract.
    pub minimum: u8,
    pub maximum: u8,
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Step {
    Snapshot,
    Write,
    Settle,
    Readback,
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Cause {
    InvalidPlan,
    Quarantined,
    Preflight(BackendError),
    Operation {
        index: u8,
        step: Step,
        error: BackendError,
    },
    Readback {
        index: u8,
        expected: u8,
        observed: u8,
    },
    SnapshotRange {
        index: u8,
        observed: u8,
    },
}
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct RestoreFailure {
    pub write: Option<BackendError>,
    pub settle: Option<BackendError>,
    pub readback: Option<BackendError>,
    pub mismatch: Option<u8>,
}
impl RestoreFailure {
    fn failed(self) -> bool {
        self.write.is_some()
            || self.settle.is_some()
            || self.readback.is_some()
            || self.mismatch.is_some()
    }
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Failure {
    pub cause: Cause,
    /// Includes a write that reported failure after possibly reaching DRAM.
    pub attempted: u16,
    pub restore: [RestoreFailure; MAX_SETTINGS],
}

/// Every operation must terminate. Preflight is read-only and must validate the
/// entire plan against reviewed topology, part, active FSP and timing evidence.
/// Reads must return the fresh full byte for the selected MR, never a cached
/// echo of the requested value. Write/read must enforce safe command ownership
/// even after an earlier operation fault. Settle must enforce the reviewed MR
/// timing for both normal and recovery commands, with no inferred duration.
/// The port must support restoration to every snapshotted byte. No physical
/// support is implied by implementing this interface in a synthetic control.
pub trait Port {
    fn preflight(&self, settings: &[Setting]) -> Result<(), BackendError>;
    fn read(&mut self, target: Target) -> Result<u8, BackendError>;
    fn write(&mut self, target: Target, value: u8) -> Result<(), BackendError>;
    fn settle(&mut self, target: Target) -> Result<(), BackendError>;
}

pub struct Transaction<P> {
    port: P,
    quarantined: bool,
}
impl<P: Port> Transaction<P> {
    pub const fn new(port: P) -> Self {
        Self {
            port,
            quarantined: false,
        }
    }
    pub const fn port(&self) -> &P {
        &self.port
    }
    pub const fn quarantined(&self) -> bool {
        self.quarantined
    }

    /// Apply at most twelve explicit settings. Snapshot the whole plan before
    /// the first write, preserve unrelated bit7, settle and freshly read back
    /// each command. Any failure restores every possibly changed target;
    /// restoration errors remain per-target and prohibit automatic retry.
    pub fn apply(&mut self, settings: &[Setting]) -> Result<(), Failure> {
        let before_write = |cause| Failure {
            cause,
            attempted: 0,
            restore: [RestoreFailure::default(); MAX_SETTINGS],
        };
        if self.quarantined {
            return Err(before_write(Cause::Quarantined));
        }
        if settings.is_empty() || settings.len() > MAX_SETTINGS {
            return Err(before_write(Cause::InvalidPlan));
        }
        for (index, setting) in settings.iter().enumerate() {
            if setting.target.channel > 1
                || setting.target.rank > 1
                || setting.minimum > setting.maximum
                || setting.maximum > 127
                || !(setting.minimum..=setting.maximum).contains(&setting.value.code())
                || settings[..index]
                    .iter()
                    .any(|old| old.target == setting.target)
            {
                return Err(before_write(Cause::InvalidPlan));
            }
        }
        self.port
            .preflight(settings)
            .map_err(|e| before_write(Cause::Preflight(e)))?;
        let mut original = [0u8; MAX_SETTINGS];
        for (index, setting) in settings.iter().enumerate() {
            original[index] = self.port.read(setting.target).map_err(|error| {
                before_write(Cause::Operation {
                    index: index as u8,
                    step: Step::Snapshot,
                    error,
                })
            })?;
            if !(setting.minimum..=setting.maximum).contains(&(original[index] & 0x7f)) {
                return Err(before_write(Cause::SnapshotRange {
                    index: index as u8,
                    observed: original[index],
                }));
            }
        }
        let mut attempted = 0u16;
        let operation = (|| {
            for (index, setting) in settings.iter().enumerate() {
                let expected = setting.value.replace_field(original[index]);
                let fail = |step, error| Cause::Operation {
                    index: index as u8,
                    step,
                    error,
                };
                attempted |= 1 << index;
                self.port
                    .write(setting.target, expected)
                    .map_err(|e| fail(Step::Write, e))?;
                self.port
                    .settle(setting.target)
                    .map_err(|e| fail(Step::Settle, e))?;
                let observed = self
                    .port
                    .read(setting.target)
                    .map_err(|e| fail(Step::Readback, e))?;
                if observed != expected {
                    return Err(Cause::Readback {
                        index: index as u8,
                        expected,
                        observed,
                    });
                }
            }
            Ok(())
        })();
        if let Err(cause) = operation {
            let mut restore = [RestoreFailure::default(); MAX_SETTINGS];
            for (index, setting) in settings.iter().enumerate() {
                if attempted & (1 << index) == 0 {
                    continue;
                }
                // An uncertain restore write is still followed by the port's
                // bounded timing/recovery operation. Each target is attempted
                // independently; no earlier fault hides a later restore.
                restore[index].write = self.port.write(setting.target, original[index]).err();
                restore[index].settle = self.port.settle(setting.target).err();
                // A failed settle cannot authorize a read before timing holds.
                if restore[index].settle.is_none() {
                    match self.port.read(setting.target) {
                        Ok(value) if value != original[index] => {
                            restore[index].mismatch = Some(value)
                        }
                        Ok(_) => {}
                        Err(error) => restore[index].readback = Some(error),
                    }
                }
            }
            self.quarantined = restore.iter().any(|error| error.failed());
            return Err(Failure {
                cause,
                attempted,
                restore,
            });
        }
        Ok(())
    }
}

impl Port for crate::search::Unbound {
    fn preflight(&self, _: &[Setting]) -> Result<(), BackendError> {
        Err(BackendError::Unsupported)
    }
    fn read(&mut self, _: Target) -> Result<u8, BackendError> {
        Err(BackendError::Unsupported)
    }
    fn write(&mut self, _: Target, _: u8) -> Result<(), BackendError> {
        Err(BackendError::Unsupported)
    }
    fn settle(&mut self, _: Target) -> Result<(), BackendError> {
        Err(BackendError::Unsupported)
    }
}
