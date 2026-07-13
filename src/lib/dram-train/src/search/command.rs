// SPDX-License-Identifier: GPL-2.0-only
//! Original synchronous acquisition transaction over a separately reviewed port.
//!
//! Static A733 correspondence: parameter write, opaque settling operation,
//! launch, at most 25,000 halfword status reads, launch clear. This does not
//! establish CPU access prerequisites, a physical time unit, done/error bits,
//! current-command freshness, or per-bit measurement polarity. A nonzero word
//! is retained as unclassified data and cannot become a training Measurement.
//! No MMIO port is provided. Asynchronous launch is deliberately unsupported.

use crate::search::BackendError;

pub const MAX_STATUS_READS: u16 = 25_000;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Plan {
    /// Opaque parameter word; no physical range or units are inferred.
    pub parameter: u16,
    pub status_reads: u16,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Unclassified {
    /// Full nonzero halfword, including unknown done/error/reserved fields.
    pub raw_status: u16,
    pub reads: u16,
    /// Diagnostic history only: observing zero then nonzero is not proof of
    /// current-command freshness or valid feedback generation.
    pub preceding_zero_reads: u16,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Cause {
    InvalidPlan,
    Quarantined,
    Port(BackendError),
    Timeout,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Failure {
    pub cause: Cause,
    pub clear_attempted: bool,
    pub clear_error: Option<BackendError>,
}

/// The implementation must establish exclusive idle ownership and the exact
/// reviewed CPU/PHY access contract before allowing mutation. Every method
/// must itself be bounded. `settle_parameter` preserves an opaque acquisition
/// interface operation; neither six microseconds nor a CPU timer is inferred.
/// No default addresses, bus width, selector, barriers or readiness are supplied.
/// Successful clear means the port accepted that operation; it does not prove
/// analog reset, status acknowledgement, result freshness or stage rollback.
pub trait Port {
    fn preflight(&self, plan: Plan) -> Result<(), BackendError>;
    fn write_parameter(&mut self, value: u16) -> Result<(), BackendError>;
    fn settle_parameter(&mut self) -> Result<(), BackendError>;
    fn set_launch(&mut self, active: bool) -> Result<(), BackendError>;
    fn read_status(&mut self) -> Result<u16, BackendError>;
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

    pub const fn quarantined(&self) -> bool {
        self.quarantined
    }

    pub const fn port(&self) -> &P {
        &self.port
    }

    pub fn observe(&mut self, plan: Plan) -> Result<Unclassified, Failure> {
        let before_launch = |cause| Failure {
            cause,
            clear_attempted: false,
            clear_error: None,
        };
        if self.quarantined {
            return Err(before_launch(Cause::Quarantined));
        }
        if plan.status_reads == 0 || plan.status_reads > MAX_STATUS_READS {
            return Err(before_launch(Cause::InvalidPlan));
        }
        self.port
            .preflight(plan)
            .map_err(|e| before_launch(Cause::Port(e)))?;
        self.port
            .write_parameter(plan.parameter)
            .map_err(|e| before_launch(Cause::Port(e)))?;
        self.port
            .settle_parameter()
            .map_err(|e| before_launch(Cause::Port(e)))?;

        // A failed launch write might nevertheless have reached the device.
        // Every path after attempting it therefore attempts exactly one clear.
        let observation = (|| {
            self.port.set_launch(true).map_err(Cause::Port)?;
            for reads in 1..=plan.status_reads {
                let raw_status = self.port.read_status().map_err(Cause::Port)?;
                if raw_status != 0 {
                    return Ok(Unclassified {
                        raw_status,
                        reads,
                        preceding_zero_reads: reads - 1,
                    });
                }
            }
            Err(Cause::Timeout)
        })();
        let clear_error = self.port.set_launch(false).err();
        self.quarantined = clear_error.is_some();
        match (observation, clear_error) {
            (Ok(raw), None) => Ok(raw),
            (Err(cause), clear_error) => Err(Failure {
                cause,
                clear_attempted: true,
                clear_error,
            }),
            (Ok(_), Some(error)) => Err(Failure {
                cause: Cause::Port(error),
                clear_attempted: true,
                clear_error: Some(error),
            }),
        }
    }
}

/// Production access stays unsupported until a real, reviewed ownership and
/// programming contract exists. No method touches hardware.
impl Port for crate::search::Unbound {
    fn preflight(&self, _: Plan) -> Result<(), BackendError> {
        Err(BackendError::Unsupported)
    }
    fn write_parameter(&mut self, _: u16) -> Result<(), BackendError> {
        Err(BackendError::Unsupported)
    }
    fn settle_parameter(&mut self) -> Result<(), BackendError> {
        Err(BackendError::Unsupported)
    }
    fn set_launch(&mut self, _: bool) -> Result<(), BackendError> {
        Err(BackendError::Unsupported)
    }
    fn read_status(&mut self) -> Result<u16, BackendError> {
        Err(BackendError::Unsupported)
    }
}
