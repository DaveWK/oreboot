//! Direct ACSM control for the register schema documented by ESWIN's
//! EIC7700X TRM Part 2, SHA256
//! 955c99f00934e5bc342e4b34b5dc3d73179d66a8395ae860877b21a6a9b8def5,
//! PDF pages 501-504 and 539-540. These register numbers correspond to the
//! A733 static access paths; that correspondence alone does not qualify the
//! fitted A733 PHY revision or its cold initialization sequence.
//!
//! No physical port is supplied. In particular, clearing MicroContMuxSel
//! does not exclude PIE or DMA, and is insufficient to implement `preflight`.
//! The original eight-halfword sequence/control format, device timings and
//! no-PMU bootstrap remain separate requirements. Completion here is an ACSM
//! control observation, never an analog training measurement.

pub const RUN_CTRL: u32 = 0x20020;
pub const DONE: u32 = 0x20021;
pub const PHY_CONFIG: u32 = 0x200a4;
pub const MAX_POLLS: u16 = 25_000;
const RUN_BIT: u16 = 1;
const DONE_BIT: u16 = 1;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ConfigurationError {
    ReservedBits,
    RankCount,
    ChannelCount,
    DbytesPerChannel,
}

/// Hardware synthesis capabilities, not the fitted memory's rank geometry,
/// configured protocol, active channel set or trained state.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Configuration {
    lpddr5_supported: bool,
    dmi_supported: bool,
    ranks: u8,
    channels: u8,
    dbytes_per_channel: u8,
}

impl Configuration {
    /// Decode only the documented EIC7700X schema. Rank/channel fields hold
    /// literal counts 1 or 2. The low nibble counts DBYTEs PER CHANNEL; it is
    /// not the total number of DBYTE instances or a lane/rank address map.
    pub const fn decode(raw: u16) -> Result<Self, ConfigurationError> {
        if raw & !0x03ff != 0 {
            return Err(ConfigurationError::ReservedBits);
        }
        let ranks = ((raw >> 6) & 3) as u8;
        let channels = ((raw >> 4) & 3) as u8;
        let dbytes_per_channel = (raw & 15) as u8;
        if ranks != 1 && ranks != 2 {
            return Err(ConfigurationError::RankCount);
        }
        if channels != 1 && channels != 2 {
            return Err(ConfigurationError::ChannelCount);
        }
        if dbytes_per_channel != 2 {
            return Err(ConfigurationError::DbytesPerChannel);
        }
        Ok(Self {
            lpddr5_supported: raw & (1 << 9) != 0,
            dmi_supported: raw & (1 << 8) != 0,
            ranks,
            channels,
            dbytes_per_channel,
        })
    }

    pub const fn total_dbytes(self) -> u8 {
        self.channels * self.dbytes_per_channel
    }
    pub const fn lpddr5_supported(self) -> bool {
        self.lpddr5_supported
    }
    pub const fn dmi_supported(self) -> bool {
        self.dmi_supported
    }
    pub const fn ranks(self) -> u8 {
        self.ranks
    }
    pub const fn channels(self) -> u8 {
        self.channels
    }
    pub const fn dbytes_per_channel(self) -> u8 {
        self.dbytes_per_channel
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Cause<E> {
    InvalidBudget,
    Quarantined,
    Port(E),
    AlreadyRunning,
    /// No post-launch low Done was observed and the port gave no independent
    /// completion witness. Done also reads 1 while the ACSM is idle (measured
    /// on the Cubie A7A, M2), so a set Done on the first poll alone cannot
    /// distinguish a finished sequence from a launch that never started.
    NoBusyObserved,
    Timeout,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Failure<E> {
    pub cause: Cause<E>,
    pub clear_attempted: bool,
    pub clear_error: Option<E>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Completion {
    pub polls: u16,
    pub busy_polls: u16,
    pub raw_done: u16,
    /// Done as read before launch (1 while idle on the A733 PHY).
    pub prior_done: u16,
    /// Completion accepted on the port's witness rather than a busy poll.
    pub witnessed: bool,
}

/// An eventual physical implementation must establish the matching register
/// schema, bounded APB access, quiescent PIE/DMA/PMU and exclusive ownership,
/// the required clocks/reset/DFI state, and a valid already-installed original
/// sequence with reviewed address, repetition, timing and device constraints.
/// This interface does not install or infer a program. Every callback must
/// itself return within a bounded interval. The poll count is not microseconds.
/// A successful Run clear means a write was accepted, not a documented abort,
/// FIFO drain, status acknowledgement or complete physical recovery.
pub trait Port {
    type Error: Copy;
    fn preflight(&mut self) -> Result<(), Self::Error>;
    fn read_control(&mut self) -> Result<u16, Self::Error>;
    fn write_control(&mut self, value: u16) -> Result<(), Self::Error>;
    fn read_done(&mut self) -> Result<u16, Self::Error>;
    /// Independent evidence that the sequence just launched has run to its
    /// end, consulted only when Done is already set at the first poll (a
    /// short sequence can finish before it). Default: none, so such a
    /// completion is rejected as [`Cause::NoBusyObserved`].
    fn completion_witness(&mut self) -> Result<bool, Self::Error> {
        Ok(false)
    }
}

pub struct DirectRun<P> {
    port: P,
    quarantined: bool,
}

impl<P: Port> DirectRun<P> {
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

    pub fn run(&mut self, polls: u16) -> Result<Completion, Failure<P::Error>> {
        let before_launch = |cause| Failure {
            cause,
            clear_attempted: false,
            clear_error: None,
        };
        if self.quarantined {
            return Err(before_launch(Cause::Quarantined));
        }
        if polls == 0 || polls > MAX_POLLS {
            return Err(before_launch(Cause::InvalidBudget));
        }
        self.port
            .preflight()
            .map_err(|e| before_launch(Cause::Port(e)))?;
        let control = self
            .port
            .read_control()
            .map_err(|e| before_launch(Cause::Port(e)))?;
        if control & RUN_BIT != 0 {
            return Err(before_launch(Cause::AlreadyRunning));
        }
        // Done is not cleared by software: it reads 1 while idle. It is
        // recorded, and freshness comes from the post-launch observation.
        let prior_done = self
            .port
            .read_done()
            .map_err(|e| before_launch(Cause::Port(e)))?;

        // The source defines start as a 0->1 edge. Preserve every other field,
        // including PIE-managed AcsmProgPtr; no translation mode is invented.
        // A failed write may have reached the PHY, so it also requires clear.
        let result = (|| {
            self.port
                .write_control(control | RUN_BIT)
                .map_err(Cause::Port)?;
            for count in 1..=polls {
                // Every earlier poll of this launch read Done clear.
                let busy_polls = count - 1;
                let raw_done = self.port.read_done().map_err(Cause::Port)?;
                if raw_done & DONE_BIT != 0 {
                    let witnessed = busy_polls == 0;
                    if witnessed && !self.port.completion_witness().map_err(Cause::Port)? {
                        return Err(Cause::NoBusyObserved);
                    }
                    return Ok(Completion {
                        polls: count,
                        busy_polls,
                        raw_done,
                        prior_done,
                        witnessed,
                    });
                }
            }
            Err(Cause::Timeout)
        })();
        let clear_error = self.port.write_control(control & !RUN_BIT).err();
        // A timeout or failed access can leave physical execution uncertain
        // even when clearing Run succeeded; require external recovery rather
        // than silently using this transaction again.
        self.quarantined = result.is_err() || clear_error.is_some();
        match (result, clear_error) {
            (Ok(completion), None) => Ok(completion),
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
