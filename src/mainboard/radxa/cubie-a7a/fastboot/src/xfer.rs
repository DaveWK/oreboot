// SPDX-License-Identifier: GPL-2.0-only

//! Bulk data phases (download, fetch upload) over a device FIFO endpoint,
//! with as few register round trips per packet as the protocol allows.
//!
//! The controller (bt0's MUSB, EP1 double buffered) is seen through
//! [`OutEp`] and [`InEp`]: "is a packet / a buffer there", "move one
//! packet", and [`OutEp::service`]/[`InEp::service`], the expensive part
//! (bus events, EP0 requests, the watchdog). While packets flow, `service`
//! runs only every [`SERVICE_EVERY`] packets; whenever the endpoint has
//! nothing to do, it runs on every spin, so a bus reset or an EP0 request
//! is seen within one idle spin, and a stalled host ends in a timeout
//! counted from the last packet.

/// Packets moved between two `service` calls while data flows (32 KiB at
/// high speed; a 16 s watchdog never notices).
pub const SERVICE_EVERY: u32 = 64;

/// What can end a data phase early.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Stop<E> {
    /// `service` failed (bus reset, ...).
    Service(E),
    /// No packet (or no free buffer) for the timeout.
    Timeout,
    /// The host sent more than the transfer has left.
    Overrun,
    /// A short packet before the end.
    Short,
}

/// A bulk OUT endpoint (host to device).
pub trait OutEp {
    type Error: Copy;
    /// The length of the packet waiting in the FIFO, if any.
    fn ready(&mut self) -> Option<usize>;
    /// Copy the waiting packet into `out` (exactly its length) and give
    /// its FIFO buffer back.
    fn take(&mut self, out: &mut [u8]);
    /// Drop the waiting packet and give its buffer back.
    fn discard(&mut self);
    fn service(&mut self) -> Result<(), Self::Error>;
    fn now_us(&mut self) -> u64;
}

/// A bulk IN endpoint (device to host).
pub trait InEp {
    type Error: Copy;
    /// Is a FIFO buffer free for one more packet?
    fn free(&mut self) -> bool;
    /// Load one packet (at most max packet) and hand it to the controller.
    fn put(&mut self, data: &[u8]);
    fn service(&mut self) -> Result<(), Self::Error>;
    fn now_us(&mut self) -> u64;
}

/// Receive exactly `out.len()` bytes in packets of up to `maxp`.
pub fn recv_exact<E: OutEp>(
    ep: &mut E,
    out: &mut [u8],
    maxp: usize,
    timeout_us: u64,
) -> Result<(), Stop<E::Error>> {
    let mut got = 0;
    let mut since = 0;
    let mut idle_since: Option<u64> = None;
    while got < out.len() {
        let Some(n) = ep.ready() else {
            ep.service().map_err(Stop::Service)?;
            let now = ep.now_us();
            match idle_since {
                None => idle_since = Some(now),
                Some(t) if now.saturating_sub(t) > timeout_us => return Err(Stop::Timeout),
                Some(_) => {}
            }
            continue;
        };
        idle_since = None;
        let want = (out.len() - got).min(maxp);
        if n > want {
            ep.discard();
            return Err(Stop::Overrun);
        }
        ep.take(&mut out[got..got + n]);
        got += n;
        if n < want && got < out.len() {
            return Err(Stop::Short);
        }
        since += 1;
        if since >= SERVICE_EVERY {
            since = 0;
            ep.service().map_err(Stop::Service)?;
        }
    }
    Ok(())
}

/// Send `data` as full packets and a short last one (no zero-length
/// packet: the host reads exactly the size announced).
pub fn send<E: InEp>(
    ep: &mut E,
    data: &[u8],
    maxp: usize,
    timeout_us: u64,
) -> Result<(), Stop<E::Error>> {
    let mut since = 0;
    for chunk in data.chunks(maxp) {
        let mut idle_since: Option<u64> = None;
        while !ep.free() {
            ep.service().map_err(Stop::Service)?;
            let now = ep.now_us();
            match idle_since {
                None => idle_since = Some(now),
                Some(t) if now.saturating_sub(t) > timeout_us => return Err(Stop::Timeout),
                Some(_) => {}
            }
        }
        ep.put(chunk);
        since += 1;
        if since >= SERVICE_EVERY {
            since = 0;
            ep.service().map_err(Stop::Service)?;
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::{collections::VecDeque, vec, vec::Vec};

    /// A double-buffered OUT FIFO fed by a host model: the host fills free
    /// buffers from its packet queue whenever the device looks (one packet
    /// per look, as a host that is not starved).
    struct Out {
        host: VecDeque<Vec<u8>>,
        fifo: VecDeque<Vec<u8>>,
        buffers: usize,
        services: Vec<usize>,
        moved: usize,
        reset_at_service: Option<usize>,
        clock: u64,
    }

    impl Out {
        fn new(packets: Vec<Vec<u8>>, buffers: usize) -> Self {
            Out {
                host: packets.into(),
                fifo: VecDeque::new(),
                buffers,
                services: Vec::new(),
                moved: 0,
                reset_at_service: None,
                clock: 0,
            }
        }
        fn host_step(&mut self) {
            if self.fifo.len() < self.buffers {
                if let Some(p) = self.host.pop_front() {
                    self.fifo.push_back(p);
                }
            }
        }
    }

    impl OutEp for Out {
        type Error = &'static str;
        fn ready(&mut self) -> Option<usize> {
            self.host_step();
            self.fifo.front().map(|p| p.len())
        }
        fn take(&mut self, out: &mut [u8]) {
            let p = self.fifo.pop_front().unwrap();
            assert_eq!(p.len(), out.len());
            out.copy_from_slice(&p);
            self.moved += 1;
        }
        fn discard(&mut self) {
            self.fifo.pop_front();
        }
        fn service(&mut self) -> Result<(), &'static str> {
            self.services.push(self.moved);
            if self.reset_at_service == Some(self.services.len()) {
                return Err("bus reset");
            }
            Ok(())
        }
        fn now_us(&mut self) -> u64 {
            self.clock += 100;
            self.clock
        }
    }

    fn packets(total: usize, maxp: usize) -> Vec<Vec<u8>> {
        let data: Vec<u8> = (0..total).map(|i| (i * 7 + 1) as u8).collect();
        data.chunks(maxp).map(|c| c.to_vec()).collect()
    }

    #[test]
    fn exact_download_double_buffered() {
        for (total, maxp, bufs) in [
            (5 << 20, 512, 2),
            (5 << 20, 512, 1),
            (100_000, 64, 2),
            (512, 512, 2),
            (1, 512, 2),
        ] {
            let mut ep = Out::new(packets(total, maxp), bufs);
            let mut out = vec![0u8; total];
            recv_exact(&mut ep, &mut out, maxp, 1_000_000).unwrap();
            assert!(out.iter().enumerate().all(|(i, &b)| b == (i * 7 + 1) as u8));
            // Service between packets at most every SERVICE_EVERY packets.
            let mut last = 0;
            for &m in &ep.services {
                assert!(m - last <= SERVICE_EVERY as usize);
                last = m;
            }
            assert!(ep.moved - last <= SERVICE_EVERY as usize);
        }
    }

    #[test]
    fn overrun_short_timeout_and_reset() {
        // The host sends 1024 bytes where 600 are expected: the second packet
        // (512) is larger than the 88 left.
        let mut ep = Out::new(packets(1024, 512), 2);
        let mut out = vec![0u8; 600];
        assert_eq!(
            recv_exact(&mut ep, &mut out, 512, 1_000_000),
            Err(Stop::Overrun)
        );
        // A short packet in the middle.
        let mut p = packets(2048, 512);
        p[1].truncate(100);
        let mut ep = Out::new(p, 2);
        let mut out = vec![0u8; 2048];
        assert_eq!(
            recv_exact(&mut ep, &mut out, 512, 1_000_000),
            Err(Stop::Short)
        );
        // The host stops after 3 packets: timeout, serviced on every idle spin.
        let mut ep = Out::new(packets(1536, 512), 2);
        let mut out = vec![0u8; 4096];
        assert_eq!(
            recv_exact(&mut ep, &mut out, 512, 10_000),
            Err(Stop::Timeout)
        );
        assert!(ep.services.len() >= 50);
        // A bus reset seen by service while data flows ends the transfer.
        let mut ep = Out::new(packets(1 << 20, 512), 2);
        ep.reset_at_service = Some(3);
        let mut out = vec![0u8; 1 << 20];
        assert_eq!(
            recv_exact(&mut ep, &mut out, 512, 1_000_000),
            Err(Stop::Service("bus reset"))
        );
        assert_eq!(ep.moved, 3 * SERVICE_EVERY as usize);
    }

    /// A double-buffered IN FIFO drained by a host model, one packet per
    /// look when it is reading.
    struct In {
        fifo: VecDeque<Vec<u8>>,
        buffers: usize,
        host_reads: bool,
        got: Vec<Vec<u8>>,
        services: usize,
        clock: u64,
    }

    impl InEp for In {
        type Error = ();
        fn free(&mut self) -> bool {
            if self.host_reads {
                if let Some(p) = self.fifo.pop_front() {
                    self.got.push(p);
                }
            }
            self.fifo.len() < self.buffers
        }
        fn put(&mut self, data: &[u8]) {
            assert!(self.fifo.len() < self.buffers, "put into a full FIFO");
            assert!(!data.is_empty() && data.len() <= 512);
            self.fifo.push_back(data.to_vec());
        }
        fn service(&mut self) -> Result<(), ()> {
            self.services += 1;
            Ok(())
        }
        fn now_us(&mut self) -> u64 {
            self.clock += 100;
            self.clock
        }
    }

    #[test]
    fn upload_packets_and_no_zlp() {
        for total in [1usize, 511, 512, 513, 1 << 20, (1 << 20) + 1] {
            let data: Vec<u8> = (0..total).map(|i| i as u8).collect();
            let mut ep = In {
                fifo: VecDeque::new(),
                buffers: 2,
                host_reads: true,
                got: vec![],
                services: 0,
                clock: 0,
            };
            send(&mut ep, &data, 512, 1_000_000).unwrap();
            ep.got.extend(ep.fifo.drain(..));
            assert_eq!(ep.got.concat(), data);
            assert_eq!(ep.got.len(), total.div_ceil(512), "no zero-length packet");
            assert!(ep.got[..ep.got.len() - 1].iter().all(|p| p.len() == 512));
        }
        // A host that never reads: the third packet times out.
        let mut ep = In {
            fifo: VecDeque::new(),
            buffers: 2,
            host_reads: false,
            got: vec![],
            services: 0,
            clock: 0,
        };
        assert_eq!(send(&mut ep, &[0u8; 4096], 512, 10_000), Err(Stop::Timeout));
        assert_eq!(ep.fifo.len(), 2);
    }
}
