//! USB mass storage, Bulk-Only Transport with SCSI, read-only, over an
//! abstract pair of bulk pipes ([`Pipe`]): for `fetch:usb0`.
//!
//! Read-only by construction: a command block wrapper can only be built by
//! [`cbw`], which refuses every SCSI operation outside [`scsi_allowed`]:
//! TEST UNIT READY, REQUEST SENSE, INQUIRY, READ CAPACITY (10) and (16),
//! READ (10) and READ (16), all device-to-host or without data. No WRITE,
//! VERIFY, SYNCHRONIZE CACHE, START STOP UNIT, PREVENT ALLOW MEDIUM
//! REMOVAL, MODE SELECT, FORMAT or UNMAP can be sent, and no CBW ever has
//! a host-to-device data phase. The unit tests check the list and run the
//! whole open/read path against a disk model that fails on anything else.
//! Error recovery uses only the class-specific Bulk-Only Mass Storage
//! Reset and CLEAR_FEATURE(ENDPOINT_HALT), which change no medium data.

pub const CBW_SIGNATURE: u32 = 0x4342_5355;
pub const CSW_SIGNATURE: u32 = 0x5342_5355;
pub const CBW_LEN: usize = 31;
pub const CSW_LEN: usize = 13;
/// Blocks per READ command (64 KiB at 512 bytes).
pub const MAX_BLOCKS: u32 = 128;

pub const TEST_UNIT_READY: u8 = 0x00;
pub const REQUEST_SENSE: u8 = 0x03;
pub const INQUIRY: u8 = 0x12;
pub const READ_CAPACITY_10: u8 = 0x25;
pub const READ_10: u8 = 0x28;
pub const READ_16: u8 = 0x88;
pub const SERVICE_ACTION_IN_16: u8 = 0x9e;
pub const SA_READ_CAPACITY_16: u8 = 0x10;

/// The SCSI operations [`cbw`] builds.
pub fn scsi_allowed(cdb: &[u8]) -> bool {
    match cdb.first() {
        Some(
            &(TEST_UNIT_READY | REQUEST_SENSE | INQUIRY | READ_CAPACITY_10 | READ_10 | READ_16),
        ) => true,
        Some(&SERVICE_ACTION_IN_16) => cdb
            .get(1)
            .is_some_and(|sa| sa & 0x1f == SA_READ_CAPACITY_16),
        _ => false,
    }
}

/// A Command Block Wrapper for `cdb`, data in (device to host) when
/// `data_len` is not 0; None for an operation outside [`scsi_allowed`].
pub fn cbw(tag: u32, data_len: u32, lun: u8, cdb: &[u8]) -> Option<[u8; CBW_LEN]> {
    if !scsi_allowed(cdb) || cdb.len() > 16 || cdb.is_empty() {
        return None;
    }
    let mut b = [0u8; CBW_LEN];
    b[0..4].copy_from_slice(&CBW_SIGNATURE.to_le_bytes());
    b[4..8].copy_from_slice(&tag.to_le_bytes());
    b[8..12].copy_from_slice(&data_len.to_le_bytes());
    b[12] = if data_len != 0 { 0x80 } else { 0 };
    b[13] = lun & 0x0f;
    b[14] = cdb.len() as u8;
    b[15..15 + cdb.len()].copy_from_slice(cdb);
    Some(b)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum XferError {
    Stall,
    Timeout,
    /// Babble, transaction or buffer error, or the host gave up.
    Error,
}

/// The bulk pair of the mass storage interface, and its recovery requests.
pub trait Pipe {
    fn bulk_out(&mut self, data: &[u8]) -> Result<(), XferError>;
    /// Up to `buf.len()` bytes; the count received (short is fine).
    fn bulk_in(&mut self, buf: &mut [u8]) -> Result<usize, XferError>;
    /// CLEAR_FEATURE(ENDPOINT_HALT) on bulk IN (`true`) or OUT.
    fn clear_halt(&mut self, in_ep: bool) -> Result<(), XferError>;
    /// Bulk-Only Mass Storage Reset (class request 0xff).
    fn reset(&mut self) -> Result<(), XferError>;
    fn delay_ms(&mut self, _ms: u32) {}
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    Refused,
    Xfer(&'static str, XferError),
    /// CSW status 1: the command failed (sense key, ASC).
    Check(u8, u8),
    /// CSW status 2, a bad CSW, or residue on a read.
    Phase,
    NotReady,
    Range,
    Unsupported(&'static str),
}

impl Error {
    pub fn message(&self) -> &'static str {
        match self {
            Error::Refused => "SCSI command not allowed (read-only)",
            Error::Xfer(w, _) => w,
            Error::Check(..) => "the disk reported an error (see the console)",
            Error::Phase => "mass storage phase error",
            Error::NotReady => "the disk did not become ready",
            Error::Range => "read past the end of the disk",
            Error::Unsupported(w) => w,
        }
    }
}

/// The open disk.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Disk {
    pub lun: u8,
    pub block_size: u32,
    pub blocks: u64,
    /// INQUIRY vendor (8) and product (16) identification, ASCII.
    pub ident: [u8; 24],
    tag: u32,
}

impl Disk {
    fn recover<P: Pipe>(p: &mut P) {
        let _ = p.reset();
        let _ = p.clear_halt(true);
        let _ = p.clear_halt(false);
    }

    /// One BOT command: CBW, data in (into `data`), CSW. Returns the bytes
    /// received.
    fn command<P: Pipe>(&mut self, p: &mut P, cdb: &[u8], data: &mut [u8]) -> Result<usize, Error> {
        self.tag = self.tag.wrapping_add(1);
        let tag = self.tag;
        let w = cbw(tag, data.len() as u32, self.lun, cdb).ok_or(Error::Refused)?;
        if let Err(e) = p.bulk_out(&w) {
            Self::recover(p);
            return Err(Error::Xfer("CBW", e));
        }
        let mut got = 0;
        if !data.is_empty() {
            match p.bulk_in(data) {
                Ok(n) => got = n,
                Err(XferError::Stall) => {
                    p.clear_halt(true)
                        .map_err(|e| Error::Xfer("clear halt", e))?;
                }
                Err(e) => {
                    Self::recover(p);
                    return Err(Error::Xfer("data", e));
                }
            }
        }
        let mut csw = [0u8; CSW_LEN];
        let n = match p.bulk_in(&mut csw) {
            Ok(n) => n,
            Err(XferError::Stall) => {
                p.clear_halt(true)
                    .map_err(|e| Error::Xfer("clear halt", e))?;
                p.bulk_in(&mut csw).map_err(|e| Error::Xfer("CSW", e))?
            }
            Err(e) => {
                Self::recover(p);
                return Err(Error::Xfer("CSW", e));
            }
        };
        let sig = u32::from_le_bytes([csw[0], csw[1], csw[2], csw[3]]);
        let ctag = u32::from_le_bytes([csw[4], csw[5], csw[6], csw[7]]);
        if n != CSW_LEN || sig != CSW_SIGNATURE || ctag != tag || csw[12] >= 2 {
            Self::recover(p);
            return Err(Error::Phase);
        }
        if csw[12] == 1 {
            return Err(self.sense(p));
        }
        Ok(got)
    }

    /// REQUEST SENSE after a failed command: (key, ASC).
    fn sense<P: Pipe>(&mut self, p: &mut P) -> Error {
        let mut s = [0u8; 18];
        match self.command(p, &[REQUEST_SENSE, 0, 0, 0, 18, 0], &mut s) {
            Ok(n) if n >= 13 => Error::Check(s[2] & 0x0f, s[12]),
            _ => Error::Check(0xff, 0xff),
        }
    }

    /// Identify, wait until ready, read the capacity.
    pub fn open<P: Pipe>(p: &mut P, lun: u8) -> Result<Disk, Error> {
        let mut d = Disk {
            lun,
            block_size: 0,
            blocks: 0,
            ident: [b' '; 24],
            tag: 0,
        };
        let mut inq = [0u8; 36];
        let n = d.command(p, &[INQUIRY, 0, 0, 0, 36, 0], &mut inq)?;
        if n >= 32 {
            d.ident.copy_from_slice(&inq[8..32]);
        }
        if inq[0] & 0x1f != 0 {
            return Err(Error::Unsupported("not a direct-access block device"));
        }
        let mut ready = false;
        for _ in 0..50 {
            match d.command(p, &[TEST_UNIT_READY, 0, 0, 0, 0, 0], &mut []) {
                Ok(_) => {
                    ready = true;
                    break;
                }
                Err(Error::Check(..)) => p.delay_ms(100),
                Err(e) => return Err(e),
            }
        }
        if !ready {
            return Err(Error::NotReady);
        }
        let mut cap = [0u8; 8];
        if d.command(p, &[READ_CAPACITY_10, 0, 0, 0, 0, 0, 0, 0, 0, 0], &mut cap)? != 8 {
            return Err(Error::Phase);
        }
        let last = u32::from_be_bytes([cap[0], cap[1], cap[2], cap[3]]);
        d.block_size = u32::from_be_bytes([cap[4], cap[5], cap[6], cap[7]]);
        d.blocks = last as u64 + 1;
        if last == u32::MAX {
            let mut cap16 = [0u8; 32];
            let mut cdb = [0u8; 16];
            cdb[0] = SERVICE_ACTION_IN_16;
            cdb[1] = SA_READ_CAPACITY_16;
            cdb[13] = 32;
            if d.command(p, &cdb, &mut cap16)? < 12 {
                return Err(Error::Phase);
            }
            let mut b8 = [0u8; 8];
            b8.copy_from_slice(&cap16[..8]);
            d.blocks = u64::from_be_bytes(b8).checked_add(1).ok_or(Error::Range)?;
            d.block_size = u32::from_be_bytes([cap16[8], cap16[9], cap16[10], cap16[11]]);
        }
        if !crate::gpt::valid_block_size(d.block_size) {
            return Err(Error::Unsupported("unsupported logical block size"));
        }
        if d.blocks.checked_mul(d.block_size as u64).is_none() {
            return Err(Error::Range);
        }
        Ok(d)
    }

    /// Read `buf.len() / block_size` blocks from `lba`.
    pub fn read<P: Pipe>(&mut self, p: &mut P, lba: u64, buf: &mut [u8]) -> Result<(), Error> {
        let bs = self.block_size as usize;
        if bs == 0 || !buf.len().is_multiple_of(bs) {
            return Err(Error::Unsupported("partial block"));
        }
        let n = (buf.len() / bs) as u64;
        if lba.checked_add(n).is_none_or(|e| e > self.blocks) {
            return Err(Error::Range);
        }
        let mut lba = lba;
        for chunk in buf.chunks_mut(bs * MAX_BLOCKS as usize) {
            let count = (chunk.len() / bs) as u32;
            let got = if lba + count as u64 <= u32::MAX as u64 + 1 {
                let l = (lba as u32).to_be_bytes();
                let c = (count as u16).to_be_bytes();
                self.command(
                    p,
                    &[READ_10, 0, l[0], l[1], l[2], l[3], 0, c[0], c[1], 0],
                    chunk,
                )?
            } else {
                let mut cdb = [0u8; 16];
                cdb[0] = READ_16;
                cdb[2..10].copy_from_slice(&lba.to_be_bytes());
                cdb[10..14].copy_from_slice(&count.to_be_bytes());
                self.command(p, &cdb, chunk)?
            };
            if got != chunk.len() {
                return Err(Error::Phase);
            }
            lba += count as u64;
        }
        Ok(())
    }
}

#[cfg(test)]
pub(crate) mod model {
    //! A Bulk-Only disk: parses CBWs, serves virtual data, panics on any
    //! operation outside the read-only list, and can inject faults.
    use super::*;
    use std::{collections::VecDeque, vec::Vec};

    pub struct DiskModel {
        pub blocks: u64,
        pub block_size: u32,
        pub capacity16_last: Option<u64>,
        pub not_ready: u32,
        pub stall_data_once: bool,
        pub short_data_once: bool,
        pub bad_tag_once: bool,
        pub fail_out: bool,
        pub ops: Vec<u8>,
        pub resets: u32,
        pending: VecDeque<Vec<u8>>,
        stalled: bool,
        sense: (u8, u8),
    }

    impl DiskModel {
        pub fn new(blocks: u64, block_size: u32) -> Self {
            DiskModel {
                blocks,
                block_size,
                capacity16_last: None,
                not_ready: 0,
                stall_data_once: false,
                short_data_once: false,
                bad_tag_once: false,
                fail_out: false,
                ops: Vec::new(),
                resets: 0,
                pending: VecDeque::new(),
                stalled: false,
                sense: (0, 0),
            }
        }
        pub fn byte(addr: u64) -> u8 {
            (addr ^ (addr >> 9) ^ (addr >> 21) ^ (addr >> 33)) as u8
        }
        fn csw(tag: u32, residue: u32, status: u8) -> Vec<u8> {
            let mut c = Vec::new();
            c.extend_from_slice(&CSW_SIGNATURE.to_le_bytes());
            c.extend_from_slice(&tag.to_le_bytes());
            c.extend_from_slice(&residue.to_le_bytes());
            c.push(status);
            c
        }
    }

    impl Pipe for DiskModel {
        fn bulk_out(&mut self, w: &[u8]) -> Result<(), XferError> {
            if self.fail_out {
                return Err(XferError::Error);
            }
            assert_eq!(w.len(), CBW_LEN);
            assert_eq!(u32::from_le_bytes([w[0], w[1], w[2], w[3]]), CBW_SIGNATURE);
            let tag = u32::from_le_bytes([w[4], w[5], w[6], w[7]]);
            let len = u32::from_le_bytes([w[8], w[9], w[10], w[11]]);
            let cdb = &w[15..15 + w[14] as usize];
            assert!(
                scsi_allowed(cdb),
                "SCSI op {:#04x} reached the disk",
                cdb[0]
            );
            assert!(len == 0 || w[12] == 0x80, "host-to-device data phase");
            self.ops.push(cdb[0]);
            let (data, status): (Vec<u8>, u8) = match cdb[0] {
                INQUIRY => {
                    let mut d = std::vec![0u8; 36];
                    d[8..32].copy_from_slice(b"Model   Disk 3.0        ");
                    (d, 0)
                }
                TEST_UNIT_READY => {
                    if self.not_ready > 0 {
                        self.not_ready -= 1;
                        self.sense = (2, 0x04);
                        (Vec::new(), 1)
                    } else {
                        (Vec::new(), 0)
                    }
                }
                REQUEST_SENSE => {
                    let mut d = std::vec![0u8; 18];
                    d[0] = 0x70;
                    d[2] = self.sense.0;
                    d[12] = self.sense.1;
                    (d, 0)
                }
                READ_CAPACITY_10 => {
                    let last = (self.blocks - 1).min(u32::MAX as u64) as u32;
                    let mut d = last.to_be_bytes().to_vec();
                    d.extend_from_slice(&self.block_size.to_be_bytes());
                    (d, 0)
                }
                SERVICE_ACTION_IN_16 => {
                    let mut d = self
                        .capacity16_last
                        .unwrap_or(self.blocks - 1)
                        .to_be_bytes()
                        .to_vec();
                    d.extend_from_slice(&self.block_size.to_be_bytes());
                    d.resize(32, 0);
                    (d, 0)
                }
                READ_10 | READ_16 => {
                    let (lba, n) = if cdb[0] == READ_10 {
                        (
                            u32::from_be_bytes([cdb[2], cdb[3], cdb[4], cdb[5]]) as u64,
                            u16::from_be_bytes([cdb[7], cdb[8]]) as u64,
                        )
                    } else {
                        let mut l = [0u8; 8];
                        l.copy_from_slice(&cdb[2..10]);
                        (
                            u64::from_be_bytes(l),
                            u32::from_be_bytes([cdb[10], cdb[11], cdb[12], cdb[13]]) as u64,
                        )
                    };
                    assert_eq!(n * self.block_size as u64, len as u64);
                    assert!(lba + n <= self.blocks, "read past the end reached the disk");
                    let start = lba * self.block_size as u64;
                    let d: Vec<u8> = (0..len as u64).map(|i| Self::byte(start + i)).collect();
                    (d, 0)
                }
                _ => unreachable!(),
            };
            let mut data = data;
            data.truncate(len as usize);
            let mut residue = len - data.len() as u32;
            if self.short_data_once && !data.is_empty() {
                self.short_data_once = false;
                residue += 512;
                data.truncate(data.len() - 512);
            }
            if self.stall_data_once && len > 0 {
                self.stall_data_once = false;
                self.stalled = true;
                self.pending.push_back(Self::csw(tag, len, 1));
                self.sense = (3, 0x11);
                return Ok(());
            }
            if len > 0 {
                self.pending.push_back(data);
            }
            let ctag = if self.bad_tag_once {
                self.bad_tag_once = false;
                tag ^ 1
            } else {
                tag
            };
            self.pending.push_back(Self::csw(ctag, residue, status));
            Ok(())
        }
        fn bulk_in(&mut self, buf: &mut [u8]) -> Result<usize, XferError> {
            if self.stalled {
                return Err(XferError::Stall);
            }
            let d = self.pending.pop_front().ok_or(XferError::Timeout)?;
            let n = d.len().min(buf.len());
            buf[..n].copy_from_slice(&d[..n]);
            Ok(n)
        }
        fn clear_halt(&mut self, _in_ep: bool) -> Result<(), XferError> {
            self.stalled = false;
            Ok(())
        }
        fn reset(&mut self) -> Result<(), XferError> {
            self.resets += 1;
            self.pending.clear();
            self.stalled = false;
            Ok(())
        }
    }
}

#[cfg(test)]
mod tests {
    use super::model::DiskModel;
    use super::*;
    use std::vec;

    #[test]
    fn allow_list_is_read_only() {
        let allowed: std::vec::Vec<u8> = (0..=255u8)
            .filter(|&op| scsi_allowed(&[op, SA_READ_CAPACITY_16]))
            .collect();
        assert_eq!(allowed, [0x00, 0x03, 0x12, 0x25, 0x28, 0x88, 0x9e]);
        // 0x9e only as READ CAPACITY (16).
        assert!(!scsi_allowed(&[0x9e, 0x11]));
        for op in [
            0x2a, 0x8a, 0xaa, 0x2e, 0x35, 0x91, 0x1b, 0x1e, 0x15, 0x55, 0x04, 0x42, 0x2f, 0x41,
            0x93,
        ] {
            assert!(!scsi_allowed(&[op, 0]), "{op:#x}");
            assert_eq!(cbw(1, 512, 0, &[op, 0, 0, 0, 0, 0, 0, 0, 0, 0]), None);
        }
        let w = cbw(7, 512, 0, &[READ_10, 0, 0, 0, 0, 1, 0, 0, 1, 0]).unwrap();
        assert_eq!((w[12], w[14]), (0x80, 10));
        let w = cbw(7, 0, 0, &[TEST_UNIT_READY, 0, 0, 0, 0, 0]).unwrap();
        assert_eq!(w[12], 0);
    }

    #[test]
    fn open_and_read() {
        let mut m = DiskModel::new(60_000_000, 512); // ~30 GB
        m.not_ready = 3;
        let mut d = Disk::open(&mut m, 0).unwrap();
        assert_eq!((d.blocks, d.block_size), (60_000_000, 512));
        assert_eq!(&d.ident[..8], b"Model   ");
        let mut buf = vec![0u8; 512 * 300];
        d.read(&mut m, 59_999_700, &mut buf).unwrap();
        assert!(buf
            .iter()
            .enumerate()
            .all(|(i, &b)| b == DiskModel::byte(59_999_700 * 512 + i as u64)));
        // 300 blocks: 128 + 128 + 44.
        let reads = m.ops.iter().filter(|&&o| o == READ_10).count();
        assert_eq!(reads, 3);
        assert_eq!(d.read(&mut m, 59_999_701, &mut buf), Err(Error::Range));
        assert_eq!(
            d.read(&mut m, 0, &mut buf[..100]),
            Err(Error::Unsupported("partial block"))
        );
    }

    #[test]
    fn read_capacity_rejects_overflow() {
        let mut m = DiskModel::new(u32::MAX as u64 + 1, 4096);
        for last in [u64::MAX, u64::MAX / 4096] {
            m.capacity16_last = Some(last);
            assert_eq!(Disk::open(&mut m, 0), Err(Error::Range));
        }
    }

    #[test]
    fn supported_logical_block_sizes() {
        for bs in [512, 1024, 2048, 4096] {
            let mut m = DiskModel::new(1024, bs);
            let mut d = Disk::open(&mut m, 0).unwrap();
            let mut one = vec![0; bs as usize];
            assert_eq!(d.block_size, bs);
            d.read(&mut m, 1, &mut one).unwrap();
            assert_eq!(one[0], DiskModel::byte(bs.into()));
        }
        let mut m = DiskModel::new(1024, 520);
        assert!(matches!(Disk::open(&mut m, 0), Err(Error::Unsupported(_))));
    }

    #[test]
    fn large_disk_uses_16_byte_commands() {
        let mut m = DiskModel::new((1u64 << 33) + 10, 512); // 4 TiB
        let mut d = Disk::open(&mut m, 0).unwrap();
        assert_eq!(d.blocks, (1u64 << 33) + 10);
        assert!(m.ops.contains(&SERVICE_ACTION_IN_16));
        let mut buf = vec![0u8; 1024];
        d.read(&mut m, (1u64 << 33) + 8, &mut buf).unwrap();
        assert_eq!(*m.ops.last().unwrap(), READ_16);
        assert_eq!(buf[3], DiskModel::byte(((1u64 << 33) + 8) * 512 + 3));
        d.read(&mut m, 5, &mut buf).unwrap();
        assert_eq!(*m.ops.last().unwrap(), READ_10);
    }

    #[test]
    fn faults_mid_transfer() {
        let mut m = DiskModel::new(1 << 20, 512);
        let mut d = Disk::open(&mut m, 0).unwrap();
        let mut buf = vec![0u8; 512 * 4];
        // Data stall: clear halt, CSW failed, sense read: an error, no hang.
        m.stall_data_once = true;
        assert_eq!(d.read(&mut m, 0, &mut buf), Err(Error::Check(3, 0x11)));
        d.read(&mut m, 0, &mut buf).unwrap();
        // Short data: the read fails rather than return a hole.
        m.short_data_once = true;
        assert_eq!(d.read(&mut m, 0, &mut buf), Err(Error::Phase));
        // A CSW with the wrong tag: reset recovery, then it works again.
        m.bad_tag_once = true;
        assert_eq!(d.read(&mut m, 0, &mut buf), Err(Error::Phase));
        assert_eq!(m.resets, 1);
        d.read(&mut m, 8, &mut buf).unwrap();
        // The disk never becomes ready.
        let mut m = DiskModel::new(1 << 20, 512);
        m.not_ready = 1000;
        assert_eq!(Disk::open(&mut m, 0), Err(Error::NotReady));
        // The host side fails.
        let mut m = DiskModel::new(1 << 20, 512);
        m.fail_out = true;
        assert!(matches!(Disk::open(&mut m, 0), Err(Error::Xfer("CBW", _))));
        // Odd block size refused.
        let mut m = DiskModel::new(1 << 20, 520);
        assert!(Disk::open(&mut m, 0).is_err());
    }
}
