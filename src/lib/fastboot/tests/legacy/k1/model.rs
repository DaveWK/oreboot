// SPDX-License-Identifier: GPL-2.0-only
// The host model of ore-edk-boot-opi c91bbc4e40 tests/fastboot-model.rs up to
// its tests: storage/MMIO stubs that panic, and a scripted USB. Edits: the
// `cfg(baseline)` variants are removed (the current ones are kept).
// The real dispatcher uses a scripted USB transport. Storage/MMIO calls panic.
extern crate self as util;
use std::cell::RefCell;
use std::collections::VecDeque;
pub mod cache {
    pub fn flush(_: usize, _: usize) {
        panic!("unexpected storage cache I/O")
    }
}
pub mod mmio {
    pub fn read8(_: usize) -> u8 {
        panic!("unexpected MMIO")
    }
    pub fn read16(_: usize) -> u16 {
        panic!("unexpected MMIO")
    }
    pub fn read32(_: usize) -> u32 {
        panic!("unexpected MMIO")
    }
    pub fn write8(_: usize, _: u8) {
        panic!("unexpected MMIO")
    }
    pub fn write16(_: usize, _: u16) {
        panic!("unexpected MMIO")
    }
    pub fn write32(_: usize, _: u32) {
        panic!("unexpected MMIO")
    }
}
#[derive(Default)]
struct Script {
    commands: VecDeque<&'static str>,
    replies: Vec<Vec<u8>>,
    disconnected: usize,
    received: usize,
    init_fail: bool,
    ack_fail: bool,
    flush_fail: bool,
    wait_fail: bool,
    recv_fail: bool,
    download_fail: bool,
}
thread_local! {static SCRIPT:RefCell<Script>=RefCell::new(Script::default());}
fn edit(f: impl FnOnce(&mut Script)) {
    SCRIPT.with(|s| f(&mut s.borrow_mut()))
}
fn view<T>(f: impl FnOnce(&Script) -> T) -> T {
    SCRIPT.with(|s| f(&s.borrow()))
}
fn reset(commands: Vec<&'static str>) {
    edit(|s| {
        *s = Script {
            commands: commands.into(),
            ..Script::default()
        }
    })
}
pub mod usb {
    #[derive(Debug, Clone, Copy)]
    pub enum Error {
        Reset,
        Timeout,
        PhyTimeout,
        FlushTimeout,
    }
    pub struct Descriptors<'a> {
        pub vendor: u16,
        pub product: u16,
        pub class: [u8; 3],
        pub strings: [&'a str; 4],
    }
    pub struct Usb<'a>(std::marker::PhantomData<&'a ()>);
    impl<'a> Usb<'a> {
        pub fn init(_: Descriptors<'a>) -> Result<Self, Error> {
            if super::view(|s| s.init_fail) {
                Err(Error::PhyTimeout)
            } else {
                Ok(Self(std::marker::PhantomData))
            }
        }
        pub fn wait_configured(&mut self) -> Result<(), Error> {
            if super::view(|s| s.wait_fail) {
                Err(Error::FlushTimeout)
            } else {
                Ok(())
            }
        }
        pub fn disconnect(&mut self) -> Result<(), Error> {
            super::edit(|s| s.disconnected += 1);
            if super::view(|s| s.flush_fail) {
                Err(Error::FlushTimeout)
            } else {
                Ok(())
            }
        }
        pub fn send(&mut self, data: &[u8]) -> Result<(), Error> {
            super::edit(|s| s.replies.push(data.to_vec()));
            if data == b"OKAY" && super::view(|s| s.ack_fail) {
                Err(Error::Timeout)
            } else {
                Ok(())
            }
        }
        pub fn recv(&mut self, out: &mut [u8]) -> Result<usize, Error> {
            if super::view(|s| s.recv_fail) {
                return Err(Error::FlushTimeout);
            }
            let cmd = super::SCRIPT.with(|s| {
                let mut s = s.borrow_mut();
                s.received += 1;
                s.commands
                    .pop_front()
                    .expect("dispatcher requested unexpected command")
            });
            out[..cmd.len()].copy_from_slice(cmd.as_bytes());
            Ok(cmd.len())
        }
        pub fn recv_into(&mut self, _: usize, _: usize) -> Result<(), Error> {
            if super::view(|s| s.download_fail) {
                Err(Error::Timeout)
            } else {
                Ok(())
            }
        }
    }
}
