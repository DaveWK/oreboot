// SPDX-License-Identifier: GPL-2.0-only
//! A register model for host tests: a sparse register file, a log of every
//! access, and a responder that plays the hardware's part (status bits that
//! a real block sets on its own).

#![allow(dead_code)]

use regio::RegIo;
use std::collections::BTreeMap;
use std::fmt;

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Access {
    R(usize, u32),
    W(usize, u32),
}

pub type Responder = Box<dyn FnMut(&mut BTreeMap<usize, u32>, usize) -> Option<u32>>;

#[derive(Default)]
pub struct Model {
    pub regs: BTreeMap<usize, u32>,
    pub log: Vec<Access>,
    pub clock: u64,
    pub text: String,
    pub responder: Option<Responder>,
}

impl Model {
    pub fn writes(&self) -> Vec<(usize, u32)> {
        self.log
            .iter()
            .filter_map(|a| match a {
                Access::W(a, v) => Some((*a, *v)),
                _ => None,
            })
            .collect()
    }
}

impl RegIo for Model {
    fn read32(&mut self, addr: usize) -> u32 {
        let v = match self
            .responder
            .as_mut()
            .and_then(|r| r(&mut self.regs, addr))
        {
            Some(v) => v,
            None => *self.regs.get(&addr).unwrap_or(&0),
        };
        self.log.push(Access::R(addr, v));
        v
    }
    fn write32(&mut self, addr: usize, val: u32) {
        self.regs.insert(addr, val);
        self.log.push(Access::W(addr, val));
    }
    fn read16(&mut self, addr: usize) -> u16 {
        self.read32(addr) as u16
    }
    fn write16(&mut self, addr: usize, val: u16) {
        self.write32(addr, val as u32)
    }
    fn udelay(&mut self, us: u32) {
        self.clock += us as u64;
    }
    fn now_us(&mut self) -> u64 {
        self.clock += 1;
        self.clock
    }
    fn log(&mut self, args: fmt::Arguments) {
        self.text.push_str(&args.to_string());
    }
}
