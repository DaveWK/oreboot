//! LPDDR5 DRAM initialisation for the Allwinner A733 on the Radxa Cubie
//! A7A, written from the register-level behaviour of Allwinner's DRAM
//! library (recorded by the emulator harness in ore-edk-boot-radxa-a7a
//! `tools/dram-harness`), the A733 manual, the DesignWare controller and PHY
//! register conventions and JEDEC LPDDR5/5X. It links no vendor code; the PHY's
//! training firmware is loaded as data (see [`phy`]).
#![no_std]

pub mod ccu;
pub mod ctrl;
pub mod hw;
pub mod mr;
pub mod para;
pub mod phy;
pub mod post;
pub mod power;
pub mod sets;
pub mod size;
pub mod timing;

pub use hw::{Error, Hw, Rail};
pub use para::Para;
