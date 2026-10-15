//! Minimal flattened device tree writer, enough to describe memory, PSCI
//! and the console to the payload: the shared `fdt-writer` crate with the
//! A733's capacities (4096-byte structure, 1024-byte strings block).

pub use fdt_writer::Fdt;
