use log::{Error, Serial};

// The 16550 register map and transmit path are the shared uart_16550 crate
// (ore-edk2-boot); the K1 keeps the port (UART0, 32-bit registers), which the
// BootROM has already configured, and the log glue below.
// U-Boot arch/riscv/dts/k1-x.dtsi, arch/riscv/dts/k1-x_spl.dts
const UART0_BASE: usize = 0xd401_7000;
const UART0: uart_16550::Uart16550 = uart_16550::Uart16550::attach(UART0_BASE, 4);

#[derive(Debug)]
pub struct K1XSerial();

impl K1XSerial {
    #[inline]
    pub fn noinit() -> Self {
        Self()
    }
}

impl Serial for K1XSerial {}

impl embedded_hal_nb::serial::ErrorType for K1XSerial {
    type Error = Error;
}

impl embedded_hal_nb::serial::Write<u8> for K1XSerial {
    #[inline]
    fn write(&mut self, c: u8) -> nb::Result<(), self::Error> {
        if UART0.try_write(&mut crate::k1io::K1Io, c) {
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    #[inline]
    fn flush(&mut self) -> nb::Result<(), self::Error> {
        let tfe_empty = true;
        if tfe_empty {
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}
