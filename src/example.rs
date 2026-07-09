//! SPI device mock for doc examples.

use core::convert::Infallible;

use crate::pec15::PEC15;
use embedded_hal::delay::DelayNs;
use embedded_hal::spi::{ErrorType, Operation, SpiDevice};

#[derive(Default)]
pub struct ExampleSPIDevice;

impl ErrorType for ExampleSPIDevice {
    type Error = Infallible;
}

impl SpiDevice<u8> for ExampleSPIDevice {
    fn transaction(&mut self, operations: &mut [Operation<'_, u8>]) -> Result<(), Self::Error> {
        for operation in operations {
            match operation {
                Operation::Transfer(read, write) => Self::transfer(read, write),
                Operation::Write(_) => {}
                Operation::Read(_) | Operation::TransferInPlace(_) | Operation::DelayNs(_) => {
                    panic!("unexpected SPI operation in doc example")
                }
            }
        }

        Ok(())
    }
}

impl ExampleSPIDevice {
    fn transfer(read: &mut [u8], write: &[u8]) {
        if write.len() < 7 || write[0] != 0xFE {
            return;
        }

        let addr = write[1];
        let data_len = write.len() - 7;
        let data = Self::data(addr, data_len);

        read.fill(0);
        read[5..5 + data_len].copy_from_slice(&data[..data_len]);
        let pec = PEC15::calc(&data[..data_len]);
        read[5 + data_len] = pec[0];
        read[6 + data_len] = pec[1];
    }

    fn data(addr: u8, len: usize) -> [u8; 16] {
        let mut data = [0u8; 16];
        match addr {
            0x80 => data[0] = 0x00,                           // STATUS: no flags set.
            0x90 => data[..3].copy_from_slice(&[0, 0, 0]),    // I1.
            0xA0 => data[..2].copy_from_slice(&[0x30, 0x39]), // BAT: 12_345 raw -> 4_629_375 uV.
            0xA6 => data[..2].copy_from_slice(&[0x00, 0x7D]), // SLOT1: 125 raw -> 25.0 C.
            0x00 => data[..6].copy_from_slice(&[0; 6]),       // C1.
            0xF7 if len >= 6 => {
                data[..6].copy_from_slice(&[0x01, 0x00, 0x00, 0x00, 0x00, 0x80]);
            }
            0xF7 if len >= 3 => data[..3].copy_from_slice(&[0, 0, 0x80]), // FIFO terminator.
            _ => {}
        }
        data
    }
}

#[derive(Default)]
pub struct ExampleDelay {
    elapsed_us: u64,
}

impl ExampleDelay {
    pub fn elapsed_us(&self) -> u64 {
        self.elapsed_us
    }
}

impl DelayNs for ExampleDelay {
    fn delay_ns(&mut self, ns: u32) {
        self.elapsed_us += u64::from(ns.div_ceil(1_000));
    }

    fn delay_us(&mut self, us: u32) {
        self.elapsed_us += u64::from(us);
    }

    fn delay_ms(&mut self, ms: u32) {
        self.elapsed_us += u64::from(ms) * 1_000;
    }
}
