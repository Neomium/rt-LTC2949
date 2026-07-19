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
            0x00 => {
                // C1 = 10,000,000; E1 = 10,000; TB1 = 10,000.
                let row = [
                    0x00, 0x00, 0x00, 0x98, 0x96, 0x80, 0x00, 0x00, 0x00, 0x00, 0x27, 0x10, 0x00, 0x00, 0x27, 0x10,
                ];
                data[..len].copy_from_slice(&row[..len]);
            }
            0x06 => data[..6].copy_from_slice(&[0x00, 0x00, 0x00, 0x00, 0x27, 0x10]), // E1 = 10,000.
            0x0C => data[..4].copy_from_slice(&[0x00, 0x00, 0x27, 0x10]),             // TB1 = 10,000.
            0x10 => {
                // C2 = -6,000,000; E2 = -6,000; TB2 = 12,000.
                let row = [
                    0xFF, 0xFF, 0xFF, 0xA4, 0x72, 0x80, 0xFF, 0xFF, 0xFF, 0xFF, 0xE8, 0x90, 0x00, 0x00, 0x2E, 0xE0,
                ];
                data[..len].copy_from_slice(&row[..len]);
            }
            0x16 => data[..6].copy_from_slice(&[0xFF, 0xFF, 0xFF, 0xFF, 0xE8, 0x90]), // E2 = -6,000.
            0x1C => data[..4].copy_from_slice(&[0x00, 0x00, 0x2E, 0xE0]),             // TB2 = 12,000.
            0x24 => data[..8].copy_from_slice(&[0x00, 0x00, 0x00, 0x00, 0x00, 0x3D, 0x09, 0x00]), // C3 = 4,000,000.
            0x2C => data[..4].copy_from_slice(&[0x00, 0x00, 0x3A, 0x98]),             // TB3 = 15,000.
            0x3C => data[..4].copy_from_slice(&[0x00, 0x00, 0x4E, 0x20]),             // TB4 = 20,000.
            0x80 => data[0] = 0x10, // STATUS: result registers updated, no errors.
            0x90 => data[..3].copy_from_slice(&[0x00, 0x03, 0xE8]), // I1 = 1,000 -> 9.5 A at 100 uOhm.
            0x93 => data[..3].copy_from_slice(&[0x0F, 0x42, 0x40]), // P1 = 1,000,000.
            0x96 => data[..3].copy_from_slice(&[0xFF, 0xFE, 0x0C]), // I2 = -500 -> -4.75 A at 100 uOhm.
            0x99 => data[..3].copy_from_slice(&[0xF8, 0x5E, 0xE0]), // P2 = -500,000.
            0x9C => data[..3].copy_from_slice(&[0x00, 0x0F, 0xA0]), // I1AVG = 4,000 -> 9.5 A.
            0xA0 => data[..2].copy_from_slice(&[0x30, 0x39]), // BAT: 12_345 raw -> 4_629_375 uV.
            0xA2 => data[..2].copy_from_slice(&[0x05, 0xD3]), // TEMP: 1,491 raw -> 25.05 C.
            0xA4 => data[..2].copy_from_slice(&[0x05, 0xB4]), // VCC: 1,460 raw -> 3.2996 V.
            0xA6 => data[..2].copy_from_slice(&[0x00, 0x7D]), // SLOT1: 125 raw -> 25.0 C.
            0xAC => data[..3].copy_from_slice(&[0xFF, 0xF8, 0x30]), // I2AVG = -2,000 -> -4.75 A.
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
