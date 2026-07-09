use embedded_hal::digital::{ErrorType, OutputPin};
use embedded_hal::spi::{Error, ErrorKind, Operation, SpiBus, SpiDevice};
use mockall::mock;

#[derive(Debug, PartialEq, Eq)]
pub enum BusError {
    Error1,
}

mock! {
    pub SPIDevice {}

    impl embedded_hal::spi::ErrorType for SPIDevice { type Error = BusError; }

    impl SpiDevice<u8> for SPIDevice{
        fn transaction<'a>(&mut self, operations: &mut [Operation<'a, u8>]) -> Result<(), BusError>;
    }
}

mock! {
    pub SPIBus {}

    impl embedded_hal::spi::ErrorType for SPIBus { type Error = BusError; }

    impl SpiBus<u8> for SPIBus{
        fn read(&mut self, words: &mut [u8]) -> Result<(), BusError>;
        fn write(&mut self, words: &[u8]) -> Result<(), BusError>;
        fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), BusError>;
        fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), BusError>;
        fn flush(&mut self) -> Result<(), BusError>;
    }
}

impl Error for BusError {
    fn kind(&self) -> ErrorKind {
        ErrorKind::Other
    }
}

#[derive(Debug, PartialEq, Eq)]
pub enum PinError {
    Error1,
}

mock! {
    pub Pin {}

    impl ErrorType for Pin { type Error = PinError; }

    impl OutputPin for Pin {
        fn set_low(&mut self) -> Result<(), PinError>;
        fn set_high(&mut self) -> Result<(), PinError>;
    }
}

impl embedded_hal::digital::Error for PinError {
    fn kind(&self) -> embedded_hal::digital::ErrorKind {
        embedded_hal::digital::ErrorKind::Other
    }
}
