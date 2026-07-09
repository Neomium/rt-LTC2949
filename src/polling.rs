use crate::spi::LatchingSpiDevice;
use embedded_hal::digital::OutputPin;
use embedded_hal::spi::{SpiBus, SpiDevice};

/// Poll Strategy
pub trait PollMethod<B: SpiDevice> {
    /// Gets called by synchronous commands, which not require any waiting/polling (e.g. writing registers)
    fn end_sync_command(&self, bus: &mut B) -> Result<(), B::Error>;
}

/// Leaves CS Low and waits until SDO goes high
pub struct SDOLinePolling {}

impl<B: SpiBus, CS: OutputPin> PollMethod<LatchingSpiDevice<B, CS>> for SDOLinePolling {
    fn end_sync_command(&self, bus: &mut LatchingSpiDevice<B, CS>) -> Result<(), crate::spi::Error<B, CS>> {
        bus.release_cs()
    }
}

/// No ADC polling is used
pub struct NoPolling {}

impl<B: SpiDevice> PollMethod<B> for NoPolling {
    fn end_sync_command(&self, _bus: &mut B) -> Result<(), B::Error> {
        Ok(())
    }
}
