//! Polling policy used after LTC2949 SPI commands.
//!
//! The LTC2949 driver keeps conversion timing host-owned. Operations such as
//! [`Client::start_wake_up`](crate::client::Client::start_wake_up) return the wait time that
//! the application must observe with its own timer, RTOS primitive, or delay provider.
//! [`PollMethod`] covers a narrower concern: what the SPI wrapper should do at the end of a
//! synchronous command transaction.
//!
//! Two policies are provided:
//!
//! * [`NoPolling`] is the default policy used by [`LTC2949::new`](crate::client::LTC2949::new).
//!   It assumes the underlying [`SpiDevice`] owns chip-select handling and has no extra
//!   end-of-command work to do.
//! * [`SDOLinePolling`] is for [`LatchingSpiDevice`]. That SPI wrapper keeps CS asserted across
//!   an operation; this policy releases CS when the command finishes. It is the shape needed for
//!   SDO-line polling style integrations, where chip-select ownership must be explicit.
//!
//! `PollMethod` does not itself sleep for conversion times. If a client method returns a wait,
//! call your platform delay/timer explicitly before the next phase.
//!
//! ```
//! use embedded_hal::delay::DelayNs;
//! use ltc2949::client::{Client, LTC2949, T_BOOT_US};
//! use ltc2949::example::{ExampleDelay, ExampleSPIDevice};
//!
//! let spi = ExampleSPIDevice::default();
//! let mut delay = ExampleDelay::default();
//! let mut client = LTC2949::new(spi); // Uses NoPolling.
//!
//! let boot_us = client.start_wake_up().unwrap();
//! assert_eq!(T_BOOT_US, boot_us);
//! delay.delay_us(boot_us);
//! client.confirm_wake_up().unwrap();
//! ```
use crate::spi::LatchingSpiDevice;
use embedded_hal::digital::OutputPin;
use embedded_hal::spi::{SpiBus, SpiDevice};

/// End-of-command SPI policy.
pub trait PollMethod<B: SpiDevice> {
    /// Called after synchronous commands so the selected SPI policy can finalize the
    /// transaction. This is separate from device conversion waits returned by high-level
    /// client methods.
    fn end_sync_command(&self, bus: &mut B) -> Result<(), B::Error>;
}

/// Releases a [`LatchingSpiDevice`] chip-select at the end of each synchronous command.
pub struct SDOLinePolling {}

impl<B: SpiBus, CS: OutputPin> PollMethod<LatchingSpiDevice<B, CS>> for SDOLinePolling {
    fn end_sync_command(&self, bus: &mut LatchingSpiDevice<B, CS>) -> Result<(), crate::spi::Error<B, CS>> {
        bus.release_cs()
    }
}

/// No extra end-of-command handling.
pub struct NoPolling {}

impl<B: SpiDevice> PollMethod<B> for NoPolling {
    fn end_sync_command(&self, _bus: &mut B) -> Result<(), B::Error> {
        Ok(())
    }
}
