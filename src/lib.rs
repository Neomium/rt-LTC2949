//! # Generic client for LTC2949 battery stack monitor
#![cfg_attr(not(test), no_std)]
#![cfg_attr(feature = "strict", deny(warnings))]

#[cfg(test)]
extern crate alloc;

pub use heapless;

pub mod client;
pub mod polling;
pub mod spi;

pub(crate) mod pec15;

#[cfg(test)]
mod mocks;
#[cfg(test)]
mod tests;
