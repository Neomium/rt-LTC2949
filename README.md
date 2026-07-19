# LTC2949 Rust driver

[![Crates.io](https://img.shields.io/crates/v/ltc2949.svg)](https://crates.io/crates/ltc2949)
[![Documentation](https://docs.rs/ltc2949/badge.svg)](https://docs.rs/ltc2949)
[![QA](https://github.com/neomium/rt-LTC2949/actions/workflows/qa.yaml/badge.svg)](https://github.com/neomium/rt-LTC2949/actions/workflows/qa.yaml)
[![License: MIT OR Apache-2.0](https://img.shields.io/badge/license-MIT%20OR%20Apache--2.0-blue.svg)](#license)

`ltc2949` is a `no_std` Rust driver for the Analog Devices
[LTC2949](https://www.analog.com/en/products/ltc2949.html) current, voltage,
charge, and energy monitor. It uses the `embedded-hal` SPI traits and supports
an LTC2949 on its own or connected in parallel with an LTC68xx cell-monitor
daisy chain on the same isoSPI bus.

The driver provides register configuration, slow and fast measurement access,
accumulator readings, status and fault information, FIFO handling, and NTC and
shunt-temperature compensation. Required waits are returned to the application
instead of blocking inside the driver.

## Example

The following function wakes the monitor, starts continuous measurement, and
reads the battery voltage:

```rust
use embedded_hal::{delay::DelayNs, spi::SpiDevice};
use ltc2949::client::{Client, Error, LTC2949, OpsControlRegister};

fn read_battery_voltage<SPI, D>(spi: SPI, delay: &mut D) -> Result<f32, Error<SPI>>
where
    SPI: SpiDevice<u8>,
    D: DelayNs,
{
    let mut monitor = LTC2949::new(spi);

    let boot_time_us = monitor.start_wake_up()?;
    delay.delay_us(boot_time_us);
    monitor.confirm_wake_up()?;

    monitor.write_opctrl(OpsControlRegister::default().with_cont(true))?;
    delay.delay_ms(100);

    Ok(monitor.read_bat()?.decode())
}
```

See the [Rust documentation](https://docs.rs/ltc2949) for detailed examples,
configuration options, timing requirements, and shared-bus usage with LTC68xx
cell monitors.

## Contributing

Contributions are welcome. Before opening a pull request, please:

- run the test suite with `cargo test`;
- format the code with `cargo fmt`;
- add or update tests for behavioral changes.

For the complete contribution guidelines, see [DEVELOPMENT.md](DEVELOPMENT.md).

## License

This project is dual-licensed under either of the following licenses, at your
option:

- [Apache License, Version 2.0](LICENSE-APACHE)
- [MIT License](LICENSE-MIT)
