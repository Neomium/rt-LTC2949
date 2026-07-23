//! # LTC2949 direct-register client.
//!
//! This module contains the high-level [`Client`] trait, the concrete [`LTC2949`] client,
//! register-oriented configuration types, result readers, and helper constants for
//! host-owned timing.
//!
//! ## Timing
//!
//! The driver never blocks. Operations with mandatory settling or acknowledge time are
//! split into non-blocking phases: the first call starts the bus/device action and returns
//! the number of microseconds the host must wait before issuing the next phase. This mirrors
//! the `CommandTime` pattern used by the LTC681X cell-monitor client while keeping delay,
//! timer, RTOS, or scheduler ownership in the application.
//!
//! Public timing constants are exposed so code can assert, schedule, or budget those waits
//! explicitly:
//!
//! * [`T_BOOT_US`] is returned by [`Client::start_wake_up`] before
//!   [`Client::confirm_wake_up`].
//! * [`T_READY_US`] is returned by [`Client::wake_isospi`] before the next transaction.
//! * [`T_MLCK_US`] is returned by [`Client::request_memory_lock`] before reading a coherent
//!   locked snapshot.
//!
//! ```
//! use embedded_hal::delay::DelayNs;
//! use ltc2949::client::{Client, LTC2949, T_BOOT_US, T_MLCK_US, T_READY_US};
//! use ltc2949::example::{ExampleDelay, ExampleSPIDevice};
//!
//! let spi = ExampleSPIDevice::default();
//! let mut delay = ExampleDelay::default();
//! let mut client = LTC2949::new(spi);
//!
//! let ready_us = client.wake_isospi().unwrap();
//! assert_eq!(T_READY_US, ready_us);
//! delay.delay_us(ready_us);
//!
//! let boot_us = client.start_wake_up().unwrap();
//! assert_eq!(T_BOOT_US, boot_us);
//! delay.delay_us(boot_us);
//! client.confirm_wake_up().unwrap();
//!
//! let lock_us = client.request_memory_lock().unwrap();
//! assert_eq!(T_MLCK_US, lock_us);
//! delay.delay_us(lock_us);
//! let _charge1 = client.read_charge1().unwrap();
//! client.unlock_memory().unwrap();
//!
//! assert_eq!(
//!     u64::from(T_READY_US) + u64::from(T_BOOT_US) + u64::from(T_MLCK_US),
//!     delay.elapsed_us()
//! );
//! ```
//!
//! ## Fast control register
//!
//! [`Client::write_factrl`] selects which current and AUX channels participate in fast
//! conversion. With `FACONV` set, conversions run continuously and place their results in the
//! corresponding FIFOs. Leave `FACONV` clear when an ADCV-style command should trigger a
//! single conversion instead.
//!
//! ```
//! # use ltc2949::client::{Client, FastControlRegister, LTC2949};
//! # use ltc2949::example::ExampleSPIDevice;
//! let mut client = LTC2949::new(ExampleSPIDevice::default());
//! let fast_channels = FastControlRegister::default()
//!     .with_faconv(true)
//!     .with_fach1(true)
//!     .with_fach2(true);
//!
//! client.write_factrl(fast_channels).unwrap();
//! ```
//!
//! ## ADC configuration register
//!
//! [`Client::write_adcconf`] configures power-result interpretation, NTC linearisation, and
//! the NTC source used for shunt temperature compensation. The PAGE1 write becomes active
//! only after an `ADJUPD` pulse while the LTC2949 is in STANDBY (`CONT = 0`).
//!
//! ```
//! # use ltc2949::client::{AdcConfiguration, Client, LTC2949, OpsControlRegister};
//! # use ltc2949::example::ExampleSPIDevice;
//! let mut client = LTC2949::new(ExampleSPIDevice::default());
//! let adc_configuration = AdcConfiguration::default()
//!     .with_p1asv(true) // P1 contains a voltage result instead of power.
//!     .with_ntc1(true); // SLOT1 contains the NTC1 temperature result.
//!
//! client.write_adcconf(adc_configuration).unwrap();
//! client
//!     .write_opctrl(OpsControlRegister::default().with_adjupd(true))
//!     .unwrap();
//! ```
//!
//! ## NTC coefficients
//!
//! [`Client::write_ntc_coefficients`] stores the reference resistor and Steinhart–Hart
//! coefficients for either NTC channel as Float24 values. The coefficients below are a
//! realistic 10 kΩ NTC configuration. Route the matching SLOT input, enable `NTC1` or `NTC2`
//! in [`AdcConfiguration`], and issue `ADJUPD` before using the temperature result.
//!
//! ```
//! # use ltc2949::client::{Channel, Client, LTC2949, NtcConfig};
//! # use ltc2949::example::ExampleSPIDevice;
//! let mut client = LTC2949::new(ExampleSPIDevice::default());
//! let ntc = NtcConfig {
//!     r_ref: 10_000.0,
//!     a: 1.1382e-3,
//!     b: 2.3267e-4,
//!     c: 0.93243e-7,
//! };
//!
//! client.write_ntc_coefficients(Channel::One, &ntc).unwrap();
//! ```
//!
//! ## Sense-resistor temperature compensation
//!
//! [`Client::write_shunt_tc`] programs the linear and quadratic temperature coefficients and
//! the nominal reference temperature for one sense resistor. This copper example uses
//! 3900 ppm/K at 25 °C and no quadratic correction. The values become active after `ADJUPD`
//! and require a configured NTC temperature source.
//!
//! ```
//! # use ltc2949::client::{Channel, Client, LTC2949, ShuntTcConfig};
//! # use ltc2949::example::ExampleSPIDevice;
//! let mut client = LTC2949::new(ExampleSPIDevice::default());
//! let copper_shunt = ShuntTcConfig {
//!     tc: 0.0039,
//!     t_ref: 25.0,
//!     tc2: 0.0,
//! };
//!
//! client.write_shunt_tc(Channel::One, &copper_shunt).unwrap();
//! ```
//!
//! ## SLOT multiplexer configuration
//!
//! [`Client::write_slot_mux`] routes a negative and positive input to SLOT1 or SLOT2. A
//! typical NTC divider connects its measurement node to `V1`; selecting `AGND` as `MUXN` and
//! `V1` as `MUXP` measures that node single-ended for NTC1 linearisation.
//!
//! ```
//! # use ltc2949::client::{Channel, Client, LTC2949, MuxInput};
//! # use ltc2949::example::ExampleSPIDevice;
//! let mut client = LTC2949::new(ExampleSPIDevice::default());
//!
//! client
//!     .write_slot_mux(Channel::One, MuxInput::Agnd, MuxInput::V1)
//!     .unwrap();
//! ```
//!
//! ## GPIO control
//!
//! [`Client::write_gpio_ctrl`] writes the raw `FGPIOCTRL` byte, which contains four 2-bit GPIO
//! control fields. In this example `GPIO1CTRL = 0b11` drives GPIO1 high while the remaining
//! fields stay `0b00` (tristate).
//!
//! ```
//! # use ltc2949::client::{Client, LTC2949};
//! # use ltc2949::example::ExampleSPIDevice;
//! let mut client = LTC2949::new(ExampleSPIDevice::default());
//! let gpio_control = 0b00_00_00_11;
//!
//! client.write_gpio_ctrl(gpio_control).unwrap();
//! ```
//!
//! ## Overcurrent configuration
//!
//! [`Client::write_occ_config`] configures both hardware overcurrent comparators. Thresholds
//! select a differential shunt voltage rather than amperes; divide by the shunt resistance
//! to obtain the current limit. With a 100 µΩ shunt, the examples below correspond to +260 A
//! and −520 A limits with 320 µs and 80 µs deglitch times.
//!
//! ```
//! # use ltc2949::client::{Client, LTC2949, OverCurrentConfig};
//! # use ltc2949::example::ExampleSPIDevice;
//! let mut client = LTC2949::new(ExampleSPIDevice::default());
//! let positive_limit = OverCurrentConfig {
//!     enable: true,
//!     threshold: 0b001,    // 26 mV / 100 µΩ = 260 A
//!     deglitch_time: 0b10, // 320 µs
//!     polarity: 0b01,      // positive current only
//! };
//! let negative_limit = OverCurrentConfig {
//!     enable: true,
//!     threshold: 0b010,    // 52 mV / 100 µΩ = 520 A
//!     deglitch_time: 0b01, // 80 µs
//!     polarity: 0b10,      // negative current only
//! };
//!
//! client.write_occ_config(positive_limit, negative_limit).unwrap();
//! ```
//!
//! ## ADAX with an LTC681X chain
//!
//! The LTC2949 and the cell-monitor client need separate [`SpiDevice`] handles backed by
//! the same physical isoSPI bus. Sending [`Client::trigger_adax`] through the LTC2949 handle
//! broadcasts ADAX (`0x0460`), so it also starts an AUX conversion on every LTC6813X in the
//! chain.
//!
//! For the LTC2949 to react, `CONT` must be set and `FACONV` must remain clear. The command
//! starts a fast single-shot conversion of the channels selected in [`FastControlRegister`]
//! and clears the LTC2949 FIFOs before conversion.
//!
//! ```
//!# use ltc2949::client::{Client, FastControlRegister, LTC2949, OpsControlRegister};
//!# use ltc2949::example::ExampleSPIDevice;
//!# use ltc681x::ltc6813::LTC6813;
//!# use ltc681x::monitor::{LTC681X, NoPolling};
//!#
//! // On hardware, create both handles from one shared isoSPI bus.
//! let meter_spi = ExampleSPIDevice::default();
//! let chain_spi = ExampleSPIDevice::default();
//!
//! let mut meter = LTC2949::new(meter_spi);
//! let _chain: LTC681X<_, NoPolling, LTC6813, 3> = LTC681X::ltc6813(chain_spi);
//!
//! meter
//!     .write_opctrl(OpsControlRegister::default().with_cont(true))
//!     .unwrap();
//! meter
//!     .write_factrl(FastControlRegister::default().with_facha(true))
//!     .unwrap();
//!
//! // One broadcast starts the LTC2949 fast channels and all LTC6813 AUX conversions.
//! meter.trigger_adax().unwrap();
//! ```
//!
//! ## Status and fault monitoring
//!
//! [`Client::read_status`] returns the decoded `STATUS` register. Its accessors distinguish
//! supply and reset events (`uvloa`, `pora`, `uvlostby`, `uvlod`), a completed result update,
//! and ADC or time-base errors. The SPI transaction can still fail independently, so handle
//! the returned [`Result`] before inspecting individual flags.
//!
//! ```
//! # use ltc2949::client::{Client, LTC2949};
//! # use ltc2949::example::ExampleSPIDevice;
//!
//! let mut client = LTC2949::new(ExampleSPIDevice::default());
//! let status = client.read_status().unwrap();
//!
//! // The example device reports a nominal status with no conversion errors.
//! assert!(!status.adcerr());
//! assert!(!status.tberr());
//! assert!(status.update()); // Result registers contain a new measurement cycle.
//! ```
//!
//! [`Client::read_faults`] decodes the main `FAULTS` register into named hardware,
//! communication, fast-acquisition, self-test, and CRC indicators. Check these flags after
//! start-up and whenever `STATUS` reports an error; [`Client::read_extfaults`] provides the
//! additional memory and fast-channel diagnostics from `EXTFAULTS`.
//!
//! ```
//! # use ltc2949::client::{Client, LTC2949};
//! # use ltc2949::example::ExampleSPIDevice;
//!
//! let mut client = LTC2949::new(ExampleSPIDevice::default());
//! let faults = client.read_faults().unwrap();
//!
//! let configuration_is_valid = !faults.crccfg() && !faults.crcmem();
//! let hardware_self_test_passed = !faults.hwbist();
//! assert!(configuration_is_valid);
//! assert!(hardware_self_test_passed);
//! ```
//!
//! ## Threshold monitoring
//!
//! The alert readers decode the PAGE0 threshold and overflow status registers into named
//! flags. [`Client::read_vt_alerts`] covers battery voltage, temperature, and the two slow
//! AUX slots; [`Client::read_ip_alerts`] covers current and power; and
//! [`Client::read_c_alerts`] covers accumulated charge. [`Client::read_ceof_alerts`],
//! [`Client::read_tb_alerts`], and [`Client::read_vcc_alerts`] report accumulator overflow,
//! time-base, supply, and overcurrent-comparator alerts.
//!
//! Alert bits are sticky read/write flags. Follow the datasheet's memory-lock procedure
//! before clearing alert registers so that an alert arriving during the clear is not lost.
//!
//! ```
//! # use ltc2949::client::{Client, LTC2949};
//! # use ltc2949::example::ExampleSPIDevice;
//!
//! let mut client = LTC2949::new(ExampleSPIDevice::default());
//! let vt_alerts = client.read_vt_alerts().unwrap();
//!
//! // The example device reports a nominal status with no thresholds exceeded.
//! assert!(!vt_alerts.bath());
//! assert!(!vt_alerts.temph());
//! assert!(!vt_alerts.slot1h());
//! ```
//!
//! PAGE1 threshold setters accept physical SI values and round to the nearest register code
//! using the scales in datasheet Tables 26–28. Current, power, charge, and energy setters
//! also take the shunt resistance so callers can use amperes, watts, coulombs, and joules
//! instead of the IC's intermediate V/V² units. Values that do not fit return
//! [`Error::InvalidThreshold`].
//!
//! ```
//! # use ltc2949::client::{AccumulatorClock, ChargeAccumulator, Channel, Client, LTC2949};
//! # use ltc2949::example::ExampleSPIDevice;
//! let mut client = LTC2949::new(ExampleSPIDevice::default());
//! let shunt_ohms = 100e-6;
//!
//! client
//!     .write_current_thresholds(Channel::One, -100.0, 100.0, shunt_ohms)
//!     .unwrap();
//! client.write_battery_thresholds(0.0, 10.0).unwrap();
//! client
//!     .write_charge_thresholds(
//!         ChargeAccumulator::Charge1,
//!         -100.0,
//!         100.0,
//!         shunt_ohms as f64,
//!         AccumulatorClock::Internal,
//!     )
//!     .unwrap();
//! ```
//!
//! ## Current measurements
//!
//! [`Client::read_current1`] and [`Client::read_current2`] return the slow-mode voltage
//! measured across the corresponding sense resistor. Divide the decoded voltage by the
//! shunt resistance to obtain amperes. [`Client::read_current1_avg`] and
//! [`Client::read_current2_avg`] expose the moving average of the four preceding measurements
//! with a four-times finer LSB.
//!
//! ```
//! # use ltc2949::client::{Client, LTC2949};
//! # use ltc2949::example::ExampleSPIDevice;
//! let mut client = LTC2949::new(ExampleSPIDevice::default());
//! let shunt_ohms = 100e-6_f32;
//!
//! let current1 = client.read_current1().unwrap();
//! let current2 = client.read_current2().unwrap();
//! let current1_avg = client.read_current1_avg().unwrap();
//! let current2_avg = client.read_current2_avg().unwrap();
//!
//! let current1_amps = current1.decode() / shunt_ohms;
//! let current2_amps = current2.decode() / shunt_ohms;
//! let current1_avg_amps = current1_avg.decode() / shunt_ohms;
//! let current2_avg_amps = current2_avg.decode() / shunt_ohms;
//! assert!((current1_amps - 9.5).abs() < 0.001);
//! assert!((current2_amps + 4.75).abs() < 0.001);
//! assert!((current1_avg_amps - 9.5).abs() < 0.001);
//! assert!((current2_avg_amps + 4.75).abs() < 0.001);
//! ```
//!
//! ## Power and battery voltage
//!
//! [`Client::read_power1`] and [`Client::read_power2`] return power-mode results by default.
//! Decode them with the corresponding shunt resistance. If `P1ASV` or `P2ASV` is enabled in
//! [`AdcConfiguration`], the same registers contain voltage instead and must be decoded with
//! [`PowerOrVoltage::decode_voltage`]. [`Client::read_bat`] always returns the pack voltage.
//!
//! ```
//! # use ltc2949::client::{Client, LTC2949};
//! # use ltc2949::example::ExampleSPIDevice;
//! let mut client = LTC2949::new(ExampleSPIDevice::default());
//! let shunt_ohms = 100e-6_f32;
//!
//! let power1_watts = client.read_power1().unwrap().decode_power(shunt_ohms);
//! let power2_watts = client.read_power2().unwrap().decode_power(shunt_ohms);
//! let battery = client.read_bat().unwrap();
//!
//! assert!((power1_watts - 0.058_368).abs() < 0.000_001);
//! assert!((power2_watts + 0.029_184).abs() < 0.000_001);
//! assert_eq!(12_345, battery.raw());
//! assert_eq!(4_629_375, (battery.decode() * 1_000_000.0) as i32);
//! ```
//!
//! ## Temperature and supply voltage
//!
//! [`Client::read_temp`] provides the internal die temperature in kelvin or degrees Celsius.
//! [`Client::read_vcc`] reports the shared analog/digital supply voltage. Both result types
//! retain the raw register value and provide unit-aware decoding methods.
//!
//! ```
//! # use ltc2949::client::{Client, LTC2949};
//! # use ltc2949::example::ExampleSPIDevice;
//! let mut client = LTC2949::new(ExampleSPIDevice::default());
//!
//! let die_temperature = client.read_temp().unwrap();
//! let supply_voltage = client.read_vcc().unwrap();
//!
//! let temperature_celsius = die_temperature.decode_celsius();
//! let vcc_volts = supply_voltage.decode();
//! assert!((temperature_celsius - 25.05).abs() < 0.01);
//! assert!((vcc_volts - 3.2996).abs() < 0.0001);
//! ```
//!
//! ## Charge accumulators
//!
//! [`Client::read_charge1`] and [`Client::read_charge2`] read the signed 48-bit channel
//! accumulators. [`Client::read_charge3`] reads the signed 64-bit weighted channel sum. For
//! the internal clock or a 4 MHz crystal, [`AccumulatedCharge::decode_coulombs`] converts the
//! raw value using the external shunt resistance; a freely configured external clock needs
//! the scale from datasheet Table 27.
//!
//! ```
//! # use ltc2949::client::{Client, LTC2949};
//! # use ltc2949::example::ExampleSPIDevice;
//! let mut client = LTC2949::new(ExampleSPIDevice::default());
//! let shunt1_ohms = 100e-6_f64;
//! let shunt2_ohms = 100e-6_f64;
//!
//! let charge1 = client.read_charge1().unwrap();
//! let charge2 = client.read_charge2().unwrap();
//! let charge3 = client.read_charge3().unwrap();
//!
//! let channel1_coulombs = charge1.decode_coulombs(shunt1_ohms);
//! let channel2_coulombs = charge2.decode_coulombs(shunt2_ohms);
//! // The weighted C3 sum uses the channel-1 shunt for conversion.
//! let weighted_coulombs = charge3.decode_coulombs(shunt1_ohms);
//! assert!((channel1_coulombs - 37.7887).abs() < 0.0001);
//! assert!((channel2_coulombs + 22.67322).abs() < 0.0001);
//! assert!((weighted_coulombs - 15.11548).abs() < 0.0001);
//! ```
//!
//! ## Time-base accumulators
//!
//! [`Client::read_time1`] through [`Client::read_time4`] return unsigned 32-bit time-base
//! counters. [`AccumulatedTime::decode`] converts them to seconds for the internal clock or a
//! 4 MHz crystal; use the datasheet formula when the external clock is configured freely.
//!
//! ```
//! # use ltc2949::client::{Client, LTC2949};
//! # use ltc2949::example::ExampleSPIDevice;
//! let mut client = LTC2949::new(ExampleSPIDevice::default());
//!
//! let time1 = client.read_time1().unwrap();
//! let time2 = client.read_time2().unwrap();
//! let time3 = client.read_time3().unwrap();
//! let time4 = client.read_time4().unwrap();
//!
//! assert!((time1.decode() - 3.97777).abs() < 0.00001);
//! assert!((time2.decode() - 4.773324).abs() < 0.00001);
//! assert!((time3.decode() - 5.966655).abs() < 0.00001);
//! assert!((time4.decode() - 7.95554).abs() < 0.00001);
//! ```
//!
//! ## Coherent accumulator snapshots
//!
//! [`Client::read_accumulators1`] and [`Client::read_accumulators2`] each read charge,
//! energy, and time in one 16-byte burst. Because all three values come from one register
//! row, they form a coherent per-channel snapshot without a separate memory-lock handshake.
//!
//! ```
//! # use ltc2949::client::{Client, LTC2949};
//! # use ltc2949::example::ExampleSPIDevice;
//! let mut client = LTC2949::new(ExampleSPIDevice::default());
//!
//! let channel1 = client.read_accumulators1().unwrap();
//! let channel2 = client.read_accumulators2().unwrap();
//!
//! assert_eq!((10_000_000, 10_000, 10_000), (
//!     channel1.charge.raw(),
//!     channel1.energy.raw(),
//!     channel1.time.raw(),
//! ));
//! assert_eq!((-6_000_000, -6_000, 12_000), (
//!     channel2.charge.raw(),
//!     channel2.energy.raw(),
//!     channel2.time.raw(),
//! ));
//! ```
//!
// `modular-bitfield`'s macro expansion emits `pub field: (bool)` etc., which trips the
// `unused_parens` lint on newer rustc. Silence the lint module-wide.
#![allow(unused_parens)]

use crate::float24::Float24;
use crate::pec15::PEC15;
use crate::polling::{NoPolling, PollMethod};
use embedded_hal::spi::{Operation, SpiDevice};
use heapless::Vec;
use modular_bitfield::prelude::*;

/// Worst-case core boot time from SLEEP/power-up to STANDBY (datasheet tBOOT).
/// Returned by [`Client::start_wake_up`].
pub const T_BOOT_US: u32 = 100_000;

/// isoSPI port start-up time after a wake edge (datasheet tREADY, 10 µs; doubled for margin).
/// Returned by [`Client::wake_isospi`].
pub const T_READY_US: u32 = 20;

/// Worst-case memory-lock acknowledge time (datasheet tMLCK, MEASURE mode; 40 ms in
/// STANDBY). Returned by [`Client::request_memory_lock`].
pub const T_MLCK_US: u32 = 130_000;

/// Encoded ID byte carried by direct-command read and write frames (datasheet Table 12).
pub(crate) struct DcmdId {
    /// `true` for a read command (`RW = 1`, `!RW = 0`), `false` for a write command.
    read: bool,
    /// Four-bit PEC-count field; encodes the number of data bytes per PEC as `N - 1`.
    pecc: u8,
}

impl DcmdId {
    pub(crate) fn read(pecc: u8) -> Self {
        Self::new(true, pecc)
    }

    pub(crate) fn write(pecc: u8) -> Self {
        Self::new(false, pecc)
    }

    pub(crate) fn new(read: bool, pecc: u8) -> Self {
        Self {
            read,
            pecc: pecc & 0x0F,
        }
    }
}

impl From<DcmdId> for u8 {
    fn from(id: DcmdId) -> Self {
        let p3 = (id.pecc >> 3) & 1;
        let p2 = (id.pecc >> 2) & 1;
        let p1 = (id.pecc >> 1) & 1;
        let p0 = id.pecc & 1;
        let rw = u8::from(id.read);
        let not_rw = rw ^ 1;
        (rw << 7) | (not_rw << 6) | ((p3 ^ p2) << 5) | (p3 << 4) | (p2 << 3) | ((p1 ^ p0) << 2) | (p1 << 1) | p0
    }
}

/// Max data bytes covered by one PEC group (`PECC + 1`).
const N_PER_PEC: usize = 16;

/// Whole 3-byte FIFO samples that fit one PEC group (`16 / 3 = 5`). Reading several samples
/// per DCMD amortises the ~9-byte header/PEC overhead instead of paying it per sample.
const FIFO_SAMPLES_PER_BURST: usize = N_PER_PEC / 3;

/// Memory page selector. PAGE0 holds measurement results, status and control;
/// PAGE1 holds thresholds and configuration. Selected via the [`RegControlRegister`] register.
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub enum Page {
    Page0 = 0,
    Page1 = 1,
}

/// Page-0 register addresses: results, accumulators, status and control/fast-mode
/// registers (datasheet Tables 24, 26-28, 57-64). Discriminant = on-bus `RADDR` byte.
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[repr(u8)]
pub enum RegAddressP0 {
    Charge1 = 0x00, // 48-bit
    Energy1 = 0x06, // 48-bit
    Time1 = 0x0C,   // 32-bit
    Charge2 = 0x10,
    Energy2 = 0x16,
    Time2 = 0x1C,
    Charge3 = 0x24, // 64-bit
    Time3 = 0x2C,
    Energy4 = 0x34, // 64-bit
    Time4 = 0x3C,
    WkupAck = 0x70,
    Status = 0x80,
    StatVT = 0x81,
    StatIP = 0x82,
    StatC = 0x83,
    StatE = 0x84,
    StatCEOF = 0x85,
    StatTB = 0x86,
    StatVCC = 0x87,
    Current1 = 0x90, // 24-bit signed
    Power1 = 0x93,
    Current2 = 0x96,
    Power2 = 0x99,
    Current1Avg = 0x9C,
    Bat = 0xA0, // 16-bit
    Temp = 0xA2,
    Vcc = 0xA4,
    Slot1 = 0xA6,
    Slot2 = 0xA8,
    Vref = 0xAA,
    Current2Avg = 0xAC,
    // Slow-mode auxiliary-MUX slot selection (datasheet Tables 57 & 58). SLOT1/2 each
    // have separate MUXN / MUXP registers, adjacent so a 2-byte burst sets both.
    Slot1MuxN = 0xEB,
    Slot1MuxP = 0xEC,
    Slot2MuxN = 0xED,
    Slot2MuxP = 0xEE,
    ExtFaults = 0xDC,
    Occ1Ctrl = 0xDE,
    Occ2Ctrl = 0xDF,
    Faults = 0xDD,
    OpCtrl = 0xF0,
    FCurGpioCtrl = 0xF1,
    FGpioCtrl = 0xF2,
    FaMuxN = 0xF3,
    FaMuxP = 0xF4,
    FaCtrl = 0xF5,
    FifoI1 = 0xF7,
    FifoI2 = 0xF8,
    FifoBat = 0xF9,
    FifoAux = 0xFA,
    RdcvIAddr = 0xFC,
    RegsCtrl = 0xFF,
}

/// Page-1 register addresses: thresholds, gain correction, ADC config, NTC-linearisation,
/// and sense-resistor TC coefficient blocks (datasheet Tables 67, 69, 71, 72, 76).
/// Discriminant = on-bus `RADDR` byte.
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[repr(u8)]
pub enum RegAddressP1 {
    // Threshold Registers
    C1Th = 0x00,
    C1Tl = 0x06,
    Tb1Th = 0x0C,
    E1Th = 0x10,
    E1Tl = 0x16,
    C2Th = 0x20,
    C2Tl = 0x26,
    Tb2Th = 0x2C,
    E2Th = 0x30,
    E2Tl = 0x36,
    C3Th = 0x44,
    Tb3Th = 0x4C,
    C3Tl = 0x54,
    E4Th = 0x64,
    Tb4Th = 0x6C,
    E4Tl = 0x74,
    I1Th = 0x80,
    I1Tl = 0x82,
    P1Th = 0x84,
    P1Tl = 0x86,
    I2Th = 0x88,
    I2Tl = 0x8A,
    P2Th = 0x8C,
    P2Tl = 0x8E,
    BatTh = 0x90,
    BatTl = 0x92,
    TempTh = 0x94,
    TempTl = 0x96,
    VccTh = 0x98,
    VccTl = 0x9A,
    Slot1Th = 0xA0,
    Slot1Tl = 0xA2,
    Slot2Th = 0xA4,
    Slot2Tl = 0xA6,
    // 2nd-order sense-resistor TC (3 bytes Float24 each), tucked low on the page.
    Rs1Tc2 = 0x5C,
    Rs2Tc2 = 0x7C,
    // NTC linearisation — datasheet Table 71. All values are Float24 (3 bytes each,
    // MSB at the lowest address). NTC1A..C and NTC2A..C live contiguously, so a single
    // 9-byte burst writes all three coefficients per channel.
    Rref1 = 0xAA,
    Rref2 = 0xAD,
    Ntc1A = 0xD0,
    Ntc1B = 0xD3,
    Ntc1C = 0xD6,
    // Sense-resistor temperature compensation — RSnTC (3 bytes) and RSnT0 (2 bytes,
    // mantissa LSB implicitly 0) are adjacent so they go out in one burst.
    Rs1Tc = 0xD9,
    Rs1T0 = 0xDC,
    AdcConf = 0xDF,
    Ntc2A = 0xE0,
    Ntc2B = 0xE3,
    Ntc2C = 0xE6,
    Rs2Tc = 0xE9,
    Rs2T0 = 0xEC,
    // Gain configuration — Float24 factors and their MUX input assignments (Table 72).
    Rs1Gc = 0xB0,
    Rs2Gc = 0xB3,
    RsRatio = 0xB6,
    BatGc = 0xB9,
    Mux1Gc = 0xC0,
    Mux2Gc = 0xC3,
    Mux3Gc = 0xC6,
    Mux4Gc = 0xC9,
    MuxNset1 = 0xBC,
    MuxPset1 = 0xBD,
    MuxNset2 = 0xBE,
    MuxPset2 = 0xBF,
    MuxNset3 = 0xCC,
    MuxPset3 = 0xCD,
    MuxNset4 = 0xCE,
    MuxPset4 = 0xCF,
}

/// Raw I1/I2 slow-mode current-sense voltage result.
///
/// The LTC2949 stores this as a signed ADC code. [`decode`](Self::decode) converts it to
/// volts at the current sense inputs.
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub struct CurrentSenseVoltage {
    /// Raw signed 24-bit ADC code, sign-extended to `i32`.
    raw: i32,
}

impl CurrentSenseVoltage {
    /// Voltage represented by one raw ADC code.
    pub const LSB_VOLTS: f32 = 950e-9;
    /// Voltage represented by one I1TH/I1TL or I2TH/I2TL threshold code.
    ///
    /// The threshold registers contain the upper 16 bits of the effective 18-bit current
    /// result, so their LSB is four times the result-register LSB (3.8 µV).
    pub const THRESHOLD_LSB_VOLTS: f32 = 4.0 * Self::LSB_VOLTS;

    /// Wraps a raw signed 24-bit ADC code, sign-extended to `i32`.
    pub const fn from_raw(raw: i32) -> Self {
        Self { raw }
    }

    /// Returns the raw signed 24-bit ADC code, sign-extended to `i32`.
    pub fn raw(self) -> i32 {
        self.raw
    }

    /// Decodes the raw ADC code into current-sense voltage in volts.
    pub fn decode(self) -> f32 {
        self.raw as f32 * Self::LSB_VOLTS
    }
}

/// Raw I1AVG/I2AVG moving-average current-sense voltage result.
///
/// The averaged result has a 4x finer LSB than the unaveraged slow-mode current result.
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub struct AveragedCurrentSenseVoltage {
    /// Raw signed 24-bit ADC code, sign-extended to `i32`.
    raw: i32,
}

impl AveragedCurrentSenseVoltage {
    /// Voltage represented by one raw ADC code.
    pub const LSB_VOLTS: f32 = 237.5e-9;

    /// Wraps a raw signed 24-bit ADC code, sign-extended to `i32`.
    pub const fn from_raw(raw: i32) -> Self {
        Self { raw }
    }

    /// Returns the raw signed 24-bit ADC code, sign-extended to `i32`.
    pub fn raw(self) -> i32 {
        self.raw
    }

    /// Decodes the raw ADC code into current-sense voltage in volts.
    pub fn decode(self) -> f32 {
        self.raw as f32 * Self::LSB_VOLTS
    }
}

/// Raw P1/P2 result.
///
/// P1/P2 contain power results by default, or voltage results when `P1ASV`/`P2ASV` is set
/// in [`AdcConfiguration`].
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub struct PowerOrVoltage {
    /// Raw signed 24-bit ADC code, sign-extended to `i32`.
    raw: i32,
}

impl PowerOrVoltage {
    /// Power-mode scale before applying the external shunt resistance.
    pub const POWER_LSB_VOLT_SQUARED: f32 = 5.8368e-12;
    /// Voltage-mode scale represented by one raw ADC code.
    pub const VOLTAGE_LSB_VOLTS: f32 = 46.875e-6;
    /// Power-mode scale represented by one P1TH/P1TL or P2TH/P2TL threshold code.
    pub const POWER_THRESHOLD_LSB_VOLT_SQUARED: f32 = 4.0 * Self::POWER_LSB_VOLT_SQUARED;
    /// Voltage-mode scale represented by one P1TH/P1TL or P2TH/P2TL threshold code.
    pub const VOLTAGE_THRESHOLD_LSB_VOLTS: f32 = 4.0 * Self::VOLTAGE_LSB_VOLTS;

    /// Wraps a raw signed 24-bit ADC code, sign-extended to `i32`.
    pub const fn from_raw(raw: i32) -> Self {
        Self { raw }
    }

    /// Returns the raw signed 24-bit ADC code, sign-extended to `i32`.
    pub fn raw(self) -> i32 {
        self.raw
    }

    /// Decodes a power-mode P1/P2 result into watts.
    pub fn decode_power(self, shunt_ohms: f32) -> f32 {
        self.raw as f32 * Self::POWER_LSB_VOLT_SQUARED / shunt_ohms
    }

    /// Decodes a voltage-mode P1/P2 result into volts.
    pub fn decode_voltage(self) -> f32 {
        self.raw as f32 * Self::VOLTAGE_LSB_VOLTS
    }
}

/// Raw BAT battery-voltage result.
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub struct BatteryVoltage {
    /// Raw signed 16-bit ADC code.
    raw: i16,
}

impl BatteryVoltage {
    /// Voltage represented by one raw ADC code.
    pub const LSB_VOLTS: f32 = 375e-6;

    /// Wraps a raw signed 16-bit ADC code.
    pub const fn from_raw(raw: i16) -> Self {
        Self { raw }
    }

    /// Returns the raw signed 16-bit ADC code.
    pub fn raw(self) -> i16 {
        self.raw
    }

    /// Decodes the raw ADC code into battery voltage in volts.
    pub fn decode(self) -> f32 {
        self.raw as f32 * Self::LSB_VOLTS
    }
}

/// Raw internal die-temperature result.
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub struct DieTemperature {
    /// Raw signed 16-bit ADC code.
    raw: i16,
}

impl DieTemperature {
    /// Temperature represented by one raw ADC code.
    pub const LSB_KELVIN: f32 = 0.2;
    /// Offset between the kelvin and Celsius temperature scales.
    pub const ZERO_CELSIUS_KELVIN: f32 = 273.15;

    /// Wraps a raw signed 16-bit ADC code.
    pub const fn from_raw(raw: i16) -> Self {
        Self { raw }
    }

    /// Returns the raw signed 16-bit ADC code.
    pub fn raw(self) -> i16 {
        self.raw
    }

    /// Decodes the raw ADC code into absolute temperature in kelvin.
    pub fn decode_kelvin(self) -> f32 {
        self.raw as f32 * Self::LSB_KELVIN
    }

    /// Decodes the raw ADC code into temperature in degrees Celsius.
    pub fn decode_celsius(self) -> f32 {
        self.decode_kelvin() - Self::ZERO_CELSIUS_KELVIN
    }
}

/// Raw A/DVCC supply-voltage result.
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub struct SupplyVoltage {
    /// Raw signed 16-bit ADC code.
    raw: i16,
}

impl SupplyVoltage {
    /// Voltage represented by one raw ADC code.
    pub const LSB_VOLTS: f32 = 2.26e-3;

    /// Wraps a raw signed 16-bit ADC code.
    pub const fn from_raw(raw: i16) -> Self {
        Self { raw }
    }

    /// Returns the raw signed 16-bit ADC code.
    pub fn raw(self) -> i16 {
        self.raw
    }

    /// Decodes the raw ADC code into supply voltage in volts.
    pub fn decode(self) -> f32 {
        self.raw as f32 * Self::LSB_VOLTS
    }
}

/// Raw SLOT1/SLOT2 result.
///
/// The value represents either an auxiliary-input voltage or an NTC temperature,
/// depending on the corresponding `NTC1`/`NTC2` bit in [`AdcConfiguration`].
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub struct SlotValue {
    /// Raw signed 16-bit ADC code.
    raw: i16,
}

impl SlotValue {
    /// Voltage represented by one raw ADC code when NTC conversion is disabled.
    pub const LSB_VOLTS: f32 = 375e-6;
    /// Temperature represented by one raw ADC code when NTC conversion is enabled.
    pub const LSB_DEGREES_CELSIUS: f32 = 0.2;

    /// Wraps a raw signed 16-bit SLOT ADC code.
    pub const fn from_raw(raw: i16) -> Self {
        Self { raw }
    }

    /// Returns the raw signed 16-bit ADC code.
    pub fn raw(self) -> i16 {
        self.raw
    }

    /// Decodes a voltage-mode SLOT result into volts.
    pub fn decode_voltage(self) -> f32 {
        self.raw as f32 * Self::LSB_VOLTS
    }

    /// Decodes an NTC-mode SLOT result into degrees Celsius.
    pub fn decode_temperature(self) -> f32 {
        self.raw as f32 * Self::LSB_DEGREES_CELSIUS
    }
}

/// Raw accumulated charge result from C1, C2 or C3.
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub struct AccumulatedCharge {
    /// Raw signed accumulator code, sign-extended to `i64` for 48-bit registers.
    raw: i64,
}

impl AccumulatedCharge {
    /// Volt-seconds represented by one raw code with the internal clock or a 4 MHz crystal.
    pub const LSB_VOLT_SECONDS: f64 = 377.887e-12;

    /// Wraps a raw signed 48-bit or 64-bit accumulator code.
    pub const fn from_raw(raw: i64) -> Self {
        Self { raw }
    }

    /// Returns the raw signed accumulator code.
    pub fn raw(self) -> i64 {
        self.raw
    }

    /// Decodes the result into volt-seconds for the internal clock or a 4 MHz crystal.
    ///
    /// With a freely configured external clock, use the LSB formula from datasheet Table 27
    /// instead because the scale depends on `fEXT`, `PRE` and `DIV`.
    pub fn decode(self) -> f64 {
        self.raw as f64 * Self::LSB_VOLT_SECONDS
    }

    /// Decodes the result into coulombs using the external shunt resistance.
    ///
    /// Pass the channel's shunt for C1/C2 and the channel-1 shunt for the weighted C3 sum.
    /// This scale applies to the internal clock or a 4 MHz crystal.
    pub fn decode_coulombs(self, shunt_ohms: f64) -> f64 {
        self.decode() / shunt_ohms
    }
}

/// Raw accumulated energy result from E1, E2 or E4.
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub struct AccumulatedEnergy {
    /// Raw signed accumulator code, sign-extended to `i64` for 48-bit registers.
    raw: i64,
}

impl AccumulatedEnergy {
    /// Volt-squared-seconds represented by one raw code with the internal clock or a 4 MHz crystal.
    pub const LSB_VOLT_SQUARED_SECONDS: f64 = 2.32175e-9;

    /// Wraps a raw signed 48-bit or 64-bit accumulator code.
    pub const fn from_raw(raw: i64) -> Self {
        Self { raw }
    }

    /// Returns the raw signed accumulator code.
    pub fn raw(self) -> i64 {
        self.raw
    }

    /// Decodes the result into volt-squared-seconds for the internal clock or a 4 MHz crystal.
    ///
    /// With a freely configured external clock, use the LSB formula from datasheet Table 27
    /// instead because the scale depends on `fEXT`, `PRE` and `DIV`.
    pub fn decode(self) -> f64 {
        self.raw as f64 * Self::LSB_VOLT_SQUARED_SECONDS
    }

    /// Decodes the result into joules using the external shunt resistance.
    ///
    /// Pass the channel's shunt for E1/E2 and the channel-1 shunt for the weighted E4 sum.
    /// This scale applies to the internal clock or a 4 MHz crystal.
    pub fn decode_joules(self, shunt_ohms: f64) -> f64 {
        self.decode() / shunt_ohms
    }
}

/// Raw accumulated time-base result from TB1 through TB4.
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub struct AccumulatedTime {
    /// Raw unsigned 32-bit accumulator code.
    raw: u32,
}

/// Charge accumulator selected for a PAGE1 threshold write.
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub enum ChargeAccumulator {
    Charge1,
    Charge2,
    /// Weighted sum of channels 1 and 2; its SI conversion uses the channel-1 shunt.
    Charge3,
}

/// Energy accumulator selected for a PAGE1 threshold write.
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub enum EnergyAccumulator {
    Energy1,
    Energy2,
    /// Weighted sum of channels 1 and 2; its SI conversion uses the channel-1 shunt.
    Energy4,
}

/// Time-base accumulator selected for a PAGE1 threshold write.
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub enum TimeBase {
    Time1,
    Time2,
    Time3,
    Time4,
}

/// Clock configuration used to scale accumulated charge, energy, and time thresholds.
///
/// [`Internal`](Self::Internal) also covers a 4 MHz crystal with the datasheet's `PRE = 2`,
/// `DIV = 30` settings (Table 26). The external-clock formula follows Table 27.
#[derive(Copy, Clone, PartialEq, Debug, Default)]
pub enum AccumulatorClock {
    #[default]
    Internal,
    External {
        /// External clock frequency in hertz.
        frequency_hz: f64,
        /// PRE field value (0–7).
        pre: u8,
        /// DIV field value (0–31).
        div: u8,
    },
}

impl AccumulatedTime {
    /// Seconds represented by one raw code with the internal clock or a 4 MHz crystal.
    pub const LSB_SECONDS: f64 = 397.777e-6;

    /// Wraps a raw unsigned 32-bit accumulator code.
    pub const fn from_raw(raw: u32) -> Self {
        Self { raw }
    }

    /// Returns the raw unsigned 32-bit accumulator code.
    pub fn raw(self) -> u32 {
        self.raw
    }

    /// Decodes the result into seconds for the internal clock or a 4 MHz crystal.
    ///
    /// With a freely configured external clock, use the LSB formula from datasheet Table 27
    /// instead because the scale depends on `fEXT`, `PRE` and `DIV`.
    pub fn decode(self) -> f64 {
        self.raw as f64 * Self::LSB_SECONDS
    }
}

/// A device register resolving to a memory [`Page`] and an on-bus address byte, so the
/// framing helpers take a register and derive the page rather than threading both.
trait Register: Copy {
    const PAGE: Page;
    fn addr(self) -> u8;
}

impl Register for RegAddressP0 {
    const PAGE: Page = Page::Page0;
    fn addr(self) -> u8 {
        self as u8
    }
}

impl Register for RegAddressP1 {
    const PAGE: Page = Page::Page1;
    fn addr(self) -> u8 {
        self as u8
    }
}

/// Operation Control register (PAGE0, 0xF0) — datasheet Table 24. `clr`, `sshot`, `adjupd`
/// and `rst` are set-only; the device clears them once done (poll to observe completion).
#[bitfield(bits = 8)]
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[repr(u8)]
pub struct OpsControlRegister {
    /// SLEEP: `0` = normal operation, `1` = enter SLEEP. The part exits SLEEP on CSB low
    /// in SPI mode or the datasheet wake-up pulse sequence in isoSPI mode.
    pub sleep: bool,
    /// CLR, set-only: clear accumulation and tracking registers, including charge, energy,
    /// time-base, max/min measurement, temperature, VCC and SLOT tracking registers.
    pub clr: bool,
    /// SSHOT, set-only: start one measurement set, update result registers, then return to
    /// STANDBY. If CONT is set, the current conversion cycle completes first.
    pub sshot: bool,
    /// CONT: enable continuous measurement. Charge, energy and time accumulation are active
    /// only while continuous mode is enabled.
    pub cont: bool,
    // Reserved bit 4. Write as 0.
    #[skip]
    __: B1,
    /// ADJUPD, set-only: request an update of page-1 configuration registers except
    /// thresholds. Issue in STANDBY; the device clears the bit when the update is done.
    pub adjupd: bool,
    // Reserved bit 6. Write as 0.
    #[skip]
    __: B1,
    /// RST, set-only: request software reset. The reset function is locked by default and
    /// requires the RSTUNLCK sequence before this bit has an effect.
    pub rst: bool,
}

/// Fast Control register (PAGE0, 0xF5) — datasheet Table 60.
#[bitfield(bits = 8)]
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[repr(u8)]
pub struct FastControlRegister {
    /// FACONV: enable continuous fast conversions. When this is set and at least one of
    /// `FACHA`, `FACH1` or `FACH2` is set, fast conversion results are written to the
    /// corresponding FIFO.
    pub faconv: bool,
    /// FACHA: include the auxiliary channel in fast mode. AUX fast conversions start after
    /// a FACONV rising edge or when an ADCV-style command is issued.
    pub facha: bool,
    /// FACH1: include channel 1 in fast mode. CH1 fast conversions start after a FACONV
    /// rising edge or when an ADCV-style command is issued.
    pub fach1: bool,
    /// FACH2: include channel 2 in fast mode. CH2 fast conversions start after a FACONV
    /// rising edge or when an ADCV-style command is issued.
    pub fach2: bool,
    // Reserved bits [7:4]. Write as 0.
    #[skip]
    __: B4,
}

/// ADC Configuration register (PAGE1, 0xDF) — datasheet Table 69. `p1asv`/`p2asv` set
/// power ADCs to voltage mode; `ntc1`/`ntc2` linearise the SLOT; `ntcslot1` ties CH2 TC to NTC1.
#[bitfield(bits = 8)]
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[repr(u8)]
pub struct AdcConfiguration {
    /// P1ASV: configure P1ADC as voltage instead of power. `0` = power mode,
    /// `1` = voltage mode.
    pub p1asv: bool,
    /// P2ASV: configure P2ADC as voltage instead of power. `0` = power mode,
    /// `1` = voltage mode.
    pub p2asv: bool,
    // Reserved bit 2. Write as 0.
    #[skip]
    __: B1,
    /// NTC1: SLOT1 result mode. `0` = voltage measurement with 375 uV LSB,
    /// `1` = NTC temperature measurement with 0.2 deg C LSB.
    pub ntc1: bool,
    /// NTC2: SLOT2 result mode. `0` = voltage measurement with 375 uV LSB,
    /// `1` = NTC temperature measurement with 0.2 deg C LSB.
    pub ntc2: bool,
    // Reserved bit 5. Write as 0.
    #[skip]
    __: B1,
    /// NTCSLOT1: shunt temperature-compensation source selection. `0` links I1 TC to
    /// SLOT1 and I2 TC to SLOT2; `1` links both I1 and I2 TC compensation to SLOT1.
    pub ntcslot1: bool,
    // Reserved bit 7. Write as 0.
    #[skip]
    __: B1,
}

/// Register Control register (common to both pages, 0xFF) — datasheet Table 23. `mlk` is
/// the 2-bit memory-lock handshake (`0b01` request, `0b10` device-acknowledged).
#[bitfield(bits = 8)]
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[repr(u8)]
pub struct RegControlRegister {
    /// PAGE: active memory-map page. `0` selects PAGE0 result/control registers;
    /// `1` selects PAGE1 threshold/configuration registers.
    pub page: bool,
    // Reserved bit 1. Write as 0.
    #[skip]
    __: B1,
    /// BCREN: broadcast read enable. Keep cleared in the parallel-to-daisy-chain topology
    /// so the LTC2949 does not respond to broadcast reads and collide with cell monitors.
    pub bcren: bool,
    // Reserved bit 3. Write as 0.
    #[skip]
    __: B1,
    /// MLK\[1:0\]: memory-lock handshake. `0b00` = unlocked, `0b01` = lock requested by
    /// master, `0b10` = memory locked/acknowledged by the LTC2949.
    pub mlk: B2,
    // Reserved bit 6. Write as 0.
    #[skip]
    __: B1,
    /// RDCVCONF: RDCV readout mode. `0` = indirect memory access from RDCVIADDR (0xFC);
    /// `1` = RDCV reports the latest fast-channel conversion results.
    pub rdcvconf: bool,
}

impl Default for OpsControlRegister {
    fn default() -> Self {
        Self::new()
    }
}

impl Default for FastControlRegister {
    fn default() -> Self {
        Self::new()
    }
}

impl Default for AdcConfiguration {
    fn default() -> Self {
        Self::new()
    }
}

impl Default for RegControlRegister {
    fn default() -> Self {
        Self::new()
    }
}

/// Status register (PAGE0, 0x80; datasheet Table 26).
#[bitfield(bits = 8)]
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[repr(u8)]
pub struct StatusRegister {
    /// UVLOA: analog supply undervoltage lockout occurred.
    pub uvloa: bool,
    /// PORA: analog supply power-on reset occurred.
    pub pora: bool,
    /// UVLOSTBY: standby regulator undervoltage lockout occurred.
    pub uvlostby: bool,
    /// UVLOD: digital supply undervoltage lockout occurred.
    pub uvlod: bool,
    /// UPDATE: result registers have been updated.
    pub update: bool,
    /// ADCERR: ADC error occurred.
    pub adcerr: bool,
    /// TBERR: time-base error occurred.
    pub tberr: bool,
    // Reserved bit 7.
    #[skip]
    __: B1,
}

/// Voltage, Temperature Threshold Alerts register (PAGE0, 0x81; datasheet Table 35).
#[bitfield(bits = 8)]
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[repr(u8)]
pub struct VTAlerts {
    /// BATH: voltage (VBATP – VBATM) high threshold exceeded.
    pub bath: bool,
    /// BATL: voltage (VBATP – VBATM) low threshold exceeded.
    pub batl: bool,
    /// TEMPH: temperature high threshold exceeded.
    pub temph: bool,
    /// TEMPL: temperature low threshold exceeded.
    pub templ: bool,
    /// SLOT1H: SLOT1 high threshold exceeded.
    pub slot1h: bool,
    /// SLOT1L: SLOT1 low threshold exceeded.
    pub slot1l: bool,
    /// SLOT2H: SLOT2 high threshold exceeded.
    pub slot2h: bool,
    /// SLOT2L: SLOT2 low threshold exceeded.
    pub slot2l: bool,
}

/// Current and power threshold alerts (STATIP, PAGE0, 0x82; datasheet Table 36).
#[bitfield(bits = 8)]
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[repr(u8)]
pub struct IPAlerts {
    /// I1H: Current1 high threshold exceeded.
    pub i1h: bool,
    /// I1L: Current1 low threshold exceeded.
    pub i1l: bool,
    /// P1H: Power1 high threshold exceeded.
    pub p1h: bool,
    /// P1L: Power1 low threshold exceeded.
    pub p1l: bool,
    /// I2H: Current2 high threshold exceeded.
    pub i2h: bool,
    /// I2L: Current2 low threshold exceeded.
    pub i2l: bool,
    /// P2H: Power2 high threshold exceeded.
    pub p2h: bool,
    /// P2L: Power2 low threshold exceeded.
    pub p2l: bool,
}

/// Charge threshold alerts (STATC, PAGE0, 0x83; datasheet Table 37).
#[bitfield(bits = 8)]
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[repr(u8)]
pub struct CAlerts {
    /// C1H: Charge1 high threshold exceeded.
    pub c1h: bool,
    /// C1L: Charge1 low threshold exceeded.
    pub c1l: bool,
    /// C2H: Charge2 high threshold exceeded.
    pub c2h: bool,
    /// C2L: Charge2 low threshold exceeded.
    pub c2l: bool,
    /// C3H: Charge3 high threshold exceeded.
    pub c3h: bool,
    /// C3L: Charge3 low threshold exceeded.
    pub c3l: bool,
    // Reserved bits 6–7.
    #[skip]
    __: B2,
}

/// Energy threshold alerts (STATE, PAGE0, 0x84; datasheet Table 38).
#[bitfield(bits = 8)]
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[repr(u8)]
pub struct EAlerts {
    /// E1H: Energy1 high threshold exceeded.
    pub e1h: bool,
    /// E1L: Energy1 low threshold exceeded.
    pub e1l: bool,
    /// E2H: Energy2 high threshold exceeded.
    pub e2h: bool,
    /// E2L: Energy2 low threshold exceeded.
    pub e2l: bool,
    // Reserved bits 4–5.
    #[skip]
    __: B2,
    /// E4H: Energy4 high threshold exceeded.
    pub e4h: bool,
    /// E4L: Energy4 low threshold exceeded.
    pub e4l: bool,
}

/// Charge and energy overflow alerts (STATCEOF, PAGE0, 0x85; datasheet Table 39).
#[bitfield(bits = 8)]
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[repr(u8)]
pub struct CEOFAlerts {
    /// C1OVF: Charge1 accumulator overflowed.
    pub c1ovf: bool,
    /// C2OVF: Charge2 accumulator overflowed.
    pub c2ovf: bool,
    /// C3OVF: Charge3 accumulator overflowed.
    pub c3ovf: bool,
    // Reserved bit 3.
    #[skip]
    __: B1,
    /// E1OVF: Energy1 accumulator overflowed.
    pub e1ovf: bool,
    /// E2OVF: Energy2 accumulator overflowed.
    pub e2ovf: bool,
    // Reserved bit 6.
    #[skip]
    __: B1,
    /// E4OVF: Energy4 accumulator overflowed.
    pub e4ovf: bool,
}

/// Time-base threshold and overflow alerts (STATTB, PAGE0, 0x86; datasheet Table 40).
#[bitfield(bits = 8)]
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[repr(u8)]
pub struct TBAlerts {
    /// T1TH: Time1 threshold exceeded.
    pub t1th: bool,
    /// T2TH: Time2 threshold exceeded.
    pub t2th: bool,
    /// T3TH: Time3 threshold exceeded.
    pub t3th: bool,
    /// T4TH: Time4 threshold exceeded.
    pub t4th: bool,
    /// T1OVF: Time1 overflowed.
    pub t1ovf: bool,
    /// T2OVF: Time2 overflowed.
    pub t2ovf: bool,
    /// T3OVF: Time3 overflowed.
    pub t3ovf: bool,
    /// T4OVF: Time4 overflowed.
    pub t4ovf: bool,
}

/// VCC and overcurrent-comparator alerts (STATVCC, PAGE0, 0x87; datasheet Table 41).
#[bitfield(bits = 8)]
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[repr(u8)]
pub struct VCCAlerts {
    /// VCCH: VCC high threshold exceeded.
    pub vcch: bool,
    /// VCCL: VCC low threshold exceeded.
    pub vccl: bool,
    /// OCC1H: Current1 remained above the OCC1 threshold for longer than its deglitch time.
    pub occ1h: bool,
    /// OCC2H: Current2 remained above the OCC2 threshold for longer than its deglitch time.
    pub occ2h: bool,
    // Reserved bits 4–7.
    #[skip]
    __: B4,
}

/// Fault register (PAGE0, 0xDD; datasheet Table 28).
#[bitfield(bits = 8)]
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[repr(u8)]
pub struct FaultsRegister {
    /// PROMERR: PROM read error occurred.
    pub promerr: bool,
    /// TSD: thermal shutdown occurred.
    pub tsd: bool,
    /// INTCOMMERR: internal communication error occurred.
    pub intcommerr: bool,
    /// EXTCOMMERR: external communication error occurred.
    pub extcommerr: bool,
    /// FAERR: fast-acquisition error occurred.
    pub faerr: bool,
    /// HWBIST: hardware built-in self-test failed.
    pub hwbist: bool,
    /// CRCCFG: configuration CRC error occurred.
    pub crccfg: bool,
    /// CRCMEM: memory CRC error occurred.
    pub crcmem: bool,
}

/// Extended fault register (PAGE0, 0xDC; datasheet Table 33).
#[bitfield(bits = 8)]
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[repr(u8)]
pub struct ExtFaultsRegister {
    /// HD1BITERR: Hamming decoder 1-bit error occurred.
    pub hd1biterr: bool,
    /// ROMERR: ROM CRC error occurred.
    pub romerr: bool,
    /// MEMERR: memory error occurred.
    pub memerr: bool,
    /// FCAERR: fast-channel error occurred.
    pub fcaerr: bool,
    /// XRAMERR: XRAM error occurred.
    pub xramerr: bool,
    /// IRAMERR: IRAM error occurred.
    pub iramerr: bool,
    // Reserved bit 6.
    #[skip]
    __: B1,
    /// HWMBISTEXEC: memory BIST was executed.
    pub hwmbistexec: bool,
}

/// FIFO sample tag values (datasheet Table 30).
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub enum FifoTag {
    /// New, valid sample.
    Ok,
    /// Sample was already read; no new data since last drain.
    ReadOverrun,
    /// FIFO filled and at least one sample was overwritten.
    WriteOverrun,
    /// Anything else returned by the device.
    Unknown(u8),
}

impl FifoTag {
    fn from_byte(b: u8) -> Self {
        match b {
            0x00 => FifoTag::Ok,
            0x55 => FifoTag::ReadOverrun,
            0xAA => FifoTag::WriteOverrun,
            other => FifoTag::Unknown(other),
        }
    }
}

/// One fast-mode FIFO sample.
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub struct FifoSample {
    /// Raw 16-bit two's-complement ADC reading. Multiply by the channel's LSB
    /// (7.60371 µV for I1/I2, 375.183 µV for BAT/AUX) to get a physical value.
    pub raw: i16,
    pub tag: FifoTag,
}

/// Coherent snapshot of one channel's charge / energy / time-base accumulators. All three
/// share a 16-byte row, so one burst reads them from the same `CONT` cycle (no lock needed).
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub struct Accumulators {
    /// Charge (48-bit two's-complement).
    pub charge: AccumulatedCharge,
    /// Energy (48-bit two's-complement).
    pub energy: AccumulatedEnergy,
    /// Time base (32-bit unsigned).
    pub time: AccumulatedTime,
}

/// Overcurrent control register payload for OCC1CTRL/OCC2CTRL (datasheet Tables 50 & 51).
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub struct OverCurrentConfig {
    /// OCCEN: enables the overcurrent comparator for the channel.
    pub enable: bool,
    /// OCCxDAC\[2:0\]: differential shunt-voltage threshold selector. Values map to
    /// 0, 26, 52, 78, 103, 155, 207 and 310 mV; convert to current with
    /// `I_limit = V_threshold / R_shunt`.
    pub threshold: u8,
    /// OCCxDGL\[1:0\]: deglitch time selector. Values map to off, 80 us, 320 us and
    /// 1280 us before the comparator event is accepted.
    pub deglitch_time: u8,
    /// OCCxPOL\[1:0\]: comparator polarity selector. `0b00` checks both current directions;
    /// `0b01` positive only; `0b10` negative only.
    pub polarity: u8,
}

/// One of the two LTC2949 measurement channels — each pairs a current/power ADC, a SLOT,
/// an NTC lineariser and a sense-resistor TC entry (suffix 1 vs. 2).
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub enum Channel {
    One,
    Two,
}

/// Steinhart–Hart parameters for one NTC channel: `1/T = a + b·ln(R) + c·ln(R)³`, with `R`
/// inferred from `r_ref`. Stored on-chip as Float24 (driver handles `f32 → Float24`).
#[derive(Copy, Clone, PartialEq, Debug)]
pub struct NtcConfig {
    /// Series reference resistor `R_ref` in ohms.
    pub r_ref: f32,
    /// Coefficient A (units: 1/K).
    pub a: f32,
    /// Coefficient B (units: 1/K).
    pub b: f32,
    /// Coefficient C (units: 1/K).
    pub c: f32,
}

/// Sense-resistor temperature-drift compensation: `R(T) = R0·[1 + tc·(T-t_ref) + tc2·(T-t_ref)²]`,
/// `T` being the linearised NTC reading. Copper ≈ `0.0039 /K`; low-TC alloys can stay uncompensated.
#[derive(Copy, Clone, PartialEq, Debug)]
pub struct ShuntTcConfig {
    /// First-order temperature coefficient (1/K). Datasheet `RSnTC`.
    pub tc: f32,
    /// Reference temperature `T0` in °C where the resistor is nominal (datasheet `RSnT0`,
    /// stored as 16-bit truncated Float24).
    pub t_ref: f32,
    /// Second-order temperature coefficient (1/K²). Datasheet `RSnTC2`. Set to
    /// `0.0` to disable the quadratic term — fine for copper.
    pub tc2: f32,
}

/// Inputs the AUX multiplexer can route to the SLOT pair (`MUXP`/`MUXN`) or the fast-mode
/// `FAMUX` registers. Discriminants match the 5-bit encoding in datasheet Table 57.
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[repr(u8)]
pub enum MuxInput {
    Agnd = 0,
    V1 = 1,
    V2 = 2,
    V3 = 3,
    V4 = 4,
    V5 = 5,
    V6 = 6,
    V7 = 7,
    V8 = 8,
    V9 = 9,
    V10 = 10,
    V11 = 11,
    V12 = 12,
    /// Overcurrent comparator test input (negative side).
    Imt = 13,
    /// Overcurrent comparator test input (positive side).
    Ipt = 14,
    VbatM = 15,
    VbatP = 16,
    Cf2M = 17,
    Cf2P = 18,
    Cf1M = 19,
    Cf1P = 20,
    /// Internal 2.39 V redundant reference.
    Vref2 = 22,
    /// `VREF2` routed through an internal 250 kΩ — useful for self-test of the
    /// AUX MUX current sources.
    Vref2Via250k = 23,
}

/// One of the four programmable AUX-MUX gain-correction settings in Table 72.
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub enum MuxGainSlot {
    One,
    Two,
    Three,
    Four,
}

/// Errors that can occur talking to an LTC2949.
pub enum Error<B: SpiDevice<u8>> {
    /// Underlying SPI transaction failed.
    BusError(B::Error),
    /// A returned PEC did not match the calculated value.
    ChecksumMismatch,
    /// A threshold was non-finite, used an invalid clock/shunt configuration, had its low
    /// bound above its high bound, or did not fit in the corresponding Table 67 register.
    InvalidThreshold,
    /// A gain factor or resistance used to derive one was zero, negative, NaN, or infinite.
    InvalidGainCorrection,
}

impl<B: SpiDevice<u8>> core::fmt::Debug for Error<B> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Error::BusError(_) => f.debug_struct("BusError").finish(),
            Error::ChecksumMismatch => f.debug_struct("ChecksumMismatch").finish(),
            Error::InvalidThreshold => f.debug_struct("InvalidThreshold").finish(),
            Error::InvalidGainCorrection => f.debug_struct("InvalidGainCorrection").finish(),
        }
    }
}

/// High-level LTC2949 operation set, the dependency-injection seam for hosts (mirrors
/// [LTC681X client](https://docs.rs/ltc681x/0.6.2/ltc681x/monitor/trait.LTC681XClient.html).
/// FIFO drains stay on [`LTC2949`].
pub trait Client {
    type Error;

    /// Starts wake-up (datasheet Figure 20): pulses the isoSPI edge and invalidates the page
    /// cache. Returns the wait ([`T_BOOT_US`]) to observe before [`confirm_wake_up`](Self::confirm_wake_up).
    fn start_wake_up(&mut self) -> Result<u32, Self::Error>;

    /// Completes wake-up by writing WKUPACK so the device does not auto-sleep after `tACKN`
    /// (1 s). Call once the [`start_wake_up`](Self::start_wake_up) wait has elapsed.
    fn confirm_wake_up(&mut self) -> Result<(), Self::Error>;

    /// Re-wakes only the isoSPI port (idles after `tIDLE` = 6.4 ms), leaving the core alone.
    /// Returns the wait ([`T_READY_US`]) before the next transaction. Cell monitors share this.
    fn wake_isospi(&mut self) -> Result<u32, Self::Error>;

    /// Writes the Operation Control register (PAGE0, 0xF0).
    fn write_opctrl(&mut self, value: OpsControlRegister) -> Result<(), Self::Error>;

    /// Reads the Operation Control register.
    fn read_opctrl(&mut self) -> Result<OpsControlRegister, Self::Error>;

    /// Writes the Fast Control register (PAGE0, 0xF5).
    fn write_factrl(&mut self, value: FastControlRegister) -> Result<(), Self::Error>;

    /// Writes the ADC Configuration register (PAGE1, 0xDF). Takes effect only after an
    /// ADJUPD pulse on OPCTRL while the core is in STANDBY.
    fn write_adcconf(&mut self, value: AdcConfiguration) -> Result<(), Self::Error>;

    /// Writes the Float24 gain-correction factor for sense resistor 1 or 2 (`RS1GC` or
    /// `RS2GC`, Table 72). The factor is dimensionless and must be positive and finite.
    /// Apply the change with an `ADJUPD` pulse while the device is in STANDBY.
    fn write_shunt_gain_correction(&mut self, channel: Channel, correction_factor: f32) -> Result<(), Self::Error>;

    /// Calculates `nominal_ohms / actual_ohms` and writes it to `RS1GC` or `RS2GC`.
    /// This directly implements the shunt-calibration calculation shown below Table 72.
    fn write_shunt_gain_correction_from_resistances(
        &mut self,
        channel: Channel,
        nominal_ohms: f32,
        actual_ohms: f32,
    ) -> Result<(), Self::Error>;

    /// Calculates and writes `RSRATIO = RS1 / RS2` as Float24. This factor is used when
    /// channel 2 contributes to the combined C3 and E4 accumulators.
    fn write_shunt_ratio(&mut self, rs1_ohms: f32, rs2_ohms: f32) -> Result<(), Self::Error>;

    /// Writes the dimensionless battery-divider gain-correction factor (`BATGC`) as Float24.
    fn write_battery_gain_correction(&mut self, correction_factor: f32) -> Result<(), Self::Error>;

    /// Configures one of the four AUX-MUX gain corrections. Writes its Float24 `MUXnGC`
    /// factor and the matching `MUXNSETn`/`MUXPSETn` input codes from Table 57.
    /// Matching is polarity-independent in the LTC2949.
    fn write_mux_gain_correction(
        &mut self,
        slot: MuxGainSlot,
        correction_factor: f32,
        negative: MuxInput,
        positive: MuxInput,
    ) -> Result<(), Self::Error>;

    /// Writes the low and high threshold for C1, C2, or C3 (Table 67), converting coulombs
    /// with `shunt_ohms` and the selected accumulator clock. C3 uses the channel-1 shunt.
    fn write_charge_thresholds(
        &mut self,
        accumulator: ChargeAccumulator,
        low_coulombs: f64,
        high_coulombs: f64,
        shunt_ohms: f64,
        clock: AccumulatorClock,
    ) -> Result<(), Self::Error>;

    /// Writes the low and high threshold for E1, E2, or E4 (Table 67), converting joules
    /// with `shunt_ohms` and the selected accumulator clock. E4 uses the channel-1 shunt.
    fn write_energy_thresholds(
        &mut self,
        accumulator: EnergyAccumulator,
        low_joules: f64,
        high_joules: f64,
        shunt_ohms: f64,
        clock: AccumulatorClock,
    ) -> Result<(), Self::Error>;

    /// Writes a TB1–TB4 high threshold in seconds (Table 67).
    fn write_time_threshold(
        &mut self,
        time_base: TimeBase,
        seconds: f64,
        clock: AccumulatorClock,
    ) -> Result<(), Self::Error>;

    /// Writes an I1/I2 low/high threshold pair in amperes. `shunt_ohms` converts the
    /// requested current to differential shunt voltage. The 16-bit threshold LSB is 3.8 µV
    /// (four times the 18-bit I1/I2 result LSB).
    fn write_current_thresholds(
        &mut self,
        channel: Channel,
        low_amperes: f32,
        high_amperes: f32,
        shunt_ohms: f32,
    ) -> Result<(), Self::Error>;

    /// Writes a power-mode P1/P2 low/high threshold pair in watts. Use this when the
    /// corresponding `PxASV` bit is clear. The 16-bit threshold LSB is four times the
    /// P1/P2 result-register LSB.
    fn write_power_thresholds(
        &mut self,
        channel: Channel,
        low_watts: f32,
        high_watts: f32,
        shunt_ohms: f32,
    ) -> Result<(), Self::Error>;

    /// Writes a voltage-mode P1/P2 low/high threshold pair in volts. Use this when the
    /// corresponding `PxASV` bit is set. The 16-bit threshold LSB is 187.5 µV.
    fn write_power_as_voltage_thresholds(
        &mut self,
        channel: Channel,
        low_volts: f32,
        high_volts: f32,
    ) -> Result<(), Self::Error>;

    /// Writes the BAT low/high threshold pair in volts.
    fn write_battery_thresholds(&mut self, low_volts: f32, high_volts: f32) -> Result<(), Self::Error>;

    /// Writes the die-temperature low/high threshold pair in degrees Celsius.
    fn write_temperature_thresholds(&mut self, low_celsius: f32, high_celsius: f32) -> Result<(), Self::Error>;

    /// Writes the A/DVCC low/high threshold pair in volts.
    fn write_vcc_thresholds(&mut self, low_volts: f32, high_volts: f32) -> Result<(), Self::Error>;

    /// Writes a SLOT1/SLOT2 low/high threshold pair in volts (corresponding `NTCx` clear).
    fn write_slot_voltage_thresholds(
        &mut self,
        slot: Channel,
        low_volts: f32,
        high_volts: f32,
    ) -> Result<(), Self::Error>;

    /// Writes a SLOT1/SLOT2 low/high threshold pair in degrees Celsius
    /// (corresponding `NTCx` set).
    fn write_slot_temperature_thresholds(
        &mut self,
        slot: Channel,
        low_celsius: f32,
        high_celsius: f32,
    ) -> Result<(), Self::Error>;

    /// Writes the Fast AUX mux selection (FAMUXP, FAMUXN).
    fn write_fast_aux_mux(&mut self, mux_n: u8, mux_p: u8) -> Result<(), Self::Error>;

    /// Writes the Float24 Steinhart–Hart coefficients and reference resistor for an NTC
    /// channel. Activate via [`write_slot_mux`](Self::write_slot_mux), [`AdcConfiguration::ntc1`] and an ADJUPD pulse.
    fn write_ntc_coefficients(&mut self, channel: Channel, params: &NtcConfig) -> Result<(), Self::Error>;

    /// Writes the sense-resistor temperature-compensation parameters for one channel. Active
    /// only after an `ADJUPD` pulse on `OpCtrl` while the core is in STANDBY.
    fn write_shunt_tc(&mut self, channel: Channel, config: &ShuntTcConfig) -> Result<(), Self::Error>;

    /// Routes `negative`/`positive` to the SLOT's `MUXN`/`MUXP` for differential AUX-ADC
    /// reads (datasheet Tables 57 & 58). NTCs typically use `(Vx, Agnd)`.
    fn write_slot_mux(&mut self, slot: Channel, negative: MuxInput, positive: MuxInput) -> Result<(), Self::Error>;

    /// Writes GPIO Control
    fn write_gpio_ctrl(&mut self, gpio: u8) -> Result<(), Self::Error>;

    /// Writes GPIO5 Control Mode
    fn write_gpio5_config(&mut self, config: u8) -> Result<(), Self::Error>;

    /// Writes GPIO4 Heartbeat
    fn write_gpio4_hb(&mut self, hb: bool) -> Result<(), Self::Error>;

    /// Writes both overcurrent-comparator control registers (`OCC1CTRL`/`OCC2CTRL`) in
    /// one PAGE0 burst. `config1` applies to channel 1; `config2` applies to channel 2.
    fn write_occ_config(&mut self, config1: OverCurrentConfig, config2: OverCurrentConfig) -> Result<(), Self::Error>;

    /// Reads and decodes the STATUS register (PAGE0, 0x80; datasheet Table 26).
    fn read_status(&mut self) -> Result<StatusRegister, Self::Error>;

    /// Reads voltage, temperature, and SLOT threshold alerts (STATVT, PAGE0, 0x81;
    /// datasheet Table 35). Set bits indicate that the corresponding high or low threshold
    /// was exceeded; the flags are sticky read/write bits.
    fn read_vt_alerts(&mut self) -> Result<VTAlerts, Self::Error>;

    /// Reads current and power threshold alerts (STATIP, PAGE0, 0x82; datasheet Table 36).
    fn read_ip_alerts(&mut self) -> Result<IPAlerts, Self::Error>;

    /// Reads accumulated-charge threshold alerts (STATC, PAGE0, 0x83; datasheet Table 37).
    fn read_c_alerts(&mut self) -> Result<CAlerts, Self::Error>;

    fn read_e_alerts(&mut self) -> Result<EAlerts, Self::Error>;

    /// Reads charge and energy accumulator overflow alerts (STATCEOF, PAGE0, 0x85;
    /// datasheet Table 39).
    fn read_ceof_alerts(&mut self) -> Result<CEOFAlerts, Self::Error>;

    /// Reads time-base threshold and overflow alerts (STATTB, PAGE0, 0x86;
    /// datasheet Table 40).
    fn read_tb_alerts(&mut self) -> Result<TBAlerts, Self::Error>;

    /// Reads VCC threshold and overcurrent-comparator alerts (STATVCC, PAGE0, 0x87;
    /// datasheet Table 41).
    fn read_vcc_alerts(&mut self) -> Result<VCCAlerts, Self::Error>;

    /// Reads and decodes the FAULTS register (PAGE0, 0xDD; datasheet Table 28).
    fn read_faults(&mut self) -> Result<FaultsRegister, Self::Error>;

    /// Reads and decodes the EXTFAULTS register (PAGE0, 0xDC; datasheet Table 33).
    fn read_extfaults(&mut self) -> Result<ExtFaultsRegister, Self::Error>;

    /// Requests the memory lock (datasheet Figure 19, `MLK = 0b01`) for a cross-register
    /// snapshot; returns the wait ([`T_MLCK_US`]). Stay on one page, then [`unlock_memory`](Self::unlock_memory).
    fn request_memory_lock(&mut self) -> Result<u32, Self::Error>;

    /// Releases the memory lock (`MLK = 0b00`), letting the register map update again.
    fn unlock_memory(&mut self) -> Result<(), Self::Error>;

    /// Reads I1 (slow-mode current 1) as a raw signed 24-bit current-sense voltage result.
    /// LSB = 950 nV.
    fn read_current1(&mut self) -> Result<CurrentSenseVoltage, Self::Error>;

    /// Reads I2 (slow-mode current 2). LSB = 950 nV.
    fn read_current2(&mut self) -> Result<CurrentSenseVoltage, Self::Error>;

    /// Reads I1AVG (moving average of the four preceding current 1 measurements).
    /// LSB = 237.5 nV.
    fn read_current1_avg(&mut self) -> Result<AveragedCurrentSenseVoltage, Self::Error>;

    /// Reads I2AVG (moving average of the four preceding current 2 measurements).
    /// LSB = 237.5 nV.
    fn read_current2_avg(&mut self) -> Result<AveragedCurrentSenseVoltage, Self::Error>;

    /// Reads P1 (power 1 or voltage if P1ASV is set). LSB = 5.8368 µV²/Ω (power) or
    /// 46.875 µV (voltage).
    fn read_power1(&mut self) -> Result<PowerOrVoltage, Self::Error>;

    /// Reads P2 (power 2 or voltage if P2ASV is set). LSB = 5.8368 µV²/Ω (power) or
    /// 46.875 µV (voltage).
    fn read_power2(&mut self) -> Result<PowerOrVoltage, Self::Error>;

    /// Reads BAT (battery voltage). LSB = 375 µV.
    fn read_bat(&mut self) -> Result<BatteryVoltage, Self::Error>;

    /// Reads internal die temperature. LSB = 0.2 °C, full-scale 819.2 K.
    fn read_temp(&mut self) -> Result<DieTemperature, Self::Error>;

    /// Reads A/DVCC supply voltage. LSB = 2.26 mV.
    fn read_vcc(&mut self) -> Result<SupplyVoltage, Self::Error>;

    /// Reads SLOT1 as a [`SlotValue`] — voltage or temperature depending on NTC1 in
    /// ADCCONF.
    fn read_slot1(&mut self) -> Result<SlotValue, Self::Error>;

    /// Reads SLOT2 as a [`SlotValue`] — voltage or temperature depending on NTC2 in
    /// ADCCONF.
    fn read_slot2(&mut self) -> Result<SlotValue, Self::Error>;

    /// Reads Charge1 (48-bit two's-complement) as [`AccumulatedCharge`].
    fn read_charge1(&mut self) -> Result<AccumulatedCharge, Self::Error>;

    /// Reads Charge2 (48-bit two's-complement) as [`AccumulatedCharge`].
    fn read_charge2(&mut self) -> Result<AccumulatedCharge, Self::Error>;

    /// Reads Charge3 — the weighted channel-1/channel-2 sum (64-bit two's-complement).
    fn read_charge3(&mut self) -> Result<AccumulatedCharge, Self::Error>;

    /// Reads Energy1 (48-bit two's-complement) as [`AccumulatedEnergy`].
    fn read_energy1(&mut self) -> Result<AccumulatedEnergy, Self::Error>;

    /// Reads Energy2 (48-bit two's-complement) as [`AccumulatedEnergy`].
    fn read_energy2(&mut self) -> Result<AccumulatedEnergy, Self::Error>;

    /// Reads Energy4 — the weighted channel-1/channel-2 sum (64-bit two's-complement).
    fn read_energy4(&mut self) -> Result<AccumulatedEnergy, Self::Error>;

    /// Reads time-base 1 (32-bit unsigned) as [`AccumulatedTime`].
    fn read_time1(&mut self) -> Result<AccumulatedTime, Self::Error>;

    /// Reads time-base 2 as [`AccumulatedTime`].
    fn read_time2(&mut self) -> Result<AccumulatedTime, Self::Error>;

    /// Reads time-base 3 as [`AccumulatedTime`].
    fn read_time3(&mut self) -> Result<AccumulatedTime, Self::Error>;

    /// Reads time-base 4 as [`AccumulatedTime`].
    fn read_time4(&mut self) -> Result<AccumulatedTime, Self::Error>;

    /// Reads channel 1's charge, energy and time-base ([`Accumulators`]) in one coherent
    /// 16-byte burst — prefer this for SoC integration over separate charge/time reads.
    fn read_accumulators1(&mut self) -> Result<Accumulators, Self::Error>;

    /// Reads channel 2's charge, energy and time-base in a single coherent 16-byte burst
    /// (row `0x10–0x1F`). See [`read_accumulators1`](Self::read_accumulators1).
    fn read_accumulators2(&mut self) -> Result<Accumulators, Self::Error>;

    /// Broadcast ADCV (0x0260): synchronous fast conversion on the LTC2949 and every cell
    /// monitor. **Hazard:** also restarts the chain, so don't call it from a separate meter task.
    fn trigger_adcv_broadcast(&mut self) -> Result<(), Self::Error>;

    /// Broadcasts ADAX (`0x0460`) on the shared bus.
    ///
    /// LTC2949 treats ADAX like every other ADCV-style command: with `CONT = 1` and
    /// `FACONV = 0`, it starts a fast single-shot conversion of the channels enabled in
    /// [`FastControlRegister`]. It does not specifically start a slow SLOT1/SLOT2 conversion.
    ///
    /// The broadcast also starts AUX conversions on attached LTC68xx cell monitors and clears
    /// the LTC2949 FIFOs before conversion. With `FACONV = 1`, LTC2949 ignores the command.
    fn trigger_adax(&mut self) -> Result<(), Self::Error>;
}

/// LTC2949 client for the parallel-to-daisy-chain topology (datasheet Figure 12(B)),
/// reached directly via `DCMD`. High-level ops are on the [`Client`] trait.
pub struct LTC2949<B, P>
where
    B: SpiDevice<u8>,
    P: PollMethod<B>,
{
    bus: B,
    poll_method: P,
    /// Tracks the currently selected page so `select_page` is a no-op when redundant.
    current_page: Option<Page>,
}

impl<B> LTC2949<B, NoPolling>
where
    B: SpiDevice<u8>,
{
    /// Constructs an LTC2949 client, addressed directly via `DCMD` (alone on the bus or in
    /// parallel with a cell-monitor chain).
    pub fn new(bus: B) -> Self {
        Self {
            bus,
            poll_method: NoPolling {},
            current_page: None,
        }
    }
}

impl<B, P> Client for LTC2949<B, P>
where
    B: SpiDevice<u8>,
    P: PollMethod<B>,
{
    type Error = Error<B>;

    fn start_wake_up(&mut self) -> Result<u32, Error<B>> {
        // A reset / SLEEP clears the device's page selection — drop the cache so the next
        // access re-issues REGSCTRL rather than trusting a stale page.
        self.current_page = None;

        // Two dummy bytes — content irrelevant, they just provide the isoSPI wake edge.
        self.bus.write(&[0x00]).map_err(Error::BusError)?;
        self.bus.write(&[0x00]).map_err(Error::BusError)?;

        // If the core was in SLEEP or just powered up it needs up to tBOOT to reach
        // STANDBY before it will accept the wake-up confirmation. The host owns the wait.
        Ok(T_BOOT_US)
    }

    fn confirm_wake_up(&mut self) -> Result<(), Error<B>> {
        // Confirm wake-up by writing 0x00 to WKUPACK (within tACKN = 1 s of STANDBY).
        self.write_bytes(RegAddressP0::WkupAck, &[0x00])
    }

    fn wake_isospi(&mut self) -> Result<u32, Error<B>> {
        // One dummy byte provides the differential edge; the port is ready after tREADY.
        // The core state is untouched. The host owns the wait.
        self.bus.write(&[0x00]).map_err(Error::BusError)?;
        Ok(T_READY_US)
    }

    fn write_opctrl(&mut self, value: OpsControlRegister) -> Result<(), Error<B>> {
        self.write_bytes(RegAddressP0::OpCtrl, &value.into_bytes())
    }

    fn read_opctrl(&mut self) -> Result<OpsControlRegister, Error<B>> {
        let mut buf = [0u8; 1];
        self.read_bytes(RegAddressP0::OpCtrl, &mut buf)?;
        Ok(OpsControlRegister::from_bytes(buf))
    }

    fn write_factrl(&mut self, value: FastControlRegister) -> Result<(), Error<B>> {
        self.write_bytes(RegAddressP0::FaCtrl, &value.into_bytes())
    }

    fn write_adcconf(&mut self, value: AdcConfiguration) -> Result<(), Error<B>> {
        self.write_bytes(RegAddressP1::AdcConf, &value.into_bytes())
    }

    fn write_shunt_gain_correction(&mut self, channel: Channel, correction_factor: f32) -> Result<(), Error<B>> {
        let address = match channel {
            Channel::One => RegAddressP1::Rs1Gc,
            Channel::Two => RegAddressP1::Rs2Gc,
        };
        self.write_gain_factor(address, correction_factor)
    }

    fn write_shunt_gain_correction_from_resistances(
        &mut self,
        channel: Channel,
        nominal_ohms: f32,
        actual_ohms: f32,
    ) -> Result<(), Error<B>> {
        let correction_factor = Self::positive_ratio(nominal_ohms, actual_ohms)?;
        self.write_shunt_gain_correction(channel, correction_factor)
    }

    fn write_shunt_ratio(&mut self, rs1_ohms: f32, rs2_ohms: f32) -> Result<(), Error<B>> {
        let ratio = Self::positive_ratio(rs1_ohms, rs2_ohms)?;
        self.write_gain_factor(RegAddressP1::RsRatio, ratio)
    }

    fn write_battery_gain_correction(&mut self, correction_factor: f32) -> Result<(), Error<B>> {
        self.write_gain_factor(RegAddressP1::BatGc, correction_factor)
    }

    fn write_mux_gain_correction(
        &mut self,
        slot: MuxGainSlot,
        correction_factor: f32,
        negative: MuxInput,
        positive: MuxInput,
    ) -> Result<(), Error<B>> {
        let (gain_address, mux_set_address) = match slot {
            MuxGainSlot::One => (RegAddressP1::Mux1Gc, RegAddressP1::MuxNset1),
            MuxGainSlot::Two => (RegAddressP1::Mux2Gc, RegAddressP1::MuxNset2),
            MuxGainSlot::Three => (RegAddressP1::Mux3Gc, RegAddressP1::MuxNset3),
            MuxGainSlot::Four => (RegAddressP1::Mux4Gc, RegAddressP1::MuxNset4),
        };
        let encoded = Self::encode_gain_factor(correction_factor)?;
        self.write_bytes(gain_address, &encoded)?;
        self.write_bytes(mux_set_address, &[negative as u8, positive as u8])
    }

    fn write_charge_thresholds(
        &mut self,
        accumulator: ChargeAccumulator,
        low_coulombs: f64,
        high_coulombs: f64,
        shunt_ohms: f64,
        clock: AccumulatorClock,
    ) -> Result<(), Error<B>> {
        let (charge_lsb, _, _) = Self::accumulator_lsbs(clock)?;
        let lsb_coulombs = Self::divide_lsb_by_shunt(charge_lsb, shunt_ohms)?;
        match accumulator {
            ChargeAccumulator::Charge1 => {
                self.write_signed_48_threshold_pair(RegAddressP1::C1Th, low_coulombs, high_coulombs, lsb_coulombs)
            }
            ChargeAccumulator::Charge2 => {
                self.write_signed_48_threshold_pair(RegAddressP1::C2Th, low_coulombs, high_coulombs, lsb_coulombs)
            }
            ChargeAccumulator::Charge3 => self.write_signed_64_threshold_pair(
                RegAddressP1::C3Th,
                RegAddressP1::C3Tl,
                low_coulombs,
                high_coulombs,
                lsb_coulombs,
            ),
        }
    }

    fn write_energy_thresholds(
        &mut self,
        accumulator: EnergyAccumulator,
        low_joules: f64,
        high_joules: f64,
        shunt_ohms: f64,
        clock: AccumulatorClock,
    ) -> Result<(), Error<B>> {
        let (_, energy_lsb, _) = Self::accumulator_lsbs(clock)?;
        let lsb_joules = Self::divide_lsb_by_shunt(energy_lsb, shunt_ohms)?;
        match accumulator {
            EnergyAccumulator::Energy1 => {
                self.write_signed_48_threshold_pair(RegAddressP1::E1Th, low_joules, high_joules, lsb_joules)
            }
            EnergyAccumulator::Energy2 => {
                self.write_signed_48_threshold_pair(RegAddressP1::E2Th, low_joules, high_joules, lsb_joules)
            }
            EnergyAccumulator::Energy4 => self.write_signed_64_threshold_pair(
                RegAddressP1::E4Th,
                RegAddressP1::E4Tl,
                low_joules,
                high_joules,
                lsb_joules,
            ),
        }
    }

    fn write_time_threshold(
        &mut self,
        time_base: TimeBase,
        seconds: f64,
        clock: AccumulatorClock,
    ) -> Result<(), Error<B>> {
        let (_, _, time_lsb) = Self::accumulator_lsbs(clock)?;
        let raw = Self::quantize_unsigned_32(seconds, time_lsb)?;
        let address = match time_base {
            TimeBase::Time1 => RegAddressP1::Tb1Th,
            TimeBase::Time2 => RegAddressP1::Tb2Th,
            TimeBase::Time3 => RegAddressP1::Tb3Th,
            TimeBase::Time4 => RegAddressP1::Tb4Th,
        };
        self.write_bytes(address, &raw.to_be_bytes())
    }

    fn write_current_thresholds(
        &mut self,
        channel: Channel,
        low_amperes: f32,
        high_amperes: f32,
        shunt_ohms: f32,
    ) -> Result<(), Error<B>> {
        let lsb_amperes =
            Self::divide_lsb_by_shunt(CurrentSenseVoltage::THRESHOLD_LSB_VOLTS as f64, shunt_ohms as f64)?;
        let address = match channel {
            Channel::One => RegAddressP1::I1Th,
            Channel::Two => RegAddressP1::I2Th,
        };
        self.write_signed_16_threshold_pair(address, low_amperes as f64, high_amperes as f64, lsb_amperes)
    }

    fn write_power_thresholds(
        &mut self,
        channel: Channel,
        low_watts: f32,
        high_watts: f32,
        shunt_ohms: f32,
    ) -> Result<(), Error<B>> {
        let lsb_watts = Self::divide_lsb_by_shunt(
            PowerOrVoltage::POWER_THRESHOLD_LSB_VOLT_SQUARED as f64,
            shunt_ohms as f64,
        )?;
        let address = match channel {
            Channel::One => RegAddressP1::P1Th,
            Channel::Two => RegAddressP1::P2Th,
        };
        self.write_signed_16_threshold_pair(address, low_watts as f64, high_watts as f64, lsb_watts)
    }

    fn write_power_as_voltage_thresholds(
        &mut self,
        channel: Channel,
        low_volts: f32,
        high_volts: f32,
    ) -> Result<(), Error<B>> {
        let address = match channel {
            Channel::One => RegAddressP1::P1Th,
            Channel::Two => RegAddressP1::P2Th,
        };
        self.write_signed_16_threshold_pair(
            address,
            low_volts as f64,
            high_volts as f64,
            PowerOrVoltage::VOLTAGE_THRESHOLD_LSB_VOLTS as f64,
        )
    }

    fn write_battery_thresholds(&mut self, low_volts: f32, high_volts: f32) -> Result<(), Error<B>> {
        self.write_signed_16_threshold_pair(
            RegAddressP1::BatTh,
            low_volts as f64,
            high_volts as f64,
            BatteryVoltage::LSB_VOLTS as f64,
        )
    }

    fn write_temperature_thresholds(&mut self, low_celsius: f32, high_celsius: f32) -> Result<(), Error<B>> {
        self.write_signed_16_threshold_pair(
            RegAddressP1::TempTh,
            (low_celsius + DieTemperature::ZERO_CELSIUS_KELVIN) as f64,
            (high_celsius + DieTemperature::ZERO_CELSIUS_KELVIN) as f64,
            DieTemperature::LSB_KELVIN as f64,
        )
    }

    fn write_vcc_thresholds(&mut self, low_volts: f32, high_volts: f32) -> Result<(), Error<B>> {
        self.write_signed_16_threshold_pair(
            RegAddressP1::VccTh,
            low_volts as f64,
            high_volts as f64,
            SupplyVoltage::LSB_VOLTS as f64,
        )
    }

    fn write_slot_voltage_thresholds(
        &mut self,
        slot: Channel,
        low_volts: f32,
        high_volts: f32,
    ) -> Result<(), Error<B>> {
        let address = match slot {
            Channel::One => RegAddressP1::Slot1Th,
            Channel::Two => RegAddressP1::Slot2Th,
        };
        self.write_signed_16_threshold_pair(
            address,
            low_volts as f64,
            high_volts as f64,
            SlotValue::LSB_VOLTS as f64,
        )
    }

    fn write_slot_temperature_thresholds(
        &mut self,
        slot: Channel,
        low_celsius: f32,
        high_celsius: f32,
    ) -> Result<(), Error<B>> {
        let address = match slot {
            Channel::One => RegAddressP1::Slot1Th,
            Channel::Two => RegAddressP1::Slot2Th,
        };
        self.write_signed_16_threshold_pair(
            address,
            low_celsius as f64,
            high_celsius as f64,
            SlotValue::LSB_DEGREES_CELSIUS as f64,
        )
    }

    fn write_fast_aux_mux(&mut self, mux_n: u8, mux_p: u8) -> Result<(), Error<B>> {
        self.write_bytes(RegAddressP0::FaMuxN, &[mux_n, mux_p])
    }

    fn write_ntc_coefficients(&mut self, channel: Channel, params: &NtcConfig) -> Result<(), Error<B>> {
        let (rref_addr, abc_addr) = match channel {
            Channel::One => (RegAddressP1::Rref1, RegAddressP1::Ntc1A),
            Channel::Two => (RegAddressP1::Rref2, RegAddressP1::Ntc2A),
        };

        let rref = Float24::new(params.r_ref).encode();
        self.write_bytes(rref_addr, &rref)?;

        let mut abc = [0u8; 9];
        abc[0..3].copy_from_slice(&Float24::new(params.a).encode());
        abc[3..6].copy_from_slice(&Float24::new(params.b).encode());
        abc[6..9].copy_from_slice(&Float24::new(params.c).encode());
        self.write_bytes(abc_addr, &abc)?;

        Ok(())
    }

    fn write_shunt_tc(&mut self, channel: Channel, config: &ShuntTcConfig) -> Result<(), Error<B>> {
        let (tc_addr, tc2_addr) = match channel {
            Channel::One => (RegAddressP1::Rs1Tc, RegAddressP1::Rs1Tc2),
            Channel::Two => (RegAddressP1::Rs2Tc, RegAddressP1::Rs2Tc2),
        };

        // RSnTC (3 bytes Float24) + RSnT0 (2 bytes truncated Float24).
        let mut tc_burst = [0u8; 5];
        tc_burst[0..3].copy_from_slice(&Float24::new(config.tc).encode());
        tc_burst[3..5].copy_from_slice(&Float24::new(config.t_ref).encode_high());
        self.write_bytes(tc_addr, &tc_burst)?;

        // RSnTC2 lives elsewhere on the page.
        let tc2 = Float24::new(config.tc2).encode();
        self.write_bytes(tc2_addr, &tc2)?;

        Ok(())
    }

    fn write_slot_mux(&mut self, slot: Channel, negative: MuxInput, positive: MuxInput) -> Result<(), Error<B>> {
        let addr_n = match slot {
            Channel::One => RegAddressP0::Slot1MuxN,
            Channel::Two => RegAddressP0::Slot2MuxN,
        };
        // MUXN and MUXP are adjacent (0xEB/0xEC for SLOT1, 0xED/0xEE for SLOT2)
        // so a single 2-byte burst configures both.
        self.write_bytes(addr_n, &[negative as u8, positive as u8])
    }

    fn write_gpio_ctrl(&mut self, gpio: u8) -> Result<(), Self::Error> {
        self.write_bytes(RegAddressP0::FGpioCtrl, &[gpio])
    }

    fn write_gpio5_config(&mut self, config: u8) -> Result<(), Self::Error> {
        self.write_bytes(RegAddressP0::FCurGpioCtrl, &[config])
    }

    fn write_gpio4_hb(&mut self, hb: bool) -> Result<(), Self::Error> {
        if hb {
            self.write_bytes(RegAddressP0::FCurGpioCtrl, &[0b1])
        } else {
            self.write_bytes(RegAddressP0::FCurGpioCtrl, &[0b0])
        }
    }

    fn write_occ_config(&mut self, config1: OverCurrentConfig, config2: OverCurrentConfig) -> Result<(), Self::Error> {
        let b =
            (config1.enable as u8) | (config1.threshold << 1) | (config1.deglitch_time << 4) | (config1.polarity << 6);
        self.write_bytes(RegAddressP0::Occ1Ctrl, &[b])?;

        let b2 =
            (config2.enable as u8) | (config2.threshold << 1) | (config2.deglitch_time << 4) | (config2.polarity << 6);
        self.write_bytes(RegAddressP0::Occ2Ctrl, &[b2])
    }

    fn read_status(&mut self) -> Result<StatusRegister, Error<B>> {
        let mut buf = [0u8; 1];
        self.read_bytes(RegAddressP0::Status, &mut buf)?;
        Ok(StatusRegister::from_bytes(buf))
    }

    fn read_vt_alerts(&mut self) -> Result<VTAlerts, Self::Error> {
        let mut buf = [0u8; 1];
        self.read_bytes(RegAddressP0::StatVT, &mut buf)?;
        Ok(VTAlerts::from_bytes(buf))
    }

    fn read_ip_alerts(&mut self) -> Result<IPAlerts, Self::Error> {
        let mut buf = [0u8; 1];
        self.read_bytes(RegAddressP0::StatIP, &mut buf)?;
        Ok(IPAlerts::from_bytes(buf))
    }

    fn read_c_alerts(&mut self) -> Result<CAlerts, Self::Error> {
        let mut buf = [0u8; 1];
        self.read_bytes(RegAddressP0::StatC, &mut buf)?;
        Ok(CAlerts::from_bytes(buf))
    }

    fn read_e_alerts(&mut self) -> Result<EAlerts, Self::Error> {
        let mut buf = [0u8; 1];
        self.read_bytes(RegAddressP0::StatE, &mut buf)?;
        Ok(EAlerts::from_bytes(buf))
    }

    fn read_ceof_alerts(&mut self) -> Result<CEOFAlerts, Self::Error> {
        let mut buf = [0u8; 1];
        self.read_bytes(RegAddressP0::StatCEOF, &mut buf)?;
        Ok(CEOFAlerts::from_bytes(buf))
    }

    fn read_tb_alerts(&mut self) -> Result<TBAlerts, Self::Error> {
        let mut buf = [0u8; 1];
        self.read_bytes(RegAddressP0::StatTB, &mut buf)?;
        Ok(TBAlerts::from_bytes(buf))
    }

    fn read_vcc_alerts(&mut self) -> Result<VCCAlerts, Self::Error> {
        let mut buf = [0u8; 1];
        self.read_bytes(RegAddressP0::StatVCC, &mut buf)?;
        Ok(VCCAlerts::from_bytes(buf))
    }

    fn read_faults(&mut self) -> Result<FaultsRegister, Error<B>> {
        let mut buf = [0u8; 1];
        self.read_bytes(RegAddressP0::Faults, &mut buf)?;
        Ok(FaultsRegister::from_bytes(buf))
    }

    fn read_extfaults(&mut self) -> Result<ExtFaultsRegister, Error<B>> {
        let mut buf = [0u8; 1];
        self.read_bytes(RegAddressP0::ExtFaults, &mut buf)?;
        Ok(ExtFaultsRegister::from_bytes(buf))
    }

    // -- Memory lock (coherent multi-register snapshots) -------------------

    fn request_memory_lock(&mut self) -> Result<u32, Error<B>> {
        // REGSCTRL is page-independent; preserve the current page bit so the lock request
        // does not also switch pages. MLK = 0b01.
        let page = self.current_page.unwrap_or(Page::Page0);
        let value = self.regsctrl_base().with_mlk(0b01);
        self.dcmd_write(RegAddressP0::RegsCtrl.addr(), &value.into_bytes())?;
        // The lock request also pins the page. Record it so that same-page reads inside the
        // lock are cache hits and do not rewrite REGSCTRL — which would clear MLK and
        // release the lock. (This is why the caller must stay on one page while locked.)
        self.current_page = Some(page);
        // The host waits the worst-case acknowledge time (tMLCK,M) before relying on the
        // snapshot. Polling REGSCTRL.MLK == 0b10 is the faster alternative but needs a
        // readback loop; the fixed wait is simpler and guaranteed.
        Ok(T_MLCK_US)
    }

    fn unlock_memory(&mut self) -> Result<(), Error<B>> {
        // MLK back to 0b00, same page preserved.
        let value = self.regsctrl_base();
        self.dcmd_write(RegAddressP0::RegsCtrl.addr(), &value.into_bytes())
    }

    fn read_current1(&mut self) -> Result<CurrentSenseVoltage, Error<B>> {
        self.read_signed_24(RegAddressP0::Current1).map(CurrentSenseVoltage::from_raw)
    }

    fn read_current2(&mut self) -> Result<CurrentSenseVoltage, Error<B>> {
        self.read_signed_24(RegAddressP0::Current2).map(CurrentSenseVoltage::from_raw)
    }

    fn read_current1_avg(&mut self) -> Result<AveragedCurrentSenseVoltage, Error<B>> {
        self.read_signed_24(RegAddressP0::Current1Avg)
            .map(AveragedCurrentSenseVoltage::from_raw)
    }

    fn read_current2_avg(&mut self) -> Result<AveragedCurrentSenseVoltage, Error<B>> {
        self.read_signed_24(RegAddressP0::Current2Avg)
            .map(AveragedCurrentSenseVoltage::from_raw)
    }

    fn read_power1(&mut self) -> Result<PowerOrVoltage, Error<B>> {
        self.read_signed_24(RegAddressP0::Power1).map(PowerOrVoltage::from_raw)
    }

    fn read_power2(&mut self) -> Result<PowerOrVoltage, Error<B>> {
        self.read_signed_24(RegAddressP0::Power2).map(PowerOrVoltage::from_raw)
    }

    fn read_bat(&mut self) -> Result<BatteryVoltage, Error<B>> {
        self.read_signed_16(RegAddressP0::Bat).map(BatteryVoltage::from_raw)
    }

    fn read_temp(&mut self) -> Result<DieTemperature, Error<B>> {
        self.read_signed_16(RegAddressP0::Temp).map(DieTemperature::from_raw)
    }

    fn read_vcc(&mut self) -> Result<SupplyVoltage, Error<B>> {
        self.read_signed_16(RegAddressP0::Vcc).map(SupplyVoltage::from_raw)
    }

    fn read_slot1(&mut self) -> Result<SlotValue, Error<B>> {
        self.read_signed_16(RegAddressP0::Slot1).map(SlotValue::from_raw)
    }

    fn read_slot2(&mut self) -> Result<SlotValue, Error<B>> {
        self.read_signed_16(RegAddressP0::Slot2).map(SlotValue::from_raw)
    }

    fn read_charge1(&mut self) -> Result<AccumulatedCharge, Error<B>> {
        self.read_signed_48(RegAddressP0::Charge1).map(AccumulatedCharge::from_raw)
    }

    fn read_charge2(&mut self) -> Result<AccumulatedCharge, Error<B>> {
        self.read_signed_48(RegAddressP0::Charge2).map(AccumulatedCharge::from_raw)
    }

    fn read_charge3(&mut self) -> Result<AccumulatedCharge, Error<B>> {
        self.read_signed_64(RegAddressP0::Charge3).map(AccumulatedCharge::from_raw)
    }

    fn read_energy1(&mut self) -> Result<AccumulatedEnergy, Error<B>> {
        self.read_signed_48(RegAddressP0::Energy1).map(AccumulatedEnergy::from_raw)
    }

    fn read_energy2(&mut self) -> Result<AccumulatedEnergy, Error<B>> {
        self.read_signed_48(RegAddressP0::Energy2).map(AccumulatedEnergy::from_raw)
    }

    fn read_energy4(&mut self) -> Result<AccumulatedEnergy, Error<B>> {
        self.read_signed_64(RegAddressP0::Energy4).map(AccumulatedEnergy::from_raw)
    }

    fn read_time1(&mut self) -> Result<AccumulatedTime, Error<B>> {
        self.read_unsigned_32(RegAddressP0::Time1).map(AccumulatedTime::from_raw)
    }

    fn read_time2(&mut self) -> Result<AccumulatedTime, Error<B>> {
        self.read_unsigned_32(RegAddressP0::Time2).map(AccumulatedTime::from_raw)
    }

    fn read_time3(&mut self) -> Result<AccumulatedTime, Error<B>> {
        self.read_unsigned_32(RegAddressP0::Time3).map(AccumulatedTime::from_raw)
    }

    fn read_time4(&mut self) -> Result<AccumulatedTime, Error<B>> {
        self.read_unsigned_32(RegAddressP0::Time4).map(AccumulatedTime::from_raw)
    }

    fn read_accumulators1(&mut self) -> Result<Accumulators, Error<B>> {
        self.read_accumulator_row(RegAddressP0::Charge1)
    }

    fn read_accumulators2(&mut self) -> Result<Accumulators, Error<B>> {
        self.read_accumulator_row(RegAddressP0::Charge2)
    }

    fn trigger_adcv_broadcast(&mut self) -> Result<(), Error<B>> {
        // ADCV broadcast: CMD0=0b00000_010, CMD1=0b01100000 = 0x0260 (datasheet Table 17).
        // For LTC2949 the exact bitmap variant (Normal mode, all cells) is don't-care; it
        // simply triggers the fast channels selected by FACTRL.
        self.send_cmd16(0x0260)
    }

    fn trigger_adax(&mut self) -> Result<(), Error<B>> {
        self.send_cmd16(0x0460)
    }
}

/// Inherent helpers: FIFO drains (generic over `N`, so off the trait) plus private
/// framing/decoding internals.
impl<B, P> LTC2949<B, P>
where
    B: SpiDevice<u8>,
    P: PollMethod<B>,
{
    fn encode_gain_factor(correction_factor: f32) -> Result<[u8; 3], Error<B>> {
        if !correction_factor.is_finite() || correction_factor <= 0.0 {
            return Err(Error::InvalidGainCorrection);
        }
        Ok(Float24::new(correction_factor).encode())
    }

    fn write_gain_factor(&mut self, address: RegAddressP1, correction_factor: f32) -> Result<(), Error<B>> {
        let encoded = Self::encode_gain_factor(correction_factor)?;
        self.write_bytes(address, &encoded)
    }

    fn positive_ratio(numerator: f32, denominator: f32) -> Result<f32, Error<B>> {
        if !numerator.is_finite() || numerator <= 0.0 || !denominator.is_finite() || denominator <= 0.0 {
            return Err(Error::InvalidGainCorrection);
        }
        let ratio = numerator / denominator;
        if !ratio.is_finite() || ratio <= 0.0 {
            return Err(Error::InvalidGainCorrection);
        }
        Ok(ratio)
    }

    fn accumulator_lsbs(clock: AccumulatorClock) -> Result<(f64, f64, f64), Error<B>> {
        match clock {
            AccumulatorClock::Internal => Ok((
                AccumulatedCharge::LSB_VOLT_SECONDS,
                AccumulatedEnergy::LSB_VOLT_SQUARED_SECONDS,
                AccumulatedTime::LSB_SECONDS,
            )),
            AccumulatorClock::External { frequency_hz, pre, div } => {
                if !frequency_hz.is_finite() || frequency_hz <= 0.0 || pre > 7 || div > 31 {
                    return Err(Error::InvalidThreshold);
                }
                let clock_factor = f64::from(1u16 << pre) * (f64::from(div) + 1.0) / frequency_hz;
                Ok((
                    1.21899e-5 * clock_factor,
                    7.4895e-5 * clock_factor,
                    12.8315 * clock_factor,
                ))
            }
        }
    }

    fn divide_lsb_by_shunt(lsb: f64, shunt_ohms: f64) -> Result<f64, Error<B>> {
        if !shunt_ohms.is_finite() || shunt_ohms <= 0.0 {
            return Err(Error::InvalidThreshold);
        }
        Ok(lsb / shunt_ohms)
    }

    fn quantize_signed(value: f64, lsb: f64, minimum: f64, maximum: f64) -> Result<i64, Error<B>> {
        if !value.is_finite() || !lsb.is_finite() || lsb <= 0.0 {
            return Err(Error::InvalidThreshold);
        }
        let scaled = value / lsb;
        if scaled <= minimum - 0.5 || scaled >= maximum + 0.5 {
            return Err(Error::InvalidThreshold);
        }
        Ok(if scaled >= 0.0 {
            (scaled + 0.5) as i64
        } else {
            (scaled - 0.5) as i64
        })
    }

    fn quantize_signed_16(value: f64, lsb: f64) -> Result<i16, Error<B>> {
        Self::quantize_signed(value, lsb, f64::from(i16::MIN), f64::from(i16::MAX)).map(|value| value as i16)
    }

    fn quantize_signed_48(value: f64, lsb: f64) -> Result<i64, Error<B>> {
        const MIN: f64 = -((1u64 << 47) as f64);
        const MAX: f64 = ((1u64 << 47) - 1) as f64;
        Self::quantize_signed(value, lsb, MIN, MAX)
    }

    fn quantize_signed_64(value: f64, lsb: f64) -> Result<i64, Error<B>> {
        // `i64::MAX as f64` rounds up to 2^63, so use an exclusive upper check before the cast.
        if !value.is_finite() || !lsb.is_finite() || lsb <= 0.0 {
            return Err(Error::InvalidThreshold);
        }
        let scaled = value / lsb;
        if !(-9_223_372_036_854_775_808.0..9_223_372_036_854_775_808.0).contains(&scaled) {
            return Err(Error::InvalidThreshold);
        }
        Ok(if scaled >= 0.0 {
            (scaled + 0.5) as i64
        } else {
            (scaled - 0.5) as i64
        })
    }

    fn quantize_unsigned_32(value: f64, lsb: f64) -> Result<u32, Error<B>> {
        if !value.is_finite() || !lsb.is_finite() || lsb <= 0.0 {
            return Err(Error::InvalidThreshold);
        }
        let scaled = value / lsb;
        if value < 0.0 || scaled >= f64::from(u32::MAX) + 0.5 {
            return Err(Error::InvalidThreshold);
        }
        Ok((scaled + 0.5) as u32)
    }

    fn write_signed_16_threshold_pair(
        &mut self,
        high_address: RegAddressP1,
        low: f64,
        high: f64,
        lsb: f64,
    ) -> Result<(), Error<B>> {
        if low > high {
            return Err(Error::InvalidThreshold);
        }
        let high = Self::quantize_signed_16(high, lsb)?;
        let low = Self::quantize_signed_16(low, lsb)?;
        let mut bytes = [0u8; 4];
        bytes[..2].copy_from_slice(&high.to_be_bytes());
        bytes[2..].copy_from_slice(&low.to_be_bytes());
        self.write_bytes(high_address, &bytes)
    }

    fn write_signed_48_threshold_pair(
        &mut self,
        high_address: RegAddressP1,
        low: f64,
        high: f64,
        lsb: f64,
    ) -> Result<(), Error<B>> {
        if low > high {
            return Err(Error::InvalidThreshold);
        }
        let high = Self::quantize_signed_48(high, lsb)?.to_be_bytes();
        let low = Self::quantize_signed_48(low, lsb)?.to_be_bytes();
        let mut bytes = [0u8; 12];
        bytes[..6].copy_from_slice(&high[2..]);
        bytes[6..].copy_from_slice(&low[2..]);
        self.write_bytes(high_address, &bytes)
    }

    fn write_signed_64_threshold_pair(
        &mut self,
        high_address: RegAddressP1,
        low_address: RegAddressP1,
        low: f64,
        high: f64,
        lsb: f64,
    ) -> Result<(), Error<B>> {
        if low > high {
            return Err(Error::InvalidThreshold);
        }
        let high = Self::quantize_signed_64(high, lsb)?.to_be_bytes();
        let low = Self::quantize_signed_64(low, lsb)?.to_be_bytes();
        self.write_bytes(high_address, &high)?;
        self.write_bytes(low_address, &low)
    }

    /// Drains up to `N` I1 FIFO samples (3 bytes each: MSB, LSB, TAG). Stops at the first
    /// non-`Ok` sample, which is included as the terminator so the caller can read its `tag`.
    pub fn read_fifo_i1<const N: usize>(&mut self) -> Result<Vec<FifoSample, N>, Error<B>> {
        self.read_fifo::<N>(RegAddressP0::FifoI1)
    }

    /// Drains up to `N` samples from the I2 FIFO.
    pub fn read_fifo_i2<const N: usize>(&mut self) -> Result<Vec<FifoSample, N>, Error<B>> {
        self.read_fifo::<N>(RegAddressP0::FifoI2)
    }

    /// Drains up to `N` samples from the BAT (P1/P2 voltage-mode) FIFO.
    pub fn read_fifo_bat<const N: usize>(&mut self) -> Result<Vec<FifoSample, N>, Error<B>> {
        self.read_fifo::<N>(RegAddressP0::FifoBat)
    }

    /// Drains up to `N` samples from the AUX FIFO.
    pub fn read_fifo_aux<const N: usize>(&mut self) -> Result<Vec<FifoSample, N>, Error<B>> {
        self.read_fifo::<N>(RegAddressP0::FifoAux)
    }

    fn read_fifo<const N: usize>(&mut self, reg: RegAddressP0) -> Result<Vec<FifoSample, N>, Error<B>> {
        // The FIFO register is non-incrementing: each 3-byte group within a burst pops the
        // next sample (datasheet "Reading the FIFOs"), so one DCMD read of 3·k bytes yields
        // k samples. Drain in bursts and stop at the first non-`Ok` tag (kept as terminator).
        let mut samples: Vec<FifoSample, N> = Vec::new();
        let mut buf = [0u8; FIFO_SAMPLES_PER_BURST * 3];
        let mut remaining = N;
        while remaining > 0 {
            let batch = remaining.min(FIFO_SAMPLES_PER_BURST);
            let bytes = &mut buf[..batch * 3];
            self.read_bytes(reg, bytes)?;
            for sample in bytes.chunks_exact(3) {
                let raw = ((sample[0] as u16) << 8 | sample[1] as u16) as i16;
                let tag = FifoTag::from_byte(sample[2]);
                let stop = !matches!(tag, FifoTag::Ok);
                let _ = samples.push(FifoSample { raw, tag });
                if stop {
                    return Ok(samples);
                }
            }
            remaining -= batch;
        }
        Ok(samples)
    }

    fn read_signed_16(&mut self, reg: RegAddressP0) -> Result<i16, Error<B>> {
        let mut buf = [0u8; 2];
        self.read_bytes(reg, &mut buf)?;
        Ok(i16::from_be_bytes(buf))
    }

    fn read_signed_24(&mut self, reg: RegAddressP0) -> Result<i32, Error<B>> {
        let mut buf = [0u8; 3];
        self.read_bytes(reg, &mut buf)?;
        // Sign-extend 24 -> 32.
        let raw = (u32::from(buf[0]) << 16) | (u32::from(buf[1]) << 8) | u32::from(buf[2]);
        let extended = if raw & 0x0080_0000 != 0 { raw | 0xFF00_0000 } else { raw };
        Ok(extended as i32)
    }

    fn read_unsigned_32(&mut self, reg: RegAddressP0) -> Result<u32, Error<B>> {
        let mut buf = [0u8; 4];
        self.read_bytes(reg, &mut buf)?;
        Ok(u32::from_be_bytes(buf))
    }

    fn read_signed_48(&mut self, reg: RegAddressP0) -> Result<i64, Error<B>> {
        let mut buf = [0u8; 6];
        self.read_bytes(reg, &mut buf)?;
        Ok(Self::sign_extend_48(&buf))
    }

    /// Reads a full 16-byte accumulator row (charge, energy, time) in one coherent burst,
    /// starting at the row's charge address (`0x00` for channel 1, `0x10` for channel 2).
    fn read_accumulator_row(&mut self, reg: RegAddressP0) -> Result<Accumulators, Error<B>> {
        let mut buf = [0u8; 16];
        self.read_bytes(reg, &mut buf)?;
        Ok(Accumulators {
            charge: AccumulatedCharge::from_raw(Self::sign_extend_48(&buf[0..6])),
            energy: AccumulatedEnergy::from_raw(Self::sign_extend_48(&buf[6..12])),
            time: AccumulatedTime::from_raw(u32::from_be_bytes([buf[12], buf[13], buf[14], buf[15]])),
        })
    }

    fn read_signed_64(&mut self, reg: RegAddressP0) -> Result<i64, Error<B>> {
        let mut buf = [0u8; 8];
        self.read_bytes(reg, &mut buf)?;
        Ok(i64::from_be_bytes(buf))
    }

    /// REGSCTRL for the current page with RDCVCONF=1, BCREN=0, MLK=00. Lets the memory-lock
    /// helpers rewrite REGSCTRL without changing the selected page.
    fn regsctrl_base(&self) -> RegControlRegister {
        let page1 = matches!(self.current_page.unwrap_or(Page::Page0), Page::Page1);
        RegControlRegister::default().with_rdcvconf(true).with_page(page1)
    }

    fn select_page(&mut self, page: Page) -> Result<(), Error<B>> {
        if self.current_page == Some(page) {
            return Ok(());
        }
        let value = RegControlRegister::default()
            .with_rdcvconf(true)
            .with_page(matches!(page, Page::Page1));
        self.dcmd_write(RegAddressP0::RegsCtrl.addr(), &value.into_bytes())?;
        self.current_page = Some(page);
        Ok(())
    }

    /// Reads `buf.len()` bytes from `reg` via a direct `DCMD` read (no shift-register
    /// prefix in the parallel topology), selecting the page first if needed.
    fn read_bytes<R: Register>(&mut self, reg: R, buf: &mut [u8]) -> Result<(), Error<B>> {
        self.select_page(R::PAGE)?;
        self.dcmd_read(reg.addr(), buf)
    }

    /// Writes `data` to `reg` via `DCMD`. The cell monitors ignore command 0xFE.
    fn write_bytes<R: Register>(&mut self, reg: R, data: &[u8]) -> Result<(), Error<B>> {
        // REGSCTRL writes are themselves the page-switch mechanism; avoid recursion.
        if reg.addr() != RegAddressP0::RegsCtrl.addr() {
            self.select_page(R::PAGE)?;
        }
        self.dcmd_write(reg.addr(), data)
    }

    /// Sends a DCMD write transaction. `data` must fit one PEC group (≤ 16 bytes); every
    /// caller already stays within that (the 9-byte NTC burst is the largest).
    /// DCMD frame layout (datasheet Table 11):
    ///
    ///   [0xFE, RADDR, PEC0, PEC1, ID, D0..D(N-1), PEC0, PEC1, D(N)..D(2N-1), PEC0, PEC1, ...]
    ///
    /// The PEC on bytes 2..4 is computed over [0xFE, RADDR]. Each subsequent PEC covers the
    /// preceding `N` data bytes. `N` is encoded in the ID byte's PECC field
    /// (`PECC = N-1`, range 0..=15). The ID byte itself carries redundancy so it is not
    /// covered by a PEC.
    fn dcmd_write(&mut self, addr: u8, data: &[u8]) -> Result<(), Error<B>> {
        // Frame: 4 (header+PEC) + 1 (ID) + ≤16 (data) + 2 (PEC) = 23 bytes max.
        assert!(
            (1..=N_PER_PEC).contains(&data.len()),
            "DCMD write requires 1 to 16 data bytes"
        );

        let mut frame = [0u8; 23];
        frame[0] = 0xFE;
        frame[1] = addr;
        let header_pec = PEC15::calc(&frame[0..2]);
        frame[2] = header_pec[0];
        frame[3] = header_pec[1];
        let n = data.len();
        frame[4] = DcmdId::write((n - 1) as u8).into();
        frame[5..5 + n].copy_from_slice(data);
        let data_pec = PEC15::calc(&frame[5..5 + n]);
        frame[5 + n] = data_pec[0];
        frame[6 + n] = data_pec[1];

        let total = 5 + n + 2;
        self.bus.write(&frame[..total]).map_err(Error::BusError)?;
        self.poll_method.end_sync_command(&mut self.bus).map_err(Error::BusError)?;
        Ok(())
    }

    // -- DCMD direct read -------------------------------------------------

    /// Direct `DCMD` read: data appears on MISO after the 5-byte header, then the PEC.
    /// `buf` must fit one PEC group (≤ 16 bytes); the 16-byte accumulator row is the largest.
    fn dcmd_read(&mut self, addr: u8, buf: &mut [u8]) -> Result<(), Error<B>> {
        assert!(
            (1..=N_PER_PEC).contains(&buf.len()),
            "DCMD read requires 1 to 16 data bytes"
        );

        let n = buf.len();
        // MOSI: header (4) + ID (1) + n dummy data bytes + 2 dummy PEC bytes.
        // MISO: 5 don't-care (command echo region) + n data + 2 PEC.
        let mut mosi = [0xFFu8; 23];
        let mut miso = [0x00u8; 23];

        mosi[0] = 0xFE;
        mosi[1] = addr;
        let header_pec = PEC15::calc(&mosi[0..2]);
        mosi[2] = header_pec[0];
        mosi[3] = header_pec[1];
        mosi[4] = DcmdId::read((n - 1) as u8).into();
        // bytes 5..5+n and the trailing PEC bytes stay 0xFF (don't-care on MOSI).

        let total = 5 + n + 2;
        self.bus
            .transaction(&mut [Operation::Transfer(&mut miso[..total], &mosi[..total])])
            .map_err(Error::BusError)?;

        buf.copy_from_slice(&miso[5..5 + n]);

        let pec = PEC15::calc(&miso[5..5 + n]);
        if pec[0] != miso[5 + n] || pec[1] != miso[6 + n] {
            // A bad PEC can mean the device reset (e.g. unexpected SLEEP/POR), in which
            // case its page selection is back to PAGE0. Drop the cache so the next access
            // re-issues REGSCTRL rather than trusting a possibly-stale page.
            self.current_page = None;
            return Err(Error::ChecksumMismatch);
        }

        self.poll_method.end_sync_command(&mut self.bus).map_err(Error::BusError)?;
        Ok(())
    }

    fn send_cmd16(&mut self, cmd: u16) -> Result<(), Error<B>> {
        let frame = Self::build_cmd16(cmd);
        self.bus.write(&frame).map_err(Error::BusError)?;
        self.poll_method.end_sync_command(&mut self.bus).map_err(Error::BusError)?;
        Ok(())
    }

    /// Sign-extends a big-endian 48-bit two's-complement value (`bytes[0..6]`) to `i64`.
    fn sign_extend_48(bytes: &[u8]) -> i64 {
        let mut raw: u64 = 0;
        for &b in &bytes[..6] {
            raw = (raw << 8) | u64::from(b);
        }
        if raw & 0x0000_8000_0000_0000 != 0 {
            raw |= 0xFFFF_0000_0000_0000;
        }
        raw as i64
    }

    /// Builds a 4-byte LTC681X-style 16-bit command (CMD0, CMD1, PEC0, PEC1).
    fn build_cmd16(cmd: u16) -> [u8; 4] {
        let mut frame = [(cmd >> 8) as u8, cmd as u8, 0, 0];
        let pec = PEC15::calc(&frame[..2]);
        frame[2] = pec[0];
        frame[3] = pec[1];
        frame
    }
}
