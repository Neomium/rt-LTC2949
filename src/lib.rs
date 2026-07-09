//! # Driver for the [LTC2949](<https://www.analog.com/en/products/ltc2949.html>) current, voltage, charge and energy monitor.
//!
//! The LTC2949 is **not** a member of the LTC681X cell-monitor family - its register map (paginated, 256+
//! bytes) and command framing (`DCMD` 0xFE with an ID byte and configurable PEC stride) are
//! distinct from the cell-monitor ADCV/RDCV model. It does however share isoSPI signalling
//! and the PEC15 checksum with the LTC681X family and is intended to live on the same bus.
//!
//! ## Bus topology
//!
//! This driver targets the **addressable, parallel-to-daisy-chain** arrangement of
//! datasheet Figure 12(B): the LTC2949 hangs off the isoSPI bus *in parallel* with a
//! daisy chain of LTC68xx cell monitors (typically via an LTC6820), rather than being
//! chained inside it. Because the LTC2949 is reached directly:
//!
//! * **Register reads and writes both use `DCMD`** (command 0xFE). The cell monitors do
//!   not understand `DCMD`, so it implicitly addresses the LTC2949 and its response
//!   comes straight back with no shift-register prefix to discard (datasheet Table 10,
//!   "Parallel to Daisy Chain or LTC2949 Sole").
//! * **`BCREN` is kept cleared** (the default) so the LTC2949 ignores broadcast RDCV
//!   reads and never collides on the bus with the cell monitors.
//! * Synchronous fast conversions are triggered with a **broadcast `ADCV`** (reaching the
//!   LTC2949 and every cell monitor at once); the LTC2949's own fast results, if needed,
//!   are read back with an **addressed RDCV** (`RDCVCONF = 1`, `BCREN = 0`).
//!
//! The same framing also covers the "LTC2949 sole" case (no cell monitors on the bus at
//! all), so no separate construction is needed for that.
//!
//! ## Scope
//!
//! Initial coverage matches the user-confirmed scope:
//!
//! * Wake-up, mode control (OPCTRL), ADC configuration (ADCCONF). Timed operations are
//!   split into non-blocking halves that return the required wait in microseconds —
//!   mirroring `CommandTime` on the cell-monitor client —
//!   so hosts with cooperative schedulers own the waiting themselves (no `DelayNs` seam).
//! * Slow-mode result registers (`I1`, `I2`, `P1`, `P2`, `BAT`, `TEMP`, `VCC`, `SLOT1/2`).
//! * Accumulators (charge `C1..C3`, energy `E1/E2/E4`, time `TB1..TB4`), plus the
//!   memory-lock handshake for coherent multi-register snapshots.
//! * Decoded status/fault registers (`STATUS`, `FAULTS`, `EXTFAULTS`).
//! * Fast mode trigger (`FACTRL`, broadcast `ADCV`) and FIFO drain.
//! * Steinhart–Hart linearisation coefficients for the two NTC channels
//!   ([`client::Client::write_ntc_coefficients`]), including the `f32 → Float24` encoding.
//!
//! The overcurrent-comparator threshold registers (used to arm the hardware
//! `LTC_OVERCURRENT` one-shot) and EEPROM are deliberately out of scope for this cut;
//! `read_faults`/`read_extfaults` expose the fault bytes but no typed bitfields yet.
//!
//! ## Example
//!
//! End-to-end initialisation for a 3 × LTC6813 stack with an LTC2949 sitting on
//! top. The example follows the datasheet-prescribed procedure for shunt
//! temperature compensation (Table 77, "Procedure to Enable Temperature
//! Compensation of Sense Resistor") so every register touched is something the
//! datasheet asks you to write before leaving STANDBY.
//!
//! ```
//! use ltc2949::client::{
//!     AdcConfiguration, Channel, Client, FastControlRegister, FifoTag, MuxInput, NtcConfig, OpsControlRegister, ShuntTcConfig,
//!     LTC2949, T_BOOT_US,
//! };
//! use embedded_hal::delay::DelayNs;
//! # use ltc2949::example::{ExampleDelay, ExampleSPIDevice};
//! # (|| -> Result<(), ltc2949::client::Error<ExampleSPIDevice>> {
//!
//! // Mock Hardware peripherals for example
//! let spi = ExampleSPIDevice::default();
//! let mut delay = ExampleDelay::default();
//!
//! // The LTC2949 hangs off the isoSPI bus in parallel with the cell-monitor chain.
//! let mut client = LTC2949::new(spi);
//!
//! // ---- Step 0 – wake the device ---------------------------------------
//! // Two dummy bytes provide the isoSPI wake edge; the device then needs tBOOT
//! // (≤100 ms, the returned value) to reach STANDBY. The driver never blocks:
//! // the host waits the returned microseconds its own way (timer poll, delay, RTOS),
//! // then confirms the wake-up so the core doesn't auto-sleep again after 1 s.
//! let boot_us = client.start_wake_up()?;
//! assert_eq!(T_BOOT_US, boot_us);
//! delay.delay_us(boot_us);
//! client.confirm_wake_up()?;
//!
//! // ---- Step 1 – stay in STANDBY (CONT=0) ------------------------------
//! // The wake-up sequence leaves the core in STANDBY; if it was already in
//! // MEASURE you'd need: client.write_opctrl(OpCtrl::default())?;
//! //
//! // It is also recommended to check STATUS / FAULTS / EXTFAULTS here and
//! // clear any UVLO/POR flags. All three registers have typed accessors.
//! let _power_on_status = client.read_status()?;
//!
//! // ---- Step 2a – program NTC1 coefficients ----------------------------
//! // Example values from datasheet Table 75 (Vishay NTCLE203E3103SB0, 10 kΩ
//! // reference divider).
//! client.write_ntc_coefficients(
//!     Channel::One,
//!     &NtcConfig {
//!         r_ref: 10_000.0,
//!         a: 1.1382e-3,
//!         b: 2.3267e-4,
//!         c: 0.93243e-7,
//!     },
//! )?;
//!
//! // ---- Step 2b – program shunt-1 temperature compensation -------------
//! // Copper sense resistor with TC = 3900 ppm/K, R_nominal characterised at
//! // 25 °C. Set both terms to 0.0 if you use a low-TC alloy and don't need
//! // compensation.
//! client.write_shunt_tc(
//!     Channel::One,
//!     &ShuntTcConfig { tc: 0.0039, t_ref: 25.0, tc2: 0.0 },
//! )?;
//!
//! // ---- Step 3 – tell the AUX ADC to report SLOT1 as a temperature -----
//! // NTC1 on, both power ADCs left in power-mode (P1ASV/P2ASV = 0).
//! client.write_adcconf(AdcConfiguration::default().with_ntc1(true))?;
//!
//! // ---- Step 4 – route the NTC's V-pin into SLOT1 ----------------------
//! // Typical wiring: VREF -- R_ref -- V1 -- NTC -- AGND. We read the V1 pin
//! // single-ended (MUXP = V1, MUXN = AGND); the device's NTC1 lineariser
//! // combines the reading with R_ref to compute R_NTC and then the
//! // temperature via Steinhart-Hart.
//! client.write_slot_mux(Channel::One, MuxInput::Agnd, MuxInput::V1)?;
//!
//! // ---- Step 5 – commit page-1 configuration (ADJUPD) ------------------
//! // OPCTRL.ADJUPD is a set-only bit; the device clears it after ≤100 ms.
//! // Poll OPCTRL.adjupd() == false to know when the update has landed.
//! client.write_opctrl(OpsControlRegister::default().with_adjupd(true))?;
//! delay.delay_ms(100);
//!
//! // ---- Step 6 – enter continuous slow-mode measurement ----------------
//! // ≈100 ms per cycle, 18-bit results. First update lands ~50 ms later.
//! client.write_opctrl(OpsControlRegister::default().with_cont(true))?;
//! delay.delay_ms(100);
//!
//! // ---- Slow-mode reads ------------------------------------------------
//! // After ≥100 ms of CONT, the result registers are populated. LSB sizes
//! // are documented on each method; here are the conversions you'll
//! // typically pair with each value.
//! let i1_raw = client.read_current1()?;       // i32, LSB = 950 nV
//! let v_shunt_uv = (i1_raw as i64) * 950 / 1_000;
//! // For a 100 µΩ shunt: current_uA = v_shunt_uv * 10
//!
//! let bat_raw = client.read_bat()?;           // i16, LSB = 375 µV pin-to-pin
//! let bat_uv  = bat_raw as i32 * 375;
//! assert_eq!(4_629_375, bat_uv);
//!
//! let slot1_raw = client.read_slot1()?;       // i16, LSB = 0.2 °C (NTC mode)
//! let temp_decic = slot1_raw as i32 * 2;      // tenths of a °C
//! assert_eq!(250, temp_decic);
//!
//! // ---- Charge accumulation (state-of-charge) --------------------------
//! // C1 keeps integrating current as long as CONT is set; coulombs are
//! // C1 · 377.887 ps · V / R_shunt (internal clock or 4 MHz crystal).
//! let charge1_raw = client.read_charge1()?;   // i64 (48-bit signed)
//! // For a 100 µΩ shunt:
//! //   coulombs = charge1_raw * 377.887e-12 / 100e-6
//!
//! // ---- Fast measurements synchronised with the cell monitors ----------
//! // Configure channels 1 and 2 for fast single-shot, then broadcast ADCV
//! // so the LTC2949 and every LTC6813 trigger at the same instant.
//! client.write_factrl(FastControlRegister::default().with_fach1(true).with_fach2(true))?;
//! client.trigger_adcv_broadcast()?;
//! delay.delay_us(800);
//!
//! // Fast continuous: set FACONV=1 and drain the FIFO periodically.
//! client.write_factrl(
//!     FastControlRegister::default().with_faconv(true).with_fach1(true).with_fach2(true),
//! )?;
//! delay.delay_us(1_260);
//! assert_eq!(302_060, delay.elapsed_us());
//!
//! // Then read up to 32 samples at a time:
//! let samples = client.read_fifo_i1::<32>()?;
//! for s in &samples {
//!     // 16-bit signed, LSB = 7.60371 µV across the shunt.
//!     let uv = (s.raw as i32 * 760_371) / 100_000;
//!     if s.tag == FifoTag::Ok {
//!         assert_eq!(1_946, uv);
//!     }
//! }
//! # Ok(())
//! # })().unwrap();
//! ```
//!
//! ## Alongside LTC6813 / LTC6812 cell monitors on one isoSPI bus
//!
//! In the Figure 12(B) topology the LTC2949 and a daisy chain of LTC68xx cell monitors
//! share a single isoSPI link (usually one LTC6820). At the `embedded-hal` level that one
//! physical [`SpiBus`](embedded_hal::spi::SpiBus) is shared between two
//! [`embedded_hal::spi::SpiDevice`] handles — one driving the
//! [`client::Client`], the other driving the [LTC681X client](https://docs.rs/ltc681x/0.6.2/ltc681x/monitor/trait.LTC681XClient.html) cell
//! monitor client — typically via `embedded-hal-bus` (e.g. `RefCellDevice`,
//! `AtomicDevice` or `CriticalSectionDevice`).
//!
//! Because both devices sit on the same wire, a single **broadcast `ADCV`** triggers a
//! synchronous conversion on every device at once: the cell monitors convert their cells
//! and the LTC2949 converts whichever fast channels its `FACTRL` selects. Issuing the
//! broadcast through the cell-monitor client puts exactly one `ADCV` on the bus, which the
//! parallel LTC2949 also hears.
//!
//! ```
//! use ltc2949::client::{FastControlRegister, Client, LTC2949, OpsControlRegister};
//! use ltc681x::ltc6813::{CellSelection, LTC6813};
//! use ltc681x::monitor::{ADCMode, LTC681X, LTC681XClient};
//!
//! // `meter_spi` and `chain_spi` are two SpiDevice handles to the *same* isoSPI bus
//! // (build them from one shared SpiBus with embedded-hal-bus). All waits below are
//! // host-owned — poll a timer, sleep, or delay as the platform allows. For an LTC6812
//! // stack, swap the type and the chain length — the flow is identical.
//! # fn demo<M, C>(meter_spi: M, chain_spi: C) -> Result<(), ()>
//! # where
//! #     M: embedded_hal::spi::SpiDevice<u8>,
//! #     C: embedded_hal::spi::SpiDevice<u8>,
//! # {
//! // The LTC2949 hangs off the bus in parallel and is addressed directly via DCMD.
//! let mut meter = LTC2949::new(meter_spi);
//! // Three LTC6813 cell monitors form the daisy chain.
//! let mut chain: LTC681X<_, _, LTC6813, 3> = LTC681X::ltc6813(chain_spi);
//!
//! // ---- Configure the meter (datasheet "Single Shunt Configuration") -----------
//! // CH1 stays in slow high-precision mode for charge/energy integration; CH2 is set
//! // to fast mode so a broadcast ADCV makes it snapshot synchronously with the cells.
//! let boot_us = meter.start_wake_up().map_err(|_| ())?;
//! // ... host waits `boot_us` microseconds ...
//! meter.confirm_wake_up().map_err(|_| ())?;
//! meter.write_opctrl(OpsControlRegister::default().with_cont(true)).map_err(|_| ())?; // CONT prereq
//! meter.write_factrl(FastControlRegister::default().with_fach2(true)).map_err(|_| ())?; // CH2 fast
//!
//! // ---- One broadcast ADCV → both devices convert at the same instant ----------
//! // start_conv_cells emits a *broadcast* ADCV (A/B address bits = 0), so it reaches
//! // the daisy chain and the parallel LTC2949 alike. Its CommandTime reports the cell
//! // conversion duration (all groups, normal mode ≈ 2.3 ms); the LTC2949 fast channel
//! // finishes in well under 1 ms, so waiting out the CommandTime covers both.
//! let timing = chain
//!     .start_conv_cells(ADCMode::Normal, CellSelection::All, false)
//!     .map_err(|_| ())?;
//! // ... host waits `timing.regular` microseconds ...
//!
//! // ---- Read each device back over its own handle ------------------------------
//! let cell_voltages = chain.read_voltages(CellSelection::All).map_err(|_| ())?;
//!
//! // CH1 high-precision current (continuously updated while CONT is set).
//! let i1_raw = meter.read_current1().map_err(|_| ())?; // i32, LSB = 950 nV
//!
//! // The CH2 fast snapshot triggered above is read from the LTC2949's fast path; the
//! // FIFO drain (see the previous example, `read_fifo_i2`) is the supported route when
//! // running fast-continuous (FACONV = 1).
//! let _ = (cell_voltages, i1_raw);
//! # Ok(())
//! # }
//! ```
#![cfg_attr(not(test), no_std)]
#![cfg_attr(feature = "strict", deny(warnings))]

#[cfg(test)]
extern crate alloc;

pub use heapless;

pub mod client;
#[cfg(feature = "example")]
pub mod example;
pub mod float24;
pub mod polling;
pub mod spi;

pub(crate) mod pec15;

#[cfg(test)]
mod mocks;
#[cfg(test)]
mod tests;
