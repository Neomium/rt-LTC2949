//! Driver for the [LTC2949](<https://www.analog.com/en/products/ltc2949.html>) current,
//! voltage, charge and energy monitor.
//!
//! The LTC2949 is **not** a member of the LTC681X cell-monitor family and does not implement
//! the [`DeviceTypes`](crate::monitor::DeviceTypes) trait — its register map (paginated, 256+
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
//!   mirroring [`CommandTime`](crate::monitor::CommandTime) on the cell-monitor client —
//!   so hosts with cooperative schedulers own the waiting themselves (no `DelayNs` seam).
//! * Slow-mode result registers (`I1`, `I2`, `P1`, `P2`, `BAT`, `TEMP`, `VCC`, `SLOT1/2`).
//! * Accumulators (charge `C1..C3`, energy `E1/E2/E4`, time `TB1..TB4`), plus the
//!   memory-lock handshake for coherent multi-register snapshots.
//! * Fault status (`STATUS`, `FAULTS`, `EXTFAULTS`) as raw bytes.
//! * Fast mode trigger (`FACTRL`, broadcast `ADCV`) and FIFO drain.
//! * Steinhart–Hart linearisation coefficients for the two NTC channels
//!   ([`LTC2949::write_ntc_coefficients`]), including the `f32 → Float24` encoding.
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
//! ```ignore
//! use ltc681x::ltc2949::{
//!     AdcConf, Channel, FaCtrl, Ltc2949Client, MuxInput, NtcConfig, OpCtrl, ShuntTcConfig, LTC2949,
//! };
//!
//! # fn demo<S>(spi: S) -> Result<(), ltc681x::ltc2949::Error<S>>
//! # where
//! #     S: embedded_hal::spi::SpiDevice<u8>,
//! # {
//! // The LTC2949 hangs off the isoSPI bus in parallel with the cell-monitor chain.
//! let mut client = LTC2949::new(spi);
//!
//! // ---- Step 0 – wake the device ---------------------------------------
//! // Two dummy bytes provide the isoSPI wake edge; the device then needs tBOOT
//! // (≤100 ms, the returned value) to reach STANDBY. The driver never blocks:
//! // the host waits the returned microseconds its own way (timer poll, delay, RTOS),
//! // then confirms the wake-up so the core doesn't auto-sleep again after 1 s.
//! let boot_us = client.start_wake_up()?;
//! // ... host waits `boot_us` microseconds ...
//! client.confirm_wake_up()?;
//!
//! // ---- Step 1 – stay in STANDBY (CONT=0) ------------------------------
//! // The wake-up sequence leaves the core in STANDBY; if it was already in
//! // MEASURE you'd need: client.write_opctrl(OpCtrl::new())?;
//! //
//! // It is also recommended to check STATUS / FAULTS / EXTFAULTS here and
//! // clear any UVLO/POR flags. The driver exposes read_status(); the FAULTS
//! // registers are not yet wrapped in typed accessors.
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
//! client.write_adcconf(AdcConf::new().with_ntc1(true))?;
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
//! client.write_opctrl(OpCtrl::new().with_adjupd(true))?;
//!
//! // ---- Step 6 – enter continuous slow-mode measurement ----------------
//! // ≈100 ms per cycle, 18-bit results. First update lands ~50 ms later.
//! client.write_opctrl(OpCtrl::new().with_cont(true))?;
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
//!
//! let slot1_raw = client.read_slot1()?;       // i16, LSB = 0.2 °C (NTC mode)
//! let temp_decic = slot1_raw as i32 * 2;      // tenths of a °C
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
//! client.write_factrl(FaCtrl::new().with_fach1(true).with_fach2(true))?;
//! client.trigger_adcv_broadcast()?;
//! // ... wait the fast conversion time (~0.8 ms) ...
//!
//! // Fast continuous: set FACONV=1 and drain the FIFO periodically.
//! client.write_factrl(
//!     FaCtrl::new().with_faconv(true).with_fach1(true).with_fach2(true),
//! )?;
//! // ... wait ≥ 1.26 ms for the first sample, then read up to 32 at a time:
//! let samples = client.read_fifo_i1::<32>()?;
//! for s in &samples {
//!     // 16-bit signed, LSB = 7.60371 µV across the shunt.
//!     let _uv = (s.raw as i32 * 760_371) / 100_000;
//! }
//! # Ok(())
//! # }
//! ```
//!
//! ## Alongside LTC6813 / LTC6812 cell monitors on one isoSPI bus
//!
//! In the Figure 12(B) topology the LTC2949 and a daisy chain of LTC68xx cell monitors
//! share a single isoSPI link (usually one LTC6820). At the `embedded-hal` level that one
//! physical [`SpiBus`](embedded_hal::spi::SpiBus) is shared between two
//! [`SpiDevice`] handles — one driving the
//! [`LTC2949`] client, the other driving the [`LTC681X`](crate::monitor::LTC681X) cell
//! monitor client — typically via `embedded-hal-bus` (e.g. `RefCellDevice`,
//! `AtomicDevice` or `CriticalSectionDevice`).
//!
//! Because both devices sit on the same wire, a single **broadcast `ADCV`** triggers a
//! synchronous conversion on every device at once: the cell monitors convert their cells
//! and the LTC2949 converts whichever fast channels its `FACTRL` selects. Issuing the
//! broadcast through the cell-monitor client puts exactly one `ADCV` on the bus, which the
//! parallel LTC2949 also hears.
//!
//! ```ignore
//! use ltc681x::ltc2949::{FaCtrl, Ltc2949Client, LTC2949, OpCtrl};
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
//! meter.write_opctrl(OpCtrl::new().with_cont(true)).map_err(|_| ())?; // CONT prereq
//! meter.write_factrl(FaCtrl::new().with_fach2(true)).map_err(|_| ())?; // CH2 fast
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

// `modular-bitfield`'s macro expansion emits `pub field: (bool)` etc., which trips the
// `unused_parens` lint on newer rustc. Silence the lint module-wide.
#![allow(unused_parens)]

use crate::monitor::{NoPolling, PollMethod};
use crate::pec15::PEC15;
use embedded_hal::spi::{Operation, SpiDevice};
use heapless::Vec;
use modular_bitfield::prelude::*;

// ---------------------------------------------------------------------------
// Timing constants (microseconds)
//
// The driver never blocks. Operations with a mandatory settling time are split into
// non-blocking halves; the first half returns the wait the host must observe before
// issuing the second, mirroring `CommandTime` on the cell-monitor client. The constants
// are public so hosts can also schedule against them directly.
// ---------------------------------------------------------------------------

/// Worst-case core boot time from SLEEP/power-up to STANDBY (datasheet tBOOT).
/// Returned by [`Ltc2949Client::start_wake_up`].
pub const T_BOOT_US: u32 = 100_000;

/// isoSPI port start-up time after a wake edge (datasheet tREADY, 10 µs; doubled for margin).
/// Returned by [`Ltc2949Client::wake_isospi`].
pub const T_READY_US: u32 = 20;

/// Worst-case memory-lock acknowledge time (datasheet tMLCK in MEASURE mode, 130 ms;
/// 40 ms in STANDBY — the MEASURE value is returned as the safe upper bound).
/// Returned by [`Ltc2949Client::request_memory_lock`].
pub const T_MLCK_US: u32 = 130_000;

// ---------------------------------------------------------------------------
// Register map (only the entries we expose are named; full map is in the
// datasheet, sections "Register Map PAGE0/PAGE1").
// ---------------------------------------------------------------------------

/// Memory page selector. PAGE0 holds measurement results, status and control;
/// PAGE1 holds thresholds and configuration. Selected via the [`RegsCtrl`] register.
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub enum Page {
    Page0 = 0,
    Page1 = 1,
}

/// Page-0 register addresses (datasheet Tables 24, 26-28, 57-64). PAGE0 holds the
/// measurement results, accumulators, status and the control/fast-mode registers. The
/// `#[repr(u8)]` discriminant is the on-bus `RADDR` byte.
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[repr(u8)]
pub enum Page0Reg {
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
    Current1 = 0x90, // 24-bit signed
    Power1 = 0x93,
    Current2 = 0x96,
    Power2 = 0x99,
    Bat = 0xA0, // 16-bit
    Temp = 0xA2,
    Vcc = 0xA4,
    Slot1 = 0xA6,
    Slot2 = 0xA8,
    Vref = 0xAA,
    // Slow-mode auxiliary-MUX slot selection (datasheet Tables 57 & 58). SLOT1/2 each
    // have separate MUXN / MUXP registers, adjacent so a 2-byte burst sets both.
    Slot1MuxN = 0xEB,
    Slot1MuxP = 0xEC,
    Slot2MuxN = 0xED,
    Slot2MuxP = 0xEE,
    ExtFaults = 0xDC,
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

/// Page-1 register addresses (datasheet Tables 69, 71, 76). PAGE1 holds thresholds and
/// configuration — here the ADC config plus the NTC-linearisation and sense-resistor
/// temperature-compensation coefficient blocks. The `#[repr(u8)]` discriminant is the
/// on-bus `RADDR` byte.
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[repr(u8)]
pub enum Page1Reg {
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
}

/// A device register that resolves to a memory [`Page`] and an on-bus address byte.
/// Implemented by [`Page0Reg`] and [`Page1Reg`] so the framing helpers can take a
/// register and derive the page automatically rather than threading both by hand.
trait Register: Copy {
    const PAGE: Page;
    fn addr(self) -> u8;
}

impl Register for Page0Reg {
    const PAGE: Page = Page::Page0;
    fn addr(self) -> u8 {
        self as u8
    }
}

impl Register for Page1Reg {
    const PAGE: Page = Page::Page1;
    fn addr(self) -> u8 {
        self as u8
    }
}

// ---------------------------------------------------------------------------
// Bit definitions
// ---------------------------------------------------------------------------

/// Operation Control register (PAGE0, 0xF0) — datasheet Table 24.
///
/// `clr`, `sshot`, `adjupd` and `rst` are set-only: the device clears them again once
/// the requested action has been performed. Polling the register lets you observe
/// completion (e.g. `read_opctrl().adjupd() == false`).
#[bitfield(bits = 8)]
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[repr(u8)]
pub struct OpCtrl {
    pub sleep: bool,
    pub clr: bool,
    pub sshot: bool,
    pub cont: bool,
    #[skip]
    __: B1,
    pub adjupd: bool,
    #[skip]
    __: B1,
    pub rst: bool,
}

/// Fast Control register (PAGE0, 0xF5) — datasheet Table 60.
#[bitfield(bits = 8)]
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[repr(u8)]
pub struct FaCtrl {
    pub faconv: bool,
    pub facha: bool,
    pub fach1: bool,
    pub fach2: bool,
    #[skip]
    __: B4,
}

/// ADC Configuration register (PAGE1, 0xDF) — datasheet Table 69.
///
/// `p1asv` / `p2asv` switch the corresponding power ADC into voltage mode.
/// `ntc1` / `ntc2` ask the device to linearise the matching SLOT through its
/// Steinhart–Hart coefficients. `ntcslot1` ties channel 2's shunt TC compensation
/// to NTC1 (single-shunt configuration).
#[bitfield(bits = 8)]
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[repr(u8)]
pub struct AdcConf {
    pub p1asv: bool,
    pub p2asv: bool,
    #[skip]
    __: B1,
    pub ntc1: bool,
    pub ntc2: bool,
    #[skip]
    __: B1,
    pub ntcslot1: bool,
    #[skip]
    __: B1,
}

/// Register Control register (common to both pages, 0xFF) — datasheet Table 23.
///
/// `mlk` is the 2-bit memory-lock handshake (`0b01` request, `0b10` acknowledged
/// from the device).
#[bitfield(bits = 8)]
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[repr(u8)]
pub struct RegsCtrl {
    pub page: bool,
    #[skip]
    __: B1,
    pub bcren: bool,
    #[skip]
    __: B1,
    pub mlk: B2,
    #[skip]
    __: B1,
    pub rdcvconf: bool,
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

/// Coherent snapshot of one channel's charge / energy / time-base accumulators.
///
/// On the device these three sit in a single 16-byte register row (channel 1 at
/// `0x00–0x0F`, channel 2 at `0x10–0x1F`). The datasheet guarantees coherency for a
/// multi-byte burst *within a row*, so reading all three in one burst yields a consistent
/// snapshot **without** the 130 ms memory lock — the values share the same `CONT` cycle.
/// Reading charge and time separately would otherwise skew by up to one 100 ms cycle.
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub struct Accumulators {
    /// Charge (48-bit two's-complement; units LSB·377.887 ps·V).
    pub charge: i64,
    /// Energy (48-bit two's-complement).
    pub energy: i64,
    /// Time base (32-bit unsigned).
    pub time: u32,
}

// ---------------------------------------------------------------------------
// NTC linearisation
// ---------------------------------------------------------------------------

/// One of the two LTC2949 measurement channels. Each channel pairs:
///
/// * a current/power ADC reading a single shunt (`I1`/`P1` vs. `I2`/`P2`),
/// * a SLOT in the auxiliary multiplexer (`SLOT1` / `SLOT2`),
/// * an NTC lineariser with its own reference resistor and Steinhart-Hart
///   coefficients (`RREF1`/`NTC1A-C` vs. `RREF2`/`NTC2A-C`),
/// * a sense-resistor temperature-compensation entry (`RS1TC`/`RS1T0`/`RS1TC2`
///   vs. `RS2*`).
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub enum Channel {
    One,
    Two,
}

/// Steinhart–Hart linearisation parameters for a single NTC channel. All four
/// values are stored on-chip in the device's custom 24-bit floating-point
/// "Float24" format; the driver handles the `f32 → Float24` conversion.
///
/// The Steinhart–Hart relation programmed by these coefficients is
///
/// ```text
/// 1 / T = A + B · ln(R_ntc) + C · (ln(R_ntc))³
/// ```
///
/// where `R_ntc` is inferred from the divider `R_ref` and the ADC measurement of
/// the NTC's pin voltage relative to `VREF` (datasheet "Temperature Measurement").
/// Typical magnitudes: `a ≈ 1e-3`, `b ≈ 2e-4`, `c ≈ 1e-7`.
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

/// Encodes an `f32` in the LTC2949's 24-bit floating-point "Float24" format
/// (datasheet Table 68): 1 sign bit, 7-bit exponent biased by 63, 16-bit mantissa
/// with implicit leading 1. Returned bytes are MSB-first as the device expects.
///
/// Subnormals, infinities and NaNs are *not* representable; values outside the
/// Float24 normal range are clamped to ±0 (underflow) or the largest
/// representable magnitude (overflow) by truncating the exponent. The driver's
/// supported use cases (resistor values, Steinhart–Hart coefficients) sit well
/// inside the normal range so this clamping is academic.
pub(crate) fn float24_encode(value: f32) -> [u8; 3] {
    let bits = value.to_bits();
    let sign = (bits >> 31) & 1;
    let f32_exp = (bits >> 23) & 0xFF;
    let f32_mantissa = bits & 0x7F_FFFF;

    // Zero / subnormal — encode as signed zero. Float24 has no subnormal range
    // worth caring about for our use cases.
    if f32_exp == 0 {
        return [(sign as u8) << 7, 0, 0];
    }

    // Re-bias: f32 bias 127 → Float24 bias 63 → subtract 64. Clamp to the
    // 7-bit Float24 exponent range.
    let exp_signed = f32_exp as i32 - 64;
    let (float24_exp, float24_mantissa) = if exp_signed < 1 {
        // Underflow → signed zero.
        (0u32, 0u32)
    } else if exp_signed > 0x7E {
        // Overflow → largest finite magnitude (exp=0x7E, mantissa=all-ones).
        (0x7E, 0xFFFF)
    } else {
        // Truncate mantissa from 23 to 16 bits.
        (exp_signed as u32, f32_mantissa >> 7)
    };

    let encoded = (sign << 23) | (float24_exp << 16) | float24_mantissa;
    [
        ((encoded >> 16) & 0xFF) as u8,
        ((encoded >> 8) & 0xFF) as u8,
        (encoded & 0xFF) as u8,
    ]
}

/// Variant of [`float24_encode`] that returns only the top two bytes, used for the
/// `RSxT0` reference-temperature registers. Per datasheet Table 71, those registers
/// occupy 16 bits and the device implicitly treats the missing mantissa LSB as 0.
pub(crate) fn float24_encode_high2(value: f32) -> [u8; 2] {
    let [b0, b1, _] = float24_encode(value);
    [b0, b1]
}

// ---------------------------------------------------------------------------
// Sense-resistor temperature compensation
// ---------------------------------------------------------------------------

/// Programmable temperature-drift compensation for a sense resistor (datasheet
/// "Sense Resistor Temperature Compensation").
///
/// The LTC2949 corrects the measured current/charge/energy of channel *n* with
///
/// ```text
/// R_sense(T) = R0 · [1 + tc · (T - t_ref) + tc2 · (T - t_ref)²]
/// ```
///
/// where `T` is the linearised NTC*n* reading. The compensation is enabled by
/// the presence of non-zero `tc` / `tc2` values together with [`AdcConf::ntc1`]
/// (or `ntc2`) being set so that SLOT*n* actually produces a temperature.
///
/// For copper shunts the typical first-order coefficient is `0.0039 /K`
/// (3900 ppm/K) with `tc2 = 0.0`. Low-TC alloy shunts (manganin, Zeranin, etc.)
/// can usually be left uncompensated.
#[derive(Copy, Clone, PartialEq, Debug)]
pub struct ShuntTcConfig {
    /// First-order temperature coefficient (1/K). Datasheet `RSnTC`.
    pub tc: f32,
    /// Reference temperature `T0` in °C — the temperature at which the sense
    /// resistor equals its nominal value. Datasheet `RSnT0`. Stored on-chip in
    /// the 16-bit truncated-mantissa variant of Float24.
    pub t_ref: f32,
    /// Second-order temperature coefficient (1/K²). Datasheet `RSnTC2`. Set to
    /// `0.0` to disable the quadratic term — fine for copper.
    pub tc2: f32,
}

// ---------------------------------------------------------------------------
// Auxiliary multiplexer inputs (datasheet Table 57)
// ---------------------------------------------------------------------------

/// Inputs the AUX multiplexer can route to either of the SLOT pair (`MUXP` /
/// `MUXN`) or to the fast-mode `FAMUX` registers. Variant discriminants match
/// the 5-bit encoding listed in datasheet Table 57.
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

// ---------------------------------------------------------------------------
// Error
// ---------------------------------------------------------------------------

/// Errors that can occur talking to an LTC2949.
pub enum Error<B: SpiDevice<u8>> {
    /// Underlying SPI transaction failed.
    BusError(B::Error),
    /// A returned PEC did not match the calculated value.
    ChecksumMismatch,
}

impl<B: SpiDevice<u8>> core::fmt::Debug for Error<B> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Error::BusError(_) => f.debug_struct("BusError").finish(),
            Error::ChecksumMismatch => f.debug_struct("ChecksumMismatch").finish(),
        }
    }
}

// ---------------------------------------------------------------------------
// Client
// ---------------------------------------------------------------------------

/// High-level LTC2949 operation set, the dependency-injection seam for hosts.
///
/// This mirrors [`LTC681XClient`](crate::monitor::LTC681XClient) for the cell monitors:
/// application tasks depend on `impl Ltc2949Client` (or a mock) rather than the concrete
/// [`LTC2949`] client, which keeps the bus type and topology out of the task signature.
///
/// The generic FIFO drains (`read_fifo_*`) are intentionally *not* part of this trait —
/// being generic over the sample count they don't mock cleanly; use them on the concrete
/// [`LTC2949`] when fast-continuous capture is required.
pub trait Ltc2949Client {
    type Error;

    /// Starts the recommended wake-up sequence (datasheet Figure 20): two dummy null
    /// bytes provide the isoSPI wake-up edge. Returns the wait in microseconds
    /// ([`T_BOOT_US`]) the host must observe before calling
    /// [`confirm_wake_up`](Self::confirm_wake_up), covering the case where the core was in
    /// SLEEP or had just powered up and is still booting to STANDBY. This also invalidates
    /// the cached page selection, since a device reset clears it.
    ///
    /// The driver never blocks — the host owns the wait (poll a timer, sleep, etc.).
    ///
    /// Use this on bring-up and after any intentional SLEEP. For a device that is already
    /// awake (core in STANDBY/MEASURE) whose isoSPI port has merely gone idle, the cheaper
    /// [`wake_isospi`](Self::wake_isospi) is sufficient.
    fn start_wake_up(&mut self) -> Result<u32, Self::Error>;

    /// Completes the wake-up sequence by writing WKUPACK, confirming wake-up so the device
    /// does not auto-return to SLEEP after `tACKN` (1 s). Call once the wait returned by
    /// [`start_wake_up`](Self::start_wake_up) has elapsed.
    fn confirm_wake_up(&mut self) -> Result<(), Self::Error>;

    /// Re-wakes only the isoSPI port (datasheet `tIDLE` = 6.4 ms), leaving the core state
    /// untouched. The port drops to IDLE after a few milliseconds of bus inactivity, so a
    /// periodic task must pulse it before each burst or the first frame is lost during the
    /// port start-up. One dummy byte gives the edge; the returned wait ([`T_READY_US`])
    /// must elapse before the next transaction. In a cooperative task loop one scheduler
    /// iteration typically exceeds it, so pulsing at the end of one cycle and transacting
    /// on the next needs no timer at all.
    ///
    /// This does **not** boot the core or confirm wake-up — use
    /// [`start_wake_up`](Self::start_wake_up) for that. Note the LTC68xx cell monitors on
    /// the same bus share this requirement.
    fn wake_isospi(&mut self) -> Result<u32, Self::Error>;

    /// Writes the Operation Control register (PAGE0, 0xF0).
    fn write_opctrl(&mut self, value: OpCtrl) -> Result<(), Self::Error>;

    /// Reads the Operation Control register.
    fn read_opctrl(&mut self) -> Result<OpCtrl, Self::Error>;

    /// Writes the Fast Control register (PAGE0, 0xF5).
    fn write_factrl(&mut self, value: FaCtrl) -> Result<(), Self::Error>;

    /// Writes the ADC Configuration register (PAGE1, 0xDF).
    ///
    /// Per the datasheet, changes to configuration registers other than thresholds only
    /// take effect after an ADJUPD pulse on OPCTRL while the core is in STANDBY.
    fn write_adcconf(&mut self, value: AdcConf) -> Result<(), Self::Error>;

    /// Writes the Fast AUX mux selection (FAMUXP, FAMUXN).
    fn write_fast_aux_mux(&mut self, mux_n: u8, mux_p: u8) -> Result<(), Self::Error>;

    /// Writes the Steinhart–Hart coefficients and reference resistor for an NTC channel,
    /// encoding each parameter in the device's Float24 format.
    ///
    /// To activate the linearisation after a successful write the caller still has to:
    ///
    /// 1. Configure the SLOT mux registers so the relevant `Vn` pin is presented to the
    ///    AUX ADC together with `VREF`.
    /// 2. Set [`AdcConf::ntc1`] (or `ntc2`) and apply it.
    /// 3. Pulse [`OpCtrl::adjupd`] while the core is in STANDBY so the device latches the
    ///    new configuration.
    ///
    /// `RREFn` is written as a single 3-byte burst at its dedicated address; the three
    /// coefficient registers (`NTCnA/B/C`) are contiguous on the page so they go out in
    /// one 9-byte burst.
    fn write_ntc_coefficients(&mut self, channel: Channel, params: &NtcConfig) -> Result<(), Self::Error>;

    /// Writes the sense-resistor temperature-compensation parameters for one channel.
    /// `RSnTC` (3 bytes) and `RSnT0` (2 bytes) sit contiguously on page 1 so they're sent
    /// in a single 5-byte burst; `RSnTC2` lives at a distinct address (3 bytes).
    ///
    /// As with [`write_ntc_coefficients`](Self::write_ntc_coefficients), the new values
    /// only become active after an `ADJUPD` pulse on `OpCtrl` while the core is in STANDBY.
    fn write_shunt_tc(&mut self, channel: Channel, config: &ShuntTcConfig) -> Result<(), Self::Error>;

    /// Configures the slow-mode SLOT auxiliary multiplexer (datasheet Tables 57 & 58). The
    /// `negative` and `positive` arguments select the inputs routed to `MUXN` and `MUXP`
    /// for the chosen SLOT, which the AUX ADC then measures differentially each Round-Robin
    /// cycle.
    ///
    /// Typical NTC wiring uses `(positive = Vx, negative = Agnd)` — a single-ended pin
    /// reading — together with [`AdcConf::ntc1`]/`ntc2` set so the device reports the
    /// result as a linearised temperature.
    fn write_slot_mux(&mut self, slot: Channel, negative: MuxInput, positive: MuxInput) -> Result<(), Self::Error>;

    /// Reads the STATUS register (raw byte, datasheet Table 26).
    fn read_status(&mut self) -> Result<u8, Self::Error>;

    /// Reads the FAULTS register (raw byte, datasheet 0xDD). Bits flag conditions such as
    /// UVLO, POR and self-test failures; consult the datasheet for the bit map. No typed
    /// bitfield is provided yet.
    fn read_faults(&mut self) -> Result<u8, Self::Error>;

    /// Reads the EXTFAULTS (extended faults) register (raw byte, datasheet 0xDC).
    fn read_extfaults(&mut self) -> Result<u8, Self::Error>;

    /// Requests the memory lock (datasheet Figure 19): writes `MLK = 0b01` to REGSCTRL.
    /// Returns the wait in microseconds ([`T_MLCK_US`], the worst-case `tMLCK,M`) the host
    /// must observe before the register map is guaranteed frozen. While locked, reads of
    /// any one page return a coherent snapshot even across registers; internal
    /// accumulation continues unaffected. Release with
    /// [`unlock_memory`](Self::unlock_memory) — the lock does not expire on its own.
    ///
    /// **Not normally required:** a single multi-byte burst within one 16-byte row
    /// (every accumulator here — e.g. `Charge1`, `Energy1`, `Time1`) is already coherent
    /// per the datasheet, so individual reads do not need locking. Use this only to snapshot
    /// *several* registers at the same instant. Stay on the currently selected page while
    /// locked — switching pages rewrites REGSCTRL and releases the lock.
    fn request_memory_lock(&mut self) -> Result<u32, Self::Error>;

    /// Releases the memory lock (`MLK = 0b00`), letting the register map update again.
    fn unlock_memory(&mut self) -> Result<(), Self::Error>;

    /// Reads I1 (slow-mode current 1) as a 24-bit two's-complement value.
    /// LSB = 950 nV for slow mode, 237.5 nV for the averaged result.
    fn read_current1(&mut self) -> Result<i32, Self::Error>;

    /// Reads I2 (slow-mode current 2). LSB = 950 nV.
    fn read_current2(&mut self) -> Result<i32, Self::Error>;

    /// Reads P1 (power 1 or voltage if P1ASV is set). LSB = 5.8368 µV²/Ω (power) or
    /// 46.875 µV (voltage).
    fn read_power1(&mut self) -> Result<i32, Self::Error>;

    /// Reads P2 (power 2 or voltage if P2ASV is set).
    fn read_power2(&mut self) -> Result<i32, Self::Error>;

    /// Reads BAT (battery voltage). LSB = 375 µV.
    fn read_bat(&mut self) -> Result<i16, Self::Error>;

    /// Reads internal die temperature. LSB = 0.2 °C, full-scale 819.2 K.
    fn read_temp(&mut self) -> Result<i16, Self::Error>;

    /// Reads A/DVCC supply voltage. LSB = 2.26 mV.
    fn read_vcc(&mut self) -> Result<i16, Self::Error>;

    /// Reads SLOT1 — voltage (375 µV/LSB) or temperature (0.2 °C/LSB) depending on NTC1
    /// in ADCCONF.
    fn read_slot1(&mut self) -> Result<i16, Self::Error>;

    /// Reads SLOT2 — voltage (375 µV/LSB) or temperature (0.2 °C/LSB) depending on NTC2
    /// in ADCCONF.
    fn read_slot2(&mut self) -> Result<i16, Self::Error>;

    /// Reads Charge1 (48-bit two's-complement, units LSB·377.887 ps·V).
    fn read_charge1(&mut self) -> Result<i64, Self::Error>;

    /// Reads Charge2 (48-bit).
    fn read_charge2(&mut self) -> Result<i64, Self::Error>;

    /// Reads Charge3 — weighted sum of channel 1 and channel 2 (64-bit).
    fn read_charge3(&mut self) -> Result<i64, Self::Error>;

    /// Reads Energy1 (48-bit).
    fn read_energy1(&mut self) -> Result<i64, Self::Error>;

    /// Reads Energy2 (48-bit).
    fn read_energy2(&mut self) -> Result<i64, Self::Error>;

    /// Reads Energy4 — weighted sum of channel 1 and channel 2 (64-bit).
    fn read_energy4(&mut self) -> Result<i64, Self::Error>;

    /// Reads time-base 1 (32-bit unsigned).
    fn read_time1(&mut self) -> Result<u32, Self::Error>;

    /// Reads time-base 2.
    fn read_time2(&mut self) -> Result<u32, Self::Error>;

    /// Reads time-base 3.
    fn read_time3(&mut self) -> Result<u32, Self::Error>;

    /// Reads time-base 4.
    fn read_time4(&mut self) -> Result<u32, Self::Error>;

    /// Reads channel 1's charge, energy and time-base ([`Accumulators`]) in a single
    /// 16-byte burst (row `0x00–0x0F`). The three values form a coherent snapshot from the
    /// same `CONT` cycle without needing the memory lock — prefer this over separate
    /// [`read_charge1`](Self::read_charge1) / [`read_time1`](Self::read_time1) calls when
    /// charge and elapsed time must line up (e.g. state-of-charge integration).
    fn read_accumulators1(&mut self) -> Result<Accumulators, Self::Error>;

    /// Reads channel 2's charge, energy and time-base in a single coherent 16-byte burst
    /// (row `0x10–0x1F`). See [`read_accumulators1`](Self::read_accumulators1).
    fn read_accumulators2(&mut self) -> Result<Accumulators, Self::Error>;

    /// Sends a broadcast ADCV (0x0260) which triggers a fast single-shot conversion on
    /// every device that recognises it — both the LTC2949 (per its FACTRL configuration)
    /// and any LTC68xx cell monitors on the same isoSPI bus. Use this when synchronous
    /// measurements with the cell monitors are required.
    ///
    /// **Cross-task hazard:** because this is a *broadcast*, it also (re)starts conversions
    /// on the cell-monitor chain. If another task owns the cell-monitor conversion schedule
    /// (as in a BMS where a separate task drives the LTC68xx ADC), calling this from the
    /// meter task will restart an in-flight chain conversion and corrupt its timing. In that
    /// design, run the LTC2949 in slow continuous mode (`OPCTRL.CONT`) and read results
    /// without this broadcast, or route the single shared ADCV through the conversion task's
    /// schedule.
    fn trigger_adcv_broadcast(&mut self) -> Result<(), Self::Error>;
}

/// LTC2949 client for the addressable, parallel-to-daisy-chain topology
/// (datasheet Figure 12(B)). The device is reached directly via `DCMD`, so no
/// chain-length parameter is required.
///
/// The high-level operations live on the [`Ltc2949Client`] trait (bring it into scope to
/// call `read_current1()`, `start_wake_up()`, etc.); generic FIFO drains are inherent.
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
    /// Constructs an LTC2949 client. The LTC2949 is addressed directly via `DCMD`,
    /// whether it sits alone on the bus or in parallel with a cell-monitor chain.
    pub fn new(bus: B) -> Self {
        Self {
            bus,
            poll_method: NoPolling {},
            current_page: None,
        }
    }
}

impl<B, P> Ltc2949Client for LTC2949<B, P>
where
    B: SpiDevice<u8>,
    P: PollMethod<B>,
{
    type Error = Error<B>;

    // -- High-level helpers ------------------------------------------------

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
        self.write_bytes(Page0Reg::WkupAck, &[0x00])
    }

    fn wake_isospi(&mut self) -> Result<u32, Error<B>> {
        // One dummy byte provides the differential edge; the port is ready after tREADY.
        // The core state is untouched. The host owns the wait.
        self.bus.write(&[0x00]).map_err(Error::BusError)?;
        Ok(T_READY_US)
    }

    fn write_opctrl(&mut self, value: OpCtrl) -> Result<(), Error<B>> {
        self.write_bytes(Page0Reg::OpCtrl, &value.into_bytes())
    }

    fn read_opctrl(&mut self) -> Result<OpCtrl, Error<B>> {
        let mut buf = [0u8; 1];
        self.read_bytes(Page0Reg::OpCtrl, &mut buf)?;
        Ok(OpCtrl::from_bytes(buf))
    }

    fn write_factrl(&mut self, value: FaCtrl) -> Result<(), Error<B>> {
        self.write_bytes(Page0Reg::FaCtrl, &value.into_bytes())
    }

    fn write_adcconf(&mut self, value: AdcConf) -> Result<(), Error<B>> {
        self.write_bytes(Page1Reg::AdcConf, &value.into_bytes())
    }

    fn write_fast_aux_mux(&mut self, mux_n: u8, mux_p: u8) -> Result<(), Error<B>> {
        self.write_bytes(Page0Reg::FaMuxN, &[mux_n, mux_p])
    }

    fn write_ntc_coefficients(
        &mut self,
        channel: Channel,
        params: &NtcConfig,
    ) -> Result<(), Error<B>> {
        let (rref_addr, abc_addr) = match channel {
            Channel::One => (Page1Reg::Rref1, Page1Reg::Ntc1A),
            Channel::Two => (Page1Reg::Rref2, Page1Reg::Ntc2A),
        };

        let rref = float24_encode(params.r_ref);
        self.write_bytes(rref_addr, &rref)?;

        let mut abc = [0u8; 9];
        abc[0..3].copy_from_slice(&float24_encode(params.a));
        abc[3..6].copy_from_slice(&float24_encode(params.b));
        abc[6..9].copy_from_slice(&float24_encode(params.c));
        self.write_bytes(abc_addr, &abc)?;

        Ok(())
    }

    fn write_shunt_tc(
        &mut self,
        channel: Channel,
        config: &ShuntTcConfig,
    ) -> Result<(), Error<B>> {
        let (tc_addr, tc2_addr) = match channel {
            Channel::One => (Page1Reg::Rs1Tc, Page1Reg::Rs1Tc2),
            Channel::Two => (Page1Reg::Rs2Tc, Page1Reg::Rs2Tc2),
        };

        // RSnTC (3 bytes Float24) + RSnT0 (2 bytes truncated Float24).
        let mut tc_burst = [0u8; 5];
        tc_burst[0..3].copy_from_slice(&float24_encode(config.tc));
        tc_burst[3..5].copy_from_slice(&float24_encode_high2(config.t_ref));
        self.write_bytes(tc_addr, &tc_burst)?;

        // RSnTC2 lives elsewhere on the page.
        let tc2 = float24_encode(config.tc2);
        self.write_bytes(tc2_addr, &tc2)?;

        Ok(())
    }

    fn write_slot_mux(
        &mut self,
        slot: Channel,
        negative: MuxInput,
        positive: MuxInput,
    ) -> Result<(), Error<B>> {
        let addr_n = match slot {
            Channel::One => Page0Reg::Slot1MuxN,
            Channel::Two => Page0Reg::Slot2MuxN,
        };
        // MUXN and MUXP are adjacent (0xEB/0xEC for SLOT1, 0xED/0xEE for SLOT2)
        // so a single 2-byte burst configures both.
        self.write_bytes(addr_n, &[negative as u8, positive as u8])
    }

    fn read_status(&mut self) -> Result<u8, Error<B>> {
        let mut buf = [0u8; 1];
        self.read_bytes(Page0Reg::Status, &mut buf)?;
        Ok(buf[0])
    }

    fn read_faults(&mut self) -> Result<u8, Error<B>> {
        let mut buf = [0u8; 1];
        self.read_bytes(Page0Reg::Faults, &mut buf)?;
        Ok(buf[0])
    }

    fn read_extfaults(&mut self) -> Result<u8, Error<B>> {
        let mut buf = [0u8; 1];
        self.read_bytes(Page0Reg::ExtFaults, &mut buf)?;
        Ok(buf[0])
    }

    // -- Memory lock (coherent multi-register snapshots) -------------------

    fn request_memory_lock(&mut self) -> Result<u32, Error<B>> {
        // REGSCTRL is page-independent; preserve the current page bit so the lock request
        // does not also switch pages. MLK = 0b01.
        let page = self.current_page.unwrap_or(Page::Page0);
        let value = self.regsctrl_base().with_mlk(0b01);
        self.dcmd_write(Page0Reg::RegsCtrl.addr(), &value.into_bytes())?;
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
        self.dcmd_write(Page0Reg::RegsCtrl.addr(), &value.into_bytes())
    }

    // -- Measurement readouts ---------------------------------------------
    //
    // The non-accumulated results live in PAGE0. They are little-endian on the
    // bus when read via direct memory access (which always returns MSB first
    // per datasheet — "reading data from LTC2949's memory map reports MSBytes
    // first, while reading fast conversion results via RDCV reports LSBytes
    // first"). We therefore decode big-endian here.

    fn read_current1(&mut self) -> Result<i32, Error<B>> {
        self.read_signed_24(Page0Reg::Current1)
    }

    fn read_current2(&mut self) -> Result<i32, Error<B>> {
        self.read_signed_24(Page0Reg::Current2)
    }

    fn read_power1(&mut self) -> Result<i32, Error<B>> {
        self.read_signed_24(Page0Reg::Power1)
    }

    fn read_power2(&mut self) -> Result<i32, Error<B>> {
        self.read_signed_24(Page0Reg::Power2)
    }

    fn read_bat(&mut self) -> Result<i16, Error<B>> {
        self.read_signed_16(Page0Reg::Bat)
    }

    fn read_temp(&mut self) -> Result<i16, Error<B>> {
        self.read_signed_16(Page0Reg::Temp)
    }

    fn read_vcc(&mut self) -> Result<i16, Error<B>> {
        self.read_signed_16(Page0Reg::Vcc)
    }

    fn read_slot1(&mut self) -> Result<i16, Error<B>> {
        self.read_signed_16(Page0Reg::Slot1)
    }

    fn read_slot2(&mut self) -> Result<i16, Error<B>> {
        self.read_signed_16(Page0Reg::Slot2)
    }

    // -- Accumulators ------------------------------------------------------

    fn read_charge1(&mut self) -> Result<i64, Error<B>> {
        self.read_signed_48(Page0Reg::Charge1)
    }

    fn read_charge2(&mut self) -> Result<i64, Error<B>> {
        self.read_signed_48(Page0Reg::Charge2)
    }

    fn read_charge3(&mut self) -> Result<i64, Error<B>> {
        self.read_signed_64(Page0Reg::Charge3)
    }

    fn read_energy1(&mut self) -> Result<i64, Error<B>> {
        self.read_signed_48(Page0Reg::Energy1)
    }

    fn read_energy2(&mut self) -> Result<i64, Error<B>> {
        self.read_signed_48(Page0Reg::Energy2)
    }

    fn read_energy4(&mut self) -> Result<i64, Error<B>> {
        self.read_signed_64(Page0Reg::Energy4)
    }

    fn read_time1(&mut self) -> Result<u32, Error<B>> {
        self.read_unsigned_32(Page0Reg::Time1)
    }

    fn read_time2(&mut self) -> Result<u32, Error<B>> {
        self.read_unsigned_32(Page0Reg::Time2)
    }

    fn read_time3(&mut self) -> Result<u32, Error<B>> {
        self.read_unsigned_32(Page0Reg::Time3)
    }

    fn read_time4(&mut self) -> Result<u32, Error<B>> {
        self.read_unsigned_32(Page0Reg::Time4)
    }

    fn read_accumulators1(&mut self) -> Result<Accumulators, Error<B>> {
        self.read_accumulator_row(Page0Reg::Charge1)
    }

    fn read_accumulators2(&mut self) -> Result<Accumulators, Error<B>> {
        self.read_accumulator_row(Page0Reg::Charge2)
    }

    // -- Fast mode ---------------------------------------------------------

    fn trigger_adcv_broadcast(&mut self) -> Result<(), Error<B>> {
        // ADCV broadcast: CMD0=0b00000_010, CMD1=0b01100000 = 0x0260 (datasheet Table 17).
        // For LTC2949 the exact bitmap variant (Normal mode, all cells) is don't-care; it
        // simply triggers the fast channels selected by FACTRL.
        self.send_cmd16(0x0260)
    }
}

/// Inherent helpers: constructor lives above; FIFO drains are generic over the sample
/// count `N` (so they stay off the object-safe-ish [`Ltc2949Client`] trait), and the
/// remaining methods are private framing/decoding internals.
impl<B, P> LTC2949<B, P>
where
    B: SpiDevice<u8>,
    P: PollMethod<B>,
{
    /// Drains up to `N` samples from the I1 FIFO. Reads stop at the first non-`Ok`
    /// sample, which **is** included as the final element so the caller can inspect its
    /// [`tag`](FifoSample::tag) — [`ReadOverrun`](FifoTag::ReadOverrun) means the FIFO held
    /// no new data (its `raw` is stale), [`WriteOverrun`](FifoTag::WriteOverrun) means the
    /// FIFO overflowed (its `raw` is valid). All preceding elements are valid `Ok` samples.
    ///
    /// Each FIFO sample is three bytes: MSB, LSB, TAG (datasheet Table 29).
    pub fn read_fifo_i1<const N: usize>(&mut self) -> Result<Vec<FifoSample, N>, Error<B>> {
        self.read_fifo::<N>(Page0Reg::FifoI1)
    }

    /// Drains up to `N` samples from the I2 FIFO.
    pub fn read_fifo_i2<const N: usize>(&mut self) -> Result<Vec<FifoSample, N>, Error<B>> {
        self.read_fifo::<N>(Page0Reg::FifoI2)
    }

    /// Drains up to `N` samples from the BAT (P1/P2 voltage-mode) FIFO.
    pub fn read_fifo_bat<const N: usize>(&mut self) -> Result<Vec<FifoSample, N>, Error<B>> {
        self.read_fifo::<N>(Page0Reg::FifoBat)
    }

    /// Drains up to `N` samples from the AUX FIFO.
    pub fn read_fifo_aux<const N: usize>(&mut self) -> Result<Vec<FifoSample, N>, Error<B>> {
        self.read_fifo::<N>(Page0Reg::FifoAux)
    }

    fn read_fifo<const N: usize>(&mut self, reg: Page0Reg) -> Result<Vec<FifoSample, N>, Error<B>> {
        // Read 3·N bytes from the (non-incrementing) FIFO register.
        // Stack-allocated buffer up to a safe maximum; bail if N is unreasonable.
        let mut samples: Vec<FifoSample, N> = Vec::new();
        let mut three = [0u8; 3];
        for _ in 0..N {
            self.read_bytes(reg, &mut three)?;
            let raw = ((three[0] as u16) << 8 | three[1] as u16) as i16;
            let tag = FifoTag::from_byte(three[2]);
            let stop = !matches!(tag, FifoTag::Ok);
            let _ = samples.push(FifoSample { raw, tag });
            if stop {
                break;
            }
        }
        Ok(samples)
    }

    // -- Decode helpers ---------------------------------------------------

    fn read_signed_16(&mut self, reg: Page0Reg) -> Result<i16, Error<B>> {
        let mut buf = [0u8; 2];
        self.read_bytes(reg, &mut buf)?;
        Ok(i16::from_be_bytes(buf))
    }

    fn read_signed_24(&mut self, reg: Page0Reg) -> Result<i32, Error<B>> {
        let mut buf = [0u8; 3];
        self.read_bytes(reg, &mut buf)?;
        // Sign-extend 24 -> 32.
        let raw = (u32::from(buf[0]) << 16) | (u32::from(buf[1]) << 8) | u32::from(buf[2]);
        let extended = if raw & 0x0080_0000 != 0 { raw | 0xFF00_0000 } else { raw };
        Ok(extended as i32)
    }

    fn read_unsigned_32(&mut self, reg: Page0Reg) -> Result<u32, Error<B>> {
        let mut buf = [0u8; 4];
        self.read_bytes(reg, &mut buf)?;
        Ok(u32::from_be_bytes(buf))
    }

    fn read_signed_48(&mut self, reg: Page0Reg) -> Result<i64, Error<B>> {
        let mut buf = [0u8; 6];
        self.read_bytes(reg, &mut buf)?;
        Ok(sign_extend_48(&buf))
    }

    /// Reads a full 16-byte accumulator row (charge, energy, time) in one coherent burst.
    /// `reg` is the row's charge address (`0x00` for channel 1, `0x10` for channel 2); the
    /// device auto-increments through energy (`+0x06`) and time (`+0x0C`).
    fn read_accumulator_row(&mut self, reg: Page0Reg) -> Result<Accumulators, Error<B>> {
        let mut buf = [0u8; 16];
        self.read_bytes(reg, &mut buf)?;
        Ok(Accumulators {
            charge: sign_extend_48(&buf[0..6]),
            energy: sign_extend_48(&buf[6..12]),
            time: u32::from_be_bytes([buf[12], buf[13], buf[14], buf[15]]),
        })
    }

    fn read_signed_64(&mut self, reg: Page0Reg) -> Result<i64, Error<B>> {
        let mut buf = [0u8; 8];
        self.read_bytes(reg, &mut buf)?;
        Ok(i64::from_be_bytes(buf))
    }

    // -- Page handling ----------------------------------------------------

    /// REGSCTRL value reflecting the currently cached page with RDCVCONF=1, BCREN=0,
    /// MLK=00. RDCVCONF stays 1 so an addressed RDCV reports fast-mode conversion results;
    /// BCREN stays 0 (parallel topology) so the LTC2949 never responds to broadcast RDCV
    /// and cannot collide with the cell monitors on the shared bus. Used by the memory-lock
    /// helpers, which must rewrite REGSCTRL without changing the selected page.
    fn regsctrl_base(&self) -> RegsCtrl {
        let page1 = matches!(self.current_page.unwrap_or(Page::Page0), Page::Page1);
        RegsCtrl::new().with_rdcvconf(true).with_page(page1)
    }

    fn select_page(&mut self, page: Page) -> Result<(), Error<B>> {
        if self.current_page == Some(page) {
            return Ok(());
        }
        let value = RegsCtrl::new()
            .with_rdcvconf(true)
            .with_page(matches!(page, Page::Page1));
        self.dcmd_write(Page0Reg::RegsCtrl.addr(), &value.into_bytes())?;
        self.current_page = Some(page);
        Ok(())
    }

    // -- Read primitive ---------------------------------------------------

    /// Reads `buf.len()` bytes starting at `reg` from the LTC2949 via a direct `DCMD`
    /// read — the LTC2949 is addressed in parallel to the cell-monitor chain, so its
    /// response returns with no shift-register prefix. The register's page is selected
    /// first if not already current.
    fn read_bytes<R: Register>(&mut self, reg: R, buf: &mut [u8]) -> Result<(), Error<B>> {
        self.select_page(R::PAGE)?;
        self.dcmd_read(reg.addr(), buf)
    }

    /// Writes `data` to `reg` via `DCMD`. The cell monitors ignore command 0xFE.
    fn write_bytes<R: Register>(&mut self, reg: R, data: &[u8]) -> Result<(), Error<B>> {
        // REGSCTRL writes are themselves the page-switch mechanism; avoid recursion.
        if reg.addr() != Page0Reg::RegsCtrl.addr() {
            self.select_page(R::PAGE)?;
        }
        self.dcmd_write(reg.addr(), data)
    }

    // -- DCMD framing -----------------------------------------------------
    //
    // DCMD frame layout (datasheet Table 11):
    //
    //   [0xFE, RADDR, PEC0, PEC1, ID, D0..D(N-1), PEC0, PEC1, D(N)..D(2N-1), PEC0, PEC1, ...]
    //
    // The PEC on bytes 2..4 is computed over [0xFE, RADDR]. Each subsequent PEC covers the
    // preceding `N` data bytes. `N` is encoded in the ID byte's PECC field
    // (`PECC = N-1`, range 0..=15). The ID byte itself carries redundancy so it is not
    // covered by a PEC.

    /// We use a fixed PECC of 15 (16 data bytes per PEC) for maximum throughput on
    /// long bursts. Short reads/writes still work — fewer than 16 bytes simply emit a
    /// single PEC at the end.
    const PECC: u8 = 15;
    const N_PER_PEC: usize = 16;

    /// Constructs the ID byte for a DCMD (datasheet Table 12).
    fn make_id(read: bool) -> u8 {
        let pecc = Self::PECC & 0x0F;
        let p3 = (pecc >> 3) & 1;
        let p2 = (pecc >> 2) & 1;
        let p1 = (pecc >> 1) & 1;
        let p0 = pecc & 1;
        let rw = u8::from(read);
        let not_rw = rw ^ 1;
        let bit5 = p3 ^ p2;
        let bit2 = p1 ^ p0;
        (rw << 7) | (not_rw << 6) | (bit5 << 5) | (p3 << 4) | (p2 << 3) | (bit2 << 2) | (p1 << 1) | p0
    }

    /// Sends a DCMD write transaction.
    fn dcmd_write(&mut self, addr: u8, data: &[u8]) -> Result<(), Error<B>> {
        // Compose the entire transaction in a stack buffer. The maximum frame size we
        // support in one call is 4 (header+PEC) + 1 (ID) + 16 (data) + 2 (PEC) = 23
        // bytes. If a caller hands us more than 16 data bytes we fall back to chunking
        // (only the FIFO drain and accumulator paths can plausibly need this, and they
        // already chunk via repeated short reads).
        if data.len() > Self::N_PER_PEC {
            // Chunk recursively. Each chunk gets its own DCMD frame.
            for (i, chunk) in data.chunks(Self::N_PER_PEC).enumerate() {
                self.dcmd_write(addr.wrapping_add((i * Self::N_PER_PEC) as u8), chunk)?;
            }
            return Ok(());
        }

        let mut frame = [0u8; 23];
        frame[0] = 0xFE;
        frame[1] = addr;
        let header_pec = PEC15::calc(&frame[0..2]);
        frame[2] = header_pec[0];
        frame[3] = header_pec[1];
        frame[4] = Self::make_id(false);

        let n = data.len();
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

    /// Reads `data.len()` bytes via a direct `DCMD` read. The slave's data appears on
    /// MISO immediately after the master finishes clocking out the 5-byte command
    /// header (`[0xFE, RADDR, PEC0, PEC1, ID]`), followed by the data PEC:
    ///
    /// `MISO: [.., .., .., .., .., D0..D(n-1), PEC0, PEC1]`
    ///
    /// Reads longer than one PEC group (16 bytes) are split into separate `DCMD`
    /// transactions; the device auto-increments its address pointer so each chunk
    /// re-issues the command at the advanced address.
    fn dcmd_read(&mut self, addr: u8, buf: &mut [u8]) -> Result<(), Error<B>> {
        if buf.len() > Self::N_PER_PEC {
            for (i, chunk) in buf.chunks_mut(Self::N_PER_PEC).enumerate() {
                self.dcmd_read(addr.wrapping_add((i * Self::N_PER_PEC) as u8), chunk)?;
            }
            return Ok(());
        }

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
        mosi[4] = Self::make_id(true);
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

    // -- ADCV broadcast ---------------------------------------------------

    fn send_cmd16(&mut self, cmd: u16) -> Result<(), Error<B>> {
        let frame = build_cmd16(cmd);
        self.bus.write(&frame).map_err(Error::BusError)?;
        self.poll_method.end_sync_command(&mut self.bus).map_err(Error::BusError)?;
        Ok(())
    }
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

