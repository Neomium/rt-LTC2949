//! # LTC2949 direct-register client.
//!
//! This module contains the high-level [`Client`] trait, the concrete [`LTC2949`] client,
//! register-oriented configuration types, raw result readers, and helper constants for
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
// `modular-bitfield`'s macro expansion emits `pub field: (bool)` etc., which trips the
// `unused_parens` lint on newer rustc. Silence the lint module-wide.
#![allow(unused_parens)]

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

/// Fixed PECC field (datasheet Table 12): 16 data bytes per PEC group.
const PECC: u8 = 15;

/// Max data bytes covered by one PEC group (`PECC + 1`).
const N_PER_PEC: usize = 16;

/// Whole 3-byte FIFO samples that fit one PEC group (`16 / 3 = 5`). Reading several samples
/// per DCMD amortises the ~9-byte header/PEC overhead instead of paying it per sample.
const FIFO_SAMPLES_PER_BURST: usize = N_PER_PEC / 3;

/// Memory page selector. PAGE0 holds measurement results, status and control;
/// PAGE1 holds thresholds and configuration. Selected via the [`RegsCtrl`] register.
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub enum Page {
    Page0 = 0,
    Page1 = 1,
}

/// Page-0 register addresses: results, accumulators, status and control/fast-mode
/// registers (datasheet Tables 24, 26-28, 57-64). Discriminant = on-bus `RADDR` byte.
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

/// Page-1 register addresses: ADC config plus the NTC-linearisation and sense-resistor
/// TC coefficient blocks (datasheet Tables 69, 71, 76). Discriminant = on-bus `RADDR` byte.
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

/// A device register resolving to a memory [`Page`] and an on-bus address byte, so the
/// framing helpers take a register and derive the page rather than threading both.
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

/// Operation Control register (PAGE0, 0xF0) — datasheet Table 24. `clr`, `sshot`, `adjupd`
/// and `rst` are set-only; the device clears them once done (poll to observe completion).
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

/// ADC Configuration register (PAGE1, 0xDF) — datasheet Table 69. `p1asv`/`p2asv` set
/// power ADCs to voltage mode; `ntc1`/`ntc2` linearise the SLOT; `ntcslot1` ties CH2 TC to NTC1.
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

/// Register Control register (common to both pages, 0xFF) — datasheet Table 23. `mlk` is
/// the 2-bit memory-lock handshake (`0b01` request, `0b10` device-acknowledged).
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

impl Default for OpCtrl {
    fn default() -> Self {
        Self::new()
    }
}

impl Default for FaCtrl {
    fn default() -> Self {
        Self::new()
    }
}

impl Default for AdcConf {
    fn default() -> Self {
        Self::new()
    }
}

impl Default for RegsCtrl {
    fn default() -> Self {
        Self::new()
    }
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
    /// Charge (48-bit two's-complement; units LSB·377.887 ps·V).
    pub charge: i64,
    /// Energy (48-bit two's-complement).
    pub energy: i64,
    /// Time base (32-bit unsigned).
    pub time: u32,
}

///Overcurrent Control Registers Table 50 & 51
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub struct OverCurrentConfig {
    pub enable: bool,
    pub threshold: u8,
    pub deglitch_time: u8,
    pub polarity: u8,
}

// ---------------------------------------------------------------------------
// NTC linearisation
// ---------------------------------------------------------------------------

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

/// Encodes an `f32` as MSB-first Float24 (datasheet Table 68): 1 sign, 7-bit exponent
/// biased 63, 16-bit mantissa. Out-of-range values clamp to ±0 or the largest magnitude.
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

/// Like [`float24_encode`] but returns only the top two bytes, for the 16-bit `RSxT0`
/// registers (datasheet Table 71; the device treats the missing mantissa LSB as 0).
pub(crate) fn float24_encode_high2(value: f32) -> [u8; 2] {
    let [b0, b1, _] = float24_encode(value);
    [b0, b1]
}

// ---------------------------------------------------------------------------
// Sense-resistor temperature compensation
// ---------------------------------------------------------------------------

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

// ---------------------------------------------------------------------------
// Auxiliary multiplexer inputs (datasheet Table 57)
// ---------------------------------------------------------------------------

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
    fn write_opctrl(&mut self, value: OpCtrl) -> Result<(), Self::Error>;

    /// Reads the Operation Control register.
    fn read_opctrl(&mut self) -> Result<OpCtrl, Self::Error>;

    /// Writes the Fast Control register (PAGE0, 0xF5).
    fn write_factrl(&mut self, value: FaCtrl) -> Result<(), Self::Error>;

    /// Writes the ADC Configuration register (PAGE1, 0xDF). Takes effect only after an
    /// ADJUPD pulse on OPCTRL while the core is in STANDBY.
    fn write_adcconf(&mut self, value: AdcConf) -> Result<(), Self::Error>;

    /// Writes the Fast AUX mux selection (FAMUXP, FAMUXN).
    fn write_fast_aux_mux(&mut self, mux_n: u8, mux_p: u8) -> Result<(), Self::Error>;

    /// Writes the Float24 Steinhart–Hart coefficients and reference resistor for an NTC
    /// channel. Activate via [`write_slot_mux`](Self::write_slot_mux), [`AdcConf::ntc1`] and an ADJUPD pulse.
    fn write_ntc_coefficients(&mut self, channel: Channel, params: &NtcConfig) -> Result<(), Self::Error>;

    /// Writes the sense-resistor temperature-compensation parameters for one channel. Active
    /// only after an `ADJUPD` pulse on `OpCtrl` while the core is in STANDBY.
    fn write_shunt_tc(&mut self, channel: Channel, config: &ShuntTcConfig) -> Result<(), Self::Error>;

    /// Routes `negative`/`positive` to the SLOT's `MUXN`/`MUXP` for differential AUX-ADC
    /// reads (datasheet Tables 57 & 58). NTCs typically use `(Vx, Agnd)`.
    fn write_slot_mux(&mut self, slot: Channel, negative: MuxInput, positive: MuxInput) -> Result<(), Self::Error>;

    /// Writes GPIO Control
    fn write_gpio_ctrl(&mut self, gpio: u8) -> Result<(), Self::Error>;

    fn write_occ_config(&mut self, config1: OverCurrentConfig, config2: OverCurrentConfig) -> Result<(), Self::Error>;

    /// Reads the STATUS register (raw byte, datasheet Table 26).
    fn read_status(&mut self) -> Result<u8, Self::Error>;

    /// Reads the FAULTS register (raw byte, datasheet 0xDD; UVLO/POR/self-test bits). No
    /// typed bitfield yet — consult the datasheet for the bit map.
    fn read_faults(&mut self) -> Result<u8, Self::Error>;

    /// Reads the EXTFAULTS (extended faults) register (raw byte, datasheet 0xDC).
    fn read_extfaults(&mut self) -> Result<u8, Self::Error>;

    /// Requests the memory lock (datasheet Figure 19, `MLK = 0b01`) for a cross-register
    /// snapshot; returns the wait ([`T_MLCK_US`]). Stay on one page, then [`unlock_memory`](Self::unlock_memory).
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

    /// Reads channel 1's charge, energy and time-base ([`Accumulators`]) in one coherent
    /// 16-byte burst — prefer this for SoC integration over separate charge/time reads.
    fn read_accumulators1(&mut self) -> Result<Accumulators, Self::Error>;

    /// Reads channel 2's charge, energy and time-base in a single coherent 16-byte burst
    /// (row `0x10–0x1F`). See [`read_accumulators1`](Self::read_accumulators1).
    fn read_accumulators2(&mut self) -> Result<Accumulators, Self::Error>;

    /// Broadcast ADCV (0x0260): synchronous fast conversion on the LTC2949 and every cell
    /// monitor. **Hazard:** also restarts the chain, so don't call it from a separate meter task.
    fn trigger_adcv_broadcast(&mut self) -> Result<(), Self::Error>;

    /// Send ADAX adressed
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

    fn write_ntc_coefficients(&mut self, channel: Channel, params: &NtcConfig) -> Result<(), Error<B>> {
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

    fn write_shunt_tc(&mut self, channel: Channel, config: &ShuntTcConfig) -> Result<(), Error<B>> {
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

    fn write_slot_mux(&mut self, slot: Channel, negative: MuxInput, positive: MuxInput) -> Result<(), Error<B>> {
        let addr_n = match slot {
            Channel::One => Page0Reg::Slot1MuxN,
            Channel::Two => Page0Reg::Slot2MuxN,
        };
        // MUXN and MUXP are adjacent (0xEB/0xEC for SLOT1, 0xED/0xEE for SLOT2)
        // so a single 2-byte burst configures both.
        self.write_bytes(addr_n, &[negative as u8, positive as u8])
    }

    fn write_gpio_ctrl(&mut self, gpio: u8) -> Result<(), Self::Error> {
        self.write_bytes(Page0Reg::FGpioCtrl, &[gpio])
    }

    fn write_occ_config(&mut self, config1: OverCurrentConfig, config2: OverCurrentConfig) -> Result<(), Self::Error> {
        let b =
            (config1.enable as u8) | (config1.threshold << 1) | (config1.deglitch_time << 4) | (config1.polarity << 6);
        self.write_bytes(Page0Reg::Occ1Ctrl, &[b])?;

        let b2 =
            (config2.enable as u8) | (config2.threshold << 1) | (config2.deglitch_time << 4) | (config2.polarity << 6);
        self.write_bytes(Page0Reg::Occ2Ctrl, &[b2])
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
    /// Drains up to `N` I1 FIFO samples (3 bytes each: MSB, LSB, TAG). Stops at the first
    /// non-`Ok` sample, which is included as the terminator so the caller can read its `tag`.
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

    /// Reads a full 16-byte accumulator row (charge, energy, time) in one coherent burst,
    /// starting at the row's charge address (`0x00` for channel 1, `0x10` for channel 2).
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

    /// REGSCTRL for the current page with RDCVCONF=1, BCREN=0, MLK=00. Lets the memory-lock
    /// helpers rewrite REGSCTRL without changing the selected page.
    fn regsctrl_base(&self) -> RegsCtrl {
        let page1 = matches!(self.current_page.unwrap_or(Page::Page0), Page::Page1);
        RegsCtrl::default().with_rdcvconf(true).with_page(page1)
    }

    fn select_page(&mut self, page: Page) -> Result<(), Error<B>> {
        if self.current_page == Some(page) {
            return Ok(());
        }
        let value = RegsCtrl::default().with_rdcvconf(true).with_page(matches!(page, Page::Page1));
        self.dcmd_write(Page0Reg::RegsCtrl.addr(), &value.into_bytes())?;
        self.current_page = Some(page);
        Ok(())
    }

    // -- Read primitive ---------------------------------------------------

    /// Reads `buf.len()` bytes from `reg` via a direct `DCMD` read (no shift-register
    /// prefix in the parallel topology), selecting the page first if needed.
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

    /// Constructs the ID byte for a DCMD (datasheet Table 12).
    fn make_id(read: bool) -> u8 {
        let pecc = PECC & 0x0F;
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

    /// Sends a DCMD write transaction. `data` must fit one PEC group (≤ 16 bytes); every
    /// caller already stays within that (the 9-byte NTC burst is the largest).
    fn dcmd_write(&mut self, addr: u8, data: &[u8]) -> Result<(), Error<B>> {
        // Frame: 4 (header+PEC) + 1 (ID) + ≤16 (data) + 2 (PEC) = 23 bytes max.
        debug_assert!(data.len() <= N_PER_PEC, "DCMD write exceeds one PEC group");

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

    /// Direct `DCMD` read: data appears on MISO after the 5-byte header, then the PEC.
    /// `buf` must fit one PEC group (≤ 16 bytes); the 16-byte accumulator row is the largest.
    fn dcmd_read(&mut self, addr: u8, buf: &mut [u8]) -> Result<(), Error<B>> {
        debug_assert!(buf.len() <= N_PER_PEC, "DCMD read exceeds one PEC group");

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
