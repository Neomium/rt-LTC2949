//! # LTC2949 Float24 encoding
//!
//! The LTC2949 stores NTC and sense-resistor temperature-compensation parameters in a
//! device-specific 24-bit floating-point format. [`Float24`] converts an `f32` into the
//! MSB-first bytes expected by those registers. The format consists of one sign bit, a 7-bit
//! exponent biased by 63, and a 16-bit mantissa; conversion truncates the lower `f32`
//! mantissa bits.
//!
//! ## Full three-byte values
//!
//! [`Float24::encode`] is used for reference resistors, Steinhart–Hart coefficients, and
//! sense-resistor temperature coefficients. These examples are values from a realistic
//! 10 kΩ NTC configuration.
//!
//! ```
//! # use ltc2949::float24::Float24;
//! let reference_resistor = Float24::new(10_000.0).encode();
//! let coefficient_a = Float24::new(1.1382e-3).encode();
//!
//! assert_eq!([0x4C, 0x38, 0x80], reference_resistor);
//! assert_eq!([0x35, 0x2A, 0x5F], coefficient_a);
//! ```
//!
//! ## Truncated two-byte values
//!
//! [`Float24::encode_high`] returns only the exponent/sign byte and the high mantissa byte.
//! The `RSxT0` registers use this representation and implicitly treat the omitted mantissa
//! byte as zero.
//!
//! ```
//! # use ltc2949::float24::Float24;
//! let reference_temperature = Float24::new(20.0).encode_high();
//!
//! assert_eq!([0x43, 0x40], reference_temperature);
//! ```
//!
//! ## Range handling
//!
//! Zero retains its sign. Values below the Float24 normal range underflow to signed zero,
//! while values above its finite range saturate at the largest magnitude.
//!
//! ```
//! # use ltc2949::float24::Float24;
//! assert_eq!([0x00, 0x00, 0x00], Float24::new(f32::MIN_POSITIVE).encode());
//! assert_eq!([0x80, 0x00, 0x00], Float24::new(-0.0).encode());
//! assert_eq!([0x7E, 0xFF, 0xFF], Float24::new(f32::MAX).encode());
//! assert_eq!([0xFE, 0xFF, 0xFF], Float24::new(f32::MIN).encode());
//! ```

/// LTC2949 Float24 value encoder.
///
/// Float24 is stored MSB-first with 1 sign bit, a 7-bit exponent biased by 63,
/// and a 16-bit mantissa (datasheet Table 68).
#[derive(Copy, Clone, PartialEq, Debug)]
pub struct Float24 {
    value: f32,
}

impl Float24 {
    /// Creates a Float24 encoder for `value`.
    pub const fn new(value: f32) -> Self {
        Self { value }
    }

    /// Encodes the value as three MSB-first Float24 bytes.
    ///
    /// Out-of-range values clamp to +/-0 or the largest finite magnitude.
    pub fn encode(self) -> [u8; 3] {
        let bits = self.value.to_bits();
        let sign = (bits >> 31) & 1;
        let f32_exp = (bits >> 23) & 0xFF;
        let f32_mantissa = bits & 0x7F_FFFF;

        // Zero / subnormal: encode as signed zero. Float24 has no subnormal range
        // worth caring about for the driver's supported configuration values.
        if f32_exp == 0 {
            return [(sign as u8) << 7, 0, 0];
        }

        // Re-bias: f32 bias 127 -> Float24 bias 63 -> subtract 64. Clamp to the
        // 7-bit Float24 exponent range.
        let exp_signed = f32_exp as i32 - 64;
        let (float24_exp, float24_mantissa) = if exp_signed < 1 {
            // Underflow -> signed zero.
            (0u32, 0u32)
        } else if exp_signed > 0x7E {
            // Overflow -> largest finite magnitude (exp=0x7E, mantissa=all-ones).
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

    /// Encodes only the top two Float24 bytes.
    ///
    /// This is used for the 16-bit `RSxT0` registers; the device treats the
    /// missing mantissa LSB as 0.
    pub fn encode_high(self) -> [u8; 2] {
        let [b0, b1, _] = self.encode();
        [b0, b1]
    }
}
