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
