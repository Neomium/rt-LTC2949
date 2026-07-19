use crate::float24::Float24;

#[test]
fn encode_zero_is_signed_zero() {
    assert_eq!([0x00, 0x00, 0x00], Float24::new(0.0).encode());
    assert_eq!([0x80, 0x00, 0x00], Float24::new(-0.0).encode());
}

#[test]
fn encode_matches_datasheet_example_0_95() {
    // Datasheet Table 68 worked example: 0.95 -> 0x3EE666.
    assert_eq!([0x3E, 0xE6, 0x66], Float24::new(0.95).encode());
}

#[test]
fn encode_matches_table_75_rref_10k() {
    // RREF1 = 10 kOhm -> 0x4C3880 from the NTCLE203E example table.
    assert_eq!([0x4C, 0x38, 0x80], Float24::new(10_000.0).encode());
}

#[test]
fn encode_matches_table_75_coefficient_a() {
    // NTC1A ~= 1.1382e-3 -> 0x352A5F.
    assert_eq!([0x35, 0x2A, 0x5F], Float24::new(1.1382e-3).encode());
}

#[test]
fn encode_matches_table_75_coefficient_b() {
    // NTC1B ~= 2.3267e-4 -> 0x32E7F1.
    assert_eq!([0x32, 0xE7, 0xF1], Float24::new(2.3267e-4).encode());
}

#[test]
fn encode_matches_table_75_coefficient_c() {
    // NTC1C ~= 0.93243e-7 -> 0x279079.
    assert_eq!([0x27, 0x90, 0x79], Float24::new(0.93243e-7).encode());
}

#[test]
fn encode_negative_value_sets_sign_bit() {
    // Negation of 0.95: same magnitude bytes with bit 7 of byte 0 set.
    let [b0, b1, b2] = Float24::new(0.95).encode();
    assert_eq!([b0 | 0x80, b1, b2], Float24::new(-0.95).encode());
}

#[test]
fn encode_high_drops_low_byte() {
    // 0.95 -> 0x3EE666; truncated -> 0x3EE6.
    assert_eq!([0x3E, 0xE6], Float24::new(0.95).encode_high());
}

#[test]
fn encode_high_matches_table_76_row() {
    // RS1T0 reference temperature = 20 C -> 0x4340 (datasheet Table 76).
    // The bottom byte is unrepresented and the device assumes 0.
    assert_eq!([0x43, 0x40], Float24::new(20.0).encode_high());
}
