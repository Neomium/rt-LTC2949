//! Byte-level SPI assertions for the LTC2949 driver.
//!
//! Each test sets up a mock [`embedded_hal::spi::SpiDevice`] with an explicit sequence
//! of expected `transaction(...)` calls and asserts that the bytes pushed onto MOSI
//! (and, where relevant, the layout of MISO data supplied) match the framing the
//! datasheet describes for the addressable, parallel-to-daisy-chain topology
//! (Figure 12(B)).
//!
//! Framing checked here:
//! * **DCMD writes** — `[0xFE, RADDR, PEC0, PEC1, ID, D0..Dn-1, DPEC0, DPEC1]` with
//!   `ID = 0x5B` for PECC=15 (write).
//! * **DCMD reads** — `[0xFE, RADDR, PEC0, PEC1, ID=0x9B, <dummy>]` on MOSI, with the
//!   slave's `[D0..Dn-1, DPEC0, DPEC1]` appearing on MISO after the 5-byte header.
//! * **Broadcast 16-bit commands** — `[CMD0, CMD1, PEC0, PEC1]` (e.g. ADCV = 0x0260).

use crate::ltc2949::{
    float24_encode, float24_encode_high2, Accumulators, AdcConf, Channel, FaCtrl, Ltc2949Client, MuxInput, NtcConfig,
    OpCtrl, ShuntTcConfig, LTC2949, T_BOOT_US, T_MLCK_US, T_READY_US,
};
use crate::mocks::MockSPIDevice;
use crate::pec15::PEC15;
use alloc::vec;
use alloc::vec::Vec;
use embedded_hal::spi::Operation;

/// ID byte for DCMD writes with PECC=15 (16 data bytes per PEC group). Datasheet
/// Table 12 — bit 7 RW=0, bit 6 !RW=1, PECC=0b1111 → 0b0101_1011.
const DCMD_ID_WRITE: u8 = 0x5B;

/// ID byte for DCMD reads with PECC=15 — bit 7 RW=1, bit 6 !RW=0 → 0b1001_1011.
const DCMD_ID_READ: u8 = 0x9B;

/// REGSCTRL bytes written by `select_page` in the parallel topology. RDCVCONF=1
/// (bit 7), BCREN=0, PAGE selects the page (bit 0):
///   * `0x80` — PAGE0
///   * `0x81` — PAGE1
const REGSCTRL_PAGE0: u8 = 0x80;
const REGSCTRL_PAGE1: u8 = 0x81;

/// REGSCTRL byte with the memory-lock request set (MLK = 0b01 in bits [5:4]) on PAGE0:
/// `0x80 | 0x10 = 0x90`.
const REGSCTRL_PAGE0_LOCK: u8 = 0x90;

// ---------------------------------------------------------------------------
// Frame builders
// ---------------------------------------------------------------------------

/// Constructs the exact byte sequence the driver should produce for a DCMD write of
/// `data` to register `addr`.
fn dcmd_write_bytes(addr: u8, data: &[u8]) -> Vec<u8> {
    let mut v = Vec::with_capacity(7 + data.len());
    v.push(0xFE);
    v.push(addr);
    let header_pec = PEC15::calc(&[0xFE, addr]);
    v.push(header_pec[0]);
    v.push(header_pec[1]);
    v.push(DCMD_ID_WRITE);
    v.extend_from_slice(data);
    let data_pec = PEC15::calc(data);
    v.push(data_pec[0]);
    v.push(data_pec[1]);
    v
}

/// Builds the expected MOSI frame and the MISO response for a DCMD read of `data.len()`
/// bytes from `addr`.
///
/// * MOSI: `[0xFE, RADDR, hpec0, hpec1, ID_read, 0xFF…]` — the command header followed
///   by `data.len() + 2` don't-care bytes the master clocks to receive the response.
/// * MISO: `[_, _, _, _, _, D0…D(n-1), dpec0, dpec1]` — first five bytes are ignored by
///   the driver (the command echo region); the data and its PEC follow.
fn dcmd_read_frames(addr: u8, data: &[u8]) -> (Vec<u8>, Vec<u8>) {
    let n = data.len();
    let total = 5 + n + 2;

    let mut tx = vec![0xFFu8; total];
    tx[0] = 0xFE;
    tx[1] = addr;
    let hpec = PEC15::calc(&[0xFE, addr]);
    tx[2] = hpec[0];
    tx[3] = hpec[1];
    tx[4] = DCMD_ID_READ;

    let mut rx = vec![0u8; total];
    rx[5..5 + n].copy_from_slice(data);
    let dpec = PEC15::calc(data);
    rx[5 + n] = dpec[0];
    rx[6 + n] = dpec[1];

    (tx, rx)
}

/// Constructs a 4-byte broadcast LTC681X-style command (CMD0, CMD1, PEC0, PEC1).
fn cmd16_bytes(cmd: u16) -> [u8; 4] {
    let mut f = [(cmd >> 8) as u8, cmd as u8, 0, 0];
    let pec = PEC15::calc(&f[..2]);
    f[2] = pec[0];
    f[3] = pec[1];
    f
}

// ---------------------------------------------------------------------------
// Mock expectation helpers
// ---------------------------------------------------------------------------

/// Records an expectation that the next `transaction()` call carries a single
/// `Operation::Write` with exactly `expected` bytes.
fn expect_write(mock: &mut MockSPIDevice, expected: Vec<u8>) {
    mock.expect_transaction().times(1).returning(move |ops| {
        assert_eq!(1, ops.len(), "expected a single Operation in the transaction");
        match &ops[0] {
            Operation::Write(bytes) => {
                assert_eq!(expected.as_slice(), *bytes, "unexpected MOSI bytes on Write");
            }
            other => panic!("expected Operation::Write, got {:?}", other),
        }
        Ok(())
    });
}

/// Records an expectation that the next `transaction()` call carries a single
/// `Operation::Transfer` with exactly `expected_tx` bytes on MOSI, and asks the
/// mock to copy `rx_payload` into the MISO buffer.
fn expect_transfer(mock: &mut MockSPIDevice, expected_tx: Vec<u8>, rx_payload: Vec<u8>) {
    assert_eq!(
        expected_tx.len(),
        rx_payload.len(),
        "test setup: TX and RX must match length"
    );
    mock.expect_transaction().times(1).returning(move |ops| {
        assert_eq!(1, ops.len(), "expected a single Operation in the transaction");
        match &mut ops[0] {
            Operation::Transfer(rx, tx) => {
                assert_eq!(expected_tx.as_slice(), *tx, "unexpected MOSI bytes on Transfer");
                assert_eq!(expected_tx.len(), rx.len());
                rx.copy_from_slice(&rx_payload);
            }
            other => panic!("expected Operation::Transfer, got {:?}", other),
        }
        Ok(())
    });
}

/// Records a direct DCMD read returning `data` from register `addr`.
fn expect_dcmd_read(mock: &mut MockSPIDevice, addr: u8, data: &[u8]) {
    let (tx, rx) = dcmd_read_frames(addr, data);
    expect_transfer(mock, tx, rx);
}

/// Records the one-time `select_page` write the driver emits the first time it
/// touches a page (cached afterwards via `current_page`).
fn expect_select_page(mock: &mut MockSPIDevice, page1: bool) {
    let byte = if page1 { REGSCTRL_PAGE1 } else { REGSCTRL_PAGE0 };
    expect_write(mock, dcmd_write_bytes(0xFF, &[byte]));
}

// ---------------------------------------------------------------------------
// DCMD ID-byte encoding (datasheet Table 12) — pure constants, no bus involved
// ---------------------------------------------------------------------------

/// Computes a DCMD ID byte from the read/write flag and the PECC field, mirroring
/// the formula in [`crate::ltc2949`]. Used here only as a self-contained witness
/// that our `DCMD_ID_WRITE` constant matches the datasheet encoding.
fn make_id(read: bool, pecc: u8) -> u8 {
    let pecc = pecc & 0x0F;
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

/// Read with PECC=15 (16 data bytes per PEC) should yield 0x9B.
#[test]
fn id_byte_read_pecc15_is_0x9b() {
    assert_eq!(0x9B, make_id(true, 15));
}

/// Write with PECC=15 (the value the driver uses) should yield 0x5B, matching
/// `DCMD_ID_WRITE`.
#[test]
fn id_byte_write_pecc15_matches_driver_constant() {
    assert_eq!(DCMD_ID_WRITE, make_id(false, 15));
}

/// Write with PECC=1 (2 data bytes per PEC) should yield 0x45 — matches the
/// MUXCONT example in the datasheet's Fast AUX Round-Robin Measurements section.
#[test]
fn id_byte_write_pecc1_matches_datasheet_example() {
    assert_eq!(0x45, make_id(false, 1));
}

// ---------------------------------------------------------------------------
// Command tests
// ---------------------------------------------------------------------------

#[test]
fn trigger_adcv_broadcast_emits_0x0260_with_pec() {
    let mut mock = MockSPIDevice::new();
    expect_write(&mut mock, cmd16_bytes(0x0260).to_vec());

    let mut client: LTC2949<_, _> = LTC2949::new(mock);
    client.trigger_adcv_broadcast().unwrap();
}

#[test]
fn wake_up_writes_two_dummy_bytes_then_selects_page_then_clears_wkupack() {
    let mut mock = MockSPIDevice::new();
    expect_write(&mut mock, vec![0x00]);
    expect_write(&mut mock, vec![0x00]);
    // The tBOOT wait is host-owned and issues no bus traffic.
    expect_select_page(&mut mock, false);
    // WKUPACK lives at 0x70; write 0x00 to confirm wake-up.
    expect_write(&mut mock, dcmd_write_bytes(0x70, &[0x00]));

    let mut client: LTC2949<_, _> = LTC2949::new(mock);
    assert_eq!(client.start_wake_up().unwrap(), T_BOOT_US);
    client.confirm_wake_up().unwrap();
}

#[test]
fn wake_isospi_emits_single_dummy_byte() {
    let mut mock = MockSPIDevice::new();
    expect_write(&mut mock, vec![0x00]);

    let mut client: LTC2949<_, _> = LTC2949::new(mock);
    assert_eq!(client.wake_isospi().unwrap(), T_READY_US);
}

#[test]
fn wake_up_invalidates_page_cache() {
    // Drive the device onto PAGE1, then wake up; the next PAGE1 access must re-issue the
    // REGSCTRL page select because start_wake_up dropped the cached page.
    let mut mock = MockSPIDevice::new();
    // 1) write_adcconf → select PAGE1 + DCMD write.
    expect_select_page(&mut mock, true);
    expect_write(&mut mock, dcmd_write_bytes(0xDF, &[0x00]));
    // 2) wake-up: two dummy bytes, then WKUPACK on PAGE0 (cache was None → select PAGE0).
    expect_write(&mut mock, vec![0x00]);
    expect_write(&mut mock, vec![0x00]);
    expect_select_page(&mut mock, false);
    expect_write(&mut mock, dcmd_write_bytes(0x70, &[0x00]));
    // 3) write_adcconf again → because the cache was invalidated by start_wake_up (and is
    //    now PAGE0 from the WKUPACK path), PAGE1 must be re-selected.
    expect_select_page(&mut mock, true);
    expect_write(&mut mock, dcmd_write_bytes(0xDF, &[0x00]));

    let mut client: LTC2949<_, _> = LTC2949::new(mock);
    client.write_adcconf(AdcConf::default()).unwrap();
    client.start_wake_up().unwrap();
    client.confirm_wake_up().unwrap();
    client.write_adcconf(AdcConf::default()).unwrap();
}

#[test]
fn read_faults_and_extfaults_read_correct_addresses() {
    let mut mock = MockSPIDevice::new();
    expect_select_page(&mut mock, false);
    expect_dcmd_read(&mut mock, 0xDD, &[0x42]); // FAULTS
    expect_dcmd_read(&mut mock, 0xDC, &[0x99]); // EXTFAULTS

    let mut client: LTC2949<_, _> = LTC2949::new(mock);
    assert_eq!(client.read_faults().unwrap(), 0x42);
    assert_eq!(client.read_extfaults().unwrap(), 0x99);
}

#[test]
fn read_accumulators1_decodes_row_in_one_coherent_burst() {
    // One 16-byte burst from 0x00 covers Charge1 (0x00–0x05), Energy1 (0x06–0x0B) and
    // Time1 (0x0C–0x0F). charge = +5, energy = -1, time = 10.
    let row = [
        0x00, 0x00, 0x00, 0x00, 0x00, 0x05, // charge = 5
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // energy = -1
        0x00, 0x00, 0x00, 0x0A, // time = 10
    ];
    let mut mock = MockSPIDevice::new();
    expect_select_page(&mut mock, false);
    expect_dcmd_read(&mut mock, 0x00, &row);

    let mut client: LTC2949<_, _> = LTC2949::new(mock);
    let acc = client.read_accumulators1().unwrap();
    assert_eq!(
        acc,
        Accumulators {
            charge: 5,
            energy: -1,
            time: 10
        }
    );
}

#[test]
fn read_accumulators2_reads_from_row_0x10() {
    let row = [0u8; 16];
    let mut mock = MockSPIDevice::new();
    expect_select_page(&mut mock, false);
    expect_dcmd_read(&mut mock, 0x10, &row);

    let mut client: LTC2949<_, _> = LTC2949::new(mock);
    let acc = client.read_accumulators2().unwrap();
    assert_eq!(
        acc,
        Accumulators {
            charge: 0,
            energy: 0,
            time: 0
        }
    );
}

#[test]
fn memory_lock_request_read_unlock_sequence() {
    // request_memory_lock writes REGSCTRL with MLK=01 (0x90) and returns the host-owned
    // wait; a read of Charge1 on the already-selected PAGE0 issues no extra REGSCTRL
    // (which would have released the lock); unlock_memory writes 0x80.
    let mut mock = MockSPIDevice::new();
    // lock request: REGSCTRL MLK. current_page starts None → base page is PAGE0, and the
    // lock pins PAGE0 into the cache.
    expect_write(&mut mock, dcmd_write_bytes(0xFF, &[REGSCTRL_PAGE0_LOCK]));
    // read Charge1 (0x00). PAGE0 is now cached, so NO REGSCTRL rewrite — straight to the
    // DCMD read.
    expect_dcmd_read(&mut mock, 0x00, &[0, 0, 0, 0, 0, 1]);
    // unlock: REGSCTRL back to plain PAGE0.
    expect_write(&mut mock, dcmd_write_bytes(0xFF, &[REGSCTRL_PAGE0]));

    let mut client: LTC2949<_, _> = LTC2949::new(mock);
    assert_eq!(client.request_memory_lock().unwrap(), T_MLCK_US);
    // ... host waits T_MLCK_US microseconds (no bus traffic) ...
    let charge = client.read_charge1().unwrap();
    client.unlock_memory().unwrap();
    assert_eq!(charge, 1);
}

// ---------------------------------------------------------------------------
// Writes
// ---------------------------------------------------------------------------

#[test]
fn write_opctrl_cont_emits_dcmd_with_bit3_set() {
    // CONT = bit 3 → byte 0x08.
    let mut mock = MockSPIDevice::new();
    expect_select_page(&mut mock, false);
    expect_write(&mut mock, dcmd_write_bytes(0xF0, &[0x08]));

    let mut client: LTC2949<_, _> = LTC2949::new(mock);
    client.write_opctrl(OpCtrl::default().with_cont(true)).unwrap();
}

#[test]
fn write_opctrl_sleep_and_rst_emits_correct_byte() {
    // SLEEP=bit0 (0x01) | RST=bit7 (0x80) → 0x81.
    let mut mock = MockSPIDevice::new();
    expect_select_page(&mut mock, false);
    expect_write(&mut mock, dcmd_write_bytes(0xF0, &[0x81]));

    let mut client: LTC2949<_, _> = LTC2949::new(mock);
    client.write_opctrl(OpCtrl::default().with_sleep(true).with_rst(true)).unwrap();
}

#[test]
fn write_factrl_enables_fast_channels_1_and_2() {
    // FACH1=bit2 (0x04) | FACH2=bit3 (0x08) → 0x0C.
    let mut mock = MockSPIDevice::new();
    expect_select_page(&mut mock, false);
    expect_write(&mut mock, dcmd_write_bytes(0xF5, &[0x0C]));

    let mut client: LTC2949<_, _> = LTC2949::new(mock);
    client
        .write_factrl(FaCtrl::default().with_fach1(true).with_fach2(true))
        .unwrap();
}

#[test]
fn write_adcconf_uses_page1_and_writes_to_0xdf() {
    // NTC1 = bit 3 → byte 0x08.
    let mut mock = MockSPIDevice::new();
    expect_select_page(&mut mock, true);
    expect_write(&mut mock, dcmd_write_bytes(0xDF, &[0x08]));

    let mut client: LTC2949<_, _> = LTC2949::new(mock);
    client.write_adcconf(AdcConf::default().with_ntc1(true)).unwrap();
}

#[test]
fn write_opctrl_then_write_factrl_only_selects_page_once() {
    // After the first call has selected PAGE0, the second call must skip the
    // REGSCTRL write thanks to the `current_page` cache.
    let mut mock = MockSPIDevice::new();
    expect_select_page(&mut mock, false);
    expect_write(&mut mock, dcmd_write_bytes(0xF0, &[0x08])); // OPCTRL.CONT
    expect_write(&mut mock, dcmd_write_bytes(0xF5, &[0x01])); // FACTRL.FACONV

    let mut client: LTC2949<_, _> = LTC2949::new(mock);
    client.write_opctrl(OpCtrl::default().with_cont(true)).unwrap();
    client.write_factrl(FaCtrl::default().with_faconv(true)).unwrap();
}

// ---------------------------------------------------------------------------
// Reads (direct DCMD)
// ---------------------------------------------------------------------------

#[test]
fn read_uses_dcmd_read_frame_with_read_id() {
    // Verify the exact MOSI frame of a direct DCMD read: command header + read ID
    // (0x9B) + don't-care padding. This is the defining behaviour of the parallel
    // topology (vs. the broadcast-RDCV path used when on top of a chain).
    let mut mock = MockSPIDevice::new();
    expect_select_page(&mut mock, false);

    // STATUS at 0x80, one data byte = 0xA5.
    let (tx, rx) = dcmd_read_frames(0x80, &[0xA5]);
    assert_eq!(0xFE, tx[0]);
    assert_eq!(0x80, tx[1]);
    assert_eq!(DCMD_ID_READ, tx[4]); // read ID, not write
    expect_transfer(&mut mock, tx, rx);

    let mut client = LTC2949::new(mock);
    assert_eq!(0xA5, client.read_status().unwrap());
}

#[test]
fn read_opctrl_decodes_bitfield() {
    // Device reports OPCTRL = CONT|ADJUPD = 0x08 | 0x20 = 0x28.
    let mut mock = MockSPIDevice::new();
    expect_select_page(&mut mock, false);
    expect_dcmd_read(&mut mock, 0xF0, &[0x28]);

    let mut client = LTC2949::new(mock);
    let value = client.read_opctrl().unwrap();
    assert!(value.cont());
    assert!(value.adjupd());
    assert!(!value.sleep());
    assert!(!value.rst());
}

#[test]
fn read_status_returns_raw_byte() {
    let mut mock = MockSPIDevice::new();
    expect_select_page(&mut mock, false);
    expect_dcmd_read(&mut mock, 0x80, &[0xA5]);

    let mut client = LTC2949::new(mock);
    assert_eq!(0xA5, client.read_status().unwrap());
}

#[test]
fn read_current1_decodes_24bit_signed_be() {
    // 24-bit two's complement of -1 = 0xFFFFFF (MSB-first on the bus).
    let mut mock = MockSPIDevice::new();
    expect_select_page(&mut mock, false);
    expect_dcmd_read(&mut mock, 0x90, &[0xFF, 0xFF, 0xFF]);

    let mut client = LTC2949::new(mock);
    assert_eq!(-1, client.read_current1().unwrap());
}

#[test]
fn read_current1_positive_value() {
    // 0x000123 = 291 (positive).
    let mut mock = MockSPIDevice::new();
    expect_select_page(&mut mock, false);
    expect_dcmd_read(&mut mock, 0x90, &[0x00, 0x01, 0x23]);

    let mut client = LTC2949::new(mock);
    assert_eq!(0x000123, client.read_current1().unwrap());
}

#[test]
fn read_bat_decodes_16bit_signed_be() {
    // 0x7FFF = 32767 (max positive).
    let mut mock = MockSPIDevice::new();
    expect_select_page(&mut mock, false);
    expect_dcmd_read(&mut mock, 0xA0, &[0x7F, 0xFF]);

    let mut client = LTC2949::new(mock);
    assert_eq!(32767, client.read_bat().unwrap());
}

#[test]
fn read_charge1_decodes_48bit_signed_be() {
    // 48-bit value 0x0000_0000_0001 = 1.
    let mut mock = MockSPIDevice::new();
    expect_select_page(&mut mock, false);
    expect_dcmd_read(&mut mock, 0x00, &[0x00, 0x00, 0x00, 0x00, 0x00, 0x01]);

    let mut client = LTC2949::new(mock);
    assert_eq!(1, client.read_charge1().unwrap());
}

#[test]
fn read_charge1_decodes_48bit_signed_negative() {
    // All-ones 48-bit → -1 after sign-extension to i64.
    let mut mock = MockSPIDevice::new();
    expect_select_page(&mut mock, false);
    expect_dcmd_read(&mut mock, 0x00, &[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]);

    let mut client = LTC2949::new(mock);
    assert_eq!(-1, client.read_charge1().unwrap());
}

#[test]
fn read_charge3_decodes_64bit_signed() {
    let payload = [0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0];
    let mut mock = MockSPIDevice::new();
    expect_select_page(&mut mock, false);
    expect_dcmd_read(&mut mock, 0x24, &payload);

    let mut client = LTC2949::new(mock);
    let expected = i64::from_be_bytes(payload);
    assert_eq!(expected, client.read_charge3().unwrap());
}

#[test]
fn read_time1_decodes_32bit_unsigned_be() {
    // 0xDEAD_BEEF.
    let mut mock = MockSPIDevice::new();
    expect_select_page(&mut mock, false);
    expect_dcmd_read(&mut mock, 0x0C, &[0xDE, 0xAD, 0xBE, 0xEF]);

    let mut client = LTC2949::new(mock);
    assert_eq!(0xDEAD_BEEF, client.read_time1().unwrap());
}

// ---------------------------------------------------------------------------
// Page handling
// ---------------------------------------------------------------------------

#[test]
fn page1_write_then_page0_read_toggles_page_bit() {
    // A page-1 write (ADCCONF) selects PAGE1; a subsequent page-0 read must
    // re-select PAGE0 before its DCMD read.
    let mut mock = MockSPIDevice::new();
    expect_select_page(&mut mock, true);
    expect_write(&mut mock, dcmd_write_bytes(0xDF, &[0x08])); // ADCCONF write
    expect_select_page(&mut mock, false); // switching back to page 0
    expect_dcmd_read(&mut mock, 0x80, &[0x00]);

    let mut client = LTC2949::new(mock);
    client.write_adcconf(AdcConf::default().with_ntc1(true)).unwrap();
    let _ = client.read_status().unwrap();
}

// ---------------------------------------------------------------------------
// Float24 encoding (datasheet Table 68 + Table 75 example values)
// ---------------------------------------------------------------------------

#[test]
fn float24_encode_zero_is_signed_zero() {
    assert_eq!([0x00, 0x00, 0x00], float24_encode(0.0));
    assert_eq!([0x80, 0x00, 0x00], float24_encode(-0.0));
}

#[test]
fn float24_encode_matches_datasheet_example_0_95() {
    // Datasheet Table 68 worked example: 0.95 → 0x3EE666.
    assert_eq!([0x3E, 0xE6, 0x66], float24_encode(0.95));
}

#[test]
fn float24_encode_matches_table_75_rref_10k() {
    // RREF1 = 10 kΩ → 0x4C3880 (from the NTCLE203E example table).
    assert_eq!([0x4C, 0x38, 0x80], float24_encode(10_000.0));
}

#[test]
fn float24_encode_matches_table_75_coefficient_a() {
    // NTC1A ≈ 1.1382e-3 → 0x352A5F.
    assert_eq!([0x35, 0x2A, 0x5F], float24_encode(1.1382e-3));
}

#[test]
fn float24_encode_matches_table_75_coefficient_b() {
    // NTC1B ≈ 2.3267e-4 → 0x32E7F1.
    assert_eq!([0x32, 0xE7, 0xF1], float24_encode(2.3267e-4));
}

#[test]
fn float24_encode_matches_table_75_coefficient_c() {
    // NTC1C ≈ 0.93243e-7 → 0x279079.
    assert_eq!([0x27, 0x90, 0x79], float24_encode(0.93243e-7));
}

#[test]
fn float24_encode_negative_value_sets_sign_bit() {
    // Negation of 0.95: same magnitude bytes with bit 7 of byte 0 set.
    let [b0, b1, b2] = float24_encode(0.95);
    assert_eq!([b0 | 0x80, b1, b2], float24_encode(-0.95));
}

// ---------------------------------------------------------------------------
// NTC coefficient writes
// ---------------------------------------------------------------------------

/// The NTCLE203E worked example from datasheet Table 75 ("NTC1 Values in NTC
/// Configuration Register"). Used as a canonical, end-to-end test vector.
const NTCLE203E_EXAMPLE: NtcConfig = NtcConfig {
    r_ref: 10_000.0,
    a: 1.1382e-3,
    b: 2.3267e-4,
    c: 0.93243e-7,
};

#[test]
fn write_ntc1_coefficients_sends_rref_then_abc_burst() {
    let mut mock = MockSPIDevice::new();
    expect_select_page(&mut mock, true);

    // 1) 3-byte write of RREF1 at p1.0xAA.
    expect_write(&mut mock, dcmd_write_bytes(0xAA, &float24_encode(10_000.0)));

    // 2) 9-byte burst writing NTC1A | NTC1B | NTC1C at p1.0xD0.
    let mut abc = vec![0u8; 9];
    abc[0..3].copy_from_slice(&float24_encode(1.1382e-3));
    abc[3..6].copy_from_slice(&float24_encode(2.3267e-4));
    abc[6..9].copy_from_slice(&float24_encode(0.93243e-7));
    expect_write(&mut mock, dcmd_write_bytes(0xD0, &abc));

    let mut client: LTC2949<_, _> = LTC2949::new(mock);
    client.write_ntc_coefficients(Channel::One, &NTCLE203E_EXAMPLE).unwrap();
}

#[test]
fn write_ntc2_coefficients_targets_distinct_addresses() {
    let mut mock = MockSPIDevice::new();
    expect_select_page(&mut mock, true);

    // RREF2 lives at p1.0xAD, the NTC2A/B/C burst at p1.0xE0.
    expect_write(&mut mock, dcmd_write_bytes(0xAD, &float24_encode(10_000.0)));
    let mut abc = vec![0u8; 9];
    abc[0..3].copy_from_slice(&float24_encode(1.1382e-3));
    abc[3..6].copy_from_slice(&float24_encode(2.3267e-4));
    abc[6..9].copy_from_slice(&float24_encode(0.93243e-7));
    expect_write(&mut mock, dcmd_write_bytes(0xE0, &abc));

    let mut client: LTC2949<_, _> = LTC2949::new(mock);
    client.write_ntc_coefficients(Channel::Two, &NTCLE203E_EXAMPLE).unwrap();
}

#[test]
fn write_ntc1_then_ntc2_only_selects_page1_once() {
    // The `current_page` cache should suppress the second select_page write.
    let mut mock = MockSPIDevice::new();
    expect_select_page(&mut mock, true);

    // NTC1
    expect_write(&mut mock, dcmd_write_bytes(0xAA, &float24_encode(10_000.0)));
    let mut abc = vec![0u8; 9];
    abc[0..3].copy_from_slice(&float24_encode(1.1382e-3));
    abc[3..6].copy_from_slice(&float24_encode(2.3267e-4));
    abc[6..9].copy_from_slice(&float24_encode(0.93243e-7));
    expect_write(&mut mock, dcmd_write_bytes(0xD0, &abc));

    // NTC2 (no extra select_page).
    expect_write(&mut mock, dcmd_write_bytes(0xAD, &float24_encode(10_000.0)));
    expect_write(&mut mock, dcmd_write_bytes(0xE0, &abc));

    let mut client: LTC2949<_, _> = LTC2949::new(mock);
    client.write_ntc_coefficients(Channel::One, &NTCLE203E_EXAMPLE).unwrap();
    client.write_ntc_coefficients(Channel::Two, &NTCLE203E_EXAMPLE).unwrap();
}

// ---------------------------------------------------------------------------
// Float24 truncated-to-2-bytes (RSnT0 reference temperature)
// ---------------------------------------------------------------------------

#[test]
fn float24_encode_high2_drops_low_byte() {
    // 0.95 → 0x3EE666; truncated → 0x3EE6.
    assert_eq!([0x3E, 0xE6], float24_encode_high2(0.95));
}

#[test]
fn float24_encode_high2_matches_table_76_row() {
    // RS1T0 reference temperature = 20 °C → 0x4340 (datasheet Table 76).
    // The bottom byte is unrepresented and the device assumes 0.
    assert_eq!([0x43, 0x40], float24_encode_high2(20.0));
}

// ---------------------------------------------------------------------------
// Shunt-resistor temperature compensation writes
// ---------------------------------------------------------------------------

/// Sense-resistor TC values from datasheet Table 76 (copper shunt nominally
/// trimmed at 20 °C, TC = 3900 ppm/K, no second-order term).
const COPPER_SHUNT_25C: ShuntTcConfig = ShuntTcConfig {
    tc: 0.0039,
    t_ref: 20.0,
    tc2: 0.0,
};

#[test]
fn write_shunt_tc_channel1_sends_burst_then_tc2() {
    let mut mock = MockSPIDevice::new();
    expect_select_page(&mut mock, true);

    // 5-byte burst at p1.0xD9 = RSnTC (3 bytes) || RSnT0 (2 bytes).
    let mut burst = vec![0u8; 5];
    burst[0..3].copy_from_slice(&float24_encode(0.0039));
    burst[3..5].copy_from_slice(&float24_encode_high2(20.0));
    expect_write(&mut mock, dcmd_write_bytes(0xD9, &burst));

    // 3-byte write of RS1TC2 at p1.0x5C.
    expect_write(&mut mock, dcmd_write_bytes(0x5C, &float24_encode(0.0)));

    let mut client: LTC2949<_, _> = LTC2949::new(mock);
    client.write_shunt_tc(Channel::One, &COPPER_SHUNT_25C).unwrap();
}

#[test]
fn write_shunt_tc_channel2_targets_distinct_addresses() {
    let mut mock = MockSPIDevice::new();
    expect_select_page(&mut mock, true);

    let mut burst = vec![0u8; 5];
    burst[0..3].copy_from_slice(&float24_encode(0.0039));
    burst[3..5].copy_from_slice(&float24_encode_high2(20.0));
    expect_write(&mut mock, dcmd_write_bytes(0xE9, &burst)); // RS2TC + RS2T0
    expect_write(&mut mock, dcmd_write_bytes(0x7C, &float24_encode(0.0))); // RS2TC2

    let mut client: LTC2949<_, _> = LTC2949::new(mock);
    client.write_shunt_tc(Channel::Two, &COPPER_SHUNT_25C).unwrap();
}

// ---------------------------------------------------------------------------
// SLOT mux configuration
// ---------------------------------------------------------------------------

#[test]
fn write_slot_mux_channel1_writes_two_byte_burst_at_0xeb() {
    // SLOT1MUXN at 0xEB, SLOT1MUXP at 0xEC. Driver emits them as one burst.
    // Negative = AGND (0), Positive = V1 (1).
    let mut mock = MockSPIDevice::new();
    expect_select_page(&mut mock, false);
    expect_write(&mut mock, dcmd_write_bytes(0xEB, &[0x00, 0x01]));

    let mut client: LTC2949<_, _> = LTC2949::new(mock);
    client.write_slot_mux(Channel::One, MuxInput::Agnd, MuxInput::V1).unwrap();
}

#[test]
fn write_slot_mux_channel2_writes_to_0xed() {
    // SLOT2MUXN at 0xED, SLOT2MUXP at 0xEE. Use VbatM/VbatP for variety.
    let mut mock = MockSPIDevice::new();
    expect_select_page(&mut mock, false);
    expect_write(&mut mock, dcmd_write_bytes(0xED, &[15, 16]));

    let mut client: LTC2949<_, _> = LTC2949::new(mock);
    client.write_slot_mux(Channel::Two, MuxInput::VbatM, MuxInput::VbatP).unwrap();
}

#[test]
fn mux_input_discriminants_match_datasheet_table_57() {
    // Spot-check the corners of Table 57 — the integer values are encoded as
    // the 5-bit MUX setting on the wire.
    assert_eq!(0, MuxInput::Agnd as u8);
    assert_eq!(1, MuxInput::V1 as u8);
    assert_eq!(12, MuxInput::V12 as u8);
    assert_eq!(15, MuxInput::VbatM as u8);
    assert_eq!(16, MuxInput::VbatP as u8);
    assert_eq!(20, MuxInput::Cf1P as u8);
    assert_eq!(22, MuxInput::Vref2 as u8);
    assert_eq!(23, MuxInput::Vref2Via250k as u8);
}
