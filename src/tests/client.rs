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

use crate::client::{
    AccumulatedCharge, AccumulatedEnergy, AccumulatedTime, AdcConfiguration, Channel, Client, DcmdId,
    FastControlRegister, FifoTag, MuxInput, NtcConfig, OpsControlRegister, OverCurrentConfig, ShuntTcConfig, SlotValue,
    LTC2949, T_BOOT_US, T_MLCK_US, T_READY_US,
};
use crate::float24::Float24;
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

fn assert_f32_approx_eq(actual: f32, expected: f32) {
    const ABSOLUTE_TOLERANCE: f32 = 1e-15;
    const RELATIVE_TOLERANCE: f32 = 1e-6;

    let difference = (actual - expected).abs();
    let tolerance = ABSOLUTE_TOLERANCE.max(expected.abs() * RELATIVE_TOLERANCE);
    assert!(
        difference <= tolerance,
        "expected {expected}, got {actual}; difference {difference} exceeds tolerance {tolerance}"
    );
}

fn assert_f64_approx_eq(actual: f64, expected: f64) {
    const ABSOLUTE_TOLERANCE: f64 = 1e-15;
    const RELATIVE_TOLERANCE: f64 = 1e-12;

    let difference = (actual - expected).abs();
    let tolerance = ABSOLUTE_TOLERANCE.max(expected.abs() * RELATIVE_TOLERANCE);
    assert!(
        difference <= tolerance,
        "expected {expected}, got {actual}; difference {difference} exceeds tolerance {tolerance}"
    );
}

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

fn f24(value: f32) -> [u8; 3] {
    Float24::new(value).encode()
}

fn f24_high(value: f32) -> [u8; 2] {
    Float24::new(value).encode_high()
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

/// Computes a DCMD ID byte from the read/write flag and the PECC field, mirroring
/// the formula in [`crate::client`]. Used here only as a self-contained witness
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
    client.write_adcconf(AdcConfiguration::default()).unwrap();
    client.start_wake_up().unwrap();
    client.confirm_wake_up().unwrap();
    client.write_adcconf(AdcConfiguration::default()).unwrap();
}

#[test]
fn read_faults_and_extfaults_read_correct_addresses() {
    let mut mock = MockSPIDevice::new();
    expect_select_page(&mut mock, false);
    expect_dcmd_read(&mut mock, 0xDD, &[0x42]); // FAULTS
    expect_dcmd_read(&mut mock, 0xDC, &[0x99]); // EXTFAULTS

    let mut client: LTC2949<_, _> = LTC2949::new(mock);
    let faults = client.read_faults().unwrap();
    assert!(faults.tsd());
    assert!(faults.crccfg());
    assert!(!faults.promerr());
    assert!(!faults.crcmem());

    let extfaults = client.read_extfaults().unwrap();
    assert!(extfaults.hd1biterr());
    assert!(extfaults.fcaerr());
    assert!(extfaults.xramerr());
    assert!(extfaults.hwmbistexec());
    assert!(!extfaults.romerr());
    assert!(!extfaults.memerr());
    assert!(!extfaults.iramerr());
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
    assert_eq!(5, acc.charge.raw());
    assert_eq!(-1, acc.energy.raw());
    assert_eq!(10, acc.time.raw());
}

#[test]
fn read_accumulators2_reads_from_row_0x10() {
    let row = [0u8; 16];
    let mut mock = MockSPIDevice::new();
    expect_select_page(&mut mock, false);
    expect_dcmd_read(&mut mock, 0x10, &row);

    let mut client: LTC2949<_, _> = LTC2949::new(mock);
    let acc = client.read_accumulators2().unwrap();
    assert_eq!(0, acc.charge.raw());
    assert_eq!(0, acc.energy.raw());
    assert_eq!(0, acc.time.raw());
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
    assert_eq!(charge.raw(), 1);
}

#[test]
fn write_opctrl_cont_emits_dcmd_with_bit3_set() {
    // CONT = bit 3 → byte 0x08.
    let mut mock = MockSPIDevice::new();
    expect_select_page(&mut mock, false);
    expect_write(&mut mock, dcmd_write_bytes(0xF0, &[0x08]));

    let mut client: LTC2949<_, _> = LTC2949::new(mock);
    client.write_opctrl(OpsControlRegister::default().with_cont(true)).unwrap();
}

#[test]
fn write_opctrl_sleep_and_rst_emits_correct_byte() {
    // SLEEP=bit0 (0x01) | RST=bit7 (0x80) → 0x81.
    let mut mock = MockSPIDevice::new();
    expect_select_page(&mut mock, false);
    expect_write(&mut mock, dcmd_write_bytes(0xF0, &[0x81]));

    let mut client: LTC2949<_, _> = LTC2949::new(mock);
    client
        .write_opctrl(OpsControlRegister::default().with_sleep(true).with_rst(true))
        .unwrap();
}

#[test]
fn write_factrl_enables_fast_channels_1_and_2() {
    // FACH1=bit2 (0x04) | FACH2=bit3 (0x08) → 0x0C.
    let mut mock = MockSPIDevice::new();
    expect_select_page(&mut mock, false);
    expect_write(&mut mock, dcmd_write_bytes(0xF5, &[0x0C]));

    let mut client: LTC2949<_, _> = LTC2949::new(mock);
    client
        .write_factrl(FastControlRegister::default().with_fach1(true).with_fach2(true))
        .unwrap();
}

#[test]
fn write_gpio_ctrl_emits_dcmd_to_fgpioctrl() {
    // FGPIOCTRL has four 2-bit GPIO control fields. 0x03 sets GPIO1CTRL=0b11
    // (drive GPIO1 high) and leaves GPIO2..GPIO4 at 0b00 (tristate).
    let mut mock = MockSPIDevice::new();
    expect_select_page(&mut mock, false);
    expect_write(&mut mock, dcmd_write_bytes(0xF2, &[0x03]));

    let mut client: LTC2949<_, _> = LTC2949::new(mock);
    client.write_gpio_ctrl(0x03).unwrap();
}

#[test]
fn write_occ_config_packs_fields_and_writes_both_channels() {
    // OCCxCTRL layout: enable=bit0, threshold=bits[3:1],
    // deglitch_time=bits[5:4], polarity=bits[7:6].

    // Threshold selects a differential shunt-voltage limit, not a direct current.
    // Current limit is V_threshold / R_shunt. For the repo's 100 uOhm example shunt:
    //   threshold 0b011 = 78 mV -> 780 A
    //   threshold 0b001 = 26 mV -> 260 A
    // The second config is disabled, so its threshold is only a packing witness.
    let config1 = OverCurrentConfig {
        enable: true,
        threshold: 0b011,
        deglitch_time: 0b10,
        polarity: 0b1,
    };
    let config2 = OverCurrentConfig {
        enable: false,
        threshold: 0b001,
        deglitch_time: 0b01,
        polarity: 0b0,
    };

    let mut mock = MockSPIDevice::new();
    expect_select_page(&mut mock, false);
    expect_write(&mut mock, dcmd_write_bytes(0xDE, &[0x67]));
    expect_write(&mut mock, dcmd_write_bytes(0xDF, &[0x12]));

    let mut client: LTC2949<_, _> = LTC2949::new(mock);
    client.write_occ_config(config1, config2).unwrap();
}

#[test]
fn write_adcconf_uses_page1_and_writes_to_0xdf() {
    // NTC1 = bit 3 → byte 0x08.
    let mut mock = MockSPIDevice::new();
    expect_select_page(&mut mock, true);
    expect_write(&mut mock, dcmd_write_bytes(0xDF, &[0x08]));

    let mut client: LTC2949<_, _> = LTC2949::new(mock);
    client.write_adcconf(AdcConfiguration::default().with_ntc1(true)).unwrap();
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
    client.write_opctrl(OpsControlRegister::default().with_cont(true)).unwrap();
    client.write_factrl(FastControlRegister::default().with_faconv(true)).unwrap();
}

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
    let status = client.read_status().unwrap();
    assert!(status.uvloa());
    assert!(status.uvlostby());
    assert!(status.adcerr());
    assert!(!status.pora());
    assert!(!status.uvlod());
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
fn read_status_decodes_bitfield() {
    let mut mock = MockSPIDevice::new();
    expect_select_page(&mut mock, false);
    expect_dcmd_read(&mut mock, 0x80, &[0x7F]);

    let mut client = LTC2949::new(mock);
    let status = client.read_status().unwrap();
    assert!(status.uvloa());
    assert!(status.pora());
    assert!(status.uvlostby());
    assert!(status.uvlod());
    assert!(status.update());
    assert!(status.adcerr());
    assert!(status.tberr());
}

#[test]
fn read_current1_decodes_24bit_signed_be() {
    // 24-bit two's complement of -1 = 0xFFFFFF (MSB-first on the bus).
    let mut mock = MockSPIDevice::new();
    expect_select_page(&mut mock, false);
    expect_dcmd_read(&mut mock, 0x90, &[0xFF, 0xFF, 0xFF]);

    let mut client = LTC2949::new(mock);
    let current = client.read_current1().unwrap();
    assert_eq!(-1, current.raw());
    assert_f32_approx_eq(current.decode(), -950e-9);
}

#[test]
fn read_current1_positive_value() {
    // 0x000123 = 291 (positive).
    let mut mock = MockSPIDevice::new();
    expect_select_page(&mut mock, false);
    expect_dcmd_read(&mut mock, 0x90, &[0x00, 0x01, 0x23]);

    let mut client = LTC2949::new(mock);
    let current = client.read_current1().unwrap();
    assert_eq!(0x000123, current.raw());
    assert_f32_approx_eq(current.decode(), 0.00027645);
}

#[test]
fn read_current1_avg_reads_moving_average_register() {
    let mut mock = MockSPIDevice::new();
    expect_select_page(&mut mock, false);
    expect_dcmd_read(&mut mock, 0x9C, &[0x00, 0x04, 0x00]);

    let mut client = LTC2949::new(mock);
    let current = client.read_current1_avg().unwrap();
    assert_eq!(0x000400, current.raw());
    assert_f32_approx_eq(current.decode(), 0.0002432);
}

#[test]
fn read_current2_avg_decodes_24bit_signed_be() {
    let mut mock = MockSPIDevice::new();
    expect_select_page(&mut mock, false);
    expect_dcmd_read(&mut mock, 0xAC, &[0xFF, 0xFE, 0x00]);

    let mut client = LTC2949::new(mock);
    let current = client.read_current2_avg().unwrap();
    assert_eq!(-512, current.raw());
    assert_f32_approx_eq(current.decode(), -0.0001216);
}

#[test]
fn read_power1_decodes_power_or_voltage_value() {
    let mut mock = MockSPIDevice::new();
    expect_select_page(&mut mock, false);
    expect_dcmd_read(&mut mock, 0x93, &[0x00, 0x01, 0x00]);

    let mut client = LTC2949::new(mock);
    let result = client.read_power1().unwrap();
    assert_eq!(0x000100, result.raw());
    assert_f32_approx_eq(result.decode_voltage(), 0.012);
    assert_f32_approx_eq(result.decode_power(0.0001), 0.000014942208);
}

#[test]
fn read_power2_decodes_24bit_signed_be() {
    let mut mock = MockSPIDevice::new();
    expect_select_page(&mut mock, false);
    expect_dcmd_read(&mut mock, 0x99, &[0xFF, 0xFF, 0xFF]);

    let mut client = LTC2949::new(mock);
    let result = client.read_power2().unwrap();
    assert_eq!(-1, result.raw());
    assert_f32_approx_eq(result.decode_voltage(), -46.875e-6);
    assert_f32_approx_eq(result.decode_power(1.0), -5.8368e-12);
}

#[test]
fn read_bat_decodes_16bit_signed_be() {
    // 0x7FFF = 32767 (max positive).
    let mut mock = MockSPIDevice::new();
    expect_select_page(&mut mock, false);
    expect_dcmd_read(&mut mock, 0xA0, &[0x7F, 0xFF]);

    let mut client = LTC2949::new(mock);
    let bat = client.read_bat().unwrap();
    assert_eq!(32767, bat.raw());
    assert_f32_approx_eq(bat.decode(), 12.287625);
}

#[test]
fn read_temp_decodes_kelvin_and_celsius() {
    let mut mock = MockSPIDevice::new();
    expect_select_page(&mut mock, false);
    expect_dcmd_read(&mut mock, 0xA2, &[0x05, 0xD4]);

    let mut client = LTC2949::new(mock);
    let temp = client.read_temp().unwrap();
    assert_eq!(1492, temp.raw());
    assert_f32_approx_eq(temp.decode_kelvin(), 298.4);
    assert_f32_approx_eq(temp.decode_celsius(), 25.25);
}

#[test]
fn read_vcc_decodes_supply_voltage() {
    let mut mock = MockSPIDevice::new();
    expect_select_page(&mut mock, false);
    expect_dcmd_read(&mut mock, 0xA4, &[0x03, 0xE8]);

    let mut client = LTC2949::new(mock);
    let vcc = client.read_vcc().unwrap();
    assert_eq!(1000, vcc.raw());
    assert_f32_approx_eq(vcc.decode(), 2.26);
}

#[test]
fn read_slot1_returns_typed_voltage_or_temperature() {
    let mut mock = MockSPIDevice::new();
    expect_select_page(&mut mock, false);
    expect_dcmd_read(&mut mock, 0xA6, &[0x00, 0x7D]);

    let mut client = LTC2949::new(mock);
    let slot = client.read_slot1().unwrap();
    assert_eq!(125, slot.raw());
    assert_f32_approx_eq(slot.decode_voltage(), 0.046875);
    assert_f32_approx_eq(slot.decode_temperature(), 25.0);
}

#[test]
fn accumulated_result_types_decode_internal_clock_scaling() {
    let charge = AccumulatedCharge::from_raw(1_000_000);
    assert_eq!(1_000_000, charge.raw());
    assert_f64_approx_eq(charge.decode(), 377.887e-6);
    assert_f64_approx_eq(charge.decode_coulombs(100e-6), 3.77887);

    let energy = AccumulatedEnergy::from_raw(1_000_000);
    assert_eq!(1_000_000, energy.raw());
    assert_f64_approx_eq(energy.decode(), 2.32175e-3);
    assert_f64_approx_eq(energy.decode_joules(100e-6), 23.2175);

    let time = AccumulatedTime::from_raw(1_000);
    assert_eq!(1_000, time.raw());
    assert_f64_approx_eq(time.decode(), 0.397777);

    let slot = SlotValue::from_raw(-100);
    assert_eq!(-100, slot.raw());
    assert_f32_approx_eq(slot.decode_voltage(), -0.0375);
    assert_f32_approx_eq(slot.decode_temperature(), -20.0);
}

#[test]
fn read_charge1_decodes_48bit_signed_be() {
    // 48-bit value 0x0000_0000_0001 = 1.
    let mut mock = MockSPIDevice::new();
    expect_select_page(&mut mock, false);
    expect_dcmd_read(&mut mock, 0x00, &[0x00, 0x00, 0x00, 0x00, 0x00, 0x01]);

    let mut client = LTC2949::new(mock);
    assert_eq!(1, client.read_charge1().unwrap().raw());
}

#[test]
fn read_charge1_decodes_48bit_signed_negative() {
    // All-ones 48-bit → -1 after sign-extension to i64.
    let mut mock = MockSPIDevice::new();
    expect_select_page(&mut mock, false);
    expect_dcmd_read(&mut mock, 0x00, &[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]);

    let mut client = LTC2949::new(mock);
    assert_eq!(-1, client.read_charge1().unwrap().raw());
}

#[test]
fn read_charge3_decodes_64bit_signed() {
    let payload = [0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0];
    let mut mock = MockSPIDevice::new();
    expect_select_page(&mut mock, false);
    expect_dcmd_read(&mut mock, 0x24, &payload);

    let mut client = LTC2949::new(mock);
    let expected = i64::from_be_bytes(payload);
    assert_eq!(expected, client.read_charge3().unwrap().raw());
}

#[test]
fn read_time1_decodes_32bit_unsigned_be() {
    // 0xDEAD_BEEF.
    let mut mock = MockSPIDevice::new();
    expect_select_page(&mut mock, false);
    expect_dcmd_read(&mut mock, 0x0C, &[0xDE, 0xAD, 0xBE, 0xEF]);

    let mut client = LTC2949::new(mock);
    assert_eq!(0xDEAD_BEEF, client.read_time1().unwrap().raw());
}

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
    client.write_adcconf(AdcConfiguration::default().with_ntc1(true)).unwrap();
    let _ = client.read_status().unwrap();
}

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
    expect_write(&mut mock, dcmd_write_bytes(0xAA, &f24(10_000.0)));

    // 2) 9-byte burst writing NTC1A | NTC1B | NTC1C at p1.0xD0.
    let mut abc = vec![0u8; 9];
    abc[0..3].copy_from_slice(&f24(1.1382e-3));
    abc[3..6].copy_from_slice(&f24(2.3267e-4));
    abc[6..9].copy_from_slice(&f24(0.93243e-7));
    expect_write(&mut mock, dcmd_write_bytes(0xD0, &abc));

    let mut client: LTC2949<_, _> = LTC2949::new(mock);
    client.write_ntc_coefficients(Channel::One, &NTCLE203E_EXAMPLE).unwrap();
}

#[test]
fn write_ntc2_coefficients_targets_distinct_addresses() {
    let mut mock = MockSPIDevice::new();
    expect_select_page(&mut mock, true);

    // RREF2 lives at p1.0xAD, the NTC2A/B/C burst at p1.0xE0.
    expect_write(&mut mock, dcmd_write_bytes(0xAD, &f24(10_000.0)));
    let mut abc = vec![0u8; 9];
    abc[0..3].copy_from_slice(&f24(1.1382e-3));
    abc[3..6].copy_from_slice(&f24(2.3267e-4));
    abc[6..9].copy_from_slice(&f24(0.93243e-7));
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
    expect_write(&mut mock, dcmd_write_bytes(0xAA, &f24(10_000.0)));
    let mut abc = vec![0u8; 9];
    abc[0..3].copy_from_slice(&f24(1.1382e-3));
    abc[3..6].copy_from_slice(&f24(2.3267e-4));
    abc[6..9].copy_from_slice(&f24(0.93243e-7));
    expect_write(&mut mock, dcmd_write_bytes(0xD0, &abc));

    // NTC2 (no extra select_page).
    expect_write(&mut mock, dcmd_write_bytes(0xAD, &f24(10_000.0)));
    expect_write(&mut mock, dcmd_write_bytes(0xE0, &abc));

    let mut client: LTC2949<_, _> = LTC2949::new(mock);
    client.write_ntc_coefficients(Channel::One, &NTCLE203E_EXAMPLE).unwrap();
    client.write_ntc_coefficients(Channel::Two, &NTCLE203E_EXAMPLE).unwrap();
}

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
    burst[0..3].copy_from_slice(&f24(0.0039));
    burst[3..5].copy_from_slice(&f24_high(20.0));
    expect_write(&mut mock, dcmd_write_bytes(0xD9, &burst));

    // 3-byte write of RS1TC2 at p1.0x5C.
    expect_write(&mut mock, dcmd_write_bytes(0x5C, &f24(0.0)));

    let mut client: LTC2949<_, _> = LTC2949::new(mock);
    client.write_shunt_tc(Channel::One, &COPPER_SHUNT_25C).unwrap();
}

#[test]
fn write_shunt_tc_channel2_targets_distinct_addresses() {
    let mut mock = MockSPIDevice::new();
    expect_select_page(&mut mock, true);

    let mut burst = vec![0u8; 5];
    burst[0..3].copy_from_slice(&f24(0.0039));
    burst[3..5].copy_from_slice(&f24_high(20.0));
    expect_write(&mut mock, dcmd_write_bytes(0xE9, &burst)); // RS2TC + RS2T0
    expect_write(&mut mock, dcmd_write_bytes(0x7C, &f24(0.0))); // RS2TC2

    let mut client: LTC2949<_, _> = LTC2949::new(mock);
    client.write_shunt_tc(Channel::Two, &COPPER_SHUNT_25C).unwrap();
}

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

/// Encodes one FIFO sample as the device returns it: MSB, LSB, TAG.
fn fifo_sample_bytes(raw: i16, tag: u8) -> [u8; 3] {
    let [msb, lsb] = raw.to_be_bytes();
    [msb, lsb, tag]
}

#[test]
fn read_fifo_i1_drains_three_samples_in_one_burst() {
    // N = 3 ≤ 5, so a single DCMD read of 9 bytes from FIFOI1 (0xF7) returns all three.
    let mut data = Vec::new();
    data.extend_from_slice(&fifo_sample_bytes(258, 0x00)); // 0x0102, Ok
    data.extend_from_slice(&fifo_sample_bytes(772, 0x00)); // 0x0304, Ok
    data.extend_from_slice(&fifo_sample_bytes(-1, 0x00)); // 0xFFFF, Ok

    let mut mock = MockSPIDevice::new();
    expect_select_page(&mut mock, false);
    expect_dcmd_read(&mut mock, 0xF7, &data);

    let mut client: LTC2949<_, _> = LTC2949::new(mock);
    let samples = client.read_fifo_i1::<3>().unwrap();

    assert_eq!(3, samples.len());
    assert_eq!(258, samples[0].raw);
    assert_eq!(772, samples[1].raw);
    assert_eq!(-1, samples[2].raw);
    assert!(samples.iter().all(|s| s.tag == FifoTag::Ok));
}

#[test]
fn read_fifo_stops_at_non_ok_terminator_within_a_burst() {
    // N = 5 → one 15-byte burst. The 2nd sample is ReadOverrun (0x55); the drain keeps it
    // as the terminator and returns immediately, ignoring the rest of the burst.
    let mut data = Vec::new();
    data.extend_from_slice(&fifo_sample_bytes(100, 0x00)); // Ok
    data.extend_from_slice(&fifo_sample_bytes(0, 0x55)); // ReadOverrun — stop here
    data.extend_from_slice(&fifo_sample_bytes(0, 0x00)); // never inspected
    data.extend_from_slice(&fifo_sample_bytes(0, 0x00));
    data.extend_from_slice(&fifo_sample_bytes(0, 0x00));

    let mut mock = MockSPIDevice::new();
    expect_select_page(&mut mock, false);
    expect_dcmd_read(&mut mock, 0xF7, &data);

    let mut client: LTC2949<_, _> = LTC2949::new(mock);
    let samples = client.read_fifo_i1::<5>().unwrap();

    assert_eq!(2, samples.len());
    assert_eq!(FifoTag::Ok, samples[0].tag);
    assert_eq!(FifoTag::ReadOverrun, samples[1].tag);
}

#[test]
fn read_fifo_spans_two_bursts_for_more_than_five_samples() {
    // N = 7 → first burst 5 samples (15 bytes), second burst 2 samples (6 bytes), both from
    // the non-incrementing FIFOI2 (0xF8). The page is selected once and cached.
    let first: Vec<u8> = (0..5).flat_map(|i| fifo_sample_bytes(i as i16, 0x00)).collect();
    let second: Vec<u8> = (5..7).flat_map(|i| fifo_sample_bytes(i as i16, 0x00)).collect();

    let mut mock = MockSPIDevice::new();
    expect_select_page(&mut mock, false);
    expect_dcmd_read(&mut mock, 0xF8, &first);
    expect_dcmd_read(&mut mock, 0xF8, &second);

    let mut client: LTC2949<_, _> = LTC2949::new(mock);
    let samples = client.read_fifo_i2::<7>().unwrap();

    assert_eq!(7, samples.len());
    for (i, s) in samples.iter().enumerate() {
        assert_eq!(i as i16, s.raw);
    }
}

#[test]
fn dcmd_id_read_with_pecc15_encodes_0x9b() {
    assert_eq!(0x9B, u8::from(DcmdId::read(15)));
}

#[test]
fn dcmd_id_write_with_pecc15_encodes_0x5b() {
    assert_eq!(0x5B, u8::from(DcmdId::write(15)));
}

#[test]
fn dcmd_id_write_with_pecc1_matches_datasheet_example() {
    assert_eq!(0x45, u8::from(DcmdId::write(1)));
}

#[test]
fn dcmd_id_pecc_is_limited_to_four_bits() {
    assert_eq!(u8::from(DcmdId::read(0x0F)), u8::from(DcmdId::read(0xFF)));
}
