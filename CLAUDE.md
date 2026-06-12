# LTC681x and LTC2949 rust drivers

## Main repo where driver is used
- `../BMS`: BMS workspace firmware based on rp pico: Support a single ltc6813 for one battery stack. isoSPI daisy chain with LTC6812 and a parallel LTC2949 connected through a LTC6820.
- `../BMS/rp-pico/`: Crate in the BMS workspace where peripherals are defined and BMS tasks are constructed.

## Data sheets for LTC chips
- `../Datasheets`: Datasheets for LTC6813, LTC6812 and LTC2949