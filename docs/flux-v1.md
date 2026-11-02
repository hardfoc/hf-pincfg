# Flux V1 pin map

Every functional pin and ADC channel of the Flux V1 valve controller board, generated from
[`src/hf_functional_pin_config_flux_v1.hpp`](../src/hf_functional_pin_config_flux_v1.hpp).

The MCU is an ESP32-S3. SPI2 is shared by the MAX22200 octal solenoid driver and the TLE92466ED six-channel
low-side driver; SPI3 is dedicated to the ADS7952 12-channel ADC. Select it with `HARDFOC_BOARD_FLUX_V1`. The board
HAL is documented at [hf-hal-flux-v1](https://hardfoc.github.io/hf-hal-flux-v1/).

## GPIO

<!-- generated: pins-flux-v1 -->
Source: `src/hf_functional_pin_config_flux_v1.hpp`, `HF_FUNCTIONAL_GPIO_PIN_LIST` (27 rows).

| Function | Category | Chip | Pin | Active | Pull | Drive | Max (mA) |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `CORE_BOOT` | core | MCU | 0 | high | none | push-pull | 0 |
| `USB_D_N` | core | MCU | 19 | high | none | push-pull | 0 |
| `USB_D_P` | core | MCU | 20 | high | none | push-pull | 0 |
| `SPI2_MISO` | comm | MCU | 35 | high | up | push-pull | 40 |
| `SPI2_SCK` | comm | MCU | 36 | high | none | push-pull | 40 |
| `SPI2_MOSI` | comm | MCU | 37 | high | none | push-pull | 40 |
| `SPI2_CS_MAX22200` | comm | MCU | 38 | low | up | push-pull | 40 |
| `SPI2_CS_TLE92466` | comm | MCU | 4 | low | up | push-pull | 40 |
| `TWAI_TX` | comm | MCU | 17 | high | up | push-pull | 40 |
| `TWAI_RX` | comm | MCU | 18 | high | up | push-pull | 40 |
| `UART_FDO2_TX` | comm | MCU | 47 | high | up | push-pull | 40 |
| `UART_FDO2_RX` | comm | MCU | 48 | high | up | push-pull | 40 |
| `WS2812_LED_DAT` | comm | MCU | 3 | high | none | push-pull | 40 |
| `MAX22200_ENABLE` | gpio | MCU | 2 | high | none | push-pull | 40 |
| `MAX22200_CMD` | gpio | MCU | 39 | high | none | push-pull | 40 |
| `MAX22200_FAULT_N` | gpio | MCU | 42 | low | up | push-pull | 40 |
| `MAX22200_TRIGA` | gpio | MCU | 40 | high | none | push-pull | 40 |
| `MAX22200_TRIGB` | gpio | MCU | 41 | high | none | push-pull | 40 |
| `TLE92466_EN` | gpio | MCU | 5 | high | none | push-pull | 40 |
| `TLE92466_RESN` | gpio | MCU | 6 | low | none | push-pull | 40 |
| `TLE92466_DRV0` | gpio | MCU | 7 | high | none | push-pull | 40 |
| `TLE92466_DRV1` | gpio | MCU | 15 | high | none | push-pull | 40 |
| `TLE92466_FAULT_N` | gpio | MCU | 16 | low | up | push-pull | 40 |
| `SPI3_CS_ADS7952_0` | comm | MCU | 10 | low | up | push-pull | 40 |
| `SPI3_MOSI` | comm | MCU | 11 | high | none | push-pull | 40 |
| `SPI3_SCK` | comm | MCU | 12 | high | none | push-pull | 40 |
| `SPI3_MISO` | comm | MCU | 13 | high | up | push-pull | 40 |
<!-- /generated -->

**Pin** is the ESP32-S3 GPIO number. **Active** low means the row is `PIN_LOGIC_INVERTED`. Every row is on the MCU:
the MAX22200 and TLE92466ED chip types exist in `HfGpioChipType` for their own outputs but carry no rows yet.

## ADC

<!-- generated: adc-flux-v1 -->
Source: `src/hf_functional_pin_config_flux_v1.hpp`, `HF_FUNCTIONAL_ADC_CHANNEL_LIST` (14 rows).

| Function | Chip | Channel | Bits | Full scale (mV) | Divider | Description |
| --- | --- | --- | --- | --- | --- | --- |
| `ESP32_NTC_THERMISTOR` | MCU | 7 | 12 | 3300 | 1.0 | NTC Thermistor Temperature Input |
| `ESP32_VBUS_MONITOR` | MCU | 8 | 12 | 3300 | 11.0 | Bus Voltage Monitor (with divider) |
| `ADS7952_0_CH0` | ADS7952 | 0 | 12 | 2500 | 1.0 | ADS7952 Dev0 CH0 |
| `ADS7952_0_CH1` | ADS7952 | 1 | 12 | 2500 | 1.0 | ADS7952 Dev0 CH1 |
| `ADS7952_0_CH2` | ADS7952 | 2 | 12 | 2500 | 1.0 | ADS7952 Dev0 CH2 |
| `ADS7952_0_CH3` | ADS7952 | 3 | 12 | 2500 | 1.0 | ADS7952 Dev0 CH3 |
| `ADS7952_0_CH4` | ADS7952 | 4 | 12 | 2500 | 1.0 | ADS7952 Dev0 CH4 |
| `ADS7952_0_CH5` | ADS7952 | 5 | 12 | 2500 | 1.0 | ADS7952 Dev0 CH5 |
| `ADS7952_0_CH6` | ADS7952 | 6 | 12 | 2500 | 1.0 | ADS7952 Dev0 CH6 |
| `ADS7952_0_CH7` | ADS7952 | 7 | 12 | 2500 | 1.0 | ADS7952 Dev0 CH7 |
| `ADS7952_0_CH8` | ADS7952 | 8 | 12 | 2500 | 1.0 | ADS7952 Dev0 CH8 |
| `ADS7952_0_CH9` | ADS7952 | 9 | 12 | 2500 | 1.0 | ADS7952 Dev0 CH9 |
| `ADS7952_0_CH10` | ADS7952 | 10 | 12 | 2500 | 1.0 | ADS7952 Dev0 CH10 |
| `ADS7952_0_CH11` | ADS7952 | 11 | 12 | 2500 | 1.0 | ADS7952 Dev0 CH11 |
<!-- /generated -->

`ADC_VBUS_MONITOR` sits behind an 11:1 divider: multiply the voltage at the pin by `voltage_divider` to get the
bus voltage. The ADS7952 runs from a 2.5 V reference.
