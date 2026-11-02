# Vortex V1 pin map

Every functional pin and ADC channel of the Vortex V1 motor controller board, generated from
[`src/hf_functional_pin_config_vortex_v1.hpp`](../src/hf_functional_pin_config_vortex_v1.hpp).

The MCU is an ESP32-C6. Besides its own GPIOs, the board routes control and status signals through a PCAL95555
I/O expander (I²C address `0x20`) and two GPIOs of the TMC9660 motor controller. Select it with
`HARDFOC_BOARD_VORTEX_V1`. The board HAL is documented at
[hf-hal-vortex-v1](https://hardfoc.github.io/hf-hal-vortex-v1/).

## GPIO

<!-- generated: pins-vortex-v1 -->
Source: `src/hf_functional_pin_config_vortex_v1.hpp`, `HF_FUNCTIONAL_GPIO_PIN_LIST` (34 rows).

| Function | Category | Chip | Pin | Active | Pull | Drive | Max (mA) |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `XTAL_32K_P` | core | MCU | 0 | high | none | push-pull | 0 |
| `XTAL_32K_N` | core | MCU | 1 | high | none | push-pull | 0 |
| `BOOT_SEL` | core | MCU | 9 | high | none | push-pull | 0 |
| `JTAG_USB_D_N` | core | MCU | 12 | high | none | push-pull | 0 |
| `JTAG_USB_D_P` | core | MCU | 13 | high | none | push-pull | 0 |
| `SPI2_MISO` | comm | MCU | 2 | high | up | push-pull | 40 |
| `SPI2_SCK` | comm | MCU | 6 | high | none | push-pull | 40 |
| `SPI2_MOSI` | comm | MCU | 7 | high | none | push-pull | 40 |
| `SPI2_CS_TMC9660` | comm | MCU | 18 | low | up | push-pull | 40 |
| `SPI2_CS_AS5047` | comm | MCU | 20 | low | up | push-pull | 40 |
| `UART_RXD` | comm | MCU | 4 | high | up | push-pull | 40 |
| `UART_TXD` | comm | MCU | 5 | high | none | push-pull | 40 |
| `TWAI_TX` | comm | MCU | 14 | high | up | push-pull | 40 |
| `TWAI_RX` | comm | MCU | 15 | high | up | push-pull | 40 |
| `I2C_SDA` | comm | MCU | 21 | high | up | push-pull | 40 |
| `I2C_SCL` | comm | MCU | 22 | high | up | push-pull | 40 |
| `I2C_PCAL95555_INT` | comm | MCU | 23 | low | up | push-pull | 40 |
| `WS2812_LED_DAT` | comm | MCU | 3 | high | none | push-pull | 40 |
| `EXT_GPIO_CS_1` | gpio | MCU | 19 | low | up | push-pull | 40 |
| `EXT_GPIO_CS_2` | gpio | MCU | 8 | low | up | push-pull | 40 |
| `PCAL_TMC_FAULT_STATUS` | gpio | PCAL95555 | 3 | low | none | push-pull | 25 |
| `PCAL_TMC_DRV_EN` | gpio | PCAL95555 | 4 | high | none | push-pull | 25 |
| `PCAL_TMC_RST_CTRL` | gpio | PCAL95555 | 5 | high | none | push-pull | 25 |
| `PCAL_PWR_GOOD` | gpio | PCAL95555 | 6 | high | none | push-pull | 25 |
| `PCAL_CAN_HS_STB_OP` | gpio | PCAL95555 | 10 | high | none | push-pull | 25 |
| `PCAL_IMU_BOOT` | gpio | PCAL95555 | 11 | low | none | push-pull | 25 |
| `PCAL_IMU_INT` | gpio | PCAL95555 | 12 | low | none | push-pull | 25 |
| `PCAL_IMU_RST` | gpio | PCAL95555 | 15 | low | none | push-pull | 25 |
| `PCAL_TMC_SPI_COMM_EN` | gpio | PCAL95555 | 13 | low | none | push-pull | 25 |
| `PCAL_TMC_WAKE_CTRL` | gpio | PCAL95555 | 14 | low | none | push-pull | 25 |
| `PCAL_TMC_GPIO17_EXP_IN` | gpio | PCAL95555 | 1 | high | none | push-pull | 25 |
| `PCAL_TMC_GPIO18_EXP_IN` | gpio | PCAL95555 | 2 | high | none | push-pull | 25 |
| `TMC_GPIO17` | gpio | TMC9660 | 17 | high | none | push-pull | 25 |
| `TMC_GPIO18` | gpio | TMC9660 | 18 | high | none | push-pull | 25 |
<!-- /generated -->

**Pin** is the GPIO number on the MCU, the port bit (0 to 15) on the PCAL95555 and the GPIO number on the TMC9660.
**Active** low means the row is `PIN_LOGIC_INVERTED`.

!!! note "One net, two rows"
    TMC9660 GPIO17 and GPIO18 are wired to PCAL95555 inputs 1 and 2. `PCAL_TMC_GPIO17_EXP_IN` and
    `PCAL_TMC_GPIO18_EXP_IN` sample them at the expander; `TMC_GPIO17` and `TMC_GPIO18` configure and drive them
    inside the TMC9660. Both views see the same nets.

## ADC

<!-- generated: adc-vortex-v1 -->
Source: `src/hf_functional_pin_config_vortex_v1.hpp`, `HF_FUNCTIONAL_ADC_CHANNEL_LIST` (15 rows).

| Function | Chip | Channel | Bits | Full scale (mV) | Divider | Description |
| --- | --- | --- | --- | --- | --- | --- |
| `TMC9660_AIN0` | TMC9660 | 0 | 16 | 3300 | 1.0 | TMC9660 ADC Input 0 (Reserved) |
| `TMC9660_AIN1` | TMC9660 | 1 | 16 | 3300 | 1.0 | TMC9660 ADC Input 1 (Reserved) |
| `TMC9660_AIN2` | TMC9660 | 2 | 16 | 3300 | 1.0 | TMC9660 ADC Input 2 (Reserved) |
| `TMC9660_AIN3` | TMC9660 | 3 | 16 | 3300 | 1.0 | TMC9660 ADC Input 3 (Temperature Sensor) |
| `TMC9660_CURRENT_I0` | TMC9660 | 10 | 16 | 3300 | 1.0 | TMC9660 Current Sense I0 |
| `TMC9660_CURRENT_I1` | TMC9660 | 11 | 16 | 3300 | 1.0 | TMC9660 Current Sense I1 |
| `TMC9660_CURRENT_I2` | TMC9660 | 12 | 16 | 3300 | 1.0 | TMC9660 Current Sense I2 |
| `TMC9660_CURRENT_I3` | TMC9660 | 13 | 16 | 3300 | 1.0 | TMC9660 Current Sense I3 |
| `TMC9660_SUPPLY_VOLTAGE` | TMC9660 | 20 | 16 | 3300 | 1.0 | TMC9660 Supply Voltage |
| `TMC9660_DRIVER_VOLTAGE` | TMC9660 | 21 | 16 | 3300 | 1.0 | TMC9660 Driver Voltage |
| `TMC9660_CHIP_TEMPERATURE` | TMC9660 | 30 | 16 | 3300 | 1.0 | TMC9660 Chip Temperature |
| `TMC9660_EXTERNAL_TEMPERATURE` | TMC9660 | 31 | 16 | 3300 | 1.0 | TMC9660 External Temperature |
| `TMC9660_MOTOR_CURRENT` | TMC9660 | 40 | 16 | 3300 | 1.0 | TMC9660 Motor Current |
| `TMC9660_MOTOR_VELOCITY` | TMC9660 | 41 | 16 | 3300 | 1.0 | TMC9660 Motor Velocity |
| `TMC9660_MOTOR_POSITION` | TMC9660 | 42 | 16 | 3300 | 1.0 | TMC9660 Motor Position |
<!-- /generated -->

The ESP32-C6 ADC channels are present in the header as comments and are not part of the map.
