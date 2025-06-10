# HF-PINCFG

HF-PINCFG provides a simple way to configure the GPIO layout for the HardFOC controller on an ESP32‑C6.
It exposes pin assignments and an initialization function that can be used by your project right away.

## Features

- Predefined GPIO layout for the ESP32-C6 platform
- Helper function for configuring pins with the ESP-IDF `gpio_config` API
- Central location to keep track of SPI, UART, I2C and other connections
- Source code is annotated with Doxygen comments for easy reference

## Pin Mapping

The header `include/hf_gpio_config.hpp` lists every pin used on the ESP32‑C6. The mappings are grouped by peripheral for quick reference.

### SPI0
- **MISO** – `GPIO_NUM_2` (data from device to controller)
- **MOSI** – `GPIO_NUM_7` (data from controller to device)
- **CLK** – `GPIO_NUM_6`
- **CS (TMC9660)** – `GPIO_NUM_18`
- **CS (AS5047P)** – `GPIO_NUM_20`
- **CS (Ext1)** – `GPIO_NUM_19`
- **CS (Ext2)** – `GPIO_NUM_8`

### UART
- **RXD** – `GPIO_NUM_4`
- **TXD** – `GPIO_NUM_5`

### Debug UART
- **TX** – `GPIO_NUM_0`
- **RX** – `GPIO_NUM_1`

### I2C
- **SDA** – `GPIO_NUM_22`
- **SCL** – `GPIO_NUM_23`

### CAN (TWAI)
- **TX** – `GPIO_NUM_19`
- **RX** – `GPIO_NUM_15`

### USB‑JTAG
- **D+** – `GPIO_NUM_13`
- **D-** – `GPIO_NUM_12`

### LED
- **WS2812** – `GPIO_NUM_3`

See the header file for detailed definitions.

## Connecting External Peripherals

1. **TMC9660 Driver** – Connect the driver chip to the SPI0 bus. The chip select line should be connected to GPIO 18.
2. **AS5047P Encoder** – This magnetic encoder is also on SPI0 with chip select GPIO 20. Ensure MISO, MOSI and CLK are connected as per the table above.
3. **External SPI devices** – Additional devices can use chip select pins 19 and 8.
4. **UART** – Primary UART uses GPIO 4 (RX) and 5 (TX). A separate debug UART is available on GPIO 0 (TX) and 1 (RX).
5. **I2C Sensors** – SDA is mapped to GPIO 22 and SCL to GPIO 23. Attach any I2C sensors here and enable pull‑ups if required.
6. **TWAI (CAN)** – Transmit on GPIO 19 and receive on GPIO 15 when using the ESP32 TWAI driver.
7. **USB‑JTAG** – D+ should be connected to GPIO 13 and D‑ to GPIO 12 for USB or JTAG use.
8. **WS2812 LED** – A single addressable LED can be driven from GPIO 3.

## Usage

Include the header and call `init_mcu_pinconfig()` early in your application:

```cpp
#include "hf_gpio_config.hpp"

void app_main() {
    init_mcu_pinconfig();
    // rest of your initialization
}
```

This will set up the pins as defined above, ensuring peripherals are ready to use.

The header file is thoroughly documented with Doxygen comments so you can
generate API reference documentation with your favorite tool.

## License

This project is distributed under the terms of the [GNU GPLv3](LICENSE).
