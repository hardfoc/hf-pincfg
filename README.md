# HF-PINCFG

HF-PINCFG defines the **hardware-agnostic functional pin configuration** for HardFOC-based boards. It abstracts what each pin does functionally (SPI, UART, motor control, sensors) independent of the underlying microcontroller or physical pin assignments.

## Supported Boards

Currently supported HardFOC board variants:
- **Vortex V1** - ESP32-C6 based controller board

## Features

- **Hardware Abstraction**: Defines WHAT pins do functionally, not WHERE they are physically
- **HardFOC Standardization**: Consistent functional pin definitions across all HardFOC board variants
- **Portable Firmware**: Write code once, run on any HardFOC board variant
- **Functional Identifiers**: Use semantic names like `MOTOR_SPI_CS` instead of `GPIO_18`
- **Board-Specific Mapping**: Each board variant maps functional pins to its physical implementation
- **Zero Runtime Cost**: All abstraction resolved at compile time

## Pin Mapping (Vortex V1 Board)

The following pin assignments are specific to the **HardFOC Vortex V1** board (ESP32-C6 based). The functional pin configuration is defined in `hf_functional_pin_config_vortex_v1.hpp`.

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

## Board Selection

The library automatically selects the appropriate pin configuration based on compile-time defines:

```cpp
// For Vortex V1 board (default)
#define HARDFOC_BOARD_VORTEX_V1

// Future board variants will be added like:
// #define HARDFOC_BOARD_VORTEX_V2
// #define HARDFOC_BOARD_NEXUS_V1
```

## Architecture

HF-PinCfg provides **functional abstraction** through a two-layer system:

### Layer 1: Functional Definition (Hardware Agnostic)
- **`hf_functional_pin_config.hpp`** - Defines WHAT each pin does functionally
- Provides semantic identifiers like `MOTOR_DRIVER_SPI_CS`, `ENCODER_SPI_MISO`
- Board selection logic chooses appropriate implementation

### Layer 2: Physical Implementation (Board Specific)  
- **`hf_functional_pin_config_vortex_v1.hpp`** - Maps functional pins to physical GPIO pins on Vortex V1
- **Future implementations** - Each HardFOC board variant maps the same functional pins to its hardware

### Result: Write Once, Run Anywhere
Your firmware uses functional identifiers and automatically works on any supported HardFOC board variant.

## Connecting External Peripherals (Vortex V1)

1. **TMC9660 Driver** – Connect the driver chip to the SPI0 bus. The chip select line should be connected to GPIO 18.
2. **AS5047P Encoder** – This magnetic encoder is also on SPI0 with chip select GPIO 20. Ensure MISO, MOSI and CLK are connected as per the table above.
3. **External SPI devices** – Additional devices can use chip select pins 19 and 8.
4. **UART** – Primary UART uses GPIO 4 (RX) and 5 (TX). A separate debug UART is available on GPIO 0 (TX) and 1 (RX).
5. **I2C Sensors** – SDA is mapped to GPIO 22 and SCL to GPIO 23. Attach any I2C sensors here and enable pull‑ups if required.
6. **TWAI (CAN)** – Transmit on GPIO 19 and receive on GPIO 15 when using the ESP32 TWAI driver.
7. **USB‑JTAG** – D+ should be connected to GPIO 13 and D‑ to GPIO 12 for USB or JTAG use.
8. **WS2812 LED** – A single addressable LED can be driven from GPIO 3.

## Usage

Include the header and call `HardFOC::init_platform_hardware()` early in your application:

```cpp
#include "hf_platform_config.hpp"

void app_main() {
    HardFOC::init_platform_hardware();
    // rest of your initialization
}
```

This will set up the pins as defined above, ensuring peripherals are ready to use.

The header file is thoroughly documented with Doxygen comments so you can
generate API reference documentation with your favorite tool.

## License

This project is distributed under the terms of the [GNU GPLv3](LICENSE).
