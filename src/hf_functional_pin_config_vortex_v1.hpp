#ifndef HF_FUNCTIONAL_PIN_CONFIG_VORTEX_V1_HPP
#define HF_FUNCTIONAL_PIN_CONFIG_VORTEX_V1_HPP

/**
 * @file hf_functional_pin_config_vortex_v1.hpp
 * @brief Functional pin mapping configuration for HardFOC Vortex V1 board.
 *
 * This file provides the functional-to-physical pin mapping for the Vortex V1 board.
 * To port to a new microcontroller or board, create a new mapping file like this one.
 *
 * The component handlers (AdcData, GpioData, etc.) use the functional identifiers
 * defined here, making the higher-level code completely portable.
 */

#include "../component-handler/hf_ext_pins_enum.hpp"
#include <cstdint>

//==============================================================================
// FUNCTIONAL PIN IDENTIFIERS (EXACTLY MATCHING USER'S PINOUT)
//==============================================================================

enum class HfFunctionalGpioPin : uint8_t {
    // ESP32C6
    XTAL_32K_P = 0,      // IO0
    XTAL_32K_N,          // IO1
    SPI0_MISO,           // IO2
    WS2812_LED_DAT,      // IO3
    UART_RXD,            // IO4
    UART_TXD,            // IO5
    SPI0_SCK,            // IO6
    SPI0_MOSI,           // IO7
    EXT_GPIO_CS_2,       // IO8
    BOOT_SEL,            // IO9
    JTAG_USB_D_N,        // IO12
    JTAG_USB_D_P,        // IO13
    TWAI_TX,             // IO14
    TWAI_RX,             // IO15
    SPI0_CS_TMC9660,     // IO18 (active low)
    EXT_GPIO_CS_1,       // IO19 (active low)
    SPI0_CS_AS5047,      // IO20 (active low)
    I2C_SDA,             // IO21
    I2C_SCL,             // IO22
    I2C_PCAL95555_INT,   // IO23 (active low)

    // PCAL95555 PORT 0
    PCAL_GPIO17,         // IO0
    PCAL_GPIO18,         // IO1
    PCAL_FAULT_STATUS,   // IO3 (active low)
    PCAL_DRV_EN,         // IO4
    PCAL_RST_CTRL,       // IO5
    PCAL_PWR_GOOD,       // IO6
    // IO2, IO7 tied to GND, treat as input

    // PCAL95555 PORT 1
    PCAL_CAN_HS_STB_OP,  // IO2
    PCAL_IMU_BOOT,       // IO3 (active low)
    PCAL_IMU_INT,        // IO4 (active low)
    PCAL_SPI_COMM_EN,    // IO5 (active low)
    PCAL_WAKE_CTRL,      // IO6 (active low)
    PCAL_IMU_RST,        // IO7 (active low)
    // IO0, IO1 tied to GND, treat as input

    HF_FUNCTIONAL_GPIO_COUNT // Always last
};

enum class HfFunctionalAdcChannel : uint8_t {
    // TMC9660
    BOARD_TEMP_SENSOR = 0, // AIN3
    
    HF_FUNCTIONAL_ADC_COUNT // Always last
};

//==============================================================================
// CHIP TYPE ENUMS
//==============================================================================

enum class HfGpioChipType : uint8_t {
    ESP32_INTERNAL = 0,
    PCAL95555_EXPANDER,
    TMC9660_CONTROLLER
};

enum class HfAdcChipType : uint8_t {
    TMC9660_CONTROLLER = 0
};

//==============================================================================
// MAPPING STRUCTS
//==============================================================================

struct HfGpioMapping {
    HfFunctionalGpioPin functional_pin;
    HfGpioChipType chip_type;
    uint8_t physical_pin;
    bool is_inverted;
    bool has_pullup;
    uint32_t max_current_ma;
    const char* description;
};

struct HfAdcMapping {
    HfFunctionalAdcChannel functional_channel;
    HfAdcChipType chip_type;
    uint8_t physical_channel;
    uint8_t resolution_bits;
    uint32_t max_voltage_mv;
    float voltage_divider;
    const char* description;
};

//==============================================================================
// PIN MAPPING TABLES (EXACTLY MATCHING USER'S PINOUT, EXPLICIT)
//==============================================================================

static constexpr HfGpioMapping HF_GPIO_MAPPING[] = {
    // ESP32C6
    { HfFunctionalGpioPin::XTAL_32K_P, HfGpioChipType::ESP32_INTERNAL, 0, false, false, 0, "XTAL 32K P" },
    { HfFunctionalGpioPin::XTAL_32K_N, HfGpioChipType::ESP32_INTERNAL, 1, false, false, 0, "XTAL 32K N" },
    { HfFunctionalGpioPin::SPI0_MISO, HfGpioChipType::ESP32_INTERNAL, 2, false, true, 40, "SPI0 MISO" },
    { HfFunctionalGpioPin::WS2812_LED_DAT, HfGpioChipType::ESP32_INTERNAL, 3, false, false, 40, "WS2812 LED Data" },
    { HfFunctionalGpioPin::UART_RXD, HfGpioChipType::ESP32_INTERNAL, 4, false, true, 40, "UART RXD" },
    { HfFunctionalGpioPin::UART_TXD, HfGpioChipType::ESP32_INTERNAL, 5, false, false, 40, "UART TXD" },
    { HfFunctionalGpioPin::SPI0_SCK, HfGpioChipType::ESP32_INTERNAL, 6, false, false, 40, "SPI0 SCK" },
    { HfFunctionalGpioPin::SPI0_MOSI, HfGpioChipType::ESP32_INTERNAL, 7, false, false, 40, "SPI0 MOSI" },
    { HfFunctionalGpioPin::EXT_GPIO_CS_2, HfGpioChipType::ESP32_INTERNAL, 8, false, false, 40, "EXT GPIO CS 2" },
    { HfFunctionalGpioPin::BOOT_SEL, HfGpioChipType::ESP32_INTERNAL, 9, false, false, 0, "BOOT SEL" },
    { HfFunctionalGpioPin::JTAG_USB_D_N, HfGpioChipType::ESP32_INTERNAL, 12, false, false, 0, "JTAG USB D N" },
    { HfFunctionalGpioPin::JTAG_USB_D_P, HfGpioChipType::ESP32_INTERNAL, 13, false, false, 0, "JTAG USB D P" },
    { HfFunctionalGpioPin::TWAI_TX, HfGpioChipType::ESP32_INTERNAL, 14, false, true, 40, "TWAI TX (CAN)" },
    { HfFunctionalGpioPin::TWAI_RX, HfGpioChipType::ESP32_INTERNAL, 15, false, true, 40, "TWAI RX (CAN)" },
    { HfFunctionalGpioPin::SPI0_CS_TMC9660, HfGpioChipType::ESP32_INTERNAL, 18, true, true, 40, "SPI0 CS TMC9660 (active low)" },
    { HfFunctionalGpioPin::EXT_GPIO_CS_1, HfGpioChipType::ESP32_INTERNAL, 19, true, true, 40, "EXT GPIO CS 1 (active low)" },
    { HfFunctionalGpioPin::SPI0_CS_AS5047, HfGpioChipType::ESP32_INTERNAL, 20, true, true, 40, "SPI0 CS AS5047 (active low)" },
    { HfFunctionalGpioPin::I2C_SDA, HfGpioChipType::ESP32_INTERNAL, 21, false, true, 40, "I2C SDA" },
    { HfFunctionalGpioPin::I2C_SCL, HfGpioChipType::ESP32_INTERNAL, 22, false, true, 40, "I2C SCL" },
    { HfFunctionalGpioPin::I2C_PCAL95555_INT, HfGpioChipType::ESP32_INTERNAL, 23, true, true, 40, "I2C PCAL95555 INT (active low)" },

    // PCAL95555 PORT 0
    { HfFunctionalGpioPin::PCAL_GPIO17, HfGpioChipType::PCAL95555_EXPANDER, 0, false, false, 25, "TMC9660 GPIO17" },
    { HfFunctionalGpioPin::PCAL_GPIO18, HfGpioChipType::PCAL95555_EXPANDER, 1, false, false, 25, "TMC9660 GPIO18" },
    { HfFunctionalGpioPin::PCAL_FAULT_STATUS, HfGpioChipType::PCAL95555_EXPANDER, 3, true, false, 25, "TMC9660 FAULT STATUS (active low)" },
    { HfFunctionalGpioPin::PCAL_DRV_EN, HfGpioChipType::PCAL95555_EXPANDER, 4, false, false, 25, "TMC9660 DRV EN" },
    { HfFunctionalGpioPin::PCAL_RST_CTRL, HfGpioChipType::PCAL95555_EXPANDER, 5, false, false, 25, "TMC9660 RST CTRL" },
    { HfFunctionalGpioPin::PCAL_PWR_GOOD, HfGpioChipType::PCAL95555_EXPANDER, 6, false, false, 25, "SWR 3V3 PG FLAG" },
    // IO2, IO7 tied to GND, treat as input (not mapped)

    // PCAL95555 PORT 1
    { HfFunctionalGpioPin::PCAL_CAN_HS_STB_OP, HfGpioChipType::PCAL95555_EXPANDER, 10, false, false, 25, "CAN HS STB OP" },
    { HfFunctionalGpioPin::PCAL_IMU_BOOT, HfGpioChipType::PCAL95555_EXPANDER, 11, true, false, 25, "IMU BOOT (active low)" },
    { HfFunctionalGpioPin::PCAL_IMU_INT, HfGpioChipType::PCAL95555_EXPANDER, 12, true, false, 25, "IMU INT (active low)" },
    { HfFunctionalGpioPin::PCAL_SPI_COMM_EN, HfGpioChipType::PCAL95555_EXPANDER, 13, true, false, 25, "TMC9660 SPI COMM EN (active low)" },
    { HfFunctionalGpioPin::PCAL_WAKE_CTRL, HfGpioChipType::PCAL95555_EXPANDER, 14, true, false, 25, "TMC WAKE CTRL (active low)" },
    { HfFunctionalGpioPin::PCAL_IMU_RST, HfGpioChipType::PCAL95555_EXPANDER, 15, true, false, 25, "IMU RST (active low)" },
    // IO8, IO9 tied to GND, treat as input (not mapped)
};

static constexpr HfAdcMapping HF_ADC_MAPPING[] = {
    // TMC9660
    { HfFunctionalAdcChannel::BOARD_TEMP_SENSOR, HfAdcChipType::TMC9660_CONTROLLER, 3, 12, 3300, 1.0f, "Board Temp Sensor (AIN3)" }
};

//==============================================================================
// MAPPING ACCESS FUNCTIONS (LOOKUP BY VALUE, NOT INDEX)
//==============================================================================

static inline const HfGpioMapping* GetGpioMapping(HfFunctionalGpioPin functional_pin) {
    for (const auto& mapping : HF_GPIO_MAPPING) {
        if (mapping.functional_pin == functional_pin) return &mapping;
    }
    return nullptr;
}

static inline const HfAdcMapping* GetAdcMapping(HfFunctionalAdcChannel functional_channel) {
    for (const auto& mapping : HF_ADC_MAPPING) {
        if (mapping.functional_channel == functional_channel) return &mapping;
    }
    return nullptr;
}

static inline bool IsGpioPinAvailable(HfFunctionalGpioPin functional_pin) {
    return GetGpioMapping(functional_pin) != nullptr;
}

static inline bool IsAdcChannelAvailable(HfFunctionalAdcChannel functional_channel) {
    return GetAdcMapping(functional_channel) != nullptr;
}

#endif // HF_FUNCTIONAL_PIN_CONFIG_VORTEX_V1_HPP 