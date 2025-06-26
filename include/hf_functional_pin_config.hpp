#ifndef HF_FUNCTIONAL_PIN_CONFIG_HPP
#define HF_FUNCTIONAL_PIN_CONFIG_HPP

/**
 * @file hf_functional_pin_config.hpp
 * @brief Functional pin mapping configuration for hardware abstraction.
 * 
 * This file provides the functional-to-physical pin mapping that makes the 
 * HardFOC system truly hardware-agnostic. To port to a new microcontroller,
 * only this file needs to be modified along with the platform-specific drivers.
 * 
 * The component handlers (AdcData, GpioData, etc.) use the functional identifiers
 * defined here, making the higher-level code completely portable.
 */

#include "hf_gpio_config.hpp"
#include "../component-handler/hf_ext_pins_enum.hpp"
#include <cstdint>

//==============================================================================
// FUNCTIONAL PIN IDENTIFIERS
//==============================================================================

/**
 * @brief Functional GPIO pin identifiers.
 * 
 * These identifiers describe WHAT the pin does, not WHERE it is physically located.
 * Application code should ONLY use these functional identifiers.
 */
enum class HfFunctionalGpioPin : uint8_t {
    // Motor control pins
    MOTOR_ENABLE = 0,
    MOTOR_BRAKE,
    MOTOR_DIRECTION,
    MOTOR_FAULT_STATUS,
    MOTOR_RESET,
    
    // System status LEDs
    LED_STATUS_OK,
    LED_STATUS_ERROR,
    LED_STATUS_COMM,
    LED_STATUS_POWER,
    
    // Communication interface pins
    COMM_CAN_TX,
    COMM_CAN_RX,
    COMM_I2C_SDA,
    COMM_I2C_SCL,
    COMM_DEBUG_TX,
    COMM_DEBUG_RX,
    
    // SPI interface pins
    SPI_MOTOR_CONTROLLER_CS,
    SPI_ENCODER_CS,
    SPI_EXTERNAL_CS,
    SPI_MISO,
    SPI_MOSI,
    SPI_CLK,
    
    // User interface pins
    USER_OUTPUT_1,
    USER_OUTPUT_2,
    USER_INPUT_1,
    USER_INPUT_2,
    
    // External device control
    EXTERNAL_RELAY_1,
    EXTERNAL_RELAY_2,
    
    // System control
    SYSTEM_POWER_ENABLE,
    SYSTEM_RESET,
    
    // Special purpose
    WS2812_DATA,
    
    HF_FUNCTIONAL_GPIO_COUNT
};

/**
 * @brief Functional ADC channel identifiers.
 * 
 * These identifiers describe WHAT the channel measures, not WHERE it is physically located.
 * Application code should ONLY use these functional identifiers.
 */
enum class HfFunctionalAdcChannel : uint8_t {
    // Motor current sensing
    MOTOR_CURRENT_PHASE_A = 0,
    MOTOR_CURRENT_PHASE_B,
    MOTOR_CURRENT_PHASE_C,
    
    // Motor voltage monitoring
    MOTOR_VOLTAGE_BUS,
    
    // System power monitoring
    SYSTEM_VOLTAGE_3V3,
    SYSTEM_VOLTAGE_5V,
    SYSTEM_VOLTAGE_12V,
    
    // Temperature sensing
    SYSTEM_TEMPERATURE_AMBIENT,
    MOTOR_TEMPERATURE,
    DRIVER_TEMPERATURE,
    
    // User analog inputs
    USER_ANALOG_INPUT_1,
    USER_ANALOG_INPUT_2,
    
    // System reference
    SYSTEM_VREF_INTERNAL,
    
    HF_FUNCTIONAL_ADC_COUNT
};

//==============================================================================
// PLATFORM-SPECIFIC MAPPING TABLES
//==============================================================================

/**
 * @brief GPIO chip types for hardware mapping.
 */
enum class HfGpioChipType : uint8_t {
    ESP32_INTERNAL = 0,
    PCAL95555_EXPANDER,
    TMC9660_CONTROLLER,
    EXTERNAL_CHIP
};

/**
 * @brief ADC chip types for hardware mapping.
 */
enum class HfAdcChipType : uint8_t {
    ESP32_INTERNAL = 0,
    TMC9660_CONTROLLER,
    EXTERNAL_CHIP
};

/**
 * @brief GPIO hardware mapping structure.
 */
struct HfGpioMapping {
    HfGpioChipType chip_type;
    uint8_t physical_pin;
    bool is_inverted;
    bool has_pullup;
    uint32_t max_current_ma;
    const char* description;
};

/**
 * @brief ADC hardware mapping structure.
 */
struct HfAdcMapping {
    HfAdcChipType chip_type;
    uint8_t physical_channel;
    uint8_t resolution_bits;
    uint32_t max_voltage_mv;
    float voltage_divider;
    const char* description;
};

//==============================================================================
// ESP32-C6 + TMC9660 + PCAL95555 PLATFORM MAPPING
//==============================================================================

/**
 * @brief Functional GPIO to physical pin mapping table.
 * 
 * This table maps functional GPIO identifiers to actual hardware pins.
 * To port to a new platform, only this table needs to be updated.
 */
static constexpr HfGpioMapping HF_GPIO_MAPPING[] = {
    // Motor control (PCAL95555 GPIO expander)
    [static_cast<uint8_t>(HfFunctionalGpioPin::MOTOR_ENABLE)] = {
        .chip_type = HfGpioChipType::PCAL95555_EXPANDER,
        .physical_pin = static_cast<uint8_t>(Pcal95555Chip1Pin::MOTOR_ENABLE_1),
        .is_inverted = false,
        .has_pullup = true,
        .max_current_ma = 25,
        .description = "Motor enable control"
    },
    [static_cast<uint8_t>(HfFunctionalGpioPin::MOTOR_BRAKE)] = {
        .chip_type = HfGpioChipType::PCAL95555_EXPANDER,
        .physical_pin = static_cast<uint8_t>(Pcal95555Chip1Pin::MOTOR_BRAKE_1),
        .is_inverted = false,
        .has_pullup = true,
        .max_current_ma = 25,
        .description = "Motor brake control"
    },
    [static_cast<uint8_t>(HfFunctionalGpioPin::MOTOR_FAULT_STATUS)] = {
        .chip_type = HfGpioChipType::PCAL95555_EXPANDER,
        .physical_pin = static_cast<uint8_t>(Pcal95555Chip1Pin::MOTOR_FAULT_1),
        .is_inverted = true,  // Active low fault signal
        .has_pullup = true,
        .max_current_ma = 25,
        .description = "Motor fault status input"
    },
    
    // System status LEDs (PCAL95555 GPIO expander)
    [static_cast<uint8_t>(HfFunctionalGpioPin::LED_STATUS_OK)] = {
        .chip_type = HfGpioChipType::PCAL95555_EXPANDER,
        .physical_pin = static_cast<uint8_t>(Pcal95555Chip1Pin::LED_STATUS_GREEN),
        .is_inverted = false,
        .has_pullup = false,
        .max_current_ma = 25,
        .description = "Status OK LED (Green)"
    },
    [static_cast<uint8_t>(HfFunctionalGpioPin::LED_STATUS_ERROR)] = {
        .chip_type = HfGpioChipType::PCAL95555_EXPANDER,
        .physical_pin = static_cast<uint8_t>(Pcal95555Chip1Pin::LED_ERROR),
        .is_inverted = false,
        .has_pullup = false,
        .max_current_ma = 25,
        .description = "Error status LED"
    },
    [static_cast<uint8_t>(HfFunctionalGpioPin::LED_STATUS_COMM)] = {
        .chip_type = HfGpioChipType::PCAL95555_EXPANDER,
        .physical_pin = static_cast<uint8_t>(Pcal95555Chip1Pin::LED_COMM),
        .is_inverted = false,
        .has_pullup = false,
        .max_current_ma = 25,
        .description = "Communication status LED"
    },
    
    // Communication interfaces (ESP32-C6 native GPIO)
    [static_cast<uint8_t>(HfFunctionalGpioPin::COMM_CAN_TX)] = {
        .chip_type = HfGpioChipType::ESP32_INTERNAL,
        .physical_pin = TWAI_TX_PIN,
        .is_inverted = false,
        .has_pullup = true,
        .max_current_ma = 40,
        .description = "CAN bus transmit"
    },
    [static_cast<uint8_t>(HfFunctionalGpioPin::COMM_CAN_RX)] = {
        .chip_type = HfGpioChipType::ESP32_INTERNAL,
        .physical_pin = TWAI_RX_PIN,
        .is_inverted = false,
        .has_pullup = true,
        .max_current_ma = 40,
        .description = "CAN bus receive"
    },
    [static_cast<uint8_t>(HfFunctionalGpioPin::COMM_I2C_SDA)] = {
        .chip_type = HfGpioChipType::ESP32_INTERNAL,
        .physical_pin = I2C_SDA_PIN,
        .is_inverted = false,
        .has_pullup = true,
        .max_current_ma = 40,
        .description = "I2C data line"
    },
    [static_cast<uint8_t>(HfFunctionalGpioPin::COMM_I2C_SCL)] = {
        .chip_type = HfGpioChipType::ESP32_INTERNAL,
        .physical_pin = I2C_SCL_PIN,
        .is_inverted = false,
        .has_pullup = true,
        .max_current_ma = 40,
        .description = "I2C clock line"
    },
    [static_cast<uint8_t>(HfFunctionalGpioPin::COMM_DEBUG_TX)] = {
        .chip_type = HfGpioChipType::ESP32_INTERNAL,
        .physical_pin = DEBUG_UART_TX_PIN,
        .is_inverted = false,
        .has_pullup = false,
        .max_current_ma = 40,
        .description = "Debug UART transmit"
    },
    [static_cast<uint8_t>(HfFunctionalGpioPin::COMM_DEBUG_RX)] = {
        .chip_type = HfGpioChipType::ESP32_INTERNAL,
        .physical_pin = DEBUG_UART_RX_PIN,
        .is_inverted = false,
        .has_pullup = true,
        .max_current_ma = 40,
        .description = "Debug UART receive"
    },
    
    // SPI interfaces (ESP32-C6 native GPIO) - Using safe pins
    [static_cast<uint8_t>(HfFunctionalGpioPin::SPI_MOTOR_CONTROLLER_CS)] = {
        .chip_type = HfGpioChipType::ESP32_INTERNAL,
        .physical_pin = SAFE_GPIO_10,  // Changed to safe pin
        .is_inverted = false,
        .has_pullup = true,
        .max_current_ma = 40,
        .description = "Motor controller chip select"
    },
    [static_cast<uint8_t>(HfFunctionalGpioPin::SPI_ENCODER_CS)] = {
        .chip_type = HfGpioChipType::ESP32_INTERNAL,
        .physical_pin = SAFE_GPIO_11,  // Changed to safe pin
        .is_inverted = false,
        .has_pullup = true,
        .max_current_ma = 40,
        .description = "Encoder chip select"
    },
    [static_cast<uint8_t>(HfFunctionalGpioPin::SPI_MISO)] = {
        .chip_type = HfGpioChipType::ESP32_INTERNAL,
        .physical_pin = CONDITIONAL_GPIO_2,  // Note: Conditional use
        .is_inverted = false,
        .has_pullup = true,
        .max_current_ma = 40,
        .description = "SPI MISO"
    },
    [static_cast<uint8_t>(HfFunctionalGpioPin::SPI_MOSI)] = {
        .chip_type = HfGpioChipType::ESP32_INTERNAL,
        .physical_pin = SAFE_GPIO_3,
        .is_inverted = false,
        .has_pullup = false,
        .max_current_ma = 40,
        .description = "SPI MOSI"
    },
    [static_cast<uint8_t>(HfFunctionalGpioPin::SPI_CLK)] = {
        .chip_type = HfGpioChipType::ESP32_INTERNAL,
        .physical_pin = SAFE_GPIO_1,
        .is_inverted = false,
        .has_pullup = false,
        .max_current_ma = 40,
        .description = "SPI Clock"
    },
    
    // User interface (PCAL95555 GPIO expander)
    [static_cast<uint8_t>(HfFunctionalGpioPin::USER_OUTPUT_1)] = {
        .chip_type = HfGpioChipType::PCAL95555_EXPANDER,
        .physical_pin = static_cast<uint8_t>(Pcal95555Chip1Pin::EXT_OUTPUT_1),
        .is_inverted = false,
        .has_pullup = false,
        .max_current_ma = 25,
        .description = "User output 1"
    },
    [static_cast<uint8_t>(HfFunctionalGpioPin::USER_OUTPUT_2)] = {
        .chip_type = HfGpioChipType::PCAL95555_EXPANDER,
        .physical_pin = static_cast<uint8_t>(Pcal95555Chip1Pin::EXT_OUTPUT_2),
        .is_inverted = false,
        .has_pullup = false,
        .max_current_ma = 25,
        .description = "User output 2"
    },
    [static_cast<uint8_t>(HfFunctionalGpioPin::USER_INPUT_1)] = {
        .chip_type = HfGpioChipType::PCAL95555_EXPANDER,
        .physical_pin = static_cast<uint8_t>(Pcal95555Chip1Pin::EXT_INPUT_1),
        .is_inverted = false,
        .has_pullup = true,
        .max_current_ma = 25,
        .description = "User input 1"
    },
    [static_cast<uint8_t>(HfFunctionalGpioPin::USER_INPUT_2)] = {
        .chip_type = HfGpioChipType::PCAL95555_EXPANDER,
        .physical_pin = static_cast<uint8_t>(Pcal95555Chip1Pin::EXT_INPUT_2),
        .is_inverted = false,
        .has_pullup = true,
        .max_current_ma = 25,
        .description = "User input 2"
    },
    
    // External device control (PCAL95555 GPIO expander)
    [static_cast<uint8_t>(HfFunctionalGpioPin::EXTERNAL_RELAY_1)] = {
        .chip_type = HfGpioChipType::PCAL95555_EXPANDER,
        .physical_pin = static_cast<uint8_t>(Pcal95555Chip1Pin::EXT_RELAY_1),
        .is_inverted = false,
        .has_pullup = false,
        .max_current_ma = 25,
        .description = "External relay 1"
    },
    [static_cast<uint8_t>(HfFunctionalGpioPin::EXTERNAL_RELAY_2)] = {
        .chip_type = HfGpioChipType::PCAL95555_EXPANDER,
        .physical_pin = static_cast<uint8_t>(Pcal95555Chip1Pin::EXT_RELAY_2),
        .is_inverted = false,
        .has_pullup = false,
        .max_current_ma = 25,
        .description = "External relay 2"
    },
    
    // Special purpose (ESP32-C6 native GPIO)
    [static_cast<uint8_t>(HfFunctionalGpioPin::WS2812_DATA)] = {
        .chip_type = HfGpioChipType::ESP32_INTERNAL,
        .physical_pin = WS2812_LED_PIN,
        .is_inverted = false,
        .has_pullup = false,
        .max_current_ma = 40,
        .description = "WS2812 LED data"
    }
};

/**
 * @brief Functional ADC to physical channel mapping table.
 * 
 * This table maps functional ADC identifiers to actual hardware channels.
 * To port to a new platform, only this table needs to be updated.
 */
static constexpr HfAdcMapping HF_ADC_MAPPING[] = {
    // Motor current sensing (TMC9660 integrated ADC)
    [static_cast<uint8_t>(HfFunctionalAdcChannel::MOTOR_CURRENT_PHASE_A)] = {
        .chip_type = HfAdcChipType::TMC9660_CONTROLLER,
        .physical_channel = 0,  // TMC9660 current sense A
        .resolution_bits = 12,
        .max_voltage_mv = 3300,
        .voltage_divider = 1.0f,
        .description = "Motor phase A current"
    },
    [static_cast<uint8_t>(HfFunctionalAdcChannel::MOTOR_CURRENT_PHASE_B)] = {
        .chip_type = HfAdcChipType::TMC9660_CONTROLLER,
        .physical_channel = 1,  // TMC9660 current sense B
        .resolution_bits = 12,
        .max_voltage_mv = 3300,
        .voltage_divider = 1.0f,
        .description = "Motor phase B current"
    },
    [static_cast<uint8_t>(HfFunctionalAdcChannel::MOTOR_CURRENT_PHASE_C)] = {
        .chip_type = HfAdcChipType::TMC9660_CONTROLLER,
        .physical_channel = 2,  // TMC9660 current sense C
        .resolution_bits = 12,
        .max_voltage_mv = 3300,
        .voltage_divider = 1.0f,
        .description = "Motor phase C current"
    },
    
    // System voltage monitoring (ESP32-C6 internal ADC)
    [static_cast<uint8_t>(HfFunctionalAdcChannel::SYSTEM_VOLTAGE_3V3)] = {
        .chip_type = HfAdcChipType::ESP32_INTERNAL,
        .physical_channel = 0,  // GPIO0 -> ADC1_CHANNEL_0
        .resolution_bits = 12,
        .max_voltage_mv = 3300,
        .voltage_divider = 1.0f,
        .description = "3.3V rail voltage"
    },
    [static_cast<uint8_t>(HfFunctionalAdcChannel::SYSTEM_VOLTAGE_5V)] = {
        .chip_type = HfAdcChipType::ESP32_INTERNAL,
        .physical_channel = 1,  // GPIO1 -> ADC1_CHANNEL_1
        .resolution_bits = 12,
        .max_voltage_mv = 3300,
        .voltage_divider = 0.66f,  // 5V -> 3.3V divider
        .description = "5V rail voltage"
    },
    
    // Temperature sensing (ESP32-C6 internal ADC)
    [static_cast<uint8_t>(HfFunctionalAdcChannel::SYSTEM_TEMPERATURE_AMBIENT)] = {
        .chip_type = HfAdcChipType::ESP32_INTERNAL,
        .physical_channel = 3,  // GPIO3 -> ADC1_CHANNEL_3
        .resolution_bits = 12,
        .max_voltage_mv = 3300,
        .voltage_divider = 1.0f,
        .description = "Ambient temperature sensor"
    },
    
    // User analog inputs (ESP32-C6 internal ADC)
    [static_cast<uint8_t>(HfFunctionalAdcChannel::USER_ANALOG_INPUT_1)] = {
        .chip_type = HfAdcChipType::ESP32_INTERNAL,
        .physical_channel = 2,  // GPIO2 -> ADC1_CHANNEL_2 (Conditional pin)
        .resolution_bits = 12,
        .max_voltage_mv = 3300,
        .voltage_divider = 1.0f,
        .description = "User analog input 1"
    }
};

//==============================================================================
// MAPPING ACCESS FUNCTIONS
//==============================================================================

/**
 * @brief Get GPIO hardware mapping for a functional pin.
 * @param functional_pin Functional GPIO pin identifier
 * @return Pointer to hardware mapping or nullptr if not found
 */
static inline const HfGpioMapping* GetGpioMapping(HfFunctionalGpioPin functional_pin) {
    uint8_t index = static_cast<uint8_t>(functional_pin);
    if (index < static_cast<uint8_t>(HfFunctionalGpioPin::HF_FUNCTIONAL_GPIO_COUNT)) {
        return &HF_GPIO_MAPPING[index];
    }
    return nullptr;
}

/**
 * @brief Get ADC hardware mapping for a functional channel.
 * @param functional_channel Functional ADC channel identifier
 * @return Pointer to hardware mapping or nullptr if not found
 */
static inline const HfAdcMapping* GetAdcMapping(HfFunctionalAdcChannel functional_channel) {
    uint8_t index = static_cast<uint8_t>(functional_channel);
    if (index < static_cast<uint8_t>(HfFunctionalAdcChannel::HF_FUNCTIONAL_ADC_COUNT)) {
        return &HF_ADC_MAPPING[index];
    }
    return nullptr;
}

/**
 * @brief Check if a functional GPIO pin is available on this platform.
 * @param functional_pin Functional GPIO pin identifier
 * @return True if available, false otherwise
 */
static inline bool IsGpioPinAvailable(HfFunctionalGpioPin functional_pin) {
    return GetGpioMapping(functional_pin) != nullptr;
}

/**
 * @brief Check if a functional ADC channel is available on this platform.
 * @param functional_channel Functional ADC channel identifier
 * @return True if available, false otherwise
 */
static inline bool IsAdcChannelAvailable(HfFunctionalAdcChannel functional_channel) {
    return GetAdcMapping(functional_channel) != nullptr;
}

#endif // HF_FUNCTIONAL_PIN_CONFIG_HPP
