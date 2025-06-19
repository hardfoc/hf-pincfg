#ifndef HF_EXT_PINS_ENUM_HPP
#define HF_EXT_PINS_ENUM_HPP

/**
 * @file hf_ext_pins_enum.hpp
 * @brief External GPIO pin definitions for PCAL95555 and other GPIO expanders.
 *
 * This header defines all external GPIO pins used by the HardFOC controller
 * through GPIO expander chips like PCAL95555.
 */

#include <cstdint>

/**
 * @name PCAL95555 GPIO Expander Pin Definitions
 * These pins are accessed through the PCAL95555 16-bit I/O expander.
 */
///@{

/**
 * @brief PCAL95555 GPIO expander pin assignments (single chip at I2C address 0x20)
 */
enum class Pcal95555Pin : uint8_t {
    // Motor control related pins
    MOTOR_ENABLE_1      = 0,  ///< Motor 1 enable signal
    MOTOR_ENABLE_2      = 1,  ///< Motor 2 enable signal
    MOTOR_BRAKE_1       = 2,  ///< Motor 1 brake signal
    MOTOR_BRAKE_2       = 3,  ///< Motor 2 brake signal
    MOTOR_FAULT_1       = 4,  ///< Motor 1 fault input
    MOTOR_FAULT_2       = 5,  ///< Motor 2 fault input
    
    // Status LEDs
    LED_STATUS_GREEN    = 6,  ///< Green status LED
    LED_STATUS_RED      = 7,  ///< Red status LED
    LED_ERROR           = 8,  ///< Error indicator LED
    LED_COMM            = 9,  ///< Communication activity LED
    
    // External control signals
    EXT_RELAY_1         = 10, ///< External relay 1
    EXT_RELAY_2         = 11, ///< External relay 2
    EXT_OUTPUT_1        = 12, ///< General purpose output 1
    EXT_OUTPUT_2        = 13, ///< General purpose output 2
    
    // Input signals
    EXT_INPUT_1         = 14, ///< General purpose input 1
    EXT_INPUT_2         = 15, ///< General purpose input 2
};

/**
 * @brief Functional mapping of external pins to logical names
 */
namespace ExtPinMap {
    // Motor control pin mappings
    constexpr auto MOTOR_ENABLE = Pcal95555Pin::MOTOR_ENABLE_1;
    constexpr auto MOTOR_FAULT = Pcal95555Pin::MOTOR_FAULT_1;
    constexpr auto MOTOR_BRAKE = Pcal95555Pin::MOTOR_BRAKE_1;
    
    // LED pin mappings
    constexpr auto LED_STATUS = Pcal95555Pin::LED_STATUS_GREEN;
    constexpr auto LED_ERROR = Pcal95555Pin::LED_ERROR;
    constexpr auto LED_COMM = Pcal95555Pin::LED_COMM;
    
    // External I/O mappings
    constexpr auto USER_OUTPUT_1 = Pcal95555Pin::EXT_OUTPUT_1;
    constexpr auto USER_OUTPUT_2 = Pcal95555Pin::EXT_OUTPUT_2;
    constexpr auto USER_INPUT_1 = Pcal95555Pin::EXT_INPUT_1;
    constexpr auto USER_INPUT_2 = Pcal95555Pin::EXT_INPUT_2;
}

/**
 * @brief I2C address for PCAL95555 chip
 */
namespace Pcal95555Addresses {
    constexpr uint8_t CHIP_ADDR = 0x20; ///< I2C address for PCAL95555
}

/**
 * @brief Pin direction configuration for PCAL95555 pins
 */
namespace Pcal95555Config {
    // Configuration - bit mask for input pins (1 = input, 0 = output)
    constexpr uint16_t INPUT_MASK = 
        (1 << static_cast<uint8_t>(Pcal95555Pin::MOTOR_FAULT_1)) |
        (1 << static_cast<uint8_t>(Pcal95555Pin::MOTOR_FAULT_2)) |
        (1 << static_cast<uint8_t>(Pcal95555Pin::EXT_INPUT_1)) |
        (1 << static_cast<uint8_t>(Pcal95555Pin::EXT_INPUT_2));
}

/**
 * @brief String representation for PCAL95555 pins
 */
constexpr const char* Pcal95555PinToString(Pcal95555Pin pin) {
    switch (pin) {
        case Pcal95555Pin::MOTOR_ENABLE_1: return "MOTOR_ENABLE_1";
        case Pcal95555Pin::MOTOR_ENABLE_2: return "MOTOR_ENABLE_2";
        case Pcal95555Pin::MOTOR_BRAKE_1: return "MOTOR_BRAKE_1";
        case Pcal95555Pin::MOTOR_BRAKE_2: return "MOTOR_BRAKE_2";
        case Pcal95555Pin::MOTOR_FAULT_1: return "MOTOR_FAULT_1";
        case Pcal95555Pin::MOTOR_FAULT_2: return "MOTOR_FAULT_2";
        case Pcal95555Pin::LED_STATUS_GREEN: return "LED_STATUS_GREEN";
        case Pcal95555Pin::LED_STATUS_RED: return "LED_STATUS_RED";
        case Pcal95555Pin::LED_ERROR: return "LED_ERROR";
        case Pcal95555Pin::LED_COMM: return "LED_COMM";
        case Pcal95555Pin::EXT_RELAY_1: return "EXT_RELAY_1";
        case Pcal95555Pin::EXT_RELAY_2: return "EXT_RELAY_2";
        case Pcal95555Pin::EXT_OUTPUT_1: return "EXT_OUTPUT_1";
        case Pcal95555Pin::EXT_OUTPUT_2: return "EXT_OUTPUT_2";
        case Pcal95555Pin::EXT_INPUT_1: return "EXT_INPUT_1";
        case Pcal95555Pin::EXT_INPUT_2: return "EXT_INPUT_2";
        default: return "UNKNOWN_PCAL95555_PIN";
    }
}

///@}

#endif // HF_EXT_PINS_ENUM_HPP