#ifndef HF_FUNCTIONAL_PIN_CONFIG_BASE_HPP
#define HF_FUNCTIONAL_PIN_CONFIG_BASE_HPP

/**
 * @file hf_functional_pin_config_base.hpp
 * @brief Shared types and helpers for functional pin configs.
 *
 * Board-specific pin config headers should include this file and then define
 * their local chip enums and XMACRO lists.
 */

#include <cstdint>
#include <string_view>

//==============================================================================
// PIN CATEGORIES (SHARED ENUM)
//==============================================================================

/**
 * @brief Pin categories for registration control
 * @details Determines which pins should be registered as GPIOs vs left for peripherals
 */
enum class HfPinCategory : uint8_t {
    PIN_CATEGORY_CORE = 0,    ///< System/core pins (skip GPIO registration)
    PIN_CATEGORY_COMM = 1,    ///< Communication pins (skip GPIO registration)
    PIN_CATEGORY_GPIO = 2,    ///< Available for GPIO operations
    PIN_CATEGORY_USER = 3     ///< User-defined pins (always register)
};

//==============================================================================
// XMACRO BOOLEAN VALUE DEFINES (FOR READABILITY)
//==============================================================================

inline constexpr bool PIN_LOGIC_NORMAL = false;   ///< Pin logic is not inverted
inline constexpr bool PIN_LOGIC_INVERTED = true;  ///< Pin logic is inverted

inline constexpr bool PIN_NO_PULL = false;        ///< No pull resistor
inline constexpr bool PIN_HAS_PULL = true;        ///< Has pull resistor

inline constexpr bool PIN_PULL_DOWN = false;      ///< Pull-down resistor
inline constexpr bool PIN_PULL_UP = true;         ///< Pull-up resistor

inline constexpr bool PIN_OPEN_DRAIN = false;     ///< Open-drain output mode
inline constexpr bool PIN_PUSH_PULL = true;       ///< Push-pull output mode

//==============================================================================
// MAPPING STRUCTURES
//==============================================================================

/**
 * @brief Structure containing comprehensive GPIO pin information with platform mapping.
 */
struct HfGpioMapping {
    uint8_t functional_pin;    ///< Functional pin identifier (enum value)
    uint8_t chip_type;         ///< Hardware chip identifier (enum value)
    uint8_t chip_unit;         ///< Chip unit/device number
    uint8_t gpio_bank;         ///< GPIO bank/port number within the chip
    uint8_t physical_pin;      ///< Physical pin number within the GPIO bank
    bool is_inverted;          ///< Whether pin logic is inverted
    bool has_pull;             ///< Whether pin has any pull resistor
    bool pull_is_up;           ///< If has_pull=true: true=pull-up, false=pull-down
    bool is_push_pull;         ///< Output mode: true=push-pull, false=open-drain
    uint32_t max_current_ma;   ///< Maximum current in milliamps
    uint8_t category;          ///< Pin category for registration control (enum value)
    std::string_view name;     ///< Human-readable pin name
};

/**
 * @brief Structure containing ADC channel information.
 */
struct HfAdcMapping {
    uint8_t functional_channel;  ///< Functional channel identifier (enum value)
    uint8_t chip_type;          ///< Hardware chip identifier (enum value)
    uint8_t chip_unit;          ///< Chip unit/device number
    uint8_t adc_unit;           ///< ADC unit/bank number within the chip
    uint8_t physical_channel;   ///< Physical channel number within the ADC unit
    uint8_t resolution_bits;    ///< ADC resolution in bits
    uint32_t max_voltage_mv;    ///< Maximum voltage in millivolts
    float voltage_divider;      ///< Voltage divider ratio
    const char* description;    ///< Channel description
};

#endif // HF_FUNCTIONAL_PIN_CONFIG_BASE_HPP
