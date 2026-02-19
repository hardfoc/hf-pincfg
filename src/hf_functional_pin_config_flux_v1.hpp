#ifndef HF_FUNCTIONAL_PIN_CONFIG_FLUX_V1_HPP
#define HF_FUNCTIONAL_PIN_CONFIG_FLUX_V1_HPP

/**
 * @file hf_functional_pin_config_flux_v1.hpp
 * @brief Functional pin mapping configuration for HardFOC Flux V1 board.
 *
 * This file provides the functional-to-physical pin mapping for the Flux V1 board.
 * The Flux board is a valve control platform using:
 *   - ESP32-C6 as the main MCU
 *   - MAX22200 octal solenoid driver (8 channels, SPI)
 *   - TLE92466ED six-channel low-side driver (6 channels, SPI)
 *   - WS2812 LED (status indication, same as Vortex)
 *   - NTC thermistor (temperature monitoring)
 *
 * The component handlers (GpioManager, ValveManager, etc.) use the functional
 * identifiers defined here, making the higher-level code completely portable.
 *
 * IMPORTANT: This file uses ONLY primitive types (bool, uint8_t, uint32_t, etc.)
 * and its own local enums. It has NO dependencies on external types or enums.
 */

#include "hf_functional_pin_config_base.hpp"

//==============================================================================
// CHIP TYPE ENUMS (LOCAL ENUMS)
//==============================================================================

enum class HfGpioChipType : uint8_t {
    ESP32_INTERNAL = 0,
    MAX22200_DRIVER,
    TLE92466ED_DRIVER
};

enum class HfAdcChipType : uint8_t {
    ESP32_INTERNAL = 0
};


//==============================================================================
// SINGLE SOURCE OF TRUTH: XMACRO PIN DEFINITIONS
//==============================================================================

/**
 * @brief XMACRO defining all GPIO pins for the Flux V1 board.
 *
 * ESP32-C6 Pin Allocation (Flux V1):
 *   GPIO0  - XTAL_32K_P          (Core)
 *   GPIO1  - XTAL_32K_N          (Core)
 *   GPIO2  - SPI2_MISO           (Comm)
 *   GPIO3  - WS2812_LED_DAT      (Comm)
 *   GPIO4  - NTC_ADC_INPUT       (ADC - thermistor)
 *   GPIO5  - MAX22200_FAULT_N    (GPIO - fault input, active low)
 *   GPIO6  - SPI2_SCK            (Comm)
 *   GPIO7  - SPI2_MOSI           (Comm)
 *   GPIO8  - TLE92466_FAULT_N    (GPIO - fault input, active low)
 *   GPIO9  - BOOT_SEL            (Core)
 *   GPIO10 - MAX22200_CMD        (GPIO - cmd pin for MAX22200)
 *   GPIO11 - MAX22200_ENABLE     (GPIO - enable for MAX22200)
 *   GPIO12 - JTAG_USB_D_N        (Core)
 *   GPIO13 - JTAG_USB_D_P        (Core)
 *   GPIO14 - TWAI_TX             (Comm)
 *   GPIO15 - TWAI_RX             (Comm)
 *   GPIO16 - TLE92466_RESN       (GPIO - reset for TLE92466, active low)
 *   GPIO17 - TLE92466_EN         (GPIO - enable for TLE92466, active high)
 *   GPIO18 - SPI2_CS_MAX22200    (Comm)
 *   GPIO19 - SPI2_CS_TLE92466    (Comm)
 *   GPIO20 - USER_GPIO_1         (GPIO - available for user)
 *   GPIO21 - I2C_SDA             (Comm - future expansion)
 *   GPIO22 - I2C_SCL             (Comm - future expansion)
 *   GPIO23 - USER_GPIO_2         (GPIO - available for user)
 *
 * Format: X(ENUM_NAME, STRING_NAME, CATEGORY, CHIP_TYPE, CHIP_UNIT, GPIO_BANK,
 *           PHYSICAL_PIN, LOGIC_INVERSION, HAS_PULL, PULL_DIRECTION, OUTPUT_MODE,
 *           MAX_CURRENT_MA)
 */
#define HF_FUNCTIONAL_GPIO_PIN_LIST(X) \
    /* CORE pins (system reserved - skip GPIO registration) */ \
    X(XTAL_32K_P, "CORE_XTAL_32K_P", HfPinCategory::PIN_CATEGORY_CORE, HfGpioChipType::ESP32_INTERNAL, 0, 0, 0, PIN_LOGIC_NORMAL, PIN_NO_PULL, PIN_PULL_DOWN, PIN_PUSH_PULL, 0) \
    X(XTAL_32K_N, "CORE_XTAL_32K_N", HfPinCategory::PIN_CATEGORY_CORE, HfGpioChipType::ESP32_INTERNAL, 0, 0, 1, PIN_LOGIC_NORMAL, PIN_NO_PULL, PIN_PULL_DOWN, PIN_PUSH_PULL, 0) \
    X(BOOT_SEL, "CORE_BOOT_SEL", HfPinCategory::PIN_CATEGORY_CORE, HfGpioChipType::ESP32_INTERNAL, 0, 0, 9, PIN_LOGIC_NORMAL, PIN_NO_PULL, PIN_PULL_DOWN, PIN_PUSH_PULL, 0) \
    X(JTAG_USB_D_N, "CORE_JTAG_USB_D_N", HfPinCategory::PIN_CATEGORY_CORE, HfGpioChipType::ESP32_INTERNAL, 0, 0, 12, PIN_LOGIC_NORMAL, PIN_NO_PULL, PIN_PULL_DOWN, PIN_PUSH_PULL, 0) \
    X(JTAG_USB_D_P, "CORE_JTAG_USB_D_P", HfPinCategory::PIN_CATEGORY_CORE, HfGpioChipType::ESP32_INTERNAL, 0, 0, 13, PIN_LOGIC_NORMAL, PIN_NO_PULL, PIN_PULL_DOWN, PIN_PUSH_PULL, 0) \
    \
    /* COMM pins (communication - skip GPIO registration) */ \
    X(SPI2_MISO, "COMM_SPI2_MISO", HfPinCategory::PIN_CATEGORY_COMM, HfGpioChipType::ESP32_INTERNAL, 0, 0, 2, PIN_LOGIC_NORMAL, PIN_HAS_PULL, PIN_PULL_UP, PIN_PUSH_PULL, 40) \
    X(SPI2_SCK, "COMM_SPI2_SCK", HfPinCategory::PIN_CATEGORY_COMM, HfGpioChipType::ESP32_INTERNAL, 0, 0, 6, PIN_LOGIC_NORMAL, PIN_NO_PULL, PIN_PULL_DOWN, PIN_PUSH_PULL, 40) \
    X(SPI2_MOSI, "COMM_SPI2_MOSI", HfPinCategory::PIN_CATEGORY_COMM, HfGpioChipType::ESP32_INTERNAL, 0, 0, 7, PIN_LOGIC_NORMAL, PIN_NO_PULL, PIN_PULL_DOWN, PIN_PUSH_PULL, 40) \
    X(SPI2_CS_MAX22200, "COMM_SPI2_CS_MAX22200", HfPinCategory::PIN_CATEGORY_COMM, HfGpioChipType::ESP32_INTERNAL, 0, 0, 18, PIN_LOGIC_INVERTED, PIN_HAS_PULL, PIN_PULL_UP, PIN_PUSH_PULL, 40) \
    X(SPI2_CS_TLE92466, "COMM_SPI2_CS_TLE92466", HfPinCategory::PIN_CATEGORY_COMM, HfGpioChipType::ESP32_INTERNAL, 0, 0, 19, PIN_LOGIC_INVERTED, PIN_HAS_PULL, PIN_PULL_UP, PIN_PUSH_PULL, 40) \
    X(TWAI_TX, "COMM_TWAI_TX", HfPinCategory::PIN_CATEGORY_COMM, HfGpioChipType::ESP32_INTERNAL, 0, 0, 14, PIN_LOGIC_NORMAL, PIN_HAS_PULL, PIN_PULL_UP, PIN_PUSH_PULL, 40) \
    X(TWAI_RX, "COMM_TWAI_RX", HfPinCategory::PIN_CATEGORY_COMM, HfGpioChipType::ESP32_INTERNAL, 0, 0, 15, PIN_LOGIC_NORMAL, PIN_HAS_PULL, PIN_PULL_UP, PIN_PUSH_PULL, 40) \
    X(I2C_SDA, "COMM_I2C_SDA", HfPinCategory::PIN_CATEGORY_COMM, HfGpioChipType::ESP32_INTERNAL, 0, 0, 21, PIN_LOGIC_NORMAL, PIN_HAS_PULL, PIN_PULL_UP, PIN_PUSH_PULL, 40) \
    X(I2C_SCL, "COMM_I2C_SCL", HfPinCategory::PIN_CATEGORY_COMM, HfGpioChipType::ESP32_INTERNAL, 0, 0, 22, PIN_LOGIC_NORMAL, PIN_HAS_PULL, PIN_PULL_UP, PIN_PUSH_PULL, 40) \
    X(WS2812_LED_DAT, "COMM_WS2812_LED_DAT", HfPinCategory::PIN_CATEGORY_COMM, HfGpioChipType::ESP32_INTERNAL, 0, 0, 3, PIN_LOGIC_NORMAL, PIN_NO_PULL, PIN_PULL_DOWN, PIN_PUSH_PULL, 40) \
    \
    /* MAX22200 control pins (managed by ESP32) */ \
    X(MAX22200_ENABLE, "GPIO_MAX22200_ENABLE", HfPinCategory::PIN_CATEGORY_GPIO, HfGpioChipType::ESP32_INTERNAL, 0, 0, 11, PIN_LOGIC_NORMAL, PIN_NO_PULL, PIN_PULL_DOWN, PIN_PUSH_PULL, 40) \
    X(MAX22200_CMD, "GPIO_MAX22200_CMD", HfPinCategory::PIN_CATEGORY_GPIO, HfGpioChipType::ESP32_INTERNAL, 0, 0, 10, PIN_LOGIC_NORMAL, PIN_NO_PULL, PIN_PULL_DOWN, PIN_PUSH_PULL, 40) \
    X(MAX22200_FAULT_N, "GPIO_MAX22200_FAULT_N", HfPinCategory::PIN_CATEGORY_GPIO, HfGpioChipType::ESP32_INTERNAL, 0, 0, 5, PIN_LOGIC_INVERTED, PIN_HAS_PULL, PIN_PULL_UP, PIN_PUSH_PULL, 40) \
    \
    /* TLE92466ED control pins (managed by ESP32) */ \
    X(TLE92466_RESN, "GPIO_TLE92466_RESN", HfPinCategory::PIN_CATEGORY_GPIO, HfGpioChipType::ESP32_INTERNAL, 0, 0, 16, PIN_LOGIC_INVERTED, PIN_NO_PULL, PIN_PULL_DOWN, PIN_PUSH_PULL, 40) \
    X(TLE92466_EN, "GPIO_TLE92466_EN", HfPinCategory::PIN_CATEGORY_GPIO, HfGpioChipType::ESP32_INTERNAL, 0, 0, 17, PIN_LOGIC_NORMAL, PIN_NO_PULL, PIN_PULL_DOWN, PIN_PUSH_PULL, 40) \
    X(TLE92466_FAULT_N, "GPIO_TLE92466_FAULT_N", HfPinCategory::PIN_CATEGORY_GPIO, HfGpioChipType::ESP32_INTERNAL, 0, 0, 8, PIN_LOGIC_INVERTED, PIN_HAS_PULL, PIN_PULL_UP, PIN_PUSH_PULL, 40) \
    \
    /* User-available GPIO pins */ \
    X(USER_GPIO_1, "GPIO_USER_1", HfPinCategory::PIN_CATEGORY_USER, HfGpioChipType::ESP32_INTERNAL, 0, 0, 20, PIN_LOGIC_NORMAL, PIN_NO_PULL, PIN_PULL_DOWN, PIN_PUSH_PULL, 40) \
    X(USER_GPIO_2, "GPIO_USER_2", HfPinCategory::PIN_CATEGORY_USER, HfGpioChipType::ESP32_INTERNAL, 0, 0, 23, PIN_LOGIC_NORMAL, PIN_NO_PULL, PIN_PULL_DOWN, PIN_PUSH_PULL, 40) \

/**
 * @brief XMACRO defining all ADC channels for the Flux V1 board.
 *
 * The Flux board uses the ESP32-C6 internal ADC for thermistor readings.
 *
 * Format: X(ENUM_NAME, STRING_NAME, CHIP_TYPE, CHIP_UNIT, ADC_UNIT,
 *           PHYSICAL_CHANNEL, RESOLUTION_BITS, MAX_VOLTAGE_MV, VOLTAGE_DIVIDER,
 *           DESCRIPTION)
 */
#define HF_FUNCTIONAL_ADC_CHANNEL_LIST(X) \
    /* ESP32-C6 Internal ADC - NTC thermistor on GPIO4 (ADC1_CH4) */ \
    X(ESP32_NTC_THERMISTOR, "ADC_NTC_THERMISTOR", HfAdcChipType::ESP32_INTERNAL, 0, 0, 4, 12, 3300, 1.0f, "NTC Thermistor Temperature Input") \
    \
    /* ESP32-C6 Internal ADC - Supply voltage monitoring (if connected) */ \
    X(ESP32_VBUS_MONITOR, "ADC_VBUS_MONITOR", HfAdcChipType::ESP32_INTERNAL, 0, 0, 0, 12, 3300, 11.0f, "Bus Voltage Monitor (with divider)") \

//==============================================================================
// GENERATED ENUMS AND TABLES
//==============================================================================

/**
 * @brief Functional GPIO pin identifiers (generated from XMACRO).
 */
enum class HfFunctionalGpioPin : uint8_t {
#define X(name, str, cat, chip, chip_unit, gpio_bank, pin, inv, pull, pull_up, push_pull, current) name,
    HF_FUNCTIONAL_GPIO_PIN_LIST(X)
#undef X
    HF_FUNCTIONAL_GPIO_COUNT // Always last
};

/**
 * @brief String names for functional pins (generated from XMACRO).
 */
static constexpr std::string_view HfFunctionalGpioPinNames[] = {
#define X(name, str, cat, chip, chip_unit, gpio_bank, pin, inv, pull, pull_up, push_pull, current) str,
    HF_FUNCTIONAL_GPIO_PIN_LIST(X)
#undef X
};

/**
 * @brief Pin categories for functional pins (generated from XMACRO).
 */
static constexpr HfPinCategory HfFunctionalGpioPinCategories[] = {
#define X(name, str, cat, chip, chip_unit, gpio_bank, pin, inv, pull, pull_up, push_pull, current) cat,
    HF_FUNCTIONAL_GPIO_PIN_LIST(X)
#undef X
};

/**
 * @brief Complete GPIO mapping table (generated from XMACRO).
 */
static constexpr HfGpioMapping HF_GPIO_MAPPING[] = {
#define X(name, str, cat, chip, chip_unit, gpio_bank, pin, inv, pull, pull_up, push_pull, current) \
    { static_cast<uint8_t>(HfFunctionalGpioPin::name), static_cast<uint8_t>(chip), chip_unit, gpio_bank, pin, inv, pull, pull_up, push_pull, current, static_cast<uint8_t>(cat), str },
    HF_FUNCTIONAL_GPIO_PIN_LIST(X)
#undef X
};

/**
 * @brief Functional ADC channel identifiers (generated from XMACRO).
 */
enum class HfFunctionalAdcChannel : uint8_t {
#define X(name, str, chip, chip_unit, adc_unit, ch, res, max_v, div, desc) name,
    HF_FUNCTIONAL_ADC_CHANNEL_LIST(X)
#undef X
    HF_FUNCTIONAL_ADC_COUNT // Always last
};

/**
 * @brief String names for functional ADC channels (generated from XMACRO).
 */
static constexpr std::string_view HfFunctionalAdcChannelNames[] = {
#define X(name, str, chip, chip_unit, adc_unit, ch, res, max_v, div, desc) str,
    HF_FUNCTIONAL_ADC_CHANNEL_LIST(X)
#undef X
};

/**
 * @brief Complete ADC mapping table (generated from XMACRO).
 */
static constexpr HfAdcMapping HF_ADC_MAPPING[] = {
#define X(name, str, chip, chip_unit, adc_unit, ch, res, max_v, div, desc) \
    { static_cast<uint8_t>(HfFunctionalAdcChannel::name), static_cast<uint8_t>(chip), chip_unit, adc_unit, ch, res, max_v, div, desc },
    HF_FUNCTIONAL_ADC_CHANNEL_LIST(X)
#undef X
};

//==============================================================================
// COMPILE-TIME SIZE CALCULATIONS
//==============================================================================

static constexpr size_t HF_GPIO_MAPPING_SIZE = sizeof(HF_GPIO_MAPPING) / sizeof(HF_GPIO_MAPPING[0]);
static constexpr size_t HF_ADC_MAPPING_SIZE = sizeof(HF_ADC_MAPPING) / sizeof(HF_ADC_MAPPING[0]);

//==============================================================================
// HELPER FUNCTIONS
//==============================================================================

inline std::string_view to_string(HfFunctionalGpioPin pin) noexcept {
    const size_t index = static_cast<size_t>(pin);
    if (index < sizeof(HfFunctionalGpioPinNames) / sizeof(HfFunctionalGpioPinNames[0])) {
        return HfFunctionalGpioPinNames[index];
    }
    return "UNKNOWN_PIN";
}

inline std::string_view to_string(HfFunctionalAdcChannel channel) noexcept {
    const size_t index = static_cast<size_t>(channel);
    if (index < sizeof(HfFunctionalAdcChannelNames) / sizeof(HfFunctionalAdcChannelNames[0])) {
        return HfFunctionalAdcChannelNames[index];
    }
    return "UNKNOWN_ADC_CHANNEL";
}

inline HfPinCategory get_pin_category(HfFunctionalGpioPin pin) noexcept {
    const size_t index = static_cast<size_t>(pin);
    if (index < sizeof(HfFunctionalGpioPinCategories) / sizeof(HfFunctionalGpioPinCategories[0])) {
        return HfFunctionalGpioPinCategories[index];
    }
    return HfPinCategory::PIN_CATEGORY_CORE;
}

inline bool should_register_as_gpio(HfPinCategory category) noexcept {
    return category == HfPinCategory::PIN_CATEGORY_GPIO ||
           category == HfPinCategory::PIN_CATEGORY_USER;
}

inline bool is_reserved_prefix(std::string_view name) noexcept {
    return name.starts_with("CORE_") ||
           name.starts_with("COMM_") ||
           name.starts_with("SYS_") ||
           name.starts_with("INTERNAL_");
}

//==============================================================================
// MAPPING ACCESS FUNCTIONS
//==============================================================================

static inline const HfGpioMapping* GetGpioMapping(HfFunctionalGpioPin functional_pin) {
    const size_t index = static_cast<size_t>(functional_pin);
    if (index < HF_GPIO_MAPPING_SIZE) {
        return &HF_GPIO_MAPPING[index];
    }
    return nullptr;
}

static inline const HfAdcMapping* GetAdcMapping(HfFunctionalAdcChannel functional_channel) {
    const size_t index = static_cast<size_t>(functional_channel);
    if (index < HF_ADC_MAPPING_SIZE) {
        return &HF_ADC_MAPPING[index];
    }
    return nullptr;
}

static inline bool IsGpioPinAvailable(HfFunctionalGpioPin functional_pin) {
    return GetGpioMapping(functional_pin) != nullptr;
}

static inline bool IsAdcChannelAvailable(HfFunctionalAdcChannel functional_channel) {
    return GetAdcMapping(functional_channel) != nullptr;
}

//==============================================================================
// UTILITY FUNCTIONS
//==============================================================================

static inline size_t GetGpioPinsByCategory(HfPinCategory category, HfFunctionalGpioPin* output, size_t max_count) {
    size_t count = 0;
    for (size_t i = 0; i < HF_GPIO_MAPPING_SIZE && count < max_count; ++i) {
        if (HfFunctionalGpioPinCategories[i] == category) {
            output[count++] = static_cast<HfFunctionalGpioPin>(i);
        }
    }
    return count;
}

static inline size_t GetGpioPinsByChipType(HfGpioChipType chip_type, HfFunctionalGpioPin* output, size_t max_count) {
    size_t count = 0;
    for (size_t i = 0; i < HF_GPIO_MAPPING_SIZE && count < max_count; ++i) {
        if (HF_GPIO_MAPPING[i].chip_type == static_cast<uint8_t>(chip_type)) {
            output[count++] = static_cast<HfFunctionalGpioPin>(i);
        }
    }
    return count;
}

static inline bool RequiresPullUp(HfFunctionalGpioPin functional_pin) {
    const auto* mapping = GetGpioMapping(functional_pin);
    return mapping && mapping->has_pull && mapping->pull_is_up;
}

static inline bool RequiresPullDown(HfFunctionalGpioPin functional_pin) {
    const auto* mapping = GetGpioMapping(functional_pin);
    return mapping && mapping->has_pull && !mapping->pull_is_up;
}

#endif // HF_FUNCTIONAL_PIN_CONFIG_FLUX_V1_HPP
