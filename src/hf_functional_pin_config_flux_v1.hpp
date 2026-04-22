#ifndef HF_FUNCTIONAL_PIN_CONFIG_FLUX_V1_HPP
#define HF_FUNCTIONAL_PIN_CONFIG_FLUX_V1_HPP

/**
 * @file hf_functional_pin_config_flux_v1.hpp
 * @brief Functional pin mapping configuration for HardFOC Flux V1 board.
 *
 * This file provides the functional-to-physical pin mapping for the Flux V1 board.
 * The Flux board is a valve control platform using:
 *   - ESP32-S3 as the main MCU
 *   - MAX22200 octal solenoid driver (8 channels, SPI)
 *   - TLE92466ED six-channel low-side driver (6 channels, SPI, shared bus)
 *   - WS2812 LED (status indication)
 *   - NTC thermistor (temperature monitoring)
 *
 * SPI bus is shared between MAX22200 and TLE92466ED (same MOSI/MISO/SCK).
 * MAX22200 pin assignment matches the hf-max22200-driver standalone examples.
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
    ESP32_INTERNAL = 0,
    ADS7952_EXTERNAL
};


//==============================================================================
// SINGLE SOURCE OF TRUTH: XMACRO PIN DEFINITIONS
//==============================================================================

/**
 * @brief XMACRO defining all GPIO pins for the Flux V1 board.
 *
 * ESP32-S3 Pin Allocation (Flux V1 dev / example build):
 *   GPIO0  - CORE_BOOT (strapping)  (Core)
 *   GPIO2  - MAX22200_ENABLE        (GPIO - enable for MAX22200)
 *   GPIO3  - WS2812_LED_DAT         (Comm - WS2812 data)
 *   GPIO4  - SPI2_CS_TLE92466       (Comm - TLE92466ED chip select)
 *   GPIO5  - TLE92466_EN            (GPIO - enable for TLE92466, active high)
 *   GPIO6  - TLE92466_RESN          (GPIO - reset for TLE92466, active low)
 *   GPIO7  - TLE92466_DRV0          (GPIO - external drive 0)
 *   GPIO8  - NTC_ADC_INPUT          (ADC1_CH7 - thermistor)
 *   GPIO9  - VBUS_ADC_INPUT         (ADC1_CH8 - bus voltage monitor)
 *   GPIO10 - SPI3_CS_ADS7952_0     (Comm - ADS7952 ADC chip select)
 *   GPIO11 - SPI3_MOSI             (Comm - ADS7952 dedicated SPI bus)
 *   GPIO12 - SPI3_SCK              (Comm - ADS7952 dedicated SPI bus)
 *   GPIO13 - SPI3_MISO             (Comm - ADS7952 dedicated SPI bus)
 *   GPIO15 - TLE92466_DRV1          (GPIO - external drive 1)
 *   GPIO16 - TLE92466_FAULT_N       (GPIO - fault input, active low)
 *   GPIO17 - TWAI_TX                (Comm - CAN TX)
 *   GPIO18 - TWAI_RX                (Comm - CAN RX)
 *   GPIO19 - USB_D_N                (Core - USB JTAG)
 *   GPIO20 - USB_D_P                (Core - USB JTAG)
 *   GPIO35 - SPI2_MISO              (Comm - shared SPI bus)
 *   GPIO36 - SPI2_SCK               (Comm - shared SPI bus)
 *   GPIO37 - SPI2_MOSI              (Comm - shared SPI bus)
 *   GPIO38 - SPI2_CS_MAX22200       (Comm - MAX22200 chip select)
 *   GPIO39 - MAX22200_CMD           (GPIO - cmd pin for MAX22200)
 *   GPIO40 - MAX22200_TRIGA         (GPIO - trigger A)
 *   GPIO41 - MAX22200_TRIGB         (GPIO - trigger B)
 *   GPIO42 - MAX22200_FAULT_N       (GPIO - fault input, active low)
 *   GPIO47 - UART_FDO2_TX           (Comm — PyroScience FDO2-G2 host TX)
 *   GPIO48 - UART_FDO2_RX           (Comm — PyroScience FDO2-G2 host RX)
 *
 * Format: X(ENUM_NAME, STRING_NAME, CATEGORY, CHIP_TYPE, CHIP_UNIT, GPIO_BANK,
 *           PHYSICAL_PIN, LOGIC_INVERSION, HAS_PULL, PULL_DIRECTION, OUTPUT_MODE,
 *           MAX_CURRENT_MA)
 */
#define HF_FUNCTIONAL_GPIO_PIN_LIST(X) \
    /* CORE pins (system reserved - skip GPIO registration) */ \
    X(CORE_BOOT, "CORE_BOOT", HfPinCategory::PIN_CATEGORY_CORE, HfGpioChipType::ESP32_INTERNAL, 0, 0, 0, PIN_LOGIC_NORMAL, PIN_NO_PULL, PIN_PULL_DOWN, PIN_PUSH_PULL, 0) \
    X(USB_D_N, "CORE_USB_D_N", HfPinCategory::PIN_CATEGORY_CORE, HfGpioChipType::ESP32_INTERNAL, 0, 0, 19, PIN_LOGIC_NORMAL, PIN_NO_PULL, PIN_PULL_DOWN, PIN_PUSH_PULL, 0) \
    X(USB_D_P, "CORE_USB_D_P", HfPinCategory::PIN_CATEGORY_CORE, HfGpioChipType::ESP32_INTERNAL, 0, 0, 20, PIN_LOGIC_NORMAL, PIN_NO_PULL, PIN_PULL_DOWN, PIN_PUSH_PULL, 0) \
    \
    /* COMM pins (communication - skip GPIO registration) */ \
    X(SPI2_MISO, "COMM_SPI2_MISO", HfPinCategory::PIN_CATEGORY_COMM, HfGpioChipType::ESP32_INTERNAL, 0, 0, 35, PIN_LOGIC_NORMAL, PIN_HAS_PULL, PIN_PULL_UP, PIN_PUSH_PULL, 40) \
    X(SPI2_SCK, "COMM_SPI2_SCK", HfPinCategory::PIN_CATEGORY_COMM, HfGpioChipType::ESP32_INTERNAL, 0, 0, 36, PIN_LOGIC_NORMAL, PIN_NO_PULL, PIN_PULL_DOWN, PIN_PUSH_PULL, 40) \
    X(SPI2_MOSI, "COMM_SPI2_MOSI", HfPinCategory::PIN_CATEGORY_COMM, HfGpioChipType::ESP32_INTERNAL, 0, 0, 37, PIN_LOGIC_NORMAL, PIN_NO_PULL, PIN_PULL_DOWN, PIN_PUSH_PULL, 40) \
    X(SPI2_CS_MAX22200, "COMM_SPI2_CS_MAX22200", HfPinCategory::PIN_CATEGORY_COMM, HfGpioChipType::ESP32_INTERNAL, 0, 0, 38, PIN_LOGIC_INVERTED, PIN_HAS_PULL, PIN_PULL_UP, PIN_PUSH_PULL, 40) \
    X(SPI2_CS_TLE92466, "COMM_SPI2_CS_TLE92466", HfPinCategory::PIN_CATEGORY_COMM, HfGpioChipType::ESP32_INTERNAL, 0, 0, 4, PIN_LOGIC_INVERTED, PIN_HAS_PULL, PIN_PULL_UP, PIN_PUSH_PULL, 40) \
    X(TWAI_TX, "COMM_TWAI_TX", HfPinCategory::PIN_CATEGORY_COMM, HfGpioChipType::ESP32_INTERNAL, 0, 0, 17, PIN_LOGIC_NORMAL, PIN_HAS_PULL, PIN_PULL_UP, PIN_PUSH_PULL, 40) \
    X(TWAI_RX, "COMM_TWAI_RX", HfPinCategory::PIN_CATEGORY_COMM, HfGpioChipType::ESP32_INTERNAL, 0, 0, 18, PIN_LOGIC_NORMAL, PIN_HAS_PULL, PIN_PULL_UP, PIN_PUSH_PULL, 40) \
    X(UART_FDO2_TX, "COMM_UART_FDO2_TX", HfPinCategory::PIN_CATEGORY_COMM, HfGpioChipType::ESP32_INTERNAL, 0, 0, 47, PIN_LOGIC_NORMAL, PIN_HAS_PULL, PIN_PULL_UP, PIN_PUSH_PULL, 40) \
    X(UART_FDO2_RX, "COMM_UART_FDO2_RX", HfPinCategory::PIN_CATEGORY_COMM, HfGpioChipType::ESP32_INTERNAL, 0, 0, 48, PIN_LOGIC_NORMAL, PIN_HAS_PULL, PIN_PULL_UP, PIN_PUSH_PULL, 40) \
    X(WS2812_LED_DAT, "COMM_WS2812_LED_DAT", HfPinCategory::PIN_CATEGORY_COMM, HfGpioChipType::ESP32_INTERNAL, 0, 0, 3, PIN_LOGIC_NORMAL, PIN_NO_PULL, PIN_PULL_DOWN, PIN_PUSH_PULL, 40) \
    \
    /* MAX22200 control pins (managed by ESP32) */ \
    X(MAX22200_ENABLE, "GPIO_MAX22200_ENABLE", HfPinCategory::PIN_CATEGORY_GPIO, HfGpioChipType::ESP32_INTERNAL, 0, 0, 2, PIN_LOGIC_NORMAL, PIN_NO_PULL, PIN_PULL_DOWN, PIN_PUSH_PULL, 40) \
    X(MAX22200_CMD, "GPIO_MAX22200_CMD", HfPinCategory::PIN_CATEGORY_GPIO, HfGpioChipType::ESP32_INTERNAL, 0, 0, 39, PIN_LOGIC_NORMAL, PIN_NO_PULL, PIN_PULL_DOWN, PIN_PUSH_PULL, 40) \
    X(MAX22200_FAULT_N, "GPIO_MAX22200_FAULT_N", HfPinCategory::PIN_CATEGORY_GPIO, HfGpioChipType::ESP32_INTERNAL, 0, 0, 42, PIN_LOGIC_INVERTED, PIN_HAS_PULL, PIN_PULL_UP, PIN_PUSH_PULL, 40) \
    X(MAX22200_TRIGA, "GPIO_MAX22200_TRIGA", HfPinCategory::PIN_CATEGORY_GPIO, HfGpioChipType::ESP32_INTERNAL, 0, 0, 40, PIN_LOGIC_NORMAL, PIN_NO_PULL, PIN_PULL_DOWN, PIN_PUSH_PULL, 40) \
    X(MAX22200_TRIGB, "GPIO_MAX22200_TRIGB", HfPinCategory::PIN_CATEGORY_GPIO, HfGpioChipType::ESP32_INTERNAL, 0, 0, 41, PIN_LOGIC_NORMAL, PIN_NO_PULL, PIN_PULL_DOWN, PIN_PUSH_PULL, 40) \
    \
    /* TLE92466ED control pins (managed by ESP32) */ \
    X(TLE92466_EN, "GPIO_TLE92466_EN", HfPinCategory::PIN_CATEGORY_GPIO, HfGpioChipType::ESP32_INTERNAL, 0, 0, 5, PIN_LOGIC_NORMAL, PIN_NO_PULL, PIN_PULL_DOWN, PIN_PUSH_PULL, 40) \
    X(TLE92466_RESN, "GPIO_TLE92466_RESN", HfPinCategory::PIN_CATEGORY_GPIO, HfGpioChipType::ESP32_INTERNAL, 0, 0, 6, PIN_LOGIC_INVERTED, PIN_NO_PULL, PIN_PULL_DOWN, PIN_PUSH_PULL, 40) \
    X(TLE92466_DRV0, "GPIO_TLE92466_DRV0", HfPinCategory::PIN_CATEGORY_GPIO, HfGpioChipType::ESP32_INTERNAL, 0, 0, 7, PIN_LOGIC_NORMAL, PIN_NO_PULL, PIN_PULL_DOWN, PIN_PUSH_PULL, 40) \
    X(TLE92466_DRV1, "GPIO_TLE92466_DRV1", HfPinCategory::PIN_CATEGORY_GPIO, HfGpioChipType::ESP32_INTERNAL, 0, 0, 15, PIN_LOGIC_NORMAL, PIN_NO_PULL, PIN_PULL_DOWN, PIN_PUSH_PULL, 40) \
    X(TLE92466_FAULT_N, "GPIO_TLE92466_FAULT_N", HfPinCategory::PIN_CATEGORY_GPIO, HfGpioChipType::ESP32_INTERNAL, 0, 0, 16, PIN_LOGIC_INVERTED, PIN_HAS_PULL, PIN_PULL_UP, PIN_PUSH_PULL, 40) \
    \
    /* SPI3 pins — dedicated ADS7952 ADC bus */ \
    X(SPI3_CS_ADS7952_0, "COMM_SPI3_CS_ADS7952_0", HfPinCategory::PIN_CATEGORY_COMM, HfGpioChipType::ESP32_INTERNAL, 0, 0, 10, PIN_LOGIC_INVERTED, PIN_HAS_PULL, PIN_PULL_UP, PIN_PUSH_PULL, 40) \
    X(SPI3_MOSI, "COMM_SPI3_MOSI", HfPinCategory::PIN_CATEGORY_COMM, HfGpioChipType::ESP32_INTERNAL, 0, 0, 11, PIN_LOGIC_NORMAL, PIN_NO_PULL, PIN_PULL_DOWN, PIN_PUSH_PULL, 40) \
    X(SPI3_SCK, "COMM_SPI3_SCK", HfPinCategory::PIN_CATEGORY_COMM, HfGpioChipType::ESP32_INTERNAL, 0, 0, 12, PIN_LOGIC_NORMAL, PIN_NO_PULL, PIN_PULL_DOWN, PIN_PUSH_PULL, 40) \
    X(SPI3_MISO, "COMM_SPI3_MISO", HfPinCategory::PIN_CATEGORY_COMM, HfGpioChipType::ESP32_INTERNAL, 0, 0, 13, PIN_LOGIC_NORMAL, PIN_HAS_PULL, PIN_PULL_UP, PIN_PUSH_PULL, 40) \

/**
 * @brief XMACRO defining all ADC channels for the Flux V1 board.
 *
 * The Flux board uses the ESP32-S3 internal ADC for thermistor readings.
 *
 * ESP32-S3 ADC1 channel mapping:
 *   GPIO1=CH0, GPIO2=CH1, GPIO3=CH2, GPIO4=CH3, GPIO5=CH4,
 *   GPIO6=CH5, GPIO7=CH6, GPIO8=CH7, GPIO9=CH8, GPIO10=CH9
 *
 * Format: X(ENUM_NAME, STRING_NAME, CHIP_TYPE, CHIP_UNIT, ADC_UNIT,
 *           PHYSICAL_CHANNEL, RESOLUTION_BITS, MAX_VOLTAGE_MV, VOLTAGE_DIVIDER,
 *           DESCRIPTION)
 */
#define HF_FUNCTIONAL_ADC_CHANNEL_LIST(X) \
    /* ESP32-S3 Internal ADC - NTC thermistor on GPIO8 (ADC1_CH7) */ \
    X(ESP32_NTC_THERMISTOR, "ADC_NTC_THERMISTOR", HfAdcChipType::ESP32_INTERNAL, 0, 0, 7, 12, 3300, 1.0f, "NTC Thermistor Temperature Input") \
    \
    /* ESP32-S3 Internal ADC - Supply voltage monitoring on GPIO9 (ADC1_CH8) */ \
    X(ESP32_VBUS_MONITOR, "ADC_VBUS_MONITOR", HfAdcChipType::ESP32_INTERNAL, 0, 0, 8, 12, 3300, 11.0f, "Bus Voltage Monitor (with divider)") \
    \
    /* ADS7952 External 12-ch ADC — Device 0 on SPI3, Vref=2.5V, VA=5.0V */ \
    /* Channels 0-11 mapped as individual functional ADC channels           */ \
    X(ADS7952_0_CH0,  "ADC_ADS7952_0_CH0",  HfAdcChipType::ADS7952_EXTERNAL, 0, 0,  0, 12, 2500, 1.0f, "ADS7952 Dev0 CH0") \
    X(ADS7952_0_CH1,  "ADC_ADS7952_0_CH1",  HfAdcChipType::ADS7952_EXTERNAL, 0, 0,  1, 12, 2500, 1.0f, "ADS7952 Dev0 CH1") \
    X(ADS7952_0_CH2,  "ADC_ADS7952_0_CH2",  HfAdcChipType::ADS7952_EXTERNAL, 0, 0,  2, 12, 2500, 1.0f, "ADS7952 Dev0 CH2") \
    X(ADS7952_0_CH3,  "ADC_ADS7952_0_CH3",  HfAdcChipType::ADS7952_EXTERNAL, 0, 0,  3, 12, 2500, 1.0f, "ADS7952 Dev0 CH3") \
    X(ADS7952_0_CH4,  "ADC_ADS7952_0_CH4",  HfAdcChipType::ADS7952_EXTERNAL, 0, 0,  4, 12, 2500, 1.0f, "ADS7952 Dev0 CH4") \
    X(ADS7952_0_CH5,  "ADC_ADS7952_0_CH5",  HfAdcChipType::ADS7952_EXTERNAL, 0, 0,  5, 12, 2500, 1.0f, "ADS7952 Dev0 CH5") \
    X(ADS7952_0_CH6,  "ADC_ADS7952_0_CH6",  HfAdcChipType::ADS7952_EXTERNAL, 0, 0,  6, 12, 2500, 1.0f, "ADS7952 Dev0 CH6") \
    X(ADS7952_0_CH7,  "ADC_ADS7952_0_CH7",  HfAdcChipType::ADS7952_EXTERNAL, 0, 0,  7, 12, 2500, 1.0f, "ADS7952 Dev0 CH7") \
    X(ADS7952_0_CH8,  "ADC_ADS7952_0_CH8",  HfAdcChipType::ADS7952_EXTERNAL, 0, 0,  8, 12, 2500, 1.0f, "ADS7952 Dev0 CH8") \
    X(ADS7952_0_CH9,  "ADC_ADS7952_0_CH9",  HfAdcChipType::ADS7952_EXTERNAL, 0, 0,  9, 12, 2500, 1.0f, "ADS7952 Dev0 CH9") \
    X(ADS7952_0_CH10, "ADC_ADS7952_0_CH10", HfAdcChipType::ADS7952_EXTERNAL, 0, 0, 10, 12, 2500, 1.0f, "ADS7952 Dev0 CH10") \
    X(ADS7952_0_CH11, "ADC_ADS7952_0_CH11", HfAdcChipType::ADS7952_EXTERNAL, 0, 0, 11, 12, 2500, 1.0f, "ADS7952 Dev0 CH11") \

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
// FDO2 UART — exported physical GPIO (must match UART_FDO2_TX / UART_FDO2_RX)
//==============================================================================

/** MCU TX line → FDO2 RX (PSUP). Same value as `HF_GPIO_MAPPING[UART_FDO2_TX].physical_pin`. */
inline constexpr int kFluxHwUartFdo2TxGpio =
    static_cast<int>(HF_GPIO_MAPPING[static_cast<size_t>(HfFunctionalGpioPin::UART_FDO2_TX)].physical_pin);

/** MCU RX line ← FDO2 TX (PSUP). Same value as `HF_GPIO_MAPPING[UART_FDO2_RX].physical_pin`. */
inline constexpr int kFluxHwUartFdo2RxGpio =
    static_cast<int>(HF_GPIO_MAPPING[static_cast<size_t>(HfFunctionalGpioPin::UART_FDO2_RX)].physical_pin);

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
