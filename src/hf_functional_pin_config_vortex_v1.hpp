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
 *
 * IMPORTANT: This file uses ONLY primitive types (bool, uint8_t, uint32_t, etc.)
 * and its own local enums. It has NO dependencies on external types or enums.
 */

#include <cstdint>
#include <string_view>

//==============================================================================
// PIN CATEGORIES (LOCAL ENUMS)
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
// CHIP TYPE ENUMS (LOCAL ENUMS)
//==============================================================================

enum class HfGpioChipType : uint8_t {
    ESP32_INTERNAL = 0,
    PCAL95555_EXPANDER,
    TMC9660_CONTROLLER
};

enum class HfAdcChipType : uint8_t {
    ESP32_INTERNAL = 0,
    TMC9660_CONTROLLER
};

//==============================================================================
// XMACRO BOOLEAN VALUE DEFINES (FOR READABILITY)
//==============================================================================

/**
 * @brief Boolean value defines for XMACRO readability
 * @details These defines make the XMACRO much more readable by clearly indicating
 * what each boolean value represents in the pin configuration.
 */

// Logic inversion
#define PIN_LOGIC_NORMAL     false    ///< Pin logic is not inverted
#define PIN_LOGIC_INVERTED   true     ///< Pin logic is inverted

// Pull resistor configuration
#define PIN_NO_PULL         false    ///< No pull resistor
#define PIN_HAS_PULL        true     ///< Has pull resistor

// Pull resistor direction (only meaningful if PIN_HAS_PULL is used)
#define PIN_PULL_DOWN       false    ///< Pull-down resistor
#define PIN_PULL_UP         true     ///< Pull-up resistor

// Output mode configuration
#define PIN_OPEN_DRAIN      false    ///< Open-drain output mode
#define PIN_PUSH_PULL       true     ///< Push-pull output mode

//==============================================================================
// MAPPING STRUCTURES
//==============================================================================

/**
 * @brief Structure containing comprehensive GPIO pin information with platform mapping.
 * @details Single source of truth for all pin configuration data.
 * 
 * IMPORTANT: This struct uses ONLY primitive types to maintain independence
 * from external type definitions. The manager/driver layer will translate
 * these primitive values to the appropriate driver enums/types.
 */
struct HfGpioMapping {
    uint8_t functional_pin;    ///< Functional pin identifier (enum value)
    uint8_t chip_type;         ///< Hardware chip identifier (enum value)
    uint8_t chip_unit;         ///< Chip unit/device number (0=first chip, 1=second chip, etc.)
    uint8_t gpio_bank;         ///< GPIO bank/port number within the chip (0=bank A, 1=bank B, etc.)
    uint8_t physical_pin;      ///< Physical pin number within the GPIO bank
    bool is_inverted;          ///< Whether pin logic is inverted
    bool has_pull;             ///< Whether pin has any pull resistor
    bool pull_is_up;           ///< If has_pull=true: true=pull-up, false=pull-down
    bool is_push_pull;         ///< Output mode: true=push-pull, false=open-drain
    uint32_t max_current_ma;   ///< Maximum current in milliamps
    uint8_t category;          ///< Pin category for registration control (enum value)
    std::string_view name;     ///< Human-readable pin name (self-documenting)
};

/**
 * @brief Structure containing ADC channel information.
 */
struct HfAdcMapping {
    uint8_t functional_channel;  ///< Functional channel identifier (enum value)
    uint8_t chip_type;          ///< Hardware chip identifier (enum value)
    uint8_t chip_unit;          ///< Chip unit/device number (0=first chip, 1=second chip, etc.)
    uint8_t adc_unit;           ///< ADC unit/bank number within the chip (0=ADC1, 1=ADC2, etc.)
    uint8_t physical_channel;   ///< Physical channel number within the ADC unit
    uint8_t resolution_bits;    ///< ADC resolution in bits
    uint32_t max_voltage_mv;    ///< Maximum voltage in millivolts
    float voltage_divider;      ///< Voltage divider ratio
    const char* description;    ///< Channel description
};

//==============================================================================
// SINGLE SOURCE OF TRUTH: XMACRO PIN DEFINITIONS
//==============================================================================

/**
 * @brief XMACRO defining all GPIO pins with complete configuration data.
 * 
 * Format: X(ENUM_NAME, STRING_NAME, CATEGORY, CHIP_TYPE, CHIP_UNIT, GPIO_BANK, PHYSICAL_PIN, LOGIC_INVERSION, HAS_PULL, PULL_DIRECTION, OUTPUT_MODE, MAX_CURRENT_MA)
 * 
 * Field descriptions:
 * - ENUM_NAME: Functional pin enum name
 * - STRING_NAME: Human-readable pin name
 * - CATEGORY: Pin category (PIN_CATEGORY_CORE, PIN_CATEGORY_COMM, PIN_CATEGORY_GPIO, PIN_CATEGORY_USER)
 * - CHIP_TYPE: Hardware chip (ESP32_INTERNAL, PCAL95555_EXPANDER, TMC9660_CONTROLLER)
 * - CHIP_UNIT: Chip unit/device number (0=first chip, 1=second chip, etc.)
 * - GPIO_BANK: GPIO bank/port number within the chip (0=bank A, 1=bank B, etc.)
 * - PHYSICAL_PIN: Physical pin number within the GPIO bank
 * - LOGIC_INVERSION: Logic inversion (PIN_LOGIC_NORMAL, PIN_LOGIC_INVERTED)
 * - HAS_PULL: Pull resistor present (PIN_NO_PULL, PIN_HAS_PULL)
 * - PULL_DIRECTION: Pull direction (PIN_PULL_DOWN, PIN_PULL_UP, ignored if HAS_PULL=PIN_NO_PULL)
 * - OUTPUT_MODE: Output mode (PIN_OPEN_DRAIN, PIN_PUSH_PULL)
 * - MAX_CURRENT_MA: Maximum current in milliamps
 * 
 * Boolean value defines for readability:
 * - PIN_LOGIC_NORMAL/PIN_LOGIC_INVERTED: Logic inversion
 * - PIN_NO_PULL/PIN_HAS_PULL: Pull resistor presence
 * - PIN_PULL_DOWN/PIN_PULL_UP: Pull resistor direction
 * - PIN_OPEN_DRAIN/PIN_PUSH_PULL: Output mode
 * 
 * This single definition generates:
 * - Functional pin enums
 * - String name tables  
 * - Category tables
 * - Complete GPIO mapping array
 * 
 * Benefits:
 * - Single source of truth (no duplication)
 * - Compile-time validation
 * - Easy to maintain and extend
 * - Self-documenting pin names
 * - Complete independence from external types
 * - Readable boolean values
 */
#define HF_FUNCTIONAL_GPIO_PIN_LIST(X) \
    /* CORE pins (system reserved - skip GPIO registration) */ \
    X(XTAL_32K_P, "CORE_XTAL_32K_P", PIN_CATEGORY_CORE, ESP32_INTERNAL, 0, 0, 0, PIN_LOGIC_NORMAL, PIN_NO_PULL, PIN_PULL_DOWN, PIN_PUSH_PULL, 0) \
    X(XTAL_32K_N, "CORE_XTAL_32K_N", PIN_CATEGORY_CORE, ESP32_INTERNAL, 0, 0, 1, PIN_LOGIC_NORMAL, PIN_NO_PULL, PIN_PULL_DOWN, PIN_PUSH_PULL, 0) \
    X(BOOT_SEL, "CORE_BOOT_SEL", PIN_CATEGORY_CORE, ESP32_INTERNAL, 0, 0, 9, PIN_LOGIC_NORMAL, PIN_NO_PULL, PIN_PULL_DOWN, PIN_PUSH_PULL, 0) \
    X(JTAG_USB_D_N, "CORE_JTAG_USB_D_N", PIN_CATEGORY_CORE, ESP32_INTERNAL, 0, 0, 12, PIN_LOGIC_NORMAL, PIN_NO_PULL, PIN_PULL_DOWN, PIN_PUSH_PULL, 0) \
    X(JTAG_USB_D_P, "CORE_JTAG_USB_D_P", PIN_CATEGORY_CORE, ESP32_INTERNAL, 0, 0, 13, PIN_LOGIC_NORMAL, PIN_NO_PULL, PIN_PULL_DOWN, PIN_PUSH_PULL, 0) \
    \
    /* COMM pins (communication - skip GPIO registration) */ \
    X(SPI2_MISO, "COMM_SPI2_MISO", PIN_CATEGORY_COMM, ESP32_INTERNAL, 0, 0, 2, PIN_LOGIC_NORMAL, PIN_HAS_PULL, PIN_PULL_UP, PIN_PUSH_PULL, 40) \
    X(SPI2_SCK, "COMM_SPI2_SCK", PIN_CATEGORY_COMM, ESP32_INTERNAL, 0, 0, 6, PIN_LOGIC_NORMAL, PIN_NO_PULL, PIN_PULL_DOWN, PIN_PUSH_PULL, 40) \
    X(SPI2_MOSI, "COMM_SPI2_MOSI", PIN_CATEGORY_COMM, ESP32_INTERNAL, 0, 0, 7, PIN_LOGIC_NORMAL, PIN_NO_PULL, PIN_PULL_DOWN, PIN_PUSH_PULL, 40) \
    X(SPI2_CS_TMC9660, "COMM_SPI2_CS_TMC9660", PIN_CATEGORY_COMM, ESP32_INTERNAL, 0, 0, 18, PIN_LOGIC_INVERTED, PIN_HAS_PULL, PIN_PULL_UP, PIN_PUSH_PULL, 40) \
    X(SPI2_CS_AS5047, "COMM_SPI2_CS_AS5047", PIN_CATEGORY_COMM, ESP32_INTERNAL, 0, 0, 20, PIN_LOGIC_INVERTED, PIN_HAS_PULL, PIN_PULL_UP, PIN_PUSH_PULL, 40) \
    X(UART_RXD, "COMM_UART_RXD", PIN_CATEGORY_COMM, ESP32_INTERNAL, 0, 0, 4, PIN_LOGIC_NORMAL, PIN_HAS_PULL, PIN_PULL_UP, PIN_PUSH_PULL, 40) \
    X(UART_TXD, "COMM_UART_TXD", PIN_CATEGORY_COMM, ESP32_INTERNAL, 0, 0, 5, PIN_LOGIC_NORMAL, PIN_NO_PULL, PIN_PULL_DOWN, PIN_PUSH_PULL, 40) \
    X(TWAI_TX, "COMM_TWAI_TX", PIN_CATEGORY_COMM, ESP32_INTERNAL, 0, 0, 14, PIN_LOGIC_NORMAL, PIN_HAS_PULL, PIN_PULL_UP, PIN_PUSH_PULL, 40) \
    X(TWAI_RX, "COMM_TWAI_RX", PIN_CATEGORY_COMM, ESP32_INTERNAL, 0, 0, 15, PIN_LOGIC_NORMAL, PIN_HAS_PULL, PIN_PULL_UP, PIN_PUSH_PULL, 40) \
    X(I2C_SDA, "COMM_I2C_SDA", PIN_CATEGORY_COMM, ESP32_INTERNAL, 0, 0, 21, PIN_LOGIC_NORMAL, PIN_HAS_PULL, PIN_PULL_UP, PIN_PUSH_PULL, 40) \
    X(I2C_SCL, "COMM_I2C_SCL", PIN_CATEGORY_COMM, ESP32_INTERNAL, 0, 0, 22, PIN_LOGIC_NORMAL, PIN_HAS_PULL, PIN_PULL_UP, PIN_PUSH_PULL, 40) \
    X(I2C_PCAL95555_INT, "COMM_I2C_PCAL95555_INT", PIN_CATEGORY_COMM, ESP32_INTERNAL, 0, 0, 23, PIN_LOGIC_INVERTED, PIN_HAS_PULL, PIN_PULL_UP, PIN_PUSH_PULL, 40) \
    X(WS2812_LED_DAT, "COMM_WS2812_LED_DAT", PIN_CATEGORY_COMM, ESP32_INTERNAL, 0, 0, 3, PIN_LOGIC_NORMAL, PIN_NO_PULL, PIN_PULL_DOWN, PIN_PUSH_PULL, 40) \
    \
    /* GPIO pins (available for GPIO operations) */ \
    X(EXT_GPIO_CS_1, "GPIO_EXT_GPIO_CS_1", PIN_CATEGORY_GPIO, ESP32_INTERNAL, 0, 0, 19, PIN_LOGIC_INVERTED, PIN_HAS_PULL, PIN_PULL_UP, PIN_PUSH_PULL, 40) \
    X(EXT_GPIO_CS_2, "GPIO_EXT_GPIO_CS_2", PIN_CATEGORY_GPIO, ESP32_INTERNAL, 0, 0, 8, PIN_LOGIC_INVERTED, PIN_HAS_PULL, PIN_PULL_UP, PIN_PUSH_PULL, 40) \
    \
    /* PCAL95555 GPIO pins (chip unit 0, bank 0) */ \
    X(PCAL_FAULT_STATUS, "GPIO_PCAL_FAULT_STATUS", PIN_CATEGORY_GPIO, PCAL95555_EXPANDER, 0, 0, 3, PIN_LOGIC_INVERTED, PIN_NO_PULL, PIN_PULL_DOWN, PIN_PUSH_PULL, 25) \
    X(PCAL_DRV_EN, "GPIO_PCAL_DRV_EN", PIN_CATEGORY_GPIO, PCAL95555_EXPANDER, 0, 0, 4, PIN_LOGIC_NORMAL, PIN_NO_PULL, PIN_PULL_DOWN, PIN_PUSH_PULL, 25) \
    X(PCAL_RST_CTRL, "GPIO_PCAL_RST_CTRL", PIN_CATEGORY_GPIO, PCAL95555_EXPANDER, 0, 0, 5, PIN_LOGIC_NORMAL, PIN_NO_PULL, PIN_PULL_DOWN, PIN_PUSH_PULL, 25) \
    X(PCAL_PWR_GOOD, "GPIO_PCAL_PWR_GOOD", PIN_CATEGORY_GPIO, PCAL95555_EXPANDER, 0, 0, 6, PIN_LOGIC_NORMAL, PIN_NO_PULL, PIN_PULL_DOWN, PIN_PUSH_PULL, 25) \
    X(PCAL_CAN_HS_STB_OP, "GPIO_PCAL_CAN_HS_STB_OP", PIN_CATEGORY_GPIO, PCAL95555_EXPANDER, 0, 0, 10, PIN_LOGIC_NORMAL, PIN_NO_PULL, PIN_PULL_DOWN, PIN_PUSH_PULL, 25) \
    X(PCAL_IMU_BOOT, "GPIO_PCAL_IMU_BOOT", PIN_CATEGORY_GPIO, PCAL95555_EXPANDER, 0, 0, 11, PIN_LOGIC_INVERTED, PIN_NO_PULL, PIN_PULL_DOWN, PIN_PUSH_PULL, 25) \
    X(PCAL_IMU_INT, "GPIO_PCAL_IMU_INT", PIN_CATEGORY_GPIO, PCAL95555_EXPANDER, 0, 0, 12, PIN_LOGIC_INVERTED, PIN_NO_PULL, PIN_PULL_DOWN, PIN_PUSH_PULL, 25) \
    X(PCAL_IMU_RST, "GPIO_PCAL_IMU_RST", PIN_CATEGORY_GPIO, PCAL95555_EXPANDER, 0, 0, 15, PIN_LOGIC_INVERTED, PIN_NO_PULL, PIN_PULL_DOWN, PIN_PUSH_PULL, 25) \
    X(PCAL_SPI_COMM_EN, "GPIO_PCAL_SPI_COMM_EN", PIN_CATEGORY_GPIO, PCAL95555_EXPANDER, 0, 0, 13, PIN_LOGIC_INVERTED, PIN_NO_PULL, PIN_PULL_DOWN, PIN_PUSH_PULL, 25) \
    X(PCAL_WAKE_CTRL, "GPIO_PCAL_WAKE_CTRL", PIN_CATEGORY_GPIO, PCAL95555_EXPANDER, 0, 0, 14, PIN_LOGIC_INVERTED, PIN_NO_PULL, PIN_PULL_DOWN, PIN_PUSH_PULL, 25) \
    \
    /* TMC9660 GPIO pins (chip unit 0, bank 0) */ \
    X(TMC_GPIO17, "GPIO_TMC_GPIO17", PIN_CATEGORY_GPIO, TMC9660_CONTROLLER, 0, 0, 17, PIN_LOGIC_NORMAL, PIN_NO_PULL, PIN_PULL_DOWN, PIN_PUSH_PULL, 25) \
    X(TMC_GPIO18, "GPIO_TMC_GPIO18", PIN_CATEGORY_GPIO, TMC9660_CONTROLLER, 0, 0, 18, PIN_LOGIC_NORMAL, PIN_NO_PULL, PIN_PULL_DOWN, PIN_PUSH_PULL, 25) \

/**
 * @brief XMACRO defining all ADC channels with complete configuration data.
 * 
 * Format: X(ENUM_NAME, STRING_NAME, CHIP_TYPE, CHIP_UNIT, ADC_UNIT, PHYSICAL_CHANNEL, RESOLUTION_BITS, MAX_VOLTAGE_MV, VOLTAGE_DIVIDER, DESCRIPTION)
 * 
 * Field descriptions:
 * - ENUM_NAME: Functional ADC channel enum name
 * - STRING_NAME: Human-readable channel name
 * - CHIP_TYPE: Hardware chip (ESP32_INTERNAL, TMC9660_CONTROLLER)
 * - CHIP_UNIT: Chip unit/device number (0=first chip, 1=second chip, etc.)
 * - ADC_UNIT: ADC unit/bank number within the chip (0=ADC1, 1=ADC2, etc.)
 * - PHYSICAL_CHANNEL: Physical channel number within the ADC unit
 * - RESOLUTION_BITS: ADC resolution in bits
 * - MAX_VOLTAGE_MV: Maximum voltage in millivolts
 * - VOLTAGE_DIVIDER: Voltage divider ratio (1.0 = no divider)
 * - DESCRIPTION: Channel description
 * 
 * This single definition generates:
 * - Functional ADC channel enums
 * - String name tables
 * - Complete ADC mapping array
 * 
 * Benefits:
 * - Single source of truth (no duplication)
 * - Compile-time validation
 * - Easy to maintain and extend
 * - Self-documenting channel names
 * - Complete independence from external types
 */
#define HF_FUNCTIONAL_ADC_CHANNEL_LIST(X) \
    /* ESP32-C6 Internal ADC Channels (ADC1) - Currently not used but available for future expansion */ \
    /* X(ESP32_ADC_CH0, "ADC_ESP32_CH0", ESP32_INTERNAL, 0, 0, 0, 12, 1100, 1.0f, "ESP32 Internal ADC Channel 0") */ \
    /* X(ESP32_ADC_CH1, "ADC_ESP32_CH1", ESP32_INTERNAL, 0, 0, 1, 12, 1100, 1.0f, "ESP32 Internal ADC Channel 1") */ \
    /* X(ESP32_ADC_CH2, "ADC_ESP32_CH2", ESP32_INTERNAL, 0, 0, 2, 12, 1100, 1.0f, "ESP32 Internal ADC Channel 2") */ \
    /* X(ESP32_ADC_CH3, "ADC_ESP32_CH3", ESP32_INTERNAL, 0, 0, 3, 12, 1100, 1.0f, "ESP32 Internal ADC Channel 3") */ \
    /* X(ESP32_ADC_CH4, "ADC_ESP32_CH4", ESP32_INTERNAL, 0, 0, 4, 12, 1100, 1.0f, "ESP32 Internal ADC Channel 4") */ \
    /* X(ESP32_ADC_CH5, "ADC_ESP32_CH5", ESP32_INTERNAL, 0, 0, 5, 12, 1100, 1.0f, "ESP32 Internal ADC Channel 5") */ \
    /* X(ESP32_ADC_CH6, "ADC_ESP32_CH6", ESP32_INTERNAL, 0, 0, 6, 12, 1100, 1.0f, "ESP32 Internal ADC Channel 6") */ \
    \
    /* TMC9660 ADC Channels - Reserved channels 0-3 for AIN inputs (only AIN3 connected to temperature sensor) */ \
    X(TMC9660_AIN0, "ADC_TMC9660_AIN0", TMC9660_CONTROLLER, 0, 0, 0, 16, 3300, 1.0f, "TMC9660 ADC Input 0 (Reserved)") \
    X(TMC9660_AIN1, "ADC_TMC9660_AIN1", TMC9660_CONTROLLER, 0, 0, 1, 16, 3300, 1.0f, "TMC9660 ADC Input 1 (Reserved)") \
    X(TMC9660_AIN2, "ADC_TMC9660_AIN2", TMC9660_CONTROLLER, 0, 0, 2, 16, 3300, 1.0f, "TMC9660 ADC Input 2 (Reserved)") \
    X(TMC9660_AIN3, "ADC_TMC9660_AIN3", TMC9660_CONTROLLER, 0, 0, 3, 16, 3300, 1.0f, "TMC9660 ADC Input 3 (Temperature Sensor)") \
    \
    /* TMC9660 Internal Monitoring Channels - Current Sensing (ADC I0-I3) */ \
    X(TMC9660_CURRENT_I0, "TMC9660_CURRENT_I0", TMC9660_CONTROLLER, 0, 0, 10, 16, 3300, 1.0f, "TMC9660 Current Sense I0") \
    X(TMC9660_CURRENT_I1, "TMC9660_CURRENT_I1", TMC9660_CONTROLLER, 0, 0, 11, 16, 3300, 1.0f, "TMC9660 Current Sense I1") \
    X(TMC9660_CURRENT_I2, "TMC9660_CURRENT_I2", TMC9660_CONTROLLER, 0, 0, 12, 16, 3300, 1.0f, "TMC9660 Current Sense I2") \
    X(TMC9660_CURRENT_I3, "TMC9660_CURRENT_I3", TMC9660_CONTROLLER, 0, 0, 13, 16, 3300, 1.0f, "TMC9660 Current Sense I3") \
    \
    /* TMC9660 Internal Monitoring Channels - Voltage Monitoring */ \
    X(TMC9660_SUPPLY_VOLTAGE, "TMC9660_SUPPLY_VOLTAGE", TMC9660_CONTROLLER, 0, 0, 20, 16, 3300, 1.0f, "TMC9660 Supply Voltage") \
    X(TMC9660_DRIVER_VOLTAGE, "TMC9660_DRIVER_VOLTAGE", TMC9660_CONTROLLER, 0, 0, 21, 16, 3300, 1.0f, "TMC9660 Driver Voltage") \
    \
    /* TMC9660 Internal Monitoring Channels - Temperature */ \
    X(TMC9660_CHIP_TEMPERATURE, "TMC9660_CHIP_TEMPERATURE", TMC9660_CONTROLLER, 0, 0, 30, 16, 3300, 1.0f, "TMC9660 Chip Temperature") \
    X(TMC9660_EXTERNAL_TEMPERATURE, "TMC9660_EXTERNAL_TEMPERATURE", TMC9660_CONTROLLER, 0, 0, 31, 16, 3300, 1.0f, "TMC9660 External Temperature") \
    \
    /* TMC9660 Internal Monitoring Channels - Motor Control Data */ \
    X(TMC9660_MOTOR_CURRENT, "TMC9660_MOTOR_CURRENT", TMC9660_CONTROLLER, 0, 0, 40, 16, 3300, 1.0f, "TMC9660 Motor Current") \
    X(TMC9660_MOTOR_VELOCITY, "TMC9660_MOTOR_VELOCITY", TMC9660_CONTROLLER, 0, 0, 41, 16, 3300, 1.0f, "TMC9660 Motor Velocity") \
    X(TMC9660_MOTOR_POSITION, "TMC9660_MOTOR_POSITION", TMC9660_CONTROLLER, 0, 0, 42, 16, 3300, 1.0f, "TMC9660 Motor Position")

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
 * @details Single source of truth for all pin configuration data.
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
 * @details Single source of truth for all ADC channel configuration data.
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

/**
 * @brief Compile-time size of GPIO mapping array.
 */
static constexpr size_t HF_GPIO_MAPPING_SIZE = sizeof(HF_GPIO_MAPPING) / sizeof(HF_GPIO_MAPPING[0]);

/**
 * @brief Compile-time size of ADC mapping array.
 */
static constexpr size_t HF_ADC_MAPPING_SIZE = sizeof(HF_ADC_MAPPING) / sizeof(HF_ADC_MAPPING[0]);

//==============================================================================
// HELPER FUNCTIONS
//==============================================================================

/**
 * @brief Convert functional pin to string name.
 * @param pin Functional pin identifier
 * @return String view of pin name
 */
inline std::string_view to_string(HfFunctionalGpioPin pin) noexcept {
    const size_t index = static_cast<size_t>(pin);
    if (index < sizeof(HfFunctionalGpioPinNames) / sizeof(HfFunctionalGpioPinNames[0])) {
        return HfFunctionalGpioPinNames[index];
    }
    return "UNKNOWN_PIN";
}

/**
 * @brief Convert functional ADC channel to string name.
 * @param channel Functional ADC channel identifier
 * @return String view of channel name
 */
inline std::string_view to_string(HfFunctionalAdcChannel channel) noexcept {
    const size_t index = static_cast<size_t>(channel);
    if (index < sizeof(HfFunctionalAdcChannelNames) / sizeof(HfFunctionalAdcChannelNames[0])) {
        return HfFunctionalAdcChannelNames[index];
    }
    return "UNKNOWN_ADC_CHANNEL";
}

/**
 * @brief Get pin category.
 * @param pin Functional pin identifier
 * @return Pin category
 */
inline HfPinCategory get_pin_category(HfFunctionalGpioPin pin) noexcept {
    const size_t index = static_cast<size_t>(pin);
    if (index < sizeof(HfFunctionalGpioPinCategories) / sizeof(HfFunctionalGpioPinCategories[0])) {
        return HfFunctionalGpioPinCategories[index];
    }
    return HfPinCategory::PIN_CATEGORY_CORE; // Safe default
}

/**
 * @brief Check if pin should be registered as GPIO.
 * @param category Pin category
 * @return true if should be registered as GPIO
 */
inline bool should_register_as_gpio(HfPinCategory category) noexcept {
    return category == HfPinCategory::PIN_CATEGORY_GPIO || 
           category == HfPinCategory::PIN_CATEGORY_USER;
}

/**
 * @brief Check if string name has reserved prefix.
 * @param name Pin name to check
 * @return true if has reserved prefix
 */
inline bool is_reserved_prefix(std::string_view name) noexcept {
    return name.starts_with("CORE_") || 
           name.starts_with("COMM_") || 
           name.starts_with("SYS_") ||
           name.starts_with("INTERNAL_");
}

//==============================================================================
// MAPPING ACCESS FUNCTIONS
//==============================================================================

/**
 * @brief Get GPIO mapping for functional pin.
 * @param functional_pin Functional pin identifier
 * @return Pointer to GPIO mapping or nullptr if not found
 */
static inline const HfGpioMapping* GetGpioMapping(HfFunctionalGpioPin functional_pin) {
    const size_t index = static_cast<size_t>(functional_pin);
    if (index < HF_GPIO_MAPPING_SIZE) {
        return &HF_GPIO_MAPPING[index];
    }
    return nullptr;
}

/**
 * @brief Get ADC mapping for functional channel.
 * @param functional_channel Functional channel identifier
 * @return Pointer to ADC mapping or nullptr if not found
 */
static inline const HfAdcMapping* GetAdcMapping(HfFunctionalAdcChannel functional_channel) {
    const size_t index = static_cast<size_t>(functional_channel);
    if (index < HF_ADC_MAPPING_SIZE) {
        return &HF_ADC_MAPPING[index];
    }
    return nullptr;
}

/**
 * @brief Check if GPIO pin is available.
 * @param functional_pin Functional pin identifier
 * @return true if pin is available
 */
static inline bool IsGpioPinAvailable(HfFunctionalGpioPin functional_pin) {
    return GetGpioMapping(functional_pin) != nullptr;
}

/**
 * @brief Check if ADC channel is available.
 * @param functional_channel Functional channel identifier
 * @return true if channel is available
 */
static inline bool IsAdcChannelAvailable(HfFunctionalAdcChannel functional_channel) {
    return GetAdcMapping(functional_channel) != nullptr;
}

//==============================================================================
// UTILITY FUNCTIONS
//==============================================================================

/**
 * @brief Get all GPIO pins of a specific category.
 * @param category Pin category to filter by
 * @param output Array to store matching pins
 * @param max_count Maximum number of pins to store
 * @return Number of pins found
 */
static inline size_t GetGpioPinsByCategory(HfPinCategory category, HfFunctionalGpioPin* output, size_t max_count) {
    size_t count = 0;
    for (size_t i = 0; i < HF_GPIO_MAPPING_SIZE && count < max_count; ++i) {
        if (HfFunctionalGpioPinCategories[i] == category) {
            output[count++] = static_cast<HfFunctionalGpioPin>(i);
        }
    }
    return count;
}

/**
 * @brief Get all GPIO pins for a specific chip type.
 * @param chip_type Chip type to filter by
 * @param output Array to store matching pins
 * @param max_count Maximum number of pins to store
 * @return Number of pins found
 */
static inline size_t GetGpioPinsByChipType(HfGpioChipType chip_type, HfFunctionalGpioPin* output, size_t max_count) {
    size_t count = 0;
    for (size_t i = 0; i < HF_GPIO_MAPPING_SIZE && count < max_count; ++i) {
        if (HF_GPIO_MAPPING[i].chip_type == static_cast<uint8_t>(chip_type)) {
            output[count++] = static_cast<HfFunctionalGpioPin>(i);
        }
    }
    return count;
}

/**
 * @brief Check if a pin requires pull-up resistor.
 * @param functional_pin Functional pin identifier
 * @return true if pin needs pull-up
 */
static inline bool RequiresPullUp(HfFunctionalGpioPin functional_pin) {
    const auto* mapping = GetGpioMapping(functional_pin);
    return mapping && mapping->has_pull && mapping->pull_is_up;
}

/**
 * @brief Check if a pin requires pull-down resistor.
 * @param functional_pin Functional pin identifier
 * @return true if pin needs pull-down
 */
static inline bool RequiresPullDown(HfFunctionalGpioPin functional_pin) {
    const auto* mapping = GetGpioMapping(functional_pin);
    return mapping && mapping->has_pull && !mapping->pull_is_up;
}

#endif // HF_FUNCTIONAL_PIN_CONFIG_VORTEX_V1_HPP 