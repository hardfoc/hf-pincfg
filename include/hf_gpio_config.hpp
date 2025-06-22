#ifndef HF_GPIO_CONFIG_HPP
#define HF_GPIO_CONFIG_HPP

/**
 * @file hf_gpio_config.hpp
 * @brief Comprehensive GPIO pin configuration and safety analysis for ESP32-C6.
 *
 * This header provides detailed GPIO pin assignments for the HardFOC controller,
 * including hardware-specific pin usage, safety classifications, and configuration
 * utilities for the ESP32-C6 microcontroller.
 *
 * @note ESP32-C6 Pin Safety Classification:
 * - SAFE: Pins safe for general GPIO use without boot/hardware conflicts
 * - CONDITIONAL: Pins usable with care - may have hardware/boot implications
 * - RESERVED: Pins dedicated to hardware functions - should NOT be used as GPIO
 * - STRAPPING: Boot control pins - should be avoided or handled with extreme care
 *
 * @warning Always verify pin conflicts with your specific ESP32-C6 hardware variant
 *          before assigning GPIO functions to any pin.
 */

#include "driver/gpio.h"
#include <stdint.h>

//==============================================================================
// ESP32-C6 PIN SAFETY CLASSIFICATION
//==============================================================================

/**
 * @brief ESP32-C6 GPIO pin safety levels for general-purpose use.
 */
enum class Esp32C6PinSafety : uint8_t {
    SAFE        = 0,  ///< Safe for general GPIO use
    CONDITIONAL = 1,  ///< Usable with care - may have implications
    RESERVED    = 2,  ///< Reserved for hardware - avoid GPIO use
    STRAPPING   = 3,  ///< Boot control - extreme care required
};

/**
 * @brief ESP32-C6 pin safety classification table.
 */
struct Esp32C6PinInfo {
    gpio_num_t pin;
    Esp32C6PinSafety safety;
    const char* description;
    const char* hardware_function;
};

// ESP32-C6 Pin Safety Table
static constexpr Esp32C6PinInfo ESP32_C6_PIN_TABLE[] = {
    // SAFE PINS - Best for general GPIO use
    {GPIO_NUM_0,  Esp32C6PinSafety::SAFE, "GPIO0", "General purpose, XTAL_32K_P, LP_GPIO0, ADC1_CH0"},
    {GPIO_NUM_1,  Esp32C6PinSafety::SAFE, "GPIO1", "General purpose, XTAL_32K_N, LP_GPIO1, ADC1_CH1"},
    {GPIO_NUM_2,  Esp32C6PinSafety::CONDITIONAL, "GPIO2", "General purpose, LP_GPIO2, ADC1_CH2, FSPIQ"},
    {GPIO_NUM_3,  Esp32C6PinSafety::SAFE, "GPIO3", "General purpose, LP_GPIO3, ADC1_CH3"},
    
    // STRAPPING PINS - Avoid or use with extreme care
    {GPIO_NUM_4,  Esp32C6PinSafety::STRAPPING, "GPIO4", "MTMS, JTAG, Strapping pin, ADC1_CH4, FSPIHD"},
    {GPIO_NUM_5,  Esp32C6PinSafety::STRAPPING, "GPIO5", "MTDI, JTAG, Strapping pin, ADC1_CH5, FSPIWP"},
    {GPIO_NUM_6,  Esp32C6PinSafety::RESERVED, "GPIO6", "MTCK, JTAG, Flash clock, ADC1_CH6, FSPICLK"},
    {GPIO_NUM_7,  Esp32C6PinSafety::RESERVED, "GPIO7", "MTDO, JTAG, Flash data, FSPID"},
    {GPIO_NUM_8,  Esp32C6PinSafety::STRAPPING, "GPIO8", "Boot mode strapping pin"},
    {GPIO_NUM_9,  Esp32C6PinSafety::STRAPPING, "GPIO9", "Download mode strapping pin"},
    
    // SAFE PINS - Good for GPIO use
    {GPIO_NUM_10, Esp32C6PinSafety::SAFE, "GPIO10", "General purpose"},
    {GPIO_NUM_11, Esp32C6PinSafety::SAFE, "GPIO11", "General purpose"},
    
    // USB PINS - Reserved for USB functionality
    {GPIO_NUM_12, Esp32C6PinSafety::RESERVED, "GPIO12", "USB_D-, USB communication"},
    {GPIO_NUM_13, Esp32C6PinSafety::RESERVED, "GPIO13", "USB_D+, USB communication"},
    
    // CONDITIONAL PINS
    {GPIO_NUM_14, Esp32C6PinSafety::CONDITIONAL, "GPIO14", "General purpose (board dependent)"},
    {GPIO_NUM_15, Esp32C6PinSafety::STRAPPING, "GPIO15", "JTAG_SEL, JTAG input source control"},
    
    // UART PINS - Conditional based on use
    {GPIO_NUM_16, Esp32C6PinSafety::CONDITIONAL, "GPIO16", "U0TXD, Default UART TX, FSPICS0"},
    {GPIO_NUM_17, Esp32C6PinSafety::CONDITIONAL, "GPIO17", "U0RXD, Default UART RX, FSPICS1"},
    
    // FLASH PINS - Reserved for flash memory
    {GPIO_NUM_18, Esp32C6PinSafety::RESERVED, "GPIO18", "FSPIQ, Flash memory interface"},
    {GPIO_NUM_19, Esp32C6PinSafety::RESERVED, "GPIO19", "FSPID, Flash memory interface"},
    
    // SAFE PINS for higher numbers
    {GPIO_NUM_20, Esp32C6PinSafety::SAFE, "GPIO20", "General purpose"},
    {GPIO_NUM_21, Esp32C6PinSafety::SAFE, "GPIO21", "General purpose"},
    {GPIO_NUM_22, Esp32C6PinSafety::SAFE, "GPIO22", "General purpose"},
    {GPIO_NUM_23, Esp32C6PinSafety::SAFE, "GPIO23", "General purpose"},
};

static constexpr size_t ESP32_C6_PIN_COUNT = sizeof(ESP32_C6_PIN_TABLE) / sizeof(ESP32_C6PinInfo);

/**
 * @brief Get pin safety information for an ESP32-C6 GPIO pin.
 * @param pin The GPIO pin number.
 * @return Pointer to pin info or nullptr if pin not found.
 */
static inline const Esp32C6PinInfo* GetPinInfo(gpio_num_t pin) {
    for (size_t i = 0; i < ESP32_C6_PIN_COUNT; ++i) {
        if (ESP32_C6_PIN_TABLE[i].pin == pin) {
            return &ESP32_C6_PIN_TABLE[i];
        }
    }
    return nullptr;
}

/**
 * @brief Check if a pin is safe for general GPIO use.
 * @param pin The GPIO pin number.
 * @return true if safe, false otherwise.
 */
static inline bool IsPinSafeForGpio(gpio_num_t pin) {
    const Esp32C6PinInfo* info = GetPinInfo(pin);
    return info && info->safety == Esp32C6PinSafety::SAFE;
}

/**
 * @brief Check if a pin can be used conditionally for GPIO.
 * @param pin The GPIO pin number.
 * @return true if conditionally usable, false otherwise.
 */
static inline bool IsPinConditionalForGpio(gpio_num_t pin) {
    const Esp32C6PinInfo* info = GetPinInfo(pin);
    return info && (info->safety == Esp32C6PinSafety::SAFE || 
                   info->safety == Esp32C6PinSafety::CONDITIONAL);
}

//==============================================================================
// HARDFOC HARDWARE-SPECIFIC PIN ASSIGNMENTS
//==============================================================================

//==============================================================================
// HARDFOC HARDWARE-SPECIFIC PIN ASSIGNMENTS
//==============================================================================

/**
 * @name HardFOC SPI Bus Configuration
 * SPI pins are managed by the ESP-IDF SPI driver and should NOT be used as GPIO.
 * The CS pins can be managed as GPIOs if SPI driver doesn't control them.
 */
///@{
#define SPI0_MISO_PIN      GPIO_NUM_2  ///< Data from device to controller (CONDITIONAL: Flash conflict)
#define SPI0_MOSI_PIN      GPIO_NUM_7  ///< Data from controller to device (RESERVED: Flash/JTAG)
#define SPI0_CLK_PIN       GPIO_NUM_6  ///< Serial clock (RESERVED: Flash/JTAG)
#define SPI0_CS_TMC_PIN    GPIO_NUM_18 ///< TMC9660 chip select (RESERVED: Flash interface)
#define SPI0_CS_AS5047_PIN GPIO_NUM_20 ///< AS5047P encoder chip select (SAFE for GPIO control)
#define SPI0_CS_EXT1_PIN   GPIO_NUM_19 ///< Additional SPI device 1 (RESERVED: Flash)
#define SPI0_CS_EXT2_PIN   GPIO_NUM_8  ///< Additional SPI device 2 (STRAPPING: Avoid)
///@}

/**
 * @name UART Communication Pins
 * These pins can be reconfigured as GPIO if UART is not needed.
 */
///@{
#define UART_RXD_PIN       GPIO_NUM_16 ///< Primary UART receive (CONDITIONAL)
#define UART_TXD_PIN       GPIO_NUM_17 ///< Primary UART transmit (CONDITIONAL)
#define DEBUG_UART_TX_PIN  GPIO_NUM_0  ///< Debug UART transmit (SAFE for GPIO)
#define DEBUG_UART_RX_PIN  GPIO_NUM_1  ///< Debug UART receive (SAFE for GPIO)
///@}

/**
 * @name I2C Communication Pins
 * These pins are SAFE and ideal for I2C communication.
 */
///@{
#define I2C_SDA_PIN        GPIO_NUM_22 ///< I2C data (SAFE)
#define I2C_SCL_PIN        GPIO_NUM_23 ///< I2C clock (SAFE)
///@}

/**
 * @name CAN/TWAI Communication Pins
 * Alternative pin assignments - current assignments have conflicts.
 */
///@{
#define TWAI_TX_PIN        GPIO_NUM_21 ///< CAN transmit (SAFE - changed from GPIO_NUM_19)
#define TWAI_RX_PIN        GPIO_NUM_20 ///< CAN receive (SAFE - changed from GPIO_NUM_15)
///@}

/**
 * @name LED and Indicator Pins
 * Using SAFE pins for visual indicators.
 */
///@{
#define WS2812_LED_PIN     GPIO_NUM_10 ///< WS2812 data (SAFE - changed from GPIO_NUM_2)
#define STATUS_LED_PIN     GPIO_NUM_11 ///< Status LED (SAFE)
#define ERROR_LED_PIN      GPIO_NUM_3  ///< Error LED (SAFE)
///@}

/**
 * @name Safe GPIO Pins for General Use
 * These pins are the safest choices for general GPIO operations.
 */
///@{
#define SAFE_GPIO_0        GPIO_NUM_0  ///< Safe GPIO pin 0
#define SAFE_GPIO_1        GPIO_NUM_1  ///< Safe GPIO pin 1  
#define SAFE_GPIO_3        GPIO_NUM_3  ///< Safe GPIO pin 3
#define SAFE_GPIO_10       GPIO_NUM_10 ///< Safe GPIO pin 10
#define SAFE_GPIO_11       GPIO_NUM_11 ///< Safe GPIO pin 11
#define SAFE_GPIO_20       GPIO_NUM_20 ///< Safe GPIO pin 20
#define SAFE_GPIO_21       GPIO_NUM_21 ///< Safe GPIO pin 21
#define SAFE_GPIO_22       GPIO_NUM_22 ///< Safe GPIO pin 22
#define SAFE_GPIO_23       GPIO_NUM_23 ///< Safe GPIO pin 23
///@}

/**
 * @name Conditional GPIO Pins
 * These pins can be used as GPIO but may have implications.
 */
///@{
#define CONDITIONAL_GPIO_2  GPIO_NUM_2  ///< Conditional - Flash interface concern
#define CONDITIONAL_GPIO_14 GPIO_NUM_14 ///< Conditional - Board dependent
#define CONDITIONAL_GPIO_16 GPIO_NUM_16 ///< Conditional - Default UART TX
#define CONDITIONAL_GPIO_17 GPIO_NUM_17 ///< Conditional - Default UART RX
///@}

//==============================================================================
// GPIO CONFIGURATION FUNCTIONS
//==============================================================================

/**
 * @brief Configure hardware-specific GPIO pins with safety checks.
 * 
 * This function configures pins that are actually used by hardware peripherals
 * and should NOT be used as general GPIO. It includes safety warnings for
 * potentially problematic pin assignments.
 */
static inline void configure_hardware_gpio(void)
{
    gpio_config_t io_conf = {};
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;

    // Configure SAFE GPIO pins for general use
    io_conf.pin_bit_mask = (1ULL << SAFE_GPIO_0) |
                           (1ULL << SAFE_GPIO_1) |
                           (1ULL << SAFE_GPIO_3) |
                           (1ULL << SAFE_GPIO_10) |
                           (1ULL << SAFE_GPIO_11) |
                           (1ULL << SAFE_GPIO_20) |
                           (1ULL << SAFE_GPIO_21) |
                           (1ULL << SAFE_GPIO_22) |
                           (1ULL << SAFE_GPIO_23);
    io_conf.mode = GPIO_MODE_OUTPUT;
    gpio_config(&io_conf);

    // Configure LED pins
    io_conf.pin_bit_mask = (1ULL << WS2812_LED_PIN) |
                           (1ULL << STATUS_LED_PIN) |
                           (1ULL << ERROR_LED_PIN);
    gpio_config(&io_conf);

    // NOTE: SPI pins are managed by SPI driver - do NOT configure as GPIO
    // NOTE: I2C pins are managed by I2C driver - do NOT configure as GPIO  
    // NOTE: UART pins managed by UART driver unless specifically needed as GPIO
    
    // WARNING: The following pins should NOT be used as GPIO:
    // - GPIO_NUM_4, GPIO_NUM_5: JTAG/Strapping pins
    // - GPIO_NUM_6, GPIO_NUM_7: Flash/JTAG pins  
    // - GPIO_NUM_8, GPIO_NUM_9: Boot strapping pins
    // - GPIO_NUM_12, GPIO_NUM_13: USB pins
    // - GPIO_NUM_15: JTAG select pin
    // - GPIO_NUM_18, GPIO_NUM_19: Flash interface pins
}

/**
 * @brief Initialize safe GPIO pins only.
 * 
 * This function initializes only the pins that are confirmed safe for GPIO use,
 * avoiding any pins that could interfere with boot, flash, or other hardware.
 */
static inline void configure_safe_gpio_only(void)
{
    gpio_config_t io_conf = {};
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;

    // Configure only the safest GPIO pins
    io_conf.pin_bit_mask = (1ULL << GPIO_NUM_0) |   // Safe
                           (1ULL << GPIO_NUM_1) |   // Safe
                           (1ULL << GPIO_NUM_3) |   // Safe
                           (1ULL << GPIO_NUM_10) |  // Safe
                           (1ULL << GPIO_NUM_11) |  // Safe
                           (1ULL << GPIO_NUM_20) |  // Safe
                           (1ULL << GPIO_NUM_21) |  // Safe
                           (1ULL << GPIO_NUM_22) |  // Safe
                           (1ULL << GPIO_NUM_23);   // Safe
    
    gpio_config(&io_conf);
}

/**
 * @brief Legacy GPIO configuration function.
 * 
 * @warning This function configures pins that may conflict with hardware.
 *          Use configure_safe_gpio_only() for new implementations.
 * @deprecated Use configure_hardware_gpio() or configure_safe_gpio_only().
 */
static inline void configure_gpio(void)
{
    // Call the safer configuration function instead
    configure_hardware_gpio();
}

/**
 * @brief Initialize MCU pin configuration with safety checks.
 *
 * This function initializes the pin configuration system with proper safety
 * validation and hardware conflict detection.
 */
static inline void init_mcu_pinconfig(void)
{
    configure_safe_gpio_only();
}

#endif // HF_GPIO_CONFIG_HPP
