#ifndef HF_PLATFORM_CONFIG_HPP
#define HF_PLATFORM_CONFIG_HPP

/**
 * @file hf_platform_config.hpp
 * @brief Minimal platform-specific configuration and initialization.
 * 
 * This header provides the minimal platform-specific configuration required
 * for the HardFOC system. All functional pin/ADC mapping is handled by
 * hf_platform_mapping.hpp. This file only contains platform initialization
 * and basic hardware definitions.
 */

#include "driver/gpio.h"
#include <stdint.h>

namespace HardFOC {

//==============================================================================
// PLATFORM IDENTIFICATION
//==============================================================================

/**
 * @brief Current hardware platform identifier.
 */
enum class HardwarePlatform : uint8_t {
    MCU_CURRENT_PLATFORM = 0,  ///< Current MCU platform (currently ESP32-C6 + TMC9660 + PCAL95555)
    // Add other platforms here when porting
};

/// Current platform configuration
static constexpr HardwarePlatform CURRENT_PLATFORM = HardwarePlatform::MCU_CURRENT_PLATFORM;

//==============================================================================
// PLATFORM-SPECIFIC HARDWARE LIMITS
//==============================================================================

/// Maximum safe GPIO current per pin (mA)
static constexpr uint8_t MAX_GPIO_CURRENT_MA = 40;

/// ADC resolution in bits
static constexpr uint8_t ADC_RESOLUTION_BITS = 12;

/// ADC reference voltage (mV)
static constexpr uint16_t ADC_VREF_MV = 3300;

/// Internal reference voltage (mV) - platform specific
static constexpr uint16_t INTERNAL_VREF_MV = 1100;

//==============================================================================
// CRITICAL HARDWARE PIN DEFINITIONS
//==============================================================================

/**
 * @brief Critical hardware pins that should never be used as GPIO.
 * 
 * These pins are reserved for essential hardware functions and using them
 * as GPIO could cause system instability or boot failures.
 */
namespace CriticalPins {
    /// Flash interface pins - NEVER use as GPIO
    static constexpr gpio_num_t FLASH_PINS[] = {
        GPIO_NUM_6,   // Flash CLK
        GPIO_NUM_7,   // Flash DATA
        GPIO_NUM_18,  // Flash QUAD SPI
        GPIO_NUM_19   // Flash QUAD SPI
    };
    
    /// Boot strapping pins - AVOID using as GPIO
    static constexpr gpio_num_t BOOT_STRAPPING_PINS[] = {
        GPIO_NUM_4,   // MTMS/JTAG
        GPIO_NUM_5,   // MTDI/JTAG
        GPIO_NUM_8,   // Boot mode
        GPIO_NUM_9,   // Download mode
        GPIO_NUM_15   // JTAG select
    };
    
    /// USB pins - Reserved for USB functionality
    static constexpr gpio_num_t USB_PINS[] = {
        GPIO_NUM_12,  // USB D-
        GPIO_NUM_13   // USB D+
    };
}

//==============================================================================
// HARDWARE CONFIGURATION PINS
//==============================================================================

/**
 * @brief Hardware-specific pins used by the platform mapping.
 * 
 * These are the actual physical pins used by various peripherals.
 * The functional mapping translates functional names to these pins.
 */
namespace HardwarePins {
    // WS2812 LED control pin
    static constexpr gpio_num_t WS2812_LED_PIN = GPIO_NUM_10;
    
    // I2C communication pins
    static constexpr gpio_num_t I2C_SDA_PIN = GPIO_NUM_22;
    static constexpr gpio_num_t I2C_SCL_PIN = GPIO_NUM_23;
    
    // CAN/TWAI communication pins
    static constexpr gpio_num_t TWAI_TX_PIN = GPIO_NUM_21;
    static constexpr gpio_num_t TWAI_RX_PIN = GPIO_NUM_20;
    
    // SPI pins (managed by SPI driver)
    static constexpr gpio_num_t SPI_MISO_PIN = GPIO_NUM_2;
    static constexpr gpio_num_t SPI_MOSI_PIN = GPIO_NUM_7;
    static constexpr gpio_num_t SPI_CLK_PIN = GPIO_NUM_6;
    static constexpr gpio_num_t SPI_CS_TMC_PIN = GPIO_NUM_18;
    static constexpr gpio_num_t SPI_CS_AS5047_PIN = GPIO_NUM_20;
    
    // Safe GPIO pins for general use
    static constexpr gpio_num_t SAFE_GPIO_PINS[] = {
        GPIO_NUM_0,   // Safe
        GPIO_NUM_1,   // Safe  
        GPIO_NUM_3,   // Safe
        GPIO_NUM_10,  // Safe
        GPIO_NUM_11,  // Safe
        GPIO_NUM_20,  // Safe
        GPIO_NUM_21,  // Safe
        GPIO_NUM_22,  // Safe
        GPIO_NUM_23   // Safe
    };
}

//==============================================================================
// PLATFORM INITIALIZATION
//==============================================================================

/**
 * @brief Initialize platform-specific hardware configuration.
 * 
 * This function performs the minimal hardware initialization required
 * for the platform. Individual drivers handle their own initialization
 * when instantiated, so this function is mainly for system-level setup.
 */
inline void init_platform_hardware(void)
{
    // Individual MCU drivers (McuGpio, McuI2c, etc.) handle their own
    // initialization when they are instantiated. No global init needed here.
}

/**
 * @brief Legacy function name for compatibility.
 * @deprecated Use init_platform_hardware() instead.
 */
inline void init_mcu_pinconfig(void)
{
    init_platform_hardware();
}

} // namespace HardFOC

#endif // HF_PLATFORM_CONFIG_HPP
