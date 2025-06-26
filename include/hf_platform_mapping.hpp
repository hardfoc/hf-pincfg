#ifndef HF_PLATFORM_MAPPING_HPP
#define HF_PLATFORM_MAPPING_HPP

/**
 * @file hf_platform_mapping.hpp
 * @brief Platform-specific hardware resource mapping for ESP32-C6 + TMC9660 + PCAL95555.
 * 
 * This file contains the complete platform mapping that connects functional identifiers
 * to actual hardware resources. To port to a new microcontroller platform:
 * 1. Update this file with new hardware mappings
 * 2. Ensure appropriate drivers exist in hf-core-drivers
 * 3. Component handlers and application code remain unchanged
 */

#include "hf_platform_config.hpp"
#include <cstdint>

namespace HardFOC {

//==============================================================================
// HARDWARE CHIP IDENTIFIERS
//==============================================================================

/**
 * @brief Hardware chip/device identifiers for the platform.
 */
enum class HardwareChip : uint8_t {
    ESP32_INTERNAL_GPIO = 0,    ///< ESP32-C6 native GPIO pins
    ESP32_INTERNAL_ADC  = 1,    ///< ESP32-C6 internal ADC units
    PCAL95555_GPIO      = 2,    ///< PCAL95555 I2C GPIO expander
    TMC9660_ADC         = 3,    ///< TMC9660 integrated ADC
    TMC9660_GPIO        = 4,    ///< TMC9660 GPIO pins
    
    HARDWARE_CHIP_COUNT
};

//==============================================================================
// FUNCTIONAL IDENTIFIERS
//==============================================================================

/**
 * @brief Functional ADC channel identifiers.
 * These describe WHAT the channel measures, not WHERE it's located.
 */
enum class FunctionalAdcChannel : uint8_t {
    // Motor current sensing
    MOTOR_CURRENT_PHASE_A = 0,
    MOTOR_CURRENT_PHASE_B,
    MOTOR_CURRENT_PHASE_C,
    
    // System power monitoring
    SYSTEM_VOLTAGE_3V3,
    SYSTEM_VOLTAGE_5V,
    SYSTEM_VOLTAGE_12V,
    
    // Temperature sensing
    SYSTEM_TEMPERATURE_AMBIENT,
    MOTOR_TEMPERATURE,
    
    // User analog inputs
    USER_ANALOG_INPUT_1,
    USER_ANALOG_INPUT_2,
    
    // System reference
    SYSTEM_VREF_INTERNAL,
    
    FUNCTIONAL_ADC_CHANNEL_COUNT
};

/**
 * @brief Functional GPIO pin identifiers.
 * These describe WHAT the pin controls, not WHERE it's located.
 */
enum class FunctionalGpioPin : uint8_t {
    // Motor control signals
    MOTOR_ENABLE = 0,
    MOTOR_BRAKE,
    MOTOR_FAULT_STATUS,
    
    // System status LEDs
    LED_STATUS_OK,
    LED_STATUS_ERROR,
    LED_STATUS_COMM,
    
    // Communication interfaces
    COMM_CAN_TX,
    COMM_CAN_RX,
    COMM_I2C_SDA,
    COMM_I2C_SCL,
    
    // SPI interfaces
    SPI_MOTOR_CONTROLLER_CS,
    SPI_ENCODER_CS,
    SPI_MISO,
    SPI_MOSI,
    SPI_CLK,
    
    // User expansion pins
    USER_OUTPUT_1,
    USER_OUTPUT_2,
    USER_INPUT_1,
    USER_INPUT_2,
    
    // External device control
    EXTERNAL_RELAY_1,
    EXTERNAL_RELAY_2,
    
    FUNCTIONAL_GPIO_PIN_COUNT
};

//==============================================================================
// HARDWARE RESOURCE DESCRIPTORS
//==============================================================================

/**
 * @brief Hardware resource descriptor for ADC channels.
 */
struct AdcHardwareResource {
    HardwareChip chip_id;          ///< Hardware chip ID
    uint8_t channel_id;            ///< Hardware channel ID within the chip
    uint8_t resolution_bits;       ///< ADC resolution in bits
    uint32_t max_voltage_mv;       ///< Maximum input voltage in millivolts
    float voltage_divider;         ///< Voltage divider ratio (if applicable)
    bool is_differential;          ///< True if differential measurement
};

/**
 * @brief Hardware resource descriptor for GPIO pins.
 */
struct GpioHardwareResource {
    HardwareChip chip_id;          ///< Hardware chip ID
    uint8_t pin_id;                ///< Hardware pin ID within the chip
    bool is_inverted;              ///< True if logic is inverted
    bool has_pullup;               ///< True if internal pullup is available
    bool has_pulldown;             ///< True if internal pulldown is available
    uint32_t max_current_ma;       ///< Maximum current capability in milliamps
};

//==============================================================================
// PLATFORM MAPPING TABLES
//==============================================================================

/**
 * @brief Platform-specific ADC channel mapping.
 */
class AdcPlatformMapping {
public:
    /**
     * @brief Get hardware resource descriptor for a functional ADC channel.
     * @param channel Functional ADC channel identifier
     * @return Pointer to hardware resource descriptor, nullptr if not available
     */
    static const AdcHardwareResource* getHardwareResource(FunctionalAdcChannel channel);
    
    /**
     * @brief Check if a functional ADC channel is available on this platform.
     * @param channel Functional ADC channel identifier
     * @return true if channel is available, false otherwise
     */
    static bool isChannelAvailable(FunctionalAdcChannel channel);

private:
    static const AdcHardwareResource adc_mapping_table_[];
};

/**
 * @brief Platform-specific GPIO pin mapping.
 */
class GpioPlatformMapping {
public:
    /**
     * @brief Get hardware resource descriptor for a functional GPIO pin.
     * @param pin Functional GPIO pin identifier
     * @return Pointer to hardware resource descriptor, nullptr if not available
     */
    static const GpioHardwareResource* getHardwareResource(FunctionalGpioPin pin);
    
    /**
     * @brief Check if a functional GPIO pin is available on this platform.
     * @param pin Functional GPIO pin identifier
     * @return true if pin is available, false otherwise
     */
    static bool isPinAvailable(FunctionalGpioPin pin);

private:
    static const GpioHardwareResource gpio_mapping_table_[];
};

} // namespace HardFOC

#endif // HF_PLATFORM_MAPPING_HPP
