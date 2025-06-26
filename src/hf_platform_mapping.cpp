#include "hf_platform_mapping.hpp"

/**
 * @file hf_platform_mapping.cpp
 * @brief Implementation of platform-specific hardware resource mappings.
 * 
 * This file contains the actual mapping tables that connect functional identifiers
 * to hardware resources for the ESP32-C6 + TMC9660 + PCAL95555 platform.
 */

namespace HardFOC {

// ADC Hardware Resource Mapping Table
const AdcHardwareResource AdcPlatformMapping::adc_mapping_table_[] = {
    // Motor current sensing (TMC9660 integrated ADC)
    [static_cast<uint8_t>(FunctionalAdcChannel::MOTOR_CURRENT_PHASE_A)] = {
        .chip_id = HardwareChip::TMC9660_ADC,
        .channel_id = 0,  // TMC9660 current sense A
        .resolution_bits = 12,
        .max_voltage_mv = 3300,
        .voltage_divider = 1.0f,
        .is_differential = false
    },
    [static_cast<uint8_t>(FunctionalAdcChannel::MOTOR_CURRENT_PHASE_B)] = {
        .chip_id = HardwareChip::TMC9660_ADC,
        .channel_id = 1,  // TMC9660 current sense B
        .resolution_bits = 12,
        .max_voltage_mv = 3300,
        .voltage_divider = 1.0f,
        .is_differential = false
    },
    [static_cast<uint8_t>(FunctionalAdcChannel::MOTOR_CURRENT_PHASE_C)] = {
        .chip_id = HardwareChip::TMC9660_ADC,
        .channel_id = 2,  // TMC9660 current sense C
        .resolution_bits = 12,
        .max_voltage_mv = 3300,
        .voltage_divider = 1.0f,
        .is_differential = false
    },
      // System voltage monitoring (ESP32-C6 internal ADC)
    [static_cast<uint8_t>(FunctionalAdcChannel::SYSTEM_VOLTAGE_3V3)] = {
        .chip_id = HardwareChip::ESP32_INTERNAL_ADC,
        .channel_id = 0,  // GPIO0 -> ADC1_CHANNEL_0 (from HardwarePins::SAFE_GPIO_PINS[0])
        .resolution_bits = 12,
        .max_voltage_mv = 3300,
        .voltage_divider = 1.0f,
        .is_differential = false
    },
    [static_cast<uint8_t>(FunctionalAdcChannel::SYSTEM_VOLTAGE_5V)] = {
        .chip_id = HardwareChip::ESP32_INTERNAL_ADC,
        .channel_id = 1,  // GPIO1 -> ADC1_CHANNEL_1
        .resolution_bits = 12,
        .max_voltage_mv = 3300,
        .voltage_divider = 0.66f,  // 5V -> 3.3V divider
        .is_differential = false
    },
    [static_cast<uint8_t>(FunctionalAdcChannel::SYSTEM_VOLTAGE_12V)] = {
        .chip_id = HardwareChip::ESP32_INTERNAL_ADC,
        .channel_id = 2,  // GPIO2 -> ADC1_CHANNEL_2
        .resolution_bits = 12,
        .max_voltage_mv = 3300,
        .voltage_divider = 0.275f,  // 12V -> 3.3V divider
        .is_differential = false
    },
    
    // Temperature sensing
    [static_cast<uint8_t>(FunctionalAdcChannel::SYSTEM_TEMPERATURE_AMBIENT)] = {
        .chip_id = HardwareChip::ESP32_INTERNAL_ADC,
        .channel_id = 3,  // GPIO3 -> ADC1_CHANNEL_3
        .resolution_bits = 12,
        .max_voltage_mv = 3300,
        .voltage_divider = 1.0f,
        .is_differential = false
    },
    [static_cast<uint8_t>(FunctionalAdcChannel::MOTOR_TEMPERATURE)] = {
        .chip_id = HardwareChip::ESP32_INTERNAL_ADC,
        .channel_id = 4,  // GPIO4 -> ADC1_CHANNEL_4
        .resolution_bits = 12,
        .max_voltage_mv = 3300,
        .voltage_divider = 1.0f,
        .is_differential = false
    },
    
    // User analog inputs
    [static_cast<uint8_t>(FunctionalAdcChannel::USER_ANALOG_INPUT_1)] = {
        .chip_id = HardwareChip::ESP32_INTERNAL_ADC,
        .channel_id = 5,  // GPIO5 -> ADC1_CHANNEL_5
        .resolution_bits = 12,
        .max_voltage_mv = 3300,
        .voltage_divider = 1.0f,
        .is_differential = false
    },
    [static_cast<uint8_t>(FunctionalAdcChannel::USER_ANALOG_INPUT_2)] = {
        .chip_id = HardwareChip::ESP32_INTERNAL_ADC,
        .channel_id = 6,  // GPIO6 -> ADC1_CHANNEL_6
        .resolution_bits = 12,
        .max_voltage_mv = 3300,
        .voltage_divider = 1.0f,
        .is_differential = false
    },
    
    // System reference
    [static_cast<uint8_t>(FunctionalAdcChannel::SYSTEM_VREF_INTERNAL)] = {
        .chip_id = HardwareChip::ESP32_INTERNAL_ADC,
        .channel_id = 255,  // Internal reference
        .resolution_bits = 12,
        .max_voltage_mv = 1100,  // ESP32-C6 internal reference
        .voltage_divider = 1.0f,
        .is_differential = false
    }
};

// GPIO Hardware Resource Mapping Table
const GpioHardwareResource GpioPlatformMapping::gpio_mapping_table_[] = {
    // Motor control signals (PCAL95555 GPIO expander)
    [static_cast<uint8_t>(FunctionalGpioPin::MOTOR_ENABLE)] = {
        .chip_id = HardwareChip::PCAL95555_GPIO,
        .pin_id = 0,  // PCAL95555 pin 0
        .is_inverted = false,
        .has_pullup = true,
        .has_pulldown = false,
        .max_current_ma = 25
    },
    [static_cast<uint8_t>(FunctionalGpioPin::MOTOR_BRAKE)] = {
        .chip_id = HardwareChip::PCAL95555_GPIO,
        .pin_id = 2,  // PCAL95555 pin 2
        .is_inverted = false,
        .has_pullup = true,
        .has_pulldown = false,
        .max_current_ma = 25
    },
    [static_cast<uint8_t>(FunctionalGpioPin::MOTOR_FAULT_STATUS)] = {
        .chip_id = HardwareChip::PCAL95555_GPIO,
        .pin_id = 4,  // PCAL95555 pin 4
        .is_inverted = true,  // Active low fault signal
        .has_pullup = true,
        .has_pulldown = false,
        .max_current_ma = 25
    },
    
    // System status LEDs (PCAL95555 GPIO expander)
    [static_cast<uint8_t>(FunctionalGpioPin::LED_STATUS_OK)] = {
        .chip_id = HardwareChip::PCAL95555_GPIO,
        .pin_id = 6,  // PCAL95555 pin 6 (Green LED)
        .is_inverted = false,
        .has_pullup = false,
        .has_pulldown = false,
        .max_current_ma = 25
    },
    [static_cast<uint8_t>(FunctionalGpioPin::LED_STATUS_ERROR)] = {
        .chip_id = HardwareChip::PCAL95555_GPIO,
        .pin_id = 8,  // PCAL95555 pin 8 (Error LED)
        .is_inverted = false,
        .has_pullup = false,
        .has_pulldown = false,
        .max_current_ma = 25
    },
    [static_cast<uint8_t>(FunctionalGpioPin::LED_STATUS_COMM)] = {
        .chip_id = HardwareChip::PCAL95555_GPIO,
        .pin_id = 9,  // PCAL95555 pin 9 (Comm LED)
        .is_inverted = false,
        .has_pullup = false,
        .has_pulldown = false,
        .max_current_ma = 25
    },
    
    // Communication interfaces (ESP32-C6 native GPIO)
    [static_cast<uint8_t>(FunctionalGpioPin::COMM_CAN_TX)] = {
        .chip_id = HardwareChip::ESP32_INTERNAL_GPIO,
        .pin_id = 19,  // GPIO19
        .is_inverted = false,
        .has_pullup = true,
        .has_pulldown = false,
        .max_current_ma = 40
    },
    [static_cast<uint8_t>(FunctionalGpioPin::COMM_CAN_RX)] = {
        .chip_id = HardwareChip::ESP32_INTERNAL_GPIO,
        .pin_id = 15,  // GPIO15
        .is_inverted = false,
        .has_pullup = true,
        .has_pulldown = false,
        .max_current_ma = 40
    },
    [static_cast<uint8_t>(FunctionalGpioPin::COMM_I2C_SDA)] = {
        .chip_id = HardwareChip::ESP32_INTERNAL_GPIO,
        .pin_id = 22,  // GPIO22
        .is_inverted = false,
        .has_pullup = true,
        .has_pulldown = false,
        .max_current_ma = 40
    },
    [static_cast<uint8_t>(FunctionalGpioPin::COMM_I2C_SCL)] = {
        .chip_id = HardwareChip::ESP32_INTERNAL_GPIO,
        .pin_id = 23,  // GPIO23
        .is_inverted = false,
        .has_pullup = true,
        .has_pulldown = false,
        .max_current_ma = 40
    },
    
    // SPI interfaces (ESP32-C6 native GPIO)
    [static_cast<uint8_t>(FunctionalGpioPin::SPI_MOTOR_CONTROLLER_CS)] = {
        .chip_id = HardwareChip::ESP32_INTERNAL_GPIO,
        .pin_id = 18,  // GPIO18 (TMC9660 CS)
        .is_inverted = false,
        .has_pullup = true,
        .has_pulldown = false,
        .max_current_ma = 40
    },
    [static_cast<uint8_t>(FunctionalGpioPin::SPI_ENCODER_CS)] = {
        .chip_id = HardwareChip::ESP32_INTERNAL_GPIO,
        .pin_id = 20,  // GPIO20 (AS5047 CS)
        .is_inverted = false,
        .has_pullup = true,
        .has_pulldown = false,
        .max_current_ma = 40
    },
    [static_cast<uint8_t>(FunctionalGpioPin::SPI_MISO)] = {
        .chip_id = HardwareChip::ESP32_INTERNAL_GPIO,
        .pin_id = 2,   // GPIO2
        .is_inverted = false,
        .has_pullup = true,
        .has_pulldown = false,
        .max_current_ma = 40
    },
    [static_cast<uint8_t>(FunctionalGpioPin::SPI_MOSI)] = {
        .chip_id = HardwareChip::ESP32_INTERNAL_GPIO,
        .pin_id = 7,   // GPIO7
        .is_inverted = false,
        .has_pullup = false,
        .has_pulldown = false,
        .max_current_ma = 40
    },
    [static_cast<uint8_t>(FunctionalGpioPin::SPI_CLK)] = {
        .chip_id = HardwareChip::ESP32_INTERNAL_GPIO,
        .pin_id = 6,   // GPIO6
        .is_inverted = false,
        .has_pullup = false,
        .has_pulldown = false,
        .max_current_ma = 40
    },
    
    // User expansion pins (PCAL95555 GPIO expander)
    [static_cast<uint8_t>(FunctionalGpioPin::USER_OUTPUT_1)] = {
        .chip_id = HardwareChip::PCAL95555_GPIO,
        .pin_id = 12,  // PCAL95555 pin 12
        .is_inverted = false,
        .has_pullup = false,
        .has_pulldown = false,
        .max_current_ma = 25
    },
    [static_cast<uint8_t>(FunctionalGpioPin::USER_OUTPUT_2)] = {
        .chip_id = HardwareChip::PCAL95555_GPIO,
        .pin_id = 13,  // PCAL95555 pin 13
        .is_inverted = false,
        .has_pullup = false,
        .has_pulldown = false,
        .max_current_ma = 25
    },
    [static_cast<uint8_t>(FunctionalGpioPin::USER_INPUT_1)] = {
        .chip_id = HardwareChip::PCAL95555_GPIO,
        .pin_id = 14,  // PCAL95555 pin 14
        .is_inverted = false,
        .has_pullup = true,
        .has_pulldown = false,
        .max_current_ma = 25
    },
    [static_cast<uint8_t>(FunctionalGpioPin::USER_INPUT_2)] = {
        .chip_id = HardwareChip::PCAL95555_GPIO,
        .pin_id = 15,  // PCAL95555 pin 15
        .is_inverted = false,
        .has_pullup = true,
        .has_pulldown = false,
        .max_current_ma = 25
    },
    
    // External device control (PCAL95555 GPIO expander)
    [static_cast<uint8_t>(FunctionalGpioPin::EXTERNAL_RELAY_1)] = {
        .chip_id = HardwareChip::PCAL95555_GPIO,
        .pin_id = 10,  // PCAL95555 pin 10
        .is_inverted = false,
        .has_pullup = false,
        .has_pulldown = false,
        .max_current_ma = 25
    },
    [static_cast<uint8_t>(FunctionalGpioPin::EXTERNAL_RELAY_2)] = {
        .chip_id = HardwareChip::PCAL95555_GPIO,
        .pin_id = 11,  // PCAL95555 pin 11
        .is_inverted = false,
        .has_pullup = false,
        .has_pulldown = false,
        .max_current_ma = 25
    }
};

// Implementation of AdcPlatformMapping methods
const AdcHardwareResource* AdcPlatformMapping::getHardwareResource(FunctionalAdcChannel channel) {
    const auto index = static_cast<uint8_t>(channel);
    const auto max_index = static_cast<uint8_t>(FunctionalAdcChannel::FUNCTIONAL_ADC_CHANNEL_COUNT);
    
    return (index < max_index) ? &adc_mapping_table_[index] : nullptr;
}

bool AdcPlatformMapping::isChannelAvailable(FunctionalAdcChannel channel) {
    return getHardwareResource(channel) != nullptr;
}

// Implementation of GpioPlatformMapping methods
const GpioHardwareResource* GpioPlatformMapping::getHardwareResource(FunctionalGpioPin pin) {
    const auto index = static_cast<uint8_t>(pin);
    const auto max_index = static_cast<uint8_t>(FunctionalGpioPin::FUNCTIONAL_GPIO_PIN_COUNT);
    
    return (index < max_index) ? &gpio_mapping_table_[index] : nullptr;
}

bool GpioPlatformMapping::isPinAvailable(FunctionalGpioPin pin) {
    return getHardwareResource(pin) != nullptr;
}

} // namespace HardFOC
