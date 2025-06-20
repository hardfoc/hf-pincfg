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
enum class Pcal95555Chip1Pin : uint8_t {
    // TMC9660 GPIO pins
    TMC_GPIO17          = 0,  ///< P0_0: TMC9660 GPIO17 pin
    TMC_GPIO18          = 1,  ///< P0_1: TMC9660 GPIO18 pin
    PCAL_GND_1          = 2,  ///< P0_2: Ground connection (not usable)
    
    // TMC9660 control and status pins
    TMC_nFAULT_STATUS   = 3,  ///< P0_3: TMC9660 fault status (active low)
    TMC_DRV_EN          = 4,  ///< P0_4: TMC9660 driver enable
    TMC_RST_CTRL        = 5,  ///< P0_5: TMC9660 reset control
    
    // Status LEDs
    LED_STATUS_GREEN    = 6,  ///< Green status LED
    LED_STATUS_RED      = 7,  ///< Red status LED
    LED_ERROR           = 8,  ///< Error indicator LED
    LED_COMM            = 9,  ///< Communication activity LED
    
    // CAN bus control
    CAN_HS_STB_OP       = 10, ///< P1_2: CAN high-speed standby output
    
    // IMU control pins
    IMU_nBOOT           = 11, ///< P1_3: IMU boot pin (active low)
    IMU_nINT            = 12, ///< P1_4: IMU interrupt (active low)
    
    // TMC9660 communication control
    TMC_SPI_COMM_nEN    = 13, ///< P1_5: TMC9660 SPI communication enable (active low)
    TMC_nWAKE_CTRL      = 14, ///< P1_6: TMC9660 wake control (active low)
    
    // IMU flow control
    IMU_nRTS            = 15, ///< P1_7: IMU ready to send (active low)
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
        case Pcal95555Chip1Pin::TMC_GPIO17: return "TMC_GPIO17";
        case Pcal95555Chip1Pin::TMC_GPIO18: return "TMC_GPIO18";
        case Pcal95555Chip1Pin::PCAL_GND_1: return "PCAL_GND_1";
        case Pcal95555Chip1Pin::TMC_nFAULT_STATUS: return "TMC_nFAULT_STATUS";
        case Pcal95555Chip1Pin::TMC_DRV_EN: return "TMC_DRV_EN";
        case Pcal95555Chip1Pin::TMC_RST_CTRL: return "TMC_RST_CTRL";
        case Pcal95555Chip1Pin::PG_3V3_SWR_FLAG: return "PG_3V3_SWR_FLAG";
        case Pcal95555Chip1Pin::PCAL_GND_2: return "PCAL_GND_2";
        case Pcal95555Chip1Pin::PCAL_GND_3: return "PCAL_GND_3";
        case Pcal95555Chip1Pin::PCAL_GND_4: return "PCAL_GND_4";
        case Pcal95555Chip1Pin::CAN_HS_STB_OP: return "CAN_HS_STB_OP";
        case Pcal95555Chip1Pin::IMU_nBOOT: return "IMU_nBOOT";
        case Pcal95555Chip1Pin::IMU_nINT: return "IMU_nINT";
        case Pcal95555Chip1Pin::TMC_SPI_COMM_nEN: return "TMC_SPI_COMM_nEN";
        case Pcal95555Chip1Pin::TMC_nWAKE_CTRL: return "TMC_nWAKE_CTRL";
        case Pcal95555Chip1Pin::IMU_nRTS: return "IMU_nRTS";
        default: return "UNKNOWN_CHIP1_PIN";
    }
}

constexpr const char* Pcal95555Chip2PinToString(Pcal95555Chip2Pin pin) {
    switch (pin) {
        case Pcal95555Chip2Pin::MOTOR_DIR_1: return "MOTOR_DIR_1";
        case Pcal95555Chip2Pin::MOTOR_DIR_2: return "MOTOR_DIR_2";
        case Pcal95555Chip2Pin::MOTOR_STEP_1: return "MOTOR_STEP_1";
        case Pcal95555Chip2Pin::MOTOR_STEP_2: return "MOTOR_STEP_2";
        case Pcal95555Chip2Pin::FAN_ENABLE_1: return "FAN_ENABLE_1";
        case Pcal95555Chip2Pin::FAN_ENABLE_2: return "FAN_ENABLE_2";
        case Pcal95555Chip2Pin::FAN_PWM_1: return "FAN_PWM_1";
        case Pcal95555Chip2Pin::FAN_PWM_2: return "FAN_PWM_2";
        case Pcal95555Chip2Pin::HEATER_ENABLE: return "HEATER_ENABLE";
        case Pcal95555Chip2Pin::HEATER_PWM: return "HEATER_PWM";
        case Pcal95555Chip2Pin::VALVE_1_OPEN: return "VALVE_1_OPEN";
        case Pcal95555Chip2Pin::VALVE_1_CLOSE: return "VALVE_1_CLOSE";
        case Pcal95555Chip2Pin::VALVE_2_OPEN: return "VALVE_2_OPEN";
        case Pcal95555Chip2Pin::VALVE_2_CLOSE: return "VALVE_2_CLOSE";
        case Pcal95555Chip2Pin::AUX_OUTPUT_1: return "AUX_OUTPUT_1";
        case Pcal95555Chip2Pin::AUX_OUTPUT_2: return "AUX_OUTPUT_2";
        default: return "UNKNOWN_CHIP2_PIN";
    }
}

///@}

#endif // HF_EXT_PINS_ENUM_HPP