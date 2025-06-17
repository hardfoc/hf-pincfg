#ifndef HF_GPIO_CONFIG_HPP
#define HF_GPIO_CONFIG_HPP

/**
 * @file hf_gpio_config.hpp
 * @brief GPIO assignments for the HardFOC controller on ESP32-C6.
 *
 * This header lists all pin numbers used by the controller and provides
 * a helper to configure them using the ESP-IDF gpio_config API.
 */

#include "driver/gpio.h"

/**
 * @name SPI0 pins
 * Data lines are labelled by their direction with respect to the controller.
 */
///@{
#define SPI0_MISO_PIN      GPIO_NUM_2  ///< Data from device to controller
#define SPI0_MOSI_PIN      GPIO_NUM_7  ///< Data from controller to device
#define SPI0_CLK_PIN       GPIO_NUM_6  ///< Serial clock
#define SPI0_CS_TMC_PIN    GPIO_NUM_18 ///< Chip select for the TMC9660 driver
#define SPI0_CS_AS5047_PIN GPIO_NUM_20 ///< Chip select for the AS5047P encoder
#define SPI0_CS_EXT1_PIN   GPIO_NUM_19 ///< Additional SPI device 1
#define SPI0_CS_EXT2_PIN   GPIO_NUM_8  ///< Additional SPI device 2
///@}

/** @name Primary UART pins */
///@{
#define UART_RXD_PIN       GPIO_NUM_4  ///< UART receive
#define UART_TXD_PIN       GPIO_NUM_5  ///< UART transmit
///@}

/** @name Debug UART pins */
///@{
#define DEBUG_UART_TX_PIN  GPIO_NUM_0  ///< Debug UART transmit
#define DEBUG_UART_RX_PIN  GPIO_NUM_1  ///< Debug UART receive
///@}

/** @name I2C pins */
///@{
#define I2C_SDA_PIN        GPIO_NUM_22 ///< I2C data
#define I2C_SCL_PIN        GPIO_NUM_23 ///< I2C clock
///@}

/** @name TWAI (CAN) pins */
///@{
#define TWAI_TX_PIN        GPIO_NUM_19 ///< CAN transmit
#define TWAI_RX_PIN        GPIO_NUM_15 ///< CAN receive
///@}

/** @name USB/JTAG pins */
///@{
#define USB_JTAG_D_PLUS_PIN  GPIO_NUM_13 ///< USB D+
#define USB_JTAG_D_MINUS_PIN GPIO_NUM_12 ///< USB D-
///@}

/** @name WS2812 LED pin */
#define WS2812_LED_PIN     GPIO_NUM_2  ///< WS2812 data
///@}

/**
 * @brief Configure all GPIOs defined above.
 *
 * Each pin is set as a general output with pulls disabled. Adapt
 * the configuration in your project if you require different modes.
 */
static inline void configure_gpio(void)
{
    gpio_config_t io_conf = {};
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;

    // Configure SPI pins
    io_conf.pin_bit_mask = (1ULL << SPI0_MISO_PIN) |
                           (1ULL << SPI0_MOSI_PIN) |
                           (1ULL << SPI0_CLK_PIN)  |
                           (1ULL << SPI0_CS_TMC_PIN) |
                           (1ULL << SPI0_CS_AS5047_PIN) |
                           (1ULL << SPI0_CS_EXT1_PIN) |
                           (1ULL << SPI0_CS_EXT2_PIN);
    io_conf.mode = GPIO_MODE_OUTPUT;
    gpio_config(&io_conf);

    // Configure primary UART pins
    io_conf.pin_bit_mask = (1ULL << UART_RXD_PIN) | (1ULL << UART_TXD_PIN);
    gpio_config(&io_conf);

    // Configure debug UART pins
    io_conf.pin_bit_mask = (1ULL << DEBUG_UART_TX_PIN) |
                           (1ULL << DEBUG_UART_RX_PIN);
    gpio_config(&io_conf);

    // Configure I2C pins
    io_conf.pin_bit_mask = (1ULL << I2C_SDA_PIN) | (1ULL << I2C_SCL_PIN);
    gpio_config(&io_conf);

    // Configure TWAI (CAN) pins
    io_conf.pin_bit_mask = (1ULL << TWAI_TX_PIN) | (1ULL << TWAI_RX_PIN);
    gpio_config(&io_conf);

    // Configure USB/JTAG pins
    io_conf.pin_bit_mask = (1ULL << USB_JTAG_D_PLUS_PIN) |
                           (1ULL << USB_JTAG_D_MINUS_PIN);
    gpio_config(&io_conf);

    // Configure WS2812 pin
    io_conf.pin_bit_mask = (1ULL << WS2812_LED_PIN);
    gpio_config(&io_conf);
}

/**
 * @brief Initialize MCU pin configuration.
 *
 * This wrapper simply calls ::configure_gpio so that applications
 * can use a more descriptive entry point when setting up pins.
 */
static inline void init_mcu_pinconfig(void)
{
    configure_gpio();
}

#endif // HF_GPIO_CONFIG_HPP
