
#ifndef GPIO_CONFIG_ESP32C6_HPP
#define GPIO_CONFIG_ESP32C6_HPP

// GPIO configuration for ESP32-C6
// This file defines the GPIO pin mappings for various peripherals on the ESP32-C6
#include "driver/gpio.h"

// ================= SPI0 Pins ===================
#define SPI0_MISO_PIN         GPIO_NUM_2     // SPI0 Master In Slave Out
#define SPI0_MOSI_PIN         GPIO_NUM_7     // SPI0 Master Out Slave In
#define SPI0_CLK_PIN          GPIO_NUM_6     // SPI0 Serial Clock
#define SPI0_CS_TMC_PIN       GPIO_NUM_18    // SPI0 Chip Select for the TMC9660
#define SPI0_CS_AS5047_PIN    GPIO_NUM_20    // SPI0 Chip Select for the AS5047P
#define SPI0_CS_EXT_1_PIN       GPIO_NUM_19  // SPI0 Chip Select for the external device (1)
#define SPI0_CS_EXT_2_PIN       GPIO_NUM_8   // SPI0 Chip Select for the external device (2)

// ================ UART Pins ==================
#define UART_RXD_PIN       GPIO_NUM_4    // UART Receive
#define UART_TXD_PIN       GPIO_NUM_5    // UART Transmit

// ============ Debug UART Pins ================
#define DEBUG_UART_TX_PIN GPIO_NUM_0    // Debug UART Transmit
#define DEBUG_UART_RX_PIN GPIO_NUM_1    // Debug UART Receive

// ================ I2C Pins ===================
#define I2C_SDA_PIN       GPIO_NUM_22   // I2C Data
#define I2C_SCL_PIN       GPIO_NUM_23   // I2C Clock

// ============= TWAI (CAN) Pins ===============
#define TWAI_TX_PIN       GPIO_NUM_19   // CAN Transmit
#define TWAI_RX_PIN       GPIO_NUM_15   // CAN Receive

// ================ USB-JTAG Pins ===============
#define USB_JTAG_D_P_PIN   GPIO_NUM_13   // USB D+
#define USB_JTAG_D_N_PIN   GPIO_NUM_12   // USB D-

// ============ WS2812 LED Pin =================
#define WS2812_LED_PIN    GPIO_NUM_3    // WS2812 Data

// Function to configure GPIOs
void configure_gpio() {
    gpio_config_t io_conf;

    // SPI Pins Configuration
    io_conf.pin_bit_mask = (1ULL << SPI_MISO_PIN) | (1ULL << SPI_MOSI_PIN) | (1ULL << SPI_CLK_PIN) | (1ULL << SPI_CS_PIN) | (1ULL << SPI_IO4_PIN) | (1ULL << SPI_IO5_PIN);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);

    // UART Pins Configuration
    io_conf.pin_bit_mask = (1ULL << UART_TX_PIN) | (1ULL << UART_RX_PIN);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);

    // Debug UART Pins Configuration
    io_conf.pin_bit_mask = (1ULL << DEBUG_UART_TX_PIN) | (1ULL << DEBUG_UART_RX_PIN);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);

    // I2C Pins Configuration
    io_conf.pin_bit_mask = (1ULL << I2C_SDA_PIN) | (1ULL << I2C_SCL_PIN);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);

    // TWAI (CAN) Pins Configuration
    io_conf.pin_bit_mask = (1ULL << TWAI_TX_PIN) | (1ULL << TWAI_RX_PIN);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);

    // USB Pins Configuration
    io_conf.pin_bit_mask = (1ULL << USB_D_PLUS_PIN) | (1ULL << USB_D_MINUS_PIN);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);

    // WS2812 LED Pin Configuration
    io_conf.pin_bit_mask = (1ULL << WS2812_LED_PIN);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);
}

#endif // GPIO_CONFIG_ESP32C6_HPP
