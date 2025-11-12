---
layout: default
title: Usage Examples
nav_order: 3
---

# Usage Examples
{: .no_toc }

This page provides practical examples of using the HF-PinCfg library in real-world embedded applications.

## Table of Contents
{: .no_toc .text-delta }

1. TOC
{:toc}

---

## Basic Pin Configuration

### Simple LED Control

```cpp
#include "hf_functional_pin_config.hpp"
#include "hf_functional_pin_config_vortex_v1.hpp"

using PinConfig = hf::functional_pin_config::vortex_v1::DefaultConfig;

void setup() {
    // Configure status LED pin
    constexpr auto status_led = PinConfig::status_led;
    pinMode(status_led.pin, OUTPUT);
    
    // Turn on the LED
    digitalWrite(status_led.pin, HIGH);
}

void loop() {
    // Blink the status LED
    static unsigned long last_toggle = 0;
    if (millis() - last_toggle > 1000) {
        constexpr auto status_led = PinConfig::status_led;
        digitalWrite(status_led.pin, !digitalRead(status_led.pin));
        last_toggle = millis();
    }
}
```

### UART Communication Setup

```cpp
#include "hf_functional_pin_config.hpp"
#include "hf_functional_pin_config_vortex_v1.hpp"

using PinConfig = hf::functional_pin_config::vortex_v1::DefaultConfig;

void setup() {
    // Configure debug UART using pin configuration
    constexpr auto uart_tx = PinConfig::debug_uart_tx;
    constexpr auto uart_rx = PinConfig::debug_uart_rx;
    
    Serial.begin(115200, SERIAL_8N1, uart_rx.pin, uart_tx.pin);
    
    Serial.print("UART initialized on pins TX:");
    Serial.print(uart_tx.pin);
    Serial.print(" RX:");
    Serial.println(uart_rx.pin);
}
```

## Advanced Configuration

### Custom Hardware Configuration

```cpp
#include "hf_functional_pin_config.hpp"

// Define a custom configuration for your specific hardware
struct MyBoardConfig {
    // System pins
    static constexpr auto power_enable = hf::functional_pin_config::PinDefinition{4, "Power Enable"};
    static constexpr auto reset_button = hf::functional_pin_config::PinDefinition{0, "Reset Button"};
    
    // Status indicators
    static constexpr auto status_led_red = hf::functional_pin_config::PinDefinition{2, "Status LED Red"};
    static constexpr auto status_led_green = hf::functional_pin_config::PinDefinition{3, "Status LED Green"};
    static constexpr auto status_led_blue = hf::functional_pin_config::PinDefinition{4, "Status LED Blue"};
    
    // Communication interfaces
    struct UART {
        static constexpr auto tx = hf::functional_pin_config::PinDefinition{17, "UART0 TX"};
        static constexpr auto rx = hf::functional_pin_config::PinDefinition{16, "UART0 RX"};
    };
    
    struct SPI {
        static constexpr auto mosi = hf::functional_pin_config::PinDefinition{23, "SPI MOSI"};
        static constexpr auto miso = hf::functional_pin_config::PinDefinition{19, "SPI MISO"};
        static constexpr auto sck = hf::functional_pin_config::PinDefinition{18, "SPI SCK"};
        static constexpr auto cs_sensor = hf::functional_pin_config::PinDefinition{5, "Sensor CS"};
        static constexpr auto cs_flash = hf::functional_pin_config::PinDefinition{15, "Flash CS"};
    };
    
    struct I2C {
        static constexpr auto sda = hf::functional_pin_config::PinDefinition{21, "I2C SDA"};
        static constexpr auto scl = hf::functional_pin_config::PinDefinition{22, "I2C SCL"};
    };
    
    // Sensor pins
    static constexpr auto temperature_sensor = hf::functional_pin_config::PinDefinition{36, "Temperature ADC"};
    static constexpr auto voltage_monitor = hf::functional_pin_config::PinDefinition{39, "Voltage Monitor ADC"};
    
    // Motor control pins
    static constexpr auto motor_pwm_a = hf::functional_pin_config::PinDefinition{25, "Motor PWM A"};
    static constexpr auto motor_pwm_b = hf::functional_pin_config::PinDefinition{26, "Motor PWM B"};
    static constexpr auto motor_dir = hf::functional_pin_config::PinDefinition{27, "Motor Direction"};
    static constexpr auto motor_enable = hf::functional_pin_config::PinDefinition{14, "Motor Enable"};
};

using PinConfig = MyBoardConfig;
```

### RGB Status LED Control

```cpp
#include "hf_functional_pin_config.hpp"

// Using the custom configuration from above
using PinConfig = MyBoardConfig;

class StatusLED {
private:
    static constexpr auto red_pin = PinConfig::status_led_red;
    static constexpr auto green_pin = PinConfig::status_led_green;
    static constexpr auto blue_pin = PinConfig::status_led_blue;
    
public:
    enum Color {
        OFF = 0,
        RED = 1,
        GREEN = 2,
        BLUE = 4,
        YELLOW = RED | GREEN,
        CYAN = GREEN | BLUE,
        MAGENTA = RED | BLUE,
        WHITE = RED | GREEN | BLUE
    };
    
    static void initialize() {
        pinMode(red_pin.pin, OUTPUT);
        pinMode(green_pin.pin, OUTPUT);
        pinMode(blue_pin.pin, OUTPUT);
        setColor(OFF);
    }
    
    static void setColor(Color color) {
        digitalWrite(red_pin.pin, (color & RED) ? HIGH : LOW);
        digitalWrite(green_pin.pin, (color & GREEN) ? HIGH : LOW);
        digitalWrite(blue_pin.pin, (color & BLUE) ? HIGH : LOW);
    }
};

void setup() {
    StatusLED::initialize();
    
    // Show initialization sequence
    StatusLED::setColor(StatusLED::RED);
    delay(500);
    StatusLED::setColor(StatusLED::GREEN);
    delay(500);
    StatusLED::setColor(StatusLED::BLUE);
    delay(500);
    StatusLED::setColor(StatusLED::WHITE);
}
```

## Platform Integration Examples

### ESP32 SPI Driver Integration

```cpp
#include "hf_functional_pin_config.hpp"
#include "hf_functional_pin_config_vortex_v1.hpp"
#include <SPI.h>

using PinConfig = hf::functional_pin_config::vortex_v1::DefaultConfig;

class SensorDriver {
private:
    SPIClass* spi;
    static constexpr auto cs_pin = PinConfig::SPI::cs_sensor;
    static constexpr auto mosi_pin = PinConfig::SPI::mosi;
    static constexpr auto miso_pin = PinConfig::SPI::miso;
    static constexpr auto sck_pin = PinConfig::SPI::sck;
    
public:
    SensorDriver() : spi(&SPI) {}
    
    void initialize() {
        // Configure CS pin
        pinMode(cs_pin.pin, OUTPUT);
        digitalWrite(cs_pin.pin, HIGH);
        
        // Initialize SPI with pin configuration
        spi->begin(sck_pin.pin, miso_pin.pin, mosi_pin.pin, cs_pin.pin);
        spi->setFrequency(1000000); // 1MHz
        spi->setDataMode(SPI_MODE0);
        
        Serial.printf("SPI initialized: MOSI=%d, MISO=%d, SCK=%d, CS=%d\n",
                     mosi_pin.pin, miso_pin.pin, sck_pin.pin, cs_pin.pin);
    }
    
    uint16_t readSensor() {
        digitalWrite(cs_pin.pin, LOW);
        uint16_t result = spi->transfer16(0x0000);
        digitalWrite(cs_pin.pin, HIGH);
        return result;
    }
};
```

### FreeRTOS Task with Pin Configuration

```cpp
#include "hf_functional_pin_config.hpp"
#include "hf_functional_pin_config_vortex_v1.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

using PinConfig = hf::functional_pin_config::vortex_v1::DefaultConfig;

void monitor_task(void* parameters) {
    constexpr auto voltage_pin = PinConfig::voltage_monitor;
    constexpr auto temp_pin = PinConfig::temperature_sensor;
    constexpr auto status_led = PinConfig::status_led;
    
    // Configure pins
    pinMode(status_led.pin, OUTPUT);
    // ADC pins are automatically configured
    
    while (true) {
        // Read sensor values
        int voltage_raw = analogRead(voltage_pin.pin);
        int temp_raw = analogRead(temp_pin.pin);
        
        // Convert to meaningful values
        float voltage = (voltage_raw / 4095.0) * 3.3 * 2; // Voltage divider
        float temperature = (temp_raw / 4095.0) * 3.3 * 100; // LM35 sensor
        
        // Log readings
        Serial.printf("Voltage: %.2fV, Temperature: %.1f°C\n", voltage, temperature);
        
        // Status indication
        if (voltage < 3.0 || temperature > 80.0) {
            // Warning condition - fast blink
            digitalWrite(status_led.pin, HIGH);
            vTaskDelay(pdMS_TO_TICKS(100));
            digitalWrite(status_led.pin, LOW);
            vTaskDelay(pdMS_TO_TICKS(100));
        } else {
            // Normal condition - slow blink
            digitalWrite(status_led.pin, HIGH);
            vTaskDelay(pdMS_TO_TICKS(500));
            digitalWrite(status_led.pin, LOW);
            vTaskDelay(pdMS_TO_TICKS(1500));
        }
    }
}

void setup() {
    Serial.begin(115200);
    
    // Create monitoring task
    xTaskCreate(monitor_task, "Monitor", 4096, NULL, 1, NULL);
}
```

## Compile-Time Validation

### Pin Conflict Detection

```cpp
#include "hf_functional_pin_config.hpp"

// Example of compile-time validation
template<typename Config>
constexpr bool validate_pin_config() {
    // Check for pin conflicts at compile time
    static_assert(Config::status_led.pin != Config::uart_tx.pin, 
                  "Status LED and UART TX cannot use the same pin");
    
    static_assert(Config::SPI::mosi.pin != Config::I2C::sda.pin,
                  "SPI MOSI and I2C SDA cannot share pins");
    
    // Validate pin ranges (ESP32 example)
    static_assert(Config::status_led.pin >= 0 && Config::status_led.pin <= 39,
                  "Status LED pin must be in valid GPIO range");
    
    return true;
}

// Apply validation to your configuration
using PinConfig = MyBoardConfig;
static_assert(validate_pin_config<PinConfig>(), "Pin configuration validation failed");
```

### Pin Usage Documentation

```cpp
#include "hf_functional_pin_config.hpp"

template<typename Config>
void print_pin_configuration() {
    Serial.println("=== Pin Configuration ===");
    Serial.printf("Status LED: GPIO%d (%s)\n", 
                  Config::status_led.pin, Config::status_led.description);
    Serial.printf("UART TX: GPIO%d (%s)\n", 
                  Config::UART::tx.pin, Config::UART::tx.description);
    Serial.printf("UART RX: GPIO%d (%s)\n", 
                  Config::UART::rx.pin, Config::UART::rx.description);
    Serial.printf("SPI MOSI: GPIO%d (%s)\n", 
                  Config::SPI::mosi.pin, Config::SPI::mosi.description);
    Serial.printf("SPI MISO: GPIO%d (%s)\n", 
                  Config::SPI::miso.pin, Config::SPI::miso.description);
    Serial.printf("SPI SCK: GPIO%d (%s)\n", 
                  Config::SPI::sck.pin, Config::SPI::sck.description);
    Serial.println("========================");
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    using PinConfig = MyBoardConfig;
    print_pin_configuration<PinConfig>();
}
```

## Best Practices

### 1. Namespace Organization

```cpp
namespace my_project {
namespace hardware {
    struct ProductionConfig {
        // Production hardware pin definitions
    };
    
    struct DevelopmentConfig {
        // Development board pin definitions
    };
}
}

// Select configuration based on build target
#ifdef PRODUCTION_BUILD
using PinConfig = my_project::hardware::ProductionConfig;
#else
using PinConfig = my_project::hardware::DevelopmentConfig;
#endif
```

### 2. Component-Based Organization

```cpp
struct SystemConfig {
    struct Power {
        static constexpr auto main_enable = PinDefinition{4, "Main Power Enable"};
        static constexpr auto battery_monitor = PinDefinition{35, "Battery Voltage"};
    };
    
    struct Communications {
        static constexpr auto wifi_enable = PinDefinition{12, "WiFi Module Enable"};
        static constexpr auto bluetooth_reset = PinDefinition{13, "Bluetooth Reset"};
    };
    
    struct Sensors {
        static constexpr auto temperature = PinDefinition{36, "Temperature Sensor"};
        static constexpr auto humidity = PinDefinition{37, "Humidity Sensor"};
        static constexpr auto motion = PinDefinition{14, "Motion Detector"};
    };
};
```

### 3. Version Management

```cpp
namespace hardware_v1 {
    struct PinConfig {
        static constexpr auto led_pin = PinDefinition{2, "Status LED"};
        // v1 hardware definitions
    };
}

namespace hardware_v2 {
    struct PinConfig {
        static constexpr auto led_pin = PinDefinition{5, "Status LED"};
        // v2 hardware definitions with updated pins
    };
}

// Select version at compile time
#define HARDWARE_VERSION 2

#if HARDWARE_VERSION == 1
using PinConfig = hardware_v1::PinConfig;
#elif HARDWARE_VERSION == 2
using PinConfig = hardware_v2::PinConfig;
#endif
```

These examples demonstrate the flexibility and power of the HF-PinCfg library for managing embedded system pin configurations in a maintainable, type-safe manner.