---
layout: default
title: API Reference
nav_order: 4
---

# API Reference
{: .no_toc }

Complete reference documentation for the HF-PinCfg library API.

## Table of Contents
{: .no_toc .text-delta }

1. TOC
{:toc}

---

## Core Types

### PinDefinition

The fundamental type representing a hardware pin with its associated metadata.

```cpp
namespace hf::functional_pin_config {
    struct PinDefinition {
        int pin;                    // Physical pin number
        const char* description;    // Human-readable description
        
        constexpr PinDefinition(int pin_num, const char* desc) 
            : pin(pin_num), description(desc) {}
    };
}
```

#### Properties

| Property | Type | Description |
|----------|------|-------------|
| `pin` | `int` | Physical pin number on the microcontroller |
| `description` | `const char*` | Human-readable description of the pin's function |

#### Example

```cpp
constexpr auto status_led = PinDefinition{2, "Status LED"};
static_assert(status_led.pin == 2);
// status_led.description == "Status LED"
```

## Configuration Structures

### Base Configuration Pattern

All pin configurations follow a consistent structure pattern:

```cpp
struct ConfigurationName {
    // Individual pins
    static constexpr PinDefinition pin_name{number, "Description"};
    
    // Grouped pins (optional)
    struct GroupName {
        static constexpr PinDefinition sub_pin{number, "Description"};
    };
};
```

### Vortex V1 Configuration

The complete Vortex V1 hardware configuration is provided as a reference implementation.

```cpp
namespace hf::functional_pin_config::vortex_v1 {
    struct DefaultConfig {
        // System pins
        static constexpr PinDefinition status_led;
        static constexpr PinDefinition reset_button;
        
        // Communication interfaces
        struct UART { /* ... */ };
        struct SPI { /* ... */ };
        struct I2C { /* ... */ };
        
        // Sensor inputs
        static constexpr PinDefinition temperature_sensor;
        static constexpr PinDefinition voltage_monitor;
        
        // Motor control outputs
        static constexpr PinDefinition motor_pwm_a;
        static constexpr PinDefinition motor_pwm_b;
        // ... additional pins
    };
}
```

## Pin Categories

### System Control Pins

Pins used for basic system operation and status indication.

| Pin Name | Type | Description |
|----------|------|-------------|
| `status_led` | Output | Main status indicator LED |
| `error_led` | Output | Error condition indicator |
| `power_enable` | Output | System power control |
| `reset_button` | Input | Hardware reset button |

### Communication Interface Pins

#### UART (Universal Asynchronous Receiver-Transmitter)

```cpp
struct UART {
    static constexpr PinDefinition tx{17, "UART TX"};
    static constexpr PinDefinition rx{16, "UART RX"};
    static constexpr PinDefinition rts{14, "UART RTS"};  // Optional
    static constexpr PinDefinition cts{15, "UART CTS"};  // Optional
};
```

#### SPI (Serial Peripheral Interface)

```cpp
struct SPI {
    static constexpr PinDefinition mosi{23, "SPI MOSI"};
    static constexpr PinDefinition miso{19, "SPI MISO"};
    static constexpr PinDefinition sck{18, "SPI SCK"};
    static constexpr PinDefinition cs_sensor{5, "Sensor CS"};
    static constexpr PinDefinition cs_flash{15, "Flash CS"};
    // Additional CS pins as needed
};
```

#### I2C (Inter-Integrated Circuit)

```cpp
struct I2C {
    static constexpr PinDefinition sda{21, "I2C SDA"};
    static constexpr PinDefinition scl{22, "I2C SCL"};
};
```

### Sensor Input Pins

Pins connected to various sensors for data acquisition.

| Pin Name | Type | Description |
|----------|------|-------------|
| `temperature_sensor` | Analog Input | Temperature measurement |
| `humidity_sensor` | Analog Input | Humidity measurement |
| `voltage_monitor` | Analog Input | Supply voltage monitoring |
| `current_monitor` | Analog Input | Current consumption monitoring |
| `motion_detector` | Digital Input | Motion/proximity detection |

### Motor Control Pins

Pins used for motor control and drive circuits.

| Pin Name | Type | Description |
|----------|------|-------------|
| `motor_pwm_a` | PWM Output | Motor phase A PWM |
| `motor_pwm_b` | PWM Output | Motor phase B PWM |
| `motor_dir` | Digital Output | Motor direction control |
| `motor_enable` | Digital Output | Motor driver enable |
| `encoder_a` | Digital Input | Encoder channel A |
| `encoder_b` | Digital Input | Encoder channel B |

## Compile-Time Features

### Type Safety

All pin definitions are `constexpr` and can be used in compile-time contexts:

```cpp
using Config = MyConfig;

// Compile-time pin access
constexpr auto led_pin = Config::status_led;
static_assert(led_pin.pin >= 0, "Pin must be non-negative");

// Compile-time validation
template<int pin_num>
constexpr bool is_valid_gpio() {
    return pin_num >= 0 && pin_num <= 39;  // ESP32 example
}

static_assert(is_valid_gpio<Config::status_led.pin>(), 
              "Status LED pin out of range");
```

### Pin Conflict Detection

Implement compile-time pin conflict detection:

```cpp
template<typename Config>
constexpr bool validate_no_conflicts() {
    // Example conflict checks
    static_assert(Config::status_led.pin != Config::UART::tx.pin,
                  "Status LED and UART TX conflict");
    
    static_assert(Config::SPI::mosi.pin != Config::I2C::sda.pin,
                  "SPI MOSI and I2C SDA conflict");
    
    return true;
}

using MyConfig = MyBoardConfig;
static_assert(validate_no_conflicts<MyConfig>(), 
              "Pin configuration has conflicts");
```

## Usage Patterns

### Basic Pin Access

```cpp
using Config = hf::functional_pin_config::vortex_v1::DefaultConfig;

void setup() {
    // Direct pin access
    constexpr auto led_pin = Config::status_led;
    pinMode(led_pin.pin, OUTPUT);
    
    // Grouped pin access
    constexpr auto uart_tx = Config::UART::tx;
    constexpr auto uart_rx = Config::UART::rx;
    Serial.begin(115200, SERIAL_8N1, uart_rx.pin, uart_tx.pin);
}
```

### Configuration Templates

Create reusable templates that work with any pin configuration:

```cpp
template<typename PinConfig>
class StatusManager {
private:
    static constexpr auto status_pin = PinConfig::status_led;
    static constexpr auto error_pin = PinConfig::error_led;
    
public:
    static void initialize() {
        pinMode(status_pin.pin, OUTPUT);
        pinMode(error_pin.pin, OUTPUT);
    }
    
    static void setStatus(bool ok) {
        digitalWrite(status_pin.pin, ok ? HIGH : LOW);
        digitalWrite(error_pin.pin, ok ? LOW : HIGH);
    }
};

// Use with any compatible configuration
using MyStatusManager = StatusManager<MyBoardConfig>;
```

### Pin Mapping Functions

Create functions that map logical functions to physical pins:

```cpp
template<typename Config>
constexpr auto get_communication_pins() {
    return std::make_tuple(
        Config::UART::tx,
        Config::UART::rx,
        Config::SPI::mosi,
        Config::SPI::miso,
        Config::SPI::sck,
        Config::I2C::sda,
        Config::I2C::scl
    );
}

// Usage
using Config = MyBoardConfig;
constexpr auto comm_pins = get_communication_pins<Config>();
```

## Platform Integration

### Arduino Framework

```cpp
#include "hf_functional_pin_config.hpp"

using Config = MyBoardConfig;

void setup() {
    // Standard Arduino pin functions work directly
    constexpr auto led = Config::status_led;
    pinMode(led.pin, OUTPUT);
    digitalWrite(led.pin, HIGH);
    
    // Analog pins
    constexpr auto sensor = Config::temperature_sensor;
    int reading = analogRead(sensor.pin);
}
```

### ESP-IDF Framework

```cpp
#include "hf_functional_pin_config.hpp"
#include "driver/gpio.h"
#include "driver/adc.h"

using Config = MyBoardConfig;

void app_main() {
    // GPIO configuration
    constexpr auto led = Config::status_led;
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << led.pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(led.pin, 1);
    
    // ADC configuration
    constexpr auto sensor = Config::temperature_sensor;
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(sensor.pin, ADC_ATTEN_DB_0);
}
```

### STM32 HAL Integration

```cpp
#include "hf_functional_pin_config.hpp"
#include "stm32f4xx_hal.h"

using Config = MyBoardConfig;

void SystemClock_Config(void);
void GPIO_Init(void);

int main(void) {
    HAL_Init();
    SystemClock_Config();
    GPIO_Init();
    
    constexpr auto led = Config::status_led;
    
    while (1) {
        HAL_GPIO_WritePin(GPIOA, 1 << led.pin, GPIO_PIN_SET);
        HAL_Delay(500);
        HAL_GPIO_WritePin(GPIOA, 1 << led.pin, GPIO_PIN_RESET);
        HAL_Delay(500);
    }
}
```

## Error Handling

### Compile-Time Validation

The library provides several mechanisms for compile-time validation:

```cpp
// Pin range validation
template<int pin, int min_pin = 0, int max_pin = 39>
constexpr bool validate_pin_range() {
    static_assert(pin >= min_pin && pin <= max_pin, 
                  "Pin number out of valid range");
    return true;
}

// Pin conflict validation
template<typename Config>
constexpr bool validate_unique_pins() {
    // Implementation depends on specific requirements
    // Can use template metaprogramming to check uniqueness
    return true;
}

// Apply validations
using Config = MyBoardConfig;
static_assert(validate_pin_range<Config::status_led.pin>());
static_assert(validate_unique_pins<Config>());
```

### Runtime Diagnostics

While the library is designed for compile-time configuration, runtime diagnostics can be helpful:

```cpp
template<typename Config>
void print_pin_diagnostics() {
    Serial.println("=== Pin Configuration Diagnostics ===");
    
    // Print all configured pins
    Serial.printf("Status LED: GPIO%d (%s)\n", 
                  Config::status_led.pin, 
                  Config::status_led.description);
    
    // Validate pin capabilities at runtime
    if (Config::temperature_sensor.pin < 32) {
        Serial.println("Temperature sensor: ADC1 capable");
    } else {
        Serial.println("Temperature sensor: ADC2 capable (limited in WiFi mode)");
    }
}
```

## Best Practices

### 1. Use Descriptive Names

```cpp
// Good
static constexpr PinDefinition motor_enable{12, "Motor Driver Enable"};
static constexpr PinDefinition emergency_stop{13, "Emergency Stop Button"};

// Avoid
static constexpr PinDefinition pin12{12, "Pin 12"};
static constexpr PinDefinition p13{13, "P13"};
```

### 2. Group Related Pins

```cpp
// Good - grouped by function
struct MotorControl {
    static constexpr PinDefinition pwm_a{25, "Motor PWM Phase A"};
    static constexpr PinDefinition pwm_b{26, "Motor PWM Phase B"};
    static constexpr PinDefinition enable{27, "Motor Enable"};
    static constexpr PinDefinition direction{14, "Motor Direction"};
};

// Less organized
static constexpr PinDefinition motor_pwm_a{25, "Motor PWM Phase A"};
static constexpr PinDefinition some_other_pin{13, "Other Pin"};
static constexpr PinDefinition motor_pwm_b{26, "Motor PWM Phase B"};
```

### 3. Use Compile-Time Validation

```cpp
template<typename Config>
constexpr bool validate_motor_config() {
    static_assert(Config::MotorControl::pwm_a.pin != Config::MotorControl::pwm_b.pin,
                  "Motor PWM pins cannot be the same");
    
    static_assert(Config::MotorControl::enable.pin >= 0,
                  "Motor enable pin must be valid");
    
    return true;
}

using Config = MyBoardConfig;
static_assert(validate_motor_config<Config>());
```

### 4. Document Pin Constraints

```cpp
struct SensorConfig {
    // ADC1 pins - can be used with WiFi enabled
    static constexpr PinDefinition voltage_monitor{36, "Voltage Monitor (ADC1_CH0)"};
    static constexpr PinDefinition current_monitor{39, "Current Monitor (ADC1_CH3)"};
    
    // ADC2 pins - limited when WiFi is active
    // Use these only when WiFi is disabled or for infrequent readings
    static constexpr PinDefinition temperature{4, "Temperature Sensor (ADC2_CH0) - WiFi conflict"};
};
```

This API reference provides comprehensive documentation for using the HF-PinCfg library effectively in embedded systems development.