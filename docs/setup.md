---
layout: default
title: Setup Guide
nav_order: 2
---

# Setup Guide
{: .no_toc }

This guide will help you get started with the HF-PinCfg library in your embedded project.

## Table of Contents
{: .no_toc .text-delta }

1. TOC
{:toc}

---

## Prerequisites

Before using HF-PinCfg, ensure your development environment meets these requirements:

### Compiler Requirements

- **C++17 or later**: The library uses modern C++ features including `constexpr` and template metaprogramming
- **Supported Compilers**:
  - GCC 7.0+
  - Clang 6.0+  
  - MSVC 19.14+ (Visual Studio 2017 15.7+)

### Platform Requirements

- **Embedded Systems**: Any microcontroller platform with C++17 compiler support
- **Development Boards**: Tested on ESP32, STM32, Arduino platforms
- **Operating Systems**: Works on bare metal, FreeRTOS, and other embedded RTOS

## Installation

### Method 1: Direct Integration

1. **Download the Library**
   ```bash
   git clone https://github.com/hardfoc/hf-pincfg.git
   ```

2. **Copy Headers to Your Project**
   ```bash
   cp hf-pincfg/src/*.hpp your-project/include/
   ```

3. **Include in Your Code**
   ```cpp
   #include "hf_functional_pin_config.hpp"
   #include "hf_functional_pin_config_vortex_v1.hpp"  // Or your platform
   ```

### Method 2: Git Submodule

1. **Add as Submodule**
   ```bash
   git submodule add https://github.com/hardfoc/hf-pincfg.git third-party/hf-pincfg
   ```

2. **Update Your Build System**
   
   **CMake Example:**
   ```cmake
   # Add include directory
   target_include_directories(your-target PRIVATE 
       third-party/hf-pincfg/src
   )
   ```

   **Arduino Example:**
   ```cpp
   // In your Arduino sketch folder, create a symlink:
   // ln -s ../third-party/hf-pincfg/src hf-pincfg
   #include "hf-pincfg/hf_functional_pin_config.hpp"
   ```

### Method 3: Package Manager

Support for popular C++ package managers is planned:

- **vcpkg**: Coming soon
- **Conan**: Coming soon  
- **PlatformIO Library**: Coming soon

## Quick Start

### Basic Usage

```cpp
#include "hf_functional_pin_config.hpp"
#include "hf_functional_pin_config_vortex_v1.hpp"

// Use the default Vortex V1 configuration
using PinConfig = hf::functional_pin_config::vortex_v1::DefaultConfig;

void setup() {
    // Access pin definitions
    constexpr auto status_led = PinConfig::status_led;
    constexpr auto uart_pin = PinConfig::debug_uart_tx;
    
    // Initialize hardware using the pin definitions
    pinMode(status_led.pin, OUTPUT);
    Serial.begin(115200, SERIAL_8N1, -1, uart_pin.pin);
}
```

### Creating Custom Configurations

```cpp
#include "hf_functional_pin_config.hpp"

// Define your custom pin configuration
struct MyCustomConfig {
    // Status indicators
    static constexpr hf::functional_pin_config::PinDefinition status_led{2, "Status LED"};
    static constexpr hf::functional_pin_config::PinDefinition error_led{3, "Error LED"};
    
    // Communication pins
    static constexpr hf::functional_pin_config::PinDefinition uart_tx{17, "UART TX"};
    static constexpr hf::functional_pin_config::PinDefinition uart_rx{16, "UART RX"};
    
    // SPI pins
    static constexpr hf::functional_pin_config::PinDefinition spi_mosi{23, "SPI MOSI"};
    static constexpr hf::functional_pin_config::PinDefinition spi_miso{19, "SPI MISO"};
    static constexpr hf::functional_pin_config::PinDefinition spi_sck{18, "SPI SCK"};
    static constexpr hf::functional_pin_config::PinDefinition spi_cs{5, "SPI CS"};
};

// Use your custom configuration
using PinConfig = MyCustomConfig;
```

## Verification

### Compile Test

Create a simple test file to verify your setup:

```cpp
#include "hf_functional_pin_config.hpp"
#include "hf_functional_pin_config_vortex_v1.hpp"

int main() {
    using Config = hf::functional_pin_config::vortex_v1::DefaultConfig;
    
    // This should compile without errors
    constexpr auto led_pin = Config::status_led;
    static_assert(led_pin.pin >= 0, "Pin number should be valid");
    
    return 0;
}
```

### Build and Run

```bash
g++ -std=c++17 -I/path/to/hf-pincfg/src test.cpp -o test
./test
```

If it compiles and runs without errors, your setup is correct!

## Next Steps

- **Explore Examples**: Check out the [Usage Examples](examples.md) page
- **Learn the API**: Read the [API Reference](api.md)
- **Platform Support**: See [Platform Support](platforms.md) for your hardware
- **Contribute**: Help improve the library with [Contributing Guide](contributing.md)

## Troubleshooting

### Common Issues

**Compiler Errors About C++17 Features**
- Ensure your compiler supports C++17 and the `-std=c++17` flag is set

**Missing Header Files**
- Verify the header files are in your include path
- Check that file permissions allow reading

**Pin Conflicts**
- Review your pin assignments for conflicts
- Use the built-in validation features to catch issues early

**Platform-Specific Issues**
- Ensure you're using the correct platform configuration
- Check hardware documentation for pin availability

### Getting Help

If you encounter issues not covered here:

1. Check the [GitHub Issues](https://github.com/hardfoc/hf-pincfg/issues) for similar problems
2. Create a new issue with detailed information about your setup
3. Join the [GitHub Discussions](https://github.com/hardfoc/hf-pincfg/discussions) for community support