---
layout: default
title: Home
nav_order: 1
description: "Hardware Agnostic C++ Pin Configuration Library for Embedded Systems"
permalink: /
---

# HF-PinCfg Library
{: .fs-9 }

Hardware-Agnostic C++ Pin Configuration Library for Embedded Systems
{: .fs-6 .fw-300 }

[Get Started](#getting-started){: .btn .btn-primary .fs-5 .mb-4 .mb-md-0 .mr-2 }
[View on GitHub](https://github.com/hardfoc/hf-pincfg){: .btn .fs-5 .mb-4 .mb-md-0 }

---

## Overview

HF-PinCfg defines **what pins do functionally** on HardFOC-based boards, independent of the underlying hardware implementation. It provides a hardware abstraction layer that lets you write firmware using semantic pin names instead of physical GPIO numbers.

### Supported Boards

- **🔧 Vortex V1** - ESP32-C6 based controller board
- **🚀 Future Boards** - Extensible architecture for upcoming HardFOC variants

### Key Features

- **🎯 Functional Abstraction**: Define WHAT pins do, not WHERE they are
- **🔄 Hardware Agnostic**: Same code works across different HardFOC board variants  
- **📝 Semantic Names**: Use `MOTOR_SPI_CS` instead of `GPIO_18`
- **⚡ Zero Runtime Cost**: All abstraction resolved at compile time
- **🏗️ Standardized Interface**: Consistent pin functions across all HardFOC boards
- **📚 Portable Firmware**: Write once, deploy on any supported HardFOC variant

### Supported Platforms

- **Vortex V1**: Complete pin configuration for Vortex V1 development board
- **Custom Platforms**: Framework for defining your own hardware configurations

## Getting Started

### Concept Example

**Instead of hardware-specific code:**
```cpp
// ❌ Hardware-dependent - breaks on different boards
gpio_config_t config = {
    .pin_bit_mask = (1ULL << GPIO_NUM_18),  // What if GPIO_18 doesn't exist?
    .mode = GPIO_MODE_OUTPUT
};
```

**Write functional, portable code:**
```cpp  
// ✅ Hardware-agnostic - works on any HardFOC board
#include "hf_functional_pin_config.hpp"

// Use semantic names that describe pin function
auto motor_cs_pin = HfPinConfig::MOTOR_DRIVER_SPI_CS;
auto encoder_miso = HfPinConfig::ENCODER_SPI_MISO;
auto status_led = HfPinConfig::STATUS_LED;

// Board variant automatically maps to correct physical pins
HardFOC::init_platform_hardware();
```

### Installation

1. Copy the header files to your project's include directory
2. Include the appropriate configuration headers
3. Define your pin configuration using the provided templates

For detailed setup instructions, see the [Setup Guide](docs/setup.md).

## Documentation

- [**Setup Guide**](docs/setup.md) - Installation and initial setup
- [**Usage Examples**](docs/examples.md) - Common usage patterns and examples  
- [**API Reference**](docs/api.md) - Complete API documentation
- [**Contributing**](docs/contributing.md) - How to contribute to the project

## Project Structure

```
hf-pincfg/
├── src/                           # Source headers
│   ├── hf_functional_pin_config.hpp          # Core pin configuration framework
│   └── hf_functional_pin_config_vortex_v1.hpp # Vortex V1 specific configurations
├── docs/                          # Documentation
├── _config/                       # Build and documentation configuration
└── README.md                      # This file
```

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Contributing

We welcome contributions! Please see our [Contributing Guide](docs/contributing.md) for details on how to submit pull requests, report issues, and contribute to the project.

## Support

- **Issues**: Report bugs and request features on [GitHub Issues](https://github.com/hardfoc/hf-pincfg/issues)
- **Documentation**: Browse the [online documentation](https://hardfoc.github.io/hf-pincfg)

---

Built with ❤️ by the [HardFOC Team](https://github.com/hardfoc)