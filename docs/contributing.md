---
layout: default
title: Contributing
nav_order: 5
---

# Contributing to HF-PinCfg
{: .no_toc }

We welcome contributions to the HF-PinCfg library! This guide will help you get started with contributing to the project.

## Table of Contents
{: .no_toc .text-delta }

1. TOC
{:toc}

---

## How to Contribute

### Types of Contributions

We welcome several types of contributions:

- **Bug Reports**: Help us identify and fix issues
- **Feature Requests**: Suggest new functionality or improvements
- **Code Contributions**: Submit bug fixes, new features, or optimizations
- **Documentation**: Improve existing docs or add new documentation
- **Platform Support**: Add support for new hardware platforms
- **Examples**: Provide usage examples and tutorials

### Getting Started

1. **Fork the Repository**
   ```bash
   # Fork on GitHub, then clone your fork
   git clone https://github.com/your-username/hf-pincfg.git
   cd hf-pincfg
   ```

2. **Set Up Development Environment**
   ```bash
   # Add upstream remote
   git remote add upstream https://github.com/hardfoc/hf-pincfg.git
   
   # Create a development branch
   git checkout -b feature/your-feature-name
   ```

3. **Make Your Changes**
   - Follow the coding standards outlined below
   - Add tests for new functionality
   - Update documentation as needed

4. **Submit a Pull Request**
   - Push your changes to your fork
   - Create a pull request with a clear description
   - Link any related issues

## Development Guidelines

### Code Style

#### C++ Coding Standards

- **C++ Standard**: Use C++17 features, ensure compatibility with C++17 compilers
- **Naming Conventions**:
  - `snake_case` for variables, functions, and namespaces
  - `PascalCase` for classes and structs
  - `UPPER_CASE` for constants and macros
  - Descriptive names that clearly indicate purpose

```cpp
// Good examples
namespace hf::functional_pin_config {
    struct VortexV1Config {
        static constexpr PinDefinition status_led{2, "Status LED"};
        static constexpr PinDefinition debug_uart_tx{17, "Debug UART TX"};
    };
}

// Avoid
struct config {
    static constexpr PinDefinition led{2, "LED"};
    static constexpr PinDefinition tx{17, "TX"};
};
```

#### Header Organization

- **Include Guards**: Use `#pragma once` at the top of header files
- **Includes**: Group system includes, then third-party, then local includes
- **Namespace**: Use consistent namespace structure
- **Documentation**: Use Doxygen-style comments for API documentation

```cpp
#pragma once

#include <cstdint>
#include <type_traits>

namespace hf::functional_pin_config {

/**
 * @brief Represents a hardware pin configuration
 * 
 * @details This structure holds the pin number and description
 * for compile-time pin configuration management.
 */
struct PinDefinition {
    int pin;                    ///< Physical pin number
    const char* description;    ///< Human-readable description
    
    /**
     * @brief Construct a pin definition
     * @param pin_num Physical pin number
     * @param desc Human-readable description
     */
    constexpr PinDefinition(int pin_num, const char* desc) 
        : pin(pin_num), description(desc) {}
};

} // namespace hf::functional_pin_config
```

### Adding New Platform Support

#### 1. Create Platform Configuration Header

Create a new header file following the naming pattern:
`hf_functional_pin_config_<platform_name>.hpp`

```cpp
#pragma once

#include "hf_functional_pin_config.hpp"

namespace hf::functional_pin_config::<platform_name> {

/**
 * @brief Default pin configuration for <Platform Name>
 * 
 * @details Complete pin mapping for the <Platform Name> development board
 * including all standard peripherals and GPIO assignments.
 */
struct DefaultConfig {
    // System pins
    static constexpr PinDefinition reset_button{0, "Reset Button (Boot)"};
    static constexpr PinDefinition status_led{2, "Built-in Status LED"};
    
    // Communication interfaces
    struct UART {
        static constexpr PinDefinition tx{1, "UART0 TX (USB)"};
        static constexpr PinDefinition rx{3, "UART0 RX (USB)"};
        // Add more UART pins as available
    };
    
    struct SPI {
        static constexpr PinDefinition mosi{23, "SPI MOSI"};
        static constexpr PinDefinition miso{19, "SPI MISO"};
        static constexpr PinDefinition sck{18, "SPI SCK"};
        // Add chip select pins
    };
    
    struct I2C {
        static constexpr PinDefinition sda{21, "I2C SDA"};
        static constexpr PinDefinition scl{22, "I2C SCL"};
    };
    
    // Add platform-specific pins
    // Sensor inputs, motor outputs, etc.
};

} // namespace hf::functional_pin_config::<platform_name>
```

#### 2. Platform Documentation

Add platform-specific documentation:

```markdown
## <Platform Name> Support

### Pin Mapping

| Function | GPIO | Notes |
|----------|------|-------|
| Status LED | 2 | Built-in LED |
| UART TX | 1 | Connected to USB-Serial |
| UART RX | 3 | Connected to USB-Serial |
| ... | ... | ... |

### Usage Example

```cpp
#include "hf_functional_pin_config_<platform_name>.hpp"

using PinConfig = hf::functional_pin_config::<platform_name>::DefaultConfig;

void setup() {
    constexpr auto led = PinConfig::status_led;
    pinMode(led.pin, OUTPUT);
}
```

### Hardware Requirements

- Microcontroller: <specifications>
- Development Board: <board name and version>
- Tested Frameworks: Arduino, ESP-IDF, etc.
```

#### 3. Validation and Testing

Add compile-time validation for the new platform:

```cpp
namespace hf::functional_pin_config::<platform_name> {

template<typename Config = DefaultConfig>
constexpr bool validate_pin_config() {
    // Platform-specific validations
    static_assert(Config::status_led.pin >= 0 && Config::status_led.pin <= 39,
                  "Status LED pin out of range for <Platform>");
    
    static_assert(Config::UART::tx.pin != Config::UART::rx.pin,
                  "UART TX and RX cannot use the same pin");
    
    // Add more platform-specific checks
    return true;
}

// Apply validation
static_assert(validate_pin_config<DefaultConfig>(), 
              "Default configuration validation failed");

} // namespace
```

### Documentation Standards

#### API Documentation

Use Doxygen-style comments for all public APIs:

```cpp
/**
 * @brief Brief description of the function/class/struct
 * 
 * @details Detailed description explaining the purpose, behavior,
 * and any important implementation details.
 * 
 * @param param_name Description of parameter
 * @return Description of return value
 * 
 * @note Important notes or warnings
 * @warning Critical warnings about usage
 * 
 * @example
 * ```cpp
 * // Usage example
 * constexpr auto pin = PinDefinition{5, "Example Pin"};
 * ```
 */
```

#### Markdown Documentation

- Use clear, descriptive headings
- Include code examples for all major features
- Provide both basic and advanced usage examples
- Link to related documentation sections
- Include troubleshooting information

### Testing

#### Compile-Time Testing

Create compile-time tests for new features:

```cpp
// test_compile_time.cpp
#include "hf_functional_pin_config.hpp"
#include "hf_functional_pin_config_vortex_v1.hpp"

using Config = hf::functional_pin_config::vortex_v1::DefaultConfig;

// Test basic functionality
static_assert(Config::status_led.pin >= 0, "Pin should be valid");
static_assert(Config::status_led.description != nullptr, "Description should exist");

// Test configuration validation
template<typename T>
constexpr bool test_validation() {
    // Add your validation tests
    return true;
}

static_assert(test_validation<Config>(), "Validation tests failed");

int main() {
    // If this compiles, tests pass
    return 0;
}
```

#### Platform Testing

Test on actual hardware when possible:

```cpp
// test_hardware.cpp - Arduino example
#include "hf_functional_pin_config_new_platform.hpp"

using Config = hf::functional_pin_config::new_platform::DefaultConfig;

void setup() {
    Serial.begin(115200);
    
    // Test pin access
    constexpr auto led = Config::status_led;
    pinMode(led.pin, OUTPUT);
    
    Serial.printf("Testing %s on GPIO%d\n", led.description, led.pin);
    
    // Basic functionality test
    for (int i = 0; i < 5; i++) {
        digitalWrite(led.pin, HIGH);
        delay(200);
        digitalWrite(led.pin, LOW);
        delay(200);
    }
    
    Serial.println("Hardware test completed successfully");
}

void loop() {
    // Test complete
}
```

## Pull Request Process

### Before Submitting

1. **Ensure Code Quality**
   - Code compiles without warnings
   - Follows the coding standards
   - Includes appropriate documentation
   - Has been tested (compile-time and hardware if possible)

2. **Update Documentation**
   - Update relevant API documentation
   - Add or update examples
   - Update README if adding new platform support

3. **Check Existing Issues**
   - Link to any related issues
   - Ensure your contribution doesn't duplicate existing work

### Pull Request Template

Use this template for pull requests:

```markdown
## Description
Brief description of changes and their purpose.

## Type of Change
- [ ] Bug fix (non-breaking change that fixes an issue)
- [ ] New feature (non-breaking change that adds functionality)
- [ ] Breaking change (fix or feature that would cause existing functionality to not work as expected)
- [ ] Documentation update
- [ ] New platform support

## Testing
- [ ] Code compiles without warnings
- [ ] Compile-time tests pass
- [ ] Hardware testing completed (if applicable)
- [ ] Documentation builds correctly

## Platform Support (if applicable)
- Hardware tested: 
- Frameworks tested: 
- Compiler versions tested: 

## Checklist
- [ ] My code follows the style guidelines
- [ ] I have performed a self-review of my code
- [ ] I have commented my code, particularly in hard-to-understand areas
- [ ] I have made corresponding changes to the documentation
- [ ] My changes generate no new warnings
- [ ] I have added tests that prove my fix is effective or that my feature works

## Related Issues
Closes #(issue_number)
```

### Review Process

1. **Automated Checks**: All PRs will be automatically checked for:
   - Code compilation
   - Documentation generation
   - Style compliance
   - Link validation

2. **Manual Review**: Maintainers will review:
   - Code quality and architecture
   - Documentation completeness
   - Testing adequacy
   - Backward compatibility

3. **Feedback Integration**: 
   - Address reviewer feedback promptly
   - Update your branch with requested changes
   - Discuss any concerns or questions

## Community Guidelines

### Code of Conduct

- Be respectful and inclusive
- Provide constructive feedback
- Help newcomers get started
- Focus on the technical aspects of contributions

### Communication

- **GitHub Issues**: For bug reports and feature requests
- **GitHub Discussions**: For questions and general discussion
- **Pull Requests**: For code contributions and reviews

### Recognition

Contributors will be acknowledged in:
- Project README contributors section
- Release notes for significant contributions
- Documentation credits

## Getting Help

If you need help with contributing:

1. **Check Documentation**: Review existing docs and examples
2. **Search Issues**: Look for similar questions or problems
3. **Ask Questions**: Create a GitHub Discussion or issue
4. **Join the Community**: Participate in discussions and reviews

Thank you for contributing to HF-PinCfg! Your contributions help make embedded development more accessible and maintainable for everyone.