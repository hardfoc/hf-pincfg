#ifndef HF_FUNCTIONAL_PIN_CONFIG_NONE_HPP
#define HF_FUNCTIONAL_PIN_CONFIG_NONE_HPP

/**
 * @file hf_functional_pin_config_none.hpp
 * @brief Empty/stub functional pin mapping for software-only builds.
 *
 * This file provides empty pin tables for HARDFOC_BOARD_NONE — used for:
 *   - Host-only compilation and unit testing
 *   - CI/CD builds without real hardware
 *   - Software-only (MCU=NONE) builds
 *
 * All pin and ADC mapping tables are empty. Lookup functions return nullptr.
 */

#include "hf_functional_pin_config_base.hpp"

//==============================================================================
// CHIP TYPE ENUMS (NONE — no real hardware)
//==============================================================================

enum class HfGpioChipType : uint8_t {
    NONE_STUB = 0
};

enum class HfAdcChipType : uint8_t {
    NONE_STUB = 0
};

//==============================================================================
// XMACRO PIN DEFINITIONS (EMPTY)
//==============================================================================

#define HF_FUNCTIONAL_GPIO_PIN_LIST(X) /* no pins */

#define HF_FUNCTIONAL_ADC_CHANNEL_LIST(X) /* no ADC channels */

//==============================================================================
// GENERATED ENUMS (EMPTY + COUNT)
//==============================================================================

enum class HfFunctionalGpioPin : uint8_t {
    _PIN_COUNT = 0
};

enum class HfFunctionalAdcChannel : uint8_t {
    _CHANNEL_COUNT = 0
};

//==============================================================================
// MAPPING TABLES (EMPTY)
//==============================================================================

inline constexpr size_t HF_GPIO_MAPPING_SIZE = 0;
inline constexpr size_t HF_ADC_MAPPING_SIZE = 0;

inline constexpr HfGpioMapping HF_GPIO_MAPPING[] = {
    // Empty — sentinel to ensure valid array (zero-length arrays are non-standard)
    {0, 0, 0, 0, 0, false, false, false, false, 0, 0, ""}
};

inline constexpr HfAdcMapping HF_ADC_MAPPING[] = {
    // Empty — sentinel
    {0, 0, 0, 0, 0, 0, 0, 0.0f, ""}
};

inline constexpr HfPinCategory HfFunctionalGpioPinCategories[] = {
    // Empty — sentinel
    HfPinCategory::PIN_CATEGORY_CORE
};

//==============================================================================
// LOOKUP FUNCTIONS
//==============================================================================

static inline const HfGpioMapping* GetGpioMapping(HfFunctionalGpioPin) {
    return nullptr;
}

static inline const HfAdcMapping* GetAdcMapping(HfFunctionalAdcChannel) {
    return nullptr;
}

static inline bool IsGpioPinAvailable(HfFunctionalGpioPin) {
    return false;
}

static inline bool IsAdcChannelAvailable(HfFunctionalAdcChannel) {
    return false;
}

//==============================================================================
// UTILITY FUNCTIONS
//==============================================================================

static inline size_t GetGpioPinsByCategory(HfPinCategory, HfFunctionalGpioPin*, size_t) {
    return 0;
}

static inline size_t GetGpioPinsByChipType(HfGpioChipType, HfFunctionalGpioPin*, size_t) {
    return 0;
}

static inline bool RequiresPullUp(HfFunctionalGpioPin) {
    return false;
}

static inline bool RequiresPullDown(HfFunctionalGpioPin) {
    return false;
}

#endif // HF_FUNCTIONAL_PIN_CONFIG_NONE_HPP
