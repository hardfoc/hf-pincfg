#ifndef HF_FUNCTIONAL_PIN_CONFIG_HPP
#define HF_FUNCTIONAL_PIN_CONFIG_HPP

// ==========================================================================
// Board selection for functional pin config
//
// Each HAL's CMakeLists.txt must define exactly ONE HARDFOC_BOARD_* macro
// via target_compile_definitions(). The selector includes the matching
// board-specific pin config header.
// ==========================================================================

#if defined(HARDFOC_BOARD_FLUX_V1)
#include "hf_functional_pin_config_flux_v1.hpp"

#elif defined(HARDFOC_BOARD_VORTEX_V1)
#include "hf_functional_pin_config_vortex_v1.hpp"

#else
#error                                                                                             \
    "No supported board variant defined! Please define exactly one of: "                           \
    "HARDFOC_BOARD_VORTEX_V1, HARDFOC_BOARD_FLUX_V1"
#endif

#endif // HF_FUNCTIONAL_PIN_CONFIG_HPP
