#ifndef HF_FUNCTIONAL_PIN_CONFIG_HPP
#define HF_FUNCTIONAL_PIN_CONFIG_HPP

// Board selection for functional pin config
#if defined(HARDFOC_BOARD_VORTEX_V1) || !defined(HARDFOC_BOARD_SELECTED)
#include "hf_functional_pin_config_vortex_v1.hpp"
#else
#error "No supported board variant defined! Please define HARDFOC_BOARD_VORTEX_V1 or another supported board."
#endif

#endif // HF_FUNCTIONAL_PIN_CONFIG_HPP
