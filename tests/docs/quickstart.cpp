// Code shown on the documentation site and in the README. Check it against every board:
//   for b in VORTEX_V1 FLUX_V1; do c++ -std=c++20 -fsyntax-only -Wall -Wextra -Isrc -DHARDFOC_BOARD_$b \
//     tests/docs/quickstart.cpp; done
#include <cstdio>

// --8<-- [start:quickstart]
#include "hf_functional_pin_config.hpp"  // the build defines one HARDFOC_BOARD_* macro

// Print where every pin that the GPIO layer should own lives on this board.
void list_gpio_pins() {
  for (size_t i = 0; i < HF_GPIO_MAPPING_SIZE; ++i) {
    const HfGpioMapping& m = HF_GPIO_MAPPING[i];
    if (!should_register_as_gpio(static_cast<HfPinCategory>(m.category))) {
      continue;  // core and comm pins belong to the system and the bus drivers
    }
    std::printf("%-28.*s chip %u pin %2u %s\n", static_cast<int>(m.name.size()), m.name.data(),
                m.chip_type, m.physical_pin, m.is_inverted ? "active-low" : "active-high");
  }
}
// --8<-- [end:quickstart]

// --8<-- [start:lookup]
// Find one pin by function: the only place a pin number comes from.
bool chip_select_is_active_low(HfFunctionalGpioPin pin) {
  const HfGpioMapping* m = GetGpioMapping(pin);
  if (m == nullptr) {
    return false;  // not on this board
  }
  return m->is_inverted;
}
// --8<-- [end:lookup]

// --8<-- [start:filter]
// Every pin a given chip carries, without a heap.
size_t pins_on_mcu(HfFunctionalGpioPin (&out)[HF_GPIO_MAPPING_SIZE]) {
  return GetGpioPinsByChipType(HfGpioChipType::ESP32_INTERNAL, out, HF_GPIO_MAPPING_SIZE);
}
// --8<-- [end:filter]

// --8<-- [start:adc]
// ADC channels carry their scaling: full-scale voltage, resolution and divider.
float counts_to_volts(HfFunctionalAdcChannel ch, uint32_t counts) {
  const HfAdcMapping* m = GetAdcMapping(ch);
  if (m == nullptr) {
    return 0.0F;
  }
  const float full_scale = static_cast<float>((1U << m->resolution_bits) - 1U);
  return static_cast<float>(counts) / full_scale * (static_cast<float>(m->max_voltage_mv) / 1000.0F) *
         m->voltage_divider;
}
// --8<-- [end:adc]

// --8<-- [start:names]
// Names for logs and consoles; categories decide who owns a pin.
void describe(HfFunctionalGpioPin pin) {
  const std::string_view name = to_string(pin);
  const bool owned_by_gpio = should_register_as_gpio(get_pin_category(pin));
  std::printf("%.*s: %s\n", static_cast<int>(name.size()), name.data(), owned_by_gpio ? "GPIO" : "reserved");
}
// --8<-- [end:names]
