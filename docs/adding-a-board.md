# Adding a board

Write a board header, hook it into the selector and generate its pin table, in five steps.

<div class="hf-steps" markdown>

1. **Copy the closest board header.** Start from
   [`src/hf_functional_pin_config_vortex_v1.hpp`](../src/hf_functional_pin_config_vortex_v1.hpp) or
   [`src/hf_functional_pin_config_flux_v1.hpp`](../src/hf_functional_pin_config_flux_v1.hpp) and name it
   `hf_functional_pin_config_<board>.hpp`, with its own include guard.
2. **Define the chips.** Edit `HfGpioChipType` and `HfAdcChipType` so they list the chips that carry pins on this
   board, MCU first (`= 0`).
3. **Write the rows.** One `X(...)` per pin in `HF_FUNCTIONAL_GPIO_PIN_LIST` and per channel in
   `HF_FUNCTIONAL_ADC_CHANNEL_LIST`, grouped by category, with the fields from the [pin model](concepts.md).
   Take every number from the schematic; record wiring facts in a `/* ... */` comment above the row. Keep the
   generated-table section and the helper functions unchanged.
4. **Hook it into the selector.** Add an `#elif defined(HARDFOC_BOARD_<BOARD>)` branch to
   [`src/hf_functional_pin_config.hpp`](../src/hf_functional_pin_config.hpp) and the macro to the `#error`
   message.
5. **Document it.** Add the board to `BOARDS` in
   [`tools/docs/generators/pins.py`](../tools/docs/generators/pins.py), copy `docs/vortex-v1.md` to a page for the
   board with its region names (`pins-<board>`, `adc-<board>`), add the page to `docs/.nav.yml` and run
   `hardfoc-docs gen`.

</div>

## Check it

```bash
c++ -std=c++20 -fsyntax-only -Wall -Wextra -Isrc -DHARDFOC_BOARD_<BOARD> tests/docs/quickstart.cpp
hardfoc-docs gen --check
mkdocs build --strict
```

The first line compiles every documented call against the new map. Then bump the submodule in the board HAL and
define the macro there.

## Rules for rows

| Rule | Why |
| --- | --- |
| Rows never move between categories silently | The HAL's GPIO manager registers `gpio` and `user` rows only |
| Chip selects and resets are `PIN_LOGIC_INVERTED` when the net is active low | Drivers set logical levels; the row holds the polarity |
| A net reachable from two chips gets two rows, one per chip, with a comment | Vortex V1 TMC9660 GPIO17/18 are the example |
| No pin number outside the header | Examples that call the SDK directly read it from the map or a constant derived from it |

Next: [Testing and CI](testing.md).
