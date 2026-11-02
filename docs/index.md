---
hide:
  - navigation
  - toc
---

<div class="hf-hero" markdown>
<span class="hf-kicker">HardFOC core library</span>

# Functional Pin Configuration

<p>One header per HardFOC board that says what every pin does, which chip carries it and how it is driven. The
GPIO and ADC layers of a board HAL register their pins from it, so a pin number is written exactly once.</p>

[Quickstart](quickstart.md){ .md-button .md-button--primary }
[Vortex V1 pins](vortex-v1.md){ .md-button }
[Flux V1 pins](flux-v1.md){ .md-button }
</div>

A pin map copied into managers, examples and documents drifts: the schematic changes, one copy is updated and the
board misbehaves in a way no compiler catches. `hf-pincfg` keeps each board's map as X-macro rows in one header.
Expanding the rows produces the functional enums, the name and category tables and the `constexpr` mapping arrays
that the board HAL iterates. The tables on this site are generated from the same rows.

## What you get

- **Functional names, not GPIO numbers.** Code asks for `HfFunctionalGpioPin::SPI2_CS_TMC9660`; the row says it
  is MCU GPIO 18, active low, pulled up. See [Pin model](concepts.md).
- **Pins on any chip.** A row can live on the MCU, an I/O expander (PCAL95555) or a motor controller's GPIO
  (TMC9660), so expander pins are first-class. See the [Vortex V1 pin map](vortex-v1.md).
- **Electrical settings with the pin.** Polarity, pull, drive mode and maximum current travel in the same row.
- **ADC channels with their scaling.** Resolution, full-scale voltage and divider ratio per channel. See the
  [API guide](api-guide.md#adc-channels).
- **Compile-time board selection.** One `HARDFOC_BOARD_*` define picks the header; no runtime tables, no heap.
  See [Integration](integration.md).
- **Header-only C++20.** Nothing to link; the arrays are `constexpr`.

## Used by

| Repository | How |
| --- | --- |
| [hf-hal-vortex-v1](https://github.com/hardfoc/hf-hal-vortex-v1) | Defines `HARDFOC_BOARD_VORTEX_V1`; `GpioManager` registers every GPIO-category row |
| [hf-hal-flux-v1](https://github.com/hardfoc/hf-hal-flux-v1) | Flux V1 board HAL; selects `HARDFOC_BOARD_FLUX_V1` |
| [hf-core](https://github.com/hardfoc/hf-core) | Adds `src/` to the include path; host builds define `HARDFOC_BOARD_NONE` |
| [hf-core-drivers](https://github.com/hardfoc/hf-core-drivers) | Carries this repository as the `internal/hf-pincfg` submodule |

## Where to go next

<div class="grid cards" markdown>

-   :material-rocket-launch-outline: **[Quickstart](quickstart.md)**

    ---

    Select a board and walk its pin map in a dozen lines.

-   :material-shape-outline: **[Pin model](concepts.md)**

    ---

    One X-macro row, every field explained.

-   :material-table: **[Vortex V1 pin map](vortex-v1.md)**

    ---

    ESP32-C6, PCAL95555 and TMC9660 pins, generated from the header.

-   :material-table-large: **[Flux V1 pin map](flux-v1.md)**

    ---

    ESP32-S3 pins for the MAX22200, TLE92466ED and ADS7952.

-   :material-plus-box-outline: **[Adding a board](adding-a-board.md)**

    ---

    Write a new header and hook it into the selector.

</div>
