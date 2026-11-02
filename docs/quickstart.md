# Quickstart

Select a board, include one header and read its pin map, in about a dozen lines.

## 1. Add the repository

Most projects get it through [hf-core-drivers](https://github.com/hardfoc/hf-core-drivers) as
`internal/hf-pincfg`. On its own:

```bash
git submodule add https://github.com/hardfoc/hf-pincfg.git external/hf-pincfg
```

## 2. Select the board

Define exactly one board macro for every translation unit that includes the header, normally in the HAL's
`CMakeLists.txt`:

```cmake
target_include_directories(my_hal PUBLIC external/hf-pincfg/src)
target_compile_definitions(my_hal PUBLIC HARDFOC_BOARD_VORTEX_V1=1)
```

Without one, the selector stops the build with `No supported board variant defined!`.

## 3. Read the map

```cpp title="tests/docs/quickstart.cpp"
--8<-- "tests/docs/quickstart.cpp:quickstart"
```

What happens:

| Step | What it does |
| --- | --- |
| `#include "hf_functional_pin_config.hpp"` | The selector includes the board header named by the define |
| `HF_GPIO_MAPPING`, `HF_GPIO_MAPPING_SIZE` | One `HfGpioMapping` per row of the board's GPIO list, in header order |
| `should_register_as_gpio()` | True for `gpio` and `user` rows; `core` and `comm` rows belong to the system and bus drivers |
| `m.chip_type`, `m.physical_pin` | Which chip carries the pin and its number on that chip |

The library needs C++20 (`std::string_view::starts_with`).

## If it does not compile

| Message | Cause | Fix |
| --- | --- | --- |
| `No supported board variant defined!` | No `HARDFOC_BOARD_*` define reached this file | Add it with `PUBLIC` scope, or to the component's compile options |
| `no member named 'starts_with'` | Built as C++17 | Build with `-std=c++20` or later |
| `redefinition of 'HfGpioChipType'` | Two board headers included | Include only `hf_functional_pin_config.hpp`, never a board header directly |

Next: [Integration](integration.md).
