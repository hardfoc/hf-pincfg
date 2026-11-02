# Integration

Add the headers to a build, select the board and pick up the version, with plain CMake or ESP-IDF.

## Requirements

| Item | Requirement |
| --- | --- |
| Language | C++20 (`std::string_view::starts_with`) |
| Dependencies | None: `<cstdint>` and `<string_view>` |
| Files | Header-only; everything is in [`src/`](../src/hf_functional_pin_config.hpp) |
| Heap, RTOS, exceptions | None |

## CMake

The repository has no `CMakeLists.txt`: add `src/` to the include path of the target that owns the board, and
define the board there with `PUBLIC` scope so every consumer sees the same map.

```cmake
target_include_directories(my_hal PUBLIC ${CMAKE_CURRENT_LIST_DIR}/external/hf-pincfg/src)
target_compile_definitions(my_hal PUBLIC HARDFOC_BOARD_FLUX_V1=1)
```

[hf-core](https://github.com/hardfoc/hf-core) does this for you: `cmake/hf_core_build_settings.cmake` adds
`hf-core-drivers/internal/hf-pincfg/src` to its include directories, and a host build (`HF_CORE_MCU=NONE`) defines
`HARDFOC_BOARD_NONE`. The board HAL then adds its own `HARDFOC_BOARD_*` define.

## ESP-IDF

Inside an ESP-IDF component, add the folder to `INCLUDE_DIRS` and the define to the component's compile options:

```cmake
idf_component_register(
    SRCS "my_hal.cpp"
    INCLUDE_DIRS "." "external/hf-pincfg/src")
target_compile_definitions(${COMPONENT_LIB} PUBLIC HARDFOC_BOARD_VORTEX_V1=1)
target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_20)
```

## Configuration macros

| Macro | Set by | Effect |
| --- | --- | --- |
| `HARDFOC_BOARD_VORTEX_V1` | Vortex V1 HAL | Includes the Vortex V1 map |
| `HARDFOC_BOARD_FLUX_V1` | Flux V1 HAL | Includes the Flux V1 map |
| `HARDFOC_BOARD_NONE` or `HF_MCU_FAMILY_NONE` | Host builds | Empty map, lookups return `nullptr` |

Exactly one must be defined. When several are, the first in the selector wins (Flux V1, then Vortex V1, then
none); do not rely on that.

## Version

[`src/pincfg_version.h`](../src/pincfg_version.h) defines `HF_PINCFG_VERSION_MAJOR`, `_MINOR`, `_PATCH` and
`HF_PINCFG_VERSION_STRING`. A firmware version manifest can pick it up with `__has_include("pincfg_version.h")`.

Next: [Pin model](concepts.md).
