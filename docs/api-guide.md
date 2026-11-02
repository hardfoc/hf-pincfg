# API guide

Look up pins and channels, filter them and turn ADC counts into volts, by task. Every block compiles against both
boards.

## The shape of the API

| Task | Call | Returns |
| --- | --- | --- |
| One pin's row | `GetGpioMapping(HfFunctionalGpioPin)` | `const HfGpioMapping*`, `nullptr` if out of range |
| One ADC channel's row | `GetAdcMapping(HfFunctionalAdcChannel)` | `const HfAdcMapping*` or `nullptr` |
| Every row | `HF_GPIO_MAPPING[]`, `HF_ADC_MAPPING[]` with `..._SIZE` | `constexpr` arrays |
| Name of a pin or channel | `to_string(pin)`, `to_string(channel)` | `std::string_view`, `"UNKNOWN_PIN"` if out of range |
| Owner of a pin | `get_pin_category(pin)`, `should_register_as_gpio(category)` | `HfPinCategory`, `bool` |
| Pins by owner or chip | `GetGpioPinsByCategory()`, `GetGpioPinsByChipType()` | count written to your array |
| Pull resistors | `RequiresPullUp(pin)`, `RequiresPullDown(pin)` | `bool` |
| Availability | `IsGpioPinAvailable(pin)`, `IsAdcChannelAvailable(ch)` | `bool` |

## Walk the pins a GPIO layer owns

```cpp
--8<-- "tests/docs/quickstart.cpp:quickstart"
```

A board HAL does the same loop and creates a driver for each row on the chip `chip_type` names
(`static_cast<HfGpioChipType>(m.chip_type)`).

## Look up one pin

```cpp
--8<-- "tests/docs/quickstart.cpp:lookup"
```

The enum value indexes the arrays directly, so the lookup is a bounds check and an array access.

## Filter by chip

```cpp
--8<-- "tests/docs/quickstart.cpp:filter"
```

`GetGpioPinsByCategory()` works the same way with an `HfPinCategory`. Both write at most `max_count` entries and
return how many they wrote.

## ADC channels

```cpp
--8<-- "tests/docs/quickstart.cpp:adc"
```

`max_voltage_mv` is the full-scale input of the converter and `voltage_divider` the ratio of the external divider
in front of it (Flux V1 bus voltage: `11.0`).

## Names and categories

```cpp
--8<-- "tests/docs/quickstart.cpp:names"
```

## Board-specific constants

A board header may export derived constants for code that calls the MCU SDK directly. Flux V1 has
`kFluxHwUartFdo2TxGpio` and `kFluxHwUartFdo2RxGpio`, read from the `UART_FDO2_TX` / `UART_FDO2_RX` rows, so they
cannot disagree with the map.

Next: [Adding a board](adding-a-board.md).
