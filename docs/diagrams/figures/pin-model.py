"""How a board's pin map becomes code: selector, X-macro rows, generated tables, the HAL that reads them.

Reads: From top to bottom. The HAL's build defines one HARDFOC_BOARD_* macro; the selector header
(src/hf_functional_pin_config.hpp) includes that board's header. The board header holds two X-macro lists, one row
per GPIO function and one per ADC channel. Expanding the lists produces the enums, name and category tables and the
mapping arrays (names read from the Vortex V1 header). The board HAL's managers iterate the mapping arrays and look
pins up by enum; no other code writes a pin number.
"""

import re
from pathlib import Path

from hardfoc_mkdocs_theme.figkit import Fig

ROOT = Path(__file__).resolve().parents[3]
SELECTOR = "src/hf_functional_pin_config.hpp"
BOARD = "src/hf_functional_pin_config_vortex_v1.hpp"


def data():
    sel = (ROOT / SELECTOR).read_text()
    board = (ROOT / BOARD).read_text()
    generated = re.findall(r"^enum class (HfFunctional\w+)", board, re.M)
    generated += re.findall(r"^static constexpr \w+(?:::\w+)? (\w+)\[\]", board, re.M)
    return {
        "boards": re.findall(r"defined\((HARDFOC_BOARD_\w+)\)", sel),
        "lists": re.findall(r"^#define (HF_FUNCTIONAL_\w+_LIST)\(X\)", board, re.M),
        "generated": generated,
        "sources": [f"{SELECTOR}:board selector", f"{BOARD}:X-macro lists and generated tables"],
    }


def build(d):
    f = Fig(
        "pin-model",
        "One row per pin, every table generated",
        alt="Flow diagram: the HAL build defines one HARDFOC_BOARD macro, the selector header includes that "
        "board's header, whose X-macro lists expand into the pin enums, name and category tables and mapping "
        "arrays, which the board HAL's GPIO and ADC managers read.",
        width=1200,
        kicker="Concepts",
        subtitle="Change a pin in one row; the enums, names and mapping arrays follow at compile time.",
        profile="overview",
        legend=[("brand", "this library"), ("teal", "the board HAL"), ("slate", "build input")],
    )
    W = f.body_w
    sel = f.card(0, 0, W, "Board selection (HAL build)", ["  ".join(d["boards"])], tone="slate", mono=True)
    hdr = f.card(0, sel.b + 40, W, "Board header: X-macro lists",
                 ["  ".join(d["lists"]),
                  "X(SPI2_CS_TMC9660, \"COMM_SPI2_CS_TMC9660\", COMM, ESP32_INTERNAL, 0, 0, 18, INVERTED, ...)"],
                 tone="brand", accent=True, mono=True)
    top = hdr.b + 40
    chips_h = f.chips(16, f.region_inner_top(top), W - 32, d["generated"], tone="brand")
    reg = f.region(0, top, W, f.region_inner_top(top) - top + chips_h + 18, "Generated at compile time",
                   tone="brand", sub="constexpr arrays, no heap, no init code")
    hal = f.card(0, reg.b + 40, W, "Board HAL managers",
                 ["GpioManager registers each gpio / user row on its chip; AdcManager each ADC row;",
                  "everything else looks pins up by HfFunctionalGpioPin"], tone="teal", accent=True)
    for a, b in ((sel, hdr), (hdr, reg), (reg, hal)):
        f.connect(a, b, "bottom", "top", tone="rail")
    return f
