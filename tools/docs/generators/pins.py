"""Pin tables generated from the X-macro rows of the board headers in ``src/``.

Regions:

- ``boards``: the board selector (``src/hf_functional_pin_config.hpp``): macro, header, MCU, row counts.
- ``pins-<board>`` / ``adc-<board>``: one row per ``X(...)`` entry of ``HF_FUNCTIONAL_GPIO_PIN_LIST`` /
  ``HF_FUNCTIONAL_ADC_CHANNEL_LIST`` in ``src/hf_functional_pin_config_<board>.hpp``, in header order.

Rows inside ``/* ... */`` comments are not part of the list and are not shown. Columns that are zero for every row
of a board (unit, bank) are left out.
"""

from __future__ import annotations

import re

from hardfoc_mkdocs_theme.paths import repo_root

SELECTOR = "src/hf_functional_pin_config.hpp"
BOARDS = {  # header stem -> (title, MCU as named in the header's file comment)
    "vortex_v1": ("Vortex V1", "ESP32-C6"),
    "flux_v1": ("Flux V1", "ESP32-S3"),
}
GPIO_FIELDS = ["enum", "name", "category", "chip", "unit", "bank", "pin", "inverted", "has_pull", "pull_up",
               "push_pull", "current"]
ADC_FIELDS = ["enum", "name", "chip", "unit", "adc_unit", "channel", "bits", "max_mv", "divider", "description"]
CATEGORY = {"PIN_CATEGORY_CORE": "core", "PIN_CATEGORY_COMM": "comm", "PIN_CATEGORY_GPIO": "gpio",
            "PIN_CATEGORY_USER": "user"}
CHIP = {"ESP32_INTERNAL": "MCU", "PCAL95555_EXPANDER": "PCAL95555", "TMC9660_CONTROLLER": "TMC9660",
        "MAX22200_DRIVER": "MAX22200", "TLE92466ED_DRIVER": "TLE92466ED", "ADS7952_EXTERNAL": "ADS7952"}


def _header(stem: str) -> str:
    return f"src/hf_functional_pin_config_{stem}.hpp"


def _list_body(text: str, macro: str) -> str:
    m = re.search(rf"^#define {macro}\(X\)(.*?)(?<!\\)\n(?!\s*X\()", text, re.M | re.S)
    if not m:
        raise SystemExit(f"[gen] no {macro} in header")
    body = re.sub(r"/\*.*?\*/", " ", m.group(1), flags=re.S)  # commented-out rows and notes
    return body.replace("\\\n", "\n")


def _split(args: str) -> list[str]:
    out, cur, quoted = [], "", False
    for c in args:
        if c == '"':
            quoted = not quoted
        if c == "," and not quoted:
            out.append(cur.strip())
            cur = ""
        else:
            cur += c
    out.append(cur.strip())
    return out


def _rows(stem: str, macro: str, fields: list[str]) -> list[dict]:
    body = _list_body((repo_root() / _header(stem)).read_text(), macro)
    rows = []
    for m in re.finditer(r"\bX\((.*?)\)\s*(?=X\(|$)", body, re.S):
        vals = _split(" ".join(m.group(1).split()))
        if len(vals) != len(fields):
            raise SystemExit(f"[gen] {_header(stem)}: {len(vals)} fields in {vals[0]}, expected {len(fields)}")
        rows.append(dict(zip(fields, vals)))
    return rows


def _enum(v: str) -> str:
    return v.split("::")[-1]


def _code(v: str) -> str:
    return f"`{v}`"


def _table(head: list[str], rows: list[list[str]]) -> str:
    out = ["| " + " | ".join(head) + " |", "| " + " | ".join("---" for _ in head) + " |"]
    out += ["| " + " | ".join(r) + " |" for r in rows]
    return "\n".join(out) + "\n"


def _drop_zero(head: list[str], rows: list[list[str]], names: list[str]) -> tuple[list[str], list[list[str]]]:
    keep = [i for i, h in enumerate(head) if h not in names or any(r[i] != "0" for r in rows)]
    return [head[i] for i in keep], [[r[i] for i in keep] for r in rows]


def gpio_table(stem: str) -> str:
    head = ["Function", "Category", "Chip", "Unit", "Bank", "Pin", "Active", "Pull", "Drive", "Max (mA)"]
    rows = []
    for r in _rows(stem, "HF_FUNCTIONAL_GPIO_PIN_LIST", GPIO_FIELDS):
        pull = "none" if r["has_pull"] == "PIN_NO_PULL" else ("up" if r["pull_up"] == "PIN_PULL_UP" else "down")
        rows.append([
            _code(r["enum"]),
            CATEGORY[_enum(r["category"])],
            CHIP.get(_enum(r["chip"]), _enum(r["chip"])),
            r["unit"], r["bank"], r["pin"],
            "low" if r["inverted"] == "PIN_LOGIC_INVERTED" else "high",
            pull,
            "push-pull" if r["push_pull"] == "PIN_PUSH_PULL" else "open-drain",
            r["current"],
        ])
    head, rows = _drop_zero(head, rows, ["Unit", "Bank"])
    return f"Source: `{_header(stem)}`, `HF_FUNCTIONAL_GPIO_PIN_LIST` ({len(rows)} rows).\n\n" + _table(head, rows)


def adc_table(stem: str) -> str:
    head = ["Function", "Chip", "Unit", "ADC unit", "Channel", "Bits", "Full scale (mV)", "Divider", "Description"]
    rows = []
    for r in _rows(stem, "HF_FUNCTIONAL_ADC_CHANNEL_LIST", ADC_FIELDS):
        rows.append([
            _code(r["enum"]),
            CHIP.get(_enum(r["chip"]), _enum(r["chip"])),
            r["unit"], r["adc_unit"], r["channel"], r["bits"], r["max_mv"],
            r["divider"].rstrip("f"),
            r["description"].strip('"'),
        ])
    head, rows = _drop_zero(head, rows, ["Unit", "ADC unit"])
    return f"Source: `{_header(stem)}`, `HF_FUNCTIONAL_ADC_CHANNEL_LIST` ({len(rows)} rows).\n\n" + _table(head, rows)


def boards() -> str:
    sel = (repo_root() / SELECTOR).read_text()
    rows = []
    for macro, header in re.findall(r"defined\((HARDFOC_BOARD_\w+)\).*?\n#include \"(\w+\.hpp)\"", sel):
        stem = header.removeprefix("hf_functional_pin_config_").removesuffix(".hpp")
        if stem in BOARDS:
            title, mcu = BOARDS[stem]
            n_gpio = len(_rows(stem, "HF_FUNCTIONAL_GPIO_PIN_LIST", GPIO_FIELDS))
            n_adc = len(_rows(stem, "HF_FUNCTIONAL_ADC_CHANNEL_LIST", ADC_FIELDS))
        else:
            title, mcu, n_gpio, n_adc = "None (host builds)", "none", 0, 0
        rows.append([_code(macro), title, mcu, _code(f"src/{header}"), str(n_gpio), str(n_adc)])
    return _table(["Define", "Board", "MCU", "Header", "GPIO rows", "ADC rows"], rows)


REGIONS = {"boards": boards}
for _stem in BOARDS:
    _slug = _stem.replace("_", "-")
    REGIONS[f"pins-{_slug}"] = lambda s=_stem: gpio_table(s)
    REGIONS[f"adc-{_slug}"] = lambda s=_stem: adc_table(s)
